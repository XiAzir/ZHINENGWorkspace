#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"

#include "drv_pdm_dma.h"
#include "drv_oled.h"
#include "drv_i2c_bus.h"
#include "drv_haptic.h"
#include "drv_imu.h"
#include "drv_sim.h"

#include "dsp_filter.h"
#include "dsp_mel.h"
#include "dsp_gcc_phat.h"
#include "dsp_noise_tracker.h"
#include "dsp_threat_tracker.h"

#include "ai_wrapper.h"

// ── 任务入口函数声明 ────────────────────────────────────────
void TaskAudio_Run(void *arg);
void TaskDOA_Run(void *arg);
void TaskAI_Run(void *arg);
void TaskDecision_Run(void *arg);
void TaskGUI_Run(void *arg);
void TaskGUI_Fast_Run(void *arg);
void TaskOrientation_Run(void *arg);
void TaskHaptic_Run(void *arg);
void TaskAdaptive_Run(void *arg);
#ifdef FEATURE_BLE
void TaskBLE_Run(void *arg);
#endif

static const char *TAG = "app_main";

void app_main(void) {
    ESP_LOGI(TAG, "AudioSense Glasses starting...");

    // 1. FreeRTOS 对象初始化（队列 + 共享原子变量）
    app_freertos_init();

    // 2. 共享 I2C 总线（必须先于 haptic/imu）
    ESP_ERROR_CHECK(drv_i2c_bus_init());
    drv_i2c_bus_log_known_devices();

    // 3. 驱动初始化
#ifdef USE_SIMULATION
    drv_sim_init();
    ESP_LOGI(TAG, "Simulation mode enabled");
#else
    ESP_ERROR_CHECK(drv_pdm_dma_init(AUDIO_SAMPLE_RATE, AUDIO_FRAME_LEN, NUM_DMA_BUFFERS));
#endif
    ESP_ERROR_CHECK(drv_oled_init());

    // 开机 Logo 展示 2 秒
    vTaskDelay(pdMS_TO_TICKS(2000));
    drv_oled_clear();

    esp_err_t haptic_err = drv_haptic_init();
    if (haptic_err != ESP_OK) {
        ESP_LOGW(TAG, "Haptic unavailable (%s); system continues without vibration.",
                 esp_err_to_name(haptic_err));
    }

    // IMU 初始化失败不致命：task_orientation 会降级为 yaw=0 桩模式
    esp_err_t imu_err = drv_imu_init();
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "IMU unavailable (%s); 3DoF anchoring runs in stub mode.",
                 esp_err_to_name(imu_err));
    }

    // 4. AI 模型加载
    AI_Init();

    // 5. 创建 RTOS 任务（绑核策略见计划文档 §3.3）
    xTaskCreatePinnedToCore(TaskAudio_Run,       "audio",    TASK_STACK_AUDIO,    NULL, 5, &h_task_audio,    0);
    xTaskCreatePinnedToCore(TaskDOA_Run,         "doa",      TASK_STACK_DOA,      NULL, 4, &h_task_doa,      0);
    xTaskCreatePinnedToCore(TaskOrientation_Run, "orient",   TASK_STACK_ORIENT,   NULL, 4, &h_task_orient,   1);
    xTaskCreatePinnedToCore(TaskAI_Run,          "ai",       TASK_STACK_AI,       NULL, 3, &h_task_ai,       1);
    xTaskCreatePinnedToCore(TaskDecision_Run,    "decision", TASK_STACK_DECISION, NULL, 2, &h_task_decision, 1);
    xTaskCreatePinnedToCore(TaskGUI_Run,         "gui_slow", TASK_STACK_GUI,      NULL, 1, NULL,             1);
    xTaskCreatePinnedToCore(TaskGUI_Fast_Run,    "gui_fast", TASK_STACK_GUI_FAST, NULL, 1, &h_task_gui_fast, 1);
    xTaskCreate(TaskHaptic_Run,                  "haptic",   TASK_STACK_HAPTIC,   NULL, 1, NULL);
    xTaskCreate(TaskAdaptive_Run,                "adaptive", TASK_STACK_ADAPTIVE, NULL, 1, NULL);
#ifdef FEATURE_BLE
    xTaskCreate(TaskBLE_Run,                     "ble",      TASK_STACK_BLE,      NULL, 1, NULL);
#endif

    ESP_LOGI(TAG, "All tasks created. System running.");
}
