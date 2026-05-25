#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_freertos.h"
#include "drv_oled.h"

static const char *TAG = "task_gui_fast";

// 3DoF 锚定快线程（文档《关于危险时间和3DoF.md》§2.2）：
//   每帧 β_render = θ_world − Yaw_current
// 若 θ_world 尚未更新（开机早期），默认为 0；Yaw 缺 IMU 时也为 0 → β=0（指正前方）。
void TaskGUI_Fast_Run(void *arg) {
    ESP_LOGI(TAG, "GUI fast task started on core %d", xPortGetCoreID());

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(GUI_FAST_TICK_MS));

#if AUDIO_SINGLE_MIC_TEST_MODE
        float beta = 0.0f;
#else
        float theta_w = theta_world_get();
        float yaw     = orientation_yaw_get();
        float beta    = theta_w - yaw;

        // 归一到 [-180, 180]
        while (beta >  180.0f) beta -= 360.0f;
        while (beta < -180.0f) beta += 360.0f;
#endif

        drv_oled_draw_arrow(beta);

        static uint32_t s_p0_cnt = 0;
        if (++s_p0_cnt % 300 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
    }
}
