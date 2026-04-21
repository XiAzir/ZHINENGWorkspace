#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "dsp_noise_tracker.h"

static const char *TAG = "task_adaptive";

// 自适应噪声基线更新周期（30s）
#define ADAPTIVE_UPDATE_INTERVAL_MS  30000

void TaskAdaptive_Run(void *arg) {
    ESP_LOGI(TAG, "Adaptive task started");

    DSPNoiseTracker_t noise_tracker;
    dsp_noise_tracker_init(&noise_tracker);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ADAPTIVE_UPDATE_INTERVAL_MS));

        // 更新噪声基线（供 dsp_mel 或 dsp_filter 使用）
        // M4 后完善：读取 task_doa_feature 暴露的环境 SPL 估计值，
        // 动态调整 IIR 截止频率或 Mel 特征的噪声减除参数
        dsp_noise_tracker_update(&noise_tracker);
        ESP_LOGD(TAG, "Noise baseline updated");
    }
}
