#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "drv_haptic.h"

static const char *TAG = "task_haptic";

// 防疲劳冷却时间（按档位）
#define HAPTIC_COOLDOWN_MS       10000   // MEDIUM：10s
#define HAPTIC_HIGH_COOLDOWN_MS   5000   // HIGH：5s
#define HAPTIC_CRITICAL_CD_MS     1500   // CRITICAL：1.5s（紧急连振）

// DRV2605L LRA 波形索引（参考数据手册）
#define WAVEFORM_MEDIUM          14    // 中等单脉冲
#define WAVEFORM_HIGH            47    // 强单脉冲
#define WAVEFORM_CRITICAL        49    // 强连振（极危）

void TaskHaptic_Run(void *arg) {
    ESP_LOGI(TAG, "Haptic task started");

    DecisionResult_t d;
    TickType_t last_tick = 0;

    for (;;) {
        if (xQueueReceive(q_decision_haptic, &d, portMAX_DELAY) != pdTRUE) continue;

        if (d.level == DANGER_SAFE) {
            last_tick = 0;  // 安全档重置冷却
            continue;
        }

        uint32_t cooldown_ms;
        uint8_t  waveform;
        switch (d.level) {
            case DANGER_CRITICAL: cooldown_ms = HAPTIC_CRITICAL_CD_MS; waveform = WAVEFORM_CRITICAL; break;
            case DANGER_HIGH:     cooldown_ms = HAPTIC_HIGH_COOLDOWN_MS; waveform = WAVEFORM_HIGH;     break;
            case DANGER_MEDIUM:   cooldown_ms = HAPTIC_COOLDOWN_MS;      waveform = WAVEFORM_MEDIUM;   break;
            default:              continue;
        }

        TickType_t now = xTaskGetTickCount();
        if (last_tick != 0 &&
            (now - last_tick) * portTICK_PERIOD_MS < cooldown_ms) {
            continue;
        }

        drv_haptic_play(waveform);
        last_tick = now;
        ESP_LOGI(TAG, "Haptic lvl=%d waveform=%d cooldown=%ums",
                 d.level, waveform, (unsigned)cooldown_ms);

        static uint32_t s_p0_cnt = 0;
        if (++s_p0_cnt % 20 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
    }
}
