#ifdef FEATURE_BLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "drv_ble.h"

static const char *TAG = "task_ble";

#define BLE_DEDUP_INTERVAL_MS    5000  // 同等级事件去重间隔

void TaskBLE_Run(void *arg) {
    ESP_LOGI(TAG, "BLE task started");

    DecisionResult_t decision;
    DangerLevel_t last_sent_level = DANGER_SAFE;
    TickType_t last_send_tick = 0;

    for (;;) {
        if (xQueueReceive(q_decision_ble, &decision, portMAX_DELAY) != pdTRUE) continue;

        // 安全等级不推送
        if (decision.level == DANGER_SAFE) continue;

        TickType_t now = xTaskGetTickCount();
        bool same_level = (decision.level == last_sent_level);
        bool within_dedup = ((now - last_send_tick) * portTICK_PERIOD_MS < BLE_DEDUP_INTERVAL_MS);

        if (same_level && within_dedup) continue;

        // 等级过滤通过，发送 BLE 帧
        drv_ble_send_event(&decision);
        last_sent_level = decision.level;
        last_send_tick = now;
    }
}

#endif // FEATURE_BLE
