#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "math.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"

static const char *TAG = "task_decision";

// class × ThreatLevel → DangerLevel
// 列：GREEN / YELLOW / ORANGE / RED（文档 §1.5）
static const DangerLevel_t s_danger_matrix[3][4] = {
    // GREEN          YELLOW          HIGH            CRITICAL
    { DANGER_SAFE,    DANGER_MEDIUM,  DANGER_HIGH,    DANGER_CRITICAL }, // 0: car_horn
    { DANGER_SAFE,    DANGER_MEDIUM,  DANGER_HIGH,    DANGER_CRITICAL }, // 1: siren
    { DANGER_SAFE,    DANGER_SAFE,    DANGER_SAFE,    DANGER_SAFE     }, // 2: background
};

static DangerLevel_t compute_danger(int pred_class, float confidence,
                                    ThreatLevel_t tl, float doa_angle) {
    if (confidence < 0.5f) return DANGER_SAFE;
    if (pred_class < 0 || pred_class > 2) return DANGER_SAFE;
    if ((int)tl < 0 || (int)tl > 3)       tl = THREAT_GREEN;

    DangerLevel_t base = s_danger_matrix[pred_class][(int)tl];

    // 正前方声源降一档（听障用户视觉可见，优先保留外围注意力）
    if (fabsf(doa_angle) <= 60.0f && base > DANGER_SAFE) {
        base = (DangerLevel_t)((int)base - 1);
    }
    return base;
}

void TaskDecision_Run(void *arg) {
    ESP_LOGI(TAG, "Decision task started on core %d", xPortGetCoreID());

    AIResult_t       ai_in;
    DecisionResult_t out;
    static uint32_t  s_p0_cnt = 0;

    for (;;) {
        if (xQueueReceive(q_ai_result, &ai_in, portMAX_DELAY) != pdTRUE) continue;

        out.pred_class      = ai_in.pred_class;
        out.confidence      = ai_in.confidence;
        out.doa_angle_deg   = ai_in.doa_angle_deg;
        out.doa_world_deg   = ai_in.doa_world_deg;
        out.threat_level    = ai_in.threat.threat_level;
        out.ttc_filtered_s  = ai_in.threat.ttc_filtered_s;
        out.spl_dba_now     = ai_in.threat.spl_dba_now;
        out.rear_warning    = false;  // M5：线阵麦阵无法区分前后；需第三麦或波束成形
        out.timestamp_ms    = ai_in.timestamp_ms;

        out.level = compute_danger(ai_in.pred_class, ai_in.confidence,
                                   ai_in.threat.threat_level, ai_in.doa_angle_deg);

        xQueueSend(q_decision_gui,    &out, 0);
        xQueueSend(q_decision_haptic, &out, 0);
        xQueueSend(q_decision_ble,    &out, 0);

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(TAG, "[P0] e2e_ms=%u", (unsigned)(now_ms - ai_in.timestamp_ms));
        if (++s_p0_cnt % 100 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes heap_free=%u psram_free=%u",
                     uxTaskGetStackHighWaterMark(NULL) * 4,
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }
    }
}
