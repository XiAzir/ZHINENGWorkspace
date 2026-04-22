#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "drv_oled.h"

static const char *TAG = "task_gui";

// 危险等级显示文本（ASCII，中文需字库，M5 完善）
static const char *s_level_text[] = {"SAFE", "CAUTION", "DANGER", "CRITICAL"};
static const char *s_class_text[] = {"CAR HORN", "SIREN", "AMBIENT"};

void TaskGUI_Run(void *arg) {
    ESP_LOGI(TAG, "GUI slow task started on core %d", xPortGetCoreID());

    DecisionResult_t d;

    for (;;) {
        if (xQueueReceive(q_decision_gui, &d, portMAX_DELAY) != pdTRUE) continue;

        // 发布世界锚定角度 → 快线程每帧读取 + 补偿当前 Yaw
        theta_world_set(d.doa_world_deg);

        // 状态区（非箭头）只在 decision 到来时刷一次；快线程专注箭头
        drv_oled_draw_status(d.level, d.pred_class, d.confidence, d.doa_angle_deg);

        static uint32_t s_p0_cnt = 0;
        if (++s_p0_cnt % 100 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
        ESP_LOGI(TAG,
                 "Lvl=%-8s Cls=%-8s Conf=%3.0f%% Rel=%+.1f Wld=%+.1f TTC=%4.1fs SPL=%.0fdBA%s",
                 s_level_text[d.level],
                 s_class_text[d.pred_class],
                 d.confidence * 100.0f,
                 d.doa_angle_deg,
                 d.doa_world_deg,
                 d.ttc_filtered_s,
                 d.spl_dba_now,
                 d.rear_warning ? " [REAR]" : "");
    }
}
