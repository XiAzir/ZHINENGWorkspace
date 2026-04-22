#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "ai_wrapper.h"

static const char *TAG = "task_ai";

// AI 预测状态由 task_ai 发布、task_doa_feature（Core0）读取。
// int 与 float 各 32-bit，用 __atomic 提供 SMP 双核间的原子性 + release/acquire 可见性。
// （float 经 uint32_t 位模式中转，RISC-V 对齐 32-bit 存取是硬件原子的。）
static int      s_last_pred_class     = 2;     // background
static uint32_t s_last_pred_conf_bits = 0;     // float bit pattern

void ai_state_set(int cls, float conf) {
    uint32_t bits;
    memcpy(&bits, &conf, sizeof(bits));
    __atomic_store_n(&s_last_pred_class,     cls,  __ATOMIC_RELEASE);
    __atomic_store_n(&s_last_pred_conf_bits, bits, __ATOMIC_RELEASE);
}

void ai_state_get(int *cls_out, float *conf_out) {
    *cls_out        = __atomic_load_n(&s_last_pred_class,     __ATOMIC_ACQUIRE);
    uint32_t bits   = __atomic_load_n(&s_last_pred_conf_bits, __ATOMIC_ACQUIRE);
    memcpy(conf_out, &bits, sizeof(*conf_out));
}

void TaskAI_Run(void *arg) {
    ESP_LOGI(TAG, "AI task started on core %d", xPortGetCoreID());

    FeatureFrame_t feat;
    AIResult_t     result;

    for (;;) {
        if (xQueueReceive(q_feature, &feat, portMAX_DELAY) != pdTRUE) continue;

        AI_Run(feat.mel_feature, &result.pred_class, &result.confidence);
        result.timestamp_ms   = feat.timestamp_ms;
        result.doa_angle_deg  = feat.doa_angle_deg;
        result.doa_world_deg  = feat.doa_world_deg;
        result.threat         = feat.threat;

        // 跨核发布最新预测（供 task_doa_feature 中的 ThreatTracker 使用）
        ai_state_set(result.pred_class, result.confidence);

        // 送决策队列
        xQueueSend(q_ai_result, &result, 0);

        static uint32_t s_p0_cnt = 0;
        if (++s_p0_cnt % 100 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
    }
}
