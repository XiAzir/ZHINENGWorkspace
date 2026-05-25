#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"
#include "ai_wrapper.h"

static const char *TAG = "task_ai";

#define AI_CLASS_CONFIRM_FRAMES 2
#define AI_CLASS_GAP_MIN 0.08f

static const char *ai_class_name(int cls) {
    switch (cls) {
    case 0: return "CAR_HORN";
    case 1: return "SIREN";
    case 2: return "AMBIENT";
    default: return "UNKNOWN";
    }
}

// AI 预测状态由 task_ai 发布、task_doa_feature（Core0）读取。
// int 与 float 各 32-bit，用 __atomic 提供 SMP 双核间的原子性 + release/acquire 可见性。
// （float 经 uint32_t 位模式中转，RISC-V 对齐 32-bit 存取是硬件原子的。）
static int      s_last_pred_class     = 2;     // background
static uint32_t s_last_pred_conf_bits = 0;     // float bit pattern

static void ai_stabilize_prediction(int raw_cls, float raw_conf, float raw_gap, bool force,
                                     int *stable_cls_out, float *stable_conf_out) {
    static int      s_stable_cls      = 2;   // background
    static float    s_stable_conf     = 0.0f;
    static int      s_candidate_cls   = 2;
    static float    s_candidate_conf   = 0.0f;
    static float    s_candidate_gap   = 0.0f;
    static uint8_t  s_candidate_hits   = 0;

    if (force) {
        s_stable_cls      = raw_cls;
        s_stable_conf     = raw_conf;
        s_candidate_cls   = raw_cls;
        s_candidate_conf  = raw_conf;
        s_candidate_gap   = raw_gap;
        s_candidate_hits  = 0;
    } else if (raw_cls == s_stable_cls) {
        s_stable_conf   = raw_conf;
        s_candidate_cls  = raw_cls;
        s_candidate_conf = raw_conf;
        s_candidate_gap  = raw_gap;
        s_candidate_hits = 0;
    } else if (raw_cls == s_candidate_cls) {
        if (s_candidate_hits < UINT8_MAX) s_candidate_hits++;
        s_candidate_conf = raw_conf;
        s_candidate_gap  = raw_gap;
        if (s_candidate_hits >= AI_CLASS_CONFIRM_FRAMES &&
            s_candidate_gap >= AI_CLASS_GAP_MIN) {
            s_stable_cls  = s_candidate_cls;
            s_stable_conf = s_candidate_conf;
            s_candidate_hits = 0;
        }
    } else {
        s_candidate_cls   = raw_cls;
        s_candidate_conf   = raw_conf;
        s_candidate_gap    = raw_gap;
        s_candidate_hits   = 1;
    }

    *stable_cls_out  = s_stable_cls;
    *stable_conf_out = s_stable_conf;
}

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

        int   raw_cls;
        float raw_conf;
        float raw_gap;
        if (!feat.audio_valid) {
            raw_cls = 2;
            raw_conf = 1.0f;
            raw_gap = 1.0f;
        } else {
            AI_Run(feat.mel_feature, &raw_cls, &raw_conf, &raw_gap);
        }
        ai_stabilize_prediction(raw_cls, raw_conf, raw_gap, !feat.audio_valid,
                                &result.pred_class, &result.confidence);
        result.timestamp_ms   = feat.timestamp_ms;
        result.doa_angle_deg  = feat.doa_angle_deg;
        result.doa_world_deg  = feat.doa_world_deg;
        result.threat         = feat.threat;

        // 跨核发布最新预测（供 task_doa_feature 中的 ThreatTracker 使用）
        ai_state_set(result.pred_class, result.confidence);

        // 送决策队列
        xQueueSend(q_ai_result, &result, 0);

        static uint32_t s_p0_cnt = 0;
        static uint32_t s_ai_diag_cnt = 0;
        if (++s_ai_diag_cnt % 10 == 0) {
            ESP_LOGI(TAG,
                     "AI diag valid=%d raw=%s conf=%.3f gap=%.3f stable=%s conf=%.3f",
                     feat.audio_valid ? 1 : 0,
                     ai_class_name(raw_cls), raw_conf, raw_gap,
                     ai_class_name(result.pred_class), result.confidence);
        }
        if (++s_p0_cnt % 100 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
    }
}
