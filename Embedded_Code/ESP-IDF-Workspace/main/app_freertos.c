#include "app_freertos.h"
#include "esp_log.h"
#include <string.h>

QueueHandle_t q_audio;
QueueHandle_t q_feature;
QueueHandle_t q_ai_result;
QueueHandle_t q_decision_gui;
QueueHandle_t q_decision_haptic;
QueueHandle_t q_decision_ble;

TaskHandle_t h_task_audio;
TaskHandle_t h_task_doa;
TaskHandle_t h_task_ai;
TaskHandle_t h_task_decision;
TaskHandle_t h_task_orient;
TaskHandle_t h_task_gui_fast;

// float 经 uint32_t 位模式传递；__atomic 保证 SMP 双核间可见性。
// 32-bit 对齐存取在 RISC-V 上硬件原子，float / bits 大小相同。
static uint32_t s_yaw_bits        = 0;  // orientation_yaw_get 默认返回 0.0f
static uint32_t s_theta_world_bits = 0;

void orientation_yaw_set(float yaw_deg) {
    uint32_t bits;
    memcpy(&bits, &yaw_deg, sizeof(bits));
    __atomic_store_n(&s_yaw_bits, bits, __ATOMIC_RELEASE);
}

float orientation_yaw_get(void) {
    uint32_t bits = __atomic_load_n(&s_yaw_bits, __ATOMIC_ACQUIRE);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

void theta_world_set(float theta_deg) {
    uint32_t bits;
    memcpy(&bits, &theta_deg, sizeof(bits));
    __atomic_store_n(&s_theta_world_bits, bits, __ATOMIC_RELEASE);
}

float theta_world_get(void) {
    uint32_t bits = __atomic_load_n(&s_theta_world_bits, __ATOMIC_ACQUIRE);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

void app_freertos_init(void) {
    q_audio            = xQueueCreate(Q_AUDIO_DEPTH,    sizeof(AudioFrame_t));
    q_feature          = xQueueCreate(Q_FEATURE_DEPTH,  sizeof(FeatureFrame_t));
    q_ai_result        = xQueueCreate(Q_AI_DEPTH,       sizeof(AIResult_t));
    q_decision_gui     = xQueueCreate(Q_DECISION_DEPTH, sizeof(DecisionResult_t));
    q_decision_haptic  = xQueueCreate(Q_DECISION_DEPTH, sizeof(DecisionResult_t));
    q_decision_ble     = xQueueCreate(Q_DECISION_DEPTH, sizeof(DecisionResult_t));
    configASSERT(q_audio && q_feature && q_ai_result &&
                 q_decision_gui && q_decision_haptic && q_decision_ble);
}
