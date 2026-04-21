#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "dsp_threat_tracker.h"   // ThreatLevel_t / TTCLevel_t / ThreatInfo_t

// ── 音频帧（task_audio → task_doa_feature）──────────────────
typedef struct {
    int16_t  pcm_ch0[AUDIO_FRAME_LEN];
    int16_t  pcm_ch1[AUDIO_FRAME_LEN];
    uint32_t timestamp_ms;
} AudioFrame_t;

// ── 特征帧（task_doa_feature → task_ai）────────────────────
typedef struct {
    int8_t       mel_feature[MEL_BINS * MEL_FRAMES];
    float        doa_angle_deg;    // 相对眼镜朝向（传感器坐标系）
    float        doa_world_deg;    // 绝对锚定：doa_angle_deg + Yaw_0（3DoF）
    ThreatInfo_t threat;
    uint32_t     timestamp_ms;
} FeatureFrame_t;

// ── AI 推理结果（task_ai → task_decision）───────────────────
// threat / DOA 从 FeatureFrame 透传，避免 decision 丢失 DSP 计算结果。
typedef struct {
    int          pred_class;       // 0=car_horn, 1=siren, 2=background
    float        confidence;
    float        doa_angle_deg;
    float        doa_world_deg;
    ThreatInfo_t threat;
    uint32_t     timestamp_ms;
} AIResult_t;

// ── 危险等级（四档，文档《关于危险时间和3DoF.md》§1.5）──────
typedef enum {
    DANGER_SAFE     = 0,
    DANGER_MEDIUM   = 1,
    DANGER_HIGH     = 2,
    DANGER_CRITICAL = 3,
} DangerLevel_t;

// ── 决策结果（task_decision → task_gui / task_haptic）────────
typedef struct {
    DangerLevel_t level;
    int           pred_class;
    float         confidence;
    float         doa_angle_deg;     // 相对角（若无 IMU 可直接用来画箭头）
    float         doa_world_deg;     // 绝对锚定角（GUI 快线程据此补偿头部）
    ThreatLevel_t threat_level;      // 透出供下游直接查表
    float         ttc_filtered_s;    // 供 BLE / 日志调试
    float         spl_dba_now;
    bool          rear_warning;
    uint32_t      timestamp_ms;
} DecisionResult_t;

// ── FreeRTOS 队列句柄（app_freertos.c 定义）─────────────────
// 决策结果分三条独立队列送往 GUI / Haptic / BLE，避免多消费者争抢。
extern QueueHandle_t q_audio;
extern QueueHandle_t q_feature;
extern QueueHandle_t q_ai_result;
extern QueueHandle_t q_decision_gui;
extern QueueHandle_t q_decision_haptic;
extern QueueHandle_t q_decision_ble;

// ── AI 状态跨核共享（task_ai 发布，task_doa_feature 读取）──────
// 内部以 __atomic_{store,load}_n 实现，保证 SMP 双核间的可见性与原子性。
void ai_state_set(int cls, float conf);
void ai_state_get(int *cls_out, float *conf_out);
