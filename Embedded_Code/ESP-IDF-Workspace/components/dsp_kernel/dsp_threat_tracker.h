#pragma once
#include "dsp_config.h"
#include <stdbool.h>

// ── 既有 TTC 粗档（dBps 阈值驱动，保留兼容）──────────────────
typedef enum {
    TTC_FAR = 0,
    TTC_STABLE,
    TTC_APPROACHING,
    TTC_IMMINENT,
} TTCLevel_t;

// ── 文档 §1.5 的 4 档威胁等级（TTC × SPL 二维矩阵输出）──────
typedef enum {
    THREAT_GREEN  = 0,   // 低
    THREAT_YELLOW = 1,
    THREAT_ORANGE = 2,
    THREAT_RED    = 3,   // 紧急
} ThreatLevel_t;

typedef struct {
    float         radial_velocity_mps;   // Doppler 径向速度（带符号，正=靠近）
    float         approach_rate_dbps;    // dSPL/dt 最小二乘斜率
    float         spl_dba_now;           // 当前 A 加权 SPL
    float         ttc_raw_s;             // 原始 TTC（已钳制到 [0, TTC_CLAMP_MAX_S]）
    float         ttc_filtered_s;        // 滑动中位数滤波后
    float         doppler_confidence;
    ThreatLevel_t threat_level;          // 二维矩阵输出
    TTCLevel_t    ttc_level;             // 保留兼容（过渡期）
} ThreatInfo_t;

typedef struct {
    float spl_dba_ring[DSP_SPL_RING_LEN];
    int   spl_ring_count;
    float peak_freq_hist[DSP_DOPPLER_HIST_LEN];
    int   peak_hist_count;
    float ttc_raw_ring[TTC_MEDIAN_WIN];
    int   ttc_ring_count;
    int   last_class;
    float last_class_conf;
} DSPThreatTracker_t;

void dsp_threat_init(DSPThreatTracker_t *inst);
void dsp_threat_update(DSPThreatTracker_t *inst,
                       const float *power_spec,
                       int n_bins,
                       int pred_class,
                       float pred_conf,
                       ThreatInfo_t *out);
