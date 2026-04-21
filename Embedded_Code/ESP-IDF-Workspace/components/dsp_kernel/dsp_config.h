#pragma once

#include "dsps_fft2r.h"
#include "dsps_biquad.h"
#include "dsps_mulc.h"
#include "dsps_wind.h"

// ── 参数（与 Python 端严格对应，详见 AI模型超参数与溯源档案.md）──
#define DSP_SAMPLE_RATE     16000
#define DSP_FFT_LEN         1024
#define DSP_HOP_LEN         512
#define DSP_N_MELS          64
#define DSP_N_BINS          513     // FFT_LEN/2 + 1
#define DSP_IIR_CUTOFF_HZ   100
#define DSP_IIR_NUM_STAGES  2
#define DSP_MIC_SPACING_M   0.05f
#define DSP_SPEED_OF_SOUND  343.0f
#define DSP_INT8_SCALE      0.02773f
#define DSP_INT8_OFFSET     (-128)
#define DSP_SPL_CAL_DB      94.0f
#define DOA_PHAT_EPS        1e-6f

// ── Doppler 跟踪参数 ────────────────────────────────────────
#define DSP_DOPPLER_F_LO     200.0f
#define DSP_DOPPLER_F_HI     800.0f
#define DSP_DOPPLER_HIST_LEN 8
#define DSP_SPL_RING_LEN     62

// ── TTC / 二维威胁矩阵参数（文档《关于危险时间和3DoF.md》§1.5–§1.6）──
#define TTC_MEDIAN_WIN       7        // 原始 TTC 滑动中位数窗口（5~10 帧）
#define TTC_CLAMP_MAX_S      30.0f    // 原始 TTC 上钳制，防 INF 污染滤波
#define TTC_YELLOW_S         5.0f     // >5s  → 低/黄分界
#define TTC_ORANGE_S         3.0f     // 3~5s → 黄/橙分界
#define TTC_RED_S            1.5f     // 1.5~3s → 橙/红分界
#define SPL_HI_DBA           75.0f    // 二维矩阵 SPL 分档（硬件校准后调）
#define TTC_SLOPE_EPS_DBPS   0.1f     // dSPL/dt 死区（低于此视为稳态/远离）
