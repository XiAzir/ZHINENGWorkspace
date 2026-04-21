#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_config.h"

typedef struct {
    float buf_l[2 * DSP_FFT_LEN];    // 左麦复数 FFT 缓冲
    float buf_r[2 * DSP_FFT_LEN];    // 右麦复数 FFT 缓冲
    float cross_buf[2 * DSP_FFT_LEN]; // 互功率谱（PHAT 归一化后）
    float gcc_buf[DSP_FFT_LEN];       // GCC 序列（实值）
    bool  initialized;
} DSPGccPhat_t;

esp_err_t dsp_gcc_phat_init(DSPGccPhat_t *inst);

// 返回 GCC 序列和 TDOA（整数采样点延迟，正值=左麦超前）
void dsp_gcc_phat_compute(DSPGccPhat_t *inst,
                           const float *ch_l, const float *ch_r,
                           float *gcc_out, int *tdoa_out);

// TDOA → DOA 角度（deg），基于麦间距和声速
float dsp_gcc_phat_tdoa_to_angle(int tdoa, float mic_spacing_m, int sample_rate);
