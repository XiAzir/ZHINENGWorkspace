#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "dsp_config.h"

typedef struct {
    float fft_buf[2 * DSP_FFT_LEN];    // 复数 FFT 缓冲（交替 Re/Im）
    float power[DSP_N_BINS];            // 功率谱（公开供 ThreatTracker 复用）
    float mel_energy[DSP_N_MELS];
    bool  initialized;
} DSPMel_t;

esp_err_t dsp_mel_init(DSPMel_t *inst);
void      dsp_mel_compute(DSPMel_t *inst, const float *input, float *log_mel_out);
