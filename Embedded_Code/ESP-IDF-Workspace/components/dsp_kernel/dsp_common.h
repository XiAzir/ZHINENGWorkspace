#pragma once

#include "esp_err.h"
#include "dsp_config.h"

extern float g_fft_w_table[2 * DSP_FFT_LEN];

esp_err_t dsp_fft_ensure_init(void);
