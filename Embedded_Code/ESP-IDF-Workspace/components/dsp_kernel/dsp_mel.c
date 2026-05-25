#include "dsp_mel.h"
#include "dsp_common.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "dsps_fft2r.h"

static const char *TAG = "dsp_mel";

static float s_window[DSP_FFT_LEN];
static bool  s_window_ready = false;

extern const float mel_filterbank[DSP_N_MELS][DSP_N_BINS];

esp_err_t dsp_mel_init(DSPMel_t *inst) {
    esp_err_t ret = dsp_fft_ensure_init();
    if (ret != ESP_OK) return ret;

    if (!s_window_ready) {
        for (int i = 0; i < DSP_FFT_LEN; i++) {
            s_window[i] = 0.5f - 0.5f * cosf(2.0f * M_PI * i / (DSP_FFT_LEN - 1));
        }
        s_window_ready = true;
    }
    inst->initialized = true;
    ESP_LOGD(TAG, "Mel init OK, FFT_LEN=%d N_MELS=%d", DSP_FFT_LEN, DSP_N_MELS);
    return ESP_OK;
}

void dsp_mel_compute(DSPMel_t *inst, const float *input, float *log_mel_out) {
    // TODO M3 优化：当前用复数 FFT + 零虚部处理实值信号；ESP-DSP 提供
    //   dsps_fft2r_rfft_fc32（实数 FFT）可省 ~50% 内存 + ~50% 算力。
    // 步骤 1：汉明加窗 + 打包为复数格式（虚部=0）
    for (int i = 0; i < DSP_FFT_LEN; i++) {
        inst->fft_buf[2 * i]     = input[i] * s_window[i];
        inst->fft_buf[2 * i + 1] = 0.0f;
    }

    // 步骤 2：复数 FFT（原地）
    dsps_fft2r_fc32(inst->fft_buf, DSP_FFT_LEN);
    dsps_bit_rev_fc32(inst->fft_buf, DSP_FFT_LEN);

    // 步骤 3：提取功率谱（利用实值 FFT 的 Hermitian 对称性）
    inst->power[0] = inst->fft_buf[0] * inst->fft_buf[0];
    inst->power[DSP_N_BINS - 1] = inst->fft_buf[1] * inst->fft_buf[1];
    for (int k = 1; k < DSP_N_BINS - 1; k++) {
        float re = inst->fft_buf[2 * k];
        float im = inst->fft_buf[2 * k + 1];
        inst->power[k] = re * re + im * im;
    }

    // 步骤 4：Mel 滤波器组投影
    for (int m = 0; m < DSP_N_MELS; m++) {
        float sum = 0.0f;
        for (int k = 0; k < DSP_N_BINS; k++) {
            sum += mel_filterbank[m][k] * inst->power[k];
        }
        inst->mel_energy[m] = sum;
    }

    // 步骤 5：对数（10·log10，dB 域）
    for (int m = 0; m < DSP_N_MELS; m++) {
        log_mel_out[m] = 10.0f * log10f(inst->mel_energy[m] + 1e-10f);
    }
}
