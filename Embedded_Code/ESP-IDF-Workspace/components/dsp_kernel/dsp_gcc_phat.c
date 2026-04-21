#include "dsp_gcc_phat.h"
#include "dsp_common.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "dsps_fft2r.h"

static const char *TAG = "dsp_gcc_phat";

esp_err_t dsp_gcc_phat_init(DSPGccPhat_t *inst) {
    esp_err_t ret = dsp_fft_ensure_init();
    if (ret != ESP_OK) return ret;
    memset(inst, 0, sizeof(DSPGccPhat_t));
    inst->initialized = true;
    ESP_LOGD(TAG, "GCC-PHAT init OK");
    return ESP_OK;
}

void dsp_gcc_phat_compute(DSPGccPhat_t *inst,
                            const float *ch_l, const float *ch_r,
                            float *gcc_out, int *tdoa_out) {
    // 步骤 1：打包实值输入为复数（零补至 FFT_LEN，输入长度 HOP_LEN）
    for (int i = 0; i < DSP_FFT_LEN; i++) {
        if (i < DSP_HOP_LEN) {
            inst->buf_l[2 * i]     = ch_l[i];
            inst->buf_r[2 * i]     = ch_r[i];
        } else {
            inst->buf_l[2 * i]     = 0.0f;  // 零补
            inst->buf_r[2 * i]     = 0.0f;
        }
        inst->buf_l[2 * i + 1] = 0.0f;
        inst->buf_r[2 * i + 1] = 0.0f;
    }

    // 步骤 2：双通道正向 FFT
    dsps_fft2r_fc32(inst->buf_l, DSP_FFT_LEN);
    dsps_bit_rev_fc32(inst->buf_l, DSP_FFT_LEN);
    dsps_fft2r_fc32(inst->buf_r, DSP_FFT_LEN);
    dsps_bit_rev_fc32(inst->buf_r, DSP_FFT_LEN);

    // 步骤 3：互功率谱 + PHAT 归一化（conj(L) × R，再除以幅度）
    for (int k = 0; k < DSP_N_BINS; k++) {
        float re_l = inst->buf_l[2 * k], im_l = inst->buf_l[2 * k + 1];
        float re_r = inst->buf_r[2 * k], im_r = inst->buf_r[2 * k + 1];
        float re_c = re_l * re_r + im_l * im_r;
        float im_c = im_l * re_r - re_l * im_r;
        float mag  = sqrtf(re_c * re_c + im_c * im_c) + DOA_PHAT_EPS;
        inst->cross_buf[2 * k]     = re_c / mag;
        inst->cross_buf[2 * k + 1] = im_c / mag;
    }
    // 负频段（共轭对称）
    for (int k = DSP_N_BINS; k < DSP_FFT_LEN; k++) {
        inst->cross_buf[2 * k]     =  inst->cross_buf[2 * (DSP_FFT_LEN - k)];
        inst->cross_buf[2 * k + 1] = -inst->cross_buf[2 * (DSP_FFT_LEN - k) + 1];
    }

    // 步骤 4：IFFT（共轭 → 正向 FFT → /N，取实部）
    // 注：ESP-DSP 未提供 dsps_fft2r_inv_fc32 公开 API，此处用 IFFT(X) = conj(FFT(conj(X)))/N
    //     的等价恒等式实现，复杂度与一次正向 FFT 相同，不是退化路径。
    //     （M3 若 ESP-DSP 上游补齐 inv API，可替换为原生调用省去两次遍历。）
    for (int k = 0; k < DSP_FFT_LEN; k++) {
        inst->cross_buf[2 * k + 1] = -inst->cross_buf[2 * k + 1];  // 取共轭
    }
    dsps_fft2r_fc32(inst->cross_buf, DSP_FFT_LEN);
    dsps_bit_rev_fc32(inst->cross_buf, DSP_FFT_LEN);
    float inv_n = 1.0f / (float)DSP_FFT_LEN;
    for (int i = 0; i < DSP_FFT_LEN; i++) {
        inst->gcc_buf[i] = inst->cross_buf[2 * i] * inv_n;
    }
    if (gcc_out) memcpy(gcc_out, inst->gcc_buf, DSP_FFT_LEN * sizeof(float));

    // 步骤 5：在 [-MAX_TDOA, +MAX_TDOA] 内找峰值
    // max TDOA = mic_spacing / c * sample_rate ≈ 0.05/343*16000 ≈ 2 samples
    const int max_tdoa = (int)(DSP_MIC_SPACING_M / DSP_SPEED_OF_SOUND * DSP_SAMPLE_RATE) + 1;
    int   best_lag = 0;
    float best_val = inst->gcc_buf[0];
    for (int lag = -max_tdoa; lag <= max_tdoa; lag++) {
        int idx = (lag >= 0) ? lag : (DSP_FFT_LEN + lag);
        if (inst->gcc_buf[idx] > best_val) {
            best_val = inst->gcc_buf[idx];
            best_lag = lag;
        }
    }
    *tdoa_out = best_lag;
}

float dsp_gcc_phat_tdoa_to_angle(int tdoa, float mic_spacing_m, int sample_rate) {
    float delay_s = (float)tdoa / (float)sample_rate;
    float cos_theta = delay_s * DSP_SPEED_OF_SOUND / mic_spacing_m;
    if (cos_theta >  1.0f) cos_theta =  1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;
    return acosf(cos_theta) * 180.0f / M_PI - 90.0f;  // [-90, +90] deg
}
