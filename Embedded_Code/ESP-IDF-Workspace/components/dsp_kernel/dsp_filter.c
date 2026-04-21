#include "dsp_filter.h"
#include <string.h>
#include <math.h>
#include <assert.h>
#include "esp_log.h"

static const char *TAG = "dsp_filter";

// 4 阶巴特沃斯 HPF @ 100Hz / 16kHz，Direct Form II Transpose
// 系数由 Python scipy.signal.butter(4, 100, 'high', fs=16000, output='sos') 生成
// 格式：{b0, b1, b2, a1, a2}（ESP-DSP biquad_f32 正 a 系数格式）
// TODO M3：确认 ESP-DSP dsps_biquad_f32 系数格式后替换为库调用
static const float s_iir_coeffs[DSP_IIR_NUM_STAGES][5] = {
    // Stage 0（由 Python 生成后填入）
    { 1.0f, -2.0f, 1.0f, -1.9999f, 0.9999f },
    // Stage 1
    { 1.0f, -2.0f, 1.0f, -1.9999f, 0.9999f },
};

void dsp_filter_hpf_init(DSPFilter_t *inst) {
    memset(inst->state, 0, sizeof(inst->state));
    inst->initialized = true;
    ESP_LOGD(TAG, "HPF initialized");
}

void dsp_filter_hpf_process(DSPFilter_t *inst, const float *input,
                              float *output, int len) {
    assert(len > 0 && len <= DSP_HOP_LEN);

    // 级联 biquad（Direct Form II Transpose）：
    //   stage 0 读 input，中间 stage 读写 tmp（in-place 安全），
    //   最后 stage 直写 output；省去尾部 memcpy 且避免读写同一缓冲。
    float tmp[DSP_HOP_LEN];
    const float *src = input;
    float       *dst;

    for (int stage = 0; stage < DSP_IIR_NUM_STAGES; stage++) {
        dst = (stage == DSP_IIR_NUM_STAGES - 1) ? output : tmp;

        const float b0 = s_iir_coeffs[stage][0];
        const float b1 = s_iir_coeffs[stage][1];
        const float b2 = s_iir_coeffs[stage][2];
        const float a1 = s_iir_coeffs[stage][3];
        const float a2 = s_iir_coeffs[stage][4];
        float w1 = inst->state[stage * 2];
        float w2 = inst->state[stage * 2 + 1];

        for (int i = 0; i < len; i++) {
            float w0 = src[i] - a1 * w1 - a2 * w2;
            dst[i]   = b0 * w0 + b1 * w1 + b2 * w2;
            w2 = w1;
            w1 = w0;
        }
        inst->state[stage * 2]     = w1;
        inst->state[stage * 2 + 1] = w2;
        src = dst;
    }
}
