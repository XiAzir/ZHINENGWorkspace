#include "dsp_noise_tracker.h"
#include <string.h>
#include <float.h>

void dsp_noise_tracker_init(DSPNoiseTracker_t *inst) {
    memset(inst, 0, sizeof(DSPNoiseTracker_t));
    inst->noise_floor_db = -60.0f;  // 初始估计
}

static float median_f(float *arr, int n) {
    // 简单冒泡排序求中位数（窗口小，不影响性能）
    float buf[NOISE_WINDOW_LEN];
    memcpy(buf, arr, n * sizeof(float));
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - 1 - i; j++) {
            if (buf[j] > buf[j + 1]) {
                float tmp = buf[j]; buf[j] = buf[j + 1]; buf[j + 1] = tmp;
            }
        }
    }
    return buf[n / 2];
}

void dsp_noise_tracker_update(DSPNoiseTracker_t *inst) {
    // M4 后：将当前环境 SPL 推入滑动窗口，取中位数更新噪声基线
    // 当前为占位实现
    if (inst->count < NOISE_WINDOW_LEN) inst->count++;
    inst->noise_floor_db = median_f(inst->spl_history, inst->count);
}

float dsp_noise_tracker_get_floor(const DSPNoiseTracker_t *inst) {
    return inst->noise_floor_db;
}
