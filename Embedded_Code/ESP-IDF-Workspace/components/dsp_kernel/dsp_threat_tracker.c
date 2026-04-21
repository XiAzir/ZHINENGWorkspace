#include "dsp_threat_tracker.h"
#include <math.h>
#include <string.h>
#include <float.h>

// A 加权权重表（513 bins，由 Python 生成后填入）
// 生成命令见计划文档 §1.7；此处用全 1 占位，M3 替换
// 注：`[0 ... N-1] = val` 是 GCC/Clang 扩展的 designated range initializer，
// ESP-IDF 工具链支持；M3 替换为真实权重表后即可移除此扩展。
static const float a_weight_table[DSP_N_BINS] = {
    [0 ... (DSP_N_BINS - 1)] = 1.0f
};

// 先进先出滑窗：未满时尾部追加；满后整体左移一位，新值写入末位。
// 代价 O(N)，但 N≤62、每 32ms 调用一次，开销可忽略；
// 好处是 ring[0..count-1] 始终按时间正序，ls_slope / mean 无需额外索引换算。
static void push_ring_f(float *ring, int *count, int max_len, float val) {
    if (*count < max_len) {
        ring[*count] = val;
        (*count)++;
    } else {
        memmove(ring, ring + 1, (max_len - 1) * sizeof(float));
        ring[max_len - 1] = val;
    }
}

static float mean_ring_f(const float *ring, int count) {
    if (count == 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < count; i++) sum += ring[i];
    return sum / count;
}

static float ls_slope(const float *y, int n, float dt) {
    if (n < 2) return 0.0f;
    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (int i = 0; i < n; i++) {
        float x = i * dt;
        sum_x  += x;
        sum_y  += y[i];
        sum_xx += x * x;
        sum_xy += x * y[i];
    }
    float denom = n * sum_xx - sum_x * sum_x;
    if (fabsf(denom) < 1e-12f) return 0.0f;
    return (n * sum_xy - sum_x * sum_y) / denom;
}

// 取 n 个浮点数的中位数（n ≤ TTC_MEDIAN_WIN ≤ 10，插入排序 O(n²) 即可）
// 中位数对脉冲噪声免疫，是文档 §1.5 明确要求的滤波方式。
static float median_n(const float *src, int n) {
    if (n <= 0) return 0.0f;
    float buf[TTC_MEDIAN_WIN];
    int len = n < TTC_MEDIAN_WIN ? n : TTC_MEDIAN_WIN;
    memcpy(buf, src, len * sizeof(float));
    for (int i = 1; i < len; i++) {
        float key = buf[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > key) { buf[j + 1] = buf[j]; j--; }
        buf[j + 1] = key;
    }
    if (len & 1) return buf[len / 2];
    return 0.5f * (buf[len / 2 - 1] + buf[len / 2]);
}

// 二维矩阵：TTC × SPL → ThreatLevel（文档 §1.6）
static ThreatLevel_t map_threat(float ttc_s, float spl_dba) {
    bool spl_hi = (spl_dba >= SPL_HI_DBA);
    if (ttc_s <= TTC_RED_S)                       return THREAT_RED;
    if (ttc_s <= TTC_ORANGE_S)                    return spl_hi ? THREAT_RED    : THREAT_ORANGE;
    if (ttc_s <= TTC_YELLOW_S)                    return spl_hi ? THREAT_ORANGE : THREAT_YELLOW;
    /* ttc_s > TTC_YELLOW_S */                    return spl_hi ? THREAT_YELLOW : THREAT_GREEN;
}

void dsp_threat_init(DSPThreatTracker_t *inst) {
    memset(inst, 0, sizeof(DSPThreatTracker_t));
    inst->last_class = 2;  // background
}

void dsp_threat_update(DSPThreatTracker_t *inst,
                        const float *power_spec,
                        int n_bins,
                        int pred_class,
                        float pred_conf,
                        ThreatInfo_t *out) {
    out->radial_velocity_mps = 0.0f;
    out->doppler_confidence  = 0.0f;
    out->ttc_level           = TTC_STABLE;

    float dt = (float)DSP_HOP_LEN / (float)DSP_SAMPLE_RATE;  // ~32ms

    // ── Doppler 峰值跟踪 ──────────────────────────────────────
    if (pred_class != 2 && pred_conf >= 0.7f) {
        const int bin_lo = (int)(DSP_DOPPLER_F_LO * DSP_FFT_LEN / DSP_SAMPLE_RATE);
        const int bin_hi = (int)(DSP_DOPPLER_F_HI * DSP_FFT_LEN / DSP_SAMPLE_RATE);

        int   peak_bin = bin_lo;
        float peak_val = power_spec[bin_lo];
        float band_sum = 0.0f;
        for (int k = bin_lo; k <= bin_hi; k++) {
            band_sum += power_spec[k];
            if (power_spec[k] > peak_val) { peak_val = power_spec[k]; peak_bin = k; }
        }

        // 抛物线插值
        float delta = 0.0f;
        if (peak_bin > bin_lo && peak_bin < bin_hi) {
            float ym1 = power_spec[peak_bin - 1];
            float y0  = power_spec[peak_bin];
            float yp1 = power_spec[peak_bin + 1];
            delta = 0.5f * (ym1 - yp1) / (ym1 - 2.0f * y0 + yp1 + 1e-10f);
        }
        float peak_freq = (peak_bin + delta) * (float)DSP_SAMPLE_RATE / (float)DSP_FFT_LEN;

        push_ring_f(inst->peak_freq_hist, &inst->peak_hist_count,
                    DSP_DOPPLER_HIST_LEN, peak_freq);

        float df_dt = ls_slope(inst->peak_freq_hist, inst->peak_hist_count, dt);
        float f0    = mean_ring_f(inst->peak_freq_hist, inst->peak_hist_count);
        // 多普勒径向速度：v = -c · (df/dt) / f0（非相对论近似，df_dt 单位 Hz/s）
        out->radial_velocity_mps = -DSP_SPEED_OF_SOUND * df_dt / (f0 + 1e-6f);
        float band_mean = band_sum / (bin_hi - bin_lo + 1);
        out->doppler_confidence = peak_val / (band_mean + 1e-10f);
    } else {
        inst->peak_hist_count = 0;
    }

    // ── A 加权 SPL + 斜率 ─────────────────────────────────────
    float a_energy = 0.0f;
    for (int k = 0; k < n_bins; k++) {
        a_energy += a_weight_table[k] * power_spec[k];
    }
    float dba_now = 10.0f * log10f(a_energy + 1e-10f) + DSP_SPL_CAL_DB;
    out->spl_dba_now = dba_now;

    push_ring_f(inst->spl_dba_ring, &inst->spl_ring_count,
                DSP_SPL_RING_LEN, dba_now);

    out->approach_rate_dbps = ls_slope(inst->spl_dba_ring, inst->spl_ring_count, dt);

    // ── TTC 原始值（文档 §1.3：TTC = 8.686 / dSPL/dt）──────────
    // 死区处理：斜率很小或为负 → 声源未靠近 → TTC 置为上钳制值（安全侧）。
    // 鸣笛间歇的短暂负斜率通过随后的中位数滤波被压掉。
    float slope = out->approach_rate_dbps;
    float ttc_raw;
    if (slope > TTC_SLOPE_EPS_DBPS) {
        ttc_raw = 8.686f / slope;
        if (ttc_raw > TTC_CLAMP_MAX_S) ttc_raw = TTC_CLAMP_MAX_S;
    } else {
        ttc_raw = TTC_CLAMP_MAX_S;
    }
    out->ttc_raw_s = ttc_raw;

    // ── 中位数滤波 ────────────────────────────────────────────
    push_ring_f(inst->ttc_raw_ring, &inst->ttc_ring_count,
                TTC_MEDIAN_WIN, ttc_raw);
    out->ttc_filtered_s = median_n(inst->ttc_raw_ring, inst->ttc_ring_count);

    // ── 威胁等级（背景/低置信直接置 GREEN，不让噪声误报）──────
    if (pred_class == 2 || pred_conf < 0.5f) {
        out->threat_level = THREAT_GREEN;
    } else {
        out->threat_level = map_threat(out->ttc_filtered_s, dba_now);
    }

    // ── TTCLevel_t 兼容档位（沿用旧的 dBps 阈值逻辑）──────────
    if (slope > 6.0f && out->radial_velocity_mps > 5.0f)
        out->ttc_level = TTC_IMMINENT;
    else if (slope > 3.0f)
        out->ttc_level = TTC_APPROACHING;
    else if (fabsf(slope) <= 1.0f)
        out->ttc_level = TTC_STABLE;
    else
        out->ttc_level = TTC_FAR;
}
