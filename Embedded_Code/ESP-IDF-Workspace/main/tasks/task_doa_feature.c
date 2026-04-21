#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>
#include <string.h>
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"

#include "dsp_filter.h"
#include "dsp_mel.h"
#include "dsp_gcc_phat.h"
#include "dsp_noise_tracker.h"
#include "dsp_threat_tracker.h"

static const char *TAG = "task_doa";

// DSP 模块实例（静态分配，避免栈溢出）
static DSPFilter_t        g_hpf;
static DSPMel_t           g_mel;
static DSPGccPhat_t       g_doa;
static DSPThreatTracker_t g_threat;

// Mel FFT 滑窗：FFT_LEN=1024=2×AUDIO_FRAME_LEN，
// 每帧左移 AUDIO_FRAME_LEN 样本（50% overlap）后追加新帧。
static float s_mel_window[DSP_FFT_LEN];

// Mel 特征环形缓冲：按帧覆盖写入，送 AI 前按时间正序重组。
static int8_t s_mel_ring[MEL_BINS * MEL_FRAMES];
static int    s_ring_write_idx     = 0;   // 下一次写入的帧槽位（也是最旧帧位置）
static int    s_ring_filled        = 0;   // 已写入帧数，封顶 MEL_FRAMES
static int    s_frames_since_infer = 0;

_Static_assert(DSP_FFT_LEN == 2 * AUDIO_FRAME_LEN,
               "FFT_LEN must equal 2*AUDIO_FRAME_LEN for 50%% overlap Mel window");

void TaskDOA_Run(void *arg) {
    ESP_LOGI(TAG, "DOA/Feature task started on core %d", xPortGetCoreID());

    dsp_filter_hpf_init(&g_hpf);
    ESP_ERROR_CHECK(dsp_mel_init(&g_mel));
    ESP_ERROR_CHECK(dsp_gcc_phat_init(&g_doa));
    dsp_threat_init(&g_threat);

    memset(s_mel_window, 0, sizeof(s_mel_window));

    static AudioFrame_t   audio_in;
    static FeatureFrame_t feat_out;
    static float float_ch0[AUDIO_FRAME_LEN];
    static float float_ch1[AUDIO_FRAME_LEN];
    static float filtered_ch0[AUDIO_FRAME_LEN];
    static float filtered_ch1[AUDIO_FRAME_LEN];
    static float log_mel[MEL_BINS];
    static float gcc[DSP_FFT_LEN];
    int   tdoa;

    for (;;) {
        if (xQueueReceive(q_audio, &audio_in, portMAX_DELAY) != pdTRUE) continue;

        // PCM int16 → float32
        for (int i = 0; i < AUDIO_FRAME_LEN; i++) {
            float_ch0[i] = audio_in.pcm_ch0[i] / 32768.0f;
            float_ch1[i] = audio_in.pcm_ch1[i] / 32768.0f;
        }

        // IIR 高通 @ 100Hz（去除风噪/直流）
        dsp_filter_hpf_process(&g_hpf, float_ch0, filtered_ch0, AUDIO_FRAME_LEN);
        dsp_filter_hpf_process(&g_hpf, float_ch1, filtered_ch1, AUDIO_FRAME_LEN);

        // GCC-PHAT DOA（函数内部会零补到 FFT_LEN）
        dsp_gcc_phat_compute(&g_doa, filtered_ch0, filtered_ch1, gcc, &tdoa);
        float doa_rel = dsp_gcc_phat_tdoa_to_angle(
            tdoa, DSP_MIC_SPACING_M, DSP_SAMPLE_RATE);
        feat_out.doa_angle_deg = doa_rel;

        // 立即锁当前 Yaw_0 做世界坐标锚定（文档《关于危险时间和3DoF.md》§2.2 慢线程）。
        // 声学延迟几十毫秒，此时读到的 yaw 与 DOA 语义对齐；GUI 快线程用最新 yaw 补偿。
        feat_out.doa_world_deg = doa_rel + orientation_yaw_get();

        // Mel 滑窗（50% overlap）：把上一帧后半作为新帧前半
        memmove(s_mel_window, s_mel_window + AUDIO_FRAME_LEN,
                AUDIO_FRAME_LEN * sizeof(float));
        memcpy(s_mel_window + AUDIO_FRAME_LEN, filtered_ch0,
               AUDIO_FRAME_LEN * sizeof(float));

        dsp_mel_compute(&g_mel, s_mel_window, log_mel);

        // INT8 量化（roundf 与 Python 端 round_nearest 一致，避免截断偏差）
        int frame_slot = s_ring_write_idx * MEL_BINS;
        for (int m = 0; m < MEL_BINS; m++) {
            float q = roundf(log_mel[m] / DSP_INT8_SCALE) + (float)DSP_INT8_OFFSET;
            if (q >  127.0f) q =  127.0f;
            if (q < -128.0f) q = -128.0f;
            s_mel_ring[frame_slot + m] = (int8_t)q;
        }
        s_ring_write_idx = (s_ring_write_idx + 1) % MEL_FRAMES;
        if (s_ring_filled < MEL_FRAMES) s_ring_filled++;
        s_frames_since_infer++;

        // 威胁跟踪（复用 power_spec，避免二次 FFT）
        int   last_cls;
        float last_conf;
        ai_state_get(&last_cls, &last_conf);
        dsp_threat_update(&g_threat, g_mel.power, DSP_N_BINS,
                          last_cls, last_conf, &feat_out.threat);

        // 滑动推理：ring 满且满足推理 hop 才送 AI（首次延迟 ≈ MEL_FRAMES × 32ms）
        if (s_ring_filled >= MEL_FRAMES &&
            s_frames_since_infer >= AI_INFER_HOP_FRAMES) {
            // 环形 → 时间正序：write_idx 指向下一个要覆盖的槽位，也即最旧帧
            int start = s_ring_write_idx;
            for (int f = 0; f < MEL_FRAMES; f++) {
                int src = ((start + f) % MEL_FRAMES) * MEL_BINS;
                int dst = f * MEL_BINS;
                memcpy(feat_out.mel_feature + dst,
                       s_mel_ring + src, MEL_BINS);
            }
            feat_out.timestamp_ms = audio_in.timestamp_ms;
            xQueueSend(q_feature, &feat_out, 0);
            s_frames_since_infer = 0;
        }
    }
}
