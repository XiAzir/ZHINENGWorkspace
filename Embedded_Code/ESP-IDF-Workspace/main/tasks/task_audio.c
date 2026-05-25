#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"

#ifdef USE_SIMULATION
#include "drv_sim.h"
#else
#include "drv_pdm_dma.h"
#endif

static const char *TAG = "task_audio";

#ifndef USE_SIMULATION
typedef struct {
    TickType_t start_tick;
    uint32_t frames;
    uint32_t samples_per_ch;
    int16_t min_ch[AUDIO_PDM_CAPTURE_CHANNELS];
    int16_t max_ch[AUDIO_PDM_CAPTURE_CHANNELS];
    int32_t peak_ch[AUDIO_PDM_CAPTURE_CHANNELS];
    int64_t sum_ch[AUDIO_PDM_CAPTURE_CHANNELS];
    int64_t sumsq_ch[AUDIO_PDM_CAPTURE_CHANNELS];
    uint32_t short_reads;
    uint32_t read_errors;
    uint32_t zero_frames;
} AudioDiagWindow_t;

static void audio_diag_reset(AudioDiagWindow_t *diag, TickType_t start_tick) {
    memset(diag, 0, sizeof(*diag));
    diag->start_tick = start_tick;
    for (int ch = 0; ch < AUDIO_PDM_CAPTURE_CHANNELS; ch++) {
        diag->min_ch[ch] = INT16_MAX;
        diag->max_ch[ch] = INT16_MIN;
    }
}
#endif

void TaskAudio_Run(void *arg) {
    ESP_LOGI(TAG, "Audio task started on core %d", xPortGetCoreID());

    AudioFrame_t frame;
    frame.timestamp_ms = 0;
#ifndef USE_SIMULATION
    AudioDiagWindow_t diag;
    audio_diag_reset(&diag, xTaskGetTickCount());
    const int pdm_channels = drv_pdm_dma_get_channels();
    const int downstream_ch0 = drv_pdm_dma_get_downstream_ch0();
    const int downstream_ch1 = drv_pdm_dma_get_downstream_ch1();
    const bool dual_slot_probe = drv_pdm_dma_is_dual_slot_probe();
    ESP_LOGI(TAG, "Audio capture uses %d PDM channel(s), downstream=%d/%d, dual_slot_probe=%d",
             pdm_channels, downstream_ch0, downstream_ch1, dual_slot_probe);
#endif

    for (;;) {
#ifdef USE_SIMULATION
        drv_sim_read_frame(frame.pcm_ch0, frame.pcm_ch1, AUDIO_FRAME_LEN);
        frame.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        // 仿真下手动延迟以匹配 32ms 帧率（512 samples @ 16kHz = 32ms）
        vTaskDelay(pdMS_TO_TICKS(32));
#else
        size_t bytes_read = 0;
        // 从 DMA 读取活动 DATA 线的 LOW/HIGH 两个 PDM slot。
        int16_t interleaved[AUDIO_FRAME_LEN * AUDIO_PDM_CAPTURE_CHANNELS];
        const size_t expected_samples = AUDIO_FRAME_LEN * (size_t)pdm_channels;
        esp_err_t err = drv_pdm_dma_read(interleaved, expected_samples, &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
            diag.read_errors++;
            ESP_LOGE(TAG, "PDM read failed: %s", esp_err_to_name(err));
            continue;
        }

        const size_t expected_bytes = expected_samples * sizeof(int16_t);
        if (bytes_read < expected_bytes) {
            diag.short_reads++;
            ESP_LOGW(TAG, "PDM short read: bytes=%u expected=%u",
                     (unsigned)bytes_read, (unsigned)expected_bytes);
            continue;
        }

        int32_t peak_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
        int64_t sumsq_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
        int64_t sum_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
        for (int i = 0; i < AUDIO_FRAME_LEN; i++) {
            frame.pcm_ch0[i] = interleaved[pdm_channels * i + downstream_ch0];
            frame.pcm_ch1[i] = interleaved[pdm_channels * i + downstream_ch1];

            for (int ch = 0; ch < pdm_channels; ch++) {
                int16_t sample = interleaved[pdm_channels * i + ch];
                int32_t abs_sample = (sample < 0) ? -(int32_t)sample : (int32_t)sample;
                if (abs_sample > peak_ch[ch]) peak_ch[ch] = abs_sample;
                if (sample < diag.min_ch[ch]) diag.min_ch[ch] = sample;
                if (sample > diag.max_ch[ch]) diag.max_ch[ch] = sample;
                sum_ch[ch] += sample;
                sumsq_ch[ch] += (int64_t)sample * sample;
            }
        }
        frame.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        diag.frames++;
        diag.samples_per_ch += AUDIO_FRAME_LEN;
        bool frame_all_zero = true;
        for (int ch = 0; ch < pdm_channels; ch++) {
            if (peak_ch[ch] > diag.peak_ch[ch]) diag.peak_ch[ch] = peak_ch[ch];
            diag.sum_ch[ch] += sum_ch[ch];
            diag.sumsq_ch[ch] += sumsq_ch[ch];
            if (peak_ch[ch] != 0) frame_all_zero = false;
        }
        if (frame_all_zero) diag.zero_frames++;

        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - diag.start_tick) >= pdMS_TO_TICKS(AUDIO_DIAG_LOG_INTERVAL_MS)) {
            uint32_t window_ms = (now_tick - diag.start_tick) * portTICK_PERIOD_MS;
            int64_t mean_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
            int64_t raw_rms2_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
            int64_t ac_rms2_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0};
            float rms_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0.0f};
            float dbfs_ch[AUDIO_PDM_CAPTURE_CHANNELS] = {0.0f};
            for (int ch = 0; ch < pdm_channels; ch++) {
                mean_ch[ch] = diag.samples_per_ch ? (diag.sum_ch[ch] / diag.samples_per_ch) : 0;
                raw_rms2_ch[ch] = diag.samples_per_ch ? (diag.sumsq_ch[ch] / diag.samples_per_ch) : 0;
                ac_rms2_ch[ch] = raw_rms2_ch[ch] - mean_ch[ch] * mean_ch[ch];
                if (ac_rms2_ch[ch] < 0) ac_rms2_ch[ch] = 0;
                rms_ch[ch] = sqrtf((float)ac_rms2_ch[ch]);
                dbfs_ch[ch] = (rms_ch[ch] > 0.0f) ? (20.0f * log10f(rms_ch[ch] / 32768.0f)) : -120.0f;
            }
            ESP_LOGI(TAG,
                     "MIC diag ch=%d downstream=%d/%d probe=%d window_ms=%u frames=%u zero_frames=%u samples=%u "
                     "ch0_min=%d ch0_max=%d ch0_mean=%lld ch0_peak=%ld ch0_rms=%.1f ch0_dbfs=%.1f "
                     "ch1_min=%d ch1_max=%d ch1_mean=%lld ch1_peak=%ld ch1_rms=%.1f ch1_dbfs=%.1f "
                     "read_err=%u short=%u",
                     pdm_channels,
                     downstream_ch0,
                     downstream_ch1,
                     dual_slot_probe,
                     (unsigned)window_ms,
                     (unsigned)diag.frames,
                     (unsigned)diag.zero_frames,
                     (unsigned)diag.samples_per_ch,
                     (int)diag.min_ch[0],
                     (int)diag.max_ch[0],
                     (long long)mean_ch[0],
                     (long)diag.peak_ch[0],
                     rms_ch[0],
                     dbfs_ch[0],
                     (int)diag.min_ch[1],
                     (int)diag.max_ch[1],
                     (long long)mean_ch[1],
                     (long)diag.peak_ch[1],
                     rms_ch[1],
                     dbfs_ch[1],
                     (unsigned)diag.read_errors,
                     (unsigned)diag.short_reads);
            if (diag.frames > 0 && diag.zero_frames == diag.frames) {
                ESP_LOGW(TAG,
                         "MIC input is all zero in this window; check PDM slot, DIN GPIO, CLK, mic power, and mic SEL wiring.");
            }
            audio_diag_reset(&diag, now_tick);
        }
#endif

        if (xQueueSend(q_audio, &frame, 0) != pdTRUE) {
            // 队列满，丢弃当前帧（允许，DSP 任务处理不过来时正常现象）
        }

        static uint32_t s_p0_cnt = 0;
        if (++s_p0_cnt % 300 == 0) {
            ESP_LOGI(TAG, "[P0] stack_hwm=%u bytes", uxTaskGetStackHighWaterMark(NULL) * 4);
        }
    }
}
