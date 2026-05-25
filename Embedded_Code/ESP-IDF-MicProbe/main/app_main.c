#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2s_pdm.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mic_probe";

#define MIC_SAMPLE_RATE       16000
#define MIC_FRAME_LEN         512
#define MIC_DMA_DESC_NUM      4
#define MIC_FAST_LOG_INTERVAL_MS 100
#define MIC_LOG_INTERVAL_MS   1000
#define MIC_WARMUP_MS         2000
#define MIC_BASELINE_WINDOWS  3
#define MIC_PROBE_TASK_STACK  12288
#define MIC_PCM_AMPLIFY_NUM   1
#define MIC_FAST_MAX_FRAMES   8
#define MIC_FAST_MAX_SAMPLES  (MIC_FRAME_LEN * MIC_FAST_MAX_FRAMES)
#define MIC_PI                3.14159265358979323846f
#define MIC_SQRT2             1.41421356237309504880f
#define MIC_DBFS_FLOOR        (-120.0f)
#define MIC_BAND_MAX_PROBES   8

#define MIC_CLK_GPIO          4
#define MIC_DIN0_GPIO         5

typedef enum {
    BAND_LOW = 0,
    BAND_HUM,
    BAND_VOICE,
    BAND_ALERT,
    BAND_COUNT,
} BandIndex;

typedef struct {
    const char *name;
    uint8_t probe_count;
    float freq[MIC_BAND_MAX_PROBES];
} BandProbe;

typedef struct {
    int16_t min;
    int16_t max;
    int32_t peak;
    int64_t sum;
    int64_t sumsq;
    uint32_t samples;
    uint32_t zeros;
    uint32_t clips;
} SlotStats;

typedef struct {
    TickType_t start_tick;
    TickType_t fast_start_tick;
    uint32_t frames;
    uint32_t fast_frames;
    uint32_t read_err;
    uint32_t short_read;
    SlotStats slot[2];
    SlotStats fast_slot[2];
    float fast_peak_dbfs[2];
    int16_t fast_pcm[MIC_FAST_MAX_SAMPLES];
    uint32_t fast_pcm_samples;
    uint32_t fast_pcm_overflow;
    uint32_t band_windows;
    float band_sum_dbfs[BAND_COUNT];
    float band_peak_dbfs[BAND_COUNT];
} WindowStats;

static i2s_chan_handle_t s_rx_chan;
static TickType_t s_probe_start_tick;
static int s_baseline_count;
static bool s_baseline_ready;
static float s_baseline_sum[2];
static float s_baseline[2];
static float s_band_baseline_sum[BAND_COUNT];
static float s_band_baseline[BAND_COUNT];

static const BandProbe s_band_probe[BAND_COUNT] = {
    [BAND_LOW] = {
        .name = "low",
        .probe_count = 5,
        .freq = {40.0f, 70.0f, 100.0f, 150.0f, 200.0f},
    },
    [BAND_HUM] = {
        .name = "hum",
        .probe_count = 6,
        .freq = {50.0f, 60.0f, 100.0f, 120.0f, 150.0f, 180.0f},
    },
    [BAND_VOICE] = {
        .name = "voice",
        .probe_count = 8,
        .freq = {300.0f, 500.0f, 700.0f, 1000.0f, 1500.0f, 2200.0f, 2800.0f, 3400.0f},
    },
    [BAND_ALERT] = {
        .name = "alert",
        .probe_count = 8,
        .freq = {700.0f, 900.0f, 1200.0f, 1500.0f, 2000.0f, 2600.0f, 3300.0f, 4200.0f},
    },
};

static void slot_reset(SlotStats *s)
{
    memset(s, 0, sizeof(*s));
    s->min = INT16_MAX;
    s->max = INT16_MIN;
}

static void window_reset(WindowStats *w)
{
    memset(w, 0, sizeof(*w));
    w->start_tick = xTaskGetTickCount();
    w->fast_start_tick = w->start_tick;
    slot_reset(&w->slot[0]);
    slot_reset(&w->slot[1]);
    slot_reset(&w->fast_slot[0]);
    slot_reset(&w->fast_slot[1]);
    w->fast_peak_dbfs[0] = -120.0f;
    w->fast_peak_dbfs[1] = -120.0f;
    for (int band = 0; band < BAND_COUNT; band++) {
        w->band_peak_dbfs[band] = MIC_DBFS_FLOOR;
    }
}

static void fast_window_reset(WindowStats *w)
{
    w->fast_start_tick = xTaskGetTickCount();
    w->fast_frames = 0;
    w->fast_pcm_samples = 0;
    w->fast_pcm_overflow = 0;
    slot_reset(&w->fast_slot[0]);
    slot_reset(&w->fast_slot[1]);
}

static void slot_push(SlotStats *s, int16_t v)
{
    int32_t av = (v < 0) ? -(int32_t)v : (int32_t)v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    if (av > s->peak) s->peak = av;
    if (v == 0) s->zeros++;
    if (av >= 32000) s->clips++;
    s->sum += v;
    s->sumsq += (int64_t)v * (int64_t)v;
    s->samples++;
}

static float slot_dbfs(const SlotStats *s, float *rms_out, int64_t *ac_rms2_out)
{
    if (s->samples == 0) {
        if (rms_out) *rms_out = 0.0f;
        if (ac_rms2_out) *ac_rms2_out = 0;
        return MIC_DBFS_FLOOR;
    }

    int64_t mean = s->sum / (int64_t)s->samples;
    int64_t raw_rms2 = s->sumsq / (int64_t)s->samples;
    int64_t ac_rms2 = raw_rms2 - mean * mean;
    if (ac_rms2 < 0) ac_rms2 = 0;

    float rms = sqrtf((float)ac_rms2);
    if (rms_out) *rms_out = rms;
    if (ac_rms2_out) *ac_rms2_out = ac_rms2;
    return (rms > 0.0f) ? (20.0f * log10f(rms / 32768.0f)) : MIC_DBFS_FLOOR;
}

static float rms_to_dbfs(float rms)
{
    return (rms > 0.0f) ? (20.0f * log10f(rms / 32768.0f)) : MIC_DBFS_FLOOR;
}

static float goertzel_rms2(const int16_t *samples, uint32_t n, float mean, float freq_hz)
{
    if (n < 16 || freq_hz <= 0.0f || freq_hz >= ((float)MIC_SAMPLE_RATE * 0.5f)) {
        return 0.0f;
    }

    const float omega = 2.0f * MIC_PI * freq_hz / (float)MIC_SAMPLE_RATE;
    const float coeff = 2.0f * cosf(omega);
    float q1 = 0.0f;
    float q2 = 0.0f;

    for (uint32_t i = 0; i < n; i++) {
        const float q0 = ((float)samples[i] - mean) + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
    }

    float power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
    if (power < 0.0f) {
        power = 0.0f;
    }

    const float rms = sqrtf(power) * MIC_SQRT2 / (float)n;
    return rms * rms;
}

static void analyze_fast_bands(const WindowStats *w, float out_dbfs[BAND_COUNT])
{
    const uint32_t n = w->fast_pcm_samples;
    if (n == 0) {
        for (int band = 0; band < BAND_COUNT; band++) {
            out_dbfs[band] = MIC_DBFS_FLOOR;
        }
        return;
    }

    int64_t sum = 0;
    for (uint32_t i = 0; i < n; i++) {
        sum += w->fast_pcm[i];
    }
    const float mean = (float)sum / (float)n;

    for (int band = 0; band < BAND_COUNT; band++) {
        const BandProbe *probe = &s_band_probe[band];
        float max_rms2 = 0.0f;
        for (uint8_t i = 0; i < probe->probe_count; i++) {
            const float rms2 = goertzel_rms2(w->fast_pcm, n, mean, probe->freq[i]);
            if (rms2 > max_rms2) {
                max_rms2 = rms2;
            }
        }
        out_dbfs[band] = rms_to_dbfs(sqrtf(max_rms2));
    }
}

static void fold_fast_bands(WindowStats *w, float out_dbfs[BAND_COUNT])
{
    analyze_fast_bands(w, out_dbfs);
    if (w->fast_pcm_samples == 0) {
        return;
    }

    w->band_windows++;
    for (int band = 0; band < BAND_COUNT; band++) {
        w->band_sum_dbfs[band] += out_dbfs[band];
        if (out_dbfs[band] > w->band_peak_dbfs[band]) {
            w->band_peak_dbfs[band] = out_dbfs[band];
        }
    }
}

static void band_average(const WindowStats *w, float out_dbfs[BAND_COUNT])
{
    for (int band = 0; band < BAND_COUNT; band++) {
        out_dbfs[band] = w->band_windows
                              ? (w->band_sum_dbfs[band] / (float)w->band_windows)
                              : MIC_DBFS_FLOOR;
    }
}

static esp_err_t mic_pdm_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = MIC_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = MIC_FRAME_LEN;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "new channel failed");

    i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE);
    i2s_pdm_rx_slot_config_t slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT,
        I2S_SLOT_MODE_STEREO);
    slot_cfg.slot_mask = I2S_PDM_RX_LINE0_SLOT_LEFT | I2S_PDM_RX_LINE0_SLOT_RIGHT;
#if SOC_I2S_SUPPORTS_PDM_RX_HP_FILTER
    slot_cfg.amplify_num = MIC_PCM_AMPLIFY_NUM;
#endif

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .clk = MIC_CLK_GPIO,
            .dins = {
                [0] = MIC_DIN0_GPIO,
                [1] = I2S_GPIO_UNUSED,
                [2] = I2S_GPIO_UNUSED,
                [3] = I2S_GPIO_UNUSED,
            },
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg), TAG, "pdm rx init failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "pdm rx enable failed");

    ESP_LOGI(TAG,
             "PDM probe init: sr=%d frame=%d dma=%d CLK=%d DIN0=%d mask=0x%x slot0=LEFT slot1=RIGHT amplify=%u",
             MIC_SAMPLE_RATE,
             MIC_FRAME_LEN,
             MIC_DMA_DESC_NUM,
             MIC_CLK_GPIO,
             MIC_DIN0_GPIO,
             (unsigned)slot_cfg.slot_mask,
             (unsigned)MIC_PCM_AMPLIFY_NUM);
    ESP_LOGI(TAG,
             "Keep quiet for the first %d seconds to lock baseline, then play short loud audio close to the mic.",
             (MIC_WARMUP_MS / 1000) + MIC_BASELINE_WINDOWS);
    ESP_LOGI(TAG,
             "Watch MIC_FAST bands: low=20-200Hz, hum=50/60Hz harmonics, voice=300-3400Hz, alert=700-4200Hz.");
    return ESP_OK;
}

static void update_baseline(const float dbfs[2], const float band_dbfs[BAND_COUNT], uint32_t clips)
{
    if (s_baseline_ready) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if ((now - s_probe_start_tick) < pdMS_TO_TICKS(MIC_WARMUP_MS)) {
        return;
    }

    if (clips > 0) {
        ESP_LOGW(TAG, "Baseline skip: clip=%" PRIu32 ", keep quiet while baseline is locking", clips);
        return;
    }

    s_baseline_sum[0] += dbfs[0];
    s_baseline_sum[1] += dbfs[1];
    for (int band = 0; band < BAND_COUNT; band++) {
        s_band_baseline_sum[band] += band_dbfs[band];
    }

    s_baseline_count++;
    if (s_baseline_count >= MIC_BASELINE_WINDOWS) {
        s_baseline[0] = s_baseline_sum[0] / (float)s_baseline_count;
        s_baseline[1] = s_baseline_sum[1] / (float)s_baseline_count;
        for (int band = 0; band < BAND_COUNT; band++) {
            s_band_baseline[band] = s_band_baseline_sum[band] / (float)s_baseline_count;
        }
        s_baseline_ready = true;
        ESP_LOGI(TAG,
                 "Baseline locked: slot0=%.1f dBFS slot1=%.1f dBFS"
                 " | low=%.1f hum=%.1f voice=%.1f alert=%.1f",
                 s_baseline[0],
                 s_baseline[1],
                 s_band_baseline[BAND_LOW],
                 s_band_baseline[BAND_HUM],
                 s_band_baseline[BAND_VOICE],
                 s_band_baseline[BAND_ALERT]);
    }
}

static void log_fast_window(WindowStats *w)
{
    float rms[2];
    int64_t ac_rms2[2];
    float dbfs[2];
    int64_t mean[2];
    float zero_pct[2];
    float band_dbfs[BAND_COUNT];

    for (int ch = 0; ch < 2; ch++) {
        const SlotStats *s = &w->fast_slot[ch];
        mean[ch] = s->samples ? (s->sum / (int64_t)s->samples) : 0;
        zero_pct[ch] = s->samples ? (100.0f * (float)s->zeros / (float)s->samples) : 100.0f;
        dbfs[ch] = slot_dbfs(s, &rms[ch], &ac_rms2[ch]);
        if (dbfs[ch] > w->fast_peak_dbfs[ch]) {
            w->fast_peak_dbfs[ch] = dbfs[ch];
        }
    }
    fold_fast_bands(w, band_dbfs);

    float delta0 = s_baseline_ready ? (dbfs[0] - s_baseline[0]) : 0.0f;
    float delta1 = s_baseline_ready ? (dbfs[1] - s_baseline[1]) : 0.0f;
    float band_delta[BAND_COUNT];
    for (int band = 0; band < BAND_COUNT; band++) {
        band_delta[band] = s_baseline_ready ? (band_dbfs[band] - s_band_baseline[band]) : 0.0f;
    }
    uint32_t fast_ms = (xTaskGetTickCount() - w->fast_start_tick) * portTICK_PERIOD_MS;

    ESP_LOGI(TAG,
             "MIC_FAST ms=%" PRIu32 " frames=%" PRIu32
             " | slot0 min=%d max=%d mean=%" PRId64 " peak=%" PRId32
             " rms=%.1f dbfs=%.1f delta=%+.1f zero=%.1f%% clip=%" PRIu32
             " | slot1 min=%d max=%d mean=%" PRId64 " peak=%" PRId32
             " rms=%.1f dbfs=%.1f delta=%+.1f zero=%.1f%% clip=%" PRIu32
             " | bands low=%.1f/%+.1f hum=%.1f/%+.1f voice=%.1f/%+.1f alert=%.1f/%+.1f ovf=%" PRIu32,
             fast_ms,
             w->fast_frames,
             w->fast_slot[0].min,
             w->fast_slot[0].max,
             mean[0],
             w->fast_slot[0].peak,
             rms[0],
             dbfs[0],
             delta0,
             zero_pct[0],
             w->fast_slot[0].clips,
             w->fast_slot[1].min,
             w->fast_slot[1].max,
             mean[1],
             w->fast_slot[1].peak,
             rms[1],
             dbfs[1],
             delta1,
             zero_pct[1],
             w->fast_slot[1].clips,
             band_dbfs[BAND_LOW],
             band_delta[BAND_LOW],
             band_dbfs[BAND_HUM],
             band_delta[BAND_HUM],
             band_dbfs[BAND_VOICE],
             band_delta[BAND_VOICE],
             band_dbfs[BAND_ALERT],
             band_delta[BAND_ALERT],
             w->fast_pcm_overflow);

    fast_window_reset(w);
}

static void log_window(WindowStats *w)
{
    float rms[2];
    int64_t ac_rms2[2];
    float dbfs[2];
    int64_t mean[2];
    float zero_pct[2];
    float tail_band_dbfs[BAND_COUNT];
    float band_avg_dbfs[BAND_COUNT];

    for (int ch = 0; ch < 2; ch++) {
        const SlotStats *s = &w->slot[ch];
        mean[ch] = s->samples ? (s->sum / (int64_t)s->samples) : 0;
        zero_pct[ch] = s->samples ? (100.0f * (float)s->zeros / (float)s->samples) : 100.0f;
        dbfs[ch] = slot_dbfs(s, &rms[ch], &ac_rms2[ch]);
    }

    fold_fast_bands(w, tail_band_dbfs);
    band_average(w, band_avg_dbfs);
    update_baseline(dbfs, band_avg_dbfs, w->slot[0].clips + w->slot[1].clips);

    float fast_peak_dbfs[2] = {w->fast_peak_dbfs[0], w->fast_peak_dbfs[1]};
    for (int ch = 0; ch < 2; ch++) {
        float fast_tail_dbfs = slot_dbfs(&w->fast_slot[ch], NULL, NULL);
        if (fast_tail_dbfs > fast_peak_dbfs[ch]) {
            fast_peak_dbfs[ch] = fast_tail_dbfs;
        }
    }

    float delta0 = s_baseline_ready ? (dbfs[0] - s_baseline[0]) : 0.0f;
    float delta1 = s_baseline_ready ? (dbfs[1] - s_baseline[1]) : 0.0f;
    uint32_t window_ms = (xTaskGetTickCount() - w->start_tick) * portTICK_PERIOD_MS;
    float fast_peak_delta0 = s_baseline_ready ? (fast_peak_dbfs[0] - s_baseline[0]) : 0.0f;
    float fast_peak_delta1 = s_baseline_ready ? (fast_peak_dbfs[1] - s_baseline[1]) : 0.0f;
    float band_avg_delta[BAND_COUNT];
    float band_peak_delta[BAND_COUNT];
    for (int band = 0; band < BAND_COUNT; band++) {
        band_avg_delta[band] = s_baseline_ready ? (band_avg_dbfs[band] - s_band_baseline[band]) : 0.0f;
        band_peak_delta[band] = s_baseline_ready ? (w->band_peak_dbfs[band] - s_band_baseline[band]) : 0.0f;
    }

    ESP_LOGI(TAG,
             "MIC_PROBE ms=%" PRIu32 " frames=%" PRIu32 " samples=%" PRIu32
             " | slot0 min=%d max=%d mean=%" PRId64 " peak=%" PRId32
             " rms=%.1f dbfs=%.1f delta=%+.1f zero=%.1f%% clip=%" PRIu32
             " | slot1 min=%d max=%d mean=%" PRId64 " peak=%" PRId32
             " rms=%.1f dbfs=%.1f delta=%+.1f zero=%.1f%% clip=%" PRIu32
             " | fast_peak0=%.1f fast_peak0_delta=%+.1f fast_peak1=%.1f fast_peak1_delta=%+.1f"
             " | band_avg low=%.1f/%+.1f hum=%.1f/%+.1f voice=%.1f/%+.1f alert=%.1f/%+.1f"
             " | band_peak low=%.1f/%+.1f hum=%.1f/%+.1f voice=%.1f/%+.1f alert=%.1f/%+.1f"
             " | read_err=%" PRIu32 " short=%" PRIu32,
             window_ms,
             w->frames,
             w->slot[0].samples,
             w->slot[0].min,
             w->slot[0].max,
             mean[0],
             w->slot[0].peak,
             rms[0],
             dbfs[0],
             delta0,
             zero_pct[0],
             w->slot[0].clips,
             w->slot[1].min,
             w->slot[1].max,
             mean[1],
             w->slot[1].peak,
             rms[1],
             dbfs[1],
             delta1,
             zero_pct[1],
             w->slot[1].clips,
             fast_peak_dbfs[0],
             fast_peak_delta0,
             fast_peak_dbfs[1],
             fast_peak_delta1,
             band_avg_dbfs[BAND_LOW],
             band_avg_delta[BAND_LOW],
             band_avg_dbfs[BAND_HUM],
             band_avg_delta[BAND_HUM],
             band_avg_dbfs[BAND_VOICE],
             band_avg_delta[BAND_VOICE],
             band_avg_dbfs[BAND_ALERT],
             band_avg_delta[BAND_ALERT],
             w->band_peak_dbfs[BAND_LOW],
             band_peak_delta[BAND_LOW],
             w->band_peak_dbfs[BAND_HUM],
             band_peak_delta[BAND_HUM],
             w->band_peak_dbfs[BAND_VOICE],
             band_peak_delta[BAND_VOICE],
             w->band_peak_dbfs[BAND_ALERT],
             band_peak_delta[BAND_ALERT],
             w->read_err,
             w->short_read);
}

static void mic_probe_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(mic_pdm_init());

    static WindowStats w;
    static int16_t samples[MIC_FRAME_LEN * 2];
    window_reset(&w);
    s_probe_start_tick = w.start_tick;

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(s_rx_chan, samples, sizeof(samples), &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
            w.read_err++;
            ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(err));
            continue;
        }
        if (bytes_read < sizeof(samples)) {
            w.short_read++;
            ESP_LOGW(TAG, "short read: bytes=%u expected=%u",
                     (unsigned)bytes_read, (unsigned)sizeof(samples));
            continue;
        }

        for (int i = 0; i < MIC_FRAME_LEN; i++) {
            slot_push(&w.slot[0], samples[2 * i]);
            slot_push(&w.slot[1], samples[2 * i + 1]);
            slot_push(&w.fast_slot[0], samples[2 * i]);
            slot_push(&w.fast_slot[1], samples[2 * i + 1]);
            if (w.fast_pcm_samples < MIC_FAST_MAX_SAMPLES) {
                w.fast_pcm[w.fast_pcm_samples++] = samples[2 * i];
            } else {
                w.fast_pcm_overflow++;
            }
        }
        w.frames++;
        w.fast_frames++;

        if ((xTaskGetTickCount() - w.fast_start_tick) >= pdMS_TO_TICKS(MIC_FAST_LOG_INTERVAL_MS)) {
            log_fast_window(&w);
        }

        if ((xTaskGetTickCount() - w.start_tick) >= pdMS_TO_TICKS(MIC_LOG_INTERVAL_MS)) {
            log_window(&w);
            window_reset(&w);
        }
    }
}

void app_main(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(
        mic_probe_task,
        "mic_probe",
        MIC_PROBE_TASK_STACK,
        NULL,
        5,
        NULL,
        0);
    configASSERT(ok == pdPASS);
}
