#include "../../main/app_config.h"
#ifdef USE_SIMULATION

#include "drv_sim.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "drv_sim";

// 从自身组件（drivers）捆绑的内聚资源池中获取真实的 WAV 信号数据流以满足精确声学预测
extern const uint8_t car_horn_wav_start[]   asm("_binary_test_audio_car_horn_wav_start");
extern const uint8_t car_horn_wav_end[]     asm("_binary_test_audio_car_horn_wav_end");
extern const uint8_t siren_wav_start[]      asm("_binary_test_audio_siren_wav_start");
extern const uint8_t siren_wav_end[]        asm("_binary_test_audio_siren_wav_end");
extern const uint8_t background_wav_start[] asm("_binary_test_audio_background_wav_start");
extern const uint8_t background_wav_end[]   asm("_binary_test_audio_background_wav_end");

// 解析 RIFF chunk，返回 PCM 数据起始偏移；失败返回 0。
// 规范：文件头 12 字节（"RIFF" + size + "WAVE"），之后是 chunk 序列（id + size + payload）。
static size_t find_wav_data_offset(const uint8_t *start, const uint8_t *end) {
    size_t filesize = (size_t)(end - start);
    if (filesize < 12) return 0;
    if (memcmp(start,     "RIFF", 4) != 0) return 0;
    if (memcmp(start + 8, "WAVE", 4) != 0) return 0;

    size_t pos = 12;
    while (pos + 8 <= filesize) {
        const uint8_t *chunk = start + pos;
        uint32_t chunk_size = (uint32_t)chunk[4]        |
                              ((uint32_t)chunk[5] <<  8) |
                              ((uint32_t)chunk[6] << 16) |
                              ((uint32_t)chunk[7] << 24);
        if (memcmp(chunk, "data", 4) == 0) {
            return pos + 8;  // 跳过 "data" + size
        }
        pos += 8 + chunk_size;
        if (chunk_size & 1) pos++;  // RIFF 对齐到偶数字节
    }
    return 0;
}

typedef struct {
    const uint8_t *data_start;
    const uint8_t *data_end;
    const uint8_t *cursor;
} SimSource_t;

static SimSource_t s_sources[3];
static SimSource_t *s_active = NULL;
static int s_active_idx = 0;

static void init_source(SimSource_t *src, const uint8_t *wav_start, const uint8_t *wav_end,
                        const char *name) {
    size_t data_off = find_wav_data_offset(wav_start, wav_end);
    if (data_off == 0) {
        ESP_LOGW(TAG, "[%s] WAV header parse failed, falling back to offset 44", name);
        data_off = 44;
    }
    src->data_start = wav_start + data_off;
    src->data_end   = wav_end;
    src->cursor     = src->data_start;
}

void drv_sim_init(void) {
    init_source(&s_sources[0], car_horn_wav_start,   car_horn_wav_end,   "car_horn");
    init_source(&s_sources[1], siren_wav_start,      siren_wav_end,      "siren");
    init_source(&s_sources[2], background_wav_start, background_wav_end, "background");

    s_active = &s_sources[s_active_idx];
    ESP_LOGI(TAG, "Sim init, source=%d (0=car_horn 1=siren 2=background)", s_active_idx);
}

void drv_sim_select_source(int idx) {
    if (idx < 0 || idx > 2) return;
    s_active_idx = idx;
    s_active = &s_sources[idx];
    s_active->cursor = s_active->data_start;
    ESP_LOGI(TAG, "Sim source → %d", idx);
}

void drv_sim_read_frame(int16_t *ch0, int16_t *ch1, int len) {
    if (!s_active) return;

    const uint8_t *p = s_active->cursor;
    for (int i = 0; i < len; i++) {
        if (p + 1 >= s_active->data_end) {
            // 当前曲目到头时，自动平滑切换到下一首测试音轨
            drv_sim_select_source((s_active_idx + 1) % 3);
            p = s_active->data_start;
        }
        int16_t sample = (int16_t)(p[0] | (p[1] << 8));  // little-endian PCM
        ch0[i] = sample;
        ch1[i] = sample;  // 两路相同（单麦克风仿真）
        p += 2;
    }
    s_active->cursor = p;
}

#endif // USE_SIMULATION
