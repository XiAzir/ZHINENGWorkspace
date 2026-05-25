#include "drv_pdm_dma.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"

static const char *TAG = "drv_pdm_dma";

// 硬件引脚（与 EV-Board 原理图对应；业务层不关心）
#define PDM_CLK_GPIO    4
#define PDM_DIN0_GPIO   5
#define PDM_DIN1_GPIO   6

#if AUDIO_PDM_ACTIVE_LINE == 0
#define PDM_ACTIVE_DIN_GPIO   PDM_DIN0_GPIO
#define PDM_ACTIVE_LINE_NAME  "GPIO5/DATA0"
#elif AUDIO_PDM_ACTIVE_LINE == 1
#define PDM_ACTIVE_DIN_GPIO   PDM_DIN1_GPIO
#define PDM_ACTIVE_LINE_NAME  "GPIO6/DATA1"
#else
#error "AUDIO_PDM_ACTIVE_LINE must be 0 or 1"
#endif

#define PDM_ACTIVE_SLOT_MASK  (I2S_PDM_RX_LINE0_SLOT_LEFT | I2S_PDM_RX_LINE0_SLOT_RIGHT)

static i2s_chan_handle_t s_rx_chan = NULL;
static int s_active_channels = AUDIO_PDM_CAPTURE_CHANNELS;
static int s_downstream_ch0 = AUDIO_PDM_DOWNSTREAM_CH0;
static int s_downstream_ch1 = AUDIO_PDM_DOWNSTREAM_CH1;

_Static_assert(AUDIO_PDM_PHYSICAL_LINES == 2, "current board wiring expects GPIO5/GPIO6 as two PDM RX lines");
_Static_assert(AUDIO_PDM_SLOTS_PER_LINE == 2, "PDM line uses LOW/HIGH SELECT slots");
_Static_assert(AUDIO_PDM_CAPTURE_CHANNELS == 2, "ESP-IDF PDM RX DMA exposes one stereo line at a time here");
_Static_assert(AUDIO_PDM_DOWNSTREAM_CH0 >= 0 && AUDIO_PDM_DOWNSTREAM_CH0 < AUDIO_PDM_CAPTURE_CHANNELS,
               "downstream ch0 is outside captured PDM channel range");
_Static_assert(AUDIO_PDM_DOWNSTREAM_CH1 >= 0 && AUDIO_PDM_DOWNSTREAM_CH1 < AUDIO_PDM_CAPTURE_CHANNELS,
               "downstream ch1 is outside captured PDM channel range");

esp_err_t drv_pdm_dma_init(int sample_rate, int frame_len, int num_dma_buf) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = num_dma_buf;
    chan_cfg.dma_frame_num = frame_len;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan));

    i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate);
    i2s_pdm_rx_slot_config_t slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT,
        I2S_SLOT_MODE_STEREO);
#if AUDIO_PDM_DIN0_DUAL_SLOT_PROBE
    slot_cfg.slot_mask = I2S_PDM_RX_LINE0_SLOT_LEFT | I2S_PDM_RX_LINE0_SLOT_RIGHT;
#else
    slot_cfg.slot_mask = PDM_ACTIVE_SLOT_MASK;
#endif
#if SOC_I2S_SUPPORTS_PDM_RX_HP_FILTER
    slot_cfg.amplify_num = AUDIO_PDM_AMPLIFY_NUM;
#endif

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .clk  = PDM_CLK_GPIO,
            .dins = {
                [0] = AUDIO_PDM_DIN0_DUAL_SLOT_PROBE ? PDM_DIN0_GPIO : PDM_ACTIVE_DIN_GPIO,
                [1] = I2S_GPIO_UNUSED,
                [2] = I2S_GPIO_UNUSED,
                [3] = I2S_GPIO_UNUSED,
            },
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));

#if AUDIO_PDM_DIN0_DUAL_SLOT_PROBE
    s_active_channels = 2;
    s_downstream_ch0 = 0;
    s_downstream_ch1 = 1;
#else
    s_active_channels = AUDIO_PDM_CAPTURE_CHANNELS;
    s_downstream_ch0 = AUDIO_PDM_DOWNSTREAM_CH0;
    s_downstream_ch1 = AUDIO_PDM_DOWNSTREAM_CH1;
#endif

    ESP_LOGI(TAG,
             "PDM DMA init OK (sr=%d frame=%d dma=%d ch=%d downstream=%d/%d active_line=%s mask=0x%x amp=%d GPIO CLK=%d DIN0=%d DIN1=%d dual_slot_probe=%d)",
             sample_rate, frame_len, num_dma_buf,
             s_active_channels,
             s_downstream_ch0,
             s_downstream_ch1,
             AUDIO_PDM_DIN0_DUAL_SLOT_PROBE ? "GPIO5/DATA0 probe" : PDM_ACTIVE_LINE_NAME,
             (unsigned)slot_cfg.slot_mask,
             AUDIO_PDM_AMPLIFY_NUM,
             PDM_CLK_GPIO, PDM_DIN0_GPIO, PDM_DIN1_GPIO,
             AUDIO_PDM_DIN0_DUAL_SLOT_PROBE);
    ESP_LOGI(TAG,
             "PDM map: ch0=active DATA SELECT_LOW/LEFT, ch1=active DATA SELECT_HIGH/RIGHT; set AUDIO_PDM_ACTIVE_LINE to switch GPIO5/GPIO6 pair");
    return ESP_OK;
}

int drv_pdm_dma_get_channels(void) {
    return s_active_channels;
}

int drv_pdm_dma_get_downstream_ch0(void) {
    return s_downstream_ch0;
}

int drv_pdm_dma_get_downstream_ch1(void) {
    return s_downstream_ch1;
}

bool drv_pdm_dma_is_dual_slot_probe(void) {
    return AUDIO_PDM_DIN0_DUAL_SLOT_PROBE != 0;
}

esp_err_t drv_pdm_dma_read(int16_t *buf, size_t buf_len,
                             size_t *bytes_read, TickType_t timeout) {
    return i2s_channel_read(s_rx_chan, buf,
                            buf_len * sizeof(int16_t), bytes_read, timeout);
}

void drv_pdm_dma_deinit(void) {
    if (s_rx_chan) {
        i2s_channel_disable(s_rx_chan);
        i2s_del_channel(s_rx_chan);
        s_rx_chan = NULL;
    }
}
