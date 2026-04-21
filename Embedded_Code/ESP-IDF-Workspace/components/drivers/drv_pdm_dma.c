#include "drv_pdm_dma.h"
#include "driver/i2s_pdm.h"
#include "esp_log.h"

static const char *TAG = "drv_pdm_dma";

// 硬件引脚（与 EV-Board 原理图对应；业务层不关心）
#define PDM_CLK_GPIO    4
#define PDM_DIN0_GPIO   5
#define PDM_DIN1_GPIO   6

static i2s_chan_handle_t s_rx_chan = NULL;

esp_err_t drv_pdm_dma_init(int sample_rate, int frame_len, int num_dma_buf) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = num_dma_buf;
    chan_cfg.dma_frame_num = frame_len;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan));

    i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate);
    i2s_pdm_rx_slot_config_t slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .clk  = PDM_CLK_GPIO,
            .din  = PDM_DIN0_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));

    ESP_LOGI(TAG, "PDM DMA init OK (sr=%d frame=%d dma=%d, GPIO CLK=%d DIN0=%d DIN1=%d)",
             sample_rate, frame_len, num_dma_buf,
             PDM_CLK_GPIO, PDM_DIN0_GPIO, PDM_DIN1_GPIO);
    return ESP_OK;
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
