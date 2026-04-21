#include "drv_oled.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

static const char *TAG = "drv_oled";

#define OLED_SPI_HOST   SPI2_HOST
#define OLED_PIN_CLK    10
#define OLED_PIN_MOSI   11
#define OLED_PIN_CS     12
#define OLED_PIN_DC     13
#define OLED_PIN_RST    14

static spi_device_handle_t s_oled_spi = NULL;

// 在事务真正开始前切换 DC 电平，避免 queue_size>1 时 GPIO 与数据错位
static void IRAM_ATTR oled_spi_pre_cb(spi_transaction_t *t) {
    int dc = (int)(intptr_t)t->user;
    gpio_set_level(OLED_PIN_DC, dc);
}

esp_err_t drv_oled_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OLED_PIN_DC) | (1ULL << OLED_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(OLED_PIN_RST, 1);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = OLED_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(OLED_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = OLED_PIN_CS,
        .queue_size = 7,
        .pre_cb = oled_spi_pre_cb,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(OLED_SPI_HOST, &dev_cfg, &s_oled_spi));

    ESP_LOGI(TAG, "OLED SPI init OK");
    // TODO M2：发送 OLED 初始化命令序列（SSD1306 或 SH1106）
    return ESP_OK;
}

void drv_oled_spi_write(const uint8_t *buf, size_t len, bool is_data) {
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = buf,
        .user      = (void *)(intptr_t)(is_data ? 1 : 0),
    };
    spi_device_transmit(s_oled_spi, &t);
}

void drv_oled_clear(void) {
    // TODO M2：清空帧缓冲并刷新
}

// M1 占位：真实像素渲染在 M2 补。目前只走 log，方便仿真联调。
void drv_oled_draw_status(int level, int pred_class, float confidence, float doa_rel_deg) {
    ESP_LOGD(TAG, "[STATUS] lvl=%d cls=%d conf=%.2f rel=%.1f",
             level, pred_class, confidence, doa_rel_deg);
}

void drv_oled_draw_arrow(float beta_deg) {
    ESP_LOGD(TAG, "[ARROW ] beta=%.1f deg", beta_deg);
}
