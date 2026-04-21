#include "drv_i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "drv_i2c_bus";

#define I2C_BUS_PORT   I2C_NUM_0
#define I2C_BUS_SDA    15
#define I2C_BUS_SCL    16

static i2c_master_bus_handle_t s_bus = NULL;

esp_err_t drv_i2c_bus_init(void) {
    if (s_bus) return ESP_OK;  // 幂等

    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_BUS_PORT,
        .sda_io_num        = I2C_BUS_SDA,
        .scl_io_num        = I2C_BUS_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Shared I2C bus ready (SDA=%d SCL=%d)", I2C_BUS_SDA, I2C_BUS_SCL);
    return ESP_OK;
}

i2c_master_bus_handle_t drv_i2c_bus_handle(void) {
    return s_bus;
}
