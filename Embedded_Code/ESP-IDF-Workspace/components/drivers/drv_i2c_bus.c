#include "drv_i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "drv_i2c_bus";

#define I2C_BUS_PORT   I2C_NUM_0
#define I2C_BUS_SDA    0
#define I2C_BUS_SCL    1
#define I2C_PROBE_TIMEOUT_MS  50

static i2c_master_bus_handle_t s_bus = NULL;

static const char *probe_result_text(esp_err_t err) {
    switch (err) {
        case ESP_OK:
            return "ACK";
        case ESP_ERR_NOT_FOUND:
            return "NACK";
        case ESP_ERR_TIMEOUT:
            return "TIMEOUT";
        case ESP_ERR_INVALID_STATE:
            return "BUS_NOT_READY";
        default:
            return esp_err_to_name(err);
    }
}

esp_err_t drv_i2c_bus_init(void) {
    if (s_bus) return ESP_OK;  // 幂等

    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_BUS_PORT,
        .sda_io_num        = I2C_BUS_SDA,
        .scl_io_num        = I2C_BUS_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
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

esp_err_t drv_i2c_bus_probe(uint16_t address) {
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    return i2c_master_probe(s_bus, address, I2C_PROBE_TIMEOUT_MS);
}

void drv_i2c_bus_log_known_devices(void) {
    static const struct {
        uint16_t    addr;
        const char *name;
    } s_known[] = {
        {0x5A, "DRV2605L"},
        {0x68, "BMI270(primary)"},
        {0x69, "BMI270(alt)"},
    };

    ESP_LOGI(TAG, "I2C probe summary on GPIO%d/GPIO%d:", I2C_BUS_SDA, I2C_BUS_SCL);
    for (size_t i = 0; i < sizeof(s_known) / sizeof(s_known[0]); i++) {
        esp_err_t err = drv_i2c_bus_probe(s_known[i].addr);
        ESP_LOGI(TAG, "  - 0x%02X %-15s -> %s",
                 s_known[i].addr,
                 s_known[i].name,
                 probe_result_text(err));
    }
}
