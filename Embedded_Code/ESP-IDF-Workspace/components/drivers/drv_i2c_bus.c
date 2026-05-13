#include "drv_i2c_bus.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "drv_i2c_bus";

#define I2C_BUS_SDA 0
#define I2C_BUS_SCL 1

#define I2C_SW_HALF_PERIOD_US       2
#define I2C_SW_STRETCH_TIMEOUT_US   1000
#define I2C_SW_LOCK_TIMEOUT_MS      100

static bool s_ready = false;
static SemaphoreHandle_t s_lock = NULL;

static inline void delay_half_period(void)
{
    esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
}

static inline void sda_release(void)
{
    gpio_set_level(I2C_BUS_SDA, 1);
}

static inline void sda_low(void)
{
    gpio_set_level(I2C_BUS_SDA, 0);
}

static inline void scl_release(void)
{
    gpio_set_level(I2C_BUS_SCL, 1);
}

static inline void scl_low(void)
{
    gpio_set_level(I2C_BUS_SCL, 0);
}

static esp_err_t wait_scl_high(void)
{
    for (int elapsed = 0; elapsed < I2C_SW_STRETCH_TIMEOUT_US; elapsed += I2C_SW_HALF_PERIOD_US) {
        if (gpio_get_level(I2C_BUS_SCL)) {
            return ESP_OK;
        }
        esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t raise_scl(void)
{
    scl_release();
    esp_err_t err = wait_scl_high();
    delay_half_period();
    return err;
}

static esp_err_t bus_lock(void)
{
    if (!s_ready || !s_lock) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(I2C_SW_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void bus_unlock(void)
{
    xSemaphoreGive(s_lock);
}

static void bus_recover(void)
{
    sda_release();
    for (int i = 0; i < 9; i++) {
        scl_release();
        esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
        scl_low();
        esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
    }
    sda_low();
    esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
    scl_release();
    esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
    sda_release();
    esp_rom_delay_us(I2C_SW_HALF_PERIOD_US);
}

static esp_err_t sw_start(void)
{
    sda_release();
    scl_release();
    esp_err_t err = wait_scl_high();
    if (err != ESP_OK) {
        return err;
    }
    delay_half_period();

    if (!gpio_get_level(I2C_BUS_SDA)) {
        bus_recover();
        if (!gpio_get_level(I2C_BUS_SDA)) {
            return ESP_ERR_TIMEOUT;
        }
    }

    sda_low();
    delay_half_period();
    scl_low();
    delay_half_period();
    return ESP_OK;
}

static esp_err_t sw_stop(void)
{
    sda_low();
    delay_half_period();
    esp_err_t err = raise_scl();
    sda_release();
    delay_half_period();
    return err;
}

static esp_err_t sw_write_byte(uint8_t byte, bool require_ack)
{
    for (int bit = 7; bit >= 0; bit--) {
        if (byte & (1U << bit)) {
            sda_release();
        } else {
            sda_low();
        }
        delay_half_period();
        esp_err_t err = raise_scl();
        if (err != ESP_OK) {
            return err;
        }
        scl_low();
        delay_half_period();
    }

    sda_release();
    delay_half_period();
    esp_err_t err = raise_scl();
    if (err != ESP_OK) {
        return err;
    }
    bool ack = (gpio_get_level(I2C_BUS_SDA) == 0);
    scl_low();
    delay_half_period();

    if (require_ack && !ack) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static esp_err_t sw_read_byte(uint8_t *byte, bool ack_after)
{
    uint8_t value = 0;
    sda_release();

    for (int bit = 7; bit >= 0; bit--) {
        delay_half_period();
        esp_err_t err = raise_scl();
        if (err != ESP_OK) {
            return err;
        }
        if (gpio_get_level(I2C_BUS_SDA)) {
            value |= (uint8_t)(1U << bit);
        }
        scl_low();
        delay_half_period();
    }

    if (ack_after) {
        sda_low();
    } else {
        sda_release();
    }
    delay_half_period();
    esp_err_t err = raise_scl();
    scl_low();
    sda_release();
    delay_half_period();
    if (err != ESP_OK) {
        return err;
    }

    *byte = value;
    return ESP_OK;
}

static const char *probe_result_text(esp_err_t err)
{
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

esp_err_t drv_i2c_bus_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << I2C_BUS_SDA) | (1ULL << I2C_BUS_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    sda_release();
    scl_release();
    bus_recover();

    s_ready = true;
    ESP_LOGI(TAG, "Shared software I2C bus ready (SDA=%d SCL=%d)", I2C_BUS_SDA, I2C_BUS_SCL);
    return ESP_OK;
}

esp_err_t drv_i2c_bus_probe(uint16_t address)
{
    esp_err_t err = bus_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = sw_start();
    if (err == ESP_OK) {
        err = sw_write_byte((uint8_t)(address << 1), true);
    }
    (void)sw_stop();
    bus_unlock();
    return err;
}

esp_err_t drv_i2c_bus_write(uint16_t address, const uint8_t *data, size_t len, bool ignore_data_nack)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = bus_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = sw_start();
    if (err == ESP_OK) {
        err = sw_write_byte((uint8_t)(address << 1), true);
    }
    for (size_t i = 0; err == ESP_OK && i < len; i++) {
        err = sw_write_byte(data[i], !ignore_data_nack);
    }

    esp_err_t stop_err = sw_stop();
    bus_unlock();
    return (err == ESP_OK) ? stop_err : err;
}

esp_err_t drv_i2c_bus_write_read(uint16_t address,
                                 const uint8_t *write_data,
                                 size_t write_len,
                                 uint8_t *read_data,
                                 size_t read_len,
                                 bool ignore_write_nack)
{
    if ((!write_data && write_len > 0) || (!read_data && read_len > 0) || read_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = bus_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = sw_start();
    if (err == ESP_OK) {
        err = sw_write_byte((uint8_t)(address << 1), true);
    }
    for (size_t i = 0; err == ESP_OK && i < write_len; i++) {
        err = sw_write_byte(write_data[i], !ignore_write_nack);
    }
    if (err == ESP_OK) {
        err = sw_start();
    }
    if (err == ESP_OK) {
        err = sw_write_byte((uint8_t)((address << 1) | 0x01), true);
    }
    for (size_t i = 0; err == ESP_OK && i < read_len; i++) {
        err = sw_read_byte(&read_data[i], i + 1 < read_len);
    }

    esp_err_t stop_err = sw_stop();
    bus_unlock();
    return (err == ESP_OK) ? stop_err : err;
}

void drv_i2c_bus_log_known_devices(void)
{
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
