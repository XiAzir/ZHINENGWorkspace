#include "drv_imu.h"
#include "drv_i2c_bus.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "drv_imu";

#define BMI270_I2C_ADDR_PRIMARY  0x68
#define BMI270_I2C_ADDR_ALT      0x69
#define BMI270_CHIP_ID           0x24

#define REG_CHIP_ID          0x00
#define REG_ERR_REG          0x02
#define REG_STATUS           0x03
#define REG_DATA_8           0x0C
#define REG_INTERNAL_STATUS  0x21
#define REG_ACC_CONF         0x40
#define REG_ACC_RANGE        0x41
#define REG_GYR_CONF         0x42
#define REG_GYR_RANGE        0x43
#define REG_INIT_CTRL        0x59
#define REG_INIT_ADDR_0      0x5B
#define REG_INIT_ADDR_1      0x5C
#define REG_INIT_DATA        0x5E
#define REG_PWR_CONF         0x7C
#define REG_PWR_CTRL         0x7D
#define REG_CMD              0x7E

#define BMI270_CONFIG_CHUNK_SIZE 32
#define BMI270_CONFIG_LOAD_TIMEOUT_MS 150
#define BMI270_CONFIG_LOAD_POLL_MS    5

#define BMI270_ACC_RANGE_2G       0x00
#define BMI270_GYR_RANGE_2000DPS  0x00
#define BMI270_PWR_CTRL_ACC_GYR_TEMP 0x0E

// 默认量程配置：±2g / ±2000 dps
#define ACC_SENSITIVITY    (9.80665f / 16384.0f)
#define GYR_SENSITIVITY    ((float)M_PI / 180.0f / 16.384f)

extern const uint8_t bmi270_context_config_file[];
extern const int bmi270_context_config_file_size;

static bool     s_ready = false;
static uint16_t s_dev_addr = BMI270_I2C_ADDR_PRIMARY;

static void delay_ms(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks > 0) {
        vTaskDelay(ticks);
    } else if (ms > 0) {
        esp_rom_delay_us(ms * 1000U);
    }
}

static esp_err_t write_bytes(uint8_t reg, const uint8_t *src, size_t n)
{
    if (!src || n > BMI270_CONFIG_CHUNK_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[BMI270_CONFIG_CHUNK_SIZE + 1];
    buf[0] = reg;
    memcpy(buf + 1, src, n);

    return drv_i2c_bus_write(s_dev_addr, buf, n + 1, true);
}

static esp_err_t set_config_addr(size_t byte_offset)
{
    if ((byte_offset & 0x01U) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t base = (uint16_t)(byte_offset / 2U);
    uint8_t addr[2] = {
        (uint8_t)(base & 0x0F),
        (uint8_t)(base >> 4),
    };

    return write_bytes(REG_INIT_ADDR_0, addr, sizeof(addr));
}

static esp_err_t w1(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return drv_i2c_bus_write(s_dev_addr, buf, sizeof(buf), true);
}

static esp_err_t rn(uint8_t reg, uint8_t *dst, size_t n)
{
    return drv_i2c_bus_write_read(s_dev_addr, &reg, 1, dst, n, true);
}

static esp_err_t verify_reg_mask(uint8_t reg, uint8_t expected, uint8_t mask, const char *name)
{
    uint8_t actual = 0;
    esp_err_t err = rn(reg, &actual, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 %s readback failed: %s", name, esp_err_to_name(err));
        return err;
    }

    if ((actual & mask) != (expected & mask)) {
        ESP_LOGE(TAG, "BMI270 %s readback mismatch: got 0x%02X expected 0x%02X mask 0x%02X",
                 name, actual, expected, mask);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static void log_reg_probe(uint8_t reg, const char *name)
{
    uint8_t val = 0;
    esp_err_t err = rn(reg, &val, 1);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BMI270 reg %-16s (0x%02X) = 0x%02X", name, reg, val);
    } else {
        ESP_LOGW(TAG, "BMI270 reg %-16s (0x%02X) read failed: %s", name, reg, esp_err_to_name(err));
    }
}

static void log_bmi270_register_fingerprint(void)
{
    ESP_LOGI(TAG, "BMI270 register fingerprint on 0x%02X:", s_dev_addr);
    log_reg_probe(REG_CHIP_ID, "CHIP_ID");
    log_reg_probe(REG_ERR_REG, "ERR_REG");
    log_reg_probe(REG_STATUS, "STATUS");
    log_reg_probe(REG_INTERNAL_STATUS, "INTERNAL_STATUS");
    log_reg_probe(REG_PWR_CONF, "PWR_CONF");
    log_reg_probe(REG_PWR_CTRL, "PWR_CTRL");
}

static esp_err_t detect_bmi270_address(uint16_t *out_addr)
{
    esp_err_t err_68 = drv_i2c_bus_probe(BMI270_I2C_ADDR_PRIMARY);
    esp_err_t err_69 = drv_i2c_bus_probe(BMI270_I2C_ADDR_ALT);

    if (err_68 == ESP_OK) {
        *out_addr = BMI270_I2C_ADDR_PRIMARY;
        return ESP_OK;
    }
    if (err_69 == ESP_OK) {
        *out_addr = BMI270_I2C_ADDR_ALT;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "BMI270 probe failed: 0x68=%s, 0x69=%s", esp_err_to_name(err_68), esp_err_to_name(err_69));
    if (err_68 == ESP_ERR_TIMEOUT || err_69 == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Check SDA/SCL pull-up, wiring, or power.");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t load_config_firmware(void)
{
    if (bmi270_context_config_file_size <= 0) {
        ESP_LOGE(TAG, "BMI270 config blob missing.");
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Loading BMI270 config blob (%d bytes)", bmi270_context_config_file_size);

    esp_err_t err = w1(REG_PWR_CONF, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    delay_ms(1);

    err = w1(REG_INIT_CTRL, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t *p = bmi270_context_config_file;
    size_t offset = 0;
    unsigned int remaining = (unsigned int)bmi270_context_config_file_size;
    while (remaining > 0) {
        unsigned int chunk = remaining > BMI270_CONFIG_CHUNK_SIZE ? BMI270_CONFIG_CHUNK_SIZE : remaining;
        err = set_config_addr(offset);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BMI270 config address set failed at offset %u: %s",
                     (unsigned int)offset, esp_err_to_name(err));
            return err;
        }

        esp_rom_delay_us(450);

        err = write_bytes(REG_INIT_DATA, p, chunk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BMI270 config chunk write failed at offset %u: %s",
                     (unsigned int)offset, esp_err_to_name(err));
            return err;
        }

        p += chunk;
        offset += chunk;
        remaining -= chunk;
    }

    err = w1(REG_INIT_CTRL, 0x01);
    if (err != ESP_OK) {
        return err;
    }
    delay_ms(20);

    uint8_t status = 0;
    for (int elapsed_ms = 20;
         elapsed_ms <= BMI270_CONFIG_LOAD_TIMEOUT_MS;
         elapsed_ms += BMI270_CONFIG_LOAD_POLL_MS) {
        err = rn(REG_INTERNAL_STATUS, &status, 1);
        if (err == ESP_OK && (status & 0x0F) == 0x01) {
            ESP_LOGI(TAG, "BMI270 config load OK (internal_status=0x%02X)", status);
            return ESP_OK;
        }
        delay_ms(BMI270_CONFIG_LOAD_POLL_MS);
    }

    ESP_LOGE(TAG, "BMI270 init status=0x%02X (expected low nibble 0x01)", status);
    return (err == ESP_OK) ? ESP_ERR_INVALID_STATE : err;
}

esp_err_t drv_imu_init(void)
{
    s_ready = false;

    uint16_t detected_addr = 0;
    esp_err_t err = detect_bmi270_address(&detected_addr);
    if (err != ESP_OK) {
        return err;
    }
    s_dev_addr = detected_addr;
    ESP_LOGI(TAG, "BMI270 detected at I2C address 0x%02X", s_dev_addr);

    uint8_t chip_id = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = rn(REG_CHIP_ID, &chip_id, 1);
        if (err == ESP_OK && chip_id == BMI270_CHIP_ID) {
            break;
        }
        delay_ms(2);
    }
    if (err != ESP_OK || chip_id != BMI270_CHIP_ID) {
        log_bmi270_register_fingerprint();
        ESP_LOGW(TAG, "BMI270 chip-id check failed at 0x%02X (chip_id=0x%02X, err=%s). "
                 "Check SDO/CSB mode pins and module power.",
                 s_dev_addr,
                 chip_id,
                 esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMI270 chip-id OK: 0x%02X", chip_id);

    err = w1(REG_CMD, 0xB6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 soft reset failed: %s", esp_err_to_name(err));
        return err;
    }
    delay_ms(10);

    err = load_config_firmware();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMI270 config load failed: %s", esp_err_to_name(err));
        return err;
    }

    const struct {
        uint8_t reg;
        uint8_t val;
        const char *name;
    } init_regs[] = {
        {REG_PWR_CTRL,  BMI270_PWR_CTRL_ACC_GYR_TEMP, "PWR_CTRL"},
        {REG_ACC_CONF,  0xA8, "ACC_CONF"},
        {REG_ACC_RANGE, BMI270_ACC_RANGE_2G, "ACC_RANGE"},
        {REG_GYR_CONF,  0xA9, "GYR_CONF"},
        {REG_GYR_RANGE, BMI270_GYR_RANGE_2000DPS, "GYR_RANGE"},
        {REG_PWR_CONF,  0x02, "PWR_CONF"},
    };

    for (size_t i = 0; i < sizeof(init_regs) / sizeof(init_regs[0]); i++) {
        err = w1(init_regs[i].reg, init_regs[i].val);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BMI270 %s write failed: %s", init_regs[i].name, esp_err_to_name(err));
            return err;
        }
    }
    delay_ms(5);

    err = verify_reg_mask(REG_ACC_RANGE, BMI270_ACC_RANGE_2G, 0x03, "ACC_RANGE");
    if (err != ESP_OK) {
        return err;
    }
    err = verify_reg_mask(REG_GYR_RANGE, BMI270_GYR_RANGE_2000DPS, 0x07, "GYR_RANGE");
    if (err != ESP_OK) {
        return err;
    }
    err = verify_reg_mask(REG_PWR_CTRL, BMI270_PWR_CTRL_ACC_GYR_TEMP, 0x0E, "PWR_CTRL");
    if (err != ESP_OK) {
        return err;
    }

    uint8_t raw[12] = {0};
    err = rn(REG_DATA_8, raw, sizeof(raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 first data read failed: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "BMI270 init OK (addr=0x%02X)", s_dev_addr);
    return ESP_OK;
}

bool drv_imu_is_ready(void)
{
    return s_ready;
}

esp_err_t drv_imu_read(ImuSample_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[12];
    esp_err_t err = rn(REG_DATA_8, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax_r = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ay_r = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t az_r = (int16_t)((raw[5] << 8) | raw[4]);
    int16_t gx_r = (int16_t)((raw[7] << 8) | raw[6]);
    int16_t gy_r = (int16_t)((raw[9] << 8) | raw[8]);
    int16_t gz_r = (int16_t)((raw[11] << 8) | raw[10]);

    out->ax = ax_r * ACC_SENSITIVITY;
    out->ay = ay_r * ACC_SENSITIVITY;
    out->az = az_r * ACC_SENSITIVITY;
    out->gx = gx_r * GYR_SENSITIVITY;
    out->gy = gy_r * GYR_SENSITIVITY;
    out->gz = gz_r * GYR_SENSITIVITY;
    out->timestamp_us = (uint32_t)esp_timer_get_time();
    return ESP_OK;
}
