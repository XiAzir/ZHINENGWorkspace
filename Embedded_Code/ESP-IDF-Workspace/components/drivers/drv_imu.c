#include "drv_imu.h"
#include "drv_i2c_bus.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "drv_imu";

#define BMI270_I2C_ADDR_PRIMARY  0x68
#define BMI270_I2C_ADDR_ALT      0x69
#define BMI270_CHIP_ID           0x24

// 关键寄存器
#define REG_CHIP_ID        0x00
#define REG_PWR_CONF       0x7C
#define REG_PWR_CTRL       0x7D
#define REG_INIT_CTRL      0x59
#define REG_INIT_DATA      0x5E
#define REG_INTERNAL_STATUS 0x21
#define REG_ACC_CONF       0x40
#define REG_GYR_CONF       0x42
#define REG_ACC_RANGE      0x41
#define REG_GYR_RANGE      0x43
#define REG_CMD            0x7E
#define REG_DATA_8         0x0C

// 默认量程配置
#define ACC_SENSITIVITY    (9.80665f / 16384.0f)
#define GYR_SENSITIVITY    ((float)M_PI / 180.0f / 16.384f)

static i2c_master_bus_handle_t s_bus = NULL;
static bool                    s_ready = false;
static uint16_t                s_dev_addr = BMI270_I2C_ADDR_PRIMARY;

// config blob 占位
__attribute__((weak)) const uint8_t bmi270_config_file[] = {0};
__attribute__((weak)) const unsigned int bmi270_config_file_len = 0;

// ── 为每次操作创建/销毁临时设备句柄，完全复刻诊断成功路径 ──

static i2c_master_dev_handle_t make_dev(void) {
    i2c_master_dev_handle_t dev = NULL;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = s_dev_addr,
        .scl_speed_hz    = 100000,
        .flags.disable_ack_check = true,
    };
    i2c_master_bus_add_device(s_bus, &cfg, &dev);
    return dev;
}

static void free_dev(i2c_master_dev_handle_t dev) {
    if (dev) i2c_master_bus_rm_device(dev);
}

static esp_err_t w1(uint8_t reg, uint8_t val) {
    i2c_master_dev_handle_t dev = make_dev();
    if (!dev) return ESP_ERR_NO_MEM;
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(dev, buf, 2, pdMS_TO_TICKS(50));
    free_dev(dev);
    return err;
}

static esp_err_t rn(uint8_t reg, uint8_t *dst, size_t n) {
    i2c_master_dev_handle_t dev = make_dev();
    if (!dev) return ESP_ERR_NO_MEM;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, dst, n, pdMS_TO_TICKS(50));
    free_dev(dev);
    return err;
}

static void log_reg_probe(uint8_t reg, const char *name) {
    uint8_t val = 0;
    esp_err_t err = rn(reg, &val, 1);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BMI270 reg %-16s (0x%02X) = 0x%02X", name, reg, val);
    } else {
        ESP_LOGW(TAG, "BMI270 reg %-16s (0x%02X) read failed: %s", name, reg, esp_err_to_name(err));
    }
}

static void log_bmi270_register_fingerprint(void) {
    ESP_LOGI(TAG, "BMI270 register fingerprint on 0x%02X:", s_dev_addr);
    log_reg_probe(REG_CHIP_ID, "CHIP_ID");
    log_reg_probe(0x01, "ERR_REG");
    log_reg_probe(0x02, "STATUS");
    log_reg_probe(REG_INTERNAL_STATUS, "INTERNAL_STATUS");
    log_reg_probe(REG_PWR_CONF, "PWR_CONF");
    log_reg_probe(REG_PWR_CTRL, "PWR_CTRL");
}

static esp_err_t detect_bmi270_address(uint16_t *out_addr) {
    esp_err_t err_68 = drv_i2c_bus_probe(BMI270_I2C_ADDR_PRIMARY);
    esp_err_t err_69 = drv_i2c_bus_probe(BMI270_I2C_ADDR_ALT);
    if (err_68 == ESP_OK) { *out_addr = BMI270_I2C_ADDR_PRIMARY; return ESP_OK; }
    if (err_69 == ESP_OK) { *out_addr = BMI270_I2C_ADDR_ALT;     return ESP_OK; }
    ESP_LOGW(TAG, "BMI270 probe failed: 0x68=%s, 0x69=%s", esp_err_to_name(err_68), esp_err_to_name(err_69));
    if (err_68 == ESP_ERR_TIMEOUT || err_69 == ESP_ERR_TIMEOUT)
        ESP_LOGW(TAG, "Check SDA/SCL pull-up, wiring, or power.");
    return (err_68 == ESP_ERR_TIMEOUT || err_69 == ESP_ERR_TIMEOUT) ? ESP_ERR_TIMEOUT : ESP_ERR_NOT_FOUND;
}

static esp_err_t load_config_firmware(void) {
    if (bmi270_config_file_len == 0) {
        ESP_LOGW(TAG, "Config blob empty, skipping firmware load.");
        return ESP_ERR_NOT_SUPPORTED;
    }
    w1(REG_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    w1(REG_INIT_CTRL, 0x00);
    const uint8_t *p = bmi270_config_file;
    unsigned int remaining = bmi270_config_file_len;
    uint8_t burst[33];
    i2c_master_dev_handle_t dev = make_dev();
    if (!dev) return ESP_ERR_NO_MEM;
    while (remaining > 0) {
        unsigned int chunk = remaining > 32 ? 32 : remaining;
        burst[0] = REG_INIT_DATA;
        memcpy(burst + 1, p, chunk);
        esp_err_t err = i2c_master_transmit(dev, burst, chunk + 1, pdMS_TO_TICKS(100));
        if (err != ESP_OK) { free_dev(dev); return err; }
        p += chunk;
        remaining -= chunk;
    }
    free_dev(dev);
    w1(REG_INIT_CTRL, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        rn(REG_INTERNAL_STATUS, &status, 1);
        if ((status & 0x0F) == 0x01) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGE(TAG, "BMI270 init status=0x%02X (expected 0x01)", status);
    return ESP_ERR_INVALID_STATE;
}

esp_err_t drv_imu_init(void) {
    s_bus = drv_i2c_bus_handle();
    if (!s_bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t detected_addr = 0;
    esp_err_t err = detect_bmi270_address(&detected_addr);
    if (err != ESP_OK) { s_ready = false; return err; }
    s_dev_addr = detected_addr;
    ESP_LOGI(TAG, "BMI270 detected at I2C address 0x%02X", s_dev_addr);

    log_bmi270_register_fingerprint();

    uint8_t chip_id = 0;
    err = rn(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BMI270_CHIP_ID) {
        ESP_LOGW(TAG, "BMI270 chip-id check failed at 0x%02X (chip_id=0x%02X, err=%s). "
                 "Check SDO/CSB mode pins and module power.", s_dev_addr, chip_id, esp_err_to_name(err));
        s_ready = false;
        return ESP_ERR_NOT_FOUND;
    }

    w1(REG_CMD, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(5));

    if (load_config_firmware() != ESP_OK) {
        ESP_LOGW(TAG, "Config load skipped/failed, IMU uninitialized.");
        s_ready = false;
        return ESP_ERR_NOT_SUPPORTED;
    }

    w1(REG_PWR_CTRL, 0x0E);
    w1(REG_ACC_CONF,  0xA8);
    w1(REG_ACC_RANGE, 0x00);
    w1(REG_GYR_CONF,  0xA9);
    w1(REG_GYR_RANGE, 0x04);
    w1(REG_PWR_CONF,  0x02);
    vTaskDelay(pdMS_TO_TICKS(1));

    s_ready = true;
    ESP_LOGI(TAG, "BMI270 init OK (addr=0x%02X)", s_dev_addr);
    return ESP_OK;
}

bool drv_imu_is_ready(void) { return s_ready; }

esp_err_t drv_imu_read(ImuSample_t *out) {
    if (!s_ready) return ESP_ERR_INVALID_STATE;

    uint8_t raw[12];
    esp_err_t err = rn(REG_DATA_8, raw, sizeof(raw));
    if (err != ESP_OK) return err;

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
