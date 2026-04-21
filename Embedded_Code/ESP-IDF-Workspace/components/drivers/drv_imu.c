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

// BMI270 I2C 地址（SDO→GND 时为 0x68；→VCC 时为 0x69）
#define BMI270_I2C_ADDR    0x68
#define BMI270_CHIP_ID     0x24

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
#define REG_DATA_8         0x0C   // 加速度起始

// 默认量程配置：±2g / ±2000 dps
#define ACC_SENSITIVITY    (9.80665f / 16384.0f)               // LSB → m/s²
#define GYR_SENSITIVITY    ((float)M_PI / 180.0f / 16.384f)    // LSB → rad/s

static i2c_master_dev_handle_t s_dev = NULL;
static bool                    s_ready = false;

// BMI270 初始化固件（M1 占位：大小为 0）。
// M5 板到后：从 Bosch BMI270 SensorAPI 仓库下载 bmi270_config_file[8192] 替换。
__attribute__((weak)) const uint8_t bmi270_config_file[] = {0};
__attribute__((weak)) const unsigned int bmi270_config_file_len = 0;

static esp_err_t w1(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t rn(uint8_t reg, uint8_t *dst, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, n, pdMS_TO_TICKS(50));
}

static esp_err_t load_config_firmware(void) {
    if (bmi270_config_file_len == 0) {
        ESP_LOGW(TAG, "BMI270 config blob is empty (M1 stub). Skipping firmware load.");
        return ESP_ERR_NOT_SUPPORTED;
    }
    // Bosch 官方流程：PWR_CONF=0 → INIT_CTRL=0 → 突发写 INIT_DATA → INIT_CTRL=1 → 轮询 INTERNAL_STATUS
    w1(REG_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));
    w1(REG_INIT_CTRL, 0x00);

    // 突发写 config blob（API 要求分 <=32 字节块）
    const uint8_t *p = bmi270_config_file;
    unsigned int remaining = bmi270_config_file_len;
    uint8_t burst[33];
    while (remaining > 0) {
        unsigned int chunk = remaining > 32 ? 32 : remaining;
        burst[0] = REG_INIT_DATA;
        memcpy(burst + 1, p, chunk);
        esp_err_t err = i2c_master_transmit(s_dev, burst, chunk + 1, pdMS_TO_TICKS(100));
        if (err != ESP_OK) return err;
        p         += chunk;
        remaining -= chunk;
    }

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
    i2c_master_bus_handle_t bus = drv_i2c_bus_handle();
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMI270_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;

    // 探测 chip id
    uint8_t chip_id = 0;
    err = rn(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BMI270_CHIP_ID) {
        ESP_LOGW(TAG, "BMI270 not detected (chip_id=0x%02X, err=%s). Stub mode.",
                 chip_id, esp_err_to_name(err));
        s_ready = false;
        return ESP_ERR_NOT_FOUND;
    }

    // 软复位
    w1(REG_CMD, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(5));

    if (load_config_firmware() != ESP_OK) {
        ESP_LOGW(TAG, "BMI270 config load skipped/failed. IMU remains uninitialized.");
        s_ready = false;
        return ESP_ERR_NOT_SUPPORTED;
    }

    // 开 acc+gyro
    w1(REG_PWR_CTRL, 0x0E);          // aux=0 gyro=1 acc=1 temp=1
    w1(REG_ACC_CONF,  0xA8);         // 100Hz, normal avg
    w1(REG_ACC_RANGE, 0x00);         // ±2g
    w1(REG_GYR_CONF,  0xA9);         // 200Hz, normal
    w1(REG_GYR_RANGE, 0x04);         // ±2000 dps
    w1(REG_PWR_CONF,  0x02);         // disable adv power save
    vTaskDelay(pdMS_TO_TICKS(1));

    s_ready = true;
    ESP_LOGI(TAG, "BMI270 init OK");
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
