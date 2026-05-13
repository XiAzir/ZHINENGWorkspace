#include "drv_haptic.h"
#include "drv_i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "drv_haptic";

#define DRV2605L_ADDR    0x5A

// DRV2605L 寄存器
#define REG_STATUS       0x00
#define REG_MODE         0x01
#define REG_REALTIME_FBK 0x02
#define REG_LIBRARY      0x03
#define REG_WAVESEQ1     0x04
#define REG_WAVESEQ2     0x05
#define REG_GO           0x0C
#define REG_RATED_VOLT   0x16
#define REG_OD_CLAMP     0x17
#define REG_FEEDBACK     0x1A
#define REG_CONTROL3     0x1D

esp_err_t drv_haptic_init(void) {
    esp_err_t err = drv_i2c_bus_probe(DRV2605L_ADDR);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C bus not initialized; call drv_i2c_bus_init() first");
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DRV2605L probe failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t status = 0;
    err = drv_haptic_read_reg(REG_STATUS, &status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DRV2605L status read failed before init: %s", esp_err_to_name(err));
        goto init_fail;
    }
    ESP_LOGI(TAG, "DRV2605L status before init: 0x%02X", status);

    // DRV2605L 初始化序列（LRA 模式）
    err = drv_haptic_write_reg(REG_MODE, 0x00);       // 退出待机
    if (err != ESP_OK) goto init_fail;
    err = drv_haptic_write_reg(REG_LIBRARY, 0x06);    // LRA 库
    if (err != ESP_OK) goto init_fail;
    err = drv_haptic_write_reg(REG_FEEDBACK, 0xB6);   // LRA 反馈控制
    if (err != ESP_OK) goto init_fail;
    err = drv_haptic_write_reg(REG_CONTROL3, 0xA3);   // 开环驱动
    if (err != ESP_OK) goto init_fail;

    ESP_LOGI(TAG, "DRV2605L init OK (shared I2C bus)");
    return ESP_OK;

init_fail:
    ESP_LOGE(TAG, "DRV2605L init failed during register write: %s", esp_err_to_name(err));
    return err;
}

esp_err_t drv_haptic_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return drv_i2c_bus_write(DRV2605L_ADDR, buf, sizeof(buf), false);
}

esp_err_t drv_haptic_read_reg(uint8_t reg, uint8_t *val) {
    return drv_i2c_bus_write_read(DRV2605L_ADDR, &reg, 1, val, 1, false);
}

esp_err_t drv_haptic_play(uint8_t waveform_idx) {
    drv_haptic_write_reg(REG_WAVESEQ1, waveform_idx);
    drv_haptic_write_reg(REG_WAVESEQ2, 0x00);   // 序列终止符
    return drv_haptic_write_reg(REG_GO, 0x01);   // 触发播放
}
