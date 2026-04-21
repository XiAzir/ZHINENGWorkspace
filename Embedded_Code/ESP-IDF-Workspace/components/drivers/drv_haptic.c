#include "drv_haptic.h"
#include "drv_i2c_bus.h"
#include "driver/i2c_master.h"
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

static i2c_master_dev_handle_t s_haptic_dev = NULL;

esp_err_t drv_haptic_init(void) {
    i2c_master_bus_handle_t bus = drv_i2c_bus_handle();
    if (!bus) {
        ESP_LOGE(TAG, "I2C bus not initialized; call drv_i2c_bus_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DRV2605L_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &s_haptic_dev));

    // DRV2605L 初始化序列（LRA 模式）
    drv_haptic_write_reg(REG_MODE, 0x00);       // 退出待机
    drv_haptic_write_reg(REG_LIBRARY, 0x06);    // LRA 库
    drv_haptic_write_reg(REG_FEEDBACK, 0xB6);   // LRA 反馈控制
    drv_haptic_write_reg(REG_CONTROL3, 0xA3);   // 开环驱动

    ESP_LOGI(TAG, "DRV2605L init OK (shared I2C bus)");
    return ESP_OK;
}

esp_err_t drv_haptic_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_haptic_dev, buf, 2, pdMS_TO_TICKS(100));
}

esp_err_t drv_haptic_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(s_haptic_dev, &reg, 1, val, 1, pdMS_TO_TICKS(100));
}

esp_err_t drv_haptic_play(uint8_t waveform_idx) {
    drv_haptic_write_reg(REG_WAVESEQ1, waveform_idx);
    drv_haptic_write_reg(REG_WAVESEQ2, 0x00);   // 序列终止符
    return drv_haptic_write_reg(REG_GO, 0x01);   // 触发播放
}
