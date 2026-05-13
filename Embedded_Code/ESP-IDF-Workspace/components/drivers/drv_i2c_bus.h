#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// 共享 I2C 总线（挂 DRV2605L 振动马达 + BMI270 IMU）。
// 当前使用软件 I2C 后端，以绕过 ESP32-P4 I2C 控制器在 BMI270 异常 NACK
// 场景下提前中止事务的问题。地址阶段仍严格检查 ACK；BMI270 可选择忽略
// 寄存器/数据阶段 NACK 后继续事务。
esp_err_t drv_i2c_bus_init(void);
esp_err_t drv_i2c_bus_probe(uint16_t address);
esp_err_t drv_i2c_bus_write(uint16_t address, const uint8_t *data, size_t len, bool ignore_data_nack);
esp_err_t drv_i2c_bus_write_read(uint16_t address,
                                 const uint8_t *write_data,
                                 size_t write_len,
                                 uint8_t *read_data,
                                 size_t read_len,
                                 bool ignore_write_nack);
void      drv_i2c_bus_log_known_devices(void);
