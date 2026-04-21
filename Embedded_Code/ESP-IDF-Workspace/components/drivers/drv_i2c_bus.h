#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"

// 共享 I2C master bus（挂 DRV2605L 振动马达 + BMI270 IMU）。
// 必须在 app_main 中、任何 add_device 之前调用一次 drv_i2c_bus_init()。
esp_err_t              drv_i2c_bus_init(void);
i2c_master_bus_handle_t drv_i2c_bus_handle(void);
