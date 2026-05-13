#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// BMI270 6 轴 IMU（加速度 + 陀螺仪），挂在共享 I2C 总线。
//
// 现阶段（M1）：未加载 BMI270 初始化固件（8KB config blob，Bosch 官方下载），
// 故 drv_imu_init() 会在 chip-id 不匹配或初始化失败时返回错误码；
// task_orientation 需对此兜底（yaw 固定 0）。板到后补齐 bmi270_config_file[]。
typedef struct {
    float    ax, ay, az;       // m/s²
    float    gx, gy, gz;       // rad/s
    uint32_t timestamp_us;
} ImuSample_t;

esp_err_t drv_imu_init(void);
esp_err_t drv_imu_read(ImuSample_t *out);
bool      drv_imu_is_ready(void);
