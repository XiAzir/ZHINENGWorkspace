#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// BMI270 6 轴 IMU（加速度 + 陀螺仪），挂在共享 I2C 总线。
// drv_imu_init() 会完成地址探测、chip-id 校验、BMI270 config blob 加载、
// acc/gyro 量程配置和一次数据读回验证；若初始化失败，task_orientation
// 仍会兜底为 yaw 固定 0 的桩模式，保证主业务继续运行。
typedef struct {
    float    ax, ay, az;       // m/s²
    float    gx, gy, gz;       // rad/s
    uint32_t timestamp_us;
} ImuSample_t;

esp_err_t drv_imu_init(void);
esp_err_t drv_imu_read(ImuSample_t *out);
bool      drv_imu_is_ready(void);
