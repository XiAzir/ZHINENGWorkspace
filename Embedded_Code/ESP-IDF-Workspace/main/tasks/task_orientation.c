#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#include "app_config.h"
#include "app_freertos.h"
#include "drv_imu.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "task_orient";

// Mahony 6 轴互补滤波参数
// Kp 加速度校正强度；Ki 偏置积分（过大会漂移，过小对温漂无效）。
#define MAHONY_KP        1.0f
#define MAHONY_KI        0.01f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float ix = 0.0f, iy = 0.0f, iz = 0.0f;

// 用加速度矢量估算初始 roll/pitch，Yaw 置 0（6 轴无磁力计给不出绝对方位）。
// 用户之后的 Yaw 是"相对开机朝向"；体验上等价于上电瞬间按下"朝向归零"。
static void init_quaternion_from_accel(const ImuSample_t *s) {
    float ax = s->ax, ay = s->ay, az = s->az;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-3f) return;  // 异常：保持单位四元数

    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw   = 0.0f;

    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}

static void mahony_update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt) {
    // 加速度归一化（自由落体或强加速时跳过修正，避免破坏陀螺积分）
    float a_norm_sq = ax*ax + ay*ay + az*az;
    if (a_norm_sq > 1e-6f) {
        float inv = 1.0f / sqrtf(a_norm_sq);
        ax *= inv; ay *= inv; az *= inv;

        // q 表示的重力方向（世界 Z 轴旋到机体坐标系）
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 误差向量 = 测到的重力 × 估算的重力
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        if (MAHONY_KI > 0.0f) {
            ix += MAHONY_KI * ex * dt;
            iy += MAHONY_KI * ey * dt;
            iz += MAHONY_KI * ez * dt;
            gx += ix;
            gy += iy;
            gz += iz;
        }

        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    // 四元数积分
    float qd0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qd1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qd2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qd3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += qd0 * dt;
    q1 += qd1 * dt;
    q2 += qd2 * dt;
    q3 += qd3 * dt;

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv;
    }
}

static float yaw_deg_from_quaternion(void) {
    float yaw_rad = atan2f(2.0f * (q0 * q3 + q1 * q2),
                           1.0f - 2.0f * (q2 * q2 + q3 * q3));
    return yaw_rad * (180.0f / (float)M_PI);
}

void TaskOrientation_Run(void *arg) {
    ESP_LOGI(TAG, "Orientation task started on core %d", xPortGetCoreID());

    bool imu_ok = drv_imu_is_ready();
    if (!imu_ok) {
        ESP_LOGW(TAG, "IMU not ready; Yaw will be pinned to 0.0 (stub mode).");
        orientation_yaw_set(0.0f);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒唤醒一次，监听未来重试（M5）
        }
    }

    // 开机静置 0.5s，用加速度初始化 roll/pitch
    ImuSample_t s;
    int init_samples = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    TickType_t t_end = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (xTaskGetTickCount() < t_end) {
        if (drv_imu_read(&s) == ESP_OK) {
            ax_sum += s.ax; ay_sum += s.ay; az_sum += s.az;
            init_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(ORIENT_TICK_MS));
    }
    if (init_samples > 0) {
        ImuSample_t avg = { .ax = ax_sum / init_samples,
                            .ay = ay_sum / init_samples,
                            .az = az_sum / init_samples };
        init_quaternion_from_accel(&avg);
        ESP_LOGI(TAG, "Quaternion initialized from %d samples", init_samples);
    }

    uint32_t t_prev = (uint32_t)esp_timer_get_time();
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(ORIENT_TICK_MS));

        if (drv_imu_read(&s) != ESP_OK) continue;
        uint32_t t_now = s.timestamp_us;
        float dt = (t_now - t_prev) * 1e-6f;
        t_prev = t_now;
        if (dt <= 0.0f || dt > 0.1f) continue;  // 丢弃异常 dt

        mahony_update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, dt);
        orientation_yaw_set(yaw_deg_from_quaternion());
    }
}
