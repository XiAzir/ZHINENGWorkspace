#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_freertos.h"
#include "drv_oled.h"

static const char *TAG = "task_gui_fast";

// 3DoF 锚定快线程（文档《关于危险时间和3DoF.md》§2.2）：
//   每帧 β_render = θ_world − Yaw_current
// 若 θ_world 尚未更新（开机早期），默认为 0；Yaw 缺 IMU 时也为 0 → β=0（指正前方）。
void TaskGUI_Fast_Run(void *arg) {
    ESP_LOGI(TAG, "GUI fast task started on core %d", xPortGetCoreID());

    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(GUI_FAST_TICK_MS));

        float theta_w = theta_world_get();
        float yaw     = orientation_yaw_get();
        float beta    = theta_w - yaw;

        // 归一到 [-180, 180]
        while (beta >  180.0f) beta -= 360.0f;
        while (beta < -180.0f) beta += 360.0f;

        drv_oled_draw_arrow(beta);
    }
}
