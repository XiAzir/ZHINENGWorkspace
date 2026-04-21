#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t drv_oled_init(void);
void      drv_oled_spi_write(const uint8_t *buf, size_t len, bool is_data);
void      drv_oled_clear(void);

// M2 完善像素渲染；M1 占位为 ESP_LOGD，便于仿真联调。
void      drv_oled_draw_status(int level, int pred_class, float confidence, float doa_rel_deg);
void      drv_oled_draw_arrow (float beta_deg);  // β_render（相对眼镜朝向）
