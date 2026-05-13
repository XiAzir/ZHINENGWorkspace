#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 屏幕分辨率
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)  // 8 pages

esp_err_t drv_oled_init(void);
void      drv_oled_spi_write(const uint8_t *buf, size_t len, bool is_data);
void      drv_oled_clear(void);
void      drv_oled_refresh(void);

// 基础绘图 API
void drv_oled_draw_pixel(int x, int y, bool color);
void drv_oled_fill(bool color);
void drv_oled_draw_char(int x, int y, char c, bool color, int scale);
void drv_oled_draw_string(int x, int y, const char *str, bool color, int scale);
void drv_oled_draw_line(int x0, int y0, int x1, int y1, bool color);
void drv_oled_draw_rect(int x, int y, int w, int h, bool color);
void drv_oled_fill_rect(int x, int y, int w, int h, bool color);
void drv_oled_draw_circle(int cx, int cy, int r, bool color);

// 业务层渲染（由 task_gui / task_gui_fast 调用）
void drv_oled_draw_status(int level, int pred_class, float confidence, float doa_rel_deg);
void drv_oled_draw_arrow(float beta_deg);  // β_render（相对眼镜朝向）
