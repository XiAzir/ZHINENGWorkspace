#include "drv_oled.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

static const char *TAG = "drv_oled";

// ── 硬件引脚（WT99P4C5-S1 开发板排针）───────────────────────
#define OLED_SPI_HOST   SPI2_HOST
#define OLED_PIN_CLK    26
#define OLED_PIN_MOSI   27
#define OLED_PIN_CS     47
#define OLED_PIN_DC     33
#define OLED_PIN_RST    32

// SSD1306 命令集
#define SSD1306_CMD_DISPLAY_OFF              0xAE
#define SSD1306_CMD_DISPLAY_ON               0xAF
#define SSD1306_CMD_SET_DISPLAY_CLK_DIV      0xD5
#define SSD1306_CMD_SET_MULTIPLEX            0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET       0xD3
#define SSD1306_CMD_SET_START_LINE           0x40
#define SSD1306_CMD_SET_CHARGE_PUMP          0x8D
#define SSD1306_CMD_SEG_REMAP_NORMAL         0xA0
#define SSD1306_CMD_SEG_REMAP_INV            0xA1
#define SSD1306_CMD_COM_SCAN_DIR_NORMAL      0xC0
#define SSD1306_CMD_COM_SCAN_DIR_INV         0xC8
#define SSD1306_CMD_SET_COM_PINS             0xDA
#define SSD1306_CMD_SET_CONTRAST             0x81
#define SSD1306_CMD_SET_PRECHARGE            0xD9
#define SSD1306_CMD_SET_VCOM_DETECT          0xDB
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME    0xA4
#define SSD1306_CMD_SET_NORMAL_DISPLAY       0xA6
#define SSD1306_CMD_SET_INVERT_DISPLAY       0xA7
#define SSD1306_CMD_SET_LOWER_COLUMN_ADDR    0x00
#define SSD1306_CMD_SET_HIGHER_COLUMN_ADDR   0x10
#define SSD1306_CMD_SET_PAGE_ADDR            0xB0
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE     0x20

static spi_device_handle_t s_oled_spi = NULL;

// 帧缓冲区：128×64 / 8 = 1024 字节
static uint8_t s_framebuf[OLED_WIDTH * OLED_PAGES];

// SPI 互斥锁：gui_slow 和 gui_fast 两个任务共享同一个 SPI 设备
static SemaphoreHandle_t s_spi_mutex = NULL;

// ── SPI 底层 ────────────────────────────────────────────────

static void IRAM_ATTR oled_spi_pre_cb(spi_transaction_t *t) {
    int dc = (int)(intptr_t)t->user;
    gpio_set_level(OLED_PIN_DC, dc);
}

static void spi_send_cmd(uint8_t cmd) {
    drv_oled_spi_write(&cmd, 1, false);
}

static void spi_send_cmd_list(const uint8_t *cmds, int len) {
    for (int i = 0; i < len; i++) {
        spi_send_cmd(cmds[i]);
    }
}

void drv_oled_spi_write(const uint8_t *buf, size_t len, bool is_data) {
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = buf,
        .user      = (void *)(intptr_t)(is_data ? 1 : 0),
    };
    spi_device_transmit(s_oled_spi, &t);
}

// ── SSD1306 初始化 ──────────────────────────────────────────

static void ssd1306_send_init_sequence(void) {
    // 标准 SSD1306 128×64 初始化序列
    static const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,               // 0xAE
        SSD1306_CMD_SET_DISPLAY_CLK_DIV, 0x80, // 0xD5, 0x80 — 推荐分频
        SSD1306_CMD_SET_MULTIPLEX, 0x3F,       // 0xA8, 0x3F — 1/64 duty
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,  // 0xD3, 0x00 — 无偏移
        SSD1306_CMD_SET_START_LINE | 0x00,     // 0x40 — 起始行 0
        SSD1306_CMD_SET_CHARGE_PUMP, 0x14,     // 0x8D, 0x14 — 启用内部电荷泵（3.3V 供电必需）
        SSD1306_CMD_SEG_REMAP_INV,             // 0xA1 — 左右翻转（适配常见模组方向）
        SSD1306_CMD_COM_SCAN_DIR_INV,          // 0xC8 — 上下翻转
        SSD1306_CMD_SET_COM_PINS, 0x12,        // 0xDA, 0x12 — 128×64 交替 COM 引脚配置
        SSD1306_CMD_SET_CONTRAST, 0xCF,        // 0x81, 0xCF — 对比度
        SSD1306_CMD_SET_PRECHARGE, 0xF1,       // 0xD9, 0xF1 — 预充电周期
        SSD1306_CMD_SET_VCOM_DETECT, 0x40,     // 0xDB, 0x40 — VCOMH 电压
        SSD1306_CMD_DISPLAY_ALL_ON_RESUME,     // 0xA4 — 跟随 RAM 内容
        SSD1306_CMD_SET_NORMAL_DISPLAY,        // 0xA6 — 非反色
        SSD1306_CMD_SET_MEMORY_ADDR_MODE, 0x02,// 0x20, 0x02 — 页寻址模式（与 refresh 中的 0xB0 命令匹配）
        SSD1306_CMD_DISPLAY_ON,                // 0xAF — 开启显示
    };
    spi_send_cmd_list(init_cmds, sizeof(init_cmds));
}

// ── 帧缓冲 → 屏幕 ─────────────────────────────────────────

void drv_oled_refresh(void) {
    if (!s_spi_mutex) return;
    xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    for (int page = 0; page < OLED_PAGES; page++) {
        spi_send_cmd(SSD1306_CMD_SET_PAGE_ADDR | page);
        spi_send_cmd(SSD1306_CMD_SET_LOWER_COLUMN_ADDR | 0);
        spi_send_cmd(SSD1306_CMD_SET_HIGHER_COLUMN_ADDR | 0);
        drv_oled_spi_write(&s_framebuf[page * OLED_WIDTH], OLED_WIDTH, true);
    }
    xSemaphoreGive(s_spi_mutex);
}

void drv_oled_clear(void) {
    memset(s_framebuf, 0x00, sizeof(s_framebuf));
    drv_oled_refresh();
}

// ── 基础绘图 API ────────────────────────────────────────────

void drv_oled_draw_pixel(int x, int y, bool color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    int page = y / 8;
    int bit  = y % 8;
    if (color) {
        s_framebuf[page * OLED_WIDTH + x] |=  (1 << bit);
    } else {
        s_framebuf[page * OLED_WIDTH + x] &= ~(1 << bit);
    }
}

void drv_oled_fill(bool color) {
    memset(s_framebuf, color ? 0xFF : 0x00, sizeof(s_framebuf));
}

// ── 5×7 ASCII 字库（可缩放）─────────────────────────────────

static const uint8_t s_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' ' (space)
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x08,0x2A,0x1C,0x2A,0x08}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x00,0x08,0x14,0x22,0x41}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x41,0x22,0x14,0x08,0x00}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x01,0x01}, // 'F'
    {0x3E,0x41,0x41,0x51,0x32}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x00,0x7F,0x41,0x41}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x41,0x41,0x7F,0x00,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x08,0x14,0x54,0x54,0x3C}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x00,0x7F,0x10,0x28,0x44}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x08,0x08,0x2A,0x1C,0x08}, // '→'
};

static int char_to_index(char c) {
    if (c >= ' ' && c <= '~') return c - ' ';
    return 0; // 默认空格
}

void drv_oled_draw_char(int x, int y, char c, bool color, int scale) {
    int idx = char_to_index(c);
    for (int col = 0; col < 5; col++) {
        uint8_t line = s_font5x7[idx][col];
        for (int row = 0; row < 7; row++) {
            if (line & (1 << row)) {
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        drv_oled_draw_pixel(x + col * scale + dx,
                                            y + row * scale + dy, color);
                    }
                }
            }
        }
    }
}

void drv_oled_draw_string(int x, int y, const char *str, bool color, int scale) {
    while (*str) {
        drv_oled_draw_char(x, y, *str, color, scale);
        x += (5 + 1) * scale; // 5px 字宽 + 1px 间距
        if (x >= OLED_WIDTH) break;
        str++;
    }
}

// ── Bresenham 画线 ──────────────────────────────────────────

void drv_oled_draw_line(int x0, int y0, int x1, int y1, bool color) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        drv_oled_draw_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void drv_oled_draw_rect(int x, int y, int w, int h, bool color) {
    drv_oled_draw_line(x, y, x + w - 1, y, color);
    drv_oled_draw_line(x, y + h - 1, x + w - 1, y + h - 1, color);
    drv_oled_draw_line(x, y, x, y + h - 1, color);
    drv_oled_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);
}

void drv_oled_fill_rect(int x, int y, int w, int h, bool color) {
    for (int i = x; i < x + w; i++) {
        for (int j = y; j < y + h; j++) {
            drv_oled_draw_pixel(i, j, color);
        }
    }
}

// ── Midpoint 画圆 ──────────────────────────────────────────

void drv_oled_draw_circle(int cx, int cy, int r, bool color) {
    int x = r, y = 0, d = 1 - r;
    while (x >= y) {
        drv_oled_draw_pixel(cx + x, cy + y, color);
        drv_oled_draw_pixel(cx - x, cy + y, color);
        drv_oled_draw_pixel(cx + x, cy - y, color);
        drv_oled_draw_pixel(cx - x, cy - y, color);
        drv_oled_draw_pixel(cx + y, cy + x, color);
        drv_oled_draw_pixel(cx - y, cy + x, color);
        drv_oled_draw_pixel(cx + y, cy - x, color);
        drv_oled_draw_pixel(cx - y, cy - x, color);
        y++;
        if (d <= 0) {
            d += 2 * y + 1;
        } else {
            x--;
            d += 2 * (y - x) + 1;
        }
    }
}

// ── 业务层渲染 ──────────────────────────────────────────────

// 危险等级 → 边框颜色（单色屏用 fill/空心区分）
static const char *s_level_text[] = {"SAFE", "WARN", "DANGER", "CRIT"};
static const char *s_class_text[] = {"HORN", "SIREN", "BG"};

// 箭头绘制：以 (cx, cy) 为圆心画一个指向前方角度的箭头
static void draw_direction_arrow(int cx, int cy, int len, float angle_deg) {
    float rad = angle_deg * (float)M_PI / 180.0f;

    // 箭头尖端
    int tip_x = cx + (int)(len * sinf(rad));
    int tip_y = cy - (int)(len * cosf(rad));

    // 箭头两翼（夹角 ±30°）
    float wing_rad = 30.0f * (float)M_PI / 180.0f;
    int wing_len = len * 2 / 3;

    int w1_x = cx + (int)(wing_len * sinf(rad + wing_rad));
    int w1_y = cy - (int)(wing_len * cosf(rad + wing_rad));
    int w2_x = cx + (int)(wing_len * sinf(rad - wing_rad));
    int w2_y = cy - (int)(wing_len * cosf(rad - wing_rad));

    // 画箭头主体
    drv_oled_draw_line(cx, cy, tip_x, tip_y, true);
    drv_oled_draw_line(tip_x, tip_y, w1_x, w1_y, true);
    drv_oled_draw_line(tip_x, tip_y, w2_x, w2_y, true);
}

void drv_oled_draw_status(int level, int pred_class, float confidence, float doa_rel_deg) {
    // 上半屏：状态区（32 行高）
    // 清除上半屏
    drv_oled_fill_rect(0, 0, OLED_WIDTH, 32, false);

    // 第1行：危险等级标签
    drv_oled_draw_string(0, 0, s_level_text[level], true, 1);

    // 第1行右侧：置信度百分比
    char buf[24];
    snprintf(buf, sizeof(buf), "%3.0f%%", confidence * 100.0f);
    drv_oled_draw_string(OLED_WIDTH - 4 * 6, 0, buf, true, 1);

    // 第2行：声源类别
    if (pred_class >= 0 && pred_class <= 2) {
        drv_oled_draw_string(0, 8, s_class_text[pred_class], true, 1);
    }

    // 第3行：相对角度（clamp 到 ±999 防止 snprintf 溢出）
    float clamped_doa = doa_rel_deg;
    if (clamped_doa > 999.0f) clamped_doa = 999.0f;
    if (clamped_doa < -999.0f) clamped_doa = -999.0f;
    snprintf(buf, sizeof(buf), "DOA %+4.0f", clamped_doa);
    drv_oled_draw_string(0, 16, buf, true, 1);

    // 危险等级指示条（顶部细线：绿/黄/红 → 空心/实心小矩形）
    int bar_w = OLED_WIDTH / 4;
    for (int i = 0; i <= level; i++) {
        drv_oled_fill_rect(i * bar_w, 26, bar_w - 2, 4, true);
    }

    drv_oled_refresh();

    ESP_LOGD(TAG, "[STATUS] lvl=%d cls=%d conf=%.2f rel=%.1f",
             level, pred_class, confidence, doa_rel_deg);
}

void drv_oled_draw_arrow(float beta_deg) {
    // 下半屏：箭头区（32 行高，y=32~63）
    drv_oled_fill_rect(0, 32, OLED_WIDTH, 32, false);

    int cx = OLED_WIDTH / 2;
    int cy = 48; // 下半屏中心
    int arrow_len = 20;

    // 画十字参考线（淡）
    drv_oled_draw_line(cx, 33, cx, 63, false); // 竖线不画，保持干净
    drv_oled_draw_line(0, cy, OLED_WIDTH - 1, cy, false);

    // 画方向箭头（beta: 0=正前方, 正=右, 负=左）
    draw_direction_arrow(cx, cy, arrow_len, beta_deg);

    // 中心小圆点
    drv_oled_draw_circle(cx, cy, 2, true);

    drv_oled_refresh();

    ESP_LOGD(TAG, "[ARROW ] beta=%.1f deg", beta_deg);
}

// ── 驱动初始化 ──────────────────────────────────────────────

esp_err_t drv_oled_init(void) {
    // 创建 SPI 互斥锁（必须在任务启动前）
    s_spi_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "OLED GPIO pin map: CLK=%d MOSI=%d CS=%d DC=%d RST=%d",
             OLED_PIN_CLK, OLED_PIN_MOSI, OLED_PIN_CS, OLED_PIN_DC, OLED_PIN_RST);

    // ── GPIO 初始化 ──
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OLED_PIN_DC) |
                        (1ULL << OLED_PIN_RST) |
                        (1ULL << OLED_PIN_CS),
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 先取消片选，避免 OLED 在复位前误收 SPI 噪声。
    gpio_set_level(OLED_PIN_CS, 1);
    gpio_set_level(OLED_PIN_DC, 1);
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 验证 GPIO 可写可读。GPIO_MODE_OUTPUT 会关闭输入通路，readback 需用 INPUT_OUTPUT。
    int cs_read  = gpio_get_level(OLED_PIN_CS);
    int dc_read  = gpio_get_level(OLED_PIN_DC);
    int rst_read = gpio_get_level(OLED_PIN_RST);
    ESP_LOGI(TAG, "GPIO readback: CS=%d DC=%d RST=%d (expect all 1)",
             cs_read, dc_read, rst_read);
    if (cs_read != 1 || dc_read != 1 || rst_read != 1) {
        ESP_LOGE(TAG, "GPIO readback mismatch! Check OLED CS/DC/RST wiring or pin choice.");
        return ESP_ERR_INVALID_STATE;
    }

    // ── 硬件复位（拉低 ≥10ms，拉高后等 ≥10ms）──
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms，给足时间
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Hardware reset pulse done (50ms low + 50ms high)");

    // ── SPI 总线初始化 ──
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = OLED_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
    };
    esp_err_t ret = spi_bus_initialize(OLED_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SPI bus initialized on SPI%d", OLED_SPI_HOST);

    // ── SPI 设备添加：调屏阶段保持低速 1MHz ──
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = OLED_PIN_CS,
        .queue_size = 7,
        .pre_cb = oled_spi_pre_cb,
    };
    ret = spi_bus_add_device(OLED_SPI_HOST, &dev_cfg, &s_oled_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ── 发送一个诊断 SPI 事务，检查是否有 bus error ──
    {
        uint8_t diag_cmd = 0xAF;  // DISPLAY_ON
        spi_transaction_t t = {
            .length = 8,
            .tx_buffer = &diag_cmd,
            .user = (void *)(intptr_t)0,
        };
        ret = spi_device_transmit(s_oled_spi, &t);
        ESP_LOGI(TAG, "Diagnostic SPI transmit (0xAF): %s",
                 ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    }

    // ── SSD1306 初始化命令序列 ──
    spi_bus_remove_device(s_oled_spi);
    s_oled_spi = NULL;

    dev_cfg.clock_speed_hz = 1 * 1000 * 1000;
    ret = spi_bus_add_device(OLED_SPI_HOST, &dev_cfg, &s_oled_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device (1MHz) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ssd1306_send_init_sequence();

    // ── 诊断：发送全白帧（如果屏幕 OK，应该全亮）──
    ESP_LOGI(TAG, "Sending ALL-ON test pattern...");
    memset(s_framebuf, 0xFF, sizeof(s_framebuf));
    drv_oled_refresh();
    ESP_LOGI(TAG, "ALL-ON pattern sent. Check if screen is fully lit now.");

    // 等待 2 秒让用户观察
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 清屏
    memset(s_framebuf, 0x00, sizeof(s_framebuf));
    drv_oled_refresh();

    // 开机 Logo：显示项目名
    drv_oled_fill(false);
    drv_oled_draw_string(16, 16, "AudioSense", true, 2);
    drv_oled_draw_string(28, 40, "Smart Glasses", true, 1);
    drv_oled_refresh();

    ESP_LOGI(TAG, "OLED init OK (SSD1306 128x64, SPI @ 1MHz)");
    return ESP_OK;
}
