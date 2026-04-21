#ifdef FEATURE_BLE

#include "drv_ble.h"
#include "esp_log.h"

static const char *TAG = "drv_ble";

// BLE 事件帧（UART/SPI 透传格式，详见协议设计文档）
typedef struct __attribute__((packed)) {
    uint8_t  header;        // 0xAA
    uint8_t  danger_level;  // DangerLevel_t (0=SAFE, 1=MEDIUM, 2=HIGH, 3=CRITICAL)
    uint8_t  pred_class;
    uint8_t  confidence;    // 0-100
    int8_t   doa_angle;     // -90 ~ +90 deg
    uint8_t  threat_level;  // ThreatLevel_t (0=GREEN, 1=YELLOW, 2=ORANGE, 3=RED)
    uint8_t  flags;         // bit0=rear_warning
    uint32_t timestamp_ms;
    uint8_t  checksum;      // XOR of bytes[1..9]
} BLEEventFrame_t;

esp_err_t drv_ble_init(void) {
    ESP_LOGI(TAG, "BLE driver init (stub — M5 implement via ESP-Hosted or UART)");
    return ESP_OK;
}

void drv_ble_send_event(const DecisionResult_t *d) {
    BLEEventFrame_t frame = {
        .header       = 0xAA,
        .danger_level = (uint8_t)d->level,
        .pred_class   = (uint8_t)d->pred_class,
        .confidence   = (uint8_t)(d->confidence * 100),
        .doa_angle    = (int8_t)d->doa_angle_deg,
        .threat_level = (uint8_t)d->threat_level,
        .flags        = d->rear_warning ? 0x01 : 0x00,
        .timestamp_ms = d->timestamp_ms,
    };
    uint8_t xor = 0;
    uint8_t *b = (uint8_t *)&frame;
    for (int i = 1; i < sizeof(BLEEventFrame_t) - 1; i++) xor ^= b[i];
    frame.checksum = xor;

    // TODO M5：通过 UART 或 ESP-Hosted 发送 frame
    ESP_LOGD(TAG, "BLE event: level=%d class=%d conf=%d angle=%d",
             frame.danger_level, frame.pred_class, frame.confidence, frame.doa_angle);
}

#endif // FEATURE_BLE
