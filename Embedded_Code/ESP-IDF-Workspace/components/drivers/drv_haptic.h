#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t drv_haptic_init(void);
esp_err_t drv_haptic_write_reg(uint8_t reg, uint8_t val);
esp_err_t drv_haptic_read_reg(uint8_t reg, uint8_t *val);
esp_err_t drv_haptic_play(uint8_t waveform_idx);
