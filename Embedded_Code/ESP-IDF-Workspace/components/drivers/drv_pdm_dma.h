#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// 初始化双路 PDM RX DMA 通道。
//   sample_rate  I2S PDM 采样率（Hz）
//   frame_len    每次 DMA 中断的采样点数（单通道）
//   num_dma_buf  DMA 描述符环数量
esp_err_t drv_pdm_dma_init(int sample_rate, int frame_len, int num_dma_buf);
esp_err_t drv_pdm_dma_read(int16_t *buf, size_t buf_len,
                            size_t *bytes_read, TickType_t timeout);
void      drv_pdm_dma_deinit(void);
