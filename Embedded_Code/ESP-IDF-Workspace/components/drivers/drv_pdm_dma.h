#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// 当前四麦硬件接法：
//   CLK  -> GPIO4（四个麦共享）
//   DATA0-> GPIO5：SELECT=LOW / SELECT=HIGH 各一颗麦
//   DATA1-> GPIO6：SELECT=LOW / SELECT=HIGH 各一颗麦
// 一根 PDM DATA 线最多挂两颗麦，必须用 SELECT 分到左右 slot，不能把多颗同 slot
// 的 DATA 输出硬并联。
#define AUDIO_PDM_PHYSICAL_LINES      2
#define AUDIO_PDM_SLOTS_PER_LINE      2

// ESP-IDF v5.4 PDM RX API 的 DMA 帧按 stereo 2 slots 暴露；主工程当前先采集
// 一条 DATA 线上的 LOW/HIGH 两颗麦。0=GPIO5，1=GPIO6。
#define AUDIO_PDM_ACTIVE_LINE         0
#define AUDIO_PDM_CAPTURE_CHANNELS    2

// 保留旧排查开关；主工程正常运行时关闭。
#define AUDIO_PDM_DIN0_DUAL_SLOT_PROBE  0

// 当前下游 AudioFrame_t 是两路，直接使用活动 DATA 线的 LOW/HIGH 两个 slot。
#define AUDIO_PDM_DOWNSTREAM_CH0    0
#define AUDIO_PDM_DOWNSTREAM_CH1    1

// PDM RX 数字放大。默认 1；调试阶段不额外放大，避免把错误槽位/浮空噪声伪装成强信号。
#define AUDIO_PDM_AMPLIFY_NUM      1

// 初始化 PDM RX DMA 通道。
//   sample_rate  I2S PDM 采样率（Hz）
//   frame_len    每次 DMA 中断的采样点数（单通道）
//   num_dma_buf  DMA 描述符环数量
esp_err_t drv_pdm_dma_init(int sample_rate, int frame_len, int num_dma_buf);
int       drv_pdm_dma_get_channels(void);
int       drv_pdm_dma_get_downstream_ch0(void);
int       drv_pdm_dma_get_downstream_ch1(void);
bool      drv_pdm_dma_is_dual_slot_probe(void);
esp_err_t drv_pdm_dma_read(int16_t *buf, size_t buf_len,
                            size_t *bytes_read, TickType_t timeout);
void      drv_pdm_dma_deinit(void);
