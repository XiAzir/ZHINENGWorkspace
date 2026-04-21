#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "app_types.h"

#define Q_AUDIO_DEPTH     4
#define Q_FEATURE_DEPTH   2
#define Q_AI_DEPTH        4
#define Q_DECISION_DEPTH  4

extern TaskHandle_t h_task_audio;
extern TaskHandle_t h_task_doa;
extern TaskHandle_t h_task_ai;
extern TaskHandle_t h_task_decision;
extern TaskHandle_t h_task_orient;
extern TaskHandle_t h_task_gui_fast;

void app_freertos_init(void);

// ── 跨任务共享方位量（__atomic float 位模式，同 ai_state_*）───
// 写入方：task_orientation        读取方：task_doa_feature, task_gui_fast
void  orientation_yaw_set(float yaw_deg);
float orientation_yaw_get(void);

// 写入方：task_gui (慢线程)       读取方：task_gui_fast
void  theta_world_set(float theta_deg);
float theta_world_get(void);
