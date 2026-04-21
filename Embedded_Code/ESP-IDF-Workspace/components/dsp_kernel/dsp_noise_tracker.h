#pragma once
#include "dsp_config.h"

#define NOISE_WINDOW_LEN  16  // 中位数滤波窗口

typedef struct {
    float   spl_history[NOISE_WINDOW_LEN];
    int     count;
    float   noise_floor_db;
} DSPNoiseTracker_t;

void  dsp_noise_tracker_init(DSPNoiseTracker_t *inst);
void  dsp_noise_tracker_update(DSPNoiseTracker_t *inst);
float dsp_noise_tracker_get_floor(const DSPNoiseTracker_t *inst);
