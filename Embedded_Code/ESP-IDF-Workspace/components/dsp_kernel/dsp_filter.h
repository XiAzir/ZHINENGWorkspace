#pragma once
#include <stdbool.h>
#include "dsp_config.h"

typedef struct {
    float state[2 * DSP_IIR_NUM_STAGES];
    bool  initialized;
} DSPFilter_t;

void dsp_filter_hpf_init(DSPFilter_t *inst);
void dsp_filter_hpf_process(DSPFilter_t *inst, const float *input,
                             float *output, int len);
