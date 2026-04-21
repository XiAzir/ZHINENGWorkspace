#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void AI_Init(void);
void AI_Run(const int8_t *mel_feature, int *pred_class, float *confidence);

#ifdef __cplusplus
}
#endif
