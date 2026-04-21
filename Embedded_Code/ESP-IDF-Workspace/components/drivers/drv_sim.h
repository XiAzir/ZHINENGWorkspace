#pragma once
#ifdef USE_SIMULATION

#include <stdint.h>

void drv_sim_init(void);
void drv_sim_select_source(int idx);   // 0=car_horn, 1=siren, 2=background
void drv_sim_read_frame(int16_t *ch0, int16_t *ch1, int len);

#endif // USE_SIMULATION
