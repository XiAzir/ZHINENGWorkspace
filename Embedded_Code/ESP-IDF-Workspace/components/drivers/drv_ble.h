#pragma once
#ifdef FEATURE_BLE

#include "esp_err.h"
#include "app_types.h"

esp_err_t drv_ble_init(void);
void      drv_ble_send_event(const DecisionResult_t *decision);

#endif // FEATURE_BLE
