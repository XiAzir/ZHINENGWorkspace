#include "dsp_common.h"
#include <stdbool.h>
#include "esp_log.h"
#include "dsps_fft2r.h"

static const char *TAG = "dsp_common";

float g_fft_w_table[2 * DSP_FFT_LEN];
static bool s_initialized = false;

esp_err_t dsp_fft_ensure_init(void) {
    if (s_initialized) return ESP_OK;
    esp_err_t ret = dsps_fft2r_init_fc32(g_fft_w_table, DSP_FFT_LEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FFT init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_initialized = true;
    return ESP_OK;
}
