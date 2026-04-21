#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "app_config.h"
#include "app_types.h"
#include "app_freertos.h"

#ifdef USE_SIMULATION
#include "drv_sim.h"
#else
#include "drv_pdm_dma.h"
#endif

static const char *TAG = "task_audio";

void TaskAudio_Run(void *arg) {
    ESP_LOGI(TAG, "Audio task started on core %d", xPortGetCoreID());

    AudioFrame_t frame;
    frame.timestamp_ms = 0;

    for (;;) {
#ifdef USE_SIMULATION
        drv_sim_read_frame(frame.pcm_ch0, frame.pcm_ch1, AUDIO_FRAME_LEN);
        frame.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        // 仿真下手动延迟以匹配 32ms 帧率（512 samples @ 16kHz = 32ms）
        vTaskDelay(pdMS_TO_TICKS(32));
#else
        size_t bytes_read = 0;
        // 从 DMA 读取双通道交织 PCM（左右声道各 AUDIO_FRAME_LEN 点）
        // TODO M1 上板验证：ESP32-P4 双 DIN + SLOT_MODE_STEREO 的 DMA payload 是否
        //   真为 L/R 交替 int16（而非 32-bit packed）。若格式不符，需调整拆分逻辑。
        int16_t interleaved[AUDIO_FRAME_LEN * 2];
        drv_pdm_dma_read(interleaved, AUDIO_FRAME_LEN * 2, &bytes_read, portMAX_DELAY);
        for (int i = 0; i < AUDIO_FRAME_LEN; i++) {
            frame.pcm_ch0[i] = interleaved[2 * i];
            frame.pcm_ch1[i] = interleaved[2 * i + 1];
        }
        frame.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
#endif

        if (xQueueSend(q_audio, &frame, 0) != pdTRUE) {
            // 队列满，丢弃当前帧（允许，DSP 任务处理不过来时正常现象）
        }
    }
}
