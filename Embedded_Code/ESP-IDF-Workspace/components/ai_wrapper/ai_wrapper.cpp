#include "ai_wrapper.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <float.h>

// 从前端配置同步过来的常量超参
#define MEL_BINS   64
#define MEL_FRAMES 32
#define MEL_FEATURE_COUNT (MEL_BINS * MEL_FRAMES)
#define MEL_DEQUANT_SCALE 0.0078125f

// ── ESP-DL v3 官方头文件 ───────────────────────────────────────
#include "dl_model_base.hpp"

static const char *TAG = "ai_wrapper";

// ESP-DL 模型实例（生命周期与系统一致，永不释放）
static dl::Model *s_model = nullptr;

void AI_Init(void) {
    ESP_LOGI(TAG, "正在从 espdl_model 分区加载 ESP-DL 模型...");
    ESP_LOGI(TAG, "Heap before ESP-DL: internal free=%u largest=%u, psram free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

    // 从 Flash 分区加载 .espdl 量化模型
    // 参数 "espdl_model" 对应 partitions.csv 中的分区标签
    s_model = new dl::Model(
        "espdl_model",                          // 分区标签
        fbs::MODEL_LOCATION_IN_FLASH_PARTITION, // 从 Flash 分区加载
        0,                                      // max_internal_size=0，优先使用 PSRAM
        dl::MEMORY_MANAGER_GREEDY               // 贪心内存管理器
    );

    if (!s_model) {
        ESP_LOGE(TAG, "ESP-DL 模型创建失败！");
        return;
    }

    // 打印模型性能概况（内存占用 + 各层延迟）
    s_model->profile();

    ESP_LOGI(TAG, "ESP-DL 模型加载完成");
}

void AI_Run(const int8_t *mel_feature, int *pred_class, float *confidence, float *score_gap) {
    if (!s_model) {
        *pred_class = 2; // 安全失效：返回 background
        *confidence = 0.0f;
        if (score_gap) *score_gap = 0.0f;
        return;
    }

    // ========== 1. 将 Mel 特征灌入模型输入张量 ==========
    dl::TensorBase *input_tensor = s_model->get_input();
    if (!input_tensor) {
        ESP_LOGE(TAG, "获取输入张量失败");
        *pred_class = 2; *confidence = 0.0f;
        if (score_gap) *score_gap = 0.0f;
        return;
    }

    int tensor_bytes = input_tensor->get_bytes();
    const char *input_mode = "int8";
    if (tensor_bytes >= (int)(MEL_FEATURE_COUNT * sizeof(float))) {
        // audiosense.espdl 当前模型输入为 FLOAT[1,64,32,1]。
        // 上游为节省队列空间保存了 0..127 的 int8 归一化值，这里还原到训练端的 [0,1] float。
        float *input_data = input_tensor->get_element_ptr<float>();
        for (int i = 0; i < MEL_FEATURE_COUNT; i++) {
            input_data[i] = (float)mel_feature[i] * MEL_DEQUANT_SCALE;
        }
        input_mode = "float";
    } else {
        // 兼容后续若重新导出为 INT8 输入的模型。
        int8_t *input_data = input_tensor->get_element_ptr<int8_t>();
        int copy_bytes = MEL_FEATURE_COUNT * sizeof(int8_t);
        if (copy_bytes > tensor_bytes) {
            copy_bytes = tensor_bytes;
        }
        memcpy(input_data, mel_feature, copy_bytes);
    }

    // ========== 2. 执行推理并计时 ==========
    int64_t t0 = esp_timer_get_time();
    s_model->run();
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "推理耗时: %lld us (%.1f ms)", (t1 - t0), (t1 - t0) / 1000.0f);

    // ========== 3. 读取输出，执行 argmax ==========
    dl::TensorBase *output_tensor = s_model->get_output();
    if (!output_tensor) {
        ESP_LOGE(TAG, "获取输出张量失败");
        *pred_class = 2; *confidence = 0.0f;
        if (score_gap) *score_gap = 0.0f;
        return;
    }

    int num_classes = output_tensor->get_size();
    if (num_classes > 3) num_classes = 3; // 安全限制

    float probs[3] = {0.0f, 0.0f, 0.0f};
    int output_bytes = output_tensor->get_bytes();
    const char *output_mode = "int8";
    if (output_bytes >= num_classes * (int)sizeof(float)) {
        // audiosense.espdl 当前 Softmax 输出为 FLOAT[1,3]。
        float *out_data = output_tensor->get_element_ptr<float>();
        for (int i = 0; i < num_classes; i++) {
            probs[i] = out_data[i];
        }
        output_mode = "float";
    } else {
        int8_t *out_data = output_tensor->get_element_ptr<int8_t>();

        // ESP-DL INT8 fallback：real_value = int_value * 2^exponent
        int exp = output_tensor->get_exponent();
        float scale = 1.0f;
        if (exp < 0) {
            for (int i = 0; i < -exp; i++) scale *= 0.5f;
        } else {
            for (int i = 0; i < exp; i++) scale *= 2.0f;
        }
        for (int i = 0; i < num_classes; i++) {
            probs[i] = (float)out_data[i] * scale;
        }
    }

    static bool s_io_logged = false;
    if (!s_io_logged) {
        ESP_LOGI(TAG, "AI tensor IO: input=%s bytes=%d output=%s bytes=%d classes=%d",
                 input_mode, tensor_bytes, output_mode, output_bytes, num_classes);
        s_io_logged = true;
    }

    // argmax 找最大预测类别
    int max_idx = 0;
    float max_val = probs[0];
    float second_val = -FLT_MAX;
    for (int i = 1; i < num_classes; i++) {
        if (probs[i] > max_val) {
            second_val = max_val;
            max_val = probs[i];
            max_idx = i;
        } else if (probs[i] > second_val) {
            second_val = probs[i];
        }
    }

    *pred_class = max_idx;
    *confidence = max_val;
    if (score_gap) {
        *score_gap = (second_val > -FLT_MAX * 0.5f) ? (max_val - second_val) : 0.0f;
        if (*score_gap < 0.0f) *score_gap = 0.0f;
    }

    // Softmax 输出理论范围 [0,1]，INT8 量化误差可能导致越界，clamp 修正
    if (*confidence < 0.0f) *confidence = 0.0f;
    if (*confidence > 1.0f) *confidence = 1.0f;

    ESP_LOGD(TAG, "预测类别=%d, 置信度=%.3f, prob=[%.3f %.3f %.3f]",
             max_idx, *confidence, probs[0], probs[1], probs[2]);
}
