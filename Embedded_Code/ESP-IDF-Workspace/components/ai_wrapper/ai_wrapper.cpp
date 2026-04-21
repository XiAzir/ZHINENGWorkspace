#include "ai_wrapper.h"
#include "esp_log.h"
#include "esp_partition.h"
#include <string.h>

// 从前端配置同步过来的常量超参（为了避免组件间的反向依赖而在 Mock 阶段手动显式声明）
#define MEL_BINS 64
#define MEL_FRAMES 32

// ── TFLite 官配基建 ──────────────────────────────────────────
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char *TAG = "ai_wrapper";

// 从编译期内联挂载下来的 CMake 二进制常量段，也就是你的 INT8 神仙模型！（清理了乱码路径）
extern const uint8_t g_model_tflite_start[] asm("_binary_audiosense_model_tflite_start");

// ── 永不回收的终生推演指针与张量空地 ───────────────────────
static const tflite::Model*           s_model       = nullptr;
static tflite::MicroInterpreter*      s_interpreter = nullptr;
static TfLiteTensor*                  s_input       = nullptr;
static TfLiteTensor*                  s_output      = nullptr;

// 分配给神经网络用来展开内存、摊放算子矩阵池的特供张量舞台。
// 因为加了 static，它被生生钉死在 120KB 的全局 BSS 静态空间里，绝不越栈。
constexpr int kTensorArenaSize = 120 * 1024;
static uint8_t s_tensor_arena[kTensorArenaSize] __attribute__((aligned(16)));

void AI_Init(void) {
    tflite::InitializeTarget(); // 唤醒 DSP / 加速向量运算支持（如果有的话）
    ESP_LOGI(TAG, "TFLite Target Initialized.");

    // 1. 无脑读取已固化在 Flash 只读区的 .tflite 原装网络架构
    s_model = tflite::GetModel(g_model_tflite_start);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "TFLite Model schema mismatch! Model is %lu, expects %d",
                 (unsigned long)s_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // 2. 紧急战术修正：新版 TFLite为了不浪费几百K的 ROM 空间，强行把以前那个“什么都能包”的 AllOpsResolver 砍掉了！
    // 没关系，咱们来手动指定。你的声学模型绝大多数是由这几个标准算子搭建的。
    static tflite::MicroMutableOpResolver<12> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddRelu();
    resolver.AddMaxPool2D();
    resolver.AddAdd();
    resolver.AddMean();  // 【刚才报错的那个要命的算子，也就是平均池化操作！】
    resolver.AddMul();
    resolver.AddSub();

    // 3. 构建核心解释器，喂入模型、算子和专属内存三要素
    static tflite::MicroInterpreter static_interpreter(
        s_model, resolver, s_tensor_arena, kTensorArenaSize);
    s_interpreter = &static_interpreter;

    // 4. 开始重锤部署舞台，生成计算图！
    TfLiteStatus allocate_status = s_interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed! 120KB Tensor Arena is not enough.");
        return;
    }

    // 5. 抓取模型嘴巴（输入）和屁股（输出）两个数据管道节点接头
    s_input = s_interpreter->input(0);
    s_output = s_interpreter->output(0);
    ESP_LOGI(TAG, "AI Engine Fully Assembled. Input_Dims=%d, Output_Dims=%d",
             s_input->dims->size, s_output->dims->size);
}

void AI_Run(const int8_t *mel_feature, int *pred_class, float *confidence) {
    if (!s_interpreter || !s_input) {
        *pred_class = 2; *confidence = 0.0f;
        return; // 防撞保护
    }

    // ========【端侧真计算神级通道】========
    // 1. 将 64×32 降噪提取出的梅尔切片，暴力灌装推入你的 INT8 TFLite 投料口！
    // 假设模型和输入一致大，全量拷贝；否则拷贝最少的容积。
    int req_bytes  = s_input->bytes; 
    int prov_bytes = MEL_BINS * MEL_FRAMES * sizeof(int8_t);
    int copy_bytes = (req_bytes < prov_bytes) ? req_bytes : prov_bytes;
    memcpy(s_input->data.int8, mel_feature, copy_bytes);

    // 2. 扣动神经突触的扳机，在微秒内轰鸣推演！
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke CPU infer failed!");
        *pred_class = 2; *confidence = 0.0f;
        return;
    }

    // 3. 收割模型大便尾端的数据（解出类目索引和 INT8 量化概率，再拉回浮点数）
    int8_t *out_data = s_output->data.int8;
    int max_idx = 0;
    int8_t max_val = out_data[0];
    
    // (假设输出只有 3 维特征池)
    for (int i = 1; i < 3; i++) {
        if (out_data[i] > max_val) {
            max_val = out_data[i];
            max_idx = i;
        }
    }
    
    // 逆向反量化公式得出真正的纯血推论信心百分比（Confidence）
    float scale = s_output->params.scale;
    int zero_pt = s_output->params.zero_point;
    
    *pred_class = max_idx;
    *confidence = scale * (float)(max_val - zero_pt);
}
