#pragma once

// ── 音频采集参数 ────────────────────────────────────────────
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_FRAME_LEN      512    // DMA 每次 DMA 中断的采样点数
#define NUM_DMA_BUFFERS      4      // DMA 描述符数量

// ── DSP 参数（与 Python 端严格对应）────────────────────────
#define DSP_FFT_LEN          1024
#define DSP_HOP_LEN          512
#define DSP_N_MELS           64
#define DSP_N_BINS           513    // FFT_LEN/2 + 1
#define DSP_IIR_CUTOFF_HZ    100
#define DSP_IIR_NUM_STAGES   2
#define DSP_MIC_SPACING_M    0.05f
#define DSP_SPEED_OF_SOUND   343.0f
#define DSP_INT8_SCALE       0.02773f
#define DSP_INT8_OFFSET      (-128)
#define DSP_SPL_CAL_DB       94.0f  // 校准常数（dBSPL，待硬件标定）

// ── Mel 特征帧参数 ──────────────────────────────────────────
#define MEL_FRAMES           32
#define MEL_BINS             64

// AI 推理触发周期（单位：audio 帧数）。
// 每 AI_INFER_HOP_FRAMES * 32ms 触发一次推理；取 3 对应 ~96ms，
// 配合 AI 推理 ~50ms → 端到端延迟 < 200ms。M1 上板后按实测耗时调优。
#define AI_INFER_HOP_FRAMES  3

// ── Doppler 跟踪参数 ────────────────────────────────────────
#define DSP_DOPPLER_F_LO     200.0f   // Hz
#define DSP_DOPPLER_F_HI     800.0f   // Hz
#define DSP_DOPPLER_HIST_LEN 8        // 历史帧数
#define DSP_SPL_RING_LEN     62       // ~2s @ 32ms/帧
#define DOA_PHAT_EPS         1e-6f

// ── RTOS 任务栈（单位：字节）────────────────────────────────
#define TASK_STACK_AUDIO     16384  // 原 2K，包含音频数据缓冲转移
#define TASK_STACK_DOA       16384  // 原 4K，包含波达特征组算核心
#define TASK_STACK_AI        65536  // 原 2K，修复 AI 神经层推演引发的爆栈 (64K满血保障)
#define TASK_STACK_DECISION  8192
#define TASK_STACK_GUI       8192
#define TASK_STACK_GUI_FAST  8192
#define TASK_STACK_HAPTIC    4096
#define TASK_STACK_BLE       8192
#define TASK_STACK_ADAPTIVE  4096
#define TASK_STACK_ORIENT    8192

// ── 3DoF / GUI 节拍 ────────────────────────────────────────
#define ORIENT_TICK_MS       5       // 200Hz IMU 读取 + Mahony 融合
#define GUI_FAST_TICK_MS     16      // ≈62.5Hz 箭头渲染

// ── 功能宏 ──────────────────────────────────────────────────
#define USE_SIMULATION       // 取消注释以使用仿真音频驱动
// #define FEATURE_BLE          // 取消注释以启用 BLE 任务
