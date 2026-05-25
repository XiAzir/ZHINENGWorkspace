# AGENTS.md

ESP32-P4 独立 PDM 麦克风验证工程。该工程只用于验证 PDM CLK/DAT/slot 是否能采到随声音变化的 PCM 数据，不包含 AI、屏幕、马达、IMU 或模型分区。

## 规则

- 编译、烧录、串口监视由用户亲自执行。
- Codex 可以修改代码、说明操作步骤、分析用户提供的日志。
- Codex 不得主动运行 `idf.py build`、`idf.py flash`、`idf.py monitor` 或等价命令。

## 关键文件

- `main/app_main.c`: PDM RX 初始化和 `MIC_PROBE` 日志。
- `README.md`: 用户操作步骤和判定标准。

## 默认硬件

- PDM CLK: GPIO4
- PDM DIN0: GPIO5
- DIN1 不使用
- 双槽诊断：同一 DIN0 同时采 `slot0=LEFT` 与 `slot1=RIGHT`
