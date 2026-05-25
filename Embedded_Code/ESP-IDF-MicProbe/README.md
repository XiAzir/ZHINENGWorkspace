# ESP32-P4 Mic Probe

这个工程只验证 PDM 麦克风输入，不包含 AI、屏幕、马达、IMU、队列和模型分区。

## 默认硬件假设

- ESP32-P4
- PDM CLK: GPIO4
- PDM DIN0: GPIO5
- DIN1 不使用
- 同一根 DIN0 同时采集 LEFT/RIGHT 两个 slot，用来判断 SEL 槽位和浮空噪声

如接线不同，改 `main/app_main.c` 顶部的 `MIC_CLK_GPIO` 和 `MIC_DIN0_GPIO`。

## 用户操作

Codex 不执行编译、烧录、监视。你在 ESP-IDF 终端里执行：

```powershell
cd E:\VSCODE-project\000QSWorkSpace\000QIANSAI\Embedded_Code\ESP-IDF-MicProbe
idf.py set-target esp32p4
idf.py build
idf.py -p COM9 flash monitor
```

端口按你实际设备修改。

## 测试方法

1. 上电后保持安静约 5 秒：前 2 秒作为 warmup，后 3 个 1 秒窗口用于记录 baseline；如果 baseline 期间出现削顶会跳过该窗口。
2. 手机贴近麦克风孔，最大音量播放 1 kHz tone、SIREN 或 CAR_HORN，持续 5 秒。
3. 先观察 `MIC_FAST` 里的 `delta` 和 `bands`，再看 `MIC_PROBE` 里的 `fast_peak*_delta`、`band_avg`、`band_peak`。

判定：

- 正常：某个 slot 的 `MIC_FAST` 或 `fast_peak*_delta` 在播放/拍手/说话瞬间相对 baseline 上升至少 6 dB，通常会更高。
- 异常：两个 slot 都几乎不变，或者长期一个 slot 全 0、另一个 slot 固定在高噪声。
- 说话有效：`voice` 或 `alert` 的 delta 应明显上升；如果只有吹气时 `low` 上升，说明主要是气流/风压响应，不代表正常声波被有效拾取。
- 工频/供电噪声：`hum` 长期偏高且随环境不变时，优先检查供电、地线、线长、屏蔽和 PDM 走线。

重点字段：

- `MIC_FAST`: 约 100 ms 短窗，更适合看拍手、喇叭、警笛这类瞬时声音。
- `fast_peak*_delta`: 最近 1 秒内最响短窗相对 baseline 的变化；判断远处声音是否被捕获时优先看它。
- `bands low=x/+d hum=x/+d voice=x/+d alert=x/+d`: 每个短窗的频段探针值和相对 baseline 的变化。`low` 用来看吹气/风压/低频振动；`hum` 看 50/60 Hz 工频及倍频；`voice` 看正常语音；`alert` 看喇叭、警笛一类更尖锐的告警声。
- `band_avg`: 最近 1 秒频段平均值；适合看持续噪声。
- `band_peak`: 最近 1 秒频段峰值；适合看短促声音。
- `slot*_dbfs`: 相对满量程 RMS，越接近 0 越大声。
- `slot*_delta`: 相对开机安静 baseline 的变化，验证声学响应主要看它。
- `slot*_zero`: 零样本比例；100% 基本表示该 slot 没数据。
- `slot*_clip`: 接近满量程的样本数；非 0 说明可能削顶。
- `read_err` / `short`: I2S 读取错误和短读计数，应保持 0。
- 当前探针使用 `amplify=1`。不建议靠数字放大解决灵敏度问题，因为它会把底噪一起放大，强输入时还会削顶。

频段含义：

- `low`: 约 20-200 Hz 的低频/次低频线索。吹气、风噪、手碰、结构振动、低频压力变化通常在这里最明显。
- `hum`: 50/60 Hz 及 100/120/150/180 Hz 附近。中国市电主要是 50 Hz，常见倍频是 100、150、200 Hz 等；60 Hz 及 120 Hz 用于兼容不同供电环境。
- `voice`: 约 300-3400 Hz。正常说话应该主要在这个频段出现可观察的 delta。
- `alert`: 约 700-4200 Hz。车喇叭、警笛、蜂鸣等危险提示音通常会在这里有更尖锐的峰值。

常见电气/电流底噪频率：

- DC 偏置和慢漂移：接近 0 Hz，通常落在 0-20 Hz。
- 市电耦合：国内通常是 50 Hz，以及 100/150/200 Hz 等倍频。
- 开关电源：本体常在几十 kHz 到数 MHz，但可能通过混叠、包络调制或布局耦合，表现成可听频段里的固定尖峰或宽带噪声。
- 数字时钟/PDM/I2S 耦合：时钟本身在 MHz 级，但如果布线、地回流或抽取滤波有问题，可能折叠成宽带底噪或固定杂散。

建议测试顺序：

1. 堵住麦克风孔，确认 `MIC_FAST` 和 `fast_peak*_delta` 是否仍长期偏高。
2. 手机播放固定音量白噪声或 1 kHz tone，分别放在 5 cm / 20 cm / 50 cm / 1 m。
3. 如果只有 5 cm 内才有明显正向 `delta`，优先检查麦克风孔位、外壳遮挡、供电噪声和 PDM 接线。
