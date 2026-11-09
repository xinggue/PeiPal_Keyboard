# ESP32-S3 原生 USB-HID 键盘模拟器固件 (C/C++ Arduino 原生版 v10.0)

本目录为适用于 **ESP32-S3**（如 ESP32-S3 DevKitC-1、ESP32-S3 SuperMini、ESP32-S3 Zero 等）的原生固件。

与 ESP32-C3 方案最大的区别在于：**无需外接 CH9329 模拟键盘芯片**！ESP32-S3 自带高速 USB OTG 控制器，支持直接作为原生 USB-HID 键盘与电脑通信，一线兼顾供电、击键输出与 USB-CDC 调试日志。

---

## 硬件连接

| ESP32-S3 引脚 | 外设模块引脚 | 功能说明 |
|---|---|---|
| **板载 Type-C** | **电脑 USB 口** | 内部直连 GPIO 19 (D-) 与 GPIO 20 (D+)，同时作为 USB HID 模拟键盘与 USB-CDC 串口 |
| **GPIO 4 (RX)** | **扫码枪 TX** | 硬件串口 1 (HardwareSerial 1) 接收扫码枪条码数据（可选，引脚可在代码中自定义） |
| **GND** | **GND** | 扫码枪模块与 ESP32-S3 共地 |
| **5V 或 3.3V** | **VCC** | 统一供电 |

> **提示**：无需任何额外的 CH9329 芯片与接线。

---

## 烧录与编译方式

### 方式一：Arduino IDE (推荐)

1. 安装 ESP32 支持包（推荐版本 >= 2.0.14 或 3.x）：
   - 开发板管理器网址：`https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. 打开 `firmware_s3/firmware_s3.ino`
3. 开发板设置（菜单栏 **Tools / 工具**）：
   - **Board (开发板)**: `ESP32S3 Dev Module`
   - **USB Mode**: `USB-OTG (TinyUSB)` *(极其关键！必须选择此项启用原生 HID)*
   - **USB CDC On Boot**: `Enabled` *(启用后板载 USB 才能输出 Serial 调试日志)*
   - **Upload Mode**: `UART0 / Hardware CDC`
   - **Flash Size**: `4MB` (或根据板载 Flash 选用 8MB/16MB)
   - **Partition Scheme**: `Default 4MB with spiffs`
   - **Upload Speed**: `921600`
4. 插入 Type-C 数据线，选择对应端口，点击 **上传 (Upload)** 即可。

### 方式二：PlatformIO (VS Code)

在 `platformio.ini` 中添加配置：
```ini
[env:esp32s3_hid]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
build_flags = 
    -DARDUINO_USB_MODE=0
    -DARDUINO_USB_CDC_ON_BOOT=1
```

---

## 核心机制与特性

### 1. 原生 USB HID 与免外置芯片
- 依托 ESP32-S3 原生 USB OTG 控制器，以 12Mbps 全速直接向宿主机发送标准 8 字节 HID 键盘报告。
- 零延迟、不丢包、免去 UART-CH9329 的二次转换。

### 2. USB 主机 LED 状态实时监听与自愈
- 通过注册 TinyUSB 的 `ARDUINO_USB_HID_KEYBOARD_LED_EVENT` 事件，自动监听 Windows / Linux 主机下发的输出报告（SET_REPORT）。
- 实时获取 NumLock、CapsLock 状态。在输入首个有效字符前，自动按亮 NumLock、熄灭 CapsLock，确保中文 Alt 码穿透正常。

### 3. GBK 字库机制（内置 + 外置双模支持）
- **即刷即用**：工程自带 `gbk_data.h`，固件直接编译进 Flash 只读区。开箱即打中文，无需额外上传文件。
- **支持外部覆盖**：支持通过 LittleFS 上传 `gbk_data.bin` 到根目录 `/gbk_data.bin`，启动时优先使用外部文件，失败自动回退内置字库。
- **输入法穿透 (`alt_ascii`)**：所有中文字符与可打印 ASCII 字符通过 `Alt + NumPad 5 位数字` 序列直接发送，彻底绕过 Windows 搜狗、微软拼音等输入法候选弹窗。

### 4. Engine B 时序协议与全功能键/组合键支持
- **延时指令 `{DELAY:ms}`**：支持在打字流中嵌入任意毫秒级停顿，用于等待宿主机界面重绘与焦点就绪。
- **全功能键/组合键**：原生支持 `{F1}`~`{F12}`、`{INSERT}`、`{DEL}`、`{TAB}`、`{ENTER}`，以及 `{CTRL+C}`、`{ALT+F4}`、`{SHIFT+INSERT}` 等任意修饰组合键。
- **统一时序调参**：支持 `{CFG:key_press_ms=... key_gap_ms=...}` 统一时序指令，打字过程中即时调速生效。
- **掉电保存**：`{CFGSAVE}` 保存到 NVS，`{CFGRESET}` 恢复出厂设置。

### 5. 二维条码扫码引擎与分包重组 (`<<SEQ:ID:M/N>>`)
- **硬件 UART1**：GPIO 4 监听扫码枪输出，滤除末尾及内部多余 `\r` 与 `\n`。
- **纯时间静默断帧**：100ms 无新字节视为单次扫描结束。
- **去重防抖锁 (Deduplication Lock)**：1500ms 内相同内容防抖防抽搐，扫新码免等待立即触发。
- **分包重组**：支持 `<<SEQ:ID:M/N>>` 最大 32 包乱序拼齐打字，具有 30 秒超时与新会话自动重置保护机制。

---

## 协议与功能对齐列表

| 功能 | 协议/指令 | 说明 |
|---|---|---|
| **BLE 广播名** | `ESP32S3 键盘_XXXX` | 默认采用芯片 MAC 末 4 字节防配对混淆；支持 Nordic UART (`6E400001-...`) |
| **设备动态改名** | `{NAME:新名称}` / `{NAME?}` / `{NAMERESET}` | 支持运行时重命名并掉电持久化保存到 ESP32 NVS |
| **异步接收** | 线程安全缓冲 | FreeRTOS 异步后台收包，前台边收边打，长文本不丢字 |
| **急停安全** | `{STOP}` 或 `{ABORT}` | 即刻原子清空输入队列并释放物理按键，返回 `STOPPED` |
| **完成回执** | `{DONE}` 或 `{DONE:<id>}` | 打完后经 BLE TX 连发 2 次回执（如 `DONE:123`），抗丢包 |
| **延时等待** | `{DELAY:ms}` | 毫秒级停顿等待（如 `{DELAY:200}`） |
| **系统组合键** | `{CTRL+ALT+F4}` 等 | 自动识别 Ctrl/Alt/Shift/Win 修饰键并与主键组合发送 |
| **NumLock/Caps 状态** | `{NUM?}` | 查询宿主机下发的当前 LED 状态并回显 |
| **时序参数热调节** | `{CFG:key=val ...}` | 运行时内存即时生效，支持 Engine B 统一参数 |
| **时序掉电保存** | `{CFGSAVE}` | 保存当前时序至 NVS |
| **时序恢复默认** | `{CFGRESET}` | 恢复出厂默认时序 |
| **查询当前参数** | `{CFG?}` | 格式化回显当前时序参数 |
