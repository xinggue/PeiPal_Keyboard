# ESP32-C3 键盘模拟器固件 (C/C++ Arduino 原生版 v10.0)

本目录为适用于 **Maker Go ESP32-C3 SuperMini**（及通用 ESP32-C3 开发板）的 C++ 原生固件，包含 BLE HID 模拟键盘与硬件 2D 条码扫码枪接收拼齐引擎，全面支持 Engine B 时序协议与控制指令。

---

## 硬件接线

| ESP32-C3 引脚 | 外设模块引脚 | 功能说明 |
|---|---|---|
| **GPIO 0 (TX)** | **CH9329 RX** | MCU 发送击键帧给 CH9329 模拟键盘芯片 |
| **GPIO 1 (RX)** | **CH9329 TX** | MCU 回读 CH9329 状态 (NumLock/CapsLock 闭环自愈) |
| **GPIO 3 (RX)** | **扫码枪 TX** | 硬件串口 0 (UART0) 高速捕获二维条码数据 (扫码枪 RX 悬空不接) |
| **GND** | **GND** | 所有模块必须共地 |
| **5V 或 3.3V** | **VCC** | 统一供电 |
| **USB-C** | **电脑端** | 板载 USB-CDC Serial (115200 波特率，启动与调试日志) |

> **注**：CH9329 与 扫码枪串口默认通信波特率均为 **115200**。

---

## 烧录与编译方式

### 方式一：Arduino IDE

1. 安装 ESP32 支持包（推荐版本 >= 2.0.14 或 3.x）：
   - 附加开发板管理器网址：`https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. 打开 `firmware_c3/firmware_c3.ino`
3. 开发板设置（菜单栏 Tools / 工具）：
   - **Board (开发板)**: `ESP32C3 Dev Module`
   - **USB CDC On Boot**: `Enabled` *(关键！启用后 USB-C 串口才能看到启动和调试日志)*
   - **Flash Size**: `4MB`
   - **Partition Scheme**: `Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)`
   - **Upload Speed**: `921600` 或 `460800`
4. 选择对应的串口端口，点击 **上传 (Upload)** 即可。

### 方式二：PlatformIO (VS Code)

如果使用 PlatformIO，在项目根目录配置 `platformio.ini`：
```ini
[env:esp32c3_supermini]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
build_flags = 
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
```

---

## 核心机制

### 1. GBK 字库机制（内置 + 外置双模支持）
- **即刷即用**：工程自带 `gbk_data.h`，固件直接编译进 Flash 只读区。开箱即可直接打中文，无需额外上传文件。
- **支持外部覆盖**：支持通过 LittleFS 上传 `gbk_data.bin` 到根目录 `/gbk_data.bin`，启动时优先使用外部文件，失败自动回退内置字库。
- **输入法穿透 (`alt_ascii`)**：所有中文字符与可打印 ASCII 字符通过 `Alt + NumPad 5 位数字` 序列直接发送，彻底绕过 Windows 搜狗、微软拼音等输入法候选弹窗。

### 2. 闭环状态检测与自愈
- 发送首个字符前，自动向 CH9329 发送 `CMD_GET_INFO (0x01)` 查询当前键盘指示灯状态。
- 若 NumLock 处于关闭状态，自动模拟 NumLock 击键开启；若 CapsLock 处于开启状态，自动模拟 CapsLock 击键关闭。

### 3. Engine B 时序协议与全功能键/组合键支持
- **延时指令 `{DELAY:ms}`**：支持在打字流中嵌入任意毫秒级停顿，用于等待宿主机界面重绘与焦点就绪。
- **全功能键/组合键**：原生支持 `{F1}`~`{F12}`、`{INSERT}`、`{DEL}`、`{TAB}`、`{ENTER}`，以及 `{CTRL+C}`、`{ALT+F4}`、`{SHIFT+INSERT}` 等任意修饰组合键。
- **统一时序调参**：支持 `{CFG:key_press_ms=... key_gap_ms=...}` 统一时序指令，打字过程中即时调速生效。

### 4. 二维条码扫码引擎与分包重组 (`<<SEQ:ID:M/N>>`)
- **硬件 UART0**：GPIO 3 监听扫码枪输出，滤除末尾及内部多余 `\r` 与 `\n`。
- **纯时间静默断帧**：100ms 无新字节视为单次扫描结束。
- **去重防抖锁 (Deduplication Lock)**：1500ms 内相同内容防抖防抽搐，扫新码免等待立即触发。
- **分包重组**：支持 `<<SEQ:ID:M/N>>` 最大 32 包乱序拼齐打字，具有 30 秒超时与新会话自动重置保护机制。

---

## 协议与功能对齐列表

| 功能 | 协议/指令 | 说明 |
|---|---|---|
| **BLE 广播名** | `ESP32C3 键盘_XXXX` | 默认采用芯片 MAC 末 4 字节防配对混淆；支持 Nordic UART (`6E400001-...`) |
| **设备动态改名** | `{NAME:新名称}` / `{NAME?}` / `{NAMERESET}` | 支持运行时重命名并掉电持久化保存到 ESP32 NVS |
| **异步接收** | 线程安全缓冲 | FreeRTOS 异步后台收包，前台边收边打，长文本不丢字 |
| **完成回执** | `{DONE}` 或 `{DONE:<id>}` | 打完后经 BLE TX 连发 2 次回执（如 `DONE:123`），抗丢包 |
| **延时等待** | `{DELAY:ms}` | 毫秒级停顿等待（如 `{DELAY:200}`） |
| **鼠标点击与控制** | `{CLICK}` / `{RCLICK}` / `{MCLICK}` / `{DBLCLICK}` / `{LDOWN}` / `{LUP}` / `{SCROLL:n}` / `{MOVE:dx,dy}` | 支持左键/右键/中键单击、双击、按住拖拽、滚轮滚动与大跨度平滑移动 |
| **鼠标角点归零与定位点击** | `{CORNER:TL}` / `{CORNER:TR}` / `{RESET_MOUSE}` / `{MOVE_TO:x,y}` / `{CLICK_AT:x,y}` / `{CLICK_TR:dx,dy}` | 支持连续推边撞墙归零至角落，基于左上角/右上角基准移动并执行点击 |
| **系统组合键** | `{CTRL+ALT+F4}` 等 | 自动识别 Ctrl/Alt/Shift/Win 修饰键并与主键组合发送 |
| **NumLock/Caps 状态** | `{NUM?}` | 手动查询 CH9329 实际 LED 状态并回显 |
| **时序参数热调节** | `{CFG:key=val ...}` | 运行时内存即时生效，支持 Engine B 统一参数 |
| **时序掉电保存** | `{CFGSAVE}` | 保存当前时序至 NVS |
| **时序恢复默认** | `{CFGRESET}` | 恢复出厂默认时序 |
| **查询当前参数** | `{CFG?}` | 格式化回显当前时序参数 |
