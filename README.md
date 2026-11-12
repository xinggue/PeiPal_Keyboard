<div align="center">

# PeiPal Keyboard

**把内容“打”进电脑的 ESP32 蓝牙键盘固件**

<br>

<a href="https://github.com/xinggue/PeiPal_Keyboard/releases"><img src="https://img.shields.io/github/v/release/xinggue/PeiPal_Keyboard?include_prereleases&color=76bad9" alt="release"></a>
<a href="https://github.com/xinggue/PeiPal_Keyboard/actions"><img src="https://img.shields.io/github/actions/workflow/status/xinggue/PeiPal_Keyboard/build_firmware.yml?color=76bad9" alt="build"></a>
<a href="https://github.com/xinggue/PeiPal_Keyboard/stargazers"><img src="https://img.shields.io/github/stars/xinggue/PeiPal_Keyboard?color=76bad9" alt="stars"></a>
<a href="./LICENSE"><img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="license"></a>

<br>
<br>

<a href="https://github.com/xinggue/PeiPal_Keyboard/releases">下载固件</a> ｜
<a href="#烧录">烧录</a> ｜
<a href="#指令速查">指令速查</a> ｜
<a href="#自行编译">自行编译</a> ｜
<a href="https://github.com/xinggue/PeiPal_Keyboard/issues">问题反馈</a>

</div>

<br>

[小培同学 PeiPal](https://github.com/xinggue/PeiPal_App) 的配套硬件。手机通过蓝牙把文本发给开发板，开发板以 USB 键盘 / 鼠标的身份向电脑逐字输入。目标电脑无需安装软件、无需联网，也不会出现 U 盘设备，内网电脑照常使用。

<br>

---

<br>

## 版本

| 目录 | 芯片 | 输出方式 |
|---|---|---|
| [`firmware_s3`](./firmware_s3) | ESP32-S3 | 芯片自带 USB，直接作为键盘 / 鼠标，无需额外模块 |
| [`firmware_c3`](./firmware_c3) | ESP32-C3 | 通过串口连接 CH9329 模块输出键盘 / 鼠标 |

推荐 **ESP32-S3**：只需一块开发板和一根 Type-C 线。

<br>

---

<br>

## 烧录

下载 [Releases](https://github.com/xinggue/PeiPal_Keyboard/releases)（国内可用 [Gitee](https://gitee.com/xinggue/PeiPal_Keyboard/releases)）中的 `PeiPal-Keyboard-ESP32S3.zip`，解压后使用其中的 **`firmware_s3.ino.merged.bin`**。

其余文件（`bootloader.bin`、`partitions.bin`、`.elf`、`.map`）烧录时用不到。

<br>

### 进入下载模式

1. 按住开发板上的 `BOOT` 键不放。
2. 插入 USB 线（或按一下 `RST`）。
3. 松开 `BOOT`。电脑上出现新的串口设备即可。

> 开发板有两个 Type-C 口时，使用标有 `USB` 或 `COM` 的那一个；烧录通过 `COM` 口最稳定。

<br>

### 方式一：乐鑫 Flash Download Tool（Windows，推荐）

1. 从[乐鑫官网](https://www.espressif.com.cn/zh-hans/support/download/other-tools)下载 **Flash Download Tools**。
2. 启动后 `ChipType` 选 **ESP32-S3**，`WorkMode` 选 **Develop**，`LoadMode` 选 **UART**。
3. 在文件列表第一行：
   - 选择 `firmware_s3.ino.merged.bin`
   - 地址填 **`0x0`**
   - 勾选该行左侧复选框
4. 下方参数：

   | 项 | 值 |
   |---|---|
   | SPI SPEED | 80MHz |
   | SPI MODE | DIO |
   | COM | 开发板对应的串口 |
   | BAUD | 921600（不稳定时降到 460800） |

5. 点击 **START**，进度条走完显示 `FINISH` 即完成。
6. 按一下 `RST` 或重新插拔 USB。

> 合并包已经包含引导程序与分区表，**只需要烧这一个文件，地址必须是 `0x0`**。不要再叠加烧 `bootloader.bin` / `partitions.bin`。

<br>

### 方式二：esptool 命令行（Windows / macOS / Linux）

```bash
pip install esptool

esptool.py --chip esp32s3 --baud 921600 write_flash 0x0 firmware_s3.ino.merged.bin
```

macOS / Linux 若找不到串口，可加 `--port /dev/cu.usbmodemXXXX`（macOS）或 `--port /dev/ttyACM0`（Linux）。

<br>

### 方式三：浏览器烧录

用 Chrome / Edge 打开 [Espressif esptool-js](https://espressif.github.io/esptool-js/)，点击 Connect 选择串口，Flash Address 填 `0x0`，选择 `firmware_s3.ino.merged.bin`，点击 Program。

<br>

### 烧录后检查

- 电脑出现一个新的 USB 键盘设备。
- 手机 App 搜索蓝牙，能看到 `ESP32S3 键盘_XXXX`（`XXXX` 为蓝牙 MAC 末 4 位，多台设备不会混淆）。
- 可用串口工具以 `115200` 连接板载 USB 查看启动日志。

<br>

### 常见问题

| 现象 | 处理 |
|---|---|
| 工具提示连接失败 / 找不到串口 | 重新按住 `BOOT` 进入下载模式；换一根能传数据的 USB 线 |
| 烧录成功但电脑没有键盘设备 | 确认使用的是板载 `USB` 口，而不是 `COM` 口 |
| 烧录成功但无法启动 | 确认文件地址是 `0x0`，且没有同时勾选其它文件 |
| 想恢复出厂 | 重新烧录一次 `merged.bin` 即可；蓝牙名称可发送 `{NAMERESET}` 恢复 |

<br>

---

<br>

## 硬件连接（S3）

| ESP32-S3 | 连接到 | 说明 |
|---|---|---|
| 板载 Type-C（USB） | 目标电脑 | 供电 + 模拟键盘 / 鼠标 |
| GPIO 4 (RX) | 扫码枪 TX | 可选，二维码输入，波特率 115200 |
| GND | 扫码枪 GND | 可选，共地 |

C3 版本的接线见 [`firmware_c3/README.md`](./firmware_c3/README.md)。

<br>

---

<br>

## 指令速查

蓝牙发送的文本中，普通字符直接输入，`{...}` 为指令。

| 类别 | 指令 |
|---|---|
| 功能键 | `{ENTER}` `{TAB}` `{ESC}` `{BS}` `{DEL}` `{SPACE}` `{UP}` `{DOWN}` `{LEFT}` `{RIGHT}` `{HOME}` `{END}` `{PGUP}` `{PGDN}` `{F1}`~`{F12}` |
| 组合键 | `{CTRL+C}` `{ALT+F4}` `{SHIFT+INSERT}` `{WIN+R}` 等 |
| 延时 | `{DELAY:200}`（毫秒） |
| 完成回执 | `{DONE:<id>}`，输入完成后经蓝牙回传 `DONE:<id>` |
| 急停 | `{STOP}` / `{ABORT}` |
| 键盘状态 | `{NUM?}` 查询 NumLock / CapsLock |
| 设备名称 | `{NAME:新名称}` `{NAME?}` `{NAMERESET}` |
| 时序参数 | `{CFG:key=val ...}` `{CFG?}` `{CFGSAVE}` `{CFGRESET}` |

<br>

### 鼠标

| 类别 | 指令 |
|---|---|
| 点击 | `{CLICK}` `{RCLICK}` `{MCLICK}` `{DBLCLICK}` |
| 按住 / 释放 | `{LDOWN}` `{LUP}` `{RDOWN}` `{RUP}` `{MDOWN}` `{MUP}` `{MOUSE_UP}` |
| 滚轮 | `{SCROLL:n}`（正数向上，负数向下） |
| 相对移动 | `{MOVE:dx,dy}` |
| 回到角落 | `{CORNER:TL}` `{CORNER:TR}` `{CORNER:BL}` `{CORNER:BR}`（`{RESET_MOUSE}` 等同于左上角） |
| 按坐标移动 / 点击 | `{MOVE_TO:x,y}` `{CLICK_AT:x,y}` `{RCLICK_AT:x,y}` `{DBLCLICK_AT:x,y}`（以左上角为原点） |
| 右上角基准点击 | `{CLICK_TR:dx,dy}` |

坐标类指令先把光标推到屏幕角落，再做相对移动。系统的“提高指针精确度”（鼠标加速）会让移动距离与像素不成正比，使用前请关闭，并在目标电脑上实测校准。

<br>

---

<br>

## 二维码输入

扫码枪接到开发板后，扫描 App 生成的二维码即可输入，无需蓝牙配对。内容较长时 App 会拆成多张码（`<<SEQ:ID:M/N>>`），开发板支持乱序扫描、自动拼齐后输入，最多 32 张，30 秒超时。

<br>

---

<br>

## 自行编译

### Arduino IDE

1. 安装 ESP32 开发板支持包（2.0.14 及以上或 3.x），附加开发板管理器网址：
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. 打开 `firmware_s3/firmware_s3.ino`。
3. 工具菜单设置：

   | 项 | 值 |
   |---|---|
   | Board | ESP32S3 Dev Module |
   | USB Mode | **USB-OTG (TinyUSB)** |
   | USB CDC On Boot | Enabled |
   | Flash Size | 4MB 或更大 |

4. 上传。

<br>

### arduino-cli

```bash
arduino-cli core install esp32:esp32

arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,USBMode=default \
  firmware_s3/firmware_s3.ino \
  --output-dir build/esp32s3
```

产物在 `build/esp32s3/`，其中 `firmware_s3.ino.merged.bin` 即可直接按上文烧录。

<br>

### 自动构建

推送到 `main` 会触发 GitHub Actions 编译 C3 / S3 并上传构建产物；推送 `v*` 标签会同时发布 Release。

<br>

---

<br>

## 开源协议

本项目基于 [Apache License 2.0](./LICENSE) 开源。
