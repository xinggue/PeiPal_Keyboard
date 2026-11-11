/**
 * ==============================================================================
 * ESP32-C3 蓝牙键盘模拟器 + 扫码模块扩展版 v10.0 (C/C++ Arduino 原生版)
 *
 * 适用于：Maker Go ESP32-C3 SuperMini / Pro Mini 等开发板
 * 硬件连接：
 *   - CH9329 键盘输出：
 *       ESP32-C3 GPIO 0 (TX) -> CH9329 RX
 *       ESP32-C3 GPIO 1 (RX) -> CH9329 TX (用于回读 NumLock/Caps/Scroll 状态)
 *   - 扫码模块：
 *       ESP32-C3 GPIO 3 (RX) -> 扫码枪 TX
 *       扫码枪 RX 悬空不接 (TX_PIN = -1)
 *   - 电脑调试 / 烧录：
 *       板载 Type-C (内置 USB-CDC Serial，波特率 115200)
 *   - 共地 GND，统一供电 5V / 3V3
 *
 * 功能特性：
 *   1. 异步 BLE 接收 + 边打边收无阻塞流控，长文本 / 汉字不丢字。
 *   2. Unicode -> GBK 快速映射，Alt+小键盘 5 位数字打汉字，彻底绕过中文输入法。
 *   3. 优先读取 LittleFS /gbk_data.bin；若未上传则自动降级使用内置 GBK_DATA_BUILTIN，即刷即用。
 *   4. 时序参数热调节 ({CFG:...})、NVS 掉电保存 ({CFGSAVE})、回显 ({CFG?})，对齐 Engine B 统一时序。
 *   5. 支持 {DELAY:ms} 毫秒级停顿标签，完美适配 HIS 界面焦点切换与响应等待。
 *   6. 支持鼠标点击与动作执行 ({CLICK}, {RCLICK}, {MCLICK}, {DBLCLICK}, {LDOWN}, {LUP}, {SCROLL:n}, {MOVE:x,y})。
 *   7. 支持任意单键与组合键自动解析执行 (如 {TAB}, {ENTER}, {F1}-{F12}, {CTRL+C}, {ALT+F4}, {SHIFT+INSERT})。
 *   7. NumLock/CapsLock 闭环自愈：打首个汉字前自动查询 CH9329 CMD_GET_INFO 并自动修复。
 *   8. 消息完成回执：{DONE} 或 {DONE:<id>} 打完后连续回执 2 次抗丢包。
 *   9. 扫码扩展：
 *       - 使用硬件 UART0 高速捕获扫码数据。
 *       - 过滤扫码枪多余的尾部/内部 '\r' 和 '\n'。
 *       - 纯时间静默断帧 (100ms 无新字节视为一帧完成)。
 *       - 防重扫去重锁 (Deduplication Lock)：同一内容 1.5 秒内防抽搐连扫；换码免等待。
 *       - 支持分包协议 <<SEQ:ID:M/N>> 乱序重组与拼齐触发。
 *       - 新 SEQ ID 或非 SEQ 码到达，立即清空重置上一轮未完成的分包会话。
 *       - 与 BLE 输入原子排队合并，天然支持 {TAB}、{ENTER}、{DELAY:ms} 等功能按键。
 *       - 与 BLE 输入原子排队合并，天然支持 {TAB}、{ENTER} 等功能按键。
 * ==============================================================================
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_mac.h>
#include "gbk_data.h"

// ==============================================================================
// 基础结构体前置声明
// ==============================================================================
struct KeyMap {
    uint8_t key;
    bool shift;
};

// 延时参数结构体 (Engine B 统一时序)
struct TimingConfig {
    float tap_press_ms;
    float tap_release_ms;
    float shift_press_ms;
    float shift_release_ms;
    float alt_hold_ms;
    float alt_digit_press;
    float alt_digit_release;
    float alt_release_ms;
    float enter_press_ms;
    float enter_release_ms;
    float tab_press_ms;
    float tab_release_ms;
    float ascii_gap_ms;
    float cjk_gap_ms;
    float numlock_auto;
    float alt_ascii; // 1.0 = 英文/数字/符号全走 Alt 码(彻底绕过输入法)；0.0 = 常规 HID 单键
    float fence_auto; // 1.0 = 开启长文本防粘滞硬重置释放栅栏
    float shake_auto; // 1.0 = 开启闭环翻转握手自检 (老旧死锁环境可选)
    float check_interval; // 自检触发周期字符数 (默认 15.0)
};

static void clamp_timing(TimingConfig &t);

// ==============================================================================
// 引脚与硬件配置 (ESP32-C3)
// ==============================================================================
#define CH9329_RX_PIN   1    // ESP32-C3 GPIO 1 (RX)，接 CH9329 TX
#define CH9329_TX_PIN   0    // ESP32-C3 GPIO 0 (TX)，接 CH9329 RX
#define CH9329_BAUDRATE 115200

#define QR_RX_PIN       3    // ESP32-C3 GPIO 3 (RX)，接扫码枪 TX
#define QR_TX_PIN       -1   // 扫码枪不接收，设为 -1
#define QR_BAUDRATE     115200

// 扫码参数配置
#define QR_SILENCE_TIMEOUT_MS  100   // 帧间静默超时：100ms 无新字节视为一帧完成
#define QR_DEDUP_INTERVAL_MS   1500  // 相同条码防重扫冷却时间 (ms)
#define QR_SESSION_TIMEOUT_MS  30000 // 未集齐分包超时时间 (30秒)
#define MAX_QR_PARTS           32    // 支持最大分包数

// BLE 默认设备前缀与动态名称
#define BLE_DEFAULT_DEVICE_PREFIX "ESP32C3 键盘"
static String current_ble_name = "";

// 生成基于芯片全球唯一 MAC 地址末 4 位的出厂名称，如 "ESP32C3 键盘_A8F2"
static String get_default_ble_name() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s_%02X%02X", BLE_DEFAULT_DEVICE_PREFIX, mac[4], mac[5]);
    return String(buf);
}

#define SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_CHAR_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // App 写入
#define TX_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // 固件通知 App

// 扫码枪硬件串口定义 (ESP32-C3 使用 HardwareSerial 0，引脚重映射到 GPIO 3)
HardwareSerial SerialQR(0);

// ==============================================================================
// HID 修饰键与键码定义
// ==============================================================================
#define MOD_NONE    0x00
#define MOD_LCTRL   0x01
#define MOD_LSHIFT  0x02
#define MOD_LALT    0x04
#define MOD_LWIN    0x08
#define MOD_RCTRL   0x10
#define MOD_RSHIFT  0x20
#define MOD_RALT    0x40
#define MOD_RWIN    0x80

#define MOD_CTRL    MOD_LCTRL
#define MOD_SHIFT   MOD_LSHIFT
#define MOD_ALT     MOD_LALT
#define MOD_WIN     MOD_LWIN

// 小键盘数字 0-9 键码 (用于 Alt Code 打中文)
static const uint8_t KEYPAD_NUMS[10] = {
    0x62, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61
};

// ==============================================================================
// 延时参数默认值 (与 App「适中」方案严格对齐：核心 5 项全 3.0ms，普通/功能键 20.0ms)
// ==============================================================================
static const TimingConfig DEFAULT_TIMING = {
    20.0f,  // tap_press_ms (功能按键按下)
    20.0f,  // tap_release_ms (功能按键后停顿)
    20.0f,  // shift_press_ms
    20.0f,  // shift_release_ms
    3.0f,   // alt_hold_ms (Alt 前置按住)
    3.0f,   // alt_digit_press (小键盘数字按下)
    3.0f,   // alt_digit_release (数字之间间隔)
    3.0f,   // alt_release_ms (Alt 尾持时长)
    20.0f,  // enter_press_ms
    20.0f,  // enter_release_ms
    20.0f,  // tab_press_ms
    20.0f,  // tab_release_ms
    20.0f,  // ascii_gap_ms (普通字符间隔)
    3.0f,   // cjk_gap_ms (汉字合成间隔)
    1.0f,   // numlock_auto (1.0 = 开)
    1.0f,   // alt_ascii (1.0 = 默认全走 Alt 码彻底防中文输入法)
    1.0f,   // fence_auto (默认开启长文本防粘滞硬重置)
    0.0f,   // shake_auto (默认关闭翻转握手)
    15.0f   // check_interval (默认每 15 个物理动作自检一次)
};

static TimingConfig timing = DEFAULT_TIMING;
static Preferences prefs;

// ==============================================================================
// 动态/内置 GBK 字库管理
// ==============================================================================
static const uint8_t *gbk_data_ptr = nullptr;
static size_t gbk_data_len = 0;
static uint8_t *gbk_fs_buf = nullptr;

static void init_gbk_data() {
    // 允许在首次挂载失败时自动格式化 LittleFS (formatOnFail = true)
    if (LittleFS.begin(true)) {
        if (LittleFS.exists("/gbk_data.bin")) {
            File f = LittleFS.open("/gbk_data.bin", "r");
            if (f) {
                size_t sz = f.size();
                gbk_fs_buf = (uint8_t*)malloc(sz);
                if (gbk_fs_buf && f.read(gbk_fs_buf, sz) == sz) {
                    gbk_data_ptr = gbk_fs_buf;
                    gbk_data_len = sz;
                    Serial.printf("GBK 数据从 LittleFS 加载成功: %u 字节\n", (unsigned)sz);
                    f.close();
                    return;
                }
                f.close();
                if (gbk_fs_buf) { free(gbk_fs_buf); gbk_fs_buf = nullptr; }
            }
        }
    }
    // 降级使用内置 flash 数组
    gbk_data_ptr = GBK_DATA_BUILTIN;
    gbk_data_len = GBK_DATA_BUILTIN_LEN;
    Serial.printf("GBK 数据使用内置 flash 数组: %u 字节\n", (unsigned)gbk_data_len);
}

// 常见医疗与科技符号直接映射表 (U+00A0..02FF, U+2000..26FF -> GBK 编码)
struct SpecialGbkEntry {
    uint32_t unicode;
    uint16_t gbk;
};

static const SpecialGbkEntry SPECIAL_GBK_MAP[] PROGMEM = {
    // 1. 常用拉丁扩展与度量 (U+00A0..02FF)
    { 0x00B0, 0xA1E3 }, // ° 度 (Alt+41443)
    { 0x00B1, 0xA1C0 }, // ± 正负号 (Alt+41408)
    { 0x00B2, 0xA986 }, // ² 平方 (Alt+43398)
    { 0x00B3, 0xA987 }, // ³ 立方 (Alt+43399)
    { 0x00B7, 0xA1A4 }, // · 间隔号 (Alt+41380)
    { 0x00D7, 0xA1C1 }, // × 乘号 (Alt+41409)
    { 0x00F7, 0xA1C2 }, // ÷ 除号 (Alt+41410)
    { 0x00A7, 0xA1EC }, // § 章节号
    { 0x00A4, 0xA1E8 }, // ¤ 货币符号

    // 2. 常用符号与单位 (U+2000..21FF)
    { 0x2014, 0xA1AA }, // — 破折号
    { 0x2026, 0xA1AD }, // … 省略号
    { 0x2030, 0xA1EB }, // ‰ 千分号 (Alt+41451)
    { 0x2103, 0xA1E6 }, // ℃ 摄氏度 (Alt+41446)
    { 0x2116, 0xA1ED }, // № 编号 (Alt+41453)

    // 3. 罗马数字 I ~ XII (U+2160..216B)
    { 0x2160, 0xA2F1 }, // Ⅰ (Alt+41713)
    { 0x2161, 0xA2F2 }, // Ⅱ (Alt+41714)
    { 0x2162, 0xA2F3 }, // Ⅲ (Alt+41715)
    { 0x2163, 0xA2F4 }, // Ⅳ (Alt+41716)
    { 0x2164, 0xA2F5 }, // Ⅴ (Alt+41717)
    { 0x2165, 0xA2F6 }, // Ⅵ (Alt+41718)
    { 0x2166, 0xA2F7 }, // Ⅶ (Alt+41719)
    { 0x2167, 0xA2F8 }, // Ⅷ (Alt+41720)
    { 0x2168, 0xA2F9 }, // Ⅸ (Alt+41721)
    { 0x2169, 0xA2FA }, // Ⅹ (Alt+41722)
    { 0x216A, 0xA2FB }, // Ⅺ (Alt+41723)
    { 0x216B, 0xA2FC }, // Ⅻ (Alt+41724)

    // 4. 数学与逻辑运算符 (U+2200..22FF)
    { 0x221A, 0xA1CC }, // √ 对号/根号
    { 0x2234, 0xA1D0 }, // ∴ 所以
    { 0x2235, 0xA1D1 }, // ∵ 因为
    { 0x2248, 0xA1E0 }, // ≈ 约等于
    { 0x2260, 0xA1D9 }, // ≠ 不等于
    { 0x2264, 0xA1DC }, // ≤ 小于等于 (Alt+41436)
    { 0x2265, 0xA1DD }, // ≥ 大于等于 (Alt+41437)
    { 0x221E, 0xA1DE }, // ∞ 无穷大

    // 5. 带圈数字 ① ~ ⑩ (U+2460..2469)
    { 0x2460, 0xA2D9 }, // ①
    { 0x2461, 0xA2DA }, // ②
    { 0x2462, 0xA2DB }, // ③
    { 0x2463, 0xA2DC }, // ④
    { 0x2464, 0xA2DD }, // ⑤
    { 0x2465, 0xA2DE }, // ⑥
    { 0x2466, 0xA2DF }, // ⑦
    { 0x2467, 0xA2E0 }, // ⑧
    { 0x2468, 0xA2E1 }, // ⑨
    { 0x2469, 0xA2E2 }, // ⑩

    // 6. 箭头与杂项符号 (U+2190..26FF)
    { 0x2190, 0xA1FB }, // ←
    { 0x2191, 0xA1FC }, // ↑
    { 0x2192, 0xA1FA }, // →
    { 0x2193, 0xA1FD }, // ↓
    { 0x25C6, 0xA1F4 }, // ◆
    { 0x25C7, 0xA1F3 }, // ◇
    { 0x25A0, 0xA1F6 }, // ■
    { 0x25A1, 0xA1F5 }, // □
    { 0x25B2, 0xA1F8 }, // ▲
    { 0x25B3, 0xA1F7 }, // △
    { 0x25CE, 0xA1F2 }, // ◎
    { 0x25CF, 0xA1F1 }, // ●
    { 0x25CB, 0xA1F0 }, // ○
    { 0x2605, 0xA1EF }, // ★
    { 0x2606, 0xA1EE }, // ☆

    // 7. 希腊字母 (GBK A6A1..A6B8 / A6C1..A6D8)。GBK 无大写 sigma 终形，跳过 U+03A2。
    { 0x0391, 0xA6A1 }, // Α
    { 0x0392, 0xA6A2 }, // Β
    { 0x0393, 0xA6A3 }, // Γ
    { 0x0394, 0xA6A4 }, // Δ
    { 0x0395, 0xA6A5 }, // Ε
    { 0x0396, 0xA6A6 }, // Ζ
    { 0x0397, 0xA6A7 }, // Η
    { 0x0398, 0xA6A8 }, // Θ
    { 0x0399, 0xA6A9 }, // Ι
    { 0x039A, 0xA6AA }, // Κ
    { 0x039B, 0xA6AB }, // Λ
    { 0x039C, 0xA6AC }, // Μ
    { 0x039D, 0xA6AD }, // Ν
    { 0x039E, 0xA6AE }, // Ξ
    { 0x039F, 0xA6AF }, // Ο
    { 0x03A0, 0xA6B0 }, // Π
    { 0x03A1, 0xA6B1 }, // Ρ
    { 0x03A3, 0xA6B2 }, // Σ
    { 0x03A4, 0xA6B3 }, // Τ
    { 0x03A5, 0xA6B4 }, // Υ
    { 0x03A6, 0xA6B5 }, // Φ
    { 0x03A7, 0xA6B6 }, // Χ
    { 0x03A8, 0xA6B7 }, // Ψ
    { 0x03A9, 0xA6B8 }, // Ω
    { 0x03B1, 0xA6C1 }, // α
    { 0x03B2, 0xA6C2 }, // β
    { 0x03B3, 0xA6C3 }, // γ
    { 0x03B4, 0xA6C4 }, // δ
    { 0x03B5, 0xA6C5 }, // ε
    { 0x03B6, 0xA6C6 }, // ζ
    { 0x03B7, 0xA6C7 }, // η
    { 0x03B8, 0xA6C8 }, // θ
    { 0x03B9, 0xA6C9 }, // ι
    { 0x03BA, 0xA6CA }, // κ
    { 0x03BB, 0xA6CB }, // λ
    { 0x03BC, 0xA6CC }, // μ
    { 0x03BD, 0xA6CD }, // ν
    { 0x03BE, 0xA6CE }, // ξ
    { 0x03BF, 0xA6CF }, // ο
    { 0x03C0, 0xA6D0 }, // π
    { 0x03C1, 0xA6D1 }, // ρ
    { 0x03C3, 0xA6D2 }, // σ
    { 0x03C4, 0xA6D3 }, // τ
    { 0x03C5, 0xA6D4 }, // υ
    { 0x03C6, 0xA6D5 }, // φ
    { 0x03C7, 0xA6D6 }, // χ
    { 0x03C8, 0xA6D7 }, // ψ
    { 0x03C9, 0xA6D8 }, // ω
    { 0x00B5, 0xA6CC }, // µ 微符号，与 μ 同形
    { 0x2126, 0xA6B8 }, // Ω 欧姆符号，与 Ω 同形
};

static uint16_t get_special_gbk(uint32_t codepoint) {
    size_t count = sizeof(SPECIAL_GBK_MAP) / sizeof(SpecialGbkEntry);
    for (size_t i = 0; i < count; i++) {
        if (SPECIAL_GBK_MAP[i].unicode == codepoint) {
            return SPECIAL_GBK_MAP[i].gbk;
        }
    }
    return 0;
}

static uint16_t get_gbk_code(uint32_t codepoint) {
    // 0. 优先查找特殊医疗/符号表 (U+00A0..02FF, U+2000..26FF)
    uint16_t special = get_special_gbk(codepoint);
    if (special > 0) return special;

    if (!gbk_data_ptr) return 0;
    size_t idx = 0;

    const uint32_t Z1_START = 0x4E00, Z1_COUNT = 20992;
    const uint32_t Z2_START = 0x3000, Z2_COUNT = 64;
    const size_t ZONE_OFFSET = (Z1_COUNT + Z2_COUNT) * 2;

    if (codepoint >= Z1_START && codepoint < Z1_START + Z1_COUNT) {
        idx = (codepoint - Z1_START) * 2;
    } else if (codepoint >= Z2_START && codepoint < Z2_START + Z2_COUNT) {
        idx = Z1_COUNT * 2 + (codepoint - Z2_START) * 2;
    } else if (codepoint >= 0xFF00 && codepoint < 0xFFF0) {
        idx = ZONE_OFFSET + (codepoint - 0xFF00) * 2;
    } else {
        return 0;
    }

    if (idx + 1 >= gbk_data_len) return 0;
    return (uint16_t)gbk_data_ptr[idx] | ((uint16_t)gbk_data_ptr[idx + 1] << 8);
}

// ==============================================================================
// 延时与急停代际控制
// ==============================================================================
static volatile uint32_t abort_gen = 0; // 收到急停指令时自增代际
static volatile uint32_t seen_gen = 0;  // 主循环消费的急停代际

static inline bool delay_abortable(float ms) {
    if (ms <= 0.0f) return true;
    uint32_t total_us = (uint32_t)(ms * 1000.0f);
    uint32_t elapsed = 0;
    while (elapsed < total_us) {
        if (seen_gen != abort_gen) return false;
        uint32_t step = (total_us - elapsed > 5000) ? 5000 : (total_us - elapsed);
        if (step >= 1000) {
            delay(step / 1000);
            delayMicroseconds(step % 1000);
        } else {
            delayMicroseconds(step);
        }
        elapsed += step;
    }
    return (seen_gen == abort_gen);
}

// ==============================================================================
// CH9329 协议发送与状态查询
// ==============================================================================
static void send_ch9329_frame(uint8_t mod, uint8_t k1 = 0, uint8_t k2 = 0, uint8_t k3 = 0,
                              uint8_t k4 = 0, uint8_t k5 = 0, uint8_t k6 = 0) {
    uint8_t frame[14] = {
        0x57, 0xAB, 0x00, 0x02, 0x08, mod, 0x00, k1, k2, k3, k4, k5, k6, 0x00
    };
    uint32_t sum = 0;
    for (int i = 0; i < 13; i++) sum += frame[i];
    frame[13] = (uint8_t)(sum & 0xFF);
    Serial1.write(frame, 14);
}

// ------------------------------------------------------------------------------
// CH9329 鼠标控制 (CMD 0x05 相对鼠标协议)
// ------------------------------------------------------------------------------
#define MOUSE_BTN_NONE   0x00
#define MOUSE_BTN_LEFT   0x01
#define MOUSE_BTN_RIGHT  0x02
#define MOUSE_BTN_MIDDLE 0x04

static uint8_t current_mouse_btn = MOUSE_BTN_NONE;

static void send_ch9329_mouse(uint8_t btn, int8_t x = 0, int8_t y = 0, int8_t wheel = 0) {
    uint8_t frame[11] = {
        0x57, 0xAB, 0x00, 0x05, 0x05, 0x01, btn, (uint8_t)x, (uint8_t)y, (uint8_t)wheel, 0x00
    };
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) sum += frame[i];
    frame[10] = (uint8_t)(sum & 0xFF);
    Serial1.write(frame, 11);
    current_mouse_btn = btn;
}

static bool tap_mouse_button(uint8_t btn) {
    if (seen_gen != abort_gen) return false;
    send_ch9329_mouse(btn, 0, 0, 0);
    if (!delay_abortable(timing.tap_press_ms)) {
        send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
        return false;
    }
    send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
    return delay_abortable(timing.tap_release_ms);
}

static bool tap_mouse_double_click(uint8_t btn) {
    if (!tap_mouse_button(btn)) return false;
    if (!delay_abortable(60.0f)) return false;
    return tap_mouse_button(btn);
}

static bool move_mouse_rel(int8_t x, int8_t y, int8_t wheel = 0) {
    if (seen_gen != abort_gen) return false;
    send_ch9329_mouse(current_mouse_btn, x, y, wheel);
    return delay_abortable(timing.tap_release_ms);
}

// 移至角落归零：通过连续大位移“撞墙”使光标归位至指定角落
// corner: 0=Top-Left(左上角 0,0), 1=Top-Right(右上角), 2=Bottom-Left(左下角), 3=Bottom-Right(右下角)
static bool reset_mouse_to_corner(uint8_t corner = 0) {
    int8_t step_x = (corner == 1 || corner == 3) ? 120 : -120;
    int8_t step_y = (corner == 2 || corner == 3) ? 120 : -120;

    for (int k = 0; k < 35; k++) {
        if (seen_gen != abort_gen) return false;
        send_ch9329_mouse(current_mouse_btn, step_x, step_y, 0);
        if (!delay_abortable(4.0f)) return false;
    }
    return delay_abortable(15.0f);
}

// 支持任意大距离平滑分步相对移动 (不受 int8_t -127~127 限制)
static bool move_mouse_large(int dx, int dy) {
    if (seen_gen != abort_gen) return false;
    while (dx != 0 || dy != 0) {
        if (seen_gen != abort_gen) return false;
        int step_x = constrain(dx, -100, 100);
        int step_y = constrain(dy, -100, 100);
        dx -= step_x;
        dy -= step_y;
        send_ch9329_mouse(current_mouse_btn, (int8_t)step_x, (int8_t)step_y, 0);
        if (dx != 0 || dy != 0) {
            if (!delay_abortable(4.0f)) return false;
        }
    }
    return delay_abortable(timing.tap_release_ms);
}

static void ch_drain() {
    while (Serial1.available()) Serial1.read();
}

static bool ch9329_get_info(uint8_t &usb_enum, uint8_t &led_status) {
    ch_drain();
    const uint8_t req[6] = {0x57, 0xAB, 0x00, 0x01, 0x00, 0x03};
    Serial1.write(req, 6);

    uint8_t buf[32];
    size_t len = 0;
    unsigned long start = millis();
    while (millis() - start < 80) {
        while (Serial1.available() && len < sizeof(buf)) {
            buf[len++] = Serial1.read();
        }
        if (len >= 14) break;
        delay(1);
    }

    if (len >= 14 && buf[0] == 0x57 && buf[1] == 0xAB && buf[3] == 0x81) {
        usb_enum = buf[6];
        led_status = buf[7];
        return true;
    }
    return false;
}

static bool tap_key(uint8_t mod, uint8_t key) {
    if (seen_gen != abort_gen) return false;
    send_ch9329_frame(mod, key);
    if (!delay_abortable(timing.tap_press_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_NONE);
    return delay_abortable(timing.tap_release_ms);
}

static bool tap_enter_key() {
    if (seen_gen != abort_gen) return false;
    send_ch9329_frame(MOD_NONE, 0x28);
    if (!delay_abortable(timing.enter_press_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_NONE);
    return delay_abortable(timing.enter_release_ms);
}

static bool tap_tab_key() {
    if (seen_gen != abort_gen) return false;
    send_ch9329_frame(MOD_NONE, 0x2B);
    if (!delay_abortable(timing.tab_press_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_NONE);
    return delay_abortable(timing.tab_release_ms);
}

static bool tap_shift_key(uint8_t key) {
    if (seen_gen != abort_gen) return false;
    send_ch9329_frame(MOD_SHIFT);
    if (!delay_abortable(timing.shift_press_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_SHIFT, key);
    if (!delay_abortable(timing.tap_press_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_SHIFT);
    if (!delay_abortable(timing.shift_release_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }
    send_ch9329_frame(MOD_NONE);
    return delay_abortable(timing.tap_release_ms);
}

// 闭环自愈键盘指示灯状态
static bool ensure_keyboard_state() {
    for (int i = 0; i < 4; i++) {
        uint8_t usb_enum = 0, led = 0;
        if (!ch9329_get_info(usb_enum, led) || usb_enum != 0x01) {
            return false;
        }
        bool numlock_ok = (led & 0x01) != 0;
        bool capslock_ok = (led & 0x02) == 0;

        if (numlock_ok && capslock_ok) {
            return true;
        }

        if (!numlock_ok) {
            tap_key(MOD_NONE, 0x53); // 按 NumLock 点亮
            delay_abortable(30.0f);
        }
        if (!capslock_ok) {
            tap_key(MOD_NONE, 0x39); // 按 CapsLock 熄灭
            delay_abortable(30.0f);
        }
    }
    return true;
}

// 长文本自愈与保护：在物理动作彻底完成的安全空隙触发执行
static void execute_safe_health_check() {
    if (seen_gen != abort_gen) return;

    // 1. 防粘滞硬重置释放栅栏：强制向电脑发送全零松开帧，并微休消除按键粘滞
    if (timing.fence_auto >= 0.5f) {
        send_ch9329_frame(MOD_NONE);
        delay_abortable(15.0f);
    }

    // 2. 静默探针：确保 NumLock 仍处于开启状态，意外熄灭则原地按亮扶正
    if (timing.numlock_auto >= 0.5f) {
        uint8_t usb_enum = 0, led = 0;
        if (ch9329_get_info(usb_enum, led) && usb_enum == 0x01) {
            bool numlock_ok = (led & 0x01) != 0;
            if (!numlock_ok) {
                tap_key(MOD_NONE, 0x53); // 按 NumLock 点亮
                delay_abortable(20.0f);
            }
        }
    }

    // 3. 闭环翻转握手：通过 CapsLock 翻转并回读验证 USB 链路活性（默认关闭）
    if (timing.shake_auto >= 0.5f) {
        uint8_t usb_enum = 0, led_before = 0;
        if (ch9329_get_info(usb_enum, led_before) && usb_enum == 0x01) {
            tap_key(MOD_NONE, 0x39); // 敲击 CapsLock 翻转
            delay_abortable(12.0f);
            uint8_t led_after = 0;
            if (ch9329_get_info(usb_enum, led_after)) {
                // 敲击后恢复原状
                tap_key(MOD_NONE, 0x39);
                delay_abortable(12.0f);
            }
        }
    }
}

// 通过 Alt Code 打汉字 (支持随时被急停打断)
static bool send_alt_code(uint16_t number) {
    if (seen_gen != abort_gen) return false;
    char digits[6];
    snprintf(digits, sizeof(digits), "%05u", (unsigned)number);

    send_ch9329_frame(MOD_ALT);
    if (!delay_abortable(timing.alt_hold_ms)) {
        send_ch9329_frame(MOD_NONE);
        return false;
    }

    for (int i = 0; i < 5; i++) {
        if (seen_gen != abort_gen) {
            send_ch9329_frame(MOD_NONE);
            return false;
        }
        uint8_t digit = digits[i] - '0';
        uint8_t key = KEYPAD_NUMS[digit];

        send_ch9329_frame(MOD_ALT, key);
        if (!delay_abortable(timing.alt_digit_press)) {
            send_ch9329_frame(MOD_NONE);
            return false;
        }
        send_ch9329_frame(MOD_ALT);

        if (i == 4) {
            if (!delay_abortable(timing.alt_release_ms)) {
                send_ch9329_frame(MOD_NONE);
                return false;
            }
        } else {
            if (!delay_abortable(timing.alt_digit_release)) {
                send_ch9329_frame(MOD_NONE);
                return false;
            }
        }
    }

    send_ch9329_frame(MOD_NONE);
    return delay_abortable(timing.alt_release_ms);
}

// ASCII 物理按键查找表
static KeyMap get_ascii_keymap(char c) {
    if (c >= 'a' && c <= 'z') return { (uint8_t)(0x04 + (c - 'a')), false };
    if (c >= 'A' && c <= 'Z') return { (uint8_t)(0x04 + (c - 'A')), true };
    if (c >= '1' && c <= '9') return { (uint8_t)(0x1E + (c - '1')), false };
    if (c == '0') return { 0x27, false };
    if (c == ' ') return { 0x2C, false };

    switch (c) {
        case '!': return { 0x1E, true };
        case '@': return { 0x1F, true };
        case '#': return { 0x20, true };
        case '$': return { 0x21, true };
        case '%': return { 0x22, true };
        case '^': return { 0x23, true };
        case '&': return { 0x24, true };
        case '*': return { 0x25, true };
        case '(': return { 0x26, true };
        case ')': return { 0x27, true };
        case '-': return { 0x2D, false };
        case '_': return { 0x2D, true };
        case '=': return { 0x2E, false };
        case '+': return { 0x2E, true };
        case '[': return { 0x2F, false };
        case '{': return { 0x2F, true };
        case ']': return { 0x30, false };
        case '}': return { 0x30, true };
        case '\\': return { 0x31, false };
        case '|': return { 0x31, true };
        case ';': return { 0x33, false };
        case ':': return { 0x33, true };
        case '\'': return { 0x34, false };
        case '"': return { 0x34, true };
        case '`': return { 0x35, false };
        case '~': return { 0x35, true };
        case ',': return { 0x36, false };
        case '<': return { 0x36, true };
        case '.': return { 0x37, false };
        case '>': return { 0x37, true };
        case '/': return { 0x38, false };
        case '?': return { 0x38, true };
        default:  return { 0, false };
    }
}

// 单键与功能键 HID 查找表
static uint8_t get_single_keycode(const String &name) {
    if (name == "BS" || name == "BACKSPACE") return 0x2A;
    if (name == "TAB") return 0x2B;
    if (name == "ENTER" || name == "RETURN") return 0x28;
    if (name == "ESC" || name == "ESCAPE") return 0x29;
    if (name == "DEL" || name == "DELETE") return 0x4C;
    if (name == "SPACE") return 0x2C;
    if (name == "INSERT" || name == "INS") return 0x49;
    if (name == "UP") return 0x52;
    if (name == "DOWN") return 0x51;
    if (name == "LEFT") return 0x50;
    if (name == "RIGHT") return 0x4F;
    if (name == "PAGEUP" || name == "PGUP") return 0x4B;
    if (name == "PAGEDOWN" || name == "PGDN") return 0x4E;
    if (name == "HOME") return 0x4A;
    if (name == "END") return 0x4D;
    if (name == "NUMLOCK" || name == "NUM") return 0x53;
    if (name == "CAPSLOCK" || name == "CAPS") return 0x39;
    if (name == "SCROLLLOCK" || name == "SCROLL") return 0x47;
    if (name == "PRTSC" || name == "PRINT") return 0x46;
    if (name == "PAUSE") return 0x48;

    // F1 - F12
    if (name.startsWith("F") && name.length() <= 3) {
        int fNum = name.substring(1).toInt();
        if (fNum >= 1 && fNum <= 12) {
            return (uint8_t)(0x3A + (fNum - 1)); // F1: 0x3A ... F12: 0x45
        }
    }

    // 单字母 A-Z (0x04 - 0x1D)
    if (name.length() == 1) {
        char ch = name[0];
        if (ch >= 'A' && ch <= 'Z') return (uint8_t)(0x04 + (ch - 'A'));
        if (ch >= 'a' && ch <= 'z') return (uint8_t)(0x04 + (ch - 'a'));
        if (ch >= '1' && ch <= '9') return (uint8_t)(0x1E + (ch - '1'));
        if (ch == '0') return 0x27;
    }

    return 0;
}

// 统一解析 {KEY} 或 {CTRL+ALT+KEY} 标签并执行敲击
// 返回是否命中已知按键/组合键标签
static bool try_execute_key_or_combo(const String &token) {
    if (!token.startsWith("{") || !token.endsWith("}")) return false;
    String raw = token.substring(1, token.length() - 1);
    raw.trim();
    raw.toUpperCase();
    if (raw.isEmpty()) return false;

    // 排除特定命令
    if (raw.startsWith("DONE") || raw.startsWith("CFG") || raw.startsWith("NAME") || raw.startsWith("DELAY") ||
        raw.startsWith("CLICK") || raw.startsWith("LCLICK") || raw.startsWith("RCLICK") || raw.startsWith("MCLICK") ||
        raw.startsWith("DBLCLICK") || raw.startsWith("DOUBLE") || raw.startsWith("SCROLL") || raw.startsWith("WHEEL") ||
        raw.startsWith("MOVE") || raw.startsWith("MOUSE") || raw.startsWith("LDOWN") || raw.startsWith("LUP") ||
        raw.startsWith("RDOWN") || raw.startsWith("RUP") || raw.startsWith("MDOWN") || raw.startsWith("MUP") ||
        raw.startsWith("CORNER") || raw.startsWith("RESET_MOUSE")) {
        return false;
    }

    uint8_t mod = MOD_NONE;
    uint8_t key = 0;

    int start = 0;
    while (start < raw.length()) {
        int plusIdx = raw.indexOf('+', start);
        if (plusIdx == -1) plusIdx = raw.length();
        String part = raw.substring(start, plusIdx);
        part.trim();
        start = plusIdx + 1;

        if (part == "CTRL" || part == "LCTRL" || part == "RCTRL") {
            mod |= MOD_CTRL;
        } else if (part == "ALT" || part == "LALT" || part == "RALT") {
            mod |= MOD_ALT;
        } else if (part == "SHIFT" || part == "LSHIFT" || part == "RSHIFT") {
            mod |= MOD_SHIFT;
        } else if (part == "WIN" || part == "CMD" || part == "META") {
            mod |= MOD_WIN;
        } else {
            // 主按键
            uint8_t kc = get_single_keycode(part);
            if (kc != 0) {
                key = kc;
            }
        }
    }

    // 如果只有修饰键，或者有主键，执行标准发送
    if (mod != MOD_NONE || key != 0) {
        if (mod == MOD_NONE && key == 0x28) {
            return tap_enter_key();
        }
        if (mod == MOD_NONE && key == 0x2B) {
            return tap_tab_key();
        }
        tap_key(mod, key);
        return true;
    }
    return false;
}

// 解析并执行鼠标动作标签：点击、双击、长按、释放、滚轮、位移
static bool try_execute_mouse_action(const String &token) {
    if (!token.startsWith("{") || !token.endsWith("}")) return false;
    String raw = token.substring(1, token.length() - 1);
    raw.trim();
    String upper = raw;
    upper.toUpperCase();
    if (upper.isEmpty()) return false;

    // 1. 左键单击
    if (upper == "CLICK" || upper == "LCLICK" || upper == "LEFT_CLICK" || upper == "MOUSE_LEFT") {
        return tap_mouse_button(MOUSE_BTN_LEFT);
    }
    // 2. 右键单击
    if (upper == "RCLICK" || upper == "RIGHT_CLICK" || upper == "MOUSE_RIGHT") {
        return tap_mouse_button(MOUSE_BTN_RIGHT);
    }
    // 3. 中键单击
    if (upper == "MCLICK" || upper == "MID_CLICK" || upper == "MIDDLE_CLICK" || upper == "MOUSE_MIDDLE") {
        return tap_mouse_button(MOUSE_BTN_MIDDLE);
    }

    // 4. 左键双击
    if (upper == "DBLCLICK" || upper == "DOUBLE_CLICK" || upper == "DOUBLECLICK" || upper == "LDBLCLICK") {
        return tap_mouse_double_click(MOUSE_BTN_LEFT);
    }

    // 5. 按下（拖拽或长按）
    if (upper == "LDOWN" || upper == "LEFT_DOWN" || upper == "CLICK_DOWN") {
        current_mouse_btn |= MOUSE_BTN_LEFT;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_press_ms);
    }
    if (upper == "RDOWN" || upper == "RIGHT_DOWN") {
        current_mouse_btn |= MOUSE_BTN_RIGHT;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_press_ms);
    }
    if (upper == "MDOWN" || upper == "MID_DOWN" || upper == "MIDDLE_DOWN") {
        current_mouse_btn |= MOUSE_BTN_MIDDLE;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_press_ms);
    }

    // 6. 释放（拖拽结束或松开）
    if (upper == "LUP" || upper == "LEFT_UP" || upper == "CLICK_UP") {
        current_mouse_btn &= ~MOUSE_BTN_LEFT;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_release_ms);
    }
    if (upper == "RUP" || upper == "RIGHT_UP") {
        current_mouse_btn &= ~MOUSE_BTN_RIGHT;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_release_ms);
    }
    if (upper == "MUP" || upper == "MID_UP" || upper == "MIDDLE_UP") {
        current_mouse_btn &= ~MOUSE_BTN_MIDDLE;
        send_ch9329_mouse(current_mouse_btn, 0, 0, 0);
        return delay_abortable(timing.tap_release_ms);
    }
    if (upper == "MOUSE_UP") {
        current_mouse_btn = MOUSE_BTN_NONE;
        send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
        return delay_abortable(timing.tap_release_ms);
    }

    // 7. 滚轮 {SCROLL:n} 或 {WHEEL:n} (正数向上滚，负数向下滚)
    if (upper.startsWith("SCROLL:") || upper.startsWith("WHEEL:")) {
        int colon = upper.indexOf(':');
        int val = upper.substring(colon + 1).toInt();
        val = constrain(val, -127, 127);
        if (val != 0) {
            move_mouse_rel(0, 0, (int8_t)val);
        }
        return true;
    }

    // 8. 角落归零 {RESET_MOUSE} / {CORNER:TL} / {CORNER:TR} / {CORNER:BL} / {CORNER:BR}
    if (upper == "RESET_MOUSE" || upper == "CORNER:TL" || upper == "CORNER:TOP_LEFT" || upper == "CORNER_TL") {
        return reset_mouse_to_corner(0); // 左上角 (0, 0)
    }
    if (upper == "CORNER:TR" || upper == "CORNER:TOP_RIGHT" || upper == "CORNER_TR") {
        return reset_mouse_to_corner(1); // 右上角
    }
    if (upper == "CORNER:BL" || upper == "CORNER:BOTTOM_LEFT" || upper == "CORNER_BL") {
        return reset_mouse_to_corner(2); // 左下角
    }
    if (upper == "CORNER:BR" || upper == "CORNER:BOTTOM_RIGHT" || upper == "CORNER_BR") {
        return reset_mouse_to_corner(3); // 右下角
    }

    // 9. 相对平滑移动 {MOVE:dx,dy} 或 {MOUSE_MOVE:dx,dy} (支持大跨度位移如 500, 1200)
    if (upper.startsWith("MOVE:") || upper.startsWith("MOUSE_MOVE:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int dx = params.substring(0, comma).toInt();
            int dy = params.substring(comma + 1).toInt();
            return move_mouse_large(dx, dy);
        }
    }

    // 10. 绝对移动定位 {MOVE_TO:x,y} (左上角归零后移动至 x, y)
    if (upper.startsWith("MOVE_TO:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int x = params.substring(0, comma).toInt();
            int y = params.substring(comma + 1).toInt();
            if (!reset_mouse_to_corner(0)) return false;
            return move_mouse_large(x, y);
        }
    }

    // 11. 指定绝对坐标点击 {CLICK_AT:x,y} / {RCLICK_AT:x,y} / {DBLCLICK_AT:x,y} (左上角基准)
    if (upper.startsWith("CLICK_AT:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int x = params.substring(0, comma).toInt();
            int y = params.substring(comma + 1).toInt();
            if (!reset_mouse_to_corner(0)) return false;
            if (!move_mouse_large(x, y)) return false;
            return tap_mouse_button(MOUSE_BTN_LEFT);
        }
    }
    if (upper.startsWith("RCLICK_AT:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int x = params.substring(0, comma).toInt();
            int y = params.substring(comma + 1).toInt();
            if (!reset_mouse_to_corner(0)) return false;
            if (!move_mouse_large(x, y)) return false;
            return tap_mouse_button(MOUSE_BTN_RIGHT);
        }
    }
    if (upper.startsWith("DBLCLICK_AT:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int x = params.substring(0, comma).toInt();
            int y = params.substring(comma + 1).toInt();
            if (!reset_mouse_to_corner(0)) return false;
            if (!move_mouse_large(x, y)) return false;
            return tap_mouse_double_click(MOUSE_BTN_LEFT);
        }
    }

    // 12. 右上角基准偏移点击 {CLICK_TR:dx,dy} (如用户习惯把光标停在右上角，dx通常为负数向左移)
    if (upper.startsWith("CLICK_TR:")) {
        int colon = upper.indexOf(':');
        String params = upper.substring(colon + 1);
        int comma = params.indexOf(',');
        if (comma != -1) {
            int dx = params.substring(0, comma).toInt();
            int dy = params.substring(comma + 1).toInt();
            if (!reset_mouse_to_corner(1)) return false;
            if (!move_mouse_large(dx, dy)) return false;
            return tap_mouse_button(MOUSE_BTN_LEFT);
        }
    }

    return false;
}

// ==============================================================================
// BLE 与多源流控安全机制
// ==============================================================================
static BLEServer *pServer = nullptr;
static BLECharacteristic *pTxCharacteristic = nullptr;
static bool ble_connected = false;

// 统一输入流队列与临界区
static String rx_stream_buf = "";
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// 蓝牙回执发送函数 (严格 MTU 20 字节分片，彻底杜绝长数据包在 iOS/Android 上被 BLE 协议栈静默丢弃)
static void send_back(const String &msg) {
    if (!ble_connected || !pTxCharacteristic) return;
    String full = msg + "\n";
    const uint8_t *data = (const uint8_t*)full.c_str();
    size_t len = full.length();
    size_t offset = 0;
    const size_t CHUNK_SIZE = 20;
    while (offset < len) {
        size_t chunk = (len - offset > CHUNK_SIZE) ? CHUNK_SIZE : (len - offset);
        pTxCharacteristic->setValue((uint8_t*)(data + offset), chunk);
        pTxCharacteristic->notify();
        offset += chunk;
        if (offset < len) {
            delay_abortable(12.0f);
        }
    }
}

// 格式化当前时序
static String describe_timing() {
    char buf[420];
    snprintf(buf, sizeof(buf),
        "tap_press_ms=%.1f tap_release_ms=%.1f shift_press_ms=%.1f shift_release_ms=%.1f "
        "alt_hold_ms=%.1f alt_digit_press=%.1f alt_digit_release=%.1f alt_release_ms=%.1f "
        "enter_press_ms=%.1f enter_release_ms=%.1f tab_press_ms=%.1f tab_release_ms=%.1f "
        "ascii_gap_ms=%.1f cjk_gap_ms=%.1f numlock_auto=%.0f alt_ascii=%.0f "
        "fence_auto=%.0f shake_auto=%.0f check_interval=%.0f",
        timing.tap_press_ms, timing.tap_release_ms, timing.shift_press_ms, timing.shift_release_ms,
        timing.alt_hold_ms, timing.alt_digit_press, timing.alt_digit_release, timing.alt_release_ms,
        timing.enter_press_ms, timing.enter_release_ms, timing.tab_press_ms, timing.tab_release_ms,
        timing.ascii_gap_ms, timing.cjk_gap_ms, timing.numlock_auto, timing.alt_ascii,
        timing.fence_auto, timing.shake_auto, timing.check_interval);
    return String(buf);
}

// 参数范围与防变砖安全网
static float clamp_val(float v, float min_v, float max_v, float def_v) {
    if (isnan(v) || isinf(v)) return def_v;
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void clamp_timing(TimingConfig &t) {
    t.tap_press_ms      = clamp_val(t.tap_press_ms, 0.0f, 2000.0f, DEFAULT_TIMING.tap_press_ms);
    t.tap_release_ms    = clamp_val(t.tap_release_ms, 0.0f, 2000.0f, DEFAULT_TIMING.tap_release_ms);
    t.shift_press_ms    = clamp_val(t.shift_press_ms, 0.0f, 2000.0f, DEFAULT_TIMING.shift_press_ms);
    t.shift_release_ms  = clamp_val(t.shift_release_ms, 0.0f, 2000.0f, DEFAULT_TIMING.shift_release_ms);
    t.alt_hold_ms       = clamp_val(t.alt_hold_ms, 0.0f, 2000.0f, DEFAULT_TIMING.alt_hold_ms);
    t.alt_digit_press   = clamp_val(t.alt_digit_press, 0.0f, 2000.0f, DEFAULT_TIMING.alt_digit_press);
    t.alt_digit_release = clamp_val(t.alt_digit_release, 0.0f, 2000.0f, DEFAULT_TIMING.alt_digit_release);
    t.alt_release_ms    = clamp_val(t.alt_release_ms, 0.0f, 2000.0f, DEFAULT_TIMING.alt_release_ms);
    t.enter_press_ms    = clamp_val(t.enter_press_ms, 0.0f, 2000.0f, DEFAULT_TIMING.enter_press_ms);
    t.enter_release_ms  = clamp_val(t.enter_release_ms, 0.0f, 2000.0f, DEFAULT_TIMING.enter_release_ms);
    t.tab_press_ms      = clamp_val(t.tab_press_ms, 0.0f, 2000.0f, DEFAULT_TIMING.tab_press_ms);
    t.tab_release_ms    = clamp_val(t.tab_release_ms, 0.0f, 2000.0f, DEFAULT_TIMING.tab_release_ms);
    t.ascii_gap_ms      = clamp_val(t.ascii_gap_ms, 0.0f, 2000.0f, DEFAULT_TIMING.ascii_gap_ms);
    t.cjk_gap_ms        = clamp_val(t.cjk_gap_ms, 0.0f, 2000.0f, DEFAULT_TIMING.cjk_gap_ms);
    t.numlock_auto      = (t.numlock_auto >= 0.5f) ? 1.0f : 0.0f;
    t.alt_ascii         = (t.alt_ascii >= 0.5f) ? 1.0f : 0.0f;
    t.fence_auto        = (t.fence_auto >= 0.5f) ? 1.0f : 0.0f;
    t.shake_auto        = (t.shake_auto >= 0.5f) ? 1.0f : 0.0f;
    t.check_interval    = clamp_val(t.check_interval, 0.0f, 500.0f, DEFAULT_TIMING.check_interval);
}

// NVS 持久化
static void load_timing_from_nvs() {
    prefs.begin("timing", true);
    timing.tap_press_ms     = prefs.getFloat("tap_press", DEFAULT_TIMING.tap_press_ms);
    timing.tap_release_ms   = prefs.getFloat("tap_rel", DEFAULT_TIMING.tap_release_ms);
    timing.shift_press_ms   = prefs.getFloat("shift_press", DEFAULT_TIMING.shift_press_ms);
    timing.shift_release_ms = prefs.getFloat("shift_rel", DEFAULT_TIMING.shift_release_ms);
    timing.alt_hold_ms      = prefs.getFloat("alt_hold", DEFAULT_TIMING.alt_hold_ms);
    timing.alt_digit_press  = prefs.getFloat("alt_d_press", DEFAULT_TIMING.alt_digit_press);
    timing.alt_digit_release= prefs.getFloat("alt_d_rel", DEFAULT_TIMING.alt_digit_release);
    timing.alt_release_ms   = prefs.getFloat("alt_rel", DEFAULT_TIMING.alt_release_ms);
    timing.enter_press_ms   = prefs.getFloat("enter_press", DEFAULT_TIMING.enter_press_ms);
    timing.enter_release_ms = prefs.getFloat("enter_rel", DEFAULT_TIMING.enter_release_ms);
    timing.tab_press_ms     = prefs.getFloat("tab_press", DEFAULT_TIMING.tab_press_ms);
    timing.tab_release_ms   = prefs.getFloat("tab_rel", DEFAULT_TIMING.tab_release_ms);
    timing.ascii_gap_ms     = prefs.getFloat("ascii_gap", DEFAULT_TIMING.ascii_gap_ms);
    timing.cjk_gap_ms       = prefs.getFloat("cjk_gap", DEFAULT_TIMING.cjk_gap_ms);
    timing.numlock_auto     = prefs.getFloat("numlock_auto", DEFAULT_TIMING.numlock_auto);
    timing.alt_ascii        = prefs.getFloat("alt_ascii", DEFAULT_TIMING.alt_ascii);
    timing.fence_auto       = prefs.getFloat("fence_auto", DEFAULT_TIMING.fence_auto);
    timing.shake_auto       = prefs.getFloat("shake_auto", DEFAULT_TIMING.shake_auto);
    timing.check_interval   = prefs.getFloat("check_int", DEFAULT_TIMING.check_interval);
    prefs.end();

    clamp_timing(timing);
}

static void save_timing_to_nvs() {
    prefs.begin("timing", false);
    prefs.putFloat("tap_press", timing.tap_press_ms);
    prefs.putFloat("tap_rel", timing.tap_release_ms);
    prefs.putFloat("shift_press", timing.shift_press_ms);
    prefs.putFloat("shift_rel", timing.shift_release_ms);
    prefs.putFloat("alt_hold", timing.alt_hold_ms);
    prefs.putFloat("alt_d_press", timing.alt_digit_press);
    prefs.putFloat("alt_d_rel", timing.alt_digit_release);
    prefs.putFloat("alt_rel", timing.alt_release_ms);
    prefs.putFloat("enter_press", timing.enter_press_ms);
    prefs.putFloat("enter_rel", timing.enter_release_ms);
    prefs.putFloat("tab_press", timing.tab_press_ms);
    prefs.putFloat("tab_rel", timing.tab_release_ms);
    prefs.putFloat("ascii_gap", timing.ascii_gap_ms);
    prefs.putFloat("cjk_gap", timing.cjk_gap_ms);
    prefs.putFloat("numlock_auto", timing.numlock_auto);
    prefs.putFloat("alt_ascii", timing.alt_ascii);
    prefs.putFloat("fence_auto", timing.fence_auto);
    prefs.putFloat("shake_auto", timing.shake_auto);
    prefs.putFloat("check_int", timing.check_interval);
    prefs.end();
}

// 设备名称 NVS 读取与保存
static void load_ble_name_from_nvs() {
    prefs.begin("device_cfg", true);
    String saved = prefs.getString("name", "");
    prefs.end();

    if (saved.length() > 0) {
        current_ble_name = saved;
    } else {
        current_ble_name = get_default_ble_name();
    }
}

static void save_ble_name_to_nvs(const String &newName) {
    prefs.begin("device_cfg", false);
    prefs.putString("name", newName);
    prefs.end();
    current_ble_name = newName;
}

static void reset_ble_name_in_nvs() {
    prefs.begin("device_cfg", false);
    prefs.remove("name");
    prefs.end();
    current_ble_name = get_default_ble_name();
}

// 处理设备名称命令 {NAME:...}
static bool handle_name_command(const String &token) {
    String upper = token;
    upper.toUpperCase();

    if (upper == "{NAME?}") {
        send_back(String("NAME:") + current_ble_name);
        return true;
    }
    if (upper == "{NAMERESET}") {
        reset_ble_name_in_nvs();
        send_back(String("NAME:RESET:") + current_ble_name);
        return true;
    }
    if (upper.startsWith("{NAME:") && upper.endsWith("}")) {
        String newName = token.substring(6, token.length() - 1);
        newName.trim();
        if (newName.length() > 0 && newName.length() <= 30) {
            save_ble_name_to_nvs(newName);
            send_back(String("NAME:OK:") + newName);
            Serial.printf("蓝牙名称已更新并写入 NVS: \"%s\" (下次广播生效)\n", newName.c_str());
        } else {
            send_back("NAME:ERR:INVALID_LENGTH");
        }
        return true;
    }
    return false;
}

// 处理时序配置命令 {CFG:...}
static bool handle_cfg_command(const String& raw_token) {
    String token = raw_token;
    token.trim();
    String upper = token;
    upper.toUpperCase();

    if (upper == "{CFG?}") {
        send_back(String("CFG:") + describe_timing());
        return true;
    }
    if (upper == "{CFGSAVE}") {
        clamp_timing(timing);
        save_timing_to_nvs();
        send_back("CFG:SAVED");
        return true;
    }
    if (upper == "{CFGRESET}") {
        timing = DEFAULT_TIMING;
        clamp_timing(timing);
        save_timing_to_nvs();
        send_back("CFG:RESET");
        return true;
    }

    if (!upper.startsWith("{CFG:") || !upper.endsWith("}")) return false;
    String body = token.substring(token.indexOf(':') + 1, token.length() - 1);
    body.trim();
    int start = 0;
    int updated = 0;

    while (start < body.length()) {
        int space = body.indexOf(' ', start);
        if (space == -1) space = body.length();
        String pair = body.substring(start, space);
        pair.trim();
        start = space + 1;
        if (pair.isEmpty()) continue;

        int eq = pair.indexOf('=');
        if (eq == -1) continue;
        String k = pair.substring(0, eq);
        k.trim();
        k.toLowerCase();
        float v = pair.substring(eq + 1).toFloat();

        if (k == "tap_press_ms")        { timing.tap_press_ms = v; updated++; }
        else if (k == "tap_release_ms") { timing.tap_release_ms = v; updated++; }
        else if (k == "shift_press_ms") { timing.shift_press_ms = v; updated++; }
        else if (k == "shift_release_ms"){ timing.shift_release_ms = v; updated++; }
        else if (k == "alt_hold_ms")    { timing.alt_hold_ms = v; updated++; }
        else if (k == "alt_digit_press"){ timing.alt_digit_press = v; updated++; }
        else if (k == "alt_digit_release"){ timing.alt_digit_release = v; updated++; }
        else if (k == "alt_release_ms") { timing.alt_release_ms = v; updated++; }
        else if (k == "enter_press_ms") { timing.enter_press_ms = v; updated++; }
        else if (k == "enter_release_ms"){ timing.enter_release_ms = v; updated++; }
        else if (k == "tab_press_ms")   { timing.tab_press_ms = v; updated++; }
        else if (k == "tab_release_ms") { timing.tab_release_ms = v; updated++; }
        else if (k == "ascii_gap_ms")   { timing.ascii_gap_ms = v; updated++; }
        else if (k == "cjk_gap_ms")     { timing.cjk_gap_ms = v; updated++; }
        else if (k == "key_press_ms")   {
            timing.tap_press_ms = v;
            timing.shift_press_ms = v;
            timing.enter_press_ms = v;
            timing.tab_press_ms = v;
            updated++;
        }
        else if (k == "key_gap_ms")     {
            timing.tap_release_ms = v;
            timing.shift_release_ms = v;
            timing.enter_release_ms = v;
            timing.tab_release_ms = v;
            updated++;
        }
        else if (k == "numlock_auto")   { timing.numlock_auto = v; updated++; }
        else if (k == "alt_ascii")      { timing.alt_ascii = v; updated++; }
        else if (k == "fence_auto")     { timing.fence_auto = v; updated++; }
        else if (k == "shake_auto")     { timing.shake_auto = v; updated++; }
        else if (k == "check_interval") { timing.check_interval = v; updated++; }
    }

    clamp_timing(timing);
    send_back(String("CFG:OK:") + updated);
    return true;
}

// BLE 连接回调
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        ble_connected = true;
        Serial.println("蓝牙已连接");
    }
    void onDisconnect(BLEServer* pServer) {
        ble_connected = false;
        Serial.println("蓝牙已断开，重新开始广播");
        pServer->getAdvertising()->start();
    }
};

// BLE 接收回调
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rxValue = pChar->getValue();
        if (rxValue.length() == 0) return;

        // 严格精确匹配急停指令 (避免正文或未来指令包含子串时误杀，无多余堆分配)
        if (rxValue.length() <= 10) {
            String cmd = rxValue;
            cmd.trim();
            cmd.toUpperCase();
            if (cmd == "{STOP}" || cmd == "{ABORT}") {
                portENTER_CRITICAL(&rxMux);
                rx_stream_buf = ""; // 清空未消费的输入缓冲
                abort_gen++;        // 代际递增
                portEXIT_CRITICAL(&rxMux);
                // 绝不在此跨线程调串口/HID或 delay，全权交由主循环安全处理
                return;
            }
        }

        portENTER_CRITICAL(&rxMux);
        rx_stream_buf += rxValue;
        portEXIT_CRITICAL(&rxMux);
    }
};

// ==============================================================================
// 扫码枪模块接收引擎 (过滤 \r\n + 纯时间静默断帧 + 去重防抖 + 分包聚合 <<SEQ:ID:M/N>>)
// ==============================================================================
static String qr_rx_raw = "";
static uint32_t last_qr_byte_time = 0;
static String last_scanned_code = "";
static uint32_t last_scanned_time = 0;

// 分包聚合会话管理器
struct QrMultiPartSession {
    bool active;
    String id;
    int total_parts;
    int received_count;
    uint32_t last_time;
    String parts[MAX_QR_PARTS];
    bool part_received[MAX_QR_PARTS];
};

static QrMultiPartSession qr_session = { false, "", 0, 0, 0 };

static void reset_qr_session(const char *reason) {
    if (qr_session.active) {
        Serial.printf("[Multi-QR Reset] 会话重置并清空缓存 (原因: %s, 原ID: %s, 已收: %d/%d)\n",
                      reason, qr_session.id.c_str(), qr_session.received_count, qr_session.total_parts);
    }
    qr_session.active = false;
    qr_session.id = "";
    qr_session.total_parts = 0;
    qr_session.received_count = 0;
    qr_session.last_time = 0;
    for (int p = 0; p < MAX_QR_PARTS; p++) {
        qr_session.parts[p] = "";
        qr_session.part_received[p] = false;
    }
}

static void process_qr_stream() {
    // 0. 检查分包超时 (30 秒无操作作废)
    if (qr_session.active && (millis() - qr_session.last_time > QR_SESSION_TIMEOUT_MS)) {
        reset_qr_session("30秒无操作超时");
    }

    // 1. 读取串口字节，直接过滤掉所有多余的 \r (0x0D) 和 \n (0x0A)
    while (SerialQR.available()) {
        char b = (char)SerialQR.read();
        if (b != '\r' && b != '\n') {
            qr_rx_raw += b;
        }
        last_qr_byte_time = millis();
    }

    // 2. 判定帧间静默超时：超过 100ms 无新字节视为扫码枪本帧发送完毕
    if (qr_rx_raw.length() > 0 && (millis() - last_qr_byte_time >= QR_SILENCE_TIMEOUT_MS)) {
        size_t len = qr_rx_raw.length();
        uint32_t now = millis();

        // 调试打印
        Serial.println("\n--------------------------------------------------------");
        Serial.printf("[QR Raw Stream] 收到 %d 字符 (已过滤 \\r\\n): \"%s\"\n", (int)len, qr_rx_raw.c_str());

        // 防重扫去重锁 (1.5 秒冷却，针对同码晃动连扫)
        bool is_duplicate = (qr_rx_raw == last_scanned_code);
        bool cooldown_active = (now - last_scanned_time < QR_DEDUP_INTERVAL_MS);

        if (is_duplicate && cooldown_active) {
            Serial.printf("[QR Anti-Duplicate] 同条码频繁连扫触发保护，丢弃 (剩余冷却: %u ms)\n",
                          (unsigned)(QR_DEDUP_INTERVAL_MS - (now - last_scanned_time)));
            Serial.println("--------------------------------------------------------");
            qr_rx_raw = "";
            return;
        }

        last_scanned_code = qr_rx_raw;
        last_scanned_time = now;

        // 3. 解析协议头 <<SEQ:ID:M/N>>
        bool is_seq_packet = false;
        String seq_id = "";
        int seq_m = 0, seq_n = 0;
        String payload = "";

        if (qr_rx_raw.startsWith("<<SEQ:")) {
            int end_tag = qr_rx_raw.indexOf(">>");
            if (end_tag != -1) {
                String meta = qr_rx_raw.substring(6, end_tag);
                int colon = meta.indexOf(':');
                int slash = meta.indexOf('/');
                if (colon != -1 && slash != -1 && slash > colon) {
                    seq_id = meta.substring(0, colon);
                    seq_id.trim();
                    seq_m = meta.substring(colon + 1, slash).toInt();
                    seq_n = meta.substring(slash + 1).toInt();
                    payload = qr_rx_raw.substring(end_tag + 2);

                    if (seq_id.length() > 0 && seq_m >= 1 && seq_n >= 1 &&
                        seq_m <= seq_n && seq_n <= MAX_QR_PARTS) {
                        is_seq_packet = true;
                    }
                }
            }
        }

        if (!is_seq_packet) {
            // ================= 普通单码逻辑 =================
            // 收到非分包普通码，若先前有未完成的 SEQ 队列，立刻清空刷新
            if (qr_session.active) {
                reset_qr_session("收到非分包普通条码");
            }

            portENTER_CRITICAL(&rxMux);
            rx_stream_buf += qr_rx_raw;
            portEXIT_CRITICAL(&rxMux);

            Serial.printf("[QR Input Engine] 普通单码已入队打字 (长度: %d 字符)\n", (int)qr_rx_raw.length());
            Serial.println("--------------------------------------------------------");
            qr_rx_raw = "";
            return;
        }

        // ================= 分包聚合逻辑 <<SEQ:ID:M/N>> =================
        // 新任务到达，清空旧队列
        if (qr_session.active && qr_session.id != seq_id) {
            reset_qr_session("新任务ID到达");
        }

        if (!qr_session.active) {
            qr_session.active = true;
            qr_session.id = seq_id;
            qr_session.total_parts = seq_n;
            qr_session.received_count = 0;
            for (int p = 0; p < MAX_QR_PARTS; p++) {
                qr_session.parts[p] = "";
                qr_session.part_received[p] = false;
            }
            Serial.printf("[Multi-QR Start] 开启新分包会话: ID=%s, 总包数=%d\n", seq_id.c_str(), seq_n);
        }

        qr_session.last_time = now;
        int slot = seq_m - 1;

        if (!qr_session.part_received[slot]) {
            qr_session.part_received[slot] = true;
            qr_session.parts[slot] = payload;
            qr_session.received_count++;
            Serial.printf("[Multi-QR Packet] 收到分包 %d/%d (正文 %d 字符) | 当前进度: %d/%d\n",
                          seq_m, seq_n, (int)payload.length(), qr_session.received_count, seq_n);
        } else {
            qr_session.parts[slot] = payload;
            Serial.printf("[Multi-QR Packet] 重新覆盖分包 %d/%d | 当前进度: %d/%d\n",
                          seq_m, seq_n, qr_session.received_count, seq_n);
        }

        // 打印收件状态
        Serial.print("  [收件清单] ");
        for (int p = 1; p <= qr_session.total_parts; p++) {
            if (qr_session.part_received[p - 1]) Serial.printf("[%d:OK] ", p);
            else Serial.printf("[%d:缺] ", p);
        }
        Serial.println();

        // 检查集齐
        if (qr_session.received_count == qr_session.total_parts) {
            String full_text = "";
            for (int p = 0; p < qr_session.total_parts; p++) {
                full_text += qr_session.parts[p];
            }

            portENTER_CRITICAL(&rxMux);
            rx_stream_buf += full_text;
            portEXIT_CRITICAL(&rxMux);

            Serial.printf("[Multi-QR Complete] 全部 %d/%d 集齐！拼接总长 %d 字符，已推入打字队列并执行！\n",
                          qr_session.total_parts, qr_session.total_parts, (int)full_text.length());
            Serial.println("--------------------------------------------------------");

            reset_qr_session("完成拼装入队");
        } else {
            Serial.printf("  >> 还差 %d 张二维码，等待扫描剩余卡片...\n",
                          qr_session.total_parts - qr_session.received_count);
            Serial.println("--------------------------------------------------------");
        }

        qr_rx_raw = "";
    }
}

// ==============================================================================
// 主打字消费引擎 (边打边收、多源合并、输入法自愈)
// ==============================================================================
static void type_single_char(uint32_t codepoint) {
    if (seen_gen != abort_gen) return;

    if (codepoint == 0x0A || codepoint == 0x0D) { // 回车换行 -> 物理 Enter 键
        tap_enter_key();
        return;
    }
    if (codepoint == 0x09) { // Tab 制表符
        tap_tab_key();
        return;
    }

    // 1. 若开启 alt_ascii，可打印 ASCII 字符直接走 Alt 码彻底绕过输入法
    if (timing.alt_ascii == 1.0f && codepoint >= 0x20 && codepoint <= 0x7E) {
        send_alt_code((uint16_t)codepoint);
        if (timing.ascii_gap_ms > 0.0f) delay_abortable(timing.ascii_gap_ms);
        return;
    }

    // 2. 常规 ASCII 物理按键
    if (codepoint < 0x80) {
        KeyMap km = get_ascii_keymap((char)codepoint);
        if (km.key != 0) {
            if (km.shift) tap_shift_key(km.key);
            else tap_key(MOD_NONE, km.key);
            if (timing.ascii_gap_ms > 0.0f) delay_abortable(timing.ascii_gap_ms);
            return;
        }
    }

    // 3. 中文 Unicode -> GBK -> Alt Code
    uint16_t gbk = get_gbk_code(codepoint);
    if (gbk > 0) {
        send_alt_code(gbk);
        if (timing.cjk_gap_ms > 0.0f) delay_abortable(timing.cjk_gap_ms);
        return;
    }

    Serial.printf("跳过未识别字符: U+%04X\n", (unsigned)codepoint);
}

static void process_typing_stream() {
    static bool numlock_verified = false;
    static int typed_action_count = 0; // 输入函数触发后计数器 (仅在物理动作完成后计数)

    // 非阻塞获取临界区数据
    String work = "";
    portENTER_CRITICAL(&rxMux);
    if (rx_stream_buf.length() > 0) {
        work = rx_stream_buf;
        rx_stream_buf = "";
    }
    portEXIT_CRITICAL(&rxMux);

    static String pending = "";

    // 检查急停代际：若代际变更，安全复位
    if (seen_gen != abort_gen) {
        seen_gen = abort_gen;
        pending = "";
        send_ch9329_frame(MOD_NONE);
        send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
        current_mouse_btn = MOUSE_BTN_NONE;
        send_back("STOPPED");
        return; // 注意：绝对不碰 rx_stream_buf！后续新文字将完整保留并在下一轮 loop 处理
    }

    if (work.length() > 0) pending += work;
    if (pending.length() == 0) return;

    size_t i = 0;
    while (i < pending.length()) {
        // 每步循环前检查急停
        if (seen_gen != abort_gen) {
            seen_gen = abort_gen;
            pending = "";
            send_ch9329_frame(MOD_NONE);
            send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
            current_mouse_btn = MOUSE_BTN_NONE;
            send_back("STOPPED");
            return;
        }

        char c = pending[i];

        // 标签解析 {KEY}
        if (c == '{') {
            int end_idx = pending.indexOf('}', i + 1);
            if (end_idx == -1) {
                // 如果未找到 '}'，但在缓冲区里已经遇到了换行符，说明这只是普通正文中的左花括号，不等待 '}'
                if (pending.indexOf('\n', i) != -1 || pending.indexOf('\r', i) != -1) {
                    type_single_char('{');
                    i++;
                    continue;
                }
                pending = pending.substring(i);
                return;
            }

            String token = pending.substring(i, end_idx + 1);
            String upper = token;
            upper.toUpperCase();

            // 0. 流内急停标签 {STOP} 或 {ABORT}
            if (upper == "{STOP}" || upper == "{ABORT}") {
                seen_gen = ++abort_gen;
                pending = "";
                send_ch9329_frame(MOD_NONE);
                send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
                current_mouse_btn = MOUSE_BTN_NONE;
                send_back("STOPPED");
                return;
            }

            // 1. 完成回执 {DONE} 或 {DONE:<id>}
            if (upper.startsWith("{DONE")) {
                String body = token.substring(1, token.length() - 1);
                body.toUpperCase();
                send_back(body);
                delay_abortable(60.0f);
                send_back(body);
                i = end_idx + 1;
                continue;
            }

            // 2. 状态查询 {NUM?}
            if (upper == "{NUM?}") {
                uint8_t usb_enum = 0, led = 0;
                if (ch9329_get_info(usb_enum, led) && usb_enum == 0x01) {
                    bool num = (led & 0x01) != 0;
                    bool caps = (led & 0x02) != 0;
                    char resp[64];
                    snprintf(resp, sizeof(resp), "NUMLOCK: %s | CAPSLOCK: %s", num ? "ON" : "OFF", caps ? "ON" : "OFF");
                    send_back(String(resp));
                } else {
                    send_back("NUMLOCK: 未知(RX未接/未枚举)");
                }
                i = end_idx + 1;
                continue;
            }

            // 3. 延时等待标签 {DELAY:ms}
            if (upper.startsWith("{DELAY:") && upper.endsWith("}")) {
                int msVal = upper.substring(7, upper.length() - 1).toInt();
                msVal = constrain(msVal, 0, 10000); // 保护最大 10 秒
                if (msVal > 0) {
                    if (!delay_abortable((float)msVal)) {
                        seen_gen = abort_gen;
                        pending = "";
                        send_ch9329_frame(MOD_NONE);
                        send_ch9329_mouse(MOUSE_BTN_NONE, 0, 0, 0);
                        current_mouse_btn = MOUSE_BTN_NONE;
                        send_back("STOPPED");
                        return;
                    }
                }
                i = end_idx + 1;
                continue;
            }

            // 4. 鼠标动作标签 (左键/右键/中键/双击/长按/释放/滚轮/位移)
            if (try_execute_mouse_action(token)) {
                i = end_idx + 1;
                continue;
            }

            // 5. 单键或组合键敲击 {TAB}, {ENTER}, {CTRL+C}, {ALT+F4}, {SHIFT+INSERT} 等
            if (try_execute_key_or_combo(token)) {
                i = end_idx + 1;

                // 物理按键彻底执行完毕后的后置计数与安全自愈检查
                typed_action_count++;
                if (timing.check_interval > 0.0f && typed_action_count >= (int)timing.check_interval) {
                    typed_action_count = 0;
                    execute_safe_health_check();
                }

                continue;
            }

            // 5. 调参命令 {CFG:...}
            if (upper.startsWith("{CFG") && handle_cfg_command(token)) {
                i = end_idx + 1;
                continue;
            }

            // 6. 设备名称命令 {NAME:...}
            if (upper.startsWith("{NAME") && handle_name_command(token)) {
                i = end_idx + 1;
                continue;
            }

            // 未知标签：退化为打 '{' 单字
            type_single_char('{');
            i++;
            continue;
        }

        // UTF-8 字符解码
        uint8_t b0 = (uint8_t)pending[i];
        size_t need_bytes = 1;
        uint32_t codepoint = 0;

        if (b0 < 0x80) {
            need_bytes = 1;
            codepoint = b0;
        } else if ((b0 & 0xE0) == 0xC0) {
            need_bytes = 2;
        } else if ((b0 & 0xF0) == 0xE0) {
            need_bytes = 3;
        } else if ((b0 & 0xF8) == 0xF0) {
            need_bytes = 4;
        } else {
            i++;
            continue;
        }

        if (i + need_bytes > pending.length()) {
            pending = pending.substring(i);
            return;
        }

        if (need_bytes == 2) {
            codepoint = ((b0 & 0x1F) << 6) | ((uint8_t)pending[i + 1] & 0x3F);
        } else if (need_bytes == 3) {
            codepoint = ((b0 & 0x0F) << 12) | (((uint8_t)pending[i + 1] & 0x3F) << 6) | ((uint8_t)pending[i + 2] & 0x3F);
        } else if (need_bytes == 4) {
            codepoint = ((b0 & 0x07) << 18) | (((uint8_t)pending[i + 1] & 0x3F) << 12) | (((uint8_t)pending[i + 2] & 0x3F) << 6) | ((uint8_t)pending[i + 3] & 0x3F);
        }

        // 遇到首个实际字符前自愈键盘指示灯
        if (codepoint >= 0x20 && !numlock_verified) {
            numlock_verified = true;
            if (timing.numlock_auto == 1.0f) {
                if (ensure_keyboard_state()) {
                    Serial.println("键盘状态自愈: 已确保 NumLock=开 且 CapsLock=关");
                } else {
                    Serial.println("键盘状态自愈: 跳过 (RX 未接或未枚举)");
                }
            }
        }

        type_single_char(codepoint);
        i += need_bytes;

        // 单个字符物理上屏彻底结束后的后置计数与安全自愈检查
        typed_action_count++;
        if (timing.check_interval > 0.0f && typed_action_count >= (int)timing.check_interval) {
            typed_action_count = 0;
            execute_safe_health_check();
        }

        taskYIELD();
    }

    pending = "";
}

// ==============================================================================
// Arduino setup() & loop()
// ==============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================================");
    Serial.println("ESP32-C3 键盘模拟器 + 扫码模块扩展版 v9.0 启动中...");

    // 1. 初始化 CH9329 键盘串口 (UART1: TX GPIO0, RX GPIO1)
    Serial1.begin(CH9329_BAUDRATE, SERIAL_8N1, CH9329_RX_PIN, CH9329_TX_PIN);
    Serial.printf("CH9329 串口初始化完成 (TX: GPIO%d, RX: GPIO%d)\n", CH9329_TX_PIN, CH9329_RX_PIN);

    // 2. 初始化扫码枪串口 (UART0: RX GPIO3, TX 无需接设为 -1)
    SerialQR.begin(QR_BAUDRATE, SERIAL_8N1, QR_RX_PIN, QR_TX_PIN);
    Serial.printf("扫码模块串口初始化完成 (RX: GPIO%d, 波特率: %d)\n", QR_RX_PIN, QR_BAUDRATE);

    // 3. 加载 GBK 字库
    init_gbk_data();

    // 4. 从 NVS 读取保存的时序参数与设备名称
    load_timing_from_nvs();
    Serial.println("当前参数: " + describe_timing());
    load_ble_name_from_nvs();
    Serial.printf("当前蓝牙广播名称: \"%s\"\n", current_ble_name.c_str());

    // 5. 初始化 BLE Nordic UART 服务
    BLEDevice::init(current_ble_name.c_str());
    BLEDevice::setMTU(517);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.printf("BLE 服务已就绪，正在广播 \"%s\"...\n", current_ble_name.c_str());
    Serial.println("系统初始化完毕，准备接收 BLE 打字数据与扫码枪输入");
    Serial.println("========================================================\n");
}

void loop() {
    // 1. 监听扫码枪输入
    process_qr_stream();

    // 2. 处理打字队列
    process_typing_stream();

    delay(2);
}
