#pragma once
#include <Arduino.h>

// ===================== 输出引脚 =====================
// 单通道版本：GPIO1（CH0）输出，支持全部模式
static const uint8_t CH_PINS[1] = { 1 };
constexpr int CHANNEL_COUNT = 1;

// ===================== 参数范围 =====================
constexpr float MAX_FREQ_HZ = 2000.0f;   // PWM 上限 2kHz
constexpr float MIN_FREQ_HZ = 0.1f;      // PWM 最低频率（软件定时生成）
constexpr float LEDC_MIN_HZ = 10.0f;     // >= 此频率用硬件 LEDC，低于用软件定时
constexpr float MIN_TIME_S  = 0.1f;      // x/y 最小步进 0.1s
constexpr float MAX_TIME_S  = 3600.0f;   // x/y 上限 1 小时
constexpr float MAX_Z_S     = 86400.0f;  // 大周期 z 上限 24 小时
constexpr uint32_t MAX_A    = 10000;     // 小循环次数上限

// ===================== 输出模式 =====================
enum OutputMode : uint8_t {
  MODE_OFF       = 0,  // 关闭：三极管截止，输出高阻
  MODE_PWM       = 1,  // PWM：0~2kHz，占空比 0~100%（1% 步进）
  MODE_LOW_PULSE = 2,  // 模式2（CH0~2）：x 秒高阻，y 秒低电平，循环
  MODE_BIG_CYCLE = 3,  // 模式3（CH0~2）：大周期 z，先低 y 再高阻 x，共 a 次拉低
  MODE_PWM_LOW   = 4,  // 模式4：x 秒 PWM + y 秒拉低，循环
  MODE_PWM_BIG   = 5,  // 模式5：大周期 z，a 次(y 拉低 + x PWM)，剩余全 PWM
};

// ===================== 通道配置 =====================
struct ChannelConfig {
  uint8_t  mode;
  float    freqHz;   // PWM 频率，0 表示恒定低电平
  float    dutyPct;  // 有效输出占空比（集电极高电平占比），0~100
  float    xSec;     // 悬空时长（s）
  float    ySec;     // 低电平时长（s）
  float    zSec;     // 大周期时长（s）
  uint32_t a;        // 小循环次数
};

// 校验并量化参数（频率 0.1Hz 步进、占空比 1% 步进、时间 0.1s 步进）。
// 返回 nullptr 表示通过，否则返回错误信息（静态字符串）。
const char* sanitizeConfig(ChannelConfig& cfg);
