#include "output_engine.h"
#include "esp_timer.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>

// Arduino core >= 3.0 使用新的 ledc 引脚 API
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define LEDC_NEW_API 1
#endif

// ===================== 运行时状态 =====================
struct ChanRuntime {
  ChannelConfig cfg;
  esp_timer_handle_t timer = nullptr;
  bool timerCreated = false;
  bool usingLedc = false;
  int ledcRes = 0;
  uint8_t phase = 0;      // 软件相位机当前相位
  uint8_t subPhase = 0;   // 软件 PWM 段内部边沿相位
  float pwmRemain = 0.0f; // 软件 PWM 段剩余时长（s）
  uint32_t loopCount = 0; // 模式3/5 已完成的小循环数
  bool level = false;     // 当前 GPIO 电平（false=三极管截止/悬空）
};
static ChanRuntime s_ch[CHANNEL_COUNT];

// 保护 s_ch[] 与 LEDC/GPIO 状态：esp_timer 回调、loop 任务、
// httpd 任务（读取配置）都会访问，必须互斥。
static SemaphoreHandle_t s_mtx = nullptr;

// ===================== 参数校验 / 量化 =====================
const char* sanitizeConfig(ChannelConfig& c) {
  switch (c.mode) {
    case MODE_OFF:
      return nullptr;

    case MODE_PWM: {
      c.freqHz = roundf(c.freqHz * 10.0f) / 10.0f;
      if (c.freqHz != 0.0f && (c.freqHz < MIN_FREQ_HZ || c.freqHz > MAX_FREQ_HZ))
        return "PWM 频率需在 0.1~2000 Hz（0 表示恒定低电平）";
      c.dutyPct = roundf(c.dutyPct);  // 1% 步进
      if (c.dutyPct < 0.0f || c.dutyPct > 100.0f)
        return "占空比需在 0~100%";
      return nullptr;
    }

    case MODE_LOW_PULSE: {
      c.xSec = roundf(c.xSec * 10.0f) / 10.0f;  // 0.1s 步进
      c.ySec = roundf(c.ySec * 10.0f) / 10.0f;
      if (c.xSec < MIN_TIME_S || c.xSec > MAX_TIME_S)
        return "x 需在 0.1~3600 s";
      if (c.ySec < MIN_TIME_S || c.ySec > MAX_TIME_S)
        return "y 需在 0.1~3600 s";
      return nullptr;
    }

    case MODE_BIG_CYCLE: {
      c.xSec = roundf(c.xSec * 10.0f) / 10.0f;
      c.ySec = roundf(c.ySec * 10.0f) / 10.0f;
      if (c.xSec < MIN_TIME_S || c.xSec > MAX_TIME_S)
        return "x 需在 0.1~3600 s";
      if (c.ySec < MIN_TIME_S || c.ySec > MAX_TIME_S)
        return "y 需在 0.1~3600 s";
      if (c.a < 1 || c.a > MAX_A)
        return "a 需为 1~10000 的整数";
      c.zSec = roundf(c.zSec * 10.0f) / 10.0f;
      float need = c.ySec * (float)c.a + c.xSec * (float)(c.a - 1);
      if (c.zSec + 0.0001f < need)
        return "z 必须大于等于 a×y+(a-1)×x";
      if (c.zSec > MAX_Z_S)
        return "z 过大（最大 86400 s）";
      return nullptr;
    }

    case MODE_PWM_LOW:
    case MODE_PWM_BIG: {
      c.freqHz = roundf(c.freqHz * 10.0f) / 10.0f;
      if (c.freqHz != 0.0f && (c.freqHz < MIN_FREQ_HZ || c.freqHz > MAX_FREQ_HZ))
        return "PWM 频率需在 0.1~2000 Hz（0 表示恒定低电平）";
      c.dutyPct = roundf(c.dutyPct);
      if (c.dutyPct < 0.0f || c.dutyPct > 100.0f)
        return "占空比需在 0~100%";
      c.xSec = roundf(c.xSec * 10.0f) / 10.0f;
      c.ySec = roundf(c.ySec * 10.0f) / 10.0f;
      if (c.xSec < MIN_TIME_S || c.xSec > MAX_TIME_S)
        return "x 需在 0.1~3600 s";
      if (c.ySec < MIN_TIME_S || c.ySec > MAX_TIME_S)
        return "y 需在 0.1~3600 s";
      if (c.mode == MODE_PWM_BIG) {
        if (c.a < 1 || c.a > MAX_A)
          return "a 需为 1~10000 的整数";
        c.zSec = roundf(c.zSec * 10.0f) / 10.0f;
        float need = c.ySec * (float)c.a + c.xSec * (float)(c.a - 1);
        if (c.zSec + 0.0001f < need)
          return "z 必须大于等于 a×y+(a-1)×x";
        if (c.zSec > MAX_Z_S)
          return "z 过大（最大 86400 s）";
      }
      return nullptr;
    }

    default:
      return "未知模式";
  }
}

// ===================== LEDC 辅助（反相补偿） =====================
// 计算某频率下可用的占空比分辨率（C3 最大 14bit）
static int computeResolution(float freqHz) {
  int res = (int)floorf(log2f(80000000.0f / freqHz));
  if (res < 8) res = 8;
  if (res > 14) res = 14;
  return res;
}

static bool attachLedc(int ch, float freq, int res) {
#ifdef LEDC_NEW_API
  return ledcAttach(CH_PINS[ch], (uint32_t)freq, (uint8_t)res);
#else
  bool ok = ledcSetup(ch, (uint32_t)freq, (uint8_t)res);
  if (ok) ledcAttachPin(CH_PINS[ch], ch);
  return ok;
#endif
}

static bool changeLedcFreq(int ch, float freq, int res) {
#ifdef LEDC_NEW_API
  return ledcChangeFrequency(CH_PINS[ch], (uint32_t)freq, (uint8_t)res);
#else
  return ledcChangeFrequency(ch, (uint32_t)freq, (uint8_t)res);
#endif
}

static void releaseLedc(int ch) {
#ifdef LEDC_NEW_API
  ledcDetach(CH_PINS[ch]);
#else
  ledcDetachPin(CH_PINS[ch]);
#endif
}

// 输出节点是集电极：GPIO 高 -> 三极管导通 -> 集电极低（反相）。
// 面板占空比指集电极高电平占比，因此 GPIO 占空比 = 100 - 面板占空比。
static void writeLedcDuty(int ch, float dutyPct, int res) {
  float gpioPct = 100.0f - dutyPct;
  uint32_t maxDuty = (1UL << res) - 1;
  uint32_t val = (uint32_t)roundf(gpioPct * 0.01f * (float)maxDuty);
  if (val > maxDuty) val = maxDuty;
#ifdef LEDC_NEW_API
  ledcWrite(CH_PINS[ch], val);
#else
  ledcWrite(ch, val);
#endif
}

// 软件 PWM 段的一个边沿：subPhase 0 = 集电极高(GPIO低)，1 = 集电极低(GPIO高)。
// 边沿时长超过本段剩余时间时截断，保证段总时长精确。
static float pwmEdge(ChanRuntime* r) {
  const ChannelConfig& c = r->cfg;
  float edge;
  if (r->subPhase == 0) {
    r->level = false;  // 集电极高，占面板占空比 dutyPct
    r->subPhase = 1;
    edge = c.dutyPct * 0.01f / c.freqHz;
  } else {
    r->level = true;   // 集电极低，占剩余 (100 - dutyPct)
    r->subPhase = 0;
    edge = (100.0f - c.dutyPct) * 0.01f / c.freqHz;
  }
  if (edge >= r->pwmRemain) {
    edge = r->pwmRemain;
    r->pwmRemain = 0.0f;
  } else {
    r->pwmRemain -= edge;
  }
  return edge;
}

// 软件 PWM 段开始：初始化剩余时长，返回第一步时长（边沿或整段静态电平）。
// 0Hz 或占空比 0% -> 整段恒拉低；占空比 100% -> 整段恒集电极高。
static float swPwmStart(ChanRuntime* r, float segSec) {
  const ChannelConfig& c = r->cfg;
  r->pwmRemain = segSec;
  r->subPhase = 0;
  if (c.freqHz <= 0.0f || c.dutyPct <= 0.0f) {
    r->pwmRemain = 0.0f;
    r->level = true;   // 恒拉低
    return segSec;
  }
  if (c.dutyPct >= 100.0f) {
    r->pwmRemain = 0.0f;
    r->level = false;  // 恒集电极高
    return segSec;
  }
  return pwmEdge(r);
}

// 进入 PWM 段：需要时挂载 LEDC 并写入占空比；返回本段时长
// （LEDC 整段时长，或软件路径的首个边沿时长）。
static float pwmSegmentStart(ChanRuntime* r, int ch, float segSec) {
  const ChannelConfig& c = r->cfg;
  if (!r->usingLedc && c.freqHz > 0.0f && c.freqHz >= LEDC_MIN_HZ) {
    r->ledcRes = computeResolution(c.freqHz);
    if (attachLedc(ch, c.freqHz, r->ledcRes)) {
      r->usingLedc = true;
    }
  }
  if (r->usingLedc) {
    writeLedcDuty(ch, c.dutyPct, r->ledcRes);
    return segSec;
  }
  return swPwmStart(r, segSec);
}

// 进入拉低段：摘除 LEDC，用纯 GPIO 高电平输出，
// 保证"拉低"段 GPIO 一定输出高电平（三极管导通 -> 集电极拉低）。
static float lowSegmentStart(ChanRuntime* r, int ch, float segSec) {
  if (r->usingLedc) {
    releaseLedc(ch);
    r->usingLedc = false;
    // ledcDetach 会清掉引脚的 periman 总线且不恢复 GPIO 注册，
    // 必须先重新 pinMode(OUTPUT)，否则后面的 digitalWrite 会静默失效，
    // 引脚停留在 GPIO 输出寄存器旧值（首次上电为 LOW）。
    pinMode(CH_PINS[ch], OUTPUT);
  }
  r->level = true;
  return segSec;
}

// ===================== 软件相位机 =====================
// 相位语义：
//   level = false -> GPIO 低 -> 三极管截止 -> 输出悬空/集电极高
//   level = true  -> GPIO 高 -> 三极管导通 -> 输出低（集电极低）
static float nextPhase(ChanRuntime* r) {
  const ChannelConfig& c = r->cfg;
  int ch = (int)(r - s_ch);
  switch (c.mode) {
    case MODE_PWM: {
      // 软件 PWM（f < LEDC_MIN_HZ）
      // level=false 是集电极高，对应面板占空比 dutyPct；
      // level=true 是集电极低，对应 (100 - dutyPct)。
      if (r->phase == 0) {
        r->level = false;  // 集电极高
        r->phase = 1;
        return c.dutyPct * 0.01f / c.freqHz;
      } else {
        r->level = true;   // 集电极低
        r->phase = 0;
        return (100.0f - c.dutyPct) * 0.01f / c.freqHz;
      }
    }

    case MODE_LOW_PULSE: {
      if (r->phase == 0) {
        r->level = false;  // 悬空 x 秒
        r->phase = 1;
        return c.xSec;
      } else {
        r->level = true;   // 低电平 y 秒
        r->phase = 0;
        return c.ySec;
      }
    }

    case MODE_BIG_CYCLE: {
      // 每个大周期：先拉低 y 秒，再释放 x 秒，共 a 次拉低（之间 a-1 次释放），
      // 剩余 z - (a*y + (a-1)*x) 秒全释放，然后进入下一大周期。
      // phase 0: 小循环拉低；phase 1: 小循环释放；phase 2: 大周期剩余释放
      if (r->phase == 0) {
        r->loopCount++;
        r->level = true;  // 三极管导通 -> 外部 PWM 被拉低
        if (r->loopCount >= c.a) {
          float rem = c.zSec - c.ySec * (float)c.a - c.xSec * (float)(c.a - 1);
          if (rem > 0.05f) {
            r->phase = 2;
          } else {
            // 无剩余时间：直接进入下一大周期的第一次拉低
            r->phase = 0;
            r->loopCount = 0;
          }
        } else {
          r->phase = 1;
        }
        return c.ySec;
      }
      if (r->phase == 1) {
        r->level = false;  // 三极管截止 -> 释放（悬空）
        r->phase = 0;
        return c.xSec;
      }
      // phase == 2：大周期剩余时间全释放，然后进入下一大周期
      r->phase = 0;
      r->loopCount = 0;
      r->level = false;
      return c.zSec - c.ySec * (float)c.a - c.xSec * (float)(c.a - 1);
    }

    case MODE_PWM_LOW: {
      // (PWM/拉低)循环：x 秒 PWM + y 秒拉低，往复。
      // phase 0: 拉低段结束/周期起点 -> 输出 PWM 段
      // phase 1: PWM 段进行中/结束 -> 输出拉低段
      if (r->phase == 1) {
        if (!r->usingLedc && r->pwmRemain > 0.0f) return pwmEdge(r);
        r->phase = 0;
        return lowSegmentStart(r, ch, c.ySec);
      }
      // phase == 0：输出 PWM 段
      r->phase = 1;
      return pwmSegmentStart(r, ch, c.xSec);
    }

    case MODE_PWM_BIG: {
      // (PWM/拉低)大周期循环：先拉低 y 秒再输出 PWM x 秒，共 a 次拉低
      // （之间 a-1 次 PWM），剩余 z-(a*y+(a-1)*x) 秒全 PWM，然后进入下一大周期。
      // phase 3: 周期起点 -> 第一次拉低；phase 1: 拉低段结束 -> PWM/剩余段
      // phase 0: PWM 段进行中/结束 -> 拉低段；phase 2: 剩余段结束 -> 下一周期
      if (r->phase == 3) {
        r->loopCount = 1;
        r->phase = 1;
        return lowSegmentStart(r, ch, c.ySec);
      }
      if (r->phase == 1) {
        if (r->loopCount < c.a) {
          r->phase = 0;
          return pwmSegmentStart(r, ch, c.xSec);
        }
        // a 次拉低完成 -> 剩余时间全 PWM
        float rem = c.zSec - c.ySec * (float)c.a - c.xSec * (float)(c.a - 1);
        if (rem > 0.05f) {
          r->phase = 2;
          return pwmSegmentStart(r, ch, rem);
        }
        // 无剩余时间：直接进入下一大周期第一次拉低
        r->loopCount = 1;
        r->phase = 1;
        return lowSegmentStart(r, ch, c.ySec);
      }
      if (r->phase == 0 || r->phase == 2) {
        // PWM 段（phase 0）或剩余 PWM 段（phase 2）进行中/结束
        if (!r->usingLedc && r->pwmRemain > 0.0f) return pwmEdge(r);
        if (r->phase == 0) {
          // PWM 段结束 -> 拉低段
          r->loopCount++;
          r->phase = 1;
          return lowSegmentStart(r, ch, c.ySec);
        }
        // phase == 2 剩余段结束 -> 下一大周期第一次拉低
        r->loopCount = 1;
        r->phase = 1;
        return lowSegmentStart(r, ch, c.ySec);
      }
    }

    default:
      return -1.0f;
  }
}

static void chanTimerCb(void* arg) {
  ChanRuntime* r = static_cast<ChanRuntime*>(arg);
  if (s_mtx && xSemaphoreTake(s_mtx, 0) != pdTRUE) {
    // 引擎正在被修改：稍后（2ms）再继续本相位，避免并发改配置
    if (r->timerCreated) esp_timer_start_once(r->timer, 2000);
    return;
  }
  int ch = (int)(r - s_ch);
  float dur = nextPhase(r);
  if (dur <= 0.0f) {
    digitalWrite(CH_PINS[ch], LOW);
  } else {
    // LEDC 模式下电平由 LEDC 占空比驱动（nextPhase 内已写入），不要用 digitalWrite 干扰
    if (!r->usingLedc) {
      digitalWrite(CH_PINS[ch], r->level ? HIGH : LOW);
    }
    if (r->timerCreated) esp_timer_start_once(r->timer, (uint64_t)(dur * 1e6f));
  }
  if (s_mtx) xSemaphoreGive(s_mtx);
}

// ===================== 对外接口 =====================
void engineInit() {
  if (s_mtx == nullptr) s_mtx = xSemaphoreCreateMutex();
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    esp_timer_create_args_t args = {};
    args.callback = chanTimerCb;
    args.arg = &s_ch[i];
    args.name = "siggen_chan";
    if (esp_timer_create(&args, &s_ch[i].timer) == ESP_OK) {
      s_ch[i].timerCreated = true;
    }
    s_ch[i].cfg = {};
    s_ch[i].cfg.mode = MODE_OFF;
    pinMode(CH_PINS[i], OUTPUT);
    digitalWrite(CH_PINS[i], LOW);
  }
}

static void applyChannelLocked(int ch, const ChannelConfig& cfg) {
  ChanRuntime* r = &s_ch[ch];
  if (r->timerCreated) esp_timer_stop(r->timer);

  r->cfg = cfg;
  const ChannelConfig& c = r->cfg;
  int pin = CH_PINS[ch];

  // ---- 关闭：三极管截止，输出悬空 ----
  if (c.mode == MODE_OFF) {
    releaseLedc(ch);
    r->usingLedc = false;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    return;
  }

  // ---- 纯 PWM：频率足够高时用硬件 LEDC（自动反相补偿） ----
  // 模式4/5 的 LEDC 由相位机按需挂载/摘除（拉低段用纯 GPIO 保证高电平）。
  if (c.mode == MODE_PWM && c.freqHz > 0.0f && c.freqHz >= LEDC_MIN_HZ) {
    r->ledcRes = computeResolution(c.freqHz);
    bool ledcOk = false;
    if (!r->usingLedc) {
      releaseLedc(ch);
      ledcOk = attachLedc(ch, c.freqHz, r->ledcRes);
    } else {
      ledcOk = changeLedcFreq(ch, c.freqHz, r->ledcRes);
    }
    if (ledcOk) {
      r->usingLedc = true;
      // 纯 PWM：LEDC 持续输出，无需定时器
      writeLedcDuty(ch, c.dutyPct, r->ledcRes);
      return;
    }
    // 配置失败则回退到软件路径
    releaseLedc(ch);
    r->usingLedc = false;
  }

  // ---- 其余情况：GPIO + 软件定时 ----
  releaseLedc(ch);
  r->usingLedc = false;
  pinMode(pin, OUTPUT);

  if (c.mode == MODE_PWM) {
    // 0Hz 或占空比 0% -> 集电极恒低（GPIO 恒高）
    if (c.freqHz == 0.0f || c.dutyPct <= 0.0f) {
      digitalWrite(pin, HIGH);
      return;
    }
    // 100% -> 集电极恒高（GPIO 恒低）
    if (c.dutyPct >= 100.0f) {
      digitalWrite(pin, LOW);
      return;
    }
    r->phase = 0;
    r->loopCount = 0;
    float dur = nextPhase(r);
    digitalWrite(pin, r->level ? HIGH : LOW);
    if (r->timerCreated) esp_timer_start_once(r->timer, (uint64_t)(dur * 1e6f));
    return;
  }

  if (c.mode == MODE_LOW_PULSE || c.mode == MODE_BIG_CYCLE ||
      c.mode == MODE_PWM_LOW || c.mode == MODE_PWM_BIG) {
    r->phase = (c.mode == MODE_PWM_BIG) ? 3 : 0;
    r->loopCount = 0;
    r->pwmRemain = 0.0f;
    r->subPhase = 0;
    float dur = nextPhase(r);
    if (dur <= 0.0f) {
      digitalWrite(pin, LOW);
      return;
    }
    // 模式4/5 的 PWM 段可能刚挂载了 LEDC，此时不要用 digitalWrite 干扰
    if (!r->usingLedc) {
      digitalWrite(pin, r->level ? HIGH : LOW);
    }
    if (r->timerCreated) esp_timer_start_once(r->timer, (uint64_t)(dur * 1e6f));
    return;
  }
}

void engineApplyChannel(int ch, const ChannelConfig& cfg) {
  if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
  applyChannelLocked(ch, cfg);
  if (s_mtx) xSemaphoreGive(s_mtx);
}

ChannelConfig engineGetChannel(int ch) {
  ChannelConfig c = {};
  if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
  c = s_ch[ch].cfg;
  if (s_mtx) xSemaphoreGive(s_mtx);
  return c;
}
