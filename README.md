# ESP32-C3 SuperMini 单通道信号发生器

基于 Arduino 框架（Arduino IDE 2.x）的单通道信号发生器固件。默认通过 **BLE** 与电脑
通信：电脑用 Chrome/Edge 打开本目录下的 `panel.html`（Web Bluetooth），无需安装任何
软件，也不用断开电脑自己的 WiFi。

> **传输层选择**：固件顶部 `#define USE_WIFI_AP 0`。
> `0` = BLE + PC 端页面（默认）；`1` = 原来的 WiFi 热点 + 固件内嵌网页。
> 两者共用 C3 的单 2.4GHz 射频，同时开启需要分时共存（RAM 与稳定性都更紧张），
> 所以做成二选一。

> **网络栈说明（仅 `USE_WIFI_AP = 1` 时相关）**：本固件不使用 Arduino 的
> `WiFi`/`WebServer` 库，而是直接调用 ESP-IDF 原生 `esp_wifi` + `esp_http_server`。
> 原因是 Arduino core 3.x 的 `WebServer` 在 ESP32-C3 上有一个已知崩溃 bug（打开网页时
> `Guru Meditation Error: Illegal instruction`，崩溃点位于 `NetworkClientSocketHandle`
> 的 shared_ptr 赋值），原生 API 可完全绕开该问题，且无需安装任何额外库。

## 功能

- 单路输出：GPIO1（CH0），支持全部 6 种模式
- 模式 1（PWM）：频率 0~2kHz（0.1Hz 步进）、占空比 0~100%（1% 步进），随时可调
  - ≥10Hz 用硬件 LEDC；<10Hz 用软件定时生成（esp_timer，µs 级抖动）
  - 0Hz = 恒定低电平
- 模式 2（低电平循环）：x 秒高阻，y 秒低电平，循环；x/y 步进 0.1s，随时可调
- 模式 3（大周期循环）：每大周期 z 秒内，先拉低 y 秒、再释放 x 秒，共 a 次拉低
  （之间 a-1 次释放），剩余 `z - (a×y + (a-1)×x)` 秒全释放，然后进入下一大周期；
  x/y/z/a 随时可调
- 模式 4（(PWM/拉低)循环）：x 秒按设定频率/占空比输出 PWM，之后 y 秒持续拉低，
  往复循环；x/y 步进 0.1s，随时可调
- 模式 5（(PWM/拉低)大周期循环）：每大周期 z 秒内，先拉低 y 秒、再输出 PWM x 秒，
  共 a 次拉低（之间 a-1 次 PWM），剩余 `z - (a×y + (a-1)×x)` 秒全输出 PWM，然后进入下一大周期；
  x/y/z/a 随时可调
- 参数自动校验：模式 3/5 要求 `z ≥ a×y + (a-1)×x`
- 配置可保存到 Flash（NVS），断电不丢失
- WiFi 发射功率可调：8.5 / 11 / 13 / 15 / 20 dBm，选择后立即生效并存入 NVS
  （**仅 `USE_WIFI_AP = 1` 时可用**；C3 SuperMini 板载天线匹配较差，高功率下可能
  出现"能搜到但连不上/秒断"，调低发射功率可改善连接稳定性；出厂默认 8.5dBm）

## 文件结构

```
SignalGenerator.ino     主程序：命令解析、BLE GATT 服务（或 WiFi AP + Web 服务器）、
                        NVS 存取
config.h                引脚、参数范围、模式定义、配置结构体
output_engine.h/.cpp    输出引擎：LEDC PWM + 软件相位机（模式2/3/4/5、低频PWM）
panel.html              PC 端控制面板（Web Bluetooth），用 Chrome/Edge 通过 localhost 打开
```

## Arduino IDE 配置

1. 安装开发板支持包（若未安装）：
   - 文件 → 首选项 → 附加开发板管理器网址，填入：
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - 工具 → 开发板 → 开发板管理器，搜索 **esp32**，安装
     **esp32 by Espressif Systems**（建议 3.x）。
2. 工具 → 开发板 → **ESP32C3 Dev Module**。
3. 工具 → Partition Scheme → **Huge APP (3MB No OTA/1MB SPIFFS)**。
   （固件约 1.2MB，必须用大分区方案，否则报 "Sketch too big"）
4. 工具 → 端口，选择 SuperMini 对应的串口（需装好 USB 驱动，通常为
   CH340/CP210x 或 USB CDC）。
5. 打开本目录下的 `SignalGenerator.ino`，点击上传。

> 代码同时兼容 Arduino core 2.x 与 3.x 的 LEDC API（自动判断）。
> 网络部分直接使用 ESP-IDF 原生 API，与 Arduino core 2.x/3.x 均兼容，
> 因此建议继续使用当前已安装的 3.x 核心，无需降级。

## 硬件接线

```
ESP32-C3 SuperMini          BC547B ×1
GPIO1 ──[1kΩ]──────────── 基极 B
发射极 E ──[200Ω]──────── GND（必须与 ESP32 共地）
集电极 C ──[10kΩ]──────── +16V（上拉电源）
集电极 C ──────────────── 信号输出点 OUT
```

输出端按模式接法：

| 模式 | 接法 |
|---|---|
| 模式 1/4/5（PWM 相关） | 集电极经 10kΩ 上拉到 16V，输出为 PWM 波；上拉电源决定了 PWM 高边电压 |
| 模式 2/3 | 同一电路即可：高阻 = 三极管截止（集电极被 10k 上拉到 16V），低 = 三极管导通拉低 |

注意：

- **反相补偿**：PWM 模式下面板占空比指"集电极高电平占比"，
  固件已自动反相（GPIO 占空比 = 100% − 面板占空比）。
- **不是电气隔离**：BC547B 只是电平转换/开关，外部电路必须与 ESP32 共地；
  需要真隔离请换光耦（如 PC817）。
- **只能拉低**：模式 2/3 中"高阻"= 三极管截止。外源必须是
  电阻上拉/限流类负载；推挽输出源不能直接接，否则导通时会短路。
- 16V 电源、上拉电阻与板子 3.3V 逻辑分开供电，仅共地。
- 上拉电阻建议 680Ω~4.7kΩ（16V 时电流 16mA~3.4mA，都在 BC547B 安全范围）。

## 使用方法（BLE 模式，默认）

1. 开发板上电，串口打印 `BLE: SignalGen-ESP32-1 已开始广播`。
2. 电脑上用 **Chrome 或 Edge** 打开本目录的 `panel.html`。
   Web Bluetooth 只在安全上下文（`https://` 或 `localhost`）可用，直接双击
   `file://` 打开会被拦，先在项目目录起一个本地静态服务：

   ```
   python -m http.server 8000
   ```

   然后浏览器访问 `http://localhost:8000/panel.html`。
3. 点"连接设备"，在弹出的选择框里选 **SignalGen-ESP32-1**（首次需要授权；
   之后可用"重新连接"直接连回，不必重新选）。
4. 选择模式 → 填写参数 → 点"应用"立即生效；"保存设置" 写入 NVS，下次上电恢复。

> 手机端：安卓 Chrome 同样支持 Web Bluetooth；iOS Safari 不支持，需把
> `USE_WIFI_AP` 改成 1 用 WiFi 模式。

## 使用方法（WiFi 模式，`USE_WIFI_AP = 1`）

1. 开发板上电后，等待几秒出现热点 **SignalGen-ESP32**（无密码；可在
   `SignalGenerator.ino` 顶部修改 SSID/密码）。
2. 电脑/手机连接该热点，浏览器打开 `http://192.168.4.1`。
3. 单通道：选择模式 → 填写参数 → 点"应用"立即生效；
   "保存设置" 将参数写入 NVS，下次上电自动恢复。
4. 若连接不稳，把工具栏"发射功率"下拉从 20dBm 逐步调低（建议先试 8.5dBm），
   改动会立即生效并保存，无需重新烧录。

## BLE 协议

服务 `8f2d0001-9b6a-4d1e-8c3f-6a1b2c3d4e5f`，两个特征：

| 特征 | 属性 | 用途 |
|---|---|---|
| `8f2d0002-…` | Write | 客户端 → 板子，命令 |
| `8f2d0003-…` | Notify | 板子 → 客户端，回复 |

命令都是换行结尾的文本行，与 HTTP 查询串格式一致：

| 命令 | 回复 |
|---|---|
| `set?ch=0&mode=1&f=1000&duty=50`（未给出的键沿用当前值） | `OK` 或错误文本 |
| `status` | 状态 JSON（同原 `/api/status`） |
| `save` | `已保存` |

> ATT 默认 MTU 只有 23（20 字节有效载荷），所以收发两端都按 `\n` 拼包，
> 固件发送时按协商后的 MTU 自动分片，`panel.html` 发送时固定 20 字节一片。
> Windows 一般会协商到 247，此时一条回复通常是一包发完。

## 常见故障

### 烧录后打开网页/操作面板就 `Guru Meditation Error: Illegal instruction` 重启

（WiFi 模式的现象；BLE 模式不经过 `WebServer`，一般不会遇到。）

现象：热点能连、网页偶尔能开，但点击"应用"或"刷新"时崩溃重启。

原因：`platform.local.txt`（位于
`%LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\<版本>\platform.local.txt`）
如果覆盖了 `compiler.cpp.flags` / `compiler.c.flags`，会丢掉 IDF 提供的
`@flags/cpp_flags` 响应文件，其中包含 C3 必需的
`-march=rv32imc_zicsr_zifencei`。缺少该参数时，编译器按工具链默认架构
（带 F/D 浮点扩展）生成浮点指令，而 ESP32-C3 **没有 FPU**，
一执行就触发非法指令崩溃。

修复：确保 `platform.local.txt` 只保留并行编译设置，不要覆盖编译参数：

```
build.jobs=6
```

改完需**重启 Arduino IDE** 再重新上传。

## 设计说明与已知限制

- 10Hz 以下 PWM、模式 2/3 由 esp_timer 软件相位机生成，边沿抖动约
  10~50µs；对 0.1s 量级的时序和 1% 占空比（10Hz 以下时 ≥1ms 脉宽）足够。
- 2kHz、占空比 1% 时脉宽 5µs，BC547B 的饱和存储时间（约 0.5~1µs）会
  造成极端占空比下的边缘非线性；对精度要求苛刻时可换 2N7002/AO3400 MOSFET。
- 模式切换/参数修改采用"立即生效，从起点重新开始"策略，避免半周期残留。
- BLE 模式下命令往返延迟由连接间隔决定（固件偏好 7.5~15ms），面板每 2s
  拉一次状态，操作手感与原来 HTTP 轮询基本一致。
- BLE 断开后输出不受影响，配置继续按最后一次生效的参数运行，重连即可继续控制。
- Web Bluetooth 只在 Chrome/Edge 等 Chromium 系浏览器可用，且必须走
  `https://` 或 `localhost`；Firefox、Safari 不支持。
