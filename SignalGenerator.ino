#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "config.h"
#include "output_engine.h"

// ===================== 传输层选择 =====================
// 1 = WiFi 热点 + 固件内嵌网页（原方案，面板从固件里取）
// 0 = BLE + PC 端 Web Bluetooth 页面（panel.html，页面放在电脑上）
// 两者共用同一 2.4GHz 射频，同时开启需要分时共存（RAM 与稳定性都更紧张），故二选一。
#define USE_WIFI_AP 0

#if USE_WIFI_AP
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_wifi.h>
// C3 上初始化 WiFi 后 USB CDC 串口可能失效，需重新使能 PHY 的 USB
#include "esp_private/phy.h"

// ===================== WiFi AP 配置 =====================
static const char* AP_SSID = "SignalGen-ESP32-1";
static const char* AP_PASS = "";  // 留空 = 无密码（长度 >=8 时启用 WPA2）
static httpd_handle_t s_httpd = nullptr;
#else
// ===================== BLE 配置 =====================
static const char* BLE_NAME     = "SignalGen-ESP32-1";
static const char* BLE_SVC_UUID = "8f2d0001-9b6a-4d1e-8c3f-6a1b2c3d4e5f";
static const char* BLE_RX_UUID  = "8f2d0002-9b6a-4d1e-8c3f-6a1b2c3d4e5f";  // 客户端写入命令
static const char* BLE_TX_UUID  = "8f2d0003-9b6a-4d1e-8c3f-6a1b2c3d4e5f";  // 服务端通知回复
#endif

// ===================== 全局对象 =====================
// 传输层回调运行在独立任务中，不能直接操作输出引擎：
// 配置先经队列交给 loop() 应用，回复也由 loop() 统一发送。
struct ApplyCmd {
  int ch;
  ChannelConfig cfg;
};
static QueueHandle_t s_cmdQueue = nullptr;

Preferences prefs;

#if USE_WIFI_AP
// ===================== WiFi 发射功率 =====================
// 单位 0.25dBm：34=8.5dBm, 44=11, 52=13, 60=15, 80=20。
// C3 SuperMini 天线匹配较差，高功率下易握手失败，降功率可改善连接稳定性。
static uint8_t s_txPowerUnits = 34;  // 默认 8.5dBm（首次上电即可稳定连接，可面板调高）
static const uint8_t TX_POWER_LEVELS[] = { 34, 44, 52, 60, 80 };

static uint8_t loadTxPowerUnits() {
  prefs.begin("siggen", true);
  uint8_t v = prefs.getUChar("wifiTx", 34);
  prefs.end();
  return v;
}

static void saveTxPowerUnits(uint8_t units) {
  prefs.begin("siggen", false);
  prefs.putUChar("wifiTx", units);
  prefs.end();
}

static bool isValidTxPower(int units) {
  for (size_t i = 0; i < sizeof(TX_POWER_LEVELS) / sizeof(TX_POWER_LEVELS[0]); i++) {
    if ((int)TX_POWER_LEVELS[i] == units) return true;
  }
  return false;
}

// ===================== 网页面板 =====================
static const char INDEX_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>信号发生器控制面板</title>
<style>
body{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;max-width:1000px;margin:16px auto;padding:0 12px;background:#f5f6f8;color:#222}
h1{font-size:20px;margin:8px 0}
.note{background:#fff8e1;border:1px solid #eed484;border-radius:8px;padding:8px 12px;font-size:13px;line-height:1.6;margin-bottom:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:12px}
.card{background:#fff;border:1px solid #ddd;border-radius:10px;padding:12px 14px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.head{font-weight:600;margin-bottom:8px;font-size:15px}
.row{display:none;align-items:center;justify-content:space-between;margin:6px 0;font-size:14px}
.row.mode-row{display:flex}
.show-pwm .pwm,.show-pat .pat,.show-big .big{display:flex}
.card input,.card select{width:130px;padding:4px 6px;border:1px solid #bbb;border-radius:6px;box-sizing:border-box}
.card button{margin-top:10px;width:100%}
.bar{margin:14px 0;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
.bar select{padding:5px 8px;border:1px solid #bbb;border-radius:6px;font-size:14px}
.bar .lbl{font-size:13px;color:#555}
#status{font-size:13px;color:#555;word-break:break-all}
#status.err{color:#b91c1c}
.conn{margin-left:auto;font-size:13px;padding:5px 12px;border-radius:999px;background:#e2e8f0;color:#64748b}
.conn.ok{background:#dcfce7;color:#15803d}
.conn.bad{background:#fee2e2;color:#b91c1c}
.footer{text-align:center;color:#94a3b8;font-size:13px;margin:18px 0 6px}
button{padding:7px 14px;border:none;border-radius:7px;background:#2563eb;color:#fff;font-size:14px;cursor:pointer}
button:hover{background:#1d4ed8}
button.ghost{background:#64748b}
button.ghost:hover{background:#475569}
</style>
</head>
<body>
<h1>信号发生器控制面板 · ESP32-C3</h1>
<div class="note">
  <b>任何含有PWM的模式：</b>需要外接上拉电源，上拉电源决定了PWM高边的电压，频率 0~2kHz。<br>
  <b>低电平循环和大周期循环模式：</b>不用接上拉电源，作用是将被控波形按规律拉低，高阻 = 三极管截止，低 = 三极管导通拉低。<br>
  <b>使用时注意共地</b>。
</div>
<div class="grid" id="channels"></div>
<div class="bar">
  <button onclick="saveAll()">保存设置</button>
  <label class="lbl" for="txp">发射功率</label>
  <select id="txp" onchange="setTxPower()">
    <option value="34">8.5 dBm</option>
    <option value="44">11 dBm</option>
    <option value="52">13 dBm</option>
    <option value="60">15 dBm</option>
    <option value="80">20 dBm</option>
  </select>
  <span id="status"></span>
  <span id="conn" class="conn">● 连接中…</span>
</div>
<div class="footer">作者：忙碌的小周</div>
<script>
const PINS = [1];
let editing = false;
addEventListener('focusin', () => editing = true);
addEventListener('focusout', () => editing = false);

function modeOptions() {
  return '<option value="0">关闭</option>'
    + '<option value="1">PWM</option>'
    + '<option value="2">低电平循环（x高阻/y低）</option>'
    + '<option value="3">大周期循环（z）</option>'
    + '<option value="4">(PWM/拉低)循环</option>'
    + '<option value="5">(PWM/拉低)大周期循环</option>';
}

function cardHTML(i) {
  return `<div class="card" id="card${i}">
    <div class="head">通道 ${i} · GPIO${PINS[i]}</div>
    <div class="row mode-row"><label>模式</label>
      <select id="mode${i}" onchange="syncFields(${i})">${modeOptions()}</select>
    </div>
    <div class="row pwm"><label>频率 Hz</label><input id="f${i}" type="number" min="0" max="2000" step="0.1" value="1000"></div>
    <div class="row pwm"><label>占空比 %</label><input id="duty${i}" type="number" min="0" max="100" step="1" value="50"></div>
    <div class="row pat"><label id="xlbl${i}">x 高阻 (s)</label><input id="x${i}" type="number" min="0.1" max="3600" step="0.1" value="1.0"></div>
    <div class="row pat"><label id="ylbl${i}">y 低 (s)</label><input id="y${i}" type="number" min="0.1" max="3600" step="0.1" value="1.0"></div>
    <div class="row big"><label>a 拉低次数</label><input id="a${i}" type="number" min="1" max="10000" step="1" value="5"></div>
    <div class="row big"><label>z 大周期 (s)</label><input id="z${i}" type="number" min="0.1" max="86400" step="0.1" value="10.0"></div>
    <button onclick="apply(${i})">应用</button>
  </div>`;
}

window.addEventListener('DOMContentLoaded', () => {
  let h = '';
  for (let i = 0; i < PINS.length; i++) h += cardHTML(i);
  document.getElementById('channels').innerHTML = h;
  syncNow();
  setInterval(syncNow, 2000);
});

function syncFields(i) {
  const m = +document.getElementById('mode' + i).value;
  const card = document.getElementById('card' + i);
  card.classList.toggle('show-pwm', m === 1 || m === 4 || m === 5);
  card.classList.toggle('show-pat', m === 2 || m === 3 || m === 4 || m === 5);
  card.classList.toggle('show-big', m === 3 || m === 5);
  const pwmXY = m === 4 || m === 5;
  const xl = document.getElementById('xlbl' + i);
  const yl = document.getElementById('ylbl' + i);
  if (xl) xl.textContent = pwmXY ? 'x PWM (s)' : 'x 高阻 (s)';
  if (yl) yl.textContent = pwmXY ? 'y 拉低 (s)' : 'y 低 (s)';
}

function setStatus(t, isErr) {
  const el = document.getElementById('status');
  el.textContent = t;
  el.classList.toggle('err', !!isErr);
}

function setConn(ok) {
  const el = document.getElementById('conn');
  el.classList.toggle('ok', ok);
  el.classList.toggle('bad', !ok);
  el.textContent = ok ? '● 已连接' : '● 连接断开';
}

async function setTxPower() {
  const v = document.getElementById('txp').value;
  try {
    const r = await fetch('/api/txpower?p=' + v);
    const t = await r.text();
    setStatus('发射功率: ' + t, !r.ok);
  } catch (e) {
    setStatus('发射功率设置失败', true);
  }
}

async function apply(i) {
  const q = new URLSearchParams();
  q.set('ch', i);
  q.set('mode', document.getElementById('mode' + i).value);
  const setq = (id, key) => {
    const el = document.getElementById(id);
    if (el) q.set(key, el.value);
  };
  setq('f' + i, 'f');
  setq('duty' + i, 'duty');
  setq('x' + i, 'x');
  setq('y' + i, 'y');
  setq('z' + i, 'z');
  setq('a' + i, 'a');
  try {
    const r = await fetch('/api/set?' + q.toString());
    const t = await r.text();
    setStatus('CH' + i + ': ' + t, !r.ok);
  } catch (e) {
    setStatus('CH' + i + ': 请求失败', true);
    setConn(false);
  }
}

async function syncNow() {
  if (editing) return;
  try {
    const r = await fetch('/api/status');
    const j = await r.json();
    for (let i = 0; i < PINS.length; i++) {
      const c = j.channels[i];
      if (!c) continue;
      const setv = (id, v) => {
        const el = document.getElementById(id);
        if (el) el.value = v;
      };
      setv('mode' + i, c.mode);
      setv('f' + i, c.f);
      setv('duty' + i, c.duty);
      setv('x' + i, c.x);
      setv('y' + i, c.y);
      setv('z' + i, c.z);
      setv('a' + i, c.a);
      syncFields(i);
    }
    const wr = await fetch('/api/wifi');
    const wj = await wr.json();
    const tx = document.getElementById('txp');
    if (tx && wj.txPowerUnits) tx.value = wj.txPowerUnits;
    setConn(true);
  } catch (e) {
    setConn(false);
  }
}

async function saveAll() {
  try {
    const r = await fetch('/api/save');
    setStatus(await r.text(), !r.ok);
  } catch (e) {
    setStatus('保存请求失败', true);
  }
}
</script>
</body>
</html>)rawliteral";
#endif  // USE_WIFI_AP

// ===================== 配置存取（NVS） =====================
static void saveAllChannels() {
  prefs.begin("siggen", false);
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    ChannelConfig c = engineGetChannel(i);
    prefs.putBytes(("ch" + String(i)).c_str(), &c, sizeof(c));
  }
  prefs.end();
}

static void loadAllChannels() {
  prefs.begin("siggen", true);
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    ChannelConfig c = {};
    size_t want = sizeof(c);
    if (prefs.getBytes(("ch" + String(i)).c_str(), &c, want) == want) {
    if (sanitizeConfig(c) == nullptr) engineApplyChannel(i, c);
    }
  }
  prefs.end();
}

// ===================== 状态 JSON =====================
static String buildStatusJson() {
  String s = "{\"channels\":[";
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    if (i > 0) s += ",";
    ChannelConfig c = engineGetChannel(i);
    s += "{";
    s += "\"mode\":" + String(c.mode);
    s += ",\"f\":" + String(c.freqHz, 1);
    s += ",\"duty\":" + String(c.dutyPct, 0);
    s += ",\"x\":" + String(c.xSec, 1);
    s += ",\"y\":" + String(c.ySec, 1);
    s += ",\"z\":" + String(c.zSec, 1);
    s += ",\"a\":" + String(c.a);
    s += "}";
  }
  s += "]}";
  return s;
}

// ===================== 命令解析（HTTP / BLE 共用） =====================
// 命令格式与 HTTP 查询串一致：
//   set?ch=0&mode=1&f=1000&duty=50&x=1&y=1&z=10&a=5
static bool queryGet(const String& q, const char* key, String& out) {
  int pos = 0;
  while (pos < (int)q.length()) {
    int amp = q.indexOf('&', pos);
    if (amp < 0) amp = (int)q.length();
    String pair = q.substring(pos, amp);
    int eq = pair.indexOf('=');
    if (eq > 0 && pair.substring(0, eq) == key) {
      out = pair.substring(eq + 1);
      return true;
    }
    pos = amp + 1;
  }
  return false;
}

// 校验并把配置投递给 loop()。成功返回 true（应用结果由 loop() 回 "OK"），
// 失败返回 false 并把错误文本写入 err。
static bool parseAndQueue(const String& raw, char* err, size_t errLen) {
  String q = raw;
  q.trim();
  if (q.startsWith("set?")) q = q.substring(4);

  String chv, modev, v;
  if (!queryGet(q, "ch", chv) || !queryGet(q, "mode", modev)) {
    snprintf(err, errLen, "缺少 ch 或 mode 参数");
    return false;
  }
  int ch = chv.toInt();
  if (ch < 0 || ch >= CHANNEL_COUNT) {
    snprintf(err, errLen, "ch 超出范围");
    return false;
  }

  ChannelConfig c = engineGetChannel(ch);
  c.mode = (uint8_t)modev.toInt();
  if (queryGet(q, "f", v))    c.freqHz = v.toFloat();
  if (queryGet(q, "duty", v)) c.dutyPct = v.toFloat();
  if (queryGet(q, "x", v))    c.xSec = v.toFloat();
  if (queryGet(q, "y", v))    c.ySec = v.toFloat();
  if (queryGet(q, "z", v))    c.zSec = v.toFloat();
  if (queryGet(q, "a", v))    c.a = (uint32_t)v.toInt();

  const char* msg = sanitizeConfig(c);
  if (msg != nullptr) {
    snprintf(err, errLen, "%s", msg);
    return false;
  }

  ApplyCmd cmd = { ch, c };
  if (s_cmdQueue == nullptr || xQueueSend(s_cmdQueue, &cmd, pdMS_TO_TICKS(200)) != pdTRUE) {
    snprintf(err, errLen, "配置队列忙，请重试");
    return false;
  }
  return true;
}

#if !USE_WIFI_AP
// ===================== BLE 传输层（NimBLE） =====================
// 协议：文本行，以 '\n' 结尾
//   set?ch=0&mode=1&f=1000&duty=50&x=1&y=1&z=10&a=5  -> "OK" 或错误文本
//   status                                            -> 状态 JSON
//   save                                              -> "已保存"
// ATT 默认 MTU 只有 23（20 字节有效载荷），所以收发两端都按 '\n' 拼包。
constexpr size_t BLE_MSG_LEN = 224;
struct BleMsg {
  char text[BLE_MSG_LEN];
};
static QueueHandle_t s_txQueue = nullptr;
static BLEServer* s_bleServer = nullptr;
static BLECharacteristic* s_txChr = nullptr;
static volatile bool s_bleConnected = false;
static volatile bool s_restartAdv = false;
static char s_rxBuf[256];
static size_t s_rxLen = 0;

static void bleReply(const String& text) {
  if (s_txQueue == nullptr) return;
  BleMsg m = {};
  strlcpy(m.text, text.c_str(), sizeof(m.text));
  xQueueSend(s_txQueue, &m, pdMS_TO_TICKS(50));
}

static void handleCommandLine(const String& raw) {
  String cmd = raw;
  cmd.trim();
  if (cmd.isEmpty()) return;
  if (cmd == "status") {
    bleReply(buildStatusJson());
    return;
  }
  if (cmd == "save") {
    saveAllChannels();
    bleReply("已保存");
    return;
  }
  char err[64];
  if (!parseAndQueue(cmd, err, sizeof(err))) {
    bleReply(err);
    return;
  }
  // 受理成功：由 loop() 应用完成后回 "OK"
}

class SignalGenRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    String v = chr->getValue();
    for (size_t i = 0; i < v.length(); i++) {
      char c = v[i];
      if (c == '\n') {
        s_rxBuf[s_rxLen] = '\0';
        handleCommandLine(String(s_rxBuf));
        s_rxLen = 0;
      } else if (s_rxLen + 1 < sizeof(s_rxBuf)) {
        s_rxBuf[s_rxLen++] = c;
      } else {
        s_rxLen = 0;
        bleReply("命令过长");
      }
    }
  }
};

class SignalGenServerCallbacks : public BLEServerCallbacks {
  // 该库在连接事件里会同时调用通用与 NimBLE 专用重载，用状态位去重
  void onConnect(BLEServer* srv) override {
    if (s_bleConnected) return;
    s_bleConnected = true;
    s_rxLen = 0;
    Serial.println("[BLE] 客户端已连接");
  }
  void onDisconnect(BLEServer* srv) override {
    if (!s_bleConnected) return;
    s_bleConnected = false;
    s_rxLen = 0;
    s_restartAdv = true;  // 回到 loop() 里重新广播，避免在协议栈任务中重入
    Serial.println("[BLE] 客户端已断开，恢复广播");
  }
};

// 发送一行（按对端 MTU 分片，末尾补 '\n' 作为行结束符）
static void sendBleLine(const char* text) {
  if (!s_bleConnected || s_txChr == nullptr) return;
  uint16_t mtu = 23;
  if (s_bleServer != nullptr) {
    uint16_t m = s_bleServer->getPeerMTU(s_bleServer->getConnId());
    if (m > mtu) mtu = m;
  }
  size_t chunk = (size_t)(mtu - 3);
  if (chunk > 180) chunk = 180;
  String payload = String(text) + "\n";
  for (size_t off = 0; off < payload.length(); off += chunk) {
    s_txChr->setValue(payload.substring(off, off + chunk));
    s_txChr->notify();
    delay(5);  // 给协议栈排队留出时间，避免连续通知被丢弃
  }
}

static bool setupBle() {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(256);

  s_bleServer = BLEDevice::createServer();
  if (s_bleServer == nullptr) return false;
  s_bleServer->setCallbacks(new SignalGenServerCallbacks());

  BLEService* svc = s_bleServer->createService(BLEUUID(BLE_SVC_UUID), 30);
  BLECharacteristic* rx = svc->createCharacteristic(
      BLE_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new SignalGenRxCallbacks());
  // NimBLE 会在带通知属性的特征上自动创建 CCCD，无需手工添加 BLE2902
  s_txChr = svc->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);  // 连接间隔偏好 7.5~15ms，操作响应更快
  adv->setMaxPreferred(0x0C);
  if (!adv->start()) {
    Serial.println("BLE 广播启动失败");
    return false;
  }
  Serial.printf("BLE: %s 已开始广播\n", BLE_NAME);
  return true;
}
#endif  // !USE_WIFI_AP

// ===================== HTTP 响应辅助 =====================
#if USE_WIFI_AP
static esp_err_t sendResp(httpd_req_t* req, const char* status,
                          const char* type, const char* body, size_t len) {
  if (status != nullptr) httpd_resp_set_status(req, status);
  if (type != nullptr) httpd_resp_set_type(req, type);
  return httpd_resp_send(req, body, len);
}

// ===================== Web 接口 =====================
static esp_err_t handleRoot(httpd_req_t* req) {
  return sendResp(req, "200 OK", "text/html; charset=utf-8",
                  INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t handleSet(httpd_req_t* req) {
  char qbuf[256];
  esp_err_t err = httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  if (err == ESP_ERR_HTTPD_RESULT_TRUNC) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    "查询参数过长", strlen("查询参数过长"));
  }
  if (err != ESP_OK) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    "缺少查询参数", strlen("缺少查询参数"));
  }

  char errMsg[64];
  if (!parseAndQueue(String(qbuf), errMsg, sizeof(errMsg))) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    errMsg, strlen(errMsg));
  }
  return sendResp(req, "200 OK", "text/plain; charset=utf-8", "OK", 2);
}

static esp_err_t handleStatus(httpd_req_t* req) {
  String s = buildStatusJson();
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, s.c_str(), s.length());
}

static esp_err_t handleSave(httpd_req_t* req) {
  saveAllChannels();
  return sendResp(req, "200 OK", "text/plain; charset=utf-8",
                  "已保存", strlen("已保存"));
}

static esp_err_t handleTxPower(httpd_req_t* req) {
  char qbuf[64];
  esp_err_t err = httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
  if (err != ESP_OK) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    "缺少 p 参数", strlen("缺少 p 参数"));
  }
  char val[8] = {0};
  if (httpd_query_key_value(qbuf, "p", val, sizeof(val)) != ESP_OK) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    "缺少 p 参数", strlen("缺少 p 参数"));
  }
  int units = atoi(val);
  if (!isValidTxPower(units)) {
    return sendResp(req, "400 Bad Request", "text/plain; charset=utf-8",
                    "发射功率值无效", strlen("发射功率值无效"));
  }
  esp_err_t werr = esp_wifi_set_max_tx_power((int8_t)units);
  if (werr != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power 失败: %s\n", esp_err_to_name(werr));
    return sendResp(req, "503 Service Unavailable", "text/plain; charset=utf-8",
                    "设置失败，请重试", strlen("设置失败，请重试"));
  }
  s_txPowerUnits = (uint8_t)units;
  saveTxPowerUnits(s_txPowerUnits);
  Serial.printf("WiFi TX power -> %d (%.1f dBm)\n",
                (int)s_txPowerUnits, s_txPowerUnits * 0.25f);
  return sendResp(req, "200 OK", "text/plain; charset=utf-8", "OK", 2);
}

static esp_err_t handleWifiStatus(httpd_req_t* req) {
  int8_t cur = 0;
  esp_wifi_get_max_tx_power(&cur);
  String s = "{\"txPowerUnits\":" + String(cur);
  s += ",\"txPowerDbm\":" + String(cur * 0.25f, 1);
  s += "}";
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, s.c_str(), s.length());
}

// ===================== WiFi AP（ESP-IDF 原生 API） =====================
// Arduino core 3.x 的 WiFi/WebServer 在 C3 上有已知崩溃 bug
// （NetworkClientSocketHandle 的 shared_ptr 赋值），因此这里直接调用
// ESP-IDF 的 esp_wifi / esp_http_server，不经过 Arduino WiFi 库。

static bool setupWiFiAp() {
  esp_err_t err = esp_netif_init();
  if (err != ESP_OK) {
    Serial.printf("esp_netif_init 失败: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("esp_event_loop_create_default 失败: %s\n", esp_err_to_name(err));
    return false;
  }

  esp_netif_create_default_wifi_ap();

  wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&wcfg);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_init 失败: %s\n", esp_err_to_name(err));
    return false;
  }
  // 保持 USB CDC 串口可用（同 Arduino WiFi 库的 C3 补丁）
  phy_bbpll_en_usb(true);

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);  // AP 配置不写入 NVS
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_storage 失败: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_set_mode(WIFI_MODE_AP);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_mode 失败: %s\n", esp_err_to_name(err));
    return false;
  }

  wifi_config_t ap = {};
  strlcpy((char*)ap.ap.ssid, AP_SSID, sizeof(ap.ap.ssid));
  ap.ap.ssid_len = (uint8_t)strlen(AP_SSID);
  ap.ap.channel = 1;
  ap.ap.max_connection = 4;
  ap.ap.authmode = (AP_PASS != nullptr && strlen(AP_PASS) >= 8)
                     ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
  if (ap.ap.authmode != WIFI_AUTH_OPEN) {
    strlcpy((char*)ap.ap.password, AP_PASS, sizeof(ap.ap.password));
  }

  err = esp_wifi_set_config(WIFI_IF_AP, &ap);
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_set_config 失败: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_wifi_start();
  if (err != ESP_OK) {
    Serial.printf("esp_wifi_start 失败: %s\n", esp_err_to_name(err));
    return false;
  }
  return true;
}

// ===================== HTTP 服务器（ESP-IDF 原生） =====================
static bool startHttpd() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.stack_size = 8192;
  cfg.lru_purge_enable = true;  // 连接过多时优先关闭最旧的空闲连接

  if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
    Serial.println("httpd_start 失败");
    return false;
  }

  // 本核心启用了 WebSocket 支持，httpd_uri_t 额外带 is_websocket 等字段，
  // 显式置零以免编译告警。
#if CONFIG_HTTPD_WS_SUPPORT
#define URI_WS_FIELDS .is_websocket = false, .handle_ws_control_frames = false, .supported_subprotocol = nullptr,
#else
#define URI_WS_FIELDS
#endif

  static const httpd_uri_t uriRoot = {
    .uri = "/", .method = HTTP_GET, .handler = handleRoot, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
  static const httpd_uri_t uriSet = {
    .uri = "/api/set", .method = HTTP_GET, .handler = handleSet, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
  static const httpd_uri_t uriStatus = {
    .uri = "/api/status", .method = HTTP_GET, .handler = handleStatus, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
  static const httpd_uri_t uriSave = {
    .uri = "/api/save", .method = HTTP_GET, .handler = handleSave, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
  static const httpd_uri_t uriTxPower = {
    .uri = "/api/txpower", .method = HTTP_GET, .handler = handleTxPower, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
  static const httpd_uri_t uriWifiStatus = {
    .uri = "/api/wifi", .method = HTTP_GET, .handler = handleWifiStatus, .user_ctx = nullptr,
    URI_WS_FIELDS
  };
#undef URI_WS_FIELDS
  httpd_register_uri_handler(s_httpd, &uriRoot);
  httpd_register_uri_handler(s_httpd, &uriSet);
  httpd_register_uri_handler(s_httpd, &uriStatus);
  httpd_register_uri_handler(s_httpd, &uriSave);
  httpd_register_uri_handler(s_httpd, &uriTxPower);
  httpd_register_uri_handler(s_httpd, &uriWifiStatus);
  return true;
}
#endif  // USE_WIFI_AP

// ===================== 主程序 =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSignal Generator for ESP32-C3 SuperMini");

  engineInit();
  loadAllChannels();

  s_cmdQueue = xQueueCreate(8, sizeof(ApplyCmd));
  if (s_cmdQueue == nullptr) {
    Serial.println("警告: 配置队列创建失败");
  }

#if USE_WIFI_AP
  s_txPowerUnits = loadTxPowerUnits();

  if (!setupWiFiAp()) {
    Serial.println("WiFi AP 启动失败，输出引擎继续运行");
  }

  // 发射功率必须在 esp_wifi_start() 之后设置；稍等片刻让 AP 完全就绪
  delay(200);
  esp_err_t tperr = esp_wifi_set_max_tx_power((int8_t)s_txPowerUnits);
  if (tperr != ESP_OK) {
    Serial.printf("esp_wifi_set_max_tx_power 失败: %s\n", esp_err_to_name(tperr));
  } else {
    Serial.printf("WiFi TX power: %d (%.1f dBm)\n",
                  (int)s_txPowerUnits, s_txPowerUnits * 0.25f);
  }

  if (startHttpd()) {
    esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap != nullptr) {
      esp_netif_ip_info_t ip = {};
      if (esp_netif_get_ip_info(ap, &ip) == ESP_OK) {
        Serial.printf("WiFi AP: %s  面板地址: http://" IPSTR "\n",
                      AP_SSID, IP2STR(&ip.ip));
      }
    }
  }
#else
  s_txQueue = xQueueCreate(8, sizeof(BleMsg));
  if (s_txQueue == nullptr) {
    Serial.println("警告: 回复队列创建失败");
  }
  if (!setupBle()) {
    Serial.println("BLE 启动失败，输出引擎继续运行");
  }
#endif

  Serial.println("输出引脚: GPIO1=CH0（单通道，支持全部模式）");
}

void loop() {
  // 消费传输层发来的配置命令（在 loop 任务中应用，避免跨线程操作输出引擎）
  ApplyCmd cmd;
  bool applied = false;
  while (s_cmdQueue != nullptr && xQueueReceive(s_cmdQueue, &cmd, 0) == pdTRUE) {
    engineApplyChannel(cmd.ch, cmd.cfg);
    applied = true;
    Serial.printf("[CH%d] mode=%u f=%.1fHz duty=%.0f%% x=%.1fs y=%.1fs z=%.1fs a=%lu\n",
                  cmd.ch, (unsigned)cmd.cfg.mode, cmd.cfg.freqHz, cmd.cfg.dutyPct,
                  cmd.cfg.xSec, cmd.cfg.ySec, cmd.cfg.zSec, (unsigned long)cmd.cfg.a);
  }

#if !USE_WIFI_AP
  if (applied) bleReply("OK");
  BleMsg msg;
  while (s_txQueue != nullptr && xQueueReceive(s_txQueue, &msg, 0) == pdTRUE) {
    sendBleLine(msg.text);
  }
  if (s_restartAdv) {
    s_restartAdv = false;
    BLEDevice::getAdvertising()->start();
  }
#else
  (void)applied;
#endif
  delay(5);
}
