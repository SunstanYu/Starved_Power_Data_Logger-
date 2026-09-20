#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include "display_gateway.h"

static const char* AP_SSID = "ESP32S3-Gateway";
static const char* AP_PASS = "12345678";
static const char* SENSOR_TYPE = "Soil moisture + DHT11 + AS7343";
static const IPAddress AP_IP(192, 168, 10, 1);
static const IPAddress AP_GW(192, 168, 10, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

static WebServer server(80);

// ★ A10：OTA。
//
// 【为什么现成方案都不适用】
//   ArduinoOTA  —— 要设备和开发机在同一局域网，靠 mDNS 推送。田里没网。
//   HTTPUpdate  —— 设备主动去服务器拉固件。同样要网。
//
// 【这个场景的天然解法】display 模式下设备【自己就是】AP + HTTP 服务器。
// 走到田里 → 拨 GPIO5 → 设备重启进 display 模式 → 手机连它的 AP
// → 打开 /update 上传 .bin → 设备重启跑新固件。全程不需要任何网络。
//
// ⚠️ 必须带认证：AP 在野外，不加认证等于谁都能刷你的固件。
static HTTPUpdateServer updater;
static const char* OTA_USER = "field";
static const char* OTA_PASS = "starved-logger";

static String getFieldValue(const String& record, const char* key) {
  String pattern = String(key) + "=";
  int start = record.indexOf(pattern);
  if (start < 0) {
    return "";
  }
  start += pattern.length();
  int end = record.indexOf(",", start);
  if (end < 0) {
    end = record.length();
  }
  return record.substring(start, end);
}

static String buildMetricCard(const String& title, const String& value, const String& unit) {
  String html;
  html += "<section class='metric-card'><div class='metric-title'>";
  html += title;
  html += "</div><div class='metric-value'>";
  html += value.length() > 0 ? value : "--";
  if (unit.length() > 0) {
    html += "<span class='metric-unit'> ";
    html += unit;
    html += "</span>";
  }
  html += "</div></section>";
  return html;
}

static String buildSpectralGrid(const String& spectralRaw) {
  String html;
  html += "<section class='panel'><h2>Multi-spectral Channels</h2>";
  if (spectralRaw.length() == 0 || spectralRaw == "none") {
    html += "<p class='muted'>No spectral data available.</p></section>";
    return html;
  }

  html += "<div class='spectral-grid'>";
  int index = 0;
  int start = 0;
  while (start <= spectralRaw.length()) {
    int end = spectralRaw.indexOf("|", start);
    if (end < 0) {
      end = spectralRaw.length();
    }
    String value = spectralRaw.substring(start, end);
    value.trim();
    html += "<div class='spectral-item'><span class='spectral-label'>CH";
    html += String(index + 1);
    html += "</span><span class='spectral-value'>";
    html += value.length() > 0 ? value : "--";
    html += "</span></div>";
    index++;
    if (end >= spectralRaw.length()) {
      break;
    }
    start = end + 1;
  }
  html += "</div></section>";
  return html;
}

static String buildLatestPanels() {
  String record = getDisplayLatestRecord();
  if (record.length() == 0) {
    return "<section class='panel'><h2>Latest Sample</h2><p class='muted'>No data has been written yet.</p></section>";
  }

  String moisture = getFieldValue(record, "moisture");
  String batteryMv = getFieldValue(record, "voltage_mv");
  if (batteryMv.length() == 0) {
    batteryMv = getFieldValue(record, "battery_mv");
  }
  String tempC = getFieldValue(record, "temp_c");
  String humidity = getFieldValue(record, "humidity");
  String spectral = getFieldValue(record, "spectral");

  String html;
  html += "<section class='panel'><h2>Latest Sample</h2>";
  html += "<div class='metrics'>";
  html += buildMetricCard("Soil Moisture", moisture, "raw");
  html += buildMetricCard("Battery", batteryMv, "mV");
  html += buildMetricCard("Temperature", tempC == "nan" ? "--" : tempC, "C");
  html += buildMetricCard("Humidity", humidity == "nan" ? "--" : humidity, "%");
  html += "</div></section>";
  html += buildSpectralGrid(spectral);
  return html;
}

static String buildPageHead() {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 Gateway</title>";
  html += "<style>";
  html += ":root{--bg:#edf3ea;--panel:#fffdf8;--ink:#142013;--muted:#5f6b61;--line:#d7e1d3;--accent:#466b46;}";
  html += "body{font-family:Georgia,serif;background:linear-gradient(180deg,#eef6ec 0%,#dfeadb 100%);color:var(--ink);margin:0;padding:20px;}";
  html += ".shell{max-width:1040px;margin:0 auto;}";
  html += ".hero{background:var(--panel);border:1px solid var(--line);border-radius:20px;padding:22px;box-shadow:0 10px 30px rgba(20,32,19,0.08);}";
  html += "h1{margin:0 0 8px 0;font-size:30px;}h2{margin:0 0 14px 0;font-size:20px;}";
  html += ".hero-meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin-top:16px;}";
  html += ".meta-chip{background:#f7fbf5;border:1px solid var(--line);border-radius:14px;padding:12px;}";
  html += ".meta-label,.metric-title,.spectral-label{display:block;color:var(--muted);font-size:12px;letter-spacing:0.08em;text-transform:uppercase;}";
  html += ".meta-value{display:block;font-size:18px;margin-top:6px;}";
  html += ".layout{display:grid;gap:16px;margin-top:16px;}";
  html += ".panel{background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:18px;box-shadow:0 8px 24px rgba(20,32,19,0.06);}";
  html += ".metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;}";
  html += ".metric-card{background:#f7fbf5;border:1px solid var(--line);border-radius:14px;padding:14px;min-height:92px;}";
  html += ".metric-value{font-size:28px;line-height:1.1;margin-top:10px;word-break:break-word;}";
  html += ".metric-unit{font-size:14px;color:var(--muted);}";
  html += ".spectral-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(88px,1fr));gap:10px;}";
  html += ".spectral-item{background:#f7fbf5;border:1px solid var(--line);border-radius:12px;padding:10px;}";
  html += ".spectral-value{display:block;font-size:18px;margin-top:6px;}";
  html += "ul{margin:0;padding-left:18px;}li{margin:6px 0;line-height:1.4;}";
  html += ".muted{color:var(--muted);font-size:14px;}";
  html += "@media (max-width:640px){body{padding:14px;}h1{font-size:24px;}.metric-value{font-size:22px;}}";
  html += "</style></head><body>";
  html += "<main class='shell'><section class='hero'>";
  html += "<h1>ESP32-S3 Sensor Gateway</h1>";
  html += "<p class='muted'>Local dashboard served directly from the device AP.</p>";
  html += "<div class='hero-meta'>";
  html += "<div class='meta-chip'><span class='meta-label'>Sensor Stack</span><span class='meta-value'>";
  html += SENSOR_TYPE;
  html += "</span></div>";
  html += "<div class='meta-chip'><span class='meta-label'>Storage</span><span class='meta-value'>";
  html += getDisplayStorageLabel();
  html += "</span></div>";
  html += "<div class='meta-chip'><span class='meta-label'>Mode</span><span class='meta-value'>Local gateway</span></div>";
  html += "</div></section>";
  html += "<section class='layout'>";
  html += buildLatestPanels();
  html += "<section class='panel'><h2>History Log</h2><ul>";
  return html;                       // ← 到历史记录的 <ul> 为止，后面流式补
}

static String buildPageTail() {
  return F("</ul><p class='muted'>Refresh the page to load the latest sample "
           "from storage.</p></section></section></main>"
           // 打开页面即对时：设备没有 RTC 也没有网络，浏览器是唯一时间源
           "<p class='muted'>Firmware update: "
           "<a href='/update'>/update</a></p>"
           "<script>"
           "fetch('/settime?epoch='+Math.floor(Date.now()/1000))"
           ".then(r=>r.text()).then(t=>console.log('clock sync:',t))"
           ".catch(e=>console.log('clock sync failed',e));"
           "</script></body></html>");
}

// ★ A1：分块发送，不再把整页拼成一个 String。
//
// 原来是 server.send(200, "text/html", buildPage())——buildPage 里把
// 【整个日志文件】拼进一个 String。文件涨到几百 KB 就会耗尽堆，而且
// String 分配失败是静默的：页面截断但不报错。
//
// 现在：Content-Length 未知 → 先发头 → 逐行发历史 → 发尾。
// 峰值内存从 O(日志大小) 降到 O(单行)。
static void emitChunk(const String& s) { server.sendContent(s); }

// ★ A3：浏览器对时。
//
// 【问题】记录里格式化时间戳的分支永远不执行——因为条件是
// `now > 1700000000`，而全项目【没有任何地方设过系统时钟】。
// 实际写进文件的只有 time_ms=millis()，而 millis() 每次 deep sleep
// 醒来都从 0 重数。靠 sampleCounter 能反推相对时间，但没有绝对时间——
// 农业数据的日变化曲线画不出来。
//
// 【为什么不用 NTP】田里没有网络回传，设备自己是 AP 不是 STA。
// 【为什么不加 RTC 芯片】要改 PCB。
//
// 【解法】用户的手机就是唯一会来的那个设备。他打开仪表盘时，
// 网页上的 JS 顺手把浏览器时间回传，设备 settimeofday()。
// ESP32 的 RTC 在 deep sleep 期间继续走，对一次能撑很久
// （晶振漂移大约每天几秒）。零硬件成本。
static void handleSetTime() {
  if (!server.hasArg("epoch")) {
    server.send(400, "text/plain", "missing epoch");
    return;
  }
  long long epoch = atoll(server.arg("epoch").c_str());
  if (epoch < 1700000000LL) {                 // 2023-11 之前的值一律拒绝
    server.send(400, "text/plain", "implausible epoch");
    return;
  }
  struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  Serial.printf("[TIME] clock set from browser: %lld\n", epoch);
  server.send(200, "text/plain", "ok");
}

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent(buildPageHead());
  streamDisplayStoredData(emitChunk);        // 每行发一次，发完即释放
  server.sendContent(buildPageTail());
  server.sendContent("");                    // 空块 = 结束
}

void displayGatewaySetup() {
  WiFi.mode(WIFI_AP);
  bool cfgOk = WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  Serial.println("[DISPLAY] AP mode enabled.");
  Serial.printf("[DISPLAY] AP config: %s\n", cfgOk ? "OK" : "FAILED");
  Serial.printf("[DISPLAY] AP start: %s\n", ok ? "OK" : "FAILED");
  Serial.printf("[DISPLAY] SSID: %s\n", AP_SSID);
  Serial.printf("[DISPLAY] PASS: %s\n", AP_PASS);
  Serial.printf("[DISPLAY] Open: http://%s/\n", ip.toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/settime", HTTP_GET, handleSetTime);
  updater.setup(&server, "/update", OTA_USER, OTA_PASS);   // ← 带认证的上传端点
  server.begin();
  Serial.println("[DISPLAY] HTTP server started.");
}

void displayGatewayLoop() {
  server.handleClient();
  delay(2);
}
