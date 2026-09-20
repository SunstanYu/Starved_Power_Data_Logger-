#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/gpio.h"  // for access to gpio_num_t / GPIO_NUM_6
#include <SPI.h>
#include <SD.h>
#include <FFat.h>
#include <Wire.h>
#include <DHTesp.h>
#include <SparkFun_AS7343.h>
#include <time.h>
#include "display_gateway.h"
#include "tasks.h"

// ====================== CONSTANTS & PINS ======================
const uint64_t SLEEP_DURATION_US = 10 * 1000000;   // sleep for 10 seconds
const uint32_t AWAKE_DURATION_MS = 20 * 1000;      // awake for 20 seconds
const uint32_t MODE_SWITCH_DELAY_MS = 5000;         // GPIO5 must stay LOW for 5s before leaving display mode
const uint32_t RECORD_EVERY_SAMPLE_COUNT = 50;      // write one record every 50 samples
const int STORAGE_MODE = 0;                         // 0 = SD card, 1 = FFat flash
int led = LED_BUILTIN;
const gpio_num_t modePin = GPIO_NUM_5;              // LOW = logger mode, HIGH = display mode

const int switch1Pin = T10;
const int switch2Pin = T6;
const int switch3Pin = T9;
const int batteryLedPin = T13;


const int sensor1Pin = A3;   // DHT11 data pin, use the real GPIO number 
const int sensor2Pin = A4;    // soil moisture sensor analog input
const int SDcardPin = T12;
const int batteryAdcPin = A2;
const char* DISPLAY_SENSOR_TYPE = "Soil moisture + DHT11 + AS7343";
const char* DISPLAY_DATA_FILE = "/sensor_log.txt";
const float ADC_VREF = 3.3f;
const float BATTERY_DIVIDER_RATIO = 3.0f;
const float BATTERY_LOW_THRESHOLD_V = 4.0f;

// ★ A5：低电压截止。
//
// 供电是 4 节 AA（标称 6.0V）。碱性电池没有锂电「过放即报废」的问题，
// 所以这里的理由不是保护电池，而是【保护数据】：
// 电压掉到 3.3V LDO 的压差余量以下时，轨会不稳；如果恰好发生在
// SD 写入中途，坏的是文件甚至整张卡——而那是几个月的数据。
//
// 所以到阈值就：刷掉待写数据 → 点亮低电量灯 → 进【无定时唤醒】的深睡。
// 不设定时器意味着它不会再自己醒来采样，只能靠 EXT0（人拨开关）或换电池。
// extern：C++ 里命名空间作用域的 const 默认是【内部链接】，
// 不加 extern 的话 tasks.cpp 链接时找不到它。
extern const float BATTERY_CUTOFF_V = 3.7f;
DHTesp dht;
SfeAS7343ArdI2C spectralSensor;
uint16_t spectralChannels[ksfAS7343NumChannels];
// const int wakePin   = T6;    // D6 = GPIO6 (used as normal input + DeepSleep wakeup)
bool forever_awake  = false; // never sleep after D6 is triggered

bool sdReady = false;
bool flashReady = false;
bool spectralReady = false;
bool displayModeActive = false;
bool tasksRunning = false;      // ★ A8：四任务是否起来了
bool modePinLowTiming = false;
unsigned long modePinLowStartMs = 0;
RTC_DATA_ATTR uint32_t sampleCounter = 0;

// ====================== HELPERS ======================

// ★ A10：OTA 回滚自检。
//
// Arduino 核心已启用 CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE：新固件首次
// 启动时处于 PENDING_VERIFY 状态，initArduino() 会调 verifyOta()——
// 返回 true 就 mark_app_valid 确认，返回 false 就【自动回滚并重启】到旧分区。
//
// 但核心里的 verifyOta() 是 weak 函数，默认无条件 `return true`：
// 只要新固件能跑到 initArduino() 就算通过。这挡得住「根本起不来」，
// 挡不住「能起来但功能坏了」。
//
// 对一个埋在田里、回收成本极高的节点，这个区别是致命的——
// 刷坏了就是报废，没有第二次机会。所以覆盖它，做真实自检。
//
// ⚠️ 核心注释提醒：这函数在启动早期跑，要短、不要阻塞太久。
// 所以这里只做「能不能起来」级别的检查，不碰 2.5s 的 DHT11 时序。
extern "C" bool verifyOta() {
  Serial.println("[OTA] verifying new image...");

  pinMode(SDcardPin, OUTPUT);
  digitalWrite(SDcardPin, HIGH);
  if (!SD.begin(SDcardPin, SPI, 1000000)) {
    Serial.println("[OTA] FAIL: SD not available -> rollback");
    return false;                          // 存储起不来 = 数据会丢，不如退回
  }
  SD.end();

  int mv = getBatteryMv();
  if (mv > 0 && mv < (int)(BATTERY_CUTOFF_V * 1000)) {
    Serial.printf("[OTA] FAIL: battery %d mV too low -> rollback\n", mv);
    return false;                          // 电量不够撑过验证期，别冒险留在新版
  }

  Serial.println("[OTA] image verified, cancelling rollback.");
  return true;
}

// ★ A5：电量到底线时的收尾动作。调用后不返回。
void enterLowBatteryShutdown(int batteryMv) {
  Serial.printf("[BAT] %d mV below cutoff %.2f V — flushing and halting.\n",
                batteryMv, BATTERY_CUTOFF_V);
  digitalWrite(batteryLedPin, HIGH);
  setPeripheralSwitches(false);          // 先断外设，减少残余放电
  Serial.flush();
  esp_deep_sleep_start();                // 无定时唤醒源 = 不会再自己醒
}

// ★ A6：资源水位。诊断「跑久了会不会崩」只能靠这个，不能靠推测。
//
// getMinFreeHeap() 是【历史最低值】——它回答的是「最危险的那一刻离
// OOM 还有多远」，比当前空闲值有用得多。栈水位同理：uxTaskGetStack-
// HighWaterMark 返回的是该任务栈的历史最小剩余字节。
void logResourceWatermark(const char* tag) {
  Serial.printf("[MEM] %s heap=%u min=%u largest=%u stack_free=%u\n",
                tag,
                ESP.getFreeHeap(),
                ESP.getMinFreeHeap(),                        // 历史最低 ← 关键
                ESP.getMaxAllocHeap(),                       // 最大连续块 ← 看碎片
                uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
}

// ★ A4：Task WDT。田里的节点卡死 = 报废，只能人去拔电池。
//
// Arduino 默认【关掉】了 task WDT（原型阶段谁也不想跑着跑着被复位），
// 但生产设备必须有人盯着「卡死」这件事：SD 总线卡住、传感器不应答、
// 某个 while 等不到条件——这些都不会自己恢复。
//
// 采样态与显示态的超时不同：采样一轮含 2.5s 的外设稳定与 DHT11 时序，
// 显示态则是持续响应 HTTP。所以由调用方给超时值。
#include "esp_task_wdt.h"

void watchdogBegin(uint32_t timeout_s) {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg = {
      .timeout_ms = timeout_s * 1000,
      .idle_core_mask = 0,          // 不监控 idle 任务
      .trigger_panic = true,        // 超时直接 panic 复位（现场会进 coredump）
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(timeout_s, true);
#endif
  esp_task_wdt_add(NULL);           // 把当前任务纳入监控
  Serial.printf("[WDT] armed, timeout=%us\n", timeout_s);
}

void watchdogFeed() { esp_task_wdt_reset(); }
void setPeripheralSwitches(bool enabled) {
  digitalWrite(switch1Pin, enabled ? HIGH : LOW);
  digitalWrite(switch2Pin, enabled ? HIGH : LOW);
  digitalWrite(switch3Pin, enabled ? HIGH : LOW);
  Serial.printf(
    "[PWR] switch1=%s switch2=%s switch3=%s\n",
    enabled ? "HIGH" : "LOW",
    enabled ? "HIGH" : "LOW",
    enabled ? "HIGH" : "LOW"
  );
}

// GPIO5 is the single hardware mode selector for the whole firmware:
// - LOW  => sensor logger mode
// - HIGH => local display gateway mode
bool isDisplayModeRequested() {
#ifdef FORCE_DISPLAY_MODE
  // 测试钩子：常驻模式要求 GPIO5 拉高，而那需要一根跳线。
  // 开这个宏可以在没有跳线的台面上验证四任务架构本身。
  // 只由 -DFORCE_DISPLAY_MODE 打开，正常构建里这段不存在。
  return true;
#endif
  return digitalRead((int)modePin) == HIGH;
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("[WAKE] Wakeup caused by EXT0 (GPIO5 HIGH).");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("[WAKE] Wakeup caused by timer.");
      break;
    default:
      Serial.printf("[WAKE] Wakeup was not caused by deep sleep: %d\n", (int)cause);
      break;
  }
}

// Logger mode uses both wake sources:
// 1. timer wakeup for the next sampling cycle
// 2. EXT0 wakeup on GPIO5 HIGH so the user can force display mode while asleep
void configureWakeSources() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_sleep_enable_ext0_wakeup(modePin, 1);
}

// A reboot is the cleanest way to switch modes at runtime because setup()
// re-runs the same initialization path and re-evaluates GPIO5.
void restartForModeSwitch(const char* reason) {
  Serial.println(reason);
  Serial.flush();
  delay(100);
  ESP.restart();
}

void enterTimedDeepSleep() {
  Serial.println("[SLEEP] Preparing to enter deep sleep...");
  configureWakeSources();
  Serial.println("[SLEEP] Entering deep sleep now.");
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}

int getMoisture() {
  int value = analogRead(sensor2Pin);
  Serial.print("Soil moisture raw value: ");
  Serial.println(value);
  return value;
}

int getBatteryMv() {
  int raw = analogRead(batteryAdcPin);
  float vadc = ((float)raw / 4095.0f) * ADC_VREF;
  float vbat = vadc * BATTERY_DIVIDER_RATIO;
  int batteryMv = (int)(vbat * 1000.0f);

  Serial.print("[BAT] RAW=");
  Serial.print(raw);
  Serial.print(" Vbat=");
  Serial.print(vbat, 3);
  Serial.println(" V");

  digitalWrite(batteryLedPin, vbat < BATTERY_LOW_THRESHOLD_V ? HIGH : LOW);
  Serial.printf("[BAT] T13=%s\n", vbat < BATTERY_LOW_THRESHOLD_V ? "HIGH" : "LOW");

  return batteryMv;
}

bool readDht11(float& temperatureC, float& humidity) {
  Serial.println("[DHT11] Reading...");
  TempAndHumidity data = dht.getTempAndHumidity();
  humidity = data.humidity;
  temperatureC = data.temperature;

  if (isnan(humidity) || isnan(temperatureC)) {
    Serial.println("[DHT11] Failed to read from DHT11");
    return false;
  }

  Serial.print("[DHT11] Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  return true;
}

bool initSpectralSensor() {
  Serial.println("[AS7343] Initializing...");
  Wire.begin(3,4);
  Wire.setClock(100000);

  if (!spectralSensor.begin(0x39, Wire)) {
    Serial.println("[AS7343] begin failed.");
    return false;
  }
  if (!spectralSensor.powerOn()) {
    Serial.println("[AS7343] powerOn failed.");
    return false;
  }
  if (!spectralSensor.setAutoSmux(AUTOSMUX_18_CHANNELS)) {
    Serial.println("[AS7343] setAutoSmux failed.");
    return false;
  }
  if (!spectralSensor.enableSpectralMeasurement()) {
    Serial.println("[AS7343] enableSpectralMeasurement failed.");
    return false;
  }

  Serial.println("[AS7343] Sensor ready.");
  return true;
}

int readSpectralChannels(uint16_t* channels, size_t channelCapacity) {
  if (!spectralReady) {
    return 0;
  }

  if (!spectralSensor.readSpectraDataFromSensor()) {
    Serial.println("[AS7343] readSpectraDataFromSensor failed.");
    return 0;
  }

  int count = spectralSensor.getData(channels);
  if (count > (int)channelCapacity) {
    count = (int)channelCapacity;
  }

  Serial.print("[AS7343] Channels: ");
  for (int i = 0; i < count; i++) {
    Serial.print(channels[i]);
    if (i + 1 < count) {
      Serial.print(",");
    }
  }
  Serial.println();
  return count;
}

bool initSdCard() {
  Serial.printf("[SD] Initializing with CS pin %d...\n", SDcardPin);
  SPI.begin();
  pinMode(SDcardPin, OUTPUT);
  digitalWrite(SDcardPin, HIGH);
  if (!SD.begin(SDcardPin, SPI, 1000000)) {
    Serial.println("[SD] SD.begin failed.");
    return false;
  }
  Serial.println("[SD] SD.begin OK.");
  return true;
}

bool initFlashStorage() {
  Serial.println("[FFat] Mounting flash storage...");
  if (!FFat.begin(true)) {
    Serial.println("[FFat] FFat.begin failed.");
    return false;
  }
  Serial.println("[FFat] FFat mounted OK.");
  return true;
}

bool isStorageReady() {
  return STORAGE_MODE == 0 ? sdReady : flashReady;
}

const char* getStorageName() {
  return STORAGE_MODE == 0 ? "SD" : "FFat";
}

String getDisplayStorageLabel() {
  return STORAGE_MODE == 0 ? "SD card log" : "FFat flash files";
}

String buildMeasurementRecord(
  int moisture,
  int batteryMv,
  float temperatureC,
  float humidity,
  bool dhtOk,
  const uint16_t* channels,
  int channelCount
) {
  time_t now = time(nullptr);
  struct tm timeinfo;
  String record;

  record += "time_ms=";
  record += String(millis());
  if (now > 1700000000 && localtime_r(&now, &timeinfo) != nullptr) {
    char timestamp[24];
    snprintf(
      timestamp,
      sizeof(timestamp),
      "%04d-%02d-%02d %02d:%02d:%02d",
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec
    );
    record += ",time=";
    record += timestamp;
  }
  record += ",sensor=";
  record += DISPLAY_SENSOR_TYPE;
  record += ",sample_count=";
  record += String(sampleCounter);
  record += ",moisture=";
  record += String(moisture);
  record += ",voltage_mv=";
  record += String(batteryMv);
  // ★ A7：显式的数据质量位。
  //
  // 原来读失败写 "nan"，但下游分不清三件事：传感器读失败、传感器真的
  // 返回了 NaN、解析出错。在 esp32-autoflash 的评测里正好栽过同一个坑
  // ——「读数恒 0」到底是代码写坏了还是传感器坏了，只看数值无法归因。
  // 带上质量位，归因就从「猜」变成「读一个字段」。
  record += ",temp_c=";
  record += dhtOk ? String(temperatureC, 1) : String("nan");
  record += ",humidity=";
  record += dhtOk ? String(humidity, 1) : String("nan");
  record += ",dht_q=";
  record += dhtOk ? "ok" : "read_fail";
  // ★ A3：时钟来源。没对过时只有相对时间（time_ms + 采样序号），
  // 下游据此判断 time= 字段可不可信，而不是看到有值就当真。
  record += ",clk=";
  record += (now > 1700000000) ? "synced" : "uptime_only";
  record += ",spectral=";
  if (channelCount > 0) {
    for (int i = 0; i < channelCount; i++) {
      record += String(channels[i]);
      if (i + 1 < channelCount) {
        record += "|";
      }
    }
  } else {
    record += "none";
  }
  return record;
}

String buildFlashRecordPath() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  char path[48];

  if (now > 1700000000 && localtime_r(&now, &timeinfo) != nullptr) {
    snprintf(
      path,
      sizeof(path),
      "/%04d%02d%02d_%02d%02d%02d.txt",
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec
    );
  } else {
    snprintf(path, sizeof(path), "/sample_%010lu_%010lu.txt", (unsigned long)sampleCounter, (unsigned long)millis());
  }

  return String(path);
}

void appendMeasurementToSd(const String& record) {
  if (!sdReady) {
    return;
  }

  File file = SD.open(DISPLAY_DATA_FILE, FILE_APPEND);
  if (!file) {
    Serial.println("[SD] Open display data file failed.");
    return;
  }

  file.println(record);
  file.close();
  Serial.println("[SD] Measurement appended.");
}

void writeMeasurementToFlash(const String& record) {
  if (!flashReady) {
    return;
  }

  String path = buildFlashRecordPath();
  File file = FFat.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.printf("[FFat] Open flash data file failed: %s\n", path.c_str());
    return;
  }

  file.println(record);
  file.close();
  Serial.printf("[FFat] Measurement written: %s\n", path.c_str());
}

void writeMeasurementRecord(const String& record) {
  if (STORAGE_MODE == 0) {
    appendMeasurementToSd(record);
  } else {
    writeMeasurementToFlash(record);
  }
}

// ★ A1：流式输出历史记录，替代把整个日志文件拼进一个 String。
//
// 【为什么必须改】
// 每条记录约 150 字节，5 分钟一采 → 一天 43 KB、一周 300 KB。
// 而 ESP32-S3 nopsram 可用堆只有 ~300 KB（WiFi 栈还要吃几十 KB）。
// 更阴险的是 String::operator+= 内存不足时【静默失败】——返回被截断的
// 字符串，页面显示不全但不报错；而且几百次「分配-拷贝-释放」会把堆打成碎片。
//
// 改法：调用方给一个 emit 回调，这里读一行发一行，内存占用从
// O(文件大小) 降到 O(单行)。同时只回放最近 MAX_DISPLAY_ROWS 条——
// 仪表盘上翻几千行没有意义，而且传输本身也耗电。
static const size_t MAX_DISPLAY_ROWS = 60;

void streamDisplayStoredData(void (*emit)(const String&)) {
  if (!isStorageReady()) {
    emit(String("<li>") + getStorageName() + " storage not ready.</li>");
    return;
  }

  if (STORAGE_MODE != 0) {   // FFat 分支数据量小，沿用整段返回
    String html = getDisplayStoredDataHtml();
    emit(html);
    return;
  }

  File file = SD.open(DISPLAY_DATA_FILE, FILE_READ);
  if (!file) {
    emit("<li>No SD data file.</li>");
    return;
  }

  // 先数总行数，再跳到「倒数 MAX_DISPLAY_ROWS 行」开始发。
  // 只做两遍顺序扫描，不把内容留在内存里。
  size_t total = 0;
  while (file.available()) {
    if (file.readStringUntil('\n').length() > 0) total++;
  }
  size_t skip = total > MAX_DISPLAY_ROWS ? total - MAX_DISPLAY_ROWS : 0;

  file.seek(0);
  size_t idx = 0, sent = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (idx++ < skip) continue;
    emit(String("<li>") + line + "</li>");     // ← 每行独立发，发完即释放
    sent++;
  }
  file.close();

  if (sent == 0) {
    emit("<li>No records yet.</li>");
  } else if (skip > 0) {
    emit(String("<li class='muted'>… showing latest ") + sent +
         " of " + total + " records</li>");
  }
}

String getDisplayStoredDataHtml() {
  String html;
  html += "<ul>";

  if (!isStorageReady()) {
    html += "<li>";
    html += getStorageName();
    html += " storage not ready.</li>";
    html += "</ul>";
    return html;
  }

  bool hasLine = false;

  if (STORAGE_MODE == 0) {
    File file = SD.open(DISPLAY_DATA_FILE, FILE_READ);
    if (!file) {
      html += "<li>No SD data file.</li>";
      html += "</ul>";
      return html;
    }

    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        continue;
      }
      hasLine = true;
      html += "<li>";
      html += line;
      html += "</li>";
    }
    file.close();
  } else {
    File root = FFat.open("/");
    if (!root || !root.isDirectory()) {
      html += "<li>No FFat root directory.</li>";
      html += "</ul>";
      return html;
    }

    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        String line = entry.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          hasLine = true;
          html += "<li>";
          html += entry.name();
          html += ": ";
          html += line;
          html += "</li>";
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  if (!hasLine) {
    html += "<li>No samples stored yet.</li>";
  }

  html += "</ul>";
  return html;
}

String getDisplayLatestRecord() {
  // ★ A8 的用户可见收益：优先返回【内存里】刚采到的那条。
  // 改造前这里只能去存储里读上一条已落盘的记录 ——
  // 而常驻模式下根本不采样，所以网页上的"实时数据"其实是上次 logger
  // 模式留下的旧数据。现在采样任务一采完就 publish，不用等写卡。
  String live;
  if (tasksGetLatest(live)) {
    return live;
  }

  if (!isStorageReady()) {
    return "";
  }

  String latestLine;

  if (STORAGE_MODE == 0) {
    File file = SD.open(DISPLAY_DATA_FILE, FILE_READ);
    if (!file) {
      return "";
    }

    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) {
        latestLine = line;
      }
    }
    file.close();
  } else {
    File root = FFat.open("/");
    if (!root || !root.isDirectory()) {
      return "";
    }

    String latestName;
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        String name = entry.name();
        if (name > latestName) {
          latestName = name;
          String line = entry.readStringUntil('\n');
          line.trim();
          latestLine = line;
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  return latestLine;
}

void initializeLoggerPeripherals() {
  Serial.println("[INIT] Powering peripherals before logger initialization...");
  // setPeripheralSwitches(true);
  delay(300);

  dht.setup(sensor1Pin, DHTesp::DHT11);
  Serial.printf("[DHT11] setup done on GPIO %d.\n", sensor1Pin);

  spectralReady = initSpectralSensor();
  if (!spectralReady) {
    Serial.println("[AS7343] Spectral sensor unavailable for this cycle.");
  }

  if (STORAGE_MODE == 0) {
    sdReady = initSdCard();
    if (!sdReady) {
      Serial.println("[SD] SD unavailable for this cycle.");
    }
  } else {
    flashReady = initFlashStorage();
    if (!flashReady) {
      Serial.println("[FFat] Flash storage unavailable for this cycle.");
    }
  }
}

void initializeDisplayStorage() {
  Serial.println("[INIT] Powering peripherals before display storage initialization...");
  setPeripheralSwitches(true);
  delay(300);
  if (STORAGE_MODE == 0) {
    sdReady = initSdCard();
  } else {
    flashReady = initFlashStorage();
  }
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(led, OUTPUT);
  pinMode(switch1Pin, OUTPUT);
  pinMode(switch2Pin, OUTPUT);
  pinMode(switch3Pin, OUTPUT);
  pinMode(batteryLedPin, OUTPUT);
  pinMode((int)modePin, INPUT);
  setPeripheralSwitches(false);
  digitalWrite(batteryLedPin, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(sensor2Pin, ADC_11db);
  analogSetPinAttenuation(batteryAdcPin, ADC_11db);
  printWakeupReason();

  // Sample GPIO5 only here to decide which runtime branch should own loop().
  // HIGH means the user wants the local AP/web UI; LOW keeps normal logging.
  displayModeActive = isDisplayModeRequested();

  if (displayModeActive) {
    Serial.println("[MODE] GPIO5 HIGH -> display mode.");
    initializeDisplayStorage();
    displayGatewaySetup();
    logResourceWatermark("display-boot");
    watchdogBegin(30);              // 常驻态：30s 没喂狗说明卡在某个请求里

    // ★ A8：常驻模式才起四任务。起不来就退回原来的单线程 loop()，
    // 功能照旧，只是又变回"要么采样要么服务网页"。
    setPeripheralSwitches(true);
    initializeLoggerPeripherals();   // 采样任务要用的外设，先初始化好
    setPeripheralSwitches(false);
    tasksRunning = tasksBegin();
    return;
  }

  Serial.println("[MODE] GPIO5 LOW -> logger mode.");
  setPeripheralSwitches(true);
  initializeLoggerPeripherals();
  setPeripheralSwitches(false);

  // D6 used as wake/mode switch input
  // If external pull-up/pull-down is provided, INPUT may be used here
  // pinMode(wakePin, PULLDOWN);  // assuming you detect HIGH as trigger

  logResourceWatermark("logger-boot");
  watchdogBegin(60);                // 采样态：一轮含 2.5s 外设等待，留足余量

  Serial.println("\n================================");
  Serial.println("[INFO] Data logger mode enabled.");
}

// ====================== LOOP ======================
void loop() {
  if (displayModeActive) {
    // Display mode stays awake while GPIO5 remains HIGH. When GPIO5 stays LOW
    // for MODE_SWITCH_DELAY_MS, reboot so setup() can switch back to logger mode.
    if (isDisplayModeRequested()) {
      if (modePinLowTiming) {
        Serial.println("[MODE] GPIO5 returned HIGH. Cancel logger switch countdown.");
        modePinLowTiming = false;
      }
      watchdogFeed();
      // ★ A8：任务起来后，HTTP 由 net 任务处理，loop() 不再自己跑
      if (tasksRunning) { tasksIdle(); } else { displayGatewayLoop(); }
      return;
    }

    if (!modePinLowTiming) {
      modePinLowTiming = true;
      modePinLowStartMs = millis();
      Serial.println("[MODE] GPIO5 went LOW. Start logger switch countdown...");
    } else if (millis() - modePinLowStartMs >= MODE_SWITCH_DELAY_MS) {
      restartForModeSwitch("[MODE] GPIO5 stayed LOW long enough. Restarting into logger mode.");
    }

    if (tasksRunning) { tasksIdle(); } else { displayGatewayLoop(); }
    return;
  }

  // ------------- Check D6: if HIGH, enter 'forever-awake' mode -------------
  // if (digitalRead(wakePin) == HIGH) {
  //   if (!forever_awake) {
  //     Serial.println("\n[WAKE PIN TRIGGERED] D6 = HIGH → Forever-awake mode enabled. Device will NEVER sleep anymore!");
  //     forever_awake = true;
  //   }
  // }
  watchdogFeed();
  static unsigned long start_time = millis();
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - start_time;

  
  Serial.print("Elapsed time: ");
  Serial.print(elapsed_time / 1000);
  Serial.print("s / ");
  Serial.print(AWAKE_DURATION_MS / 1000);
  Serial.println("s");

  // LED blinking
  digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(led, LOW);
  delay(300);

  // -------- If in 'forever-awake' mode, skip deep-sleep ----------
  if (forever_awake) {
    Serial.println("[INFO] Forever-awake mode active. Skipping deep-sleep...");
    delay(1000);
    return;
  }

  // ================== Normal deep-sleep flow ==================
  if (elapsed_time >= AWAKE_DURATION_MS) {
    // Allow GPIO5 HIGH to interrupt the logger cycle immediately.
    if (isDisplayModeRequested()) {
      restartForModeSwitch("[MODE] GPIO5 HIGH detected. Restarting into display mode.");
    }

    setPeripheralSwitches(true);
    delay(500);
    initializeLoggerPeripherals();

    int moisture = getMoisture();
    int batteryMv = getBatteryMv();
    if (batteryMv > 0 && batteryMv < (int)(BATTERY_CUTOFF_V * 1000)) {
      enterLowBatteryShutdown(batteryMv);   // 在写卡【之前】判，确保还有电落盘
    }
    float temperatureC = 0.0f;
    float humidity = 0.0f;
    bool dhtOk = readDht11(temperatureC, humidity);
    int spectralCount = readSpectralChannels(spectralChannels, ksfAS7343NumChannels);

    sampleCounter++;
    Serial.printf("[SAMPLE] Count=%lu, record every %lu samples.\n", (unsigned long)sampleCounter, (unsigned long)RECORD_EVERY_SAMPLE_COUNT);

    if (sampleCounter % RECORD_EVERY_SAMPLE_COUNT == 0) {
      if (isStorageReady()) {
        String record = buildMeasurementRecord(
          moisture,
          batteryMv,
          temperatureC,
          humidity,
          dhtOk,
          spectralChannels,
          spectralCount
        );
        writeMeasurementRecord(record);
      } else {
        Serial.printf("[%s] Storage unavailable. Measurement not recorded.\n", getStorageName());
      }
    } else {
      Serial.println("[SAMPLE] Measurement sampled but not recorded this cycle.");
    }

    logResourceWatermark("post-sample");   // 每轮落一条，跑几天就能看出趋势

    Serial.println("\n================================");
    Serial.println("Logger cycle complete. Entering timed deep sleep...");
    Serial.print("Device will wake up after ");
    Serial.print(SLEEP_DURATION_US / 1000000ULL);
    Serial.println(" seconds, or immediately if GPIO5 goes HIGH.");
    Serial.println("================================\n");

    delay(2000);
    //delay>1s才能给DHT11留出采样时间
    setPeripheralSwitches(false);
    // Deep-sleep is temporarily disabled for debugging. Keep the same timing
    // budget by delaying for the original sleep interval, then restart the
    // awake window timer for the next sample cycle.
    enterTimedDeepSleep();

    // 2) D6 HIGH wakeup (EXT0)
    //    Note: EXT0 uses RTC IO; GPIO6 is a RTC IO on ESP32-S3 and is usable
    //    second parameter 1 = wake on HIGH, 0 = wake on LOW
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_6, 1);
    
    // Enter deep-sleep (program 'powers off' here; on wake, starts from setup())
    return;
    //模拟deepsleep
    // delay(SLEEP_DURATION_US / 1000);
    
  }

  delay(500);
}
