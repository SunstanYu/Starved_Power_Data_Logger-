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
DHTesp dht;
SfeAS7343ArdI2C spectralSensor;
uint16_t spectralChannels[ksfAS7343NumChannels];
// const int wakePin   = T6;    // D6 = GPIO6 (used as normal input + DeepSleep wakeup)
bool forever_awake  = false; // never sleep after D6 is triggered

bool sdReady = false;
bool flashReady = false;
bool spectralReady = false;
bool displayModeActive = false;
bool modePinLowTiming = false;
unsigned long modePinLowStartMs = 0;
RTC_DATA_ATTR uint32_t sampleCounter = 0;

// ====================== HELPERS ======================
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
  record += ",temp_c=";
  if (dhtOk) {
    record += String(temperatureC, 1);
  } else {
    record += "nan";
  }
  record += ",humidity=";
  if (dhtOk) {
    record += String(humidity, 1);
  } else {
    record += "nan";
  }
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
    return;
  }

  Serial.println("[MODE] GPIO5 LOW -> logger mode.");
  setPeripheralSwitches(true);
  initializeLoggerPeripherals();
  setPeripheralSwitches(false);

  // D6 used as wake/mode switch input
  // If external pull-up/pull-down is provided, INPUT may be used here
  // pinMode(wakePin, PULLDOWN);  // assuming you detect HIGH as trigger

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
      displayGatewayLoop();
      return;
    }

    if (!modePinLowTiming) {
      modePinLowTiming = true;
      modePinLowStartMs = millis();
      Serial.println("[MODE] GPIO5 went LOW. Start logger switch countdown...");
    } else if (millis() - modePinLowStartMs >= MODE_SWITCH_DELAY_MS) {
      restartForModeSwitch("[MODE] GPIO5 stayed LOW long enough. Restarting into logger mode.");
    }

    displayGatewayLoop();
    return;
  }

  // ------------- Check D6: if HIGH, enter 'forever-awake' mode -------------
  // if (digitalRead(wakePin) == HIGH) {
  //   if (!forever_awake) {
  //     Serial.println("\n[WAKE PIN TRIGGERED] D6 = HIGH → Forever-awake mode enabled. Device will NEVER sleep anymore!");
  //     forever_awake = true;
  //   }
  // }
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
