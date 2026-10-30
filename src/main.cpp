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
const uint32_t MODE_SWITCH_DELAY_MS = 5000;         // modePin(GPIO12) must stay LOW for 5s before leaving display mode
const uint32_t RECORD_EVERY_SAMPLE_COUNT = 50;      // write one record every 50 samples
const int STORAGE_MODE = 0;                         // 0 = SD card, 1 = FFat flash
int led = LED_BUILTIN;
// ★ 2026-10-30：从 GPIO5 换到 GPIO12。
// GPIO5 在本板上是 J5 三脚传感器的信号脚（DHT11 插这儿），和模式选择【撞脚】——
// DHT11 模块的上拉让 GPIO5 恒为 HIGH，于是固件永远进 display 模式、永远进不了 logger。
// GPIO12 是 J8 六脚排针的第 1 脚，本板上空着。
// 选它还有一个硬约束：configureWakeSources() 用 modePin 做 EXT0 深睡唤醒源，
// 而 EXT0 要求 RTC GPIO —— ESP32-S3 的 RTC GPIO 只有 GPIO0~21。
// J8 那六个脚（GPIO12~17）都在范围内，GPIO35 那类就不行。
const gpio_num_t modePin = GPIO_NUM_12;             // LOW = logger mode, HIGH = display mode

// ---- 固态继电器 ASSR-1218：拉高才给对应那一路传感器供电 ----
// 来源 StarvedLogger_PCB.kicad_pcb，完整表见 hardware/PINOUT.md。
// 【为什么必须显式管】不拉高时传感器【完全没电】，而没电的 I2C/单总线
// 初始化仍可能逐步"返回成功"（库只检查通信错误，不回读验证），
// 症状只剩「读数恒 0 且不随环境变化」—— 和传感器损坏完全同构。
const int ssr3v3SensorPin  = 33;   // U6 -> /3.3v Sensor   -> J6 四脚 3.3V I2C
const int ssr3v3Sensor1Pin = 47;   // U5 -> /3.3v Sensor_1 -> J5 三脚 3.3V 单总线（DHT11）
const int ssr6vSensorPin   = 26;   // U4 -> /6v Sensor     -> J7 三脚 6V
const int ssr6vSensor1Pin  = 18;   // U7 -> /6v Sensor_1   -> J4 四脚 5V I2C
const uint32_t SENSOR_RAIL_SETTLE_MS = 500;   // 合闸后等轨电压建立 + 传感器上电复位

void enableSensorRails(bool on) {
  const int pins[] = {ssr3v3SensorPin, ssr3v3Sensor1Pin,
                      ssr6vSensorPin, ssr6vSensor1Pin};
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, on ? HIGH : LOW);
  }
  Serial.printf("[RAIL] sensor rails %s (GPIO %d/%d/%d/%d)\n",
                on ? "ON" : "OFF", ssr3v3SensorPin, ssr3v3Sensor1Pin,
                ssr6vSensorPin, ssr6vSensor1Pin);
}

// ⚠️ 本板上这三个脚不是电源开关，【不要再用】。保留仅为标注冲突：
//   T10 = GPIO10 -> /I2C_SDA 5v      T6 = GPIO6 -> 未连接
//   T9  = GPIO9  -> /I2C_SCL 3.3V
// 外设供电请用 enableSensorRails()。
const int switch1Pin = T10;    // = GPIO10，实为 I2C SDA 5V —— 勿驱动
const int switch2Pin = T6;     // = GPIO6，本板未连接
const int switch3Pin = T9;     // = GPIO9，实为 I2C SCL 3.3V —— 勿驱动
const int batteryLedPin = T13;


// ★ 2026-10-30：A3(GPIO15) → GPIO5。本板 J5 三脚 3.3V 传感器的信号脚经 R5 1k 接 GPIO5。
// 原来的 A3 是 Adafruit Feather variant 的别名，在本板上是 J8 的空脚。
// 供电走 U5 固态继电器（GPIO47），不拉高没电。
const int sensor1Pin = 5;    // DHT11 data pin -> J5 Pin_3
// ⚠️ 2026-10-30 未改，因为改不了 —— 这是个【硬件问题】，需要人决定：
//   · A4 = GPIO14，在本板上是 J8 六脚排针的第 3 脚，【空着】，
//     analogRead 读到的是悬空噪声（几百 mV，不是 0 —— 见下面 A5 那条注释的同款坑）
//   · 本板只剩 J7 一个三脚模拟插座，信号脚是 GPIO21
//   · 但 ESP32-S3 的 ADC 只有 GPIO1~10(ADC1) 和 GPIO11~20(ADC2)，
//     【GPIO21 根本不是 ADC 脚】，analogRead(21) 不可能工作
// 所以土壤湿度在这块板上没有可用的模拟通道。要么改板、要么换成数字/I2C 的湿度传感器。
const int sensor2Pin = A4;    // soil moisture sensor analog input -- 见上，本板无效
// ★ 2026-10-30：SD 卡四根线按 StarvedLogger_PCB.kicad_pcb 的网表改正。
//
// 【改之前是错的，而且错了三条】原来是 `SDcardPin = T12`（GPIO12）加裸
// `SPI.begin()`，后者走 Feather variant 的默认值 MOSI=35 SCK=36 MISO=37。
// 对照 J3 实际接线：
//       CS   旧 GPIO12 / 实际 GPIO35   ✗
//       MOSI 旧 GPIO35 / 实际 GPIO36   ✗（旧的 MOSI 正好是实际的 CS）
//       SCK  旧 GPIO36 / 实际 GPIO38   ✗（旧的 SCK 正好是实际的 MOSI）
//       MISO 旧 GPIO37 / 实际 GPIO37   ✓ 只有这条碰巧撞对
// 往 CS 线上发时钟、拿 MOSI 当片选，SD.begin() 不可能成功。
//
// 【来源】J3(Conn_01x08) 脚2/3/4/5 = /CS /DI /DO /CLK，
// 分别连 U1 封装脚 31/32/33/34 = IO35/IO36/IO37/IO38。
// 注意封装脚号不是 GPIO 号。详见 hardware/PINOUT.md。
//
// 【已上板实测】显式 SPI.begin(38,37,36,35) + SD.begin(35,...) 后，
// 写入→读回→逐字节比对连续 14 轮全部 match，文件字节数单调递增。
//
// GPIO35/36/37 在【八线 PSRAM】的 S3 模组上是给 PSRAM 用的、不可做 GPIO。
// 本板是 ESP32-S3-MINI-1（无 PSRAM）所以没问题；换成 R8 之类型号这条总线会整个废掉。
const int SDcardPin = 35;    // /CS  → U1 封装脚 31
const int SD_MOSI   = 36;    // /DI  → U1 封装脚 32（主→卡）
const int SD_MISO   = 37;    // /DO  → U1 封装脚 33（卡→主）
const int SD_SCK    = 38;    // /CLK → U1 封装脚 34
// ★ 2026-10-30：A2(GPIO16) → GPIO2。本板 /BAT_SENSE 网络接 U1 封装脚 6 = GPIO2（ADC1_CH1）。
// 实测佐证：三态扫描里 GPIO2 恒 3118 且 spread=0，是全板唯一的低阻分压输出。
const int batteryAdcPin = 2;
const char* DISPLAY_SENSOR_TYPE = "Soil moisture + DHT11 + AS7343";
const char* DISPLAY_DATA_FILE = "/sensor_log.txt";
const float ADC_VREF = 3.3f;
// ★ 2026-10-30：3.0 → 2.0。网表上 /BAT_SENSE 只接两颗电阻：
// R11 10k 接 +6V、R10 10k 接 GND，分压比 = (10+10)/10 = 2.0。
// 原来的 3.0 和档案里写的「200k/100k 分压」都是【旧台面接线】的参数，不是这块 PCB。
// 按 3.0 算会让电压虚高 50%（6V 的电池显示成 9V），直接影响下面的低电压截止判定。
const float BATTERY_DIVIDER_RATIO = 2.0f;
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

// ★ A5 修（2026-09-20 上板实测发现）
//
// 【现象】把固件烧进一块【没接电池】的裸板，上电不到 1 秒就永久睡死，
// USB 从系统里彻底消失，esptool 也够不着，只能按 RESET 救。
//
// 【原因】判据写的是 `batteryMv > 0`，本意是「读数有效」。
// 但 batteryAdcPin 悬空时 ADC 不读 0 —— 它读到的是几百 mV 的噪声。
// 于是 `> 0 && < 3700` 成立，节点认为自己没电了，执行永久停机。
//
// 【为什么这在田里是致命的】分压电阻虚焊、接插件松动、电池仓接触不良 ——
// 任何一种都会让 ADC 悬空，节点就把自己变成砖，而且【没有唤醒源】，
// 必须有人走到田里按复位。本来是保护数据的机制，变成了失效机制。
//
// 两道闸：
//   ① 合理性下限：4 节 AA 经 ÷3 分压，哪怕快没电也在 3V 以上。
//      读到低于 1V 说明【没接电池 / ADC 悬空】，不是「电池空了」。
//   ② 连续判定：单次 ADC 毛刺不该让一个田间节点停机。
// ★ 第二次修（第一次的判据是错的，上板又睡死一次才发现）
//
// 第一版下限设 1000mV，依据是「悬空 ADC 读数低」。**这个假设不成立** ——
// 悬空引脚读的是随机的中间值，不是 0 附近。分压比 3:1，ADC 只要落在
// 0.33~1.23V，换算出来就是 1.0~3.7V，正好落进低电量窗口，
// 连续 3 次照样停机。加了连续判定只是把「1 秒睡死」变成「90 秒睡死」。
//
// 【按物理量重新定界】供电是 4 节 AA 碱性电池：
//     满电 6.4V ·  标称 6.0V ·  0.9V/节 = 3.6V 已经耗尽
// 所以 3.7V 截止是对的。但 3.0V（0.75V/节）是**深度放空**，
// 一个还能跑 WiFi 的节点不可能读到这个值 —— 读到就说明【线路断了】，
// 不是电池空了。下限提到 3000mV，可动作窗口变成 3.0~3.7V。
//
// 【再加一道：稳定性】电平本身区分不了悬空和真电压，但**噪声能**：
// 接了电池经分压出来的读数是稳的，悬空引脚会飘。
// 所以不只要连续 3 次低，还要这 3 次的极差小于 300mV。
const int BATTERY_SANITY_FLOOR_MV = 3000;   // 0.75V/节 = 深度放空，读到只能是线路断了
const int BATTERY_LOW_STREAK = 3;           // 连续 3 次
const int BATTERY_STABLE_SPREAD_MV = 300;   // 这 3 次的极差上限——悬空会飘，电池不会

// 返回 true 表示「确实该停机了」。
bool batteryShouldShutdown(int batteryMv) {
  static int streak = 0;
  static int lo = 0, hi = 0;

#ifdef BENCH_NO_BATTERY
  // ★ 独立开关，不再跟 FORCE_DISPLAY_MODE 绑在一起。
  //
  // 这两个是【两件事】：FORCE_DISPLAY_MODE 是「没接跳线」，
  // BENCH_NO_BATTERY 是「没接电池」。第一版把旁路写在
  // #ifdef FORCE_DISPLAY_MODE 里，结果台面验证时这个函数【一进来就返回】，
  // 下面那两道判据（合理性下限、稳定性）一次都没在真硬件上跑过 ——
  // 而它们恰恰是那个把板子睡死两次的 bug 的修复本身。
  //
  // 耦合两个无关的开关，等于让其中一个永远测不到。
  Serial.printf("[BAT] %d mV (bench: low-battery shutdown disabled)\n", batteryMv);
  return false;
#endif

  if (batteryMv < BATTERY_SANITY_FLOOR_MV) {
    streak = 0;
    Serial.printf("[BAT] %d mV below sanity floor %d -> NO BATTERY / wiring fault, "
                  "not low battery\n", batteryMv, BATTERY_SANITY_FLOOR_MV);
    return false;                       // 线路断了，绝不停机
  }
  if (batteryMv >= (int)(BATTERY_CUTOFF_V * 1000)) {
    streak = 0;
    return false;
  }

  if (streak == 0) { lo = hi = batteryMv; }
  else { if (batteryMv < lo) lo = batteryMv; if (batteryMv > hi) hi = batteryMv; }
  streak++;

  if (streak < BATTERY_LOW_STREAK) {
    Serial.printf("[BAT] %d mV below cutoff (%d/%d) — waiting for confirmation\n",
                  batteryMv, streak, BATTERY_LOW_STREAK);
    return false;
  }
  if (hi - lo > BATTERY_STABLE_SPREAD_MV) {
    Serial.printf("[BAT] readings unstable (spread %d mV > %d) -> floating pin, "
                  "not a dying battery\n", hi - lo, BATTERY_STABLE_SPREAD_MV);
    streak = 0;
    return false;                       // 飘 = 悬空，不是电池在掉电
  }
  return true;
}
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

  // ★ 这里原本直接 SD.begin 而没有 SPI.begin —— 本函数在启动早期跑，
  // 可能先于 initSdCard()，那时 SPI 还没按本板引脚配过。
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SDcardPin);
  pinMode(SDcardPin, OUTPUT);
  digitalWrite(SDcardPin, HIGH);
  if (!SD.begin(SDcardPin, SPI, 1000000)) {
    Serial.println("[OTA] FAIL: SD not available -> rollback");
    return false;                          // 存储起不来 = 数据会丢，不如退回
  }
  SD.end();

  // 同一个坑：`mv > 0` 挡不住悬空 ADC。分压虚焊会让每次 OTA 都无条件回滚，
  // 而回滚是【静默】的 —— 你只会看到"升级没生效"，查不到为什么。
  int mv = getBatteryMv();
  if (mv >= BATTERY_SANITY_FLOOR_MV && mv < (int)(BATTERY_CUTOFF_V * 1000)) {
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

  // ★ 改：留一个【很长周期】的定时唤醒，而不是彻底不醒。
  //
  // 原设计是无唤醒源，理由是「不会再自己醒来采样」。但那等于
  // 把「需要人工干预」写死进固件 —— 而碱性电池在低温下电压会回升，
  // 白天升温后可能又能撑一阵；分压虚焊修好后也该能自己恢复。
  //
  // 6 小时醒一次只是读个 ADC 再睡，占空比接近 0，
  // 对续航的影响可以忽略，换来的是【可恢复】。
  esp_sleep_enable_timer_wakeup(6ULL * 3600ULL * 1000000ULL);
  esp_deep_sleep_start();
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
// ★ 2026-10-30 重写。原来这里对 switch1/2/3Pin 做 digitalWrite，而在本板上：
//     switch1Pin = T10 = GPIO10 -> /I2C_SDA 5v
//     switch2Pin = T6  = GPIO6  -> 网表里根本没有 IO6（未连接）
//     switch3Pin = T9  = GPIO9  -> /I2C_SCL 3.3V      ★
// 等于把两条 I2C 线当推挽输出在驱动。三个脚在本板上【没有一个是电源开关】——
// 真正的外设供电是四路 ASSR-1218 固态继电器（见 enableSensorRails）。
//
// 【为什么以前没炸】logger 模式每次深睡醒来 setup() 重跑、Wire.begin() 把引脚
// 抢回去，所以侥幸能用。但四任务模式里每轮采样结束会调 (false)，
// 把 SCL 钳在 LOW 且之后没有 Wire.begin —— I2C 从此再也起不来。
void setPeripheralSwitches(bool enabled) {
  enableSensorRails(enabled);
}

// modePin(GPIO12) is the single hardware mode selector for the whole firmware:
// - LOW  => sensor logger mode
// - HIGH => local display gateway mode
bool isDisplayModeRequested() {
#ifdef FORCE_DISPLAY_MODE
  // 测试钩子：常驻模式要求 modePin(GPIO12) 拉高，而那需要一根跳线。
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
      Serial.println("[WAKE] Wakeup caused by EXT0 (GPIO12 HIGH).");
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
// 2. EXT0 wakeup on modePin(GPIO12) HIGH so the user can force display mode while asleep
void configureWakeSources() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_sleep_enable_ext0_wakeup(modePin, 1);
}

// A reboot is the cleanest way to switch modes at runtime because setup()
// re-runs the same initialization path and re-evaluates modePin.
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

// ---- I2C 总线（GPIO8=SDA / GPIO9=SCL，总线自带 4.7k 上拉）----
// 单独成函数：这条总线上挂着两个来源完全不同的器件 ——
// TMP117 是板载、+3.3V 直供、永远在线；AS7343 是插在 J6 上、受 U6 继电器(GPIO33)控制。
// 总线初始化必须早于两者，且不能埋在任何一个器件的 init 里。
void initI2cBus() {
  Wire.begin(8, 9);
  Wire.setClock(100000);
  Serial.println("[I2C] bus up on SDA=8 SCL=9 @100kHz");
}

// ---- TMP117 板载温度传感器（U2 @ 0x48）----
// 不引库：寄存器就三个，裸 I2C 比多一个依赖干净。
//   0x00 温度结果  16 位有符号，7.8125 m°C/LSB
//   0x0F 器件 ID   固定 0x0117
// 2026-10-30 上板实测：ID 读回 0x0117，31.8~32.1°C，抖动 24~101 m°C，23 轮零失败。
const uint8_t TMP117_ADDR      = 0x48;   // ADD0 接 GND
const uint8_t TMP117_REG_TEMP  = 0x00;
const uint8_t TMP117_REG_ID    = 0x0F;
const uint16_t TMP117_DEVICE_ID = 0x0117;
bool tmp117Ready = false;

static bool tmp117Read16(uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)TMP117_ADDR, 2) != 2) return false;
  out = ((uint16_t)Wire.read() << 8) | Wire.read();     // 大端
  return true;
}

bool initTmp117() {
  uint16_t id = 0;
  if (!tmp117Read16(TMP117_REG_ID, id)) {
    Serial.println("[TMP117] no response at 0x48.");
    return false;
  }
  // ★ 判据是器件 ID，不是「地址有应答」。
  // 地址应答只说明总线上有东西；读到 0x0117 才说明那东西是 TMP117。
  // 这是今天反复撞到的那个形状的解药：「调用没报错」≠「拿到的是对的东西」。
  if (id != TMP117_DEVICE_ID) {
    Serial.printf("[TMP117] wrong device id 0x%04X (expect 0x%04X).\n", id, TMP117_DEVICE_ID);
    return false;
  }
  Serial.println("[TMP117] ready, id=0x0117.");
  return true;
}

bool readTmp117(float& degC) {
  if (!tmp117Ready) return false;
  uint16_t v;
  if (!tmp117Read16(TMP117_REG_TEMP, v)) {
    Serial.println("[TMP117] read failed.");
    return false;
  }
  degC = (int16_t)v * 0.0078125f;
  Serial.printf("[TMP117] Board temperature: %.4f C\n", degC);
  return true;
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

  // ★ 2026-10-30：判据补一条「全 0 也算失败」。
  // 原来只判 isnan，而 DHT11 读不到时返回的是 0.0/0.0 —— 【0.0 不是 NaN】，
  // 原判据直接放它过去，一条 t=0.0 h=0.0 的垃圾读数会被当成有效测量写进记录。
  // 和本仓库 A5 那条「batteryMv > 0 挡不住悬空 ADC」是同一个形状：
  // 判据写的是「不是非法值」，想表达的是「读数有效」。
  //
  // 不加重试、不加延时：实测加了 2.4s 的重试与重新 setup 完全无效，
  // 说明当前读不到的原因不在时序。原有节奏保持不变。
  if (isnan(humidity) || isnan(temperatureC)
      || (humidity == 0.0f && temperatureC == 0.0f)) {
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
  // I2C 总线已由 setup() 的 initI2cBus() 初始化 —— 不在这里 begin。
  // 原因：TMP117(U2) 和 AS7343 共用这条总线，但 TMP117 是 +3.3V 直供、
  // 永远在线，不该因为 AS7343 初始化失败或顺序调整就读不到。
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
  Serial.printf("[SD] Initializing with CS=%d SCK=%d MISO=%d MOSI=%d...\n",
                SDcardPin, SD_SCK, SD_MISO, SD_MOSI);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SDcardPin);   // 必须显式指定：默认值是 Feather 的，和本板不符
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
  int channelCount,
  float boardTempC,
  bool tmpOk
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
  // ★ 2026-10-30 新增：TMP117 板载温度。和 DHT11 的 temp_c 是两回事 ——
  // DHT11 量的是 J5 外接探头处的环境温度，TMP117 焊在板上、量的是板子自身温度
  // （实测比室温高几度，挨着 ESP32 有自热）。两个都留着，别合并。
  // 质量位同样显式给，理由见上面 A7 那条。
  record += ",board_temp_c=";
  record += tmpOk ? String(boardTempC, 4) : String("nan");
  record += ",tmp_q=";
  record += tmpOk ? "ok" : "read_fail";
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

// ★ A8：存储访问的 RAII 锁。
//
// 下面几个函数都有多个提前返回路径，手工配对 storageLock/storageUnlock
// 迟早会漏掉一条 —— 漏一次就是永久死锁。让析构函数去做。
//
// 四任务没起来时 storageLock() 是空操作，所以 logger 模式不受影响。
struct StorageGuard {
  bool held;
  explicit StorageGuard(uint32_t ms = 4000) : held(storageLock(ms)) {}
  ~StorageGuard() { if (held) storageUnlock(); }
};

void writeMeasurementRecord(const String& record) {
  StorageGuard g;                       // ← net 任务可能正在读同一张卡
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
  // 整段发送期间都持锁。发一个页面可能要几秒（分块走 WiFi），
  // 这段时间 storage 任务写不进去 —— 但队列能缓 4 条 × 30s = 2 分钟，
  // 缓冲足够，而正确性不能让。
  StorageGuard g;
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
  StorageGuard g;                       // 递归锁，被上面那个函数嵌套调用也安全
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

  StorageGuard g;
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

  initI2cBus();                       // 必须早于 TMP117 和 AS7343
  spectralReady = initSpectralSensor();
  tmp117Ready = initTmp117();         // 板载、直供，和 AS7343 成败无关
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

  // ★ 2026-10-30：上电标记行。
  // 卡上是 append 的历史数据，光看记录分不出「这一行是这次启动写的」还是
  // 「上次运行留下的」。排查 AS7343 时正是「线接回去但没复位、读的还是旧状态」
  // 这个形状耗掉了很多时间 —— 标记行让每次上电在数据里留下明确的分界。
  if (isStorageReady()) {
    writeMeasurementRecord("2026/10/30 test：");
    Serial.println("[SD] boot marker written.");
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
  // switch1/2/3Pin 在本板上是 I2C 线和未连接脚，【不配成 OUTPUT】——
  // 配了就会在 Wire.begin() 之前/之后抢走 SCL。理由见 setPeripheralSwitches()。
  pinMode(batteryLedPin, OUTPUT);
  // ★ 2026-10-30：INPUT → INPUT_PULLDOWN。
  // modePin 换到 GPIO12(J8 脚1) 之后【物理上悬空】——板上没有任何上下拉，
  // 而 isDisplayModeRequested() 判的是 digitalRead()==HIGH，悬空脚读数随机，
  // 开机进哪个模式就成了掷硬币。下拉让默认落在 LOW = logger 模式，
  // 和本文件开头对 modePin 的定义一致。
  // 深睡时保持低电平还顺带避免 EXT0(configureWakeSources) 被浮空噪声误唤醒。
  pinMode((int)modePin, INPUT_PULLDOWN);
  setPeripheralSwitches(false);
  digitalWrite(batteryLedPin, LOW);

  // ★ 2026-10-30：先给传感器那几路上电，再做任何传感器初始化。
  // 顺序反了的话 begin() 会在没电的总线上跑完并"成功"，之后读数恒 0。
  enableSensorRails(true);
  delay(SENSOR_RAIL_SETTLE_MS);

  analogReadResolution(12);
  analogSetPinAttenuation(sensor2Pin, ADC_11db);
  analogSetPinAttenuation(batteryAdcPin, ADC_11db);
  printWakeupReason();

  // Sample modePin (GPIO12) only here to decide which runtime branch should own loop().
  // HIGH means the user wants the local AP/web UI; LOW keeps normal logging.
  displayModeActive = isDisplayModeRequested();

  if (displayModeActive) {
    Serial.println("[MODE] GPIO12 HIGH -> display mode.");
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

  Serial.println("[MODE] GPIO12 LOW -> logger mode.");
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
    // Display mode stays awake while GPIO12 remains HIGH. When GPIO12 stays LOW
    // for MODE_SWITCH_DELAY_MS, reboot so setup() can switch back to logger mode.
    if (isDisplayModeRequested()) {
      if (modePinLowTiming) {
        Serial.println("[MODE] GPIO12 returned HIGH. Cancel logger switch countdown.");
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
      Serial.println("[MODE] GPIO12 went LOW. Start logger switch countdown...");
    } else if (millis() - modePinLowStartMs >= MODE_SWITCH_DELAY_MS) {
      restartForModeSwitch("[MODE] GPIO12 stayed LOW long enough. Restarting into logger mode.");
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
    // Allow GPIO12 HIGH to interrupt the logger cycle immediately.
    if (isDisplayModeRequested()) {
      restartForModeSwitch("[MODE] GPIO12 HIGH detected. Restarting into display mode.");
    }

    setPeripheralSwitches(true);
    delay(500);
    initializeLoggerPeripherals();

    int moisture = getMoisture();
    int batteryMv = getBatteryMv();
    if (batteryShouldShutdown(batteryMv)) {
      enterLowBatteryShutdown(batteryMv);   // 在写卡【之前】判，确保还有电落盘
    }
    float temperatureC = 0.0f;
    float humidity = 0.0f;
    bool dhtOk = readDht11(temperatureC, humidity);
    float boardTempC = 0.0f;
    bool tmpOk = readTmp117(boardTempC);
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
          spectralCount,
          boardTempC,
          tmpOk
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
    Serial.println(" seconds, or immediately if GPIO12 goes HIGH.");
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
