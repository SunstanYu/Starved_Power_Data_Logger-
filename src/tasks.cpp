// ★ A8：FreeRTOS 四任务改造
//
// ## 改之前的问题
//
// 常驻（display）模式下 loop() 里只有一句 displayGatewayLoop()。
// 它要么在等 HTTP 请求，要么在渲染页面 —— 【整个模式期间一次样都不采】。
// 所以用户看到的"实时数据"其实是上一次 logger 模式落盘的旧记录，
// 而拔下开关切回 logger 模式又意味着网页断了。
// 两件事共用一个线程，就只能二选一。
//
// ## 为什么是任务而不是状态机
//
// 也可以把 loop() 写成非阻塞状态机，但采样路径里有几段【真的阻塞】：
//   · DHT11 的单总线时序要 ~2.5s，且不能被打断
//   · SD 写入一次几十到几百 ms，取决于卡的擦写块状态
// 状态机里这些都得手工切成若干小步，每一步记住自己走到哪 ——
// 代码会变形，而且 DHT11 那段根本切不开。
//
// FreeRTOS 是抢占式的：任务可以【在任意指令处】被切走，
// 所以阻塞调用照原样写，调度器负责让别的任务继续跑。
// 代价是共享数据随时可能被切，必须自己加锁 ——
// 这正是下面 g_latestMux 的由来。
//
// ## 四个任务的划分依据：按【阻塞原因】分，不是按功能分
//
// | 任务 | 核 | 优先级 | 为什么单独一个任务 |
// | --- | --- | --- | --- |
// | Sensor  | 1 | 3 | 被 I²C / 单总线时序阻塞 |
// | Storage | 1 | 2 | 被 SPI + 文件系统阻塞，时长不可预测 |
// | Net     | 0 | 1 | 被 socket 阻塞；和 WiFi 驱动同核，少跨核传数据 |
// | Superv. | 0 | 4 | 谁都不等，所以能如实判断别人是不是卡住了 |
//
// 采样和存储之间用【队列】而不是共享缓冲区：一次 SD 写入慢下来时，
// 采样节奏不受影响，记录先在队列里排着。队列满了就丢最旧的 ——
// 对环境监测来说，新数据比旧数据有价值。
//
// 监控任务优先级最高（4）是有意的：它必须能抢占正在死等的任务，
// 否则一个卡住的低优先级任务会让"发现卡住"这件事本身也卡住。

#include "tasks.h"
#include "display_gateway.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// main.cpp 里的采样 / 存储原语
extern int getMoisture();
extern int getBatteryMv();
extern bool readDht11(float& temperatureC, float& humidity);
extern int readSpectralChannels(uint16_t* channels, size_t channelCapacity);
extern String buildMeasurementRecord(int moisture, int batteryMv, float temperatureC,
                                     float humidity, bool dhtOk,
                                     const uint16_t* spectralChannels, int spectralCount);
extern void writeMeasurementRecord(const String& record);
extern bool isStorageReady();
extern void setPeripheralSwitches(bool enabled);
extern void logResourceWatermark(const char* tag);
extern void enterLowBatteryShutdown(int batteryMv);
extern const float BATTERY_CUTOFF_V;
extern bool batteryShouldShutdown(int batteryMv);
extern uint16_t spectralChannels[];

// ---------------------------------------------------------------- 参数

static const uint32_t SENSOR_PERIOD_MS = 30 * 1000;   // 常驻模式下每 30s 采一次
static const uint32_t SUPERVISOR_PERIOD_MS = 5 * 1000;
static const uint32_t WDT_TIMEOUT_S = 60;             // 要盖得住 30s 采样周期 + 2.5s 时序
static const UBaseType_t RECORD_QUEUE_LEN = 4;

// ★ 队列里放的必须是 POD，不能是 String。
//
// xQueueSend 是按字节 memcpy 的。传 String 进去，复制的只是它内部那个
// 【堆指针】；发送方的局部 String 一出作用域就析构、把堆块还回去，
// 接收方拿到的指针就悬空了 —— 之后要么读到垃圾，要么二次 free 崩溃。
// 而且这种崩溃是间歇的：只有在分配器恰好复用了那块内存时才炸。
//
// 所以定长字符数组。代价是 4×264 ≈ 1KB 静态占用，换来的是不会有悬空指针。
struct RecordMsg { char line[264]; };

// 栈深按【实测水位】留 2 倍余量，见 Supervisor 里的 uxTaskGetStackHighWaterMark 日志。
static const uint32_t STACK_SENSOR  = 4096;
static const uint32_t STACK_STORAGE = 6144;   // 文件系统调用吃栈
static const uint32_t STACK_NET     = 8192;   // WebServer + String 渲染
static const uint32_t STACK_SUPERV  = 3072;

// ---------------------------------------------------------------- 共享状态

static QueueHandle_t      g_recordQ    = nullptr;
static SemaphoreHandle_t  g_latestMux  = nullptr;
// 递归锁：写入路径里可能嵌套调用其它也要锁的函数，
// 普通互斥量在这种情况下会把自己锁死。
static SemaphoreHandle_t  g_storageMux = nullptr;
static String             g_latest;            // ← 被 mux 保护
static volatile bool      g_latestValid = false;

static TaskHandle_t g_hSensor = nullptr, g_hStorage = nullptr,
                    g_hNet = nullptr, g_hSuperv = nullptr;

// 各任务的心跳。监控任务靠它判断谁卡住了。
// volatile + 32 位对齐写入在 Xtensa 上是原子的，所以这几个不用加锁。
static volatile uint32_t g_beatSensor = 0, g_beatStorage = 0, g_beatNet = 0;

static void publishLatest(const String& rec) {
  // ★ 这里必须加锁：String 的赋值不是原子的（要改指针、长度、可能重新分配堆）。
  // 抢占式调度下，Net 任务可能恰好在"指针已换、长度还没换"的瞬间读到它。
  if (xSemaphoreTake(g_latestMux, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_latest = rec;
    g_latestValid = true;
    xSemaphoreGive(g_latestMux);
  }
}

bool storageLock(uint32_t timeout_ms) {
  if (!g_storageMux) return true;          // 单线程路径，无需加锁
  return xSemaphoreTakeRecursive(g_storageMux, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void storageUnlock() {
  if (g_storageMux) xSemaphoreGiveRecursive(g_storageMux);
}

bool tasksGetLatest(String& out) {
  if (!g_latestMux || !g_latestValid) return false;
  if (xSemaphoreTake(g_latestMux, pdMS_TO_TICKS(50)) != pdTRUE) return false;
  out = g_latest;
  xSemaphoreGive(g_latestMux);
  return true;
}

// ---------------------------------------------------------------- 任务

static void sensorTask(void*) {
  esp_task_wdt_add(nullptr);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    g_beatSensor++;
    esp_task_wdt_reset();

    setPeripheralSwitches(true);
    vTaskDelay(pdMS_TO_TICKS(500));          // 外设上电稳定

    int moisture  = getMoisture();
    int batteryMv = getBatteryMv();
    float t = 0.0f, h = 0.0f;
    bool dhtOk = readDht11(t, h);            // ← 这里会阻塞 ~2.5s，Net 任务照跑
    int n = readSpectralChannels(spectralChannels, 12);

    setPeripheralSwitches(false);

    String rec = buildMeasurementRecord(moisture, batteryMv, t, h, dhtOk,
                                        spectralChannels, n);
    publishLatest(rec);                      // 网页立刻就能看到，不用等落盘

    RecordMsg msg;
    snprintf(msg.line, sizeof(msg.line), "%s", rec.c_str());

    // 队列满了就丢最旧的：新鲜数据比积压的旧数据有价值。
    if (xQueueSend(g_recordQ, &msg, 0) != pdTRUE) {
      RecordMsg drop;
      xQueueReceive(g_recordQ, &drop, 0);
      xQueueSend(g_recordQ, &msg, 0);
      Serial.println("[TASK] record queue full, dropped oldest");
    }

    if (batteryShouldShutdown(batteryMv)) {
      // ★ 先拿存储锁再停机。storage 任务可能正写到一半 ——
      // 直接 esp_deep_sleep_start() 会在写入中途掐断电，
      // 损坏的恰恰是 A5 本来要保护的那份数据。
      // 拿到锁 = 没有写入在进行中。拿不到也得走，但至少等满 5s。
      storageLock(5000);
      enterLowBatteryShutdown(batteryMv);    // 不返回
    }

    // vTaskDelayUntil 而不是 vTaskDelay：周期从【上次唤醒时刻】算，
    // 采样本身花掉的 3s 不会累积成漂移。
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

static void storageTask(void*) {
  esp_task_wdt_add(nullptr);
  RecordMsg msg;

  for (;;) {
    g_beatStorage++;
    esp_task_wdt_reset();

    // 阻塞等队列，超时只是为了能按时喂狗 —— 没数据时这个任务不占 CPU。
    if (xQueueReceive(g_recordQ, &msg, pdMS_TO_TICKS(2000)) == pdTRUE) {
      if (isStorageReady()) {
        writeMeasurementRecord(String(msg.line));  // ← SPI + 文件系统，可能几百 ms
      } else {
        Serial.println("[TASK] storage not ready, record kept in RAM only");
      }
    }
  }
}

static void netTask(void*) {
  esp_task_wdt_add(nullptr);

  for (;;) {
    g_beatNet++;
    esp_task_wdt_reset();
    displayGatewayLoop();
    vTaskDelay(pdMS_TO_TICKS(2));            // 让出 CPU，否则同核低优先级任务饿死
  }
}

static void supervisorTask(void*) {
  esp_task_wdt_add(nullptr);
  uint32_t pS = 0, pT = 0, pN = 0;
  bool first = true;

  for (;;) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));

    // 心跳不动 = 那个任务卡在某个调用里出不来。
    // Task WDT 最终也会抓到，但它抓到时是 panic 复位；
    // 这里先把【是谁】打出来，coredump 之外多一条线索。
    if (!first) {
      if (g_beatSensor == pS)  Serial.println("[SUPERV] ⚠ sensor task stalled");
      if (g_beatStorage == pT) Serial.println("[SUPERV] ⚠ storage task stalled");
      if (g_beatNet == pN)     Serial.println("[SUPERV] ⚠ net task stalled");
    }
    first = false;
    pS = g_beatSensor; pT = g_beatStorage; pN = g_beatNet;

    // A6 的水位日志，按任务分别报 —— 单看全局堆看不出是谁的栈快满了。
    Serial.printf("[SUPERV] stack_free sensor=%u storage=%u net=%u superv=%u  heap=%u min=%u\n",
                  (unsigned)uxTaskGetStackHighWaterMark(g_hSensor) * sizeof(StackType_t),
                  (unsigned)uxTaskGetStackHighWaterMark(g_hStorage) * sizeof(StackType_t),
                  (unsigned)uxTaskGetStackHighWaterMark(g_hNet) * sizeof(StackType_t),
                  (unsigned)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t),
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  }
}

// ---------------------------------------------------------------- 启动

static void teardown() {
  if (g_hSensor)  { vTaskDelete(g_hSensor);  g_hSensor = nullptr; }
  if (g_hStorage) { vTaskDelete(g_hStorage); g_hStorage = nullptr; }
  if (g_hNet)     { vTaskDelete(g_hNet);     g_hNet = nullptr; }
  if (g_hSuperv)  { vTaskDelete(g_hSuperv);  g_hSuperv = nullptr; }
  if (g_recordQ)  { vQueueDelete(g_recordQ); g_recordQ = nullptr; }
  if (g_latestMux){ vSemaphoreDelete(g_latestMux); g_latestMux = nullptr; }
  if (g_storageMux){ vSemaphoreDelete(g_storageMux); g_storageMux = nullptr; }
}

bool tasksBegin() {
  g_recordQ = xQueueCreate(RECORD_QUEUE_LEN, sizeof(RecordMsg));
  g_latestMux = xSemaphoreCreateMutex();
  g_storageMux = xSemaphoreCreateRecursiveMutex();
  if (!g_recordQ || !g_latestMux || !g_storageMux) {
    Serial.println("[TASK] queue/mutex alloc failed");
    teardown();
    return false;
  }

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg = {
      .timeout_ms = WDT_TIMEOUT_S * 1000,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif

  // 核绑定：Net 跟 WiFi 驱动都在核 0，采样/存储的阻塞 I/O 放核 1，
  // 免得 2.5s 的单总线时序把 WiFi 的软中断挤掉。
  BaseType_t ok = pdPASS;
  ok &= xTaskCreatePinnedToCore(sensorTask,     "sensor",  STACK_SENSOR,  nullptr, 3, &g_hSensor,  1);
  ok &= xTaskCreatePinnedToCore(storageTask,    "storage", STACK_STORAGE, nullptr, 2, &g_hStorage, 1);
  ok &= xTaskCreatePinnedToCore(netTask,        "net",     STACK_NET,     nullptr, 1, &g_hNet,     0);
  ok &= xTaskCreatePinnedToCore(supervisorTask, "superv",  STACK_SUPERV,  nullptr, 4, &g_hSuperv,  0);

  if (ok != pdPASS) {
    Serial.println("[TASK] xTaskCreate failed -> fall back to single-threaded loop");
    teardown();
    return false;
  }

  logResourceWatermark("tasks-started");
  Serial.println("[TASK] 4 tasks running (sensor/storage/net/supervisor)");
  return true;
}

void tasksIdle() {
  // loop() 自己也是一个任务（优先级 1，核 1）。四任务起来后它没事干，
  // 但【不能忙等】—— 那会白占核 1，把 sensor/storage 挤慢。
  vTaskDelay(pdMS_TO_TICKS(1000));
}
