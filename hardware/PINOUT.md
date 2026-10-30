# StarvedLogger PCB 引脚对照表

**唯一事实来源：** `hardware/StarvedLogger_PCB/`（KiCad 工程）。
本文件是从 `StarvedLogger_PCB.kicad_pcb` 的网表机械导出 + 上板实测交叉验证的结果，
**不是手抄**。板子改版后重新导出，别手工维护。

- MCU：**ESP32-S3-MINI-1**（U1，无 PSRAM）
- 导出日期：2026-10-30
- 实测验证：2026-10-29/30，三态扫描（INPUT / PULLUP / PULLDOWN）+ 各子系统上板测试

> ⚠️ **封装脚号 ≠ GPIO 号。** 原理图上管脚旁的红色数字是模组的封装脚号。
> 例如 GPIO5 在封装脚 9，封装脚 5 是 GPIO1。看框内的 `IOxx` 名字，别看框外的数字。

---

## 一、完整映射

| GPIO | 封装脚 | 网络 | 用途 | 实测佐证 |
|---|---|---|---|---|
| GPIO0  | 4  | `Net-(U1-IO0)` | SW1 启动按键（R2 10k 上拉） | — |
| GPIO1  | 5  | `/ALERT` | TMP117 报警输出（R24 4.7k 上拉） | 三态恒 4095，spread=0 |
| GPIO2  | 6  | `/BAT_SENSE` | 电池电压分压 **ADC1_CH1** | 恒 3118，spread=0（低阻分压输出） |
| GPIO3  | 7  | `/LOW_BAT_LED` | 低电量指示灯（R12 1k → D1） | 约 1357，spread≈256 |
| GPIO5  | 9  | `Net-(U1-IO5)` | **J5** 三脚 3.3V 传感器信号（R5 1k 串联）→ DHT11 | SSR 合闸后被模块上拉钉在 4095 |
| GPIO8  | 12 | `/I2C_SDA 3.3v` | 3.3V I2C 数据（R? 4.7k 上拉） | 恒 4095，spread=0 |
| GPIO9  | 13 | `/I2C_SCL 3.3V` | 3.3V I2C 时钟（R8 4.7k 上拉） | 恒 4095，spread=0 |
| GPIO10 | 14 | `/I2C_SDA 5v` | 5V I2C 数据（R15 4.7k 上拉） | 恒 4095，spread=0 |
| GPIO11 | 15 | `/I2C_SCL 5v` | 5V I2C 时钟（R14 4.7k 上拉） | 恒 4095，spread=0 |
| GPIO12 | 16 | `Net-(J8-Pin_1)` | **J8** 六脚排针 第1脚 | 悬空 |
| GPIO13 | 17 | `Net-(J8-Pin_2)` | **J8** 第2脚 | 悬空 |
| GPIO14 | 18 | `Net-(J8-Pin_3)` | **J8** 第3脚 | 悬空 |
| GPIO15 | 19 | `Net-(J8-Pin_4)` | **J8** 第4脚 | 悬空 |
| GPIO16 | 20 | `Net-(J8-Pin_5)` | **J8** 第5脚 | 悬空 |
| GPIO17 | 21 | `Net-(J8-Pin_6)` | **J8** 第6脚 | 悬空 |
| GPIO18 | 22 | `Net-(U1-IO18)` | **SSR U7** 控制（R13 0.33k）→ `/6v Sensor_1` → J4 | — |
| GPIO21 | 25 | `Net-(U1-IO21)` | **J7** 三脚 6V 传感器信号（R4 1k 串联） | — |
| GPIO26 | 26 | `Net-(U1-IO26)` | **SSR U4** 控制（R3 0.33k）→ `/6v Sensor` → J7 | — |
| GPIO33 | 28 | `Net-(U1-IO33)` | **SSR U6** 控制（R9 0.33k）→ `/3.3v Sensor` → J6 | 拉高后 AS7343 出数 |
| GPIO34 | 29 | `Net-(J10-Pin_5)` | **J10** 五脚排针 第5脚 | — |
| GPIO35 | 31 | `/CS` | **SD 卡片选** → J3 脚2 | 实测读写通过 |
| GPIO36 | 32 | `/DI` | **SD 卡 MOSI**（主→卡）→ J3 脚3 | 实测读写通过 |
| GPIO37 | 33 | `/DO` | **SD 卡 MISO**（卡→主）→ J3 脚4 | 实测读写通过 |
| GPIO38 | 34 | `/CLK` | **SD 卡 SCK** → J3 脚5 | 实测读写通过 |
| GPIO39 | 35 | `Net-(J10-Pin_4)` | **J10** 第4脚 | — |
| GPIO40 | 36 | `Net-(J10-Pin_3)` | **J10** 第3脚 | — |
| GPIO41 | 37 | `Net-(J10-Pin_2)` | **J10** 第2脚 | — |
| GPIO47 | 27 | `Net-(U1-IO47)` | **SSR U5** 控制（R6 0.33k）→ `/3.3v Sensor_1` → J5 | 拉高后 DHT11 出数 |
| GPIO48 | 30 | `Net-(J10-Pin_1)` | **J10** 第1脚 | — |
| —      | 23/24 | `/USB-` `/USB+` | 原生 USB，经 R22/R23 22R 到 J9 USB-C | **任何情况下不要配置这两个脚** |
| —      | 39/40 | TXD0 / RXD0 | → J1 UART_DEBUG | — |
| —      | 45 | `Net-(U1-EN)` | SW2 复位（R1 10k 上拉 + C4 1µF） | — |

**未连接：** GPIO4、GPIO6、GPIO7（原理图上标 ✗，三态扫描确认悬空）。

---

## 二、固态继电器（电源开关）

四颗 **ASSR-1218**。对应的 GPIO 拉高才给那一路供电 —— **不拉高，挂在上面的传感器完全没电**。

| SSR | 控制 GPIO | 供电网络 | 接到 | 插座上有什么 |
|---|---|---|---|---|
| U6 | **GPIO33** | `/3.3v Sensor` | **J6** 四脚 | 3.3V I2C（AS7343 插这儿） |
| U5 | **GPIO47** | `/3.3v Sensor_1` | **J5** 三脚 | 3.3V 单总线（DHT11，信号在 GPIO5） |
| U4 | **GPIO26** | `/6v Sensor` | **J7** 三脚 | 6V 传感器（信号在 GPIO21） |
| U7 | **GPIO18** | `/6v Sensor_1` | **J4** 四脚 | 5V I2C |

每路都配一颗指示灯：D2(`/3.3v Sensor`)、D3(`/3.3v Sensor_1`)、D4(`/6v Sensor`)、D5(`/6v Sensor_1`)。

> **坑：** SSR 合闸或插拔传感器之后**必须复位 MCU**。初始化只在 `setup()` 里跑一次，
> 线接回去但不复位 = 传感器仍然停在坏的配置上，现象是读数恒 0 且不随光照/环境变化。
> 2026-10-30 的 AS7343 排查在这里卡了很久：接回松线后连读三次全 0，点一下 SW1 立刻出数。

---

## 三、直供（不受 SSR 控制）

| 器件 | 说明 |
|---|---|
| **U8 AP2112K-3.3** | LDO，`+6V` → `+3.3V` 主轨 |
| **U2 TMP117** | 板载温度传感器，`+3.3V` 直供，挂 3.3V I2C，报警线 → GPIO1。**一直在线** |
| **J3 SD 卡** | 脚7 = `+3.3V` 直供，**不需要开任何 SSR** |
| `+6V` | 来自 J2 BAT_IN |

---

## 四、J3（SD 卡）

| J3 脚 | 网络 | GPIO |
|---|---|---|
| 1 | 未连接 | — |
| 2 | `/CS` | GPIO35 |
| 3 | `/DI`（MOSI） | GPIO36 |
| 4 | `/DO`（MISO） | GPIO37 |
| 5 | `/CLK`（SCK） | GPIO38 |
| 6 | GND | — |
| 7 | +3.3V | — |
| 8 | 未连接 | — |

```cpp
SPI.begin(38, 37, 36, 35);        // SCK, MISO, MOSI, SS
SD.begin(35, SPI, 1000000);       // CS = GPIO35
```

> **不能用裸 `SPI.begin()`。** 默认值来自 Adafruit Feather 的 variant
> （MOSI=35 SCK=36 MISO=37 SS=42），和本板四条线错三条，且错得刚好互相串位：
> 旧的 MOSI(35) 是实际的 CS，旧的 SCK(36) 是实际的 MOSI。
>
> **GPIO35/36/37 在八线 PSRAM 的 S3 模组上归 PSRAM 用。** 本板是 MINI-1（无 PSRAM）
> 所以可用；换成 R8 之类型号这条 SPI 总线会整个废掉。

---

## 五、⚠️ 代码与硬件仍然对不上的地方

`src/main.cpp` 的引脚常量有一部分是照 **Adafruit Feather 开发板**写的，不是照本板。

| 项目 | `main.cpp` 当前值 | 本板实际 | 状态 |
|---|---|---|---|
| SD 四条线 | ~~T12 + 默认 SPI~~ | GPIO35/36/37/38 | ✅ **2026-10-30 已改正** |
| I2C | `Wire.begin(3, 4)` | GPIO8 / GPIO9 | ❌ 未改 |
| `sensor1Pin`（DHT11） | `A3` = GPIO15 | GPIO5 | ❌ 未改 |
| `batteryAdcPin` | `A2` = GPIO16 | GPIO2 | ❌ 未改 |
| `modePin` | GPIO5 | GPIO5 **被 DHT11 占用** | ❌ 冲突 |
| `BATTERY_DIVIDER_RATIO` | `3.0f` | **2.0**（R11 10k / R10 10k） | ❌ 未改 |
| SSR 供电 | 代码完全不管 | 四路都要先拉高才有电 | ❌ 缺失 |

两条要特别说明：

**`modePin` 和 DHT11 撞在 GPIO5 上。** 固件把 GPIO5 当模式选择输入读，而 DHT11 模块的
上拉让它恒为 HIGH → 永远进 display 模式，永远进不了 logger 模式。

**电池分压比是 2.0 不是 3.0。** 网表上 `/BAT_SENSE` 只接两颗电阻：R11 10k 接 `+6V`、
R10 10k 接 GND，分压比 = (10+10)/10 = **2.0**。代码里的 `3.0f` 和档案注释里写的
「200k/100k 分压」都是**旧台面接线**的参数，不是这块 PCB。按 3.0 算出来的电压会高 50%。

---

## 六、怎么重新导出

板子改版后，把 `.kicad_pcb` 的 footprint 块解析一遍即可（pad → `(net "…")` → `(pinfunction "IOxx_yy")`，
其中 `xx` 是 GPIO 号、`yy` 是封装脚号）。**不要手工维护这张表** —— 手抄一定会和板子漂移，
而漂移的表比没有表更危险。
