# Starved Power Data Logger

ESP32-S3 data logger for low-power sensor collection with SD card storage and a local Wi-Fi display mode. The firmware samples soil moisture, DHT11 temperature/humidity, AS7343 spectral channels, and battery voltage, then appends periodic records to a single SD card log file.

## Hardware Target

- Board: Adafruit Feather ESP32-S3 No PSRAM
- Framework: Arduino through PlatformIO
- Storage: SPI SD card by default
- Sensors:
  - DHT11 temperature/humidity sensor
  - Analog soil moisture sensor
  - SparkFun AS7343 spectral sensor over I2C
- Battery monitor: ADC voltage divider, expected 200k upper / 100k lower

## Pin Map

| Function | Pin |
| --- | --- |
| Mode switch | GPIO5 |
| DHT11 data | A3 |
| Soil moisture ADC | A4 |
| Battery ADC | A2 |
| SD card CS | T12 |
| Battery LED | T13 |
| Peripheral switch 1 | T10 |
| Peripheral switch 2 | T6 |
| Peripheral switch 3 | T9 |
| AS7343 I2C SDA | GPIO3 |
| AS7343 I2C SCL | GPIO4 |

The SD card uses the board's default SPI pins. Only CS is configured explicitly.

## Operating Modes

GPIO5 selects the runtime mode:

- LOW: logger mode
- HIGH: local display mode

In logger mode, the device samples sensors, records data periodically, then enters timed deep sleep. GPIO5 is also configured as an EXT0 wake source, so driving GPIO5 HIGH can wake the device into display mode.

In display mode, the ESP32-S3 starts a local Wi-Fi access point and serves a dashboard from the device.

Display access point:

- SSID: `ESP32S3-Gateway`
- Password: `12345678`
- URL: `http://192.168.10.1/`

To leave display mode, hold GPIO5 LOW for 5 seconds. The firmware restarts and returns to logger mode.

## Sampling and Storage

Current logger timing:

- Awake window: 20 seconds
- Deep sleep duration: 10 seconds
- Record interval: every 50 samples

Storage mode is currently SD card:

```cpp
const int STORAGE_MODE = 0;
```

The SD card is initialized with a conservative SPI frequency:

```cpp
SD.begin(SDcardPin, SPI, 1000000);
```

Records are appended to:

```text
/sensor_log.txt
```

## Log Format

Each record is one line of comma-separated `key=value` fields:

```text
time_ms=23034,time=2027-02-13 01:31:08,sensor=Soil moisture + DHT11 + AS7343,sample_count=100,moisture=4095,voltage_mv=4670,temp_c=24.1,humidity=23.0,spectral=0|0|0|0|0|2|0|0|0|0|0|2|0|0|0|0|0|2
```

Fields:

- `time_ms`: milliseconds since the current boot
- `time`: system time if the ESP32 time source is valid
- `sensor`: sensor stack label
- `sample_count`: retained sample counter
- `moisture`: raw ADC reading from the soil moisture sensor
- `voltage_mv`: calculated battery voltage in millivolts
- `temp_c`: DHT11 temperature in Celsius, or `nan` on failed read
- `humidity`: DHT11 relative humidity, or `nan` on failed read
- `spectral`: AS7343 channel values separated by `|`

Note: `time_ms` resets after deep sleep because the chip restarts. The `time` field is only trustworthy after a valid time source is configured.

## Battery Voltage Calculation

The firmware assumes a 200k / 100k divider:

```text
Vadc = Vbat / 3
Vbat = Vadc * 3
```

The calculation is:

```cpp
int raw = analogRead(batteryAdcPin);
float vadc = ((float)raw / 4095.0f) * 3.3f;
float vbat = vadc * 3.0f;
int batteryMv = (int)(vbat * 1000.0f);
```

ADC setup:

```cpp
analogReadResolution(12);
analogSetPinAttenuation(batteryAdcPin, ADC_11db);
```

The battery measurement point should be the battery positive terminal through the divider. Do not hard-wire USB/VBUS and the battery positive terminal together unless the board's power path explicitly supports that connection.

## Build and Upload

Install PlatformIO, then run:

```sh
pio run
```

Upload to the board:

```sh
pio run --target upload
```

Open the serial monitor:

```sh
pio device monitor --baud 115200
```

## Project Layout

```text
.
├── platformio.ini
├── partitions.csv
├── src
│   ├── main.cpp
│   ├── display_gateway.cpp
│   └── display_gateway.h
├── include
├── lib
└── test
```

## Notes

- `.pio` build output is ignored by Git.
- The project includes an FFat partition, but the active storage mode is SD card.
- The display dashboard reads the same storage source used by the logger.
- If the AS7343 values saturate, reduce gain or integration time in the sensor configuration.
- If soil moisture stays near 4095, verify the sensor output range and ADC wiring.
