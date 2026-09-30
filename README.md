# CosFly

Flight firmware for **CosFly V1**, a small brushed-motor quadcopter built around the ESP32-S3.

> Status: work in progress. The firmware builds; it has not been flight-tested yet.

## Hardware

| Part | Chip | Interface |
|---|---|---|
| MCU | ESP32-S3FN8 | – |
| IMU | Bosch BMI270 | SPI |
| Barometer | Bosch BMP280 | I2C (0x76) |
| Magnetometer | Bosch BMM150 | I2C (0x10) |
| Status LEDs | TI PCA9534A GPIO expander | I2C (0x38) |
| Config EEPROM | Microchip 24AA64 | I2C (0x50) |
| Charger | TI BQ24075 | – |
| Motors | 4× brushed, SI2302 MOSFET drivers | PWM |

Schematics are in [`Documents/Schematics`](Documents/Schematics) and datasheets in [`hardware/datasheet`](hardware/datasheet). The pin map lives in [`sdkconfig.defaults.esp32s3`](sdkconfig.defaults.esp32s3).

## Building

Requires [ESP-IDF v5.2](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/index.html), target `esp32s3`.

```
idf.py build
idf.py flash monitor
```

Settings (pins, Wi-Fi name, LED options) are under **CosFly Config** in `idf.py menuconfig`.

## Layout

```
main/                    entry point, menuconfig definitions (Kconfig.projbuild)
components/core/         Crazyflie flight stack: stabilizer, controllers, estimators
components/drivers/      chip drivers (vendor code + ESP-IDF port files)
components/platform/     board definition (CosFly = "CF01")
components/config/       task priorities and stack sizes
Python-Scripts/          PC-side scripts using cflib
```

## Talking to it

The drone creates a Wi-Fi access point named `CosFly_<MAC>`. Connect to it and use
[cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/) or
[cflib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/) to fly, log data and tune parameters.

## Credits

CosFly is a port of an existing open-source stack:

- [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) by Bitcraze AB: the flight control core
- [ESP-Drone](https://github.com/espressif/esp-drone) by Espressif Systems: the ESP32 port
- [LiteWing](https://github.com/jobitjoseph/LiteWing) by CircuitDigest: the ESP32-S3 board adaptation this port started from

Vendor sensor drivers: Bosch [BMI270 SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI) (BSD-3-Clause), [LibDriver](https://github.com/libdriver) BMP280 and BMM150 (MIT), and the [PCA9534 driver](https://github.com/Hossein-M98) by Mahda Embedded System (MIT).

## License

GPL-3.0, inherited from Crazyflie and ESP-Drone. See [LICENSE](LICENSE). Vendor drivers keep their own licenses, noted in their source files.
