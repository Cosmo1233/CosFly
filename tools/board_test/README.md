# CosFly V1 board test

A standalone bring-up app, separate from the flight firmware. It checks each part of the
board in turn, prints PASS / WARN / FAIL with a hint for each failure, then streams live
sensor readings.

| Check | How |
|---|---|
| ESP32-S3, flash, USB | chip info, flash size (expect 8 MB) |
| I2C bus | SDA/SCL idle high before the driver starts, then an address scan for 0x10, 0x38, 0x50, 0x76 |
| BMP280 | chip ID 0x58, temperature and pressure in a plausible range |
| BMM150 | power-up, chip ID 0x32, raw field readings |
| PCA9534A + LEDs | pull-up levels on P0/P6, then a walk through D2 green/red, D3 blue, D9 red |
| 24AA64 EEPROM | write, read back and restore the last byte |
| BMI270 | INT1/INT2 idle high, chip ID 0x24, 8 KB config upload, Bosch accel self-test, data-ready pulse count on both INT pins, 1 g at rest |
| Battery ADC | VBAT through the 1/3 divider |
| BOOT button | GPIO0 high when released |
| Wi-Fi + antenna | network scan with signal strengths |
| Last reset | why the board last restarted (brownout is flagged), and whether a full-throttle test was cut short |

Keys in the live readout (`h` prints them):

| Key | Test |
|---|---|
| `m` | each motor alone at 25 %, with the battery voltage dip it causes (battery fitted, **props off**) |
| `t` | full throttle: all motors 0 → 25 → 50 → 75 → 100 %, logging battery sag, vibration and Wi-Fi sensitivity at each step (charged battery, **props on, strapped down**, capital `Y` to start, any key stops) |
| `v` | switch the breakout header's VCOM (VCOM_EXP, J3 pin 9) off / on |
| `l` | LED walk |
| `w` | antenna test (averaged scans, then a test access point for a phone to measure) |

BOOT presses are reported in the live readout. RESET restarts the board and reruns everything. The `up`
column shows uptime, so you can tell whether unplugging or plugging USB on battery reset the board.

## Run it

From the IDF PowerShell (the `IDF_v5.2.7_Powershell` shortcut):

```
cd C:\Users\cosmo\Documents\CosFly\tools\board_test
idf.py -p COMx flash monitor
```

`COMx` is the port the board shows up as in Device Manager. If it does not show up, or
flashing will not start, hold BOOT, tap RESET, release BOOT, and flash again.

Keep the board flat and still for the first few seconds (IMU checks). Press RESET to run
everything again; `Ctrl+]` leaves the monitor.

When you are done, flash the flight firmware again from the repository root with
`idf.py flash`.
