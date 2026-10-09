/*
 * CosFly V1 board bring-up test.
 *
 * A small standalone app, NOT the flight firmware. It checks the parts on the
 * board one at a time, prints PASS / WARN / FAIL for each with a hint at what
 * to probe when something fails, then streams live sensor readings so you can
 * tilt and turn the board and watch them respond.
 *
 * Order matters: the cheapest, most fundamental checks come first (do the bus
 * lines even idle high?) so a failure further down is easier to explain.
 *
 * Pins and addresses below come from the CosFly V1 schematics and match
 * sdkconfig.defaults.esp32s3 in the flight firmware.
 */
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_dev.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "bmi270.h"

/* ---------------------------------------------------------------------------
 * Board definition (CosFly V1)
 * ------------------------------------------------------------------------- */
#define PIN_I2C_SDA      33     // INT_SDA, pull-up R7
#define PIN_I2C_SCL      34     // INT_SCL, pull-up R8
#define PIN_SPI_MISO     37
#define PIN_SPI_MOSI     38
#define PIN_SPI_SCK      39
#define PIN_SPI_CS       40
#define PIN_IMU_INT1     36     // pull-up R27
#define PIN_IMU_INT2     35     // pull-up R29
#define PIN_BOOT_BTN     0      // S1, pull-up R16, low when pressed
#define VBAT_ADC_CHANNEL ADC_CHANNEL_3  // GPIO4
#define VBAT_DIVIDER     3.0f   // R15 200k / R18 100k

#define ADDR_BMM150      0x10
#define ADDR_PCA9534A    0x38
#define ADDR_EEPROM      0x50
#define ADDR_BMP280      0x76

// PCA9534A (U3) port pins
#define EXP_GREEN_L      1      // D2 green, active low
#define EXP_RED_L        2      // D2 red, active low
#define EXP_BLUE         3      // D3 blue, active high
#define EXP_PWR_SEL      6      // D9 power LED colour via S4: high/input = green, low = red
#define EXP_VCOM_EN      0      // U2 breakout VCOM switch: input (R12 pull-up) = on, low = off

static const struct {
    int gpio;
    const char *name;
    const char *gateResistor;
} motors[4] = {
    { 42, "M1 front-right (Motor R-Up,   Q4, J8)", "R22" },
    { 7,  "M2 back-right  (Motor R-Down, Q3, J7)", "R21" },
    { 8,  "M3 back-left   (Motor L-Down, Q1, J5)", "R19" },
    { 2,  "M4 front-left  (Motor L-Up,   Q2, J6)", "R20" },
};
#define MOTOR_TEST_DUTY   64    // of 255, i.e. 25 %
#define MOTOR_ALL_DUTY    128   // 50 %, all four together
#define MOTOR_ALL_SECONDS 10

/* ---------------------------------------------------------------------------
 * Result bookkeeping
 * ------------------------------------------------------------------------- */
typedef enum { PASS, WARN, FAIL } verdict_t;
static const char *const verdictName[] = { "PASS", "WARN", "FAIL" };

#define MAX_RESULTS 32
static struct {
    const char *test;
    verdict_t verdict;
} results[MAX_RESULTS];
static int resultCount;

static void section(const char *title)
{
    printf("\n--- %s ---\n", title);
}

static void report(const char *test, verdict_t verdict, const char *fmt, ...)
{
    va_list args;
    printf("  [%s] %-20s ", verdictName[verdict], test);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");

    if (resultCount < MAX_RESULTS) {
        results[resultCount].test = test;
        results[resultCount].verdict = verdict;
        resultCount++;
    }
}

// Waits up to timeoutMs for a key typed in the serial monitor. -1 if none.
static int readKey(int timeoutMs)
{
    uint8_t c;
    return usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(timeoutMs)) > 0 ? c : -1;
}

/* ---------------------------------------------------------------------------
 * I2C helpers (one bus, I2C0, 400 kHz like the flight firmware)
 * ------------------------------------------------------------------------- */
#define I2C_PORT    I2C_NUM_0
#define I2C_HZ      400000
#define I2C_TIMEOUT pdMS_TO_TICKS(20)

static esp_err_t i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, buf, len, I2C_TIMEOUT);
}

static esp_err_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, addr, buf, sizeof(buf), I2C_TIMEOUT);
}

// True if a chip acknowledges its address
static bool i2cProbe(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

/* ---------------------------------------------------------------------------
 * NVS (the ESP32's own flash). Used to remember that a full-throttle test was
 * running, so a reset or power loss in the middle of it is reported at the
 * next boot instead of looking like a normal restart.
 * ------------------------------------------------------------------------- */
#define NVS_NAMESPACE "boardtest"
#define NVS_KEY_THR   "thr_pct"

static void nvsInit(void)
{
    static bool done;
    if (done) {
        return;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    done = true;
}

// pct >= 0: the throttle test is running at this step. pct < 0: it finished.
static void throttleMark(int pct)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (pct >= 0) {
        nvs_set_i32(h, NVS_KEY_THR, pct);
    } else {
        nvs_erase_key(h, NVS_KEY_THR);
    }
    nvs_commit(h);
    nvs_close(h);
}

static bool throttleWasInterrupted(int *pct)
{
    nvs_handle_t h;
    int32_t value;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    bool found = nvs_get_i32(h, NVS_KEY_THR, &value) == ESP_OK;
    nvs_close(h);
    *pct = value;
    return found;
}

/* ---------------------------------------------------------------------------
 * ESP32-S3 and USB
 * ------------------------------------------------------------------------- */
static const char *resetReasonText(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:  return "power-on (power switch, battery/USB plugged in, or the RESET button)";
    case ESP_RST_USB:      return "reset over USB (flashing, or the serial monitor connecting)";
    case ESP_RST_BROWNOUT: return "BROWNOUT: the 3.3 V supply dipped too low";
    case ESP_RST_PANIC:    return "software crash";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog (software hung)";
    default:               return "other";
    }
}

static void testChip(void)
{
    section("ESP32-S3 (U1)");

    esp_reset_reason_t reason = esp_reset_reason();
    printf("  Last reset: %s\n", resetReasonText(reason));
    if (reason == ESP_RST_BROWNOUT) {
        report("Supply at last reset", FAIL, "brownout: 3V3 dipped low enough to reset the ESP32");
    }
    int pct;
    if (throttleWasInterrupted(&pct)) {
        report("Last throttle test", FAIL, "never finished: the board reset or lost power at the %d%% step "
               "(see 'Last reset' above)", pct);
        throttleMark(-1);
    }

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("  %d cores, silicon v%d.%d, MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
           chip.cores, chip.revision / 100, chip.revision % 100,
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    uint32_t flashBytes = 0;
    esp_flash_get_physical_size(NULL, &flashBytes);
    report("Flash size", flashBytes == 8 * 1024 * 1024 ? PASS : WARN,
           "%" PRIu32 " MB (an ESP32-S3FN8 has 8 MB)", flashBytes >> 20);

    report("USB-C data", PASS, "you are reading this over USB, so D+/D-, D1 and the 40 MHz crystal work");
}

/* ---------------------------------------------------------------------------
 * I2C bus: idle levels, then an address scan
 * ------------------------------------------------------------------------- */
static const struct {
    uint8_t addr;
    const char *name;
} i2cExpected[] = {
    { ADDR_BMM150,   "BMM150 magnetometer (U6)" },
    { ADDR_PCA9534A, "PCA9534A LED expander (U3)" },
    { ADDR_EEPROM,   "24AA64 EEPROM (U4)" },
    { ADDR_BMP280,   "BMP280 barometer (U7)" },
};
#define I2C_EXPECTED_COUNT (sizeof(i2cExpected) / sizeof(i2cExpected[0]))

static void testI2cBus(void)
{
    section("I2C bus (SDA GPIO33, SCL GPIO34)");

    // Before the I2C driver owns the pins, read them as plain inputs with the
    // ESP32's internal pull-ups off. Only R7/R8 can pull them high now, so a
    // low line means a short to GND, a missing pull-up, or a chip holding SDA.
    gpio_config_t in = {
        .pin_bit_mask = BIT64(PIN_I2C_SDA) | BIT64(PIN_I2C_SCL),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&in);
    esp_rom_delay_us(100);
    int sda = gpio_get_level(PIN_I2C_SDA);
    int scl = gpio_get_level(PIN_I2C_SCL);
    if (sda && scl) {
        report("Bus idle levels", PASS, "SDA and SCL idle high (pull-ups R7/R8 OK)");
    } else {
        report("Bus idle levels", FAIL, "SDA=%d SCL=%d, expected 1 and 1: short to GND, "
               "missing pull-up, or a chip holding the line", sda, scl);

        // Again with the ESP32's weak (~45k) internal pull-ups. A line that
        // now reads high is only missing its pull-up resistor; one that stays
        // low is being actively held to GND.
        gpio_pullup_en(PIN_I2C_SDA);
        gpio_pullup_en(PIN_I2C_SCL);
        esp_rom_delay_us(500);
        const struct { const char *name; int pin, level; } lines[] = {
            { "SDA (GPIO33)", PIN_I2C_SDA, sda },
            { "SCL (GPIO34)", PIN_I2C_SCL, scl },
        };
        for (int i = 0; i < 2; i++) {
            if (lines[i].level) {
                continue;
            }
            printf("    %s: %s\n", lines[i].name, gpio_get_level(lines[i].pin)
                   ? "goes high with the internal pull-up -> its 4.7k pull-up (R7/R8) is missing or open"
                   : "still low with the internal pull-up -> shorted to GND, or a chip is holding it low");
        }
        gpio_pullup_dis(PIN_I2C_SDA);
        gpio_pullup_dis(PIN_I2C_SCL);
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,   // the board has its own
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    if (i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        report("I2C driver", FAIL, "could not start the I2C driver");
        return;
    }

    bool found[128] = { false };
    int foundCount = 0;
    printf("  Scanning 0x08-0x77:");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2cProbe(addr)) {
            found[addr] = true;
            foundCount++;
            printf(" 0x%02X", addr);
        }
    }
    printf("%s\n", foundCount ? "" : " nothing answered");

    int missing = 0;
    for (int i = 0; i < I2C_EXPECTED_COUNT; i++) {
        bool ok = found[i2cExpected[i].addr];
        printf("    0x%02X %-28s %s\n", i2cExpected[i].addr, i2cExpected[i].name, ok ? "found" : "MISSING");
        missing += !ok;
        found[i2cExpected[i].addr] = false;
    }
    for (int addr = 0; addr < 128; addr++) {
        if (found[addr]) {
            printf("    0x%02X unexpected device (wrong address strap?)\n", addr);
        }
    }

    if (missing) {
        report("Bus scan", FAIL, "%d of %d expected chips did not answer", missing, (int)I2C_EXPECTED_COUNT);
    } else {
        report("Bus scan", PASS, "all %d expected chips answered", (int)I2C_EXPECTED_COUNT);
    }
}

/* ---------------------------------------------------------------------------
 * BMP280 barometer
 * ------------------------------------------------------------------------- */
static struct {
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmpCal;
static bool bmpReady;

// Integer compensation formulas from the BMP280 datasheet, section 3.11.3
static bool bmp280Read(float *tempC, float *pressHpa)
{
    uint8_t d[6];
    if (i2cRead(ADDR_BMP280, 0xF7, d, sizeof(d)) != ESP_OK) {
        return false;
    }
    int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
    if (adcP == 0x80000 || adcT == 0x80000) {
        return false;   // reset value: no measurement made yet
    }

    int32_t var1 = ((((adcT >> 3) - ((int32_t)bmpCal.T1 << 1))) * bmpCal.T2) >> 11;
    int32_t var2 = (((((adcT >> 4) - (int32_t)bmpCal.T1) * ((adcT >> 4) - (int32_t)bmpCal.T1)) >> 12)
                    * bmpCal.T3) >> 14;
    int32_t tFine = var1 + var2;
    *tempC = ((tFine * 5 + 128) >> 8) / 100.0f;

    int64_t v1 = (int64_t)tFine - 128000;
    int64_t v2 = v1 * v1 * bmpCal.P6;
    v2 += (v1 * bmpCal.P5) << 17;
    v2 += (int64_t)bmpCal.P4 << 35;
    v1 = ((v1 * v1 * bmpCal.P3) >> 8) + ((v1 * bmpCal.P2) << 12);
    v1 = ((((int64_t)1 << 47) + v1) * bmpCal.P1) >> 33;
    if (v1 == 0) {
        return false;
    }
    int64_t p = 1048576 - adcP;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)bmpCal.P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)bmpCal.P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + ((int64_t)bmpCal.P7 << 4);
    *pressHpa = (uint32_t)p / 256.0f / 100.0f;   // Q24.8 Pa -> hPa
    return true;
}

static void testBmp280(void)
{
    section("BMP280 barometer (U7, I2C 0x76)");

    uint8_t id = 0;
    if (i2cRead(ADDR_BMP280, 0xD0, &id, 1) != ESP_OK) {
        report("BMP280", FAIL, "no answer at 0x76: check U7 soldering, SDO to GND, CSB to 3V3");
        return;
    }
    if (id == 0x58) {
        report("BMP280 chip ID", PASS, "0x58");
    } else if (id == 0x60) {
        report("BMP280 chip ID", WARN, "0x60 is a BME280 (humidity version); the flight driver expects 0x58");
    } else {
        report("BMP280 chip ID", FAIL, "0x%02X, expected 0x58", id);
        return;
    }

    i2cWrite(ADDR_BMP280, 0xE0, 0xB6);  // soft reset
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t c[24];
    if (i2cRead(ADDR_BMP280, 0x88, c, sizeof(c)) != ESP_OK) {
        report("BMP280 calibration", FAIL, "could not read the factory calibration");
        return;
    }
#define LE16(i) ((uint16_t)(c[i] | (c[(i) + 1] << 8)))
    bmpCal.T1 = LE16(0);
    bmpCal.T2 = (int16_t)LE16(2);
    bmpCal.T3 = (int16_t)LE16(4);
    bmpCal.P1 = LE16(6);
    bmpCal.P2 = (int16_t)LE16(8);
    bmpCal.P3 = (int16_t)LE16(10);
    bmpCal.P4 = (int16_t)LE16(12);
    bmpCal.P5 = (int16_t)LE16(14);
    bmpCal.P6 = (int16_t)LE16(16);
    bmpCal.P7 = (int16_t)LE16(18);
    bmpCal.P8 = (int16_t)LE16(20);
    bmpCal.P9 = (int16_t)LE16(22);
#undef LE16

    // Continuous measurement: temperature x2, pressure x16 oversampling,
    // IIR filter coefficient 4, 0.5 ms standby
    i2cWrite(ADDR_BMP280, 0xF5, (0 << 5) | (2 << 2));
    i2cWrite(ADDR_BMP280, 0xF4, (2 << 5) | (5 << 2) | 3);
    vTaskDelay(pdMS_TO_TICKS(150));

    float t, p;
    if (!bmp280Read(&t, &p)) {
        report("BMP280 reading", FAIL, "no measurement came out");
        return;
    }
    bool plausible = t > 0.0f && t < 60.0f && p > 700.0f && p < 1100.0f;
    report("BMP280 reading", plausible ? PASS : FAIL, "%.2f C, %.2f hPa%s", t, p,
           plausible ? "" : " (implausible: expect room temperature and ~1013 hPa minus altitude)");
    bmpReady = true;
}

/* ---------------------------------------------------------------------------
 * BMM150 magnetometer
 * ------------------------------------------------------------------------- */
static bool bmmReady;

// Raw, uncompensated counts (about 0.3 uT each). Enough to see the board is
// alive and responds to turning it; the flight driver applies the trim data.
static bool bmm150Read(int *x, int *y, int *z)
{
    uint8_t d[6];
    if (i2cRead(ADDR_BMM150, 0x42, d, sizeof(d)) != ESP_OK) {
        return false;
    }
    *x = (int16_t)((d[1] << 8) | d[0]) >> 3;     // 13 bit
    *y = (int16_t)((d[3] << 8) | d[2]) >> 3;     // 13 bit
    *z = (int16_t)((d[5] << 8) | d[4]) >> 1;     // 15 bit
    return true;
}

static void testBmm150(void)
{
    section("BMM150 magnetometer (U6, I2C 0x10)");

    // After power-up the BMM150 sits in suspend mode, where only register 0x4B
    // works and the chip ID reads 0. Set its power bit first.
    if (i2cWrite(ADDR_BMM150, 0x4B, 0x01) != ESP_OK) {
        report("BMM150", FAIL, "no answer at 0x10: check U6 (0.4 mm WLCSP) soldering, PS/CSB/SDO straps");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t id = 0;
    i2cRead(ADDR_BMM150, 0x40, &id, 1);
    if (id != 0x32) {
        report("BMM150 chip ID", FAIL, "0x%02X, expected 0x32", id);
        return;
    }
    report("BMM150 chip ID", PASS, "0x32");

    // Normal mode, 10 Hz, "regular" preset (9 XY / 15 Z repetitions)
    i2cWrite(ADDR_BMM150, 0x51, (9 - 1) / 2);
    i2cWrite(ADDR_BMM150, 0x52, 15 - 1);
    i2cWrite(ADDR_BMM150, 0x4C, 0x00);
    vTaskDelay(pdMS_TO_TICKS(250));

    int x, y, z;
    if (!bmm150Read(&x, &y, &z)) {
        report("BMM150 reading", FAIL, "data read failed");
        return;
    }
    bool overflow = x == -4096 || y == -4096 || z == -16384;
    bool allZero = x == 0 && y == 0 && z == 0;
    report("BMM150 reading", (overflow || allZero) ? FAIL : PASS, "raw X %d  Y %d  Z %d%s", x, y, z,
           overflow ? " (overflow: a magnet nearby, or a bad sensor)" : allZero ? " (all zero: not measuring)" : "");
    bmmReady = true;
}

/* ---------------------------------------------------------------------------
 * PCA9534A LED expander and the LEDs behind it
 * ------------------------------------------------------------------------- */
#define EXP_REG_INPUT  0
#define EXP_REG_OUTPUT 1
#define EXP_REG_CONFIG 3        // 1 = input, 0 = output

// LEDs off. P0's output level is 0: it only matters while P0 is an output,
// which is how VCOM_EXP gets switched off (see vcomExpToggle()).
#define EXP_LEDS_OFF   ((uint8_t)(0xFF & ~BIT(EXP_BLUE) & ~BIT(EXP_VCOM_EN)))
#define EXP_LED_CONFIG ((uint8_t)~(BIT(EXP_GREEN_L) | BIT(EXP_RED_L) | BIT(EXP_BLUE)))

static uint8_t expOutputs = EXP_LEDS_OFF;
static bool expReady;
static bool vcomExpOff;     // P0 driven low = breakout VCOM switched off

static bool expWrite(uint8_t reg, uint8_t val)
{
    return i2cWrite(ADDR_PCA9534A, reg, val) == ESP_OK;
}

// A config register value with P0 made an output while VCOM_EXP is off
static uint8_t expConfig(uint8_t base)
{
    return vcomExpOff ? (uint8_t)(base & ~BIT(EXP_VCOM_EN)) : base;
}

static bool ledWalk(void)
{
    printf("  Watch the board: D2 green, D2 red, D3 blue (1.5 s each), then D9 (power LED) red for 2 s.\n");

    // Levels first, then directions, so nothing flashes on
    bool ok = expWrite(EXP_REG_OUTPUT, EXP_LEDS_OFF);
    ok &= expWrite(EXP_REG_CONFIG, expConfig(EXP_LED_CONFIG));

    const struct {
        uint8_t outputs;
        const char *what;
    } steps[] = {
        { EXP_LEDS_OFF & ~BIT(EXP_GREEN_L), "D2 green" },
        { EXP_LEDS_OFF & ~BIT(EXP_RED_L),   "D2 red" },
        { EXP_LEDS_OFF | BIT(EXP_BLUE),     "D3 blue" },
    };
    for (int i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        printf("    %s\n", steps[i].what);
        ok &= expWrite(EXP_REG_OUTPUT, steps[i].outputs);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    // D9: driving P6 low flips S4 to the red LED. Back as an input, the R40
    // pull-up selects green again.
    printf("    D9 red\n");
    ok &= expWrite(EXP_REG_OUTPUT, EXP_LEDS_OFF & ~BIT(EXP_PWR_SEL));
    ok &= expWrite(EXP_REG_CONFIG, expConfig(EXP_LED_CONFIG & ~BIT(EXP_PWR_SEL)));
    vTaskDelay(pdMS_TO_TICKS(2000));
    ok &= expWrite(EXP_REG_CONFIG, expConfig(EXP_LED_CONFIG));

    expOutputs = EXP_LEDS_OFF;
    ok &= expWrite(EXP_REG_OUTPUT, expOutputs);
    return ok;
}

// Key 'v': switch the breakout header's VCOM (U2, via expander P0) off and on
static void vcomExpToggle(void)
{
    if (!expReady) {
        printf("\nThe LED expander isn't responding, so VCOM_EXP can't be switched.\n\n");
        return;
    }
    vcomExpOff = !vcomExpOff;
    bool ok = expWrite(EXP_REG_OUTPUT, expOutputs);
    ok &= expWrite(EXP_REG_CONFIG, expConfig(EXP_LED_CONFIG));
    printf("\nVCOM_EXP is now %s%s. Measure breakout J3 pin 9 to GND: expect %s.\n"
           "Press 'v' again to switch it %s.\n\n",
           vcomExpOff ? "OFF" : "ON", ok ? "" : " (I2C write FAILED)",
           vcomExpOff ? "about 0 V" : "VCOM (about 5 V on USB, battery voltage otherwise)",
           vcomExpOff ? "back on" : "off");
}

static void testPca9534(void)
{
    section("PCA9534A LED expander (U3, I2C 0x38)");

    // All pins back to inputs first. The expander has no reset pin, so after
    // the RESET button it still holds whatever the last program set.
    if (!expWrite(EXP_REG_CONFIG, 0xFF)) {
        report("PCA9534A", FAIL, "no answer at 0x38: check U3 soldering and A0-A2 to GND");
        return;
    }
    uint8_t in = 0;
    i2cRead(ADDR_PCA9534A, EXP_REG_INPUT, &in, 1);
    int p0 = (in >> 0) & 1, p6 = (in >> EXP_PWR_SEL) & 1;
    // P0 (VCOM_EXP_EN) and P6 (LED_GREEN_RED_R) have pull-ups R12 and R40
    report("PCA9534A inputs", (p0 && p6) ? PASS : WARN, "P0 VCOM_EXP_EN=%d, P6 LED_GREEN_RED_R=%d (pull-ups, expect 1 and 1)", p0, p6);

    expReady = ledWalk();
    report("LED walk", expReady ? PASS : FAIL, expReady ? "I2C writes OK, now confirm by eye" : "I2C writes failed");
}

/* ---------------------------------------------------------------------------
 * 24AA64 EEPROM (2-byte addresses, 8 KB). The flight firmware does not use
 * it yet: its eeprom.c is ESP-Drone's stub that never talks to the chip.
 * ------------------------------------------------------------------------- */
static esp_err_t eepromRead(uint16_t mem, uint8_t *val)
{
    uint8_t addr[2] = { mem >> 8, mem & 0xFF };
    return i2c_master_write_read_device(I2C_PORT, ADDR_EEPROM, addr, 2, val, 1, I2C_TIMEOUT);
}

static esp_err_t eepromWrite(uint16_t mem, uint8_t val)
{
    uint8_t buf[3] = { mem >> 8, mem & 0xFF, val };
    esp_err_t err = i2c_master_write_to_device(I2C_PORT, ADDR_EEPROM, buf, sizeof(buf), I2C_TIMEOUT);
    if (err != ESP_OK) {
        return err;
    }
    // The chip ignores its address until the internal write (max 5 ms) is done
    for (int i = 0; i < 20; i++) {
        if (i2cProbe(ADDR_EEPROM)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

static void testEeprom(void)
{
    section("24AA64 EEPROM (U4, I2C 0x50)");

    // Flip the last byte, read it back, then put the original back
    const uint16_t addr = 0x1FFF;
    uint8_t original, readBack;
    if (eepromRead(addr, &original) != ESP_OK) {
        report("EEPROM", FAIL, "no answer at 0x50: check U4 soldering and A0-A2 to GND");
        return;
    }
    uint8_t pattern = ~original;
    if (eepromWrite(addr, pattern) != ESP_OK || eepromRead(addr, &readBack) != ESP_OK) {
        report("EEPROM write", FAIL, "write did not complete");
        return;
    }
    eepromWrite(addr, original);
    if (readBack != pattern) {
        report("EEPROM write", FAIL, "wrote 0x%02X, read 0x%02X: is WP (pin 7) really at GND?", pattern, readBack);
        return;
    }
    report("EEPROM write", PASS, "write and read-back at 0x%04X OK (original value restored)", addr);
}

/* ---------------------------------------------------------------------------
 * BMI270 IMU on SPI
 * ------------------------------------------------------------------------- */
#define IMU_SPI_HOST     SPI2_HOST
#define IMU_SPI_CLOCK_HZ (8 * 1000 * 1000)  // same as the flight firmware
#define ACC_LSB_PER_G    (32768.0f / 4.0f)      // +-4 g range
#define GYR_LSB_PER_DPS  (32768.0f / 500.0f)    // +-500 dps range

static spi_device_handle_t imuSpi;
static struct bmi2_dev imu;
static bool imuReady;
static bool gpioIsrReady;   // GPIO interrupt service installed (shared with the BOOT button)
static volatile uint32_t int1Count, int2Count;

// SPI glue for the Bosch API, same as bmi270_port.c in the flight firmware.
// For reads Bosch has already set bit 7 of reg_addr and asks for one extra
// dummy byte in front, which it strips itself.
static BMI2_INTF_RETURN_TYPE imuRead(uint8_t reg, uint8_t *data, uint32_t len, void *intf)
{
    spi_transaction_t t = {
        .addr = reg,
        .length = len * 8,
        .rxlength = len * 8,
        .rx_buffer = data,
    };
    return spi_device_polling_transmit(imuSpi, &t) == ESP_OK ? BMI2_INTF_RET_SUCCESS : -1;
}

static BMI2_INTF_RETURN_TYPE imuWrite(uint8_t reg, const uint8_t *data, uint32_t len, void *intf)
{
    spi_transaction_t t = {
        .addr = reg,
        .length = len * 8,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(imuSpi, &t) == ESP_OK ? BMI2_INTF_RET_SUCCESS : -1;
}

static void imuDelayUs(uint32_t us, void *intf)
{
    if (us < 1000) {
        esp_rom_delay_us(us);
    } else {
        vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
    }
}

static const char *bmiError(int8_t rslt)
{
    switch (rslt) {
    case BMI2_E_COM_FAIL:       return "SPI transfer failed";
    case BMI2_E_DEV_NOT_FOUND:  return "wrong chip ID";
    case BMI2_E_CONFIG_LOAD:    return "config file upload failed: SPI unreliable?";
    case BMI2_E_SELF_TEST_FAIL: return "sensor outside Bosch limits";
    default:                    return "see bmi2_defs.h";
    }
}

static void IRAM_ATTR countEdge(void *counter)
{
    (*(volatile uint32_t *)counter)++;
}

static bool bmi270Read(float acc[3], float gyr[3])
{
    struct bmi2_sens_data d;
    if (bmi2_get_sensor_data(&d, &imu) != BMI2_OK) {
        return false;
    }
    acc[0] = d.acc.x / ACC_LSB_PER_G;
    acc[1] = d.acc.y / ACC_LSB_PER_G;
    acc[2] = d.acc.z / ACC_LSB_PER_G;
    gyr[0] = d.gyr.x / GYR_LSB_PER_DPS;
    gyr[1] = d.gyr.y / GYR_LSB_PER_DPS;
    gyr[2] = d.gyr.z / GYR_LSB_PER_DPS;
    return true;
}

static void testBmi270(void)
{
    section("BMI270 IMU (U5, SPI: MISO 37, MOSI 38, SCK 39, CS 40)");
    printf("  Keep the board still and flat for the next few seconds.\n");

    spi_bus_config_t bus = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = PIN_SPI_MOSI,
        .sclk_io_num = PIN_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t dev = {
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = IMU_SPI_CLOCK_HZ,
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 1,
    };
    if (spi_bus_initialize(IMU_SPI_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK ||
        spi_bus_add_device(IMU_SPI_HOST, &dev, &imuSpi) != ESP_OK) {
        report("SPI driver", FAIL, "could not start the SPI driver");
        return;
    }

    // The BMI270 powers up in I2C mode and switches to SPI on the first rising
    // edge of CS, so the first read is thrown away. Then soft-reset it: the
    // RESET button only resets the ESP32, the IMU keeps its old setup.
    uint8_t buf[2];
    uint8_t softReset = 0xB6;
    imuRead(0x80, buf, 2, NULL);
    imuWrite(0x7E, &softReset, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(5));
    imuRead(0x80, buf, 2, NULL);    // back in I2C mode after the reset

    // Interrupt lines, before the IMU drives them: only R27/R29 hold them high
    gpio_config_t in = {
        .pin_bit_mask = BIT64(PIN_IMU_INT1) | BIT64(PIN_IMU_INT2),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&in);
    int int1 = gpio_get_level(PIN_IMU_INT1);
    int int2 = gpio_get_level(PIN_IMU_INT2);
    report("INT1/INT2 idle", (int1 && int2) ? PASS : FAIL, "INT1=%d INT2=%d (expect 1 and 1 from R27/R29)", int1, int2);

    // Raw chip-ID read, so a wiring fault shows up as a value you can reason about
    imuRead(0x80, buf, 2, NULL);
    uint8_t id = buf[1];
    if (id != 0x24) {
        const char *hint = id == 0xFF ? "MISO stays high: IMU not answering (CS, SCK, MOSI, power?)"
                         : id == 0x00 ? "MISO stuck low"
                         : "garbled SPI or wrong chip";
        report("BMI270 chip ID", FAIL, "0x%02X, expected 0x24: %s", id, hint);
        return;
    }
    report("BMI270 chip ID", PASS, "0x24");

    imu.intf = BMI2_SPI_INTF;
    imu.read = imuRead;
    imu.write = imuWrite;
    imu.delay_us = imuDelayUs;
    imu.read_write_len = 32;
    imu.config_file_ptr = NULL;     // the config blob built into bmi270.c

    // Soft reset, chip ID check, then the 8 KB config upload, read back and
    // verified by the chip. A good test of the whole SPI link.
    int8_t rslt = bmi270_init(&imu);
    if (rslt != BMI2_OK) {
        report("BMI270 init", FAIL, "bmi270_init() = %d (%s)", rslt, bmiError(rslt));
        return;
    }
    report("BMI270 init", PASS, "8 KB config uploaded and accepted");

    // Bosch's built-in self-test: the chip deflects its own proof mass
    // electrostatically and checks the response on each axis. It ends with a
    // soft reset, so the chip is set up again afterwards.
    rslt = bmi2_perform_accel_self_test(&imu);
    report("Accel self-test", rslt == BMI2_OK ? PASS : FAIL, rslt == BMI2_OK ? "all axes within limits" : "%d (%s)",
           rslt, bmiError(rslt));
    if (bmi270_init(&imu) != BMI2_OK) {
        report("BMI270 re-init", FAIL, "init after the self-test failed");
        return;
    }

    // Accel +-4 g and gyro +-500 dps at 100 Hz: finer steps than flight
    // settings, easier to read by eye
    struct bmi2_sens_config cfg[2] = { { .type = BMI2_ACCEL }, { .type = BMI2_GYRO } };
    bmi2_get_sensor_config(cfg, 2, &imu);
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_set_sensor_config(cfg, 2, &imu);
    if (rslt == BMI2_OK) {
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &imu);
    }
    if (rslt == BMI2_OK) {
        rslt = bmi2_sensor_enable(sensors, 2, &imu);
    }
    if (rslt != BMI2_OK) {
        report("BMI270 config", FAIL, "%d (%s)", rslt, bmiError(rslt));
        return;
    }

    // Data-ready on both INT pins, active-low open-drain (the board has
    // pull-ups), so each pin should pulse 100 times a second
    struct bmi2_int_pin_config pins;
    bmi2_get_int_pin_config(&pins, &imu);
    pins.pin_type = BMI2_INT_BOTH;
    pins.int_latch = BMI2_INT_NON_LATCH;
    for (int i = 0; i < 2; i++) {
        pins.pin_cfg[i].lvl = BMI2_INT_ACTIVE_LOW;
        pins.pin_cfg[i].od = BMI2_INT_OPEN_DRAIN;
        pins.pin_cfg[i].output_en = BMI2_INT_OUTPUT_ENABLE;
        pins.pin_cfg[i].input_en = BMI2_INT_INPUT_DISABLE;
    }
    bmi2_set_int_pin_config(&pins, &imu);
    bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT_BOTH, &imu);

    gpio_install_isr_service(0);
    gpioIsrReady = true;
    gpio_isr_handler_add(PIN_IMU_INT1, countEdge, (void *)&int1Count);
    gpio_isr_handler_add(PIN_IMU_INT2, countEdge, (void *)&int2Count);
    vTaskDelay(pdMS_TO_TICKS(200));     // let the filters settle

    // For one second: count edges and average the readings. Reading every
    // 2 ms also clears data-ready each time, so the pins keep toggling.
    float accSum[3] = { 0 }, gyrSum[3] = { 0 };
    int samples = 0;
    int1Count = 0;
    int2Count = 0;
    int64_t end = esp_timer_get_time() + 1000000;
    while (esp_timer_get_time() < end) {
        float acc[3], gyr[3];
        if (bmi270Read(acc, gyr)) {
            for (int i = 0; i < 3; i++) {
                accSum[i] += acc[i];
                gyrSum[i] += gyr[i];
            }
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    uint32_t c1 = int1Count, c2 = int2Count;
    report("INT1 (GPIO36)", (c1 > 80 && c1 < 120) ? PASS : FAIL, "%" PRIu32 " pulses in 1 s, expected ~100", c1);
    report("INT2 (GPIO35)", (c2 > 80 && c2 < 120) ? PASS : FAIL, "%" PRIu32 " pulses in 1 s, expected ~100", c2);

    if (samples == 0) {
        report("IMU data", FAIL, "no samples could be read");
        return;
    }
    float a[3], g[3];
    for (int i = 0; i < 3; i++) {
        a[i] = accSum[i] / samples;
        g[i] = gyrSum[i] / samples;
    }
    float aNorm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    report("Accelerometer", (aNorm > 0.9f && aNorm < 1.1f) ? PASS : FAIL,
           "X %+.2f  Y %+.2f  Z %+.2f g, total %.2f g (expect 1.00 at rest)", a[0], a[1], a[2], aNorm);
    float gMax = fmaxf(fabsf(g[0]), fmaxf(fabsf(g[1]), fabsf(g[2])));
    report("Gyroscope", gMax < 5.0f ? PASS : WARN, "X %+.1f  Y %+.1f  Z %+.1f dps (expect near 0 at rest)%s",
           g[0], g[1], g[2], gMax < 5.0f ? "" : ", was the board moving?");
    imuReady = true;
}

/* ---------------------------------------------------------------------------
 * Battery voltage
 * ------------------------------------------------------------------------- */
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t adcCali;
static bool adcReady;

static float readVbat(void)
{
    if (!adcReady) {
        return 0.0f;
    }
    int sumMv = 0;
    for (int i = 0; i < 16; i++) {
        int raw = 0, mv = 0;
        adc_oneshot_read(adc, VBAT_ADC_CHANNEL, &raw);
        adc_cali_raw_to_voltage(adcCali, raw, &mv);
        sumMv += mv;
    }
    return sumMv / 16.0f / 1000.0f * VBAT_DIVIDER;
}

static void testBattery(void)
{
    section("Battery voltage (GPIO4, divider R15/R18)");

    adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_chan_cfg_t chan = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .chan = VBAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_new_unit(&unit, &adc) != ESP_OK ||
        adc_oneshot_config_channel(adc, VBAT_ADC_CHANNEL, &chan) != ESP_OK ||
        adc_cali_create_scheme_curve_fitting(&cali, &adcCali) != ESP_OK) {
        report("Battery ADC", FAIL, "could not set up the ADC");
        return;
    }
    adcReady = true;

    float v = readVbat();
    if (v > 3.0f && v < 4.35f) {
        report("Battery voltage", PASS, "%.2f V", v);
    } else if (v < 0.5f) {
        report("Battery voltage", WARN, "%.2f V: no battery. Fine on USB, but the motors need one", v);
    } else {
        report("Battery voltage", WARN, "%.2f V. With no battery the charger probes the BAT pin, so odd values "
               "are normal; with a battery fitted, check R15/R18", v);
    }
}

/* ---------------------------------------------------------------------------
 * Buttons
 * ------------------------------------------------------------------------- */
static volatile uint32_t bootPresses;

// Counts BOOT presses, ignoring contact bounce within 50 ms
static void IRAM_ATTR bootPressed(void *arg)
{
    static int64_t last;
    int64_t now = esp_timer_get_time();
    if (now - last > 50000) {
        bootPresses++;
    }
    last = now;
}

static void testButtons(void)
{
    section("Buttons");

    gpio_config_t in = {
        .pin_bit_mask = BIT64(PIN_BOOT_BTN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&in);
    int level = gpio_get_level(PIN_BOOT_BTN);
    report("BOOT button (GPIO0)", level ? PASS : FAIL, level
           ? "high while released; press it during the live readout and each press is reported"
           : "low while released: button stuck or shorted");
    if (!gpioIsrReady) {            // normally already installed by the IMU test
        gpio_install_isr_service(0);
        gpioIsrReady = true;
    }
    gpio_isr_handler_add(PIN_BOOT_BTN, bootPressed, NULL);
    printf("  RESET button: press it any time. The board restarts, and 'Last reset' at the top\n"
           "  of the new report should say power-on.\n");
}

/* ---------------------------------------------------------------------------
 * Wi-Fi: a scan exercises the whole RF path (matching network + ANT1) and
 * puts current spikes on the 3.3 V rail.
 * ------------------------------------------------------------------------- */
static int byRssi(const void *a, const void *b)
{
    return ((const wifi_ap_record_t *)b)->rssi - ((const wifi_ap_record_t *)a)->rssi;
}

// One-time Wi-Fi driver setup, shared by the boot test and the antenna test
static bool wifiInit(void)
{
    static bool ready;
    if (ready) {
        return true;
    }
    nvsInit();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ready = esp_wifi_init(&cfg) == ESP_OK;
    return ready;
}

static void testWifi(void)
{
    section("Wi-Fi radio and chip antenna (ANT1)");

    if (!wifiInit() || esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK || esp_wifi_start() != ESP_OK) {
        report("Wi-Fi start", FAIL, "the radio did not start");
        return;
    }

    printf("  Scanning...\n");
    if (esp_wifi_scan_start(NULL, true) != ESP_OK) {
        report("Wi-Fi scan", FAIL, "scan failed");
        esp_wifi_stop();
        return;
    }
    uint16_t total = 0;
    esp_wifi_scan_get_ap_num(&total);
    wifi_ap_record_t aps[16];
    uint16_t count = sizeof(aps) / sizeof(aps[0]);
    esp_wifi_scan_get_ap_records(&count, aps);
    esp_wifi_stop();

    qsort(aps, count, sizeof(aps[0]), byRssi);
    for (int i = 0; i < count && i < 5; i++) {
        printf("    %4d dBm  ch %2d  %s\n", aps[i].rssi, aps[i].primary, (const char *)aps[i].ssid);
    }
    if (count == 0) {
        report("Wi-Fi scan", FAIL, "no networks found: check ANT1 and its matching network (L1, L3, C1)");
    } else {
        report("Wi-Fi scan", PASS, "%d networks, strongest %d dBm. Compare with a phone in the same spot",
               total, aps[0].rssi);
    }
}

/* ---------------------------------------------------------------------------
 * Antenna test (key 'w'). Receive: several scans averaged per network, since
 * a single scan's RSSI jumps by several dB. Transmit: afterwards the board
 * stays on as a test access point whose signal a phone can measure.
 * ------------------------------------------------------------------------- */
#define RF_SCANS        8
#define RF_MAX_NETWORKS 32
#define RF_AP_SSID      "CosFly-RF-test"
#define RF_AP_PASSWORD  "cosfly-rf-test"    // only so nothing joins it
#define RF_AP_CHANNEL   6

static struct {
    uint8_t bssid[6];
    char ssid[33];
    int channel, seen, sum, min, max;
} rfNets[RF_MAX_NETWORKS];
static int rfNetCount;

static int byAverageRssi(const void *a, const void *b)
{
    const typeof(rfNets[0]) *x = a, *y = b;
    return y->sum / y->seen - x->sum / x->seen;
}

static void rfRecord(const wifi_ap_record_t *ap)
{
    int i;
    for (i = 0; i < rfNetCount; i++) {
        if (memcmp(rfNets[i].bssid, ap->bssid, 6) == 0) {
            break;
        }
    }
    if (i == rfNetCount) {
        if (rfNetCount == RF_MAX_NETWORKS) {
            return;
        }
        memcpy(rfNets[i].bssid, ap->bssid, 6);
        snprintf(rfNets[i].ssid, sizeof(rfNets[i].ssid), "%s", (const char *)ap->ssid);
        rfNets[i].channel = ap->primary;
        rfNets[i].min = ap->rssi;
        rfNets[i].max = ap->rssi;
        rfNetCount++;
    }
    rfNets[i].seen++;
    rfNets[i].sum += ap->rssi;
    rfNets[i].min = ap->rssi < rfNets[i].min ? ap->rssi : rfNets[i].min;
    rfNets[i].max = ap->rssi > rfNets[i].max ? ap->rssi : rfNets[i].max;
}

static void rfTest(void)
{
    printf("\n=== Antenna test. Put the board down, flat, and don't touch it.\n");
    if (!wifiInit()) {
        printf("Wi-Fi did not start.\n\n");
        return;
    }
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // Receive: average every network's signal over several scans
    rfNetCount = 0;
    static wifi_ap_record_t aps[RF_MAX_NETWORKS];
    for (int scan = 1; scan <= RF_SCANS; scan++) {
        printf("  scan %d/%d\n", scan, RF_SCANS);
        if (esp_wifi_scan_start(NULL, true) != ESP_OK) {
            continue;
        }
        uint16_t count = RF_MAX_NETWORKS;
        esp_wifi_scan_get_ap_records(&count, aps);
        for (int i = 0; i < count; i++) {
            rfRecord(&aps[i]);
        }
    }
    qsort(rfNets, rfNetCount, sizeof(rfNets[0]), byAverageRssi);
    printf("\n  RECEIVE: average / weakest / strongest over %d scans\n", RF_SCANS);
    printf("   avg  min  max  seen  ch  network\n");
    for (int i = 0; i < rfNetCount && i < 12; i++) {
        printf("  %4d %4d %4d  %2d/%d  %2d  %s\n", rfNets[i].sum / rfNets[i].seen, rfNets[i].min,
               rfNets[i].max, rfNets[i].seen, RF_SCANS, rfNets[i].channel, rfNets[i].ssid);
    }

    // Transmit: stay on as an access point at full power for a phone to measure
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t ap = {
        .ap = {
            .ssid = RF_AP_SSID,
            .ssid_len = sizeof(RF_AP_SSID) - 1,
            .password = RF_AP_PASSWORD,
            .channel = RF_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,
        },
    };
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_start();
    esp_wifi_set_max_tx_power(80);      // 0.25 dBm steps: 80 = 20 dBm, the chip's maximum
    int8_t power = 0;
    esp_wifi_get_max_tx_power(&power);
    printf("\n  TRANSMIT: now broadcasting \"%s\" on channel %d at %.1f dBm.\n"
           "  Read its signal with a Wi-Fi analyser app on a phone. Press 'w' to scan again.\n\n",
           RF_AP_SSID, RF_AP_CHANNEL, power / 4.0f);
}

/* ---------------------------------------------------------------------------
 * Motors (only on request, from the live readout)
 * ------------------------------------------------------------------------- */
static void motorsOff(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(motors[i].gpio);
        gpio_set_direction(motors[i].gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(motors[i].gpio, 0);
    }
}

// Same PWM as the flight firmware: 15 kHz, 8 bit, all motors at 0
static void motorsPwmInit(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 15000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = motors[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
        };
        ledc_channel_config(&ch);
    }
}

static void motorSet(int i, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, i, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
}

static void motorsPwmStop(void)
{
    for (int i = 0; i < 4; i++) {
        ledc_stop(LEDC_LOW_SPEED_MODE, i, 0);
    }
    motorsOff();
}

// Drives each motor pin high for 100 us and reads the pad back. Each pin sees
// only its 47R gate resistor, the MOSFET gate and a 10k pull-down, so a
// healthy pin reads back 1. Reading 0 while driven high means the pin is
// damaged or something (a shorted gate) is dragging it down. 100 us is far
// too short for a motor to move.
static void testMotorPins(void)
{
    section("Motor drive pins");
    for (int i = 0; i < 4; i++) {
        int pin = motors[i].gpio;
        gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(100);
        int high = gpio_get_level(pin);
        gpio_set_level(pin, 0);
        esp_rom_delay_us(100);
        int low = gpio_get_level(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);

        static char label[4][24];   // report() keeps the pointer
        snprintf(label[i], sizeof(label[i]), "Motor pin GPIO%d", pin);
        if (high == 1 && low == 0) {
            report(label[i], PASS, "%s: drives high and low", motors[i].name);
        } else {
            report(label[i], FAIL, "%s: driven high reads %d, driven low reads %d. Pin damaged, or "
                   "%s / the MOSFET gate shorted", motors[i].name, high, low, motors[i].gateResistor);
        }
    }
}

// Key 'g': hold one motor's gate on so the drive path can be measured with a
// meter. Motors must be unplugged, since that motor would otherwise run flat out.
static void gateHold(void)
{
    printf("\n*** GATE HOLD: one motor pin is switched fully ON for 30 s. ***\n"
           "UNPLUG ALL MOTORS FIRST. Which motor (1-4)? Any other key cancels.\n");
    int c = readKey(30000);
    if (c < '1' || c > '4') {
        printf("Cancelled.\n\n");
        return;
    }
    int i = c - '1';
    gpio_set_level(motors[i].gpio, 1);
    printf("  %s: GPIO%d is held HIGH for 30 s (any key ends it early). Measure to GND:\n"
           "    - the ESP32 end of %s ........ about 3.3 V (if 0 V: GPIO%d isn't driving)\n"
           "    - the MOSFET's gate, pin 1 .... about 3.3 V (if 0 V but the line above is 3.3 V: %s is open)\n"
           "  Both at 3.3 V means the drive is fine and the fault is the MOSFET or its other joints.\n",
           motors[i].name, motors[i].gpio, motors[i].gateResistor, motors[i].gpio, motors[i].gateResistor);
    readKey(30000);
    gpio_set_level(motors[i].gpio, 0);
    printf("  Released: GPIO%d is low again.\n\n", motors[i].gpio);
}

static void motorTest(void)
{
    float v = readVbat();
    if (v < 3.4f) {
        printf("\nMotor test needs the battery: VBAT reads %.2f V. The motors run straight off\n"
               "VBAT, which USB alone cannot supply.\n\n", v);
        return;
    }
    printf("\n*** MOTOR TEST. PROPELLERS OFF! ***\n"
           "Each motor runs at %d%% for 3 s, in order M1..M4, then all four together at\n"
           "%d%% for %d s. Any key stops it.\n"
           "Press 'y' to start, any other key to cancel.\n",
           MOTOR_TEST_DUTY * 100 / 255, MOTOR_ALL_DUTY * 100 / 255, MOTOR_ALL_SECONDS);
    if (readKey(15000) != 'y') {
        printf("Cancelled.\n\n");
        return;
    }
    motorsPwmInit();

    // A running motor pulls the battery voltage down a little (its internal
    // resistance), so the dip shows electrically that the MOSFET switched and
    // current flowed. The first 0.5 s (spin-up surge) is not averaged.
    bool aborted = false;
    float dip[4] = { 0 };
    for (int i = 0; i < 4 && !aborted; i++) {
        float idle = readVbat();
        printf("  %s spinning... ", motors[i].name);
        motorSet(i, MOTOR_TEST_DUTY);
        float sum = 0;
        int n = 0;
        int64_t start = esp_timer_get_time();
        while (!aborted && esp_timer_get_time() - start < 3000000) {
            aborted = readKey(20) >= 0;
            if (esp_timer_get_time() - start > 500000) {
                sum += readVbat();
                n++;
            }
        }
        motorSet(i, 0);
        float loaded = n ? sum / n : idle;
        dip[i] = idle - loaded;
        printf("VBAT %.3f -> %.3f V (dip %.0f mV)\n", idle, loaded, dip[i] * 1000);
        if (!aborted) {
            aborted = readKey(700) >= 0;
        }
    }

    // All four together for a while: the channels have to hold up when they
    // run at once, and the battery voltage is logged throughout
    if (!aborted) {
        float idle = readVbat(), sum = 0, lowest = idle;
        int n = 0;
        printf("  All four at %d%% for %d s... ", MOTOR_ALL_DUTY * 100 / 255, MOTOR_ALL_SECONDS);
        for (int i = 0; i < 4; i++) {
            motorSet(i, MOTOR_ALL_DUTY);
        }
        int64_t start = esp_timer_get_time();
        while (!aborted && esp_timer_get_time() - start < MOTOR_ALL_SECONDS * 1000000LL) {
            aborted = readKey(20) >= 0;
            if (esp_timer_get_time() - start > 500000) {
                float v = readVbat();
                sum += v;
                lowest = fminf(lowest, v);
                n++;
            }
        }
        for (int i = 0; i < 4; i++) {
            motorSet(i, 0);
        }
        if (n) {
            printf("VBAT %.3f V idle -> %.3f V average, %.3f V lowest\n", idle, sum / n, lowest);
        }
    }
    motorsPwmStop();

    if (!aborted) {
        for (int i = 0; i < 4; i++) {
            if (dip[i] < 0.010f) {
                printf("  ! %s: no measurable dip. Motor unplugged, or MOSFET not switching?\n", motors[i].name);
            }
        }
    }
    printf(aborted ? "Stopped.\n\n"
           : "Done. Did the named corner spin each time? Diagonal pairs M1/M3 and M2/M4\n"
             "must spin in opposite directions to each other.\n\n");
}

/* ---------------------------------------------------------------------------
 * Full-throttle test (key 't'). Props ON, drone strapped down. All four
 * motors step 0 -> 25 -> 50 -> 75 -> 100 %. At each step a background task
 * samples the battery voltage and the IMU (vibration) while the main task
 * runs a Wi-Fi scan (is the radio still as sensitive with the motors' noise
 * around?). A reset mid-test is caught at the next boot via throttleMark().
 * ------------------------------------------------------------------------- */
#define THR_STEPS    5
#define THR_REFS     5      // strongest networks at 0 %, used as the Wi-Fi reference
#define THR_MIN_VBAT 3.6f

static const int thrPercent[THR_STEPS] = { 0, 25, 50, 75, 100 };

static struct {
    SemaphoreHandle_t lock;
    volatile bool running;
    volatile bool aborted;
    volatile bool batteryOnly;  // plugging USB back in also stops the motors
    int vbatCount;
    float vbatSum, vbatMin;
    int imuCount, clipped;
    double acc[3], acc2[3], gyr[3], gyr2[3];
} thr;

static uint8_t thrRef[THR_REFS][6];
static int thrRefCount;

static void throttleSetAll(uint32_t duty)
{
    if (thr.aborted) {
        duty = 0;
    }
    for (int i = 0; i < 4; i++) {
        motorSet(i, duty);
    }
}

static void throttleResetStats(void)
{
    xSemaphoreTake(thr.lock, portMAX_DELAY);
    thr.vbatCount = 0;
    thr.vbatSum = 0;
    thr.vbatMin = 99.0f;
    thr.imuCount = 0;
    thr.clipped = 0;
    for (int i = 0; i < 3; i++) {
        thr.acc[i] = thr.acc2[i] = thr.gyr[i] = thr.gyr2[i] = 0;
    }
    xSemaphoreGive(thr.lock);
}

static bool throttleStopRequested(void);

// Background task: every 5 ms, stop the motors if a key arrived (or USB came
// back during a battery-only run), then sample the battery voltage and the IMU
static void throttleSampler(void *arg)
{
    while (thr.running) {
        if (throttleStopRequested()) {
            thr.aborted = true;
            throttleSetAll(0);
        }
        float v = readVbat();
        float a[3], g[3];
        bool imuOk = imuReady && bmi270Read(a, g);

        xSemaphoreTake(thr.lock, portMAX_DELAY);
        thr.vbatSum += v;
        thr.vbatCount++;
        if (v < thr.vbatMin) {
            thr.vbatMin = v;
        }
        if (imuOk) {
            bool clip = false;
            for (int i = 0; i < 3; i++) {
                thr.acc[i] += a[i];
                thr.acc2[i] += a[i] * a[i];
                thr.gyr[i] += g[i];
                thr.gyr2[i] += g[i] * g[i];
                clip |= fabsf(a[i]) > 3.95f || fabsf(g[i]) > 495.0f;   // at the +-4 g / +-500 dps limits
            }
            thr.clipped += clip;
            thr.imuCount++;
        }
        xSemaphoreGive(thr.lock);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}

// Overall noise of a 3-axis signal: standard deviations of the axes combined
static float noise3(const double sum[3], const double sum2[3], int n)
{
    double var = 0;
    for (int i = 0; i < 3; i++) {
        double mean = sum[i] / n;
        var += sum2[i] / n - mean * mean;
    }
    return var > 0 ? (float)sqrt(var) : 0.0f;
}

// One blocking Wi-Fi scan (about 2 s). With makeRefs, remember the strongest
// networks as references. Returns the number of networks seen (-1 on error),
// and how many references were seen with their average signal.
static int throttleScan(bool makeRefs, int *refsSeen, float *refRssi)
{
    static wifi_ap_record_t aps[RF_MAX_NETWORKS];
    uint16_t count = RF_MAX_NETWORKS;
    *refsSeen = 0;
    *refRssi = 0;
    if (esp_wifi_scan_start(NULL, true) != ESP_OK || esp_wifi_scan_get_ap_records(&count, aps) != ESP_OK) {
        return -1;
    }
    if (makeRefs) {
        qsort(aps, count, sizeof(aps[0]), byRssi);
        thrRefCount = count < THR_REFS ? count : THR_REFS;
        for (int i = 0; i < thrRefCount; i++) {
            memcpy(thrRef[i], aps[i].bssid, 6);
        }
    }
    int sum = 0;
    for (int r = 0; r < thrRefCount; r++) {
        for (int i = 0; i < count; i++) {
            if (memcmp(thrRef[r], aps[i].bssid, 6) == 0) {
                sum += aps[i].rssi;
                (*refsSeen)++;
                break;
            }
        }
    }
    if (*refsSeen) {
        *refRssi = (float)sum / *refsSeen;
    }
    return count;
}

/* ---------------------------------------------------------------------------
 * Battery-only runs. With USB plugged in, the charger powers the electronics
 * from USB, so a sagging battery can never brown out the ESP32. The battery-
 * only tests therefore wait for USB to be unplugged, run, save their results
 * to flash, and print them once USB is back ('r' reprints them any time).
 * Plugging USB back in during a run stops the motors.
 * ------------------------------------------------------------------------- */
static bool usbConnected(void)
{
    return usb_serial_jtag_is_connected();
}

static bool waitUsbUnplug(int seconds)
{
    printf("\n  >>> UNPLUG THE USB CABLE NOW (within %d s). The motors start 5 s after it is out.\n"
           "  >>> To stop early: plug USB back in, or pull the battery.\n", seconds);
    int64_t end = esp_timer_get_time() + seconds * 1000000LL;
    while (esp_timer_get_time() < end) {
        if (!usbConnected()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (!usbConnected()) {
                return true;
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("  USB is still connected: cancelled.\n\n");
    return false;
}

static void waitUsbReplug(void)
{
    while (!usbConnected()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(2500));    // give the PC time to reopen the port
}

static void nvsSaveBlob(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, key, data, len);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool nvsLoadBlob(const char *key, void *data, size_t len)
{
    nvs_handle_t h;
    size_t got = len;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    bool ok = nvs_get_blob(h, key, data, &got) == ESP_OK && got == len;
    nvs_close(h);
    return ok;
}

static bool throttleStopRequested(void)
{
    return readKey(0) >= 0 || (thr.batteryOnly && usbConnected());
}

/* ---------------------------------------------------------------------------
 * Full-throttle test: 't' on USB power, 'b' on battery only. Props ON, drone
 * strapped down. All four motors step 0 -> 25 -> 50 -> 75 -> 100 %. At each
 * step a background task samples the battery voltage and the IMU (vibration)
 * while the main task runs a Wi-Fi scan (is the radio still as sensitive with
 * the motors' noise around?). A reset mid-test is caught at the next boot via
 * throttleMark().
 * ------------------------------------------------------------------------- */
#define NVS_KEY_THR_RES "thr_res"

typedef struct {
    uint8_t valid, batteryOnly, completed, steps, refCount;
    uint16_t clipped;
    float vAvg[THR_STEPS], vMin[THR_STEPS], acc[THR_STEPS], gyr[THR_STEPS], rssi[THR_STEPS];
    int8_t refsSeen[THR_STEPS];
    int16_t nets[THR_STEPS];
} thr_result_t;

static void printThrottleResults(const thr_result_t *r)
{
    printf("\n  FULL-THROTTLE TEST (%s)\n", r->batteryOnly ? "battery only" : "on USB power");
    printf("  thr  |  VBAT avg   min  |  vibration: accel    gyro   |  Wi-Fi: refs seen, avg signal\n");
    for (int s = 0; s < r->steps; s++) {
        printf("  %3d%% |  %.2f V  %.2f V |  %6.0f mg  %6.1f dps  |  ", thrPercent[s], r->vAvg[s], r->vMin[s],
               r->acc[s] * 1000, r->gyr[s]);
        if (r->nets[s] < 0) {
            printf("scan failed\n");
        } else {
            printf("%d/%d, %.0f dBm (%d networks)\n", r->refsSeen[s], r->refCount, r->rssi[s], r->nets[s]);
        }
    }
    if (!r->completed) {
        printf("\n  STOPPED before the end (key press, or USB plugged back in).\n\n");
        return;
    }

    int last = THR_STEPS - 1;
    printf("\n  RESULTS\n"
           "  - The board did not reset at full throttle, so the 3.3 V supply held up%s.\n"
           "  - Battery sag: %.2f V with motors off -> %.2f V lowest at 100%% (%.0f mV).\n",
           r->batteryOnly ? " on the battery alone" : " (but on USB power, which hides battery sag:\n"
           "    run 'b' for the real test)",
           r->vAvg[0], r->vMin[last], (r->vAvg[0] - r->vMin[last]) * 1000);
    if (r->vMin[last] < 3.4f) {
        printf("    ! Under about 3.4 V the 3.3 V buck runs out of headroom, so 3V3 droops at full\n"
               "      throttle as the battery empties. A partly used battery may brown out.\n");
    } else {
        printf("    OK: stays above about 3.4 V, where the 3.3 V buck still has headroom.\n");
    }

    if (r->refCount > 0 && r->refsSeen[0] > 0 && r->nets[last] >= 0) {
        if (r->refsSeen[last] == 0) {
            printf("  - Wi-Fi: ! all reference networks were lost at 100%%: the motors are swamping the radio.\n");
        } else {
            float lost = r->rssi[0] - r->rssi[last];
            printf("  - Wi-Fi: reference networks %d/%d with motors off, %d/%d at 100%%, signal %+.0f dB.\n",
                   r->refsSeen[0], r->refCount, r->refsSeen[last], r->refCount, -lost);
            if (r->refsSeen[last] < r->refsSeen[0] || lost > 6.0f) {
                printf("    ! The motors seem to be desensitising the radio. Keep motor wires and VBAT\n"
                       "      away from the antenna, and check for noise on 3V3.\n");
            } else {
                printf("    OK: no clear loss of sensitivity with the motors running.\n");
            }
        }
    }

    float worstAcc = 0, worstGyr = 0;
    for (int s = 1; s < r->steps; s++) {
        worstAcc = fmaxf(worstAcc, r->acc[s]);
        worstGyr = fmaxf(worstGyr, r->gyr[s]);
    }
    // Rough guide for a drone this size: the flight firmware filters vibration,
    // but the less there is, the better it flies
    printf("  - Vibration (worst step): accel %.0f mg, gyro %.1f dps%s\n", worstAcc * 1000, worstGyr,
           r->clipped ? ". ! Readings hit the sensor's +-4 g / +-500 dps limits" : "");
    printf("    Rough guide for the accelerometer: under 300 mg good, 300-1000 mg usable,\n"
           "    over 1000 mg check props (balance, damage), motor mounts and how the board is held.\n"
           "  - Feel the MOSFETs (Q1-Q4), the flyback diodes and the battery wires: warm is\n"
           "    fine, too hot to keep a finger on is not.\n\n");
}

static void throttleTest(bool batteryOnly)
{
    float v = readVbat();
    if (v < THR_MIN_VBAT) {
        printf("\nThe full-throttle test needs a charged battery: VBAT reads %.2f V, need %.1f V or more.\n\n",
               v, THR_MIN_VBAT);
        return;
    }
    printf("\n*** FULL-THROTTLE TEST%s: ALL FOUR MOTORS UP TO 100%% ***\n"
           "  - Props ON, drone strapped or clamped down firmly. Hands, cables and loose\n"
           "    things well clear of the props. Eye protection is a good idea.\n"
           "  - Steps: 0%% (motors off), 25, 50, 75, 100%%. About 3 s each, 100%% for 5 s.\n"
           "  - %s\n"
           "Press capital 'Y' to continue, anything else cancels.\n",
           batteryOnly ? " ON BATTERY ONLY" : "",
           batteryOnly ? "Results are saved and printed when USB is plugged back in."
                       : "ANY KEY STOPS THE MOTORS IMMEDIATELY.");
    if (readKey(30000) != 'Y') {
        printf("Cancelled.\n\n");
        return;
    }
    if (batteryOnly) {
        if (!waitUsbUnplug(60)) {
            return;
        }
    } else {
        for (int s = 3; s > 0; s--) {
            printf("  starting in %d...\n", s);
            if (readKey(1000) >= 0) {
                printf("Cancelled.\n\n");
                return;
            }
        }
    }

    // Wi-Fi as a plain scanner (the antenna test may have left it as an access point)
    bool wifiOk = wifiInit();
    if (wifiOk) {
        esp_wifi_stop();
        esp_wifi_set_mode(WIFI_MODE_STA);
        wifiOk = esp_wifi_start() == ESP_OK;
    }

    if (!thr.lock) {
        thr.lock = xSemaphoreCreateMutex();
    }
    thr.batteryOnly = batteryOnly;
    thr.aborted = false;
    thr.running = true;
    throttleResetStats();
    motorsPwmInit();
    xTaskCreate(throttleSampler, "throttle", 4096, NULL, 10, NULL);

    static thr_result_t r;
    memset(&r, 0, sizeof(r));
    r.valid = 1;
    r.batteryOnly = batteryOnly;
    uint32_t duty = 0;
    for (int s = 0; s < THR_STEPS && !thr.aborted; s++) {
        int pct = thrPercent[s];
        throttleMark(pct);

        // Ramp over 300 ms rather than jumping, sparing the battery a current spike
        uint32_t target = pct * 255 / 100;
        for (int k = 1; k <= 10 && !thr.aborted; k++) {
            throttleSetAll(duty + ((int)target - (int)duty) * k / 10);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        duty = target;
        vTaskDelay(pdMS_TO_TICKS(300));     // let the motors settle

        throttleResetStats();
        int64_t start = esp_timer_get_time();
        int seen = 0;
        float avg = 0;
        int nets = wifiOk ? throttleScan(s == 0, &seen, &avg) : -1;
        int64_t holdUs = (pct == 100 ? 5000 : 3000) * 1000LL;
        while (!thr.aborted && esp_timer_get_time() - start < holdUs) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (thr.aborted) {
            break;
        }

        xSemaphoreTake(thr.lock, portMAX_DELAY);
        r.vAvg[s] = thr.vbatCount ? thr.vbatSum / thr.vbatCount : 0;
        r.vMin[s] = thr.vbatMin;
        r.acc[s] = thr.imuCount ? noise3(thr.acc, thr.acc2, thr.imuCount) : 0;
        r.gyr[s] = thr.imuCount ? noise3(thr.gyr, thr.gyr2, thr.imuCount) : 0;
        r.clipped += thr.clipped;
        xSemaphoreGive(thr.lock);
        r.nets[s] = nets;
        r.refsSeen[s] = seen;
        r.rssi[s] = avg;
        r.refCount = thrRefCount;
        r.steps = s + 1;
        printf("  %3d%% done: VBAT %.2f V avg, %.2f V lowest\n", pct, r.vAvg[s], r.vMin[s]);
    }

    // Wind down over 300 ms, then hand the pins back to plain GPIO, held low
    for (int k = 9; k >= 0; k--) {
        throttleSetAll(duty * k / 10);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    thr.running = false;
    vTaskDelay(pdMS_TO_TICKS(50));          // let the sampler task exit
    motorsPwmStop();
    if (wifiOk) {
        esp_wifi_stop();
    }
    r.completed = !thr.aborted;
    nvsSaveBlob(NVS_KEY_THR_RES, &r, sizeof(r));
    throttleMark(-1);

    if (batteryOnly) {
        waitUsbReplug();
    }
    printThrottleResults(&r);
}

/* ---------------------------------------------------------------------------
 * Endurance test (key 'e'): battery only, props ON, strapped down. All four
 * motors run at a chosen throttle until the battery, measured under load,
 * reaches END_CUTOFF_V. Gives the run time at that throttle (a flight-time
 * estimate if it matches the hover throttle) and the battery curve. Saved to
 * flash every 10 s, so even a brownout leaves the data behind.
 * ------------------------------------------------------------------------- */
#define NVS_KEY_END_RES  "end_res"
#define END_MAX_POINTS   96             // one per 10 s
#define END_MAX_SECONDS  (15 * 60)
// Under load. This board's battery path sags about 0.5 V at 50 % throttle,
// so 3.1 V under load is roughly 3.6 V resting: a safe point to stop a LiPo,
// and about where micro-quad firmware lands on low battery.
#define END_CUTOFF_V     3.10f
#define END_MAX_TEMP_C   60.0f          // board temperature (BMP280)
#define END_MIN_START_V  3.90f

enum { END_RUNNING, END_CUTOFF, END_HOT, END_TIMEOUT, END_STOPPED };
static const char *const endStopText[] = {
    "the board reset or lost power during the run (brownout?)",
    "battery reached the cut-off voltage",
    "board got too hot",
    "15-minute limit reached",
    "stopped by hand (USB plugged back in)",
};

typedef struct {
    uint8_t valid, pct, stopReason, count;
    uint16_t seconds, startMv, restMv;
    struct {
        uint16_t t, avgMv, minMv;
        int8_t tempC;
    } pts[END_MAX_POINTS];
} end_result_t;

static void printEnduranceResults(const end_result_t *r)
{
    printf("\n  ENDURANCE TEST at %d%% throttle (battery only)\n", r->pct);
    printf("  Stopped after %d min %02d s: %s.\n", r->seconds / 60, r->seconds % 60,
           endStopText[r->stopReason < 5 ? r->stopReason : 0]);
    printf("  Battery: %.2f V before, %.2f V resting afterwards.\n", r->startMv / 1000.0f, r->restMv / 1000.0f);
    printf("   time  | VBAT avg  lowest | board temp\n");
    for (int i = 0; i < r->count; i++) {
        printf("  %2d:%02d  |  %.2f V   %.2f V |  %d C\n", r->pts[i].t / 60, r->pts[i].t % 60,
               r->pts[i].avgMv / 1000.0f, r->pts[i].minMv / 1000.0f, r->pts[i].tempC);
    }
    if (r->stopReason == END_CUTOFF) {
        printf("\n  FLIGHT TIME ESTIMATE: about %d min %02d s, IF %d%% is your hover throttle.\n"
               "  Hover throttle depends on weight and props; in flight it is usually 40-60%% for a\n"
               "  well-matched micro quad. The flight firmware's log will tell you the real figure.\n"
               "  Real flights use extra throttle to manoeuvre, so expect a little less.\n\n",
               r->seconds / 60, r->seconds % 60, r->pct);
    } else if (r->stopReason == END_RUNNING) {
        printf("\n  ! The run never finished. If the battery wasn't pulled, the 3.3 V supply browned\n"
               "    out: the last row shows the battery voltage just before.\n\n");
    } else {
        printf("\n");
    }
}

static void enduranceTest(void)
{
    float v = readVbat();
    if (v < END_MIN_START_V) {
        printf("\nThe endurance test needs a fully charged battery: VBAT reads %.2f V, need %.1f V\n"
               "or more. Leave it charging on USB until D10 goes out.\n\n", v, END_MIN_START_V);
        return;
    }
    printf("\n*** ENDURANCE TEST: ALL FOUR MOTORS UNTIL THE BATTERY IS FLAT ***\n"
           "  - Props ON, drone strapped down, props clear of everything. Stay with it.\n"
           "  - Runs on battery only until the battery reaches %.2f V under load (max 15 min).\n"
           "Throttle? Press 3, 4, 5 or 6 for 30-60%%. Anything else cancels.\n", END_CUTOFF_V);
    int c = readKey(30000);
    if (c < '3' || c > '6') {
        printf("Cancelled.\n\n");
        return;
    }
    int pct = (c - '0') * 10;
    printf("  %d%% throttle. Press capital 'Y' to continue, anything else cancels.\n", pct);
    if (readKey(30000) != 'Y' || !waitUsbUnplug(60)) {
        printf("Cancelled.\n\n");
        return;
    }

    static end_result_t r;
    memset(&r, 0, sizeof(r));
    r.valid = 1;
    r.pct = pct;
    r.stopReason = END_RUNNING;
    r.startMv = readVbat() * 1000;
    nvsSaveBlob(NVS_KEY_END_RES, &r, sizeof(r));

    if (!thr.lock) {
        thr.lock = xSemaphoreCreateMutex();
    }
    thr.batteryOnly = true;
    thr.aborted = false;
    thr.running = true;
    throttleResetStats();
    motorsPwmInit();
    xTaskCreate(throttleSampler, "endurance", 4096, NULL, 10, NULL);

    uint32_t duty = pct * 255 / 100;
    for (int k = 1; k <= 20 && !thr.aborted; k++) {      // 2 s ramp up
        throttleSetAll(duty * k / 20);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    int64_t start = esp_timer_get_time();
    float sum10 = 0, min10 = 99;
    int n10 = 0, lowSeconds = 0;
    while (!thr.aborted) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int seconds = (esp_timer_get_time() - start) / 1000000;

        // One-second average of the battery under load
        xSemaphoreTake(thr.lock, portMAX_DELAY);
        float avg1 = thr.vbatCount ? thr.vbatSum / thr.vbatCount : 0;
        float min1 = thr.vbatMin;
        xSemaphoreGive(thr.lock);
        throttleResetStats();
        sum10 += avg1;
        min10 = fminf(min10, min1);
        n10++;

        float tempC = 0, press;
        bool haveTemp = bmpReady && bmp280Read(&tempC, &press);
        lowSeconds = avg1 < END_CUTOFF_V ? lowSeconds + 1 : 0;

        int reason = END_RUNNING;
        if (lowSeconds >= 3) {
            reason = END_CUTOFF;            // 3 s in a row, so a spike doesn't end it
        } else if (haveTemp && tempC > END_MAX_TEMP_C) {
            reason = END_HOT;
        } else if (seconds >= END_MAX_SECONDS) {
            reason = END_TIMEOUT;
        }

        if (n10 == 10 || reason != END_RUNNING) {
            if (r.count < END_MAX_POINTS) {
                r.pts[r.count].t = seconds;
                r.pts[r.count].avgMv = sum10 / n10 * 1000;
                r.pts[r.count].minMv = min10 * 1000;
                r.pts[r.count].tempC = haveTemp ? (int8_t)tempC : 0;
                r.count++;
            }
            r.seconds = seconds;
            sum10 = 0;
            min10 = 99;
            n10 = 0;
            nvsSaveBlob(NVS_KEY_END_RES, &r, sizeof(r));
        }
        if (reason != END_RUNNING) {
            r.stopReason = reason;
            break;
        }
    }
    if (thr.aborted) {
        r.stopReason = END_STOPPED;
        r.seconds = (esp_timer_get_time() - start) / 1000000;
    }

    throttleSetAll(0);
    thr.running = false;
    vTaskDelay(pdMS_TO_TICKS(50));
    motorsPwmStop();

    vTaskDelay(pdMS_TO_TICKS(15000));       // let the battery recover before the resting reading
    r.restMv = readVbat() * 1000;
    nvsSaveBlob(NVS_KEY_END_RES, &r, sizeof(r));

    waitUsbReplug();
    printEnduranceResults(&r);
}

// Key 'r': print the last saved results
static void printSavedResults(void)
{
    static thr_result_t t;
    static end_result_t e;
    bool any = false;
    if (nvsLoadBlob(NVS_KEY_THR_RES, &t, sizeof(t)) && t.valid) {
        printThrottleResults(&t);
        any = true;
    }
    if (nvsLoadBlob(NVS_KEY_END_RES, &e, sizeof(e)) && e.valid) {
        printEnduranceResults(&e);
        any = true;
    }
    if (!any) {
        printf("\nNo saved results yet.\n\n");
    }
}

/* ---------------------------------------------------------------------------
 * Breakout pin finder (key 'p'). Every breakout GPIO becomes an input held
 * low by its internal pull-down (E_SDA/E_SCL idle high through their 2.2k
 * pull-ups R2/R1). Touch a header pin with a wire from 3V3 (or GND for the
 * two I2C pins) and the board names the signal that changed, so a mirrored
 * header, swapped nets or a dry ESP32 joint all show up. E_GPIO2 (J3 pin 8)
 * is on the expander on V1 and on GPIO35 on V2, so it isn't checked here.
 * ------------------------------------------------------------------------- */
static const struct {
    int gpio;
    const char *name;
    const char *where;
    bool pulledUp;
} breakoutPins[] = {
    { 9,  "E_RX1",   "J2 pin 2", false },
    { 11, "E_TX1",   "J2 pin 3", false },
    { 10, "E_SDA",   "J2 pin 4", true  },
    { 13, "E_SCL",   "J2 pin 5", true  },
    { 15, "E_CS3",   "J2 pin 6", false },
    { 17, "E_CS2",   "J2 pin 7", false },
    { 6,  "E_CS1",   "J2 pin 8", false },
    { 5,  "E_CS0",   "J2 pin 9", false },
    { 41, "E_TX2",   "J3 pin 1", false },
    { 1,  "E_RX2",   "J3 pin 2", false },
    { 18, "E_SCLK",  "J3 pin 3", false },
    { 16, "E_MISO",  "J3 pin 4", false },
    { 21, "E_MOSI",  "J3 pin 5", false },
    { 12, "E_OW",    "J3 pin 6", false },
    { 14, "E_GPIO1", "J3 pin 7", false },
};
#define BREAKOUT_PIN_COUNT (sizeof(breakoutPins) / sizeof(breakoutPins[0]))
#define PIN_FINDER_SECONDS 180

static void pinFinder(void)
{
    int idle[BREAKOUT_PIN_COUNT], last[BREAKOUT_PIN_COUNT];
    bool seen[BREAKOUT_PIN_COUNT] = { false };

    printf("\n*** BREAKOUT PIN FINDER (%d s, any key ends it) ***\n", PIN_FINDER_SECONDS);
    for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
        gpio_reset_pin(breakoutPins[i].gpio);
        gpio_set_direction(breakoutPins[i].gpio, GPIO_MODE_INPUT);
        gpio_set_pull_mode(breakoutPins[i].gpio, breakoutPins[i].pulledUp ? GPIO_FLOATING : GPIO_PULLDOWN_ONLY);
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // Idle check: everything low, except the two I2C lines high from R1/R2
    bool idleOk = true;
    for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
        idle[i] = gpio_get_level(breakoutPins[i].gpio);
        last[i] = idle[i];
        if (idle[i] != (int)breakoutPins[i].pulledUp) {
            idleOk = false;
            printf("  ! %s (GPIO%d, %s) idles %s: %s\n", breakoutPins[i].name, breakoutPins[i].gpio,
                   breakoutPins[i].where, idle[i] ? "HIGH" : "LOW",
                   breakoutPins[i].pulledUp ? "its 2.2k pull-up is missing" : "shorted to 3V3 or a neighbour?");
        }
    }
    if (idleOk) {
        printf("  Idle levels OK (I2C pull-ups present).\n");
    }
    printf("  Touch each header pin: 3V3 wire for all, GND wire for J2 pins 4 and 5 (E_SDA, E_SCL).\n\n");

    int64_t end = esp_timer_get_time() + PIN_FINDER_SECONDS * 1000000LL;
    while (esp_timer_get_time() < end) {
        if (readKey(10) >= 0) {
            break;
        }
        int changed = 0;
        for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
            int level = gpio_get_level(breakoutPins[i].gpio);
            if (level != last[i]) {
                esp_rom_delay_us(2000);         // ignore the wire bouncing on contact
                level = gpio_get_level(breakoutPins[i].gpio);
            }
            if (level != last[i]) {
                last[i] = level;
                if (level != idle[i]) {
                    changed++;
                    seen[i] = true;
                    printf("  %-8s (GPIO%-2d) <- touched   expected at %s\n", breakoutPins[i].name,
                           breakoutPins[i].gpio, breakoutPins[i].where);
                }
            }
        }
        if (changed > 1) {
            printf("  ! %d signals changed at once: are those pins bridged together?\n", changed);
        }
    }

    printf("\n  Pins never touched or never seen:");
    int missing = 0;
    for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
        if (!seen[i]) {
            printf(" %s (%s)", breakoutPins[i].name, breakoutPins[i].where);
            missing++;
        }
    }
    printf(missing ? "\n\n" : " none, all %d confirmed.\n\n", (int)BREAKOUT_PIN_COUNT);

    for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
        gpio_reset_pin(breakoutPins[i].gpio);
    }
}

// Key 'q': meter-only breakout check. Each pin of one header outputs a
// 10 kHz PWM with its own duty cycle, which a multimeter on DC volts reads as
// a steady voltage: 10 % (about 0.34 V) on the first pin, 20 % on the next,
// and so on. Probing along the header, the readings must climb in even steps;
// a mirrored header reads backwards, swapped nets read out of order. Any key
// moves from J2 to J3, then ends.
static void breakoutMeterCheck(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    const struct { const char *header; int first, count; } headers[] = {
        { "J2 (left)", 0, 8 },      // breakoutPins[0..7]: J2 pins 2-9
        { "J3 (right)", 8, 7 },     // breakoutPins[8..14]: J3 pins 1-7
    };
    for (int h = 0; h < 2; h++) {
        // Every breakout pin low first, then this header's pins get their PWM
        for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
            gpio_reset_pin(breakoutPins[i].gpio);
            gpio_set_direction(breakoutPins[i].gpio, GPIO_MODE_OUTPUT);
            gpio_set_level(breakoutPins[i].gpio, 0);
        }
        printf("\n*** METER CHECK, header %s: probe each pin to GND on DC volts ***\n", headers[h].header);
        for (int k = 0; k < headers[h].count; k++) {
            int i = headers[h].first + k;
            uint32_t duty = (k + 1) * 256 / 10;
            ledc_channel_config_t ch = {
                .gpio_num = breakoutPins[i].gpio,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = k,
                .timer_sel = LEDC_TIMER_1,
                .duty = duty,
            };
            ledc_channel_config(&ch);
            printf("  %-9s %-8s (GPIO%-2d)  expect about %.2f V\n", breakoutPins[i].where, breakoutPins[i].name,
                   breakoutPins[i].gpio, duty / 256.0f * 3.37f);
        }
        printf("  Any key: %s.\n", h == 0 ? "move on to J3" : "finish");
        readKey(600000);
        for (int k = 0; k < headers[h].count; k++) {
            ledc_stop(LEDC_LOW_SPEED_MODE, k, 0);
        }
    }
    for (int i = 0; i < BREAKOUT_PIN_COUNT; i++) {
        gpio_reset_pin(breakoutPins[i].gpio);
    }
    printf("  Meter check finished.\n\n");
}

/* ---------------------------------------------------------------------------
 * Summary and live readout
 * ------------------------------------------------------------------------- */
static int printSummary(void)
{
    int counts[3] = { 0 };
    printf("\n================ SUMMARY ================\n");
    for (int i = 0; i < resultCount; i++) {
        printf("  %s  %s\n", verdictName[results[i].verdict], results[i].test);
        counts[results[i].verdict]++;
    }
    printf("  -> %d pass, %d warn, %d fail\n", counts[PASS], counts[WARN], counts[FAIL]);
    return counts[FAIL];
}

static void printHelp(void)
{
    printf("\nKeys:\n"
           "  m  motor test, one motor at a time (battery fitted, PROPS OFF)\n"
           "  t  full-throttle test on USB power, all motors to 100%% (PROPS ON, STRAPPED DOWN)\n"
           "  b  full-throttle test on BATTERY ONLY: you unplug USB (PROPS ON, STRAPPED DOWN)\n"
           "  e  endurance: motors run until the battery is flat, battery only (PROPS ON, STRAPPED)\n"
           "  r  print the last saved battery-only results\n"
           "  v  switch the breakout VCOM (VCOM_EXP) off / on\n"
           "  g  hold one motor's MOSFET gate on, to measure it (MOTORS UNPLUGGED)\n"
           "  p  breakout pin finder: touch header pins and the board names each signal\n"
           "  q  breakout meter check: each header pin outputs its own voltage to probe\n"
           "  l  LED walk\n"
           "  w  antenna test\n"
           "  h  this help\n"
           "Buttons: press BOOT and each press is reported below. Press RESET to restart and\n"
           "  run every test again ('Last reset' at the top should then say power-on).\n"
           "Battery only: unplug USB. D3 keeps blinking while the board runs on the battery.\n"
           "  Plug USB back in and reopen the monitor: if the 'up' time didn't restart from\n"
           "  zero, the switch-over between battery and USB didn't reset the board.\n\n");
}

static void liveReadout(int failures)
{
    // Overall result on D2 (green = all passed, red = something failed),
    // D3 blue blinks to show the board is alive
    uint8_t resultLed = failures ? EXP_RED_L : EXP_GREEN_L;

    printf("\n=== Live readout. Tilt, turn and breathe on the board and watch the numbers.");
    printHelp();

    uint32_t presses = bootPresses;
    for (int line = 0;; line++) {
        if (line % 20 == 0) {
            printf("    up  |   accel X/Y/Z (g)   |    gyro X/Y/Z (dps)   |  mag raw X/Y/Z  |  pressure  temp  | VBAT  | BOOT\n");
        }
        if (bootPresses != presses) {
            presses = bootPresses;
            printf("  >> BOOT button pressed (%" PRIu32 " so far)\n", presses);
        }

        float acc[3] = { 0 }, gyr[3] = { 0 }, tempC = 0, pressHpa = 0;
        int mx = 0, my = 0, mz = 0;
        if (imuReady) {
            bmi270Read(acc, gyr);
        }
        if (bmpReady) {
            bmp280Read(&tempC, &pressHpa);
        }
        if (bmmReady) {
            bmm150Read(&mx, &my, &mz);
        }
        printf("%5llds | %+5.2f %+5.2f %+5.2f | %+6.1f %+6.1f %+6.1f | %5d %5d %5d | %7.2f %5.1fC | %.2fV | %s\n",
               esp_timer_get_time() / 1000000, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], mx, my, mz,
               pressHpa, tempC, readVbat(), gpio_get_level(PIN_BOOT_BTN) ? "up" : "PRESSED");

        if (expReady) {
            expOutputs = (EXP_LEDS_OFF & ~BIT(resultLed)) | ((line & 1) ? BIT(EXP_BLUE) : 0);
            expWrite(EXP_REG_OUTPUT, expOutputs);
        }

        switch (readKey(500)) {
        case 'm':
            motorTest();
            break;
        case 't':
            throttleTest(false);
            break;
        case 'b':
            throttleTest(true);
            break;
        case 'e':
            enduranceTest();
            break;
        case 'r':
            printSavedResults();
            break;
        case 'v':
            vcomExpToggle();
            break;
        case 'g':
            gateHold();
            break;
        case 'p':
            pinFinder();
            break;
        case 'q':
            breakoutMeterCheck();
            break;
        case 'l':
            if (expReady) {
                ledWalk();
            }
            break;
        case 'w':
            rfTest();
            break;
        case 'h':
            printHelp();
            break;
        default:
            break;
        }
    }
}

void app_main(void)
{
    // MOSFET gates low before anything else (R23-R26 already pull them down)
    motorsOff();

    // Install the USB console driver so keys typed in the monitor can be read
    usb_serial_jtag_driver_config_t usb = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usb);
    esp_vfs_usb_serial_jtag_use_driver();

    nvsInit();
    vTaskDelay(pdMS_TO_TICKS(1000));    // time for the monitor to attach
    printf("\n\n========== CosFly V1 board test ==========\n");

    testChip();
    testI2cBus();
    testBmp280();
    testBmm150();
    testPca9534();
    testEeprom();
    testBmi270();
    testBattery();
    testMotorPins();
    testButtons();
    testWifi();

    liveReadout(printSummary());
}
