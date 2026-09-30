/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * CosFly V1 sensor implementation, derived from the ESP-Drone
 * sensors_mpu6050_hm5883L_ms5611.c.
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Hardware (CosFly V1 schematic, sensors sheet):
 *   BMI270 IMU   - SPI2: MISO/MOSI/SCK/CS = CONFIG_SPI_PIN_*
 *                  INT1 = CONFIG_MPU_PIN_INT, INT2 = CONFIG_IMU_PIN_INT2,
 *                  both with 4.7k pull-ups -> active-low open-drain.
 *   BMP280 baro  - I2C0 (INT_SDA/INT_SCL), SDO = GND -> 0x76
 *   BMM150 mag   - I2C0, CSB = SDO = GND, PS = VDDIO (I2C) -> 0x10
 *
 * Timing:
 *   The stabilizer and every filter in the Crazyflie stack assume a 1 kHz
 *   sensor loop. The BMI270 cannot output exactly 1 kHz (its rates are
 *   25 Hz x 2^n: 800, 1600, 3200...), so the IMU runs at 1600 Hz and a 1 kHz
 *   esp_timer paces the loop, reading the latest sample each tick. INT1 is
 *   still configured as data-ready and counted in an ISR (log imu_drdy.count)
 *   so the wiring can be checked, e.g. with a scope or from the PC client.
 *
 *   The barometer and magnetometer are slow (~25 Hz) and live on the shared
 *   I2C bus, so they are read in their own lower-priority task. That way an
 *   I2C transfer (or an LED update on the same bus) never delays the IMU loop.
 */
#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "esp_timer.h"
#include "driver/gpio.h"

#include "sensors_bmi270_bmp280_bmm150.h"
#include "system.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "usec_time.h"
#include "static_mem.h"
#include "stm32_legacy.h"

#include "i2cdev.h"
#include "bmi270_port.h"
#include "bmi270.h"
#include "bmp280_port.h"
#include "bmm150_port.h"

#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"

/* ---------------------------------------------------------------------------
 * Sensor settings
 * ------------------------------------------------------------------------- */

// BMI270 ranges. At +-2000 dps / +-16 g the scale factors match the MPU6050
// settings LiteWing used, so the bias thresholds below carry over unchanged.
#define SENSORS_DEG_PER_LSB_CFG  (2000.0f / 32768.0f)   // 16.384 LSB/dps
#define SENSORS_G_PER_LSB_CFG    (16.0f / 32768.0f)     // 2048 LSB/g

#define SENSORS_LOOP_PERIOD_US   1000                   // 1 kHz, see "Timing" above

#define BARO_MAG_PERIOD_MS       20                     // 50 Hz task, baro and mag alternate -> 25 Hz each

#define GPIO_IMU_INT1            CONFIG_MPU_PIN_INT

/* ---------------------------------------------------------------------------
 * Board orientation
 *
 * The Crazyflie body frame is x = forward, y = left, z = up. Edit these three
 * lines if the BMI270 is rotated on the PCB relative to that. Bench check,
 * props off, logging acc.x/y/z from the PC client:
 *   board flat              -> acc.z ~ +1
 *   nose (front) pointed down -> acc.x ~ -1
 *   left side pointed down  -> acc.y ~ -1
 * Gyro uses the same mapping, so fix the accelerometer and the gyro follows.
 * ------------------------------------------------------------------------- */
#define IMU_TO_BODY_X(s)  ( (s)[0])
#define IMU_TO_BODY_Y(s)  ( (s)[1])
#define IMU_TO_BODY_Z(s)  ( (s)[2])

/* ---------------------------------------------------------------------------
 * Gyro bias / accelerometer scale calibration (unchanged from ESP-Drone)
 * ------------------------------------------------------------------------- */
#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000)
#define SENSORS_MAN_TEST_LEVEL_MAX 5.0f
#define SENSORS_ACC_SCALE_SAMPLES 200

#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
#define GYRO_VARIANCE_BASE 5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0 / 100)
#define ROLL_CALIB (CONFIG_ROLL_CALIB * 1.0 / 100)

// Software low-pass filters, run at the 1 kHz loop rate
#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */
static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;   // given by the 1 kHz timer
static xSemaphoreHandle dataReady;          // given to the stabilizer

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuTickTimestamp;
static esp_timer_handle_t sensorsTimer;

static struct bmi2_dev bmi270;
static bmp280_handle_t bmp280;
static bmm150_handle_t bmm150;

static bool isBmi270Present = false;
static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

// Counts BMI270 INT1 data-ready pulses. Should climb at ~1600/s.
static volatile uint32_t imuDrdyCount = 0;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void processAccGyroMeasurements(const uint8_t *buffer);
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC(baroMagTask, BARO_MAG_TASK_STACKSIZE);

/* ---------------------------------------------------------------------------
 * Public read functions (called by sensors.c)
 * ------------------------------------------------------------------------- */
bool sensorsBmi270Bmp280Bmm150ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsBmi270Bmp280Bmm150ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsBmi270Bmp280Bmm150ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsBmi270Bmp280Bmm150ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsBmi270Bmp280Bmm150Acquire(sensorData_t *sensors, const uint32_t tick)
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi270Bmp280Bmm150AreCalibrated(void)
{
    return gyroBiasFound;
}

void sensorsBmi270Bmp280Bmm150WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

/* ---------------------------------------------------------------------------
 * 1 kHz IMU loop
 * ------------------------------------------------------------------------- */
static void sensorsTimerCallback(void *arg)
{
    imuTickTimestamp = usecTimestamp();
    xSemaphoreGive(sensorsDataReady);
}

static void IRAM_ATTR imuInt1IsrHandler(void *arg)
{
    imuDrdyCount++;
}

static void sensorsTask(void *param)
{
    // Accel X/Y/Z then gyro X/Y/Z, 16-bit little-endian, registers 0x0C..0x17
    uint8_t buffer[12];

    systemWaitStart();

    while (1) {
        if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            sensorData.interruptTimestamp = imuTickTimestamp;

            if (bmi2_get_regs(BMI2_ACC_X_LSB_ADDR, buffer, sizeof(buffer), &bmi270) == BMI2_OK) {
                processAccGyroMeasurements(buffer);
            }

            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            // Unlock the stabilizer task
            xSemaphoreGive(dataReady);
        }
    }
}

static void processAccGyroMeasurements(const uint8_t *buffer)
{
    Axis3f accScaled;
    int16_t acc[3];
    int16_t gyr[3];

    for (int i = 0; i < 3; i++) {
        acc[i] = (int16_t)((buffer[2 * i + 1] << 8) | buffer[2 * i]);
        gyr[i] = (int16_t)((buffer[6 + 2 * i + 1] << 8) | buffer[6 + 2 * i]);
    }

    accelRaw.x = IMU_TO_BODY_X(acc);
    accelRaw.y = IMU_TO_BODY_Y(acc);
    accelRaw.z = IMU_TO_BODY_Z(acc);
    gyroRaw.x = IMU_TO_BODY_X(gyr);
    gyroRaw.y = IMU_TO_BODY_Y(gyr);
    gyroRaw.z = IMU_TO_BODY_Z(gyr);

    // Gyro bias is found once the board has been still for a moment
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);

    // Accelerometer scale is measured while the board is still
    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

    accScaled.x = accelRaw.x * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.y = accelRaw.y * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.z = accelRaw.z * SENSORS_G_PER_LSB_CFG / accScale;

    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
}

/* ---------------------------------------------------------------------------
 * Barometer + magnetometer task (~25 Hz each, I2C)
 * ------------------------------------------------------------------------- */

// Standard-atmosphere altitude from pressure, as used by the Crazyflie
// LPS25H/BMP388 drivers. Only changes in altitude matter for height hold.
static float pressureToAltitude(float pressureMbar)
{
    const float CONST_PF = 0.1902630958f;   // (1/5.25588f)
    const float FIX_TEMP = 25.0f;           // fixed temperature for ASL
    const float SEA_LEVEL_MBAR = 1013.25f;

    return ((powf((SEA_LEVEL_MBAR / pressureMbar), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
}

static void baroMagTask(void *param)
{
    bool readBaroNext = true;
    TickType_t lastWake = xTaskGetTickCount();

    systemWaitStart();

    while (1) {
        vTaskDelayUntil(&lastWake, M2T(BARO_MAG_PERIOD_MS));

        if (readBaroNext && isBarometerPresent) {
            uint32_t tRaw, pRaw;
            float tC, pPa;

            if (bmp280_read_temperature_pressure(&bmp280, &tRaw, &tC, &pRaw, &pPa) == 0) {
                baro_t baro;
                baro.pressure = pPa / 100.0f;   // Pa -> mbar
                baro.temperature = tC;
                baro.asl = pressureToAltitude(baro.pressure);
                xQueueOverwrite(barometerDataQueue, &baro);
            }
        } else if (!readBaroNext && isMagnetometerPresent) {
            int16_t raw[3];
            float ut[3];

            if (bmm150_read(&bmm150, raw, ut) == 0) {
                // Same body-frame mapping as the IMU is a starting point only:
                // the BMM150's axes on the PCB must be checked separately.
                Axis3f mag;
                mag.x = IMU_TO_BODY_X(ut) / 100.0f; // uT -> gauss
                mag.y = IMU_TO_BODY_Y(ut) / 100.0f;
                mag.z = IMU_TO_BODY_Z(ut) / 100.0f;
                xQueueOverwrite(magnetometerDataQueue, &mag);
            }
        }

        readBaroNext = !readBaroNext;
    }
}

/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */
static bool bmi270DeviceInit(void)
{
    int8_t rslt;

    if (bmi270PortInit(&bmi270) != BMI2_OK) {
        DEBUG_PRINTE("BMI270 SPI bus init [FAIL]\n");
        return false;
    }

    // Soft reset, chip-ID check and upload of the 8 KB feature config
    rslt = bmi270_init(&bmi270);
    if (rslt != BMI2_OK) {
        DEBUG_PRINTE("BMI270 init [FAIL] (%d). Check SPI wiring / CS.\n", rslt);
        return false;
    }
    DEBUG_PRINTI("BMI270 SPI connection [OK] (chip id 0x%02X)\n", bmi270.chip_id);

    // Accel and gyro: 1600 Hz, +-16 g, +-2000 dps, performance-optimised filters
    struct bmi2_sens_config cfg[2];
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    bmi2_get_sensor_config(cfg, 2, &bmi270);

    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    rslt = bmi2_set_sensor_config(cfg, 2, &bmi270);
    if (rslt != BMI2_OK) {
        DEBUG_PRINTE("BMI270 sensor config [FAIL] (%d)\n", rslt);
        return false;
    }

    // Advanced power save adds a 450 us wait to every register access,
    // which a 1 kHz loop cannot afford.
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi270);

    uint8_t sensList[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sensList, 2, &bmi270);
    if (rslt != BMI2_OK) {
        DEBUG_PRINTE("BMI270 sensor enable [FAIL] (%d)\n", rslt);
        return false;
    }

    // INT1: data-ready, active-low open-drain (4.7k pull-up R27).
    // INT2: output disabled for now; its pull-up (R29) holds it high.
    struct bmi2_int_pin_config pinCfg;
    bmi2_get_int_pin_config(&pinCfg, &bmi270);
    pinCfg.pin_type = BMI2_INT_BOTH;
    pinCfg.int_latch = BMI2_INT_NON_LATCH;
    pinCfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
    pinCfg.pin_cfg[0].od = BMI2_INT_OPEN_DRAIN;
    pinCfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pinCfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    pinCfg.pin_cfg[1].lvl = BMI2_INT_ACTIVE_LOW;
    pinCfg.pin_cfg[1].od = BMI2_INT_OPEN_DRAIN;
    pinCfg.pin_cfg[1].output_en = BMI2_INT_OUTPUT_DISABLE;
    pinCfg.pin_cfg[1].input_en = BMI2_INT_INPUT_DISABLE;
    bmi2_set_int_pin_config(&pinCfg, &bmi270);
    bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi270);

    return true;
}

static void bmp280DeviceInit(void)
{
    bmp280PortLink(&bmp280, I2C0_DEV, BMP280_ADDRESS_ADO_LOW);

    if (bmp280_init(&bmp280) != 0) {
        DEBUG_PRINTW("BMP280 I2C connection [FAIL]\n");
        return;
    }

    // Bosch "indoor navigation" preset: ~26 Hz output, lowest noise
    bmp280_set_temperature_oversampling(&bmp280, BMP280_OVERSAMPLING_x2);
    bmp280_set_pressure_oversampling(&bmp280, BMP280_OVERSAMPLING_x16);
    bmp280_set_filter(&bmp280, BMP280_FILTER_COEFF_16);
    bmp280_set_standby_time(&bmp280, BMP280_STANDBY_TIME_0P5_MS);
    bmp280_set_mode(&bmp280, BMP280_MODE_NORMAL);

    isBarometerPresent = true;
    DEBUG_PRINTI("BMP280 I2C connection [OK]\n");
}

static void bmm150DeviceInit(void)
{
    bmm150PortLink(&bmm150, I2C0_DEV, BMM150_ADDRESS_CSB_SDO_00);

    if (bmm150_init(&bmm150) != 0) {
        DEBUG_PRINTW("BMM150 I2C connection [FAIL]\n");
        return;
    }

    // "Regular" preset (repxy 9, repz 15) in continuous (normal) mode at 25 Hz
    bmm150_set_repxy_number(&bmm150, 9);
    bmm150_set_repz_number(&bmm150, 15);
    bmm150_set_data_rate(&bmm150, BMM150_DATA_RATE_25HZ);
    bmm150_set_mode(&bmm150, BMM150_MODE_NORMAL);

    isMagnetometerPresent = true;
    DEBUG_PRINTI("BMM150 I2C connection [OK]\n");
}

static void sensorsDeviceInit(void)
{
    // Give the sensors time to power up
    while (xTaskGetTickCount() < 2000) {
        vTaskDelay(M2T(50));
    }

    i2cdevInit(I2C0_DEV);

    isBmi270Present = bmi270DeviceInit();
    bmp280DeviceInit();
    bmm150DeviceInit();

    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
    }

    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180);

    DEBUG_PRINTI("sensors init done");
}

static void sensorsInterruptInit(void)
{
    gpio_config_t ioConf = {
        .intr_type = GPIO_INTR_NEGEDGE,           // INT1 is active-low
        .pin_bit_mask = (1ULL << GPIO_IMU_INT1),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,                          // external 4.7k pull-up
    };
    gpio_config(&ioConf);

    // Returns ESP_ERR_INVALID_STATE if another driver already installed it,
    // which is fine: the service is shared.
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_IMU_INT1, imuInt1IsrHandler, NULL);
}

static void sensorsTaskInit(void)
{
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();

    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
    barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

    STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
    STATIC_MEM_TASK_CREATE(baroMagTask, baroMagTask, BARO_MAG_TASK_NAME, NULL, BARO_MAG_TASK_PRI);

    const esp_timer_create_args_t timerArgs = {
        .callback = sensorsTimerCallback,
        .name = "sensors1k",
    };
    esp_timer_create(&timerArgs, &sensorsTimer);
    esp_timer_start_periodic(sensorsTimer, SENSORS_LOOP_PERIOD_US);
}

void sensorsBmi270Bmp280Bmm150Init(void)
{
    if (isInit) {
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    sensorsInterruptInit();
    sensorsTaskInit();

    isInit = true;
}

bool sensorsBmi270Bmp280Bmm150Test(void)
{
    bool testStatus = true;

    if (!isInit) {
        DEBUG_PRINTE("Error while initializing sensor task\r\n");
        testStatus = false;
    }

    // Only the IMU is required to fly; baro and mag are reported but optional.
    testStatus &= isBmi270Present;

    return testStatus;
}

bool sensorsBmi270Bmp280Bmm150ManufacturingTest(void)
{
    return sensorsBmi270Bmp280Bmm150Test();
}

void sensorsBmi270Bmp280Bmm150SetAccMode(accModes accMode)
{
    switch (accMode) {
    case ACC_MODE_PROPTEST:
        // Wide filter so propeller vibration shows up in the prop test
        for (uint8_t i = 0; i < 3; i++) {
            lpf2pInit(&accLpf[i], 1000, 250);
        }
        break;
    case ACC_MODE_FLIGHT:
    default:
        for (uint8_t i = 0; i < 3; i++) {
            lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
        }
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Calibration helpers (unchanged from ESP-Drone)
 * ------------------------------------------------------------------------- */

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples.
 * Should be called when platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}

/**
 * Calculates the bias first when the gyro variance is below threshold.
 * Requires a buffer but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        if (gyroBiasRunning.isBiasValueFound) {
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("isBiasValueFound!");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int64_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 */
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
                (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }

    return foundBias;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data from menuconfig (PITCH_CALIB / ROLL_CALIB) to rotate the
 * accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

/* ---------------------------------------------------------------------------
 * Params and logs, visible from the PC client (cfclient) or cflib scripts
 * ------------------------------------------------------------------------- */
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMI270, &isBmi270Present)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP280, &isBarometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMM150, &isMagnetometerPresent)
PARAM_GROUP_STOP(imu_sensors)

LOG_GROUP_START(imu_drdy)
LOG_ADD(LOG_UINT32, count, &imuDrdyCount)
LOG_GROUP_STOP(imu_drdy)
