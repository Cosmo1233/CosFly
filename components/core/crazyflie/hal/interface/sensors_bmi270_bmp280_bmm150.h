/**
 * CosFly V1 sensor implementation: BMI270 IMU (SPI), BMP280 barometer and
 * BMM150 magnetometer (both on the internal I2C bus).
 *
 * These functions are not called directly; sensors.c picks this
 * implementation when the platform config says
 * SensorImplementation_bmi270_bmp280_bmm150 and forwards to it.
 */
#ifndef __SENSORS_BMI270_BMP280_BMM150_H__
#define __SENSORS_BMI270_BMP280_BMM150_H__

#include "sensors.h"

void sensorsBmi270Bmp280Bmm150Init(void);
bool sensorsBmi270Bmp280Bmm150Test(void);
bool sensorsBmi270Bmp280Bmm150AreCalibrated(void);
bool sensorsBmi270Bmp280Bmm150ManufacturingTest(void);
void sensorsBmi270Bmp280Bmm150Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsBmi270Bmp280Bmm150WaitDataReady(void);
bool sensorsBmi270Bmp280Bmm150ReadGyro(Axis3f *gyro);
bool sensorsBmi270Bmp280Bmm150ReadAcc(Axis3f *acc);
bool sensorsBmi270Bmp280Bmm150ReadMag(Axis3f *mag);
bool sensorsBmi270Bmp280Bmm150ReadBaro(baro_t *baro);
void sensorsBmi270Bmp280Bmm150SetAccMode(accModes accMode);

#endif // __SENSORS_BMI270_BMP280_BMM150_H__
