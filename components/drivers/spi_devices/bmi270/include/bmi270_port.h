/**
 * CosFly: ESP-IDF SPI port layer for the Bosch BMI270 SensorAPI.
 *
 * The Bosch API never touches hardware directly. It calls the read/write/
 * delay_us function pointers in struct bmi2_dev, and this file provides those
 * using the ESP-IDF spi_master driver. Sensor settings (ODR, range, interrupt
 * mapping) are chosen by the caller after bmi270_init(), not here.
 */
#ifndef BMI270_PORT_H
#define BMI270_PORT_H

#include "bmi2.h"

/**
 * Bring up the IMU SPI bus (CONFIG_SPI_PIN_*, hardware chip select on
 * CONFIG_SPI_PIN_CS0) and fill in @p dev's interface fields.
 *
 * Call this, then bmi270_init(dev).
 *
 * @return BMI2_OK on success, BMI2_E_COM_FAIL if the SPI bus could not be set up.
 */
int8_t bmi270PortInit(struct bmi2_dev *dev);

#endif
