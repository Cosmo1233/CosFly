/**
 * CosFly: ESP-IDF port layer for the LibDriver BMP280 driver.
 *
 * LibDriver drivers are hardware-agnostic: they talk to the chip only through
 * function pointers stored in a handle (iic_read, iic_write, delay_ms, ...).
 * This file fills those pointers in with the project's i2cdev functions, so
 * the vendor code in driver_bmp280.c can stay unmodified.
 */
#ifndef BMP280_PORT_H
#define BMP280_PORT_H

#include "driver_bmp280.h"
#include "i2cdev.h"

/**
 * Point every function pointer in @p handle at the ESP-IDF implementation and
 * select the I2C interface on @p bus. Call this before bmp280_init().
 */
void bmp280PortLink(bmp280_handle_t *handle, I2C_Dev *bus, bmp280_address_t addr);

#endif
