/**
 * CosFly: ESP-IDF port layer for the LibDriver BMM150 driver.
 *
 * Same idea as bmp280_port.h: fills in the handle's function pointers with
 * the project's i2cdev functions so driver_bmm150.c stays unmodified.
 */
#ifndef BMM150_PORT_H
#define BMM150_PORT_H

#include "driver_bmm150.h"
#include "i2cdev.h"

/**
 * Point every function pointer in @p handle at the ESP-IDF implementation and
 * select the I2C interface on @p bus. Call this before bmm150_init().
 */
void bmm150PortLink(bmm150_handle_t *handle, I2C_Dev *bus, bmm150_address_t addr);

#endif
