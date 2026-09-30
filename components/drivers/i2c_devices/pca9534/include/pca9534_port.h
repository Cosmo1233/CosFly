/**
 * CosFly: ESP-IDF port layer for the Mahda PCA9534 driver.
 *
 * Replaces the library's sample PCA9534_platform.c, which installs its own
 * I2C driver on I2C_NUM_0 (SDA = GPIO2). On CosFly I2C0 is already owned by
 * i2cdev on GPIO33/34 and GPIO2 is Motor L-Up, so this port routes the
 * library's Send/Receive through i2cdev instead, sharing the bus mutex with
 * the barometer, magnetometer and EEPROM.
 */
#ifndef PCA9534_PORT_H
#define PCA9534_PORT_H

#include "PCA9534.h"
#include "i2cdev.h"

/**
 * Link @p handler's platform functions to @p bus. Call this, then
 * PCA9534_Init().
 */
void pca9534PortLink(PCA9534_Handler_t *handler, I2C_Dev *bus);

#endif
