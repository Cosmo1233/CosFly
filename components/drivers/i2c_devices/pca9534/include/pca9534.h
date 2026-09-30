/**
 * CosFly: minimal driver for the TI PCA9534 / PCA9534A 8-bit I2C GPIO expander.
 *
 * The chip has four registers (datasheet section 8.6):
 *   0x00 Input    - pin levels as read
 *   0x01 Output   - levels driven on pins configured as outputs
 *   0x02 Polarity - inverts the Input register (not used here)
 *   0x03 Config   - 1 = input (power-on default), 0 = output
 *
 * Output writes go through a cached copy of the Output register, so changing
 * one pin is a single I2C write instead of a read-modify-write.
 */
#ifndef PCA9534_H
#define PCA9534_H

#include <stdbool.h>
#include <stdint.h>

#include "i2cdev.h"

// 7-bit addresses with A2..A0 = 000. Add the A2..A0 value for other straps.
#define PCA9534_ADDRESS_BASE   0x20    // PCA9534  : 0100 A2 A1 A0
#define PCA9534A_ADDRESS_BASE  0x38    // PCA9534A : 0111 A2 A1 A0

/**
 * Bring up @p bus (if not already) and check the expander answers at @p addr.
 * Loads the chip's current Output register into the cache.
 */
bool pca9534Init(I2C_Dev *bus, uint8_t addr);

/** True if pca9534Init() succeeded. */
bool pca9534Test(void);

/**
 * Set every output level (Output register), then make the pins in
 * @p outputMask outputs (bit = 1 -> output). Writing the levels first means
 * pins never glitch to the power-on default when they switch to outputs.
 */
bool pca9534ConfigOutputs(uint8_t outputMask, uint8_t initialLevels);

/** Drive one pin (0..7) high or low. The pin must already be an output. */
bool pca9534WritePin(uint8_t pin, bool high);

/** Read the level on all eight pins. */
bool pca9534ReadInputs(uint8_t *levels);

#endif
