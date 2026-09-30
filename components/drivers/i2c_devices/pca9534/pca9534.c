/**
 * CosFly: minimal driver for the TI PCA9534 / PCA9534A I2C GPIO expander.
 * See pca9534.h.
 */
#include "pca9534.h"

#define PCA9534_REG_INPUT    0x00
#define PCA9534_REG_OUTPUT   0x01
#define PCA9534_REG_POLARITY 0x02
#define PCA9534_REG_CONFIG   0x03

static I2C_Dev *bus;
static uint8_t devAddr;
static uint8_t outputCache;
static bool isInit = false;

bool pca9534Init(I2C_Dev *i2cBus, uint8_t addr)
{
    bus = i2cBus;
    devAddr = addr;

    i2cdevInit(bus);    // no-op if the bus is already up

    // Reading the Output register both proves the chip is there and seeds
    // the cache (power-on value is 0xFF).
    isInit = i2cdevReadByte(bus, devAddr, PCA9534_REG_OUTPUT, &outputCache);

    return isInit;
}

bool pca9534Test(void)
{
    return isInit;
}

bool pca9534ConfigOutputs(uint8_t outputMask, uint8_t initialLevels)
{
    if (!isInit) {
        return false;
    }

    outputCache = initialLevels;
    bool pass = i2cdevWriteByte(bus, devAddr, PCA9534_REG_OUTPUT, outputCache);
    // Config register: 1 = input, 0 = output
    pass &= i2cdevWriteByte(bus, devAddr, PCA9534_REG_CONFIG, (uint8_t)~outputMask);

    return pass;
}

bool pca9534WritePin(uint8_t pin, bool high)
{
    if (!isInit || pin > 7) {
        return false;
    }

    uint8_t next = high ? (outputCache | (1 << pin)) : (outputCache & ~(1 << pin));
    if (next == outputCache) {
        return true;    // no change, skip the bus transfer
    }

    outputCache = next;
    return i2cdevWriteByte(bus, devAddr, PCA9534_REG_OUTPUT, outputCache);
}

bool pca9534ReadInputs(uint8_t *levels)
{
    if (!isInit) {
        return false;
    }

    return i2cdevReadByte(bus, devAddr, PCA9534_REG_INPUT, levels);
}
