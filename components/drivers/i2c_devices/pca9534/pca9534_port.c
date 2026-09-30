/**
 * CosFly: ESP-IDF port layer for the Mahda PCA9534 driver.
 * See pca9534_port.h for why this replaces PCA9534_platform.c.
 */
#include "pca9534_port.h"

// The library's function pointers take no context argument, so the bus has
// to live in a file-level variable.
static I2C_Dev *portBus;

// The library passes 7-bit addresses (0x38 etc.) and builds the register
// byte into Data itself, so these are plain I2C writes and reads.
static int8_t portSend(uint8_t Address, uint8_t *Data, uint8_t Len)
{
    return i2cdevWrite(portBus, Address, Len, Data) ? 0 : -1;
}

static int8_t portReceive(uint8_t Address, uint8_t *Data, uint8_t Len)
{
    return i2cdevRead(portBus, Address, Len, Data) ? 0 : -1;
}

// i2cdevInit() is a no-op if the bus is already up, so it is safe to call
// here even though the sensors also initialise I2C0.
static int8_t portInit(void)
{
    i2cdevInit(portBus);
    return 0;
}

void pca9534PortLink(PCA9534_Handler_t *handler, I2C_Dev *bus)
{
    portBus = bus;

    PCA9534_PLATFORM_LINK_INIT(handler, portInit);
    PCA9534_PLATFORM_LINK_DEINIT(handler, NULL);
    PCA9534_PLATFORM_LINK_SEND(handler, portSend);
    PCA9534_PLATFORM_LINK_RECEIVE(handler, portReceive);
}
