/**
 * CosFly: ESP-IDF port layer for the LibDriver BMP280 driver.
 * See bmp280_port.h for what this file is for.
 */
#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include "bmp280_port.h"

// LibDriver's function pointers take no context argument, so the bus the
// chip is on has to live in a file-level variable.
static I2C_Dev *portBus;

// LibDriver passes 8-bit (already left-shifted) I2C addresses; i2cdev wants
// the 7-bit address.
static uint8_t portIicRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2cdevReadReg8(portBus, addr >> 1, reg, len, buf) ? 0 : 1;
}

static uint8_t portIicWrite(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2cdevWriteReg8(portBus, addr >> 1, reg, len, buf) ? 0 : 1;
}

// The bus itself is brought up by i2cdevInit() before the driver is used.
static uint8_t portIicInit(void) { return 0; }
static uint8_t portIicDeinit(void) { return 0; }

// bmp280_init() refuses to run unless the SPI pointers are also non-NULL, even
// in I2C mode. These are never called on CosFly (CSB is tied high = I2C).
static uint8_t portSpiInit(void) { return 1; }
static uint8_t portSpiDeinit(void) { return 1; }
static uint8_t portSpiRead(uint8_t reg, uint8_t *buf, uint16_t len) { return 1; }
static uint8_t portSpiWrite(uint8_t reg, uint8_t *buf, uint16_t len) { return 1; }

static void portDelayMs(uint32_t ms)
{
    if (ms < portTICK_PERIOD_MS) {
        esp_rom_delay_us(ms * 1000);
    } else {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static void portDebugPrint(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void bmp280PortLink(bmp280_handle_t *handle, I2C_Dev *bus, bmp280_address_t addr)
{
    portBus = bus;

    DRIVER_BMP280_LINK_INIT(handle, bmp280_handle_t);
    DRIVER_BMP280_LINK_IIC_INIT(handle, portIicInit);
    DRIVER_BMP280_LINK_IIC_DEINIT(handle, portIicDeinit);
    DRIVER_BMP280_LINK_IIC_READ(handle, portIicRead);
    DRIVER_BMP280_LINK_IIC_WRITE(handle, portIicWrite);
    DRIVER_BMP280_LINK_SPI_INIT(handle, portSpiInit);
    DRIVER_BMP280_LINK_SPI_DEINIT(handle, portSpiDeinit);
    DRIVER_BMP280_LINK_SPI_READ(handle, portSpiRead);
    DRIVER_BMP280_LINK_SPI_WRITE(handle, portSpiWrite);
    DRIVER_BMP280_LINK_DELAY_MS(handle, portDelayMs);
    DRIVER_BMP280_LINK_DEBUG_PRINT(handle, portDebugPrint);

    bmp280_set_interface(handle, BMP280_INTERFACE_IIC);
    bmp280_set_addr_pin(handle, addr);
}
