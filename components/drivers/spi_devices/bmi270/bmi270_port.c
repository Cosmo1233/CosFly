/**
 * CosFly: ESP-IDF SPI port layer for the Bosch BMI270 SensorAPI.
 * See bmi270_port.h for what this file is for.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

#include "bmi270_port.h"

// SPI2 (FSPI) is routed to the IMU pins through the GPIO matrix. The BMI270
// allows up to 10 MHz; 8 MHz leaves margin and still reads a full
// accel + gyro sample (13 bytes) in ~15 us.
#define IMU_SPI_HOST     SPI2_HOST
#define IMU_SPI_CLOCK_HZ (8 * 1000 * 1000)

// Largest single burst the Bosch API may ask for. Without DMA the ESP32-S3
// SPI peripheral moves at most 64 bytes per transaction, so the 8 KB config
// file that bmi270_init() uploads is split into chunks of this size.
#define IMU_SPI_MAX_BURST 32

static spi_device_handle_t imuSpi;

/*
 * Bosch has already set bit 7 of reg_addr for reads and asks for one extra
 * "dummy" byte at the front, which it strips afterwards (bmi2_get_regs()).
 * So a read is simply: send reg_addr, then clock in len bytes.
 */
static BMI2_INTF_RETURN_TYPE spiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.addr = reg_addr;
    t.length = len * 8;     // full-duplex: MOSI sends zeros while MISO is read
    t.rxlength = len * 8;
    t.rx_buffer = reg_data;

    return (spi_device_polling_transmit(imuSpi, &t) == ESP_OK) ? BMI2_INTF_RET_SUCCESS : -1;
}

static BMI2_INTF_RETURN_TYPE spiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.addr = reg_addr;
    t.length = len * 8;
    t.tx_buffer = reg_data;

    return (spi_device_polling_transmit(imuSpi, &t) == ESP_OK) ? BMI2_INTF_RET_SUCCESS : -1;
}

// The API asks for anything from 2 us to tens of ms. Busy-wait the short ones,
// sleep the long ones so other tasks can run.
static void delayUs(uint32_t period, void *intf_ptr)
{
    if (period < 1000) {
        esp_rom_delay_us(period);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    }
}

int8_t bmi270PortInit(struct bmi2_dev *dev)
{
    spi_bus_config_t bus = {
        .miso_io_num = CONFIG_SPI_PIN_MISO,
        .mosi_io_num = CONFIG_SPI_PIN_MOSI,
        .sclk_io_num = CONFIG_SPI_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,   // default = 64 bytes when DMA is disabled
    };
    if (spi_bus_initialize(IMU_SPI_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) {
        return BMI2_E_COM_FAIL;
    }

    spi_device_interface_config_t devcfg = {
        .address_bits = 8,      // the register address goes out in the address phase
        .mode = 0,              // BMI270 supports SPI modes 0 and 3
        .clock_speed_hz = IMU_SPI_CLOCK_HZ,
        .spics_io_num = CONFIG_SPI_PIN_CS0,  // hardware-driven CS
        .queue_size = 1,
    };
    if (spi_bus_add_device(IMU_SPI_HOST, &devcfg, &imuSpi) != ESP_OK) {
        return BMI2_E_COM_FAIL;
    }

    memset(dev, 0, sizeof(*dev));
    dev->intf = BMI2_SPI_INTF;
    dev->read = spiRead;
    dev->write = spiWrite;
    dev->delay_us = delayUs;
    dev->read_write_len = IMU_SPI_MAX_BURST;
    dev->config_file_ptr = NULL;    // use the config blob built into bmi270.c
    dev->intf_ptr = NULL;

    return BMI2_OK;
}
