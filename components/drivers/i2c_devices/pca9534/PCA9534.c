/**
 **********************************************************************************
 * @file   PCA9534.c
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  PCA9534 and PCA9534A 8-bit I/O expander driver
 **********************************************************************************
 *
 * Copyright (c) 2025 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */

/* Includes ---------------------------------------------------------------------*/
#include "PCA9534.h"
#include <stdio.h>


/* Private Constants ------------------------------------------------------------*/
/**
 * @brief  I2C device addresses
 */
#define PCA9534_I2C_ADDRESS_BASE    0x20
#define PCA9534A_I2C_ADDRESS_BASE   0x38

/**
 * @brief  Register addresses
 */
#define PCA9534_REG_INPUT_PORT      0x00
#define PCA9534_REG_OUTPUT_PORT     0x01
#define PCA9534_REG_POLARITY_INVERT 0x02
#define PCA9534_REG_CONFIGURATION   0x03



/**
 ==================================================================================
                       ##### Private Functions #####
 ==================================================================================
 */
static PCA9534_Result_t
PCA9534_WriteReg(PCA9534_Handler_t *Handler, uint8_t Address, uint8_t Data)
{
  uint8_t Buffer[2] = {Address, Data};
  if (Handler->Platform.Send(Handler->AddressI2C, Buffer, 2) < 0)
    return PCA9534_FAIL;

  return PCA9534_OK;
}

static PCA9534_Result_t
PCA9534_ReadReg(PCA9534_Handler_t *Handler, uint8_t Address, uint8_t *Data)
{
  if (Handler->Platform.Send(Handler->AddressI2C, &Address, 1) < 0)
    return PCA9534_FAIL;

  if (Handler->Platform.Receive(Handler->AddressI2C, Data, 1) < 0)
    return PCA9534_FAIL;

  return PCA9534_OK;
}



/**
 ==================================================================================
                            ##### Public Functions #####
 ==================================================================================
 */

/**
 * @brief  Initializer function
 * @note   This function must be called after initializing platform dependent
 *         layer and before using other functions.
 * @param  Handler: Pointer to handler
 * @param  Device: Device type
 * @param  Address: Address pins state (0 <= Address <= 7)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 *         - PCA9534_INVALID_PARAM: Invalid parameter.
 */
PCA9534_Result_t
PCA9534_Init(PCA9534_Handler_t *Handler, PCA9534_Device_t Device,
             uint8_t Address)
{
  if (!Handler)
    return PCA9534_INVALID_PARAM;

  if (Device != PCA9534_DEVICE_PCA9534 &&
      Device != PCA9534_DEVICE_PCA9534A)
    return PCA9534_INVALID_PARAM;
  Handler->Device = Device;

  if (PCA9534_SetAddressI2C(Handler, Address) != PCA9534_OK)
    return PCA9534_INVALID_PARAM;

  if (!Handler->Platform.Send || !Handler->Platform.Receive)
    return PCA9534_INVALID_PARAM;

  if (Handler->Platform.Init)
  {
    if (Handler->Platform.Init() < 0)
      return PCA9534_FAIL;
  }

  // Reset all registers to default values
  if (PCA9534_WriteReg(Handler, PCA9534_REG_OUTPUT_PORT, 0xFF) != PCA9534_OK)
    return PCA9534_FAIL;

  if (PCA9534_WriteReg(Handler, PCA9534_REG_POLARITY_INVERT, 0x00) != PCA9534_OK)
    return PCA9534_FAIL;

  if (PCA9534_WriteReg(Handler, PCA9534_REG_CONFIGURATION, 0xFF) != PCA9534_OK)
    return PCA9534_FAIL;

  return PCA9534_OK;
}


/**
 * @brief  Deinitialize function
 * @param  Handler: Pointer to handler
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_DeInit(PCA9534_Handler_t *Handler)
{
  if (!Handler)
    return PCA9534_INVALID_PARAM;

  if (Handler->Platform.DeInit)
    return ((Handler->Platform.DeInit() >= 0) ? PCA9534_OK : PCA9534_FAIL);

  return PCA9534_OK;
}


/**
 * @brief  Set I2C Address
 * @param  Handler: Pointer to handler
 * @param  Address: Address pins state (0 <= Address <= 7)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_INVALID_PARAM: One of parameters is invalid.
 */
PCA9534_Result_t
PCA9534_SetAddressI2C(PCA9534_Handler_t *Handler, uint8_t Address)
{
  if (Address > 7)
    return PCA9534_INVALID_PARAM;

  switch (Handler->Device)
  {
  case PCA9534_DEVICE_PCA9534:
    Handler->AddressI2C = PCA9534_I2C_ADDRESS_BASE | Address;
    break;

  case PCA9534_DEVICE_PCA9534A:
    Handler->AddressI2C = PCA9534A_I2C_ADDRESS_BASE | Address;
    break;

  default:
    return PCA9534_INVALID_PARAM;
  }

  return PCA9534_OK;
}

/**
 * @brief  Set direction of pins
 * @param  Handler: Pointer to handler
 * @param  Dir: Direction of pins (1: Output, 0: Input)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_SetDir(PCA9534_Handler_t *Handler, uint8_t Dir)
{
  Dir = ~Dir;
  if (PCA9534_WriteReg(Handler, PCA9534_REG_CONFIGURATION, Dir) != PCA9534_OK)
    return PCA9534_FAIL;
  return PCA9534_OK;
}


/**
 * @brief  Set the direction of one bit
 * @param  Handler: Pointer to handler
 * @param  Pos: Position of bit (0 <= Pos <= 7)
 * @param  Dir: Direction of bit (1: Output, 0: Input)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 *         - PCA9534_INVALID_PARAM: One of parameters is invalid.
 */
PCA9534_Result_t
PCA9534_SetDirOne(PCA9534_Handler_t *Handler, uint8_t Pos, uint8_t Dir)
{
  if (Pos > 7)
    return PCA9534_INVALID_PARAM;

  uint8_t Reg = 0;
  if (PCA9534_ReadReg(Handler, PCA9534_REG_CONFIGURATION, &Reg) != PCA9534_OK)
    return PCA9534_FAIL;

  if (Dir)
    Reg &= ~(1 << Pos);
  else
    Reg |= (1 << Pos);

  return PCA9534_SetDir(Handler, Reg);
}


/**
 * @brief  Read data from the device
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to data
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 *         - PCA9534_INVALID_PARAM: Invalid parameter.
 */
PCA9534_Result_t
PCA9534_Read(PCA9534_Handler_t *Handler, uint8_t *Data)
{
  if (!Data)
    return PCA9534_INVALID_PARAM;

  if (PCA9534_ReadReg(Handler, PCA9534_REG_INPUT_PORT, Data) != PCA9534_OK)
    return PCA9534_FAIL;

  return PCA9534_OK;
}


/**
 * @brief  Write data to the device
 * @param  Handler: Pointer to handler
 * @param  Data: Data to write
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_Write(PCA9534_Handler_t *Handler, uint8_t Data)
{
  if (PCA9534_WriteReg(Handler, PCA9534_REG_OUTPUT_PORT, Data) != PCA9534_OK)
    return PCA9534_FAIL;

  return PCA9534_OK;
}


/**
 * @brief  Write data to the one bit
 * @param  Handler: Pointer to handler
 * @param  Pos: Position of bit (0 <= Pos <= 7)
 * @param  Value: Value to write (1: High, 0: Low)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 *         - PCA9534_INVALID_PARAM: One of parameters is invalid.
 */
PCA9534_Result_t
PCA9534_WriteOne(PCA9534_Handler_t *Handler, uint8_t Pos, uint8_t Value)
{
  if (Pos > 7)
    return PCA9534_INVALID_PARAM;

  uint8_t Reg = 0;
  if (PCA9534_ReadReg(Handler, PCA9534_REG_OUTPUT_PORT, &Reg) != PCA9534_OK)
    return PCA9534_FAIL;

  if (Value)
    Reg |= (1 << Pos);
  else
    Reg &= ~(1 << Pos);

  return PCA9534_Write(Handler, Reg);
}


/**
 * @brief  Toggle the output bits
 * @param  Handler: Pointer to handler
 * @param  Mask: Mask of bits to toggle
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_Toggle(PCA9534_Handler_t *Handler, uint8_t Mask)
{
  uint8_t Reg = 0;
  if (PCA9534_ReadReg(Handler, PCA9534_REG_OUTPUT_PORT, &Reg) != PCA9534_OK)
    return PCA9534_FAIL;

  Reg ^= Mask;
  return PCA9534_Write(Handler, Reg);
}


/**
 * @brief  Toggle the one bit
 * @param  Handler: Pointer to handler
 * @param  Pos: Position of bit (0 <= Pos <= 7)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 *         - PCA9534_INVALID_PARAM: One of parameters is invalid.
 */
PCA9534_Result_t
PCA9534_ToggleOne(PCA9534_Handler_t *Handler, uint8_t Pos)
{
  if (Pos > 7)
    return PCA9534_INVALID_PARAM;

  uint8_t Mask = 1 << Pos;
  return PCA9534_Toggle(Handler, Mask);
}
