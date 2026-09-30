/**
 **********************************************************************************
 * @file   PCA9534.h
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

/* Define to prevent recursive inclusion ----------------------------------------*/
#ifndef	_PCA9534_H_
#define _PCA9534_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ---------------------------------------------------------------------*/
#include <stdint.h>


/* Exported Data Types ----------------------------------------------------------*/
/**
 * @brief  Library functions result data type
 */
typedef enum PCA9534_Result_e
{
  PCA9534_OK              = 0,
  PCA9534_FAIL            = 1,
  PCA9534_INVALID_PARAM   = 2,
} PCA9534_Result_t;

/**
 * @brief  Device type
 */
typedef enum PCA9534_Device_e
{
  PCA9534_DEVICE_PCA9534 = 0,
  PCA9534_DEVICE_PCA9534A = 1,
} PCA9534_Device_t;


/**
 * @brief  Function type for Initialize/Deinitialize the platform dependent layer.
 * @retval
 *         -  0: The operation was successful.
 *         - -1: The operation failed.
 */
typedef int8_t (*PCA9534_Platform_InitDeinit_t)(void);

/**
 * @brief  Function type for Send/Receive data to/from the slave.
 * @param  Address: Address of slave (0 <= Address <= 127)
 * @param  Data: Pointer to data
 * @param  Len: data len in Bytes
 * @retval
 *         -  0: The operation was successful.
 *         - -1: Failed to send/receive.
 *         - -2: Bus is busy.
 *         - -3: Slave doesn't ACK the transfer.
 */
typedef int8_t (*PCA9534_Platform_SendReceive_t)(uint8_t Address,
                                                 uint8_t *Data, uint8_t Len);

/**
 * @brief  Platform dependent layer data type
 * @note   It is optional to initialize this functions:
 *         - Init
 *         - DeInit
 * @note   It is mandatory to initialize this functions:
 *         - Send
 *         - Receive
 * @note   If success the functions must return 0
 */
typedef struct PCA9534_Platform_s
{
  // Initialize platform dependent layer
  PCA9534_Platform_InitDeinit_t Init;
  // De-initialize platform dependent layer
  PCA9534_Platform_InitDeinit_t DeInit;

  // Send data to the slave
  PCA9534_Platform_SendReceive_t Send;
  // Receive data from the slave
  PCA9534_Platform_SendReceive_t Receive;
} PCA9534_Platform_t;


/**
 * @brief  Handler data type
 * @note   User must initialize platform dependent layer functions
 */
typedef struct PCA9534_Handler_s
{
  // Device type
  PCA9534_Device_t Device;

  // I2C Address
  uint8_t AddressI2C;

  // Platform dependent layer
  PCA9534_Platform_t Platform;
} PCA9534_Handler_t;


/* Exported Macros --------------------------------------------------------------*/
/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define PCA9534_PLATFORM_LINK_INIT(HANDLER, FUNC) \
  (HANDLER)->Platform.Init = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define PCA9534_PLATFORM_LINK_DEINIT(HANDLER, FUNC) \
  (HANDLER)->Platform.DeInit = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define PCA9534_PLATFORM_LINK_SEND(HANDLER, FUNC) \
  (HANDLER)->Platform.Send = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define PCA9534_PLATFORM_LINK_RECEIVE(HANDLER, FUNC) \
  (HANDLER)->Platform.Receive = FUNC




/**
 ==================================================================================
                        ##### Initialization Functions #####
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
             uint8_t Address);


/**
 * @brief  Deinitialize function
 * @param  Handler: Pointer to handler
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_DeInit(PCA9534_Handler_t *Handler);


/**
 * @brief  Set I2C Address
 * @param  Handler: Pointer to handler
 * @param  Address: Address pins state (0 <= Address <= 7)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_INVALID_PARAM: One of parameters is invalid.
 */
PCA9534_Result_t
PCA9534_SetAddressI2C(PCA9534_Handler_t *Handler, uint8_t Address);



/**
 ==================================================================================
                             ##### I/O Functions #####
 ==================================================================================
 */

/**
 * @brief  Set direction of pins
 * @param  Handler: Pointer to handler
 * @param  Dir: Direction of pins (1: Output, 0: Input)
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_SetDir(PCA9534_Handler_t *Handler, uint8_t Dir);


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
PCA9534_SetDirOne(PCA9534_Handler_t *Handler, uint8_t Pos, uint8_t Dir);


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
PCA9534_Read(PCA9534_Handler_t *Handler, uint8_t *Data);


/**
 * @brief  Write data to the device
 * @param  Handler: Pointer to handler
 * @param  Data: Data to write
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_Write(PCA9534_Handler_t *Handler, uint8_t Data);


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
PCA9534_WriteOne(PCA9534_Handler_t *Handler, uint8_t Pos, uint8_t Value);


/**
 * @brief  Toggle the output bits
 * @param  Handler: Pointer to handler
 * @param  Mask: Mask of bits to toggle
 * @retval PCA9534_Result_t
 *         - PCA9534_OK: Operation was successful.
 *         - PCA9534_FAIL: Failed to send or receive data.
 */
PCA9534_Result_t
PCA9534_Toggle(PCA9534_Handler_t *Handler, uint8_t Mask);


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
PCA9534_ToggleOne(PCA9534_Handler_t *Handler, uint8_t Pos);



#ifdef __cplusplus
}
#endif

#endif //! _PCA9534_H_
