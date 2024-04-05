/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  @file   coines_common.h
 *  @brief  This file contains common functions and declarations for LEGACY and COINES Bridge
 */

#ifndef COINES_COMMON_H
#define COINES_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* includes */
/**********************************************************************************/
#include "coines.h"

/*********************************************************************/
/* Typedef definitions */
/**********************************************************************/
typedef struct
{
    int16_t code;
    const char *message;
} error_code_mapping;

/*********************************************************************/
/* Macro definitions */
/**********************************************************************/
#define NUM_ERROR_CODES                            44

/*! COINES_SDK success code */
#define COINES_SUCCESS                             0

/*! COINES_SDK error code - failure */
#define COINES_E_FAILURE                           -1

/*! COINES_SDK error code - IO error */
#define COINES_E_COMM_IO_ERROR                     -2

/*! COINES_SDK error code - Init failure */
#define COINES_E_COMM_INIT_FAILED                  -3

/*! COINES_SDK error code - failure to open device */
#define COINES_E_UNABLE_OPEN_DEVICE                -4

/*! COINES_SDK error code - Device not found */
#define COINES_E_DEVICE_NOT_FOUND                  -5

/*! COINES_SDK error code - failure to claim interface */
#define COINES_E_UNABLE_CLAIM_INTF                 -6

/*! COINES_SDK error code - failure to allocate memory */
#define COINES_E_MEMORY_ALLOCATION                 -7

/*! COINES_SDK error code - Feature not supported */
#define COINES_E_NOT_SUPPORTED                     -8

/*! COINES_SDK error code - Null pointer */
#define COINES_E_NULL_PTR                          -9

/*! COINES_SDK error code - Wrong response */
#define COINES_E_COMM_WRONG_RESPONSE               -10

/*! COINES_SDK error code - Not configured */
#define COINES_E_SPI16BIT_NOT_CONFIGURED           -11

/*! COINES_SDK error code - SPI invalid bus interface */
#define COINES_E_SPI_INVALID_BUS_INTF              -12

/*! COINES_SDK error code - SPI instance configured already */
#define COINES_E_SPI_CONFIG_EXIST                  -13

/*! COINES_SDK error code - SPI bus not enabled */
#define COINES_E_SPI_BUS_NOT_ENABLED               -14

/*! COINES_SDK error code - SPI instance configuration failed */
#define COINES_E_SPI_CONFIG_FAILED                 -15

/*! COINES_SDK error code - I2C invalid bus interface */
#define COINES_E_I2C_INVALID_BUS_INTF              -16

/*! COINES_SDK error code - I2C bus not enabled */
#define COINES_E_I2C_BUS_NOT_ENABLED               -17

/*! COINES_SDK error code - I2C instance configuration failed */
#define COINES_E_I2C_CONFIG_FAILED                 -18

/*! COINES_SDK error code - I2C instance configured already */
#define COINES_E_I2C_CONFIG_EXIST                  -19

/*! COINES_SDK error code - Timer initialization failed */
#define COINES_E_TIMER_INIT_FAILED                 -20

/*! COINES_SDK error code - Invalid timer instance */
#define COINES_E_TIMER_INVALID_INSTANCE            -21

/*! COINES_SDK error code - Invalid timer instance */
#define COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE    -22

/*! COINES_SDK error code - EEPROM reset failed */
#define COINES_E_EEPROM_RESET_FAILED               -23

/*! COINES_SDK error code - EEPROM read failed */
#define COINES_E_EEPROM_READ_FAILED                -24

/*! COINES_SDK error code - Initialization failed */
#define COINES_E_INIT_FAILED                       -25

/*! COINES_SDK error code - Streaming not configure */
#define COINES_E_STREAM_NOT_CONFIGURED             -26

/*! COINES_SDK error code - Streaming invalid block size */
#define COINES_E_STREAM_INVALID_BLOCK_SIZE         -27

/*! COINES_SDK error code - Streaming sensor already configured */
#define COINES_E_STREAM_SENSOR_ALREADY_CONFIGURED  -28

/*! COINES_SDK error code - Streaming sensor config memory full */
#define COINES_E_STREAM_CONFIG_MEMORY_FULL         -29

/*! COINES_SDK error code - Invalid payload length */
#define COINES_E_INVALID_PAYLOAD_LEN               -30

/*! COINES_SDK error code - channel allocation failed */
#define COINES_E_CHANNEL_ALLOCATION_FAILED         -31

/*! COINES_SDK error code - channel de-allocation failed */
#define COINES_E_CHANNEL_DEALLOCATION_FAILED       -32

/*! COINES_SDK error code - channel assignment failed */
#define COINES_E_CHANNEL_ASSIGN_FAILED             -33

/*! COINES_SDK error code - channel enable failed */
#define COINES_E_CHANNEL_ENABLE_FAILED             -34

/*! COINES_SDK error code - channel disable failed */
#define COINES_E_CHANNEL_DISABLE_FAILED            -35

/*! COINES_SDK error code - GPIO invalid pin number */
#define COINES_E_INVALID_PIN_NUMBER                -36

/*! COINES_SDK error code - GPIO invalid pin number */
#define COINES_E_MAX_SENSOR_COUNT_REACHED          -37

/*! COINES_SDK error code - EEPROM write failed */
#define COINES_E_EEPROM_WRITE_FAILED               -38

/*! COINES_SDK error code - Invalid EEPROM write length */
#define COINES_E_INVALID_EEPROM_RW_LENGTH          -39

/*! COINES_SDK error code - Invalid serial com config */
#define COINES_E_INVALID_SCOM_CONFIG               -40

/*! COINES_SDK error code - Invalid BLE config */
#define COINES_E_INVALID_BLE_CONFIG                -41

/*! COINES_SDK error code - Serial com port in use */
#define COINES_E_SCOM_PORT_IN_USE                  -42

/*! COINES_SDK error code - incompatible firmware for the selected comm type */
#define COINES_E_INCOMPATIBLE_FIRMWARE             -43

/**********************************************************************************/
/* Function declarations */
/**********************************************************************************/

/*!
 * @brief This API is used to COINES_SDK error codes to error strings
 *
 * @param[in]  error_code     : error_code
 *
 * @return Error string for the given error code
 */
const char *get_coines_error_str(int16_t error_code);

#ifdef __cplusplus
}
#endif

#endif
