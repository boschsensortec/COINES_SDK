/**
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 * BSD-3-Clause
 * Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    api.h
 * @brief   This file contains api layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef API_H_
#define API_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
#define ROBERT_BOSCH_USB_VID                   (0x108C)
#define ARDUINO_USB_VID                        (0x2341)

#define BST_APP31_CDC_USB_PID                  (0xAB38)
#define BST_APP30_CDC_USB_PID                  (0xAB3C)
#define ARDUINO_NICLA_USB_PID                  (0x0060)
#define BST_HEAR3X_CDC_USB_PID                 (0x4B3C)

/*! Number of boards compactible with COINES_SDK */
#define COINES_COMPACTIBLE_BOARDS              5

/*! USB COINES_SDK Development desktop device vendor ID*/
#define COINES_DEVICE_DD_VENDOR                (0x152A)

/*! USB COINES_SDK Development desktop device product ID*/
#define COINES_DEVICE_DD_PRODUCT               (0x80C0)

#define DEFAULT_BAUD_RATE                      115200
#define NICLA_BAUD_RATE                        9600

#ifndef COINES_BUFFER_SIZE
#define COINES_BUFFER_SIZE                     3084
#endif

#define COINES_STREAM_CONFIG_BUFF_SIZE         (50)
#define COINES_POLL_STREAM_COMMON_PAYLOAD_LEN  (4)

#define PAYLOAD_POS                            (0)


/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * @brief command id
 */
enum coines_cmds {
    COINES_CMD_ID_ECHO,
    COINES_CMD_ID_GET_BOARD_INFO,
    COINES_CMD_ID_SET_PIN,
    COINES_CMD_ID_GET_PIN,
    COINES_CMD_ID_SET_VDD_VDDIO,
    COINES_CMD_ID_SPI_CONFIG,
    COINES_CMD_ID_SPI_DECONFIG,
    COINES_CMD_ID_SPI_WORD_CONFIG,
    COINES_CMD_ID_SPI_WRITE_REG_16,
    COINES_CMD_ID_SPI_WRITE_REG,
    COINES_CMD_ID_SPI_READ_REG_16,
    COINES_CMD_ID_SPI_READ_REG,
    COINES_CMD_ID_I2C_CONFIG,
    COINES_CMD_ID_I2C_DECONFIG,
    COINES_CMD_ID_I2C_WRITE_REG,
    COINES_CMD_ID_I2C_READ_REG,
    COINES_CMD_ID_I2C_WRITE,
    COINES_CMD_ID_I2C_READ,
    COINES_CMD_ID_GET_TEMP,
    COINES_CMD_ID_GET_BATTERY,
    COINES_CMD_ID_RESET,
    COINES_CMD_ID_SET_LED,
    COINES_CMD_ID_POLL_STREAM_COMMON,
    COINES_CMD_ID_POLL_STREAM_CONFIG,
    COINES_CMD_ID_INT_STREAM_CONFIG,
    COINES_CMD_ID_FIFO_STREAM_CONFIG,
    COINES_CMD_ID_STREAM_START_STOP,
    COINES_READ_SENSOR_DATA,
    COINES_CMD_ID_SOFT_RESET,
    COINES_CMD_ID_SHUTTLE_EEPROM_WRITE,
    COINES_CMD_ID_SHUTTLE_EEPROM_READ,
    COINES_CMD_ID_DMA_INT_STREAM_CONFIG,
    COINES_CMD_ID_I2C_WRITE_REG_16,
    COINES_CMD_ID_I2C_READ_REG_16,
    COINES_N_CMDS
};

/*!
 * @brief structure to store the streaming sensor info
 */
struct coines_stream_sensor_info
{
    uint16_t no_of_sensors_enabled; /**< Number of sensors enabled */
    uint16_t sensors_byte_count[COINES_MAX_SENSOR_COUNT]; /**< Sensor byte count */
};

/*!
 * @brief structure to store the streaming settings
 */
struct coines_streaming_settings
{
    uint8_t sensor_id; /*< streaming sensor id */
    struct coines_streaming_config stream_config; /*< streaming config */
    struct coines_streaming_blocks data_blocks; /*< streaming data blocks */
};

/**********************************************************************************/
/* functions */
/**********************************************************************************/

#endif /* API_H_ */
