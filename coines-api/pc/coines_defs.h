/**
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    coines_defs.h
 * @brief This layer provides declarations of COINES_SDK layer/module
 *
 */

/*!
 * @addtogroup coines_api
 * @{*/

#ifndef COINES_DEFS_H_
#define COINES_DEFS_H_

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! communication interface debug enable */
#define COINES_DEBUG_ENABLED                               0

/*APP 2.0  device information*/
/*! USB COINES_SDK Development desktop device vendor ID*/
#define COINES_DEVICE_DD_VENDOR                            INT16_C(0x152A)

/*! USB COINES_SDK Development desktop device product ID*/
#define COINES_DEVICE_DD_PRODUCT                           INT16_C(0x80C0)

/*! Robert Bosch GmbH USB vendor ID*/
#define COINES_ROBERT_BOSCH_VID                            INT16_C(0x108C)

/*! USB COINES_SDK APP3.0 device product ID (WinUSB) */
#define COINES_WINUSB_APP30_PID                            INT16_C(0xAB30)

/*!
 * @brief COINES_SDK command id codes
 */

/*! Command Identifier */
#define  COINES_CMD_ID                                     UINT8_C(0xAA)

/*! Read response overhead size */
#define  COINES_OVERHEADS_SIZE                             UINT8_C(6)
#define  COINES_BYTEPOS_PACKET_SIZE                        UINT8_C(1)

/*! Position where the status(success/failure) is stored in the response */
#define  COINES_RESPONSE_STATUS_POSITION                   UINT8_C(3)

/*!  Position of the command identifier in the COINES_SDK protocol command */
#define  COINES_COMMAND_ID_POSITION                        UINT8_C(2)

/*! position of the packet index for polling streaming*/
#define  COINES_PACKET_INDEX_POSITION                      UINT8_C(3)

/*!
 * @brief COINES_SDK AND Development desktop firmware asynchronous command response id codes
 */
#define  COINES_RSPID_POLLING_STREAMING_DATA               UINT8_C(0x87)
#define  COINES_RSPID_INT_STREAMING_DATA                   UINT8_C(0x8A)

/*!
 * @brief Development desktop firmware command id codes
 */

/* Set command identifier*/
#define  COINES_DD_SET                                     UINT8_C(0x01)

/*! Get command identifier */
#define  COINES_DD_GET                                     UINT8_C(0x02)

/*! General streaming settings command identifier */
#define  COINES_DD_GENERAL_STREAMING_SETTINGS              UINT8_C(0x03)

/*! Sensor streaming settings in polling mode command identifier */
#define  COINES_DD_POLLING_STREAMING_SETTINGS              UINT8_C(0x0F)

/*! Polling streaming response command identifier */
#define  COINES_DD_POLLING_RESPONSE_IDENTIFIER             UINT8_C(0x07)

/*! Read/Write response data start position*/
#define  COINES_DD_READ_WRITE_DATA_START_POSITION          UINT8_C(0x0B)

/*! Read/Write response data start position for SPI 16 bit */
#define  COINES_SPI_16BIT_READ_WRITE_DATA_START_POSITION   UINT8_C(0x0C)

/*! DD2.0 Response Identifier */
#define  COINES_DD_RESP_ID                                 UINT8_C(0xAA)

/*! READ operation RESPONSE ID */
#define  COINES_READ_RESP_ID                               UINT8_C(0x42)

/*! Over head carriage return size response position */
#define  COINES_DD_CARRIAGE_SIZE                           UINT8_C(0x02)

/*!  Position of the protocol identifier(0xAA/0x55) */
#define COINES_IDENTIFIER_POSITION                         UINT8_C(0)

/*! Position where length of command is stored */
#define COINES_COMMAND_LENGTH_POSITION                     UINT8_C(1)

/*!  Command id response position */
#define  COINES_DD_COMMAND_ID_RESPONSE_POSITION            UINT8_C(0x04)

/*!  Command Feature position */
#define  COINES_DD_FEATURE_POSITION                        UINT8_C(0x05)

/*! response overhead size*/
#define  COINES_DD_OVERHEAD_SIZE                           UINT8_C(0x0D)

/*! response size position*/
#define  COINES_DD_RESPONSE_SIZE_POSITION                  UINT8_C(1)

/*!  status response position */
#define  COINES_DD_STATUS_RESPONSE_POSITION                UINT8_C(3)

/*!  Read command packet number position */
#define COINES_READ_RESPONSE_PKT_POSITION                  UINT8_C(4)

/*! dd response identifier position*/
#define  COINES_DD_RESPONSE_IDENTIFIER_POSITION            UINT8_C(5)

/*! Response identifier position for extended payload(> 64 bytes) */
#define  COINES_EXTENDED_READ_RESPONSE_ID                  UINT8_C(0x43)

/*! Response identifier position for getting Num ber of bytes (MSB) in payload */
#define COINES_BYTEPOS_LEN_MSB                             UINT8_C(0x08)

/*! Response identifier position for getting Number of bytes (LSB) in payload */
#define COINES_BYTEPOS_LEN_LSB                             UINT8_C(0x09)

/*!  16bit Packet length calculation by combining 8 bit MSB and LSB length */
#define COINES_CALC_PACKET_LENGTH(MSB, LSB)                ((MSB << 8) | (LSB))

/*! @brief DD command IDs*/
#define COINES_CMDID_IODIR                                 UINT8_C(0x04)
#define COINES_CMDID_IO_VALUE                              UINT8_C(0x05)
#define COINES_CMDID_SPIMODE                               UINT8_C(0x06)
#define COINES_CMDID_SPISPEED                              UINT8_C(0x08)
#define COINES_CMDID_I2CSPEED                              UINT8_C(0x09)
#define COINES_CMDID_SPICHIPSELECT                         UINT8_C(0x0A)
#define COINES_CMDID_I2CWRITEANDREAD                       UINT8_C(0x0F)
#define COINES_CMDID_SPIWRITEANDREAD                       UINT8_C(0x10)
#define COINES_CMDID_INTERFACE                             UINT8_C(0x11)
#define COINES_CMDID_SENSORSPI_CONFIGURATION               UINT8_C(0x12)
#define COINES_CMDID_SENSORI2C_CONFIGURATION               UINT8_C(0x13)
#define COINES_CMDID_SHUTTLEBOARD_VDD_VDDIO_CONFIGURATION  UINT8_C(0x14)
#define COINES_CMDID_MULTIO_CONFIGURATION                  UINT8_C(0x15)
#define COINES_CMDID_SENSORWRITEANDREAD                    UINT8_C(0x16)
#define COINES_CMDID_BOARDMODE                             UINT8_C(0x18)
#define COINES_CMDID_SPISETTINGS                           UINT8_C(0x19)
#define COINES_CMDID_DELAY                                 UINT8_C(0x1A)
#define COINES_CMDID_BOARDINFORMATION                      UINT8_C(0x1F)
#define COINES_CMDID_SENSOR_WRITE_DELAYREAD                UINT8_C(0x22)
#define COINES_CMDID_STARTSTOP_RESPONSE                    UINT8_C(0x23)
#define COINES_CMDID_THIRD_PARTY_WRITEANDREAD              UINT8_C(0x28)
#define COINES_CMDID_TIMER_CFG_CMD_ID                      UINT8_C(0x29)
#define COINES_CMDID_UNKNOWN_INSTRUCTION                   UINT8_C(0x2B)
#define COINES_CMDID_GENERALSETTINGS                       UINT8_C(0x43)
#define COINES_CMDID_STARTSTOP_POLLING_RESPONSE            UINT8_C(0x46)
#define COINES_CMDID_APP_SWITCH                            UINT8_C(0x30)

#define COINES_CMDID_16BIT_SPIWRITEANDREAD                 UINT8_C(0x33)

#define COINES_CMDIDEXT_STARTSTOP_STREAM_POLLING           UINT8_C(0x06)
#define COINES_CMDIDEXT_STARTSTOP_STREAM_INT               UINT8_C(0x0A)

#define COINES_CMDIDEXT_STREAM_INT                         UINT8_C(0x0E)
#define COINES_CMDIDEXT_STREAM_POLLING                     UINT8_C(0x0F)

#define COINES_STREAM_ONE_SAMPLE                           UINT8_C(0x01)
#define COINES_STREAM_TWO_SAMPLES                          UINT8_C(0x02)
#define COINES_STREAM_INFINITE_SAMPLES                     UINT8_C(0xFF)

/*! Type of communication interface */
#define COINES_BOARD_API_IF_SPI                            UINT8_C(0)
#define COINES_BOARD_API_IF_I2C                            UINT8_C(1)

/*! Invalid data from firmware */
#define COINES_INVALID_DATA                                UINT8_C(255)

/*! Size of the data packets*/
#define COINES_PACKET_SIZE                                 UINT8_C(64)

/*! Size of the data packets*/
#define COINES_PACKET_PAYLOAD                              UINT8_C(46)

/*! COINES_SDK specific logical values */
#define COINES_TRUE                                        UINT8_C(1)
#define COINES_FALSE                                       UINT8_C(0)

/*! Maximum sensor ID supported during streaming */
#define COINES_SENSOR_ID_MAX_COUNT                         UINT8_C(5)

/*! Streaming modes in DD2.0 */
#define COINES_DD_BURST_MODE                               UINT8_C(1)
#define COINES_DD_NORMAL_MODE                              UINT8_C(2)
#define COINES_DD_BURST_MODE_CONTINUOUS                    UINT8_C(3)

/*! COINES_SDK interface SDO logic levels */
#define COINES_INTF_SDO_LOW                                0
#define COINES_INTF_SDO_HIGH                               1

/*! Mini shuttle pin ID for Multi-IO config */
#define COINES_MINI_SHUTTLE_PIN_ID                         (1 << 15)

/*! COINES_SDK interrupt timeout */
#define COINES_INTERRUPT_TIMEOUT                           0xF0F0

/*! Debug time printf macros */
#if (COINES_DEBUG_ENABLED == 1)
#include <stdio.h>
#define DEBUG_PRINT(...)                                   printf(__VA_ARGS__)
#define DEBUG_PRINT_BUF(arr, size)                         { uint32_t dbg_idx; \
                                                             for (dbg_idx = 0; dbg_idx < (size); dbg_idx++) { printf( \
                                                                                                                  "%x :", \
                                                                                                                  (arr)[ \
                                                                                                                      dbg_idx]); \
                                                             } \
}
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_BUF(arr, size)
#endif

/**********************************************************************************/
/* data structure declarations  */

/*!
 * @brief COINES_SDK board type
 */
typedef enum
{
    COINES_BOARD_DD = 0xC0
    /*< DD */
} coines_board_t;

/*!
 * @brief COINES_SDK data buffer
 */
typedef struct
{
    uint8_t buffer[COINES_DATA_BUF_SIZE]; /*< Data buffer */
    uint32_t buffer_size; /*< buffer size */
    uint8_t error; /*< error code */
    coines_board_t board_type; /*< board type */
} coines_command_t;

/*!
 * @brief COINES_SDK response buffer
 */
typedef struct
{
    uint8_t buffer[COINES_DATA_BUF_SIZE]; /*< Data buffer */
    uint32_t buffer_size; /*< buffer size */
} coines_rsp_buffer_t;

/*!
 * @brief COINES_SDK stream response buffer
 */
typedef struct
{
    uint8_t buffer[COINES_STREAM_RSP_BUF_SIZE]; /*< Data buffer */
    uint32_t buffer_size; /*< buffer size */
} coines_stream_rsp_buffer_t;

#ifdef __cplusplus
}
#endif

#endif /* COINES_DEFS_H_ */
/** @}*/
