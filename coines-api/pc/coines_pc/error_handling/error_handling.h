/*!
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
 * @file error_handling.h
 *
 * @brief This module is to map module error codes to coines error codes.
 *
 *
 */

#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* includes */
/**********************************************************************************/
#include "coines.h"
#include "circular_buffer.h"

/*********************************************************************/
/* Macro definitions */
/*********************************************************************/

#define BASE_OFFSET                                 -100

#define PLATFORM_SERIAL_DEV_NOT_FOUND               (BASE_OFFSET - 1)
#define PLATFORM_SERIAL_PORT_IN_USE                 (BASE_OFFSET - 2)
#define PLATFORM_SERIAL_PORT_NOT_OPEN               (BASE_OFFSET - 3)
#define PLATFORM_SERIAL_CONFIG_FAILED               (BASE_OFFSET - 4)
#define PLATFORM_SERIAL_READ_FAILED                 (BASE_OFFSET - 5)
#define PLATFORM_SERIAL_WRITE_FAILED                (BASE_OFFSET - 6)
#define PLATFORM_SERIAL_FLUSH_FAILED                (BASE_OFFSET - 7)
#define PLATFORM_SERIAL_PORT_NAME_NOT_FOUND         (BASE_OFFSET - 8)
#define PLATFORM_SERIAL_DEVICE_NOT_SERIAL           (BASE_OFFSET - 9)

#define PLATFORM_BLE_ADAPTOR_NOT_FOUND              (BASE_OFFSET - 11)
#define PLATFORM_BLE_CONNECT_FAILED                 (BASE_OFFSET - 12)
#define PLATFORM_BLE_WRITE_FAILED                   (BASE_OFFSET - 13)
#define PLATFORM_BLE_LIB_NOT_LOADED                 (BASE_OFFSET - 14)
#define PLATFORM_BLE_PERIPHERAL_NOT_FOUND           (BASE_OFFSET - 15)
#define PLATFORM_BLE_INVALID_COM_CONFIG             (BASE_OFFSET - 16)
#define PLATFORM_BLE_TX_NOTIFY_FAILED               (BASE_OFFSET - 17)
#define PLATFORM_BLE_ADAPTER_BLUETOOTH_NOT_ENABLED  (BASE_OFFSET - 18)
#define PLATFORM_BLE_DISCONNECT_FAILED              (BASE_OFFSET - 19)
#define PLATFORM_BLE_PERIPHERAL_NOT_CONNECTED       (BASE_OFFSET - 20)
#define PLATFORM_BLE_APP_BOARD_NOT_FOUND            (BASE_OFFSET - 21)

#define PLATFORM_THREAD_CREATE_FAILED               (BASE_OFFSET - 22)
#define PLATFORM_THREAD_START_FAILED                (BASE_OFFSET - 23)
#define PLATFORM_THREAD_STOP_FAILED                 (BASE_OFFSET - 24)
#define PLATFORM_BLE_NO_DATA_TO_READ                (BASE_OFFSET - 25)
#define PLATFORM_BLE_READ_FAILED                    (BASE_OFFSET - 26)

#define DECODER_BRIDGE_RESP_CMD_MISMATCH            (BASE_OFFSET - 31)
#define DECODER_BRIDGE_RESP_HEADER_NOK              (BASE_OFFSET - 32)
#define DECODER_BRIDGE_RESP_HEADER_UNKNOWN          (BASE_OFFSET - 33)
#define DECODER_BRIDGE_RESP_NULL_PTR                (BASE_OFFSET - 34)
#define DECODER_BRIDGE_INVALID_PACKET_LEN           (BASE_OFFSET - 35)

#define ENCODER_BRIDGE_RESP_NULL_PTR                (BASE_OFFSET - 41)

#define INTERFACE_FAILED                            (BASE_OFFSET - 51)
#define INTERFACE_TYPE_NOT_SUPPORTED                (BASE_OFFSET - 52)

/*********************************************************************/
/* Typedef definitions */
/**********************************************************************/
typedef struct
{
    int16_t coines_error_code;
    int16_t module_error_code;
} error_lookup;

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
int16_t get_coines_error_mapping(int16_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_HANDLING_H */
