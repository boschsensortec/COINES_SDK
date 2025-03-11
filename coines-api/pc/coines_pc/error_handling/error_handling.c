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
 * @file error_handling.c
 *
 * @brief This module is to map module error codes to coines error codes.
 *
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "error_handling.h"

/*********************************************************************/
/* Other file includes */
/*********************************************************************/
#include "coines.h"

/*********************************************************************/
/* static variables */
/*********************************************************************/

static const error_lookup error_lookup_list[] = {

    /* Serial com error codes */
    { COINES_E_DEVICE_NOT_FOUND, PLATFORM_SERIAL_DEV_NOT_FOUND },
    { COINES_E_SCOM_PORT_IN_USE, PLATFORM_SERIAL_PORT_IN_USE },
    { COINES_E_UNABLE_OPEN_DEVICE, PLATFORM_SERIAL_PORT_NOT_OPEN },
    { COINES_E_SCOM_INVALID_CONFIG, PLATFORM_SERIAL_CONFIG_FAILED },
    { COINES_E_SERIAL_COMM_FAILED, PLATFORM_SERIAL_READ_FAILED },
    { COINES_E_SERIAL_COMM_FAILED, PLATFORM_SERIAL_WRITE_FAILED },
    { COINES_E_SERIAL_COMM_FAILED, PLATFORM_SERIAL_FLUSH_FAILED },
    { COINES_E_SERIAL_COMM_FAILED, PLATFORM_SERIAL_PORT_NAME_NOT_FOUND },
    { COINES_E_SERIAL_COMM_FAILED, PLATFORM_SERIAL_DEVICE_NOT_SERIAL },

    /* BLE error codes */
    { COINES_E_BLE_INVALID_CONFIG, PLATFORM_BLE_INVALID_COM_CONFIG },
    { COINES_E_BLE_ADAPTOR_NOT_FOUND, PLATFORM_BLE_ADAPTOR_NOT_FOUND },
    { COINES_E_BLE_ADAPTER_BLUETOOTH_NOT_ENABLED, PLATFORM_BLE_ADAPTER_BLUETOOTH_NOT_ENABLED },
    { COINES_E_BLE_PERIPHERAL_NOT_FOUND, PLATFORM_BLE_PERIPHERAL_NOT_FOUND },
    { COINES_E_BLE_LIBRARY_NOT_LOADED, PLATFORM_BLE_LIB_NOT_LOADED },
    { COINES_E_BLE_APP_BOARD_NOT_FOUND, PLATFORM_BLE_APP_BOARD_NOT_FOUND },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_CONNECT_FAILED }, { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_WRITE_FAILED },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_TX_NOTIFY_FAILED },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_DISCONNECT_FAILED },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_PERIPHERAL_NOT_CONNECTED },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_NO_DATA_TO_READ },
    { COINES_E_BLE_COMM_FAILED, PLATFORM_BLE_READ_FAILED },

    /* Pthread error codes */
    { COINES_E_PTHREAD_FAILED, PLATFORM_THREAD_CREATE_FAILED },
    { COINES_E_PTHREAD_FAILED, PLATFORM_THREAD_START_FAILED }, { COINES_E_PTHREAD_FAILED, PLATFORM_THREAD_STOP_FAILED },

    /* Decoder error codes */
    { COINES_E_DECODER_FAILED, DECODER_BRIDGE_RESP_CMD_MISMATCH },
    { COINES_E_DECODER_FAILED, DECODER_BRIDGE_RESP_HEADER_NOK },
    { COINES_E_DECODER_FAILED, DECODER_BRIDGE_RESP_HEADER_UNKNOWN },
    { COINES_E_DECODER_FAILED, DECODER_BRIDGE_RESP_NULL_PTR },
    { COINES_E_DECODER_FAILED, DECODER_BRIDGE_INVALID_PACKET_LEN },

    /* Encoder error codes */
    { COINES_E_ENCODER_FAILED, ENCODER_BRIDGE_RESP_NULL_PTR },

    /* Interface error codes */
    { COINES_E_INTERFACE_FAILED, INTERFACE_FAILED }, { COINES_E_INTERFACE_FAILED, INTERFACE_TYPE_NOT_SUPPORTED },
};

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used for COINES error code mapping
 */
int16_t get_coines_error_mapping(int16_t error_code)
{
    if( error_code > 0) 
    {
        return COINES_E_FAILURE;
    }

    if (error_code > BASE_OFFSET)
    {
        return error_code;
    }

    for (uint16_t i = 0; i < sizeof(error_lookup_list) / sizeof(error_lookup_list[0]); i++)
    {
        if (error_code == error_lookup_list[i].module_error_code)
        {
            return error_lookup_list[i].coines_error_code;
        }
    }

    return COINES_E_FAILURE;
}
