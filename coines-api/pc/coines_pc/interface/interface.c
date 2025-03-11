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
 *  @file   interface.c
 *  @brief  This module defines interface APIs to be used by the protocol layer
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "interface.h"
#include "platform.h"
#include "coines.h"
#include "error_handling.h"
#include "circular_buffer.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/
int16_t com_read_status = COINES_SUCCESS;

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*********************************************************************/
/* functions */
/*********************************************************************/

/**
 * @brief This function is used to open the specified interface.
 *
 * @param interface_type The type of interface to open.
 * @param config The configuration for the interface.
 * @return int16_t Returns the status of the operation.
 */
int16_t interface_open(enum coines_comm_intf interface_type, void *config)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:

            return platform_open_serial(config);
        case COINES_COMM_INTF_BLE:

            return platform_open_ble(config);
        default:
            break;
    }

    return INTERFACE_FAILED;
}

/**
 * @brief This function is used to close the specified interface.
 *
 * @param interface_type The type of interface to close.
 * @return int16_t Returns the status of the operation.
 */
int16_t interface_close(enum coines_comm_intf interface_type)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:

            return platform_close_serial();

        case COINES_COMM_INTF_BLE:

            return platform_close_ble();

        default:
            break;
    }

    return INTERFACE_FAILED;

}

/**
 * @brief This function is used to send a packet over the specified interface.
 *
 * @param interface_type The type of interface to send the packet over.
 * @param data The data to send.
 * @param length The length of the data to send.
 * @return int16_t Returns the status of the operation.
 */
int16_t interface_send_packet(enum coines_comm_intf interface_type, uint8_t *data, uint16_t length)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:

            return platform_send_serial(data, length);

        case COINES_COMM_INTF_BLE:

            return platform_send_ble(data, length);

        default:
            break;
    }

    return INTERFACE_TYPE_NOT_SUPPORTED;
}

/**
 * @brief This function is used to send a packet over the specified interface.
 *
 * @param interface_type The type of interface to send the packet over.
 * @param data  Pointer to the buffer to store the data
 * @param length Length of the buffer
 * @return uint16_t number of bytes read
 */
uint16_t interface_receive_packet(enum coines_comm_intf interface_type, uint8_t *data, uint16_t length)
{
    uint32_t n_bytes_read = 0;

    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:
            com_read_status = platform_receive_serial(data, (uint32_t)length, &n_bytes_read);
            break;
        case COINES_COMM_INTF_BLE:
            com_read_status = platform_receive_ble(data, (uint32_t)length, &n_bytes_read);
            break;
        default:
            break;
    }

    return (uint16_t)n_bytes_read;
}

/*!
 * @brief This API is used to flush the buffer
 *
 */
void interface_flush(enum coines_comm_intf interface_type)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:
            platform_flush_serial();
            break;
        case COINES_COMM_INTF_BLE:
            break;
        default:
            break;
    }
}

/*!
 * @brief This API is used to scan BLE devices
 *
 * @param[out] ble_info            : Array of structure containing found BLE peripherals' info like Address and Identifier
 * @param[out] peripheral_count    : The number of found BLE peripherals
 * @param[in]  scan_timeout_ms     : Timeout for BLE scan
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Fail.
 */
int16_t interface_ble_scan(void *ble_info, uint8_t *peripheral_count, uint32_t scan_timeout_ms)
{
    return platform_scan_ble(ble_info, peripheral_count, scan_timeout_ms);
}

/**
 * @brief Controls continuous read operation for the specified communication interface.
 *
 * This function starts or stops the continuous read operation based on the interface type and the start/stop flag.
 *
 * @param[in] interface_type The type of communication interface (e.g., USB, BLE).
 * @param[in] start_stop Flag to start (non-zero) or stop (zero) the continuous read operation.
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Fail.
 */
int16_t interface_control_continuous_read(enum coines_comm_intf interface_type, uint8_t start_stop)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:
            if (start_stop)
            {
                return platform_serial_read_thread_start();
            }
            else
            {
                return platform_serial_read_thread_stop();
            }

        case COINES_COMM_INTF_BLE:
            if (start_stop)
            {
                platform_ble_attach_notify_cb(true);
            }
            else
            {
                platform_ble_attach_notify_cb(false);
            }

            return COINES_SUCCESS;
        default:
            break;
    }

    return INTERFACE_TYPE_NOT_SUPPORTED;
}

/**
 * @brief Checks the connection status of the specified communication interface.
 *
 * This function checks the status of the specified communication interface to determine if it is connected or not.
 * 
 * @param[in] interface_type The type of communication interface (e.g., USB, BLE).
 *
 * @return Result of API execution status.
 * @retval 0 -> Success.
 * @retval Any non zero value -> Fail.
 */
int16_t interface_check_connection_status(enum coines_comm_intf interface_type)
{
    switch (interface_type)
    {
        case COINES_COMM_INTF_USB:
            return platform_check_serial_port_status();

        case COINES_COMM_INTF_BLE:
            return platform_check_ble_status();

        default:
            break;
    }

    return INTERFACE_TYPE_NOT_SUPPORTED;
}

