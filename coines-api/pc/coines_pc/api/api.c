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
 *  @file   api.c
 *  @brief  This module defines COINES_SDK APIs to be used by the user applications
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
#include "api.h"
#include "protocol.h"
#include "interface.h"
#include "coines.h"
#include "error_handling.h"
#include "circular_buffer.h"

/*********************************************************************/
/* local macro definitions */
/********************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/
uint8_t *resp_buffer;

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*! Variable to hold the communication interface type */
enum coines_comm_intf interface_type = COINES_COMM_INTF_USB;
struct coines_serial_com_config serial_com_config[COINES_COMPACTIBLE_BOARDS] = {
    { DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_APP30_CDC_USB_PID, NULL, COINES_BUFFER_SIZE },
    { DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_APP31_CDC_USB_PID, NULL, COINES_BUFFER_SIZE },
    { NICLA_BAUD_RATE,   ARDUINO_USB_VID, ARDUINO_NICLA_USB_PID, NULL, COINES_BUFFER_SIZE },    
    { DEFAULT_BAUD_RATE, ROBERT_BOSCH_USB_VID, BST_HEAR3X_CDC_USB_PID, NULL, COINES_BUFFER_SIZE },
};
/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static void resize_resp_buffer(uint16_t new_rx_buffer_size);
static int8_t verify_com_port_name(char *com_port_name);

/*********************************************************************/
/* static function */
/*********************************************************************/

/**
 * @brief Resizes the response buffer to the specified size.
 *
 * This function resizes the response buffer to the specified size if the new size is greater than the current buffer size.
 *
 * @param new_rx_buffer_size The new size of the response buffer.
 */
static void resize_resp_buffer(uint16_t new_rx_buffer_size)
{
    if (new_rx_buffer_size > COINES_BUFFER_SIZE)
    {
        /* resize resp_buffer to user input */
        resp_buffer = (uint8_t*)realloc(resp_buffer, new_rx_buffer_size * sizeof(uint16_t));
    }
}

static int8_t verify_com_port_name(char *com_port_name)
{
    #ifdef PLATFORM_WINDOWS
    if (strncmp(com_port_name, "COM", 3) != 0)
    #else
    if (strncmp(com_port_name, "/dev/", 5) != 0)
    #endif
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/*********************************************************************/
/* functions */
/*********************************************************************/

/**
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param intf_type The type of communication interface.
 * @param arg The argument for the communication interface.
 * @return int16_t Returns the status of the operation.
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    int16_t ret = 0;

    interface_type = intf_type;
    resp_buffer = (uint8_t*)malloc(COINES_BUFFER_SIZE * sizeof(uint8_t));

    switch (intf_type)
    {
        case COINES_COMM_INTF_USB:
            if (arg == NULL)
            {
                for (int i = 0; i < COINES_COMPACTIBLE_BOARDS; i++)
                {
                    ret = interface_open(intf_type, &serial_com_config[i]);
                    if (ret == COINES_SUCCESS)
                    {
                        break;
                    }
                }
            }
            else
            {
                struct coines_serial_com_config* serial_config = (struct coines_serial_com_config*) arg;
                ret = verify_com_port_name(serial_config->com_port_name);
                if (ret != 0)
                {
                    return COINES_E_SCOM_INVALID_CONFIG;
                }

                resize_resp_buffer(serial_config->rx_buffer_size);
                ret = interface_open(intf_type, arg);
            }

            break;

        case COINES_COMM_INTF_BLE:
            ret = interface_open(intf_type, arg);
            break;

        default:
            break;
    }

    return get_coines_error_mapping(ret);
}

/**
 * @brief This API is used to close the active communication (USB, COM, or BLE).
 *
 * @param intf_type The type of communication interface.
 * @param arg The argument for the communication interface.
 * @return int16_t Returns the status of the operation.
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    int16_t ret;
    (void)arg;

    ret = interface_close(intf_type);
    return get_coines_error_mapping(ret);
}

/**
 * @brief This API is used to get the board information.
 *
 * @param board_info Pointer to the structure to store the board information.
 * @return int16_t Returns the status of the operation.
 */
int16_t coines_get_board_info(struct coines_board_info *board_info)
{
    int16_t ret;
    uint16_t resp_length = 0;

    if (board_info == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_GET_BOARD_INFO, NULL, 0);

    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_GET_BOARD_INFO, resp_buffer, &resp_length);
    }

    if (ret == COINES_SUCCESS)
    {
        memcpy(&board_info->hardware_id, &resp_buffer[PAYLOAD_POS], 2);
        memcpy(&board_info->software_id, &resp_buffer[PAYLOAD_POS + 2], 2);
        memcpy(&board_info->board, &resp_buffer[PAYLOAD_POS + 4], 1);
        memcpy(&board_info->shuttle_id, &resp_buffer[PAYLOAD_POS + 5], 2);
    }

    return get_coines_error_mapping(ret);
}

/*!
 * @brief Calls BLE scan function
 */
int16_t coines_scan_ble_devices(struct ble_peripheral_info *ble_info,
                                uint8_t *peripheral_count,
                                uint32_t scan_timeout_ms)
{
    int16_t ret;

    ret = interface_ble_scan(ble_info, peripheral_count, scan_timeout_ms);

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to test the communication.
 *
 */
int16_t coines_echo_test(uint8_t *data, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_ECHO, data, length);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_ECHO, resp_buffer, &resp_length);
    }

    if ((ret == COINES_SUCCESS) && (memcmp(data, &resp_buffer[PAYLOAD_POS], length)))
    {
        return COINES_E_COMM_WRONG_RESPONSE;
    }

    return get_coines_error_mapping(ret);
}

/*!
 * @brief Get COINES_SDK library version
 *
 * @return pointer to version string
 */
const char *coines_get_version()
{
    return (const char *)COINES_VERSION;
}

int16_t coines_read_temp_data(float *temp_data)
{
    (void)temp_data;

    return COINES_E_FAILURE;
}

/*!
 * @brief This API is used to trigger the soft reset.
 *
 */
void coines_soft_reset(void)
{
    uint8_t payload[4];
    int16_t ret;
    uint16_t resp_length = 0;

    payload[0] = 0x00;
    payload[1] = 0x00;
    payload[2] = 0x0F;
    payload[3] = 0x00;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SOFT_RESET, payload, 4);
    if (ret == COINES_SUCCESS)
    {
        (void)protocol_decode_packet(interface_type, COINES_CMD_ID_SOFT_RESET, resp_buffer, &resp_length);
    }
}

/*!
 * @brief This API is used to flush the buffer
 *
 */
void coines_flush_intf(enum coines_comm_intf intf)
{
    interface_flush(intf);
}

/*!
 * @brief This API is used to write data over the specified interface
 *
 */
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    (void)interface_send_packet(intf, (uint8_t *) buffer, len);
}

/*!
 * @brief This API is used to read data over the specified interface
 *
 */
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    return interface_receive_packet(intf, (uint8_t *) buffer, len);
}
