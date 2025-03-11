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
 *  @file   bridge_encoder.c
 *  @brief  This module defines encoder APIs to be used by the api layer
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "bridge_encoder.h"
#include "protocol.h"
#include "interface.h"
#include "coines.h"
#include "error_handling.h"

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/
static uint8_t cmd_packet[BRIDGE_SERIAL_PACKET_SIZE] = { 0 };

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int16_t encoder(enum coines_comm_intf interface_type,
                       uint8_t command,
                       uint8_t *payload_header,
                       uint16_t header_length,
                       uint8_t *payload_body,
                       uint16_t body_length);

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to format and send given packets
 *
 */
static int16_t encoder(enum coines_comm_intf interface_type,
                       uint8_t command,
                       uint8_t *payload_header,
                       uint16_t header_length,
                       uint8_t *payload_body,
                       uint16_t body_length)
{
    int16_t ret;
    uint16_t bytes_to_write = 0;
    uint16_t packet_size_limit = 0;
    uint16_t packet_size;

    memset(cmd_packet, 0x00, BRIDGE_SERIAL_PACKET_SIZE);

    if ((header_length != 0) && (payload_header == NULL))
    {
        return get_coines_error_mapping(ENCODER_BRIDGE_RESP_NULL_PTR);
    }

    if (interface_type == COINES_COMM_INTF_USB)
    {
        packet_size_limit = BRIDGE_SERIAL_PACKET_SIZE;
    }
    else if (interface_type == COINES_COMM_INTF_BLE)
    {
        packet_size_limit = BRIDGE_BLE_PACKET_SIZE;
    }

    packet_size = header_length + body_length + 4;

    cmd_packet[BRIDGE_PROTO_HEADER_POS] = BRIDGE_CMD_HEADER;
    memcpy(&cmd_packet[BRIDGE_PROTO_LENGTH_POS], &packet_size, 2);
    cmd_packet[BRIDGE_PROTO_CMD_POS] = command;
    if (header_length != 0)
    {
        memcpy(&cmd_packet[BRIDGE_PROTO_PAYLOAD_POS], payload_header, header_length);
        if (payload_body && body_length)
        {
            memcpy(&cmd_packet[BRIDGE_PROTO_PAYLOAD_POS + header_length], payload_body, body_length);
        }
    }

    for (uint16_t pi = 0; pi < packet_size; pi += bytes_to_write)
    {
        if (packet_size < packet_size_limit)
        {
            bytes_to_write = packet_size;
        }
        else if ((packet_size - pi) < packet_size_limit)
        {
            bytes_to_write = packet_size - pi;
        }
        else
        {
            bytes_to_write = packet_size_limit;
        }

        ret = interface_send_packet(interface_type, &cmd_packet[pi], bytes_to_write);
        if (ret != COINES_SUCCESS)
        {
            return ret;
        }

    }

    return COINES_SUCCESS;

}

/*!
 * @brief This API is used to format and send multiple packets
 *
 */
int16_t encode_multi_packet(enum coines_comm_intf interface_type,
                            uint8_t command,
                            uint8_t *payload_header,
                            uint16_t header_length,
                            uint8_t *payload_body,
                            uint16_t body_length)
{
    return encoder(interface_type, command, payload_header, header_length, payload_body, body_length);
}

/*!
 * @brief This API is used to format and send a packet
 *
 */
int16_t encode_packet(enum coines_comm_intf interface_type,
                      uint8_t command,
                      uint8_t *payload_header,
                      uint16_t header_length)
{
    return encoder(interface_type, command, payload_header, header_length, NULL, 0);
}
