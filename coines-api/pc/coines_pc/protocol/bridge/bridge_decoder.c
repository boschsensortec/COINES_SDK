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
 *  @file   bridge_decoder.c
 *  @brief  This module defines decoder APIs to be used by the api layer
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
#include "bridge_decoder.h"
#include "mqueue.h"
#include "protocol.h"
#include "interface.h"
#include "coines.h"
#include "error_handling.h"
#include "mqueue.h"
#include "streaming.h"
#include "circular_buffer.h"
#include "api.h"


/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* extern variables */
/*********************************************************************/
/*! Variable to hold write status */
extern int8_t com_read_status;
extern circular_buffer_t stream_cbuf;
extern volatile bool is_decode_thread_running;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static int16_t receive_resp_packets(enum coines_comm_intf interface_type, uint8_t *resp_buffer, uint16_t length);
static void enqueue_payload_by_cmd(uint8_t *resp_buffer, uint16_t payload_length);

/*********************************************************************/
/* static functions */
/*********************************************************************/
static int16_t receive_resp_packets(enum coines_comm_intf interface_type, uint8_t *resp_buffer, uint16_t length)
{
    uint32_t start_time;
    uint32_t current_time;
    int16_t bytes_available;
    int16_t packet_idx = 0;

    start_time = coines_get_millis();
    do
    {
        bytes_available = (int16_t)interface_receive_packet(interface_type, &resp_buffer[packet_idx], length);
        if (com_read_status == COINES_SUCCESS)
        {
            if (bytes_available)
            {
                packet_idx += bytes_available;
            }
        }
        else
        {
            return com_read_status;
        }

        current_time = coines_get_millis();
        if ((current_time - start_time) > READ_TIMEOUT_MS)
        {
            return COINES_E_READ_TIMEOUT;
        }
    } while (packet_idx < length);

    return COINES_SUCCESS;
}

static void enqueue_payload_by_cmd(uint8_t *resp_buffer, uint16_t packet_length)
{
    uint8_t mqueue_idx = 0;
    uint8_t offset = 0;
    uint16_t payload_length = 0;
    
    if (resp_buffer[BRIDGE_PROTO_CMD_POS] == BRIDGE_PROTO_STREAM_RESP_CMD_ID)
    {
        mqueue_idx = resp_buffer[BRIDGE_PROTO_PAYLOAD_POS]; /* sensor id */
        offset = BRIDGE_PROTO_PAYLOAD_POS + 1;
        payload_length = (packet_length - BRIDGE_PROTO_PAYLOAD_POS) - 1;
    }
    else
    {
        payload_length = packet_length;
    }

    (void)mqueue_add_data(mqueue_idx, resp_buffer + offset, payload_length);

}

/*********************************************************************/
/* functions */
/*********************************************************************/
int16_t decode_response_processor(uint8_t *resp_buffer, uint16_t packet_length, uint8_t command, uint16_t *resp_length)
{
    uint16_t payload_length;
    uint8_t *payload_start;

    if (resp_buffer[BRIDGE_PROTO_HEADER_POS] == BRIDGE_RESP_OK_HEADER)
    {
        payload_length = packet_length - BRIDGE_PROTO_PAYLOAD_POS;

        *resp_length = payload_length;

        /* Create a pointer to the start of the payload in resp_buffer */
        payload_start = &resp_buffer[BRIDGE_PROTO_PAYLOAD_POS];

        if (resp_buffer[BRIDGE_PROTO_CMD_POS] != command)
        {
            return DECODER_BRIDGE_RESP_CMD_MISMATCH;
        }

        /* Copy the payload to the start of resp_buffer */
        memcpy(resp_buffer, payload_start, payload_length);

    }
    else if (resp_buffer[BRIDGE_PROTO_HEADER_POS] == BRIDGE_RESP_NOK_HEADER)
    {
        if (resp_buffer[BRIDGE_PROTO_PAYLOAD_POS] != COINES_SUCCESS)
        {
            return (int8_t)resp_buffer[BRIDGE_PROTO_PAYLOAD_POS];
        }
        else
        {
            return DECODER_BRIDGE_RESP_HEADER_NOK;
        }
    }
    else
    {
        return DECODER_BRIDGE_RESP_HEADER_UNKNOWN;
    }

    return COINES_SUCCESS;

}

/*!
 * @brief This API is used read and parse the received response
 *
 */
int16_t decode_packet(enum coines_comm_intf interface_type, uint8_t command, uint8_t *resp_buffer,
                      uint16_t *resp_length)
{
    int16_t ret;
    uint16_t packet_length = 0;
    uint16_t initial_read_len = 3;
    uint16_t payload_read_len;

    if (resp_buffer == NULL || resp_length == NULL)
    {
        return DECODER_BRIDGE_RESP_NULL_PTR;
    }

    // Read the initial 3 bytes of the response packet to get packet length
    ret = receive_resp_packets(interface_type, &resp_buffer[0], initial_read_len);

    if (ret != COINES_SUCCESS)
    {
        return ret;
    }

    // Extract the packet length from the response buffer
    memcpy(&packet_length, &resp_buffer[BRIDGE_PROTO_LENGTH_POS], BRIDGE_PROTO_LENGTH_BYTES);

    if((packet_length > COINES_BUFFER_SIZE) || (packet_length <= initial_read_len)){
       return DECODER_BRIDGE_INVALID_PACKET_LEN;
    }

    // Calculate the remaining payload length
    payload_read_len =  (uint16_t)(packet_length - initial_read_len);

    // Read the remaining payload
    ret = receive_resp_packets(interface_type, &resp_buffer[initial_read_len], payload_read_len);

    if (ret != COINES_SUCCESS)
    {
        return ret;
    }

    ret = decode_response_processor(resp_buffer, packet_length, command, resp_length);

    return ret;
}

int16_t decode_and_enqueue_packet(void)
{
    int16_t ret;
    uint16_t packet_length = 0;
    uint16_t initial_read_len = 3;
    uint8_t resp_buffer[MQUEUE_PACKET_SIZE];

    memset(resp_buffer, 0, sizeof(resp_buffer));

    /* Read the initial 3 bytes of the response packet */
    ret = circular_buffer_get(&stream_cbuf, &resp_buffer[0], initial_read_len);

    if (ret != CIRCULAR_BUFFER_SUCCESS)
    {
        return ret;
    }

    memcpy(&packet_length, &resp_buffer[BRIDGE_PROTO_LENGTH_POS], BRIDGE_PROTO_LENGTH_BYTES);

    if (packet_length)
    {
        do
        {
            ret = circular_buffer_get(&stream_cbuf, &resp_buffer[initial_read_len], (uint16_t)(packet_length - initial_read_len));
            if (ret != COINES_SUCCESS)
            {
                /* Introduce a small delay to avoid busy-waiting */
                coines_delay_usec(10);
            }
        } while (ret != COINES_SUCCESS && is_decode_thread_running);

        if (resp_buffer[BRIDGE_PROTO_HEADER_POS] == BRIDGE_RESP_OK_HEADER)
        {
            enqueue_payload_by_cmd(resp_buffer, packet_length);
        }
    }

    return ret;
}
