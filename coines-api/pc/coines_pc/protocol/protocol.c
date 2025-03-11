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
 *
 *
 * @file    protocol.c
 * @brief   This file contains the implementation of the protocol layer functions.
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "protocol.h"
#include "bridge_encoder.h"
#include "bridge_decoder.h"
#include "coines.h"
#include "streaming.h"
#include "mqueue.h"
#include "interface.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/
extern uint8_t mqueue_full_mask;
uint8_t proto_stop_resp_arr[PROTO_STOP_RESP_LEN] = PROTO_STREAM_STOP_RESP_ARR;
/*********************************************************************/
/* static variables */
/*********************************************************************/

static pthread_t protocol_decode_thread;
volatile bool is_decode_thread_running = false;
static pthread_mutex_t decode_mutex = PTHREAD_MUTEX_INITIALIZER;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/
static void *protocol_decode_thread_func(void *);

/*********************************************************************/
/* static functions */
/*********************************************************************/
static void *protocol_decode_thread_func(void *arg)
{
    thread_params_t *params = (thread_params_t *)arg;

    while (is_decode_thread_running)
    {
        /* If any queue in the mqueue library is almost full, corresponding bit will be set in mqueue_full_mask.
            Here, we check the entire mqueue. If none of the queues are almost full, proceed to decode the packet.
            Otherwise, skip decoding the packet. */
        if(!(mqueue_full_mask & 0xFF))
        {
            /* Decode the packet from cicular buffer and add it to the mqueue */
            (void) decode_and_enqueue_packet();
        }
    }

    pthread_exit(NULL);
    free(params);

    return NULL;
}

int16_t protocol_decode_thread_start(enum coines_comm_intf interface_type)
{
    pthread_mutex_lock(&decode_mutex);
    is_decode_thread_running = true;
    pthread_mutex_unlock(&decode_mutex);

    thread_params_t params;

    params.interface_type = interface_type;

    return (int16_t)pthread_create(&protocol_decode_thread, NULL, protocol_decode_thread_func, (void *)&params);

}

int16_t protocol_decode_thread_stop(void)
{

    pthread_mutex_lock(&decode_mutex);
    is_decode_thread_running = false;
    pthread_mutex_unlock(&decode_mutex);

    return (int16_t)pthread_join(protocol_decode_thread, NULL);

}

/*********************************************************************/
/* functions */
/*********************************************************************/

/**
 * @brief This API is used to encodes a packet for communication using the specified communication interface, command, payload, and length.
 *
 */
int16_t protocol_encode_packet(enum coines_comm_intf interface_type, uint8_t command, uint8_t *payload, uint16_t length)
{
    return encode_packet(interface_type, command, payload, length);
}

/**
 * @brief This API is used to encodes a packet for communication over the specified interface.
 *
 */
int16_t protocol_encode_multi_packet(enum coines_comm_intf interface_type,
                                     uint8_t command,
                                     uint8_t *payload,
                                     uint16_t length,
                                     uint8_t *reg_data,
                                     uint16_t reg_data_length)
{
    return encode_multi_packet(interface_type, command, payload, length, reg_data, reg_data_length);
}

/**
 * @brief This API is used to decodes a packet received over the specified communication interface.
 *        It takes the command byte and the response buffer as input, and updates the response length accordingly.
 *
 */
int16_t protocol_decode_packet(enum coines_comm_intf interface_type,
                               uint8_t command,
                               uint8_t *resp_buffer,
                               uint16_t *resp_length)
{
    int16_t ret = COINES_SUCCESS;
    uint16_t length = 0;
    uint16_t packet_length = 0;
    uint32_t start_time;
    uint32_t current_time;

    pthread_mutex_lock(&decode_mutex);
    if (is_decode_thread_running)
    {
        start_time = coines_get_millis();
        do
        {
            ret = mqueue_read_rsp(resp_buffer, &length);
            if (ret == MQUEUE_SUCCESS)
            {
                memcpy(&packet_length, &resp_buffer[BRIDGE_PROTO_LENGTH_POS], BRIDGE_PROTO_LENGTH_BYTES);
                ret = decode_response_processor(resp_buffer, packet_length, command, resp_length);
            }

            coines_delay_usec(100);

            current_time = coines_get_millis();
            if (((current_time - start_time) > READ_TIMEOUT_MS) && (ret == MQUEUE_EMPTY))
            {
                pthread_mutex_unlock(&decode_mutex);

                return COINES_E_READ_TIMEOUT;
            }
        } while (ret != COINES_SUCCESS);
    }
    else
    {
        ret = decode_packet(interface_type, command, resp_buffer, resp_length);
    }

    pthread_mutex_unlock(&decode_mutex);

    return ret;
}

/**
 * @brief This API is used to starts or stops the continuous packet decoding operation based on the interface type and the start/stop flag.
 *
 */
int16_t protocol_decode_continuous_packets(enum coines_comm_intf interface_type, uint8_t start_stop)
{
    return interface_control_continuous_read(interface_type, start_stop);
}
