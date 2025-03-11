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
 * @file    protocol.h
 * @brief   This file contains protocol layer function prototypes, variable declarations and Macro definitions
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdint.h>

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "coines.h"
#include "bridge.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
#ifdef COINES_BRIDGE_PROTOCOL
    #define PROTO_PAYLOAD_POS          BRIDGE_PROTO_PAYLOAD_POS
    #define PROTO_HEADER_POS           BRIDGE_PROTO_HEADER_POS
    #define PROTO_LENGTH_POS           BRIDGE_PROTO_LENGTH_POS
    #define PROTO_CMD_POS              BRIDGE_PROTO_CMD_POS
    #define PROTO_RESP_OK_HEADER       BRIDGE_RESP_OK_HEADER
    #define PROTO_LENGTH_BYTES         BRIDGE_PROTO_LENGTH_BYTES
    #define PROTO_WRITE_CMD            BRIDGE_WRITE_CMD
    #define PROTO_STREAM_STOP_RESP_ARR BRIDGE_STREAM_STOP_RESP_ARR
#endif

#define PROTO_STOP_RESP_LEN   5
/**********************************************************************************/
/* typedef definitions */
/**********************************************************************************/
typedef struct
{
    enum coines_comm_intf interface_type;
} thread_params_t;

/**********************************************************************************/
/* function prototype declarations*/
/**********************************************************************************/

/**
 * @brief This API is used to encodes a packet for communication using the specified communication interface, command, payload, and length.
 *
 * @param interface_type The communication interface to use for encoding the packet.
 * @param command The command to include in the packet.
 * @param payload The payload data to include in the packet.
 * @param length The length of the payload data.
 * @return Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_encode_packet(enum coines_comm_intf interface_type, uint8_t command, uint8_t *payload,
                               uint16_t length);

/**
 * @brief This API is used to decodes a packet received over the specified communication interface.
 *        It takes the command byte and the response buffer as input, and updates the response length accordingly.
 *
 * @param interface_type The communication interface over which the packet was received.
 * @param command The command byte of the received packet.
 * @param resp_buffer The buffer to store the decoded response.
 * @param resp_length A pointer to the variable that will hold the length of the decoded response.
 *
 * @return Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_decode_packet(enum coines_comm_intf interface_type,
                               uint8_t command,
                               uint8_t *resp_buffer,
                               uint16_t *resp_length);

/**
 * @brief This API is used to encodes a packet for communication over the specified interface.
 *
 * @param[in] interface_type The communication interface to use.
 * @param[in] command The command to include in the packet.
 * @param[in] payload The payload to include in the packet.
 * @param[in] length The length of the payload.
 * @param[in] payload Register data to be written
 * @param[in] length The length of the register data.
 *
 * @return Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_encode_multi_packet(enum coines_comm_intf interface_type,
                                     uint8_t command,
                                     uint8_t *payload,
                                     uint16_t length,
                                     uint8_t *reg_data,
                                     uint16_t reg_data_length);

/**
 * @brief This API is used to starts or stops the continuous packet decoding operation based on the interface type and the start/stop flag.
 *
 * @param[in] interface_type The type of communication interface (e.g., USB, BLE).
 * @param[in] start_stop Flag to start (non-zero) or stop (zero) the continuous packet decoding operation.
 *
 * @return Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_decode_continuous_packets(enum coines_comm_intf interface_type, uint8_t start_stop);

/**
 * @brief This API is used to start the protocol decode thread for the specified communication interface.
 *
 * @param[in] interface_type The type of communication interface (e.g., USB, BLE).
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_decode_thread_start(enum coines_comm_intf interface_type);

/**
 * @brief This API is used to stop the protocol decode thread.
 *
 * @return int16_t Returns 0 if successful, or a negative error code if failed.
 */
int16_t protocol_decode_thread_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H_ */
