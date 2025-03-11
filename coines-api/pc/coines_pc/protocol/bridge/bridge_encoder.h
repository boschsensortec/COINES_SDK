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
 * @file    bridge_encoder.h
 * @brief   This file contains encoder layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef BRIDGE_ENCODER_H_
#define BRIDGE_ENCODER_H_

/**********************************************************************************/
/* system header files */
/**********************************************************************************/
#include <stdint.h>
#include "bridge.h"
#include "coines.h"

/**********************************************************************************/
/* header includes */
/**********************************************************************************/

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/**
 * @brief Encodes a packet for communication.
 *
 * This function encodes a packet for communication using the specified communication interface, command, payload, and length.
 *
 * @param interface_type The communication interface to use for encoding the packet.
 * @param command The command to include in the packet.
 * @param payload The payload data to include in the packet.
 * @param length The length of the payload data.
 * @return The encoded packet as a 16-bit signed integer.
 */
int16_t encode_packet(enum coines_comm_intf interface_type, uint8_t command, uint8_t *payload, uint16_t length);

int16_t encode_multi_packet(enum coines_comm_intf interface_type,
                            uint8_t command,
                            uint8_t *payload_header,
                            uint16_t header_length,
                            uint8_t *payload_body,
                            uint16_t body_length);

#endif /* BRIDGE_ENCODER_H_ */
