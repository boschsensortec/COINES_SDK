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
 * @file    bridge_decoder.h
 * @brief   This file contains decoder layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef BRIDGE_DECODER_H_
#define BRIDGE_DECODER_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "bridge.h"
#include "coines.h"

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
 * @brief Decodes a packet received from the COINES bridge.
 *
 * This function decodes a packet received from the COINES bridge based on the specified communication interface,
 * command, response buffer, and response length.
 *
 * @param interface_type The communication interface used for the COINES bridge.
 * @param command The command byte of the received packet.
 * @param resp_buffer The buffer to store the decoded response.
 * @param resp_length The length of the decoded response.
 * @return The status of the decoding process. Returns a signed 16-bit integer.
 */
int16_t decode_packet(enum coines_comm_intf interface_type, uint8_t command, uint8_t *resp_buffer,
                      uint16_t *resp_length);
int16_t decode_response_processor(uint8_t *resp_buffer, uint16_t packet_length, uint8_t command, uint16_t *resp_length);
int16_t decode_and_enqueue_packet(void);

#endif /* ENCODER_H_ */
