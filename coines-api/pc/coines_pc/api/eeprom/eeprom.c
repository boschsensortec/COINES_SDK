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
 * @file eeprom.c
 *
 * @brief This module provides APIs for EEPROM operations.
 *
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

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "eeprom.h"

/*********************************************************************/
/* header files */
/*********************************************************************/
#include "coines.h"
#include "api.h"
#include "protocol.h"
#include "error_handling.h"

/*********************************************************************/
/* local macro definitions */
/*********************************************************************/

/*********************************************************************/
/* constant definitions */
/*********************************************************************/

/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static variables */
/*********************************************************************/

/*********************************************************************/
/* extern variables */
/*********************************************************************/
extern enum coines_comm_intf interface_type;
extern uint8_t *resp_buffer;

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*********************************************************************/
/* functions */
/*********************************************************************/

/*!
 * @brief This API is used to write the content into shuttle eeprom
 */
int16_t coines_shuttle_eeprom_write(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_multi_packet(interface_type,
                                       COINES_CMD_ID_SHUTTLE_EEPROM_WRITE,
                                       (uint8_t *)&start_addr,
                                       1,
                                       buffer,
                                       length);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SHUTTLE_EEPROM_WRITE, resp_buffer, &resp_length);
    }

    return get_coines_error_mapping(ret);
}

/*!
 * @brief This API is used to read the content from the shuttle eeprom
 */
int16_t coines_shuttle_eeprom_read(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SHUTTLE_EEPROM_READ, (uint8_t *)&start_addr, length);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SHUTTLE_EEPROM_READ, resp_buffer, &resp_length);
        if (ret == COINES_SUCCESS)
        {
            memcpy(buffer, &resp_buffer[PAYLOAD_POS], length);
        }
    }

    return get_coines_error_mapping(ret);
}
