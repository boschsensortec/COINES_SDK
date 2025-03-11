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
 * @file gpio.c
 *
 * @brief This module provides APIs for controlling General Purpose Input/Output (GPIO) interfaces.
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
#include "gpio.h"

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
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 *
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{
    uint8_t payload[3] = { pin_number, direction, pin_value };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SET_PIN, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SET_PIN, resp_buffer, &resp_length);
    }

    (void)resp_length;

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 *
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    uint8_t payload[3] = { pin_number, *pin_direction, *pin_value };
    int16_t ret;
    uint16_t resp_length = 0;

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_GET_PIN, payload, 3);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_GET_PIN, resp_buffer, &resp_length);
    }

    if (ret == COINES_SUCCESS)
    {
        if (resp_length >= 3 && pin_number == resp_buffer[PAYLOAD_POS])
        {
            *pin_direction = (enum coines_pin_direction)resp_buffer[PAYLOAD_POS + 1];
            *pin_value = (enum coines_pin_value)resp_buffer[PAYLOAD_POS + 2];
        }
        else
        {
            return COINES_E_COMM_WRONG_RESPONSE;
        }
    }

    return get_coines_error_mapping(ret);
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    uint8_t payload[4] = { 0 };
    int16_t ret;
    uint16_t resp_length = 0;

    memcpy(payload, &vdd_millivolt, 2);
    memcpy(&payload[2], &vddio_millivolt, 2);

    ret = protocol_encode_packet(interface_type, COINES_CMD_ID_SET_VDD_VDDIO, payload, 4);
    if (ret == COINES_SUCCESS)
    {
        ret = protocol_decode_packet(interface_type, COINES_CMD_ID_SET_VDD_VDDIO, resp_buffer, &resp_length);
    }

    return get_coines_error_mapping(ret);
}
