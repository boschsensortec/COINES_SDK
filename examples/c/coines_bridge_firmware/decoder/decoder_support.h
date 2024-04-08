/**
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 * @file        decoder_support.h
 *
 * @brief       This file decodes the commands and process accordingly
 *
 */

#ifndef APPLICATION_DECODER_SUPPORT_H_
#define APPLICATION_DECODER_SUPPORT_H_
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif

/*!
*
* @brief       : API to validate the command parameters
*
* @param[in]   : buffer - pointer to the command.
*
* @return      : returns decoder_rsp_t enum type
*
*/
decoder_rsp_t parameter_prevalidation(const uint8_t *buffer);
/*!
 *
 * @brief       : API will read and update the MCU time in response packets,
 *                based on the use_timestamp variable
 *
 * return       : void
 */
void decoder_update_timestamp(void);
/*!
 *
 * @brief       :API to parse and update the error response
 *
 * @param[in]   : error_code - To hold error response
 * @param[in]   : feature_id - To hold feature, applicable only set/get commands
 * @param[in]    : cmd -  Command for which error triggered
 *
 * @return      : returns decoder_rsp_t enum type
 *
 */
decoder_rsp_t decoder_error_fuction(decoder_rsp_t error_code, int8_t feature_id, int8_t cmd);
/*!
 *
 * @brief       : API to calculate the length
 *
 * @param[in]    : cmd -  Command for which length is calculated
 *
 * @return       : return uint8_t type
 *;
 */
uint8_t decoder_calc_len(const uint8_t *cmd);
/*!
 *
 * @brief       : API will sets/gets sensor vdd voltage
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x01_vdd(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API will sets/gets sensor vddio voltage
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x02_vddio(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API will sets/gets i2c speed
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x09_i2cspeed(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to set/get the interfaces to communicate with sensors
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x11_interface(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to set shuttle vdd and vddio voltages
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x14_shuttle_pwr_cfg(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to set/get IO pins
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x15_multiioconfig(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to set/get spi configurations
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x19_spiconfig(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API used to get the board information
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x1f_boardinfo(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to set switch from application to BTL
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x30_app_switch(const uint8_t* buffer, decoder_dir_t dir);
/*!
 *
 * @brief       : API to invoke BTL
 *
 * @param[in]   : buffer - pointer to the command
 * @param[in]    : dir -  Tells about set or get
 *
 * @return      : returns decoder_rsp_t enum type
 */
decoder_rsp_t decoder_0x31_invoke_btl(const uint8_t* buffer, decoder_dir_t dir);


#ifdef __cplusplus
}
#endif
#endif /* APPLICATION_DECODER_SUPPORT_H_ */
