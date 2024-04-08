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
 * @file    comm_intf.h
 * @brief This file contains communication interface related function, variable declarations and Macro definitions
 *
 */

/*!
 * @addtogroup comm_intf_api
 * @{*/

#ifndef COMM_INTF_COMM_INTF_H_
#define COMM_INTF_COMM_INTF_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines_defs.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/

/*! Ring buffer size*/
#define COMM_INTF_RSP_BUF_SIZE  UINT32_C(1048576)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/

/*!
 * * @brief Structure used to hold the streaming information
 */
typedef struct
{
    uint16_t no_of_sensors_enabled; /**< Number of sensors enabled */
    uint16_t sensors_byte_count[COINES_MAX_SENSOR_COUNT]; /**< Sensor byte count */
} comm_stream_info_t;

/**********************************************************************************/
/* function prototype declarations */

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 * @param[in] intf_type: Type of interface(USB, COM, or BLE).
 * @param[out] board_type  : board_type
 *
 * @return Result of API execution status
 * @retval zero -> Success /Negative value -> Error
 */
int16_t comm_intf_open(enum coines_comm_intf intf_type, coines_board_t *board_type);

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 *
 * @param[in] intf_type: Type of interface(USB, COM, or BLE).
 *
 * @return void
 */
void comm_intf_close(enum coines_comm_intf intf_type);

/*!
 * @brief This API is used to initiate the command transfer
 *
 * @param[in] cmd_type : command type
 * @param[in] int_feature : feature type
 *
 * @return void
 */
void comm_intf_init_command_header(uint8_t cmd_type, uint8_t int_feature);

/*!
 * @brief This API is used to write the uint8_t data into command buffer
 *
 * @param[in] data : data to write
 *
 * @return void
 */
void comm_intf_put_u8(uint8_t data);

/*!
 * @brief This API is used to write the uint16_t data into command buffer
 *
 * @param[in] data : data to write
 *
 * @return void
 */
void comm_intf_put_u16(uint16_t data);

/*!
 * @brief This API is used to write the uint32_t data into command buffer
 *
 * @param[in] data : data to write
 *
 * @return void
 */
void comm_intf_put_u32(uint32_t data);

/*!
 * @brief This API is used to send and get the command response from board
 *
 * @param[out] rsp_buf : COINES_SDK response buffer
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Negative value -> Error
 */
int16_t comm_intf_send_command(coines_rsp_buffer_t* rsp_buf);

/*!
 * @brief This API is used to trigger/stop the streaming feature
 *
 * @param[in] state :  state  1- enable/ 0 -disable
 * @param[in] sensor_info : streaming sensor info
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Negative value -> Error
 */
int16_t comm_intf_start_stop_streaming(uint8_t state, comm_stream_info_t *sensor_info);

/*!
 * @brief This API is used to process the streaming response
 *
 * @param[in] sensor_id :  sensor_id
 * @param[in] no_ofsamples : number of samples
 * @param[out] rsp_buf  : response buffer
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval Negative value -> Error
 */
int16_t comm_intf_process_stream_response(uint8_t sensor_id, uint32_t no_ofsamples,
                                          coines_stream_rsp_buffer_t* rsp_buf);

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 *
 *  @param[in] delay_ms   :  delay in milliseconds.
 *
 *  @return void
 */
void comm_intf_delay(uint32_t delay_ms);

/*!
 *  @brief This API is used for processing non streaming response
 *
 *  @param[in] rsp_buf   :  Response buffer
 *
 *  @return void
 */
int16_t comm_intf_process_non_streaming_response(coines_rsp_buffer_t* rsp_buf);

#endif /* COMM_INTF_COMM_INTF_H_ */

/** @}*/
