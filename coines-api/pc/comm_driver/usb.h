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
 * @file    usb.h
 * @brief This file contains USB module related macro and API declarations
 *
 */

/*!
 * @addtogroup usb_api
 * @{*/

#ifndef COMM_DRIVER_USB_H_
#define COMM_DRIVER_USB_H_

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines_defs.h"

/**********************************************************************************/
/* data structure declarations */
/**********************************************************************************/

/*!
 * @brief usb response buffer
 */
typedef struct
{
    uint8_t buffer[COINES_DATA_BUF_SIZE]; /**< Data buffer */
    int buffer_size; /**< buffer size */
} usb_rsp_buffer_t;

/**********************************************************************************/
/* function declarations */
/**********************************************************************************/

/*!
 * @brief This internal callback function triggered for USB async response call back in events.
 */
typedef void (*usb_async_response_call_back)(usb_rsp_buffer_t* rsp_buf);

/*!
 *  @brief This API is used to establish the LIB USB communication.
 *
 *  @param[in] comm_buf : communication buffer
 *
 *  @param[in] rsp_cb : response callback
 *
 *  @return Result of API execution status
 *
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t usb_open_device(coines_command_t * comm_buf, usb_async_response_call_back rsp_cb);

/*!
 *  @brief This API closes the USB connection
 *
 *  @return void
 */
void usb_close_device(void);

/*!
 *  @brief This API is used to send the data to the board through USB.
 *
 *  @param[in] buffer     : Data to be sent through USB.
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int16_t usb_send_command(coines_command_t * buffer);

#endif /* COMM_DRIVER_USB_H_ */

/** @}*/
