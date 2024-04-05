/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
