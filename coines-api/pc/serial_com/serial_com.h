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
 *  @file   serial_com.h
 *  @brief  This file contains serial communication interface related function prototype, variable declarations and Macro definitions
 */

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include <stddef.h>

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
#define SCOM_OK                     INT8_C(0)
#define SCOM_E_DEV_NOT_FOUND        INT8_C(-1)
#define SCOM_E_PORT_IN_USE          INT8_C(-2)
#define SCOM_E_PORT_NOT_OPEN        INT8_C(-3)
#define SCOM_E_CONFIG_FAILED        INT8_C(-4)
#define SCOM_E_READ_FAILED          INT8_C(-5)
#define SCOM_E_WRITE_FAILED         INT8_C(-6)
#define SCOM_E_FLUSH_FAILED         INT8_C(-7)
#define SCOM_E_PORT_NAME_NOT_FOUND  INT8_C(-8)
#define SCOM_E_DEV_NOT_SERIAL_COM   INT8_C(-9)

#define SCOM_OUT_BUFFER_SIZE        UINT32_C(2060)
#define SCOM_IN_BUFFER_SIZE         UINT32_C(2060)

/**********************************************************************************/
/* function declarations */
/**********************************************************************************/

/*!
 *  @brief This API is used to initialize the serial communication.
 *
 *  @param[in] baud_rate : baudrate
 *
 *  @param[in] vendor_id : vendor id
 *
 *  @param[in] product_id : product id
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int8_t scom_open_id(uint32_t baud_rate, uint16_t vendor_id, uint16_t product_id, char *com_port_name);

/*!
 *  @brief This API is used to initialize the serial communication with default configuration.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int8_t scom_open(void);

/*!
 *  @brief This API is used to read the data.
 *
 *  @param[in] buffer : buffer to store the received data
 *
 *  @param[in] n_bytes : number of bytes to read
 *
 *  @param[in] n_bytes_read : number of bytes read
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
int8_t scom_read(void *buffer, uint32_t n_bytes, uint32_t *n_bytes_read);

/*!
 *  @brief This API is used to send the data.
 *
 *  @param[in] buffer : data
 *
 *  @param[in] n_bytes : number of bytes to send
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */

int8_t scom_write(void *buffer, uint32_t n_bytes);

/*!
 * @brief This API is used to close the serial communication.
 *
 * @return void
 */
int8_t scom_close(void);

/*!
 * @brief This API is used to clear transmit and receive buffer.
 *
 * @return void
 */
int8_t scom_clear_buffer(void);

#ifdef __cplusplus
}
#endif

#endif
