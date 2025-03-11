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
 * @file    uart_common.h
 */

#ifndef UART_COMMON_H_
#define UART_COMMON_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nrf_uart.h"
#include "nrf_log.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "nrf_delay.h"
#include "coines.h"

#define CTS_PIN_NOT_CONNECTED  (int)0xFFFFFFFF
#define RTS_PIN_NOT_CONNECTED  (int)0xFFFFFFFF
#define UART_TX_BUF_SIZE       256                              /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE       256

/**
 * @brief  Function for initializing the UART communication.
 *
 * @param[in]  parity         : parity
 * @param[in]  flow_control   : flow control mode
 * @param[in]  baud_rate      : baud rate
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
uint32_t uart_init(enum coines_uart_parity parity, enum coines_uart_flow_control flow_control, uint32_t baud_rate);

/**
 * @brief  Function to read the data in UART communication
 *
 * @param[out] read_buffer : Pointer to the buffer to store the data
 * @param[in]  len         : Length of the buffer
 *
 * @return Number of bytes read
 */
uint16_t uart_read(uint8_t *read_buffer, uint16_t len);

/**
 * @brief  Function to write the data in UART communication
 *
 * @param[in] write_buffer : Pointer to the buffer which data need to written
 * @param[in] len          : Length of the buffer
 *
 * @return Results of API execution status.
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
uint32_t uart_write(uint8_t *write_buffer, uint16_t len);

#endif /* UART_COMMON_H_ */

/** @}*/
