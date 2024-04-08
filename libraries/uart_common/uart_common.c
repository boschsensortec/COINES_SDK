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
 * @file    uart_common.c
 * @brief   UART communication common APIs for NICLA and APP30
 *
 */

#include "uart_common.h"

#define DEFAULT_RX_PIN_NUMBER_NICLA  9                             /* GPIO2 = p9 */
#define DEFAULT_TX_PIN_NUMBER_NICLA  20                            /* GPIO1 = p20 */
#define DEFAULT_RX_PIN_NUMBER_APP3X  NRF_GPIO_PIN_MAP(1,10)        /* GPIO_7 = p1_10 */
#define DEFAULT_TX_PIN_NUMBER_APP3X  NRF_GPIO_PIN_MAP(1,11)        /* GPIO_6 = p1_11 */
#define BUFFER_SIZE                  1024
#define BAUD_RATE_MUTLIPLY_FACTOR    ((uint64_t)1 << 32U)
#define SYSTEM_CLOCK                 16000000U

/* static variables */
/**********************************************************************************/
static uint8_t temp_rx;
static uint8_t uart_buffer[BUFFER_SIZE] = { 0 };
volatile uint16_t serial_idx = 0, read_idx = 0;

/**
 * @brief  Event handler function to be called when an event occurs in the UART module.
 */
static void uart_event_handler(app_uart_evt_t * p_event)
{

    switch (p_event->evt_type)
    {
        case APP_UART_TX_EMPTY:
            break;

            /* Need to comment this section to receive RX data through app_uart_get(); from application */
        case APP_UART_DATA_READY:
            (void)app_uart_get(&temp_rx);
            uart_buffer[serial_idx] = temp_rx;

            /* Overwrite the last byte in case the buffer has been read */
            if (serial_idx < (BUFFER_SIZE - 1))
            {
                serial_idx++;
            }
            else
            {
                serial_idx = (BUFFER_SIZE - 1);
            }

            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);

            /* To avoid resetting behavior of the device when a UART error is detected */
#ifdef MCU_NICLA
            printf("\r\nCOMMUNICATION ERROR\r\n");
#endif
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);

            /* To avoid resetting behavior of the device when a UART error is detected */
#ifdef MCU_NICLA
            printf("\r\nUART APP_UART_FIFO.\r\n");
#endif
            break;
        default:
            break;
    }
}

/**
 * @brief  Function for initializing the UART communication.
 */
uint32_t uart_init(enum coines_uart_parity parity,
                   enum coines_uart_flow_control flow_control,
                   uint32_t baud_rate)
{
    uint32_t err_code;
    app_uart_comm_params_t uart_config;
    
#ifdef MCU_NICLA
    uart_config.rx_pin_no = DEFAULT_RX_PIN_NUMBER_NICLA;
    uart_config.tx_pin_no = DEFAULT_TX_PIN_NUMBER_NICLA;
#else
    uart_config.rx_pin_no = DEFAULT_RX_PIN_NUMBER_APP3X;
    uart_config.tx_pin_no = DEFAULT_TX_PIN_NUMBER_APP3X;
#endif

    uart_config.flow_control = flow_control;
    uart_config.use_parity = parity;
    baud_rate = (baud_rate * BAUD_RATE_MUTLIPLY_FACTOR)/SYSTEM_CLOCK;
    uart_config.baud_rate = (baud_rate + 0x800) & 0xFFFFF000;

    APP_UART_FIFO_INIT(&uart_config,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_MID,
                       err_code);

    /*Explicitly pulling up RX pin. The APP_UART_COMMUNICATION_ERROR will be triggering if the RX pin is left
     * floating.*/
    nrf_gpio_cfg_input(uart_config.rx_pin_no, NRF_GPIO_PIN_PULLUP);

    APP_ERROR_CHECK(err_code);

    return (uint32_t)err_code;
}

/**
 * @brief  Function to read data in UART communication
 */
uint16_t uart_read(uint8_t *read_buffer, uint16_t len)
{
    uint16_t bytes_read = 0;

    while ((read_idx <= serial_idx) && (bytes_read < len))
    {
        read_buffer[bytes_read] = uart_buffer[read_idx];
        read_idx++;
        bytes_read++;
    }

    if (read_idx >= serial_idx)
    {
        read_idx = 0;
        serial_idx = 0;
    }

    return bytes_read;
}

/**
 * @brief  Function to write the data in UART communication
 */
uint32_t uart_write(uint8_t *write_buffer, uint16_t len)
{
    uint32_t error;

    for (uint16_t i = 0; i < len; i++)
    {
        error = app_uart_put(write_buffer[i]);
        if (error != NRF_SUCCESS)
        {
            return error;
        }
    }

    return NRF_SUCCESS;
}
