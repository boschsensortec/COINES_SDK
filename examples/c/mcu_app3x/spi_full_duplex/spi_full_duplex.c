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
 * @file    spi_full_duplex.c
 * @brief   Test the spi full duplex transfer.
 *          Works on MCU_APP3X target with the BMI270 sensor. For other sensors, 
 *          update the SPI CS and voltage range based on the datasheet. 
 */

/*********************************************************************/
/* header includes */
/**********************************************************************/
#include <stdio.h>
#include <stdbool.h>

#include "spi_full_duplex.h"
/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
static uint8_t tx_buff[] = {CHIP_ID_ADDR | 0x80};
static uint8_t tx_len = sizeof(tx_buff);
static uint8_t rx_buff[sizeof(tx_buff) + NO_OF_BYTES];
static uint8_t rx_len = sizeof(tx_buff) + NO_OF_BYTES;

/*********************************************************************/
/* static function declaration */
/**********************************************************************/

/*********************************************************************/
/* function definition */
/**********************************************************************/
int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till USB is connnected

    /* Configure SPI bus */
    coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_PLATFORM_SPEED_8_MHZ, COINES_SPI_MODE0);
    coines_set_pin_config(COINES_SHUTTLE_PIN_21, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);

    /* Power up the board */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
	coines_delay_msec(200);

    /* Performing a dummy read to bring interface back to SPI from I2C interface */
    (void)coines_spi_transfer(COINES_SPI_BUS_0, COINES_MINI_SHUTTLE_PIN_2_1, tx_buff, tx_len, rx_buff, rx_len);
    coines_delay_msec(200);

    /* Read Chip ID*/
    (void)coines_spi_transfer(COINES_SPI_BUS_0, COINES_MINI_SHUTTLE_PIN_2_1, tx_buff, tx_len, rx_buff,rx_len);
    coines_delay_msec(200);

    /* In full duplex SPI, rx_buff contains the data received during the entire transfer, including the bytes clocked out while sending tx_buff.
    To get the actual sensor response, skip the bytes corresponding to the transmitted command (tx_buff) and any dummy bytes.*/
    printf("Chip ID: 0x%02X\n", rx_buff[tx_len+DUMMY_BYTE_LEN]);

    /* Write accel config */
    uint8_t accel_config[] = {0x40, 0xAC};
    (void)coines_spi_transfer(COINES_SPI_BUS_0, COINES_MINI_SHUTTLE_PIN_2_1, accel_config, sizeof(accel_config), NULL, 0);
    coines_delay_msec(200);

    /* De-configure SPI bus*/
    coines_deconfig_spi_bus(COINES_SPI_BUS_0);

    /* Power-off the shuttle board */
    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
	
	/* Coines interface reset */
	coines_soft_reset();
	coines_delay_msec(100);

    /* Close the communication */
    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
}

