/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file	read_register_data.c
 * @brief	Sample file to read 16bit length SPI register data using LIB COINES
 *
 */

/*********************************************************************/
/* system header files */
/*********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*********************************************************************/
/* own header files */
/*********************************************************************/
#include "coines.h"

/*********************************************************************/
/* local macro definitions */


/*********************************************************************/
/* global variables */
/*********************************************************************/

/*********************************************************************/
/* static function declarations */
/*********************************************************************/

/*!
 * @brief	internal API is used to initialize the sensor interface
 */
static void init_sensor_interface(void);

/*********************************************************************/
/* functions */
/*********************************************************************/
/*!
 *  @brief This internal API is used to initialize the SPI interface.
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_sensor_interface(void)
{
    /* Switch VDD & VDDIO for sensor ON */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
    coines_delay_msec(100);
    /* Configure 16bit length SPI bus */
    coines_config_word_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_1_MHZ, COINES_SPI_MODE0,COINES_SPI_TRANSFER_16BIT);
}
/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])
{
	int16_t rslt;
	uint16_t times_to_read = 0;
	uint16_t read_data[256] = {0};
	uint16_t reg_addr = 0x00;
	/*Number of bytes */
	uint16_t count = 0;

	rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);

	if (rslt < 0)
	{
		printf( "\n Unable to connect with Application Board ! \n"
				" 1. Check if the board is connected and powered on. \n"
				" 2. Check if Application Board USB driver is installed. \n"
				" 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
		exit(rslt);
	}

	init_sensor_interface();
	/* after sensor init introduce 200 msec sleep */
	coines_delay_msec(200);
	/* Max. number of words that can be read by using COINES_SDK is 512 (1024 bytes)*/
	while (times_to_read < 256)
	{
		coines_read_16bit_spi(COINES_SPI_BUS_0,COINES_SHUTTLE_PIN_7,reg_addr,read_data,count);

		for(int i = 0; i < times_to_read;i++)
		{
			printf("%x ",read_data[i]);
		}
		printf("\n");
		fflush(stdout);

		coines_delay_msec(20);
		times_to_read++;
		count++;
	}

	coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
	return EXIT_SUCCESS;
}

