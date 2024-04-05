/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    template.c
 * @date    Oct-15-2019
 * @brief   Template project for COINES_SDK example
 *
 */

#include <stdio.h>
#include "coines.h"

int main()
{
    int rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); // Connect to Application board
    if( rslt < 0 )
    {
        printf("Unable to connect to Application Board !\n");
        return rslt;
    }

    // Some test C code to get Application Board information
    struct coines_board_info data = {};
    rslt = coines_get_board_info(&data);

    if (rslt == COINES_SUCCESS)
    {
        printf("Hardware - v%d.%d \n", (data.hardware_id & 0xF0) >> 4 , data.hardware_id & 0x0F);
        printf("Software - v%d.%d \n", (data.software_id & 0xF0) >> 4 , data.software_id & 0x0F);
        printf("Type of board - %d \n", data.board );
        printf("Shuttle ID - 0x%x\n", data.shuttle_id);
    }

    printf("COINES version - %s\n", coines_get_version());

/* This code is only compiled for targets, TARGET=PC and TARGET=MCU_APP30 */
#if defined(PC) || defined(MCU_APP30)
    FILE *fp;
    /* File is created in APP3.0 flash memory for TARGET=MCU_APP30 !*/
    fp = fopen("hello.txt", "w");
    fprintf(fp, "Hello from Bosch Sensortec GmbH !");
    fclose(fp);
#endif

    // Your C code goes here ...
    // Refer doc/BST-DHW-AN013.pdf for information regarding COINES_SDK API.
    // Use COINES_SDK with sensorAPI to play with our sensors.
    // Get sensorAPI from https://github.com/BoschSensortec

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    return 0;
}
