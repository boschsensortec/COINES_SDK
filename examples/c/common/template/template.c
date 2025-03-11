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
 * @file    template.c
 * @date    Oct-15-2019
 * @brief   Template project for COINES_SDK example
 *
 * Works for PC, MCU_APP30 and MCU_APP31 targets
 **/

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
    fclose(fp);
#endif

    // Your C code goes here ...
    // Refer doc/BST-DHW-AN013.pdf for information regarding COINES_SDK API.
    // Use COINES_SDK with sensorAPI to play with our sensors.
    // Get sensorAPI from https://github.com/BoschSensortec

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    return 0;
}
