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
 * @file    ble_usb_print.c
 * @brief  Prints table of 2 upto 10 over BLE/USB
 *
 * Works for MCU_APP30 and MCU_APP31 targets
 **/

#include<stdio.h>
#include"coines.h"

/* Macro for switching between USB and BLE */
#define BLE_NUS 1
//#define USB_SERIAL 1

#if defined(USB_SERIAL)
#define PRINTF(...)    printf(__VA_ARGS__)
#define COMM_INTF      COINES_COMM_INTF_USB
#elif defined(BLE_NUS)
#define PRINTF(...)    fprintf(bt_w,__VA_ARGS__)
#define COMM_INTF      COINES_COMM_INTF_BLE
#endif

int main()
{
    struct coines_ble_config bleconfig =
                                         {
                                           .name = "APP3.0_BLE_USB_PRINT",
                                           .tx_power = COINES_TX_POWER_0_DBM
                                         };
    coines_ble_config(&bleconfig);
    coines_open_comm_intf(COMM_INTF, NULL); //Wait here till USB serial/BLE is connnected

    PRINTF("Printing table of 2 over BLE ...\r\n");

    for (int i = 1; i <= 10; i++)
        PRINTF("%d x 2 = %d \r\n", i, i * 2);

    PRINTF("Done !\r\n");

    coines_close_comm_intf(COMM_INTF, NULL);

    return 0;
}
