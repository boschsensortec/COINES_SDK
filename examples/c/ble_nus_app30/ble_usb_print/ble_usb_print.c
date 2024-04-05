/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    ble_usb_print.c
 * @brief  Prints table of 2 upto 10 over BLE/USB
 *
 */

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
