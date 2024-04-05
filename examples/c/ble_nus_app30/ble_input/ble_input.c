/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    ble_input.c
 * @brief   Demonstrating BLE input
 *  Tested with
 *  1. Serial Bluetooth Terminal [https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal]
 *  2. ble-nus-term.py (See 'util' folder)
 */

#include<stdio.h>
#include"coines.h"

int main()
{
    int a, b;
    struct coines_ble_config bleconfig =
                                         {
                                           .name = "APP3.0_BLE_INPUT",
                                           .tx_power = COINES_TX_POWER_0_DBM
                                         };
    coines_ble_config(&bleconfig);
    coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL); //Wait here till BLE is connnected

    fprintf(bt_w, "Enter 2 numbers separated by space\r\n");

    while (1)
    {
        while (fscanf(bt_r, "%d %d", &a, &b) == EOF); // Wait for input
        fprintf(bt_w, "Sum of %d and %d is %d\r\n", a, b, a + b);
    }

    coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);

    return 0;
}
