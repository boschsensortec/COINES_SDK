/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    ble_print.c
 * @brief  Prints table of 2 upto 10 over BLE
 *
 */

#include<stdio.h>
#include"coines.h"

int main()
{
    coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL); //Wait here till BLE is connnected

    fprintf(bt_w, "Printing table of 2 over BLE ...\r\n");

    for(int i=1;i<=10;i++)
        fprintf(bt_w, "%d x 2 = %d \r\n",i,i*2);

    fprintf(bt_w, "Done !\r\n");

    coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);

    return 0;
}
