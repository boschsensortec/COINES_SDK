/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    ble_list_file_details.c
 * @brief  Prints file details over BLE
 *
 */

#include <stdio.h>
#include "coines.h"
#include <sys/stat.h>

int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL); //Wait here till BLE is connnected

    DIR *d;
    struct dirent *dir;
    struct stat st;

    fprintf(bt_w,"-------------------- File list --------------------\r\n");
    d = opendir(".");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            stat(dir->d_name, &st);
            fprintf(bt_w, "%-32s\t%0.2f kB\r\n", dir->d_name, (int)st.st_size/1024.0);
        }
        closedir(d);
    }

    coines_close_comm_intf(COINES_COMM_INTF_BLE, NULL);

    return(0);
}