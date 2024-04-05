/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    file_size.c
 * @brief   Prints file size
 *  Works for PC and MCU_APP30 targets !
 *
 */

#include <stdio.h>
#include <sys/stat.h>
#include "coines.h"

/* NOTE: File names are case sensitive in FlogFS and Unix systems !*/
#define FILE_NAME "README.TXT" 

int main()
{
    struct stat st;
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till serial port is opened

    if(stat(FILE_NAME, &st) != 0)
    {
        printf("File doesn't exist - %s",FILE_NAME);
        return -1;
    }
    printf("File size is %d bytes", (int)st.st_size);

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return 0;
}
