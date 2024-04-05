/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    file_listing.c
 * @brief Prints file size
 *  Works for PC and MCU_APP30 targets !
 *
 */

#include <stdio.h>
#include "coines.h"

#if defined(PC)
#include <dirent.h>
#endif

int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till USB is connnected
    DIR *d;
    struct dirent *dir;
    d = opendir(".");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            printf("%s\n", dir->d_name);
        }
        closedir(d);
    }

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return(0);
}