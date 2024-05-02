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
 * @file    ble_list_file_details.c
 * @brief  Prints file details over BLE
 *
 * Works for MCU_APP30 and MCU_APP31 targets
 **/

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