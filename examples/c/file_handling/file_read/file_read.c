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
 * @file    file_read.c
 * @brief   Read 10 lines of "test.txt" and print it on serial console
 *  Copy small file named "test.txt" in Flash memory before running this program
 *
 * Works for MCU_APP30 and MCU_APP31 targets
 **/

#include<stdio.h>
#include"coines.h"
int main()
{
    char a[128];
    int lines=10;
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till serial port is opened
    
    FILE *fp = fopen("test.txt","r");
    
    while(lines--)
    {
        fgets(a,128,fp);
        printf("%s",a);
    }
    
    fflush(stdout);
    fclose(fp);
    
    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    
    return 0;
}