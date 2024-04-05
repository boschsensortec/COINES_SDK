/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    file_read.c
 * @brief   Read 10 lines of "test.txt" and print it on serial console
 *  Copy small file named "test.txt" in Flash memory before running this program
 *
 */

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
