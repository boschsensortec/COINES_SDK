/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    file_write.c
 * @brief 	Prints table of 2 upto 10 in a file named "table_of_2.txt" 
 *
 */

#include<stdio.h>
#include"coines.h"
int main()
{
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till serial port is opened
	
	remove("table_of_2.txt");
    
    FILE *fp = fopen("table_of_2.txt","w");
	
	printf("Printing table of 2 to file ...\r\n");
    
    for(int i=1;i<=10;i++)
		fprintf(fp,"%d x 2 = %d \n",i,i*2); 
    
    fclose(fp);
	
	printf("Done !\r\n");
    
    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
    
    return 0;
}
