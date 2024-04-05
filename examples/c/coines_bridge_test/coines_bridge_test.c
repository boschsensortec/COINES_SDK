/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "coines.h"

static enum coines_comm_intf comm_intf = COINES_COMM_INTF_USB;

static void check(int8_t res);
void com_echo_test(void);
int16_t coines_echo_test(uint8_t *data, uint16_t length);

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    check(coines_open_comm_intf(comm_intf, NULL));

    coines_delay_msec(10);

    com_echo_test();

    get_board_test();
    set_vdd_test();
    set_get_pin_test();

    coines_delay_msec(10);
    check(coines_close_comm_intf(comm_intf, NULL));

    return 0;
}

static void check(int8_t res)
{
    if (res)
    {
        printf("Error %d\r\n", res);
        exit(0);
    }
}

static void test_value_range(uint64_t value, uint64_t expected, uint64_t tolerance, char fun_name[])
{
    if ((value < expected - tolerance) || (value > expected + tolerance))
    {
        printf("%s AUTOMATION_FAIL : Expected value %lld tolerance %lld Recieved %lld \r\n",
               fun_name,
               expected,
               tolerance,
               value);
    }
}

void com_echo_test(void)
{
    uint8_t buffer_out[2048] = { 0 };
    uint32_t transactions_pass = 0, transactions_fail = 0;
    uint32_t start_time;
    int8_t res = COINES_SUCCESS;

    for (uint16_t i = 0; i < sizeof(buffer_out); i++)
    {
        buffer_out[i] = (uint8_t)i;
    }

    for (uint32_t n = 32; (n <= sizeof(buffer_out)) && (res == COINES_SUCCESS); n = n * 2)
    {
        transactions_fail = 0;
        transactions_pass = 0;
        printf("Payload size of %u bytes \r\n", n);
        start_time = coines_get_millis();
        while (coines_get_millis() <= (start_time + 5000))
        {
            /*printf("%u %u\n", transactions_pass, n); */
            res = coines_echo_test(buffer_out, n);
            check(res);
            if (res)
            {
                transactions_fail++;
            }
            else
            {
                transactions_pass++;
            }
        }

        if (transactions_fail)
        {
            printf("Number of transactions in 5 seconds :: AUTOMATION_FAIL: %u. Throughput %u Bps\r\n",
                   transactions_pass,
                   transactions_fail,
                   transactions_pass * n / 5);
        }

        printf("Number of transactions in 5 seconds :: Pass: %u Throughput %u Bps\r\n",
               transactions_pass,
               transactions_pass * n / 5);
    }
}

/* enum coines_cmds - Ref coines_brige_client.h */

/* COINES_CMD_ID_GET_BOARD_INFO */
void get_board_test()
{
    /*int16_t rslt; */
    struct coines_board_info board_info = { 0 };

    /* Check for default configuration */
    test_value_range(coines_get_board_info(&board_info), 0, 0, "get board");
    test_value_range(board_info.board, 5, 0, "B info");
    test_value_range(board_info.hardware_id, 0x11, 0, "B Hw ID");
    test_value_range(board_info.shuttle_id, 0x1b8, 0, "B S ID"); /*@karthika : Kindly provide minimum maximum range */
    test_value_range(board_info.software_id, 0x0120, 0, "B Sw ID");

    /* Check for configurations of feature
    //configure the eeprom as random number ( e.g. board_info.shuttle_id + 1)
    test_value_range(coines_get_board_info(&board_info),"get_board");
    test_value_range(board_info.board,5,0);
    test_value_range(board_info.hardware_id,0x11,0);
    test_value_range(board_info.shuttle_id,0x1f9+1,0);
    test_value_range(board_info.software_id,0x10,0);
    */

    /* Check for functionality error
    //configure the eeprom as 0x00
    test_value_range(coines_get_board_info(&board_info),"get_board");
    test_value_range(board_info.board,5,0);
    test_value_range(board_info.hardware_id,0x11,0);
    test_value_range(board_info.shuttle_id,0x1f9,0);
    test_value_range(board_info.software_id,0x10,0);
    */
}

/* COINES_CMD_ID_SET_VDD_VDDIO */
void set_vdd_test()
{
    int16_t return_val = COINES_SUCCESS;
    uint16_t vdd_millivolt = 1800;
    uint16_t vddio_millivolt = 1800;

    /* check for default configuration - NA */

    /* check for configuration of the feature */
    for (uint16_t index_vdd = 0; index_vdd < 5000; index_vdd = index_vdd + 100)
    {
        for (uint16_t index_vddio = 0; index_vddio < 5000; index_vddio = index_vddio + 100)
        {
            if ((vdd_millivolt > 3600) || (vddio_millivolt > 3600)) /*to be updated as per hw */
            {
                return_val = COINES_E_NOT_SUPPORTED;
            }
            else
            {
                return_val = COINES_SUCCESS;
            }

            /*TODO
             * :test_value_range(coines_set_shuttleboard_vdd_vddio_config(index_vdd,index_vddio),COINES_SUCCESS,0,"set
             * vdd"); //to be checked if hw damange wont happen */
        }
    }

    /* check for the functional testing of the feature ? TBD */
}

/* COINES_CMD_ID_SET_PIN */
void set_get_pin_test()
{
    enum coines_multi_io_pin pin_number = COINES_SHUTTLE_PIN_9;
    enum coines_pin_direction direction = COINES_PIN_DIRECTION_OUT;
    enum coines_pin_value pin_value = COINES_PIN_VALUE_LOW;
    enum coines_pin_direction get_direction;
    enum coines_pin_value get_pin_value;

    /* Check for default configuration //TODO :Enable the tests
    for(uint8_t index = 1 ; index< COINES_SHUTTLE_PIN_MAX; index++)
    {
        test_value_range(coines_get_pin_config((enum coines_multi_io_pin)index,&get_direction,&get_pin_value),0,0,"get_pin");
        test_value_range(pin_number,index,0,"pin no");
        test_value_range(get_direction,1,1,"direction");
        test_value_range(get_pin_value,1,1,"pin val");
    }*/

    /* Check for configurations of feature */
    for (uint8_t index = 1; index < COINES_SHUTTLE_PIN_MAX; index++)
    {
        /*test_value_range(coines_set_pin_config((enum coines_multi_io_pin)index,direction,pin_value),0,0,"set pin");
        test_value_range(coines_get_pin_config((enum coines_multi_io_pin)pin_number,&get_direction,&get_pin_value),0,0,"get pin");
        test_value_range(pin_number,index,0,"pin index");
        test_value_range(get_direction,direction,0,"pin direction");
        test_value_range(get_pin_value,pin_value,0,"pin value");*/
    }

    /* Check the functionality of the features? */
}
