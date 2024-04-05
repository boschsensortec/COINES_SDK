/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "coines.h"

#define ROBERT_BOSCH_USB_VID   (0x108C)
#define ARDUINO_USB_VID        (0x2341)
#define BST_APP30_CDC_USB_PID  (0xAB3C)
#define BST_APP20_CDC_USB_PID  (0xAB2C)
#define ARDUINO_NICLA_USB_PID  (0x0060)

/*! Variable to hold the communication interface type */
const enum coines_comm_intf comm_intf = COINES_COMM_INTF_USB;

/*!
 * @brief Exits by printing the error code
 * @param func_name
 * @param err_code
 */
void check_com_result(char *func_name, int16_t err_code)
{
    if (err_code != COINES_SUCCESS)
    {
        const char *err_str = get_coines_error_str(err_code);
        printf("\n%s : %s", func_name, err_str);
        exit(1);
    }
}

/*!
 * @brief API to initialize App board by establishing BLE connection
 * @param ble_name BLE peripheral name
 * @return Error code
 */
int16_t coines_board_init()
{
    struct coines_serial_com_config scom_config;
    struct coines_board_info board_info;

    scom_config.baud_rate = 38400;
    scom_config.vendor_id = ROBERT_BOSCH_USB_VID;
    scom_config.product_id = BST_APP30_CDC_USB_PID;
    scom_config.com_port_name = "COM5";
    scom_config.rx_buffer_size = 2048;

    int16_t result = coines_open_comm_intf(comm_intf, &scom_config);

    check_com_result("Coines open", result);

    result = coines_get_board_info(&board_info);
    if (result == COINES_SUCCESS)
    {
        printf("\nBoard Info:");
        printf("\n\tboard_info.board:0x%02X", board_info.board);
        printf("\n\tboard_info.hardware_id:0x%02X", board_info.hardware_id);
        printf("\n\tboard_info.shuttle_id:0x%02X", board_info.shuttle_id);
        printf("\n\tboard_info.software_id:0x%02X", board_info.software_id);
    }

    coines_delay_msec(100);

    /* Power up the board */
    coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

    coines_delay_msec(200);

    return result;
}

/*!
 * @brief API to deinitialize App board by closing BLE connection
 */
void coines_board_deinit(void)
{
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(100);

    /* COINES_SDK interface reset */
    coines_soft_reset();
    coines_delay_msec(100);

    coines_close_comm_intf(comm_intf, NULL);
}

int main(void)
{
    uint8_t buffer_out[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    int8_t result;

    /* Open comm connection */
    coines_board_init();

    /* Test an echo command */
    result = coines_echo_test(buffer_out, sizeof(buffer_out));
    check_com_result("Coines echo", result);
    if (result == COINES_SUCCESS)
    {
        printf("\nEcho test: Success\n");
    }

    /* Close comm connection */
    coines_board_deinit();

    return 0;
}
