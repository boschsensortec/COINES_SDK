/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include <string.h>

#include "coines.h"

#define COM_READ_BUFF  2048

int main(void)
{
    enum coines_comm_intf comm_intf = COINES_COMM_INTF_USB;
    uint8_t buff[COM_READ_BUFF] = { 0 };
    uint16_t bytes_available;

    coines_open_comm_intf(comm_intf, NULL); /* Wait here till USB is connected */

    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF); /* Turn of the LED to indicate that the board is connected */

    while (1)
    {
        bytes_available = coines_intf_available(comm_intf);
        if (bytes_available)
        {
            coines_read_intf(comm_intf, buff, bytes_available);
            coines_write_intf(comm_intf, buff, bytes_available);
        }
    }

    coines_close_comm_intf(comm_intf, NULL);

    return 0;
}
