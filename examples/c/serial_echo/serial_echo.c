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
 * Works for MCU_APP30, MCU_APP31 and MCU_APP20 targets
 **/

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
