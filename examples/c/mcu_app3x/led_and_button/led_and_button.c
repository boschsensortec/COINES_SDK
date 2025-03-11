/**
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
 * @file    led_and_button.c
 * @brief   On pressing button 1 Red and blue LED will glow and for button 2 green LED will glow
 * Works only for MCU_APP30 target
 **/
#include <stdio.h>
#include <stdbool.h>
#include "coines.h"

/* Callback for button 1 interrupt */
static void button1CB(uint32_t param1, uint32_t param2);
/* Callback for button 2 interrupt */
static void button2CB(uint32_t param1, uint32_t param2);

int main(void)
{
    coines_open_comm_intf(COINES_COMM_INTF_USB, NULL); //Wait here till USB is connnected

    coines_set_pin_config(COINES_APP30_BUTTON_1, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_APP30_BUTTON_1, button1CB, COINES_PIN_INTERRUPT_FALLING_EDGE);

    coines_set_pin_config(COINES_APP30_BUTTON_2, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_APP30_BUTTON_2, button2CB, COINES_PIN_INTERRUPT_FALLING_EDGE);

    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    return (0);
}

/*Callback for button 1 event */
void button1CB(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;

    coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
    coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_ON);
}

/*Callback for button 2 event */
void button2CB(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    
    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
    coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);
}
