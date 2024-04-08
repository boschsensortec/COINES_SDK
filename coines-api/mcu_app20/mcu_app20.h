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
 * @file    mcu_app20.h
 * @date    Jan 2, 2019
 * @brief This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_APP20_H_
#define MCU_APP20_H_
/**********************************************************************************/
/* system header includes */
/**********************************************************************************/

#include <compiler.h>
#include <status_codes.h>

/* From module: Generic board support */
#include <board.h>

/* From module: IOPORT - General purpose I/O service */
#include <ioport.h>

/* From module: Interrupt management - SAM implementation */
#include <interrupt.h>

/* From module: PIO - Parallel Input/Output Controller */
#include <pio.h>

/* From module: PMC - Power Management Controller */
#include <pmc.h>
#include <sleep.h>

/* From module: Part identification macros */
#include <parts.h>

/* From module: Sleep manager - SAM implementation */
#include <sam/sleepmgr.h>
#include <sleepmgr.h>

/* From module: System Clock Control - SAM4S implementation */
#include <sysclk.h>

/* From module: USB CDC Protocol */
#include <usb_protocol_cdc.h>

/* From module: USB Device CDC (Single Interface Device) */
#include <udi_cdc.h>

/* From module: USB Device CDC Standard I/O (stdio) - SAM implementation */
#include <stdio_usb.h>

/* From module: USB Device Stack Core (Common API) */
#include <udc.h>
#include <udd.h>

/* From module: pio_handler support enabled */
#include <pio_handler.h>

#include <delay.h>
#include <twi.h>
#include <wdt.h>
#include <spi.h>
#include <rstc.h>
#include <tc.h>
#include <pdc.h>
#include <spi_master.h>
#include <pio_handler.h>
#include <string.h>

/**********************************************************************************/
/*macro definitions */
/**********************************************************************************/
#define SHUTTLE_VDD_EN           PIO_PA0_IDX
#define SHUTTLE_VDDIO_EN         PIO_PA1_IDX

#define SHUTTLE_COD0             PIO_PC18_IDX
#define SHUTTLE_COD1             PIO_PC19_IDX
#define SHUTTLE_COD2             PIO_PC20_IDX
#define SHUTTLE_COD3             PIO_PC22_IDX
#define SHUTTLE_COD4             PIO_PC23_IDX
#define SHUTTLE_COD5             PIO_PC29_IDX
#define SHUTTLE_COD6             PIO_PC15_IDX
#define SHUTTLE_COD7             PIO_PC13_IDX
#define SHUTTLE_COD8             PIO_PC12_IDX

#define SHUTTLE_IO0              PIO_PC0_IDX
#define SHUTTLE_IO1              PIO_PC1_IDX
#define SHUTTLE_IO2              PIO_PC2_IDX
#define SHUTTLE_IO3              PIO_PC3_IDX
#define SHUTTLE_IO4              PIO_PC4_IDX
#define SHUTTLE_IO5              PIO_PC5_IDX
#define SHUTTLE_IO6              PIO_PC6_IDX
#define SHUTTLE_IO7              PIO_PC7_IDX
#define SHUTTLE_IO8              PIO_PC8_IDX

#define SHUTTLE_SPI_CS0          PIO_PA11_IDX
#define SHUTTLE_SDO              PIO_PA12_IDX

#define BOARD_LED1               PIO_PC17_IDX
#define BOARD_LED2               PIO_PC28_IDX
#define BOARD_LED3               PIO_PC30_IDX
#define BOARD_LED4               PIO_PC31_IDX

/*********** Reserved Memory Area for performing application switch***********/
#define  MAGIC_LOCATION          (0x2001FFE4)
#define  MAGIC_INFO_ADDR         ((S8 *)(MAGIC_LOCATION))
#define  APP_START_ADDR          (*(U32 *)(MAGIC_LOCATION + 4))
#define  APP_SP_VALUE            (*(U32 *)APP_START_ADDR)
#define  APP_RESET_HANDLER_ADDR  (*(U32 *)(APP_START_ADDR + 4))

/****************************************************************************/
typedef void (*timed_interrupt_cb)(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
void (*timer_interrupt_handler[COINES_TIMER_INSTANCE_MAX]) (void);
struct coines_timed_interrupt_config
{
    timed_interrupt_cb cb;
};

struct coines_timed_interrupt_config timed_interrupt_config[COINES_SHUTTLE_PIN_MAX];

void TC0_Handler    ( void ) __attribute__ ((weak, alias("tc0_handler")));
void TC1_Handler    ( void ) __attribute__ ((weak, alias("tc1_handler")));
void TC2_Handler    ( void ) __attribute__ ((weak, alias("tc2_handler")));
#endif /* MCU_APP20_H_ */
