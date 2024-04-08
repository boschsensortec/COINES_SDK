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
 * @file    mcu_app3x_support.h
 * @date    July 02, 2023
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_APP3X_SUPPORT_H_
#define MCU_APP3X_SUPPORT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrfx_spim.h"
#include "nrfx_twim.h"
#include "nrf_drv_i2s.h"
#include "nrfx_timer.h"
#include "nrf_timer.h"
#include "nrfx_ppi.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "app30_eeprom.h"
#include "flogfs.h"
#include "w25_common.h"
#include "w25m02gw.h"
#include "w25n02jw.h"
#include "ble_service.h"

#include "uart_common.h"

#if defined(MCU_APP31)

/* I2C common */
#include "i2c_common.h"

/* PMIC driver */
#include "bq25120.h"

#define RESET_INT                NRF_GPIO_PIN_MAP(0, 27) /*RESET line from PMIC */
#define CHRG_LSCTRL              NRF_GPIO_PIN_MAP(0, 03) /*LoadSwitch/LDO control pin */
#define CHRG_CD                  NRF_GPIO_PIN_MAP(0, 28) /*PMIC Chip disable pin */
#define VDDIO_EN                 NRF_GPIO_PIN_MAP(0, 02) /*VDDIO Sensor pin */
#define VDD_EN                   NRF_GPIO_PIN_MAP(1, 12) /*VDD Sensor pin */
#define LS_EN                    NRF_GPIO_PIN_MAP(0, 31) /*Level Shifter pin for Stemma connector */
#define POWER_INT                NRF_GPIO_PIN_MAP(1, 15) /*Interrupt line form PMIC */
#define VIN_DEC                  NRF_GPIO_PIN_MAP(1, 13) /*VIN detection pin */
#endif

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
#define  GPIO_0                  NRF_GPIO_PIN_MAP(0, 14)      /* SB1_4 - P0.14 (I2C1_SCL) */
#define  GPIO_1                  NRF_GPIO_PIN_MAP(0, 13)      /* SB1_5 - P0.13 (I2C1_SDA) */
#define  GPIO_2                  NRF_GPIO_PIN_MAP(1, 1)      /* INT1 - SB1_6 - P1.01 */
#define  GPIO_3                  NRF_GPIO_PIN_MAP(1, 8)      /* INT2 - SB1_7 - P1.08 */
#define  GPIO_CS                 NRF_GPIO_PIN_MAP(0, 24)      /* SB2_1 - P0.24 */
#define  GPIO_SDO                NRF_GPIO_PIN_MAP(0, 15)      /* SB2_3 - P0.15*/
#define  GPIO_4                  NRF_GPIO_PIN_MAP(1, 3)      /* SB2_5 - P1.03 */
#define  GPIO_5                  NRF_GPIO_PIN_MAP(1, 2)      /* SB2_6 - P1.02 */
#define  GPIO_6                  NRF_GPIO_PIN_MAP(1, 11)      /* SB2_7 - P1.11 */
#define  GPIO_7                  NRF_GPIO_PIN_MAP(1, 10)      /* SB2_8 - P1.10 */
#define  GPIO_SDI                NRF_GPIO_PIN_MAP(0, 6)      /* SB2_4 - P0.6*/
#define  GPIO_SCK                NRF_GPIO_PIN_MAP(0, 16)      /* SB2_2 - P0.16*/

#define MCU_LED_R                NRF_GPIO_PIN_MAP(0, 7)
#define MCU_LED_G                NRF_GPIO_PIN_MAP(0, 11)
#define MCU_LED_B                NRF_GPIO_PIN_MAP(0, 12)

#define SWITCH1                  NRF_GPIO_PIN_MAP(1, 9)
#define SWITCH2                  NRF_GPIO_PIN_MAP(0, 25)
#if defined(MCU_APP31)
#define SWITCH3                  NRF_GPIO_PIN_MAP(1, 9)
#endif

#define VDD_SEL                  NRF_GPIO_PIN_MAP(0, 27)
#define VDD_PS_EN                NRF_GPIO_PIN_MAP(0, 3)
#define VDDIO_PS_EN              NRF_GPIO_PIN_MAP(0, 28)

/**********************************************************************************/
/* functions */
/**********************************************************************************/
/****** Reserved Memory Area for performing application switch - 16 bytes******/
#define  MAGIC_LOCATION          (0x2003FFF4)
#define  MAGIC_INFO_ADDR         ((int8_t *)(MAGIC_LOCATION))
#define  APP_START_ADDR          (*(uint32_t *)(MAGIC_LOCATION + 4))
#define  APP_SP_VALUE            (*(uint32_t *)APP_START_ADDR)
#define  APP_RESET_HANDLER_ADDR  (*(uint32_t *)(APP_START_ADDR + 4))

#define TIMER_PRESCALAR          0
#define TIMER_FREQUENCY          16       /*mhz*/
/* #define TIMER_COUNTER_BITS             32 */
#define TIMER_TICKS_PER_SECOND   (TIMER_FREQUENCY / (1 + TIMER_PRESCALAR))

/* #define TIMER_TICKS_TO_USEC(t)         (((uint64_t)t * UINT64_C(1)) / TIMER_TICKS_PER_SECOND) */
#define TIMER_TICKS_TO_NSEC(t)   (((uint64_t)t * UINT64_C(1000)) / TIMER_TICKS_PER_SECOND)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/
typedef void (*ISR_CB)(uint32_t pin, uint32_t polarity);

typedef void (*timed_interrupt_cb)(uint64_t timestamp, uint32_t multiio_pin, uint32_t multiio_pin_polarity);
struct coines_timed_interrupt_config
{
    uint8_t timer_cc_channel;
    nrf_ppi_channel_t ppi_channel;
    timed_interrupt_cb cb;
};

/**@brief Function for converting battery voltage to percentage.
 *
 * @details This is just an estimated percentage considering Maximum charging voltage as 4.2 and cut-off voltage as 3.0.
 *          It will vary between different batteries
 *
 * @param[in] mvolts voltage(in milli volts) to be converted into percentage.
 *
 * @retval battery level in percentage.
 */
uint8_t battery_level_in_percentage(const uint16_t mvolts);

/*!
 * @brief  Callback to read battery voltage
 *
 * @return None
 */
void bat_status_read_callback(void);

/*!
 *
 * @brief       : USB event callback handler
 *
 * @param[in]   : type of usb event
 *
 * @return      : None
 */
void usbd_user_ev_handler(app_usbd_event_type_t event);

/*!
 * @brief This API is used to config RTC
 */
uint32_t rtc_config(void);

#endif /* MCU_APP3X_SUPPORT_H_ */
