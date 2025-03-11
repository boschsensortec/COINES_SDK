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
 * @file    mcu_nicla_support.h
 * @date    May 10, 2022
 * @brief   COINES_SDK support file for mcu_nicla.c
 *
 */
#ifndef MCU_NICLA_SUPPORT_H_
#define MCU_NICLA_SUPPORT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>

#include "nrf.h"
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
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_power.h"
#include "nrf_nvmc.h"

#include "app_error.h"
#include "app_util.h"

#include "ble_service.h"

/* NICLA I2C common */
#include "i2c_common.h"

/* NICLA UART common */
#include "uart_common.h"

/* PMIC driver */
#include "bq25120.h"

/* LED driver */
#include "led.h"

/* FLash driver */
#include "mx25_ext_flash.h"

/* LittleFS */
#include "lfs.h"
#include "lfs_util.h"

/**********************************************************************************/
/* functions */
/**********************************************************************************/
/****** Reserved Memory Area for performing application switch - 16 bytes******/
#define  MAGIC_LOCATION          (0x2000F804)
#define  MAGIC_INFO_ADDR         ((int8_t *)(MAGIC_LOCATION))
#define  APP_START_ADDR          (*(uint32_t *)(MAGIC_LOCATION + 4))
#define  APP_SP_VALUE            (*(uint32_t *)APP_START_ADDR)
#define  APP_RESET_HANDLER_ADDR  (*(uint32_t *)(APP_START_ADDR + 4))

/**@brief Function for converting battery voltage to percentage.
 *
 * @details This is just an estimated percentage considering Maximum charging voltage as 4.2 and cut-off voltage as 3.0.
 *          It will vary between different batteries
 *
 * @param[in] mvolts voltage(in milli volts) to be converted into percentage.
 *
 * @retval battery level in percentage.
 */

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
/* void usbd_user_ev_handler(app_usbd_event_type_t event); */

/*!
 * @brief This API is used to config RTC
 */
uint32_t rtc_config(void);

/*!
 * @brief Unmount flash
 */
void fs_deinit(void);

/*!
 * @brief Configuring p21 as normal GPIO
 */
void reconfig_reset_pin(void);

/*!
 * @brief Configure NFCT pins as GPIOs. UART RX is assigned to P0.9, same pin that is used as the antenna input for NFC by default
 */
void reconfig_nfct_pin(void);

#endif /* MCU_NICLA_SUPPORT_H_ */
