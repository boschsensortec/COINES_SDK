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
 * @file    mcu_nicla.h
 * @date    May 10, 2022
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_NICLA_H_
#define MCU_NICLA_H_

#include "mcu_nicla_support.h"
#include "coines.h"

#include <sys/stat.h>

#define GPIO_0                       NRF_GPIO_PIN_MAP(0, 24)   /* GPIO_38 */
#define GPIO_1                       NRF_GPIO_PIN_MAP(0, 20)   /* GPIO_45 */
#define GPIO_2                       NRF_GPIO_PIN_MAP(0, 9)    /* GPIO_21 */
#define GPIO_3                       NRF_GPIO_PIN_MAP(0, 10)   /* GPIO_22 */

#define GPIO_13                      NRF_GPIO_PIN_MAP(0, 14)   /* GPIO_13 */
#define GPIO_16_SWO                  NRF_GPIO_PIN_MAP(0, 18)   /* GPIO_16/SWO */
#define GPIO_20_ADC1                 NRF_GPIO_PIN_MAP(0, 2)    /* GPIO_20/ADC1 */
#define GPIO_26_ADC2                 NRF_GPIO_PIN_MAP(0, 30)   /* GPIO_26/ADC2/GPIO_28 */
#define BQ_CD                        NRF_GPIO_PIN_MAP(0, 25)   /* GPIO_30/GPIO_31 */
#define GPIO_35                      NRF_GPIO_PIN_MAP(0, 19)   /* GPIO_35/GPIO_37 */
#define GPIO_RESET					 NRF_GPIO_PIN_MAP(0, 21)   /* GPIO_12*/
#define RX_PIN_NUMBER				 NRF_GPIO_PIN_MAP(0, 9)
#define TX_PIN_NUMBER				 NRF_GPIO_PIN_MAP(0, 20)

#define MAX_FILE_DESCRIPTORS          5

#define LED_BLINK_MAX_DELAY           (64)

#define NRF_CLOCK_INIT_FAILED         1
#define NRF_CLOCK_INIT_FAILED_MASK    (1 << NRF_CLOCK_INIT_FAILED)
#define NRF_POWER_INIT_FAILED         2
#define NRF_POWER_INIT_FAILED_MASK    (1 << NRF_POWER_INIT_FAILED)
#define NRF_FLASH_INIT_FAILED         3
#define NRF_FLASH_INIT_FAILED_MASK    (1 << NRF_FLASH_INIT_FAILED)
#define NRF_GPIO_INIT_FAILED          4
#define NRF_GPIO_INIT_FAILED_MASK     (1 << NRF_GPIO_INIT_FAILED)
#define NRF_RTC_INIT_FAILED           5
#define NRF_RTC_INIT_FAILED_MASK      (1 << NRF_RTC_INIT_FAILED)
#define NRF_SYSTICK_INIT_FAILED       6
#define NRF_SYSTICK_INIT_FAILED_MASK  (1 << NRF_SYSTICK_INIT_FAILED)
#define NRF_ADC_INIT_FAILED           7
#define NRF_ADC_INIT_FAILED_MASK      (1 << NRF_ADC_INIT_FAILED)
#define NRF_USB_INIT_FAILED           8
#define NRF_USB_INIT_FAILED_MASK      (1 << NRF_USB_INIT_FAILED)

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE                2060
#define MAX_PIN_NUM					  33	/* Max pins on the nRF52832 + 1 */
#endif

/**********************************************************************************/
/* functions */
/**********************************************************************************/
typedef void (*ISR_CB)(uint32_t pin, uint32_t polarity);
uint8_t pmic_pull_battery_level(void);
/*lint -e18 */
int opendir(const char *dirname);
char readdir(char* name);
int closedir(void);
#endif /* MCU_NICLA_H_ */
