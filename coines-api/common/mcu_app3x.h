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
 * @file    mcu_app3x.h
 * @date    Mar 1, 2021
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_APP3X_H_
#define MCU_APP3X_H_

#include "mcu_app3x_support.h"
#include "coines.h"

#include <sys/stat.h>

#define VBAT_MON_EN                   NRF_GPIO_PIN_MAP(0, 02)

#define MAX_FILE_DESCRIPTORS          5

#define CDC_ACM_COMM_INTERFACE        0
#define CDC_ACM_COMM_EPIN             NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE        1
#define CDC_ACM_DATA_EPIN             NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT            NRF_DRV_USBD_EPOUT1

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
#define NRF_TIMER_INIT_FAILED         9
#define NRF_TIMER_INIT_FAILED_MASK    (1 << NRF_TIMER_INIT_FAILED)

#define EEPROM_READ_LEN               (10)
#define EEPROM_CS_BYTE_INDEX          (9)

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE                2060
#endif

/**********************************************************************************/
/* functions */
/**********************************************************************************/
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);

/*lint -e778 -e845 -e746 -e785 -e446 -e734 */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_NONE);

#if defined(MCU_APP31)
uint8_t pmic_pull_battery_level(void);
void coines_get_device_ficr(uint64_t * devid);
#endif

#endif /* MCU_APP3X_H_ */