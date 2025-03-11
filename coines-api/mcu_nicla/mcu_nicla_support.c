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
 * @file    mcu_nicla_support.c
 * @date    May 10, 2022
 * @brief   COINES_SDK support file for mcu_nicla.c
 */

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"
#include "mcu_nicla.h"
#include "mcu_nicla_support.h"

/**********************************************************************************/
/* own header files */
/**********************************************************************************/

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define RTC_PRESCALAR         0
#define RTC_COUNTER_BITS      24
#define RTC_TICKS_PER_SECOND  (32768 / (1 + RTC_PRESCALAR))
// #define RTC_RESOLUTION_USEC   (1000000 / RTC_TICKS_PER_SECOND)
#define RTC_TICKS_TO_USEC(t)  (((uint64_t)t * UINT64_C(1000000)) / RTC_TICKS_PER_SECOND)

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
uint32_t app_start_address = APP_START_ADDRESS;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/* Handle for RTC2 */
/*lint -e64*/
const nrfx_rtc_t rtc_handle = NRFX_RTC_INSTANCE(2);
static volatile uint32_t rtc_overflow = 0;

/* PMIC device instance for battery voltage reading*/
extern struct led_dev led_dev;
volatile uint8_t batt_status_percentage = 0;


/* Timer instance loop-up table
NRFX_TIMER_INSTANCE(0) - used by softdevice
NRFX_TIMER_INSTANCE(1) - used by eeprom
NRFX_TIMER_INSTANCE(3) - used by pmic
NRFX_TIMER_INSTANCE(4) - used for button event
*/
static const nrfx_timer_t timer_instance[COINES_TIMER_INSTANCE_MAX] = {
                                            #if NRFX_TIMER2_ENABLED
    NRFX_TIMER_INSTANCE(2),
                                            #endif/* NRFX_TIMER2_ENABLED */
                                            #if NRFX_TIMER3_ENABLED
    NRFX_TIMER_INSTANCE(3),
                                            #endif/* NRFX_TIMER3_ENABLED */
                                            #if NRFX_TIMER4_ENABLED
    NRFX_TIMER_INSTANCE(4),
                                            #endif/* NRFX_TIMER4_ENABLED */
	/*lint -e133*/
};

/* Timer configuration */
static nrfx_timer_config_t timer_config = {
    .frequency = NRF_TIMER_FREQ_1MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = 4, .p_context = NULL
};

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

/*!
 *
 * @brief       : API to get shuttle ID
 *
 * @param[in]   : None
 * @return      : shuttle id
 */
static uint16_t get_shuttle_id(void)
{
    return 0x159; /* BHI260AP */
}

/*!
 * @brief This API is RTC interrupt handler which will be used to handle the interrupt events
 */
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{

    switch (int_type)
    {
        case NRF_DRV_RTC_INT_COMPARE0:
        case NRF_DRV_RTC_INT_COMPARE1:
        case NRF_DRV_RTC_INT_COMPARE2:
        case NRF_DRV_RTC_INT_COMPARE3:
        case NRF_DRV_RTC_INT_TICK:

            break;
        case NRF_DRV_RTC_INT_OVERFLOW:
            /* RTC overflow event */
            rtc_overflow++;
            break;

        default:
            /* Unknown RTC event. Added for completeness */
            break;
    }
}

/**********************************************************************************/
/* functions */
/**********************************************************************************/

/*!
 * @brief This API is used to close the active communication(USB,COM or BLE).
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    (void)intf_type;
    (void)arg;

    fs_deinit();
    return led_deinit(&led_dev);
}

/**
 * @brief This API is used to stop the I2S/TDM interface from reading data from the sensor
 */
void coines_deconfig_i2s_bus()
{
    nrf_drv_i2s_stop();
    nrfx_i2s_uninit();
}


/*!
 *  @brief This API is used for introducing a delay in milliseconds
 */
void coines_delay_msec(uint32_t delay_millisec)
{
    nrf_delay_ms(delay_millisec);
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 */
void coines_delay_usec(uint32_t delay_microsec)
{
    nrf_delay_us(delay_microsec);
}

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 */
int16_t coines_trigger_timer(enum coines_timer_config timer_cfg, enum coines_time_stamp_config timestamp_cfg)
{
    (void)timer_cfg;
    (void)timestamp_cfg;

    return COINES_E_NOT_SUPPORTED;
}

/*!
 * @brief Get COINES_SDK library version
 *
 * @return pointer to version string
 */
const char* coines_get_version()
{
    return COINES_VERSION;
}

/*!
 * @brief Resets the device
 *
 * @note  After reset device jumps to the address specified in makefile (APP_START_ADDRESS).
 *
 * @return void
 */
void coines_soft_reset(void)
{
    memcpy((uint32_t *)MAGIC_LOCATION, "COIN", 4); /* *MAGIC_LOCATION = 0x4E494F43; // 'N','O','I','C' */
    APP_START_ADDR = app_start_address; /* Application start address; */

    NVIC_SystemReset();
}

/*!
 *  @brief This API is used to get the board information.
 */
int16_t coines_get_board_info(struct coines_board_info *data)
{

    if (data != NULL)
    {
        data->board = 8; /* Arduino Nicla Sense */
        data->hardware_id = 0x11;
        data->shuttle_id = get_shuttle_id();
        data->software_id = 0x10;

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/*!
 * @brief  Callback to read battery voltage
 *
 * @return None
 */
void bat_status_read_callback(void)
{
	(void) ble_service_battery_level_update(pmic_pull_battery_level(), 1);
}

static uint64_t get_rtc_ticks(void)
{
    return (uint64_t)nrf_drv_rtc_counter_get(&rtc_handle) + ((uint64_t)rtc_overflow << RTC_COUNTER_BITS);
}

/*!
 * @brief This API is used to get the current counter(RTC) reference time in usec
 *
 * @param[in]   : None
 * @return      : counter(RTC) reference time in usec
 * */
uint64_t coines_get_realtime_usec(void)
{
    return RTC_TICKS_TO_USEC(get_rtc_ticks());
}

/*!
 * @brief This API is used to introduce delay based on high precision RTC(LFCLK crystal)
 * with the resolution of of 30.517 usec
 *
 * @param[in]   : required delay in microseconds
 * @return      : None
 */
void coines_delay_realtime_usec(uint32_t period)
{
    uint64_t end_time = coines_get_realtime_usec() + period;

    while (coines_get_realtime_usec() < end_time)
    {
        /* Do nothing */
    }
}

/*!
 * @brief This API is used to config RTC
 */
uint32_t rtc_config(void)
{
    uint32_t err_code;
    /*lint -e1786 -e778 */
    nrfx_rtc_config_t rtc_conf = NRFX_RTC_DEFAULT_CONFIG;

    /* Configure the prescaler to generate ticks for a specific time unit */
    /* if prescalar = 1, tick resolution =  32768 / (1 + 1) = 16384Hz = 61.035us */
    rtc_conf.prescaler = RTC_PRESCALAR;

    /* Initialize the RTC and pass the configurations along with the interrupt handler */
    err_code = nrfx_rtc_init(&rtc_handle, &rtc_conf, rtc_handler);
    if (err_code == NRFX_SUCCESS)
    {
        nrfx_rtc_overflow_enable(&rtc_handle, true);

        /* start the RTC */
        nrfx_rtc_enable(&rtc_handle);
    }

    return err_code;
}

/*!
 * @brief This API is used to config the hardware timer in firmware
 */
int16_t coines_timer_config(enum coines_timer_instance instance, void* handler)
{
    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        if (nrfx_timer_init(&timer_instance[instance], &timer_config, (nrfx_timer_event_handler_t)handler) != NRFX_SUCCESS)//lint !e611
        {
            return COINES_E_TIMER_INIT_FAILED;
        }
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to start the hardware timer in firmware
 */
int16_t coines_timer_start(enum coines_timer_instance instance, uint32_t timeout)
{

    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        nrfx_timer_extended_compare(&timer_instance[instance],
                                    NRF_TIMER_CC_CHANNEL0,
									nrfx_timer_us_to_ticks(&timer_instance[instance], timeout),
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    (bool)true);
        nrfx_timer_enable(&timer_instance[instance]);
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief This API is used to start the hardware timer in firmware
 */
int16_t coines_timer_stop(enum coines_timer_instance instance)
{
    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        if (nrfx_timer_is_enabled(&timer_instance[instance]))
        {
            nrfx_timer_disable(&timer_instance[instance]);
        }
    }
    else
    {
        return COINES_E_TIMER_INVALID_INSTANCE;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief Attaches a timed interrupt to a Multi-IO pin
 */
int16_t coines_attach_timed_interrupt(enum coines_multi_io_pin pin_number,
                                      void (*timed_interrupt_cb)(uint64_t,
                                                                 uint32_t,
                                                                 uint32_t),
                                      enum coines_pin_interrupt_mode int_mode)
{
	(void)pin_number;
	(void)int_mode;
	(void)timed_interrupt_cb;
    return COINES_E_FAILURE;
}

/*!
 * @brief Detaches a timed interrupt from a Multi-IO pin
 */
int16_t coines_detach_timed_interrupt(enum coines_multi_io_pin pin_number)
{
	(void)pin_number;
    return COINES_E_FAILURE;
}

/*!
 * @brief This API is used to write the content into shuttle eeprom
 */
int16_t coines_shuttle_eeprom_write(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{ 
	(void)start_addr;
	(void)buffer;
	(void)length;
    return COINES_E_EEPROM_WRITE_FAILED;
}

/*!
 * @brief This API is used to read the content from shuttle eeprom
 */
int16_t coines_shuttle_eeprom_read(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{  
	(void)start_addr;
	(void)buffer;
	(void)length;
    return COINES_E_EEPROM_READ_FAILED;
}

/*!
 * @brief Configuring p21 as normal GPIO
 */
void reconfig_reset_pin(void)
{
	if (NRF_UICR->PSELRESET[0] != 0xFFFFFFFF || NRF_UICR->PSELRESET[1] !=  0xFFFFFFFF)
	{
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->PSELRESET[0] =  0xFFFFFFFF;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->PSELRESET[1] =  0xFFFFFFFF;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NVIC_SystemReset();
	}
}

/*!
 * @brief Configure NFCT pins as GPIOs. UART RX is assigned to P0.9, same pin that is used as the antenna input for NFC by default
 */
void reconfig_nfct_pin(void)
{
	if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
		NVIC_SystemReset();
	}
}
/** @}*/
