/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mcu_app3x_support.c
 * @date    Aug 01, 2022
 * @brief   COINES_SDK support file for mcu_app31.c
 */

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"
#include "mcu_app3x_support.h"

/**********************************************************************************/
/* own header files */
/**********************************************************************************/

/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/
#define RTC_PRESCALAR          0
#define RTC_COUNTER_BITS       24
#define RTC_TICKS_PER_SECOND   (32768 / (1 + RTC_PRESCALAR))
#define RTC_RESOLUTION_USEC    (1000000 / RTC_TICKS_PER_SECOND)
#define RTC_TICKS_TO_USEC(t)   (((uint64_t)t * UINT64_C(1000000)) / RTC_TICKS_PER_SECOND)

#define EEPROM_RW_RETRY_COUNT  10

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/
#if defined(MCU_APP31)
extern uint8_t pmic_pull_battery_level(void);
#endif

/**********************************************************************************/
/* static variables */
/**********************************************************************************/
/* Handle for RTC2 */
const nrfx_rtc_t rtc_handle = NRFX_RTC_INSTANCE(2);
volatile uint32_t rtc_count = 0;
static uint8_t volatile rtc_overflow = 0;
static volatile bool is_timer_enabled[COINES_TIMER_INSTANCE_MAX] = { false, false };

#if defined(MCU_APP30)
/* Timer instance loop-up table
NRFX_TIMER_INSTANCE(0) - used by softdevice
NRFX_TIMER_INSTANCE(1) - used by eeprom
NRFX_TIMER_INSTANCE(3) - used for timestamp
*/
static const nrfx_timer_t timer_instance[COINES_TIMER_INSTANCE_MAX] = {
                                            #if NRFX_TIMER2_ENABLED
    NRFX_TIMER_INSTANCE(2),
                                            #endif/* NRFX_TIMER3_ENABLED */
                                            #if NRFX_TIMER4_ENABLED
    NRFX_TIMER_INSTANCE(4),
                                            #endif/* NRFX_TIMER4_ENABLED */
    /*lint -e133*/
};
#else
/* Timer instance loop-up table
NRFX_TIMER_INSTANCE(0) - used by softdevice
NRFX_TIMER_INSTANCE(1) - used by eeprom
NRFX_TIMER_INSTANCE(3) - used for pmic periodic reading
NRFX_TIMER_INSTANCE(4) - used for timestamp
*/
static const nrfx_timer_t timer_instance[COINES_TIMER_INSTANCE_MAX] = {
                                            #if NRFX_TIMER2_ENABLED
    NRFX_TIMER_INSTANCE(2),
                                            #endif/* NRFX_TIMER2_ENABLED */
                                            #if NRFX_TIMER3_ENABLED
    NRFX_TIMER_INSTANCE(3),
                                            #endif/* NRFX_TIMER3_ENABLED */
};
#endif

/* Timer configuration */
static nrfx_timer_config_t timer_config = {
    .frequency = NRF_TIMER_FREQ_1MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = 3, .p_context = NULL
};

/* Timer cc channel lookup table */
static uint32_t timer_cc_channel[6] =
{ NRF_TIMER_TASK_CAPTURE0, NRF_TIMER_TASK_CAPTURE1, NRF_TIMER_TASK_CAPTURE2, NRF_TIMER_TASK_CAPTURE3,
  NRF_TIMER_TASK_CAPTURE4, NRF_TIMER_TASK_CAPTURE5 };

/* Timer cc channel start index for timestamp capture event */
/* timer_cc_channel_no -> 0 used for compare event(overflow) */
/* timer_cc_channel_no -> 1 to 5 used for timestamp capture events */
static uint8_t timer_cc_channel_no = 1;

/* Holds timer overflow count */
extern volatile uint32_t timer_overflow_count;

/* Holds the allocated ppi channel */
static nrf_ppi_channel_t ppi_channel;

static ISR_CB isr_cb[COINES_SHUTTLE_PIN_MAX];

bool int_pin_usage_native_emulated[COINES_SHUTTLE_PIN_MAX] = { false };

struct coines_timed_interrupt_config timed_interrupt_config[COINES_SHUTTLE_PIN_MAX];

extern const nrfx_timer_t free_running_timer_instance;

extern uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX];

static nrfx_gpiote_in_config_t gpio_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);

/* Array to map corresponding COINES_SDK polarity states to nRF polarity states*/
/*lint -e26 */
uint8_t map_nrfpol_to_coinespol[COINES_PIN_INTERRUPT_MODE_MAXIMUM] = {
    0, [NRF_GPIOTE_POLARITY_LOTOHI] = COINES_PIN_INT_POLARITY_LOW_TO_HIGH,
    [NRF_GPIOTE_POLARITY_HITOLO] = COINES_PIN_INT_POLARITY_HIGH_TO_LOW,
    [NRF_GPIOTE_POLARITY_TOGGLE] = COINES_PIN_INT_POLARITY_TOGGLE
};

static void gpiohandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

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
static uint16_t get_shuttle_id()
{
    uint16_t shuttle_id = 0;

    NRFX_IRQ_DISABLE(USBD_IRQn);
    (void)app30_eeprom_read(0x01, (uint8_t *)&shuttle_id, 2);
    NRFX_IRQ_ENABLE(USBD_IRQn);

    return shuttle_id;
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

            break;
        case NRF_DRV_RTC_INT_OVERFLOW:
            /* RTC overflow event */
            rtc_overflow = (rtc_overflow != 255) ? (rtc_overflow + 1) : 0;
            break;

        case NRF_DRV_RTC_INT_TICK:
            /* RTC tick event */
            rtc_count = (rtc_overflow << RTC_COUNTER_BITS) | nrf_drv_rtc_counter_get(&rtc_handle);
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

    return COINES_SUCCESS;
}

void coines_deconfig_i2s_bus()
{
    nrf_drv_i2s_stop();
    nrfx_i2s_uninit();
}

/*!
 *  @brief This API is used for introducing a delay in milliseconds
 */
void coines_delay_msec(uint32_t delay_ms)
{
    nrf_delay_ms(delay_ms);
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 */
void coines_delay_usec(uint32_t delay_us)
{
    nrf_delay_us(delay_us);
}

/*!
 * @brief This API is used to trigger the timer in firmware and enable or disable system time stamp
 */
int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{
    (void)tmr_cfg;
    (void)ts_cfg;

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
    APP_START_ADDR = APP_START_ADDRESS; /* Application start address; */

    NVIC_SystemReset();
}

/*!
 *  @brief This API is used to get the board information.
 */
int16_t coines_get_board_info(struct coines_board_info *data)
{

    if (data != NULL)
    {
#if defined(MCU_APP30)
        data->board = 5;
#elif defined(MCU_APP31)
        data->board = 9;
#else
        data->board = 0xFE;
#endif
        data->hardware_id = 0x11;
        data->shuttle_id = get_shuttle_id();
        data->software_id = 0x10;
#if (defined(MCU_APP30)||defined(MCU_APP31))
        if (app30_eeprom_romid(data->eeprom_id))
        {
            return COINES_SUCCESS;
        }
        else
        {
            return COINES_E_FAILURE;
        }
#else
        return COINES_SUCCESS;
#endif
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/*
 * @brief Function for converting battery voltage to percentage.
 *
 * @details This is just an estimated percentage considering Maximum charging voltage as 4.2 and cut-off voltage as 3.0.
 *          It will vary between different batteries
 *
 * @param[in] mvolts voltage(in milli volts) to be converted into percentage.
 *
 * @retval battery level in percentage.
 */
uint8_t battery_level_in_percentage(const uint16_t mvolts)
{
    float output_volt;
    uint8_t battery_level;

    const float battery_max = 4.200f; /*maximum voltage of battery */
    const float battery_min = 3.000f; /*minimum voltage of battery before shutdown */
    float input_volt = mvolts;

    output_volt = (((input_volt / 1000.0f) - battery_min) / (battery_max - battery_min)) * 100.0f;
    battery_level = (uint8_t)output_volt;

    return battery_level;
}
/*!
 * @brief  Callback to read battery voltage
 *
 * @return None
 */
void bat_status_read_callback(void)
{
#if defined(MCU_APP30)
    (void)nrfx_saadc_sample();
#else
    (void) ble_service_battery_level_update(pmic_pull_battery_level(), 1);
#endif
}

/*!
 *
 * @brief       : USB event callback handler
 *
 * @param[in]   : type of usb event
 *
 * @return      : None
 */
void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }

            break;
        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        default:
            break;
    }
}

/*!
 * @brief This API is used to get the current counter(RTC) reference time in usec
 *
 * @param[in]   : None
 * @return      : counter(RTC) reference time in usec
 * */
uint32_t coines_get_realtime_usec(void)
{
    return (uint32_t)RTC_TICKS_TO_USEC(rtc_count);
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
    uint32_t tick_count;
    uint32_t num_ticks;

    /*lint -e653 -e524 */
    num_ticks = (period < RTC_RESOLUTION_USEC) ? 1 : (uint32_t)roundf(((float)period / RTC_RESOLUTION_USEC));

    tick_count = ((rtc_overflow << RTC_COUNTER_BITS) | nrf_drv_rtc_counter_get(&rtc_handle));

    while ((((rtc_overflow << RTC_COUNTER_BITS) | nrf_drv_rtc_counter_get(&rtc_handle)) - tick_count) < num_ticks)
        ;
}

/*!
 * @brief This API is used to config RTC
 */
uint32_t rtc_config(void)
{
    uint32_t err_code;

    /*lint -e1786 -e778 */
    nrfx_rtc_config_t rtc_config_struct = NRFX_RTC_DEFAULT_CONFIG;

    /* Configure the prescaler to generate ticks for a specific time unit */
    /* if prescalar = 1, tick resolution =  32768 / (1 + 1) = 16384Hz = 61.035us */
    rtc_config_struct.prescaler = RTC_PRESCALAR;

    /* Initialize the RTC and pass the configurations along with the interrupt handler */
    err_code = nrfx_rtc_init(&rtc_handle, &rtc_config_struct, rtc_handler);
    if (err_code == NRFX_SUCCESS)
    {
        /* Generate a tick event on each tick */
        nrfx_rtc_tick_enable(&rtc_handle, true);

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
        if (!is_timer_enabled[instance])
        {
            /*lint -e64*/
            if (nrfx_timer_init(&timer_instance[instance], &timer_config, handler) != NRFX_SUCCESS)
            {
                return COINES_E_TIMER_INIT_FAILED;
            }

            is_timer_enabled[instance] = true;
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
 * @brief This API is used to deconfig the hardware timer in firmware
 */
int16_t coines_timer_deconfig(enum coines_timer_instance instance)
{
    if (instance < COINES_TIMER_INSTANCE_MAX)
    {
        if (is_timer_enabled[instance])
        {
            nrfx_timer_uninit(&timer_instance[instance]);
            is_timer_enabled[instance] = false;
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
 * @brief This API is used to write the content into shuttle eeprom
 */
int16_t coines_shuttle_eeprom_write(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t retval = COINES_SUCCESS;
    uint8_t retry_count = EEPROM_RW_RETRY_COUNT;
    bool is_write_success = false;

    if (buffer == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    if ((length < 1) || (length > EEPROM_RW_CONTENT_SIZE))
    {
        return COINES_E_INVALID_EEPROM_RW_LENGTH;
    }

    do
    {
        if (app30_eeprom_write((uint8_t)start_addr, buffer, (uint8_t)length))
        {
            is_write_success = true;
            break;
        }
    }while (retry_count--);

    if (!is_write_success)
    {
        retval = COINES_E_EEPROM_WRITE_FAILED;
    }

    return retval;
}

/*!
 * @brief This API is used to read the content from the shuttle eeprom
 */
int16_t coines_shuttle_eeprom_read(uint16_t start_addr, uint8_t *buffer, uint16_t length)
{
    int16_t retval = COINES_SUCCESS;
    uint8_t retry_count = EEPROM_RW_RETRY_COUNT;
    bool is_read_success = false;

    if (buffer == NULL)
    {
        return COINES_E_NULL_PTR;
    }

    if ((length < 1) || (length > EEPROM_RW_CONTENT_SIZE))
    {
        return COINES_E_INVALID_EEPROM_RW_LENGTH;
    }

    do
    {
        if (app30_eeprom_read(start_addr, buffer, (uint8_t)length))
        {
            is_read_success = true;
            break;
        }
    }while (retry_count--);

    if (!is_read_success)
    {
        retval = COINES_E_EEPROM_READ_FAILED;
    }

    return retval;

}

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                             void (*interrupt_cb)(uint32_t, uint32_t),
                             enum coines_pin_interrupt_mode int_mode)
{
    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return;
    }

    if (int_mode == COINES_PIN_INTERRUPT_CHANGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    }

    if (int_mode == COINES_PIN_INTERRUPT_RISING_EDGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    }

    if (int_mode == COINES_PIN_INTERRUPT_FALLING_EDGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    }

    (void)nrfx_gpiote_in_init(pin_num, &gpio_config, gpiohandler);
    nrfx_gpiote_in_event_enable(pin_num, true);
    isr_cb[pin_number] = interrupt_cb;
    int_pin_usage_native_emulated[pin_number] = true;

}

/*!
 *
 * @brief Detaches a interrupt from a Multi-IO pin
 *
 */
void coines_detach_interrupt(enum coines_multi_io_pin pin_number)
{
    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return;
    }

    /* Cleanup */
    isr_cb[pin_number] = NULL;
    int_pin_usage_native_emulated[pin_number] = false;
    nrfx_gpiote_in_uninit(pin_num);
    nrfx_gpiote_in_event_disable(pin_num);
}

/*!
 * @brief Read multiio index from the pin_number
 */
static enum coines_multi_io_pin get_multiio_pin(uint32_t pin_number)
{
    uint8_t i;

    if (pin_number != 0xff)
    {
        for (i = 0; i < COINES_SHUTTLE_PIN_MAX; i++)
        {
            if ((pin_number == multi_io_map[i]) && int_pin_usage_native_emulated[i])
            {
                return (enum coines_multi_io_pin)i;
            }
        }
    }

    return COINES_SHUTTLE_PIN_MAX;
}

/*!
 * @brief GPIOTE event handler for timed interrupt
 */
static void attach_timed_interrupt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)action;
    uint64_t total_ticks;
    uint32_t latest_ticks_count;

    enum coines_multi_io_pin pin_number;

    pin_number = get_multiio_pin((uint32_t)pin);

    if (pin_number == COINES_SHUTTLE_PIN_MAX)
    {
        return;
    }

    /* Get the captured ticks from the timer */
    latest_ticks_count = nrfx_timer_capture_get(&free_running_timer_instance,
                                                (nrf_timer_cc_channel_t)timed_interrupt_config[pin_number].timer_cc_channel);

    total_ticks = ((uint64_t)timer_overflow_count << 32) | latest_ticks_count;

    /* Callback with timestamp */
    timed_interrupt_config[pin_number].cb(TIMER_TICKS_TO_NSEC(total_ticks),
                                          (uint32_t)pin_number,
                                          (uint32_t)map_nrfpol_to_coinespol[action]);

}

/*!
 * @brief Attaches a timed interrupt to a Multi-IO pin
 */
int16_t coines_attach_timed_interrupt(enum coines_multi_io_pin pin_number,
                                      timed_interrupt_cb interrupt_cb,
                                      enum coines_pin_interrupt_mode int_mode)
{
    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_INVALID_PIN_NUMBER;
    }

    if (int_mode == COINES_PIN_INTERRUPT_CHANGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    }

    if (int_mode == COINES_PIN_INTERRUPT_RISING_EDGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    }

    if (int_mode == COINES_PIN_INTERRUPT_FALLING_EDGE)
    {
        gpio_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    }

    (void)nrfx_gpiote_in_init(pin_num, &gpio_config, attach_timed_interrupt_handler);
    nrfx_gpiote_in_event_enable(pin_num, true);

    if (NRFX_SUCCESS == nrfx_ppi_channel_alloc(&ppi_channel))
    {
        if (NRFX_SUCCESS ==
            nrfx_ppi_channel_assign(ppi_channel, nrfx_gpiote_in_event_addr_get(pin_num),
                                    nrfx_timer_task_address_get(&free_running_timer_instance,
                                                                (nrf_timer_task_t)timer_cc_channel[timer_cc_channel_no])))
        {
            if (NRFX_SUCCESS != nrfx_ppi_channel_enable(ppi_channel))
            {
                return COINES_E_CHANNEL_ENABLE_FAILED;
            }
        }
        else
        {
            return COINES_E_CHANNEL_ASSIGN_FAILED;
        }
    }
    else
    {
        return COINES_E_CHANNEL_ALLOCATION_FAILED;
    }

    /* Allocate one cc channel for each timed interrupt config */
    if (timer_cc_channel_no <= NRF_TIMER_CC_CHANNEL5)
    {
        timed_interrupt_config[pin_number].timer_cc_channel = timer_cc_channel_no++;
    }
    else
    {
        return COINES_E_TIMER_CC_CHANNEL_NOT_AVAILABLE;
    }

    timed_interrupt_config[pin_number].ppi_channel = ppi_channel;
    timed_interrupt_config[pin_number].cb = interrupt_cb;
    int_pin_usage_native_emulated[pin_number] = true;

    return COINES_SUCCESS;

}

/*!
 * @brief Detaches a timed interrupt from a Multi-IO pin
 */
int16_t coines_detach_timed_interrupt(enum coines_multi_io_pin pin_number)
{
    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_INVALID_PIN_NUMBER;
    }

    /* Cleanup */
    nrfx_gpiote_in_uninit(pin_num);
    nrfx_gpiote_in_event_disable(pin_num);

    int_pin_usage_native_emulated[pin_number] = false;

    if (NRFX_SUCCESS == nrfx_ppi_channel_disable(timed_interrupt_config[pin_number].ppi_channel))
    {
        if (NRFX_SUCCESS != nrfx_ppi_channel_free(timed_interrupt_config[pin_number].ppi_channel))
        {
            return COINES_E_CHANNEL_DISABLE_FAILED;
        }
    }
    else
    {
        return COINES_E_CHANNEL_DEALLOCATION_FAILED;
    }

    memset(&timed_interrupt_config[pin_number], 0, sizeof(struct coines_timed_interrupt_config));

    if (timer_cc_channel_no)
    {
        timer_cc_channel_no--;
    }

    return COINES_SUCCESS;

}

/*!
 * @brief GPIO Interrupt handler
 */
static void gpiohandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    for (int i = 0; i < COINES_SHUTTLE_PIN_MAX; i++)
    {
        if (pin == multi_io_map[i] && isr_cb[i] != NULL)
        {
            isr_cb[i]((uint32_t)i, (uint32_t)map_nrfpol_to_coinespol[action]);
            break;
        }
    }
}

/** @}*/
