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
 * @file    pmic_common.c
 * @date    July 12, 2023
 * @brief   PMIC driver for BLD and MTP
 *
 */

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include "pmic_common.h"

/*GPIO interrupt default configuration*/
static nrfx_gpiote_in_config_t gpio_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);

/*Timer instance*/
static const nrfx_timer_t timer_instance[1] = {
    NRFX_TIMER_INSTANCE(3)
};

/* Timer configuration */
static nrfx_timer_config_t timer_config = {
    .frequency = NRF_TIMER_FREQ_1MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
    .interrupt_priority = 7, .p_context = NULL
};

/*PMIC device handler*/
struct bq_dev pmic_dev =
{ .cd_pin_state = 1, .i2c_address = BQ_DRV_ADDR, .read = common_i2c_read, .write = common_i2c_write,
  .delay_ms = common_delay_ms, .cd_set = common_pmic_cd_set, .battery_connected = 0 };
static volatile uint8_t pmic_battery_percent = 0;
volatile uint16_t batt_status_in_milli_volts = 0;
/*Prototypes*/
void pmic_cyclic_reading(void);
void pmic_check_battery_and_faults(struct bq_dev *dev, struct fault_mask_reg *fault_state);
uint8_t battery_level_in_percentage(const uint16_t mvolts);
void gpio_input_config(void);

/*!
 * @brief Callback for VIN detection
 */
void vin_detection_cb(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    uint8_t pinvalue;

  
	pinvalue = nrf_gpio_pin_read(VIN_DEC);
    if (pinvalue > 0)
    {
        /* Vin plugged => Activate charging */
        (void)bq_charge_enable(&pmic_dev);
    }
    else if (pinvalue == 0)
    {
        /* Vin unplugged => Deactivate charging */
        (void)bq_charge_disable(&pmic_dev);
    }
}

/*!
 * @brief Callback for INT interrupt
 */
void power_interrupt_cb(uint32_t param1, uint32_t param2)
{
    /*
     * 1 : Wake 2 event occured : Press > 1500ms
     * 2 : Wake 1 event occured : Press > 80ms
     * 3 : Both Wake 1 and Wake 2 occured
     */
    (void)param1;
    (void)param2;
    uint8_t event = 0xFF;
    struct fault_mask_reg faults;
    uint8_t pinvalue;

    /*Acknowledge button press*/
    (void)bq_ack_push_button_events(&pmic_dev, &event);
    /*Acknowledge faults*/
    //(void)bq_get_faults(&pmic_dev, &faults);
    /*Read Vin pin*/
    pinvalue = nrf_gpio_pin_read(VIN_DEC);
    
    switch (event)
    {
        case 1: /*Activate Ship mode*/
            if (pinvalue == 0)
            {
                (void)bq_cd_set(&pmic_dev, 1);
                (void)bq_delay_ms(&pmic_dev, 10);
                (void)bq_set_mode_config(&pmic_dev, BQ_SHIP_MODE_ENABLE);
            }
            break;
        case 2: /*NTD*/
            /*Let the user chose*/
            break;
        case 3:/*NTD*/
            /*This case never been reached*/
            break;
        default:
            break;
    }

    if((pmic_dev.battery_connected)&&(pinvalue == 1))
    {    /*Check battery and get faults*/
        pmic_check_battery_and_faults(&pmic_dev, &faults);
    }
}
/*!
 * @brief This API is used to check battery and faults
 */
void pmic_check_battery_and_faults(struct bq_dev *dev, struct fault_mask_reg *fault_state)
{
    struct bq_term_precharge_config prechrg_cfg;
    uint8_t pinvalue;    

    /*Acknowledge faults*/
    (void)bq_get_faults(&pmic_dev, fault_state);

    pinvalue = nrf_gpio_pin_read(VIN_DEC);
    if ((pinvalue == 1)&&(fault_state->bat_uvlo == 1)) /*Battery not connected*/
    {
        /*Disable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = BQ_ITERM_DISABLE;
        (void)bq_set_term_precharge_current(&pmic_dev, prechrg_cfg);
        //dev->battery_connected = 0;
    }
    else if ((pinvalue == 1)&&(fault_state->bat_uvlo == 0)) /*Battery connected*/
    {
        /*Enable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = BQ_ITERM_DISABLE;
        (void)bq_set_term_precharge_current(&pmic_dev, prechrg_cfg);
        //dev->battery_connected = 1;
    }; 
}
/*!
 * @brief Callback for pmic reading Timer Instance 1
 */
void pmic_cyclic_reading(void)
{
    uint8_t bat_vbbm = 0;
    int8_t rslt;
    struct fault_mask_reg faults;

    /*update battery percentage*/
    rslt = bq_get_battery_voltage(&pmic_dev, &bat_vbbm); /*Voltage Based Battery Monitor */
    if (BQ_OK == rslt)
    {
        batt_status_in_milli_volts = (uint16_t) (((bat_vbbm * 4.2) / 100.0) * 1000.0); /*Convert VBMON percentage to
                                                                                        * voltage => Volt = VBMON *
                                                                                        * VBATREG */
        pmic_battery_percent = battery_level_in_percentage(batt_status_in_milli_volts);
    }

    /*Check battery and get faults*/
    if(pmic_dev.battery_connected == 0)
    {
        pmic_check_battery_and_faults(&pmic_dev, &faults);
    }
}

/*!
 * @brief  This function initialize pmic feature
 */
void pmic_control_config(void)
{
	/*I2C Bus init*/
	(void)common_i2c_init();
	
	/*CD line init*/
	(void)common_pmic_cd_init();
	
	/*PMIC configuration*/
    common_delay_ms(10);
    (void)bq_init(&pmic_dev);
    common_delay_ms(10);

	/*Power_int and Vin_detect config*/
	gpio_input_config();
	
	/*Check Vin presence*/
	uint8_t pinvalue;
	pinvalue = nrf_gpio_pin_read(VIN_DEC);
    if (pinvalue == 0)
    {
        (void)bq_charge_disable(&pmic_dev);
        (void)bq_cd_set(&pmic_dev, 0);
        common_delay_ms(10);
        (void)bq_cd_set(&pmic_dev, 1);
        common_delay_ms(10);
        
    }
    else
    {
		(void)bq_charge_enable(&pmic_dev);
    }
	
	/*Initiate a battery reading*/
    pmic_cyclic_reading();

    /* Initialize Timer for periodical pmic reg reading every 30 seconds to prevent watchdog reset*/
	if (nrfx_timer_init(&timer_instance[0], &timer_config, pmic_cyclic_reading) != NRFX_SUCCESS)
	{
		/*NTD*/
	}
	else
	{
		nrfx_timer_extended_compare(&timer_instance[0],
							NRF_TIMER_CC_CHANNEL0,
							nrfx_timer_us_to_ticks(&timer_instance[0], 30000000),
							NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
							(bool)true);
        nrfx_timer_enable(&timer_instance[0]);
	}
	
    /*Acknowledge wake up interrupt when exiting ship mode*/
    uint8_t ButtonAck=0;
    (void)bq_ack_push_button_events(&pmic_dev, &ButtonAck);
    //struct fault_mask_reg faults;
    //pmic_check_battery_and_faults(&pmic_dev, &faults);
}

void gpio_input_config(void)
{
	if (NRFX_SUCCESS != nrfx_gpiote_init())
    {
        
    }
	
	/*Config input gpios*/
	nrf_gpio_cfg_input(VIN_DEC, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(P_INT, NRF_GPIO_PIN_PULLUP);
	
	/*Attach Vin_dec interrupt*/
	gpio_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    gpio_config.pull = NRF_GPIO_PIN_PULLDOWN;
	(void)nrfx_gpiote_in_init(VIN_DEC, &gpio_config, vin_detection_cb);
    nrfx_gpiote_in_event_enable(VIN_DEC, true);
	
	/*Attach INT interrupt*/
	gpio_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    gpio_config.pull = NRF_GPIO_PIN_PULLUP;
	(void)nrfx_gpiote_in_init(P_INT, &gpio_config, power_interrupt_cb);
    nrfx_gpiote_in_event_enable(P_INT, true);
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