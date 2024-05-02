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
 * @file    mcu_app3x.c
 * @date    Aug 01, 2022
 * @brief   COINES_SDK support file for APP3.1 MCU
 *
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include "mcu_app3x.h"
#include "mcu_app3x_interface.h"

#define TMP112Q1_DEV_ADDRESS           0x48 /*Device address of TMP112Q1*/
#define TMP112Q1_REG_ADDRESS           0x00 /*reg address of TMP112Q1*/

#if defined(MCU_APP30)
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  3600 /**< Reference voltage (in milli volts) used by ADC while doing conversion.\
                                                    internal reference is .6V and gain is 1/6 hence reference voltage is .6/(1/6)=3600*/
#define ADC_RES_10BIT                  1024 /**< Maximum digital value for 10-bit ADC\
 * conversion. */

#define BATTERY_VOLTAGE_SCALE_FACTOR   2.67   /**<Factor to convert output voltage of voltage divider to actual battery\
 * voltage R1 is 300kohm and R2 is 180kohm*/

/**Macro to convert the result of ADC conversion in millivolts */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    (((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT)

#endif

#define CPU_FREQ_HZ                    64000000   /**<ARM cortex M4 is running at 64Mhz*/
#define BLE_MTU                        244   /*Maximum payload for BLE com*/

volatile bool serial_connected = false;
static uint8_t serial_buffer[RX_BUFFER_SIZE] = { 0 };
static volatile uint16_t serial_idx = 0, read_idx = 0;
static uint8_t temp_rx;

volatile bool ble_nus_connected = false;
volatile bool ble_bas_connected = false;
extern volatile size_t ble_nus_available;
uint32_t baud_rate = 0;
volatile uint32_t g_millis = 0;

volatile bool tx_pending = false;
volatile uint8_t batt_status_percentage = 0;
volatile uint16_t batt_status_in_milli_volts = 0;
volatile bool coines_adc_transfer_done = false;
static int8_t battery_meas_rslt = COINES_SUCCESS;
char *ble_device_name;
enum coines_tx_power ble_tx_power = COINES_TX_POWER_0_DBM;

/*lint -e26 */
#if defined(MCU_APP30)
uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX] = {
    [COINES_SHUTTLE_PIN_9] = GPIO_1, [COINES_SHUTTLE_PIN_14] = GPIO_4, [COINES_SHUTTLE_PIN_15] = GPIO_5,
    [COINES_SHUTTLE_PIN_16] = GPIO_SDI, [COINES_SHUTTLE_PIN_22] = GPIO_7, [COINES_SHUTTLE_PIN_8] = GPIO_0,
    [COINES_SHUTTLE_PIN_20] = GPIO_2, [COINES_SHUTTLE_PIN_21] = GPIO_3, [COINES_SHUTTLE_PIN_19] = GPIO_6,
    [COINES_SHUTTLE_PIN_7] = GPIO_CS, 0, 0, 0, 0, 0, 0,

    /* Native APP3.0 pins */
    [COINES_MINI_SHUTTLE_PIN_1_4] = GPIO_0, [COINES_MINI_SHUTTLE_PIN_1_5] = GPIO_1,
    [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_2, [COINES_MINI_SHUTTLE_PIN_1_7] = GPIO_3,
    [COINES_MINI_SHUTTLE_PIN_2_5] = GPIO_4, [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_5,
    [COINES_MINI_SHUTTLE_PIN_2_1] = GPIO_CS, [COINES_MINI_SHUTTLE_PIN_2_3] = GPIO_SDO, [COINES_APP30_LED_R] = MCU_LED_R,
    [COINES_APP30_LED_G] = MCU_LED_G, [COINES_APP30_LED_B] = MCU_LED_B, [COINES_APP30_BUTTON_1] = SWITCH1,
    [COINES_APP30_BUTTON_2] = SWITCH2, [COINES_MINI_SHUTTLE_PIN_2_7] = GPIO_6, [COINES_MINI_SHUTTLE_PIN_2_8] = GPIO_7,
    [COINES_SHUTTLE_PIN_SDO] = GPIO_SDO
};
#else
uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX] = {
    [COINES_SHUTTLE_PIN_9] = GPIO_1, [COINES_SHUTTLE_PIN_14] = GPIO_4, [COINES_SHUTTLE_PIN_15] = GPIO_5,
    [COINES_SHUTTLE_PIN_16] = GPIO_SDI, [COINES_SHUTTLE_PIN_22] = GPIO_7, [COINES_SHUTTLE_PIN_8] = GPIO_0,
    [COINES_SHUTTLE_PIN_20] = GPIO_2, [COINES_SHUTTLE_PIN_21] = GPIO_3, [COINES_SHUTTLE_PIN_19] = GPIO_6,
    [COINES_SHUTTLE_PIN_7] = GPIO_CS, 0, 0, 0, 0, 0, 0,

    /* Native APP3.1 pins */
    [COINES_MINI_SHUTTLE_PIN_1_4] = GPIO_0, [COINES_MINI_SHUTTLE_PIN_1_5] = GPIO_1,
    [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_2, [COINES_MINI_SHUTTLE_PIN_1_7] = GPIO_3,
    [COINES_MINI_SHUTTLE_PIN_2_5] = GPIO_4, [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_5,
    [COINES_MINI_SHUTTLE_PIN_2_1] = GPIO_CS, [COINES_MINI_SHUTTLE_PIN_2_3] = GPIO_SDO,
    [COINES_MINI_SHUTTLE_PIN_2_8] = GPIO_7, [COINES_MINI_SHUTTLE_PIN_2_7] = GPIO_6, [COINES_SHUTTLE_PIN_SDO] = GPIO_SDO,
    [COINES_APP31_LED_R] = MCU_LED_R, [COINES_APP31_LED_G] = MCU_LED_G, [COINES_APP31_LED_B] = MCU_LED_B,
    [COINES_APP31_BUTTON_3] = SWITCH3, [COINES_APP31_BUTTON_2] = SWITCH2, [COINES_APP31_RESET_INT] = RESET_INT,
    [COINES_APP31_LSLDO] = CHRG_LSCTRL, [COINES_APP31_CD] = CHRG_CD, [COINES_APP31_P_INT] = POWER_INT,
    [COINES_APP31_VDDIO_EN] = VDDIO_EN, [COINES_APP31_VDD_EN] = VDD_EN, [COINES_APP31_LS_EN] = LS_EN,
    [COINES_APP31_VIN_DEC] = VIN_DEC
};
#endif

flog_write_file_t write_file[MAX_FILE_DESCRIPTORS];
flog_read_file_t read_file[MAX_FILE_DESCRIPTORS];
volatile bool fd_in_use[MAX_FILE_DESCRIPTORS] = { false }, fd_rw[MAX_FILE_DESCRIPTORS] = { false };
volatile int file_descriptors_used = 0;

DIR dir_h;
flogfs_ls_iterator_t iter;
nrf_saadc_value_t adc_buffer;

static uint32_t const * volatile mp_block_to_check = NULL;
static void coines_i2s_recv(nrf_drv_i2s_buffers_t const * p_released, uint32_t status);

static coines_tdm_callback tdm_data_callback = NULL;

static uint32_t i2s_buffer_rx[2][COINES_TDM_BUFFER_SIZE_WORDS];

nrf_drv_i2s_buffers_t const initial_buffers = {
    .p_rx_buffer = i2s_buffer_rx[0], .p_tx_buffer = NULL,
};

nrf_drv_i2s_buffers_t const next_buffers = {
    .p_rx_buffer = i2s_buffer_rx[1], .p_tx_buffer = NULL,
};

/*For PMIC I2C communication*/
#if defined(MCU_APP31)
struct bq_dev pmic_dev =
{ .cd_pin_state = 1, .i2c_address = BQ_DRV_ADDR, .read = common_i2c_read, .write = common_i2c_write,
  .delay_ms = common_delay_ms, .cd_set = common_pmic_cd_set, .battery_connected = 0 };
static volatile uint8_t pmic_battery_percent = 0;
void pmic_cyclic_reading(void);
void pmic_check_battery_and_faults(struct bq_dev *dev, struct fault_mask_reg *fault_state);

#endif

#if defined(MCU_APP30)
const nrfx_timer_t free_running_timer_instance = NRFX_TIMER_INSTANCE(3);
#else
const nrfx_timer_t free_running_timer_instance = NRFX_TIMER_INSTANCE(4);
#endif

/* Holds timer overflow count */
volatile uint32_t timer_overflow_count = 0;

#if defined(MCU_APP30)

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        coines_adc_transfer_done = true;
        nrf_saadc_value_t adc_result;
        uint16_t adc_result_in_milli_volts;

        adc_result = p_event->data.done.p_buffer[0];
        (void)nrfx_saadc_buffer_convert(&adc_buffer, 1);
        adc_result_in_milli_volts = (uint16_t)ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        batt_status_in_milli_volts = (uint16_t)(adc_result_in_milli_volts * BATTERY_VOLTAGE_SCALE_FACTOR);

        batt_status_percentage = battery_level_in_percentage(batt_status_in_milli_volts);

        if (batt_status_percentage >= 100)
        {
            batt_status_percentage = 100;
        }

        if (ble_bas_connected)
        {
            (void)ble_service_battery_level_update(batt_status_percentage, 1);
        }
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
ret_code_t adc_configure(void)
{
    /*lint -e1786 */
    nrfx_saadc_config_t adc_config = NRFX_SAADC_DEFAULT_CONFIG;

    ret_code_t err_code = nrfx_saadc_init(&adc_config, saadc_event_handler);

    if (NRF_SUCCESS == err_code)
    {
        nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
        err_code = nrfx_saadc_channel_init(7, &config);
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = nrfx_saadc_buffer_convert(&adc_buffer, 1);
    }

    return err_code;
}
#endif

/*!
 * @brief  Callback to read temperature data
 *
 * @param[out] data - place holder to store the temperature data
 * @param[out] length - length of the data
 *
 * @return None
 */
static void temp_data_read_callback(void * data, uint8_t *length)
{
    int16_t rslt;

    rslt = coines_read_temp_data((float*)data);
    if (COINES_SUCCESS == rslt)
    {
        *length = 4;
    }
    else
    {
        *length = 0;
    }
}

/*!
 * @brief       : API to check communication port connection
 */
void check_com_port_connection(int set)
{
    static bool reset_board = false;

    if (set == 1)
    {
        serial_connected = true;
    }
    else
    {
        serial_connected = false;
    }

    /*
     *  Trigger MCU reset if USB CDC port is opened and closed at 1200bps
     *  https://www.arduino.cc/en/Main/Arduino_BoardLeonardo
     *  See # Automatic (Software) Reset and Bootloader Initiation
     */
    if (set == 1 && (baud_rate == 1200 || baud_rate == 2400))
    {
        reset_board = true;
    }

    if (set == 0 && reset_board == true)
    {
        if (baud_rate == 1200)
        {
            APP_START_ADDR = 0xF0000; /*Jump to USB DFU Bootloader */
        }
        else if (baud_rate == 2400)
        {
            APP_START_ADDR = 0x28000; /*Jump to USB MTP Firmware */

        }

        memcpy(MAGIC_INFO_ADDR, "COIN", 4); /*Write magic string "COIN" */
        nrf_delay_ms(100);
        NVIC_SystemReset();
    }
}

/*!
 *
 * @brief       :Event handler for USB CDC ACM
 *
 * @param[in]   :* p inst,event
 *
 * @return      : None
 */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event)
{

    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    app_usbd_cdc_line_coding_t p_line_encoding = p_cdc_acm->specific.p_data->ctx.request.payload.line_coding;

    memcpy(&baud_rate, p_line_encoding.dwDTERate, 4);
    ret_code_t ret;

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            check_com_port_connection(1);

            /* Setup a read */
            (void)app_usbd_cdc_acm_read(&m_app_cdc_acm, &temp_rx, 1);
            tx_pending = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            check_com_port_connection(0);
            tx_pending = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            tx_pending = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            do
            {
                serial_buffer[serial_idx] = temp_rx;

                /* Overwrite the last byte in case the buffer has been read */
                if (serial_idx < (RX_BUFFER_SIZE - 1))
                {
                    serial_idx++;
                }
                else
                {
                    serial_idx = (RX_BUFFER_SIZE - 1);
                }

                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &temp_rx, 1);
            } while (ret == NRF_SUCCESS);
            break;
        }
        default:
            break;
    }
}

/**@brief Function for initializing external flash.
 */
static uint32_t external_flash_init(void)
{
    uint32_t ret_status = NRF_SUCCESS;
    w25_deviceinfo_t info;
    flog_initialize_params_t params = { .number_of_blocks = FS_NUM_BLOCKS, .pages_per_block = FS_PAGES_PER_BLOCK };

    if (FLOG_FAILURE == flogfs_initialize(&params))
    {
        ret_status |= NRF_FLASH_INIT_FAILED_MASK;
    }

    if (flogfs_mount() == FLOG_FAILURE)
    {
        w25_get_manufacture_and_devid(&info);
        for (uint16_t i = 0; i < FS_NUM_BLOCKS; i++)
        {
            if (info.device_id == W25M02GW_DEVICE_ID)
            {
                (void)w25m02gw_erase_block((i * W25M02GW_BLOCK_SIZE) + 1, W25M02GW_BLOCK_SIZE);
            }

#if defined(MCU_APP30)
            if (info.device_id == W25N02JW_DEVICE_ID)
#else
            if ((info.device_id == W25N02JW_DEVICE_ID) || (info.device_id == W25N02KW_DEVICE_ID))
#endif
            {
                (void)w25n02jw_erase_block((i * W25N02JW_BLOCK_SIZE) + 1, W25N02JW_BLOCK_SIZE);
            }
        }

        (void)flogfs_format();
    }
    else
    {
        /* Do nothing */
    }

    return ret_status;
}

/**@brief Function for  initializing usb.
 */
static uint32_t usb_init()
{
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    uint32_t ret_status = NRF_SUCCESS;

    app_usbd_serial_num_generate();
    if (NRF_SUCCESS == app_usbd_init(&usbd_config))
    {
        const app_usbd_class_inst_t* class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
        if (NRF_SUCCESS != app_usbd_class_append(class_cdc_acm))
        {
            ret_status |= NRF_USB_INIT_FAILED_MASK;
        }

        if (NRF_SUCCESS != app_usbd_power_events_enable())
        {
            ret_status |= NRF_USB_INIT_FAILED_MASK;
        }
    }
    else
    {
        ret_status |= NRF_USB_INIT_FAILED_MASK;
    }

    return ret_status;
}

/*!
 * @brief This API is hardware timer interrupt handler which will be used to handle the interrupt events
 */
static void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    (void)p_context;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            timer_overflow_count++;
            break;

        default:

            /*Do nothing. */
            break;
    }
}

#if defined(MCU_APP31)

/*!
 * @brief This API is used to check battery and faults
 */
void pmic_check_battery_and_faults(struct bq_dev *dev, struct fault_mask_reg *fault_state)
{
    struct bq_term_precharge_config prechrg_cfg;
    enum coines_pin_direction pin_direction = COINES_PIN_DIRECTION_IN;
    enum coines_pin_value pin_value = COINES_PIN_VALUE_LOW;

    /*Acknowledge faults*/
    (void)bq_get_faults(&pmic_dev, fault_state);

    (void)coines_get_pin_config(COINES_APP31_VIN_DEC, &pin_direction, &pin_value);
    if ((pin_value == COINES_PIN_VALUE_HIGH) && (fault_state->bat_uvlo == 1)) /*Battery not connected*/
    {
        /*Disable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;/*10% of Max charging current = 15mA */
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        (void)bq_set_term_precharge_current(&pmic_dev, prechrg_cfg);
#ifdef PRE_CHARGE_EN

        /*lint -e715 */
        dev->battery_connected = 0;
#endif
    }
    else if ((pin_value == COINES_PIN_VALUE_HIGH) && (fault_state->bat_uvlo == 0)) /*Battery connected*/
    {
        /*Enable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;/*10% of Max charging current = 15mA */
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        (void)bq_set_term_precharge_current(&pmic_dev, prechrg_cfg);
#ifdef PRE_CHARGE_EN

        /*lint -e715 */
        dev->battery_connected = 1;
#endif
    }

    ;
}

/*!
 * @brief Callback for pmic reading Timer Instance 1
 */
void pmic_cyclic_reading(void)
{
    uint8_t bat_vbbm = 0;
    struct fault_mask_reg faults;

    /*update battery percentage*/
    battery_meas_rslt = bq_get_battery_voltage(&pmic_dev, &bat_vbbm); /*Voltage Based Battery Monitor */
    if (BQ_OK == battery_meas_rslt)
    {
        batt_status_in_milli_volts = (uint16_t) (((bat_vbbm * 4.2) / 100.0) * 1000.0); /*Convert VBMON percentage to
         * voltage => Volt = VBMON *
         * VBATREG */
        pmic_battery_percent = battery_level_in_percentage(batt_status_in_milli_volts);
    }
    else{
        battery_meas_rslt = COINES_E_FAILURE;
    }

    /*Check battery and get faults*/
    if (pmic_dev.battery_connected == 0)
    {
        pmic_check_battery_and_faults(&pmic_dev, &faults);
    }
}

/*!
 * @brief Function to get last obtained value of the battery level in percent.
 */
uint8_t pmic_pull_battery_level(void)
{
    return pmic_battery_percent;
}

/*!
 * @brief Callback for VIN detection
 */
void vin_detection_cb(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;
    enum coines_pin_direction pindirection;
    enum coines_pin_value pinvalue;

    coines_get_pin_config(COINES_APP31_VIN_DEC, &pindirection, &pinvalue);
    if (pinvalue == COINES_PIN_VALUE_HIGH)
    {
        /* Vin plugged => Activate charging */
        (void)bq_charge_enable(&pmic_dev);
    }
    else if (pinvalue == COINES_PIN_VALUE_LOW)
    {
        /* Vin unplugged => Deactivate charging */
        (void)bq_charge_disable(&pmic_dev);
    }
}

/*!
 * @brief Callback for Reset interrupt
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
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    /*Acknowledge button press*/
    (void)bq_ack_push_button_events(&pmic_dev, &event);
    (void)coines_get_pin_config(COINES_APP31_VIN_DEC, &pin_direction, &pin_value);
    switch (event)
    {
        case 1: /*Activate Ship mode only when Vin unplugged*/
            if (pin_value == COINES_PIN_VALUE_LOW)
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

    if ((pmic_dev.battery_connected) && (pin_value == COINES_PIN_VALUE_HIGH))
    {    /*Check battery and get faults*/
        pmic_check_battery_and_faults(&pmic_dev, &faults);
    }
}

/*
 * @brief This API is used to get the factory device ID FICR
 */
void coines_get_device_ficr(uint64_t * devid)
{
    uint32_t device_id0 = (uint32_t) NRF_FICR->DEVICEID[0];
    uint32_t device_id1 = (uint32_t) NRF_FICR->DEVICEID[1];

    devid[0] = (uint64_t) device_id0;
    devid[1] = (uint64_t) device_id1;
}
#endif

/*!
 * @brief Initializes the EEPROM and reads shuttle config
 */
static void eeprom_init(void)
{
    uint16_t shuttle_id = 0;
    uint8_t eeprom_read_buff[EEPROM_READ_LEN];

    app30_eeprom_init();
    if (app30_eeprom_read(0x60, eeprom_read_buff,
                          EEPROM_READ_LEN) && app30_eeprom_read(0x01, (uint8_t *)&shuttle_id, 2))
    {

        /* Check the CS pin byte value, Should not be 0xff, if 0xff read failed. And check if shuttle id is in the range
         */
        if ((eeprom_read_buff[EEPROM_CS_BYTE_INDEX] != 0xff) && (shuttle_id > 0 && shuttle_id <= 0x1ff))
        {
            memcpy(multi_io_map, eeprom_read_buff, 10);
        }
    }
}

/*!
 * @brief This API is used to initialize the communication according to interface type.
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    (void)arg;
    uint32_t error_status = NRF_SUCCESS;

    /* Timer configuration */
    nrfx_timer_config_t timer_config = {
        .frequency = NRF_TIMER_FREQ_16MHz, .mode = NRF_TIMER_MODE_TIMER, .bit_width = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, .p_context = NULL
    };

    ble_service_init_t init_handle = {
        .temp_read_callback = temp_data_read_callback, .batt_status_read_callback = bat_status_read_callback,
        .adv_name = ble_device_name, .tx_power = (int8_t)ble_tx_power
    };

    /* Initialize the low frequency clock */
    if (NRFX_SUCCESS != nrf_drv_clock_init())
    {
        error_status |= NRF_CLOCK_INIT_FAILED_MASK;
    }

    if (NRFX_SUCCESS != nrf_drv_power_init(NULL))
    {
        error_status |= NRF_POWER_INIT_FAILED_MASK;
    }

    /* Request the clock to not to generate events */
    nrf_drv_clock_lfclk_request(NULL);
    nrf_drv_clock_hfclk_request(NULL);

    while (!nrf_drv_clock_lfclk_is_running() && !nrf_drv_clock_hfclk_is_running())
        ;

    /* Initialize the EEPROM and read shuttle config */
    eeprom_init();

    error_status |= external_flash_init();

#if defined(MCU_APP30)

    /* Configure the board switch and led */
    nrf_gpio_cfg_input(SWITCH1, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(SWITCH2, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_cfg_output(MCU_LED_R);
    nrf_gpio_cfg_output(MCU_LED_G);
    nrf_gpio_cfg_output(MCU_LED_B);

    nrf_gpio_pin_set(MCU_LED_R);
    nrf_gpio_pin_set(MCU_LED_G);
    nrf_gpio_pin_set(MCU_LED_B);
#endif

#if defined(MCU_APP31)

    /* Configure the board switch and led */
    coines_set_pin_config(COINES_APP31_BUTTON_3, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_set_pin_config(COINES_APP31_BUTTON_2, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_set_pin_config(COINES_APP31_LED_R, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_LED_B, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

    /*Config RESET if not configured P0.18*/
    if (NRF_UICR->PSELRESET[0] != 18 || NRF_UICR->PSELRESET[1] != 18)
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        NRF_UICR->PSELRESET[0] = 18;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        NRF_UICR->PSELRESET[1] = 18;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        NVIC_SystemReset();
    }

#endif

    /* Configure rtc  */
    if (NRF_SUCCESS != rtc_config())
    {
        error_status |= NRF_RTC_INIT_FAILED_MASK;
    }

    /*For coines_get_millis() API*/
    if (NRF_SUCCESS != SysTick_Config(64000))
    {
        error_status |= NRF_SYSTICK_INIT_FAILED_MASK;
    }

    if (intf_type == COINES_COMM_INTF_BLE)
    {
        ble_service_init(&init_handle);
    }

    error_status |= usb_init();

#if defined(MCU_APP30)
    nrf_gpio_cfg_output(VDD_PS_EN);
    nrf_gpio_cfg_output(VDDIO_PS_EN);
    nrf_gpio_cfg_output(VDD_SEL);
    nrf_gpio_cfg_output(VBAT_MON_EN);

    nrf_gpio_pin_clear(VDD_PS_EN);
    nrf_gpio_pin_clear(VDD_SEL);
    nrf_gpio_pin_clear(VDDIO_PS_EN);
    nrf_gpio_pin_clear(VBAT_MON_EN);

#endif
#if defined(MCU_APP31)

    /* Configure the board essential pins */
    coines_set_pin_config(COINES_APP31_LSLDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_VDDIO_EN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_VDD_EN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_LS_EN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    coines_set_pin_config(COINES_APP31_CD, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    coines_set_pin_config(COINES_APP31_VIN_DEC, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_LOW);
    coines_set_pin_config(COINES_APP31_P_INT, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_set_pin_config(COINES_APP31_RESET_INT, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
#endif

    if (NRFX_SUCCESS != nrfx_gpiote_init())
    {
        error_status |= NRF_GPIO_INIT_FAILED_MASK;
    }

#if defined(MCU_APP30)
    if (NRFX_SUCCESS != adc_configure())
    {
        error_status |= NRF_ADC_INIT_FAILED_MASK;
    }

#endif

    /* Configure hardware timer for capturing timestamp */
    if (NRF_SUCCESS ==
        nrfx_timer_init(&free_running_timer_instance, &timer_config, (nrfx_timer_event_handler_t)timer_handler))
    {

        nrfx_timer_extended_compare(&free_running_timer_instance,
                                    NRF_TIMER_CC_CHANNEL0,
                                    0xFFFFFFFF,
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    (bool)true);

        nrfx_timer_enable(&free_running_timer_instance);
    }
    else
    {
        error_status |= NRF_TIMER_INIT_FAILED_MASK;
    }

    setbuf(stdin, NULL);
    if (NRF_SUCCESS != error_status)
    {
        if (error_status & (NRF_CLOCK_INIT_FAILED_MASK | NRF_POWER_INIT_FAILED_MASK))
        {
            while (1)
                ;
        }
        else
        {
            if (error_status &
                (NRF_RTC_INIT_FAILED_MASK | NRF_SYSTICK_INIT_FAILED_MASK | NRF_USB_INIT_FAILED_MASK |
                 NRF_GPIO_INIT_FAILED_MASK | NRF_ADC_INIT_FAILED_MASK | NRF_TIMER_INIT_FAILED_MASK))
            {
                return COINES_E_INIT_FAILED;
            }
        }
    }

#if defined(MCU_APP30)
    nrf_gpio_pin_clear(MCU_LED_R);
#endif

#if defined(MCU_APP31)
    coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
    coines_set_led(COINES_LED_BLUE, COINES_LED_STATE_OFF);

    /* Initialize i2c interface */
    (void)common_i2c_init();

    /* Initialize power module */
    (void)common_pmic_cd_init();
    coines_delay_msec(10);
    (void)bq_init(&pmic_dev);
    coines_delay_msec(10);
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    (void)coines_get_pin_config(COINES_APP31_VIN_DEC, &pin_direction, &pin_value);
    if (pin_value == COINES_PIN_VALUE_LOW)
    {
        (void)bq_charge_disable(&pmic_dev);
        (void)bq_cd_set(&pmic_dev, 0);
        coines_delay_msec(10);
        (void)bq_cd_set(&pmic_dev, 1);
        coines_delay_msec(10);
    }
    else
    {
        (void)bq_charge_enable(&pmic_dev);
    }

    coines_attach_interrupt(COINES_APP31_VIN_DEC, vin_detection_cb, COINES_PIN_INTERRUPT_CHANGE);
    coines_attach_interrupt(COINES_APP31_P_INT, power_interrupt_cb, COINES_PIN_INTERRUPT_FALLING_EDGE);

    /*Initiate a battery reading*/
    pmic_cyclic_reading();

    /* Initialize Timer for periodical pmic reg reading every 30 seconds to prevent watchdog reset*/
    if (COINES_SUCCESS != coines_timer_config(COINES_TIMER_INSTANCE_1, pmic_cyclic_reading))
    {
        return COINES_E_INIT_FAILED;
    }
    else
    {
        (void)coines_timer_start(COINES_TIMER_INSTANCE_1, 30000000);
    }

    /*Set sensor VDD and VDDIO to 0V*/
    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);

    /*Acknowledge wake up interrupt when exiting ship mode*/
    uint8_t ButtonAck = 0;
    (void)bq_ack_push_button_events(&pmic_dev, &ButtonAck);

    /*Set RED Led ON waiting for communication*/
    coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
#endif

    while (!(serial_connected || nrf_gpio_pin_read(SWITCH2) == 0 || ble_nus_connected))
    {
        nrf_delay_ms(100);
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{
    nrf_gpio_pin_pull_t pin_pull;

    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_FAILURE;
    }

    if (direction == COINES_PIN_DIRECTION_IN)
    {
        pin_pull = pin_value ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN;
        nrf_gpio_cfg_input(pin_num, pin_pull);
    }
    else if (direction == COINES_PIN_DIRECTION_OUT)
    {
        nrf_gpio_cfg_output(pin_num);
        nrf_gpio_pin_write(pin_num, pin_value);
    }

    return COINES_SUCCESS;
}

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    uint32_t pin_num = multi_io_map[pin_number];

    if (pin_num == 0 || pin_num == 0xff)
    {
        return COINES_E_FAILURE;
    }

    if ((pin_value != NULL) || (pin_direction != NULL))
    {
        if (pin_value != NULL)
        {
            *pin_value = (enum coines_pin_value)nrf_gpio_pin_read(pin_num);
        }

        if (pin_direction != NULL)
        {
            *pin_direction = (enum coines_pin_direction)nrf_gpio_pin_dir_get(pin_num);
        }

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

static void coines_i2s_recv(nrf_drv_i2s_buffers_t const * p_released, uint32_t status)
{
    /* 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
     each time next buffers are requested, so data corruption is not
     expected. */

    /*lint -e716 -e722 */
    ASSERT(p_released);

    /* When the handler is called after the transfer has been stopped
     (no next buffers are needed, only the used buffers are to be
     released), there is nothing to do. */
    if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    /* First call of this handler occurs right after the transfer is started.
     No data has been transferred yet at this point, so there is nothing to
     check. Only the buffers for the next part of the transfer should be
     provided. */
    if (!p_released->p_rx_buffer)
    {

        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffers));
    }
    else
    {
        mp_block_to_check = (uint32_t *)p_released->p_rx_buffer;

        /* The driver has just finished accessing the buffers pointed by
         'p_released'. They can be used for the next part of the transfer
         that will be scheduled now. */
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));

        if (tdm_data_callback)
        {
            tdm_data_callback(mp_block_to_check);
        }
    }
}

int16_t coines_config_i2s_bus(uint16_t data_words, coines_tdm_callback callback)
{
    uint32_t rslt;

    tdm_data_callback = callback;
    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;

    /*
     * In Master mode the MCK frequency and the MCK/LRCK ratio should be
     * set properly in order to achieve desired audio sample rate (which
     * is equivalent to the LRCK frequency).
     */

    config.sck_pin = GPIO_4; /* GPIO 4 ->GPIO_37 */
    config.mck_pin = NRF_DRV_I2S_PIN_NOT_USED;
    config.lrck_pin = GPIO_5; /*GPIO 5 -> GPIO_36 */
    config.sdin_pin = GPIO_6; /* GPIO 6 ; Use as low drive - low frequency GPIO only */
    config.sdout_pin = NRF_DRV_I2S_PIN_NOT_USED;
    config.mck_setup = NRF_I2S_MCK_32MDIV21;
    config.ratio = NRF_I2S_RATIO_32X; /* 32X- to support 48Khz sample rate = 1.523Mhz/32 */
    config.channels = NRF_I2S_CHANNELS_STEREO;
    config.sample_width = NRF_I2S_SWIDTH_16BIT;
    config.mode = NRF_I2S_MODE_MASTER;
    config.format = NRF_I2S_FORMAT_I2S;
    config.alignment = NRF_I2S_ALIGN_LEFT;

    rslt = nrf_drv_i2s_init(&config, coines_i2s_recv);
    if (rslt == NRF_SUCCESS)
    {
        if (data_words > COINES_TDM_BUFFER_SIZE_WORDS)
        {
            return COINES_E_MEMORY_ALLOCATION;
        }
        else
        {
            rslt = nrf_drv_i2s_start(&initial_buffers, data_words, 0);
        }
    }

    if (rslt != NRFX_SUCCESS)
    {
        return COINES_E_FAILURE;
    }

    return COINES_SUCCESS;
}

uint16_t coines_intf_available(enum coines_comm_intf intf)
{
    if ((intf == COINES_COMM_INTF_USB) && (serial_connected))
    {
        if (read_idx >= serial_idx)
        {
            return 0;
        }

        return serial_idx - read_idx;
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        return (uint16_t)ble_nus_available;
    }

    return 0;
}

bool coines_intf_connected(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        return serial_connected;
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        return ble_nus_connected;
    }

    return false;
}

/*!
 * @brief API to write BLE data as packets
 */
static void ble_write_data_packets(void *buffer, uint32_t n_bytes)
{
    uint32_t total_bytes_sent = 0;
    uint32_t bytes_remaining = n_bytes;
    uint32_t chunk_size = 0;
    uint8_t payload[BLE_MTU] = { 0 };
    uint8_t *buffer_ptr = (uint8_t *)buffer;

    while (bytes_remaining > 0)
    {
        /* Calculate the size of the current chunk */
        chunk_size = (bytes_remaining > BLE_MTU) ? BLE_MTU : bytes_remaining;
        memcpy(payload, &buffer_ptr[total_bytes_sent], chunk_size);

        /* Send the payload via the BLE service */
        (void)ble_service_nus_write(payload, chunk_size);

        /* Update the total number of bytes sent and the number of bytes yet to be sent */
        total_bytes_sent += chunk_size;
        bytes_remaining -= chunk_size;
    }
}

void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    if ((intf == COINES_COMM_INTF_USB) && (serial_connected))
    {
        /* Wait for previous transfer to complete */
        while (tx_pending);
        tx_pending = true;
        (void)app_usbd_cdc_acm_write(&m_app_cdc_acm, buffer, len);
        while (tx_pending)
        {
            coines_yield();
        }
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        ble_write_data_packets(buffer, len);
    }
}

void coines_flush_intf(enum coines_comm_intf intf)
{
    if ((intf == COINES_COMM_INTF_USB) && (serial_connected))
    {
        serial_idx = 0;
        read_idx = 0;
        memset(serial_buffer, 0, RX_BUFFER_SIZE);
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        (void)fflush(bt_w);
    }
}

static uint16_t read_serial_buffer(uint8_t *buffer, uint16_t len)
{
    uint16_t bytes_read = 0;

    NRFX_IRQ_DISABLE(USBD_IRQn);

    while ((read_idx <= serial_idx) && (bytes_read < len))
    {
        buffer[bytes_read] = serial_buffer[read_idx];
        read_idx++;
        bytes_read++;
    }

    if (read_idx >= serial_idx)
    {
        read_idx = 0;
        serial_idx = 0;
    }

    NRFX_IRQ_ENABLE(USBD_IRQn);

    return bytes_read;
}

uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    uint16_t bytes_read = 0;

    if ((intf == COINES_COMM_INTF_USB) && (serial_connected))
    {
        bytes_read = read_serial_buffer((uint8_t*)buffer, len);
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        bytes_read = (uint16_t)ble_service_nus_read(buffer, len);
    }

    return bytes_read;
}

/*!
 * @brief SysTick timer handler
 */
void SysTick_Handler(void)
{
    g_millis++;
}

/* For stdio functions */

int __attribute__((weak)) _write(int fd, const void *buffer, int len)
{
    if ((fd == 1 || fd == 2) && serial_connected == true)
    {
        tx_pending = true;
        (void)app_usbd_cdc_acm_write(&m_app_cdc_acm, buffer, (size_t)len);
        while (tx_pending)
            ;

        return len;
    }
    else if (fd >= 3 && fd < MAX_FILE_DESCRIPTORS + 3)
    {
        return (int)flogfs_write(&write_file[fd - 3], (const uint8_t*)buffer, (uint32_t)len);
    }
    else if (fd == BLE_NUS_FD_W && ble_nus_connected == true)
    {
        return (int)ble_service_nus_write(buffer, (size_t)len);
    }
    else
    {
        return len;
    }
}

int __attribute__((weak)) _read(int fd, void *buffer, int len)
{
    if (fd == 0)
    {
        while ((serial_idx - read_idx) < len)
            ;

        return read_serial_buffer((uint8_t*)buffer, (uint16_t)len);
    }
    else if (fd >= 3 && fd < MAX_FILE_DESCRIPTORS + 3)
    {
        return (int)flogfs_read(&read_file[fd - 3], (uint8_t*)buffer, (uint32_t)len);
    }
    else if (fd == BLE_NUS_FD_R && ble_nus_connected == true)
    {
        return (int)ble_service_nus_read(buffer, (size_t)len);
    }
    else
    {
        return len;
    }
}

int __attribute__((weak)) _open(const char *file_name, int flags)
{
    int fd = 0;
    bool fd_val;

    if (strcmp(file_name, BLE_DEVICE_FILE) == 0)
    {
        if (flags == O_RDONLY)
        {
            return BLE_NUS_FD_R;
        }
        else
        {
            return BLE_NUS_FD_W;
        }
    }

    if (file_descriptors_used > MAX_FILE_DESCRIPTORS)
    {
        return -1;
    }

    for (int i = 0; i < MAX_FILE_DESCRIPTORS; i++)
    {
        fd_val = fd_in_use[i];
        if (fd_val == false)
        {
            fd = i + 3;
            fd_in_use[i] = true;
            break;
        }
    }

    if (flags == O_RDONLY)
    {
        if (flogfs_open_read(&read_file[fd - 3], file_name) == FLOG_SUCCESS)
        {
            fd_rw[fd - 3] = false;
        }
        else
        {
            return -1;
        }
    }
    else
    {
        if (flogfs_open_write(&write_file[fd - 3], file_name) == FLOG_SUCCESS)
        {
            fd_rw[fd - 3] = true;
        }
        else
        {
            return -1;
        }
    }

    file_descriptors_used++;

    return fd;

}

int __attribute__((weak)) _close(int fd)
{
    bool fd_val;
    flog_result_t retval;

    fd_val = fd_rw[fd - 3];
    if (fd_val == true)
    {
        retval = flogfs_close_write(&write_file[fd - 3]);
    }
    else
    {
        retval = flogfs_close_read(&read_file[fd - 3]);
    }

    fd_in_use[fd - 3] = false;
    file_descriptors_used--;
    if (FLOG_SUCCESS != retval)
    {
        return -1;
    }

    return 0;
}

int __attribute__((weak)) _unlink(const char *file_name)
{
    if (flogfs_rm(file_name) == FLOG_SUCCESS)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int __attribute__((weak)) _stat(const char *file_name, struct stat *st)
{
    int fd;

    if (flogfs_check_exists(file_name) == FLOG_FAILURE)
    {
        return -1;
    }

    if ((fd = _open(file_name, O_RDONLY)) < 0)
    {
        return -1;
    }

    if (fd < 3)
    {
        return -1;
    }

    memset(st, 0, sizeof(struct stat));
    st->st_mode = S_IFREG;
    st->st_size = (off_t)flogfs_read_file_size(&read_file[fd - 3]);
    st->st_blksize = 1024;
    if (0 != _close(fd))
    {
        return -1;
    }

    return 0;
}

int __attribute__((weak)) _fstat(int fd, struct stat *buf)
{
    (void)fd;
    (void)buf;

    return -1;
}

int __attribute__((weak)) _isatty(int fd)
{
    (void)fd;

    return -1;
}

int __attribute__((weak)) _lseek(int fd, int pos, int whence)
{
    (void)fd;
    (void)pos;
    (void)whence;

    return -1;
}

void __attribute__((weak)) _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
}

int __attribute__((weak)) _getpid(void)
{
    return -1;
}

/* For compatibility with dirent.h */

DIR *opendir(const char *dirname)
{
    (void)dirname;
    flogfs_start_ls(&iter);

    return &dir_h;
}

struct dirent *readdir(DIR *dirp)
{
    (void)dirp;

    if (flogfs_ls_iterate(&iter, dir_h.dd_dir.d_name))
    {
        dir_h.dd_dir.d_namlen = strlen(dir_h.dd_dir.d_name);

        return &dir_h.dd_dir;
    }
    else
    {
        return NULL;
    }
}

int closedir(DIR *dirp)
{
    (void)dirp;

    flogfs_stop_ls(&iter);

    return 0;
}

/*!
 * @brief This API is used to read the temperature sensor data.
 *
 * @param[out] temp_conv_data       :  Buffer to retrieve the sensor data in degC.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_temp_data(float *temp_data)
{
    int16_t error;
    uint8_t temp_sens_dev_addr = TMP112Q1_DEV_ADDRESS;
    uint8_t reg_addr = TMP112Q1_REG_ADDRESS;
    uint8_t read_length = 2;
    uint8_t temp_buffer[read_length];
    uint16_t temp_raw_data;

    enum coines_i2c_bus temp_bus_intf = COINES_I2C_BUS_1;

    if (coines_is_i2c_enabled(temp_bus_intf))
    {
        /* Send dummy data (0) or preserve and send previous read temp data */
        *temp_data = 0;

        error = COINES_SUCCESS;
    }
    else
    {
        /* Configure I2C_BUS_1 for onboard temperature sensor */
        error = coines_config_i2c_bus_internal(temp_bus_intf, COINES_I2C_STANDARD_MODE, COINES_I2C_PIN_INTERNAL_TEMP);

        if (COINES_SUCCESS != error)
        {
            return error;
        }
    }

    error = coines_read_i2c(temp_bus_intf, temp_sens_dev_addr, reg_addr, temp_buffer, read_length);

    if (COINES_SUCCESS == error)
    {
        temp_raw_data = (uint16_t)temp_buffer[0];
        temp_raw_data = (uint16_t)((temp_raw_data << 4) | (temp_buffer[1] >> 4));

        if (temp_raw_data & (1 << 11))
        {
            /* Negative temperature */
            temp_raw_data = ~temp_raw_data;
            temp_raw_data &= 0xFFF;
            temp_raw_data += 1;
            *temp_data = (float)(temp_raw_data * 0.0625);
        }
        else
        {
            /* Positive temperature */
            *temp_data = (float)(temp_raw_data * 0.0625);
        }
    }

    return error;
}

/*!
 * @brief This API is used to read the battery status .
 *
 * @param[out] bat_status_mv            :  Buffer to retrieve the battery status in millivolt.
 *
 * @param[out] bat_status_percent       :  Buffer to retrieve the battery status in %.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int16_t coines_read_bat_status(uint16_t *bat_status_mv, uint8_t *bat_status_percent)
{
#if defined(MCU_APP30)
    coines_adc_transfer_done = false;

    battery_meas_rslt = nrfx_saadc_sample();

    if (NRFX_SUCCESS == battery_meas_rslt)
    {
        while (!coines_adc_transfer_done)
            ;

        *bat_status_percent = batt_status_percentage;
        *bat_status_mv = batt_status_in_milli_volts;

        return COINES_SUCCESS;
    }

    return COINES_E_FAILURE;
#else
    *bat_status_percent = pmic_battery_percent;
    *bat_status_mv = batt_status_in_milli_volts;

    return battery_meas_rslt;
#endif
}

/*!
 *  @brief This API is used to configure BLE name and power.This API should be called
 *         before calling coines_open_comm_intf().
 *
 *  @param[in] ble_config : structure holding ble name and power details
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
int16_t coines_ble_config(struct coines_ble_config *ble_config)
{
    if (NULL != ble_config)
    {
        ble_device_name = ble_config->name;
        ble_tx_power = ble_config->tx_power;

        return COINES_SUCCESS;
    }

    return COINES_E_NULL_PTR;
}

/*!
 *  @brief This API is used to set led state(on or off).
 *
 *  @param[in] led             : led to which the state has to be set
 *  @param[in] led_state       : state to be set to the given led
 *
 *  @return Results of API execution status.
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 */
#if defined(MCU_APP30)
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state)
{
    int16_t retval;

    switch (led)
    {
        case COINES_LED_RED:
            retval = coines_set_pin_config(COINES_APP30_LED_R,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        case COINES_LED_GREEN:
            retval = coines_set_pin_config(COINES_APP30_LED_G,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        case COINES_LED_BLUE:
            retval = coines_set_pin_config(COINES_APP30_LED_B,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        default:
            retval = COINES_E_NOT_SUPPORTED;
            break;
    }

    return retval;
}
#else
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state)
{
    int16_t retval;

    led_state = (enum coines_led_state)(1 - led_state);

    switch (led)
    {
        case COINES_LED_RED:
            retval = coines_set_pin_config(COINES_APP31_LED_R,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        case COINES_LED_GREEN:
            retval = coines_set_pin_config(COINES_APP31_LED_G,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        case COINES_LED_BLUE:
            retval = coines_set_pin_config(COINES_APP31_LED_B,
                                           COINES_PIN_DIRECTION_OUT,
                                           (enum coines_pin_value)led_state);
            break;
        default:
            retval = COINES_E_NOT_SUPPORTED;
            break;
    }

    return retval;
}
#endif

/*!
 *  @brief This API function is used to get the pin direction and pin state.
 */
#if defined(MCU_APP30)
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    if (vdd_millivolt == 0)
    {
        nrf_gpio_pin_write(VDD_PS_EN, 0);
        nrf_gpio_pin_write(VDD_SEL, 0);
    }
    else if ((vdd_millivolt > 0) && (vdd_millivolt <= 1800))
    {
        nrf_gpio_pin_write(VDD_PS_EN, 1);
        nrf_gpio_pin_write(VDD_SEL, 0);
    }
    else
    {
        nrf_gpio_pin_write(VDD_PS_EN, 1);
        nrf_gpio_pin_write(VDD_SEL, 1);
    }

    if (vddio_millivolt == 0)
    {
        nrf_gpio_pin_write(VDDIO_PS_EN, 0);
    }
    else
    {
        nrf_gpio_pin_write(VDDIO_PS_EN, 1);
    }

    return COINES_SUCCESS;
}
#else
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    struct bq_load_switch ldocfg;
    struct bq_sys_vout syscfg;
    uint8_t lscode = 0;
    uint8_t syscode = 0;
    int16_t ret = COINES_E_FAILURE;

    ldocfg.reset = BQ_LOAD_LDO_RESET_TIME;

    if (vdd_millivolt < 800)
    {
        nrf_gpio_pin_write(CHRG_LSCTRL, 0);
        nrf_gpio_pin_write(VDD_EN, 0);
        ldocfg.enable = BQ_LOAD_LDO_DISABLE;
        ret = bq_set_load_ldo(&pmic_dev, ldocfg);
    }
    else
    {
        vdd_millivolt = (vdd_millivolt > 3300)?3300:vdd_millivolt; /*3300mV Maximum */

        /*We should disable the LDO output before changing the output voltage */
        nrf_gpio_pin_write(CHRG_LSCTRL, 0);
        nrf_gpio_pin_write(VDD_EN, 0);
        ldocfg.enable = BQ_LOAD_LDO_DISABLE;
        (void)bq_set_load_ldo(&pmic_dev, ldocfg);

        /*Setting the new LDO output voltage */
        lscode = (uint8_t) ((vdd_millivolt - 800) / 100); /*Output voltage = 0,8V + Ls_LDOCODE x 100mV */
        ldocfg.code = lscode << 2; /*LS_LDO code start from Bit2 */
        ldocfg.enable = BQ_LOAD_LDO_ENABLE;
        ret = bq_set_load_ldo(&pmic_dev, ldocfg);
        nrf_gpio_pin_write(CHRG_LSCTRL, 1);
        nrf_gpio_pin_write(VDD_EN, 1);
    }

    if (vddio_millivolt < 1800)
    {
        nrf_gpio_pin_write(VDDIO_EN, 0);
    }
    else
    {
        vddio_millivolt = (vddio_millivolt > 3300)?3300:vddio_millivolt; /*3300mV Maximum */

        if ((vddio_millivolt >= 1800) && (vddio_millivolt <= 2800))
        {
            syscfg.sys_select = BQ_SYS_VOUT_SEL_1;
            syscode = (uint8_t) ((vddio_millivolt - 1300) / 100); /*Output voltage = 1,3V + SysVoutCODE x 100mV */
        }
        else if ((vddio_millivolt > 2800) && (vddio_millivolt <= 3300))
        {
            syscfg.sys_select = BQ_SYS_VOUT_SEL_3;
            syscode = (uint8_t) ((vddio_millivolt - 1800) / 100); /*Output voltage = 1,8V + SysVoutCODE x 100mV */
        }

        syscfg.code = syscode << 1; /*SYS_VOUT code start from Bit1 */
        syscfg.enable = BQ_SYS_VOUT_ENABLE;
        (void)bq_set_sys_vout(&pmic_dev, syscfg);
        nrf_gpio_pin_write(VDDIO_EN, 1);
    }

    return ret;
}
#endif

/*!
 * @brief This API returns the number of milliseconds passed since the program started
 */
uint32_t coines_get_millis(void)
{
    return g_millis;
}

/*!
 * @brief This API returns the number of microseconds passed since the program started
 */
uint64_t coines_get_micro_sec(void)
{
    uint32_t millis_count;
    uint32_t systick_count;
    uint32_t pend_flag2, pend_flag;

    systick_count = SysTick->VAL;
    pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    millis_count = coines_get_millis();

    do
    {
        pend_flag = pend_flag2;
        pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    } while ((pend_flag != pend_flag2) || (millis_count != coines_get_millis()) || (systick_count < SysTick->VAL));

    /*lint -e524 -e647 -e737*/
    return ((millis_count + pend_flag) * 1000) +
           (((SysTick->LOAD - systick_count) * (1048576 / (CPU_FREQ_HZ / 1000000))) >> 20);
}

/**
 * @brief Weak function for the yield call
 */
static void weak_yield(void);

/**
 * @brief This API can be defined to perform a task when yielded from an ongoing blocking call
 */
void coines_yield(void) __attribute__ ((weak, alias("weak_yield")));

/**
 * @brief Empty function for the dummy yield call
 */
static void weak_yield(void) /*lint -e528 */
{

}

/*!
 *  @brief This API is used to execute the function inside critical region.
 */
void coines_execute_critical_region(coines_critical_callback callback)
{
    CRITICAL_REGION_ENTER();
    if (callback)
    {
        callback();
    }

    CRITICAL_REGION_EXIT();
}
