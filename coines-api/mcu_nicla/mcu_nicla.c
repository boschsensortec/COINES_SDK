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
 * @file    mcu_nicla.c
 * @date    May 10, 2022
 * @brief   COINES_SDK support file for NICLA MCU
 *
 */

#include <stddef.h>
#include <stdbool.h>
#include "mcu_nicla.h"
#include "mcu_nicla_interface.h"

#define CPU_FREQ_HZ  64000000                     /**<ARM cortex M4 is running at 64Mhz*/

volatile bool serial_connected = false;
extern volatile uint16_t serial_idx, read_idx;

volatile bool ble_nus_connected = false;
volatile bool ble_bas_connected = false;
extern volatile size_t ble_nus_available;

volatile uint32_t g_millis = 0;

volatile bool tx_pending = false;
char *ble_device_name;
enum coines_tx_power ble_tx_power = COINES_TX_POWER_4_DBM;

static ISR_CB isr_cb[MAX_PIN_NUM]; /* Max pins on the nRF52832 + 1 */
static void gpiohandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

/*lint -e26 */
uint8_t multi_io_map[COINES_SHUTTLE_PIN_MAX] = {
    [COINES_SHUTTLE_PIN_9] = 0xFF, [COINES_SHUTTLE_PIN_14] = 0xFF, [COINES_SHUTTLE_PIN_15] = 0xFF,
    [COINES_SHUTTLE_PIN_16] = 0xFF, [COINES_SHUTTLE_PIN_22] = 0xFF, [COINES_SHUTTLE_PIN_8] = GPIO_16_SWO,
    [COINES_SHUTTLE_PIN_20] = 0xFF, [COINES_SHUTTLE_PIN_21] = GPIO_13, [COINES_SHUTTLE_PIN_19] = 0xFF,
    [COINES_SHUTTLE_PIN_7] = SPI0_SEN_CS_PIN, 0, 0, 0, 0, 0, 0,

    /* Native APP3.0 pins */
    [COINES_MINI_SHUTTLE_PIN_1_4] = 0xFF, [COINES_MINI_SHUTTLE_PIN_1_5] = 0xFF, [COINES_MINI_SHUTTLE_PIN_1_6] = GPIO_13,
    [COINES_MINI_SHUTTLE_PIN_1_7] = 0xFF, [COINES_MINI_SHUTTLE_PIN_2_5] = 0xFF,
    [COINES_MINI_SHUTTLE_PIN_2_6] = GPIO_16_SWO, [COINES_MINI_SHUTTLE_PIN_2_1] = SPI0_SEN_CS_PIN,
    [COINES_MINI_SHUTTLE_PIN_2_3] = 0xFF, 0, 0, 0,

    /*lint -e485 */
    [COINES_NICLA_BUTTON_RESET] = GPIO_RESET,

    /*lint -e485 */
    [COINES_NICLA_CD_PIN] = BQ_CD, [COINES_MINI_SHUTTLE_PIN_2_7] = 0xFF, [COINES_MINI_SHUTTLE_PIN_2_8] = 0xFF,
    [COINES_SHUTTLE_PIN_SDO] = 0xFF,

    /*lint -e785 */
};

/* Input pin configuration */
static nrfx_gpiote_in_config_t gpio_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);

#define DEBOUNCING_TIME        180 /* 180 ms */
#define BUTTON_PRESS_TIME_OUT  1000000  /* 1 SECOND */

static volatile uint8_t button_pressed_count = 0;   /*button pressed count*/
static volatile uint32_t current_button_pressed_time = 0;   /*current button pressed time in ms*/
static volatile uint32_t last_button_pressed_time = 0;  /*last button pressed time in ms*/

/*For PMIC I2C communication*/
struct bq_dev pmic_dev =
{ .i2c_address = BQ_DRV_ADDR, .cd_pin_state = 1, .read = common_i2c_read, .write = common_i2c_write,
  .delay_ms = common_delay_ms, .cd_set = common_pmic_cd_set };
static volatile uint8_t pmic_battery_percent = 0;

static uint8_t battery_level_in_percentage(const uint16_t mvolts);

/*For LED I2C communication*/
struct led_dev led_dev =
{ .i2c_address = LED_DRV_ADDR, .read = common_i2c_read, .write = common_i2c_write, .delay_ms = common_delay_ms };

#define PMIC_FAULTS_READ_INTERVAL  APP_TIMER_TICKS(5000)     /* PMIC fault read interval (ticks). This value corresponds
                                                              * to 5 seconds. */
APP_TIMER_DEF(bq_fault_read_timer_id); /* PMIC fault read timer. */

#define FS_BUFFER_SIZE             128
#define FS_LOOK_AHEAD_SIZE         32

/**********************************************************************************/
/* Buffers used by FS */
/**********************************************************************************/
static uint8_t file_buffer[FS_BUFFER_SIZE];
static uint8_t read_buffer[FS_BUFFER_SIZE];
static uint8_t prog_buffer[FS_BUFFER_SIZE];
static uint8_t lookahead_buffer[FS_LOOK_AHEAD_SIZE / 4U];

/* Variables used by the file system */
lfs_t lfs;
lfs_file_t file[MAX_FILE_DESCRIPTORS + 1];

/**********************************************************************************/
/* Configuration for file */
/**********************************************************************************/
const struct lfs_file_config file_cfg = {
    /*lint -e785 */
    .buffer = file_buffer,
};

/**********************************************************************************/
/* Configuration of the filesystem */
/**********************************************************************************/
const struct lfs_config cfg = {
    .context = NULL,

    /* block device operations */
    .read = flash_read, .prog = flash_prog, .erase = flash_erase, .sync = flash_sync,

    /* block device configuration */
    .read_size = FS_BUFFER_SIZE, .prog_size = FS_BUFFER_SIZE, .block_size = 4096, .block_count = 512, /* 2 Mbyte
                                                                                                       * external flash
                                                                                                      */
    .block_cycles = 500, .cache_size = FS_BUFFER_SIZE, .lookahead_size = FS_LOOK_AHEAD_SIZE,

    /* Buffers */
    .read_buffer = read_buffer, .prog_buffer = prog_buffer, .lookahead_buffer = lookahead_buffer, .name_max = 64,
    .file_max = 0, .attr_max = 0, .metadata_max = 0,
};

/* Variables used by the re-targeting api */
volatile bool fd_in_use[MAX_FILE_DESCRIPTORS] = { false };
volatile int file_descriptors_used = 0;

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
 * @brief Callback for Button Timer Instance 0
 */
void select_operation_on_button_press(void)
{
    coines_timer_stop(COINES_TIMER_INSTANCE_2);
    current_button_pressed_time = 0;
    last_button_pressed_time = 0;
    /*uint8_t bq_stat = 0; */

    switch (button_pressed_count)
    {
        case 1:
            /*Reset*/
            button_pressed_count = 0;
            (void)bq_device_reset(&pmic_dev);
            (void)bq_delay_ms(&pmic_dev, 50);
            NVIC_SystemReset();
            break;
        case 2:
            /*Enable Ship Mode*/
            button_pressed_count = 0;
            printf("\r\nEnable Ship Mode\r\n");
            (void)bq_cd_set(&pmic_dev, 1);
            (void)bq_delay_ms(&pmic_dev, 10);
            (void)bq_set_mode_config(&pmic_dev, BQ_SHIP_MODE_ENABLE);
            (void)bq_delay_ms(&pmic_dev, 10);
            (void)bq_cd_set(&pmic_dev, 0);

            break;
        case 3:
            /*BLE DFU Mode*/
            button_pressed_count = 0;
            memcpy((uint32_t *)MAGIC_LOCATION, "BOOT", 4); /* 0x544F4F42 ==> 'T' ,'O', 'O', 'B' */
            NVIC_SystemReset();
            break;

        default:
            button_pressed_count = 0;
            break;

    }

}

/*!
 * @brief Callback for button 1 event
 */
void button1_cb_nicla(uint32_t param1, uint32_t param2)
{
    (void)param1;
    (void)param2;

    current_button_pressed_time = coines_get_millis();

    /*To Analyze button press time interval*/

    /*static volatile unsigned int buffer[100];
      static unsigned int i =0;
      buffer[i++] = coines_get_millis();*/

    if (button_pressed_count != 0)
    {
        if ((current_button_pressed_time - last_button_pressed_time) > DEBOUNCING_TIME)
        {
            coines_timer_stop(COINES_TIMER_INSTANCE_2);
            last_button_pressed_time = current_button_pressed_time;
            button_pressed_count++;
            coines_timer_start(COINES_TIMER_INSTANCE_2, BUTTON_PRESS_TIME_OUT);
        }
    }
    else
    {
        last_button_pressed_time = current_button_pressed_time;
        button_pressed_count++;
        coines_timer_start(COINES_TIMER_INSTANCE_2, BUTTON_PRESS_TIME_OUT);
    }
}

/**@brief Function for converting battery voltage to percentage.
 *
 * @details This is just an estimated percentage considering Maximum charging voltage as 4.2 and cut-off voltage as 3.0.
 *          It will vary between different batteries
 */
static uint8_t battery_level_in_percentage(const uint16_t mvolts)
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
 * @brief Callback for pmic reading and vin detection Timer Instance 1
 */
void pmic_reading_vin_detection(void)
{
    struct bq_status bqstatus;
    uint8_t bat_vbbm = 0;
    int8_t rslt;
    uint16_t batt_status_in_milli_volts = 0;

    /*update charging status*/
    if (pmic_dev.cd_pin_state == 1)
    {
        (void)bq_charge_enable(&pmic_dev);
    }

    (void)bq_get_status(&pmic_dev, &bqstatus);
    if (bqstatus.status == 0) /*Status different from "Charge in progress" and "Charge done" */
    {
        (void)bq_charge_disable(&pmic_dev);
    }
    else
    {
        /*NTD*/
        /*Keep CD low when Vin is present => Charge enabled*/
    }

    /*update battery percentage*/

    rslt = bq_get_battery_voltage(&pmic_dev, &bat_vbbm); /*Voltage Based Battery Monitor */
    if (BQ_OK == rslt)
    {
        batt_status_in_milli_volts = (uint16_t) (((bat_vbbm * 4.2) / 100.0) * 1000.0); /*Convert VBMON percentage to
                                                                                        * voltage => Volt = VBMON *
                                                                                        * VBATREG */
        pmic_battery_percent = battery_level_in_percentage(batt_status_in_milli_volts);
    }
}

uint8_t pmic_pull_battery_level(void)
{
    return pmic_battery_percent;
}

/*!
 * @brief Callback for pmic fault detection
 */
static void pmic_fault_detection_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    struct fault_mask_reg faults = { 0, 0, 0, 0 };

    (void)bq_get_faults(&pmic_dev, &faults);

}

/*!
 * @brief FS error checking
 */
__STATIC_INLINE void fs_check_error(int error)
{
    if (error < 0)
    {
        /*lint -e774 -e831 */
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
}

/*!
 * @brief Mounts flash for the first time
 */
void fs_init(void)
{

    /* mount the filesystem */
    int err = lfs_mount(&lfs, &cfg);

    /* reformat if we can't mount the filesystem, this should only happen on the first boot */
    if (err)
    {
        printf("\r\nUnable to mount fs.\r\n");

        err = lfs_format(&lfs, &cfg);
        if (err)
        {
            fs_check_error(err);
        }

        err = lfs_mount(&lfs, &cfg);

        if (err)
        {
            fs_check_error(err);
        }
    }
}

/*!
 * @brief De-initialize flash
 */
void fs_deinit(void)
{
    int err = lfs_unmount(&lfs);

    if (err)
    {
        fs_check_error(err);
    }
}

/*!
 * @brief This API is used to initialize the communication according to interface type.
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void*arg)
{
    uint32_t error_status = NRF_SUCCESS;

    struct coines_comm_intf_config *intfconfig = (struct coines_comm_intf_config *)arg;

    if (intfconfig == NULL)
    {
        return COINES_E_NULL_PTR;
    }

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

    if (NRFX_SUCCESS != nrfx_gpiote_init())
    {
        error_status |= NRF_GPIO_INIT_FAILED_MASK;
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
                 NRF_GPIO_INIT_FAILED_MASK | NRF_ADC_INIT_FAILED_MASK))
            {
                return COINES_E_INIT_FAILED;
            }
        }
    }

    /* Initialize i2c interface */
    (void)common_i2c_init();

    /* Initialize power module */
    (void)common_pmic_cd_init();
    (void)bq_init(&pmic_dev);

    /* Initialize Timer for periodical pmic reg reading and vin detection every 1 second to prevent watchdog reset*/
    if (COINES_SUCCESS != coines_timer_config(COINES_TIMER_INSTANCE_1, pmic_reading_vin_detection))
    {
        return COINES_E_INIT_FAILED;
    }
    else
    {
        coines_timer_start(COINES_TIMER_INSTANCE_1, 1000000);
    }

    /* Initialize led module */
    (void)led_init(&led_dev);

    /*Configuring p21 as normal GPIO*/
    reconfig_reset_pin();

    /* Initialize Timer */
    if (COINES_SUCCESS != coines_timer_config(COINES_TIMER_INSTANCE_2, select_operation_on_button_press))
    {
        return COINES_E_INIT_FAILED;
    }

    /*Configuring Button Interrupt*/
    coines_set_pin_config(COINES_NICLA_BUTTON_RESET, COINES_PIN_DIRECTION_IN, COINES_PIN_VALUE_HIGH);
    coines_attach_interrupt(COINES_NICLA_BUTTON_RESET, button1_cb_nicla, COINES_PIN_INTERRUPT_FALLING_EDGE);

    /* Configure NFCT pins as GPIOs. UART RX is assigned to P0.9, same pin that is used as the antenna input for NFC by
     * default*/
    reconfig_nfct_pin();

    if (NRF_SUCCESS != uart_init(COINES_UART_PARITY_NONE, COINES_UART_FLOW_CONTROL_DISABLED, intfconfig->uart_baud_rate))
    {
        return COINES_E_UART_INIT_FAILED;
    }

    if (intf_type == COINES_COMM_INTF_BLE)
    {
        ble_service_init(&init_handle);
    }

    /* Initialize SPI for BHI260 sensor/external flash */
    if (COINES_SUCCESS != coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE0))
    {
        return COINES_E_INIT_FAILED;
    }

    /* Initialize flash module */
    (void)flash_init();

    /* Initialize littleFS filesystem module */
    fs_init();

    if (intf_type != COINES_COMM_INTF_BLE)
    {
        (void)app_timer_init();
    }
    /* Initialize Timer for polling PMIC faults every 5 second*/
    (void)app_timer_create(&bq_fault_read_timer_id, APP_TIMER_MODE_REPEATED, pmic_fault_detection_timeout_handler);
    (void)app_timer_start(bq_fault_read_timer_id, PMIC_FAULTS_READ_INTERVAL, NULL);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the pin(MULTIIO/SPI/I2C in shuttle board).
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin multiio_pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{
    nrf_gpio_pin_pull_t pin_pull;

    uint32_t pin_num = multi_io_map[multiio_pin_number];

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
int16_t coines_get_pin_config(enum coines_multi_io_pin multiio_pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    uint32_t pin_num = multi_io_map[multiio_pin_number];

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

/*!
 * @brief Attaches a interrupt to a Multi-IO pin
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                             void (*callback)(uint32_t, uint32_t),
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
    isr_cb[pin_num] = callback;
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
    nrfx_gpiote_in_uninit(pin_num);
    nrfx_gpiote_in_event_disable(pin_num);
}

/*!
 * @brief GPIO Interrupt handler
 */
static void gpiohandler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (isr_cb[pin] != NULL)
    {
        isr_cb[pin](pin, (uint32_t)action);
    }
}

uint16_t coines_intf_available(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
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
        /* return serial_connected; TODO How to check COM (USB-Serial Bridge) is connected or not in NICLA */
    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        return ble_nus_connected;
    }

    return false;
}

void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{

    if (intf == COINES_COMM_INTF_USB)
    {
        uint8_t * tx_data = (uint8_t *)buffer;

        uart_write(tx_data, len);

    }
    else if (intf == COINES_COMM_INTF_BLE)
    {
        (void)ble_service_nus_write(buffer, len);
    }
}

uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    uint16_t bytes_read = 0;

    if (intf == COINES_COMM_INTF_USB)
    {
        bytes_read = uart_read((uint8_t*)buffer, len);
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
#ifndef SONAR_ANALYSIS
void SysTick_Handler(void)
{
    g_millis++;
}
#endif

/* For stdio functions */

int _write(int fd, const char * p_char, int len)
{
    int i;

    if ((fd == 1 || fd == 2))
    {
        for (i = 0; i < len; i++)
        {
            UNUSED_VARIABLE(uart_write((uint8_t *)p_char++, 1));
        }

        return len;
    }
    else if (fd >= 3 && fd < MAX_FILE_DESCRIPTORS + 3)
    {
        return (int)lfs_file_write(&lfs, &file[fd - 3], p_char, (uint32_t)len);
    }
    else
    {
        return len;
    }
}

int _read(int fd, char * p_char, int len)
{
    if (fd == 0)
    {
        while ((serial_idx - read_idx) < len)
            ;

        return uart_read((uint8_t*)p_char, (uint16_t)len);
    }
    else if (fd >= 3 && fd < MAX_FILE_DESCRIPTORS + 3)
    {
        return (int)lfs_file_read(&lfs, &file[fd - 3], p_char, (uint32_t)len);
    }
    else
    {
        return len;
    }
}

int __attribute__((weak)) _open(const char *file_name, int flags)
{
    int fd = 0;
    bool fd_value;

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
        fd_value = fd_in_use[i];
        if (fd_value == false)
        {
            fd = i + 3;
            fd_in_use[i] = true;
            break;
        }
    }

    if (flags == O_RDONLY)
    {
        if (lfs_file_opencfg(&lfs, &file[fd - 3], file_name, LFS_O_RDONLY, &file_cfg) < 0)
        {
            return -1;
        }
    }
    else
    {
        if (lfs_file_opencfg(&lfs, &file[fd - 3], file_name, LFS_O_RDWR | LFS_O_CREAT, &file_cfg) < 0)
        {
            return -1;
        }
    }

    file_descriptors_used++;

    return fd;
}

int __attribute__((weak)) _close(int fd)
{
    if (lfs_file_close(&lfs, &file[fd - 3]) < 0)
    {
        return -1;
    }

    fd_in_use[fd - 3] = false;
    file_descriptors_used--;

    return 0;
}

int __attribute__((weak)) _unlink(const char *file_name)
{
    if (lfs_remove(&lfs, file_name) < 0)
    {
        return -1;
    }

    return 0;
}

int __attribute__((weak)) _stat(const char *file_name, struct stat *st)
{
    struct lfs_info info;

    if (lfs_stat(&lfs, file_name, &info) < 0)
    {
        return -1;

    }

    if (lfs_file_open(&lfs, &file[5], file_name, LFS_O_RDONLY) < 0)
    {
        return -1;
    }

    memset(st, 0, sizeof(struct stat));
    st->st_mode = S_IFREG;
    st->st_size = lfs_file_size(&lfs, &file[5]);
    st->st_blksize = 4095;

    if (lfs_file_close(&lfs, &file[5]) < 0)
    {
        return -1;
    }

    return 0;
}

int __attribute__((weak)) _fstat(int file_desc, struct stat *buf)
{
    (void)file_desc;
    (void)buf;

    return -1;
}

int __attribute__((weak)) _isatty(int file_desc)
{
    (void)file_desc;

    return -1;
}

int __attribute__((weak)) _lseek(int file_desc, int pos, int whence)
{
    (void)file_desc;
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

/* Read and print the entries in the directory */
struct lfs_info info;
lfs_dir_t dir;

int opendir(const char *dirname)
{
    (void)dirname;
    if (lfs_dir_open(&lfs, &dir, dirname) < 0)
    {
        printf("\r\n dir not opened\r\n");

        return -1;
    }

    return 0;
}

char readdir(char* name)
{
    if (lfs_dir_read(&lfs, &dir, &info) > 0)
    {
        memcpy(name, info.name, 64);

        return 1;
    }
    else
    {
        return 0;
    }
}

int closedir(void)
{
    (void)lfs_dir_close(&lfs, &dir);

    return 0;
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
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state)
{
    int16_t retval;
    uint8_t state = 0;

    switch (led_state)
    {
        case COINES_LED_STATE_ON:
            state = 0xFF; /*led ON with full intensity */
            break;
        case COINES_LED_STATE_OFF:
            state = 0x00; /*Led OFF */
            break;
        default:
            break;
    }
    switch (led)
    {
        case COINES_LED_RED:
            /*retval = nicla_led_set_rgb_color(COINES_LED_RED, (enum coines_pin_value)led_state); */
            retval = led_setcolorandintensity(&led_dev, RED, state);
            break;
        case COINES_LED_GREEN:
            /*retval = nicla_led_set_rgb_color(COINES_LED_GREEN, (enum coines_pin_value)led_state); */
            retval = led_setcolorandintensity(&led_dev, GREEN, state);
            break;
        case COINES_LED_BLUE:
            /*retval = nicla_led_set_rgb_color(COINES_LED_BLUE, (enum coines_pin_value)led_state); */
            retval = led_setcolorandintensity(&led_dev, BLUE, state);
            break;
        default:
            retval = COINES_E_NOT_SUPPORTED;
            break;
    }

    return retval;
}

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
    uint32_t millis;
    uint32_t systick_count;
    uint32_t pend_flag2, pend_flag;

    systick_count = SysTick->VAL;
    pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    millis = coines_get_millis();

    do
    {
        pend_flag = pend_flag2;
        pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    }while ((pend_flag != pend_flag2) || (millis != coines_get_millis()) || (systick_count < SysTick->VAL));

    /*lint -e524 -e647 */
    return ((millis + pend_flag) * 1000) +
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

int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    (void)vdd_millivolt;
    struct bq_load_switch ldocfg;
    uint8_t lscode = 0;

    ldocfg.enable = BQ_LOAD_LDO_ENABLE;
    ldocfg.reset = BQ_LOAD_LDO_RESET_TIME;

    if (vddio_millivolt > 800)
    {
        lscode = (uint8_t) ((vddio_millivolt - 800) / 100); /*Output voltage = 0,8V + Ls_LDOCODE x 100mV */
    }
    else if (vddio_millivolt == 0)
    {
        ldocfg.enable = BQ_LOAD_LDO_DISABLE;
    }

    ldocfg.code = (uint8_t)(lscode << 2); /*LS_LDO code start from Bit2 */

    return bq_set_load_ldo(&pmic_dev, ldocfg);
}

void coines_flush_intf(enum coines_comm_intf intf)
{
    UNUSED_PARAMETER(intf);
}

/*!
 * TODO
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
    UNUSED_PARAMETER(temp_data);

    return COINES_SUCCESS;
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

/*!
 * @brief This API is used to switch off the board
 */
void coines_ship_mode(void)
{
    (void)bq_cd_set(&pmic_dev, 1);
    (void)bq_delay_ms(&pmic_dev, 10);
    (void)bq_set_mode_config(&pmic_dev, BQ_SHIP_MODE_ENABLE);
    (void)bq_delay_ms(&pmic_dev, 10);
    (void)bq_cd_set(&pmic_dev, 0);
}
