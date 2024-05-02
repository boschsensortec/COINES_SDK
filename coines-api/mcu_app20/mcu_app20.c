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
 * @file    mcu_app20.c
 * @date    Jan 2, 2019
 * @brief   COINES_SDK support file for APP2.0 MCU
 *
 */

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/
#include <stddef.h>
#include "coines.h"
#include "mcu_app20.h"

#define CPU_FREQ_HZ                    120000000  /**<ARM cortex M4 is running at 120Mhz*/
#define TC0_CHANNEL_CAPTURE 		   0
#define TC1_CHANNEL_CAPTURE 		   1
#define TC2_CHANNEL_CAPTURE 		   2
#define TC_INTERRUPT_PRIORITY		   6 /** Configurable priority values are in the range 0–15 */
#define DESIRED_DEFAULT_FREQUENCY	   50

#define TC_CHANNEL_0        0
#define TC_CHANNEL_1        1
#define TC_CHANNEL_2        2
/**********************************************************************************/
/* variable declarations */
/**********************************************************************************/

bool serial_connected = false;
uint32_t baud_rate = 0;
uint32_t g_millis = 0;

typedef void (*ISR_CB)(uint32_t id, uint32_t index);

static ISR_CB isr_cb[9];

static volatile uint32_t timer_overflow_count = 0;

static const uint8_t timer_counter_channel[COINES_TIMER_INSTANCE_MAX] = {
		ID_TC1, ID_TC2
};

/* Timer instance loop-up table
  TIMER_INSTANCE(0) - used by free running timer for timed interrupt
  TIMER_INSTANCE(1) - available for user
  TIMER_INSTANCE(2) - available for user
*/
static const uint8_t timer_instance[COINES_TIMER_INSTANCE_MAX] = {

    TC1_CHANNEL_CAPTURE, TC2_CHANNEL_CAPTURE
    /*lint -e133*/
};

static volatile bool is_timer_enabled[COINES_TIMER_INSTANCE_MAX] = { false, false };

#define TIMER_TICKS_TO_NSEC(t)         (((uint64_t)t * UINT64_C(1000)) / 30)
/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

/*!
 *
 * @brief       : GPIO Interrupt Handler
 *
 * @param[in]   : id,index
 *
 * @return      : None
 */
static void piohandler(uint32_t id, uint32_t index);

/**********************************************************************************/
/* functions */
/**********************************************************************************/
/*!
 *
 * @brief       :  TC0, Channel 2 handler
 *
 */
void tc2_handler(void)
{
	(void)tc_get_status(TC0, TC_CHANNEL_2);
	timer_interrupt_handler[1]();
}

/*!
 *
 * @brief       : TC0, Channel 1 handler
 *
 */
void tc1_handler(void)
{
    (void)tc_get_status(TC0, TC_CHANNEL_1);
	timer_interrupt_handler[0]();
}

/*!
 *
 * @brief       : Counter overflow interrupt handler: TC0, Channel 0
 *
 */
void tc0_handler(void)
{
    (void)tc_get_status(TC0, TC_CHANNEL_0);
	timer_overflow_count++;
}

/*!
 *
 * @brief       : API to get the hardware pin
 *
 */
uint32_t get_hw_pin(enum coines_multi_io_pin shuttle_pin)
{

#define PIN_MAP(sp, hp) \
    case COINES_SHUTTLE_PIN_##sp:  \
        return SHUTTLE_##hp        \

    switch (shuttle_pin)
    {

            PIN_MAP(7, SPI_CS0);
            PIN_MAP(8, IO5);
            PIN_MAP(9, IO0);
            PIN_MAP(14, IO1);
            PIN_MAP(15, IO2);
            PIN_MAP(16, IO3);
            PIN_MAP(19, IO8);
            PIN_MAP(20, IO6);
            PIN_MAP(21, IO7);
            PIN_MAP(22, IO4);
            PIN_MAP(SDO,SDO);
        default:
            break;
    }
    /*typecasted for pclint(info) - Error check not done in any calling function*/
    return (uint32_t)COINES_E_NOT_SUPPORTED;

}

    /*!
     *
     * @brief       : API to get the shuttle ID
     */
static uint16_t get_shuttle_id()
{
    uint32_t shuttle_id = 0;

    shuttle_id |= pio_get_pin_value(SHUTTLE_COD0) << 0;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD1) << 1;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD2) << 2;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD3) << 3;

    shuttle_id |= pio_get_pin_value(SHUTTLE_COD4) << 4;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD5) << 5;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD6) << 6;
    shuttle_id |= pio_get_pin_value(SHUTTLE_COD7) << 7;

    shuttle_id |= pio_get_pin_value(SHUTTLE_COD8) << 8;

    return (uint16_t)shuttle_id;
}

/*!
 *
 * @brief       : This API is to check communication port connection
 *
 */
void check_com_port_connection(int set)
{
    static bool reset_board = false;
    uint32_t timeout = 100;
    
    if (set == 1)
    {
        serial_connected = true;
    }

    /*
     *  Trigger MCU reset if USB CDC port is opened and closed at 1200bps
     *  https://www.arduino.cc/en/Main/Arduino_BoardLeonardo
     *  See # Automatic (Software) Reset and Bootloader Initiation
     */
    if (set == 1 && baud_rate == 1200)
    {
        reset_board = true;
    }

    if (set == 0 && reset_board == true && (baud_rate == 1200))
    {
        /*lint -e{747} -e{712}*/
        delay_ms(timeout);
        APP_START_ADDR = 0; /*Jump to USB DFU Bootloader */
        rstc_start_software_reset(RSTC);
    }
}

void get_connection_parameters(usb_cdc_line_coding_t *cfg)
{
    baud_rate = cfg->dwDTERate;
}

/*!
 * @brief This API is used to initialize the communication according to interface type.
 *
 */
int16_t coines_open_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    (void)arg;
    (void)intf_type;

    uint32_t timeout = 100;
    sysclk_init();
    wdt_disable(WDT);

    /*For coines_get_millis() API*/
    (void)SysTick_Config(sysclk_get_peripheral_hz() / 1000);

    sysclk_enable_peripheral_clock(ID_PIOA);
    sysclk_enable_peripheral_clock(ID_PIOB);
    sysclk_enable_peripheral_clock(ID_PIOC);

    (void)pio_configure_pin(SHUTTLE_VDD_EN, PIO_OUTPUT_0);
    (void)pio_configure_pin(SHUTTLE_VDDIO_EN, PIO_OUTPUT_0);

    (void)pio_configure_pin(SHUTTLE_COD0, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD1, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD2, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD3, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD4, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD5, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD6, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD7, PIO_INPUT);
    (void)pio_configure_pin(SHUTTLE_COD8, PIO_INPUT);

    /* For coines_attach_interrupt() API*/
    NVIC_DisableIRQ(PIOC_IRQn);
    NVIC_ClearPendingIRQ(PIOC_IRQn);
    NVIC_SetPriority(PIOC_IRQn, 6);

    irq_initialize_vectors();
    /*lint -e{746} -e{534} prototype not made presence & ignore return value */ 
    cpu_irq_enable();
    stdio_usb_init();

    while (!serial_connected)
    {
        /*lint -e{747} -e{712}*/
        delay_ms(timeout);
    }

    /** Configure TC0 channel 0 as free running timer for timed Interrupt */
    sysclk_enable_peripheral_clock(ID_TC0);

    tc_init(TC0, TC0_CHANNEL_CAPTURE,  TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_CPCTRG);
    tc_write_rc(TC0, TC0_CHANNEL_CAPTURE, 0);

    NVIC_DisableIRQ((IRQn_Type) ID_TC0);
    NVIC_ClearPendingIRQ((IRQn_Type) ID_TC0);
    NVIC_SetPriority((IRQn_Type) ID_TC0, TC_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ((IRQn_Type) ID_TC0);

    tc_enable_interrupt(TC0, TC0_CHANNEL_CAPTURE, TC_IER_COVFS);
    tc_start(TC0, TC0_CHANNEL_CAPTURE);

    return COINES_SUCCESS;
}

/*!
 *
 * @brief       This API is used to close the active communication(USB,COM or BLE).
 *
 */
int16_t coines_close_comm_intf(enum coines_comm_intf intf_type, void *arg)
{
    (void)arg;
    (void)intf_type;
    (void)fflush(stdout);

    /** Disabling and clearing TC0 channel 0 for timestamp */
    tc_disable_interrupt(TC0, TC0_CHANNEL_CAPTURE, TC_IER_COVFS);
    tc_stop(TC0, TC0_CHANNEL_CAPTURE);

    NVIC_ClearPendingIRQ((IRQn_Type) ID_TC0);
    NVIC_DisableIRQ((IRQn_Type) ID_TC0);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to get the board information.
 *
 *
 */
int16_t coines_get_board_info(struct coines_board_info *p_data)
{

    if (p_data != NULL)
    {
        p_data->board = 3;
        p_data->hardware_id = 0x11;
        p_data->shuttle_id = get_shuttle_id();
        p_data->software_id = 0x01;

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/*!
 * @brief   This API is used to close the active communication(USB,COM or BLE).
 *
 *
 */
int16_t coines_set_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction direction,
                              enum coines_pin_value pin_value)
{

    if (direction == COINES_PIN_DIRECTION_IN)
    {
        if (pin_value == COINES_PIN_VALUE_LOW)
        {
            (void)pio_configure_pin(get_hw_pin(pin_number), PIO_INPUT | 0);
        }
        else if (pin_value == COINES_PIN_VALUE_HIGH)
        {
            (void)pio_configure_pin(get_hw_pin(pin_number), PIO_INPUT | PIO_PULLUP);
        }
    }
    else if (direction == COINES_PIN_DIRECTION_OUT)
    {
        if (pin_value == COINES_PIN_VALUE_LOW)
        {
            (void)pio_configure_pin(get_hw_pin(pin_number), PIO_OUTPUT_0);
        }
        else if (pin_value == COINES_PIN_VALUE_HIGH)
        {
            (void)pio_configure_pin(get_hw_pin(pin_number), PIO_OUTPUT_1);
        }
    }
    
    
    return COINES_SUCCESS;
}

/*!
 *
 *  @brief This API function is used to get the pin direction and pin state.
 *
 */
int16_t coines_get_pin_config(enum coines_multi_io_pin pin_number,
                              enum coines_pin_direction *pin_direction,
                              enum coines_pin_value *pin_value)
{
    if ((pin_value != NULL) || (pin_direction != NULL))
    {
        if (pin_value != NULL)
        {
            if (pio_get_pin_value(get_hw_pin(pin_number)) == 0)
            {
                *pin_value = COINES_PIN_VALUE_LOW;
            }
            else
            {
                *pin_value = COINES_PIN_VALUE_HIGH;
            }
        }

        if (pin_direction != NULL)
        {
			Pio *p_pio = pio_get_pin_group(get_hw_pin(pin_number));
			
            if((p_pio->PIO_OSR & (1 << (get_hw_pin(pin_number) & 0x1F))) == 0)
    		{
				 *pin_direction = COINES_PIN_DIRECTION_IN;
    		}
			else
			{
				*pin_direction = COINES_PIN_DIRECTION_OUT;
			}
        }

        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_NULL_PTR;
    }
}

/*!
 *  @brief This API is used to configure the VDD and VDDIO of the sensor.
 *
 */
int16_t coines_set_shuttleboard_vdd_vddio_config(uint16_t vdd_millivolt, uint16_t vddio_millivolt)
{
    if (vdd_millivolt > 0)
    {
        pio_set_pin_high(SHUTTLE_VDD_EN);
    }
    else
    {
        pio_set_pin_low(SHUTTLE_VDD_EN);
    }

    if (vddio_millivolt > 0)
    {
        pio_set_pin_high(SHUTTLE_VDDIO_EN);
    }
    else
    {
        pio_set_pin_low(SHUTTLE_VDDIO_EN);
    }

    return COINES_SUCCESS;

}

/*!
 *  @brief This API is used to configure the SPI bus
 *
 */
int16_t coines_config_spi_bus(enum coines_spi_bus bus, enum coines_spi_speed spi_speed, enum coines_spi_mode spi_mode)
{
    (void)bus;
    struct spi_device device;

    device.id = 0;

    sysclk_enable_peripheral_clock(ID_SPI);
    sysclk_disable_peripheral_clock(ID_TWI0);

    (void)pio_configure(PIOA, PIO_PERIPH_A, PIO_PA12A_MISO, 1);
    (void)pio_configure(PIOA, PIO_PERIPH_A, PIO_PA13A_MOSI, 1);
    (void)pio_configure(PIOA, PIO_PERIPH_A, PIO_PA14A_SPCK, 1);

    spi_master_init(SPI);
    spi_master_setup_device(SPI, &device, spi_mode, sysclk_get_peripheral_hz() / spi_speed, device.id);
    spi_enable(SPI);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to de-configure the SPI bus
 */
int16_t coines_deconfig_spi_bus(enum coines_spi_bus bus)
{
    (void)bus;
    spi_disable(SPI);

    return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to configure the I2C bus
 *
 */
int16_t coines_config_i2c_bus(enum coines_i2c_bus bus, enum coines_i2c_mode i2c_mode)
{
    (void)bus;
    
    twi_options_t i2c_config;

    i2c_config.master_clk = sysclk_get_peripheral_hz();

    sysclk_enable_peripheral_clock(ID_TWI0);
    sysclk_disable_peripheral_clock(ID_SPI);

    (void)pio_configure(PIOA, PIO_PERIPH_A, PIO_PA3A_TWD0, 1);
    (void)pio_configure(PIOA, PIO_PERIPH_A, PIO_PA4A_TWCK0, 1);

    if (i2c_mode == COINES_I2C_STANDARD_MODE)
    {
        i2c_config.speed = 100000;
    }
    else if (i2c_mode == COINES_I2C_FAST_MODE)
    {
        i2c_config.speed = 400000;
    }
    else if (i2c_mode == COINES_I2C_SPEED_1_7_MHZ)
    {
        i2c_config.speed = 1700000;
    }
    else if (i2c_mode == COINES_I2C_SPEED_3_4_MHZ)
    {
        i2c_config.speed = 3400000;
    }

    if(twi_master_init(TWI0, &i2c_config) == TWI_SUCCESS)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_FAILURE;
    }
}

/*!
 *  @brief This API is used to de-configure the I2C bus
 */
int16_t coines_deconfig_i2c_bus(enum coines_i2c_bus bus)
{
    (void)bus;
    twi_reset(TWI0);

    return COINES_SUCCESS;
}

/*!
 *  @brief      This API is used to write the data in I2C communication.
 *
 */
int8_t coines_write_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    (void)bus;

    twi_packet_t packet_tx;

    packet_tx.chip = dev_addr;
    packet_tx.addr[0] = reg_addr;
    packet_tx.addr_length = 1;
    packet_tx.buffer = reg_data;
    packet_tx.length = count;

    if(twi_master_write(TWI0, &packet_tx) == TWI_SUCCESS)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_COMM_IO_ERROR;
    }
}

/*!
 *  @brief  This API is used to read the data in I2C communication.
 *
 */
int8_t coines_read_i2c(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    (void)bus;
    
    twi_packet_t packet_rx;

    packet_rx.chip = dev_addr;
    packet_rx.addr[0] = reg_addr;
    packet_rx.addr_length = 1;
    packet_rx.buffer = reg_data;
    packet_rx.length = count;

    if(twi_master_read(TWI0, &packet_rx) == TWI_SUCCESS)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_COMM_IO_ERROR;
    }
}

/*!
 *
 * @brief       : This API is used to write the data in SPI communication.
 *
 */
int8_t coines_write_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    (void)bus;
    pdc_packet_t spi_packet;
    uint8_t tx_buff[count + 1];

    tx_buff[0] = reg_addr;
    memcpy(&tx_buff[1], reg_data, count);

    /*Setup SPI transfer*/
    Pdc *spi_pdc = spi_get_pdc_base(SPI);
    spi_packet.ul_addr = (uint32_t)tx_buff;
    spi_packet.ul_size = count + 1;
    pdc_rx_init(spi_pdc, &spi_packet, NULL);
    spi_packet.ul_addr = (uint32_t)tx_buff;
    spi_packet.ul_size = count + 1;
    pdc_tx_init(spi_pdc, &spi_packet, NULL);

    if (dev_addr == 0)
    {
        dev_addr = COINES_SHUTTLE_PIN_7;
    }

    /*Do the SPI transfer*/
    (void)pio_configure_pin(get_hw_pin((enum coines_multi_io_pin)dev_addr), PIO_OUTPUT_0);
    pdc_enable_transfer(spi_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
    while ((spi_read_status(SPI) & SPI_SR_RXBUFF) == 0)
        ;

    pio_set_pin_high(get_hw_pin((enum coines_multi_io_pin)dev_addr));
    pdc_disable_transfer(spi_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

    return COINES_SUCCESS;
}

/*!
 *
 * @brief       : This API is used to read the data in SPI communication.
 *
 */
int8_t coines_read_spi(enum coines_spi_bus bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    (void)bus;
    pdc_packet_t spi_packet;
    uint8_t tx_buff[count + 1];
    uint8_t rx_buff[count + 1];

    tx_buff[0] = reg_addr;

    /*Setup SPI transfer*/
    Pdc *spi_pdc = spi_get_pdc_base(SPI);
    spi_packet.ul_addr = (uint32_t)rx_buff;
    spi_packet.ul_size = count + 1;
    pdc_rx_init(spi_pdc, &spi_packet, NULL);
    spi_packet.ul_size = count + 1;
    spi_packet.ul_addr = (uint32_t)tx_buff;
    pdc_tx_init(spi_pdc, &spi_packet, NULL);

    if (dev_addr == 0)
    {
        dev_addr = COINES_SHUTTLE_PIN_7;
    }

    /*Do the SPI transfer*/
    (void)pio_configure_pin(get_hw_pin((enum coines_multi_io_pin)dev_addr), PIO_OUTPUT_0);
    pdc_enable_transfer(spi_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
    while ((spi_read_status(SPI) & SPI_SR_RXBUFF) == 0)
        ;

    pio_set_pin_high(get_hw_pin((enum coines_multi_io_pin)dev_addr));
    pdc_disable_transfer(spi_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
    memcpy(reg_data, &rx_buff[1], count);

    return COINES_SUCCESS;
}

/*!
 *
 *  @brief This API is used for introducing a delay in milliseconds
 */
void coines_delay_msec(uint32_t delay_ms)
{
    /*lint -e{747} -e{712}*/
    delay_ms(delay_ms);
}

/*!
 *  @brief This API is used for introducing a delay in microseconds
 */
void coines_delay_usec(uint32_t delay_us)
{
    /*lint -e{747} -e{712}*/
    delay_us(delay_us);
}

/*!
 *
 * @brief   This API is used to send the streaming settings to the board.
 *
 */
int16_t coines_config_streaming(uint8_t channel_id,
                                struct coines_streaming_config *stream_config,
                                struct coines_streaming_blocks *data_blocks)
{
    (void)channel_id;
    (void)stream_config;
    (void)data_blocks;

    return COINES_E_NOT_SUPPORTED;
}

/*!
 *
 * @brief   This API is used to start or stop the streaming.
 *
 */
int16_t coines_start_stop_streaming(enum coines_streaming_mode stream_mode, uint8_t start_stop)
{
    (void)stream_mode;
    (void)start_stop;

    return COINES_E_NOT_SUPPORTED;
}

/*!
 * @brief    This API is used to read the streaming sensor data.
 *
 */
int16_t coines_read_stream_sensor_data(uint8_t sensor_id,
                                       uint32_t number_of_samples,
                                       uint8_t *data,
                                       uint32_t *valid_samples_count)
{
    (void)sensor_id;
    (void)number_of_samples;
    (void)data;
    (void)valid_samples_count;

    return COINES_E_NOT_SUPPORTED;
}

/*!
 *
 * @brief   This API is used to trigger the timer in firmware and enable or disable system time stamp
 *
 */
int16_t coines_trigger_timer(enum coines_timer_config tmr_cfg, enum coines_time_stamp_config ts_cfg)
{
    (void)tmr_cfg;
    (void)ts_cfg;
    return COINES_E_NOT_SUPPORTED;
}

uint32_t coines_get_millis(void)
{
    return g_millis;
}

/*!
 * @brief This API returns the number of microseconds passed since the program started
 */
uint64_t coines_get_micro_sec(void)
{
    uint32_t systick_count;
    uint32_t pend_flag, pend_flag2;
    uint32_t millis_count;

    systick_count = SysTick->VAL;
    pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    millis_count = coines_get_millis();

    do
    {
        pend_flag = pend_flag2;
        pend_flag2 = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) || ((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)));
    }
    while ((pend_flag != pend_flag2) || (millis_count != coines_get_millis()) || (systick_count < SysTick->VAL));
    /*lint -e{647}*/
    return ((millis_count + pend_flag) * 1000) + (((SysTick->LOAD - systick_count) * (1048576 / (CPU_FREQ_HZ / 1000000))) >> 20);
}
/*!
 * @brief This API attaches a interrupt to a Multi-IO pin
 *
 */
void coines_attach_interrupt(enum coines_multi_io_pin pin_number,
                             void (*callback)(uint32_t, uint32_t),
                             enum coines_pin_interrupt_mode int_mode)
{
    uint32_t mcu_int_mode = 0;

    pio_set_input(PIOC, 1 << (get_hw_pin(pin_number) - PIO_PC0_IDX), PIO_INPUT);

    if (int_mode == COINES_PIN_INTERRUPT_CHANGE)
    {
        mcu_int_mode = (PIO_IT_FALL_EDGE) & ~(PIO_IT_AIME);
    }

    if (int_mode == COINES_PIN_INTERRUPT_RISING_EDGE)
    {
        mcu_int_mode = PIO_IT_RISE_EDGE;
    }

    if (int_mode == COINES_PIN_INTERRUPT_FALLING_EDGE)
    {
        mcu_int_mode = PIO_IT_FALL_EDGE;
    }

    (void)pio_handler_set(PIOC, ID_PIOC, 1 << (get_hw_pin(pin_number) - PIO_PC0_IDX), mcu_int_mode, piohandler);
    isr_cb[get_hw_pin(pin_number) - PIO_PC0_IDX] = callback;

    NVIC_DisableIRQ(PIOC_IRQn);
    NVIC_ClearPendingIRQ(PIOC_IRQn);
    NVIC_EnableIRQ(PIOC_IRQn);

    pio_enable_pin_interrupt(get_hw_pin(pin_number));
}

/*!
 *
 * @brief   This API detaches a interrupt from a Multi-IO pin
 *
 */
void coines_detach_interrupt(enum coines_multi_io_pin pin_number)
{
    pio_disable_pin_interrupt(get_hw_pin(pin_number));
}

/*!
 *
 * @brief       : GPIO Interrupt Handler
 */
static void piohandler(uint32_t id, uint32_t pin_index)
{
    int i = 0;

    if (id == ID_PIOC)
    {
        for (i = 0; i < 9; i++)
        {
            if (pin_index & (1 << i))
            {
                isr_cb[i](id, pin_index);
            }
        }
    }
}

/*!
 *
 * @brief       : SysTick timer Handler
 */
void SysTick_Handler(void)
{
    g_millis++;
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
    memcpy((uint32_t *)MAGIC_LOCATION, "COIN", 4); // *MAGIC_LOCATION = 0x4E494F43; // 'N','O','I','C'
    APP_START_ADDR = APP_START_ADDRESS; // Application start address;

    rstc_start_software_reset(RSTC);
}

/*!
 * @brief This API is used to get the current reference time in usec
 *
 * @param[in]   : None
 * @return      : reference time in usec
 * */
uint32_t coines_get_realtime_usec(void)
{
    /* TODO */
    return 0;
}

/*!
 * @brief This API is used to introduce delay based on high precision timer
 *
 * @param[in]   : required delay in microseconds
 * @return      : None
 */
void coines_delay_realtime_usec(uint32_t period)
{
    /* TODO */
    (void)period;
}

/*!
 *  @brief This API is used to set led state(on or off).
 */
int16_t coines_set_led(enum coines_led led, enum coines_led_state led_state)
{
    switch (led)
    {
    case COINES_LED_RED:
        if (led_state == COINES_LED_STATE_ON)
        {
            (void)pio_configure_pin(BOARD_LED1, PIO_OUTPUT_0);
        }
        else
        {
            (void)pio_configure_pin(BOARD_LED1, PIO_OUTPUT_1);
        }
        break;

    case COINES_LED_GREEN:
        if (led_state == COINES_LED_STATE_ON)
        {
            (void)pio_configure_pin(BOARD_LED2, PIO_OUTPUT_0);
        }
        else
        {
            (void)pio_configure_pin(BOARD_LED2, PIO_OUTPUT_1);
        }
        break;
    case COINES_LED_BLUE:
        if (led_state == COINES_LED_STATE_ON)
        {
            (void)pio_configure_pin(BOARD_LED3, PIO_OUTPUT_0);
        }
        else
        {
            (void)pio_configure_pin(BOARD_LED3, PIO_OUTPUT_1);
        }
        break;

    default:
        break;
    }

    return COINES_SUCCESS;
}

/*!
 * @brief Read data over the specified interface
 */
uint16_t coines_read_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        return (uint16_t)udi_cdc_read_buf(buffer, len);
    }

    return 0;
}

/*!
 * @brief Write data over the specified interface
 *
 */
void coines_write_intf(enum coines_comm_intf intf, void *buffer, uint16_t len)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        (void)udi_cdc_write_buf(buffer, len);
    }
}

/*!
 * @brief Return the number of bytes available in the read buffer of the interface
 */
uint16_t coines_intf_available(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        return (uint16_t)udi_cdc_get_nb_received_data();
    }

    return 0;
}

/*!
 *  @brief This API is used to write the data in I2C communication.
 */
int8_t coines_i2c_set(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *p_data, uint8_t count)
{
    (void)bus;
    
    twi_packet_t packet_tx;

    packet_tx.chip = dev_addr;
    packet_tx.addr_length = 0;
    packet_tx.buffer = p_data;
    packet_tx.length = count;

    if(twi_master_write(TWI0, &packet_tx) == TWI_SUCCESS)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_COMM_IO_ERROR;
    }
}

/*!
 *  @brief This API is used to read the data in I2C communication.
 */
int8_t coines_i2c_get(enum coines_i2c_bus bus, uint8_t dev_addr, uint8_t *data, uint8_t count)
{
    (void)bus;
    
    twi_packet_t packet_rx;

    packet_rx.chip = dev_addr;
    packet_rx.addr_length = 0;
    packet_rx.buffer = data;
    packet_rx.length = count;

    if(twi_master_read(TWI0, &packet_rx) == TWI_SUCCESS)
    {
        return COINES_SUCCESS;
    }
    else
    {
        return COINES_E_COMM_IO_ERROR;
    }
}

/*!
 * @brief GPIO event handler for timed interrupt
 */
static void attach_timed_interrupt_handler(uint32_t id, uint32_t pin_index)
{
	uint64_t total_ticks;
	uint32_t latest_ticks_count;
	int i = 0;
	enum coines_multi_io_pin pin_number = COINES_SHUTTLE_PIN_MAX;

	if (id == ID_PIOC)
	{
		for (i = 0; i < 9; i++)
		{
			if (pin_index & (1 << i))
			{
				pin_number = (enum coines_multi_io_pin )i;
			}
		}
	}

	if (pin_number == COINES_SHUTTLE_PIN_MAX)
	{
		return;
	}

	latest_ticks_count = tc_read_cv(TC0, TC0_CHANNEL_CAPTURE);
    /*lint -e{647}*/
	total_ticks = (uint64_t)(latest_ticks_count + (65536 * timer_overflow_count));

	/* Callback with timestamp */
	timed_interrupt_config[pin_number].cb(TIMER_TICKS_TO_NSEC(total_ticks), (uint32_t)pin_number, (uint32_t)id);//TODO: check polarity
}

/*!
 * @brief Attaches a timed interrupt to a pin
 */
int16_t coines_attach_timed_interrupt(enum coines_multi_io_pin pin_number,
                                      timed_interrupt_cb interrupt_cb,
									  enum coines_pin_interrupt_mode int_mode)
{
	uint32_t mcu_int_mode = 0;

	pio_set_input(PIOC, get_hw_pin(pin_number) - PIO_PC0_IDX, PIO_INPUT);

	if (int_mode == COINES_PIN_INTERRUPT_CHANGE)
	{
		mcu_int_mode = (PIO_IT_FALL_EDGE) & ~(PIO_IT_AIME);
	}

	if (int_mode == COINES_PIN_INTERRUPT_RISING_EDGE)
	{
		mcu_int_mode = PIO_IT_RISE_EDGE;
	}

	if (int_mode == COINES_PIN_INTERRUPT_FALLING_EDGE)
	{
		mcu_int_mode = PIO_IT_FALL_EDGE;
	}

	(void)pio_handler_set(PIOC, ID_PIOC, 1 << (get_hw_pin(pin_number) - PIO_PC0_IDX), mcu_int_mode, attach_timed_interrupt_handler);

	timed_interrupt_config[pin_number].cb = interrupt_cb;

	NVIC_DisableIRQ(PIOC_IRQn);
	NVIC_ClearPendingIRQ(PIOC_IRQn);
	NVIC_EnableIRQ(PIOC_IRQn);

	pio_enable_pin_interrupt(get_hw_pin(pin_number));

	return COINES_SUCCESS;
}

/*!
 * @brief Detaches a timed interrupt from a Multi-IO pin
 */
int16_t coines_detach_timed_interrupt(enum coines_multi_io_pin pin_number)
{
	pio_disable_pin_interrupt(get_hw_pin(pin_number));

	memset(&timed_interrupt_config[pin_number], 0, sizeof(struct coines_timed_interrupt_config));

	return COINES_SUCCESS;
}

/*!
 * @brief This API is used to config the hardware timer in firmware
 */
int16_t coines_timer_config(enum coines_timer_instance instance, void* handler)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	if (instance < COINES_TIMER_INSTANCE_MAX)
	{
		if (!is_timer_enabled[instance])
		{
			/** Configure TC0 channel clock  */
			sysclk_enable_peripheral_clock(timer_counter_channel[instance]);
            /*lint -e{64}*/
			timer_interrupt_handler[instance] = handler;

			(void)tc_find_mck_divisor(DESIRED_DEFAULT_FREQUENCY, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
			tc_init(TC0, timer_instance[instance], ul_tcclks | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR);
			tc_write_rc(TC0, timer_instance[instance], (ul_sysclk / ul_div) / DESIRED_DEFAULT_FREQUENCY);

			NVIC_DisableIRQ((IRQn_Type) timer_counter_channel[instance]);
			NVIC_ClearPendingIRQ((IRQn_Type) timer_counter_channel[instance]);
			NVIC_SetPriority((IRQn_Type) timer_counter_channel[instance], TC_INTERRUPT_PRIORITY);
			NVIC_EnableIRQ((IRQn_Type) timer_counter_channel[instance]);

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
 * @brief This API is used to deconfig the hardware timer in firmware
 */
int16_t coines_timer_deconfig(enum coines_timer_instance instance)
{
	if (instance < COINES_TIMER_INSTANCE_MAX)
	{
		if (is_timer_enabled[instance])
		{
			sysclk_disable_peripheral_clock(timer_counter_channel[instance]);

			NVIC_ClearPendingIRQ((IRQn_Type) timer_counter_channel[instance]);
			NVIC_DisableIRQ((IRQn_Type) timer_counter_channel[instance]);

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
int16_t coines_timer_start(enum coines_timer_instance instance, uint32_t timeout)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	double desired_freq;

	if (instance < COINES_TIMER_INSTANCE_MAX)
	{
		if (is_timer_enabled[instance])
		{
			desired_freq = (float)((float)1000000 /timeout); /* timeout in microseconds */ 
			(void)tc_find_mck_divisor((uint32_t)desired_freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
            tc_init(TC0, timer_instance[instance], ul_tcclks | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR);
			tc_write_rc(TC0, timer_instance[instance],(uint32_t)(((float)ul_sysclk /(float)ul_div)/desired_freq));

			tc_enable_interrupt(TC0, timer_instance[instance], TC_IER_CPCS);
			tc_start(TC0, timer_instance[instance]);
		}
	}
	else
	{
		return COINES_E_TIMER_INVALID_INSTANCE;
	}

	return COINES_SUCCESS;
}

/*!
 * @brief This API is used to stop the hardware timer in firmware
 */
int16_t coines_timer_stop(enum coines_timer_instance instance)
{
	if (instance < COINES_TIMER_INSTANCE_MAX)
	{
		if (is_timer_enabled[instance])
		{
			tc_stop(TC0, timer_instance[instance]);
			tc_disable_interrupt(TC0, timer_instance[instance], TC_IER_CPCS);
		}
	}
	else
	{
		return COINES_E_TIMER_INVALID_INSTANCE;
	}

	return COINES_SUCCESS;
}

/*!
 *  @brief This API is used to execute the function inside critical region.
 */
void coines_execute_critical_region(coines_critical_callback callback)
{
	irqflags_t flags;

	flags = cpu_irq_save();
	if (callback)
	{
		callback();
	}

	cpu_irq_restore(flags);
}

void coines_flush_intf(enum coines_comm_intf intf)
{
    (void)intf;
}

bool coines_intf_connected(enum coines_comm_intf intf)
{
    if (intf == COINES_COMM_INTF_USB)
    {
        return serial_connected;
    }

    return false;
}
