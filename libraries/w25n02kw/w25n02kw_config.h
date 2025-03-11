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
 * @file        w25n02kw_config.h
 *
 * @brief
 *
 */

/*!
* @addtogroup w25n02kw_config
* @brief
* @{*/

#ifndef W25N02KW_CONFIG_H_
#define W25N02KW_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include "coines.h"
#include "nrfx_spim.h"
#include "nrf_gpio.h"
/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/*! Write protect pin of w25n02kw chip */
#define W25N02KW_WRITE_PROTECT_PIN   NRF_GPIO_PIN_MAP(0, 22)
/*! Hold pin of w25n02kw chip */
#define W25N02KW_HOLD_PIN           NRF_GPIO_PIN_MAP(0, 23)
/*! Chip Select pin of w25n02kw chip */
#define W25N02KW_CHIP_SEL_PIN       NRF_GPIO_PIN_MAP(0, 17)

#define SPI_MOSI_PIN_FLASH                 NRF_GPIO_PIN_MAP(0, 20)
#define SPI_MISO_PIN_FLASH                 NRF_GPIO_PIN_MAP(0, 21)
#define SPI_CLK_PIN_FLASH                  NRF_GPIO_PIN_MAP(0, 19)
#define SPI_CS_PIN_FLASH                   NRF_GPIO_PIN_MAP(0, 17)
#define SPI_TX_SIZE  (256 + 1)

extern const nrfx_spim_t flash_spi_instance;
extern nrfx_spim_config_t flash_spi_config;
static nrfx_spim_xfer_desc_t flash_spi_xfer;
static uint8_t flash_spi_tx_buff[SPI_TX_SIZE];
/**********************************************************************************/
/* type definitions */
/**********************************************************************************/

/*!
 * @brief       This function sets the gpio pin given as input
 *
 * @retval      none
 */

void w25n02kw_gpio_pin_output_set(uint8_t pin)
{
  nrf_gpio_cfg_output(pin);
  nrf_gpio_pin_set(pin);
}

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */

uint8_t w25n02kw_spi_init()
{
    flash_spi_config.miso_pin = SPI_MISO_PIN_FLASH;
    flash_spi_config.mosi_pin = SPI_MOSI_PIN_FLASH;
    flash_spi_config.sck_pin = SPI_CLK_PIN_FLASH;
    flash_spi_config.ss_pin = SPI_CS_PIN_FLASH;
    flash_spi_config.frequency = NRF_SPIM_FREQ_8M;

    if (nrfx_spim_init(&flash_spi_instance, &flash_spi_config, NULL, NULL) != NRFX_SUCCESS)
    {
        return 0;
    }
    return 1;
}

/*!
 * @brief       This function sends and receives the data via SPI
 *
 * @param[in]   spi_entity : SPI handle
 * @param[in]   address : reg address
 * @param[in]   tx_buffer : transmission buffer
 * @param[in]   tx_count : transmission buffer size
 * @param[in]   rx_buffer : receiver buffer
 * @param[in]   rx_count : receiver buffer size
 *
 */

uint8_t w25n02kw_spi_rx_tx(uint8_t spi_entity, uint8_t address, uint8_t* tx_buffer, uint16_t tx_count, uint8_t* rx_buffer, uint16_t rx_count)
{
	(void)spi_entity;
    flash_spi_tx_buff[0] = address;
    memcpy(&flash_spi_tx_buff[1], tx_buffer, tx_count);

    flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
    flash_spi_xfer.tx_length = tx_count + 1;
    flash_spi_xfer.p_rx_buffer = rx_buffer;
    flash_spi_xfer.rx_length = rx_count;

    if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
    {
        return 1;
    }
   
    coines_yield();
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* W25N02KW_CONFIG_H_ */

/** @}*/

