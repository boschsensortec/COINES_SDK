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
#include "nrf_gpio.h"
#include "w25_common.h"
/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/

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
   w25_gpio_pin_output_set(pin);
}

/*!
 * @brief       This function initialises the spi module
 *
 * @retval      spi handle
 */

uint8_t w25n02kw_spi_init()
{
    return w25_spi_init();
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
    return w25_spi_rx_tx(spi_entity, address, tx_buffer, tx_count, rx_buffer, rx_count);
}

#ifdef __cplusplus
}
#endif

#endif /* W25N02KW_CONFIG_H_ */

/** @}*/

