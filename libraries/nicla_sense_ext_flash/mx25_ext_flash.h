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
 * @file    mx25_ext_flash.h
 * @date    Nov 30, 2022
 * @brief   MX25R1635F NOR Flash driver
 */

#ifndef FLASH_H
#define FLASH_H

#include "lfs.h"

/*! Write Status Register*/
#define SPIM_MX25_CMD_WR_SR 	0x01
/*! Reset Enable */
#define SPIM_MX25_CMD_RST_EN 	0x66
/*! Reset Memory */
#define SPIM_MX25_CMD_RST 		0x99
/*! Read Electronic Manufacturer & Device ID */
#define SPIM_MX25_CMD_REMS 		0x90
/*! Write Enable */
#define SPIM_MX25_CMD_WR_EN		0x06
/*! Read */
#define SPIM_MX25_CMD_READ		0x03
/*! Page Program */
#define SPIM_MX25_CMD_PP		0x02
/*! Read Status Register */
#define SPIM_MX25_CMD_RD_SR 	0x05
/*! To Erase Selected Sector */
#define SPIM_MX25_CMD_SE 		0x20
/*! To Erase Block 32KBr */
#define SPIM_MX25_CMD_BE 		0x52
/*! To Read Security Register */
#define SPIM_MX25_CMD_RD_SCUR 	0x2b
/*! First byte of send buffer will receive nothing.Hence reading from 2nd byte */
#define MX25_MIN_RCV_BYTES_LEN  0x02
/*! Size of a sectors in bytes */
#define MX25_SECTOR_SIZE        4095
/*! No of available sectors */
#define MX25_AVAILABLE_SECTORS  512
/*! Size of the flash in bytes */
#define MX25_FLASH_SIZE         0x1FFFFF
/*! Size of a block in bytes */
#define MX25_BLOCK_SIZE         32760

/*! Position of BUSY bit of status register */
#define MX25_BUSY_STAT                 (1 << 0)
/*! Position of E_FAIL bit of security register */
#define MX25_E_FAIL_STAT               (1 << 6)
/*! Position of P_FAIL bit of security register */
#define MX25_P_FAIL_STAT  			   (1 << 5)


/**
 * @brief Enum which holds the error codes
 */
typedef enum mx25_errorcode_enum_type
{
    MX25_ERROR,
    MX25_ERR_LOCATION_INVALID,
    MX25_ERR_BUFFER_INVALID,
    MX25_ERR_BYTE_LEN_INVALID,
    MX25_UNINITIALIZED,
    MX25_INITIALIZED,
    MX25_INITIALIZATION_FAILED,
    MX25_ERASE_SUCCESS,
    MX25_ERASE_FAILURE,
    MX25_READ_SUCCESS,
    MX25_READ_FAILURE,
    MX25_ECC_FAILURE,
    MX25_WRITE_SUCCESS,
    MX25_WRITE_FAILURE,
    MX25_BUSY,
	MX25_SPIM_INIT_FAILURE
} mx25_error_t;
/**
 * @brief structure to hold the flash device information
 */
typedef struct mx25_deviceinfo
{
    uint8_t mfg_id;     /**< Manufacturer Id*/
    uint16_t device_id; /**< Device Id*/

} mx25_deviceinfo_t;

uint8_t mx25_read_reg(uint8_t reg);
uint32_t mx25_read_electronic_id(mx25_deviceinfo_t* info);
uint32_t mx25_spim_rx_tx(uint8_t address, uint8_t* tx_buffer, uint16_t tx_count, uint8_t* rx_buffer, uint16_t rx_count);
void mx25_device_reset(void);
uint32_t flash_init(void);
uint32_t mx25_erase_sector(uint32_t add);
uint32_t mx25_read(uint8_t* data_ptr, uint32_t no_of_bytes_to_read, uint32_t read_loc);
uint32_t mx25_write(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);
uint32_t m25_erase_block(uint32_t erase_loc);
uint32_t mx25_mass_erase(void);
uint8_t mx25_spim_init(void);
int flash_sync(const struct lfs_config *c);
int flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int flash_erase(const struct lfs_config *c, lfs_block_t block);

#endif
