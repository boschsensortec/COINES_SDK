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
 * @file    w25m02gw.h
 * @date    Aug 3, 2020
 * @brief   W25M02GW NAND Flash driver
 */

/*!
 * @addtogroup interpreter
 * @brief
 * @{*/

#ifndef W25M02GW_H_
#define W25M02GW_H_

#include "w25n01gw.h"
#include "w25_nand_error_codes.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*! No of W25N01GWTBIG dies present in W25M02GWTBIG */
#define W25M02GW_NO_OF_DIES             2

/*! No of Sectors per page */
#define W25M02GW_NO_OF_SEC_PER_PAGE     W25N01GW_NO_OF_SEC

/*! No of pages per block */
#define W25M02GW_NO_OF_PAGES_PER_BLOCK  W25N01GW_NO_OF_PAGES

/*! No of blocks in one die */
#define W25M02GW_NO_OF_BLOCKS_PER_DIE   W25N01GW_AVAILABLE_BLOCKS /*No.of Blocks in variant W25M02GWTBIG */

/*! Total No of pages in one die*/
#define W25M02GW_TOTAL_PAGES_PER_DIE    (W25M02GW_NO_OF_BLOCKS_PER_DIE * W25M02GW_NO_OF_PAGES_PER_BLOCK)

/*! Size of a Flash in per die */
#define W25M02GW_FLASH_SIZE_PER_DIE     W25N01GW_FLASH_SIZE

/*! Size of a sectors in bytes */
#define W25M02GW_SECTOR_SIZE            W25N01GW_SECTOR_SIZE

/*! Size of a page in bytes */
#define W25M02GW_PAGE_SIZE              W25N01GW_PAGE_SIZE

/*! Size of a block in bytes */
#define W25M02GW_BLOCK_SIZE             W25N01GW_BLOCK_SIZE

/*! Size of the flash in bytes */
#define W25M02GW_FLASH_SIZE             (W25N01GW_FLASH_SIZE * W25M02GW_NO_OF_DIES)


/**
 * @brief structure to hold the flash device information
 */
typedef struct w25m02gw_deviceinfo_type
{
    uint8_t mfg_id;     /**< Manufacturer Id*/
    uint16_t device_id; /**< Device Id*/
} w25m02gw_device_info_t;

/**
 * @brief Structure to return the characteristics of the storage device
 */
typedef struct w25m02gw_memoryparams_type
{
    uint32_t memory_size;       /**< Total memory size of the storage device in bytes*/
    uint16_t sector_size;       /**< Sector size*/
    uint32_t no_of_sectors;     /**< Total number of Sectors in storage device*/
    uint16_t erase_block_units; /**< Erase block size in unit of pages*/
} w25m02gw_memory_params_t;

/*!
 * @brief This function initializes the w25n01gw driver
 *
 *
 * @param[in]   void
 *
 * @retval W25N01GW_INITIALIZED : If initialization is success
 * @retval W25N01GW_INITIALIZATION_FAILED : If initialization has failed
 * @retval W25N01GW_INITIALIZED : If initialization is success
 *
 */
w25_nand_error_t w25m02gw_init(void);

/*!
 * @brief This function gets the initialization status of the module
 *
 *
 * @param[in]   void
 *
 * @retval W25N01GW_UNINITIALIZED : If the module is  uninitialized
 * @retval W25N01GW_INITIALIZED : If the module is initialized
 *
 */
w25_nand_error_t w25m02gw_get_device_init_status(void);

/*!
 * @brief This function erases the blocks based on the input position and length
 *
 *
 * @param[in] pos : Position of the flash from where the erase should happen(however this api erases the entire block where this location of
 *                     memory is located. eg: Say pos is 25. Then block number is 1.the entire 1st block will be erased irrespctive of position and len
 * @param[in] len : No of bytes to be erased
 *
 * @retval W25N01GW_ERASE_SUCCESS : Erase success
 * @retval W25N01GW_ERR_LOCATION_INVALID : Invalid location
 * @retval W25N01GW_ERASE_FAILURE : Erase failure
 *
 */
w25_nand_error_t w25m02gw_erase_block(uint32_t pos, uint32_t len);

/*!
 * @brief This function erases the entire flash memory
 *
 * @retval W25N01GW_ERASE_SUCCESS : Erase success
 * @retval W25N01GW_ERASE_FAILURE : Erase failure
 *
 */
w25_nand_error_t w25m02gw_mass_erase(void);

/*!
 * @brief This function reads from the flash
 *
 *
 * @param[in,out] data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]   num_of_bytes_to_read : No of bytes to read
 * @param[in]   read_loc : Location from which the data is to be read
 *
 * @retval  W25N01GW_READ_SUCCESS : Read is success
 * @retval  W25N01GW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval  W25N01GW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval  W25N01GW_ERR_LOCATION_INVALID : Invalid location
 * @retval  W25N01GW_ECC_FAILURE : Read is failed
 *
 *
 */
w25_nand_error_t w25m02gw_read(uint8_t* data_ptr, uint32_t num_of_bytes_to_read, uint32_t read_loc);

/*!
 * @brief This function reads the spare bytes from the page
 *
 *
 * @param[in,out]  data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]  no_of_bytes_to_read : No of bytes to read
 * @param[in]  page_num : Page number
 * @param[in]  page_off : Location from which the data is to be read
 *
 * @retval W25N01GW_READ_SUCCESS : Read is success
 * @retval W25N01GW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval W25N01GW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval W25N01GW_ECC_FAILURE : Read is failed
 *
 *
 */
w25_nand_error_t w25m02gw_read_spare(uint8_t* dataPtr, int8_t num_of_bytes_to_read, uint32_t page_num, uint16_t page_off);
/*!
 * @brief This function writes the spare bytes to the page
 *
 *
 * @param[in]   data_ptr : Pointer to the buffer which holds the data that is to be written
 * @param[in]   no_of_bytes_to_write : No of bytes to write
 * @param[in]   page_num : Page number
 * @param[in]   page_off : Location to which the data is to be written
 *
 * @retval W25N01GW_WRITE_SUCCESS : Write is success
 * @retval W25N01GW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval W25N01GW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval W25N01GW_WRITE_FAILURE : Write is failed
 *
 *
 */
w25_nand_error_t w25m02gw_write_spare(const uint8_t* data_ptr, uint8_t no_of_bytes_to_write, uint32_t page_num, uint16_t page_off);

/*!
 * @brief This function writes the bytes to the page
 *
 *
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 *
 * @retval W25N01GW_WRITE_SUCCESS : Write is success
 * @retval W25N01GW_ERR_BUFFER_INVALID   : Buffer Pointer is NULL
 * @retval W25N01GW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval W25N01GW_ERR_LOCATION_INVALID : Location is Invalid
 * @retval W25N01GW_WRITE_FAILURE : Write is failed
 *
 *
 */
w25_nand_error_t w25m02gw_write(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);

/*!
 * @brief       This function gives the memory parameters of the flash
 *
 * @param[in]   w25m02gw_memory_params_t* : Structure pointer to which the memory parameter details are filled
 *
 * @return      void
 *
 */
void w25m02gw_get_memory_params(w25m02gw_memory_params_t* flashMemoryParams);

/*!
 * @brief       This function gives the manufacture id and device id
 *
 * @param[in,out]   w25m02gw_device_info_t* - Structure pointer to which the manufacture id and device id is filled
 *
 * @return      void
 *
 */
void w25m02gw_get_manufacture_and_devid(w25m02gw_device_info_t* info);
/*!
 * @brief       This function is used load the sector data into a buffer
 *
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 *
 * @return      w25_nand_error_t
 *
 */


w25_nand_error_t w25m02gw_load_sector(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);
/*!
 * @brief       This function is used load the sector spare data into a buffer
 *
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 *
 * @return      w25_nand_error_t
 *
 */
w25_nand_error_t w25m02gw_load_sector_spare(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write,uint32_t page_num);
/*!
 * @brief       This function is used to perform the actual write of the buffer that is filled by w25m02gw_load_sector and w25m02gw_load_sector functions
 *
 * @param[in]  page_num : Page num to which the data is to be written
 * @param[in]  sector_num : Sector number to which the data is to be written
 *
 * @return      w25_nand_error_t
 *
 */
w25_nand_error_t w25m02gw_write_sector_with_spare(uint32_t page_num, uint8_t sector_num);
#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_W25N01GW_H_ */

/** @}*/
