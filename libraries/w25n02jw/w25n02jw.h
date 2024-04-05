/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    w25n02jw.h
 * @date    Apr 30, 2021
 * @brief   W25N02JW NAND Flash driver
 */

/*!
 * @addtogroup interpreter
 * @brief
 * @{*/

#ifndef W25N02JW_H_
#define W25N02JW_H_

#include <stdint.h>
#include <stddef.h>
#include "w25_nand_error_codes.h"
#include "w25_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*! No of Sectors per page */
#define W25N02JW_NO_OF_SEC          4

/*! No of pages per block */
#define W25N02JW_NO_OF_PAGES        64

/*! No of blocks in variant W25N02JW */
#define W25N02JW_NO_OF_BLOCKS       2048    /*No.of Blocks in variant W25N02JW */

    /*! No of blocks for replacement */
#define W25N02JW_BBM_NO_OF_REPLACEMENT_BLOCKS 40

    /*! No of available blocks */
#define W25N02JW_AVAILABLE_BLOCKS  2008

/*! Size of a sectors in bytes */
#define W25N02JW_SECTOR_SIZE        512

/*! Size of a page in bytes */
#define W25N02JW_PAGE_SIZE          (W25N02JW_NO_OF_SEC * W25N02JW_SECTOR_SIZE)

/*! Size of spare portion in bytes */
#define W25N02JW_SPARE_BYTES_SIZE  64

/*! Size of a block in bytes */
#define W25N02JW_BLOCK_SIZE         (W25N02JW_NO_OF_PAGES * W25N02JW_PAGE_SIZE)

/*! Size of the flash in bytes */
#define W25N02JW_FLASH_SIZE         (W25N02JW_BLOCK_SIZE * W25N02JW_AVAILABLE_BLOCKS)

/*! Total number of blocks in the flash */
#define W25N02JW_TOTAL_SECTORS      (W25N02JW_NO_OF_SEC * W25N02JW_NO_OF_PAGES * W25N02JW_AVAILABLE_BLOCKS)

/*! No of Sectors per block */
#define W25N02JW_SECTORS_PER_BLOCK  (W25N02JW_NO_OF_PAGES * W25N02JW_NO_OF_SEC)

/*! Start position of Spare bytes in a page  */
#define W25N02JW_SPARE_BYTES_START_POS  2048

typedef enum w25n02jw_reg_enum_type {
    W25N02JW_PROTECT_REG_ADDR   = 0xA0, /**< Protection Register*/
    W25N02JW_CONFIG_REG_ADDR    = 0xB0, /**< Configuration Register*/
    W25N02JW_STATUS_REG_ADDR    = 0xC0  /**< Status Register*/
} w25n02jw_reg_t;

/**
 * @brief structure to hold the flash device information
 */
typedef struct w25n02jw_deviceinfo
{
    uint8_t mfg_id;     /**< Manufacturer Id*/
    uint16_t device_id; /**< Device Id*/

} w25n02jw_deviceinfo_t;

/*!
 * @brief       This function finds the good block and replaces the bad block with the known good block
 *
 * @param[in]   lba - This is the logical block address , the address of the bad block
 *
 * @return      None
 */
w25_nand_error_t w25n02jw_bbm_block_swap(uint16_t lba);

/*!
 * @brief This function opens the page that is to be read
 *
 * @param[in]   page_num - Page number
 *
 * @retval W25N02JW_WRITE_SUCCESS : Write is success
 * @retval W25N02JW_ERR_BUFFER_INVALID  :  Buffer Pointer is NULL
 * @retval W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval W25N02JW_WRITE_FAILURE : Write is failed
 *
 */
w25_nand_error_t w25n02jw_page_read(uint32_t page_num);

/*!
 * @brief       This function reads bbm table
 * @param[in]   void
 *
 * @retval      None
 */
void w25n02jw_read_bbm_table(uint8_t* data_ptr);

/*!
 * @brief       This function resets the chip
 * @param[in]   void
 *
 * @retval      None
 */
void w25n02jw_device_reset(void);

/*!
 * @brief       This function writes to the register
 *
 * @param[in]   w25n02jw_reg_enum_t : Register to which the reg_value to be written
 * @param[in]   reg_value : value to be written
 *
 * @retval      None
 */
void w25n02jw_write_reg(w25_reg_t reg, uint8_t reg_value);

/*!
 * @brief       This function initial bad block management
 *
 * @param[in]   void
 *
 * @retval      None
 */
void w25n02jw_initial_bbm(void);

/*!
 * @brief This function initializes the w25n02jw driver
 *
 * @param[in]   void
 *
 * @retval  W25N02JW_INITIALIZED : If initialization is success
 * @retval  W25N02JW_INITIALIZATION_FAILED : If initialization has failed
 * @retval  W25N02JW_ERROR : If SPI initialization fails
 *
 */
w25_nand_error_t w25n02jw_init(void);

/*!
 * @brief This function gets the initialization status of the module
 *
 * @param[in]   void
 *
 * @retval  W25N02JW_INITIALIZED : If the module is initialized
 * @retval  W25N02JW_UNINITIALIZED : If the module is  uninitialized
 *
 */
w25_nand_error_t w25n02jw_get_device_init_status(void);

/*!
 * @brief This function erases the blocks based on the input position and length
 *
 * @param[in]   pos : position of the flash from where the erase should happen(however this api erases the entire block where this location of
 *                     memory is located. eg: Say pos is 25. Then block number is 1.the entire 1st block will be erased irrespctive of position and len
 * @param[in]   len : No of bytes to be erased
 *
 * @retval  W25N02JW_ERASE_SUCCESS : Erase success
 * @retval  W25N02JW_ERR_LOCATION_INVALID : Invalid location
 * @retval  W25N02JW_ERASE_FAILURE : Erase failure
 *
 */
w25_nand_error_t w25n02jw_erase_block(uint32_t pos, uint32_t len);

/*!
 * @brief This function erases the entire flash memory
 *
 * @param[in] void
 *
 * @retval W25N02JW_ERASE_SUCCESS : Erase success
 * @retval W25N02JW_ERASE_FAILURE : Erase failure
 *
 */
w25_nand_error_t w25n02jw_mass_erase(void);

/*!
 * @brief This function reads from the flash
 *
 * @param[in]   data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]   no_of_bytes_to_read : No of bytes to read
 * @param[in]   read_loc : Location from which the data is to be read
 *
 * @retval  W25N02JW_READ_SUCCESS : Read is success
 * @retval  W25N02JW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval  W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval  W25N02JW_ERR_LOCATION_INVALID : Invalid location
 * @retval  W25N02JW_ECC_FAILURE : Read is failed
 *
 */
w25_nand_error_t w25n02jw_read(uint8_t* data_ptr, uint32_t no_of_bytes_to_read, uint32_t read_loc);

/*!
 * @brief This function reads the spare bytes from the page
 *
 * @param[in,out]  data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]   no_of_bytes_to_read : No of bytes to read
 * @param[in]   page_num : Page number
 * @param[in]   page_off : Location from which the data is to be read
 *
 * @retval   W25N02JW_READ_SUCCESS : Read is success
 * @retval   W25N02JW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval   W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval   W25N02JW_ECC_FAILURE : Read is failed
 *
 */
w25_nand_error_t w25n02jw_read_spare(uint8_t* data_ptr, int8_t no_of_bytes_to_read, uint32_t page_num, uint16_t page_off);

/*!
 * @brief This function writes the spare bytes to the page
 *
 * @param[in]   data_ptr : Pointer to the buffer which holds the data to be written
 * @param[in]   no_of_bytes_to_write : No of bytes to write
 * @param[in]   page_num : Page number
 * @param[in]   page_off : Location to which the data is to be written
 *
 * @retval      W25N02JW_WRITE_SUCCESS        : Write is success
 * @retval      W25N02JW_ERR_BUFFER_INVALID   : Buffer Pointer is NULL
 * @retval      W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval      W25N02JW_WRITE_FAILURE        : Write is failed
 *
 */
w25_nand_error_t w25n02jw_write_spare(const uint8_t* data_ptr, uint8_t no_of_bytes_to_write, uint16_t page_num, uint16_t page_off);

/*!
 * @brief This function writes the bytes to the page
 *
 * @param[in]   data_ptr : Pointer to the buffer of which the data is to be written.
 * @param[in]   no_of_bytes_to_write : No of bytes to write
 * @param[in]   write_loc : Location to which the data is to be written
 *
 * @retval  W25N02JW_WRITE_SUCCESS        : Write is success
 * @retval  W25N02JW_ERR_BUFFER_INVALID   : Buffer Pointer is NULL
 * @retval  W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval  W25N02JW_ERR_LOCATION_INVALID : Location is Invalid
 * @retval  W25N02JW_WRITE_FAILURE        : Write is failed
 *
 */
w25_nand_error_t w25n02jw_write(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);

/*!
 * @brief       This function gives the memory parameters of the flash
 *
 * @param[in,out]  *flash_memory_params: Structure pointer to which the memory parameter details are filled
 *
 * @retval      None
 */
void w25n02jw_get_memory_params(w25_memory_params_t* flash_memory_params);

/*!
 * @brief       This function gives the manufacture id and device id
 *
 * @param[in,out]  *info : Structure pointer to which the manufacture id and device id is filled
 *
 * @retval      None
 */
void w25n02jw_get_manufacture_and_devid(w25_deviceinfo_t* info);

/*!
 * @brief       This function selects the die
 *
 * @param[in]   die_number - Die number which is to be selected
 *
 * @retval      void
 *
 */

void w25n02jw_die_select(uint8_t die_number);

/*!
 * @brief       This function sets the initial configuration of the die
 *
 * @retval      void
 *
 */
void w25n02jw_init_protect_reg(void);

/*!
 * @brief       This function selects the die
 *
 * @retval      void
 *
 */
void w25n02jw_init_config_reg(void);
/*!
 * @brief       This function is used load the sector data into a buffer
 *
 * @param[in]  data_ptr : Pointer to the buffer which holds the data that is to be written.
 * @param[in]  no_of_bytes_to_write : No of bytes to write
 * @param[in]  write_loc : Location to which the data is to be written
 *
 * @return      w25_nand_error_t
 */
w25_nand_error_t w25n02jw_load_sector(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc);
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
w25_nand_error_t w25n02jw_load_sector_spare(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write,uint32_t page_num);
/*!
 * @brief       This function is used to perform the actual write of the buffer that is filled by w25n02jw_load_sector and w25n02jw_load_sector_spare functions
 *
 * @param[in]  page_num : Page num to which the data is to be written
 * @param[in]  sector_num : Sector number to which the data is to be written
 *
 * @return      w25_nand_error_t
 *
 */
w25_nand_error_t w25n02jw_write_sector_with_spare(uint32_t page_num, uint8_t sector_num);
/*!
 * @brief       This function checks the factory shipped chip and updates the LUT. TO BE TESTED
 *
 * @return      void
 *
 */

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_W25N02JW_H_ */

/** @}*/
