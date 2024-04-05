/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    w25_common.h
 * @date    Apr 30, 2021
 * @brief   W25N02JW NAND Flash driver
 */

/*!
 * @addtogroup interpreter
 * @brief
 * @{*/

#ifndef W25_COMMON_H_
#define W25_COMMON_H_

#include <stdint.h>
#include <stddef.h>
#include "w25_nand_error_codes.h"
#ifdef __cplusplus
extern "C"
{
#endif

/*! Device Id of w25n01gwtbig */
#define W25N01GW_DEVICE_ID        0xBA21

/*! Device Id of w25m02gwtbig */
#define W25M02GW_DEVICE_ID        0xBB21

/*! Device Id of w25n02jwtbig */
#define W25N02JW_DEVICE_ID        0xBF22

/*! Device Id of w25n02kwzeir */
#define W25N02KW_DEVICE_ID		  0xBA22 

/*! Manufacturer id of w25n01gwtbig */
#define W25_MANUFACTURER_ID           0xEF

/*! No of Sectors per page */
#define W25_NO_OF_SEC          4

/*! No of pages per block */
#define W25_NO_OF_PAGES        64

/*! Size of a sectors in bytes */
#define W25_SECTOR_SIZE        512

/*! Size of a page in bytes */
#define W25_PAGE_SIZE          (W25_NO_OF_SEC * W25_SECTOR_SIZE)

/*! Size of a block in bytes */
#define W25_BLOCK_SIZE         (W25_NO_OF_PAGES * W25_PAGE_SIZE)

/**
 * @brief Enum which holds the register address in Flash
 */
typedef enum w25_reg_enum_type {
    W25_PROTECT_REG_ADDR   = 0xA0, /**< Protection Register*/
    W25_CONFIG_REG_ADDR    = 0xB0, /**< Configuration Register*/
    W25_STATUS_REG_ADDR    = 0xC0  /**< Status Register*/
} w25_reg_t;
/**
 * @brief Structure to return the characteristics of the storage device
 */
typedef struct w25_memoryparams
{
    uint32_t memory_size;       /**< Total memory size of the storage device in bytes*/
    uint16_t sector_size;       /**< Sector size*/
    uint32_t no_of_Sectors;     /**< Total number of Sectors in storage device*/
    uint16_t erase_block_units; /**< Erase block size in unit of pages*/
} w25_memory_params_t;

/**
 * @brief structure to hold the flash device information
 */
typedef struct w25_deviceinfo
{
    uint8_t mfg_id;     /**< Manufacturer Id*/
    uint16_t device_id; /**< Device Id*/

} w25_deviceinfo_t;
/**
 * @brief function to get device information
 */
void w25_get_manufacture_and_devid(w25_deviceinfo_t* info);

/**
 * @brief function for flash chip intitalization common across chips
 */
w25_nand_error_t w25_init(uint16_t *device_id);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_W25_COMMON_H */

/** @}*/
