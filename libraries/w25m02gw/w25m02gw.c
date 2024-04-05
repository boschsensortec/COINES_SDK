/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    w25m02gw.c
 * @date    Aug 3, 2020
 * @brief   W25M02GW NAND Flash driver
 */

/*!
 * @addtogroup
 * @brief
 * @{*/

/**********************************************************************************/
/* system header includes */
/**********************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/**********************************************************************************/
/* own header files */
/**********************************************************************************/

#include "w25m02gw.h"
#include "w25_common.h"
/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

#define W25M02GW_DIE_ZERO         0x00
#define W25M02GW_DIE_ONE          0x01

/*! Holds the initialization status of the module */
w25_nand_error_t w25m02gw_init_status = W25_NAND_UNINITIALIZED;

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

w25_nand_error_t w25m02gw_init()
{
    w25_deviceinfo_t info;
    w25_nand_error_t ret_code = W25_NAND_INITIALIZATION_FAILED;

    if (w25m02gw_init_status != W25_NAND_INITIALIZED)
    {
        ret_code = w25n01gw_init();
        if (W25_NAND_INITIALIZED != ret_code)
        {
            return W25_NAND_INITIALIZATION_FAILED;
        }
        w25n01gw_get_manufacture_and_devid(&info);

        if ((info.device_id == W25M02GW_DEVICE_ID))
        {
            /* 0th die will be initialized in the driver itself.
             * initialize the 1st die*/
            w25n01gw_die_select(W25M02GW_DIE_ONE);

            w25n01gw_init_protect_reg();
            w25n01gw_init_config_reg();

            w25m02gw_init_status = W25_NAND_INITIALIZED;
            ret_code = W25_NAND_INITIALIZED;

        }
        else
        {
            ret_code = W25_NAND_INITIALIZATION_FAILED;
        }
    }
    else
    {
        ret_code = W25_NAND_INITIALIZED;
    }

    return ret_code;
}

w25_nand_error_t w25m02gw_erase_block(uint32_t pos, uint32_t len)
{
    w25_nand_error_t res = W25_NAND_ERROR;
    uint32_t page_num;
    uint16_t block_num_to_start_erase;
    uint16_t num_of_blocks_to_erase;

    /*Check if the requested position and length of the data to be erased lies within the Flash size*/
    if ((pos - 1) % W25M02GW_BLOCK_SIZE != 0 || len % W25M02GW_BLOCK_SIZE != 0)
    {
        res = W25_NAND_ERR_LOCATION_INVALID;

        return res;
    }

    /*Get the Page number from which the data is to be erased.*/
    page_num = (pos - 1) / W25M02GW_PAGE_SIZE;

    /*Block number from which the erase should start*/
    block_num_to_start_erase = (uint16_t)(page_num / W25M02GW_NO_OF_PAGES_PER_BLOCK);

    /* Number of blocks to erase*/
    num_of_blocks_to_erase = (uint16_t)(len / W25M02GW_BLOCK_SIZE);

    while (num_of_blocks_to_erase)
    {
        if (block_num_to_start_erase < W25M02GW_NO_OF_BLOCKS_PER_DIE)
        {
            /*Enable Die 0*/
            w25n01gw_die_select(W25M02GW_DIE_ZERO);
            if (block_num_to_start_erase + num_of_blocks_to_erase <= W25M02GW_NO_OF_BLOCKS_PER_DIE)
            {
                /*Erase the blocks*/
                (void)w25n01gw_erase_block(pos, len);
                num_of_blocks_to_erase = 0;
            }
            else
            {
                /*Erase the blocks*/
                (void)w25n01gw_erase_block(pos, (W25M02GW_NO_OF_BLOCKS_PER_DIE - block_num_to_start_erase) * W25M02GW_BLOCK_SIZE);
                num_of_blocks_to_erase = num_of_blocks_to_erase - (W25M02GW_NO_OF_BLOCKS_PER_DIE - block_num_to_start_erase);
                block_num_to_start_erase = W25M02GW_NO_OF_BLOCKS_PER_DIE;
                len = len - (W25M02GW_NO_OF_BLOCKS_PER_DIE - block_num_to_start_erase) * W25M02GW_BLOCK_SIZE;
                pos = pos + (W25M02GW_NO_OF_BLOCKS_PER_DIE - block_num_to_start_erase) * W25M02GW_BLOCK_SIZE;
            }
        }
        else
        {
            block_num_to_start_erase = block_num_to_start_erase - W25M02GW_NO_OF_BLOCKS_PER_DIE;
            pos = pos - W25M02GW_FLASH_SIZE_PER_DIE;

            /*Enable Die 1*/
            w25n01gw_die_select(W25M02GW_DIE_ONE);
            (void)w25n01gw_erase_block(pos, len);
            num_of_blocks_to_erase = 0;

        }
    }

    res = W25_NAND_ERASE_SUCCESS;

    return res;
}

w25_nand_error_t w25m02gw_mass_erase(void)
{
    /*Enable Die 0*/
    w25n01gw_die_select(W25M02GW_DIE_ZERO);
    (void)w25n01gw_mass_erase();

    /*Enable Die 1*/
    w25n01gw_die_select(W25M02GW_DIE_ONE);
    (void)w25n01gw_mass_erase();

    return W25_NAND_ERASE_SUCCESS;
}

w25_nand_error_t w25m02gw_read_spare(uint8_t* data_ptr, int8_t no_of_bytes_to_read, uint32_t page_num, uint16_t page_off)
{
    w25_nand_error_t ret_code = W25_NAND_READ_FAILURE;

    if (page_num < W25M02GW_TOTAL_PAGES_PER_DIE)
    {
        /*Enable Die 0*/
        w25n01gw_die_select(W25M02GW_DIE_ZERO);
        ret_code = w25n01gw_read_spare(data_ptr, no_of_bytes_to_read, page_num, page_off);

    }
    else
    {
        /* Update the page_num with respect to die 1*/
        page_num = page_num - W25M02GW_TOTAL_PAGES_PER_DIE;

        /*Enable Die 1*/
        w25n01gw_die_select(W25M02GW_DIE_ONE);
        ret_code = w25n01gw_read_spare(data_ptr, no_of_bytes_to_read, page_num, page_off);
    }

    return ret_code;

}

w25_nand_error_t w25m02gw_read(uint8_t* data_ptr, uint32_t num_of_bytes_to_read, uint32_t read_loc)
{
    w25_nand_error_t ret_code = W25_NAND_ERROR;
    if (data_ptr == NULL)
    {
        return W25_NAND_ERR_BUFFER_INVALID;
    }

    if (num_of_bytes_to_read == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }

    if (read_loc > W25M02GW_FLASH_SIZE)
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    while (num_of_bytes_to_read > 0)
    {
        /*Check if write location lies in Die 0*/
        if (read_loc < W25M02GW_FLASH_SIZE_PER_DIE)
        {
            /*Enable Die 0*/
            w25n01gw_die_select(W25M02GW_DIE_ZERO);
            if (read_loc + num_of_bytes_to_read <= W25M02GW_FLASH_SIZE_PER_DIE)
            {
                /*The entire buffer can be written in Die 0*/
                ret_code = w25n01gw_read(data_ptr, num_of_bytes_to_read, read_loc);
                if (ret_code != W25_NAND_READ_SUCCESS)
                {
                    return ret_code;
                }

                num_of_bytes_to_read = 0;
            }
            else
            {
                /* A part of the buffer needs to be read in Die 0 and remaining in Die1 */
                ret_code = w25n01gw_read(data_ptr, W25M02GW_FLASH_SIZE_PER_DIE - read_loc, read_loc);
                if (ret_code != W25_NAND_READ_SUCCESS)
                {
                    return ret_code;
                }

                /* Update the remaining number of bytes that is to be read */
                num_of_bytes_to_read = num_of_bytes_to_read - (W25M02GW_FLASH_SIZE_PER_DIE - read_loc);

                /* Update the new read location*/
                read_loc = W25M02GW_FLASH_SIZE_PER_DIE;

            }
        }
        else
        {
            /*Enable Die 1*/
            w25n01gw_die_select(W25M02GW_DIE_ONE);

            /* Update the read location with respect to Die 1*/
            read_loc = read_loc - W25M02GW_FLASH_SIZE_PER_DIE;
            ret_code = w25n01gw_read(data_ptr, num_of_bytes_to_read, read_loc);
            if (ret_code != W25_NAND_READ_SUCCESS)
            {
                return ret_code;
            }

            num_of_bytes_to_read = 0;

        }
    }

    return W25_NAND_READ_SUCCESS;
}

w25_nand_error_t w25m02gw_write_spare(const uint8_t* data_ptr,
                                      uint8_t num_of_bytes_to_write,
                                      uint32_t page_num,
                                      uint16_t page_off)
{
    w25_nand_error_t ret_code = W25_NAND_WRITE_FAILURE;

    if (page_num < W25M02GW_TOTAL_PAGES_PER_DIE)
    {
        /*Enable Die 0*/
        w25n01gw_die_select(W25M02GW_DIE_ZERO);
        if (w25n01gw_write_spare(data_ptr, num_of_bytes_to_write, (uint16_t)page_num, page_off) == W25_NAND_WRITE_SUCCESS)
        {
            ret_code = W25_NAND_WRITE_SUCCESS;
        }
    }
    else
    {
        /* Update the page_num with respect to die 1*/
        page_num = page_num - W25M02GW_TOTAL_PAGES_PER_DIE;

        /*Enable Die 1*/
        w25n01gw_die_select(W25M02GW_DIE_ONE);
        if (w25n01gw_write_spare(data_ptr, num_of_bytes_to_write, (uint16_t)page_num, page_off) == W25_NAND_WRITE_SUCCESS)
        {
            ret_code = W25_NAND_WRITE_SUCCESS;
        }
    }

    return ret_code;
}

w25_nand_error_t w25m02gw_write(const uint8_t* data_ptr, uint32_t num_of_bytes_to_write, uint32_t write_loc)
{
    w25_nand_error_t ret_code = W25_NAND_ERROR;

    if (data_ptr == NULL)
    {
        return W25_NAND_ERR_BUFFER_INVALID;
    }

    if (num_of_bytes_to_write == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }

    if (write_loc > W25M02GW_FLASH_SIZE)
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    while (num_of_bytes_to_write > 0)
    {
        /*Check if write location lies in Die 0*/
        if (write_loc < W25M02GW_FLASH_SIZE_PER_DIE)
        {
            /*Enable Die 0*/
            w25n01gw_die_select(W25M02GW_DIE_ZERO);
            if (write_loc + num_of_bytes_to_write <= W25M02GW_FLASH_SIZE_PER_DIE)
            {
                /*The entire buffer can be written in Die 0*/
                ret_code = w25n01gw_write(data_ptr, num_of_bytes_to_write, write_loc);
                if (ret_code != W25_NAND_WRITE_SUCCESS)
                {
                    return ret_code;
                }

                num_of_bytes_to_write = 0;
            }
            else
            {
                /* A part of the buffer needs to be written in Die 0 and remaining in Die1 */
                ret_code = w25n01gw_write(data_ptr, W25M02GW_FLASH_SIZE_PER_DIE - write_loc, write_loc);
                if (ret_code != W25_NAND_WRITE_SUCCESS)
                {
                    return ret_code;
                }

                num_of_bytes_to_write = num_of_bytes_to_write - (W25M02GW_FLASH_SIZE_PER_DIE - write_loc);
                write_loc = W25M02GW_FLASH_SIZE_PER_DIE;

            }
        }
        else
        {
            /*Enable Die 1*/
            w25n01gw_die_select(W25M02GW_DIE_ONE);
            write_loc = write_loc - W25M02GW_FLASH_SIZE_PER_DIE;
            ret_code = w25n01gw_write(data_ptr, num_of_bytes_to_write, write_loc);
            if (ret_code != W25_NAND_WRITE_SUCCESS)
            {
                return ret_code;
            }

            num_of_bytes_to_write = 0;

        }
    }

    return W25_NAND_WRITE_SUCCESS;
}

w25_nand_error_t w25m02gw_get_device_init_status()
{
    return w25m02gw_init_status;
}

w25_nand_error_t w25m02gw_load_sector(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc)
{
    w25_nand_error_t ret_code;
    /*Check if write location lies in Die 0*/
    if (write_loc < W25M02GW_FLASH_SIZE_PER_DIE)
    {
        /*Enable Die 0*/
        w25n01gw_die_select(W25M02GW_DIE_ZERO);
        ret_code = w25n01gw_load_sector(data_ptr, no_of_bytes_to_write, write_loc);

    }
    else
    {
        /*Enable Die 1*/
        w25n01gw_die_select(W25M02GW_DIE_ONE);
        write_loc = write_loc - W25M02GW_FLASH_SIZE_PER_DIE;
        ret_code = w25n01gw_load_sector(data_ptr, no_of_bytes_to_write, write_loc);

    }
    return ret_code;
}
w25_nand_error_t w25m02gw_load_sector_spare(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t page_num)
{
    w25_nand_error_t ret_code;
    /*Check if write location lies in Die 0*/
    if (page_num < W25M02GW_TOTAL_PAGES_PER_DIE)
    {
        /*Enable Die 0*/
        w25n01gw_die_select(W25M02GW_DIE_ZERO);
        ret_code = w25n01gw_load_sector_spare(data_ptr, no_of_bytes_to_write, page_num);

    }
    else
    {
        /*Enable Die 1*/
        w25n01gw_die_select(W25M02GW_DIE_ONE);
        page_num = page_num - W25M02GW_TOTAL_PAGES_PER_DIE;
        ret_code = w25n01gw_load_sector_spare(data_ptr, no_of_bytes_to_write, page_num);

    }

    return ret_code;
}
w25_nand_error_t w25m02gw_write_sector_with_spare(uint32_t page_num, uint8_t sector_num)
{
    w25_nand_error_t ret_code;
    /*Check if write location lies in Die 0*/
    if (page_num < W25M02GW_TOTAL_PAGES_PER_DIE)
    {
        /*Enable Die 0*/
        w25n01gw_die_select(W25M02GW_DIE_ZERO);
        ret_code = w25n01gw_write_sector_with_spare(page_num, sector_num);

    }
    else
    {
        /*Enable Die 1*/
        w25n01gw_die_select(W25M02GW_DIE_ONE);
        page_num = page_num - W25M02GW_TOTAL_PAGES_PER_DIE;
        ret_code = w25n01gw_write_sector_with_spare(page_num, sector_num);

    }
    return ret_code;
}
