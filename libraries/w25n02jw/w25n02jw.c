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
 * @file    w25n02jw.c
 * @date    Apr 30, 2021
 * @brief   W25N02JW NAND Flash driver
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

#include "w25n02jw_config.h"
#include "w25_common.h"
#include "w25n02jw.h"
/**********************************************************************************/
/* local macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* constant definitions */
/**********************************************************************************/

/**********************************************************************************/
/* global variables */
/**********************************************************************************/

/*! Holds the initialization status of the module */
w25_nand_error_t w25n02jw_init_status = W25_NAND_UNINITIALIZED;

/*! Holds the spi handle */
extern uint8_t w25_spi_intf_handle;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

/*! Opcode to read the Status Register */
#define W25N02JW_CMD_RD_REG                0x0F

/*! Opcode to write the Status Register */
#define W25N02JW_CMD_WR_REG                0x1F

/*! Opcode to enable write */
#define W25N02JW_CMD_WR_ENABLE             0x06

/*! Opcode to erase block */
#define W25N02JW_CMD_128KB_BLCK_ERASE      0xD8

/*! Opcode to load program data */
#define W25N02JW_CMD_LD_PRGM_DATA          0x02

/*! Opcode to load random program data */
#define W25N02JW_CMD_RANDM_PRGM_DATA       0x84

/*! Opcode to execute program */
#define W25N02JW_CMD_PRGM_EXEC             0x10

/*! Opcode to read from page */
#define W25N02JW_CMD_PAGE_DATA_RD          0x13

/*! Opcode to read */
#define W25N02JW_CMD_RD_DATA               0x03

/*! Opcode to get JEDEC id */
#define W25N02JW_CMD_JDEC_ID               0x9F

/*! Opcode to Reset */
#define W25N02JW_CMD_RESET                 0xFF

/*! Opcode to update the look up table for bad block management */
#define W25N02JW_CMD_BBM_LUT_UPDATE        0xA1

/*! Opcode to read the Look up table*/
#define W25N02JW_CMD_READ_LUT              0xA5

/*! Position of BUSY bit of status register-3 */
#define W25N02JW_BUSY_STAT                 (1 << 0)

/*! Position of Erase Failure bit of status register-3 */
#define W25N02JW_EFAIL_STAT                (1 << 2)

/*! Position of Program Failure Latch bit of status register-3 */
#define W25N02JW_PFAIL_STAT                (1 << 3)

/*! Position of ECC0 bit of status register-3 */
#define W25N02JW_ECC0_STAT                 (1 << 4)

/*! Position of ECC1 bit of status register-3 */
#define W25N02JW_LUT_F_STAT                 (1 << 6)

/*! Position were the Buffer read/ continuous read mode is set in Config register */
#define W25N02JW_REG_CONF_BUF              (1 << 3)

/*! Position of the bit to enable ECC */
#define W25N02JW_REG_CONF_ECCE             (1 << 4)
/*! Command length for reading from the registers */
#define W25N02JW_RD_SREG_CMD_LEN           0x01

/*! First two bytes of send buffer will receive nothing.Hence reading from 3rd byte */
#define W25N02JW_MIN_RCV_BYTES_LEN         0x03

/*! No of dummy bytes that gets added while reading from the flash */
#define W25N02JW_DUMMY_BYTES_LEN_READ      0x04

/*! Max no of bytes stored in look up table. 40 entries. Each entry is of 4 byte length */
#define W25N02JW_MAX_LUT_SIZE                160

/*! Max no of bytes to be stored as spare bytes */
#define W25N02JW_MAX_SPARE_BYTES                4

/*! Holds the sector buffer data */
static uint8_t w25n02jw_sector_buffer[W25N02JW_SECTOR_SIZE] = { 0 };
/*! Holds the sector buffer filled bytes */
static uint16_t w25n02jw_sector_buff_data_size = 0;
/*! Holds the sector spare buffer data */
static uint8_t w25n02jw_sector_spare_buffer[W25N02JW_MAX_SPARE_BYTES] = { 0 };
/*! Holds the spare buffer data size*/
static uint16_t w25n02jw_sector_spare_buff_data_size = 0;
/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

/*!
 * @brief       This function reads from the register
 *
 * @param[in]   w25n02jw_reg_enum_t : Register to be read
 *
 * @retval      Register value
 */
static uint8_t w25n02jw_read_reg(w25_reg_t reg);

/**********************************************************************************/
/* function definitions */
/**********************************************************************************/
/*!
 * @brief This function opens the page that is to be read
 *
 * @param[in]   page_num  Page number
 *
 * @retval  W25N02JW_WRITE_SUCCESS  : Write is success
 * @retval  W25N02JW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval  W25N02JW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval  W25N02JW_WRITE_FAILURE : Write is failed
 *
 */
w25_nand_error_t w25n02jw_page_read(uint32_t page_num)
{
    uint8_t cmd_buffer[3] = { 0 };

    cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
    cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
    cmd_buffer[2] = (page_num & 0xff); /*Page Address */

    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_PAGE_DATA_RD, &cmd_buffer[0], 3, NULL, 0);

    /*Wait until Ready*/
    while (w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR) & W25N02JW_BUSY_STAT)
    {
        ;
    }

    return W25_NAND_READ_SUCCESS;
}

/*!
 * @brief       This function reads from the register
 *
 * @param[in]   reg - Register to be read
 *
 * @return      uint8_t - Register value
 */
static uint8_t w25n02jw_read_reg(w25_reg_t reg)
{
    uint8_t recv_buff[W25N02JW_MIN_RCV_BYTES_LEN] = { 0 };

    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle,
    W25N02JW_CMD_RD_REG,
                             (uint8_t*)&reg,
                             W25N02JW_RD_SREG_CMD_LEN,
                             &recv_buff[0],
                             W25N02JW_MIN_RCV_BYTES_LEN);

    return recv_buff[W25N02JW_MIN_RCV_BYTES_LEN - 1];

}

/*!
 * @brief       This function writes to the register
 *
 * @param[in]   reg - Register to which the reg_value to be written
 * @param[in]   reg_value - value to be written
 *
 * @return      None
 */
void w25n02jw_write_reg(w25_reg_t reg, uint8_t reg_value)
{
    uint8_t cmd_buffer[2] = { 0 };

    cmd_buffer[0] = reg;
    cmd_buffer[1] = reg_value;

    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_REG, &cmd_buffer[0], 2, NULL, 0);
}

/*!
 * @brief       This function resets the chip
 *
 */
void w25n02jw_device_reset(void)
{
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_RESET, NULL, 0, NULL, 0);
}
/*!
 * @brief       This function reads the Look up table holding the swap blocks
 *
 *  @param[in,out]  data_ptr : Pointer to the buffer to which the look up table contents is to be read.
 *
 * @return      None
 */

void w25n02jw_read_bbm_table(uint8_t* data_ptr)
{
    uint8_t cmd_buffer[1] = { 0 };

    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_READ_LUT, &cmd_buffer[0], 1, data_ptr, W25N02JW_MAX_LUT_SIZE + 2);

}

/*!
 * @brief       This function finds the good block and replaces the bad block with the known good block
 *
 * @param[in]   lba - This is the logical block address , the address of the bad block
 *
 * @return      None
 */
w25_nand_error_t w25n02jw_bbm_block_swap(uint16_t lba)
{
    uint8_t cmd_buffer[4] = { 0 };
    uint8_t lut_contents[W25N02JW_MAX_LUT_SIZE + 2] = { 0 };
    uint8_t index_t;
    uint16_t lba_read = 0;
    uint16_t pba_read = 0;

    if (lba >= W25N02JW_AVAILABLE_BLOCKS)
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }
    /*Check if space is available in Look up table for swapping*/
    if (w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR) & W25N02JW_LUT_F_STAT)
    {
        return W25_NAND_BBM_FAILURE;
    }

    /*Read the Look up table*/
    w25n02jw_read_bbm_table(lut_contents);
    /* Check if the link is already present. If already present then find the next available block
     * Initialization value of index_t is given as 2 because when reading from SPI, only after 2 bytes the Look up Table data pops up.
     * This is not mentioned in the datasheet and is a deviation from the expected behaviour
     * */
    for (index_t = 2; index_t < W25N02JW_MAX_LUT_SIZE + 2;)
    {
        lba_read = ((lut_contents[index_t] << 8) & 0xFF00) | lut_contents[index_t + 1];

        /*Check if link space is available*/
        if (lba_read == 0)
        {
            /*Link space is available*/
            break;
        }
        pba_read = ((lut_contents[index_t + 2] << 8) & 0xFF00) | lut_contents[index_t + 3];
        index_t += 4;
    }

    if (pba_read == 0)
    {
        /*No link is available hence use the first replacement block i.e block no: 1004*/
        pba_read = W25N02JW_AVAILABLE_BLOCKS;
    }
    else if ((pba_read >= W25N02JW_AVAILABLE_BLOCKS) && (pba_read < W25N02JW_NO_OF_BLOCKS - 1))
    {
        pba_read++;
    }
    else
    {
        /*All the reserved blocks ie till block number 1023 is being used.Hence return Failure*/
        return W25_NAND_BBM_FAILURE;
    }

    /*Enable write*/
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

    /* Provide instruction to swap the two blocks*/
    cmd_buffer[0] = (lba & 0xFF00) >> 8;
    cmd_buffer[1] = (lba & 0x00FF);
    cmd_buffer[2] = (pba_read & 0xFF00) >> 8;
    cmd_buffer[3] = (pba_read & 0x00FF);
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_BBM_LUT_UPDATE, &cmd_buffer[0], 4, NULL, 0);

    /*Wait until operation is done*/
    while ((w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_BUSY_STAT)
    {
        ;
    }
    return W25_NAND_BBM_SUCCESS;
}

/*!
 * @brief This function writes to protection register
 *
 */

void w25n02jw_init_protect_reg()
{
    /*Initalise the Protection register*/
    w25n02jw_write_reg((w25_reg_t)W25N02JW_PROTECT_REG_ADDR, 0);
}
/*!
 * @brief This function writes to configuration register
 *
 */

void w25n02jw_init_config_reg()
{
    uint8_t reg_val;

    /*Initalise the Config register
     * 1)Read from Config Register.
     * 2)Set the ECC-E and BUF
     * */
    reg_val = w25n02jw_read_reg((w25_reg_t)W25N02JW_CONFIG_REG_ADDR);
    reg_val |= W25N02JW_REG_CONF_BUF;
    reg_val |= W25N02JW_REG_CONF_ECCE;
    w25n02jw_write_reg((w25_reg_t)W25N02JW_CONFIG_REG_ADDR, reg_val);

}

/*!
 * @brief This function initializes the w25n02jw driver
 *
 */
w25_nand_error_t w25n02jw_init()
{
    memset(w25n02jw_sector_buffer, 0xFF, sizeof(w25n02jw_sector_buffer));
    memset(w25n02jw_sector_spare_buffer, 0xFF, sizeof(w25n02jw_sector_spare_buffer));
    return W25_NAND_INITIALIZED;
}

/*!
 * @brief This function erases the blocks based on the input position and length
 *
 */
w25_nand_error_t w25n02jw_erase_block(uint32_t pos, uint32_t len)
{
    uint8_t cmd_buffer[3] = { 0 };

    w25_nand_error_t res = W25_NAND_ERROR;
    uint32_t page_num = 0;
    uint8_t reg_value = 0;
    uint8_t block_num = 0;
    pos = pos - 1;

    if (pos % W25N02JW_BLOCK_SIZE != 0 || len % W25N02JW_BLOCK_SIZE != 0)
    {
        res = W25_NAND_ERR_LOCATION_INVALID;

        return res;
    }

    while (len > 0)
    {
        page_num = pos / W25N02JW_PAGE_SIZE;

        /* Enable write */

        (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

        /*Block Erase*/
        cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
        cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
        cmd_buffer[2] = (page_num & 0xff); /*Page Address */

        (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_128KB_BLCK_ERASE, &cmd_buffer[0], 3, NULL, 0);
        /*Wait until all the requested blocks are erase*/
        while ((reg_value = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_BUSY_STAT)
        {
            ;
        }

        if ((reg_value & W25N02JW_PFAIL_STAT) || (reg_value & W25N02JW_EFAIL_STAT))
        {

            /*If LUT is empty then initiate block swap*/
            if ((w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_LUT_F_STAT)
            {
                /*Get the block number*/
                block_num = (uint8_t)(pos / W25N02JW_BLOCK_SIZE);

                /*Initiate block swap*/
                (void)w25n02jw_bbm_block_swap(block_num);
                /*Repeat the block erase*/
                /* Enable write */
                (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

                /*Block Erase*/
                (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_128KB_BLCK_ERASE, &cmd_buffer[0], 3, NULL, 0);

                /*Wait until all the requested blocks are erase*/
                while ((reg_value = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_BUSY_STAT)
                {
                    ;
                }

                if ((reg_value & W25N02JW_PFAIL_STAT) || (reg_value & W25N02JW_EFAIL_STAT))
                {
                    return W25_NAND_ERASE_FAILURE;
                }
            }
            else
            {
                return W25_NAND_ERASE_FAILURE;
            }
        }

        pos += W25N02JW_BLOCK_SIZE;
        len -= W25N02JW_BLOCK_SIZE;

    }

    res = W25_NAND_ERASE_SUCCESS;

    return res;
}

/*!
 * @brief This function erases the entire flash memory
 *
 */
w25_nand_error_t w25n02jw_mass_erase(void)
{

    for (uint16_t i = 0; i < W25N02JW_AVAILABLE_BLOCKS; i++)
    {
        if (w25n02jw_erase_block((i * W25N02JW_BLOCK_SIZE) + 1, W25N02JW_BLOCK_SIZE) != W25_NAND_ERASE_SUCCESS)
        {
            return W25_NAND_ERASE_FAILURE;
        }
    }

    return W25_NAND_ERASE_SUCCESS;
}

/*!
 * @brief This function reads the spare bytes from the page
 */
w25_nand_error_t w25n02jw_read_spare(uint8_t* data_ptr, int8_t no_of_bytes_to_read, uint32_t page_num, uint16_t page_off)
{
    uint8_t cmd_buffer[4] = { 0 };

    uint8_t dummy_byte = 0, reg_val;

    uint8_t rcv_buff[256] = { 0 };

    if (data_ptr == NULL)
    {
        return W25_NAND_ERR_BUFFER_INVALID;
    }

    if (no_of_bytes_to_read == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }

    /*Page read*/
    (void)w25n02jw_page_read(page_num);

    /* Read the data to the data buffer*/
    cmd_buffer[2] = dummy_byte;
    cmd_buffer[0] = ((page_off >> 8) & 0xff);
    cmd_buffer[1] = (page_off & 0xff);

    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle,
    W25N02JW_CMD_RD_DATA,
                             &cmd_buffer[0],
                             3,
                             rcv_buff,
                             no_of_bytes_to_read + W25N02JW_DUMMY_BYTES_LEN_READ);

    /*Copy the data after ignoring the 4 dummy bytes that gets added*/
    memcpy(data_ptr, &rcv_buff[W25N02JW_DUMMY_BYTES_LEN_READ], no_of_bytes_to_read);
    reg_val = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR);
    if ((reg_val & W25N02JW_ECC0_STAT) || (reg_val & W25N02JW_ECC0_STAT))
    {
        return W25_NAND_ECC_FAILURE;
    }

    return W25_NAND_READ_SUCCESS;

}

/*!
 * @brief This function reads from the flash
 */
w25_nand_error_t w25n02jw_read(uint8_t* data_ptr, uint32_t no_of_bytes_to_read, uint32_t read_loc)
{
    uint8_t cmd_buffer[3] = { 0 };
    uint32_t page_num = 0, page_off = 0;
    uint16_t rd_len_page = 0;
    uint8_t rd_len = 0;
    uint8_t dummy_byte = 0, reg_val;

    uint8_t rcv_buff[256] = { 0 };

    if (data_ptr == NULL)
    {
        return W25_NAND_ERR_BUFFER_INVALID;
    }

    if (no_of_bytes_to_read == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }

    if (read_loc > W25N02JW_FLASH_SIZE)
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    while (no_of_bytes_to_read > 0)
    {
        /*if (w25n02jw_readReg(W25N02JW_CONFIG_REG_ADDR)&W25N02JW_REG_CONF_ECCE) Check if ECC-E flag is set. If set
         * page size is 1024 else page size is 2048+64, as the bytes used to store */
        page_num = read_loc / (W25N02JW_PAGE_SIZE);
        page_off = read_loc % (W25N02JW_PAGE_SIZE);

        rd_len_page = (uint16_t)MIN(no_of_bytes_to_read, ((W25N02JW_PAGE_SIZE - page_off) + 1));

        /*Page read*/
        (void)w25n02jw_page_read(page_num);

        /* Read to the data buffer*/
        cmd_buffer[2] = dummy_byte;

        while (rd_len_page > 0)
        {
            data_ptr += rd_len;
            cmd_buffer[0] = ((page_off >> 8) & 0xff);
            cmd_buffer[1] = (page_off & 0xff);

            /*Since every read cycle has four dummy bytes added in front.*/
            rd_len = (uint8_t)MIN(rd_len_page, 251);

            /*Driver supports maximum reading of 256 bytes. Hence update the page Offset and read 256 bytes in a
             * cycle.*/
            /*Here extra 4 bytes(W25N02JW_DUMMY_BYTES_LEN_READ) are read because , the first 4 bytes received from the flash memory was not containing valid data
             * This is a deviation from the datasheet specification
             * */
            (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle,
            W25N02JW_CMD_RD_DATA,
                                     &cmd_buffer[0],
                                     3,
                                     rcv_buff,
                                     rd_len + W25N02JW_DUMMY_BYTES_LEN_READ);

            /*Copy to the destination buffer*/
            memcpy(data_ptr, &rcv_buff[W25N02JW_DUMMY_BYTES_LEN_READ], rd_len);

            /*Update the parameters like remaining data to be read from the page, page_offset,remaining no of bytes to be
             * read and the new read location*/
            rd_len_page -= rd_len;
            page_off += rd_len;
            no_of_bytes_to_read -= rd_len;
            read_loc += rd_len;
        }
    }

    reg_val = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR);
    if ((reg_val & W25N02JW_ECC0_STAT) || (reg_val & W25N02JW_ECC0_STAT))
    {
        return W25_NAND_ECC_FAILURE;
    }

    return W25_NAND_READ_SUCCESS;
}

/*!
 * @brief This function loads the data to the sector buffer
 */
w25_nand_error_t w25n02jw_load_sector(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc)
{
    uint16_t sec_off;
    w25_nand_error_t ret_code = W25_NAND_ERROR;

    sec_off = write_loc % W25N02JW_SECTOR_SIZE;
    if (no_of_bytes_to_write <= W25N02JW_SECTOR_SIZE && w25n02jw_sector_buff_data_size <= W25N02JW_SECTOR_SIZE)
    {
        memcpy(&w25n02jw_sector_buffer[sec_off], data_ptr, no_of_bytes_to_write);
        ret_code = W25_NAND_WRITE_SUCCESS;
        w25n02jw_sector_buff_data_size += (uint16_t)no_of_bytes_to_write;
    }
    return ret_code;

}

w25_nand_error_t w25n02jw_load_sector_spare(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t page_num)
{
    w25_nand_error_t ret_code = W25_NAND_ERROR;
    (void)page_num;
    if (no_of_bytes_to_write <= 4)
    {
        memcpy(&w25n02jw_sector_spare_buffer[0], data_ptr, no_of_bytes_to_write);
        w25n02jw_sector_spare_buff_data_size += (uint16_t)no_of_bytes_to_write;
        ret_code = W25_NAND_WRITE_SUCCESS;
    }
    return ret_code;
}

w25_nand_error_t w25n02jw_write_sector_with_spare(uint32_t page_num, uint8_t sector_num)
{
    uint8_t cmd_buffer[3] = { 0 };
    uint8_t op_code;
    uint16_t page_off, block_num = 0;
    uint8_t tx_buf[3 + 128] = { 0 };
    uint8_t reg_value;
    uint16_t txn_off, txn_len;

    if (w25n02jw_sector_buff_data_size == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }
    if ((page_num > W25N02JW_NO_OF_PAGES * W25N02JW_AVAILABLE_BLOCKS) || (sector_num > 4))
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    page_off = sector_num * W25N02JW_SECTOR_SIZE;

    op_code = W25N02JW_CMD_LD_PRGM_DATA;

    /* Enable write */
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

    /*Load the data bytes into Databuffer*/
    for (txn_off = 0, txn_len = 0; txn_off < W25N02JW_SECTOR_SIZE; txn_off += txn_len)
    {
        txn_len = MIN(125, W25N02JW_SECTOR_SIZE - txn_off);
        tx_buf[0] = (page_off + txn_off) >> 8;
        tx_buf[1] = (page_off + txn_off) & 0xff;

        /*Copy the buffer that is to be written to the tx_buff*/
        memcpy(tx_buf + 2, &w25n02jw_sector_buffer[txn_off], txn_len);

        /*Load the data*/
        (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, op_code, &tx_buf[0], 2 + txn_len, NULL, 0);

        op_code = W25N02JW_CMD_RANDM_PRGM_DATA;

    }
    /*Load spare bytes*/
    if (w25n02jw_sector_spare_buff_data_size != 0)
    {
        page_off = 0x804 + sector_num * 0x10;

        /*Load the spare bytes into Databuffer*/
        tx_buf[0] = (page_off) >> 8;
        tx_buf[1] = (page_off) & 0xff;

        /*Copy the buffer that is to be written to the tx_buff*/
        memcpy(tx_buf + 2, w25n02jw_sector_spare_buffer, 4);
        (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_RANDM_PRGM_DATA, &tx_buf[0],
                                 2 + 4,
                                 NULL, 0);
    }
    /*Execute the program to write to the flash*/
    cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
    cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
    cmd_buffer[2] = (page_num & 0xff); /*Page Address */

    /*write to the flash*/
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_PRGM_EXEC, &cmd_buffer[0], 3, NULL, 0);

    /*Wait until data is written to the flash*/
    while ((reg_value = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_BUSY_STAT)
    {
        ;
    }

    if (reg_value & W25N02JW_PFAIL_STAT)
    {
        /*If LUT is empty then initiate block swap*/
        if ((w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_LUT_F_STAT)
        {
            block_num = (uint16_t)(page_num / W25N02JW_NO_OF_PAGES);
            /*Give the bad block number to swap with the good block*/
            (void)w25n02jw_bbm_block_swap(block_num);

            /*Erase the block*/
            (void)w25n02jw_erase_block((block_num * W25N02JW_BLOCK_SIZE) + 1, W25N02JW_BLOCK_SIZE);

            /*rewrite the entire buffer*/
            page_off = sector_num * W25N02JW_SECTOR_SIZE;

            (void)w25n02jw_page_read(page_num);
            op_code = W25N02JW_CMD_LD_PRGM_DATA;

            /* Enable write */
            (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

            /*Load the data bytes into Databuffer*/
            for (txn_off = 0, txn_len = 0; txn_off < W25N02JW_SECTOR_SIZE; txn_off += txn_len)
            {
                txn_len = MIN(125, W25N02JW_SECTOR_SIZE - txn_off);
                tx_buf[0] = (page_off + txn_off) >> 8;
                tx_buf[1] = (page_off + txn_off) & 0xff;

                /*Copy the buffer that is to be written to the tx_buff*/
                memcpy(tx_buf + 2, &w25n02jw_sector_buffer[txn_off], txn_len);

                /*Load the data*/
                (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, op_code, &tx_buf[0], 2 + txn_len, NULL, 0);

                op_code = W25N02JW_CMD_RANDM_PRGM_DATA;

            }
            /*Load spare bytes*/
            if (w25n02jw_sector_spare_buff_data_size != 0)
            {
                page_off = 0x800 + sector_num * 0x10;

                /*Load the spare bytes into Databuffer*/
                tx_buf[0] = (page_off) >> 8;
                tx_buf[1] = (page_off) & 0xff;

                /*Copy the buffer that is to be written to the tx_buff*/
                memcpy(tx_buf + 2, w25n02jw_sector_spare_buffer, 4);
                (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_RANDM_PRGM_DATA, &tx_buf[0],
                                         2 + 4,
                                         NULL, 0);
            }
            /*Execute the program to write to the flash*/
            cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
            cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
            cmd_buffer[2] = (page_num & 0xff); /*Page Address */

            /*write to the flash*/
            (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle, W25N02JW_CMD_PRGM_EXEC, &cmd_buffer[0], 3, NULL, 0);

            /*Wait until data is written to the flash*/
            while ((reg_value = w25n02jw_read_reg((w25_reg_t)W25N02JW_STATUS_REG_ADDR)) & W25N02JW_BUSY_STAT)
            {
                ;
            }
            if (reg_value & W25N02JW_PFAIL_STAT)
            {
                return W25_NAND_WRITE_FAILURE;
            }
        }
        else
        {
            return W25_NAND_WRITE_FAILURE;
        }

    }
    w25n02jw_sector_buff_data_size = 0;
    w25n02jw_sector_spare_buff_data_size = 0;
    memset(w25n02jw_sector_buffer, 0xFF, sizeof(w25n02jw_sector_buffer));
    memset(w25n02jw_sector_spare_buffer, 0xFF, sizeof(w25n02jw_sector_spare_buffer));
    return W25_NAND_WRITE_SUCCESS;

}

/*!
 * @brief       This function gives the manufacture id and device id
 */
void w25n02jw_get_manufacture_and_devid(w25_deviceinfo_t* info)
{
    uint8_t dummy_byte = 0;
    uint8_t recv_buff[5];

    memset(recv_buff, 0, 5);

    /*Gets the manufacture id and device id
     * mfg id and device id together is only 3 bytes.
     * However 5 bytes are read because first two bytes received are not valid data.This is a deviation from the datasheet specification
     * */
    (void)w25n02jw_spi_rx_tx(w25_spi_intf_handle,
    W25N02JW_CMD_JDEC_ID,
                             &dummy_byte,
                             W25N02JW_RD_SREG_CMD_LEN,
                             &recv_buff[0],
                             5);

    /*Update the structure*/
    info->mfg_id = recv_buff[2];
    info->device_id = recv_buff[3] << 8 | recv_buff[4];

}

/*!
 * @brief This function gets the initialization status of the module
 */
w25_nand_error_t w25n02jw_get_device_init_status(void)
{
    return w25n02jw_init_status;
}

/*!
 * @brief       This function gives the memory parameters of the flash
 */
void w25n02jw_get_memory_params(w25_memory_params_t* flashMemoryParams)
{
    /* Get the memorySize from the macro defined for the flash module */
    flashMemoryParams->erase_block_units = 1;
    flashMemoryParams->memory_size = W25N02JW_FLASH_SIZE;
    flashMemoryParams->no_of_Sectors = W25N02JW_TOTAL_SECTORS;
    flashMemoryParams->sector_size = W25N02JW_SECTOR_SIZE;
}

/* @brief       This function checks the factory shipped chip and updates the LUT. TO BE TESTED
 */
void w25n02jw_initial_bbm(void)
{
    /* Reads the flash chip block by block and if Byte 0 of Page 0 or first byte of the 64-Byte spare area is non 0xFF then the block is marked
     * as bad bbm is done*/
    uint8_t bad_block_marker_value = 0;
    uint8_t spare_bad_block_marker_value = 0;
    for (uint16_t block_num = 0; block_num < W25N02JW_AVAILABLE_BLOCKS; block_num++)
    {
        /*Reading the byte 0 of page 0 of each block*/
        (void)w25n02jw_read(&bad_block_marker_value, 1, block_num * (W25N02JW_PAGE_SIZE + W25N02JW_SPARE_BYTES_SIZE));
        /*Reading the byte 0 of spare 0 of page 0 */
        (void)w25n02jw_read_spare(&spare_bad_block_marker_value, 1, (block_num * W25N02JW_NO_OF_PAGES), W25N02JW_SPARE_BYTES_START_POS);
        if ((bad_block_marker_value != 0xFF) || (spare_bad_block_marker_value != 0xFF))
        {
            /*Initiate block swap*/
            (void)w25n02jw_bbm_block_swap(block_num);
        }
    }
}
