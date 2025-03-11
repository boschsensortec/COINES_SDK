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
 * @file    w25n02kw.c
 * @date    Apr 30, 2021
 * @brief   W25N02KW NAND Flash driver
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
#include "w25n02kw_config.h"
#include "w25_common.h"
#include "w25n02kw.h"
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
w25_nand_error_t w25n02kw_init_status = W25_NAND_UNINITIALIZED;

/*! Holds the spi handle */
extern uint8_t w25_spi_intf_handle;

bool format_triggered = false;

/**********************************************************************************/
/* static variables */
/**********************************************************************************/

/*! Opcode to read the Status Register */
#define W25N02KW_CMD_RD_REG                0x0F

/*! Opcode to write the Status Register */
#define W25N02KW_CMD_WR_REG                0x1F

/*! Opcode to enable write */
#define W25N02KW_CMD_WR_ENABLE             0x06

/*! Opcode to erase block */
#define W25N02KW_CMD_128KB_BLCK_ERASE      0xD8

/*! Opcode to load program data */
#define W25N02KW_CMD_LD_PRGM_DATA          0x02

/*! Opcode to load random program data */
#define W25N02KW_CMD_RANDM_PRGM_DATA       0x84

/*! Opcode to execute program */
#define W25N02KW_CMD_PRGM_EXEC             0x10

/*! Opcode to read from page */
#define W25N02KW_CMD_PAGE_DATA_RD          0x13

/*! Opcode to read */
#define W25N02KW_CMD_RD_DATA               0x03

/*! Opcode to get JEDEC id */
#define W25N02KW_CMD_JDEC_ID               0x9F

/*! Opcode to Reset */
#define W25N02KW_CMD_RESET                 0xFF

/*! Position of BUSY bit of status register-3 */
#define W25N02KW_BUSY_STAT                 (1 << 0)

/*! Position of Erase Failure bit of status register-3 */
#define W25N02KW_EFAIL_STAT                (1 << 2)

/*! Position of Program Failure Latch bit of status register-3 */
#define W25N02KW_PFAIL_STAT                (1 << 3)

/*! Position of ECC0 bit of status register-3 */
#define W25N02KW_ECC0_STAT                 (1 << 4)

/*! Position of ECC1 bit of status register-3 */
#define W25N02KW_ECC1_STAT                 (1 << 5)

/*! Position were the Buffer read/ continuous read mode is set in Config register */
#define W25N02KW_REG_CONF_BUF              (1 << 3)

/*! Position of the bit to enable ECC */
#define W25N02KW_REG_CONF_ECCE             (1 << 4)

/*! Command length for reading from the registers */
#define W25N02KW_RD_SREG_CMD_LEN            0x01 // TODO confirm

/*! First two bytes of send buffer will receive nothing.Hence reading from 3rd byte */
#define W25N02KW_MIN_RCV_BYTES_LEN          0x03 // TODO confirm

/*! No of dummy bytes that gets added while reading from the flash */
#define W25N02KW_DUMMY_BYTES_LEN_READ       0x04 // TODO confirm

/*! Max no of bytes stored in look up table. 40 entries. Each entry is of 4 byte length */
#define W25N02KW_MAX_LUT_SIZE               160

/*! Max no of bytes to be stored as spare bytes */
#define W25N02KW_MAX_SPARE_BYTES            4

/*! Max no of spare blocks to use for swapping*/
#define W25N02KW_MAX_SPARE_BLOCKS           39

/*! Spare block start number*/
#define W25N02KW_SPARE_BLOCKS_START         2009

/*! Bad block look up table size*/
#define W25N02KW_BBM_TABLE_SIZE             (W25N02KW_AVAILABLE_BLOCKS * sizeof(uint16_t))

/*! Magic number to check the availability of bbm tables*/
#define W25N02KW_MAGIC_NUMBER               0xDEADBEEF

/*! Block number where the BBM tables + Magic number are stored*/
#define W25N02KW_MAPPING_TABLE_BLOCK        2008

/******************************************************************************/
/* constant definitions 												      */
/******************************************************************************/

/******************************************************************************/
/* global variables 														  */
/******************************************************************************/


/******************************************************************************/
/* static variables 														  */
/******************************************************************************/

/*! Holds the sector buffer data */
static uint8_t w25n02kw_sector_buffer[W25N02KW_SECTOR_SIZE] = { 0 };
/*! Holds the sector buffer filled bytes */
static uint16_t w25n02kw_sector_buff_data_size = 0;
/*! Holds the sector spare buffer data */
static uint8_t w25n02kw_sector_spare_buffer[W25N02KW_MAX_SPARE_BYTES] = { 0 };
/*! Holds the spare buffer data size*/
static uint16_t w25n02kw_sector_spare_buff_data_size = 0;

/*! Logical to physical addr tracking table (bbm look up table)*/
static uint16_t lpn_to_ppn[W25N02KW_AVAILABLE_BLOCKS];
/*! Table to tack the 39 spare blocks for swapping: 0 unused block, 1 used block*/
static uint8_t spare_block_used[W25N02KW_MAX_SPARE_BLOCKS]; 
/*! Buffer used for read/write swap operations*/
static uint8_t page_buffer[4059];
/*! Magic number holder*/
static volatile uint32_t magic_number;

/**********************************************************************************/
/* static function declaration */
/**********************************************************************************/

/*!
 * @brief       This function reads from the register
 *
 * @param[in]   w25n02kw_reg_enum_t : Register to be read
 *
 * @retval      Register value
 */
static uint8_t w25n02kw_read_reg(w25_reg_t reg);

/*!
 * @brief       This function finds a new spare block from the spare_block_used table
 *
 * @param[in]   new_block_num : the free spare block to be used
 *
 * @retval      return 1 in case of success (spare block available) else return 0
 */
static uint8_t find_spare_block(uint16_t* new_block_num);

/*!
 * @brief       This function saves the bbm tables (lpn_to_ppn, spare_block_used, magic_number) to W25N02KW_MAPPING_TABLE_BLOCK
 *
 * @param[in]   void
 *
 * @retval      void
 */
static void save_mapping_table(void);

/*!
 * @brief       This function check if a block is bad
 *
 * @param[in]   block_num : Block to be checked
 *
 * @retval      return 1 if is bad block else return 0
 */
static uint8_t is_bad_block(uint16_t block_num);

//****************************** */
/**********************************************************************************/
/* function definitions */
/**********************************************************************************/

/*!
 * @brief       This function check if a block is bad
 *
 * @param[in]   block_num : Block to be checked
 *
 * @retval      return 1 if is bad block else return 0
 */
static uint8_t is_bad_block(uint16_t block_num)
{
    /* Reads the flash chip block by block and if Byte 0 of Page 0 or first byte of the 64-Byte spare area is non 0xFF then the block is marked
     * as bad bbm is done*/
    uint8_t spare_bad_block_marker_value = 0;
    /*Reading the byte 0 of spare 0 of page 0 */
    (void)w25n02kw_read_spare(&spare_bad_block_marker_value, 1, (block_num * W25N02KW_NO_OF_PAGES), W25N02KW_SPARE_BYTES_START_POS);
    if (spare_bad_block_marker_value != 0xFF)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*!
 * @brief       This function finds a new spare block from the spare_block_used table
 *
 * @param[in]   new_block_num : the free spare block to be used
 *
 * @retval      return 1 in case of success (spare block available) else return 0
 */
static uint8_t find_spare_block(uint16_t* new_block_num)
{
    uint16_t block = 0;
    
    while(block < W25N02KW_MAX_SPARE_BLOCKS)
    {
        // Check if block is marked as unsed
        if (spare_block_used[block] == 0)
        {
            *new_block_num = block + (uint16_t)W25N02KW_SPARE_BLOCKS_START;
            spare_block_used[block] = 1;
            return 1;
        }

        block = block + 1;
    }

    //no free spare blocks available
    return 0;
}

/*!
 * @brief       This function saves the bbm tables (lpn_to_ppn, spare_block_used, magic_number) to W25N02KW_MAPPING_TABLE_BLOCK
 *
 * @param[in]   void
 *
 * @retval      void
 */
static void save_mapping_table()
{
    magic_number = W25N02KW_MAGIC_NUMBER; //magic number to identify the lookup tables
    uint16_t ctr=0;  
    uint8_t cmd_buffer[3] = { 0 };
    uint32_t page_num = W25N02KW_MAPPING_TABLE_BLOCK * W25N02KW_NO_OF_PAGES;   

    /*erase block 2008 before writing new tables*/

    /* Enable write */
    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

    /*Block Erase*/
    cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
    cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
    cmd_buffer[2] = (page_num & 0xff); /*Page Address */

    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_128KB_BLCK_ERASE, &cmd_buffer[0], 3, NULL, 0);

    /*Wait until all the requested blocks are erase*/
    while ((w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR)) & W25N02KW_BUSY_STAT)
    {
        ;
    }

    /*Initialize page_buffer with 0*/
    memset(page_buffer, 0, sizeof(page_buffer));

    /*load page_buffer with the magic number*/
    page_buffer[0] = (uint8_t)(magic_number & 0xFF);
    page_buffer[1] = (uint8_t)((magic_number>>8) & 0xFF);
    page_buffer[2] = (uint8_t)((magic_number>>16) & 0xFF);
    page_buffer[3] = (uint8_t)((magic_number>>24) & 0xFF);

    /*Load lpn_to_ppn look up table to page_buffer*/
    memcpy(page_buffer + 4, lpn_to_ppn, W25N02KW_BBM_TABLE_SIZE);

    /*Load spare_block_used table to page_buffer*/
    memcpy(page_buffer + 4 + W25N02KW_BBM_TABLE_SIZE, spare_block_used, W25N02KW_MAX_SPARE_BLOCKS);

    /*We need to write 4059 bytes in the nand flash so we need 2 pages => 2 * 2048 bytes per page*/
    //Write in Page 3 of Block  W25N02KW_MAPPING_TABLE_BLOCK 
    for (ctr = 0; ctr < W25N02KW_PAGE_SIZE; ctr=ctr+W25N02KW_SECTOR_SIZE)
    {
        w25n02kw_load_sector(page_buffer + ctr, W25N02KW_SECTOR_SIZE, 0);
        w25n02kw_write_sector_with_spare(W25N02KW_MAPPING_TABLE_BLOCK * W25N02KW_NO_OF_PAGES + 3, ctr / W25N02KW_SECTOR_SIZE);
    }
    //Write in Page 4 of Block  W25N02KW_MAPPING_TABLE_BLOCK 
    for (ctr = 0; ctr < W25N02KW_PAGE_SIZE; ctr=ctr+W25N02KW_SECTOR_SIZE)
    {
        w25n02kw_load_sector(page_buffer + W25N02KW_PAGE_SIZE + ctr, W25N02KW_SECTOR_SIZE, 0);
        w25n02kw_write_sector_with_spare((W25N02KW_MAPPING_TABLE_BLOCK * W25N02KW_NO_OF_PAGES) + 4, ctr / W25N02KW_SECTOR_SIZE);
    }

    //Flush buffers
    memset(page_buffer, 0, sizeof(page_buffer));
}

/*!
 * @brief       This function Initialize the bbm tables (lpn_to_ppn, spare_block_used, magic_number) from W25N02KW_MAPPING_TABLE_BLOCK
 *
 * @param[in]   void
 *
 * @retval      void
 */
void initialize_bbm_tables(void)
{
    // Read from page 3 of magic block
    w25n02kw_read(page_buffer, W25N02KW_PAGE_SIZE, W25N02KW_MAPPING_TABLE_BLOCK * W25N02KW_NO_OF_PAGES * W25N02KW_PAGE_SIZE +(3*W25N02KW_PAGE_SIZE) );
    
    // Read from page 4 of magic block
    w25n02kw_read(page_buffer+W25N02KW_PAGE_SIZE, W25N02KW_PAGE_SIZE,(W25N02KW_MAPPING_TABLE_BLOCK * W25N02KW_NO_OF_PAGES * W25N02KW_PAGE_SIZE)+(4*W25N02KW_PAGE_SIZE));

    //memcpy(&magic_number, page_buffer, sizeof(uint32_t));
    magic_number =  (uint32_t)page_buffer[0] |
                    ((uint32_t)page_buffer[1] << 8) |
                    ((uint32_t)page_buffer[2] << 16) |
                    ((uint32_t)page_buffer[3] << 24);

    if (magic_number == W25N02KW_MAGIC_NUMBER) /* If the magic number exists then extract tables*/
    {
        // Load the bbm mapping table
        for (uint16_t i = 0; i < W25N02KW_AVAILABLE_BLOCKS; i++)
        {
            lpn_to_ppn[i] = (uint16_t)page_buffer[sizeof(uint32_t) + 2 * i] + ((uint16_t)page_buffer[sizeof(uint32_t) + 2 * i + 1] << 8);
        }

        // Load the spare block mapping table
        memcpy(spare_block_used, page_buffer + 4 + W25N02KW_BBM_TABLE_SIZE, W25N02KW_MAX_SPARE_BLOCKS);
        
    }
    else /* The magic number does not exist, Initialize tables to 0 state*/
    {
        // Initialize the bbm mapping table
        for (uint16_t i = 0; i < W25N02KW_AVAILABLE_BLOCKS; i++)
        {
            lpn_to_ppn[i] = i; //no swap
        }
        // Initialize the spare block mapping table
        for (uint16_t i = 0; i < W25N02KW_MAX_SPARE_BLOCKS; i++)
        {
            //erase block
            (void)w25n02kw_erase_block(((W25N02KW_SPARE_BLOCKS_START + i) * W25N02KW_BLOCK_SIZE) + 1, W25N02KW_BLOCK_SIZE);

            //mark block as unused
            spare_block_used[i] = 0;
        }

        // Save the initialized tables
        save_mapping_table();
    }
}

w25_nand_error_t w25n02kw_block_bad(uint16_t block)
{
    return 0;
}

/*!
 * @brief This function opens the page that is to be read
 *
 * @param[in]   page_num  Page number
 *
 * @retval  W25N02KW_WRITE_SUCCESS  : Write is success
 * @retval  W25N02KW_ERR_BUFFER_INVALID : Buffer Pointer is NULL
 * @retval  W25N02KW_ERR_BYTE_LEN_INVALID : Length is 0
 * @retval  W25N02KW_WRITE_FAILURE : Write is failed
 *
 */
w25_nand_error_t w25n02kw_page_read(uint32_t page_num)
{
    uint8_t cmd_buffer[3] = { 0 };
    uint16_t block_num;
    uint32_t new_page_num, rest_pos;

    /* Check if block is bad */
    block_num = page_num / W25N02KW_NO_OF_PAGES;
    
    if ((block_num != lpn_to_ppn[block_num])&&(block_num < W25N02KW_AVAILABLE_BLOCKS)) //Block is bad and already swapped => pull new block
    {
        rest_pos =  page_num % W25N02KW_NO_OF_PAGES;
        block_num = lpn_to_ppn[block_num]; //new block swap number
        new_page_num = (block_num * W25N02KW_NO_OF_PAGES) + rest_pos; //update new page num
    }
    else
    {
        new_page_num = page_num; //No swap
    }

    cmd_buffer[0] = ((new_page_num >> 16) & 0xff); /*Page Address */
    cmd_buffer[1] = ((new_page_num >> 8) & 0xff); /*Page Address */
    cmd_buffer[2] = (new_page_num & 0xff); /*Page Address */

    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_PAGE_DATA_RD, &cmd_buffer[0], 3, NULL, 0);

    /*Wait until Ready*/
    while (w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR) & W25N02KW_BUSY_STAT)
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
static uint8_t w25n02kw_read_reg(w25_reg_t reg)
{
    uint8_t recv_buff[W25N02KW_MIN_RCV_BYTES_LEN] = { 0 };

    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle,
    W25N02KW_CMD_RD_REG,
                             (uint8_t*)&reg,
                             W25N02KW_RD_SREG_CMD_LEN,
                             &recv_buff[0],
                             W25N02KW_MIN_RCV_BYTES_LEN);

    return recv_buff[W25N02KW_MIN_RCV_BYTES_LEN - 1];

}

/*!
 * @brief       This function writes to the register
 *
 * @param[in]   reg - Register to which the reg_value to be written
 * @param[in]   reg_value - value to be written
 *
 * @return      None
 */
void w25n02kw_write_reg(w25_reg_t reg, uint8_t reg_value)
{
    uint8_t cmd_buffer[2] = { 0 };

    cmd_buffer[0] = reg;
    cmd_buffer[1] = reg_value;

    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_WR_REG, &cmd_buffer[0], 2, NULL, 0);
}

/*!
 * @brief       This function resets the chip
 *
 */
void w25n02kw_device_reset(void)
{
    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_RESET, NULL, 0, NULL, 0);
}

/*!
 * @brief       This function reads the Look up table holding the swap blocks
 *
 *  @param[in,out]  data_ptr : Pointer to the buffer to which the look up table contents is to be read.
 *
 * @return      None
 */
void w25n02kw_read_bbm_table(uint8_t* data_ptr)
{
    // TBD
}
/*!
 * @brief       This function finds the good block and replaces the bad block with the known good block
 *
 * @param[in]   block - This is the bad block number
 *
 * @return      None
 */
w25_nand_error_t w25n02kw_bbm_block_swap(uint16_t block, uint16_t bbm_block, uint32_t page_num, uint8_t sector_num)
{
    uint16_t new_spare_block;
	uint16_t block_copy;
	uint32_t page_off;
	uint8_t spare_data[64];
	
    if (find_spare_block(&new_spare_block))
    {
		if (page_num != -1) // write operation => we need to transfer the previous pages from bad block to new block
		{
            //erase new spare block
            w25n02kw_erase_block((new_spare_block * W25N02KW_BLOCK_SIZE) + 1, W25N02KW_BLOCK_SIZE);

			if (block != bbm_block) //already swapped block, copy data from the old swapped block
			{
				block_copy = bbm_block;
			}
			else //block not swapped before, copy data from the main block
			{
				block_copy = block;
			}
			
			//Get page offset number
			page_off = page_num - (block_copy * W25N02KW_NO_OF_PAGES);
			
			//init page_buffer and spare_data
			memset(page_buffer, 0xFF, sizeof(page_buffer));
			memset(spare_data, 0xFF, sizeof(spare_data));
			
			//Start Page transfer from bad block to new good block
			for(uint8_t page_nb=0; page_nb<page_off; page_nb++)
			{
				//Read page main array
				(void)w25n02kw_read(page_buffer, W25N02KW_PAGE_SIZE,(block_copy * W25N02KW_NO_OF_PAGES * W25N02KW_PAGE_SIZE) + (page_nb * W25N02KW_PAGE_SIZE));
				
				//Read page spare area
				(void)w25n02kw_read_spare(spare_data, 64, (block_copy * W25N02KW_NO_OF_PAGES) + page_nb, W25N02KW_SPARE_BYTES_START_POS);
				
				//Write to new page location
				uint16_t sec;
				uint16_t spare;
				for (sec = 0, spare =0; sec < W25N02KW_PAGE_SIZE; sec=sec+W25N02KW_SECTOR_SIZE, spare++)
				{
					w25n02kw_load_sector(page_buffer+sec, W25N02KW_SECTOR_SIZE, 0);
					w25n02kw_load_sector_spare(spare_data+4+(spare*16), 4,0);
					w25n02kw_write_sector_with_spare(new_spare_block*W25N02KW_NO_OF_PAGES + page_nb,sec/W25N02KW_SECTOR_SIZE);
				}
			}

            //Copy healthy sectors from page of bad block => read until sector_num
            if (sector_num > 0)//at least one sector should be copied
            {
                //Read page main array
			    (void)w25n02kw_read(page_buffer, sector_num*W25N02KW_SECTOR_SIZE ,(block_copy * W25N02KW_NO_OF_PAGES * W25N02KW_PAGE_SIZE) + (page_off * W25N02KW_PAGE_SIZE));
    
			    //Read page spare area
			    (void)w25n02kw_read_spare(spare_data, 64, (block_copy * W25N02KW_NO_OF_PAGES) + page_off, W25N02KW_SPARE_BYTES_START_POS);
    
			    //Write to new page location
			    uint16_t sec_off;
			    uint16_t spare_off;
			    for (sec_off = 0, spare_off =0; sec_off < (sector_num*W25N02KW_SECTOR_SIZE); sec_off=sec_off+W25N02KW_SECTOR_SIZE, spare_off++)
			    {
			    	w25n02kw_load_sector(page_buffer+sec_off, W25N02KW_SECTOR_SIZE, 0);
			    	w25n02kw_load_sector_spare(spare_data+4+(spare_off*16), 4,0);
			    	w25n02kw_write_sector_with_spare(new_spare_block*W25N02KW_NO_OF_PAGES + page_off,sec_off/W25N02KW_SECTOR_SIZE);
			    }
            }

			//update look up table of bad-to-good blocks
            lpn_to_ppn[block] = new_spare_block;

            //Flush page_buffer
			memset(page_buffer, 0, sizeof(page_buffer));
		}
		else //erase operation
		{
            //update look up table
            lpn_to_ppn[block] = new_spare_block;
			w25n02kw_erase_block((block * W25N02KW_BLOCK_SIZE) + 1, W25N02KW_BLOCK_SIZE);
		}
		
		// Save the updated tables
        save_mapping_table();
		
        return W25_NAND_BBM_SUCCESS;
    }
    else //Max number of block swap reached, so swap slot available
    {
        return W25_NAND_BBM_FAILURE; //no more blocks available for swap
    }
}

/*!
 * @brief This function writes to protection register
 *
 */
void w25n02kw_init_protect_reg()
{
    /*Initalise the Protection register*/
    w25n02kw_write_reg((w25_reg_t)W25N02KW_PROTECT_REG_ADDR, 0);
}

/*!
 * @brief This function writes to configuration register
 *
 */
void w25n02kw_init_config_reg()
{
    uint8_t reg_val;

    /*Initalise the Config register
     * 1)Read from Config Register.
     * 2)Set the ECC-E and BUF
     * */
    reg_val = w25n02kw_read_reg((w25_reg_t)W25N02KW_CONFIG_REG_ADDR);
    reg_val |= W25N02KW_REG_CONF_BUF;
    reg_val |= W25N02KW_REG_CONF_ECCE;
    w25n02kw_write_reg((w25_reg_t)W25N02KW_CONFIG_REG_ADDR, reg_val);

}

/*!
 * @brief This function initializes the w25n02kw driver
 *
 */
w25_nand_error_t w25n02kw_init()
{
    memset(w25n02kw_sector_buffer, 0xFF, sizeof(w25n02kw_sector_buffer));
    memset(w25n02kw_sector_spare_buffer, 0xFF, sizeof(w25n02kw_sector_spare_buffer));
    return W25_NAND_INITIALIZED;
}

/*!
 * @brief This function erases the blocks based on the input position and length
 *
 */
w25_nand_error_t w25n02kw_erase_block(uint32_t pos, uint32_t len)
{
    uint8_t cmd_buffer[3] = { 0 };

    w25_nand_error_t res = W25_NAND_ERROR;
    uint32_t page_num = 0;
    uint32_t new_pos = 0;
    uint32_t rest_pos = 0;
    uint8_t reg_value = 0;
    uint16_t block_num = 0;
    uint16_t org_block = 0;
    pos = pos - 1;

    if (pos % W25N02KW_BLOCK_SIZE != 0 || len % W25N02KW_BLOCK_SIZE != 0)
    {
        res = W25_NAND_ERR_LOCATION_INVALID;
        return res;
    }

    page_num = pos / W25N02KW_PAGE_SIZE;

    /* Check if block is bad */
    block_num = pos / W25N02KW_BLOCK_SIZE;
    org_block = block_num;

    if (block_num == W25N02KW_MAPPING_TABLE_BLOCK) //Skip BBM Block data erase store in block 2008
    {
        return W25_NAND_ERASE_SUCCESS;
    }

    if ((block_num != lpn_to_ppn[block_num])&&(block_num < W25N02KW_AVAILABLE_BLOCKS))
    {
        //calculate relative new position of the block replacement
        rest_pos = pos % W25N02KW_BLOCK_SIZE;
        block_num = lpn_to_ppn[block_num]; //new block swap number
        new_pos = (block_num * W25N02KW_BLOCK_SIZE) + rest_pos;
        page_num = new_pos / W25N02KW_PAGE_SIZE;//update new page num
    }

    if (is_bad_block(org_block) && is_bad_block(block_num)) //if block swapped before org_block will be different from block_num else same block number will be passed
    {
        //swap block
        if (w25n02kw_bbm_block_swap(org_block, block_num, -1, -1) == W25_NAND_BBM_SUCCESS)
        {
            //erase is performed inside block_swap
            return W25_NAND_ERASE_SUCCESS;
        }
        else
        {
            return W25_NAND_ERASE_FAILURE;
        }
    }

    while (len > 0)
    {
        /* Enable write */
        (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

        /*Block Erase*/
        cmd_buffer[0] = ((page_num >> 16) & 0xff); /*Page Address */
        cmd_buffer[1] = ((page_num >> 8) & 0xff); /*Page Address */
        cmd_buffer[2] = (page_num & 0xff); /*Page Address */

        (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_128KB_BLCK_ERASE, &cmd_buffer[0], 3, NULL, 0);

        /*Wait until all the requested blocks are erase*/
        while ((reg_value = w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR)) & W25N02KW_BUSY_STAT)
        {
            ;
        }

        pos += W25N02KW_BLOCK_SIZE;
        len -= W25N02KW_BLOCK_SIZE;

    }

    res = W25_NAND_ERASE_SUCCESS;

    return res;
}

/*!
 * @brief This function erases the entire flash memory
 *
 */
w25_nand_error_t w25n02kw_mass_erase(void)
{

    for (uint16_t i = 0; i < W25N02KW_AVAILABLE_BLOCKS; i++)
    {
        if (w25n02kw_erase_block((i * W25N02KW_BLOCK_SIZE) + 1, W25N02KW_BLOCK_SIZE) != W25_NAND_ERASE_SUCCESS)
        {
            return W25_NAND_ERASE_FAILURE;
        }
    }

    return W25_NAND_ERASE_SUCCESS;
}

/*!
 * @brief This function reads the spare bytes from the page
 */
w25_nand_error_t w25n02kw_read_spare(uint8_t* data_ptr, int8_t no_of_bytes_to_read, uint32_t page_num, uint16_t page_off)
{
    uint8_t cmd_buffer[4] = { 0 };

    uint8_t dummy_byte = 0, reg_val;

    uint8_t rcv_buff[256] = { 0 };
    /*uint16_t block_num;
    uint32_t new_page_num;*/
    
    if (data_ptr == NULL)
    {
        return W25_NAND_ERR_BUFFER_INVALID;
    }

    if (no_of_bytes_to_read == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }

    /*Page read*/
    (void)w25n02kw_page_read(page_num);

    /* Read the data to the data buffer*/
    cmd_buffer[2] = dummy_byte;
    cmd_buffer[0] = ((page_off >> 8) & 0xff);
    cmd_buffer[1] = (page_off & 0xff);

    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle,
    W25N02KW_CMD_RD_DATA,
                             &cmd_buffer[0],
                             3,
                             rcv_buff,
                             no_of_bytes_to_read + W25N02KW_DUMMY_BYTES_LEN_READ);

    /*Copy the data after ignoring the 4 dummy bytes that gets added*/
    memcpy(data_ptr, &rcv_buff[W25N02KW_DUMMY_BYTES_LEN_READ], no_of_bytes_to_read);
    reg_val = w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR);
    if ((reg_val & W25N02KW_ECC0_STAT) || (reg_val & W25N02KW_ECC1_STAT))
    {
        return W25_NAND_ECC_FAILURE;
    }

    return W25_NAND_READ_SUCCESS;

}

/*!
 * @brief This function reads from the flash
 */
w25_nand_error_t w25n02kw_read(uint8_t* data_ptr, uint32_t no_of_bytes_to_read, uint32_t read_loc)
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

    if (read_loc > (W25N02KW_BLOCK_SIZE * W25N02KW_NO_OF_BLOCKS))
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    while (no_of_bytes_to_read > 0)
    {
        /*if (w25n02kw_readReg(W25N02KW_CONFIG_REG_ADDR)&W25N02KW_REG_CONF_ECCE) Check if ECC-E flag is set. If set
         * page size is 1024 else page size is 2048+64, as the bytes used to store */
        page_num = read_loc / (W25N02KW_PAGE_SIZE);
        page_off = read_loc % (W25N02KW_PAGE_SIZE);

        rd_len_page = (uint16_t)MIN(no_of_bytes_to_read, ((W25N02KW_PAGE_SIZE - page_off) + 1));

        /*Page read*/
        (void)w25n02kw_page_read(page_num);

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
            /*Here extra 4 bytes(W25N02KW_DUMMY_BYTES_LEN_READ) are read because , the first 4 bytes received from the flash memory was not containing valid data
             * This is a deviation from the datasheet specification
             * */
            (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle,
            W25N02KW_CMD_RD_DATA,
                                     &cmd_buffer[0],
                                     3,
                                     rcv_buff,
                                     rd_len + W25N02KW_DUMMY_BYTES_LEN_READ);

            /*Copy to the destination buffer*/
            memcpy(data_ptr, &rcv_buff[W25N02KW_DUMMY_BYTES_LEN_READ], rd_len);

            /*Update the parameters like remaining data to be read from the page, page_offset,remaining no of bytes to be
             * read and the new read location*/
            rd_len_page -= rd_len;
            page_off += rd_len;
            no_of_bytes_to_read -= rd_len;
            read_loc += rd_len;
        }
    }

    reg_val = w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR);
    if ((reg_val & W25N02KW_ECC0_STAT) || (reg_val & W25N02KW_ECC1_STAT))
    {
        return W25_NAND_ECC_FAILURE;
    }

    return W25_NAND_READ_SUCCESS;
}

/*!
 * @brief This function loads the data to the sector buffer
 */
w25_nand_error_t w25n02kw_load_sector(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc)
{
    uint16_t sec_off;
    w25_nand_error_t ret_code = W25_NAND_ERROR;

    sec_off = write_loc % W25N02KW_SECTOR_SIZE;
    if (no_of_bytes_to_write <= W25N02KW_SECTOR_SIZE && w25n02kw_sector_buff_data_size <= W25N02KW_SECTOR_SIZE)
    {
        memcpy(&w25n02kw_sector_buffer[sec_off], data_ptr, no_of_bytes_to_write);
        ret_code = W25_NAND_WRITE_SUCCESS;
        w25n02kw_sector_buff_data_size += (uint16_t)no_of_bytes_to_write;
    }
    return ret_code;

}

w25_nand_error_t w25n02kw_load_sector_spare(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t page_num)
{
    w25_nand_error_t ret_code = W25_NAND_ERROR;
    (void)page_num;
    if (no_of_bytes_to_write <= 4)
    {
        memcpy(&w25n02kw_sector_spare_buffer[0], data_ptr, no_of_bytes_to_write);
        w25n02kw_sector_spare_buff_data_size += (uint16_t)no_of_bytes_to_write;
        ret_code = W25_NAND_WRITE_SUCCESS;
    }
    return ret_code;
}

w25_nand_error_t w25n02kw_write_sector_with_spare(uint32_t page_num, uint8_t sector_num)
{
    uint8_t cmd_buffer[3] = { 0 };
    uint8_t op_code;
    uint16_t page_off, block_num, org_block = 0;
    uint8_t tx_buf[3 + 128] = { 0 };
    uint8_t reg_value;
    uint16_t txn_off, txn_len;
    uint32_t new_page_num,rest_pos;
    
    if (w25n02kw_sector_buff_data_size == 0)
    {
        return W25_NAND_ERR_BYTE_LEN_INVALID;
    }
    if ((page_num > W25N02KW_NO_OF_PAGES * W25N02KW_NO_OF_BLOCKS) || (sector_num > 4))
    {
        return W25_NAND_ERR_LOCATION_INVALID;
    }

    /* Check if block is bad */
    block_num = page_num / W25N02KW_NO_OF_PAGES;
    rest_pos = page_num % W25N02KW_NO_OF_PAGES;
    org_block = block_num;

    if ((block_num != lpn_to_ppn[block_num])&&(block_num < W25N02KW_AVAILABLE_BLOCKS))
    {
        block_num = lpn_to_ppn[block_num]; //new block swap number
        new_page_num = (block_num * W25N02KW_NO_OF_PAGES) + rest_pos; 
    }
    else
    {
        new_page_num = page_num;
    }

    if (is_bad_block(org_block) && is_bad_block(block_num)) //if block swapped before org_block will be different from block_num else same block number will be passed
    {
        //swap block
        if (w25n02kw_bbm_block_swap(org_block, block_num, new_page_num, sector_num) == W25_NAND_BBM_SUCCESS)
        {
            block_num = lpn_to_ppn[block_num]; //new block swap number
            new_page_num = (block_num * W25N02KW_NO_OF_PAGES) + rest_pos; 
        }
        else
        {
            return W25_NAND_WRITE_FAILURE;
        }
    }

    page_off = sector_num * W25N02KW_SECTOR_SIZE;

    op_code = W25N02KW_CMD_LD_PRGM_DATA;

    /* Enable write */
    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_WR_ENABLE, NULL, 0, NULL, 0);

    /*Load the data bytes into Databuffer*/
    for (txn_off = 0, txn_len = 0; txn_off < W25N02KW_SECTOR_SIZE; txn_off += txn_len)
    {
        txn_len = MIN(125, W25N02KW_SECTOR_SIZE - txn_off);
        tx_buf[0] = (page_off + txn_off) >> 8;
        tx_buf[1] = (page_off + txn_off) & 0xff;

        /*Copy the buffer that is to be written to the tx_buff*/
        memcpy(tx_buf + 2, &w25n02kw_sector_buffer[txn_off], txn_len);

        /*Load the data*/
        (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, op_code, &tx_buf[0], 2 + txn_len, NULL, 0);

        op_code = W25N02KW_CMD_RANDM_PRGM_DATA;

    }
    /*Load spare bytes*/
    if (w25n02kw_sector_spare_buff_data_size != 0)
    {
        page_off = 0x804 + sector_num * 0x10;

        /*Load the spare bytes into Databuffer*/
        tx_buf[0] = (page_off) >> 8;
        tx_buf[1] = (page_off) & 0xff;

        /*Copy the buffer that is to be written to the tx_buff*/
        memcpy(tx_buf + 2, w25n02kw_sector_spare_buffer, 4);
        (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_RANDM_PRGM_DATA, &tx_buf[0],
                                 2 + 4,
                                 NULL, 0);
    }
    /*Execute the program to write to the flash*/
    cmd_buffer[0] = ((new_page_num >> 16) & 0xff); /*Page Address */
    cmd_buffer[1] = ((new_page_num >> 8) & 0xff); /*Page Address */
    cmd_buffer[2] = (new_page_num & 0xff); /*Page Address */

    /*write to the flash*/
    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle, W25N02KW_CMD_PRGM_EXEC, &cmd_buffer[0], 3, NULL, 0);

    /*Wait until data is written to the flash*/
    while ((reg_value = w25n02kw_read_reg((w25_reg_t)W25N02KW_STATUS_REG_ADDR)) & W25N02KW_BUSY_STAT)
    {
        ;
    }

    w25n02kw_sector_buff_data_size = 0;
    w25n02kw_sector_spare_buff_data_size = 0;
    memset(w25n02kw_sector_buffer, 0xFF, sizeof(w25n02kw_sector_buffer));
    memset(w25n02kw_sector_spare_buffer, 0xFF, sizeof(w25n02kw_sector_spare_buffer));
    return W25_NAND_WRITE_SUCCESS;

}

/*!
 * @brief       This function gives the manufacture id and device id
 */
void w25n02kw_get_manufacture_and_devid(w25_deviceinfo_t* info)
{
    uint8_t dummy_byte = 0;
    uint8_t recv_buff[5];

    memset(recv_buff, 0, 5);

    /*Gets the manufacture id and device id
     * mfg id and device id together is only 3 bytes.
     * However 5 bytes are read because first two bytes received are not valid data.This is a deviation from the datasheet specification
     * */
    (void)w25n02kw_spi_rx_tx(w25_spi_intf_handle,
    W25N02KW_CMD_JDEC_ID,
                             &dummy_byte,
                             W25N02KW_RD_SREG_CMD_LEN,
                             &recv_buff[0],
                             5);

    /*Update the structure*/
    info->mfg_id = recv_buff[2];
    info->device_id = recv_buff[3] << 8 | recv_buff[4];

}

/*!
 * @brief This function gets the initialization status of the module
 */
w25_nand_error_t w25n02kw_get_device_init_status(void)
{
    return w25n02kw_init_status;
}

/*!
 * @brief       This function gives the memory parameters of the flash
 */
void w25n02kw_get_memory_params(w25_memory_params_t* flashMemoryParams)
{
    /* Get the memorySize from the macro defined for the flash module */
    flashMemoryParams->erase_block_units = 1;
    flashMemoryParams->memory_size = W25N02KW_FLASH_SIZE;
    flashMemoryParams->no_of_Sectors = W25N02KW_TOTAL_SECTORS;
    flashMemoryParams->sector_size = W25N02KW_SECTOR_SIZE;
}

/* @brief       This function checks the factory shipped chip and updates the LUT. TO BE TESTED
 */
void w25n02kw_initial_bbm(void)
{
    /* Reads the flash chip block by block and if Byte 0 of Page 0 or first byte of the 64-Byte spare area is non 0xFF then the block is marked
     * as bad bbm is done*/
    uint8_t spare_bad_block_marker_value = 0;
    for (uint16_t block_num = 0; block_num < W25N02KW_AVAILABLE_BLOCKS; block_num++)
    {
        /*Reading the byte 0 of spare 0 of page 0 */
        (void)w25n02kw_read_spare(&spare_bad_block_marker_value, 1, (block_num * W25N02KW_NO_OF_PAGES), W25N02KW_SPARE_BYTES_START_POS);
        if (spare_bad_block_marker_value != 0xFF)
        {
            /*Initiate block swap*/
            (void)w25n02kw_bbm_block_swap(block_num,lpn_to_ppn[block_num],-1,-1);
        }
    }
}
