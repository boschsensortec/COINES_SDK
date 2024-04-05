/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mx25_ext_flash.c
 * @date    Nov 30, 2022
 * @brief   MX25R1635F NOR Flash Driver
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdlib.h>
#include "nrfx_spim.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "mx25_ext_flash.h"

#define SPI_TX_SIZE  			256

#define SPIM0_INSTANCE			2
#define SPI0_SEN_MOSI_PIN		NRF_GPIO_PIN_MAP(0,4)
#define SPI0_SEN_MISO_PIN		NRF_GPIO_PIN_MAP(0,5)
#define SPI0_SEN_SCK_PIN		NRF_GPIO_PIN_MAP(0,3)
#define SPI0_EXT_FLASH_CS_PIN	NRF_GPIO_PIN_MAP(0,26)

const nrfx_spim_t flash_spi_instance = NRFX_SPIM_INSTANCE(2);
nrfx_spim_config_t flash_spi_config = NRFX_SPIM_DEFAULT_CONFIG;
static nrfx_spim_xfer_desc_t flash_spi_xfer;
static uint8_t flash_spi_tx_buff[SPI_TX_SIZE + 4];
static uint8_t m_buffer_rx[SPI_TX_SIZE] = {0};
#ifdef FLASH_TEST
static uint8_t flash_spi_rx_buff[SPI_TX_SIZE];
static uint8_t m_buffer_tx[SPI_TX_SIZE] = {0};
static void flash_test(void);
#endif

/*!
 * @brief       This function reads a region in a block
 *
 * @param[in]   c - Configuration provided during initialization of the littlefs
 * @param[in]   block - Block address
 * @param[in]   buffer- Pointer to the buffer to which the data is to be read
 * @param[in]   size - No of bytes to read
 *
 */
int flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    /* Calculate the  max size/address that is accessible */
    const uint32_t max_size = c->block_count * c->block_size;

    /* Check to make sure the block is correct. */
    if (block > c->block_count ||
        (block * c->block_size + off) > max_size)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    }

    /* Calculate the memory address by multiplying the blocks and adding the offset. */
    uint32_t addr = block * c->block_size + off;

    /* Read the data */
    mx25_read((uint8_t *)buffer, size, addr);

    return 0;
}

/*!
 * @brief       This function program a region in a block
 *
 * @param[in]   c - Configuration provided during initialization of the littlefs
 * @param[in]   block - Block address
 * @param[in]   buffer- Pointer to the buffer of which the data is to be written
 * @param[in]   size - No of bytes to write
 *
 */
int flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	/* Calculate the  max size/address that is accessible */
	const uint32_t max_size = c->block_count * c->block_size;

	/* Check to make sure the block is correct. */
	if (block > c->block_count ||
			(block * c->block_size + off) > max_size)
	{
		APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
	}

	/* Calculate the memory address by multiplying the blocks and adding the offset. */
	uint32_t addr = block * c->block_size + off;

	/* Read the data */
	mx25_write((uint8_t *)buffer, size, addr);


	return 0;
}

/*!
 * @brief       This function program a region in a block
 *
 * @param[in]   c - Configuration provided during initialization of the littlefs
 * @param[in]   block - Block address
 *
 */
int flash_erase(const struct lfs_config *c, lfs_block_t block)
{
    /* Check to make sure the block is correct. */
    if (block > c->block_count)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    }

    /* Calculate the memory address by multiplying the blocks and adding the offset. */
    uint32_t addr = block * c->block_size;

    /* Erase at a certain address */
    mx25_erase_sector(addr);

    return 0;
}

/*!
 * @brief       This function reads from the register
 *
 * @param[in]   reg - Register to be read
 *
 * @return      uint8_t - Register value
 */
uint8_t mx25_read_reg(uint8_t reg)
{
	flash_spi_tx_buff[0] = reg;
	uint8_t recv_buff[MX25_MIN_RCV_BYTES_LEN] = { 0 }; /** First byte of send buffer will receive nothing.Hence reading from 2nd byte */

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 1;
	flash_spi_xfer.p_rx_buffer = &recv_buff[0];
	flash_spi_xfer.rx_length = MX25_MIN_RCV_BYTES_LEN;

	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in reading status reg\r\n");
	}

	 return recv_buff[MX25_MIN_RCV_BYTES_LEN - 1];
}

/*!
 * @brief       This function sends and receives the data via SPIM
 *
 * @param[in]   address : reg address
 * @param[in]   tx_buffer : transmission buffer
 * @param[in]   tx_count : transmission buffer size
 * @param[in]   rx_buffer : receiver buffer
 * @param[in]   rx_count : receiver buffer size
 *
 */
uint32_t mx25_spim_rx_tx(uint8_t address, uint8_t* tx_buffer, uint16_t tx_count, uint8_t* rx_buffer, uint16_t rx_count)
{
	nrfx_err_t err_code = NRFX_SUCCESS;
	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = address;
	memcpy(&flash_spi_tx_buff[1], tx_buffer, tx_count);

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = tx_count + 1;
	flash_spi_xfer.p_rx_buffer = rx_buffer;
	flash_spi_xfer.rx_length = rx_count;

	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		err_code = NRFX_ERROR_INTERNAL;
		printf("\r\n Error Reset Enable CMD\r\n");
		return err_code;
	}
	return err_code;
}

/*!
 * @brief This function erases the selected sector
 *
 *@param[in]   add - Sector address to be erased
 *
 */
uint32_t mx25_erase_sector(uint32_t add)
{
	uint32_t err_code = MX25_ERROR;
	uint8_t cmd_buffer[3] = { 0 };
	uint32_t sector_address = add;
	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	/*
	 * Sector Erase
	 */

	/*Check if the requested position to be erased lies within the Flash size*/
	if (/*(sector_address % MX25_SECTOR_SIZE) != 0 ||*/ sector_address > MX25_FLASH_SIZE)
	{
		err_code = MX25_ERR_LOCATION_INVALID;
		return err_code;
	}

	cmd_buffer[0] = ((sector_address >> 16) & 0xff);
	cmd_buffer[1] = ((sector_address >> 8) & 0xff);
	cmd_buffer[2] = (sector_address & 0xff);
	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = SPIM_MX25_CMD_SE;
	memcpy(&flash_spi_tx_buff[1], cmd_buffer, 3);

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 4;
	flash_spi_xfer.p_rx_buffer = NULL;
	flash_spi_xfer.rx_length = 0;
	//printf("\r\n Process of erasing first sector start\r\n");
	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in sector erase CMD\r\n");
		err_code = NRFX_ERROR_INTERNAL;
		return err_code;
	}
	/*Wait until all the requested sector erased*/
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}
	uint8_t reg_value = 0;
	reg_value = mx25_read_reg(SPIM_MX25_CMD_RD_SCUR);
	if ((reg_value & MX25_E_FAIL_STAT) || (reg_value & MX25_P_FAIL_STAT))
	{
		printf("\r\n Erase Fail \r\n");
		err_code = MX25_ERASE_FAILURE;
		return err_code;
	}
	err_code = MX25_ERASE_SUCCESS;
	return err_code;
}

/*!
 * @brief This function reads from the flash
 *
 * @param[in]   data_ptr : Pointer to the buffer to which the data is to be read.
 * @param[in]   no_of_bytes_to_read : No of bytes to read
 * @param[in]   read_loc : Location from which the data is to be read
 *
 */
uint32_t mx25_read(uint8_t* data_ptr, uint32_t no_of_bytes_to_read, uint32_t read_loc)
{
	uint32_t err_code = MX25_ERROR;
	uint8_t cmd_buffer[3] = { 0 };

	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	//printf("\r\n Process of reading data at start\r\n");
	if (data_ptr == NULL)
	{
		err_code = MX25_ERR_BUFFER_INVALID;
		return err_code;
	}

	if ((no_of_bytes_to_read == 0) || (no_of_bytes_to_read > (SPI_TX_SIZE-1)))
	{
		err_code = MX25_ERR_BYTE_LEN_INVALID;
		return err_code;
	}

	if (read_loc > MX25_FLASH_SIZE)
	{
		err_code = MX25_ERR_LOCATION_INVALID;
		return err_code;
	}

	/*
	 * Read Data
	 */
	cmd_buffer[0] = ((read_loc >> 16) & 0xff);
	cmd_buffer[1] = ((read_loc >> 8) & 0xff);
	cmd_buffer[2] = (read_loc & 0xff);
	memset(m_buffer_rx, 0, sizeof(m_buffer_rx));
	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = SPIM_MX25_CMD_READ;
	memcpy(&flash_spi_tx_buff[1], cmd_buffer, 3);
	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 4;
	flash_spi_xfer.p_rx_buffer = m_buffer_rx;
	flash_spi_xfer.rx_length = no_of_bytes_to_read + 4; /*the first 4 bytes and last byte (256th) received from the flash memory was not containing valid data: only 251 bytes valid data got in each read*/

	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in Read Data CMD\r\n");
		err_code = NRFX_ERROR_INTERNAL;
		return err_code;
	}
	/*Wait until all the requested sector are read*/
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}

	/*Copy to the destination buffer*/
	memcpy((uint8_t *)data_ptr, &m_buffer_rx[4], no_of_bytes_to_read);

	err_code = MX25_READ_SUCCESS;
	return err_code;

}

/*!
 * @brief This function writes the bytes to the page
 *
 * @param[in]   data_ptr : Pointer to the buffer of which the data is to be written.
 * @param[in]   no_of_bytes_to_write : No of bytes to write
 * @param[in]   write_loc : Location to which the data is to be written
 *
 */
uint32_t mx25_write(const uint8_t* data_ptr, uint32_t no_of_bytes_to_write, uint32_t write_loc)
{
	uint32_t err_code = MX25_ERROR;
	uint8_t cmd_buffer[3] = { 0 };

	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	if (data_ptr == NULL)
	{
		err_code = MX25_ERR_BUFFER_INVALID;
		return err_code;

	}

	if (no_of_bytes_to_write == 0)
	{
		err_code = MX25_ERR_BYTE_LEN_INVALID;
		return err_code;
	}

	if (write_loc > MX25_FLASH_SIZE)
	{
		err_code = MX25_ERR_LOCATION_INVALID;
		return err_code;
	}

	/*
	 * Write Data
	 */
	cmd_buffer[0] = ((write_loc >> 16) & 0xff);
	cmd_buffer[1] = ((write_loc >> 8) & 0xff);
	cmd_buffer[2] = (write_loc & 0xff);

	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = SPIM_MX25_CMD_PP;
	memcpy(&flash_spi_tx_buff[1], cmd_buffer, 3);

	memcpy(&flash_spi_tx_buff[4], data_ptr, no_of_bytes_to_write);
	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = no_of_bytes_to_write + 4;
	flash_spi_xfer.p_rx_buffer = NULL;
	flash_spi_xfer.rx_length = 0;

	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in write spim tx/rx \r\n");
	}
	/*Wait until all the requested data are written*/
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}

	err_code = MX25_WRITE_SUCCESS;
	return err_code;
}

/*!
 * @brief This function erases the blocks - 32KB
 *
 * @param[in]   erase_loc - Block address to be erased
 */
uint32_t m25_erase_block(uint32_t erase_loc)
{
	uint32_t err_code = MX25_ERROR;
	uint8_t cmd_buffer[3] = { 0 };
	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	/*
	 * Sector Erase
	 */

	/*Check if the requested position to be erased lies within the Flash size*/
	if (erase_loc > MX25_FLASH_SIZE)
	{
		err_code = MX25_ERR_LOCATION_INVALID;

		return err_code;
	}

	cmd_buffer[0] = ((erase_loc >> 16) & 0xff);
	cmd_buffer[1] = ((erase_loc >> 8) & 0xff);
	cmd_buffer[2] = (erase_loc & 0xff);
	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = SPIM_MX25_CMD_BE;
	memcpy(&flash_spi_tx_buff[1], cmd_buffer, 1);

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 4;
	flash_spi_xfer.p_rx_buffer = NULL;
	flash_spi_xfer.rx_length = 0;
	printf("\r\n Process of erasing first sector start\r\n");
	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in sector erase CMD\r\n");
		err_code = NRFX_ERROR_INTERNAL;
		return err_code;
	}
	/*Wait until all the requested sectors are erased*/
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}
	uint8_t reg_value = 0;
	reg_value = mx25_read_reg(SPIM_MX25_CMD_RD_SCUR);
	if ((reg_value & MX25_E_FAIL_STAT) || (reg_value & MX25_P_FAIL_STAT))
	{
		printf("\r\n Erase Fail \r\n");
		err_code = MX25_ERASE_FAILURE;
		return err_code;
	}
	err_code = MX25_ERASE_SUCCESS;
	return err_code;
}

/*!
 * @brief This function erases the entire flash memory
 *
 */
uint32_t mx25_mass_erase(void)
{

	uint32_t err_code = MX25_ERROR;

	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);


	memset(flash_spi_tx_buff, 0, sizeof(flash_spi_tx_buff));
	flash_spi_tx_buff[0] = 0x60;

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 1;
	flash_spi_xfer.p_rx_buffer = NULL;
	flash_spi_xfer.rx_length = 0;
	printf("\r\n Process of erasing first sector start\r\n");
	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		printf("\r\n Error in sector erase CMD\r\n");
		err_code = NRFX_ERROR_INTERNAL;
		return err_code;
	}
	/*Wait until all the requested sectors are erased*/
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}
	uint8_t reg_value = 0;
	reg_value = mx25_read_reg(SPIM_MX25_CMD_RD_SCUR);
	if ((reg_value & MX25_E_FAIL_STAT) || (reg_value & MX25_P_FAIL_STAT))
	{
		printf("\r\n Erase Fail \r\n");
		err_code = MX25_ERASE_FAILURE;
		return err_code;
	}
	err_code = MX25_ERASE_SUCCESS;
	return err_code;
}

#ifdef FLASH_TEST
/*!
 * @brief To test flash driver api's
 *
 */
static void flash_test(void)
{
	uint8_t data_ptr[SPI_TX_SIZE] = {0};

	srand(0);
	for (int i = 0; i < SPI_TX_SIZE; ++i)
	{
		m_buffer_tx[i] = (uint8_t)rand();
	}

	if (mx25_erase_sector(0x008000) != MX25_ERASE_SUCCESS)
	{
		printf("\r\n Sector Erase Fail!!! \r\n");
	}

	if (m25_erase_block(0x010000) != MX25_ERASE_SUCCESS)
	{
		printf("\r\n 32K Block Erase Fail!!! \r\n");
	}

	if (mx25_read(data_ptr, 16 /*SPI_TX_SIZE-5*/, 0x008000) != MX25_READ_SUCCESS)
	{
		printf("\r\n Read Data Fail\r\n");
	}
	if(data_ptr[4] != 0xff)
	{
		printf("\r\n Sector Erase Failure \r\n");
	}

	if (mx25_read(data_ptr, SPI_TX_SIZE-5, 0x010000) != MX25_READ_SUCCESS)
	{
		printf("\r\n Read Data Fail\r\n");
	}
	if(data_ptr[0] != 0xff)
	{
		printf("\r\n Block Erase Failure \r\n");
	}

	if (mx25_write(m_buffer_tx, 128, 0x008000) != MX25_WRITE_SUCCESS)
	{
		printf("\r\n Write Data Fail\r\n");
	}

	if (mx25_read(data_ptr, 128, 0x008000) != MX25_READ_SUCCESS)
	{
		printf("\r\n Read Data Fail\r\n");
	}
	if(data_ptr[0] == 0xa5)
	{
		printf("\r\nSuccess: Read data 0xa5 \r\n");
	}

	if (memcmp(m_buffer_tx, &data_ptr[0], 5) == 0)
	{
		printf("\r\nData consistent\r\n");
	}
	else
	{
		printf("\r\nData inconsistent\r\n");
	}

}
#endif
/*!
 * @brief   This function reads the manufacture id and device id
 *
 */
uint32_t mx25_read_electronic_id(mx25_deviceinfo_t* info)
{
	uint8_t recv_buff[6];
	memset(recv_buff, 0, 6);
	nrfx_err_t err_code = NRFX_SUCCESS;

	flash_spi_tx_buff[0] = SPIM_MX25_CMD_REMS;
	flash_spi_tx_buff[3] = 0x00; /** Output the manufacturer ID first*/

	flash_spi_xfer.p_tx_buffer = flash_spi_tx_buff;
	flash_spi_xfer.tx_length = 0 + 4;
	flash_spi_xfer.p_rx_buffer = recv_buff;
	flash_spi_xfer.rx_length = 6; /** 6 bytes are read because first 4 bytes received are not valid data. */

	if (nrfx_spim_xfer(&flash_spi_instance, &flash_spi_xfer, 0) != NRFX_SUCCESS)
	{
		err_code = NRFX_ERROR_INTERNAL;
		printf("\r\n Error Read Electronic ID\r\n");
		return err_code;
	}

	info->mfg_id = recv_buff[4];  	/**< Manufacturer Id : C2*/
	info->device_id = recv_buff[5]; /**< Device Id : 15*/

	return err_code;
}

/*!
 * @brief   This function resets the chip
 *
 */
void mx25_device_reset(void)
{
	mx25_spim_rx_tx(SPIM_MX25_CMD_RST, NULL, 0, NULL, 0);
	/*Wait until all the device is powered up */
	while ((mx25_read_reg(SPIM_MX25_CMD_RD_SR)) & MX25_BUSY_STAT)
	{
		;
	}
}

/*!
 * @brief       This function Sync the state of the underlying block device
 *
 * @param[in]   c - Configuration provided during initialization of the littlefs
 *
 */
int flash_sync(const struct lfs_config *c)
{
    return 0;
}

/*!
 * @brief This function initializes the MX25R1635F driver
 *
 */
uint32_t flash_init()
{
	mx25_deviceinfo_t  info;
	nrfx_err_t err_code = NRFX_SUCCESS;

	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	/*
	 * Reset Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_RST_EN, NULL, 0, NULL, 0);

	/*
	 * Reset CMD
	 */
	mx25_device_reset();

	/*
	 * Write Enable CMD
	 */
	mx25_spim_rx_tx(SPIM_MX25_CMD_WR_EN, NULL, 0, NULL, 0);

	/*
	 * Read Electronic ID
	 */
	mx25_read_electronic_id(&info);

	/*
	 * Function to test Flash driver
	 */
#ifdef FLASH_TEST
	flash_test();
#endif

	return err_code;

}


