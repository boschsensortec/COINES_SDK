/*
Copyright (c) 2013, Ben Nahill <bnahill@gmail.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FLogFS Project.
*/

/*!
 * @file flogfs_conf.cpp
 * @author Ben Nahill <bnahill@gmail.com>
 * @ingroup FLogFS
 *
 * @brief Platform-specific interface details
 */
#include <stdlib.h>

#include "w25_common.h"
#include "w25n02jw.h"
#include "flogfs.h"

#include "w25n01gw.h"
#include "w25m02gw.h"


/*! Device Id of w25n01gwtbig */
#define W25N01GW_DEVICE_ID        0xBA21

/*! Device Id of w25m02gwtbig */
#define W25M02GW_DEVICE_ID        0xBB21

/*! Device Id of w25n02jwtbig */
#define W25N02JW_DEVICE_ID		  0xBF22

/*! Device Id of w25n02kwzeir */
#define W25N02KW_DEVICE_ID		  0xBA22 

typedef struct flash_functions
{
	void (*initial_bbm)(void);
	void (*init_protect_reg)(void);
	void (*init_config_reg)(void);
	void (*write_reg)(w25_reg_t, uint8_t);
	void (*device_reset)(void);
	void (*read_bbm_table)(uint8_t *);
	void (*get_memory_params)(w25_memory_params_t *);
	void (*get_manufacture_and_devid)(w25_deviceinfo_t *);
	w25_nand_error_t (*init)(void);
	w25_nand_error_t (*mass_erase)(void);
	w25_nand_error_t (*page_read)(uint32_t);
	w25_nand_error_t (*bbm_block_swap)(uint16_t);
	w25_nand_error_t (*get_device_init_status)(void);
	w25_nand_error_t (*erase_block)(uint32_t , uint32_t);
	w25_nand_error_t (*read)(uint8_t* , uint32_t , uint32_t );
	w25_nand_error_t (*write_sector_with_spare)(uint32_t , uint8_t);
	w25_nand_error_t (*load_sector)(const uint8_t* , uint32_t , uint32_t );	
	w25_nand_error_t (*read_spare)(uint8_t* , int8_t , uint32_t , uint16_t );	
	w25_nand_error_t (*load_sector_spare)(const uint8_t* , uint32_t , uint32_t);	
}flash_f;

typedef uint8_t flash_spare_t[59];

typedef uint8_t fs_lock_t;
#define nullptr NULL


static inline void fs_lock_initialize(fs_lock_t * lock){
	//chMtxInit(lock);
}

static inline void fs_lock(fs_lock_t * lock){
	//chMtxLock(lock);
}

static inline void fs_unlock(fs_lock_t * lock){
	//chMtxUnlock();
}

static flash_spare_t flog_spare_buffer;
static uint16_t flash_block;
static uint16_t flash_page;
static uint8_t flash_sector = 0;
static uint8_t have_metadata;
static uint8_t page_open;
flash_f flash;
volatile uint8_t flash_init_status=false;
static inline flog_result_t flash_initialize(void){
	page_open = 0;
	uint16_t device_id = 0;
	if(flash_init_status ==true)
	{
    return FLOG_SUCCESS;
    }

	if(w25_init(&device_id) == W25_NAND_INITIALIZED)
	{
		if((device_id == W25N01GW_DEVICE_ID) || (device_id == W25M02GW_DEVICE_ID))
		{
			flash.initial_bbm = w25n01gw_initial_bbm;
			flash.init_protect_reg = w25n01gw_init_protect_reg;
			flash.init_config_reg = w25n01gw_init_config_reg;
			flash.write_reg = w25n01gw_write_reg;
			flash.device_reset = w25n01gw_device_reset;
			flash.read_bbm_table = w25n01gw_read_bbm_table;
			flash.get_memory_params = w25n01gw_get_memory_params;
			flash.get_manufacture_and_devid = w25n01gw_get_manufacture_and_devid;
			flash.page_read = w25n01gw_page_read;
			flash.bbm_block_swap = w25n01gw_bbm_block_swap;

			if(device_id == W25N01GW_DEVICE_ID) /* Hearable device */
			{
				flash.init = w25n01gw_init;
				flash.mass_erase = w25n01gw_mass_erase;
				flash.get_device_init_status = w25n01gw_get_device_init_status;
				flash.erase_block = w25n01gw_erase_block;
				flash.read = w25n01gw_read;
				flash.write_sector_with_spare = w25n01gw_write_sector_with_spare;
				flash.load_sector = w25n01gw_load_sector;
				flash.read_spare = w25n01gw_read_spare;
				flash.load_sector_spare = w25n01gw_load_sector_spare;
			}
			else /*APP3.0 - W25M02GW Old Flash chip */
			{
				flash.init = w25m02gw_init;
				flash.mass_erase = w25m02gw_mass_erase;
				flash.get_device_init_status = w25m02gw_get_device_init_status;
				flash.erase_block = w25m02gw_erase_block;
				flash.read = w25m02gw_read;
				flash.write_sector_with_spare = w25m02gw_write_sector_with_spare;
				flash.load_sector = w25m02gw_load_sector;
				flash.read_spare = w25m02gw_read_spare;
				flash.load_sector_spare = w25m02gw_load_sector_spare;
			}
		}
		else if((device_id == W25N02JW_DEVICE_ID) || (device_id == W25N02KW_DEVICE_ID)) /*APP3.0 - W25N02JW Latest Flash chip and APP3.1 - W25N02KW Latest Flash chip*/
		{
			flash.initial_bbm = w25n02jw_initial_bbm;
			flash.init_protect_reg = w25n02jw_init_protect_reg;
			flash.init_config_reg = w25n02jw_init_config_reg;
			flash.write_reg = w25n02jw_write_reg;
			flash.device_reset = w25n02jw_device_reset;
			flash.read_bbm_table = w25n02jw_read_bbm_table;
			flash.get_memory_params = w25n02jw_get_memory_params;
			flash.get_manufacture_and_devid = w25n02jw_get_manufacture_and_devid;
			flash.init = w25n02jw_init;
			flash.mass_erase = w25n02jw_mass_erase;
			flash.page_read = w25n02jw_page_read;
			flash.bbm_block_swap = w25n02jw_bbm_block_swap;
			flash.get_device_init_status = w25n02jw_get_device_init_status;
			flash.erase_block = w25n02jw_erase_block;
			flash.read = w25n02jw_read;
			flash.write_sector_with_spare = w25n02jw_write_sector_with_spare;
			flash.load_sector = w25n02jw_load_sector;
			flash.read_spare = w25n02jw_read_spare;
			flash.load_sector_spare = w25n02jw_load_sector_spare;
		}
		else
		{
			return FLOG_FAILURE;
		}
		/* Init buffers */
		flash.init();
		flash_init_status = true;
		return FLOG_SUCCESS;
	}
	else
	{
		return FLOG_FAILURE;
	}
}

static inline void flash_lock(void){
	//flash.lock();
}

static inline void flash_unlock(void){
	//flash.unlock();
}

static inline flog_result_t flash_open_page(uint16_t block, uint16_t page){
	flash_block = block;
	flash_page = page;
	have_metadata = 0;
	return FLOG_SUCCESS;
	// Read flash page to cache
	//return FLOG_RESULT(flash.page_open(block, page));
}

static inline void flash_close_page(void){
	page_open = 0;
	//flash.unlock();
}

static inline flog_result_t flash_erase_block(uint16_t block){
	page_open = 0;
	flog_result_t ret_val;
	if(flash.erase_block((block*W25_BLOCK_SIZE)+1,W25_BLOCK_SIZE)==W25_NAND_ERASE_SUCCESS)
	{
	    ret_val = FLOG_SUCCESS;
	}
	else
	{
	    ret_val =  FLOG_FAILURE;
	}
	return ret_val;
}

static inline flog_result_t flash_get_spares(void){
//	// Read metadata from flash
//	if(!have_metadata){
//		have_metadata = 1;
//	}
//	return FLOG_RESULT(flash.page_read_continued(flog_spare_buffer, 0x800,
//	                                             sizeof(flog_spare_buffer)));
	return FLOG_SUCCESS;
}

static inline uint8_t * flash_spare(uint8_t sector){
	return &flog_spare_buffer[sector * 16 + 4];
}

static inline flog_result_t flash_block_is_bad(void){
//	uint8_t buffer;
//	flash.page_read_continued(&buffer, 0x800, 1);
//	return FLOG_RESULT(buffer == 0);
	return FLOG_RESULT(0);
}

static inline void flash_set_bad_block(void){

}

/*!
 @brief Commit the changes to the active page
 */
static inline void flash_commit(void){
	page_open = 0;
	flash.write_sector_with_spare((flash_block*FS_PAGES_PER_BLOCK)+flash_page,flash_sector);
//	flash.page_commit();
}

/*!
 @brief Read data from the flash cache (current page only)
 @param dst The destination buffer to fill
 @param chunk_in_page The chunk index within the current page
 @param offset The offset data to retrieve
 @param n The number of bytes to transfer
 @return The success or failure of the operation
 */
static inline flog_result_t flash_read_sector(uint8_t * dst, uint8_t sector, uint16_t offset, uint16_t n){
	sector = sector%FS_SECTORS_PER_PAGE;
	uint32_t read_loc =(flash_block*FS_SECTOR_SIZE*FS_PAGES_PER_BLOCK*FS_SECTORS_PER_PAGE)+( flash_page*FS_SECTORS_PER_PAGE*FS_SECTOR_SIZE )+( FS_SECTOR_SIZE * sector) + offset;
	if(flash.read(dst,n,read_loc) == W25_NAND_READ_SUCCESS)
	{
		return FLOG_SUCCESS;
	}
	else
	{
		return FLOG_FAILURE;
	}
}

static inline flog_result_t flash_read_spare(uint8_t * dst, uint8_t sector){
	sector = sector%FS_SECTORS_PER_PAGE;
	if(flash.read_spare(dst,4,flash_page+(flash_block*64), 0x804+sector * 0x10) == W25_NAND_READ_SUCCESS)
	{
		return FLOG_SUCCESS;
	}
	else
	{
		return FLOG_FAILURE;
	}
}

/*!
 @brief Write chunk data to the flash cache
 @param src A pointer to the data to transfer
 @param chunk_in_page The chunk index within the current page
 @param offset The offset to write the data
 @param n The number of bytes to write
 */
static inline void flash_write_sector(uint8_t const * src, uint8_t sector, uint16_t offset, uint16_t n){
	 flash_sector = sector%FS_SECTORS_PER_PAGE;
    uint32_t write_loc =(flash_block*FS_SECTOR_SIZE*FS_PAGES_PER_BLOCK*FS_SECTORS_PER_PAGE)+( flash_page*FS_SECTORS_PER_PAGE*FS_SECTOR_SIZE )+( FS_SECTOR_SIZE * flash_sector) + offset;
	flash.load_sector(src,n, write_loc);
}


/*!
 @brief Write the spare data for a sector
 @param chunk_in_page The chunk index within the current page

 @note This doesn't commit the transaction
 */
static inline void flash_write_spare(uint8_t const * src, uint8_t sector){

    flash_sector = sector%FS_SECTORS_PER_PAGE;

	flash.load_sector_spare(src,4,(flash_block*FS_PAGES_PER_BLOCK)+flash_page);
}

void flash_debug_warn(char const *f, ...){

}

void flash_debug_error(char const *f, ...){

}
void flash_debug_panic(void) {
}
void flash_high_level(flog_high_level_event_t hle) {
}

uint32_t flash_random(uint32_t max) {
    return rand()%1024;
}

