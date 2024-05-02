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
 * @file flogfs.c
 * @author Ben Nahill <bnahill@gmail.com>
 * @ingroup FLogFS
 * @brief Core file system implementation
 */

#include <stdio.h>
#include <string.h>

#include "flogfs.h"
#include "flogfs_private.h"

#ifndef IS_DOXYGEN
#if !FLOG_BUILD_CPP
#ifdef __cplusplus
extern "C" {
#endif
#endif
#endif

#include "flogfs_conf_implement.h"

static void flog_assert(const char *msg, const char *file, int lineno) {
    flash_debug_error("Assertion failed: %s:%d %s", file, lineno, msg);
    flash_debug_panic();
}

#define assert(EX) (void)((EX) || (flog_assert(#EX, __FILE__, __LINE__), 0))

//! @Holds the value of latest preallocated block -Done as part of bug fix
uint16_t last_preallocated_block = 0;
//! @addtogroup FLogPrivate
//! @{

typedef struct flog_block_alloc_t {
    flog_block_idx_t block;
    flog_block_age_t age;
    struct flog_block_alloc_t *next;
} flog_block_alloc_t;

typedef struct {
    //! Block indices and ages
    flog_block_alloc_t blocks[FS_PREALLOCATE_SIZE];
    //! The number of entries
    uint16_t n;
    //! The sum of all ages
    flog_block_age_t age_sum;
    flog_block_alloc_t *free;
    flog_block_alloc_t *available;
    flog_block_alloc_t *pending;
} flog_prealloc_list_t;

typedef struct {
    flog_block_idx_t block;
    flog_write_file_t *file;
} flog_dirty_block_t;

typedef struct {
    flog_file_id_t file_id;
    flog_block_idx_t first_block;
} flog_file_find_result_t;

/*!
 @brief The complete FLogFS state structure
 */
typedef struct {
    //! The head of the list of read files
    flog_read_file_t *read_head;
    //! The head of the list of write files
    flog_write_file_t *write_head;

    //! The maximum file ID active in the system
    uint32_t max_file_id;

    //! The state of the file system
    flog_state_t state;

    //! A list of preallocated blocks for quick access
    flog_prealloc_list_t prealloc;

    //! The most recent timestamp (sequence number)
    //! @note To put a stamp on a new operation, you should preincrement. This
    //! is the timestamp of the most recent operation
    flog_timestamp_t t;

    //! Version
    uint32_t version;

    //! The location of the first inode block
    flog_block_idx_t inode0;

    //! @brief Flash cache status
    //! @note This must be protected under @ref flogfs_t::lock !
    struct {
        flog_block_idx_t current_open_block;
        flog_page_index_t current_open_page;
        uint_fast8_t page_open;
        flog_result_t page_open_result;
    } cache_status;

    //! A lock to serialize some FS operations
    fs_lock_t lock;
    //! A lock to block any allocation-related operations
    fs_lock_t allocate_lock;
    //! A lock to serialize deletion operations
    fs_lock_t delete_lock;

    //! The one dirty_block
    //! @note This may only be accessed under @ref flogfs_t::allocate_lock
    flog_dirty_block_t dirty_block;
    //! The moving allocator head
    flog_block_idx_t allocate_head;

    //! Initialized parameters.
    flog_initialize_params_t params;
} flogfs_t;

/* Note: This done as part of bug fix, deletion of block is separated. */
typedef struct {
    flog_block_idx_t block_idx[FS_NUM_BLOCKS];
    uint16_t invalid_block_count;
} flog_invalidated_block_t;

static flog_invalidated_block_t flog_invalidated_block;

//! A single static instance
static flogfs_t flogfs;

static inline void flog_lock_fs() {
    fs_lock(&flogfs.lock);
}
static inline void flog_unlock_fs() {
    fs_unlock(&flogfs.lock);
}

static inline void flog_lock_allocate() {
    fs_lock(&flogfs.allocate_lock);
}
static inline void flog_unlock_allocate() {
    fs_unlock(&flogfs.allocate_lock);
}

static inline void flog_lock_delete() {
    fs_lock(&flogfs.delete_lock);
}
static inline void flog_unlock_delete() {
    fs_unlock(&flogfs.delete_lock);
}

/*!
 @brief Go find a suitable free block to use
 @return A block. The index will be FLOG_BLOCK_IDX_INVALID if invalid.

 This attempts to claim a block from the block preallocation list and searches
 for a new one of the list is empty

 @note This requires flogfs_t::allocate_lock
 */
static flog_block_alloc_t flog_allocate_block(int32_t threshold);

/*!
 @brief Get the next block entry from any valid block
 @param block The previous block
 @returns The next block

 This is fine for both inode and file blocks

 If block == FLOG_BLOCK_IDX_INVALID, it will be returned
 */
static inline flog_block_idx_t flog_universal_get_next_block(flog_block_idx_t block);

/*!
 @brief Find a file inode entry
 @param[in] filename The filename to check for
 @param[out] iter An inode iterator -- If nothing is found, this will point to
                  the next free inode iterator
 @retval Fileinfo with first_block == FLOG_BLOCK_IDX_INVALID if not found

 @note This requires the FS lock, \ref flogfs_t::lock
 */
static flog_file_find_result_t flog_find_file(char const *filename, flog_inode_iterator_t *iter);

/*!
 @brief Open a page (read to flash cache) only if necessary
 */
static inline flog_result_t flog_open_page(flog_block_idx_t block, flog_page_index_t page);

/*!
 @brief Open the page corresponding to a sector
 @param block The block
 @param sector The sector you wish to access
 */
static inline flog_result_t flog_open_sector(flog_block_idx_t block, flog_sector_idx_t sector);

static void flog_close_sector();

static flog_result_t flogfs_read_calc_file_size(flog_read_file_t *file);

/*!
 @brief Initialize an inode iterator
 @param[in,out] iter The iterator structure
 @param[in] inode0 The first inode block to use
 */
static void flog_inode_iterator_initialize(flog_inode_iterator_t *iter, flog_block_idx_t inode0);

/*!
 @brief Advance an inode iterator to the next entry
 @param[in,out] iter The iterator structure

 This doesn't deal with any allocation. That is done with
 flog_inode_prepare_new.

 @warning You MUST check on every iteration for the validity of the entry and
 not iterate past an unallocated entry.
 */
static void flog_inode_iterator_next(flog_inode_iterator_t *iter);

/*!
 @brief Claim a new inode entry as your own.

 @note This requires the @ref flogfs_t::allocate_lock. It might allocate
 something.

 @warning Please be sure that iter points to the first unallocated entry
 */
static flog_result_t flog_inode_prepare_new(flog_inode_iterator_t *iter);

/*!
 @brief Get the value of the next sector in sequence
 @param sector The previous sector
 @return Next sector

 Sectors are written and read out of order so this is used to get the correct
 sequence
 */
static inline flog_sector_idx_t flog_increment_sector(flog_sector_idx_t sector);

static flog_result_t flog_flush_write(flog_write_file_t *file);

/*!
 @brief Add a free block candidate to the preallocation list

 @note This requires the allocation lock
 */
static void flog_prealloc_push(flog_block_idx_t block, flog_block_age_t age);

/*!
 @brief Take the youngest block from the preallocation list
 @retval Index The allocated block index
 @retval FLOG_BLOCK_IDX_INVALID if empty

 @note This requires the allocation lock
 */
static flog_block_alloc_t flog_prealloc_pop(int32_t threshold);

static uint_fast8_t flog_prealloc_contains(flog_block_idx_t block);

static flog_result_t flog_prealloc_initialize();

static flog_result_t flog_prealloc_prime();

/*!
 @brief Invalidate a chain of blocks
 @param base The first block in the chain
 */
static void flog_invalidate_chain(flog_block_idx_t base, flog_file_id_t file_id);
/* Note: This done as part of bug fix, deletion of block is separated. */
static void flog_invalidate_file_block(flog_block_idx_t block, flog_file_id_t file_id);

/*!
 @brief Check for a dirty block and flush it to allow for a new allocation

 @note This requires the allocation lock
 */
static void flog_flush_dirty_block();

static flog_result_t flog_commit_file_sector(flog_write_file_t *file, uint8_t const *data, flog_sector_nbytes_t n);

static flog_block_type_t flog_get_block_type(flog_block_idx_t block);

static void flog_block_statistics_write(flog_block_idx_t block, flog_block_statistics_sector_with_key_t const *stat);

static void flog_block_statistics_read(flog_block_idx_t block, flog_block_statistics_sector_with_key_t *stat);

static uint_fast8_t invalid_block(flog_block_statistics_sector_with_key_t *sector) {
    return memcmp(sector->key, flog_block_statistics_key, sizeof(flog_block_statistics_key)) != 0;
}

static uint_fast8_t invalid_block_or_older_version(flog_block_statistics_sector_with_key_t *sector) {
    return sector->header.version != flogfs.version || invalid_block(sector);
}

static uint_fast8_t invalid_sector_spare(flog_file_sector_spare_t *spare) {
    return spare->type_id == FLOG_SECTOR_TYPE_ID_ERASED && spare->nbytes == FLOG_SECTOR_NBYTES_ERASED;
}

/* Since this is not used, commenting it out
static uint_fast8_t invalid_universal_invalidation_header(flog_inode_file_invalidation_header_t *header) {
    return header->timestamp != FLOG_TIMESTAMP_ERASED;
}
*/
static uint_fast8_t invalid_inode_file_allocation_header(flog_inode_file_allocation_header_t *header) {
    return header->file_id == FLOG_FILE_ID_ERASED;
}

static uint_fast8_t invalid_inode_file_invalidation(flog_inode_file_invalidation_t *invalidation) {
    return invalidation->header.timestamp == FLOG_TIMESTAMP_ERASED;
}

static uint_fast8_t invalid_file_tail_sector_header(flog_file_tail_sector_header_t *header) {
    return header->universal.timestamp == FLOG_TIMESTAMP_ERASED;
}

static uint_fast8_t invalid_block_index(flog_block_idx_t idx) {
    return idx == FLOG_BLOCK_IDX_ERASED;
}

static uint_fast8_t invalid_file_id(flog_file_id_t file_id) {
    return file_id == FLOG_FILE_ID_ERASED;
}

static uint_fast8_t invalid_timestamp(flog_timestamp_t timestamp) {
    return timestamp == FLOG_TIMESTAMP_ERASED;
}

static uint_fast8_t is_file_init_sector_header_for_file(flog_file_init_sector_header_t *header, flog_file_id_t file_id) {
    return (file_id != FLOG_FILE_ID_ERASED) && (header->file_id == file_id);
}

static uint_fast8_t flog_prealloc_is_empty();

static uint_fast8_t flog_prealloc_is_full();

static flog_result_t unlock_and_fail() {
    flash_unlock();
    flog_unlock_fs();
    return FLOG_FAILURE;
}

//! @}

flog_result_t flogfs_initialize(flog_initialize_params_t *params) {
    fs_lock_initialize(&flogfs.allocate_lock);
    fs_lock_initialize(&flogfs.lock);
    fs_lock_initialize(&flogfs.delete_lock);

    flogfs.state = FLOG_STATE_RESET;
    flogfs.cache_status.page_open = 0;
    flogfs.version = 31337;
    flogfs.dirty_block.block = FLOG_BLOCK_IDX_INVALID;
    flogfs.dirty_block.file = NULL;

    flogfs.params = *params;

    return flash_initialize();
}

flog_result_t flogfs_format() {
    flog_block_idx_t block;
    flog_block_idx_t first_valid = FLOG_BLOCK_IDX_INVALID;

    union {
        flog_inode_init_sector_t main_buffer;
        flog_inode_init_sector_spare_t spare_buffer;
    } buffer_union;

    flog_block_statistics_sector_with_key_t statistics_sector;

    flash_lock();
    flog_lock_fs();

    flash_high_level(FLOG_FORMAT_BEGIN);

    if (flogfs.state == FLOG_STATE_MOUNTED) {
        flogfs.state = FLOG_STATE_RESET;
    }

    for (block = FS_FIRST_BLOCK; block < flogfs.params.number_of_blocks; block++) {
        flog_open_page(block, 0);
        if (FLOG_SUCCESS == flash_block_is_bad()) {
            continue;
        }
        flog_block_statistics_read(block, &statistics_sector);
        flog_close_sector();

        if (invalid_block(&statistics_sector)) {
            statistics_sector.header.age = 0;
            memcpy(statistics_sector.key, flog_block_statistics_key, sizeof(flog_block_statistics_key));
            statistics_sector.header.version = flogfs.version;
        }
        else {
            statistics_sector.header.version++;
            flogfs.version = statistics_sector.header.version;
        }
        statistics_sector.header.next_block = FLOG_BLOCK_IDX_INVALID;
        statistics_sector.header.next_age = FLOG_BLOCK_AGE_INVALID;
        statistics_sector.header.timestamp = 0;

        if (FLOG_FAILURE == flash_erase_block(block)) {
            flog_unlock_fs();
            flash_unlock();
            flash_debug_error("FLogFS:" LINESTR);
            return FLOG_FAILURE;
        }

        flog_block_statistics_write(block, &statistics_sector);
		/*Commenting out as flash commit is already called in the above function*/
        //flash_commit();

        if (first_valid == FLOG_BLOCK_IDX_INVALID) {
            first_valid = block;
            break;
        }
    }

    assert(first_valid != FLOG_BLOCK_IDX_INVALID);

    flog_open_sector(first_valid, FLOG_INIT_SECTOR);

    buffer_union.main_buffer.universal.timestamp = 0;
    buffer_union.main_buffer.previous = FLOG_BLOCK_IDX_INVALID;
    flash_write_sector((const uint8_t *)&buffer_union.main_buffer, FLOG_INIT_SECTOR, 0, sizeof(buffer_union.main_buffer));

    buffer_union.spare_buffer.inode_index = 0;
    buffer_union.spare_buffer.type_id = FLOG_BLOCK_TYPE_INODE;
    flash_write_spare((const uint8_t *)&buffer_union.spare_buffer, FLOG_INIT_SECTOR);

    flash_commit();

    flash_high_level(FLOG_FORMAT_END);

    flog_unlock_fs();
    flash_unlock();
    return FLOG_SUCCESS;
}

flog_result_t flog_prealloc_initialize() {
    flogfs.prealloc.n = 0;
    flogfs.prealloc.free = NULL;
    flogfs.prealloc.available = NULL;
    flogfs.prealloc.pending = NULL;

    for (uint8_t i = 0; i < FS_PREALLOCATE_SIZE; ++i) {
        flog_block_alloc_t *block = &flogfs.prealloc.blocks[i];
        block->block = FLOG_BLOCK_IDX_INVALID;
        block->age = FLOG_BLOCK_AGE_INVALID;
        block->next = flogfs.prealloc.free;
        flogfs.prealloc.free = block;
    }

    return FLOG_SUCCESS;
}

flog_result_t flog_prealloc_prime() {
    flog_block_statistics_sector_with_key_t statistics_sector;
    flog_inode_init_sector_spare_t inode_spare;
    flog_block_idx_t block=0;
    flog_result_t fr;
    uint16_t tries = FS_NUM_BLOCKS-1-last_preallocated_block;
    uint16_t i=last_preallocated_block+1;


    flog_lock_fs();
    flash_lock();

    flash_high_level(FLOG_PRIME_BEGIN);

    flash_debug_warn("Priming");


    while (!flog_prealloc_is_full() && tries > 0) {
	/* Commenting this as the random() always generates a similar sequence when it is called.
	Hence sometimes it ends up such that the pre allocation buffer remains empty and other operations (file create etc) do not follow.
	*/
       // block = flash_random(flogfs.params.number_of_blocks);
        block = i;
        i++;

        if (block == 0) {
            continue;
        }

        if (flog_prealloc_contains(block)) {
            flash_debug_warn("%d: Prealloc contains", block);
            continue;
        }

        if (FLOG_FAILURE == flash_open_page(block, 0)) {
            flash_debug_warn("%d: Unable to open", block);
            continue;
        }

        if (FLOG_SUCCESS == flash_block_is_bad()) {
            flash_debug_warn("%d: Bad block", block);
            continue;
        }
        flog_block_statistics_read(block, &statistics_sector);
        flash_read_spare((uint8_t *)&inode_spare, FLOG_INIT_SECTOR);
        flog_close_sector();

        if (invalid_block_or_older_version(&statistics_sector)) {
            statistics_sector.header.age = 0;
            statistics_sector.header.next_block = FLOG_BLOCK_IDX_INVALID;
            statistics_sector.header.next_age = FLOG_BLOCK_AGE_INVALID;
            statistics_sector.header.timestamp = 0;
            statistics_sector.header.version = flogfs.version;
            memcpy(statistics_sector.key, flog_block_statistics_key, sizeof(flog_block_statistics_key));

            if (FLOG_FAILURE == flash_erase_block(block)) {
                return unlock_and_fail();
            }

            flog_block_statistics_write(block, &statistics_sector);
			/*Commenting out as flash commit is already called in the above function*/
            //flash_commit(); 

            flog_prealloc_push(block, statistics_sector.header.age);
        }
        else {
            switch (inode_spare.type_id) {
            case FLOG_BLOCK_TYPE_UNALLOCATED: {
                flog_prealloc_push(block, statistics_sector.header.age);
                break;
            }
            default: {
                flash_debug_warn("%d: Bad type (%d)", block, inode_spare.type_id);
                break;
            }
            }
        }
        //So if a portion of memory which is inbetween is deleted, and storage space shows full, then we need to check from the first block to see, if free blocks are available
        if(!flog_prealloc_is_full()&&tries==1&&last_preallocated_block!=0)
        {
            tries=last_preallocated_block;
            i=1;
            last_preallocated_block=0;
        }
        tries--;
    }

    if(last_preallocated_block!=(FS_NUM_BLOCKS-1))
    {
        last_preallocated_block = block;
    }
    else
    {
        last_preallocated_block=0;
    }

    fr = FLOG_RESULT(!flog_prealloc_is_empty());
    if (fr == FLOG_FAILURE) {
        flash_debug_error("flog_prealloc_prime: flog_prealloc_is_empty");
    }

    flash_high_level(FLOG_PRIME_END);

    flog_unlock_fs();
    flash_unlock();
    return FLOG_SUCCESS;
}

static flog_block_idx_t flogfs_find_first_inode() {
    flog_block_statistics_sector_with_key_t statistics_sector;
    flog_inode_init_sector_spare_t inode_spare;
    flog_block_idx_t block;

    for (block = FS_FIRST_BLOCK; block < FS_INODE0_MAX_BLOCK; block++) {
        if (!flash_open_page(block, 0)) {
            continue;
        }

        if (flash_block_is_bad()) {
            continue;
        }

        flog_block_statistics_read(block, &statistics_sector);
        flash_read_spare((uint8_t *)&inode_spare, FLOG_INIT_SECTOR);
        flog_close_sector();

        if (invalid_block_or_older_version(&statistics_sector)) {
            continue;
        }

        // TODO: Previous if we found two inode0 blocks we would compare
        // timestamps. Should we be guarding against that?
        if (inode_spare.type_id == FLOG_BLOCK_TYPE_INODE) {
            if (inode_spare.inode_index == 0) {
                return block;
            }
        }
    }

    return FLOG_BLOCK_IDX_INVALID;
}

static flog_result_t flogfs_inspect() {
    flog_inode_iterator_t inode_iter;
    flog_inode_file_allocation_header_t allocation;

    for (flog_inode_iterator_initialize(&inode_iter, flogfs.inode0); ; flog_inode_iterator_next(&inode_iter)) {
        flog_open_sector(inode_iter.block, inode_iter.sector);
        flash_read_sector((uint8_t *)&allocation, inode_iter.sector, 0, sizeof(flog_inode_file_allocation_header_t));
        if (invalid_inode_file_allocation_header(&allocation)) {
            break;
        }

        if (allocation.file_id > flogfs.max_file_id) {
            flogfs.max_file_id = allocation.file_id;
        }
    }

    return FLOG_SUCCESS;
}

flog_result_t flogfs_mount() {
    flog_lock_fs();

    if (flogfs.state == FLOG_STATE_MOUNTED) {
        flog_unlock_fs();
        return FLOG_SUCCESS;
    }

    flash_lock();

    flogfs.allocate_head = 0;
    flogfs.max_file_id = 0;
    flogfs.cache_status.current_open_block = 0;
    flogfs.cache_status.current_open_page =0;
    flogfs.cache_status.page_open=0;
    flogfs.cache_status.page_open_result=0;
    flogfs.read_head = NULL;
    flogfs.write_head = NULL;
    flogfs.dirty_block.block = FLOG_BLOCK_IDX_INVALID;
    flogfs.dirty_block.file = NULL;
    flogfs.inode0 = flogfs_find_first_inode();
    last_preallocated_block = 0;

    if (flogfs.inode0 == FLOG_BLOCK_IDX_INVALID) {
        return unlock_and_fail();
    }

    if (!flogfs_inspect()) {
        return unlock_and_fail();
    }

    flog_prealloc_initialize();

    if (!flog_prealloc_prime()) {
        return unlock_and_fail();
    }

    flogfs.state = FLOG_STATE_MOUNTED;

    flash_unlock();
    flog_unlock_fs();
    return FLOG_SUCCESS;
}

flog_result_t flogfs_fsck() {
    flog_lock_fs();

    assert(flogfs.state == FLOG_STATE_MOUNTED);

    flash_lock();

    // TODO: Check for unclaimed blocks (how?)
    // TODO: Check for incomplete deletions. 

    flash_unlock();
    flog_unlock_fs();
    return FLOG_SUCCESS;
}

flog_result_t flogfs_open_read(flog_read_file_t *file, char const *filename) {
    flog_file_find_result_t find_result;
    flog_inode_iterator_t inode_iter;
    flog_file_sector_spare_t file_sector_spare;
    flog_read_file_t *file_iter;

    if (strlen(filename) >= FLOG_MAX_FNAME_LEN) {
        return FLOG_FAILURE;
    }

    flog_lock_fs();
    flash_lock();

    find_result = flog_find_file(filename, &inode_iter);
    if (find_result.first_block == FLOG_BLOCK_IDX_INVALID) {
        goto failure;
    }

    file->read_head = 0;
    file->first_block = find_result.first_block;
    file->block = file->first_block;
    file->id = find_result.file_id;

    if (!flogfs_read_calc_file_size(file)) {
        goto failure;
    }

    flog_open_sector(file->block, FLOG_INIT_SECTOR);
    flash_read_spare((uint8_t *)&file_sector_spare, FLOG_INIT_SECTOR);
    if (file_sector_spare.nbytes != 0) {
        file->sector = FLOG_INIT_SECTOR;
        file->offset = sizeof(flog_file_init_sector_header_t);
    } else {
        flog_open_sector(file->block, 1);
        flash_read_spare((uint8_t *)&file_sector_spare, 1);
        file->sector = flog_increment_sector(FLOG_INIT_SECTOR);
        file->offset = 0;
    }

    file->sector_remaining_bytes = file_sector_spare.nbytes;

    file->next = 0;
    if (flogfs.read_head) {
        for (file_iter = flogfs.read_head; file_iter->next; file_iter = file_iter->next) {
        }
        file_iter->next = file;
    } else {
        flogfs.read_head = file;
    }

    flash_unlock();
    flog_unlock_fs();
    return FLOG_SUCCESS;

failure:
    flash_unlock();
    flog_unlock_fs();
    return FLOG_FAILURE;
}

flog_result_t flogfs_close_read(flog_read_file_t *file) {
    flog_read_file_t *iter;

    flog_lock_fs();
    if (flogfs.read_head == file) {
        flogfs.read_head = file->next;
    } else {
        iter = flogfs.read_head;
        while (iter->next) {
            if (iter->next == file) {
                iter->next = file->next;
                break;
            }
            iter = iter->next;
        }
        flog_unlock_fs();
        return FLOG_FAILURE;
    }
    flog_unlock_fs();
    return FLOG_SUCCESS;
}

flog_result_t flogfs_check_exists(char const *filename) {
    flog_inode_iterator_t inode_iter;
    flog_file_find_result_t find_result;

    flog_lock_fs();
    flash_lock();

    find_result = flog_find_file(filename, &inode_iter);

    flash_unlock();
    flog_unlock_fs();

    return (find_result.first_block == FLOG_BLOCK_IDX_INVALID) ? FLOG_FAILURE : FLOG_SUCCESS;
}

uint32_t flogfs_read(flog_read_file_t *file, uint8_t *dst, uint32_t nbytes) {
    uint32_t count = 0;
    uint16_t to_read;
    flog_file_sector_spare_t file_sector_spare;
    flog_block_idx_t block;
    flog_sector_idx_t sector;

    union {
        flog_file_tail_sector_header_t file_tail_sector_header;
        flog_file_init_sector_header_t file_init_sector_header;
    } buffer_union;

    flog_lock_fs();
    flash_lock();

    while (nbytes) {
        if (file->sector_remaining_bytes == 0) {
            // We are/were at the end of file, look into the existence of new data
            // This block is responsible for setting:
            // -- file->sector_remaining_bytes
            // -- file->offset
            // -- file->sector
            // -- file->block
            // and bailing on the loop if EOF is encountered
            if (file->sector == FLOG_TAIL_SECTOR) {
                // This was the last sector in the block, check the next
                flog_open_sector(file->block, FLOG_TAIL_SECTOR);
                flash_read_sector((uint8_t *)&buffer_union.file_tail_sector_header, FLOG_TAIL_SECTOR, 0, sizeof(flog_file_tail_sector_header_t));
                block = buffer_union.file_tail_sector_header.universal.next_block;
                // Now check out that new block and make sure it's legit
                flog_open_sector(block, FLOG_INIT_SECTOR);
                flash_read_sector((uint8_t *)&buffer_union.file_init_sector_header, FLOG_INIT_SECTOR, 0, sizeof(flog_file_init_sector_header_t));
                if (buffer_union.file_init_sector_header.file_id != file->id) {
                    // This next block hasn't been written. EOF for now
                    goto done;
                }

                file->block = block;

                flash_read_spare((uint8_t *)&file_sector_spare, FLOG_INIT_SECTOR);
                if (file_sector_spare.nbytes == 0) {
                    // It's possible for the first sector to have 0 bytes
                    // Data is in next sector
                    file->sector = flog_increment_sector(FLOG_INIT_SECTOR);

                    /*Issue fixed: Read the spare bytes after incrementing the sector.
                     * This should be done, as during read,this sector may get missed out and entire sector data will not be read.
                     * Resulting in data getting lost.
                     */
                    flash_read_spare((uint8_t *)&file_sector_spare, file->sector);
                }
                else {
                    file->sector = FLOG_INIT_SECTOR;
                }
            } else {
                // Increment to next sector but don't necessarily update file state
                sector = flog_increment_sector(file->sector);

                flog_open_sector(file->block, sector);
                flash_read_spare((uint8_t *)&file_sector_spare, sector);

                if (invalid_sector_spare(&file_sector_spare)) {
                    // We're looking at an empty sector, GTFO
                    goto done;
                }
                else {
                    file->sector = sector;
                }
            }

            file->sector_remaining_bytes = file_sector_spare.nbytes;
            switch (file->sector) {
            case FLOG_TAIL_SECTOR:
                file->offset = sizeof(flog_file_tail_sector_header_t);
                break;
            case FLOG_INIT_SECTOR:
                file->offset = sizeof(flog_file_init_sector_header_t);
                break;
            default:
                file->offset = 0;
            }
        }

        // Figure out how many to read
        to_read = MIN(nbytes, file->sector_remaining_bytes);

        if (to_read) {
            // Read this sector now
            flog_open_sector(file->block, file->sector);
            flash_read_sector(dst, file->sector, file->offset, to_read);
            count += to_read;
            nbytes -= to_read;
            dst += to_read;
            // Update file stats
            file->offset += to_read;
            file->sector_remaining_bytes -= to_read;
            file->read_head += to_read;
        }
    }

done:
    flash_unlock();
    flog_unlock_fs();

    return count;
}

uint32_t flogfs_write(flog_write_file_t *file, uint8_t const *src, uint32_t nbytes) {
    uint32_t count = 0;
    flog_sector_nbytes_t bytes_written;

    flog_lock_fs();
    flash_lock();

    while (nbytes) {
        if (nbytes >= file->sector_remaining_bytes) {
            bytes_written = file->sector_remaining_bytes;
            if (flog_commit_file_sector(file, src, file->sector_remaining_bytes) == FLOG_FAILURE) {
                break;
            }

            src += bytes_written;
            nbytes -= bytes_written;
            count += bytes_written;
        } else {
            memcpy(file->sector_buffer + file->offset, src, nbytes);
            count += nbytes;
            file->sector_remaining_bytes -= nbytes;
            file->offset += nbytes;
            file->bytes_in_block += nbytes;
            file->file_size += nbytes;
            nbytes = 0;
        }
    }

    flash_unlock();
    flog_unlock_fs();

    return count;
}

uint32_t flogfs_read_file_size(flog_read_file_t *file) {
    return file->file_size;
}

uint32_t flogfs_write_file_size(flog_write_file_t *file) {
    return file->file_size;
}

typedef struct flogfs_walk_file_state_t {
    flog_block_idx_t block;
    flog_sector_idx_t sector;
    uint_fast8_t last_block;
    flog_file_tail_sector_header_t *tail_header;
    flog_file_sector_spare_t *sector_spare;
} flogfs_walk_file_state_t;

typedef enum {
    FLOG_WALK_FAILURE,
    FLOG_WALK_CONTINUE,
    FLOG_WALK_STOP,
    FLOG_WALK_SKIP_BLOCK,
} flog_read_walk_file_result_t;

typedef flog_read_walk_file_result_t (*file_walk_file_fn_t)(flogfs_walk_file_state_t *state, void *arg);

static flog_read_walk_file_result_t flogfs_read_walk_sectors(flog_read_file_t *file, flogfs_walk_file_state_t *state, file_walk_file_fn_t walk, void *arg) {
    flog_file_sector_spare_t file_sector_spare;

    state->sector = FLOG_INIT_SECTOR;
    state->sector_spare = &file_sector_spare;

    while (true) {
        flog_open_sector(state->block, state->sector);
        flash_read_spare((uint8_t *)&file_sector_spare, state->sector);
        if (invalid_sector_spare(&file_sector_spare)) {
            break;
        }

        switch (walk(state, arg)) {
        case FLOG_WALK_FAILURE: {
            return FLOG_WALK_FAILURE;
        }
        case FLOG_WALK_STOP: {
            return FLOG_WALK_STOP;
        }
        case FLOG_WALK_SKIP_BLOCK: {
            return FLOG_WALK_CONTINUE;
        }
        case FLOG_WALK_CONTINUE: {
            break;
        }
        default: {
            assert(false);
            break;
        }
        }

        if (state->sector == FLOG_TAIL_SECTOR) {
            break;
        }

        state->sector = flog_increment_sector(state->sector);
    }

    return FLOG_WALK_CONTINUE;
}

static flog_result_t flogfs_read_walk_file(flog_read_file_t *file, file_walk_file_fn_t walk, void *arg) {
    flog_file_tail_sector_header_t tail_header;
    flogfs_walk_file_state_t state;
    state.block = file->first_block;
    state.tail_header = &tail_header;

    while (1) {
        state.sector_spare = nullptr;

        flog_open_sector(state.block, FLOG_TAIL_SECTOR);
        flash_read_sector((uint8_t *)&tail_header, FLOG_TAIL_SECTOR, 0, sizeof(flog_file_tail_sector_header_t));
        state.last_block = invalid_file_tail_sector_header(&tail_header);

        switch (walk(&state, arg)) {
        case FLOG_WALK_FAILURE: {
            return FLOG_FAILURE;
        }
        case FLOG_WALK_STOP: {
            return FLOG_SUCCESS;
        }
        case FLOG_WALK_CONTINUE: {
            switch (flogfs_read_walk_sectors(file, &state, walk, arg)) {
            case FLOG_WALK_FAILURE: {
                return FLOG_FAILURE;
            }
            case FLOG_WALK_STOP: {
                return FLOG_SUCCESS;
            }
            case FLOG_WALK_CONTINUE: {
                break;
            }
            default: {
                assert(false);
                break;
            }
            }
            break;
        }
        case FLOG_WALK_SKIP_BLOCK: {
            break;
        }
        }

        if (state.last_block) {
            break;
        }

        if (state.block == tail_header.universal.next_block) {
            return FLOG_FAILURE;
        }
        state.block = tail_header.universal.next_block;
    }

    return FLOG_SUCCESS;
}

flog_read_walk_file_result_t file_size_calculator_walk(flogfs_walk_file_state_t *state, void *arg) {
    if (state->sector_spare != nullptr) {
        *((uint32_t *)arg) += state->sector_spare->nbytes;
    }
    else {
        if (!state->last_block) {
            *((uint32_t *)arg) += state->tail_header->bytes_in_block;
            return FLOG_WALK_SKIP_BLOCK;
        }
    }

    return FLOG_WALK_CONTINUE;
}

static flog_result_t flogfs_read_calc_file_size(flog_read_file_t *file) {
    file->file_size = 0;
    return flogfs_read_walk_file(file, file_size_calculator_walk, &file->file_size);
}

typedef struct file_seek_t {
    flog_result_t status;
    uint32_t position;
    uint32_t desired;
    flog_block_idx_t block;
    flog_sector_idx_t sector;
    uint16_t offset;
    uint16_t bytes_remaining;
} file_seek_t;

flog_read_walk_file_result_t file_seek_walk(flogfs_walk_file_state_t *state, void *arg) {
    file_seek_t *seek = (file_seek_t *)arg;

    seek->block = state->block;
    seek->sector = state->sector;

    if (state->sector_spare == nullptr) {
        if (state->last_block) {
            return FLOG_WALK_CONTINUE;
        }
        uint32_t end_of_block = seek->position + state->tail_header->bytes_in_block;
        if (seek->desired > end_of_block) {
            seek->position = end_of_block;
            return FLOG_WALK_SKIP_BLOCK;
        }
    }
    else {
        uint32_t end_of_sector = seek->position + state->sector_spare->nbytes;
        if (seek->desired > end_of_sector) {
            seek->position = end_of_sector;
            return FLOG_WALK_CONTINUE;
        }
        else {
            seek->status = FLOG_SUCCESS;
            seek->offset = (seek->desired - seek->position);
            seek->bytes_remaining = state->sector_spare->nbytes - seek->offset;
            return FLOG_WALK_STOP;
        }
    }

    return FLOG_WALK_CONTINUE;
}

flog_result_t flogfs_read_seek(flog_read_file_t *file, uint32_t position) {
    file_seek_t seek;
    flog_result_t fr = FLOG_SUCCESS;

    flog_lock_fs();

    if (flogfs.state != FLOG_STATE_MOUNTED) {
        flog_unlock_fs();
        return FLOG_FAILURE;
    }

    flash_lock();

    seek.position = 0;
    seek.desired = position;
    seek.block = file->first_block;
    seek.sector = FLOG_INIT_SECTOR;
    seek.status = FLOG_FAILURE;

    if (!flogfs_read_walk_file(file, file_seek_walk, &seek)) {
        fr = FLOG_FAILURE;
    }
    else {
        fr = seek.status;
        file->block = seek.block;
        file->sector = seek.sector;
        switch (seek.sector) {
        case FLOG_TAIL_SECTOR:
            file->offset = seek.offset+sizeof(flog_file_tail_sector_header_t);
            break;
        case FLOG_INIT_SECTOR:
            file->offset = seek.offset+sizeof(flog_file_init_sector_header_t);
            break;
        default:
            file->offset = seek.offset;
            break;
        }
        file->sector_remaining_bytes = seek.bytes_remaining;
    }

    flash_unlock();
    flog_unlock_fs();
    return fr;

}

uint32_t flogfs_read_tell(flog_read_file_t *file) {
  return file->read_head;
}

flog_result_t flogfs_open_write(flog_write_file_t *file, char const *filename) {
    flog_inode_iterator_t inode_iter;
    flog_block_alloc_t alloc_block;
    flog_file_find_result_t find_result;
    flog_file_sector_spare_t file_sector_spare;
    union {
        flog_inode_file_allocation_t allocation;
        flog_file_init_sector_header_t file_init_sector_header;
        flog_file_tail_sector_header_t file_tail_sector_header;
    } buffer_union;

    flog_lock_fs();

    if (flogfs.state != FLOG_STATE_MOUNTED) {
        flog_unlock_fs();
        return FLOG_FAILURE;
    }

    flash_lock();

    find_result = flog_find_file(filename, &inode_iter);

    file->base_threshold = 0;

    if (find_result.first_block != FLOG_BLOCK_IDX_INVALID) {
        // TODO: Make sure file isn't already open for writing

        // File already exists
        file->block = find_result.first_block;
        file->id = find_result.file_id;
        file->sector = FLOG_INIT_SECTOR;
        // Count bytes from 0
        file->file_size = 0;
        // Iterate to the end of the file
        // First check each terminated block
        while (1) {
            flog_open_sector(file->block, FLOG_TAIL_SECTOR);
            flash_read_sector((uint8_t *)&buffer_union.file_tail_sector_header, FLOG_TAIL_SECTOR, 0, sizeof(flog_file_tail_sector_header_t));
            if (invalid_file_tail_sector_header(&buffer_union.file_tail_sector_header)) {
                // This block is incomplete
                break;
            }
            file->block = buffer_union.file_tail_sector_header.universal.next_block;
            file->file_size += buffer_union.file_tail_sector_header.bytes_in_block;
        }
        // Now file->block is the first incomplete block
        // Scan it sector-by-sector
        while (1) {
            // For each block in the file
            flog_open_sector(file->block, file->sector);
            flash_read_spare((uint8_t *)&file_sector_spare, file->sector);
            if (invalid_sector_spare(&file_sector_spare)) {
                // No data
                // We will write here!
                if (file->sector == FLOG_TAIL_SECTOR) {
                    file->offset = sizeof(flog_file_tail_sector_header_t);
                } else {
                    file->offset = 0;
                }
                file->sector_remaining_bytes = FS_SECTOR_SIZE - file->offset;
                break;
            }
            file->file_size += file_sector_spare.nbytes;
            file->sector = flog_increment_sector(file->sector);
        }
    } else {
        if (flog_inode_prepare_new(&inode_iter) != FLOG_SUCCESS) {
            goto failure;
        }

        strcpy(buffer_union.allocation.filename, filename);
        buffer_union.allocation.filename[FLOG_MAX_FNAME_LEN - 1] = '\0';

        flog_lock_allocate();

        flog_flush_dirty_block();

        alloc_block = flog_allocate_block(file->base_threshold);
        if (alloc_block.block == FLOG_BLOCK_IDX_INVALID) {
            flog_unlock_allocate();
            goto failure;
        }

        flogfs.dirty_block.block = alloc_block.block;
        flogfs.dirty_block.file = file;

        flog_unlock_allocate();

        buffer_union.allocation.header.file_id = ++flogfs.max_file_id;
        buffer_union.allocation.header.first_block = alloc_block.block;
        buffer_union.allocation.header.first_block_age = ++alloc_block.age;
        buffer_union.allocation.header.timestamp = ++flogfs.t;

        // Write the new inode entry
        flog_open_sector(inode_iter.block, inode_iter.sector);
        flash_write_sector((uint8_t *)&buffer_union.allocation, inode_iter.sector, 0, sizeof(flog_inode_file_allocation_t));
        flash_commit();

        file->block = alloc_block.block;
        file->block_age = alloc_block.age;
        file->id = flogfs.max_file_id;
        file->bytes_in_block = 0;
        file->file_size = 0;
        file->sector = FLOG_INIT_SECTOR;
        file->offset = sizeof(flog_file_init_sector_header_t);
        file->sector_remaining_bytes = FS_SECTOR_SIZE - sizeof(flog_file_init_sector_header_t);
    }

    file->next = NULL;
    if (flogfs.write_head == NULL) {
        flogfs.write_head = file;
    } else {
        flog_write_file_t *file_iter;
        for (file_iter = flogfs.write_head; file_iter->next; file_iter = file_iter->next) {
        }
        file_iter->next = file;
    }

    flash_unlock();
    flog_unlock_fs();

    return FLOG_SUCCESS;

failure:
    flash_unlock();
    flog_unlock_fs();

    return FLOG_FAILURE;
}

/*!
 @details
 ### Internals
 To close a write, all outstanding data must simply be flushed to flash. If any
 blocks are newly-allocated, they must be committed.

 TODO: Deal with files that can't be flushed due to no space for allocation
 */
flog_result_t flogfs_close_write(flog_write_file_t *file) {
    flog_write_file_t *iter;
    flog_result_t result;

    flog_lock_fs();
    flash_lock();

    if (flogfs.write_head == file) {
        flogfs.write_head = file->next;
    } else {
        iter = flogfs.write_head;
        while (iter->next) {
            if (iter->next == file) {
                iter->next = file->next;
                break;
            }
            iter = iter->next;
        }
        goto failure;
    }

    result = flog_flush_write(file);

    flash_unlock();
    flog_unlock_fs();

    return result;

failure:
    flash_unlock();
    flog_unlock_fs();
    return FLOG_FAILURE;
}

flog_result_t flogfs_rm(char const *filename) {
    flog_file_find_result_t find_result;
    flog_inode_iterator_t inode_iter;
    flog_block_idx_t block, next_block;
    flog_inode_file_invalidation_t invalidation;

    flog_lock_fs();
    flash_lock();

    find_result = flog_find_file(filename, &inode_iter);
    if (find_result.first_block == FLOG_BLOCK_IDX_INVALID) {
        goto failure;
    }

    // Navigate to the end to find the last block
    block = find_result.first_block;
    while (1) {
        next_block = flog_universal_get_next_block(block);
        if (next_block == FLOG_BLOCK_IDX_INVALID) {
            break;
        }
        block = next_block;
    }

    invalidation.header.last_block = block;
    invalidation.header.timestamp = ++flogfs.t;
    flog_open_sector(inode_iter.block, inode_iter.sector + 1);
    flash_write_sector((uint8_t *)&invalidation, inode_iter.sector + 1, 0, sizeof(flog_inode_file_invalidation_t));
    flash_commit();

    flog_invalidate_chain(find_result.first_block, find_result.file_id);

    flash_unlock();
    flog_unlock_fs();
    return FLOG_SUCCESS;

failure:
    flash_unlock();
    flog_unlock_fs();
    return FLOG_FAILURE;
}

void flogfs_start_ls(flogfs_ls_iterator_t *iter) {
    flog_inode_iterator_initialize(iter, flogfs.inode0);
}

uint_fast8_t flogfs_ls_iterate(flogfs_ls_iterator_t *iter, char *fname_dst) {
    union {
        flog_inode_file_allocation_header_t allocation;
        flog_inode_file_invalidation_t invalidation;
    } buffer_union;

    while (1) {
        flog_open_sector(iter->block, iter->sector);
        flash_read_sector((uint8_t *)&buffer_union.allocation, iter->sector, 0, sizeof(flog_inode_file_allocation_header_t));
        if (invalid_file_id(buffer_union.allocation.file_id)) {
            // Nothing here. Done.
            return 0;
        }
        // Now check to see if it's valid
        flog_open_sector(iter->block, iter->sector + 1);
        flash_read_sector((uint8_t *)&buffer_union.invalidation, iter->sector + 1, 0, sizeof(flog_inode_file_invalidation_t));
        if (invalid_timestamp(buffer_union.invalidation.header.timestamp)) {
            // This file's good
            // Now check to see if it's valid
            // Go read the filename
            flog_open_sector(iter->block, iter->sector);
            flash_read_sector((uint8_t *)fname_dst, iter->sector, sizeof(flog_inode_file_allocation_header_t), FLOG_MAX_FNAME_LEN);
            fname_dst[FLOG_MAX_FNAME_LEN - 1] = '\0';
            flog_inode_iterator_next(iter);
            return 1;
        } else {
            flog_inode_iterator_next(iter);
        }
    }
}

void flogfs_stop_ls(flogfs_ls_iterator_t *iter) {
}

flog_result_t flog_commit_file_sector(flog_write_file_t *file, uint8_t const *data, flog_sector_nbytes_t n) {
    flog_file_sector_spare_t file_sector_spare;

    if (file->sector == FLOG_TAIL_SECTOR) {
        flog_file_tail_sector_header_t *const file_tail_sector_header = (flog_file_tail_sector_header_t *)file->sector_buffer;
        flog_block_alloc_t next_block;

        flog_lock_allocate();

        flog_flush_dirty_block();

        next_block = flog_allocate_block(file->base_threshold);
        if (next_block.block == FLOG_BLOCK_IDX_INVALID) {
            flog_unlock_allocate();
            return FLOG_FAILURE;
        }

        flogfs.dirty_block.block = next_block.block;
        flogfs.dirty_block.file = file;

        flog_unlock_allocate();

       // uint16_t bytes_in_sector = FS_SECTOR_SIZE - sizeof(flog_file_tail_sector_header_t);
	   /*Issue fixed: In tail Sector, by default the no of data bytes was always hardcorded to FS_SECTOR_SIZE - sizeof(flog_file_tail_sector_header_t)
	   	* Because of which when reading back the file , if the tail sector's data portion is not fully filled chunk of 0xFF s will
		* be added to the file, thereby corrupting the file. Hence modifying the implementation to store only the written bytes to spare bytes
	    */
        uint16_t bytes_in_sector = file->offset +n;

        // Prepare the header
        // memzero(file_tail_sector_header, sizeof(flog_file_tail_sector_header_t)); // Optional
        file->bytes_in_block += n; // Not bytes_in_sector, this is updated on in memory/non-flushed writes.
		//file_sector_spare.nbytes = bytes_in_sector;
        file_sector_spare.nbytes = bytes_in_sector - sizeof(flog_file_tail_sector_header_t);

        file_tail_sector_header->universal.next_age = next_block.age + 1;
        file_tail_sector_header->universal.next_block = next_block.block;
        file_tail_sector_header->universal.timestamp = ++flogfs.t;
        file_tail_sector_header->bytes_in_block = file->bytes_in_block;

        flog_open_sector(file->block, FLOG_TAIL_SECTOR);
        flash_write_sector((uint8_t const *)file_tail_sector_header, FLOG_TAIL_SECTOR, 0, file->offset);
        if (n) {
            flash_write_sector(data, FLOG_TAIL_SECTOR, file->offset, n);
        }
        flash_write_spare((uint8_t const *)&file_sector_spare, FLOG_TAIL_SECTOR);
        flash_commit();

        // Ready the file structure for the next block/sector
        file->block = next_block.block;
        file->block_age = next_block.age;
        file->sector = FLOG_INIT_SECTOR;
        file->sector_remaining_bytes = FS_SECTOR_SIZE - sizeof(flog_file_init_sector_header_t);
        file->offset = sizeof(flog_file_init_sector_header_t);
        file->bytes_in_block = 0;
        file->file_size += n;

        return FLOG_SUCCESS;
    } else {
        flog_file_init_sector_header_t *const file_init_sector_header = (flog_file_init_sector_header_t *)file->sector_buffer;

        uint16_t bytes_in_sector = file->offset + n;

        flog_lock_allocate();
        // So if this block is the dirty block...
        if (flogfs.dirty_block.file == file) {
            flogfs.dirty_block.block = FLOG_BLOCK_IDX_INVALID;
            flogfs.dirty_block.file = nullptr;
        }
        flog_unlock_allocate();
        file_sector_spare.type_id = FLOG_BLOCK_TYPE_FILE;
        file_sector_spare.nbytes = bytes_in_sector;

        // We need to just write the data and advance
        if (file->sector == FLOG_INIT_SECTOR) {
            // memzero(file_init_sector_header, sizeof(flog_file_init_sector_header_t)); // Optional
            file_init_sector_header->file_id = file->id;
            file_init_sector_header->age = file->block_age;
            file_sector_spare.nbytes -= sizeof(flog_file_init_sector_header_t); // file->offset had accounted for this.
        }

        flog_open_sector(file->block, file->sector);
        if (file->offset) {
            // This is either sector 0 or there was data already
            // First write prior data/header
            flash_write_sector(file->sector_buffer, file->sector, 0, file->offset);
        }
        if (n) {
            flash_write_sector(data, file->sector, file->offset, n);
        }

        flash_write_spare((uint8_t const *)&file_sector_spare, file->sector);
        flash_commit();

        // Now update stuff for the new sector
        file->sector = flog_increment_sector(file->sector);
        if (file->sector == FLOG_TAIL_SECTOR) {
            file->offset = sizeof(flog_file_tail_sector_header_t);
        } else {
            file->offset = 0;
        }
        file->bytes_in_block += n;
        file->sector_remaining_bytes = FS_SECTOR_SIZE - file->offset;
        file->file_size += n;
        return FLOG_SUCCESS;
    }
}

flog_result_t flog_walk(file_walk_fn_t walk_fn, void *arg) {
    flog_block_statistics_sector_with_key_t block_sector;
    flog_inode_init_sector_spare_t inode_spare;

    union {
        flog_file_init_sector_header_t file;
    } init_sector;

    union {
        flog_universal_tail_sector_t universal;
        flog_file_tail_sector_header_t file;
    } tail_sector;

    flogfs_walk_state_t state;

    flog_lock_fs();
    flash_lock();

    for (state.block = 0; state.block < flogfs.params.number_of_blocks; ++state.block) {
        flash_open_page(state.block, 0);

        flog_block_statistics_read(state.block, &block_sector);

        flash_read_spare((uint8_t *)&inode_spare, FLOG_INIT_SECTOR);

        state.valid_block = !invalid_block_or_older_version(&block_sector);

        if (state.valid_block) {
            flog_open_sector(state.block, FLOG_INIT_SECTOR);
            flash_read_sector((uint8_t *)&init_sector, FLOG_INIT_SECTOR, 0, sizeof(init_sector));
            flog_close_sector();

            flog_open_sector(state.block, FLOG_TAIL_SECTOR);
            flash_read_sector((uint8_t *)&tail_sector, FLOG_TAIL_SECTOR, 0, sizeof(tail_sector));
            flog_close_sector();

            state.next_block = tail_sector.universal.next_block;
            state.age = block_sector.header.age;
            state.type_id = inode_spare.type_id;
            state.file_id = init_sector.file.file_id;
            state.bytes_in_block = tail_sector.file.bytes_in_block;

            walk_fn(&state, arg);

            switch (state.type_id) {
            case FLOG_BLOCK_TYPE_INODE: {
                flogfs_walk_inode_block_state_t *inode = &state.types.inode;
                flog_inode_file_allocation_t allocation;
                flog_inode_file_invalidation_t invalidation;

                inode->sector = FLOG_INODE_FIRST_ENTRY_SECTOR;

                while (inode->sector < flogfs.params.pages_per_block * FS_SECTORS_PER_PAGE) {
                    inode->valid = false;
                    inode->file_name[0] = 0;
                    inode->file_id = FLOG_FILE_ID_INVALID;
                    inode->first_block = FLOG_BLOCK_IDX_INVALID;
                    inode->last_block = FLOG_BLOCK_IDX_INVALID;
                    inode->created_at = FLOG_TIMESTAMP_INVALID;
                    inode->deleted_at = FLOG_TIMESTAMP_INVALID;

                    flog_open_sector(state.block, inode->sector);
                    flash_read_sector((uint8_t *)&allocation, inode->sector, 0, sizeof(flog_inode_file_allocation_t));
                    flog_close_sector();

                    state.types.inode.valid = !invalid_inode_file_allocation_header(&allocation.header);
                    if (inode->valid) {
                        flog_open_sector(state.block, inode->sector + 1);
                        flash_read_sector((uint8_t *)&invalidation, inode->sector + 1, 0, sizeof(flog_inode_file_invalidation_t));
                        flog_close_sector();

                        strcpy(inode->file_name, allocation.filename);
                        inode->file_id = allocation.header.file_id;
                        inode->first_block = allocation.header.first_block;
                        inode->created_at = allocation.header.timestamp;

                        inode->deleted = !invalid_inode_file_invalidation(&invalidation);
                        inode->last_block = invalidation.header.last_block;
                        inode->deleted_at = invalidation.header.timestamp;
                    }

                    walk_fn(&state, arg);

                    inode->sector += 2;
                }
                break;
            }
            case FLOG_BLOCK_TYPE_FILE: {
                flogfs_walk_file_block_state_t *fb = &state.types.file;
                flog_file_sector_spare_t file_sector_spare;

                fb->sector = FLOG_INIT_SECTOR;

                while (true) {
                    flog_open_sector(state.block, fb->sector);
                    flash_read_spare((uint8_t *)&file_sector_spare, fb->sector);
                    flog_close_sector();

                    fb->valid = !invalid_sector_spare(&file_sector_spare);
                    fb->file_id = init_sector.file.file_id;
                    fb->size = file_sector_spare.nbytes;

                    walk_fn(&state, arg);

                    if (fb->sector == FLOG_TAIL_SECTOR) {
                        break;
                    }

                    fb->sector = flog_increment_sector(fb->sector);
                }
                break;
            }
            }
        }
        else {
            walk_fn(&state, arg);
        }
    }

    flog_unlock_fs();
    flash_unlock();

    return FLOG_SUCCESS;
}

uint32_t flogfs_available_space(){
     uint16_t i=0;
     flog_inode_init_sector_spare_t inode_spare;
     uint16_t no_of_free_blocks = 0;


     for(i=0;i<FS_NUM_BLOCKS;i++)
     {
         flash_open_page(i, 0);
         flash_read_spare((uint8_t *)&inode_spare, FLOG_INIT_SECTOR);
         /* Check if the block is occupied */
         if(inode_spare.type_id == 0xFF){
             no_of_free_blocks++;
         }

     }

    return (no_of_free_blocks*FS_PAGES_PER_BLOCK*FS_SECTORS_PER_PAGE*FS_SECTOR_SIZE);
}

/**********************************start************************************************/
/* Note: This is done to ensure that time taken by delete file is reduced, hence deletion of block is separated*/
flog_result_t flogfs_invalidate(char const *filename) {
    flog_file_find_result_t find_result;
    flog_inode_iterator_t inode_iter;
    flog_block_idx_t block, next_block;
    flog_inode_file_invalidation_t invalidation;

    flog_lock_fs();
    flash_lock();

    find_result = flog_find_file(filename, &inode_iter);
    if (find_result.first_block == FLOG_BLOCK_IDX_INVALID) {
        goto failure;
    }

    // Navigate to the end to find the last block
    block = find_result.first_block;
    while (1) {
        next_block = flog_universal_get_next_block(block);
        if (next_block == FLOG_BLOCK_IDX_INVALID) {
            break;
        }
        block = next_block;
    }

    invalidation.header.last_block = block;
    invalidation.header.timestamp = ++flogfs.t;
    flog_open_sector(inode_iter.block, inode_iter.sector + 1);
    flash_write_sector((uint8_t *)&invalidation, inode_iter.sector + 1, 0, sizeof(flog_inode_file_invalidation_t));
    flash_commit();

    flog_invalidate_file_block(find_result.first_block, find_result.file_id);

    flash_unlock();
    flog_unlock_fs();
    return FLOG_SUCCESS;

failure:
    flash_unlock();
    flog_unlock_fs();
    return FLOG_FAILURE;
}

void flog_delete_invalidated_block(void)
{
    uint16_t index = 0;
    for(index = 0; index < flog_invalidated_block.invalid_block_count; index++)
    {
        flash_erase_block(flog_invalidated_block.block_idx[index]);
    }
    flog_invalidated_block.invalid_block_count = 0;
}


static void flog_invalidate_file_block(flog_block_idx_t block, flog_file_id_t file_id) {
    flog_file_tail_sector_header_t file_tail_sector;
    flog_block_statistics_sector_with_key_t block_statistics;
    flog_block_idx_t num_freed = 0;

    flog_invalidated_block.invalid_block_count = 0;

    union {
        flog_file_init_sector_header_t init_sector;
    } init_buffer_union;

    flog_lock_delete();

    while (block != FLOG_BLOCK_IDX_INVALID) {
        switch (flog_get_block_type(block)) {
        case FLOG_BLOCK_TYPE_UNALLOCATED: {
            flog_block_statistics_read(block, &block_statistics);
            block = FLOG_BLOCK_IDX_INVALID;
            break;
        }
        case FLOG_BLOCK_TYPE_FILE: {
            flog_open_sector(block, FLOG_INIT_SECTOR);
            flash_read_sector((uint8_t *)&init_buffer_union.init_sector, FLOG_INIT_SECTOR, 0, sizeof(flog_file_init_sector_header_t));
            if (is_file_init_sector_header_for_file(&init_buffer_union.init_sector, file_id)) {
                flog_open_sector(block, FLOG_TAIL_SECTOR);
                flash_read_sector((uint8_t *)&file_tail_sector, FLOG_TAIL_SECTOR, 0, sizeof(flog_file_tail_sector_header_t));
                block_statistics.header.age = init_buffer_union.init_sector.age;
                block_statistics.header.next_block = file_tail_sector.universal.next_block;
                block_statistics.header.next_age = file_tail_sector.universal.next_age;
                block_statistics.header.timestamp = ++flogfs.t;
                block_statistics.header.version = flogfs.version;
                flog_close_sector();

                /****************************************************************************/
                /* Note: This done as part of bugfix, deletion of block is separated.
                Invalidation of File is only done here.
                Invalidated block and index count is updated */
                //flash_erase_block(block);
                flog_invalidated_block.block_idx[flog_invalidated_block.invalid_block_count] = block;
                flog_invalidated_block.invalid_block_count++;
                /****************************************************************************/
                // TODO: Add these to prealloc?

                flog_block_statistics_write(block, &block_statistics);

                num_freed += 1;

                block = block_statistics.header.next_block;
            }
            else {
                assert(false);
            }
            break;
        }
        default: {
            assert(0);
            break;
        }
        }
    }

    flog_unlock_delete();
}
/***************************************End***********************************************/

static flog_result_t flog_flush_write(flog_write_file_t *file) {
    flog_result_t fr = flog_commit_file_sector(file, 0, 0);
    if (!fr) {
        return FLOG_FAILURE;
    }

    // The above commit may have allocated a new block, leaving that block dirty
    // so we flush again in that situation.
    if (file == flogfs.dirty_block.file) {
        fr = flog_commit_file_sector(file, 0, 0);
        assert(flogfs.dirty_block.file == nullptr);
        assert(flogfs.dirty_block.block == FLOG_BLOCK_IDX_INVALID);
    }

    return fr;
}

static uint_fast8_t flog_prealloc_has_block(flog_block_alloc_t *list, flog_block_idx_t block_number) {
    for (flog_block_alloc_t *iter = list; iter != NULL; iter = iter->next) {
        if (iter->block == block_number) {
            return true;
        }
    }
    return false;
}

static uint_fast8_t flog_prealloc_contains(flog_block_idx_t block) {
    return flog_prealloc_has_block(flogfs.prealloc.free, block) ||
           flog_prealloc_has_block(flogfs.prealloc.pending, block) ||
           flog_prealloc_has_block(flogfs.prealloc.available, block);
}

static flog_block_alloc_t *flog_prealloc_block_prepend(flog_block_alloc_t *list, flog_block_alloc_t *entry) {
    entry->next = list;
    return entry;
}

static flog_block_alloc_t *flog_prealloc_block_append(flog_block_alloc_t *list, flog_block_alloc_t *entry) {
    entry->next = NULL;

    assert(list != entry);

    if (list == NULL) {
        return entry;
    }
    else {
        for (flog_block_alloc_t *iter = list; iter != NULL; ) {
            if (iter->next == NULL) {
                iter->next = entry;
                break;
            }
            else {
                iter = iter->next;
            }

        }
    }
    return list;
}

static void flog_prealloc_block_remove_pending(flog_block_idx_t block_number) {
    flog_block_alloc_t *last = NULL;

    for (flog_block_alloc_t *iter = flogfs.prealloc.pending; iter != NULL; iter = iter->next) {
        if (iter->block == block_number) {
            flash_debug_warn("Pending->Free: block=%d", block_number);

            iter->block = FLOG_BLOCK_IDX_INVALID;
            iter->age = FLOG_BLOCK_AGE_INVALID;

            if (last == NULL) {
                flogfs.prealloc.pending = iter->next;
            }
            else {
                last->next = iter->next;
            }

            flogfs.prealloc.free = flog_prealloc_block_prepend(flogfs.prealloc.free, iter);
            break;
        }

        last = iter;
    }
}

static void flog_prealloc_push(flog_block_idx_t block, flog_block_age_t age) {
    flog_block_alloc_t *entry;

    assert(!flog_prealloc_contains(block));
    assert(flogfs.prealloc.free != NULL);

    entry = flogfs.prealloc.free;
    flogfs.prealloc.free = flogfs.prealloc.free->next;

    entry->next = NULL;
    entry->block = block;
    entry->age = age;

    flogfs.prealloc.age_sum += age;
    flogfs.prealloc.n++;

    flogfs.prealloc.available = flog_prealloc_block_append(flogfs.prealloc.available, entry);

    flash_debug_warn("Free->Available: block=%d size=%d", entry->block, flogfs.prealloc.n);
}

static flog_block_alloc_t flog_prealloc_pop(int32_t threshold) {
    flog_block_alloc_t *entry;

    flash_debug_warn("Pop");

    assert(flogfs.prealloc.available != NULL);

    entry = flogfs.prealloc.available;
    flogfs.prealloc.available = entry->next;

    flogfs.prealloc.n -= 1;
    flogfs.prealloc.pending = flog_prealloc_block_append(flogfs.prealloc.pending, entry);
    flash_debug_warn("Available->Pending: block=%d", entry->block);

    return *entry;
}

static flog_result_t flog_open_page(flog_block_idx_t block, flog_page_index_t page) {
    if (flogfs.cache_status.page_open && (flogfs.cache_status.current_open_block == block) &&
        (flogfs.cache_status.current_open_page == page)) {
        return flogfs.cache_status.page_open_result;
    }
    flogfs.cache_status.page_open_result = flash_open_page(block, page);
    flogfs.cache_status.page_open = 1;
    flogfs.cache_status.current_open_block = block;
    flogfs.cache_status.current_open_page = page;

    flog_prealloc_block_remove_pending(block);

    return flogfs.cache_status.page_open_result;
}

static flog_result_t flog_open_sector(flog_block_idx_t block, flog_sector_idx_t sector) {
    return flog_open_page(block, sector / FS_SECTORS_PER_PAGE);
}

static void flog_close_sector() {
    flogfs.cache_status.page_open = 0;
}

static flog_block_idx_t flog_universal_get_next_block(flog_block_idx_t block) {
    if (block == FLOG_BLOCK_IDX_INVALID) {
        return block;
    }
    flog_open_sector(block, FLOG_TAIL_SECTOR);
    flash_read_sector((uint8_t *)&block, FLOG_TAIL_SECTOR, 0, sizeof(block));
    if (invalid_block_index(block)) {
        return FLOG_BLOCK_IDX_INVALID;
    }
    return block;
}

static void flog_inode_iterator_initialize(flog_inode_iterator_t *iter, flog_block_idx_t inode0) {
    flog_inode_init_sector_spare_t inode_init_sector_spare;
    flog_universal_tail_sector_t tail_sector;

    iter->block = inode0;
    flog_open_sector(inode0, FLOG_TAIL_SECTOR);
    flash_read_sector((uint8_t *)&tail_sector, FLOG_TAIL_SECTOR, 0, sizeof(flog_universal_tail_sector_t));
    iter->next_block = tail_sector.next_block;

    // Get the current inode block index
    flog_open_sector(inode0, FLOG_INIT_SECTOR);
    flash_read_spare((uint8_t *)&inode_init_sector_spare, FLOG_INIT_SECTOR);
    iter->inode_block_idx = inode_init_sector_spare.inode_index;

    // This is zero anyways
    iter->inode_idx = 0;
    iter->sector = FLOG_INODE_FIRST_ENTRY_SECTOR;
    // TODO: Coerce iter->next_block?
}

/*!
 @details
 ### Internals
 Inode entries are organized sequentially in pairs of sectors following the
 first page. The first page contains simple header information. To iterate to
 the next entry, we simply advance by two sectors. If this goes past the end of
 the block, the next block is checked. If the next block hasn't yet been
 allocated, the sector is set to invalid to indicate that the next block has to
 be allocated using flog_inode_prepare_new().
 */
static void flog_inode_iterator_next(flog_inode_iterator_t *iter) {
    iter->sector += 2;
    iter->inode_idx += 1;
    if (iter->sector >= flogfs.params.pages_per_block * FS_SECTORS_PER_PAGE) {
        // The next sector is in ANOTHER BLOCK!!!
        if (iter->next_block != FLOG_BLOCK_IDX_INVALID) {
            // The next block actually already exists
            iter->block = iter->next_block;
            // Check the next block
            iter->next_block = flog_universal_get_next_block(iter->block);

            // Point to the first inode sector of the next block
            iter->sector = FLOG_INODE_FIRST_ENTRY_SECTOR;
        }
        else {
            // Don't do anything; this is dumb
            // flash_debug_warn("FLogFS:" LINESTR);
            // iter->sector -= 2;
            // iter->inode_idx -= 1;
            assert(false);
        }
    }
}

static flog_result_t flog_inode_prepare_new(flog_inode_iterator_t *iter) {
    flog_block_alloc_t block_alloc;

    union {
        flog_universal_tail_sector_t inode_tail_sector;
        flog_inode_init_sector_t inode_init_sector;
        flog_inode_init_sector_spare_t inode_init_sector_spare;
    } buffer_union;

    if (iter->sector == (flogfs.params.pages_per_block * FS_SECTORS_PER_PAGE) - 2) {
        // assert(iter->next_block != FLOG_BLOCK_IDX_INVALID);

        flog_lock_allocate();

        flog_flush_dirty_block();

        block_alloc = flog_allocate_block(0);
        if (block_alloc.block == FLOG_BLOCK_IDX_INVALID) {
            flog_unlock_allocate();
            return FLOG_FAILURE;
        }

        flog_unlock_allocate();

        flog_open_sector(iter->block, FLOG_TAIL_SECTOR);
        buffer_union.inode_tail_sector.next_age = block_alloc.age + 1;
        buffer_union.inode_tail_sector.next_block = block_alloc.block;
        buffer_union.inode_tail_sector.timestamp = ++flogfs.t;
        flash_write_sector((uint8_t *)&buffer_union.inode_tail_sector, FLOG_TAIL_SECTOR, 0, sizeof(flog_universal_tail_sector_t));
        flash_commit();

        flog_open_sector(block_alloc.block, FLOG_INIT_SECTOR);
        buffer_union.inode_init_sector.universal.timestamp = flogfs.t;
        flash_write_sector((uint8_t *)&buffer_union.inode_init_sector, FLOG_INIT_SECTOR, 0, sizeof(flog_inode_init_sector_t));

        buffer_union.inode_init_sector_spare.type_id = FLOG_BLOCK_TYPE_INODE;
        buffer_union.inode_init_sector_spare.nothing = 0;
        buffer_union.inode_init_sector_spare.inode_index = ++iter->inode_block_idx;
        flash_write_spare((uint8_t *)&buffer_union.inode_init_sector_spare, FLOG_INIT_SECTOR);
        flash_commit();

        iter->next_block = block_alloc.block;
    }

    return FLOG_SUCCESS;
}

static void flog_block_statistics_write(flog_block_idx_t block, flog_block_statistics_sector_with_key_t const *sector) {
    assert(sector->header.version != 0);

    flog_open_sector(block, FLOG_BLOCK_STATISTICS_SECTOR);
    flash_write_sector((uint8_t const *)sector, FLOG_BLOCK_STATISTICS_SECTOR, 0, sizeof(flog_block_statistics_sector_with_key_t));
    flash_commit();
}

static void flog_block_statistics_read(flog_block_idx_t block, flog_block_statistics_sector_with_key_t *sector) {
    flog_open_sector(block, FLOG_BLOCK_STATISTICS_SECTOR);
    flash_read_sector((uint8_t *)sector, FLOG_BLOCK_STATISTICS_SECTOR, 0, sizeof(flog_block_statistics_sector_with_key_t));
}

static void flog_invalidate_chain(flog_block_idx_t block, flog_file_id_t file_id) {
    flog_file_tail_sector_header_t file_tail_sector;
    flog_block_statistics_sector_with_key_t block_statistics;
    flog_block_idx_t num_freed = 0;

    union {
        flog_file_init_sector_header_t init_sector;
    } init_buffer_union;

    flog_lock_delete();

    while (block != FLOG_BLOCK_IDX_INVALID) {
        switch (flog_get_block_type(block)) {
        case FLOG_BLOCK_TYPE_UNALLOCATED: {
            flog_block_statistics_read(block, &block_statistics);
            block = FLOG_BLOCK_IDX_INVALID;
            break;
        }
        case FLOG_BLOCK_TYPE_FILE: {
            flog_open_sector(block, FLOG_INIT_SECTOR);
            flash_read_sector((uint8_t *)&init_buffer_union.init_sector, FLOG_INIT_SECTOR, 0, sizeof(flog_file_init_sector_header_t));
            if (is_file_init_sector_header_for_file(&init_buffer_union.init_sector, file_id)) {
                flog_open_sector(block, FLOG_TAIL_SECTOR);
                flash_read_sector((uint8_t *)&file_tail_sector, FLOG_TAIL_SECTOR, 0, sizeof(flog_file_tail_sector_header_t));
                block_statistics.header.age = init_buffer_union.init_sector.age;
                block_statistics.header.next_block = file_tail_sector.universal.next_block;
                block_statistics.header.next_age = file_tail_sector.universal.next_age;
                block_statistics.header.timestamp = ++flogfs.t;
                block_statistics.header.version = flogfs.version;
                flog_close_sector();

                flash_erase_block(block);

                // TODO: Add these to prealloc?

                flog_block_statistics_write(block, &block_statistics);

                num_freed += 1;

                block = block_statistics.header.next_block;
            }
            else {
                assert(false);
            }
            break;
        }
        default: {
            assert(0);
            break;
        }
        }
    }

    flog_unlock_delete();
}

static flog_block_type_t flog_get_block_type(flog_block_idx_t block) {
    flog_file_sector_spare_t spare;

    if (flog_open_sector(block, FLOG_INIT_SECTOR) != FLOG_SUCCESS) {
        return FLOG_BLOCK_TYPE_ERROR;
    }
    flash_read_spare((uint8_t *)&spare, FLOG_INIT_SECTOR);

    return (flog_block_type_t)spare.type_id;
}

static uint_fast8_t flog_prealloc_is_empty() {
    return flogfs.prealloc.n == 0;
}

static uint_fast8_t flog_prealloc_is_full() {
    return flogfs.prealloc.free == NULL;
}

static flog_block_alloc_t flog_allocate_block(int32_t threshold) {
    flog_block_alloc_t block;

    if (flog_prealloc_is_empty()) {
        flash_debug_error("flog_allocate_block: flog_prealloc_is_empty");
        block.block = FLOG_BLOCK_IDX_INVALID;
        return block;
    }

    for (flog_block_idx_t i = flogfs.params.number_of_blocks; i; i--) {
        block = flog_prealloc_pop(threshold);
        if (flog_prealloc_is_empty()) {
            flog_prealloc_prime();
        }
        if (block.block != FLOG_BLOCK_IDX_INVALID) {
            return block;
        }

        threshold -= 1;
    }

    return block;
}

static flog_sector_idx_t flog_increment_sector(flog_sector_idx_t sector) {
    flog_sector_idx_t last_sector = (flogfs.params.pages_per_block * FS_SECTORS_PER_PAGE) - 1;
    if (sector == FLOG_TAIL_SECTOR - 1) {
        return FS_SECTORS_PER_PAGE;
    }
    if (sector == last_sector) {
        return FLOG_TAIL_SECTOR;
    }
    return sector + 1;
}

static flog_file_find_result_t flog_find_file(char const *filename, flog_inode_iterator_t *iter) {
    union {
        flog_inode_file_allocation_t allocation;
        flog_inode_file_invalidation_t invalidation;
    } buffer_union;

    flog_file_find_result_t found;

    for (flog_inode_iterator_initialize(iter, flogfs.inode0); ; flog_inode_iterator_next(iter)) {
        flog_open_sector(iter->block, iter->sector);
        flash_read_sector((uint8_t *)&buffer_union.allocation, iter->sector, 0, sizeof(flog_inode_file_allocation_t));

        if (invalid_inode_file_allocation_header(&buffer_union.allocation.header)) {
            found.first_block = FLOG_BLOCK_IDX_INVALID;
            return found;
        }

        if (strncmp(filename, buffer_union.allocation.filename, FLOG_MAX_FNAME_LEN) != 0) {
            continue;
        }

        found.first_block = buffer_union.allocation.header.first_block;
        found.file_id = buffer_union.allocation.header.file_id;

        flog_open_sector(iter->block, iter->sector + 1);
        flash_read_sector((uint8_t *)&buffer_union.invalidation, iter->sector + 1, 0, sizeof(flog_timestamp_t));

        if (!invalid_inode_file_invalidation(&buffer_union.invalidation)) {
            continue;
        }

        return found;
    }
}

static void flog_flush_dirty_block() {
    if (flogfs.dirty_block.block != FLOG_BLOCK_IDX_INVALID) {
        flog_flush_write(flogfs.dirty_block.file);
        flogfs.dirty_block.block = FLOG_BLOCK_IDX_INVALID;
    }
}

#ifndef IS_DOXYGEN
#if !FLOG_BUILD_CPP
#ifdef __cplusplus
}
#endif
#endif
#endif
