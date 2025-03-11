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
 * @file flogfs_conf.h
 * @author Ben Nahill <bnahill@gmail.com>
 * @ingroup FLogFS
 *
 * @brief Platform-specific interface details
 */

#ifndef __FLOGFS_CONF_H_
#define __FLOGFS_CONF_H_

#include "flogfs.h"

#include <stdbool.h>




//! @addtogroup FLogConf
//! @{

//! @name Flash module parameters
//! @{
#define FS_SECTOR_SIZE       (512)
#define FS_SECTORS_PER_PAGE  (4)
#define FS_PAGES_PER_BLOCK   (64)

#define W25N02_FS_NUM_BLOCKS        (2008)
#define W25N01_FS_NUM_BLOCKS        (1004)

#if defined(MCU_HEAR3X)
#define FS_NUM_BLOCKS        W25N01_FS_NUM_BLOCKS
#else
#define FS_NUM_BLOCKS        W25N02_FS_NUM_BLOCKS
#endif

//Total no of blocks available in w25m02gw/w25n02jw variant is 2048 in which 40 blocks are reserved
//! @}

#define FS_SECTORS_PER_BLOCK (FS_SECTORS_PER_PAGE * FS_PAGES_PER_BLOCK)

//! The number of blocks to preallocate
#define FS_PREALLOCATE_SIZE  (10)
#define FS_INODE0_MAX_BLOCK (32)

//! @} // FLogConf

#endif
