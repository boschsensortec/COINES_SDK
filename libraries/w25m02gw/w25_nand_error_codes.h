/**
 *
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 * @file        w25_nand_error_codes.h
 *
 * @brief
 */

/*!
 * @addtogroup w25_nand_error_codes
 * @brief
 * @{*/


#ifndef W25_NAND_ERROR_CODES_H_
#define W25_NAND_ERROR_CODES_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**********************************************************************************/
/* header includes */
/**********************************************************************************/


/**********************************************************************************/
/* (extern) variable declarations */
/**********************************************************************************/


/**********************************************************************************/
/* function prototype declarations */
/**********************************************************************************/

/**
 * @brief Enum which holds the error codes
 */
typedef enum w25_nand_errorcode_enum_type
{
    W25_NAND_ERROR,
    W25_NAND_ERR_LOCATION_INVALID,
    W25_NAND_ERR_BUFFER_INVALID,
    W25_NAND_ERR_BYTE_LEN_INVALID,
    W25_NAND_UNINITIALIZED,
    W25_NAND_INITIALIZED,
    W25_NAND_INITIALIZATION_FAILED,
    W25_NAND_ERASE_SUCCESS,
    W25_NAND_ERASE_FAILURE,
    W25_NAND_READ_SUCCESS,
    W25_NAND_READ_FAILURE,
    W25_NAND_ECC_FAILURE,
    W25_NAND_WRITE_SUCCESS,
    W25_NAND_WRITE_FAILURE,
    W25_NAND_BUSY,
    W25_NAND_BBM_FAILURE,
    W25_NAND_BBM_SUCCESS
} w25_nand_error_t;

/**********************************************************************************/
/* inline function definitions */
/**********************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* W25_NAND_ERROR_CODES_H_ */

/** @}*/
