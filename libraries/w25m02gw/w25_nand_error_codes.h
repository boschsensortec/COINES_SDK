/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
