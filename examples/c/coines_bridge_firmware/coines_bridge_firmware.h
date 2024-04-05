/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    coines_bridge_firmware.h
 * @brief   This file contains function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef COINES_BRIDGE_FIRMWARE_H_
#define COINES_BRIDGE_FIRMWARE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************************/
/* header includes */
/**********************************************************************************/
#include <stdint.h>
#include "coines.h"

/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/
/* COM_READ_BUFF_SIZE - Configurable in compile time through makefile, default value is 2048 */
#define READ_BUFF_SIZE                    UINT16_C(2060)

#define WRITE_BUFF_SIZE                   UINT16_C(COM_READ_BUFF_SIZE + COINES_MAX_HEADER_LEN + TIMESTAMP_SIZE)

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* COINES_BRIDGE_FIRMWARE_H_ */
