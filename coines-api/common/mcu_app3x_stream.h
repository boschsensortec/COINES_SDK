/**
 * Copyright (C) 2023 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    mcu_app3x_stream.h
 * @date    Mar 10, 2023
 * @brief   This file contains COINES_SDK layer function prototypes, variable declarations and Macro definitions
 *
 */
#ifndef MCU_APP3X_STREAM_H_
#define MCU_APP3X_STREAM_H_

#include <stdint.h>
#include <stdio.h>

#include "coines.h"
/**********************************************************************************/
/* macro definitions */
/**********************************************************************************/

/**********************************************************************************/
/* data structure declarations  */
/**********************************************************************************/
struct coines_streaming_settings 
{
    uint8_t sensor_id; /*< streaming sensor id */
    struct coines_streaming_config stream_config; /*< streaming config */
    struct coines_streaming_blocks data_blocks; /*< streaming data blocks */
    uint32_t sampling_period_us;
    uint32_t gst_ticks_counter;
    uint32_t gst_multiplier;
    uint8_t DATA_intline[2]; /*< interrupt line state */
};
/**********************************************************************************/
/* functions */
/**********************************************************************************/

#endif /* MCU_APP3X_STREAM_H_ */