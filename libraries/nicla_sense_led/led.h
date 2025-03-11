/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
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
 * @file       Led.h
 * @date       2022-05-25
 * @version    v0.0.1
*/
#ifndef _LED_H
#define _LED_H

/* Header includes */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "led_defs.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif


int8_t led_init(struct led_dev *dev);
int8_t led_deinit(struct led_dev *dev);
int8_t led_setconfig(struct led_dev *dev, struct led_settings led_cfg);
int8_t led_device_reset(struct led_dev *dev);
int8_t led_update(struct led_dev *dev);
int8_t led_setcolorandintensity(struct led_dev *dev, enum led_rgb_Color color,uint8_t intensity);
int8_t led_write_reg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct led_dev *dev);
int8_t led_read_reg(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct led_dev *dev);
int8_t led_delay_ms(struct led_dev *dev, uint32_t period);
const char* led_check_rslt(int8_t rslt);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _LED_H */
