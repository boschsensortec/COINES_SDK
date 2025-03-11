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
 * @file       BQ25120.h
 * @date       2022-05-30
 * @update	  2022-09-22
 * @version    v0.0.1
*/
#ifndef _BQ25120_H
#define _BQ25120_H

/* Header includes */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "bq25120_defs.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/*! macro definitions */
#ifdef PRE_CHARGE_EN
#define PRECHRG_CFG_TERM_STATE BQ_ITERM_ENABLE
#else
#define PRECHRG_CFG_TERM_STATE BQ_ITERM_DISABLE
#endif

int8_t bq_init(struct bq_dev *dev);
int8_t bq_set_mode_config(struct bq_dev *dev, uint8_t fctmode);
int8_t bq_get_status(struct bq_dev *dev, struct bq_status *state);
int8_t bq_device_reset(struct bq_dev *dev);
int8_t bq_set_charge_current(struct bq_dev *dev, struct bq_fast_charge_config cfg);
int8_t bq_set_faults_mask(struct bq_dev *dev, struct fault_mask_reg cfg);
int8_t bq_get_faults(struct bq_dev *dev, struct fault_mask_reg *state);
int8_t bq_set_ts_ctrl_fault_mask(struct bq_dev *dev, struct ts_flt_mask_config cfg);
int8_t bq_get_ts_ctrl_fault(struct bq_dev *dev, uint8_t *tsfault);
int8_t bq_set_term_precharge_current(struct bq_dev *dev, struct bq_term_precharge_config cfg);
int8_t bq_set_battery_charge_volt(struct bq_dev *dev, const uint8_t cfg);
int8_t bq_set_sys_vout(struct bq_dev *dev, struct bq_sys_vout cfg);
int8_t bq_set_load_ldo(struct bq_dev *dev, struct bq_load_switch cfg);
int8_t bq_set_push_button(struct bq_dev *dev, struct bq_push_btn cfg);
int8_t bq_ack_push_button_events(struct bq_dev *dev, uint8_t *EventOcuured);
int8_t bq_set_ilim_bat_uvlo(struct bq_dev *dev, struct bq_ilim_bat_uvlo cfg);
int8_t bq_get_battery_voltage(struct bq_dev *dev, uint8_t *batvolt);
int8_t bq_get_battery_soc(struct bq_dev *dev, uint8_t *batsoc);
int8_t bq_set_vin_dpm_timer(struct bq_dev *dev, struct bq_vim_and_timers cfg);
int8_t bq_charge_enable(struct bq_dev *dev);
int8_t bq_charge_disable(struct bq_dev *dev);
int8_t bq_write_reg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bq_dev *dev);
int8_t bq_read_reg(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bq_dev *dev);
int8_t bq_delay_ms(struct bq_dev *dev, uint32_t period);
int8_t bq_cd_set(struct bq_dev *dev, uint8_t cd_state);
int8_t bq_battery_connected(struct bq_dev *dev);
int8_t bq_get_load_ldo(struct bq_dev *dev, uint8_t *ldovalue);
int16_t bq_get_ldo_voltage(uint8_t ldovalue);
const char* bq_check_rslt(int8_t rslt);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BQ25120_H */