/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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
 * @file       BQ25120.c
 * @date       2022-05-30
 * @update	  2022-09-22
 * @version    v0.0.2
*/

/*! @file BQ25120.c
 * @brief BQ25120 module driver.
 */

#include "bq25120.h"

/***************** Static function declarations ******************************/

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bq_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t null_ptr_check(const struct bq_dev *dev);

/***************** Global variables declarations ******************************/
// batterz SOC lookup table 
/* 
        Vbat[V]	   VBMON	SOC%	Index
        2.52	     60	     0	      0
        2.604	     62	     0	      1
        2.688	     64	     0	      2
        2.772	     66	     0	      3
        2.856	     68	     0	      4
        2.94	     70	     0	      5
        3.024	     72	     0	      6
        3.108	     74	     0	      7
        3.192	     76	     0	      8
        3.276	     78	     0	      9
        3.36	     80	     0	      10
        3.444	     82	     0	      11
        3.528	     84	     5	      12
        3.612	     86	     20	      13
        3.696	     88	     35	      14
        3.78	     90	     50	      15
        3.864	     92	     65	      16
        3.948	     94	     80	      17
        4.032	     96	     95	      18
        4.116	     98	     100	  19
        4.2	        100	     100	  20
*/
uint8_t bat_soc_table[21] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 20, 35, 50, 65, 80, 95, 100, 100};
#define Align(x)    ((x) - 60) / 2
/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It performs the reading status register and resets all BQ registers through I2C.
 */
int8_t bq_init(struct bq_dev *dev)
{
    int8_t rslt;
    uint8_t read_data = 0;
	struct bq_ilim_bat_uvlo ilim_cfg;
	struct bq_term_precharge_config prechrg_cfg;
	struct bq_fast_charge_config fastchrg_cfg;
    struct bq_load_switch ldo_cfg;
    struct bq_push_btn pushbtn_cfg;
    struct ts_flt_mask_config tscfg = {0, 0, 0, 0, 0};
    struct fault_mask_reg fault_cfg = {0, 0, 0, 0};
    uint8_t batvoltage = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        /* Read the BQ status register */
        rslt = bq_read_reg(BQ_STATUS_AND_MODE_CTRL_REG, &read_data, 1, dev);

        /* Proceed if everything is fine until now */
        if (rslt == BQ_OK)
        {
            /* Reset all registers*/
            rslt = bq_device_reset(dev);
            if (rslt == BQ_OK)
            {
                /* Give time to driver to reset */
                rslt = bq_delay_ms(dev,50);
				
                // Set BQ normal operation mode:
                rslt |= bq_set_mode_config(dev,BQ_NORMAL_OP_ENABLE);

				//Config for LP-402025-1S-3 battery :
                batvoltage = BQ_VBREG_320_MV | BQ_VBREG_160_MV | BQ_VBREG_80_MV | BQ_VBREG_40_MV; //Battery charging voltage = 3,6V + VbregCODE x 10mV (MAX = 4,650 V)
				bq_set_battery_charge_volt(dev, batvoltage);

				ilim_cfg.ilimcode = BQ_INLIM_200_MA; 
				ilim_cfg.batuvlo = BQ_BUVLO_2_4_V; //Discharge Cut-off Voltage
				rslt |= bq_set_ilim_bat_uvlo(dev, ilim_cfg);

				fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
				fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
				fastchrg_cfg.charger_state = BQ_CHARGER_ENABLE;
				fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
				rslt |= bq_set_charge_current(dev, fastchrg_cfg);

                prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
				prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
				prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
				rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);

                ldo_cfg.enable = BQ_LOAD_LDO_DISABLE;
                ldo_cfg.code = BQ_LOAD_LDO_800_MV | BQ_LOAD_LDO_200_MV; // Set LDO output to 0,8+0,8+0,2=1,8V
                ldo_cfg.reset = BQ_LOAD_LDO_RESET_TIME;
                rslt |= bq_set_load_ldo(dev, ldo_cfg);

                pushbtn_cfg.wake1_timer = BQ_MR_W1_SUP_80_MS;
                pushbtn_cfg.wake2_timer = BQ_MR_W2_SUP_1500_MS;
                pushbtn_cfg.afterreset = BQ_MR_RESET_HI_Z_MODE;
	            pushbtn_cfg.reset_timer = BQ_MR_RESET_5_S;
	            pushbtn_cfg.output = BQ_MR_INT_OUTPUT_PG;
	            rslt |= bq_set_push_button(dev, pushbtn_cfg);

	            tscfg.ts_fct = BQ_TS_FCT_DISABLE;
	            tscfg.int_fct = BQ_INT_FCT_DISABLE;
	            tscfg.mr_wake = BQ_MR_WAKE_IT_ENABLE;
	            tscfg.mr_reset = BQ_MR_RESET_IT_ENABLE;
	            tscfg.timer_fault = BQ_TIMER_FAULT_IT_ENABLE;
	            rslt |=bq_set_ts_ctrl_fault_mask(dev, tscfg);

	            fault_cfg.vin_ov = BQ_VIN_OVERVOLTAGE_ENABLE;
	            fault_cfg.vin_uv = BQ_VIN_UNDERVOLTAGE_ENABLE;
	            fault_cfg.bat_uvlo = BQ_BAT_UVLO_ENABLE;
	            fault_cfg.bat_ocp = BQ_BAT_OCP_ENABLE;
	            rslt |= bq_set_faults_mask(dev, fault_cfg);
            }
            else
            {
                rslt = BQ_DEV_NOT_FOUND;
            }
        }
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API sets the essential configuration of the BQ.
 * @param[in] fctmode : Operational mode (Normal operation or Ship mode)
 */
int8_t bq_set_mode_config(struct bq_dev *dev, uint8_t fctmode)
{
    int8_t rslt;
    uint8_t registervalue = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        /* Read the BQ status register */
        rslt = bq_read_reg(BQ_STATUS_AND_MODE_CTRL_REG, &registervalue, 1, dev);
        bq_delay_ms(dev,10);

        /* Proceed if everything is fine until now */
        if (rslt == BQ_OK)
        {
            registervalue = registervalue | fctmode;
            /*Write selected mode configuration*/
            rslt = bq_write_reg(BQ_STATUS_AND_MODE_CTRL_REG,&registervalue,1,dev);
        }        
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API gets the essential status of the BQ.
 */
int8_t bq_get_status(struct bq_dev *dev, struct bq_status *state)
{
    int8_t rslt;
    uint8_t registervalue = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        /* Read the BQ status register */
        rslt = bq_read_reg(BQ_STATUS_AND_MODE_CTRL_REG, &registervalue, 1, dev);
        bq_delay_ms(dev,10);

        /* Proceed if everything is fine until now */
        if (rslt == BQ_OK)
        {
            state->status = registervalue>>6;
			state->reset_fault = (registervalue&0x10)>>4;
			state->safety_timer_fault = (registervalue&0x08)>>3;
			state->vin_dpm_status = (registervalue&0x04)>>2;
			state->cd_status = (registervalue&0x02)>>1;
			state->sw_status = registervalue&0x01;
        }        
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API performs the reset of all BQ register to their default values.
 */
int8_t bq_device_reset(struct bq_dev *dev)
{
    uint8_t data_cmd[1];
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        rslt = bq_read_reg(BQ_ILIM_BATTERY_UVLO_CTRL_REG,&data_cmd[0],1,dev);
        if (rslt == BQ_OK)
        {   
            data_cmd[0] = data_cmd[0] | BQ_RESET_ALL_REGS;
            rslt = bq_write_reg(BQ_ILIM_BATTERY_UVLO_CTRL_REG,&data_cmd[0],1,dev);
        }
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API is used to select charging current.
 */
int8_t bq_set_charge_current(struct bq_dev *dev, struct bq_fast_charge_config cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.code | cfg.range | cfg.charger_state | cfg.hz_mode;
        rslt = bq_write_reg(BQ_FAST_CHARGE_CTRL_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to configure the Faults and Faults Mask Register.
 */
int8_t bq_set_faults_mask(struct bq_dev *dev, struct fault_mask_reg cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.vin_ov | cfg.vin_uv | cfg.bat_uvlo | cfg.bat_ocp;
        rslt = bq_write_reg(BQ_FAULT_AND_FAULT_MASK_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API gets Faults from Faults and Faults Mask Register of the BQ.
 */
int8_t bq_get_faults(struct bq_dev *dev, struct fault_mask_reg *state)
{
    int8_t rslt;
    uint8_t registervalue = 0;
    struct bq_fast_charge_config fastchrg_cfg;
    struct bq_term_precharge_config prechrg_cfg;
    struct bq_ilim_bat_uvlo ilim_cfg;
    uint8_t cd_entry_state;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        cd_entry_state = dev->cd_pin_state; //Save CD initial state

        bq_delay_ms(dev,10);
        bq_cd_set(dev,1);
        bq_delay_ms(dev,20);

        /*Set Buvlo to 2,2V*/
        ilim_cfg.ilimcode = BQ_INLIM_200_MA; 
        ilim_cfg.batuvlo = BQ_BUVLO_2_2_V; //Discharge Cut-off Voltage
        rslt |= bq_set_ilim_bat_uvlo(dev, ilim_cfg);

        /*Disable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);

        /*Disable charge to show faults*/
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_DISABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);
        bq_delay_ms(dev,100);

        /* Read the BQ status register */
        rslt = bq_read_reg(BQ_FAULT_AND_FAULT_MASK_REG, &registervalue, 1, dev);

        /* Proceed if everything is fine until now */
        if (rslt == BQ_OK)
        {
			state->vin_ov = registervalue>>7;
			state->vin_uv = (registervalue&0x40)>>6;
			state->bat_uvlo = (registervalue&0x20)>>5;
			state->bat_ocp = (registervalue&0x10)>>4;
        }       

        /*Enable charge*/
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_ENABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);

        /*Set Buvlo to 2,2V*/
        ilim_cfg.ilimcode = BQ_INLIM_200_MA; 
        ilim_cfg.batuvlo = BQ_BUVLO_2_4_V; //Discharge Cut-off Voltage
        rslt |= bq_set_ilim_bat_uvlo(dev, ilim_cfg);
        
        bq_delay_ms(dev,10);
        bq_cd_set(dev,cd_entry_state);
        bq_delay_ms(dev,10);

    }
    else
    {
        rslt = BQ_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API is used to configure the TS Control and Faults Masks Register.
 */
int8_t bq_set_ts_ctrl_fault_mask(struct bq_dev *dev, struct ts_flt_mask_config cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.ts_fct | cfg.int_fct | cfg.mr_wake | cfg.mr_reset | cfg.timer_fault;
        rslt = bq_write_reg(BQ_TS_CTRL_AND_FAULT_MASK_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to get TS control faults.
 * @param EventOcuured : 0 Normal, No TS fault  
 *						 1 TS temp < TCOLD or TS temp > THOT (Charging suspended)
 *						 2 TCOOL > TS temp > TCOLD (Charging current reduced by half)
 *						 3 TWARM < TS temp < THOT (Charging voltage reduced by 140 mV)
 */
int8_t bq_get_ts_ctrl_fault(struct bq_dev *dev, uint8_t *tsfault)
{
    int8_t rslt;
    uint8_t readdata = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        rslt = bq_read_reg(BQ_TS_CTRL_AND_FAULT_MASK_REG,&readdata,1,dev);
        *tsfault = (readdata&0x60)>>0x05;
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to select termination and pre-charge current.
 */
int8_t bq_set_term_precharge_current(struct bq_dev *dev, struct bq_term_precharge_config cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.code | cfg.range | cfg.term_state;
        rslt = bq_write_reg(BQ_TERM_PRECHARGE_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to select battery charging voltage.
 */
int8_t bq_set_battery_charge_volt(struct bq_dev *dev, const uint8_t cfg)
{
    int8_t rslt;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        rslt = bq_write_reg(BQ_BATTERY_VOLTAGE_CTRL_REG,&cfg,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to set SYS Vout voltage.
 */
int8_t bq_set_sys_vout(struct bq_dev *dev, struct bq_sys_vout cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.enable | cfg.sys_select | cfg.code;
        rslt = bq_write_reg(BQ_SYS_VOUT_CTRL_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to control the load switch and LDO.
 */
int8_t bq_set_load_ldo(struct bq_dev *dev, struct bq_load_switch cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.enable | cfg.code | cfg.reset;
        rslt = bq_write_reg(BQ_LOAD_AND_LDO_CTRL_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to configure the push-button.
 */
int8_t bq_set_push_button(struct bq_dev *dev, struct bq_push_btn cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.wake1_timer | cfg.wake2_timer | cfg.afterreset | cfg.reset_timer | cfg.output;
        rslt = bq_write_reg(BQ_PUSH_BUTTON_CTRL_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to get the push-button wake events.
 * @param EventOcuured : 0 No events occured 
 *						 1 Wake 2 event occured
 *						 2 Wake 1 event occured
 *						 3 Both Wake 1 and Wake 2 occured
 */
int8_t bq_ack_push_button_events(struct bq_dev *dev, uint8_t *EventOcuured)
{
    int8_t rslt;
	uint8_t readreg = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        rslt = bq_read_reg(BQ_PUSH_BUTTON_CTRL_REG,&readreg,1,dev);
		*EventOcuured = readreg & 0x03;
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to set input current limit and the Battery undervoltage-lockout.
 */
int8_t bq_set_ilim_bat_uvlo(struct bq_dev *dev, struct bq_ilim_bat_uvlo cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.ilimcode | cfg.batuvlo;
        rslt = bq_write_reg(BQ_ILIM_BATTERY_UVLO_CTRL_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used read the current battery voltage [in % of Vba].
 */
int8_t bq_get_battery_voltage(struct bq_dev *dev, uint8_t *batvolt)
{
    int8_t rslt;
    uint8_t sendupdate = BQ_BATTERY_VOLTAGE_UPDATE;
    uint8_t readdata = 0;
    uint8_t vrange = 0;
    uint8_t vth = 0;
    uint8_t cd_entry_state =0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {   
        cd_entry_state = dev->cd_pin_state; //Save CD initial state

        bq_cd_set(dev,1); //Push CD high to get an accurate reading of the battery voltage
        bq_delay_ms(dev,20);
        
        rslt = bq_write_reg(BQ_VOLT_BASED_BATT_MONITOR_REG,&sendupdate,1,dev);
        bq_delay_ms(dev,2);
        /* Check for arguments validity */
        if (rslt == BQ_OK)
        {
            rslt = bq_read_reg(BQ_VOLT_BASED_BATT_MONITOR_REG,&readdata,1,dev);
            if (rslt == BQ_OK)
            {
                vrange = (readdata & 0x60) >> 5;
                vth = (readdata & 0x1C) >> 2;
                
                switch(vrange)
                {
                    case 0:
                        *batvolt = 60;
                        break;
                    case 1:
                        *batvolt = 70;
                        break;
                    case 2:
                        *batvolt = 80;
                        break;
                    case 3:
                        *batvolt = 90;
                        break;
                    default:
                        *batvolt = 0;
                        break;                       
                };
                
                switch(vth)
                {
                    case 0:
						*batvolt = *batvolt + 0;
						break;
					case 1:
						*batvolt = *batvolt + 2;
						break;
					case 2:
						*batvolt = *batvolt + 4;
						break;
					case 3:
                    case 4:
						*batvolt = *batvolt + 6;
						break;
                    case 5:
					case 6:
						*batvolt = *batvolt + 8;
						break;
					case 7:
						*batvolt = *batvolt + 10;
						break;
					default:
                        *batvolt = 0;
						break;                                     
                };
            }
            else
            {
                rslt = BQ_COMM_FAIL;
            }
        }
        else
        {
            rslt = BQ_COMM_FAIL;
        }

        bq_cd_set(dev,cd_entry_state); //Put CD back to its original state

    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used read the current battery state of charge [%].
 */
int8_t bq_get_battery_soc(struct bq_dev *dev, uint8_t *batsoc)
{
    int8_t rslt;
    uint8_t readdata = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        rslt = bq_get_battery_voltage(dev, &readdata);
        if (rslt == BQ_OK)
        {
            if(readdata == 60)
            {
                *batsoc = 0;
            }
            else
            {
                *batsoc = bat_soc_table[Align(readdata)];
            }
        }
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to set the VINDPM threshold and timers configuration.
 */
int8_t bq_set_vin_dpm_timer(struct bq_dev *dev, struct bq_vim_and_timers cfg)
{
    int8_t rslt;
    uint8_t configuration = 0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        configuration = cfg.vindpm_en | cfg.vindpm_code | cfg.timer2x | cfg.safetytimer;
        rslt = bq_write_reg(BQ_VIN_DPM_AND_TIMERS_REG,&configuration,1,dev);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to set enable charging.
 */
int8_t bq_charge_enable(struct bq_dev *dev)
{
    int8_t rslt;
    struct bq_fast_charge_config fastchrg_cfg;
    struct bq_ilim_bat_uvlo ilim_cfg;
    struct fault_mask_reg faults;
    struct bq_term_precharge_config prechrg_cfg;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == BQ_OK)&&(dev->battery_connected == 0))
    {
        bq_delay_ms(dev,10);
        bq_cd_set(dev,1);
        bq_delay_ms(dev,20);

        /*Set Buvlo to 2,2V*/
        ilim_cfg.ilimcode = BQ_INLIM_200_MA; 
        ilim_cfg.batuvlo = BQ_BUVLO_2_2_V; //Discharge Cut-off Voltage
        rslt |= bq_set_ilim_bat_uvlo(dev, ilim_cfg);
        //bq_delay_ms(dev,20);

        /*Disable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);

        /*Disable charge Bit*/
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_DISABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);
        bq_delay_ms(dev,100);

        /*Read Buvlo fault*/
        bq_get_faults(dev, &faults);

        if(faults.bat_uvlo == 1) /*Battery not connected*/
        {
            /*Disable IPREM/TERM Current*/
            prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
            prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
            prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
            rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);
#ifdef PRE_CHARGE_EN
            dev->battery_connected = 0;
#endif
        }
        else /*Battery connected*/
        {
            /*Enable IPREM/TERM Current*/
            prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
            prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
            prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
            rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);
#ifdef PRE_CHARGE_EN
            dev->battery_connected = 1;
#endif
        }

        /*Set Buvlo to defaul 2,4V*/
        ilim_cfg.ilimcode = BQ_INLIM_200_MA; 
        ilim_cfg.batuvlo = BQ_BUVLO_2_4_V; //Discharge Cut-off Voltage
        rslt |= bq_set_ilim_bat_uvlo(dev, ilim_cfg);    

        /*Enable charge bit*/       
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_ENABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);  

        bq_delay_ms(dev,10);
        bq_cd_set(dev,0);
        bq_delay_ms(dev,20);
    }
    else if ((rslt == BQ_OK)&&(dev->battery_connected == 1))
    {
        /*Enable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);

        /*Enable charge bit*/       
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_ENABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);  

        bq_delay_ms(dev,10);
        bq_cd_set(dev,0);
        bq_delay_ms(dev,20);       
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to set disable charging.
 */
int8_t bq_charge_disable(struct bq_dev *dev)
{
    int8_t rslt;
    struct bq_fast_charge_config fastchrg_cfg;
    struct bq_term_precharge_config prechrg_cfg;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == BQ_OK)
    {
        /*Disable charge bit*/       
        fastchrg_cfg.range = BQ_ICHRG_RANGE_40_300_MA;
        fastchrg_cfg.code = BQ_ICHRG_CURRENT_8_80_MA | BQ_ICHRG_CURRENT_2_20_MA | BQ_ICHRG_CURRENT_1_10_MA; // 110 mA + 40 mA = 150 mA (Battery max charging current = 155 mA)
        fastchrg_cfg.charger_state = BQ_CHARGER_ENABLE;
        fastchrg_cfg.hz_mode = BQ_HIGH_IMP_MODE_DISABLE;
        rslt |= bq_set_charge_current(dev, fastchrg_cfg);  

        /*Enable IPREM/TERM Current*/
        prechrg_cfg.range = BQ_IPRETERM_RANGE_6_37_MA;
        prechrg_cfg.code = BQ_IPRETERM_CURRENT_1_2_MA | BQ_IPRETERM_CURRENT_4_8_MA;//10% of Max charging current = 15mA
        prechrg_cfg.term_state = PRECHRG_CFG_TERM_STATE;
        rslt |= bq_set_term_precharge_current(dev, prechrg_cfg);

        bq_delay_ms(dev,10);
        bq_cd_set(dev,1);
        bq_delay_ms(dev,20);

#ifdef PRE_CHARGE_EN
        dev->battery_connected = 1;
#endif
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API writes the given data to the register address of the driver.
 */
int8_t bq_write_reg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bq_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == BQ_OK) && (reg_data != NULL))
    {
        if (length != 0)
        {
            dev->intf_rslt = dev->write(reg_addr, reg_data, length, dev->i2c_address);

            /* Check for communication error */
            if (dev->intf_rslt != BQ_INTF_RET_SUCCESS)
            {
                rslt = BQ_COMM_FAIL;
            }
        }
        else
        {
            rslt = BQ_INVALID_LEN;
        }
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    return rslt;    
}

/*!
 * @brief This API reads the data from the given register address of the driver.
 */
int8_t bq_read_reg(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bq_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BQ_OK) && (reg_data != NULL))
    {
        /* Read the data using I2C */
        dev->intf_rslt = dev->read(reg_addr, reg_data, length, dev->i2c_address);

        /* Check for communication error */
        if (dev->intf_rslt != BQ_INTF_RET_SUCCESS)
        {
            rslt = BQ_COMM_FAIL;
        }

        /*Check for data validity*/
        if(*reg_data == BQ_TRASH_READ_VALUE)
        {
            rslt = BQ_INVALID_ADDRESS;
        }
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to set a delay to be elapsed.
 */
int8_t bq_delay_ms(struct bq_dev *dev, uint32_t period)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BQ_OK) 
    {
        dev->delay_ms(period);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to control the cd line.
 */
int8_t bq_cd_set(struct bq_dev *dev, uint8_t cd_state)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BQ_OK) 
    {   
        dev->cd_pin_state = cd_state;
        dev->cd_set(cd_state);
    }
    else
    {
        rslt = BQ_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to check error/result by giving the result code.
 */ 
const char* bq_check_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BQ_OK:
            return "API ended with success\r\n";
            break;
        case BQ_NULL_PTR:
            return "Error: Null pointer \r\n";
            break;
        case BQ_COMM_FAIL:
            return "Error: Communication failure\r\n";
            break;
        case BQ_INVALID_LEN:
            return "Error: Incorrect length parameter\r\n";
            break;
        case BQ_DEV_NOT_FOUND:
            return "Error: Device not found\r\n";
            break;
        case BQ_CONFIGURATION_ERR:
            return "Error: Configuration Error\r\n";
            break;
        case BQ_CMD_EXEC_FAILED:
            return "Error: Command execution failed\r\n";
            break;
        case BQ_INVALID_ADDRESS:
            return "Error: Invalid I2C register address\r\n";
            break;
        default:
            return "Unknown error code\r\n";
            break;
    }
}

/****************** Static Function Definitions *******************************/

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bq_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) ||
        (dev->i2c_address == 0))
    {
        /* Device structure pointer is not valid */
        rslt = BQ_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BQ_OK;
    }

    return rslt;
}
