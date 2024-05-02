/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 * @file       BQ25120_defs.h
 * @date       2022-05-30
 * @update	   2022-09-22
 * @version    v0.0.2
*/
#ifndef BQ25120_DEFS_H_
#define BQ25120_DEFS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif


/***************************************************************************************************/

/**
 * LED_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 */
#ifndef BQ_INTF_RET_TYPE
#define BQ_INTF_RET_TYPE                        int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BQ_INTF_RET_SUCCESS
#define BQ_INTF_RET_SUCCESS                     INT8_C(0)
#endif


/**\name API success code */
#define BQ_OK                                   INT8_C(0)

/**\name API error codes */
#define BQ_NULL_PTR                             INT8_C(-1)
#define BQ_COMM_FAIL                            INT8_C(-2)
#define BQ_CMD_EXEC_FAILED                      INT8_C(-3)
#define BQ_CONFIGURATION_ERR                    INT8_C(-4)
#define BQ_INVALID_LEN                          INT8_C(-5)
#define BQ_DEV_NOT_FOUND                        INT8_C(-6)
#define BQ_INVALID_ADDRESS                      INT8_C(-7)

/***************************************************************************************************/

/*********Macros for different BQ25120 device registers**********/
/**\Device I2C address*/
#define BQ_DRV_ADDR                             0x6A 

/**\Main device configuration registers (Read/Write)*/	
#define BQ_STATUS_AND_MODE_CTRL_REG             0x00
#define BQ_FAULT_AND_FAULT_MASK_REG             0x01
#define BQ_TS_CTRL_AND_FAULT_MASK_REG           0x02
#define BQ_FAST_CHARGE_CTRL_REG                 0x03
#define BQ_TERM_PRECHARGE_REG                   0x04
#define BQ_BATTERY_VOLTAGE_CTRL_REG             0x05
#define BQ_SYS_VOUT_CTRL_REG                    0x06
#define BQ_LOAD_AND_LDO_CTRL_REG                0x07
#define BQ_PUSH_BUTTON_CTRL_REG                 0x08
#define BQ_ILIM_BATTERY_UVLO_CTRL_REG           0x09
#define BQ_VOLT_BASED_BATT_MONITOR_REG          0x0A
#define BQ_VIN_DPM_AND_TIMERS_REG               0x0B


#define BQ_TRASH_READ_VALUE                     0xFF

/**\Status ans Mode control register parameters*/
#define BQ_NORMAL_OP_ENABLE                     UINT8_C(0 << 5)
#define BQ_SHIP_MODE_ENABLE                     UINT8_C(1 << 5)

/**\Faults and Faults Mask Register*/
//VIN overvoltage fault
#define BQ_VIN_OVERVOLTAGE_ENABLE               UINT8_C(0 << 3)
#define BQ_VIN_OVERVOLTAGE_DISABLE              UINT8_C(1 << 3)
//VIN undervoltage fault
#define BQ_VIN_UNDERVOLTAGE_ENABLE              UINT8_C(0 << 2)
#define BQ_VIN_UNDERVOLTAGE_DISABLE             UINT8_C(1 << 2)
// BAT UVLO fault
#define BQ_BAT_UVLO_ENABLE              		UINT8_C(0 << 1)
#define BQ_BAT_UVLO_DISABLE               		UINT8_C(1 << 1)
// BAT_OCP fault
#define BQ_BAT_OCP_ENABLE              			UINT8_C(0 << 0)
#define BQ_BAT_OCP_DISABLE               		UINT8_C(1 << 0)

/**\TS Control and Faults Masks Register*/
//TS function
#define BQ_TS_FCT_ENABLE                 		UINT8_C(1 << 7)
#define BQ_TS_FCT_DISABLE                		UINT8_C(0 << 7)
//INT function
#define BQ_INT_FCT_ENABLE              			UINT8_C(1 << 3)
#define BQ_INT_FCT_DISABLE               		UINT8_C(0 << 3)
//Mask interrupt from Wake Condition from MR
#define BQ_MR_WAKE_IT_ENABLE              		UINT8_C(0 << 2)
#define BQ_MR_WAKE_IT_DISABLE               	UINT8_C(1 << 2)
//Mask RESET interrupt from MR . The RESET output is not masked by this bit.
#define BQ_MR_RESET_IT_ENABLE              		UINT8_C(0 << 1)
#define BQ_MR_RESET_IT_DISABLE               	UINT8_C(1 << 1)
//Mask Timer fault interrupt (safety)
#define BQ_TIMER_FAULT_IT_ENABLE              	UINT8_C(0 << 0)
#define BQ_TIMER_FAULT_IT_DISABLE               UINT8_C(1 << 0)

/**\Fast charge control register parameters*/
//Range selection
#define BQ_ICHRG_RANGE_5_35_MA                  UINT8_C(0 << 7)
#define BQ_ICHRG_RANGE_40_300_MA                UINT8_C(1 << 7)
//charging current code selection
//if range 5mA to 35mA : charging current = 5mA + IchCODE x 1mA
//else if range 40mA to 300mA : charging current = 40mA + IchCODE x 10mA
#define BQ_ICHRG_CURRENT_16_160_MA              UINT8_C(1 << 6)
#define BQ_ICHRG_CURRENT_8_80_MA               UINT8_C(1 << 5)
#define BQ_ICHRG_CURRENT_4_40_MA                UINT8_C(1 << 4)
#define BQ_ICHRG_CURRENT_2_20_MA                UINT8_C(1 << 3)
#define BQ_ICHRG_CURRENT_1_10_MA                UINT8_C(1 << 2)
//Charger state
#define BQ_CHARGER_ENABLE                       UINT8_C(0 << 1)
#define BQ_CHARGER_DISABLE                      UINT8_C(1 << 1)
//High impedance mode
#define BQ_HIGH_IMP_MODE_ENABLE                 UINT8_C(1 << 0)
#define BQ_HIGH_IMP_MODE_DISABLE                UINT8_C(0 << 0)

/**\Termination and pre-charge control register parameters*/
//Range selection
#define BQ_IPRETERM_RANGE_500_UA_5_MA            UINT8_C(0 << 7)
#define BQ_IPRETERM_RANGE_6_37_MA                UINT8_C(1 << 7)
//Termination and pre-charge current code selection
//if range 5uA to 5mA : current = 500uA + IchCODE x 500uA
//else if range 6mA to 37mA : current = 6mA + IchCODE x 1mA
#define BQ_IPRETERM_CURRENT_8_16_MA              UINT8_C(1 << 6)
#define BQ_IPRETERM_CURRENT_4_8_MA               UINT8_C(1 << 5)
#define BQ_IPRETERM_CURRENT_2_4_MA               UINT8_C(1 << 4)
#define BQ_IPRETERM_CURRENT_1_2_MA               UINT8_C(1 << 3)
#define BQ_IPRETERM_CURRENT_500_UA_1_MA          UINT8_C(1 << 2)
//Charge current termination state
#define BQ_ITERM_ENABLE                          UINT8_C(1 << 1)
#define BQ_ITERM_DISABLE                         UINT8_C(0 << 1)

/**\Battery voltage control register parameters*/
//Battery charging voltage = 3,6V + VbregCODE x 10mV (MAX = 4,650 V)
#define BQ_VBREG_640_MV                          UINT8_C(1 << 6)
#define BQ_VBREG_320_MV                          UINT8_C(1 << 6)
#define BQ_VBREG_160_MV                          UINT8_C(1 << 5)
#define BQ_VBREG_80_MV                           UINT8_C(1 << 4)
#define BQ_VBREG_40_MV                           UINT8_C(1 << 3)
#define BQ_VBREG_20_MV                           UINT8_C(1 << 2)
#define BQ_VBREG_10_MV                           UINT8_C(1 << 1)

/**\SYS VOUT control register parameters*/
//SYS VOUT state
#define BQ_SYS_VOUT_ENABLE                       UINT8_C(1 << 7)
#define BQ_SYS_VOUT_DISABLE                      UINT8_C(0 << 7)
//SYS VOUT range selection for further explanation please refer to bq25120 data sheet [9.6.7] 
#define BQ_SYS_VOUT_SEL_0                        UINT8_C(0 << 5) //1.1 V and 1.2 V selection
#define BQ_SYS_VOUT_SEL_1                        UINT8_C(1 << 5) //1.3 V through 2.8 V selection
#define BQ_SYS_VOUT_SEL_2                        UINT8_C(2 << 5) //1.5V through 2.75 V selection
#define BQ_SYS_VOUT_SEL_3                        UINT8_C(3 << 5) //1.8 V through 3.3 V selection
//SYS VOUT voltage code selection
//if range BQ_SYS_VOUT_SEL_1 : Output voltage = 1,3V + SysVoutCODE x 100mV
//if range BQ_SYS_VOUT_SEL_3 : Output voltage = 1,8V + SysVoutCODE x 100mV
//else Check Table 19. SYS_SEL Codes in data sheet
#define BQ_SYS_VOUT_STEP_800_MV                  UINT8_C(1 << 4)
#define BQ_SYS_VOUT_STEP_400_MV                  UINT8_C(1 << 3)
#define BQ_SYS_VOUT_STEP_200_MV                  UINT8_C(1 << 2)
#define BQ_SYS_VOUT_STEP_100_MV                  UINT8_C(1 << 1)

/**\Load Switch and LDO control register parameters*/
//Load Switch and LDO state
#define BQ_LOAD_LDO_ENABLE                       UINT8_C(1 << 7)
#define BQ_LOAD_LDO_DISABLE                      UINT8_C(0 << 7)
//Load Switch and LDO voltage code selection
//Output voltage = 0,8V + Ls_LDOCODE x 100mV
#define BQ_LOAD_LDO_1600_MV                      UINT8_C(1 << 6)
#define BQ_LOAD_LDO_800_MV                       UINT8_C(1 << 5)
#define BQ_LOAD_LDO_400_MV                       UINT8_C(1 << 4)
#define BQ_LOAD_LDO_200_MV                       UINT8_C(1 << 3)
#define BQ_LOAD_LDO_100_MV                       UINT8_C(1 << 2)
//Reset configuration
#define BQ_LOAD_LDO_RESET_TIME                   UINT8_C(0 << 0)
#define BQ_LOAD_LDO_RESET_TIME_AND_VIN           UINT8_C(1 << 0)

/**\Push-button control register parameters*/
//MR Timer adjustment for WAKE1
#define BQ_MR_W1_SUP_80_MS	                     UINT8_C(0 << 7)
#define BQ_MR_W1_SUP_600_MS                      UINT8_C(1 << 7)
//MR Timer adjustment for WAKE2
#define BQ_MR_W2_SUP_1000_MS	                 UINT8_C(0 << 6)
#define BQ_MR_W2_SUP_1500_MS                     UINT8_C(1 << 6)
//device state after reset
#define BQ_MR_RESET_SHIP_MODE	                 UINT8_C(0 << 5)
#define BQ_MR_RESET_HI_Z_MODE                    UINT8_C(1 << 5)
//MR Timer adjustment for reset
#define BQ_MR_RESET_5_S	                 		 UINT8_C(0 << 3)
#define BQ_MR_RESET_9_S                     	 UINT8_C(1 << 3)
#define BQ_MR_RESET_11_S	                 	 UINT8_C(2 << 3)
#define BQ_MR_RESET_15_S                     	 UINT8_C(3 << 3)
//INT output
#define BQ_MR_INT_OUTPUT_PG	                 	 UINT8_C(0 << 2)
#define BQ_MR_INT_VOLTAGE_SHIFTED                UINT8_C(1 << 2)

/**\ILIM and Battery UVLO control register parameters*/
//Reset all bq registers
#define BQ_RESET_ALL_REGS                        UINT8_C(1 << 7)
#define BQ_NO_RESET                              UINT8_C(0 << 7)
//Input current limit
//I(INLIM) is calculated using the following equation: I(INLIM) = 50 mA +I(INLIM)CODE x 50 mA
#define BQ_INLIM_200_MA                          UINT8_C(1 << 5)
#define BQ_INLIM_100_MA                          UINT8_C(1 << 4)
#define BQ_INLIM_50_MA                           UINT8_C(1 << 3)
//Battery undervoltage-lockout selection
//Please note the following combinations :
/*  000, 001    : RESERVED
    010         : BUVLO = 3.0 V
    011         : BUVLO = 2.8 V
    100         : BUVLO = 2.6 V
    101         : BULVO = 2.4 V
    110, 111    : BUVLO = 2.2 V
*/
#define BQ_BUVLO_3_0_V                           UINT8_C(2 << 0)
#define BQ_BUVLO_2_8_V                           UINT8_C(3 << 0)
#define BQ_BUVLO_2_6_V                           UINT8_C(4 << 0)
#define BQ_BUVLO_2_4_V                           UINT8_C(5 << 0)
#define BQ_BUVLO_2_2_V                           UINT8_C(6 << 0)

/**\Voltage Based Battery Monitor register parameters*/
//Initiate a new VBATREG reading
#define BQ_BATTERY_VOLTAGE_UPDATE                UINT8_C(1 << 7)

/**\VIN_DPM and Timers register parameters*/
//Load Switch and LDO state
#define BQ_VIN_DPM_ENABLE                        UINT8_C(0 << 7)
#define BQ_VIN_DPM_DISABLE                       UINT8_C(1 << 7)
//VINDPM threshold code selection
//VINDPM = 4.2 + VINDPM_CODE x 100 mV
#define BQ_VIN_DPM_400_MV                        UINT8_C(1 << 6)
#define BQ_VIN_DPM_200_MV                        UINT8_C(1 << 5)
#define BQ_VIN_DPM_100_MV                        UINT8_C(1 << 4)
//2XTMR configuration
#define BQ_2XTMR_EN_SLOWED                       UINT8_C(1 << 3)
#define BQ_2XTMR_EN_NOT_SLOWED                   UINT8_C(0 << 3)
//Safety Timer Time Limit
#define BQ_TMR_SET_30_MIN                        UINT8_C(0 << 1)
#define BQ_TMR_SET_3_H                           UINT8_C(1 << 1)
#define BQ_TMR_SET_9_H                           UINT8_C(2 << 1)
#define BQ_TMR_DISABLE                           UINT8_C(3 << 1)

/***************************************************************************************************/

/*!
 * @brief Type definitions
 */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] i2c_address : i2c device address
 * @retval 0 for Success
 * @retval <0 for Failure
 */
typedef BQ_INTF_RET_TYPE (*bq_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, uint8_t i2c_address);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] i2c_address : i2c device address
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef BQ_INTF_RET_TYPE (*bq_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, uint8_t i2c_address);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period    : Delay in microseconds.
 *
 */
typedef void (*bq_delay_ms_fptr_t)(uint32_t period);

/*!
 * @brief Chip Disable pin control function pointer which should be mapped to
 * pin control function of the user
 *
 * @param[in] cd_state    : CD pin state to set.
 *
 */
typedef BQ_INTF_RET_TYPE (*bq_cd_fptr_t)(uint8_t cd_state);

/***************************************************************************************************/

/*!
 * @brief BQ25120 driver device structure
 */
struct bq_dev
{
    /*! I2C bq device address*/
    uint8_t i2c_address;

    /*! Chip disable pin state*/
    uint8_t cd_pin_state;

    /*! To store interface pointer error */
    BQ_INTF_RET_TYPE intf_rslt;

    /*! Read function pointer */
    bq_read_fptr_t read;

    /*! Write function pointer */
    bq_write_fptr_t write;

    /*! Delay function pointer */
    bq_delay_ms_fptr_t delay_ms;

    /*! Chip Disable function pointer */
    bq_cd_fptr_t cd_set;

    /*! battery connected status */
    uint8_t battery_connected;
};

/*!
 * @brief BQ status structure
 */
struct bq_status
{
    uint8_t status;
    uint8_t reset_fault;
    uint8_t safety_timer_fault;
    uint8_t vin_dpm_status;
	uint8_t cd_status;
	uint8_t sw_status;
};

/*!
 * @brief Faults and Faults Mask Register configuration and read structure
 */
struct fault_mask_reg
{
    uint8_t vin_ov;
    uint8_t vin_uv;
    uint8_t bat_uvlo;
    uint8_t bat_ocp;
};

/*!
 * @brief TS Control and Faults Masks Register configuration structure
 */
struct ts_flt_mask_config
{
    uint8_t ts_fct;
    uint8_t int_fct;
    uint8_t mr_wake;
    uint8_t mr_reset;
	uint8_t timer_fault;
};

/*!
 * @brief Fast charge control register configuration structure
 */
struct bq_fast_charge_config
{
    uint8_t range;
    uint8_t code;
    uint8_t charger_state;
    uint8_t hz_mode;
};

/*!
 * @brief Termination and pre-charge control register configuration structure
 */
struct bq_term_precharge_config
{
    uint8_t range;
    uint8_t code;
    uint8_t term_state;
};

/*!
 * @brief SYS VOUT control register configuration structure
 */
struct bq_sys_vout
{
    uint8_t enable;
    uint8_t sys_select;
    uint8_t code;
};

/*!
 * @brief Load Switch and LDO control register configuration structure
 */
struct bq_load_switch
{
    uint8_t enable;
    uint8_t code;
    uint8_t reset;
};

/*!
 * @brief Push button control register configuration structure
 */
struct bq_push_btn
{
    uint8_t wake1_timer;
    uint8_t wake2_timer;
    uint8_t afterreset;
	uint8_t reset_timer;
	uint8_t output;
};


/*!
 * @brief ILIM and Battery UVLO control register configuration structure
 */
struct bq_ilim_bat_uvlo
{
    uint8_t ilimcode;
    uint8_t batuvlo;
};

/*!
 * @brief VIN_DPM and Timers register configuration structure
 */
struct bq_vim_and_timers
{
    uint8_t vindpm_en;
    uint8_t vindpm_code;
    uint8_t timer2x;
    uint8_t safetytimer;
};

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BQ25120_DEFS_H_ */
