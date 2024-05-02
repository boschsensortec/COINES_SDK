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
 * @file       Led_defs.h
 * @date       2022-05-25
 * @version    v0.0.1
*/
#ifndef LED_DEFS_H_
#define LED_DEFS_H_

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
#ifndef LED_INTF_RET_TYPE
#define LED_INTF_RET_TYPE                       int8_t
#endif

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef LED_INTF_RET_SUCCESS
#define LED_INTF_RET_SUCCESS                    INT8_C(0)
#endif


/**\name API success code */
#define LED_OK                                  INT8_C(0)

/**\name API error codes */
#define LED_NULL_PTR                            INT8_C(-1)
#define LED_COMM_FAIL                           INT8_C(-2)
#define LED_CMD_EXEC_FAILED                     INT8_C(-3)
#define LED_CONFIGURATION_ERR                   INT8_C(-4)
#define LED_INVALID_LEN                         INT8_C(-5)
#define LED_DEV_NOT_FOUND                       INT8_C(-6)

/***************************************************************************************************/

/*********Macros for different led driver device registers**********/
/**\Device I2C address*/
#define LED_DRV_ADDR                            0x53 

/**\Led driver product ID*/
#define LED_DRV_PROD_ID                         0xCE

/**\Main device configuration registers (Rear/Write)*/	
#define LED_DRV_ID_REG                          0x00
#define LED_DRV_OP_CONF_REG                     0x01
#define LED_DRV_OUT_EN_REG                      0x02
#define LED_DRV_CURRENT_REG                     0x03
#define LED_DRV_HOLD_FCT_REG                    0x04

/**\Main device state register (Read Only)*/
#define LED_DRV_PAT1_STATE                      0x0D
#define LED_DRV_PAT2_STATE                      0x0E
#define LED_DRV_PAT3_STATE                      0x0F

/**\Macros for command to use for Update and Reset*/
#define LED_DRV_EXECUTE_CMD                     0xC5

/**\Color update registers*/
#define LED_DRV_UPDATE                          0x40

/**\Device reset register*/
#define LED_DRV_RESET                           0x4F

/* Note :
* Output 1 -> BLUE
* Output 2 -> GREEN
* Output 3 -> RED
*/

/**\Current Mode*/
/**\Current Level Registers*/
#define LED_DRV_OUT1_CURR_LVL                   0x10  
#define LED_DRV_OUT2_CURR_LVL                   0x21
#define LED_DRV_OUT3_CURR_LVL                   0x32

/**\Pattern mode*/
/**\ RGB Colors for pattern 1*/
#define LED_DRV_PAT1_COLOR1_R                   0x10
#define LED_DRV_PAT1_COLOR1_G                   0x11
#define LED_DRV_PAT1_COLOR1_B                   0x12
#define LED_DRV_PAT1_COLOR2_R                   0x13
#define LED_DRV_PAT1_COLOR2_G                   0x14
#define LED_DRV_PAT1_COLOR2_B                   0x15
#define LED_DRV_PAT1_COLOR3_R                   0x16
#define LED_DRV_PAT1_COLOR3_G                   0x17
#define LED_DRV_PAT1_COLOR3_B                   0x18

/**\RGB Colors for pattern 2*/	
#define LED_DRV_PAT2_COLOR1_R                   0x20
#define LED_DRV_PAT2_COLOR1_G                   0x21
#define LED_DRV_PAT2_COLOR1_B                   0x22
#define LED_DRV_PAT2_COLOR2_R                   0x23
#define LED_DRV_PAT2_COLOR2_G                   0x24
#define LED_DRV_PAT2_COLOR2_B                   0x25
#define LED_DRV_PAT2_COLOR3_R                   0x26
#define LED_DRV_PAT2_COLOR3_G                   0x27
#define LED_DRV_PAT2_COLOR3_B                   0x28

/**\RGB Colors for pattern 3*/	
#define LED_DRV_PAT3_COLOR1_R                   0x30
#define LED_DRV_PAT3_COLOR1_G                   0x31
#define LED_DRV_PAT3_COLOR1_B                   0x32
#define LED_DRV_PAT3_COLOR2_R                   0x33
#define LED_DRV_PAT3_COLOR2_G                   0x34
#define LED_DRV_PAT3_COLOR2_B                   0x35
#define LED_DRV_PAT3_COLOR3_R                   0x36
#define LED_DRV_PAT3_COLOR3_G                   0x37
#define LED_DRV_PAT3_COLOR3_B                   0x38

/**\Ts,T1,T2,T3,TP,T4 setting register for pattern 1*/
#define LED_DRV_PAT1_TIME_SET_TS_T1             0x19
#define LED_DRV_PAT1_TIME_SET_T2_T3             0x1A
#define LED_DRV_PAT1_TIME_SET_TP_T4             0x1B

/**\Ts,T1,T2,T3,TP,T4 setting register for pattern 2*/
#define LED_DRV_PAT2_TIME_SET_TS_T1             0x29
#define LED_DRV_PAT2_TIME_SET_T2_T3             0x2A
#define LED_DRV_PAT2_TIME_SET_TP_T4             0x2B

/**\Ts,T1,T2,T3,TP,T4 setting register for pattern 3*/
#define LED_DRV_PAT3_TIME_SET_TS_T1             0x39
#define LED_DRV_PAT3_TIME_SET_T2_T3             0x3A
#define LED_DRV_PAT3_TIME_SET_TP_T4             0x3B

/**\Color enable register for each Pattern*/
#define LED_DRV_PAT1_COLOR_EN                   0x1C
#define LED_DRV_PAT2_COLOR_EN                   0x2C
#define LED_DRV_PAT3_COLOR_EN                   0x3C

/**\Color cycle times registers for each Pattern*/
#define LED_DRV_PAT1_COLOR_CYC_TIME             0x1D
#define LED_DRV_PAT2_COLOR_CYC_TIME             0x2D
#define LED_DRV_PAT3_COLOR_CYC_TIME             0x3D

/**\NXT registers for each Pattern (Next step/pattern to be executed)*/
#define LED_DRV_PAT1_NXT                        0x1E
#define LED_DRV_PAT2_NXT                        0x2E
#define LED_DRV_PAT3_NXT                        0x3E

/**\Loop times registers for each Pattern*/
#define LED_DRV_PAT1_LOOP_TIMES                 0x1F
#define LED_DRV_PAT2_LOOP_TIMES                 0x2F
#define LED_DRV_PAT3_LOOP_TIMES                 0x3F

/**\Update register for each Pattern*/
#define LED_DRV_PAT1_UPDATE                     0x41
#define LED_DRV_PAT2_UPDATE                     0x42
#define LED_DRV_PAT3_UPDATE                     0x43

/***************************************************************************************************/

/**\Macros to select the Operating Configure Register settings for the led driver.*/
#define LED_DRV_SOFTWARE_SHUTDOWN_MODE          UINT8_C(0)
#define LED_DRV_NORMAL_OPERATION                UINT8_C(1)

#define LED_DRV_SINGLE_MODE                     UINT8_C(3 << 1) // also UINT8_C(0 << 1)
#define LED_DRV_RG_W_MODE                       UINT8_C(1 << 1)
#define LED_DRV_RGB_MODE                        UINT8_C(2 << 1)

#define LED_DRV_CURRENT_LVL_MODE                UINT8_C(0 << 4)
#define LED_DRV_PATTERN_MODE                    UINT8_C(1 << 4)

/**\Macros to select the Output Enable Register settings for the led driver.*/
#define LED_DRV_OUTPUT_1                        UINT8_C(1)
#define LED_DRV_OUTPUT_2                        UINT8_C(1 << 1)
#define LED_DRV_OUTPUT_3                        UINT8_C(1 << 2)

/**\Macros to select the Current Band Register settings for the led driver.*/
#define LED_DRV_OP1_CURRENT_10MA                UINT8_C(0)
#define LED_DRV_OP1_CURRENT_20MA                UINT8_C(1)
#define LED_DRV_OP1_CURRENT_30MA                UINT8_C(2)
#define LED_DRV_OP1_CURRENT_40MA                UINT8_C(3)

#define LED_DRV_OP2_CURRENT_10MA                UINT8_C(0 << 2)
#define LED_DRV_OP2_CURRENT_20MA                UINT8_C(1 << 2)
#define LED_DRV_OP2_CURRENT_30MA                UINT8_C(2 << 2)
#define LED_DRV_OP2_CURRENT_40MA                UINT8_C(3 << 2)

#define LED_DRV_OP3_CURRENT_10MA                UINT8_C(0 << 4)
#define LED_DRV_OP3_CURRENT_20MA                UINT8_C(1 << 4)
#define LED_DRV_OP3_CURRENT_30MA                UINT8_C(2 << 4)
#define LED_DRV_OP3_CURRENT_40MA                UINT8_C(3 << 4)

/**\Macros to select the Hold settings for the led driver.(ONLY IN PATTERN MODE)*/
#define LED_DRV_OP1_HOLD_TIME_T4                UINT8_C(0)
#define LED_DRV_OP1_HOLD_TIME_T2                UINT8_C(1)
#define LED_DRV_OP1_HOLD_OFF                    UINT8_C(0 << 1)
#define LED_DRV_OP1_HOLD_ON                     UINT8_C(1 << 1)

#define LED_DRV_OP2_HOLD_TIME_T4                UINT8_C(0 << 2)
#define LED_DRV_OP2_HOLD_TIME_T2                UINT8_C(1 << 2)
#define LED_DRV_OP2_HOLD_OFF                    UINT8_C(0 << 3)
#define LED_DRV_OP2_HOLD_ON                     UINT8_C(1 << 3)

#define LED_DRV_OP3_HOLD_TIME_T4                UINT8_C(0 << 4)
#define LED_DRV_OP3_HOLD_TIME_T2                UINT8_C(1 << 4)
#define LED_DRV_OP3_HOLD_OFF                    UINT8_C(0 << 5)
#define LED_DRV_OP3_HOLD_ON                     UINT8_C(1 << 5)

/**\Macros for MAX light intensity*/
#define LED_DRV_MAX_INTENSITY                   0xFF

//*********************** Macros for Pattern mode *******************
/**\Macros for time selection*/	
#define TIME_0S_03                              0x00
#define TIME_0S_13                              0x01
#define TIME_0S_26                              0x02
#define TIME_0S_38                              0x03
#define TIME_0S_51                              0x04
#define TIME_0S_77                              0x05
#define TIME_1S_04                              0x06
#define TIME_1S_60                              0x07
#define TIME_2S_10                              0x08
#define TIME_2S_60                              0x09
#define TIME_3S_10                              0x0A
#define TIME_4S_20                              0x0B
#define TIME_5S_20                              0x0C
#define TIME_6S_20                              0x0D
#define TIME_7S_30                              0x0E
#define TIME_8S_30                              0x0F

/**\Macros for pattern cycles*/
#define PAT_CYC_ENDLESS                         0x00
#define PAT_CYC_1_TIME                          0x15
#define PAT_CYC_2_TIMES                         0x2A
#define PAT_CYC_3_TIMES                         0x3F


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
typedef LED_INTF_RET_TYPE (*led_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, uint8_t i2c_address);

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
typedef LED_INTF_RET_TYPE (*led_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, uint8_t i2c_address);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] i2c_address : i2c device address
 *
 */
typedef void (*led_delay_ms_fptr_t)(uint32_t period);

/***************************************************************************************************/

/*!
 * @brief led driver device main configuration structure
 */
struct led_settings
{
    uint8_t operation_mode;
    uint8_t outputs_en;
    uint8_t max_current;
    uint8_t holdfct_en;
};

/*!
 * @brief Led colors enumeration
 */
enum led_rgb_Color{
    RED = 0,
    GREEN,
    BLUE
};

/*!
 * @brief led driver device structure
 */
struct led_dev
{
    /*! Chip Id */
    uint8_t chip_id;

    /*! I2C bq device address*/
    uint8_t i2c_address;

    /*! To store interface pointer error */
    LED_INTF_RET_TYPE intf_rslt;

    /*! Read function pointer */
    led_read_fptr_t read;

    /*! Write function pointer */
    led_write_fptr_t write;

    /*! Delay function pointer */
    led_delay_ms_fptr_t delay_ms;

};

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* LED_DEFS_H_ */
