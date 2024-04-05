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
 * @file       Led.c
 * @date       2022-05-25
 * @version    v0.0.1
*/

/*! @file Led.c
 * @brief Led module driver.
 */

#include "led.h"

/***************** Static function declarations ******************************/

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of led_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval <0 -> Fail
 */
static int8_t null_ptr_check(const struct led_dev *dev);

/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It performs the reading of the chip-id and resets the led driver through I2C.
 */
int8_t led_init(struct led_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;
    struct led_settings led_cfg;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        /* Read the chip-id of led driver */
        rslt = led_read_reg(LED_DRV_ID_REG, &chip_id, 1, dev);

        /* Proceed if everything is fine until now */
        if (rslt == LED_OK)
        {
            /* Check for chip id validity */
            if (chip_id == LED_DRV_PROD_ID)
            {
                dev->chip_id = chip_id;

                /* Reset the sensor */
                rslt = led_device_reset(dev);
                if (rslt == LED_OK)
                {
                    /* Give time to driver to reset */
                    rslt = led_delay_ms(dev,20);

                    //Led driver configuration
                    led_cfg.operation_mode = LED_DRV_NORMAL_OPERATION		//Normal operation mode
                                            | LED_DRV_SINGLE_MODE			//Single mode
                                            | LED_DRV_CURRENT_LVL_MODE;		//Current level mode

                    led_cfg.outputs_en = LED_DRV_OUTPUT_1 					//Enable output 1
                                        | LED_DRV_OUTPUT_2 					//Enable output 2
                                        | LED_DRV_OUTPUT_3 ;				//Enable output 3

                    led_cfg.max_current = LED_DRV_OP1_CURRENT_10MA 			//Max current 10 mA for output 1
                                        | LED_DRV_OP2_CURRENT_10MA			//Max current 10 mA for output 2
                                        | LED_DRV_OP3_CURRENT_10MA;			//Max current 10 mA for output 3

                    led_cfg.holdfct_en = 0; //hold time for each output only in Pattern Mode
                    rslt = led_setconfig(dev, led_cfg); //set led configuration
                }
            }
            else
            {
                rslt = LED_DEV_NOT_FOUND;
            }
        }
    }
    else
    {
        rslt = LED_NULL_PTR;
    }

    return rslt; 
}

/*!
 *  @brief This API push the led driver into shutdown mode.
 */
int8_t led_deinit(struct led_dev *dev)
{
    int8_t rslt = LED_DEV_NOT_FOUND;
    uint8_t readreg=0;
    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        rslt = led_read_reg(LED_DRV_OP_CONF_REG,&readreg,1,dev);
        /* Proceed if everything is fine until now */
        if (rslt == LED_OK)
        {
            readreg |= LED_DRV_SOFTWARE_SHUTDOWN_MODE;
            rslt = led_write_reg(LED_DRV_OP_CONF_REG,&readreg,1,dev);
        }
    }
    else
    {
        rslt = LED_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API sets the essential configuration of the led driver.
 */
int8_t led_setconfig(struct led_dev *dev, struct led_settings led_cfg)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        rslt = led_write_reg(LED_DRV_OP_CONF_REG,&led_cfg.operation_mode,1,dev);
        if (rslt == LED_OK)
        {
            rslt = led_write_reg(LED_DRV_OUT_EN_REG,&led_cfg.outputs_en,1,dev);
            if (rslt == LED_OK)
            {
                rslt = led_write_reg(LED_DRV_CURRENT_REG,&led_cfg.max_current,1,dev);
                if (rslt == LED_OK)
                {
                    rslt = led_write_reg(LED_DRV_HOLD_FCT_REG,&led_cfg.holdfct_en,1,dev);
                }
            }
        }
    }
    else
    {
        rslt = LED_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API performs the reset of the driver.
 */
int8_t led_device_reset(struct led_dev *dev)
{
    uint8_t data_cmd[1];
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        data_cmd[0] = LED_DRV_EXECUTE_CMD;
        rslt = led_write_reg(LED_DRV_RESET,&data_cmd[0],1,dev);
        led_delay_ms(dev,50);
    }
    else
    {
        rslt = LED_NULL_PTR;
    }

    return rslt; 
}

/*!
 * @brief This API update the led driver after writing a new data in a register.
 */
int8_t led_update(struct led_dev *dev)
{
    uint8_t data_cmd[1];
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        data_cmd[0] = LED_DRV_EXECUTE_CMD;
        rslt = led_write_reg(LED_DRV_UPDATE,&data_cmd[0],1,dev);
    }
    else
    {
        rslt = LED_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API is used to select the color and its corresponding intensity.
 */
int8_t led_setcolorandintensity(struct led_dev *dev, enum led_rgb_Color color,uint8_t intensity)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if (rslt == LED_OK)
    {
        switch (color)
        {
        case BLUE:
            rslt = led_write_reg(LED_DRV_OUT1_CURR_LVL,&intensity,1,dev);
            break;

        case GREEN:
            rslt = led_write_reg(LED_DRV_OUT2_CURR_LVL,&intensity,1,dev);
            break;

        case RED:
            rslt = led_write_reg(LED_DRV_OUT3_CURR_LVL,&intensity,1,dev);
            break;

        default:
            break;
        }

        rslt = led_update(dev);
    }
    else
    {
        rslt = LED_NULL_PTR;
    }
    
    return rslt; 
}

/*!
 * @brief This API writes the given data to the register address of the driver.
 */
int8_t led_write_reg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct led_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Check for arguments validity */
    if ((rslt == LED_OK) && (reg_data != NULL))
    {
        if (length != 0)
        {
            dev->intf_rslt = dev->write(reg_addr, reg_data, length, dev->i2c_address);

            /* Check for communication error */
            if (dev->intf_rslt != LED_INTF_RET_SUCCESS)
            {
                rslt = LED_COMM_FAIL;
            }
        }
        else
        {
            rslt = LED_INVALID_LEN;
        }
    }
    else
    {
        rslt = LED_NULL_PTR;
    }
    return rslt;    
}

/*!
 * @brief This API reads the data from the given register address of the driver.
 */
int8_t led_read_reg(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct led_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == LED_OK) && (reg_data != NULL))
    {
        /* Read the data using I2C */
        dev->intf_rslt = dev->read(reg_addr, reg_data, length, dev->i2c_address);

        /* Check for communication error */
        if (dev->intf_rslt != LED_INTF_RET_SUCCESS)
        {
            rslt = LED_COMM_FAIL;
        }
    }
    else
    {
        rslt = LED_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to set a delay to be elapsed.
 */
int8_t led_delay_ms(struct led_dev *dev, uint32_t period)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == LED_OK) 
    {
        dev->delay_ms(period);
    }
    else
    {
        rslt = LED_NULL_PTR;
    }
    return rslt;
}

/*!
 * @brief This API is used to check error/result by giving the result code.
 */ 
const char* led_check_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case LED_OK:
            return "API ended with success\r\n";
            break;
        case LED_NULL_PTR:
            return "Error: Null pointer \r\n";
            break;
        case LED_COMM_FAIL:
            return "Error: Communication failure\r\n";
            break;
        case LED_INVALID_LEN:
            return "Error: Incorrect length parameter\r\n";
            break;
        case LED_DEV_NOT_FOUND:
            return "Error: Device not found\r\n";
            break;
        case LED_CONFIGURATION_ERR:
            return "Error: Configuration Error\r\n";
            break;
        case LED_CMD_EXEC_FAILED:
            return "Error: Command execution failed\r\n";
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
static int8_t null_ptr_check(const struct led_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL) ||
        (dev->i2c_address == 0))
    {
        /* Device structure pointer is not valid */
        rslt = LED_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = LED_OK;
    }

    return rslt;
}
