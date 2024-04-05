/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "coines.h"

/******************************************************************************/
/*! Macro for external temperature sensor - I2C address */

#define EXTERNAL_TEMP_I2C_ADDR     UINT8_C(0x48)
#define EXTERNAL_TEMP_FIFO_ADDR    UINT8_C(0x08) /* FIFO - reg address */
#define EXTERNAL_TEMP_MEAS_ADDR    UINT8_C(0x14) /* Trigger measurement - reg address */
#define EXTERNAL_TEMP_READ_LEN     UINT8_C(0x02) /* burst read length */

/******************************************************************************/
/*! Error codes */

/*! Success code */
#define EXTERNAL_TEMP_SUCCESS                  0
/*! Error code - IO error */
#define EXTERNAL_TEMP_E_COMM_IO_ERROR         -1
/*! Error code - Init failure */
#define EXTERNAL_TEMP_E_COMM_INIT_FAILED      -2
/*! Error code - Device not found */
#define EXTERNAL_TEMP_E_DEVICE_NOT_FOUND      -3
/*! Error code - failure to claim interface */
#define EXTERNAL_TEMP_E_UNABLE_CLAIM_INTF     -4
/*! Error code - Feature not supported */
#define EXTERNAL_TEMP_E_NOT_SUPPORTED         -5
/*! Error code - Null pointer */
#define EXTERNAL_TEMP_E_NULL_PTR              -6

/******************************************************************************/
/*! Macros to define communication interfaces */

/*! Comm interface - SPI */
#define EXTERNAL_TEMP_SPI_INTF               UINT8_C(0)
/*! Comm interface - I2C */
#define EXTERNAL_TEMP_I2C_INTF               UINT8_C(1)

/******************************************************************************/
/*! Function Pointers                                                         */
/******************************************************************************/

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data     : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs.
 *
 *  retval =  0 -> Success
 *  retval != 0 -> Failure
 *
 */
typedef int8_t (*external_temp_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data     : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * retval  = 0 -> Success
 * retval != 0 -> Failure
 *
 */
typedef int8_t (*external_temp_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                                   void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*external_temp_delay_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/* Structure declarations */
/******************************************************************************/
/*!
 * @brief  Structure to store the interface related configurations
 */
struct external_temp_intf_config
{
    uint8_t external_temp_dev_addr;   /* Device address of the interface selected */
    uint8_t external_temp_bus;        /* Bus instance of the interface selected */
};

/*!  @name Structure to define external temperature sensor configurations */
struct external_temp_dev
{
    /*! Read function pointer */
    external_temp_read_fptr_t temp_read;

    /*! Write function pointer */
    external_temp_write_fptr_t temp_write;

    /*!  Delay function pointer */
    external_temp_delay_fptr_t temp_delay_us;
    
    /*! Type of Interface  */
    uint8_t intf;

    /*! Void interface pointer */
    void *intf_ptr;

    /*! To store interface pointer error */
    int8_t intf_rslt;
};

/**********************************************************************************/
/* Function prototype declarations */
/**********************************************************************************/
/*!
 *  @brief Deinitializes COINES_SDK platform
 *
 * @param[in] intf_type      : Communication interface type selection.
 * 
 *  @return void.
 */
int8_t external_temp_coines_init(enum coines_comm_intf intf_type);

/*!
 *  @brief Function to initialize I2C instance for external temperature sensor.
 *
 *  @param[in] temp_dev  : Structure instance of external_temp_dev.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t external_temp_interface_init(struct external_temp_dev *temp_dev);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length       : No of bytes to read.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = 0 -> Success
 *  @retval != 0  -> Failure Info
 *
 */
int8_t external_temp_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data     : Pointer to the data buffer whose value is to be written.
 *  @param[in] length       : No of bytes to write.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return Status of execution
 *  @retval = 0 -> Success
 *  @retval != 0  -> Failure Info
 *
 */
int8_t external_temp_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input 
 *          provided in some of the APIs.
 *
 *  @param[in] period       : The required wait time in microsecond.
 *  @param[in] intf_ptr     : Interface pointer
 *
 *  @return void.
 *
 */
void external_temp_delay_us(uint32_t period, void *intf_ptr);

/*!
 * @brief This API is used to read the external temperature sensor data.
 *
 * @param[in] temp_dev      : Structure instance of external_temp_dev.
 * @param[out] temp_data    : Buffer to retrieve the sensor data.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval Any non zero value -> Fail
 */
int8_t external_temp_get_sensor_data(struct external_temp_dev *temp_dev, float *temp_data);

/*!
 *  @brief Deinitializes COINES_SDK platform
 *
 * @param[in] intf      : Communication interface type selection.
 * 
 *  @return void.
 */
void external_temp_coines_deinit(enum coines_comm_intf intf);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void external_temp_error_codes_print_result(const char api_name[], int8_t rslt);