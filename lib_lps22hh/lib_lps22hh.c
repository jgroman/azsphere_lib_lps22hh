/***************************************************************************//**
* @file    lib_lps22hh.c
* @version 1.0.0
* @authors STMicroelectronics
* https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm6dso_STdC/driver
* @authors AVNet developers
* @authors Jaroslav Groman
*
* @par Project Name
*
*
* @par Description
*    .
*
* @par Target device
*    Azure Sphere MT3620
*
* @par Related hardware
*    Avnet Azure Sphere Starter Kit
*
* @par Code Tested With
*    1. Silicon: Avnet Azure Sphere Starter Kit
*    2. IDE: Visual Studio 2017
*    3. SDK: Azure Sphere SDK Preview
*
* @par Notes
*    .
*
*******************************************************************************/

/*
 * Some of the code in this file was copied from ST Micro.
 * Below is their licence information.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "lsm6dso_reg.h"

#include "lib_lps22hh.h"

 /*******************************************************************************
 * Forward declarations of private functions
 *******************************************************************************/

 /**
  * @brief Platform dependent I2C Read function.
  *
  * @param fd_i2c I2C interface file descriptor.
  * @param i2c_addr I2C device address.
  * @param reg_addr Register address to be read from.
  * @param p_data Pointer to buffer for read data.
  * @param data_len Number of bytes to be read.
  *
  * @result 0 for success or -1 for failure.
  */
static ssize_t
i2c_read(int fd_i2c, I2C_DeviceAddress i2c_addr, uint8_t reg_addr,
    uint8_t *p_data, uint32_t data_len);

/**
 * @brief Platform dependent I2C Write function.
 *
 * @param fd_i2c I2C interface file descriptor.
 * @param i2c_addr I2C device address.
 * @param reg_addr Register address to be written to.
 * @param p_data Pointer to data to be transmitted.
 * @param data_len Number of bytes to be transmitted.
 *
 * @result 0 for success or -1 for failure.
 */
static ssize_t
i2c_write(int fd_i2c, I2C_DeviceAddress i2c_addr, uint8_t reg_addr,
    const uint8_t *p_data, uint32_t data_len);

/**
 * @brief Platform dependent log print function.
 *
 * @param p_format The message string to log.
 * @param ... Argument list.
 *
 * @result 0 for success, or -1 for failure, in which case errno is set
 * to the error value.
 */
static int
log_printf(const char *p_format, ...);

static int32_t
lsm6dso_i2c_write(void *p_fd_i2c, uint8_t reg_addr,
    uint8_t *p_data, uint16_t data_len);

static int32_t
lsm6dso_i2c_read(void *p_fd_i2c, uint8_t reg_addr,
    uint8_t *p_data, uint16_t data_len);

static void
delay_ms(int delay_ms);

static int32_t
lps22hh_read_via_lsm6dso(void *ctx, uint8_t reg, uint8_t *data, uint16_t len);

static int32_t
lps22hh_write_via_lsm6dso(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);

/*******************************************************************************
* Global variables
*******************************************************************************/

int g_fd_i2c = -1;
stmdev_ctx_t g_lsm6dso_ctx;
stmdev_ctx_t g_lps22hh_ctx;

/*******************************************************************************
* Public function definitions
*******************************************************************************/

int
lps22hh_open_via_lsm6dso(int fd_i2c)
{
    int result = -1;
    
    uint8_t device_id;
    uint8_t device_reset_flag;

    g_fd_i2c = fd_i2c;
    
    g_lsm6dso_ctx.write_reg = lsm6dso_i2c_write;
    g_lsm6dso_ctx.read_reg = lsm6dso_i2c_read;
    g_lsm6dso_ctx.handle = &g_fd_i2c;

    g_lps22hh_ctx.read_reg = lps22hh_read_via_lsm6dso;
    g_lps22hh_ctx.write_reg = lps22hh_write_via_lsm6dso;
    g_lps22hh_ctx.handle = &g_fd_i2c;

    // Force switch LSM6DSO to standard register set which includes WHO_AM_I
    lsm6dso_mem_bank_set(&g_lsm6dso_ctx, LSM6DSO_USER_BANK);

    lsm6dso_device_id_get(&g_lsm6dso_ctx, &device_id);
    if (device_id != LSM6DSO_ID)
    {
        ERROR("LSM6DSO hub device not found.", __FUNCTION__);
        result = -1;
    }
    else
    {
        DEBUG("LSM6DSO hub device detected.", __FUNCTION__);
        result = 0;

        // Restore default configuration
        lsm6dso_reset_set(&g_lsm6dso_ctx, PROPERTY_ENABLE);
        do 
        {
            result = lsm6dso_reset_get(&g_lsm6dso_ctx, &device_reset_flag);
        } 
        while (device_reset_flag);

        // Disable I3C interface
        result = lsm6dso_i3c_disable_set(&g_lsm6dso_ctx, LSM6DSO_I3C_DISABLE);

        // Enable Block Data Update
        lsm6dso_block_data_update_set(&g_lsm6dso_ctx, PROPERTY_ENABLE);
    }

    if (result != -1)
    {
        // Enable internal pull up resistors on master I2C interface
        lsm6dso_sh_pin_mode_set(&g_lsm6dso_ctx, LSM6DSO_INTERNAL_PULL_UP);

        // Check for LPS22HH connected to LSM6DSO Sensor Hub
        lps22hh_device_id_get(&g_lps22hh_ctx, &device_id);
        if (device_id != LPS22HH_ID)
        {
            ERROR("LPS22HH sensor not found.", __FUNCTION__);
            result = -1;
        }
        else
        {
            DEBUG("LPS22HH sensor detected.", __FUNCTION__);
            result = 0;

            // Restore the default configuration
            lps22hh_reset_set(&g_lps22hh_ctx, PROPERTY_ENABLE);
            do 
            {
                lps22hh_reset_get(&g_lps22hh_ctx, &device_reset_flag);
            } 
            while (device_reset_flag);

            // Enable Block Data Update
            lps22hh_block_data_update_set(&g_lps22hh_ctx, PROPERTY_ENABLE);

            // Set Output Data Rate
            lps22hh_data_rate_set(&g_lps22hh_ctx, LPS22HH_10_Hz_LOW_NOISE);
        }
    }

    return result;
}

/*******************************************************************************
* Private function definitions
*******************************************************************************/

static void
delay_ms(int delay_ms)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = delay_ms * 10000;
    nanosleep(&ts, NULL);
    return;
}

static int32_t 
lps22hh_write_via_lsm6dso(void* ctx, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t buf_raw[32];
    int32_t ret;
    uint8_t drdy;
    lsm6dso_status_master_t master_status;
    lsm6dso_sh_cfg_write_t sh_cfg_write;

    // Configure Sensor Hub to write to the LPS22HH, and send the write data
    sh_cfg_write.slv0_add = (LPS22HH_I2C_ADD_L & 0xFEU) >> 1; // 7bit I2C addr
    sh_cfg_write.slv0_subadd = reg;
    sh_cfg_write.slv0_data = *data;
    
    ret = lsm6dso_sh_cfg_write(&g_lsm6dso_ctx, &sh_cfg_write);

    // Disable accelerometer
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_OFF);

    // Enable I2C Master
    lsm6dso_sh_master_set(&g_lsm6dso_ctx, PROPERTY_ENABLE);

    // Enable accelerometer to trigger Sensor Hub operation
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_104Hz);

    // Read and ignore accelerometer data
    lsm6dso_acceleration_raw_get(&g_lsm6dso_ctx, buf_raw);

    // Wait for accelerometer data transfer to complete
    do
    {
        delay_ms(20);
        lsm6dso_xl_flag_data_ready_get(&g_lsm6dso_ctx, &drdy);
    } 
    while (!drdy);

    // Wait for I²C master transfer to complete
    do
    {
        delay_ms(20);
        lsm6dso_sh_status_get(&g_lsm6dso_ctx, &master_status);
    } 
    while (!master_status.sens_hub_endop);

    // Disable I2C master and accelerometer
    lsm6dso_sh_master_set(&g_lsm6dso_ctx, PROPERTY_DISABLE);
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_OFF);

#   ifdef LPS22HH_I2C_DEBUG
    log_printf("LPS %s:  REG (0x%02X) WRITE (%d bytes): ", __FUNCTION__, reg, len);
    for (int i = 0; i < len; i++)
    {
        log_printf("%02X ", data[i]);
    }
    log_printf("\n");
#   endif   // LPS22HH_I2C_DEBUG

    return ret;
}

static int32_t
lps22hh_read_via_lsm6dso(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    lsm6dso_sh_cfg_read_t sh_cfg_read;
    uint8_t buf_raw[6];
    int32_t ret;
    uint8_t drdy;
    lsm6dso_status_master_t master_status;

    // Disable accelerometer
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_OFF);

    // Configure Sensor Hub to read LPS22HH
    sh_cfg_read.slv_add = (LPS22HH_I2C_ADD_L & 0xFEU) >> 1; // 7bit I2C address
    sh_cfg_read.slv_subadd = reg;
    sh_cfg_read.slv_len = (uint8_t)len;

    ret = lsm6dso_sh_slv0_cfg_read(&g_lsm6dso_ctx, &sh_cfg_read);

    // WRITE_ONCE is mandatory for read
    //lsm6dso_sh_write_mode_set(&g_lsm6dso_ctx, PROPERTY_ENABLE);

    // Using slave 0 only
    lsm6dso_sh_slave_connected_set(&g_lsm6dso_ctx, LSM6DSO_SLV_0);

    // I2C master enable
    lsm6dso_sh_master_set(&g_lsm6dso_ctx, PROPERTY_ENABLE);

    // Enable accelerometer to trigger Sensor Hub operation
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_104Hz);

    // Read and ignore accelerometer data
    lsm6dso_acceleration_raw_get(&g_lsm6dso_ctx, buf_raw);

    // Wait for accelerometer data transfer to complete
    do 
    {
        delay_ms(20);
        lsm6dso_xl_flag_data_ready_get(&g_lsm6dso_ctx, &drdy);
    } 
    while (!drdy);

    // Wait for I2C master transfer to complete
    do
    {
        lsm6dso_sh_status_get(&g_lsm6dso_ctx, &master_status);
    } 
    while (!master_status.sens_hub_endop);

    // Disable I2C master and accelerometer
    lsm6dso_sh_master_set(&g_lsm6dso_ctx, PROPERTY_DISABLE);
    lsm6dso_xl_data_rate_set(&g_lsm6dso_ctx, LSM6DSO_XL_ODR_OFF);

    // Read SensorHub registers
    lsm6dso_sh_read_data_raw_get(&g_lsm6dso_ctx, data, (uint8_t)len);

#   ifdef LPS22HH_I2C_DEBUG
    log_printf("LPS %s:  REG (0x%02X) READ (%d bytes): ", __FUNCTION__, reg, len);
    for (int i = 0; i < len; i++)
    {
        log_printf("%02X ", data[i]);
    }
    log_printf("\n");
#   endif   // LPS22HH_I2C_DEBUG

    return ret;
}

static int32_t
lsm6dso_i2c_write(void *p_fd_i2c, uint8_t reg_addr, 
    uint8_t *p_data, uint16_t data_len)
{
    return i2c_write(*(int *)p_fd_i2c, LSM6DSO_HUB_I2C_ADDR, reg_addr,
        p_data, (uint32_t)data_len);
}

static int32_t
lsm6dso_i2c_read(void *p_fd_i2c, uint8_t reg_addr,
    uint8_t *p_data, uint16_t data_len)
{
    return i2c_read(*(int *)p_fd_i2c, LSM6DSO_HUB_I2C_ADDR, reg_addr,
        p_data, (uint32_t)data_len);
}

static ssize_t
i2c_read(int fd_i2c, I2C_DeviceAddress i2c_addr, uint8_t reg_addr, 
    uint8_t *p_data, uint32_t data_len)
{
    ssize_t result = -1;

    if (p_data)
    {
#   	ifdef LPS22HH_I2C_DEBUG
        DEBUG("(0x%02X): REG READ [%02X] bytes %d", __FUNCTION__, i2c_addr, 
            reg_addr, data_len);
#       endif   // LPS22HH_I2C_DEBUG

        result = I2CMaster_WriteThenRead(fd_i2c, i2c_addr,
            &reg_addr, 1, p_data, data_len);

        if (result == -1)
        {
#   	    ifdef LPS22HH_DEBUG
            DEBUG("Error %s (%d) on I2C WR operation at addr 0x%02X",
                __FUNCTION__, strerror(errno), errno, i2c_addr);
#           endif   // LPS22HH_DEBUG
        }
        else
        {
#   	    ifdef LPS22HH_I2C_DEBUG
            log_printf("Hub %s (0x%02X):  READ ", __FUNCTION__, i2c_addr);
            for (int i = 0; i < data_len; i++)
            {
                log_printf("%02X ", p_data[i]);
            }
            log_printf("\n");
#           endif   // LPS22HH_I2C_DEBUG
        }
    }

    if (result > 0)
    {
        // STM driver expects only -1 or 0 return values
        result = 0;
    }

    return result;
}

static ssize_t
i2c_write(int fd_i2c, I2C_DeviceAddress i2c_addr, uint8_t reg_addr, 
    const uint8_t *p_data, uint32_t data_len)
{
    ssize_t result = -1;

    if (p_data)
    {
#   	ifdef LPS22HH_I2C_DEBUG
        DEBUG("(0x%02X): REG WRITE [%02X] bytes %d", __FUNCTION__, i2c_addr, 
            reg_addr, data_len);
#       endif   // LPS22HH_I2C_DEBUG

        uint8_t buffer[data_len + 1];

        buffer[0] = reg_addr;
        for (uint32_t i = 0; i < data_len; i++)
        {
            buffer[i + 1] = p_data[i];
        }

#		ifdef LPS22HH_I2C_DEBUG
        log_printf("Hub %s (0x%02X):  WRITE ", __FUNCTION__, i2c_addr);
        for (int i = 0; i < data_len; i++)
        {
            log_printf("%02X ", p_data[i]);
        }
        log_printf("\n");
#		endif   // LPS22HH_I2C_DEBUG

        // Select register and write data
        result = I2CMaster_Write(fd_i2c, i2c_addr, buffer, data_len + 1);

        if (result == -1)
        {
#		    ifdef LPS22HH_DEBUG
            DEBUG("Error %s (%d) writing %d byte(s) to I2C addr 0x%02X",
                __FUNCTION__, strerror(errno), errno, data_len + 1, i2c_addr);
#           endif   // LPS22HH_DEBUG
        }
    }

    if (result > 0)
    {
        // STM driver expects only -1 or 0 return values
        result = 0;
    }

    return result;
}

static int
log_printf(const char *p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    int result = Log_DebugVarArgs(p_format, args);
    va_end(args);

    return result;
}

/* [] END OF FILE */
