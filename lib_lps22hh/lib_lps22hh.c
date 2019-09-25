/***************************************************************************//**
* @file    lib_lps22hh.c
* @version 1.0.0
* @authors STMicroelectronics
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

#include <unistd.h>

#include <applibs/log.h>
#include <applibs/i2c.h>


#include <lib_lps22hh.h>

 /*******************************************************************************
 * Forward declarations of private functions
 *******************************************************************************/

 /**
  * @brief Platform dependent I2C Read function.
  *
  * @param .
  * @param reg_addr Register address to be read from.
  * @param p_data Pointer to buffer for read data.
  * @param data_len Number of bytes to be read.
  *
  * @result The number of bytes successfully read, or -1 for failure.
  */
static ssize_t
i2c_read(int fd_i2c, uint8_t reg_addr, uint8_t *p_data,
    uint32_t data_len);

/**
 * @brief Platform dependent I2C Write function.
 *
 * @param .
 * @param reg_addr Register address to be written to.
 * @param p_data Pointer to data to be transmitted.
 * @param data_len Number of bytes to be transmitted.
 *
 * @result The number of bytes successfully written, or -1 for failure.
 */
static ssize_t
i2c_write(int fd_i2c, uint8_t reg_addr, const uint8_t *p_data,
    uint32_t data_len);




/*******************************************************************************
* Private function definitions
*******************************************************************************/

static ssize_t
i2c_read(int fd_i2c, uint8_t reg_addr, uint8_t *p_data,
    uint32_t data_len)
{
    ssize_t result = -1;

    if (p_mlx && p_data)
    {
#   	ifdef MLX90614_I2C_DEBUG
        DEBUG_DEV(" REG READ [%02X] bytes %d", __FUNCTION__, p_mlx, reg_addr,
            data_len);
#       endif

        // Select register and read its data
        result = I2CMaster_WriteThenRead(fd_i2c, p_mlx->i2c_addr,
            &reg_addr, 1, p_data, data_len);

        if (result == -1)
        {
#   	    ifdef MLX90614_I2C_DEBUG
            DEBUG_DEV("Error %d (%s) on I2C WR operation at addr 0x%02X",
                __FUNCTION__, p_mlx, errno, strerror(errno), p_mlx->i2c_addr);
#           endif
        }
        else
        {
#   	ifdef MLX90614_I2C_DEBUG
            log_printf("MLX %s (0x%02X):  READ ", __FUNCTION__, p_mlx->i2c_addr);
            for (int i = 0; i < data_len; i++)
            {
                log_printf("%02X ", p_data[i]);
            }
            log_printf("\n");
#           endif
        }
    }

    // Return length of read data only
    return result - 1;
}

static ssize_t
i2c_write(int fd_i2c, uint8_t reg_addr, const uint8_t *p_data,
    uint32_t data_len)
{
    ssize_t result = -1;

    if (p_mlx && p_data)
    {
#   	ifdef MLX90614_I2C_DEBUG
        DEBUG_DEV(" REG WRITE [%02X] bytes %d", __FUNCTION__, p_mlx, reg_addr,
            data_len);
#       endif

        uint8_t buffer[data_len + 1];

        buffer[0] = reg_addr;
        for (uint32_t i = 0; i < data_len; i++)
        {
            buffer[i + 1] = p_data[i];
        }

#		ifdef MLX90614_I2C_DEBUG
        log_printf("MLX %s (0x%02X):  WRITE ", __FUNCTION__, p_mlx->i2c_addr);
        for (int i = 0; i < data_len; i++)
        {
            log_printf("%02X ", p_data[i]);
        }
        log_printf("\n");
#		endif

        // Select register and write data
        result = I2CMaster_Write(p_mlx->i2c_fd, p_mlx->i2c_addr, buffer,
            data_len + 1);

        if (result == -1)
        {
#		    ifdef MLX90614_I2C_DEBUG
            DEBUG_DEV("Error %d (%s) on writing %d byte(s) to I2C addr 0x%02X",
                __FUNCTION__, p_mlx, errno, strerror(errno), data_len + 1,
                p_mlx->i2c_addr);
#           endif
        }
    }

    return result;
}