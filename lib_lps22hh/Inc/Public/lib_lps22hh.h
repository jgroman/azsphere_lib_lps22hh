/***************************************************************************//**
* @file    lib_lps22hh.h
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


#ifndef LIB_LPS22HH_H
#define LIB_LPS22HH_H

#ifdef __cplusplus
extern "C" {
#endif

// Uncomment line below to enable debugging messages
#define LPS22HH_DEBUG

// Uncomment line below to enable I2C debugging messages
//#define LPS22HH_I2C_DEBUG


#ifdef LPS22HH_DEBUG
#define DEBUG(s, f, ...) log_printf("%s %s: " s "\n", "LPS22HH", f, \
                                                                 ## __VA_ARGS__)
#else
#define DEBUG(s, f, ...)
#endif // LPS22HH_DEBUG

#define ERROR(s, f, ...) log_printf("%s %s: " s "\n", "LPS22HH", f, \
                                                                ## __VA_ARGS__)

/*******************************************************************************
*   Included Headers
*******************************************************************************/

#include "lsm6dso_reg.h"
#include "lps22hh_reg.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define LSM6DSO_HUB_I2C_ADDR     (0x6A)

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

extern stmdev_ctx_t g_lps22hh_ctx;

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

int
lps22hh_open_via_lsm6dso(int fd_i2c);

#ifdef __cplusplus
}
#endif

#endif  // LIB_LPS22HH_H

/* [] END OF FILE */
