/***************************************************************************//**
* @file    lib_lps22hh.h
* @version 1.0.0
* @authors STMicroelectronics
*          https://github.com/STMicroelectronics/STMems_Standard_C_drivers/
*
* @authors AVNet developers
* @authors Jaroslav Groman
*
* @par Project Name
*      LPS22HH sensor support library for Azure Sphere
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
 * Parts of code are Copyright (c) STMicroelectronics. 
 * Below is their licence information.
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 */

#ifndef LIB_LPS22HH_H
#define LIB_LPS22HH_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
*   Included Headers
*******************************************************************************/

#include "lps22hh_reg.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define LSM6DSO_HUB_I2C_ADDR     (0x6A)

/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

stmdev_ctx_t
*lps22hh_open_via_lsm6dso(int fd_i2c);

void
lps22hh_close(stmdev_ctx_t *p_lps);

#ifdef __cplusplus
}
#endif

#endif  // LIB_LPS22HH_H

/* [] END OF FILE */
