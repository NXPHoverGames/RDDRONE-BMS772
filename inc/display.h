/****************************************************************************
 * nxp_bms/BMS_v1/inc/display.h
 *
 * BSD 3-Clause License
 *
 * Copyright 2021-2022 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ** ###################################################################
 **     Filename    : display.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-19
 **     Abstract    :
 **        display module.
 **        This module contains all functions needed for MCU display
 **
 ** ###################################################################*/
/*!
 ** @file display.h
 **
 ** @version 01.00
 **
 ** @brief
 **        display module. this module contains the functions for MCU display
 **
 */
#ifndef DISPLAY_H
#define DISPLAY_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to initialize the display part
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_initialize(bool skipSelfTest);

/*!
 * @brief   This function is used to uninitialize the display part
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_uninitialize(void);

/*!
 * @brief   This function is used to turn on or off the power to the display
 *
 * @param   on If true, it will turn on the display, false otherwise.
 *
 * @return  0 if ok, -1 if there is an error
 */
int display_setPower(bool on);

/*!
 * @brief   This function is used to get the state of the display, on or off.
 *
 * @param   none.
 *
 * @return  true if on, false otherwise.
 */
bool display_getPower(void);

/*!
 * @brief   This function is used to update the values of the display with the actual information
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int display_updateValues(
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* DISPLAY_H */
