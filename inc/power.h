/****************************************************************************
 * nxp_bms/BMS_v1/inc/power.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2021 NXP
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
 **     Filename    : power.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-19
 **     Abstract    :
 **        power module.
 **        This module contains all functions needed for MCU power
 **
 ** ###################################################################*/
/*!
 ** @file power.h
 **
 ** @version 01.00
 **
 ** @brief
 **        power module. this module contains the functions for MCU power
 **
 */
#ifndef POWER_H
#define POWER_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Types
 ******************************************************************************/
//! @brief this enum could be used to set or get the MCU power mode. 
typedef enum{
    RUN_MODE,       // In this mode the MCU will be in RUN mode at 80MHz with all the peripherals enabled
    STANDBY_MODE,   // In this mode the MCU will be in VLPR mode at 2MHz with the SPI and CLI enabled 
    VLPR_MODE,      // In this mode the MCU will be in VLPR mode at 2MHz with the CLI enabled 
    ERROR_VALUE     // This will be returned if there is an error.      
}mcuPowerModes_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to initialize the power part  
 *
 * @return  0 if ok, -1 if there is an error
 */
int power_initialize(void);

/*!
 * @brief   Function that will set or get the MCU power mode
 * @note    Could be called from multiple threads
 *
 * @param   setNotGet if true it is used to set the power mode, false to get it
 * @param   newValue if setNotGet is true, this is the new power mode, could be ERROR_VALUE otherwise
 *
 * @return  the MCU power mode from the mcuPowerModes_t enum, ERROR_VALUE if error
 */
mcuPowerModes_t power_setNGetMcuPowerMode(bool setNotGet, mcuPowerModes_t newValue);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* POWER_H */
