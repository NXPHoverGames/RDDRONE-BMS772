/****************************************************************************
 * nxp_bms/BMS_v1/inc/SMBus.h
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
 **     Filename    : SMBus.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2021-05-10
 **     Abstract    :
 **        SMBus module.
 **        This module contains all functions needed for using SMBus (smart battery bus)
 **
 ** ###################################################################*/
/*!
 ** @file SMBus.h
 **
 ** @version 01.00
 **
 ** @brief
 **        SMBus module. this module contains the functions to control the SMBus data
 **
 */
#ifndef SMBUS_H_
#define SMBUS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"
/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the SMBus (smart battery bus)
 *          it will open the device and start the SMBus task
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(SMBus_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int SMBus_initialize(void);

/*!
 * @brief   this function will increase the semaphore so the SMBus
 *          task will update the BMS status of the SBS driver (SMBus)  
 *
 * @param   resetCurrent If this value is true, the current will be set to 0
 *          This could be used if a low power state is used and the SMBus struct 
 *          will not be updated anymore
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *          May be NULL if resetCurrent is true
 *
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
int SMBus_updateInformation(bool resetCurrent, commonBatteryVariables_t *pCommonBatteryVariables,
calcBatteryVariables_t *pCalcBatteryVariables);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SMBUS_H_ */
