/****************************************************************************
 * nxp_bms/BMS_v1/inc/balancing.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2022 NXP
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
 **     Filename    : balancing.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2022-02-18
 **     Abstract    :
 **        balancing module.
 **        This module contains all functions needed for balancing
 **
 ** ###################################################################*/
/*!
 ** @file balancing.h
 **
 ** @version 01.00
 **
 ** @brief
 **        balancing module. this module contains the functions to control the balancing part
 **
 */
#ifndef BALANCING_H_
#define BALANCING_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "data.h"
#include "bcc.h"

/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/
/*! 
 *  @brief  Enumeration to control and check the balancing state.
 */
typedef enum
{
    BALANCE_OFF,
    BALANCE_TO_LOWEST_CELL,
    BALANCE_TO_STORAGE,
    BALANCE_ERROR
}balanceState_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the balancing part
 * 
 * @param   drvConfig the address the BCC driver configuration
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(balancing_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int balancing_initialize(bcc_drv_config_t* const drvConfig);

/*!
 * @brief   this function will set the new balance state. 
 * @note    It can be used to re-start the balancing sequence as well.
 *
 * @param   newBalanceState The new balance state from the balanceState_t enum.
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_setBalanceState(balanceState_t newBalanceState);

/*!
 * @brief   this function will check the balance state. 
 *
 * @param   none
 *
 * @return  The current balancing state from the balanceState_t enum.
 *          If an error occurs, it will return BALANCE_ERROR
 */
balanceState_t balancing_getBalanceState(void);

/*!
 * @brief   this function will initiate the balancing and control/check it.
 * @note    Should be called cyclically after a new measurement.
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   lowestCellVoltage The lowest cell voltage of all cells in V.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_handleCellBalancing(commonBatteryVariables_t *pCommonBatteryVariables,
    float lowestCellVoltage);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BALANCING_H_ */
