/****************************************************************************
 * nxp_bms/BMS_v1/inc/uavcan.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2021 NXP
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
 **     Filename    : uavcan.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-05-23
 **     Abstract    :
 **        uavcan module.
 **        This module contains all functions needed for uavcan
 **
 ** ###################################################################*/
/*!
 ** @file uavcan.h
 **
 ** @version 01.00
 **
 ** @brief
 **        uavcan module. this module contains the functions for uavcan
 **
 */
#ifndef UAVCAN_H_
#define UAVCAN_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include "cli.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define UAVCAN_MAX_SUB_ID       8191
#define UAVCAN_UNSET_SUB_ID     65535
 
/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function initializes the UAVCAN part
 *
 *          It will create the task to check and update the data        
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int uavcan_initialize(void);

/*!
 * @brief   this function will increase the semaphore so the UAVCAN task will send the BMS status using UAVCAN  
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int uavcan_sendBMSStatus(void);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* UAVCAN_H_ */
