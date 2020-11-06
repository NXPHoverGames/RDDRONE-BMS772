/****************************************************************************
 * nxp_bms/BMS_v1/inc/sbc.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP 
 * All rights reserved.
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
 **     Filename    : sbc.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-06-24
 **     Abstract    :
 **        sbc module.
 **        This module contains all functions needed for using sbc to configure the power
 **
 ** ###################################################################*/
/*!
 ** @file sbc.h
 **
 ** @version 01.00
 **
 ** @brief
 **        sbc module. this module contains the functions to control the power of the sbc
 **
 */
#ifndef SBC_H_
#define SBC_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * defines
 ******************************************************************************/

/*******************************************************************************
 * types
 ******************************************************************************/
//! this enum is used to set the mode of the SBC
typedef enum 
{
	SBC_SLEEP 	= 1, 	//!< in this mode the SBC will turn off both the CAN tranceiver and the power supply (to MCU)
	SBC_STANDBY = 2, 	//!< in this mode the SBC will turn off the CAN tranceiver, V1 is on
	SBC_NORMAL 	= 3		//!< in this mode the SBC will have the power supply and the CAN tranceicer on
}sbc_mode_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief 	this function is used to initialze the SBC 
 * 			
 * @param 	none
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_initialize(void);

/*!
 * @brief 	This function is used to verify the SBC using SPI 
 * 			It will check the Device identification register (0x7E)
 * 			
 * @param 	None
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_verifySbc(void);

/*!
 * @brief 	this function is used to set the SBC mode 
 * 			
 * @param 	newMode the new mode from the sbc_mode_t enum
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setSbcMode(sbc_mode_t newMode);

/*!
 * @brief 	this function is used to set the CAN FD mode on or off 
 * 			
 * @param 	on if true, CAN FD will be tollerated, if false it will not
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setCANFDMode(bool on);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SBC_H_ */
