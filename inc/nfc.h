/****************************************************************************
 * nxp_bms/BMS_v1/inc/nfc.h
 *
 * BSD 3-Clause License
 *
 * Copyright 2020-2023 NXP
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
 **     Filename    : nfc.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-08-25
 **     Abstract    :
 **        nfc module.
 **        This module contains all functions needed for using nfc
 **
 ** ###################################################################*/
/*!
 ** @file nfc.h
 **
 ** @version 01.00
 **
 ** @brief
 **        nfc module. this module contains the functions to control the NTAG5
 **
 */
#ifndef NFC_H_
#define NFC_H_

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
 * @brief   This function will initialze the NFC
 *          it will test the i2C connection with the chip and read the slave address
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(nfc_initialize(false))
 *          {
 *            // do something with the error
 *          }
 */
int nfc_initialize(bool skipSelfTest);

/*!
 * @brief   This function can be used to set the hard power-down (HPD) mode of the NFC
 *
 * @param   HPD if true, the microcontroller will set the NFC chip in hard power-down mode.
 *          if false, it will disable this mode.
 * @return  0 if ok, -1 if there is an error
 * @example
 *          if(nfc_setHPD())
 *          {
 *              // do something with the error
 *          }
 */
int nfc_setHPD(bool HPD);

/*!
 * @brief   This function can be used to update the BMS parameter to the NTAG
 * @note  Blocking
 *
 * @param   setOutdatedText If true it will set the text "outdated, please tap again!",
 *              If false, it will update the NTAG with the latest BMS data.
 * @param   wakingUpMessage If true it will set the waking up text (for sleep mode)
 *              If false, it will set the charge-relaxation string.
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t to update the information
 *              May be NULL if setOutdatedText or wakingUpMessage is true
 * @param   pCalcBatteryVariables pointer to the calcBatteryVariables_t to update the information
 *              May be NULL if setOutdatedText or wakingUpMessage is true
 *
 * @return  0 if ok, -1 if there is an error
 */
int nfc_updateBMSStatus(bool setOutdatedText, bool wakingUpMessage,
    commonBatteryVariables_t *pCommonBatteryVariables, calcBatteryVariables_t *pCalcBatteryVariables);

/*!
 * @brief   This function can be used to disable the NFC.
 *          Calling nfc_updateBMSStatus() will just return 0.
 * @note    It will be placed in HPD mode.
 *
 * @param   disable, true if it needs to be disabled
 * @example
 *          if(nfc_disableNFC(true))
 *          {
 *              // do something with the error
 *          }
 */
int nfc_disableNFC(bool disable);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* NFC_H_ */
