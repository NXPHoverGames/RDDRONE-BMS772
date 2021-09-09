/*
 * Copyright 2016 - 2021 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ###################################################################
 **     Filename    : bcc_diag.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K118
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Battery Cell Controller (BCC) module - diagnostic functions.
 **         This module contains functions linked to diagnostics on BCC6 chip.
 **
 ** ###################################################################*/
/*!
 ** @file bcc_diag.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - diagnostic functions.
 **         This module contains functions linked to diagnostics on BCC6 chip. \n
 ** @note
 **         This module was adapted from BCC SW examples by A. Meyer.
 */

#ifndef BCC_DIAG_H_
#define BCC_DIAG_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

 #include "bcc_define.h"                            // Include definitions and external functions

/* Modules */

/* Other */

#include <math.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* ADC1-A and ADC1-B functional verification. */
#define DIAG_ADC1

/* Over-voltage and under-voltage functional verification. */
#define DIAG_CELL_OV_UV

/* CTx open detect. */
#define DIAG_CTX_OPEN

/* Cell voltage channel functional verification. */
#define DIAG_CELL_VOLT

/* Cell terminal leakage diagnostics. */
#define DIAG_CELL_LEAK

/* Current measurement diagnostic. */
#define DIAG_CURRENT_MEAS

/* Verification of the current shunt connection to the current channel low-pass
 * filter. */
#define DIAG_SHUNT_CONN

/* GPIOx OT/UT functional verification. */
#define DIAG_GPIOX_OT_UT

/* GPIOx open terminal diagnostics. */
#define DIAG_GPIOX_OPEN

/* Cell balance open load diagnostics. */
#define DIAG_CBX_OPEN


/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Calculates diagnostic time constants.
 *
 * @param ctFilterComp Pointer to structure with CT filter components.
 * @param isenseFilterComp Pointer to structure with ISENSE filter components.
 * @param diagConst Pointer to structure where diagnostic time constants will
 *                  be stored.
 */
void bcc_diag_calcDiagConst(ct_filter_t *ctFilterComp, isense_filter_t *isenseFilterComp,
    bcc_diag_const_t *diagConst);

/*!
 * Run all diagnostics enabled by DIAG_* macros in diagnostics.h file.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_runDiagnostics(uint8_t cid);

/*!
 * @brief This function performs ADC1-A and ADC1-B functional verification.
 * It uses BCC_Diag_ADC1 function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagAdc1(uint8_t cid);

/*!
 * @brief This function performs over-voltage and under-voltage functional
 * verification. It uses BCC_Diag_OvUv function of the BCC driver.
 *
 * Note: Set Overvoltage and Undervoltage Threshold Registers according to
 * used batteries (e.g. via BCC_INIT_CONF in main.c).
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagOvUv(uint8_t cid);

/*!
 * @brief This function performs CTx open detect diagnostics.
 * It uses BCC_Diag_CTxOpen function of the BCC driver.
 *
 * Note that minimal voltage of a battery cell is 1.5 V.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagCTxOpen(uint8_t cid);

/*!
 * @brief This function performs cell voltage channel functional verification.
 * It uses BCC_Diag_CellVolt function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagCellVolt(uint8_t cid);


/*!
 * @brief This function performs the cell terminal leakage diagnostics.
 * It uses BCC_Diag_CellLeak function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagCellLeak(uint8_t cid);

/*!
 * @brief This function performs the current measurement diagnostics.
 * It uses BCC_Diag_CurrentMeas function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagCurrentMeas(uint8_t cid);

/*!
 * @brief This function performs the verification of current shunt connection.
 * It uses BCC_Diag_ShuntConn function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagShuntConn(uint8_t cid);

/*!
 * @brief This function performs the GPIOx OT/UT functional verification.
 * It uses BCC_Diag_GPIOxOtUt function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagGPIOxOtUt(uint8_t cid);

/*!
 * @brief This function performs the GPIOx open terminal diagnostics.
 * It uses BCC_Diag_GPIOxOpen function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagGPIOxOpen(uint8_t cid);

/*!
 * @brief This function performs the cell balance fault diagnostics.
 * It uses BCC_Diag_CBxOpen function of the BCC driver.
 *
 * @param cid Cluster Identification Address.
 *
 * @return An error provided by the BCC driver.
 */
bcc_status_t bcc_diag_testDiagCBxOpen(uint8_t cid);

/*******************************************************************************
 * EOF
 ******************************************************************************/
#endif /* BCC_DIAG_H_ */
