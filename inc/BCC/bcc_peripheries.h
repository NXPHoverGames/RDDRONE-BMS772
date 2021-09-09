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
 */

/*!
 * File: bcc_peripheries.h
 *
 * This file implements functions for LPSPI and GPIO operations required by BCC
 * driver. Adapted from BCC SW example code version 1.1.
 */

#ifndef BCC_PERIPHERIES_H_
#define BCC_PERIPHERIES_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "BCC/Derivatives/bcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief This function performs one 40b transfer via SPI bus. Intended for SPI
 * mode only. This function needs to be implemented for specified MCU by the
 * user.
 *
 * The byte order of buffers is given by BCC_MSG_BIGEND macro (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param transBuf Pointer to 40b data buffer to be sent.
 * @param recvBuf Pointer to 40b data buffer for received data.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[]);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[], uint16_t recvTrCnt);

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
void BCC_MCU_Assert(bool x);

/*!
 * @brief Writes logic 0 or 1 to the CSB pin (or CSB_TX in case of TPL mode).
 * This function needs to be implemented by the user.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to CSB (CSB_TX) pin.
 */
void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Writes logic 0 or 1 to the RST pin.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to RST pin.
 */
void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value);

/*!
 *      MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 */
uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance);

#endif /* BCC_PERIPHERIES_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
