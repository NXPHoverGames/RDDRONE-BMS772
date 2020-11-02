/*
 * Copyright 2016 - 2019 NXP
 * All rights reserved.
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
 * @file bcc_spi.h
 *
 * This file provides access to the functions for SPI communication of BCC driver
 * in SPI mode.
 */

#ifndef __BCC_SPI_H
#define __BCC_SPI_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_communication.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function reads a value from addressed register of selected
 * Battery Cell Controller device. Intended for SPI mode only.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regCnt Number of registers to read.
 * @param regVal Pointer to memory where content of selected 16 bit registers
 *               is stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_ReadSpi(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal);

/*!
 * @brief This function writes a value to addressed register of selected
 * Battery Cell Controller device. Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 * @param retReg Automatic response of BCC, which contains register addressed
 *               in previous access. You can pass NULL when you do not care
 *               about the response.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t regAddr, uint16_t regVal, uint16_t* retReg);

/*!
 * @brief This function uses No Operation command of BCC to verify communication
 * without performing any operation. Intended for SPI mode only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_VerifyComSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid);
/*! @} */

#endif /* __BCC_SPI_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
