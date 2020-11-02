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
 * @file bcc_spi.c
 *
 * This file implements low level access functions for SPI communication of BCC
 * driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_spi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Returns true when a response message is equal to zero except
 * CRC field.
 *
 * @param resp Response message to be checked.
 * @return True when the response is zero (except CRC), false otherwise.
 */
#define BCC_IS_NULL_RESP(resp) \
    (((resp)[BCC_MSG_IDX_DATA_H] == 0U) && \
     ((resp)[BCC_MSG_IDX_DATA_L] == 0U) && \
     ((resp)[BCC_MSG_IDX_ADDR] == 0U) && \
     ((resp)[BCC_MSG_IDX_CID_CMD] == 0U))

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_ReadSpi
 * Description   : This function reads a value from addressed register of
 *                 selected Battery Cell Controller device. Intended for SPI
 *                 mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_ReadSpi(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t rxBuf[BCC_MSG_SIZE]; /* Buffer for receiving. */
    uint8_t regIdx;              /* Index of a register. */
    uint8_t rc;                  /* Rolling Counter value. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(regVal != NULL);

    if (((uint8_t)cid > drvConfig->devicesCnt) || (regAddr > BCC_MAX_REG_ADDR) ||
        (regCnt == 0U) || ((regAddr + regCnt - 1U) > BCC_MAX_REG_ADDR))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Calculate Rolling Counter (RC) value and increment RC index. */
    if (cid != BCC_CID_UNASSIG)
    {
        /* RC is not intended for global messages. */
        rc = (uint8_t)BCC_GET_RC(drvConfig->drvData.rcTbl[(uint8_t)cid - 1U]);
        drvConfig->drvData.rcTbl[(uint8_t)cid - 1U] = BCC_INC_RC_IDX(drvConfig->drvData.rcTbl[(uint8_t)cid - 1U]);
    }
    else
    {
        rc = 0;
    }

    /* Create frame for request. */
    BCC_PackFrame((uint16_t)1U, regAddr, cid, BCC_CMD_READ | rc, txBuf);

    /* Send request for data. Required data are returned with the following transfer. */
    error = BCC_MCU_TransferSpi(drvConfig->drvInstance, txBuf, rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Check CRC and discard the response. */
    if ((error = BCC_CheckCRC(rxBuf)) != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Read required data. */
    for (regIdx = 0U; regIdx < regCnt; regIdx++)
    {
        /* Increment register address. */
        regAddr++;
        if (regAddr > 0x7FU)
        {
            regAddr = 0x00U;
        }

        BCC_PackFrame((uint16_t)1U, regAddr, cid, BCC_CMD_READ | rc, txBuf);

        /* Send request for data. Required data are returned with the following transfer. */
        error = BCC_MCU_TransferSpi(drvConfig->drvInstance, txBuf, rxBuf);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        error = BCC_CheckCRC(rxBuf);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        if (BCC_IS_NULL_RESP(rxBuf))
        {
            return BCC_STATUS_NULL_RESP;
        }

        if (cid != BCC_CID_UNASSIG)
        {
            /* RC and TAG ID are not intended for global messages. */
            error = BCC_CheckRcTagId(drvConfig->device[(uint8_t)cid - 1U], rxBuf, rc,
                                     drvConfig->drvData.tagId[(uint8_t)cid - 1U]);
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
        }

        /* Store data. */
        *(regVal + regIdx) = BCC_GET_MSG_DATA(rxBuf);
    }

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_WriteSpi
 * Description   : This function writes a value to addressed register of
 *                 selected Battery Cell Controller device. Intended for SPI
 *                 mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_WriteSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t regAddr, uint16_t regVal, uint16_t* retReg)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t rxBuf[BCC_MSG_SIZE]; /* Buffer for receiving. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if (((uint8_t)cid > drvConfig->devicesCnt) || (regAddr > BCC_MAX_REG_ADDR))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Create frame for writing. */
    BCC_PackFrame(regVal, regAddr, cid, BCC_CMD_WRITE, txBuf);

    error = BCC_MCU_TransferSpi(drvConfig->drvInstance, txBuf, rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    error = BCC_CheckCRC(rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Store content of received frame. */
    if (retReg != NULL)
    {
        *retReg = BCC_GET_MSG_DATA(rxBuf);
    }

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_VerifyComSpi
 * Description   : This function uses No Operation command of BCC to verify
 *                 communication without performing any operation. Intended
 *                 for SPI mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_VerifyComSpi(const bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t rxBuf[BCC_MSG_SIZE]; /* Buffer for receiving. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Create frame for writing.
    * Note: Memory Data and Memory Address fields can contain any value. */
    BCC_PackFrame(0x00U, 0x00U, cid, BCC_CMD_NOOP, txBuf);

    error = BCC_MCU_TransferSpi(drvConfig->drvInstance, txBuf, rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    return BCC_CheckCRC(rxBuf);
}
