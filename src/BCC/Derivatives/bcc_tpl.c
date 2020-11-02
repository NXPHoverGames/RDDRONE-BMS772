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
 * @file bcc_tpl.c
 *
 * This file implements low level access functions for SPI communication of BCC
 * driver in TPL mode.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_tpl.h"

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_ReadTpl
 * Description   : This function reads a value from addressed register of
 *                 selected Battery Cell Controller device. Intended for TPL
 *                 mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_ReadTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t const *rxBuf = NULL; /* Pointer to received data. */
    uint8_t regIdx;              /* Index of a received register. */
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
        rc = 0U;
    }

    /* Create frame for request. */
    BCC_PackFrame((uint16_t)regCnt, regAddr, cid, BCC_CMD_READ | rc, txBuf);

    error = BCC_MCU_TransferTpl(drvConfig->drvInstance, txBuf, drvConfig->drvData.rxBuf, regCnt + 1);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Check and store responses. */
    for (regIdx = 0U; regIdx < regCnt; regIdx++)
    {
        /* Pointer to beginning of a frame. */
        rxBuf = (uint8_t *)(drvConfig->drvData.rxBuf + ((1U + regIdx) * BCC_MSG_SIZE));

        error = BCC_CheckCRC(rxBuf);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
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
 * Function Name : BCC_Reg_WriteTpl
 * Description   : This function writes a value to addressed register of
 *                 selected Battery Cell Controller device. Intended for TPL
 *                 mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_WriteTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regVal, uint16_t* retReg)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t const *rxBuf;        /* Pointer to received data. */
    uint8_t rc;                  /* Rolling counter value. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if (((uint8_t)cid > drvConfig->devicesCnt) || (regAddr > BCC_MAX_REG_ADDR))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Calculate Rolling Counter (RC) and increment RC index. */
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

    /* Create frame for writing. */
    BCC_PackFrame(regVal, regAddr, cid, BCC_CMD_WRITE | rc, txBuf);

    error = BCC_MCU_TransferTpl(drvConfig->drvInstance, txBuf, drvConfig->drvData.rxBuf, 2);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Skip an echo frame. */
    rxBuf = (uint8_t *)(drvConfig->drvData.rxBuf + BCC_MSG_SIZE);

    error = BCC_CheckCRC(rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Check Rolling Counter value. */
    if ((*(rxBuf + BCC_MSG_IDX_CID_CMD) & BCC_MSG_RC_MASK) != rc)
    {
        return BCC_STATUS_COM_RC;
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
 * Function Name : BCC_Reg_WriteGlobalTpl
 * Description   : This function writes a value to addressed register of all
 *                 configured BCC devices. Intended for TPL mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_WriteGlobalTpl(bcc_drv_config_t* const drvConfig,
    uint8_t regAddr, uint16_t regVal)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Buffer for sending data via TPL. */

    BCC_MCU_Assert(drvConfig != NULL);

    /* Check input parameters. */
    if (regAddr > BCC_MAX_REG_ADDR)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Create frame for writing. */
    BCC_PackFrame(regVal, regAddr, BCC_CID_UNASSIG, BCC_CMD_GLOB_WRITE, txBuf);

    return BCC_MCU_TransferTpl(drvConfig->drvInstance, txBuf, drvConfig->drvData.rxBuf, 1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_VerifyComTpl
 * Description   : This function uses No Operation command of BCC to verify
 *                 communication without performing any operation. Intended
 *                 for TPL mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_VerifyComTpl(bcc_drv_config_t* const drvConfig, bcc_cid_t cid)
{
    uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
    uint8_t const *rxBuf;        /* Pointer to received data. */
    uint8_t rc;                  /* Rolling counter value. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Calculate Rolling Counter (RC) and increment RC index. */
    rc = (uint8_t)BCC_GET_RC(drvConfig->drvData.rcTbl[(uint8_t)cid - 1U]);
    drvConfig->drvData.rcTbl[(uint8_t)cid - 1U] = BCC_INC_RC_IDX(drvConfig->drvData.rcTbl[(uint8_t)cid - 1U]);

    /* Create frame for writing.
    * Note: Memory Data and Memory Address fields can contain any value. */
    BCC_PackFrame(0x00U, 0x00U, cid, BCC_CMD_NOOP | rc, txBuf);

    error = BCC_MCU_TransferTpl(drvConfig->drvInstance, txBuf, drvConfig->drvData.rxBuf, 2);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Skip an echo frame. */
    rxBuf = (uint8_t *)(drvConfig->drvData.rxBuf + BCC_MSG_SIZE);

    error = BCC_CheckCRC(rxBuf);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Check Rolling Counter value. */
    if ((*(rxBuf + BCC_MSG_IDX_CID_CMD) & BCC_MSG_RC_MASK) != rc)
    {
        return BCC_STATUS_COM_RC;
    }

    return BCC_STATUS_SUCCESS;
}
