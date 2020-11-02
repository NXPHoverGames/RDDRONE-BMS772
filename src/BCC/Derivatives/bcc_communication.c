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
 * @file bcc_communication.c
 *
 * This file implements functions for SPI communication of BCC driver used in
 * both SPI and TPL mode.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_communication.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Size of CRC table. */
#define BCC_CRC_TBL_SIZE    256U

/*!
 * @brief This macro determines format of a MC33771 frame with use of register
 * address. Following registers only use TAG ID (reading): SYS_DIAG (5),
 * FAULT1_STATUS (24), FAULT2_STATUS (25), FAULT3_STATUS (26) and from
 * CC_NB_SAMPLES (2D) to MEAS_ VBG_DIAG_ADC1B (4A).
 *
 * @param regAddr Address of a register (typically extracted from received
 *                frame).
 *
 * @return True if format of frame with selected register address use TAG ID,
 *         false otherwise.
 */
#define BCC_HAS_TAG_ID_MC33771(regAddr) \
  (((regAddr) == BCC_REG_SYS_DIAG_ADDR) || \
   ((regAddr) == BCC_REG_FAULT1_STATUS_ADDR) || \
   ((regAddr) == BCC_REG_FAULT2_STATUS_ADDR) || \
   ((regAddr) == BCC_REG_FAULT3_STATUS_ADDR) || \
   (((regAddr) >= BCC_REG_CC_NB_SAMPLES_ADDR) && \
   ((regAddr) <= BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR)) \
  )

/*!
 * @brief This macro determines format of a MC33772 frame with use of register
 *  address. Following registers only use TAG ID (reading): SYS_DIAG (5),
 * FAULT1_STATUS (24), FAULT2_STATUS (25), FAULT3_STATUS (26), from
 * CC_NB_SAMPLES (2D) to MEAS_STACK (32) and from MEAS_CELL7 (3A) to
 * MEAS_VBG_DIAG_ADC1B (4A).
 *
 * @param regAddr Address of a register (typically extracted from received
 *                frame).
 *
 * @return True if format of frame with selected register address use TAG ID,
 *         false otherwise.
 */
#define BCC_HAS_TAG_ID_MC33772(regAddr) \
  (((regAddr) == BCC_REG_SYS_DIAG_ADDR) || \
   ((regAddr) == BCC_REG_FAULT1_STATUS_ADDR) || \
   ((regAddr) == BCC_REG_FAULT2_STATUS_ADDR) || \
   ((regAddr) == BCC_REG_FAULT3_STATUS_ADDR) || \
   (((regAddr) >= BCC_REG_CC_NB_SAMPLES_ADDR) && \
   ((regAddr) <= BCC_REG_MEAS_STACK_ADDR)) || \
   (((regAddr) >= BCC_REG_MEAS_CELLX_ADDR_MC33772_START) && \
   ((regAddr) <= BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR)) \
  )

/*******************************************************************************
 * Constants and global variables
 ******************************************************************************/

/* Table with precalculated CRC values. */
static const uint8_t BCC_CRC_TABLE[BCC_CRC_TBL_SIZE] = {
    0x00U, 0x2fU, 0x5eU, 0x71U, 0xbcU, 0x93U, 0xe2U, 0xcdU,
    0x57U, 0x78U, 0x09U, 0x26U, 0xebU, 0xc4U, 0xb5U, 0x9aU,
    0xaeU, 0x81U, 0xf0U, 0xdfU, 0x12U, 0x3dU, 0x4cU, 0x63U,
    0xf9U, 0xd6U, 0xa7U, 0x88U, 0x45U, 0x6aU, 0x1bU, 0x34U,
    0x73U, 0x5cU, 0x2dU, 0x02U, 0xcfU, 0xe0U, 0x91U, 0xbeU,
    0x24U, 0x0bU, 0x7aU, 0x55U, 0x98U, 0xb7U, 0xc6U, 0xe9U,
    0xddU, 0xf2U, 0x83U, 0xacU, 0x61U, 0x4eU, 0x3fU, 0x10U,
    0x8aU, 0xa5U, 0xd4U, 0xfbU, 0x36U, 0x19U, 0x68U, 0x47U,
    0xe6U, 0xc9U, 0xb8U, 0x97U, 0x5aU, 0x75U, 0x04U, 0x2bU,
    0xb1U, 0x9eU, 0xefU, 0xc0U, 0x0dU, 0x22U, 0x53U, 0x7cU,
    0x48U, 0x67U, 0x16U, 0x39U, 0xf4U, 0xdbU, 0xaaU, 0x85U,
    0x1fU, 0x30U, 0x41U, 0x6eU, 0xa3U, 0x8cU, 0xfdU, 0xd2U,
    0x95U, 0xbaU, 0xcbU, 0xe4U, 0x29U, 0x06U, 0x77U, 0x58U,
    0xc2U, 0xedU, 0x9cU, 0xb3U, 0x7eU, 0x51U, 0x20U, 0x0fU,
    0x3bU, 0x14U, 0x65U, 0x4aU, 0x87U, 0xa8U, 0xd9U, 0xf6U,
    0x6cU, 0x43U, 0x32U, 0x1dU, 0xd0U, 0xffU, 0x8eU, 0xa1U,
    0xe3U, 0xccU, 0xbdU, 0x92U, 0x5fU, 0x70U, 0x01U, 0x2eU,
    0xb4U, 0x9bU, 0xeaU, 0xc5U, 0x08U, 0x27U, 0x56U, 0x79U,
    0x4dU, 0x62U, 0x13U, 0x3cU, 0xf1U, 0xdeU, 0xafU, 0x80U,
    0x1aU, 0x35U, 0x44U, 0x6bU, 0xa6U, 0x89U, 0xf8U, 0xd7U,
    0x90U, 0xbfU, 0xceU, 0xe1U, 0x2cU, 0x03U, 0x72U, 0x5dU,
    0xc7U, 0xe8U, 0x99U, 0xb6U, 0x7bU, 0x54U, 0x25U, 0x0aU,
    0x3eU, 0x11U, 0x60U, 0x4fU, 0x82U, 0xadU, 0xdcU, 0xf3U,
    0x69U, 0x46U, 0x37U, 0x18U, 0xd5U, 0xfaU, 0x8bU, 0xa4U,
    0x05U, 0x2aU, 0x5bU, 0x74U, 0xb9U, 0x96U, 0xe7U, 0xc8U,
    0x52U, 0x7dU, 0x0cU, 0x23U, 0xeeU, 0xc1U, 0xb0U, 0x9fU,
    0xabU, 0x84U, 0xf5U, 0xdaU, 0x17U, 0x38U, 0x49U, 0x66U,
    0xfcU, 0xd3U, 0xa2U, 0x8dU, 0x40U, 0x6fU, 0x1eU, 0x31U,
    0x76U, 0x59U, 0x28U, 0x07U, 0xcaU, 0xe5U, 0x94U, 0xbbU,
    0x21U, 0x0eU, 0x7fU, 0x50U, 0x9dU, 0xb2U, 0xc3U, 0xecU,
    0xd8U, 0xf7U, 0x86U, 0xa9U, 0x64U, 0x4bU, 0x3aU, 0x15U,
    0x8fU, 0xa0U, 0xd1U, 0xfeU, 0x33U, 0x1cU, 0x6dU, 0x42U
};

/*******************************************************************************
 * Prototypes of internal functions
 ******************************************************************************/

/*!
 * @brief This function calculates CRC value of passed data array.
 *
 * @param data An array used for CRC calculation.
 * @param dataLen Length of the array Data.
 *
 * @return Computed CRC value.
 */
static uint8_t BCC_CalcCRC(const uint8_t *data, uint8_t dataLen);

/*******************************************************************************
 * Internal function
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CalcCRC
 * Description   : This function calculates CRC value of passed data array.
 *
 *END**************************************************************************/
static uint8_t BCC_CalcCRC(const uint8_t *data, uint8_t dataLen)
{
    uint8_t crc;      /* Result. */
    uint8_t tableIdx; /* Index to the CRC table. */
    uint8_t dataIdx;  /* Index to the data array (memory). */

    BCC_MCU_Assert(data != NULL);

    /* Expanding value. */
    crc = 0x42U;

    for (dataIdx = 0U; dataIdx < dataLen; dataIdx++)
    {
#ifdef BCC_MSG_BIGEND
        tableIdx = crc ^ (*(data + dataIdx));
#else
        tableIdx = crc ^ (*(data + BCC_MSG_SIZE - 1 - dataIdx));
#endif
        crc = BCC_CRC_TABLE[tableIdx];
    }

    return crc;
}

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_PackFrame
 * Description   : This function packs all the parameters into a frame according
 *                 to the BCC frame format (see BCC datasheet).
 *
 *END**************************************************************************/
void BCC_PackFrame(uint16_t data, uint8_t addr, bcc_cid_t cid, uint8_t cmd,
    uint8_t frame[])
{
    BCC_MCU_Assert(frame != NULL);

    /* Memory Data field. */
    frame[BCC_MSG_IDX_DATA_H] = (uint8_t)(data >> 8U);
    frame[BCC_MSG_IDX_DATA_L] = (uint8_t)(data & 0xFFU);

    /* Memory Address fields. Master/Slave field is always 0 for sending. */
    frame[BCC_MSG_IDX_ADDR] = (addr & BCC_MSG_ADDR_MASK);

    /* Physical Address (Cluster ID). */
    frame[BCC_MSG_IDX_CID_CMD] = ((uint8_t)cid & 0x0FU) << 4U;

    /* Command field. */
    frame[BCC_MSG_IDX_CID_CMD] |= (cmd & 0x0FU);

    /* CRC field. */
    frame[BCC_MSG_IDX_CRC] = BCC_CalcCRC(frame, BCC_MSG_SIZE - 1U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CheckCRC
 * Description   : This function calculates CRC of a received frame and compares
 *                 it with CRC field of the frame.
 *
 *END**************************************************************************/
bcc_status_t BCC_CheckCRC(const uint8_t *resp)
{
    uint8_t frameCrc;  /* CRC value from resp. */
    uint8_t compCrc;   /* Computed CRC value. */

    BCC_MCU_Assert(resp != NULL);

    /* Check CRC. */
    frameCrc = *(uint8_t *)(resp + BCC_MSG_IDX_CRC);
    compCrc = BCC_CalcCRC(resp, BCC_MSG_SIZE - 1U);
    return (compCrc != frameCrc) ? BCC_STATUS_CRC : BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CheckRcTagId
 * Description   : This function checks value of the Command field of a frame.
 *
 *END**************************************************************************/
bcc_status_t BCC_CheckRcTagId(bcc_device_t devType, const uint8_t *resp,
        uint8_t rc, uint8_t tagId)
{
    uint8_t field;    /* Command field of a frame. */
    uint8_t regAddr;  /* Address field from a frame. */

    BCC_MCU_Assert(resp != NULL);

    field = *(uint8_t *)(resp + BCC_MSG_IDX_CID_CMD);
    regAddr = *(uint8_t *)(resp + BCC_MSG_IDX_ADDR) & BCC_MSG_ADDR_MASK;

    if (((devType == BCC_DEVICE_MC33771) && BCC_HAS_TAG_ID_MC33771(regAddr)) ||
        ((devType == BCC_DEVICE_MC33772) && BCC_HAS_TAG_ID_MC33772(regAddr)))
    {
        /* Check TAG ID. */
        if ((field & BCC_MSG_TAGID_MASK) != (tagId & BCC_MSG_TAGID_MASK))
        {
            return BCC_STATUS_COM_TAG_ID;
        }
    }
    else
    {
        /* Check Rolling Counter value. */
        if ((field & BCC_MSG_RC_MASK) != (rc & BCC_MSG_RC_MASK))
        {
            return BCC_STATUS_COM_RC;
        }
    }

    return BCC_STATUS_SUCCESS;
}
