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
 * @file bcc_communication.h
 *
 * This file provides access to the functions used for SPI communication of BCC
 * driver in both SPI and TPL mode.
 */

#ifndef __BCC_COMM_H
#define __BCC_COMM_H

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/** SPI Frame. */
#ifdef BCC_MSG_BIGEND

/*! @brief Index to memory data field of SPI frame (higher byte). */
#define BCC_MSG_IDX_DATA_H        0U
/*! @brief Index to memory data field of SPI frame (lower byte). */
#define BCC_MSG_IDX_DATA_L        1U
/*! @brief Index to memory address field of SPI frame. */
#define BCC_MSG_IDX_ADDR          2U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CID_CMD       3U
/*! @brief Index to CRC field of SPI frame. */
#define BCC_MSG_IDX_CRC           4U

#else

/*! @brief Index to memory data field of SPI frame (higher byte). */
#define BCC_MSG_IDX_DATA_H        4U
/*! @brief Index to memory data field of SPI frame (lower byte). */
#define BCC_MSG_IDX_DATA_L        3U
/*! @brief Index to memory address field of SPI frame. */
#define BCC_MSG_IDX_ADDR          2U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CID_CMD       1U
/*! @brief Index to CRC field of SPI frame. */
#define BCC_MSG_IDX_CRC           0U

#endif

/** BCC Commands. */
/*! @brief No operation command. */
#define BCC_CMD_NOOP              0x00U
/*! @brief Read command. */
#define BCC_CMD_READ              0x01U
/*! @brief Write command. */
#define BCC_CMD_WRITE             0x02U
/*! @brief Global write command. */
#define BCC_CMD_GLOB_WRITE        0x03U

/*!
 * @brief Returns Memory Data field from a frame.
 *
 * @param msg Pointer to the frame.
 * @return Memory data field.
 */
#define BCC_GET_MSG_DATA(msg) \
    (((uint16_t)*((msg) + BCC_MSG_IDX_DATA_H) << 8U) | \
      (uint16_t)*((msg) + BCC_MSG_IDX_DATA_L))

/*! @brief Mask for memory address field of frame. */
#define BCC_MSG_ADDR_MASK   0x7FU
/*! @brief Mask for RC field of frame. */
#define BCC_MSG_RC_MASK     0x0CU
/*! @brief Mask for TAG ID field of frame. */
#define BCC_MSG_TAGID_MASK  0x0FU

/*!
 * @brief Converts binary value to Gray code used for Rolling Counter.
 * It is a 2-bit Gray code. Final value is shifted left by 2 bits.
 *
 * @param binVal Binary value to be converted.
 * @return Converted value shifted left by 2 bits.
 */
#define BCC_GET_RC(binVal) \
    ((((binVal) >> 1) ^ (binVal)) << 2)

/*!
 * @brief Increments rcIdx value and executes modulo 4.
 *
 * @param rcIdx Index to be incremented.
 * @return Incremented rcIdx value.
 */
#define BCC_INC_RC_IDX(rcIdx) \
    (((rcIdx) + 1U) & 0x03U)

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function packs all the parameters into a frame according to
 * the BCC frame format (see BCC datasheet).
 *
 * Note the frame is packed in Little-endian because of the LPSPI periphery.
 *
 * @param data 16 bit Memory Data field of the BCC frame.
 * @param addr 7 bit Memory Address field of the BCC frame.
 * @param cid 4 bit Physical Address field of the BCC frame.
 * @param cmd 4 bit Command field of the BCC frame.
 * @param frame Pointer to a 5-byte array where all the fields and computed
 *        CRC will be stored. Note that fields are stored into an array in
 *        reverse order (due to processing order in the LPSPI master and
 *        slave drivers).
 */
void BCC_PackFrame(uint16_t data, uint8_t addr, bcc_cid_t cid, uint8_t cmd,
        uint8_t frame[]);

/*!
 * @brief This function calculates CRC of a received frame and compares
 * it with CRC field of the frame.
 *
 * @param resp Pointer to memory that contains a response (frame)
 *        to be checked.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_CheckCRC(const uint8_t *resp);

/*!
 * @brief This function checks value of the Command field of a frame.
 *
 * @param devType Device type.
 * @param resp Pointer to memory that contains a response (frame)
 *        to be checked.
 * @param rc Expected value of Rolling Counter.
 * @param tagId Expected value of TAG ID.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_CheckRcTagId(bcc_device_t devType, const uint8_t *resp,
        uint8_t rc, uint8_t tagId);

/*! @} */

#endif /* __BCC_COMM_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
