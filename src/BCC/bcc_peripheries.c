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
 * File: bcc_peripheries.c
 *
 * This file implements functions for LPSPI and GPIO operations required by BCC
 * driver. Adapted from BCC SW example code version 1.1.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <assert.h>

#include "BCC/bcc_peripheries.h"            // Include header file
#include "gpio.h"
#include "spi.h"
#include "cli.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BCC_SPI_TRANSMISSION_BYTES  5

/* Number of bytes what 40b needs to be aligned in S32K118 SDK LPSPI driver
 * to. */
#define LPSPI_ALIGNMENT   8

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/
pthread_mutex_t gSPIMutex;


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief This function will initialize the spi mutex for the BCC function
 *
 * @return  int If successful, the pthread_mutex_init() function shall return zero; 
 *          otherwise, an error number shall be returned to indicate the error.
 */
int BCC_initialze_spi_mutex(void)
{
    // initialzie the mutex
    return pthread_mutex_init(&gSPIMutex, NULL);
}

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
bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[], uint8_t recvBuf[])
{
    // transfer the spi message with the spi transfer function

    uint8_t tBuf[LPSPI_ALIGNMENT];
    uint8_t rBuf[LPSPI_ALIGNMENT];

    tBuf[0] = transBuf[4]; 
    tBuf[1] = transBuf[3];
    tBuf[2] = transBuf[2];
    tBuf[3] = transBuf[1];
    tBuf[4] = transBuf[0];

    // lock the mutex
    pthread_mutex_lock(&gSPIMutex);

    if(spi_BMSTransferData(BCC_SPI_BUS, tBuf, rBuf))
    {
        cli_printfError("BCC ERROR: SPI transfer failed!\n");
    }

    // unlock the mutex
    pthread_mutex_unlock(&gSPIMutex);

    recvBuf[0] = rBuf[4];
    recvBuf[1] = rBuf[3];
    recvBuf[2] = rBuf[2];
    recvBuf[3] = rBuf[1];
    recvBuf[4] = rBuf[0];

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[], uint16_t recvTrCnt)
{
    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_Assert
 * Description   : User implementation of assert.
 *
 *END**************************************************************************/
void BCC_MCU_Assert(bool x)
{
    DEBUGASSERT(x);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteCsbPin
 * Description   : Writes logic 0 or 1 to the CSB pin (or CSB_TX in case of TPL
 *                 mode).
 *
 *END**************************************************************************/
void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value)
{
    // nothing
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteRstPin
 * Description   : Writes logic 0 or 1 to the RST pin.
 *
 *END**************************************************************************/
void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value)
{
    // write the pin with the value
    gpio_writePin(BCC_RESET, value);
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value)
{
    // nothing
}

/*FUNCTION**********************************************************************
 *
 *                 MODIFIED. NOT USED BUT NEEDED TO COMPILE BCC SW LIBRARY.
 *
 *END**************************************************************************/
uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance)
{
    return 0;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
