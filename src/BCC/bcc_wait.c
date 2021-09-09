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

/*
 * File: bcc_wait.c
 *
 * This file implements functions for busy waiting for BCC driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <unistd.h>

#include <stdint.h>
#include <stdbool.h>
#include "BCC/bcc_wait.h"       // Include header file

#define MS_TO_US 1000

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitMs(uint16_t delay)
{
    // implement the wait function
    BCC_MCU_WaitUs(delay*MS_TO_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitUs(uint32_t delay)
{
    // sleep for an amount of us
    usleep(delay);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
