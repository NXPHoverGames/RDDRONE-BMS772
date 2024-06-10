/****************************************************************************
 * nxp_bms/BMS_v1/src/balancing.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2022 NXP
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "balancing.h"
#include "data.h"
#include "cli.h"

#include "bcc_spiwrapper.h"
#include "bcc_configuration.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define RBAL 82 //!< [Ohm] balancing resistor (84 Ohm for the Drone BMS)
#define MAX_BALANCING_MINUTES 511 //!< the maximum value to set in the balance driver

/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief  the BCC driver configuration */
bcc_drv_config_t* gPBccDrvConfig;

/*! @brief  variable to indicate of it is initialized */
static bool gBalancePartInitialized = false;

/*! @brief  mutex for the balance variable */
static pthread_mutex_t gBalanceMutex;

/*! @brief  variable for the balance state */
static balanceState_t gBalanceState = BALANCE_OFF;

/*! @brief  variable for initializing the balance function */
static bool gBalanceStartInitialized = false;

/*! @brief  variable to keep track of which cells are enabled in balancing */
static uint8_t gBalanceCellEnabled = 0;

/*! @brief  variable to keep track of how much more minutes (times 511 min) it should run per cell */
static uint8_t gCellBalanceTimes[6] = {0, 0, 0, 0, 0, 0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*!
 * @brief   this function will turn off the balance drivers
 * @note    It will turn off both the individual cell driver as the overall CB driver
 *
 * @param   None
 * 
 * @return  If successful, the function will return BALANCE_OFF. 
 *          Otherwise, it will return BALANCE_ERROR
 */
static balanceState_t turnOffBalancing(void);

/*!
 * @brief   this function will calculate the cell balance minutes 
 *          and  initiate the balancing 
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   dischargeVoltage The voltage to balance / self-discharge to.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
static int calculateAndEnableBalance(commonBatteryVariables_t *pCommonBatteryVariables,
    float dischargeVoltage);

/*!
 * @brief   this function will check if balancing is done by either the timer or the voltage
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   balanceVoltage The voltage to balance to V.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
static int checkBalancing(commonBatteryVariables_t *pCommonBatteryVariables,
    float balanceVoltage);

/*
 * @brief   This function is used to check if balancing is done for the specific BCC index
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   bccIndex index of the BCC cell (not 2 or 3 with a 3 cell battery): cells: 1, 2, 3, ... -> 0, 1, .. 5 
 * @param   done address of the variable to become true if balancing is done.
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
static int checkBalancingCellDone(bcc_drv_config_t* const drvConfig, 
    uint8_t bccIndex, bool *done);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the balancing part
 * 
 * @param   drvConfig the address the BCC driver configuration
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(balancing_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int balancing_initialize(bcc_drv_config_t* const drvConfig)
{
    int retValue = 0;

    // check if not initialized
    if(!gBalancePartInitialized)
    {
        DEBUGASSERT(drvConfig != NULL);

        gPBccDrvConfig = drvConfig;

        // initialze the mutex
        pthread_mutex_init(&gBalanceMutex, NULL);

        // make sure it is initialzed
        gBalancePartInitialized = true;

        // Make sure to initialze the balance start 
        gBalanceStartInitialized = false;
    }

    // return to the user
    return retValue;
}

/*!
 * @brief   this function will set the new balance state. 
 * @note    It can be used to re-start the balancing sequence as well.
 *
 * @param   newBalanceState The new balance state from the balanceState_t enum.
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_setBalanceState(balanceState_t newBalanceState)
{
    int retValue = -1;

    if(gBalancePartInitialized)
    {
        // lock the mutex
        pthread_mutex_lock(&gBalanceMutex);

        // Make sure to initialze the balance start again
        // It will restart if balancing has already happend
        gBalanceStartInitialized = false;

        // check for change
        if(newBalanceState != gBalanceState)
        {
            // check if the balancing should be off
            if(newBalanceState == BALANCE_OFF)
            {
                // turn off balancing
                newBalanceState = turnOffBalancing();
            }

            // save the new value
            gBalanceState = newBalanceState;
        }

        // check if OK
        if(gBalanceState != BALANCE_ERROR)
        {
            retValue = 0;
        }

        // unlock the mutex
        pthread_mutex_unlock(&gBalanceMutex);
    }

    return retValue;
}

/*!
 * @brief   this function will check the balance state. 
 *
 * @param   none
 *
 * @return  The current balancing state from the balanceState_t enum.
 *          If an error occurs, it will return BALANCE_ERROR
 */
balanceState_t balancing_getBalanceState(void)
{
    balanceState_t retValue = BALANCE_ERROR;

    if(gBalancePartInitialized)
    {
        // lock the mutex
        pthread_mutex_lock(&gBalanceMutex);

        retValue = gBalanceState;

        // unlock the mutex
        pthread_mutex_unlock(&gBalanceMutex);
    }

    return retValue;
}

/*!
 * @brief   this function will initiate the balancing and control/check it.
 * @note    Should be called cyclically after a new measurement.
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   lowestCellVoltage The lowest cell voltage of all cells in V.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
int balancing_handleCellBalancing(commonBatteryVariables_t *pCommonBatteryVariables,
    float lowestCellVoltage)
{
    int retValue = -1;
    float balanceReferenceVoltage;

    if(gBalancePartInitialized)
    {
        // lock the mutex
        pthread_mutex_lock(&gBalanceMutex);

        // check what to do
        switch(gBalanceState)
        {
            case BALANCE_OFF:
                // do nothing
            break;
            case BALANCE_TO_LOWEST_CELL:

                // balance to the lowest cell
                balanceReferenceVoltage = lowestCellVoltage;

                // check if balancing needs to be initialzed
                if(!gBalanceStartInitialized)
                {
                    // calculate the cell balance minutes and start balancing to lowest cell
                    if(calculateAndEnableBalance(pCommonBatteryVariables, balanceReferenceVoltage))
                    {
                        // there is an error
                        gBalanceState = BALANCE_ERROR;
                        cli_printfError("balancing ERROR: Could not calculate and start balancing to lowest cell!\n");
                    }
                    else
                    {
                        // Balancing is initialzed
                        gBalanceStartInitialized = true;
                    }
                }
            break;
            case BALANCE_TO_STORAGE:

                // balance to the storage 
                if(data_getParameter(V_STORAGE, &balanceReferenceVoltage, NULL) == NULL)
                {
                   cli_printfError("Balancing ERROR: getting storage voltage went wrong!\n");
                   balanceReferenceVoltage = V_STORAGE_DEFAULT;
                }

                // check if balancing needs to be initialzed
                if(!gBalanceStartInitialized)
                {
                    // calculate the cell balance minutes and start balancing to storage
                    if(calculateAndEnableBalance(pCommonBatteryVariables, balanceReferenceVoltage))
                    {
                        // there is an error
                        gBalanceState = BALANCE_ERROR;
                        cli_printfError("balancing ERROR: Could not calculate and start balancing to storage!\n");
                    }
                    else
                    {
                        // Balancing is initialzed
                        gBalanceStartInitialized = true;
                    }
                }
            break;
            case BALANCE_ERROR:
                cli_printfError("Balance ERROR: gBalanceState == BALANCE_ERROR \n");

                cli_printfWarning("Turning off balancing.... \n");

                // turn off balancing
                if(turnOffBalancing())
                {
                    // there is an error
                    gBalanceState = BALANCE_ERROR;

                    cli_printfError("balancing ERROR: Could not turn off balancing!\n");
                }
                else
                {
                    // state that balancing is off now, to not turn it off anymore
                    gBalanceState = BALANCE_OFF;
                }
            break;
        }

        // check if it is balancing
        if(gBalanceState == BALANCE_TO_STORAGE ||
            gBalanceState == BALANCE_TO_LOWEST_CELL)
        {
            // check balancing for voltage and timer, restart the CB driver if needed (511 min elapsed)
            if(checkBalancing(pCommonBatteryVariables, balanceReferenceVoltage))
            {
                cli_printfError("Balance ERROR: checkBalancing() failed!\n");

                gBalanceState = BALANCE_ERROR;
            }
        }

        // check if succeeded
        if(gBalanceState != BALANCE_ERROR)
        {
            retValue = OK;
        }

        // unlock the mutex
        pthread_mutex_unlock(&gBalanceMutex);
    }

    return retValue;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*!
 * @brief   this function will turn off the balance drivers
 * @note    It will turn off both the individual cell driver as the overall CB driver
 *
 * @param   None
 * 
 * @return  If successful, the function will return BALANCE_OFF. 
 *          Otherwise, it will return BALANCE_ERROR
 */
static balanceState_t turnOffBalancing(void)
{
    bcc_status_t bccStatus;
    int i;
    balanceState_t returnValue = BALANCE_OFF;

    // turn off the cell balance driver
    bccStatus = bcc_spiwrapper_BCC_CB_Enable(gPBccDrvConfig, BCC_CID_DEV1, false);
    if(bccStatus != BCC_STATUS_SUCCESS)
    {
        cli_printfError("Balancing ERROR: couldn't turn off CB driver in turnOffBalancing()! %d\n",
            bccStatus);

        // return with an error
        returnValue = BALANCE_ERROR;
    }
    else
    {
        // turn off all individual cells balance drivers
        for(i = 0; i < 6; i++)
        {
            bccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(gPBccDrvConfig, BCC_CID_DEV1, 
                i, false, 0xFF);

            // check for errors
            if(bccStatus != BCC_STATUS_SUCCESS)
            {
                cli_printfError("balancing ERROR: couldnt turn off BCC cell index%d error: %d\n",
                    i+1, bccStatus);

                // return with an error
                returnValue = BALANCE_ERROR;
            }

            // clear the bit in the balancing variable 
            gBalanceCellEnabled &= ~(1<<i);

            // reset the amout of balance times
            gCellBalanceTimes[i] = 0;
        }
    }

    // set the correct balance state
    gBalanceState = returnValue;

    return returnValue;
}

/*!
 * @brief   this function will calculate the cell balance minutes 
 *          and  initiate the balancing 
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   dischargeVoltage The voltage to balance / self-discharge to.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
static int calculateAndEnableBalance(commonBatteryVariables_t *pCommonBatteryVariables,
    float dischargeVoltage)
{
    int i, bccCellIndex, returnValue = 0;
    uint8_t cellMarginMv;
    uint16_t balanceMin;
    float ocvSlope;
    bcc_status_t bccStatus;

    // calculate for which cells the cell balance needs to be on

    // get the cell margin in mv
    if(data_getParameter(V_CELL_MARGIN, &cellMarginMv, NULL) == NULL)
    {
        cli_printfError("Balancing ERROR: getting cell margin went wrong!\n");
        cellMarginMv = V_CELL_MARGIN_DEFAULT;
        // return error
        returnValue |= -1;
    }

    // get the OCV slope
    if(data_getParameter(OCV_SLOPE, &ocvSlope, NULL) == NULL)
    {
        cli_printfError("Balancing ERROR: getting storage voltage went wrong!\n");
        ocvSlope = OCV_SLOPE_DEFAULT;
        // return error
        returnValue |= -1;
    }

    // turn off the driver
    bccStatus = bcc_spiwrapper_BCC_CB_Enable(gPBccDrvConfig, BCC_CID_DEV1, false);

    // check for errors
    if(bccStatus != BCC_STATUS_SUCCESS)
    {
        cli_printfError("balancing ERROR: couldn't turn off CB driver! %d\n",
            bccStatus);
        // return error
        returnValue |= -1;
    }

    // reset the variable
    gBalanceCellEnabled = 0;

    // check for which cell it needs to do this
    for(i = 0; i < pCommonBatteryVariables->N_cells; i++)
    {
        // reset the amout of balance times
        gCellBalanceTimes[i] = 0;

        // set the bcc index
        if(i >= 2)
        {
            // calculate the BCC pin index
            bccCellIndex = (6-pCommonBatteryVariables->N_cells) + i;
        }
        else
        {
            // it is the first 2 cells
            bccCellIndex = i;
        }

        // output equation to the user
        cli_printf("Balancing will be enabled for cell%d if %.3f > %.3f\n", i+1, 
            pCommonBatteryVariables->V_cellVoltages.V_cellArr[i], 
            (dischargeVoltage+((float)cellMarginMv/1000)));

        // check if the CB driver should be on for this cell
        if(pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] > 
            (dischargeVoltage+((float)cellMarginMv/1000)))
        {
            // calculate the CB timer
            balanceMin = 
                ((pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] - dischargeVoltage)*RBAL) / 
                (pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] * ocvSlope / 1000);

            cli_printf("Estimated cell%d balance minutes: %dmin\n", i+1, balanceMin);

            // reduce the balance minutes while it is larger than the max that can be set
            while(balanceMin > MAX_BALANCING_MINUTES)
            {
                // decrease balanceMin with MAX_BALANCING_MINUTES 
                balanceMin -= MAX_BALANCING_MINUTES;

                // increase the amount of times for this cell
                gCellBalanceTimes[i]++;
            }

            // write the balance cell register to turn balancing on for this cell
            bccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(gPBccDrvConfig, BCC_CID_DEV1, 
                bccCellIndex, true, balanceMin);
            
            // check for errors
            if(bccStatus != BCC_STATUS_SUCCESS)
            {
                cli_printfError("balancing ERROR: couldnt turn on cell%d balance: %d\n", 
                    i+1, bccStatus);
                // return error
                returnValue |= -1;
            }

            // increase the balancing variable 
            gBalanceCellEnabled |= (1<<i);
        }
        else
        {
            // write the balance cell register to turn balancing off for this cell
            bccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(gPBccDrvConfig, BCC_CID_DEV1, 
                bccCellIndex, false, 0xFF);

            // check for errors
            if(bccStatus != BCC_STATUS_SUCCESS)
            {
                cli_printfError("balancing ERROR: couldnt turn off cell%d balance: %d\n", 
                    i+1, bccStatus);
                // return error
                returnValue |= -1;
            }
        }
    }

    // check if cells are balanced
    if(gBalanceCellEnabled)
    {
        // lock the mutex to print this together
        cli_printLock(true);

        // output to the user
        cli_printfTryLock("Setting cell balance on for cell: ");

        // check which cells have CB on
        for (i = 0; i < 6; i++)
        {
            // check if on
            if(gBalanceCellEnabled & (1 << i))
            {
                // print the cell number
                cli_printfTryLock("%d, ", i+1);
            }
        }

        // remove the ","
        cli_printfTryLock("\b\b \n");

        // unlock the mutex to print this together
        cli_printLock(false);

        // turn on the cell balance driver
        if(bcc_spiwrapper_BCC_CB_Enable(gPBccDrvConfig, BCC_CID_DEV1, true) != BCC_STATUS_SUCCESS)
        {
            cli_printfError("balancing ERROR: couldn't turn on CB driver!\n");
            // return error
            returnValue |= -1;
        }
    }
    // if no cells are balanced
    else
    {
        // output that no cells need to be balanced
        cli_printf("No cells need to be balanced\n");

        // set the correct balance state
        gBalanceState = BALANCE_OFF;
    }

    return returnValue;
}

/*!
 * @brief   this function will check if balancing is done by either the timer or the voltage
 *
 * @param   pCommonBatteryVariables pointer to the commonBatteryVariables_t 
 *          for the battery information.
 * @param   balanceVoltage The voltage to balance to V.
 * 
 * @return  If successful, the function will return zero (OK). 
 *          Otherwise, an error number will be returned to indicate the error:
 */
static int checkBalancing(commonBatteryVariables_t *pCommonBatteryVariables,
    float balanceVoltage)
{
    int i, bccCellIndex, returnValue = 0;
    bool balancingDone;
    bcc_status_t bccStatus;

    // check if balancing is active
    if(gBalanceCellEnabled)
    {
        // go through each ell
        for(i = 0; i < 6; i++)
        {
            // check if balancing is enabled
            if(gBalanceCellEnabled & (1<<i))
            {
                // map the cells (1, 2, 3, ...) to the BCC cells (1, 2, ..., 6) 
                if(i >= 2)
                {
                    // calculate the BCC pin index
                    bccCellIndex = (6-pCommonBatteryVariables->N_cells) + i;
                }
                else
                {
                    // it is the first 2 cells
                    bccCellIndex = i;
                }

                // check if the balance time has timed out
                if(checkBalancingCellDone(gPBccDrvConfig, bccCellIndex, &balancingDone))
                {
                    cli_printfError("Balancing ERROR: could not check if balancing is done\n");
                    cli_printf("Setting balancing to be done for cell%d\n", i+1);
                    balancingDone = true;
                    // return error
                    returnValue |= -1;
                }

                // check if balancing time is done
                if(balancingDone)
                {
                    // check if the balancing needs to be on for at least the maximum time again
                    if(gCellBalanceTimes[i])
                    {
                        // turn on cell balancing for MAX_BALANCING_MINUTES for that cell
                        bccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(gPBccDrvConfig, 
                            BCC_CID_DEV1, bccCellIndex, true, MAX_BALANCING_MINUTES);

                        if(bccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("Balancing ERROR: couldnt turn on cell%d balance: %d\n", 
                                i+1, bccStatus);
                            // return error
                            returnValue |= -1;
                        }

                        // decrease the number of cell balance times
                        gCellBalanceTimes[i]--;
                    }
                    else
                    {
                        // output to the user
                        cli_printf("Balancing done for cell%d\n", i+1);

                        // clear the bit in the variable
                        gBalanceCellEnabled = gBalanceCellEnabled & ~(1<<i);
                    }
                }
                // if the balance timer is not done
                else
                {
                    // check if the cell voltage is not higher than the to discharge to voltage
                    if(pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] <= balanceVoltage)
                    {
                        // output to the user
                        cli_printf("Balancing done for cell%d due to voltage reached\n", i+1);

                        // clear the bit in the variable
                        gBalanceCellEnabled = gBalanceCellEnabled & ~(1<<i);

                        // turn off cell balancing for that cell
                        bccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(gPBccDrvConfig, 
                            BCC_CID_DEV1, bccCellIndex, false, 0xFF);
                        if(bccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("Balancing ERROR: could not set cell CB %d\n", bccStatus);
                            // return error
                            returnValue |= -1;
                        }
                    }
                }
            }
        }
    }

    // check if nothing is being balanced
    if(!gBalanceCellEnabled)
    {
        // turn off the cell balance driver
        bccStatus = bcc_spiwrapper_BCC_CB_Enable(gPBccDrvConfig, BCC_CID_DEV1, false);
        if(bccStatus != BCC_STATUS_SUCCESS)
        {
            cli_printfError("Balancing ERROR: couldn't turn off CB driver in checkBalancing! %d\n",
                bccStatus);
            // return error
            returnValue |= -1;
        }

        // set the correct balance state
        gBalanceState = BALANCE_OFF;
    }

    return returnValue;
}

/*
 * @brief   This function is used to check if balancing is done for the specific BCC index
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   bccIndex index of the BCC cell (not 2 or 3 with a 3 cell battery): cells: 1, 2, 3, ... -> 0, 1, .. 5 
 * @param   done address of the variable to become true if balancing is done.
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
static int checkBalancingCellDone(bcc_drv_config_t* const drvConfig, 
    uint8_t bccIndex, bool *done)
{
    int retValue = -1;
    bcc_status_t error;
    uint16_t retReg = 0;

    // check for null pointer
    DEBUGASSERT(done != NULL);

    // check if bccIndex is not too high
    if(bccIndex > 5)
    {
        cli_printfError("checkBalancingCellDone ERROR: bccIndex > 5!\n");

        return retValue;
    }

    // read the register
    error = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_CB1_CFG_ADDR + bccIndex),
        1, &retReg);

    // make the return value
    retValue = error;

    // check if balancing is done
    if(!(retReg & BCC_R_CB_STS_MASK))
    {
        // write the variable
        *done = true;
    }
    else
    {
        // write the variable
        *done = false;
    }

    // return
    return retValue;
}
