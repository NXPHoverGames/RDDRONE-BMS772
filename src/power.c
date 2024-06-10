/****************************************************************************
 * nxp_bms/BMS_v1/src/power.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2021-2023 NXP
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

#include <stdint.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/boardctl.h>

#include "cli.h"
#include "power.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#ifndef MCU_POWER_DOMAIN
    #define MCU_POWER_DOMAIN    0
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
//! mutex for setting or getting the MCU power mode
pthread_mutex_t gPowerModeLock;

/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/
/*!
 * @brief   This function is used to initialize the power part  
 *
 * @return  0 if ok, -1 if there is an error
 */
int power_initialize(void)
{
    // initialize the mutex
    pthread_mutex_init(&gPowerModeLock, NULL);

#ifdef DISABLE_PM
    cli_printfWarning("PM Functionality is disabled! BMS will not be low power at all!\r\n");
#endif

    // return
    return OK;
}

/*!
 * @brief   Function that will set or get the MCU power mode
 * @note    Could be called from multiple threads
 *
 * @param   setNotGet if true it is used to set the power mode, false to get it
 * @param   newValue if setNotGet is true, this is the new power mode, could be ERROR_VALUE otherwise
 *
 * @return  the MCU power mode from the mcuPowerModes_t enum, ERROR_VALUE if error
 */
mcuPowerModes_t power_setNGetMcuPowerMode(bool setNotGet, mcuPowerModes_t newValue)
{
    mcuPowerModes_t lvRetValue = ERROR_VALUE;
    int error;

#ifndef DISABLE_PM

    // variable to check the PM state
    struct boardioc_pm_ctrl_s pmVariable = {
        .action =   BOARDIOC_PM_QUERYSTATE,
        .domain =   MCU_POWER_DOMAIN,
        .state =    0,
        .count =    0,
        .priority = 0,
    };

    // lock the mutex
    pthread_mutex_lock(&gPowerModeLock);

    // check if the user want to get the current MCU power state (PM_STATE)
    if(setNotGet == false)
    {
        // check the pm state and check for errors
        pmVariable.action = BOARDIOC_PM_QUERYSTATE;

        // do the boardctl to check the mode
        if(boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&pmVariable))
        {
            // get the error 
            error = errno; 

            // error
            cli_printfError("setNGetMcuPowerMode ERROR: Could not get power state: %d\n", 
                error);

            // set the error value
            lvRetValue = ERROR_VALUE;
        }
        else
        {
            // set a good value
            lvRetValue = RUN_MODE;
        }
    }
    // if the user wants to set the new MCU power mode
    else
    {
        // set the value 
        lvRetValue = newValue; 

        // check the new mode 
        switch(newValue)
        {
            // if the user wants to set the new run mode
            case RUN_MODE:
                pmVariable.state = PM_NORMAL;
            break;

            // if the user wants to set the standby mode
            case STANDBY_MODE:
                pmVariable.state = PM_STANDBY;
            break; 

            // if the user wants to set the VLPR mode
            case VLPR_MODE:
                pmVariable.state = PM_SLEEP;
            break;

            // wrong input 
            case ERROR_VALUE:
                
                // output error and return
                cli_printfError("setNGetMcuPowerMode ERROR: wrong input!\n");

                //return lvRetValue;
            break;
        }

        // check if there is no error
        if(lvRetValue != ERROR_VALUE)
        {
            // change the mode
            pmVariable.action = BOARDIOC_PM_CHANGESTATE;
            
            // do the boardctl to change the MCU power mode
            if(boardctl(BOARDIOC_PM_CONTROL, (uintptr_t)&pmVariable))
            {
                // get the error 
                error = errno; 

                // error 
                cli_printfError("setNGetMcuPowerMode ERROR: Updating MCU power mode went wrong: state: %d %d\n",
                    pmVariable.state, error);

                // set the error value
                lvRetValue = ERROR_VALUE;
            }
        }
    }

    // check if there is no error
    if(lvRetValue != ERROR_VALUE)
    {
        // check which state it is
        switch(pmVariable.state)
        {
            case PM_NORMAL:
                // set the value
                lvRetValue = RUN_MODE;
            break;
            case PM_STANDBY:
                // set the value
                lvRetValue = STANDBY_MODE;
            break;
            case PM_SLEEP:
                // set the value
                lvRetValue = VLPR_MODE;
            break;
            default:
                // output the error
                cli_printfError("setNGetMcuPowerMode ERROR: Power mode is not normal, standby nor sleep: %d\n",
                    pmVariable.state);

                // set the error value
                lvRetValue = ERROR_VALUE;
            break;
        }
    }

    // unlock the mutex
    pthread_mutex_unlock(&gPowerModeLock);
#else
    lvRetValue = RUN_MODE;
#endif

    // return the value
    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/
