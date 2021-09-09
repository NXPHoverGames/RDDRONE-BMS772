/****************************************************************************
 * nxp_bms/BMS_v1/inc/cli.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2021 NXP
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
 ** ###################################################################
 **     Filename    : cli.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        cli module.
 **        This module contains all functions needed for the command line interface (cli) 
 **        via the UART connection
 **
 ** ###################################################################*/
/*!
 ** @file Data.h
 **
 ** @version 01.00
 **
 ** @brief
 **        Data module. this module contains the functions for the shared memory
 **
 */
#ifndef CLI_H_
#define CLI_H_

#ifndef CONFIG_LIBC_FLOATINGPOINT
#error "Please enable CONFIG_LIBC_FLOATINGPOINT"
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "BMS_data_types.h"
#include <stdio.h>

/*******************************************************************************
 * defines
 ******************************************************************************/
#define HELP_INDEX          0
#define GET_INDEX           1
#define SET_INDEX           2
#define SHOW_INDEX          3
#define RESET_INDEX         4
#define SLEEP_INDEX         5
#define WAKE_INDEX          6
#define DEEP_SLEEP_INDEX    7
#define SAVE_INDEX          8
#define LOAD_INDEX          9
#define DEFAULT_INDEX       10
#define TIME_INDEX          11

/*******************************************************************************
 * types
 ******************************************************************************/
/*! @brief these are the possible cli commands */
typedef enum 
{
    CLI_HELP        = HELP_INDEX,       //!< the user want the help page
    CLI_GET         = GET_INDEX,        //!< the user wants to get a variable 
    CLI_SET         = SET_INDEX,        //!< the user wants to set a variable
    CLI_SHOW        = SHOW_INDEX,       //!< the user wants to set the visability of the cyclic measurements
    CLI_RESET       = RESET_INDEX,      //!< the user wants to reset the fault
    CLI_SLEEP       = SLEEP_INDEX,      //!< the user wants to set the device to sleep 
    CLI_WAKE        = WAKE_INDEX,       //!< the user wants to wake up the device 
    CLI_DEEP_SLEEP  = DEEP_SLEEP_INDEX, //!< the user wants to set the device to deep sleep
    CLI_SAVE        = SAVE_INDEX,       //!< the user want to save the parameters to flash
    CLI_LOAD        = LOAD_INDEX,       //!< the user wants to load the parameters from flash
    CLI_DEFAULT     = DEFAULT_INDEX,    //!< the user wants to set the deafault parameters
    CLI_TIME        = TIME_INDEX,       //!< the user wants to get the time since boot
    CLI_WRONG                           //!< the user has a wrong input
}commands_t;

/*! @brief this callback function is needed to let the main handle coommands the CLI cannot do */
typedef void (*userCommandCallbackBatFuntion)(commands_t command, uint8_t value);

/*! @brief this callback function is needed to get the main state variables */
typedef states_t (*getMainStateCallbackBatFuntion)(void);

/*! @brief this callback function is needed to get the charge state variables */
typedef charge_states_t (*getChargeStateCallbackBatFuntion)(void);

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function is needed to get the address of the state and the mutex to the cli
 *          
 * @param   p_userCommandCallbackBatFuntion address of the function to be called when a commands needs to be processed by the main
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 */
int cli_initialize(userCommandCallbackBatFuntion p_userCommandCallbackBatFuntion);

/*!
 * @brief   this function is deinitialize the cli
 *          
 * @param   None
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(cli_deinitialize())
 *          {
 *              // do something with the error
 *          }
 */
int cli_deinitialize(void);

/*!
 * @brief   this function is the cli function. 
 *          it will take care of the commands send to the BMS
 *          like the "help", "set" and "get" command
 *          
 * @param   the amount of arguments in char *argv[] 
 * @param   the arguments to the function
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example (in main.c) cli_processCommands(argc, argv);
 */
int cli_processCommands(int argc, char **argv);

/*!
 * @brief   this function is the same as printf(), but it will make sure it can be used
 *          by multiple threads 
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printf(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in red
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printfError(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in orange
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printfWarning(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is the same as cli_printf(), but it will print the message in green
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printfGreen(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is the same as cli_printf(), but with no mutex lock
 * @warning should be used with cli_printLock()
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printfNoLock(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is the same as cli_printf(), but with a try mutex lock
 * @warning should be used with cli_printLock()
 *          
 * @param   fmt This is the string that contains the text to be written to stdout. 
 *          It can optionally contain embedded format tags that are replaced by 
 *          the values specified in subsequent additional arguments and formatted as requested. 
 *          Format tags prototype is %[flags][width][.precision][length]specifier
 *
 * @param   the arguments of the function
 *
 * @return  If successful, the total number of characters written is returned. 
 *          On failure, a negative number is returned.
 */
int cli_printfTryLock(FAR const IPTR char *fmt, ...);

/*!
 * @brief   this function is used to lock and unlock the cli_print mutex. 
 *          it can be used to make sure updating the measured data is done without 
 *          writting random things in between.
 * @warning keep in mind that normal cli_printf() will hang the microcontroller, so use 
 *          cli_printfTryLock instead
 *          
 * @param   lock when true it will lock, when false it will unlock the cli_print mutex
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int cli_printLock(bool lock);

/*!
 * @brief   this function is used to update the data on the CLI when needed.
 *          
 * @param   none
 *
 * @return  0 If successful, otherwise an error will indicate the error
 */
int cli_updateData(void);

/*!
 * @brief   this function is used to get the string of a state.
 *          
 * @param   getMainState if true, it will get the main state string.
 *          If false, it will get the charge state string.
 * @param   state the state to get the string of, if getMainState is true from the states_t
 *          otherwise from the charge_states_t.
 * @param   dest This is the pointer to the destination array where the content is to be copied.
 *
 * @return  This returns a pointer to the destination string dest.
 */
char *cli_getStateString(bool getMainState, uint8_t state, char *dest); 

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* CLI_H_ */
