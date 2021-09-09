/****************************************************************************
 * nxp_bms/BMS_v1/inc/data.h
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
 *
 ** ###################################################################
 **     Filename    : data.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        data module.
 **        This module contains all functions needed for shared data
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
#ifndef DATA_H_
#define DATA_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "BMS_data_types.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/*! @brief  The bit in the reset cause register that indicates a reset cause by 
 *          the external pin (in this case the watchdog)
 */
#define RESET_CAUSE_EXT_PIN   (1 << 6)

/*******************************************************************************
 * types
 ******************************************************************************/

/* @brief this callback function is used to handle the parameter change */
//typedef int (*parameterChangeCallbackFunction)(int argc, char *argv[]);

/*! @brief this callback function is used to handle the parameter change */
typedef int (*parameterChangeCallbackFunction)(parameterKind_t changedParameter, void *newValue);

/*! @brief this callback function is needed to get the main state variables */
typedef states_t (*getMainStateCallbackBatFuntion)(void);

/*! @brief this callback function is needed to get the charge state variables */
typedef charge_states_t (*getChargeStateCallbackBatFuntion)(void);
/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief       this function is needed to use the data_setParameter and data_getParameter
 *              it will initialize the mutex and return the outcome of the pthread_mutex_init function
 *          
 * @param       p_parameterChangeCallbackFunction the address of the function to start a task to handle a parameter change
 * @param       p_getChargeStateCallbackBatFuntion the address of the function to get the main state
 * @param       p_userCommandCallbackBatFuntion the address of function to get the charge state
 *
 * @return      If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example     if(data_initialize())
 *              {
 *                  // do something with the error
 *              }
 */
int data_initialize(parameterChangeCallbackFunction p_parameterChangeCallbackFunction, 
    getMainStateCallbackBatFuntion p_getMainStateCallbackBatFuntion, 
    getChargeStateCallbackBatFuntion p_getChargeStateCallbackBatFuntion);

/*!
 * @brief       function to the type a certain parameter from the data struct in data.c
 *              this function could be used before the data_setParameter or data_getParameter function
 *     
 * @param       parameterKind the parameter of which the type should be returned, from the parameterKind enum in BMS_data_types.h
 *
 * @retval      the type of the parameter as a valueType_t enum value, defined in BMS_data_types.h
 *              it will return STRINGVAL if the parameterKind was NONE
 *             
 * @example     valueType_t paramType;
 *              switch(paramType) 
 *              {
 *                case FLOATVAL: // so somthing with floating points values
 *                break;
 *                case STRINGVAL: // do somthing with string values
 *                break;
 *                case default: // something with int32_t values (uint16 and uint8 can be set and get with int32)
 *                break;
 *              }
 *        
 */
valueType_t data_getType(parameterKind_t parameterKind);

/*!
 * @brief       function to get a certain parameter from the data struct in data.c
 *              this function could be used after the data_setParameter function
 *              there are 2 ways to get the parameter, either with the outData parameter 
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initializeData should have been called once
 *     
 * @param       parameterKind the parameter value it wants, from the parameterKind enum in BMS_data_types.h
 * @param       outData pointer to the value that needs to become the parameter value, could be int64_t, float or char*
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes) 
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      a void pointer to the value 
 * @example     int64_t lvGetParam; 
 *              uint8_t *pReadUint8_tData = NULL;
 *              pReadUint8_tData = data_getParameter(STATE_OF_CHARGE, &lvGetParam, NULL);
 *              if(pReadUint8_tData != NULL)
 *              {
 *                // do something with value
 *              }
 *              else
 *              {
 *                // something went wrong!
 *              }
 *              other example:
 *              char  *GetString = malloc(sizeof(char[32]));
 *              uint16_t *Lenght = malloc(sizeof(uint16_t));
 *              data_getParameter(MODEL_NAME, GetString, Lenght);
 *              free(GetString);
 *              free(Lenght); 
 */
void* data_getParameter(parameterKind_t parameterKind, void* outData, uint16_t* outLength);

/*!
 * @brief       function to set a certain parameter in the data struct
 *              after this, the value can be read with the data_setParameter funciton
 *              this function only sets the new value if it is within the range 
 *              of the parameter as discribed in BMS_data_limits.h
 *              it is possible to signal if a parameter has changed (work in progress!)
 *              data_initialize should have been called once         
 * 
 * @param       parameterKind the setting it wants to set from the parameterKind enum in BMS_data_types.h
 * @param       inNewValue a pointer to the value it will be
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 * @example:    int32_t newValue = 3; or float newValue = 3.3
 *              if(data_setParameter(STATE_OF_CHARGE, &newValue))
 *              {
 *                // it went wrong!
 *              }
 *  
 *              for model name:
 *              char newModelName[] = "New model name\0";
 *              if(data_setParameter(MODEL_NAME, newModelName))
 *              {
 *                // it went wrong!
 *              }
 */
int data_setParameter(parameterKind_t parameterKind, void* inNewValue);

/*!
 * @brief       function to lock the mutex (make sure that no other tasks/threads can acces/change the data)
 *              make sure the mutex is unlocked again with data_unlockMutex
 *              should always be used with data_unlockMutex
 *       
 * @param       none
 *
 * @retval      0 if succeeded, otherwise the error of pthread_mutex_lock
 *              
 * @example     uint8_t stateOfCharge; 
 *              data_lockMutex();
 *              stateOfCharge = *(data_getAdr(STATE_OF_CHARGE));
 *              data_unlockMutex();
 *              
 */
int data_lockMutex(void);

/*!
 * @brief       function to unlock the mutex (make sure that other tasks/threads can acces/change the data)
 *              make sure the mutex is locked with the data_lockMutex function before calling this funciton
 *              should always be used with the data_lockMutex
 *       
 * @param       none
 *
 * @retval      0 if succeeded, otherwise the error of pthread_mutex_unlock
 *              
 * @example     uint8_t stateOfCharge; 
 *              data_lockMutex();
 *              stateOfCharge = *(data_getAdr(STATE_OF_CHARGE));
 *              data_unlockMutex();
 *              
 */
int data_unlockMutex(void);

/*!
 * @brief       function to get the address of a certain parameter from the data struct in data.c
 *              this function is faster than the get or set function
 *              this function should be used with the data_lockMutex() and data_unlockMutex() functions
 * 
 *       
 * @param       parameterKind the parameter of which the address should be returned, from the parameterKind enum in BMS_data_types.h
 *
 * @retval      an void pointer addres to the value
 * @warning     be sure to lock te mutex before this function and unlock it after. 
 *              shouldn't be used to set a variable!
 *              use with caution, a variable could be changed with this function but there are no limit checks
 *              there is alno no check if the varaible has changed (the signal for other functions)
 *              
 * @example     uint8_t stateOfCharge; 
 *              data_lockMutex();
 *              stateOfCharge = *(data_getAdr(STATE_OF_CHARGE));
 *              data_unlockMutex();
 *              
 */
void* data_getAdr(parameterKind_t parameterKind);


/*!
 * @brief   function to get the unit as a strings of a certain parameter 
 *     
 * @param   parameterKind the parameter of which the unit should be returned, 
 *          from the parameterKind enum in BMS_data_types.h
 *
 * @retval  a char pointer (string) of the value
 *        
 */
char* data_getUnit(parameterKind_t parameterKind);

/*!
 * @brief   function to get the type as a strings of a certain parameter  
 *     
 * @param   parameterKind the parameter of which the type should be returned, 
 *          from the parameterKind enum in BMS_data_types.h
 *
 * @retval  a char pointer (string) of the value
 *        
 */
char* data_getTypeString(parameterKind_t parameterKind);

/*!
 * @brief   function that will return the main state, it will use a mutex 
 * @note    Could be called from multiple threads
 *  
 * @param   none
 *
 * @return  the state of the main state machine. 
 */
states_t data_getMainState(void);

/*!
 * @brief   function that will return the charge state, it will use a mutex 
 * @note    Could be called from multiple threads
 *  
 * @param   none
 *
 * @return the state of the charge state machine. 
 */
charge_states_t data_getChargeState(void);

/*!
 * @brief   function to set a bit in the status flags (status_flags)
 * @note    if the flags are S_FLAGS_UKNOWN, clear the status_flags first.
 *     
 * @param   bit the bit that needs to change in the status_flags variable (0-7) use the STATUS_*_BIT defines for it
 * @param   value the new value of this bit as a bool
 *
 * @retval  0 if succeeded, otherwise the error of pthread_mutex_unlock
 *        
 * @example if(data_statusFlagBit(STATUS_OVERLOAD_BIT, 1))
 *          {
 *            // do something with the error
 *          }
 *        
 */
int data_statusFlagBit(uint8_t bit, bool value);

/*!
 * @brief       function to save the parameters in flash
 * @note        The eeeprom is used for this (4KB)
 * @note        Multi-thread protected
 * 
 * @retval      0 if it went OK, negative otherwise                 
 */
int data_saveParameters(void);

/*!
 * @brief       function to load the parameters from flash
 * @note        The eeeprom is used for this (4KB)
 * @note        Multi-thread protected
 * 
 * @retval      0 if it went OK, negative otherwise                 
 */
int data_loadParameters(void);

/*!
 * @brief     this function will set the default values of the BMS to the data struct
 * @note      Multi-thread protected
 *      
 * @param     none
 *
 * @return    0 if succeeded, negative otherwise
 */
int data_setDefaultParameters(void);

/*!
 * @brief     this function will return the difference of 2 timespec structs
 *      
 * @param     timeHigh  The high time (later sampled)
 * @param     timeLow   The low time (earlier sampled)
 *
 * @return    The difference in between timeHigh and timeLow in us
 */
int data_getUsTimeDiff(struct timespec timeHigh, struct timespec timeLow);

/*!
 * @brief   function to get the MCU reset cause
 * 
 * @param   none
 *
 * @return  the value of the reset cause register of the MCU, 0 if error
 */
unsigned int data_getResetCause(void);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* DATA_H_ */
