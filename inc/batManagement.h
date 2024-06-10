/****************************************************************************
 * nxp_bms/BMS_v1/inc/batManagement.h
 *
 * BSD 3-Clause License
 *
 * Copyright 2020-2022 NXP
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
 **     Filename    : batManagement.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-04-03
 **     Abstract    :
 **        batManagement module.
 **        This module contains all functions needed for batManagement
 **
 ** ###################################################################*/
/*!
 ** @file batManagement.h
 **
 ** @version 01.00
 **
 ** @brief
 **        batManagement module. this module contains the functions to manage the battery
 **
 */
#ifndef BAT_MANAGEMENT_H_
#define BAT_MANAGEMENT_H_

#ifndef CONFIG_LIBC_FLOATINGPOINT
#    error "Please enable CONFIG_LIBC_FLOATINGPOINT"
#endif
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
//#include <string.h>
#include "ledState.h"
#include "BMS_data_types.h"
#include "balancing.h"

/*******************************************************************************
 * defines
 ******************************************************************************/
/*! @brief use with the batManagement_setGatePower to close the gate */
#define GATE_CLOSE true
/*! @brief use with the batManagement_setGatePower to open the gate */
#define GATE_OPEN false

#define BMS_FAULT_CELL1_BIT_SHIFT             0
#define BMS_FAULT_CELL2_BIT_SHIFT             1
#define BMS_FAULT_CELL3_BIT_SHIFT             2
#define BMS_FAULT_CELL4_BIT_SHIFT             3
#define BMS_FAULT_CELL5_BIT_SHIFT             4
#define BMS_FAULT_CELL6_BIT_SHIFT             5
#define BMS_FAULT_R_TEMPERATURE_BIT_SHIFT     6
#define BMS_FAULT_BAT_TEMPERATURE_BIT_SHIFT   7
#define BMS_FAULT_T_TEMPERATURE_BIT_SHIFT     8
#define BMS_FAULT_AFE_TEMPERATURE_BIT_SHIFT   9
#define BMS_FAULT_CELL_UV_BIT_SHIFT           10
#define BMS_FAULT_CELL_OV_BIT_SHIFT           11
#define BMS_FAULT_SW_CELL_OV_BIT_SHIFT        12
#define BMS_FAULT_PCB_UV_BIT_SHIFT            13
#define BMS_FAULT_PCB_OV_BIT_SHIFT            14
#define BMS_FAULT_UT_BIT_SHIFT                15
#define BMS_FAULT_OT_BIT_SHIFT                16
#define BMS_FAULT_AVG_OVER_CURRENT_BIT_SHIFT  17
#define BMS_FAULT_PEAK_OVER_CURRENT_BIT_SHIFT 18
#define BMS_FAULT_SLEEP_OC_BIT_SHIFT          19
#define BMS_FAULT_CC_OVRERFLOW_BIT_SHIFT      20
#define BMS_FAULT_CSB_WAKEUP_BIT_SHIFT        21
#define BMS_FAULT_OTHER_FAULT_BIT_SHIFT       22
#define BMS_FAULT_TEMPERATURE_BITS_SHIFT      BMS_FAULT_R_TEMPERATURE_BIT_SHIFT
//! @brief  this is used to check the cell voltage is just below the CELL_OV voltage
#define CHARGE_COMPLETE_MARGIN_DIV            10

/*******************************************************************************
 * types
 ******************************************************************************/

/*! @brief these are the modes the AFE can be set to */
typedef enum
{
    AFE_NORMAL      = 0, /*!< the update measurements and the diagnostic could be run
                              AFE will measure at interval BCC_CYCLIC_TIMER_INTERVAL_NORMAL*/
    AFE_SLOW        = 1, /*!< the update measurements and the diagnostic could be run
                              AFE will measure at interval BCC_CYCLIC_TIMER_INTERVAL_SLOW
                              The mode can be used to reduce power*/
    AFE_SLEEP_MEAS  = 2, /*!< the AFE is set to sleep mode but cyclic measurements will 
                              stay on to monitor faults, the values will not be updated 
                              AFE will measure at interval BCC_CYCLIC_TIMER_INTERVAL_SLEEP*/
    AFE_SLEEP       = 3  /*!< the AFE will be set to sleep and will not measure anything,
                              it will not provide any saftey feature */
} AFEmode_t;

/*!
 *  @brief  Enumeration to check what triggered a fault.
 *  @note   This could be from the BCC or by the measurements which are checked.
 */
typedef enum
{
    BMS_CELL1               = (1<<BMS_FAULT_CELL1_BIT_SHIFT),               /*!< there is a fault with cell 1 */
    BMS_CELL2               = (1<<BMS_FAULT_CELL2_BIT_SHIFT),               /*!< there is a fault with cell 2 */
    BMS_CELL3               = (1<<BMS_FAULT_CELL3_BIT_SHIFT),               /*!< there is a fault with cell 3 */
    BMS_CELL4               = (1<<BMS_FAULT_CELL4_BIT_SHIFT),               /*!< there is a fault with cell 4 */
    BMS_CELL5               = (1<<BMS_FAULT_CELL5_BIT_SHIFT),               /*!< there is a fault with cell 5 */
    BMS_CELL6               = (1<<BMS_FAULT_CELL6_BIT_SHIFT),               /*!< there is a fault with cell 6 */
    BMS_R_TEMP              = (1<<BMS_FAULT_R_TEMPERATURE_BIT_SHIFT),       /*!< there is a fault with the sense resistor temperature*/
    BMS_BAT_TEMP            = (1<<BMS_FAULT_BAT_TEMPERATURE_BIT_SHIFT),     /*!< there is a fault with the battery temperature*/
    BMS_T_TEMP              = (1<<BMS_FAULT_T_TEMPERATURE_BIT_SHIFT),       /*!< there is a fault with the power switch transitor temperature*/
    BMS_AFE_TEMP            = (1<<BMS_FAULT_AFE_TEMPERATURE_BIT_SHIFT),     /*!< there is a fault with the analog front end (BCC) temperature*/
    BMS_CELL_UV             = (1<<BMS_FAULT_CELL_UV_BIT_SHIFT),             /*!< there is a cell undervoltage fault */
    BMS_CELL_OV             = (1<<BMS_FAULT_CELL_OV_BIT_SHIFT),             /*!< there is a cell overvoltage fault */
    BMS_SW_CELL_OV          = (1<<BMS_FAULT_SW_CELL_OV_BIT_SHIFT),          /*!< there is a SW cell overvoltage fault */
    BMS_PCB_UV              = (1<<BMS_FAULT_PCB_UV_BIT_SHIFT),              /*!< the PCB has an undervoltage */
    BMS_PCB_OV              = (1<<BMS_FAULT_PCB_OV_BIT_SHIFT),              /*!< the PCB has an overvoltage */
    BMS_UT                  = (1<<BMS_FAULT_UT_BIT_SHIFT),                  /*!< these is an undertemperature with the BMS; one of the cells or/and PCB*/
    BMS_OT                  = (1<<BMS_FAULT_OT_BIT_SHIFT),                  /*!< there is an overtemperature with the BMS; one of the cells or/and PCB*/
    BMS_AVG_OVER_CURRENT    = (1<<BMS_FAULT_AVG_OVER_CURRENT_BIT_SHIFT),    /*!< there is an average overcurrent */
    BMS_PEAK_OVER_CURRENT   = (1<<BMS_FAULT_PEAK_OVER_CURRENT_BIT_SHIFT),   /*!< there is an peak overcurrent */
    BMS_SLEEP_OC            = (1<<BMS_FAULT_SLEEP_OC_BIT_SHIFT),            /*!< there is a sleep over current */
    BMS_CC_OVERFLOW         = (1<<BMS_FAULT_CC_OVRERFLOW_BIT_SHIFT),        /*!< there is an overflow in the coulomb counter registers */
    BMS_CSB_WAKEUP          = (1<<BMS_FAULT_CSB_WAKEUP_BIT_SHIFT),          /*!< the BCC is woken by a SPI transfer (CSB wake-up detected) */
    BMS_OTHER_FAULT         = (1<<BMS_FAULT_OTHER_FAULT_BIT_SHIFT)          /*!< any other fault given by the BCC*/
} BMSFault_t;
// callback function pointers

/*! @brief this callback function is needed to report to the main that there is a current overflow */
typedef void (*swMeasuredFaultCallbackFunction)(bool triggerFault);

/*! @brief this callback function is needed to change the color in the charging state */
typedef void (*changeLedColorCallbackBatFuntion)(
    LEDColor_t newColor, LEDColor_t newAltColor, uint16_t blinkPeriodms);

/*! @brief this callback function is used to check for new state transitions on the measured current */
typedef int (*checkForTransitionCurrentCallbackFunction)(float *currentA);

/*! @brief this callback function is needed to report that new measured data is set */
typedef void (*newMeasurementsCallbackFunction)(void);

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   this function initializes the battery management unit
 *
 *          It will configure the BCC, connect the callback functions
 *          and set the power switches open, disconecting the battery
 *
 * @param   p_swMeasuredFaultCallbackFunction the address of the function to call when a sw measured fault
 *          occured.
 * @param   p_changeLedColorCallbackBatFuntion the address of the function to call to change the LED color
 * @param   p_checkForTransitionCurrentCallbackFunction the address of the function to check for the new
 *          transition current
 * @param   p_newMeasurementsCallbackFunction the address of the function to call when new data is set
 *          should be quick
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error:
 *
 */
int batManagement_initialize(swMeasuredFaultCallbackFunction p_swMeasuredFaultCallbackFunction,
    changeLedColorCallbackBatFuntion                         p_changeLedColorCallbackBatFuntion,
    checkForTransitionCurrentCallbackFunction                p_checkForTransitionCurrentCallbackFunction,
    newMeasurementsCallbackFunction p_newMeasurementsCallbackFunction, bool skipSelfTest);

/*!
 * @brief   This function is used to set the gate driver it can turns the gate driver OFF,
 *          so output power is OFF. But it can also set it on, enabling output power.
 *          Does not function when in over-current (hardware protection turns
 *          the gate driver OFF automatically).
 *          This function is protected against multiple threads using this or the on function
 *          batManagement_initialize should be called before calling this function
 *
 * @param   on if true the gate will be set on, otherwise it will be set off
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setGatePower(bool on);

/*!
 * @brief   This function is used to check the AFE.
 *          It will check the fault masks of the AFE and set it in the variable
 *          It will check the whole configuration and change it if possible.
 *          batManagement_initialize should be called before calling this function
 *
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t
 *          enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_checkAFE(uint32_t *BMSFault, bool resetFaultPin);

/*!
 * @brief   This function is used to check what the fault is.
 *          It will check the fault masks of the AFE and set it in the variable
 *          batManagement_initialize should be called before calling this function
 *
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t
 *          enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_checkFault(uint32_t *BMSFault, bool resetFaultPin);

/*!
 * @brief   This function is used to set the AFE to a desired mode.
 *          In the AFE_NORMAL mode the update measurements and the diagnostic could be run.
 *          In the AFE_SLEEP_MEAS the AFE is set to sleep mode but cyclic measurements will
 *          stay on to monitor faults,  the values will not be updated.
 *          In the AFE_SLEEP mode the AFE will be set to sleep and will not measure anything,
 *          it will not provide any saftey feature.
 *          The OV and UV of cells and the temperatures are reported with the fault pin!
 *          In a sleep mode, the
 *          BatManagement_initialize should be called before calling this function
 *
 * @note    It will disable the measurements and enable them in AFE_NORMAL or AFE_SLOW if they were on.
 *
 * @warning In the AFE_NORMAL mode the batManagement_UpdateMeasurements should be on to check for an over
 *          current!
 *
 *
 * @param   mode The desired mode to set the AFE to.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setAFEMode(AFEmode_t mode);

/*!
 * @brief   This function could be used to configure a new configuration
 *          It could also calculate a new value based on the input
 *          like if remaining capacity changes, the new SoC is calculated.
 *          it will make sure the system adjusts this variable.
 *          this function should be called when a configuration variable changes.
 *          batManagement_initialize should be called before calling this function
 *
 * @warning In this, or underlying function the data_setParameter and data_getParameter may not be used.
 *
 * @param   changedParam this is the parameter that changed, from the parameterKind_t enum
 * @param   newValue the new value that was set
 * @param   extraValue Address of the extra value that can be used, may be NULL if not used.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_changedParameter(parameterKind_t changedParam, void *newValue, void *extraValue);

/*!
 * @brief   This function is used to enable or disable the battery management task.
 *          It will do the cyclic measurements, calculate the values, check for errors or transitions,
 *          If needed, it will check balancing.
 *          For example this task will measue and calculate the voltages, temperatures, current
 *          and estimate the SoC, SoH and average current.
 *          batManagement_initialize should be called before calling this function
 *
 * @param   enable if true it will enable the task and if false it will stop the task.
 *
 * @return  If successful, the function will return zero (OK). 1 if it already had the value, negative for an
 *          error.
 */
int batManagement_enableBatManagementTask(bool enable);

/*!
 * @brief   This function is used to get the status of the battery management task
 *
 * @param   on the address of the variable to indicate the battery management status.
 *          If true the measurement task is still running, false otherwise.
 *
 * @return  If successful, the function will return zero (OK), negative for an error.
 */
int batManagement_getBatManagementStatus(bool *on);

/*!
 * @brief   This function is used to enable the cyclic diagnostics.
 *          It will read the T_ftti time and start the diagnostics task.
 *          The diagnostics will check if everything is ok and if not
 *          it will report to the callback function diagnosticsFail
 *          batManagement_initialize should be called before calling this function
 *          still needs to be implemented!
 *
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_doDiagnosticsOn(bool on);

/*!
 * @brief   This function is used to do a measurement
 *          This function will start a conversion and wait (blocking)
 *          until the BCC is done with the measurement.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_doMeasurement(void);

/*!
 * @brief   This function is used to check if n_cells is in line with the measurements
 * @note    A measurements should be done first.
 *
 * @param   nCellsOK the address of the variable to become 1 if the cells are OK.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_checkNCells(bool *nCellsOK);

/*!
 * @brief   This function is used to get the output voltage and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_getOutputVoltage(void);

/*!
 * @brief   This function is used to get the cell voltages and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_getCellVoltages(void);

/*!
 * @brief   This function is used to get the battery current and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   checkCurrent if true, it will check the current as well for an overcurrent fault.
 *          It will set the overcurrent bit and trigger the main to act on it.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_getBattCurrent(bool checkCurrent);

/*!
 * @brief   This function is used to start or stop the charging sequence.
 *          Normally it will stop after completion of the charging sequence, but a fault could occur.
 *          It will start the charging task.
 *          This will implement the charging state machine.
 *          batManagement_initialize should be called before calling this function
 *
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_startCharging(bool on);

/*!
 * @brief   this function will set the new balance state.
 * @note    It can be used to re-start the balancing sequence as well.
 *
 * @param   newBalanceState The new balance state from the balanceState_t enum.
 *          Use BALANCE_OFF, BALANCE_TO_LOWEST_CELL or BALANCE_TO_STORAGE.
 *
 * @return  If successful, the function will return zero (OK).
 *          Otherwise, an error number will be returned to indicate the error:
 */
int batManagement_setBalanceState(balanceState_t newBalanceState);

/*!
 * @brief   this function will check the balance state.
 *
 * @param   none
 *
 * @return  The current balancing state from the balanceState_t enum.
 *          If an error occurs, it will return BALANCE_ERROR
 */
balanceState_t batManagement_getBalanceState(void);

/*!
 * @brief   This function is used to check if the output voltage is at least OUTPUT_ON_VOLTAGE
 *
 * @return  1 if output is active (>= OUTPUT_ON_VOLTAGE)
 *          0 if not active, -1 if something went wrong
 */
int batManagement_checkOutputVoltageActive(void);

/*!
 * @brief   This function is used to get the highest cell voltage
 *
 * @return  the highest cell voltage
 */
float batManagement_getHighestCellV(void);

/*!
 * @brief   This function is used to get the lowest cell voltage
 *
 * @return  the lowest cell voltage
 */
float batManagement_getLowestCellV(void);

/*!
 * @brief   This function is used to check if the end of cell balancing charge is there
 *
 * @param   set true if the newValue needs to be set, false if read
 * @param   newValue if set is true, this is the new value
 *
 * @return  0 if it is not done, 1 if so
 *          if error, a negative number will be returned to indicate the error.
 */
int batManagement_SetNReadEndOfCBCharge(bool set, int newValue);

/*!
 * @brief   This function is used to check if the charger should charge to storage voltage
 *
 * @param   set true if the newValue needs to be set, false if read
 * @param   newValue if set is true, this is the new value
 *
 * @return  0 if not charging to storage, 1 if so, -1 if error
 */
int batManagement_SetNReadChargeToStorage(bool set, bool newValue);

/*!
 * @brief   This function is to save the remaining capacity to the full charge capacity
 *          this will be used when it is at the end of the charge cycle
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_saveFullChargeCap(void);

/*!
 * @brief   This function is to calculate the remaining capacity
 *          this will be used when an CC overflow occures
 * @note    it will read and reset the CC registers
 * @param   clearingCCOverflow If not NULL this will be set true if the CC register is cleared.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_calcRemaningCharge(bool *clearingCCOverflow);

/*
 * @brief   This function can be used to calibrate the state of charge (SoC)
 * @note    A predefined table and the lowest cell voltage will be used for this
 * @note    can be called from mulitple threads
 * @warning The battery (voltage) needs to be relaxed before this is used!
 *
 * @param   calibrateARem if true, it will set the a-rem to calibrate SoC (mostly needed).
 *          If false, it will calibrate a-full based on a-rem with correct SoC (charge complete).
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 *          Could return -1 when the current > sleepcurrent.
 */
int batManagement_calibrateStateOfCharge(bool calibrateARem);

/*!
 * @brief   This function is used to output the cell voltages
 *
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_outputCellVoltages(void);

/*!
 * @brief   This function is used to enable or disable the CC_OVR_FLT mask,
 *          to set if the fault pin needs to be set or not with this fault.
 *
 * @param   enable if this fault needs to be enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setCCOvrFltEnable(bool enable);

/*!
 * @brief   This function is used to update the CSB_WUP_FLT bit in fault mask1 of the BCC,
 *          to set if the fault pin needs to be set or not with this fault.
 *
 * @param   enable if this fault needs to be enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setCSbFltEnable(bool enable);

/*!
 * @brief   This function is used to check if the sleep current threshold mask is enabled,
 *
 * @param   enabled address of the variable to be true if enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_checkSleepCurrentTh(bool *enabled);

/*!
 * @brief This function will initialize the spi mutex for the BCC function
 *
 * @return  int If successful, the pthread_mutex_init() function shall return zero;
 *          otherwise, an error number shall be returned to indicate the error.
 */
extern int BCC_initialze_spi_mutex(void);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BAT_MANAGEMENT_H_ */
