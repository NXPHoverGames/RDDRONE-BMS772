/****************************************************************************
 * nxp_bms/BMS_v1/inc/batManagement.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
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
 **     Date   		: 2020-04-03
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
#error "Please enable CONFIG_LIBC_FLOATINGPOINT"
#endif
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
//#include <string.h>
#include "ledState.h"
#include "BMS_data_types.h"

/*******************************************************************************
 * defines
 ******************************************************************************/
/*! @brief use with the batManagement_setGatePower to close the gate */
#define GATE_CLOSE 	true 	
/*! @brief use with the batManagement_setGatePower to open the gate */
#define GATE_OPEN 	false	

#define BMS_FAULT_CELL1_BIT_SHIFT 			0
#define BMS_FAULT_CELL2_BIT_SHIFT 			1
#define BMS_FAULT_CELL3_BIT_SHIFT 			2
#define BMS_FAULT_CELL4_BIT_SHIFT 			3
#define BMS_FAULT_CELL5_BIT_SHIFT 			4
#define BMS_FAULT_CELL6_BIT_SHIFT 			5
#define BMS_FAULT_R_TEMPERATURE_BIT_SHIFT 	6
#define BMS_FAULT_BAT_TEMPERATURE_BIT_SHIFT 7

#define BMS_FAULT_T_TEMPERATURE_BIT_SHIFT 	8
#define BMS_FAULT_AFE_TEMPERATURE_BIT_SHIFT	9
#define	BMS_FAULT_CELL_UV_BIT_SHIFT 		10   
#define	BMS_FAULT_CELL_OV_BIT_SHIFT 		11  
#define	BMS_FAULT_PCB_UV_BIT_SHIFT 			12   
#define	BMS_FAULT_PCB_OV_BIT_SHIFT 			13   
#define	BMS_FAULT_UT_BIT_SHIFT 				14 	
#define	BMS_FAULT_OT_BIT_SHIFT 				15 	

#define	BMS_FAULT_OVER_CURRENT_BIT_SHIFT	16
#define BMS_FAULT_SLEEP_OC_BIT_SHIFT 		17
#define BMS_FAULT_CC_OVRERFLOW_BIT_SHIFT	18
#define BMS_FAULT_OTHER_FAULT_BIT_SHIFT		19

#define BMS_FAULT_TEMPERATURE_BITS_SHIFT 	BMS_FAULT_R_TEMPERATURE_BIT_SHIFT

//! @brief  this is used to check the cell voltage is just below the CELL_OV voltage
 #define CHARGE_COMPLETE_MARGIN_DIV 			10
/*******************************************************************************
 * types
 ******************************************************************************/

/*! @brief these are the modes the AFE can be set to */
typedef enum 
{
	AFE_NORMAL		= 0, /*!< the update measurements and the diagnostic could be run */
	AFE_SLEEP_MEAS	= 1, /*!< the AFE is set to sleep mode but cyclic measurements will 
 							  stay on to monitor faults, the values will not be updated */
	AFE_SLEEP 		= 2  /*!< the AFE will be set to sleep and will not measure anything,
							  it will not provide any saftey feature */
}AFEmode_t;

/*! @brief Measurements provided by Battery Cell Controller.
 *
 */

typedef enum 
{
	BMS_CELL1			= (1<<BMS_FAULT_CELL1_BIT_SHIFT),	  			/*!< there is a fault with cell 1 */
	BMS_CELL2			= (1<<BMS_FAULT_CELL2_BIT_SHIFT),	  			/*!< there is a fault with cell 2 */
	BMS_CELL3			= (1<<BMS_FAULT_CELL3_BIT_SHIFT),	  			/*!< there is a fault with cell 3 */
	BMS_CELL4			= (1<<BMS_FAULT_CELL4_BIT_SHIFT),	  			/*!< there is a fault with cell 4 */
	BMS_CELL5			= (1<<BMS_FAULT_CELL5_BIT_SHIFT),	  			/*!< there is a fault with cell 5 */
	BMS_CELL6			= (1<<BMS_FAULT_CELL6_BIT_SHIFT),	  			/*!< there is a fault with cell 6 */
	BMS_R_TEMP			= (1<<BMS_FAULT_R_TEMPERATURE_BIT_SHIFT),		/*!< there is a fault with the sense resistor temperature*/
	BMS_BAT_TEMP		= (1<<BMS_FAULT_BAT_TEMPERATURE_BIT_SHIFT),		/*!< there is a fault with the battery temperature*/
	BMS_T_TEMP 			= (1<<BMS_FAULT_T_TEMPERATURE_BIT_SHIFT), 		/*!< there is a fault with the power switch transitor temperature*/
	BMS_AFE_TEMP 		= (1<<BMS_FAULT_AFE_TEMPERATURE_BIT_SHIFT),		/*!< there is a fault with the analog front end (BCC) temperature*/
	BMS_CELL_UV 		= (1<<BMS_FAULT_CELL_UV_BIT_SHIFT),  			/*!< there is a cell undervoltage fault */
	BMS_CELL_OV 		= (1<<BMS_FAULT_CELL_OV_BIT_SHIFT),  			/*!< there is a cell overvoltage fault */
	BMS_PCB_UV 			= (1<<BMS_FAULT_PCB_UV_BIT_SHIFT),  			/*!< the PCB has an undervoltage */
	BMS_PCB_OV 			= (1<<BMS_FAULT_PCB_OV_BIT_SHIFT),  			/*!< the PCB has an overvoltage */
	BMS_UT 				= (1<<BMS_FAULT_UT_BIT_SHIFT), 					/*!< these is an undertemperature with the BMS; one of the cells or/and PCB*/
	BMS_OT 				= (1<<BMS_FAULT_OT_BIT_SHIFT), 					/*!< there is an overtemperature with the BMS; one of the cells or/and PCB*/
	BMS_OVER_CURRENT 	= (1<<BMS_FAULT_OVER_CURRENT_BIT_SHIFT),  		/*!< there is an overcurrent */
	BMS_SLEEP_OC	 	= (1<<BMS_FAULT_SLEEP_OC_BIT_SHIFT),  			/*!< there is a sleep over current */
	BMS_CC_OVERFLOW 	= (1<<BMS_FAULT_CC_OVRERFLOW_BIT_SHIFT), 		/*!< there is an overflow in the coulomb counter registers */
	BMS_OTHER_FAULT 	= (1<<BMS_FAULT_OTHER_FAULT_BIT_SHIFT)			/*!< any other fault given by the BCC*/
}BMSFault_t;

// callback function pointers

/*! @brief this callback function is needed to report to the main that there is a current overflow */
typedef void (*overCurrentCallbackFunction)(void);

/*! @brief this callback function is needed to change the color in the charging state */
typedef void (*changeLedColorCallbackBatFuntion)(LEDColor_t newColor, bool BlinkOn);

/*! @brief this callback function is needed to report that new measured data is set */
typedef void (*newMeasurementsCallbackFunction)(void);

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief 	this function initializes the battery management unit
 *
 * 			It will configure the BCC, connect the callback functions 
 * 			and set the power switches open, disconecting the battery
 * 			
 * @param 	p_overCurrentCallbackFunction the address of the function to call when an overcurrent occurs
 * @param 	p_changeLedColorCallbackBatFuntion the address of the function to call to change the LED color
 * @param 	p_newMeasurementsCallbackFunction the address of the function to call when new data is set
 * 			should be quick
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int batManagement_initialize(overCurrentCallbackFunction p_overCurrentCallbackFunction, 
							changeLedColorCallbackBatFuntion p_changeLedColorCallbackBatFuntion,
							newMeasurementsCallbackFunction p_newMeasurementsCallbackFunction); 

/*!
 * @brief 	This function is used to set the gate driver it can turns the gate driver OFF, 
 * 			so output power is OFF. But it can also set it on, enabling output power.
 * 			Does not function when in over-current (hardware protection turns
 * 			the gate driver OFF automatically).
 * 			This function is protected against multiple threads using this or the on function 
 * 			batManagement_initialize should be called before calling this function
 *
 * @param 	on if true the gate will be set on, otherwise it will be set off
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setGatePower(bool on);

/*!
 * @brief 	This function is used to check the AFE. 
 * 			It will check the fault masks of the AFE and set it in the variable
 * 			It will check the whole configuration and change it if possible.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	BMSFault this is the address of the BMSFault_t variable to store the error
 * @param 	resetFaultPin this will reset the BCC_FAULT pin if it is up. 
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkAFE(BMSFault_t *BMSFault, bool resetFaultPin);

/*!
 * @brief 	This function is used to check what the fault is. 
 * 			It will check the fault masks of the AFE and set it in the variable
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	BMSFault this is the address of the BMSFault_t variable to store the error in 
 * @param 	resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkFault(BMSFault_t *BMSFault, bool resetFaultPin);

/*!
 * @brief 	This function is used to set the AFE to a desired mode. 
 * 			In the AFE_NORMAL mode the update measurements and the diagnostic could be run.
 * 			In the AFE_SLEEP_MEAS the AFE is set to sleep mode but cyclic measurements will
 * 			stay on to monitor faults,  the values will not be updated.
 * 			In the AFE_SLEEP mode the AFE will be set to sleep and will not measure anything,
 * 			it will not provide any saftey feature.
 * 			The OV and UV of cells and the temperatures are reported with the fault pin! 
 * 			In a sleep mode, the 
 * 			BatManagement_initialize should be called before calling this function
 * 			
 * @warning In the AFE_NORMAL mode the batManagement_UpdateMeasurements should be on to check for an over current!
 *			
 *
 * @param 	mode The desired mode to set the AFE to.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setAFEMode(AFEmode_t mode); 

/*!
 * @brief 	This function could be used to configure a new configuration
 * 			It could also calculate a new value based on the input
 * 			like if remaining capacity changes, the new SoC is calculated.
 * 			it will make sure the system adjusts this variable.
 * 			this function should be called when a configuration variable changes.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	changedParam this is the parameter that changed, from the parameterKind_t enum
 * @param 	newValue the new value that was set
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_changedParameter(parameterKind_t changedParam, void* newValue);

/*!
 * @brief 	This function is used to enable the cyclic measurements. 
 * 			It will read the T_meas time and start the measurements task.
 * 			This task will measue and calculate the voltages, temperatures, current 
 * 			and estimate the SoC, SoH and average current.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	on if true it will start the task and if false it will delete the task.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_updateMeasurementsOn(bool on);

/*!
 * @brief 	This function is used to enable the cyclic diagnostics. 
 * 			It will read the T_ftti time and start the diagnostics task.
 * 			The diagnostics will check if everything is ok and if not
 * 			it will report to the callback function diagnosticsFail
 * 			batManagement_initialize should be called before calling this function
 * 			still needs to be implemented!!!!!
 * 			
 * @param 	on if true it will start the task and if false it will delete the task.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_doDiagnosticsOn(bool on);

/*!
 * @brief 	This function is used to do a measurement
 * 			This function will start a conversion and wait (blocking)
 *			until the BCC is done with the measurement.
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_doMeasurement(void); 

/*!
 * @brief 	This function is used to check if n_cells is in line with the measurements
 * @note 	A measurements should be done first.
 *
 * @param   nCellsOK the address of the variable to become 1 if the cells are OK.
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkNCells(bool *nCellsOK); 

/*!
 * @brief 	This function is used to get the output voltage and set it in the data struct
 * @note 	A measurements should be done first.
 *
 * @param   none
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_getOutputVoltage(void); 

/*!
 * @brief 	This function is used to start or stop the charging sequence. 
 * 			Normally it will stop after completion of the charging sequence, but a fault could occur.
 * 			It will start the charging task.
 * 			This will implement the charging state machine.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	on if true it will start the task and if false it will delete the task.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_startCharging(bool on);

/*!
 * @brief 	This function is used to start or stop the self discharging sequence. 
 * 			it will self-discharge each cell to the storage voltage.
 * 			Normally it will stop after completion of the charging sequence, but a fault could occur.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	on if true it will do self discharge, with false it will end
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_selfDischarge(bool on);

/*!
 * @brief 	This function is used to start or stop cell balancing. 
 * 			it will discharge the higher cells to the lowest cell.
 * 			Normally it will stop after completion of the charging sequence, but a fault could occur.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	on if true it will start balancing, with false it will end.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setBalancing(bool on);

/*!
 * @brief 	This function is used to check if cell balancing is active. 
 * 			
 * @return 	0 if cell balancing is not active, a positive number if so
 * 			if error a negative number will be returned to indicate the error. 
 */
int batManagement_checkBalancing(void);

/*!
 * @brief 	This function is used to check if the output voltage is at least OUTPUT_ON_VOLTAGE
 * 			
 * @return 	1 if output is active (>= OUTPUT_ON_VOLTAGE)
 * 			0 if not active, -1 if something went wrong
 */
int batManagement_checkOutputVoltageActive(void);

/*!
 * @brief 	This function is used to get the highest cell voltage
 * 			
 * @return 	the highest cell voltage
 */
float batManagement_getHighestCellV(void);

/*!
 * @brief 	This function is used to get the lowest cell voltage
 * 			
 * @return 	the lowest cell voltage
 */
float batManagement_getLowestCellV(void);

/*!
 * @brief 	This function is used to check if the end of cell balancing charge is there
 * 
 * @param 	set true if the newValue needs to be set, false if read
 * @param 	newValue if set is true, this is the new value
 * 			
 * @return 	0 if it is not done, 1 if so
 * 			if error, a negative number will be returned to indicate the error. 
 */
int batManagement_SetNReadEndOfCBCharge(bool set, int newValue);


/*!
 * @brief 	This function is used to check if the charger should charge to storage voltage
 * 
 * @param 	set true if the newValue needs to be set, false if read
 * @param 	newValue if set is true, this is the new value
 * 			
 * @return 	0 if not charging to storage, 1 if so, -1 if error
 */
int batManagement_SetNReadChargeToStorage(bool set, bool newValue);

/*!
 * @brief 	This function is to save the remaining capacity to the full charge capacity
 * 			this will be used when it is at the end of the charge cycle
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_saveFullChargeCap(void);

/*!
 * @brief 	This function is to calculate the remaining capacity 
 * 			this will be used when an CC overflow occures
 * @note 	it will read and reset the CC registers
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_calcRemaningCharge(void);

/*!
 * @brief 	This function is used to set the OCV interrupt timer. 
 * 			This function will be used to calculate the next OCV time, and set the timer.
 * 			If the functions isn't called with false, the time of the timer will increase with 50%
 * 			until the T_OCV_CYCLIC1 time is reached. 
 * 			To stop the timer and to reset the timer value, this should be called with false.
 * 			
 * @param 	on, true to calculate and set the next OCV timer, false to stop and reset the timer. 
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setOCVTimer(bool on);


/*!
 * @brief This function will initialize the spi mutex for the BCC function
 *
 * @return 	int If successful, the pthread_mutex_init() function shall return zero; 
 * 			otherwise, an error number shall be returned to indicate the error.
 */
extern int BCC_initialze_spi_mutex(void);

/*
 * @brief   This function is used to enable or disable the display of the measurements
 *
 * @param   showCommand which measurements to enable or disable
 * @param   value the new value of that bit
 *
 * @return  none
 */
void batManagement_setShowMeas(showCommands_t showCommand, bool value);
/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BAT_MANAGEMENT_H_ */
