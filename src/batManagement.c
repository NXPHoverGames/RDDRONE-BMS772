/****************************************************************************
 * nxp_bms/BMS_v1/src/batManagement.c
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, NXP Drone and Rover Team
 * All rights reserved.
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

#include <nuttx/board.h>
#include "batManagement.h"
#include <time.h>
#include <semaphore.h>
#include <math.h>

#include "gpio.h"
#include "bcc.h"
#include "bcc_configuration.h"
#include "bcc_monitoring.h"
#include "bcc_define.h"

#include "data.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/*! @brief if OUTPUT_UPDATE is defined, it will output the measurements on the CLI*/
//#define OUTPUT_UPDATE

//#define OUTPUT_OTHER_FAULT
//#define DEBUG_OV_UV
//#define DEBUG_OT_UT
//#define DEBUG_FAULT_STATUS
#define DEBUG_OUTPUT_ERROR_CURRENT

#define RBAL 82 //!< [Ohm] balancing resistor (84 Ohm for the Drone BMS)

#define DEFAULT_MEASURE_PRIORITY 		100
#define DEFAULT_MEASURE_STACK_SIZE_L 	1024 //2048
#define DEFAULT_MEASURE_STACK_SIZE_M 	1536 //2048
#define DEFAULT_MEASURE_STACK_SIZE_H 	2048 //2048

#ifndef MAX_NSEC
#define MAX_NSEC	 				999999999
#endif

#define MAX_SEC 					0xFFFFFFFF 					

//! this macro is used to print a byte value to binary, use as variable to BYTE_TO_BINARY_PATTERN
#ifndef BYTE_TO_BINARY
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
// the callback functions
/*! @brief  callback function to report there is an overcurrent */
overCurrentCallbackFunction 			g_overCurrentCallbackFunctionfp;

/*! @brief  callback function to change the LED color */
changeLedColorCallbackBatFuntion 		g_changeLedColorCallbackBatFuntionfp;

/*! @brief  callback function to act on new setted measurements */
newMeasurementsCallbackFunction 		g_newMeasurementsCallbackFunctionfp;

/*! @brief  mutex for controlling the gate */
static pthread_mutex_t gGateLock;		

/*! @brief  semaphore for the continue measure task*/
static sem_t gMeasureSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gMeasureSemInitialized = false; 

/*! @brief  semaphore for the continue measure task*/
static sem_t gCalcOtherSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gCalcOtherSemInitialized = false; 

/*! @brief  semaphore for the continues charging task*/
static sem_t gChargeSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gChargeSemInitialized = false; 

/*! @brief  semaphore for the self discharge task*/
static sem_t gSdChargeSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gSdChargeSemInitialized = false; 

/*! @brief  semaphore to enable the do selfdischarge function*/
static sem_t gDoSdChargeSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gDoSdChargeSemInitialized = false;

/*! @brief  semaphore to enable the do selfdischarge function*/
static sem_t gDoCellBalanceSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gDoCellBalanceSemInitialized = false;

/*! @brief variable to indicate of it is initialized */
static bool gBatManInitialized = false;	

static uint32_t gMeasCycleTime = 1000;
//static bool gMeasCycleLessThan1s = false;
static struct timespec gTargetTime;

/*! @brief  variable to pass to the sdchar task */
static bool gSelfDischargeOn = false;

/*! @brief  variable to pass to the sdchar task */
static bool gCellBalanceOn = false;

/*! @brief  mutex for the measureTime */
static pthread_mutex_t gMeasureTimeMutex;
static bool gMeasureTimeMutexInitialized = false;

/*! @brief  mutex for the balancing value */
static pthread_mutex_t gBalanceCellsMutex;
static bool gBalanceCellsMutexInitialized = false;

/*! @brief  mutex for the discharing handshake variable value */
static pthread_mutex_t gDisCharHandshakeVarMutex;
static bool gDisCharHandshakeVarMutexInitialized = false;

/*! @brief  mutex for the balancing value */
static pthread_mutex_t gEndOfChargeValueMutex;
static bool gEndOfChargeValueMutexInitialized = false;

/*! @brief  mutex for the balancing value */
static pthread_mutex_t gChargeToStorageVarMutex;
static bool gChargeToStorageVarMutexInitialized = false;

/*! @brief  mutex for the chargingstate variable */
static pthread_mutex_t chargingStateMutex;
static bool chargingStateMutexInitialized = false;	

/*! @brief  [-]         BCC configuration data */
//bcc_data_t      g_bccData;
bcc_drv_config_t gBccDrvConfig;

/*! @brief  [-]         BCC CT filter configuration data */
ct_filter_t     g_ctFilterComp;

/*! @brief  [-]         BCC I-sense filter configuration data */
isense_filter_t g_isenseFilterComp;

/*! @brief  [-]         BCC NTC configuration data */
ntc_config_t    g_ntcConfig;

/*! @brief  variable to keep track of the lowest cell voltage*/
float gLowestCellVoltage = V_CELL_OV_DEFAULT;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief 	function to do the meanual measurements, calculate current
 * 			and if the semaphore is availeable read the rest and do the calculations
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_MeasTaskFunc(int argc, char *argv[]);

/*!
 * @brief function to increase the semaphore at the right time to calculate the other measurements
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_OtherCalcTaskFunc(int argc, char *argv[]);

/*!
 * @brief function to check if the discharging/CB is done
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_SelfDischargeTaskFunc(int argc, char *argv[]);

/*!
 * @brief 	function used to calculate the new measureinterval to read and calculate 
 * 			voltages and temperatures
 * 			it will set the variable gMeasCycleTime with the right value
 * 
 * @param measMs the send interval in ms
 */
static void batManagement_calcSendInterval(uint16_t measMs);


/*!
 * @brief 	function to initialize the BCC
 * 		  	could return without init if communication failed!
 * 
 * @param none
 * @return the bcc status from bcc_status_t
 */
static bcc_status_t BatManagement_initializeBCC(void);

/*!
 * @brief 	function to set the battery under and over temperature threshold
 * 
 * @param 	chargingMode true if the temperature thresholds are set for the charging mode.
 * 			false if the temperature thresholds are set for the normal mode
 * @return 	the bcc status
 */
static bcc_status_t batManagement_setBatTempTHState(bool chargingMode);

/*!
 * @brief 	function to set of get the chargingMode variable
 * 
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the chargingMode variable
 */
static bool batManagement_setNGetChargingState(bool set, bool setValue); 

/*!
 * @brief 	function to set of get the active balancing cells the battery under and over temperature threshold
 * 			this function can be used by multiple threads
 * 
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the activeBalancingCells, UINT8_MAX if failed
 */
static uint8_t batManagement_setNGetActiveBalancingCells(bool set, uint8_t setValue);

/*!
 * @brief 	function to set of get the dischargingHandshakeVariable variable
 * 
 * @param 	forSelfDischarge if this is true it is for the self discharge handshake, 
 			otherwise it is for the balancing handshake
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the dischargingHandshakeVariable variable or UINT8_MAX for error
 */
static uint8_t batManagement_setNGetDisCharHandshakeVar(bool forSelfDischarge, bool set, bool setValue);  

/****************************************************************************
 * main
 ****************************************************************************/
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
							newMeasurementsCallbackFunction p_newMeasurementsCallbackFunction)
{
	int lvRetValue = !gBatManInitialized;
	int error, errcode;
	bool boolValue = 1;

	// check if already configured
	if(!gBatManInitialized)
	{

		// connect the callback functions
		g_overCurrentCallbackFunctionfp = p_overCurrentCallbackFunction;
		g_changeLedColorCallbackBatFuntionfp = p_changeLedColorCallbackBatFuntion;
		g_newMeasurementsCallbackFunctionfp = p_newMeasurementsCallbackFunction;

	  	// initialze the mutex
		pthread_mutex_init(&gGateLock, NULL);
		pthread_mutex_init(&gMeasureTimeMutex, NULL);
		gMeasureTimeMutexInitialized = true;
		pthread_mutex_init(&gBalanceCellsMutex, NULL);
 		gBalanceCellsMutexInitialized = true;
 		pthread_mutex_init(&gDisCharHandshakeVarMutex, NULL);
		gDisCharHandshakeVarMutexInitialized = true;
 		pthread_mutex_init(&gEndOfChargeValueMutex, NULL);
		gEndOfChargeValueMutexInitialized = true;
		pthread_mutex_init(&gChargeToStorageVarMutex, NULL);
		gChargeToStorageVarMutexInitialized = true;
  		pthread_mutex_init(&chargingStateMutex, NULL);
  		chargingStateMutexInitialized = true;

		// initialize the semaphore in the monitoring part
		error = bcc_monitoring_initializeSem();
		// check for errors
		if(error)
		{
			cli_printf("coulnd't initialize monitoring semaphore! error: %d\n", error);
			return error;
		}

		// initialize the semaphore
		error = sem_init(&gMeasureSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gMeasureSem, SEM_PRIO_NONE);
			// set the variable true
			gMeasureSemInitialized = true;
		}

		// initialize the semaphore
		error = sem_init(&gCalcOtherSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze other sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gCalcOtherSem, SEM_PRIO_NONE);
			// set the variable true
			gCalcOtherSemInitialized = true;
		}

		// initialize the semaphore
		error = sem_init(&gChargeSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze charge sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gChargeSem, SEM_PRIO_NONE);
			// set the variable true
			gChargeSemInitialized = true;
		}

		// initialize the semaphore
		error = sem_init(&gSdChargeSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze sdchar sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gSdChargeSem, SEM_PRIO_NONE);
			// set the variable true
			gSdChargeSemInitialized = true;
		}

		// initialize the semaphore
		error = sem_init(&gDoSdChargeSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze dosdchar sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gDoSdChargeSem, SEM_PRIO_NONE);
			// set the variable true
			gDoSdChargeSemInitialized = true;
		}

		// initialize the semaphore
		error = sem_init(&gDoCellBalanceSem, 0, 0);
		
		if(error)
		{
			// output to user
			cli_printf("batManagement ERROR: failed to initialze docellbalance sem! error: %d\n", error);
		}
		else
		{
			sem_setprotocol(&gDoCellBalanceSem, SEM_PRIO_NONE);
			// set the variable true
			gDoCellBalanceSemInitialized = true;
		}

		// create the task
		lvRetValue = task_create("meas", DEFAULT_MEASURE_PRIORITY, DEFAULT_MEASURE_STACK_SIZE_M, batManagement_MeasTaskFunc, NULL);
		// check for errors
		if(lvRetValue < 0)
	    {
	    	// inform user
	    	errcode = errno;
	      	cli_printf("batManagement: ERROR: Failed to start task: %d\n", errcode);
	      	return lvRetValue;
	    }

		// create the task
		lvRetValue = task_create("otherCalc", DEFAULT_MEASURE_PRIORITY, DEFAULT_MEASURE_STACK_SIZE_L, batManagement_OtherCalcTaskFunc, NULL);
		// check for errors
		if(lvRetValue < 0)
	    {
	    	// inform user
	    	errcode = errno;
	      	cli_printf("batManagement: ERROR: Failed to start task: %d\n", errcode);
	      	return lvRetValue;
	    }

		// create the task
		lvRetValue = task_create("sdchar", DEFAULT_MEASURE_PRIORITY, DEFAULT_MEASURE_STACK_SIZE_M, batManagement_SelfDischargeTaskFunc, NULL);
		// check for errors
		if(lvRetValue < 0)
	    {
	    	// inform user
	    	errcode = errno;
	      	cli_printf("batManagement: ERROR: Failed to start task: %d\n", errcode);
	      	return lvRetValue;
	    }

	    // reset the BCC
	    // write the reset pin 
	    lvRetValue = gpio_writePin(BCC_RESET, 1);

	    // wait a time longer than needed 
	    usleep(1000);

	    // pull down the reset pin
	    lvRetValue = gpio_writePin(BCC_RESET, 0);

		// initialize SPI mutex
		lvRetValue = BCC_initialze_spi_mutex();
		if(lvRetValue)
		{
			cli_printf("failed to initialze spi mutex!\n");
			return lvRetValue;
		}

	 	// // set that the battery managementment is configured
	 	gBatManInitialized = true;

	 	//turn off the gate
	 	batManagement_setGatePower(GATE_OPEN);

		cli_printf("SELF-TEST START: BCC\n");

		// initalize the BCC
		lvRetValue = BatManagement_initializeBCC(); 
		// check for errors
		if(lvRetValue < 0)
	    {
	    	// inform user
	    	//errcode = errno;
	      	cli_printf("batManagement: ERROR: Failed to initialze BCC: %d\n", lvRetValue);
	      	cli_printf("SELF-TEST FAIL:  BCC\n");
	      	return lvRetValue;
	    }

	    cli_printf("SELF-TEST PASS:  BCC\n");

	    // do the Gate check at least once 
	    do
	    {
	    	cli_printf("SELF-TEST START: GATE\n");

	    	//turn off the gate
	 		batManagement_setGatePower(GATE_OPEN);

			// check the gate output to verify if it works
			// do a measurement
			lvRetValue = bcc_monitoring_doBlockingMeasurement(&gBccDrvConfig);
			// check for errors
			if(lvRetValue != 0)
		    {
		    	// inform user
		    	//errcode = errno;
		      	cli_printf("batManagement: ERROR: Failed to do measurement: %d\n", lvRetValue);
		      	cli_printf("SELF-TEST FAIL:  GATE\n");
		      	return lvRetValue;
		    }

			// get the output voltage and check if it is off
			// read the output voltage 
			lvRetValue = bcc_monitoring_checkOutput(&gBccDrvConfig, &boolValue);

			// check for errors
			if(lvRetValue != 0)
		    {
		    	// inform user
		    	//errcode = errno;
		      	cli_printf("batManagement: ERROR: Failed get output: %d\n", lvRetValue);
		      	cli_printf("SELF-TEST FAIL:  GATE\n");
		      	return lvRetValue;
		    }

		    // check if the output is high
		    if(boolValue == 1)
		    {
		    	cli_printf("batManagement ERROR: Failed to disable the gate!\n");
		    	cli_printf("Make sure there is no charger connected and press the button to try again\n");
		    	cli_printf("SELF-TEST FAIL:  GATE\n");

		    	//lvRetValue = BCC_STATUS_DIAG_FAIL;

		    	// set the LED color to indicate the charger is connected and is wrong
		    	g_changeLedColorCallbackBatFuntionfp(RED_BLUE, LED_BLINK_ON);

		    	// loop until the button is not pushed 
		    	while(!gpio_readPin(SBC_WAKE))
		    	{
		    		// wait for a little while 
		    		usleep(1000*10);
		    	}

		    	// loop until the button is pushed 
		    	while(gpio_readPin(SBC_WAKE))
		    	{
		    		// wait for a little while 
		    		usleep(1000*10);
		    	}
		    }

		// loop while the output is high
		}while(boolValue == 1);

		cli_printf("SELF-TEST PASS:  GATE\n");

		// change the LED color to off
		g_changeLedColorCallbackBatFuntionfp(OFF, LED_BLINK_OFF);

	 	// change the return value
	 	lvRetValue = !gBatManInitialized;
	}

	// return the value
	return lvRetValue;
} 

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
int batManagement_setGatePower(bool on)
{
	int lvRetValue = -1;

	// check if initialized
	if(!gBatManInitialized)
	{
		// error
		cli_printf("ERROR: Battery management not initialized, pleaze initialze\n");
		return lvRetValue;
	}

	// set return value for 0, if everything goes right it will stay 0
	lvRetValue = 0;

	// lock the mutex
	pthread_mutex_lock(&gGateLock);

	// Set Data pin to 1 so GATE_IN signal turns 1 (power OFF) when clock rises.
	lvRetValue += gpio_writePin(GATE_CTRL_D, !on);			

	// Reset then set again Clock signal so Data gets through the flip-flop
	lvRetValue += gpio_writePin(GATE_CTRL_CP, 0);		

	// N.B.: flip-flop is working on rising edge.	
	lvRetValue += gpio_writePin(GATE_CTRL_CP, 1);			

	// Reset pin for idle mode.
	lvRetValue += gpio_writePin(GATE_CTRL_CP, 0);

	// unlock the mutex
	pthread_mutex_unlock(&gGateLock);

	// return value to the user
	return lvRetValue;
}

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
int batManagement_checkAFE(BMSFault_t *BMSFault, bool resetFaultPin)
{
	int lvRetValue = 0;

	// check for faults
	batManagement_checkFault(BMSFault, resetFaultPin);

	return lvRetValue;
}

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
int batManagement_checkFault(BMSFault_t *BMSFault, bool resetFaultPin)
{
	int lvRetValue = -1, i;
	bcc_status_t lvBccStatus;
	bcc_fault_status_t resetBCCFaultValue; 
	uint16_t lvBccFaultStatus[BCC_STAT_CNT]; 
	uint8_t NumCells = 0;
	uint8_t checkBCCCell = 0;
	int32_t *dataReturn = NULL;
	float lvCurrent, lvMaxCurrent;
	uint16_t maskedFaultReg = 0;

	// get the amount of cells
	dataReturn = (int32_t*)data_getParameter(N_CELLS, &NumCells, NULL);
	if(dataReturn == NULL)
	{
		cli_printf("batManagement_checkFault ERROR: couldn't get num cells\n");
		return lvRetValue;
	}

	// reset the fault value
	*BMSFault = 0;

	// get the BCC fault
	lvBccStatus = BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);

#ifdef DEBUG_FAULT_STATUS
	
	for(i = 0; i < BCC_STAT_CNT; i++)
	{
		if(lvBccFaultStatus[i])
		{
			cli_printf("faultStatus[%d]: %d\n", i, lvBccFaultStatus[i]);
		}
	}
#endif

	// check the OV and UV fault
	for(i = 0; i < NumCells; i++)
	{
		// check which bcc cell it is
		if(i >= 2)
        {
            // calculat the BCC pin index
            checkBCCCell = (6-NumCells) + i;
        }
        else
        {
            // it is the first 2 cells
            checkBCCCell = i;
        }

		// check for an cell overvoltage
		if((lvBccFaultStatus[BCC_FS_CELL_OV] >> checkBCCCell) & 1)
		{
			// set the cell and overvoltage 
			*BMSFault |= (1<<i);
			*BMSFault |= BMS_CELL_OV;
#ifdef DEBUG_OV_UV
			cli_printf("OV on: cell%d\n", i+1);

			dataReturn = (int32_t*)data_getParameter((parameterKind_t)(V_CELL1 + i), &lvCurrent, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_checkFault ERROR: couldn't get cell voltage\n");
			}
			else
			{
				cli_printf("cell %d voltage: %.3f\n", i+1, lvCurrent);
			}
#endif
		}

		// check for cell undervoltage
		if((lvBccFaultStatus[BCC_FS_CELL_UV] >> checkBCCCell) & 1)
		{
			// set the cell and undervoltage 
			*BMSFault |= (1<<i);
			*BMSFault |= BMS_CELL_UV;
#ifdef DEBUG_OV_UV
			cli_printf("UV on: cell%d\n", i+1);

			dataReturn = (int32_t*)data_getParameter((parameterKind_t)(V_CELL1 + i), &lvCurrent, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_checkFault ERROR: couldn't get cell voltage\n");
			}
			else
			{
				cli_printf("cell %d voltage: %.3f\n", i+1, lvCurrent);
			}
#endif
		}
	}

	if(lvBccFaultStatus[BCC_FS_CELL_OV])
	{
		// set the new value
		resetBCCFaultValue = BCC_FS_CELL_OV;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	if(lvBccFaultStatus[BCC_FS_CELL_UV])
	{
		// set the new value
		resetBCCFaultValue = BCC_FS_CELL_UV;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// TODO check PCB voltage (AN4?)

#ifdef DEBUG_OT_UT
			cli_printf("OT reg: "BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8)));
#endif
#ifdef DEBUG_OT_UT
			cli_printf("UT reg: "BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY((lvBccFaultStatus[BCC_FS_AN_OT_UT])));
#endif
	// check for an under and over temperature
	if(lvBccFaultStatus[BCC_FS_AN_OT_UT])
	{
#ifdef DEBUG_OT_UT
		if(lvBccFaultStatus[BCC_FS_AN_OT_UT] & ((1<<4) +(1<<(4+8))))
		{
			// get the batt voltage
			dataReturn = (int32_t*)data_getParameter(V_OUT, &lvCurrent, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_checkFault ERROR: couldn't get voltage\n");
			}
			else
			{
				cli_printf("AN4 voltage: %.3f\n", lvCurrent/VOLTDIV_BATT_OUT);
			}
		}
#endif
		// check for an undertemperature 
		if(lvBccFaultStatus[BCC_FS_AN_OT_UT] & 0xF)
		{
			// set the fault
			*BMSFault |= (lvBccFaultStatus[BCC_FS_AN_OT_UT] & 0xF) << BMS_FAULT_TEMPERATURE_BITS_SHIFT;
			*BMSFault |= BMS_UT;
#ifdef DEBUG_OT_UT
			cli_printf("OT on: "BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY((lvBccFaultStatus[BCC_FS_AN_OT_UT] & 0xF)));
#endif
		}

		// check for an overtemperature 
		if ((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8) & 0xF)
		{
			// set the fault
			*BMSFault |= ((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8) & 0xF) << BMS_FAULT_TEMPERATURE_BITS_SHIFT;
			*BMSFault |= BMS_OT;
#ifdef DEBUG_OT_UT
			cli_printf("OT on: "BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8) & 0xF)));
#endif
		}

		// set the new value
		resetBCCFaultValue = BCC_FS_AN_OT_UT;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}		
	}

	// check over current (coulomb count and sample?)
	dataReturn = (int32_t*)data_getParameter(I_BATT, &lvCurrent, NULL);
	if(dataReturn == NULL)
	{
		cli_printf("batManagement_checkFault ERROR: couldn't get current\n");
	}
	else
	{
		// check if discharging
		if(lvCurrent < 0)
		{
			// get the max discharge current
			dataReturn = (int32_t*)data_getParameter(I_OUT_MAX, &lvMaxCurrent, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_changedParameter ERROR: couldn't get max current\n");
			}
			else
			{
				// compare to check for an overcurrent fault
				if ((-1*lvCurrent) > lvMaxCurrent)
				{
					// set the overcurrent fault bit
					*BMSFault |= BMS_OVER_CURRENT;
				}
			}
		}

		// if it is charging
		else
		{		
			// get the max charge current
			dataReturn = (int32_t*)data_getParameter(I_CHARGE_MAX, &lvMaxCurrent, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_changedParameter ERROR: couldn't get max current\n");
			}
			else
			{
				// compare to check for an overcurrent fault
				if (lvCurrent > lvMaxCurrent)
				{
					// set the overcurrent fault bit
					*BMSFault |= BMS_OVER_CURRENT;
				}
			}

		}
	}

	// check for the sleep over current
	if(lvBccFaultStatus[BCC_FS_FAULT1] & BCC_RW_IS_OC_FLT_MASK)
	{
		// set the sleep overcurrent fault bit
		*BMSFault |= BMS_SLEEP_OC;
	}

	// check for a CC overflow
	if(lvBccFaultStatus[BCC_FS_FAULT3] & BCC_R_CC_OVR_FLT_MASK)
	{
		// set the CC overflow fault bit
		*BMSFault |= BMS_CC_OVERFLOW;
	}

	// check the BCC_FS_CB_OPEN fault register
	if (lvBccFaultStatus[BCC_FS_CB_OPEN])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_CB_OPEN reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_CB_OPEN], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_CB_OPEN] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_CB_OPEN]));
#endif
		// set the new value
		resetBCCFaultValue = BCC_FS_CB_OPEN;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// check the BCC_FS_CB_SHORT fault register
	if (lvBccFaultStatus[BCC_FS_CB_SHORT])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_CB_SHORT reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_CB_SHORT], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_CB_SHORT] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_CB_SHORT]));
#endif

		// set the new value
		resetBCCFaultValue = BCC_FS_CB_SHORT;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// check the BCC_FS_GPIO_STATUS fault register
	if (lvBccFaultStatus[BCC_FS_GPIO_STATUS])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_GPIO_STATUS reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_GPIO_STATUS], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_GPIO_STATUS] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_GPIO_STATUS]));
#endif
		// set the new value
		resetBCCFaultValue = BCC_FS_GPIO_STATUS;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// check the BCC_FS_GPIO_SHORT fault register
	if (lvBccFaultStatus[BCC_FS_GPIO_SHORT])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_GPIO_SHORT reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_GPIO_SHORT], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_GPIO_SHORT] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_GPIO_SHORT]));
#endif
		// set the new value
		resetBCCFaultValue = BCC_FS_GPIO_SHORT;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// check the BCC_FS_COMM fault register
	if (lvBccFaultStatus[BCC_FS_COMM])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_COMM reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_COMM], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_COMM] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_COMM]));
#endif
		// set the new value
		resetBCCFaultValue = BCC_FS_COMM;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// make the masked fault register
	maskedFaultReg = lvBccFaultStatus[BCC_FS_FAULT1] & ~(BCC_RW_I2C_ERR_FLT_MASK + BCC_RW_IS_OC_FLT_MASK + BCC_R_AN_OT_FLT_MASK
		+ BCC_R_AN_UT_FLT_MASK + BCC_R_CT_OV_FLT_MASK + BCC_R_CT_UV_FLT_MASK);

	if (maskedFaultReg)//& 0xFFBF)
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_FAULT1 reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 maskedFaultReg, BYTE_TO_BINARY(maskedFaultReg >> 8), BYTE_TO_BINARY(maskedFaultReg));
#endif
	}

	// check if fault1 register needs to be cleared
	if(lvBccFaultStatus[BCC_FS_FAULT1])
	{
		// set the new value
		resetBCCFaultValue = BCC_FS_FAULT1;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	if (lvBccFaultStatus[BCC_FS_FAULT2])
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_FAULT2 reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 lvBccFaultStatus[BCC_FS_FAULT2], BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_FAULT2] >> 8), BYTE_TO_BINARY(lvBccFaultStatus[BCC_FS_FAULT2]));
#endif

		// set the new value
		resetBCCFaultValue = BCC_FS_FAULT2;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// make the masked fault register
	maskedFaultReg = lvBccFaultStatus[BCC_FS_FAULT3] & ~(BCC_R_CC_OVR_FLT_MASK);

	// check for the other faults
	if (maskedFaultReg)
	{
		// set the fault
		*BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
		// output to the user
		cli_printf("BCC fault error: fault in BCC_FS_FAULT3 reg: %d: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN "\n",
		 maskedFaultReg, BYTE_TO_BINARY(maskedFaultReg >> 8), BYTE_TO_BINARY(maskedFaultReg));
#endif
	}

	if(lvBccFaultStatus[BCC_FS_FAULT3])
	{
		// set the new value
		resetBCCFaultValue = BCC_FS_FAULT3;

		// check if the pin needs to be resetted
		if(resetFaultPin)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
		
			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// output to the user
				cli_printf("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
			}
		}
	}

	// everything went ok
	lvRetValue = 0; 

	return lvRetValue;
}

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
int batManagement_setAFEMode(AFEmode_t mode)
{
	int lvRetValue = 0;
	static AFEmode_t previousAFEMode = AFE_NORMAL;

	// check what to do
	switch(mode)
	{
		case AFE_NORMAL:
			// wake the AFE
			BCC_WakeUp(&gBccDrvConfig);

			// do the verification
    		// check if SPI is initialized
    		lvRetValue = BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);
	    
			// check for errors
		  	if(lvRetValue)
		  	{
		  		// ouptut to the user 
		  		cli_printf("batManagement_setAFEMode ERROR: Couldn't verify com! %d\n", lvRetValue);
		  	}

			// set the cyclic measurement on
			lvRetValue = BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR,
				BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_CONTINOUS);

			if(lvRetValue)
			{
				// ouptut to the user
				cli_printf("batManagement_setAFEMode ERROR: Couldn't set measurements! %d\n", lvRetValue);
			}

		break;
		case AFE_SLEEP_MEAS:

			// set the cyclic measurement on
			lvRetValue = BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
				BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_CONTINOUS);

			// if comming from the normal mode
			if(previousAFEMode == AFE_NORMAL)
			{
				// set the BCC to sleep
				lvRetValue = BCC_Sleep(&gBccDrvConfig);
			}

		break;
		case AFE_SLEEP:

			// check if from sleep
			if(previousAFEMode != AFE_SLEEP_MEAS)
			{

				// set the cyclic measurement off
				lvRetValue = BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
					BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_DISABLED);
			}

			// if comming from the normal mode
			if(previousAFEMode == AFE_NORMAL)
			{
				// set the BCC to sleep
				lvRetValue = BCC_Sleep(&gBccDrvConfig);
			}
		break;
	}

	// save the previous AFE mode 
	previousAFEMode = mode;

	// return 
	return lvRetValue;
} 

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
int batManagement_changedParameter(parameterKind_t changedParam, void* newValue)
{
	int lvRetValue = -1;
	float lvFloatValue1, lvFloatValue2;
	int32_t lvInt32Value1, lvInt32Value2;
	int32_t *dataReturn = NULL;
	bool lvBoolVal = false;

	//check what needs to be done 
	switch(changedParam)
	{
		// in case it is the battery current
		case I_BATT:

			lvRetValue = -1;
			// check if the max isn't 
			// get current and the max current
			lvFloatValue1 = *(float*)newValue;

			// check if not charging mode 
			if(!batManagement_setNGetChargingState(false, 0))
			{
				// get the max pack current
				dataReturn = (int32_t*)data_getParameter(I_OUT_MAX, &lvFloatValue2, NULL);
				if(dataReturn == NULL)
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
					return lvRetValue;
				}
			}
			else
			{
				// get the max charge current
				dataReturn = (int32_t*)data_getParameter(I_CHARGE_MAX, &lvFloatValue2, NULL);
				if(dataReturn == NULL)
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
					return lvRetValue;
				}

				// check if needs to check for end of CB charge
				if((batManagement_SetNReadEndOfCBCharge(false, 0) & 1) != 1)
				{
					// get the end of charge current
					if(data_getParameter(I_CHARGE_FULL, &lvInt32Value1, NULL) == NULL)
				    {
				       cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
				       lvInt32Value1 = I_CHARGE_FULL_DEFAULT;
				    } 

				    // limit the value
				    lvInt32Value1 &= UINT16_MAX;

				    // check if the current is lower or equal than that current
				    if(lvFloatValue1 <= ((float)lvInt32Value1 / 1000))
				    {
				    	cli_printf("End of charge current %.3fA <= %.3fA\n", lvFloatValue1, ((float)lvInt32Value1 / 1000));
				    	
				    	// set the variable to indicate end of charge
				    	batManagement_SetNReadEndOfCBCharge(true, 1);
				    }
				}
			}

			//cli_printf("curr: %.3fA, max: %.3fA\n", fabs(lvFloatValue1), lvFloatValue2);

			// TODO make with a max burst (that the FETs can handle)
			// compare to check for an error
			if(fabs(lvFloatValue1) > lvFloatValue2)
			{

#ifdef DEBUG_OUTPUT_ERROR_CURRENT
				cli_printf("error current: %.3fA\n", lvFloatValue1);
#endif
				// call the callbackfunction for an overcurrent
				g_overCurrentCallbackFunctionfp();
			}

#ifdef OUTPUT_UPDATE
			// ouput to the user if needed
			cli_printf("Bat current: %.3fA\n", lvFloatValue1);
#endif
		break;


		// in case it is the remaining capacity or the full charge capacity 
		case A_REM:

			// get the value
			lvFloatValue1 = *(float*)newValue;

			// set the boolvalue true to indicate it is remaining capacity
			lvBoolVal = true;

#ifdef OUTPUT_UPDATE
			// ouput to the user if needed
			cli_printf("remaining capacity: %.3fAh\n", lvFloatValue1);
#endif
		case A_FULL:
			// calculate the new state of charge 

			// check which parameter to get
			// get the remaining capacity and full charge capacity 
			if(!lvBoolVal)
			{
				dataReturn = (int32_t*)data_getParameter(A_REM, &lvFloatValue1, NULL);
				if(dataReturn == NULL)
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
					return lvRetValue;
				}

				// get the full charge capacity
				lvFloatValue2 = *(float*)newValue;
			}
			else
			{
				dataReturn = (int32_t*)data_getParameter(A_FULL, &lvFloatValue2, NULL);
				if(dataReturn == NULL)
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
					return lvRetValue;
				}
			}

			// check for the limits
			if(lvFloatValue2 < lvFloatValue1)
			{
				// set the remaining capacity to be the full_charge_capacity
				if(data_setParameter(A_REM, &lvFloatValue2))
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
					return lvRetValue;
				}
			}
			else
			{
				// calculate the new state of charge 
				lvInt32Value1 = ((int32_t)((lvFloatValue1 / lvFloatValue2) * 100)) & UINT8_MAX;

				// save the new state of charge 
				if(data_setParameter(S_CHARGE, &lvInt32Value1))
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
					return lvRetValue;
				}
			}

#ifdef OUTPUT_UPDATE
			// ouput to the user if needed
			cli_printf("SoC: %d%%\n", lvInt32Value1);
#endif

			// if it is full charge capacity
			if(!lvBoolVal)
			{
				// get the factory capacity
				dataReturn = (int32_t*)data_getParameter(A_FACTORY, &lvFloatValue1, NULL);
				if(dataReturn == NULL)
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
					return lvRetValue;
				}
				
				// calculate the new State of health
				lvInt32Value1 = ((int32_t)((lvFloatValue2 / lvFloatValue1) * 100)) & UINT8_MAX;

				// set the new state of health
				if(data_setParameter(S_HEALTH, &lvInt32Value1))
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
					return lvRetValue;
				}

#ifdef OUTPUT_UPDATE
			// ouput to the user if needed
			cli_printf("SoH: %d%%\n", lvInt32Value1);
#endif
			}

			lvRetValue = 0;
			 
		break;

		case A_FACTORY:

			// convert the parameter
			lvFloatValue1 = *(float*)newValue;

			// get the factory capacity
			dataReturn = (int32_t*)data_getParameter(A_FULL, &lvFloatValue2, NULL);
			if(dataReturn == NULL)
			{
				cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
				return lvRetValue;
			}
			
			// calculate the new State of health
			lvInt32Value1 = ((int32_t)((lvFloatValue2 / lvFloatValue1) * 100)) & UINT8_MAX;

			// set the new state of health
			if(data_setParameter(S_HEALTH, &lvInt32Value1))
			{
				cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
				return lvRetValue;
			}

#ifdef OUTPUT_UPDATE
			// ouput to the user if needed
			cli_printf("SoH: %d%%\n", lvInt32Value1);
#endif

		break;

		case T_MEAS:
			lvInt32Value1 = (*(uint16_t*)newValue) & UINT16_MAX;
			// check if is not a whole division of T_MEAS_MAX
			if(T_MEAS_MAX % lvInt32Value1)
			{
				cli_printf("inserted T_meas: %d not a whole division of %d\n", lvInt32Value1, T_MEAS_MAX);
				// calculate lower measurement ratio
				while(T_MEAS_MAX % lvInt32Value1)
				{
					if(!lvBoolVal)
					{
						// decrease the update ratio
						lvInt32Value1--;

						if(lvInt32Value1 == 0)
						{
							// set the value
							lvBoolVal = true;

							//increase once for equasion
							lvInt32Value1++;
						}
					}
					else
					{
						// increase the update ratio
						lvInt32Value1++;
					}
				}

				// output to user
				cli_printf("setting new T_meas: %dms\n", lvInt32Value1);

				// set the new, faster ratio
				// save the new state of charge 
				if(data_setParameter(T_MEAS, &lvInt32Value1))
				{
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
					return lvRetValue;
				}
			}
			else
			{
				// calculate the new value
				batManagement_calcSendInterval((uint16_t)lvInt32Value1);
			}
		break;
		// check if the BCC_INIT_CONF needs to check 

		case C_BATT:
#ifdef OUTPUT_UPDATE
	
			lvFloatValue1 = *(float*)newValue;

			// ouput to the user if needed
			cli_printf("Batt temperature: %.3fC\n", lvFloatValue1);
#endif

		break;
		case V_OUT:
#ifdef OUTPUT_UPDATE			
			lvFloatValue1 = *(float*)newValue;

			// ouput to the user if needed
			cli_printf("Batt voltage: %.3fV\n", lvFloatValue1);
#endif

		break;
		case V_BATT:
#ifdef OUTPUT_UPDATE			
			lvFloatValue1 = *(float*)newValue;

			// ouput to the user if needed
			cli_printf("Batt pack voltage: %.3fV\n", lvFloatValue1);
#endif
		break;
		case P_AVG:
#ifdef OUTPUT_UPDATE			
			lvFloatValue1 = *(float*)newValue;

			// ouput to the user if needed
			cli_printf("Batt average power: %.3fA\n", lvFloatValue1);
#endif

		break;
		case V_CELL1:
		case V_CELL2:
		case V_CELL3:
		case V_CELL4:
		case V_CELL5:
		case V_CELL6:

			// save the value
			lvFloatValue1 = *(float*)newValue;

			// check if end of charge voltage needs to be checked
			if((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) != 2)
			{
				// check if it needs to be charged to the V_CELL_OV voltage
				if(!batManagement_SetNReadChargeToStorage(false, 0))
				{
					// get the cell over voltage
				    if(data_getParameter(V_CELL_OV, &lvFloatValue2, NULL) == NULL)
				    {
				       cli_printf("main ERROR: getting cell over voltage went wrong!\n");
				       lvFloatValue2 = V_CELL_OV_DEFAULT;
				    } 

				    // get the cell margin in mv
					if(data_getParameter(V_CELL_MARGIN, &lvInt32Value1, NULL) == NULL)
				    {
				       cli_printf("batManagement_selfDischarge ERROR: getting cell margin went wrong!\n");
				       lvInt32Value1 = V_CELL_MARGIN_DEFAULT;
				    } 

				    // limit the value 
			    	lvInt32Value1 &= UINT8_MAX;
				}
				// if charge to storage
				else
				{
					// get the cell over voltage
				    if(data_getParameter(V_STORAGE, &lvFloatValue2, NULL) == NULL)
				    {
				       cli_printf("main ERROR: getting storage voltage went wrong!\n");
				       lvFloatValue2 = V_STORAGE_DEFAULT;
				    } 

				    // set the margin to 0
				    lvInt32Value1 = 0;

				}

			    //check if the to charge to voltage is reached
			    if(lvFloatValue1 >= (lvFloatValue2 - ((((float)lvInt32Value1)/1000)/CHARGE_COMPLETE_MARGIN_DIV)))
			    {

			    	cli_printf("End of charge voltage! cell%d %.3f >= %.3f\n", ((changedParam - V_CELL1)+1),
			    	lvFloatValue1, (lvFloatValue2 - ((((float)lvInt32Value1)/1000)/CHARGE_COMPLETE_MARGIN_DIV)));
			    	// set the end of charge variable
			    	batManagement_SetNReadEndOfCBCharge(true, 2);
			    }
			    // if(lvFloatValue1 >= (lvFloatValue2))
			    // {
			    // 	// set the end of charge variable
			    // 	batManagement_SetNReadEndOfCBCharge(true, 1);
			    // }

			}

#ifdef OUTPUT_UPDATE			
			// ouput to the user if needed
			cli_printf("Cell%d voltage: %.3fV\n", ((changedParam - V_CELL1)+1), lvFloatValue1);
#endif

		break;

		case C_AFE:
		
#ifdef OUTPUT_UPDATE
			// get the temperature 			
			lvFloatValue1 = *(float*)newValue;

			// print to the user
			cli_printf("R temperature: %.3fC\n", lvFloatValue1);
#endif
			break;
		case C_T:

#ifdef OUTPUT_UPDATE
			// get the temperature 			
			lvFloatValue1 = *(float*)newValue;

			// print to the user
			cli_printf("T temperature: %.3fC\n", lvFloatValue1);
#endif
			break;
		case C_R:

#ifdef OUTPUT_UPDATE
			// get the temperature 			
			lvFloatValue1 = *(float*)newValue;

			// print to the user
			cli_printf("R temperature: %.3fC\n", lvFloatValue1);
#endif

		break;

		// in case of a new cell over voltage threshold
		case V_CELL_OV:

			// get the new value
			lvFloatValue1 = *(float*)newValue;

			// set the new upper cell voltage threshold in the BCC 
			bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, NULL, &lvFloatValue1);
		break;

		// in case of a new cell under voltage threshold
		case V_CELL_UV:

			// get the new value
			lvFloatValue1 = *(float*)newValue;

			// set the new upper cell voltage threshold in the BCC
			bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, &lvFloatValue1, NULL);

		break;

		// in case of the new PCB temperature thresholds
		case C_PCB_OT:
			// save the value in a float
			lvFloatValue1 = *(float*)newValue;

			// make the temperature bits (0b1101)
		    lvInt32Value1 = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

		    // seting PCB temperatuer threshold registers for upper threshold
		    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
		    	NULL, &lvFloatValue1);

		     // check for error
		    if(lvRetValue != BCC_STATUS_SUCCESS)
		    {
				cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
		        return lvRetValue;
		    }
		break;

		// in case of the new PCB temperature thresholds
		case C_PCB_UT:
			// save the value in a float
			lvFloatValue1 = *(float*)newValue;

			// make the temperature bits (0b1101)
		    lvInt32Value1 = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

		    // seting PCB temperatuer threshold registers for upper threshold
		    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
		    	&lvFloatValue1, NULL);

		     // check for error
		    if(lvRetValue != BCC_STATUS_SUCCESS)
		    {
				cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
		        return lvRetValue;
		    }
		break;

		// in case of the new cell temperature thresholds
		case C_CELL_OT:

			// check if not charging mode 
			if(!batManagement_setNGetChargingState(false, 0))
			{
				// set the upper threshold 

				// save the value in a float
				lvFloatValue1 = *(float*)newValue;

				// make the temperature bits
			    lvInt32Value1 = 1 << ANX_C_BATT;

			    // set the right temperature 
			    // seting PCB temperatuer threshold registers
			    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
			    NULL, &lvFloatValue1); 

			    // check for error
			    if(lvRetValue != BCC_STATUS_SUCCESS)
			    {
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
			        return lvRetValue;
			    }
			}

		break;

		// in case of the new cell temperature thresholds
		case C_CELL_UT:

			// check if not charging mode 
			if(!batManagement_setNGetChargingState(false, 0))
			{
				// set the lower threshold

				// save the value in a float
				lvFloatValue1 = *(float*)newValue;

				// make the temperature bits
			    lvInt32Value1 = 1 << ANX_C_BATT;

			    // set the right temperature 
			    // seting PCB temperatuer threshold registers
			    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
			    &lvFloatValue1, NULL); 

			    // check for error
			    if(lvRetValue != BCC_STATUS_SUCCESS)
			    {
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
			        return lvRetValue;
			    }
			}

		break;

		// in case of the new cell temperature thresholds
		case C_CELL_OT_CHARGE:

			// check if charging mode 
			if(batManagement_setNGetChargingState(false, 0))
			{
				// set the upper threshold 

				// save the value in a float
				lvFloatValue1 = *(float*)newValue;

				// make the temperature bits
			    lvInt32Value1 = 1 << ANX_C_BATT;

			    // set the right temperature 
			    // seting PCB temperatuer threshold registers
			    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
			    NULL, &lvFloatValue1); 

			    // check for error
			    if(lvRetValue != BCC_STATUS_SUCCESS)
			    {
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
			        return lvRetValue;
			    }
			}

		break;

		// in case of the new cell temperature thresholds
		case C_CELL_UT_CHARGE:

			// check if charging mode 
			if(batManagement_setNGetChargingState(false, 0))
			{
				// set the lower threshold 

				// save the value in a float
				lvFloatValue1 = *(float*)newValue;

				// make the temperature bits
			    lvInt32Value1 = 1 << ANX_C_BATT;

			    // set the right temperature 
			    // seting PCB temperatuer threshold registers
			    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, (uint8_t)lvInt32Value1, 
			    &lvFloatValue1, NULL); 

			    // check for error
			    if(lvRetValue != BCC_STATUS_SUCCESS)
			    {
					cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
			        return lvRetValue;
			    }
			}

		break;
		// in case the state of health changes
		case S_HEALTH:

			// get the state of health
			lvInt32Value1 = *(uint8_t*)newValue & UINT8_MAX;

			// get the bad battery threshold
			if(data_getParameter(BATT_EOL, &lvInt32Value2, NULL) == NULL)
         	{
         	  	cli_printf("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
			    return lvRetValue;
         	}

         	// limit the value 
         	lvInt32Value2 &= UINT8_MAX;

         	// check if it is a good battery
         	if(lvInt32Value1 <= lvInt32Value2)
         	{
         		// set the bad battery bit
         		data_statusFlagBit(STATUS_BAD_BATTERY_BIT, 1);
         	}
         	// else
         	// {
         	// 	// reset the bad battery bit
         	// 	data_statusFlagBit(STATUS_BAD_BATTERY_BIT, 0);
         	// }

         	// set the ask pars bit since the state of health changed
         	data_statusFlagBit(STATUS_ASK_PARS_BIT, 1);

		break;

		// if the number of cells change
		case N_CELLS:

			// get the cell count
			lvInt32Value1 = *(uint8_t*)newValue & UINT8_MAX;

			// call the function to change the cell count in the BCC
			lvRetValue = bcc_configuration_changeCellCount(&gBccDrvConfig, BCC_CID_DEV1, 
				(uint8_t)lvInt32Value1);

			// check for errors
			if(lvRetValue != BCC_STATUS_SUCCESS)
			{
				// output error 
				cli_printf("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
			    return lvRetValue;
			}

		break;

		// if the user want to disable or enable the temperature measurement
		case SENSOR_ENABLE:

			// get the value
			lvInt32Value1 = *(uint8_t*)newValue & 1;

		    // enable or disable the temperature sensor 
		    lvRetValue = bcc_configuration_disableNEnableANx(&gBccDrvConfig, BCC_CID_DEV1, 
		    	(1 << ANX_C_BATT), (!(bool)(lvInt32Value1)));

		    // check for error
		    if(lvRetValue != BCC_STATUS_SUCCESS)
		    {
		        cli_printf("batManagement_changedParameter ERROR: failed to set batt temperature measurement: %d\n", lvRetValue);
		        return lvRetValue;
		    }
    break;

		default:
			lvRetValue = 0;
	}

	// set the variable to ok
	lvRetValue = 0;

	// return to the user
	return lvRetValue;
}

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
int batManagement_updateMeasurementsOn(bool on)//, bool restart);
{
	int lvRetValue = 0;
	//int errcode;
	int semValueArr[2] = {0, 0};
	uint16_t measValue;
	static bool measurementsOn = false;

	// check if semaphores are initialized
	if(!gMeasureSemInitialized && !gCalcOtherSemInitialized)
	{
		cli_printf("batManagement_updateMeasurementsOn ERROR: semaphores not initialzed!\n");
		return 1;
	}

	// check if the task needs to be started
	if(on && !measurementsOn)
	{
		// get the T_meas value
		data_getParameter(T_MEAS, &measValue, NULL);

		// calculate the new time 
		batManagement_calcSendInterval(measValue);

		//cli_printf("increasing semaphore!\n");

		// post the semaphores
		sem_post(&gMeasureSem);
		sem_post(&gCalcOtherSem);

		// measurements on
		measurementsOn = true;

		do
		{
			// read the semaphores
			sem_getvalue(&gMeasureSem, &semValueArr[0]);
			sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

			//cli_printf("semaphore values before: %d, %d\n", semValueArr[0], semValueArr[1]);

			// check if there are more than one
			if(semValueArr[0] > 1)
			{
				// get the semaphore
				sem_wait(&gMeasureSem);
			}
			if(semValueArr[1] > 1)
			{
				// get the semaphore
				sem_wait(&gCalcOtherSem);
			}
		}
		while(semValueArr[0] > 1 || semValueArr[1] > 1);
	}
	else if (!on && measurementsOn)
	{
		//cli_printf("decreasing semaphore!\n");
		// decrease the semaphore
		sem_wait(&gMeasureSem);
		sem_wait(&gCalcOtherSem);

		// measurements off
		measurementsOn = false;

		do
		{
			// read the semaphores
			sem_getvalue(&gMeasureSem, &semValueArr[0]);
			sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

			//cli_printf("semaphore values before: %d, %d\n", semValueArr[0], semValueArr[1]);

			if(semValueArr[0] < 0)
			{
				// post the semaphore
				sem_post(&gMeasureSem);
			}
			if(semValueArr[1] < 0)
			{
				// post the semaphore
				sem_post(&gCalcOtherSem);
			}
		}
		while(semValueArr[0] < 0 || semValueArr[1] < 0);
	}

	// read the semaphores
	//sem_getvalue(&gMeasureSem, &semValueArr[0]);
	//sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

	//cli_printf("semaphore values after: %d, %d\n", semValueArr[0], semValueArr[1]);

	return lvRetValue;
}

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
int batManagement_doDiagnosticsOn(bool on)
{
	int lvRetValue = 0;
	return lvRetValue;
}

/*!
 * @brief 	This function is used to do a measurement
 * 			This function will start a conversion and wait (blocking)
 *			until the BCC is done with the measurement.
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_doMeasurement(void)
{
	int lvRetValue = -1;
	bcc_status_t error;

	// do the measurements and wait until done
	error = bcc_monitoring_doBlockingMeasurement(&gBccDrvConfig);

	// check for errors 
	if(error != BCC_STATUS_SUCCESS)
	{
		// output to the user
		cli_printf("batManagement_doMeasurement ERROR: failed to do measurement: %d\n", error);
	}

	// save the error
	lvRetValue = (int)error;

	// return
	return lvRetValue;
}

/*!
 * @brief 	This function is used to check if n_cells is in line with the measurements
 * @note 	A measurements should be done first.
 *
 * @param   nCellsOK the address of the variable to become 1 if the cells are OK.
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkNCells(bool *nCellsOK)
{
	int lvRetValue;

	// check if n cells is ok
	lvRetValue = bcc_monitoring_checkNCells(&gBccDrvConfig, nCellsOK);

	return lvRetValue;
}

/*!
 * @brief 	This function is used to get the output voltage and set it in the data struct
 * @note 	A measurements should be done first.
 *
 * @param   none
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_getOutputVoltage(void)
{
	int lvRetValue;

	// check if n cells is ok
	lvRetValue = bcc_monitoring_getOutputVoltage(&gBccDrvConfig);

	return lvRetValue;
}

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
int batManagement_startCharging(bool on)
{
	int lvRetValue = 0;

	batManagement_setNGetChargingState(true, on);

	return lvRetValue;
}

/*!
 * @brief 	This function is used to start or stop the self discharging sequence. 
 * 			it will self-discharge each cell to the storage voltage.
 * 			Normally it will stop after completion of the charging sequence, but a fault could occur.
 * 			It will start the self discharging task.
 * 			batManagement_initialize should be called before calling this function
 * 			
 * @param 	on if true it will start the task and if false it will delete the task.
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_selfDischarge(bool on)
{
	int lvRetValue = 0, i;
	bcc_status_t lvBccStatus;

	// set the value
	gSelfDischargeOn = on;

	// check if off to turn it off faster
	if(!on)
	{
		// to debug the CB 
		//cli_printf("setting CB driver OFF\n");
		// set everything off!
		// turn off the driver
		if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
		{
			lvRetValue = 0;
		}
		else
		{
			lvRetValue = -1;
			cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
		}

		// loop through them
		for(i = 0; i < 6; i++)
		{
			//cli_printf("setting CB %d off\n", i+1);
			// set every individual cell balance driver off
			lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// set the return value
				lvRetValue = -1;

				// output to the user
				cli_printf("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
				//return lvRetValue;
			}	
		}	

	    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
    	{
    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 0);
    	}
	}
	// if on
	else
	{
		// set the handshake variable for self discharge
		batManagement_setNGetDisCharHandshakeVar(true, true, 1);
	}

	// increase the do selfdicharge semaphores
	sem_post(&gDoSdChargeSem);
	//sem_post(&gSdChargeSem);

	return lvRetValue;
}

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
int batManagement_setBalancing(bool on)
{
	int lvRetValue = 0, i;
	bcc_status_t lvBccStatus;

	// set the variable
	gCellBalanceOn = on;

	// check if off to turn it off faster
	if(!on)
	{
		// debug CB 
		//cli_printf("setting CB driver OFF CB\n");
		// set everything off
		// turn off the driver
		lvBccStatus = BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);
		if(lvBccStatus == BCC_STATUS_SUCCESS)
		{
			lvRetValue = 0;
		}
		else
		{
			lvRetValue = -1;
			cli_printf("batManagement_setBalancing ERROR: couldn't turn off CB driver! %d\n", lvBccStatus);
			return lvRetValue;
		}		

		// loop through them
		for(i = 0; i < 6; i++)
		{
			//cli_printf("setting CB %d off CB\n", i);
			// set every individual cell balance driver off
			lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// set the return value
				lvRetValue = -1;

				// output to the user
				cli_printf("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
				//return lvRetValue;
			}	
		}	

		if(lvRetValue != BCC_STATUS_SUCCESS)
		{
			// could not turn of the CB drivers
			return lvRetValue;
		}

		// set the active balance cells variable to 0 
	    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
    	{
    		cli_printf("batManagement_setBalancing ERROR: can't set the activeBalancingCells with %d!\n", 0);
    	}
	}
	else
	{
		// set the handshake variable for balancing
		batManagement_setNGetDisCharHandshakeVar(false, true, 1);
	}

	//cli_printf("posting do sem balance sem with %d!\n", on);

	// post the sem to do the balancing 
	sem_post(&gDoCellBalanceSem);


	return lvRetValue;
}

/*!
 * @brief 	This function is used to check if cell balancing is active. 
 * 			
 * @return 	0 if cell balancing is not active, a positive number if so
 * 			if error a negative number will be returned to indicate the error. 
 */
int batManagement_checkBalancing(void)
{
	int lvRetValue;

	// get the value
	lvRetValue = batManagement_setNGetActiveBalancingCells(false, 0) & UINT8_MAX;

	// check the handshake variable
	lvRetValue |= batManagement_setNGetDisCharHandshakeVar(false, false, 0);

	// check for errors
	if(lvRetValue == UINT8_MAX)
	{
		// set the error value
		lvRetValue = -1;
	}

	return lvRetValue;
}

/*!
 * @brief 	This function is used to check if the output voltage is at least OUTPUT_ON_VOLTAGE
 * 			
 * @return 	1 if output is active (>= OUTPUT_ON_VOLTAGE)
 * 			0 if not active, -1 if something went wrong
 */
int batManagement_checkOutputVoltageActive(void)
{
	float outputVoltage;
	bool lvRetValue = -1;

	// get the output voltage 
    if(data_getParameter(V_OUT, &outputVoltage, NULL) == NULL)
    {
       cli_printf("batManagement_checkOutputVoltageActive ERROR: getting ouput voltage went wrong!\n");
       return lvRetValue;
    } 

    // check if it is active
    if(outputVoltage < (float)OUTPUT_ON_VOLTAGE)
    {
    	// cli_printf("output voltage %.3fV < %.3f", outputVoltage, OUTPUT_ON_VOLTAGE);
    	// set the value
    	lvRetValue = 0;
    }
    else
    {
    	// set the value
    	lvRetValue = 1;
    }

    // output to the user
    return lvRetValue;
}

/*!
 * @brief 	This function is used to get the highest cell voltage
 * 			
 * @return 	the highest cell voltage
 */
float batManagement_getHighestCellV(void)
{
	float lvRetValue = 0, cellVoltage;
	uint8_t nCells;
	int i;

	// get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
       cli_printf("batManagement_getHighestCellV ERROR: getting cell count went wrong!\n");
       nCells = N_CELLS_DEFAULT;      
    } 

	// check for which cell it needs to do this
    for(i = 0; i < nCells; i++)
    {
    	// get the cell voltage 
	    if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
	    {
	       cli_printf("batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i+1);
	       cellVoltage = 0;	      
	 	}

	 	// check if it is the highest
	 	if(cellVoltage > lvRetValue)
	 	{
	 		// set the new voltage
	 		lvRetValue = cellVoltage;
	 	} 
	}

	// if done return the highest voltage
	return lvRetValue;
}

/*!
 * @brief 	This function is used to get the lowest cell voltage
 * 			
 * @return 	the lowest cell voltage
 */
float batManagement_getLowestCellV(void)
{
	float lvRetValue = V_CELL1_MAX, cellVoltage;
	uint8_t nCells;
	int i;

	// get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
       cli_printf("batManagement_getHighestCellV ERROR: getting cell count went wrong!\n");
       nCells = N_CELLS_DEFAULT;      
    } 

	// check for which cell it needs to do this
    for(i = 0; i < nCells; i++)
    {
    	// get the cell voltage 
	    if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
	    {
	       cli_printf("batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i+1);
	       cellVoltage = V_CELL1_MAX;	      
	 	}

	 	// check if it is the highest
	 	if(cellVoltage < lvRetValue)
	 	{
	 		// set the new voltage
	 		lvRetValue = cellVoltage;
	 	} 
	}

	// if done return the highest voltage
	return lvRetValue;
}


/*!
 * @brief 	This function is used to check if the end of cell balancing charge is there
 * 
 * @param 	set true if the newValue needs to be set, false if read
 * @param 	newValue if set is true, this is the new value
 * 			
 * @return 	0 if it is not done, 1 if so with current and 2 if ended with voltage
 * 			if error, a negative number will be returned to indicate the error. 
 */
int batManagement_SetNReadEndOfCBCharge(bool set, int newValue)
{
	int lvRetValue = -1;
	static bool endOfChargeVariable = true;

	// check if not initialized 
	if(!gEndOfChargeValueMutexInitialized)
	{
		cli_printf("batManagement_SetNReadEndOfCBCharge ERROR mutex not initialzed!\n");
		return lvRetValue;
	}

	// lock the mutex
	pthread_mutex_lock(&gEndOfChargeValueMutex);

	// set the variable if needed 
	if(set)
	{
		// if(newValue == 0)
		// {
		// 	cli_printf("setting EndOfCBCharge to 0!\n");
		// }
		// set the variable 
		endOfChargeVariable = (bool)newValue;
	}

	// save the variable 
	lvRetValue = endOfChargeVariable;

	// unlock the mutex
	pthread_mutex_unlock(&gEndOfChargeValueMutex);

	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	This function is used to check if the charger should charge to storage voltage
 * 
 * @param 	set true if the newValue needs to be set, false if read
 * @param 	newValue if set is true, this is the new value
 * 			
 * @return 	0 if not charging to storage, 1 if so, -1 if error
 */
int batManagement_SetNReadChargeToStorage(bool set, bool newValue)
{
	int lvRetValue = -1;
	static bool chargeToStorageVariable = false;

	// check if not initialized 
	if(!gChargeToStorageVarMutexInitialized)
	{
		cli_printf("batManagement_SetNReadEndOfCBCharge ERROR mutex not initialzed!\n");
		return lvRetValue;
	}

	// lock the mutex
	pthread_mutex_lock(&gChargeToStorageVarMutex);

	// set the variable if needed 
	if(set)
	{
		// if(newValue == 0)
		// {
		// 	cli_printf("setting EndOfCBCharge to 0!\n");
		// }
		// set the variable 
		chargeToStorageVariable = newValue;
	}

	// save the variable 
	lvRetValue = chargeToStorageVariable;

	// unlock the mutex
	pthread_mutex_unlock(&gChargeToStorageVarMutex);

	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	This function is to save the remaining capacity to the full charge capacity
 * 			this will be used when it is at the end of the charge cycle
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_saveFullChargeCap(void)
{
	int lvRetValue = -1;
	float remainingCap;

	// get the remaining capacity 
    if(data_getParameter(A_REM, &remainingCap, NULL) == NULL)
    {
       cli_printf("batManagement_saveFullChargeCap ERROR: getting A_REM went wrong!\n");
       return lvRetValue;
 	}

	// save the full charge capacity
	if(data_setParameter(A_FULL, &remainingCap))
	{
		cli_printf("batManagement_saveFullChargeCap ERROR: couldn't set full charge cap\n");
		return lvRetValue;
	}

	// set the return variable
	lvRetValue = 0;

	// return
	return lvRetValue;
}

/*!
 * @brief 	This function is to calculate the remaining capacity 
 * 			this will be used when an CC overflow occures
 * @note 	it will read and reset the CC registers
 * 			
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_calcRemaningCharge(void)
{
	int lvRetValue = -1;
	float remainingCap, avgCurrent, deltaCharge;
	uint16_t samples;
	bcc_status_t lvBccStatus;
	uint16_t lvBccFaultStatus[BCC_STAT_CNT]; 
	uint16_t retRegVal;


	// get the remaining capacity 
    if(data_getParameter(A_REM, &remainingCap, NULL) == NULL)
    {
       cli_printf("batManagement_saveFullChargeCap ERROR: getting A_REM went wrong!\n");
       return lvRetValue;
 	}

 	// calculate dCharge and reset CC registors
 	lvRetValue = bcc_monitoring_calcDCharge(&gBccDrvConfig, &samples, &avgCurrent, &deltaCharge, true);

 	// check for errors
 	if(lvRetValue)
 	{
 		// output and return
 		cli_printf("batManagement_calcRemaningCharge ERROR: couldn't get delta charge!\n");
 		return lvRetValue;
 	}

 	// calc new remaining charge 
 	remainingCap = remainingCap + deltaCharge;

 	// set the new remaining charge 
    if(data_setParameter(A_REM, &remainingCap))
    {
       cli_printf("batManagement_saveFullChargeCap ERROR: setting A_REM went wrong! %.3f\n", remainingCap);
       return lvRetValue;
 	}

 	// ouput to the user
 	//cli_printf("calcRemaningCharge: Avg current: %.3fA samples: %d\n", avgCurrent, avgCurrent);

 	// get the BCC fault
	lvBccStatus = BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);

	// check for errors
	if(lvBccStatus != BCC_STATUS_SUCCESS)
	{
		// return error 
		cli_printf("batManagement_saveFullChargeCap ERROR: getting fault went wrong! %.3f\n", remainingCap);
       	return lvRetValue;
	}

	// check for a CC overflow
	if(lvBccFaultStatus[BCC_FS_FAULT3] & BCC_R_CC_OVR_FLT_MASK)
	{
		cli_printf("clearing CC overflow\n");

		// reset the CC by writing the CC_OVT and CC_P_OVF, CC_N_OVF, SAMP_OVF bits
		lvBccStatus = BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_ADC2_OFFSET_COMP_ADDR, 
			BCC_R_CC_OVT_MASK + BCC_R_SAMP_OVF_MASK + BCC_R_CC_N_OVF_MASK + BCC_R_CC_P_OVF_MASK, 0);

		// check for errors
		if(lvBccStatus != BCC_STATUS_SUCCESS)
		{
			// return error 
			cli_printf("batManagement_saveFullChargeCap ERROR: clearing CC went wrong! %.3f\n", remainingCap);
	       	return lvRetValue;
		}

		//read the register
		lvBccStatus |= BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1,
		 (BCC_REG_ADC2_OFFSET_COMP_ADDR), 1, &retRegVal);

		// get the BCC fault
		lvBccStatus = BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);
		if(lvBccFaultStatus[BCC_FS_FAULT3] & BCC_R_CC_OVR_FLT_MASK)
		{
			// clear the fault
			lvBccStatus = BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, BCC_FS_FAULT3);

			// check for errors
			if(lvBccStatus != BCC_STATUS_SUCCESS)
			{
				// return error 
				cli_printf("batManagement_saveFullChargeCap ERROR: clearing fault went wrong! %.3f\n", remainingCap);
		       	return lvRetValue;
			}
		}		
	}

 	// return 
 	lvRetValue = 0;
 	return lvRetValue;
}

/*!
 * @brief 	This function is used to set the OCV interrupt timer. 
 * 			This function will be used to calculate the next OCV time, and set the timer.
 * 			If the functions isn't called with false, the time of the timer will increase with 50%
 * 			until the T_OCV_CYCLIC1 time is reached. 
 * 			To stop the timer and to reset the timer value, this should be called with false.
 * 			
 * @param 	on true to calculate and set the next OCV timer, false to stop and reset the timer. 
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setOCVTimer(bool on)
{
	int lvRetValue = 0;

	// TODO make this

	// return
	return lvRetValue;
}

/*
 * @brief   This function is used to enable or disable the display of the measurements
 *
 * @param   showCommand which measurements to enable or disable
 * @param   value the new value of that bit
 *
 * @return  none
 */
void batManagement_setShowMeas(showCommands_t showCommand, bool value)
{
	// pass it through to the bcc_monitoring part
	bcc_monitoring_setShowMeas(showCommand, value);
}

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief 	function to do the meanual measurements, calculate current
 * 			and if the semaphore is availeable read the rest and do the calculations
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_MeasTaskFunc(int argc, char *argv[])
{
	int lvRetValue = 0;
	bcc_status_t bcc_status;

	// endless loop
	while(1)
	{
		// wait for the semaphore
		sem_wait(&gMeasureSem);

		// post a new semaphore to keep measuring if needed
		sem_post(&gMeasureSem);

		// do the measurements
		bcc_status = bcc_monitoring_updateMeasurements(&gBccDrvConfig, SHUNT_RESISTOR_UOHM, 
			&gLowestCellVoltage, &gGateLock);

		// check for errors
		if(bcc_status != BCC_STATUS_SUCCESS && bcc_status != SEM_OCCUPIED_ERROR)
		{
			// set the LED to red blinking
			//g_changeLedColorCallbackBatFuntionfp(RED, true);

			cli_printf("batManagement ERROR: failed to update measurements! error: %d\n", bcc_status);
		}

		// if it did all the measurements
		else if(bcc_status == BCC_STATUS_SUCCESS)
		{

			// callback that data needs to be send
			g_newMeasurementsCallbackFunctionfp();

			//cli_printf("lowest cell voltage: %.3f\n", gLowestCellVoltage);

			// get the semaphore value
			sem_getvalue(&gSdChargeSem, &lvRetValue);

			// check if the sem is not 1
			if(lvRetValue < 1)
			{
				// increase the balance semaphore
				sem_post(&gSdChargeSem);
			}
		}

		// sleep for little time
		usleep(1); 
	}

	// for compiler
	return lvRetValue;
}

/*!
 * @brief function to increase the semaphore at the right time to calculate the other measurements
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_OtherCalcTaskFunc(int argc, char *argv[])
{
	int lvRetValue = -1, ret;
	bcc_status_t bcc_status;
	struct timespec  waitTime;
	//uint8_t secondFlip = 0;
	int32_t intValue;

	// get the T_meas value
	data_getParameter(T_MEAS, &intValue, NULL);

	// limit the value
	intValue &= 0xFFFF;

	// calculate gMeasCycleTime
	batManagement_calcSendInterval((uint16_t)intValue);

	// make an endless loop
	while(1)
	{

		// wait for the semaphore, there will be one unless the should not be done any measurements
		sem_wait(&gCalcOtherSem);

		// post a new semaphore to keep goiong
		sem_post(&gCalcOtherSem);

		// lock mutex
		pthread_mutex_lock(&gMeasureTimeMutex);

		//cli_printf("updating measurements!\n");

		// update the measurements
		bcc_status = bcc_monitoring_doAllCalculations();

		// check for errors
		if(bcc_status != BCC_STATUS_SUCCESS)
		{
			cli_printf("batManagement ERROR: failed to update measurements! error: %d\n", bcc_status);
		}

	    // make the new gTargetTime time
		gTargetTime.tv_sec += (((gTargetTime.tv_nsec/1000) + gMeasCycleTime*1000) / ((MAX_NSEC/1000) + 1));
	    gTargetTime.tv_nsec = ((((gTargetTime.tv_nsec/1000) + gMeasCycleTime*1000) % ((MAX_NSEC/1000) + 1))*1000);

		// get the new time interval
		if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
		{
			cli_printf("batManagement ERROR: failed to get waitTime time!\n");
		}
		
		// make the waiting time base on the current time

		// substract the seconds 
		waitTime.tv_sec = gTargetTime.tv_sec - waitTime.tv_sec;

		if(gTargetTime.tv_nsec - waitTime.tv_nsec < 0)
		{
			// substract and add the max
			waitTime.tv_nsec = gTargetTime.tv_nsec - waitTime.tv_nsec + MAX_NSEC + 1;

			// substract one second
			waitTime.tv_sec--;
		}
		else
		{
			// substract
			waitTime.tv_nsec = gTargetTime.tv_nsec - waitTime.tv_nsec;
		}

		// check if the values are less than zero (if pauzed or didn't make it in time)
		if((waitTime.tv_sec < 0 || waitTime.tv_sec > 10)  || waitTime.tv_nsec < 0)
		{
			// set the target time to the right next target time
			// get the new time interval
			if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
			{
				cli_printf("batManagement ERROR: failed to get waitTime time2!\n");
			}

			//cli_printf("new cur: %d s %d ms\n", waitTime.tv_sec, waitTime.tv_nsec/1000000);

			// check if the current time is indeed more than the target time
			if((waitTime.tv_sec > gTargetTime.tv_sec) || 
			  ((waitTime.tv_sec == gTargetTime.tv_sec) && (waitTime.tv_nsec > gTargetTime.tv_nsec)))
			{

				// calculate the difference
				intValue = (((waitTime.tv_sec*1000) + (waitTime.tv_nsec/1000000)) - 
				((gTargetTime.tv_sec*1000) + (gTargetTime.tv_nsec/1000000)));

				//cli_printf("diff: %d ms\n", intValue);

				// make the right targetTime
				// calc the sec (using only us precision)
				gTargetTime.tv_sec += (((gTargetTime.tv_nsec/1000) + 
					((((int)(intValue / gMeasCycleTime) + 1) * gMeasCycleTime)*1000)) / 
				((MAX_NSEC/1000) + 1));

				// calc the nsec (using only us precision)
				gTargetTime.tv_nsec = ((((gTargetTime.tv_nsec/1000) + 
					((((int)(intValue / gMeasCycleTime) + 1) * gMeasCycleTime)*1000))) % 
				((MAX_NSEC/1000) + 1))*1000;
				
				//cli_printf("adding:  %d ms\n", (((int)(intValue / gMeasCycleTime) + 1) * gMeasCycleTime));
				
				//cli_printf("new tar: %d s %d ms\n", gTargetTime.tv_sec, gTargetTime.tv_nsec/1000000);
				
				// make the wait time to wait for the next interval
				// calc the sec
				waitTime.tv_sec = ((gMeasCycleTime - (intValue % gMeasCycleTime))*1000) / 
				((MAX_NSEC/1000) + 1);

				// calc the nsec
				waitTime.tv_nsec = (((gMeasCycleTime - (intValue % gMeasCycleTime))*1000) % 
					((MAX_NSEC/1000) + 1))*1000;

				//cli_printf("new wait: %d s %d ms\n", waitTime.tv_sec, waitTime.tv_nsec/1000000);
			}
			else
			{
				// set to 0 to make sure this sleep is skipped
				waitTime.tv_sec = 0;
				waitTime.tv_nsec = 0;
			}
		}
		
		// lock mutex
		pthread_mutex_unlock(&gMeasureTimeMutex);

		//printf("sleeptime: %d s %d ms\n", waitTime.tv_sec, waitTime.tv_nsec/1000000);
		
    	// sleep for the amount of time
	    
	    ret = nanosleep(&waitTime, &waitTime);
	    if(ret)
	    {
	    	cli_printf("batManagement ERROR: meas nanosleep went wrong! error: %d time: %ds %dus\n", 
	    		errno, waitTime.tv_sec, waitTime.tv_nsec/1000);
	    }
	}

	return lvRetValue;
}

/*!
 * @brief function to check if the discharging/CB is done
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_SelfDischargeTaskFunc(int argc, char *argv[])
{
	int lvRetValue = -1, i, index;
	uint8_t nCells = 0, cellMarginMv = 0;
	float dischargeVoltage = 0, cellVoltage = 0, ocvSlope;
	uint16_t balanceMin = 0;
	uint8_t balancingCells = 0;	
	int error;
	bcc_status_t lvBccStatus;
	static bool driverOn = true;
	static bool goToStorageVoltage = false;

	// loop 
	while(1)
	{
		// wait for the semaphore to be increased after the measuremetns are done 
		sem_wait(&gSdChargeSem);

		// check the semaphore if the other measurements need to be done
	    if(sem_trywait(&gDoSdChargeSem))
	    {
	    	// save errno 
	    	error = errno;

	    	 // check if the semaphore is occupied, than stop after this measurements
		    if(error == EAGAIN)
		    {
		    	// sem occupied
		    }
		    else if (error)
		    {
		        cli_printf("batManagement_SelfDischargeTaskFunc ERROR: \nsem_trywait(&gDoSdChargeSem) error: %d\n", error);
		        //cli_printf("EAGAIN = %d, EDEADLK = %d, EINTR = %d, EINVAL = &d\n", EAGAIN, EDEADLK, EINTR, EINVAL);
		        //return error;
		    }
	    }
	    // the user used the selfdischarge function and has set the gSelfDischargeOn variable
	    else
	    {
	    	// do the self balance function
	    	// check if balancing needs to be on
			if(gSelfDischargeOn)
			{
				// set the variable on 
				goToStorageVoltage = true;

				// get the number of cells
			    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
			       nCells = N_CELLS_DEFAULT;			      
			    } 

			    // check for errors
			    if(nCells < 3 || 6 < nCells)
			    {
			    	cli_printf("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
			    	nCells = N_CELLS_DEFAULT;
			    	//return lvRetValue;
			    }

			    // get the storage voltage
			    if(data_getParameter(V_STORAGE, &dischargeVoltage, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
			       dischargeVoltage = V_STORAGE_DEFAULT;			      
			    } 

			    // get the cell margin in mv
				if(data_getParameter(V_CELL_MARGIN, &cellMarginMv, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting cell margin went wrong!\n");
			       cellMarginMv = V_CELL_MARGIN_DEFAULT;			      
			    } 

			    // get the OCV slope
			    if(data_getParameter(OCV_SLOPE, &ocvSlope, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");			      
			       ocvSlope = OCV_SLOPE_DEFAULT;
			    } 

			    //cli_printf("setting driver OFF\n");

			    // turn off the driver
				if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
				{
					// clear the variable if ok
					driverOn = false;
				}
				else
				{
					cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
				}	

				// reset the variable
				balancingCells = 0;		

				// check for which cell it needs to do this
			    for(i = 0; i < nCells; i++)
			    {
			    	// map the cells (1, 2, 3, ...) to the BCC cells (1, 2, ..., 6) 
			    	if(i >= 2)
			        {
			            // calculat the BCC pin index
			            index = (6-nCells) + i;
			        }
			        else
			        {
			            // it is the first 2 cells
			            index = i;
			        }

			    	// get the cell voltage 
			    	 // get the storage voltage
				    if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
				    {
				       cli_printf("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i);
				       cellVoltage = 3.5;				      
				    } 

				    // output equation to the user
				    cli_printf("Discharge when cell%d voltage: %.3f > %.3f\n", i+1, cellVoltage, (dischargeVoltage + ((float)cellMarginMv/1000)));

				    // check if the CB driver should be on for this cell
				    if(cellVoltage > (dischargeVoltage + ((float)cellMarginMv/1000)))
				    {
				    	// TODO use this formula? and check it?
				    	// calculate the CB timer
				    	//balanceMin = ((cellVoltage – dischargeVoltage)*RBAL)/(cellVoltage*ocvSlope);
				    	balanceMin = 0xFF;

				    	//cli_printf("setting CB %d on SD\n", index+1);

				    	// write the cell register
				    	lvBccStatus =  BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, true, balanceMin);
				    	
				    	// check for errors
				    	if(lvBccStatus != BCC_STATUS_SUCCESS)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
				    	}

				    	// increase the balancing variable 
				    	balancingCells |= (1<<i);
				    }
				    else
				    {
				    	//cli_printf("setting CB %d off SD\n", index+1);

				    	// write the cell register
				    	lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, false, 0xFF);

				    	// check for errors
				    	if(lvBccStatus != BCC_STATUS_SUCCESS)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
				    	}

				    	// clear the bit in the balancing variable 
				    	balancingCells &= ~(1<<i);
				    }
			    }

			    cli_printf("Setting cell self discharge on for cell: ");

			    // check which cells have CB on
			    for (i = 0; i < 6; i++)
			    {
			    	// check if on
			    	if(balancingCells & (1 << i))
			    	{
			    		// print the cell number
			    		cli_printf("%d, ", i+1);
			    	}
			    }

			    // remove the ","
			    cli_printf("\b\b \n");

			    // check if balancing is on
			    if(balancingCells)
			    {
			    	//cli_printf("enabling CB driver\n");
			    	// turn off the driver
					if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, true) == BCC_STATUS_SUCCESS)
					{
						// clear the variable if ok
						driverOn = true;
					}
					else
					{
						cli_printf("batManagement_selfDischarge ERROR: couldn't turn on CB driver!\n");
					}			

			    	// set the global variable 
			    	// get them
			    	balancingCells |= batManagement_setNGetActiveBalancingCells(false, 0);

			    	// set themn
			    	if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
			    	{
			    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", balancingCells);
			    	}
			    }

			    // set the handshake variable for self discharge to 0
				batManagement_setNGetDisCharHandshakeVar(true, true, 0);
			}
			else
			{
				// check if it was self discharging 
				if(goToStorageVoltage)
	    		{
	    			//cli_printf("disabling CB driver\n");
					// turn off cell balancing
					// turn off the driver
					if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
					{
						// clear the variable if ok
						driverOn = false;
					}
					else
					{
						cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
					}		

					// loop through them
					for(i = 0; i < 6; i++)
					{
						//cli_printf("setting CB %d off SD2\n", i+1);
						// set every individual cell balance driver off
						lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

						// check for errors
						if(lvBccStatus != BCC_STATUS_SUCCESS)
						{
							// set the return value
							lvRetValue = -1;

							// output to the user
							cli_printf("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
						}	
					}	

					// set the global variable 
				    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
			    	{
			    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 0);
			    	}
			    }
			}
	    }

	    // check the semaphore if the user want to enable cell balaning
	    if(sem_trywait(&gDoCellBalanceSem))
	    {
	    	// save errno 
	    	error = errno;

	    	 // check if the semaphore is occupied, than stop after this measurements
		    if(error == EAGAIN)
		    {
		    	// sem occupied
		    }
		    else if (error)
		    {
		        cli_printf("batManagement_SelfDischargeTaskFunc ERROR: \nsem_trywait(&gDoCellBalanceSem) error: %d\n", error);
		    }
	    }
	   	else
	    {
	    	// check the variable to see if cell balancing needs to be on or off
	    	if(gCellBalanceOn)
	    	{
	    		// set the variable off
				goToStorageVoltage = false;

	    		// calculate for which cell the cell balance function needs to be on
	    		// get the number of cells
			    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
			       nCells = N_CELLS_DEFAULT;
			    } 

			    // get the cell margin in mv
				if(data_getParameter(V_CELL_MARGIN, &cellMarginMv, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting cell margin went wrong!\n");
			       cellMarginMv = V_CELL_MARGIN_DEFAULT;			      
			    } 

			    // get the OCV slope
			    if(data_getParameter(OCV_SLOPE, &ocvSlope, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");			      
			       ocvSlope = OCV_SLOPE_DEFAULT;
			    } 

			    //cli_printf("setting CB driver OFF CBon\n");

			    // turn off the driver
				if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
				{
					// clear the variable if ok
					driverOn = false;
				}
				else
				{
					cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
				}		

				// reset the variable
				balancingCells = 0;	

				// check for which cell it needs to do this
			    for(i = 0; i < nCells; i++)
			    {
			    	// set the bcc index
			    	if(i >= 2)
			        {
			            // calculate the BCC pin index
			            index = (6-nCells) + i;
			        }
			        else
			        {
			            // it is the first 2 cells
			            index = i;
			        }

			    	// get the cell voltage 
				    if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
				    {
				       cli_printf("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i);
				       cellVoltage = V_CELL1_DEFAULT;				      
				    } 

				    // output equation to the user
				    cli_printf("Balance when cell%d voltage: %.3f > %.3f\n", i+1, cellVoltage, (gLowestCellVoltage+((float)cellMarginMv/1000)));

				    // check if the CB driver should be on for this cell
				    if(cellVoltage > (gLowestCellVoltage+((float)cellMarginMv/1000)))
				    {
				    	// TODO use this formula? and check it?
				    	// calculate the CB timer
				    	//balanceMin = ((cellVoltage – gLowestCellVoltage)*RBAL)/(cellVoltage*ocvSlope);
				    	balanceMin = 0xFF;

				    	//cli_printf("setting CB %d on CBon\n", index +1);
				    	// write the cell register
				    	lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, true, balanceMin);
				    	
				    	// check for errors
				    	if(lvBccStatus != BCC_STATUS_SUCCESS)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
				    	}

				    	// increase the balancing variable 
				    	balancingCells |= (1<<i);
				    }
				    else
				    {

				    	//cli_printf("setting CB %d off CBon\n", index +1);
				    	// write the cell register
				    	lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, false, 0xFF);
				    	// check for errors
				    	if(lvBccStatus != BCC_STATUS_SUCCESS)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: couldnt turn off cell%d balance: %d\n", i, lvRetValue);
				    	}

				    	// clear the bit in the balancing variable 
				    	balancingCells &= ~(1<<i);
				    }
			    }

			    // output to the user
			    // cli_printf("setting cell balance on for "BYTE_TO_6BINARY_PATTERN" = %d\n", 
			    cli_printf("setting cell balance on for cell: ");

			    // check which cells have CB on
			    for (i = 0; i < 6; i++)
			    {
			    	// check if on
			    	if(balancingCells & (1 << i))
			    	{
			    		// print the cell number
			    		cli_printf("%d, ", i+1);
			    	}
			    }

			    // remove the ","
			    cli_printf("\b\b \n");

			    //cli_printf("balancingCells = %d\n", balancingCells);

			    // check if balancing is on
			    if(balancingCells)
			    {
			    	// let the LED blink blue to indicate balancing
					g_changeLedColorCallbackBatFuntionfp(BLUE, LED_BLINK_ON);

					//cli_printf("setting CB driver ON CBon\n");
			    	// turn on the driver
					if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, true) == BCC_STATUS_SUCCESS)
					{
						// clear the variable if ok
						driverOn = true;
					}
					else
					{
						cli_printf("batManagement_selfDischarge ERROR: couldn't turn on CB driver!\n");
					}			

			    	// set the global variable 
			    	balancingCells |= batManagement_setNGetActiveBalancingCells(false, 0);

			    	if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
			    	{
			    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", balancingCells);
			    	}
			    }

			    // set the handshake variable for balancing to 0
				batManagement_setNGetDisCharHandshakeVar(false, true, 0);
	    	}
	    	// if balancing needs to be off
	    	else
	    	{
	    		// check if it was balancing
	    		if(!goToStorageVoltage)
	    		{

	    			//cli_printf("setting CB driver OFF CBoff\n");
	    			// turn off the driver
		    		// turn off the driver
					if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
					{
						// clear the variable if ok
						driverOn = false;
					}
					else
					{
						cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
					}	

					// loop through them
					for(i = 0; i < 6; i++)
					{

						//cli_printf("setting CB %d OFF CBoff\n", i+1);
						// set every individual cell balance driver off
						lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

						// check for errors
						if(lvBccStatus != BCC_STATUS_SUCCESS)
						{
							// set the return value
							lvRetValue = -1;

							// output to the user
							cli_printf("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
							//return lvRetValue;
						}	
					}	

					// set the active balance variable 
				    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
			    	{
			    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 0);
			    	}
	    		}
	    	}
	    }
	    
		// check if balancing is not done
		if(batManagement_setNGetActiveBalancingCells(false, 0))
		{
			// check if the cells need to be balanced to the lowest cell voltage or the storage voltage
			if(goToStorageVoltage)
			{
				// get the storage voltage
			    if(data_getParameter(V_STORAGE, &dischargeVoltage, NULL) == NULL)
			    {
			       cli_printf("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
			       dischargeVoltage = V_STORAGE_DEFAULT;			      
			    } 
			}
			else
			{
				// save the lowest cell voltage in dischargeVoltage for balancing during charging
				// save the lowest cell voltage
				dischargeVoltage = gLowestCellVoltage;
			}

			// go through each ell
			for(i = 0; i< 6; i++)
			{
				if(batManagement_setNGetActiveBalancingCells(false, 0) & (1<<i))
				{
					// get the cell voltage
					if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
				    {
				       cli_printf("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i);
				       cellVoltage = V_CELL1_DEFAULT;				      
				    } 

				    // check if the cell voltage is not higher than the to discharge to voltage
				    if(cellVoltage <= dischargeVoltage)
				    {

				    	// get the number of cells
					    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
					    {
					       cli_printf("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
					       nCells = N_CELLS_DEFAULT;					      
					    } 

				    	// set the bcc index
				    	if(i >= 2)
				        {
				            // calculate the BCC pin index
				            index = (6-nCells) + i;
				        }
				        else
				        {
				            // it is the first 2 cells
				            index = i;
				        }

				        //cli_printf("setting CB %d off check\n", index +1);

				    	// turn off cell balancing
				    	lvBccStatus = BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, false, 0xFF);
				    	if(lvBccStatus != BCC_STATUS_SUCCESS)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: could not set cell CB %d\n", lvBccStatus);
				    	}

				    	// clear the bit in the variable
				    	balancingCells = batManagement_setNGetActiveBalancingCells(false, 0) & ~(1<<i);

				    	// set the new variable
				    	if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
				    	{
				    		cli_printf("batManagement_selfDischarge ERROR: can't set the activeBalancingCells2 with %d!\n", balancingCells);
				    	}
				    }
				}
			}

			// check if no cell balancing is active, turn on the driver
			if(!batManagement_setNGetActiveBalancingCells(false, 0))
			{
				 //cli_printf("setting CB driver off check\n");
				// turn off the driver
				if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
				{
					// clear the variable if ok
					driverOn = false;
				}
				else
				{
					cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
				}			
			}
		}
		else
		{
			// check if the driver is on
			if(driverOn)
			{
				//cli_printf("setting CB driver off check\n");
				// turn off the driver
				if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false) == BCC_STATUS_SUCCESS)
				{
					// clear the variable if ok
					driverOn = false;
				}
				else
				{
					cli_printf("batManagement_selfDischarge ERROR: couldn't turn off CB driver!\n");
				}				
			}
			
		}
	}

	return lvRetValue;
}

/*!
 * @brief 	function used to calculate the new measureinterval to read and calculate 
 * 			voltages and temperatures
 * 			it will set the variable gMeasCycleTime with the right value
 * 
 * @param measMs the send interval in ms
 */
static void batManagement_calcSendInterval(uint16_t measMs)
{
	uint8_t period = T_MEAS_MAX/measMs;
	uint8_t elements = 10;

	// check if initialzed
	if(gMeasureTimeMutexInitialized)
	{
		// lock mutex
		pthread_mutex_lock(&gMeasureTimeMutex);

		// set the new average interval to every second if possible 
		if(measMs <= 1000)
		{
			// calculate the new amount of elements if there is rest 
			while(period%elements)
			{
				// decrease the elements
				elements--;
			}

			// set the interval to make sure every second the moving avg is done
			bcc_monitoring_setAverageInterval((uint8_t)period/elements, elements);
		}
		else
		{
			// set the interval to every time and check how many elements it has
			bcc_monitoring_setAverageInterval(1, period);
		}

		// get the time before the start of the measurements
		// to make sure the duration of the time cycle of the measurements starts from this
		// so the duration of the measurements is not neglected 
		if(clock_gettime(CLOCK_REALTIME, &gTargetTime) == -1)
		{
			cli_printf("batManagement ERROR: failed to get gTargetTime time!\n");
		}

		// make the cycletime for ms
		gMeasCycleTime = measMs;

		// set the next measurement target
		gTargetTime.tv_sec--;

		// set the nsec to max to make sure there is no negative number 
		gTargetTime.tv_nsec = MAX_NSEC;
		//}

		// unlock mutex
		pthread_mutex_unlock(&gMeasureTimeMutex);
	}
}

/*!
 * @brief 	function to initialize the BCC
 * 		  	could return without init if communication failed!
 * 
 * @param none
 * @return the bcc status from bcc_status_t
 */
static bcc_status_t BatManagement_initializeBCC(void)
{
	int lvRetValue = -1, i;
	float lowerTH, upperTH;
	uint16_t PCBOV, PCBUV;
	uint8_t configBits, nCells, measCycle;
	//bool enableBatTemp;
	void* dataReturn;

	/* Calculate BCC diagnostic time constants (g_bccData.diagTimeConst). */
    /* CT filter components. */
    g_ctFilterComp.rLpf1 = 3000U;            /* R_LPF-1 3kOhm */
    g_ctFilterComp.rLpf2 = 2000U;            /* R_LPF-2 2kOhm */
    g_ctFilterComp.cLpf = 100U;              /* C_LPF 100nF */
    g_ctFilterComp.cIn = 47U;                /* C_IN 47nF */

    /* ISENSE filter components. */
    g_isenseFilterComp.rLpfi = 127U;         /* R_LPFI 127Ohm */
    g_isenseFilterComp.cD = 2200U;           /* C_D 2.2uF */
    g_isenseFilterComp.cLpfi = 47U;          /* C_LPFI 47nF */
    g_isenseFilterComp.rShunt = 500U;		 /* R_SHUNT 0.5mOhm */
    g_isenseFilterComp.iMax = 24570U;        /* I_MAX 24.57A */

    //bcc_diag_calcDiagConst(&g_ctFilterComp, &g_isenseFilterComp, &g_bccData.diagTimeConst);

    /* Initialize BCC driver configuration structure (gBccDrvConfig). */
    gBccDrvConfig.drvInstance = BCC_INITIAL_DRIVER_INSTANCE;
    gBccDrvConfig.devicesCnt = BCC_DEVICES;

    gBccDrvConfig.device[BCC_FIRST_INDEX] = BCC_DEVICE_MC33772;
    gBccDrvConfig.cellCnt[BCC_FIRST_INDEX] = BCC_DEFAULT_CELLCNT;
    gBccDrvConfig.commMode = BCC_MODE_SPI;

    /* Precalculate NTC look up table for fast temperature measurement. */
    g_ntcConfig.rntc = NTC_PULL_UP;              /* NTC pull-up 10kOhm */
    g_ntcConfig.refTemp = NTC_REF_TEMP;          /* NTC resistance 10kOhm at 25 degC */
    g_ntcConfig.refRes = NTC_REF_RES;            /* NTC resistance 10kOhm at 25 degC */
    g_ntcConfig.beta = NTC_BETA;
    bcc_monitoring_fillNtcTable(&g_ntcConfig);

    i = 0;

    // do the verification
    do
    {
    	// check if SPI is initialized
    	lvRetValue = BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);
	    
	    i++;
    }while(lvRetValue != BCC_STATUS_SUCCESS || i == 3);
	
	// check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to verify com: %d\n", lvRetValue);
        return lvRetValue;
    }

    /* Initialize BCC device */
    lvRetValue = BCC_Init(&gBccDrvConfig, BCC_INIT_CONF);

     // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to initalize BCC: %d\n", lvRetValue);
        return lvRetValue;
    }
    else
    {
    	//cli_printf("BCC init succeeded!\n");
    }

    // get the battery temperature enable variable 
	dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, &nCells, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
    	// set the default value
    	nCells = SENSOR_ENABLE_DEFAULT;

    	cli_printf("BatManagement_initializeBCC ERROR: couldn't get SENSOR_ENABLE! setting default\n");
    }
    // set the config bits
    configBits = 1 << ANX_C_BATT;

    // limit the boolean
    nCells &= 1;

    // enable or disable the temperature sensor 
    lvRetValue = bcc_configuration_disableNEnableANx(&gBccDrvConfig, BCC_CID_DEV1, configBits, 
    	!nCells);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set batt temperature measurement: %d\n", lvRetValue);
        return lvRetValue;
    }

    // check if disabled
    if(!nCells)
    {
    	// output to the user
    	cli_printf("WARNING: battery temperature sensor is disabled!\n");
    	cli_printf("If this needs to be enabled write: \"bms set sensor-enable 1\" in the terminal\n");
    }

    // get the PCB temperatures
    data_getParameter(C_PCB_UT, &lowerTH, NULL);
    data_getParameter(C_PCB_OT, &upperTH, NULL);

    // make the temperature bits (0b1101)
    configBits = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

    // seting PCB temperatuer threshold registers
    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, configBits, 
    	&lowerTH, &upperTH);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set PCB temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the PCB temperatures 
    data_getParameter(V_MIN, &PCBUV, NULL);
    data_getParameter(V_MAX, &PCBOV, NULL);

    // make the temperature bits (0b1101)
    configBits = (1 << ANX_V_OUT);

    //cli_printf("PCBUV: %d\n", (PCBUV & 0xFF));
	//cli_printf("PCBOV: %d\n", (PCBOV & 0xFF));

    // make it millivolt and div by 11
    PCBUV = 0;
    PCBOV = ((PCBOV & 0xFF) * 1000)/(float)VOLTDIV_BATT_OUT;

    //cli_printf("PCBUV: %d\n", PCBUV);
	//cli_printf("PCBOV: %d\n", PCBOV);

    // seting PCB voltage threshold registers
    lvRetValue = bcc_configuration_changeANxVTH(&gBccDrvConfig, BCC_CID_DEV1, configBits, 
    	&PCBUV, &PCBOV);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set PCB temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // set the normal state
    batManagement_setNGetChargingState(true, false);

    // set the battery temperature thresholds
    lvRetValue = batManagement_setBatTempTHState(false);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set BAT temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the cell voltage thresholds
	data_getParameter(V_CELL_UV, &lowerTH, NULL);
    data_getParameter(V_CELL_OV, &upperTH, NULL);

    // set the right cell voltage registervalues
    lvRetValue = bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, 
    	&lowerTH, &upperTH);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set cell TH BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the measurement cycle for in the sleep mode
    data_getParameter(T_CYCLIC, &measCycle, NULL);

    // set the cyclic timer
    lvRetValue = bcc_configuration_changeCyclicTimer(&gBccDrvConfig, BCC_CID_DEV1, measCycle);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the number of cells
    data_getParameter(N_CELLS, &nCells, NULL);

    // set the odd or even
    lvRetValue = bcc_configuration_changeCellCount(&gBccDrvConfig, BCC_CID_DEV1, nCells);
	
	// check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // return to user
	return lvRetValue;
}

/*!
 * @brief 	function to set the battery under and over temperature threshold
 * 
 * @param 	chargingMode true if the temperature thresholds are set for the charging mode.
 * 			false if the temperature thresholds are set for the normal mode
 * @return 	the bcc status
 */
static bcc_status_t batManagement_setBatTempTHState(bool chargingMode)
{
	bcc_status_t lvRetValue = BCC_STATUS_SUCCESS;
	float lowerTH, upperTH;
	uint8_t temperatureBits;

	// check which threshold to set
	if(!chargingMode)
	{
		 // get the battery temperatures thresholds
	    data_getParameter(C_CELL_UT, &lowerTH, NULL);
	    data_getParameter(C_CELL_OT, &upperTH, NULL);
	}
	// if normal mode
	else
	{
	    // get the battery temperatures thresholds
	    data_getParameter(C_CELL_UT_CHARGE, &lowerTH, NULL);
	    data_getParameter(C_CELL_OT_CHARGE, &upperTH, NULL);
	}

    // make the temperature bits
    temperatureBits = 1 << ANX_C_BATT;

    // set the right temperature 
    // seting PCB temperatuer threshold registers
    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, temperatureBits, 
    &lowerTH, &upperTH);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief 	function to set of get the chargingMode variable
 * 
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the chargingMode variable, or -1 if error
 */
static bool batManagement_setNGetChargingState(bool set, bool setValue)
{
	bool lvRetValue = 0;
	bcc_status_t lvBccStatus;
	static bool chargingMode = false;
	static bool oldChargingMode = true;

	// check if mutex is not initialized
	if(!chargingStateMutexInitialized)
	{
		cli_printf("batManagement_setNGetChargingState ERROR: mutex not initialzed!\n");
		while(1);
	}

	// lock mutex
	pthread_mutex_lock(&chargingStateMutex);

	// if the value needs to be set
	if(set)
	{
		// set the variable
		chargingMode = setValue;

		//cli_printf("setting chargestate with %d\n", setValue);
	}

	// save the variable
	lvRetValue = chargingMode;

	// unlock the mutex
	pthread_mutex_unlock(&chargingStateMutex);

	// check if the old variable has changed
	if(set && (lvRetValue != oldChargingMode))
	{
		// set the temp threshold
		lvBccStatus = batManagement_setBatTempTHState(lvRetValue);

		// check for errors
		if(lvBccStatus != BCC_STATUS_SUCCESS)
		{
			// ouput to the user
			cli_printf("batManagement_setNGetChargingState ERROR: couldn't set the right temp TH: %d\n", lvBccStatus);
		}
	}

	// return
	return lvRetValue;

}

/*!
 * @brief 	function to set of get the active balancing cells the battery under and over temperature threshold
 * 			this function can be used by multiple threads
 * 
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the activeBalancingCells, UINT8_MAX if failed
 */
static uint8_t batManagement_setNGetActiveBalancingCells(bool set, uint8_t setValue)
{
	/*! @brief  variable to indicate balancing is on */
	static uint8_t activeBalancingCells = 0;
	uint8_t lvRetValue = UINT8_MAX;

	// check if mutex not initialized
	if(!gBalanceCellsMutexInitialized)
	{
		// return max value
		return lvRetValue;
	}

	// lock the mutex
	pthread_mutex_lock(&gBalanceCellsMutex);

	// set the value if needed 
	if(set)
	{
		// check the range
		if(setValue > 0x3F)
		{
			// unlock the mutex
			pthread_mutex_unlock(&gBalanceCellsMutex);

			// return max value
			return lvRetValue;
		}
		else
		{
			// set the value
			activeBalancingCells = setValue; 
		}
	}

	// save the value
	lvRetValue = activeBalancingCells;

	// unlock the mutex
	pthread_mutex_unlock(&gBalanceCellsMutex);

	// return the value
	return lvRetValue;
	
}

/*!
 * @brief 	function to set of get the dischargingHandshakeVariable variable
 * 
 * @param 	forSelfDischarge if this is true it is for the self discharge handshake, 
 			otherwise it is for the balancing handshake
 * @param 	set true if the value is being set, false if get
 * @param 	setValue if set is true, this is the new value of it
 *
 * @return 	the value of the dischargingHandshakeVariable variable or UINT8_MAX for error
 */
static uint8_t batManagement_setNGetDisCharHandshakeVar(bool forSelfDischarge, bool set, bool setValue)
{
	/*! @brief  variable to indicate balancing is on */
	static uint8_t dischargingHandShakeVariable = 0;
	uint8_t lvRetValue = UINT8_MAX;

	// check if mutex not initialized
	if(!gDisCharHandshakeVarMutexInitialized)
	{
		// return max value
		return lvRetValue;
	}

	// lock the mutex
	pthread_mutex_lock(&gDisCharHandshakeVarMutex);

	// set the value if needed 
	if(set)
	{
		// set the value
		if(setValue)
		{
			// set the right bit 
			dischargingHandShakeVariable |= (1 << forSelfDischarge); 
		}
		else
		{
			// clear the right bit
			dischargingHandShakeVariable &= ~(1 << forSelfDischarge); 
		}
	}

	// save the value
	lvRetValue = dischargingHandShakeVariable;

	// unlock the mutex
	pthread_mutex_unlock(&gDisCharHandshakeVarMutex);

	// return the value
	return lvRetValue;
}

//#endif
