/****************************************************************************
 * nxp_bms/BMS_v1/src/batManagement.c
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

#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "gpio.h"
#include "bcc.h"
#include "bcc_spiwrapper.h"
#include "bcc_configuration.h"
#include "bcc_monitoring.h"
#include "bcc_define.h"

#include "data.h"
#include "cli.h"
#include "spi.h"
/****************************************************************************
 * Defines
 ****************************************************************************/
//#define OUTPUT_OTHER_FAULT
//#define DEBUG_OV_UV
//#define DEBUG_OT_UT
//#define DEBUG_FAULT_STATUS
#define DEBUG_OUTPUT_ERROR_CURRENT

#define RBAL 82 //!< [Ohm] balancing resistor (84 Ohm for the Drone BMS)

#define CALC_OTHER_PRIORITY             140
#define MEASURE_PRIORITY                120
#define SDCHAR_PRIORITY                 110
#define DEFAULT_MEASURE_STACK_SIZE_L    1024 //2048
#define DEFAULT_MEASURE_STACK_SIZE_M    1536 //2048
#define DEFAULT_MEASURE_STACK_SIZE_H    2048 //2048

#define MAX_SEC                         0xFFFFFFFF                  

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
/*! 
 *  @brief  Enumeration to check what triggered a (software) fault.
 *  @note   This is checked with the new measurement from the BCC. 
 */
typedef enum 
{
    BMS_SW_CELL1_OV             = (1<<BMS_FAULT_CELL1_BIT_SHIFT),               /*!< there is a SW OV fault with cell 1 */
    BMS_SW_CELL2_OV             = (1<<BMS_FAULT_CELL2_BIT_SHIFT),               /*!< there is a SW OV fault with cell 2 */
    BMS_SW_CELL3_OV             = (1<<BMS_FAULT_CELL3_BIT_SHIFT),               /*!< there is a SW OV fault with cell 3 */
    BMS_SW_CELL4_OV             = (1<<BMS_FAULT_CELL4_BIT_SHIFT),               /*!< there is a SW OV fault with cell 4 */
    BMS_SW_CELL5_OV             = (1<<BMS_FAULT_CELL5_BIT_SHIFT),               /*!< there is a SW OV fault with cell 5 */
    BMS_SW_CELL6_OV             = (1<<BMS_FAULT_CELL6_BIT_SHIFT),               /*!< there is a SW OV fault with cell 6 */
    BMS_SW_AVG_OVER_CURRENT     = (1<<BMS_FAULT_AVG_OVER_CURRENT_BIT_SHIFT),    /*!< there is a SW average overcurrent */
    BMS_SW_PEAK_OVER_CURRENT    = (1<<BMS_FAULT_PEAK_OVER_CURRENT_BIT_SHIFT),   /*!< there is a SW peak overcurrent */
}BMSSWFault_t;
/****************************************************************************
 * private data
 ****************************************************************************/
// the callback functions
/*! @brief  callback function to report there is an overcurrent */
swMeasuredFaultCallbackFunction         g_swMeasuredFaultCallbackFunctionfp;

/*! @brief  callback function to change the LED color */
changeLedColorCallbackBatFuntion        g_changeLedColorCallbackBatFuntionfp;

/*! @brief  callback function to act on new setted measurements */
newMeasurementsCallbackFunction         g_newMeasurementsCallbackFunctionfp;

/*! @brief  mutex for controlling the gate */
static pthread_mutex_t gGateLock;       

/*! @brief  semaphore for the continue measure task*/
static sem_t gMeasureSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gMeasureSemInitialized = false;

/*! @brief  semaphore for skipping the wait in the continue measure task*/
static sem_t gSkipMeasureWaitSem;
/*! @brief  to indicate the semaphore is initialized*/
static bool gSkipMeasureWaitSemInitialized = false;

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

/*! @brief  variable to slow down the current measurement to t-meas
            can be used to reduce MCU load */
static bool gSlowCurrentMeasurements = false;

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

/*! @brief  variable to keep track of the lowest cell voltage */
float gLowestCellVoltage = V_CELL_OV_DEFAULT;

/*! @brief  Variable to keep track of a sw defined fault using the BMSSWFault_t enum */
uint32_t gSWFaultVariable = 0;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief   function to do the meanual measurements, calculate current
 *          and if the semaphore is availeable read the rest and do the calculations
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
 * @brief   function used to calculate the new measureinterval to read and calculate 
 *          voltages and temperatures
 *          it will set the variable gMeasCycleTime with the right value
 * 
 * @param measMs the send interval in ms
 */
static void batManagement_calcSendInterval(uint16_t measMs);

/*!
 * @brief   function to initialize the BCC
 *          could return without init if communication failed!
 * 
 * @param none
 * @return the bcc status from bcc_status_t
 */
static bcc_status_t BatManagement_initializeBCC(void);

/*!
 * @brief   function to set the battery under and over temperature threshold
 * 
 * @param   chargingMode true if the temperature thresholds are set for the charging mode.
 *          false if the temperature thresholds are set for the normal mode
 * @return  the bcc status
 */
static bcc_status_t batManagement_setBatTempTHState(bool chargingMode);

/*!
 * @brief   function to set of get the chargingMode variable
 * 
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the chargingMode variable
 */
static bool batManagement_setNGetChargingState(bool set, bool setValue); 

/*!
 * @brief   function to set of get the active balancing cells the battery under and over temperature threshold
 *          this function can be used by multiple threads
 * 
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the activeBalancingCells, UINT8_MAX if failed
 */
static uint8_t batManagement_setNGetActiveBalancingCells(bool set, uint8_t setValue);

/*!
 * @brief   function to set of get the dischargingHandshakeVariable variable
 * 
 * @param   forSelfDischarge if this is true it is for the self discharge handshake, 
            otherwise it is for the balancing handshake
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the dischargingHandshakeVariable variable or UINT8_MAX for error
 */
static uint8_t batManagement_setNGetDisCharHandshakeVar(bool forSelfDischarge, bool set, bool setValue);  

/****************************************************************************
 * main
 ****************************************************************************/
/*!
 * @brief   this function initializes the battery management unit
 *
 *          It will configure the BCC, connect the callback functions 
 *          and set the power switches open, disconecting the battery
 *          
 * @param   p_swMeasuredFaultCallbackFunction the address of the function to call when a sw measured fault occured. 
 * @param   p_changeLedColorCallbackBatFuntion the address of the function to call to change the LED color
 * @param   p_newMeasurementsCallbackFunction the address of the function to call when new data is set
 *          should be quick
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 *
 */
int batManagement_initialize(swMeasuredFaultCallbackFunction p_swMeasuredFaultCallbackFunction,
                            changeLedColorCallbackBatFuntion p_changeLedColorCallbackBatFuntion,
                            newMeasurementsCallbackFunction p_newMeasurementsCallbackFunction,
                            bool skipSelfTest)
{
    int lvRetValue = !gBatManInitialized;
    int error, errcode;
    bool boolValue = 1;
    float current = 0.0;
    uint8_t sleepCurrentmA = 0; 

    // check if already configured
    if(!gBatManInitialized)
    {
        // connect the callback functions
        g_swMeasuredFaultCallbackFunctionfp = p_swMeasuredFaultCallbackFunction;
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
            cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", 
                error);
            return error;
        }

        // initialize the semaphore
        error = sem_init(&gMeasureSem, 0, 0);
        
        if(error)
        {
            // output to user
            cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", 
                error);
        }
        else
        {
            sem_setprotocol(&gMeasureSem, SEM_PRIO_NONE);
            // set the variable true
            gMeasureSemInitialized = true;
        }

        // initialize the semaphore
        error = sem_init(&gSkipMeasureWaitSem, 0, 0);
        
        if(error)
        {
            // output to user
            cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", 
                error);
        }
        else
        {
            sem_setprotocol(&gSkipMeasureWaitSem, SEM_PRIO_NONE);
            // set the variable true
            gSkipMeasureWaitSemInitialized = true;
        }

        // initialize the semaphore
        error = sem_init(&gCalcOtherSem, 0, 0);
        
        if(error)
        {
            // output to user
            cli_printfError("batManagement ERROR: failed to initialze other sem! error: %d\n", 
                error);
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
            cli_printfError("batManagement ERROR: failed to initialze charge sem! error: %d\n", 
                error);
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
            cli_printfError("batManagement ERROR: failed to initialze sdchar sem! error: %d\n", 
                error);
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
            cli_printfError("batManagement ERROR: failed to initialze dosdchar sem! error: %d\n",
             error);
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
            cli_printfError("batManagement ERROR: failed to initialze docellbalance sem! error: %d\n", 
                error);
        }
        else
        {
            sem_setprotocol(&gDoCellBalanceSem, SEM_PRIO_NONE);
            // set the variable true
            gDoCellBalanceSemInitialized = true;
        }

        // create the task
        lvRetValue = task_create("meas", MEASURE_PRIORITY, 
            DEFAULT_MEASURE_STACK_SIZE_H, batManagement_MeasTaskFunc, NULL);
        // check for errors
        if(lvRetValue < 0)
        {
            // inform user
            errcode = errno;
            cli_printfError("batManagement ERROR: Failed to start task: %d\n", errcode);
            return lvRetValue;
        }

        // create the task
        lvRetValue = task_create("otherCalc", CALC_OTHER_PRIORITY, 
            DEFAULT_MEASURE_STACK_SIZE_L, batManagement_OtherCalcTaskFunc, NULL);
        // check for errors
        if(lvRetValue < 0)
        {
            // inform user
            errcode = errno;
            cli_printfError("batManagement ERROR: Failed to start task: %d\n", errcode);
            return lvRetValue;
        }

        // create the task
        lvRetValue = task_create("sdchar", SDCHAR_PRIORITY, 
            DEFAULT_MEASURE_STACK_SIZE_M, batManagement_SelfDischargeTaskFunc, NULL);
        // check for errors
        if(lvRetValue < 0)
        {
            // inform user
            errcode = errno;
            cli_printfError("batManagement ERROR: Failed to start task: %d\n", errcode);
            return lvRetValue;
        }

        // initialize SPI mutex
        lvRetValue = BCC_initialze_spi_mutex();
        if(lvRetValue)
        {
            cli_printfError("batManagement ERROR: failed to initialze spi mutex!\n");
            return lvRetValue;
        }

        // set that the battery management is configured
        gBatManInitialized = true;

        // reset the BCC
        // write the reset pin 
        lvRetValue = gpio_writePin(BCC_RESET, 1);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("batManagement ERROR: writing BCC_RESET high went wrong!\n");
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // wait a time longer than needed 
        usleep(1000);

        // pull down the reset pin
        lvRetValue = gpio_writePin(BCC_RESET, 0);

        // check if it went wrong
        if(lvRetValue)
        {
            cli_printfError("batManagement ERROR: writing BCC_RESET low went wrong!\n");
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // check if the reset does not come from the external watchdog 
        if(!(skipSelfTest))
        {
            //turn off the gate
            lvRetValue = batManagement_setGatePower(GATE_OPEN);

            // check if it went wrong
            if(lvRetValue)
            {
                cli_printfError("batManagement ERROR: opening the gate went wrong!\n");
                cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
                return lvRetValue;
            }

            cli_printf("SELF-TEST BCC: START\n");
        }
        else
        {
            // output warning 
            cli_printfWarning("WARNING: not turning off power due to watchdog reset!\n");
        }

        // initalize the BCC
        lvRetValue = BatManagement_initializeBCC();
        
        // check for errors
        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            // inform user
            //errcode = errno;
            cli_printfError("batManagement ERROR: Failed to initialze BCC: %d\n", 
                lvRetValue);

            // Check if the self-test shouldn't be skipped
            if(!skipSelfTest)
            {
                
                cli_printf("SELF-TEST BCC: \e[31mFAIL\e[39m\n");
                return lvRetValue;
            }
        }

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST BCC: \e[32mPASS\e[39m\n");

            // do the Gate check at least once 
            do
            {
                cli_printf("SELF-TEST GATE: START\n");

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
                    cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", 
                        lvRetValue);
                    cli_printf("SELF-TEST GATE: \e[31mFAIL\e[39m\n");
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
                    cli_printfError("batManagement ERROR: Failed get output: %d\n", lvRetValue);
                    cli_printf("SELF-TEST GATE: \e[31mFAIL\e[39m\n");
                    return lvRetValue;
                }

                // check if the output is high
                if(boolValue == 1)
                {
                    cli_printfError("batManagement ERROR: Failed to disable the gate!\n");
                    cli_printf("Make sure there is no charger connected and press the button to try again\n");
                    cli_printf("SELF-TEST GATE: \e[31mFAIL\e[39m\n");

                    //lvRetValue = BCC_STATUS_DIAG_FAIL;

                    // set the LED color to indicate the charger is connected and is wrong
                    g_changeLedColorCallbackBatFuntionfp(RED, BLUE, LED_BLINK_ON);

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

            // change the LED color to RED again
            g_changeLedColorCallbackBatFuntionfp(RED, OFF, LED_BLINK_OFF);

            cli_printf("SELF-TEST GATE: \e[32mPASS\e[39m\n");
            cli_printf("SELF-TEST CURRENT_SENSE: START\n");

            // get the sleepcurrent
            if(data_getParameter(I_SLEEP_OC, &sleepCurrentmA, NULL) == NULL)
            {
                cli_printfError("batManagement ERROR: getting sleepcurrent went wrong!\n");
                sleepCurrentmA = I_SLEEP_OC_DEFAULT;
            } 

            // do a blockingmeasurement
            lvRetValue = bcc_monitoring_doBlockingMeasurement(&gBccDrvConfig);

            // check for errors
            if(lvRetValue != 0)
            {
                // inform user
                cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", 
                    lvRetValue);
                cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
                return lvRetValue;
            }

            // get the ISENSE pin open load value 
            lvRetValue = bcc_monitoring_getIsenseOpenLoad(&gBccDrvConfig, &boolValue);
            
            // check for errors
            if(lvRetValue != 0)
            {
                // inform user
                //errcode = errno;
                cli_printfError("batManagement ERROR: Failed to open load value: %d\n", 
                    lvRetValue);
                cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
                return lvRetValue;
            }

            // check if the ISENSE pins have an open load
            if(boolValue)
            {
                // there is an ISENSE pins open load detected
                cli_printfError("batManagement ERROR: ISENSE open load pin detected!\n");
                cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
                return BCC_STATUS_PARAM_RANGE;
            } 

            // set the current in the struct
            lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, SHUNT_RESISTOR_UOHM);

            // check for errors
            if(lvRetValue != 0)
            {
                // inform user
                cli_printfError("batManagement ERROR: Failed to read current: %d\n", 
                    lvRetValue);
                cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
                return lvRetValue;
            }

            // get the batterycurrent
            if(data_getParameter(I_BATT, &current, NULL) == NULL)
            {
                cli_printfError("batManagement ERROR: getting current went wrong!\n");
                current = I_BATT_MAX;
            }

            // check if the abs current is higher than the sleepcurrent or the overcurrent is hight
            if(((int)(fabs(current)*1000) > sleepCurrentmA) || (gpio_readPin(OVERCURRENT)))
            {
                // why did it happend?
                if(((int)(fabs(current)*1000) > sleepCurrentmA))
                {
                    cli_printfError("BatManagement ERROR: current: |%.3fA| > %.3fA\n", 
                        current, (float)sleepCurrentmA/1000.0);
                }
                else
                {
                    cli_printfError("BatManagement ERROR: overcurrent pin high!\n");
                }

                // output that the test has failed
                cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
                return BCC_STATUS_PARAM_RANGE;
            }

            // mention that the self test has passed
            cli_printf("SELF-TEST CURRENT_SENSE: \e[32mPASS\e[39m\n");
        }
        // if the self test should be skipped
        else
        {
            // make sure to set the current in the struct
            // do a blockingmeasurement
            lvRetValue = bcc_monitoring_doBlockingMeasurement(&gBccDrvConfig);

            // check for errors
            if(lvRetValue != 0)
            {
                // inform user
                cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", 
                    lvRetValue);
            }

            // set the current in the struct
            lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, 
                SHUNT_RESISTOR_UOHM);

            // check for errors
            if(lvRetValue != 0)
            {
                // inform user
                cli_printfError("batManagement ERROR: Failed to read current: %d\n", 
                    lvRetValue);
            }
        }

        // change the return value
        lvRetValue = !gBatManInitialized;
    }

    // return the value
    return lvRetValue;
}

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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setGatePower(bool on)
{
    int lvRetValue = -1;

    // check if initialized
    if(!gBatManInitialized)
    {
        // error
        cli_printfError("batmanagement ERROR: Battery management not initialized, pleaze initialze\n");
        return lvRetValue;
    }

    // set return value for 0, if everything goes right it will stay 0
    lvRetValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gGateLock);

    // Set Data pin to 1 so GATE_IN signal turns 1 (power OFF) when clock rises.
    lvRetValue += gpio_writePin(GATE_CTRL_D, !on);

    // check if writing GATE_CTRL_D went wrong
    if(lvRetValue)
    {
        cli_printfError("batmanagement ERROR: could not write GATE_CTRL_D to %d\n", !on);
    }           

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
 * @brief   This function is used to check the AFE. 
 *          It will check the fault masks of the AFE and set it in the variable
 *          It will check the whole configuration and change it if possible.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up. 
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkAFE(uint32_t *BMSFault, bool resetFaultPin)
{
    int lvRetValue = 0;

    // check for faults
    batManagement_checkFault(BMSFault, resetFaultPin);

    return lvRetValue;
}

/*!
 * @brief   This function is used to check what the fault is. 
 *          It will check the fault masks of the AFE and set it in the variable
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkFault(uint32_t *BMSFault, bool resetFaultPin)
{
    int lvRetValue = -1, i;
    bcc_status_t lvBccStatus;
    bcc_fault_status_t resetBCCFaultValue; 
    uint16_t lvBccFaultStatus[BCC_STAT_CNT]; 
    uint8_t NumCells = 0;
    uint8_t checkBCCCell = 0;
    int32_t *dataReturn = NULL;
    //float lvCurrent, lvMaxCurrent;
    uint16_t maskedFaultReg = 0;

    // reset the fault value
    *BMSFault = 0;

    // check for a software fault (sw cell ov, peak overcurrent, overcurrent)
    if(gSWFaultVariable)
    {
        // set the fault 
        *BMSFault |= gSWFaultVariable;

        // check if there was a cell ov 
        if(gSWFaultVariable & (BMS_SW_CELL1_OV + BMS_SW_CELL2_OV +
            BMS_SW_CELL3_OV + BMS_SW_CELL4_OV + BMS_SW_CELL5_OV + 
            BMS_SW_CELL6_OV))
        {
            // set the sw cell ov bit
            *BMSFault |= BMS_SW_CELL_OV;
        }

        // check if the fault needs to be reset
        if(resetFaultPin)
        {
            // make sure to trigger a fault in order to act on the SW fault 
            g_swMeasuredFaultCallbackFunctionfp(true);
        }
    }

    // get the amount of cells
    dataReturn = (int32_t*)data_getParameter(N_CELLS, &NumCells, NULL);
    if(dataReturn == NULL)
    {
        cli_printfError("batManagement_checkFault ERROR: couldn't get num cells\n");
        return lvRetValue;
    }

    // lock the BCC SPI for this thread
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("batManagement_checkFault ERROR: Couldn't lock spi\n");
    }

    // get the BCC fault
    lvBccStatus = bcc_spiwrapper_BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);

    // unlock the BCC spi
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("batManagement_checkFault ERROR: Couldn't unlock spi\n");
    }

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

            dataReturn = (int32_t*)data_getParameter((parameterKind_t)(V_CELL1 + i), 
                &lvCurrent, NULL);
            if(dataReturn == NULL)
            {
                cli_printfError("batManagement_checkFault ERROR: couldn't get cell voltage\n");
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
                cli_printfError("batManagement_checkFault ERROR: couldn't get cell voltage\n");
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
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
                cli_printfError("batManagement_checkFault ERROR: couldn't get voltage\n");
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
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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

    // check for the CSB wakeup
    if(lvBccFaultStatus[BCC_FS_FAULT1] & BCC_RW_CSB_WUP_FLT_MASK)
    {
        // set the sleep overcurrent fault bit
        *BMSFault |= BMS_CSB_WAKEUP;
    }

    // check the BCC_FS_CB_OPEN fault register
    if (lvBccFaultStatus[BCC_FS_CB_OPEN])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError("BCC fault error: fault in BCC_FS_CB_OPEN reg: %d: \n", lvBccFaultStatus[BCC_FS_CB_OPEN]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_CB_OPEN;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
        cli_printfError("BCC fault error: fault in BCC_FS_CB_SHORT reg: %d: \n", lvBccFaultStatus[BCC_FS_CB_SHORT]);
#endif

        // set the new value
        resetBCCFaultValue = BCC_FS_CB_SHORT;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
        cli_printfError("BCC fault error: fault in BCC_FS_GPIO_STATUS reg: %d: \n", lvBccFaultStatus[BCC_FS_GPIO_STATUS]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_GPIO_STATUS;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
        cli_printfError("BCC fault error: fault in BCC_FS_GPIO_SHORT reg: %d: \n", lvBccFaultStatus[BCC_FS_GPIO_SHORT]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_GPIO_SHORT;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // check the BCC_FS_COMM fault register
    if((lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF)
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError("BCC fault error: fault in BCC_FS_COMM reg: %d: \n", (lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF);
#endif
        // check if there are communication errors
        if(lvBccFaultStatus[BCC_FS_COMM])
        {
            // output the amount of errors
            cli_printfError("%d communication error(s) detected!\n", ((lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF));
        }

        // set the new value
        resetBCCFaultValue = BCC_FS_COMM;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // check if the max is reached
            if(((lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF) >= 255)
            {
                // clear the fault
                // write 0 to the register to reset it because it is w0c: write 0 to clear 
                lvRetValue = bcc_spiwrapper_BCC_Reg_Write(&gBccDrvConfig, BCC_CID_DEV1, 
                    BCC_REG_COM_STATUS_ADDR, 0, NULL);

                // check for errors
                if(lvBccStatus != BCC_STATUS_SUCCESS)
                {
                    // output to the user
                    cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
                }
            }
            else
            {
                cli_printfWarning("not resetting communication errors, max (255) not reached\n", lvBccFaultStatus[BCC_FS_COMM]);
            }
        }
    }

    // make the masked fault register
    maskedFaultReg = lvBccFaultStatus[BCC_FS_FAULT1] & ~(BCC_RW_I2C_ERR_FLT_MASK + BCC_RW_IS_OC_FLT_MASK + BCC_R_AN_OT_FLT_MASK
        + BCC_R_AN_UT_FLT_MASK + BCC_R_CT_OV_FLT_MASK + BCC_R_CT_UV_FLT_MASK + BCC_RW_CSB_WUP_FLT_MASK);

    if(maskedFaultReg)//& 0xFFBF)
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError("BCC fault error: fault in BCC_FS_FAULT1 reg: %d: \n", maskedFaultReg);
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
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    if (lvBccFaultStatus[BCC_FS_FAULT2])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError("BCC fault error: fault in BCC_FS_FAULT2 reg: %d: \n", lvBccFaultStatus[BCC_FS_FAULT2]);
#endif

        // set the new value
        resetBCCFaultValue = BCC_FS_FAULT2;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
        cli_printfError("BCC fault error: fault in BCC_FS_FAULT3 reg: %d: \n", maskedFaultReg);
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
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);
        
            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError("batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // everything went ok
    lvRetValue = 0; 

    return lvRetValue;
}

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
 * @warning In the AFE_NORMAL mode the batManagement_UpdateMeasurements should be on to check for an over current!
 *          
 *
 * @param   mode The desired mode to set the AFE to.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setAFEMode(AFEmode_t mode)
{
    int lvRetValue = 0;
    int i;
    uint16_t retRegVal;
    int8_t measurementState = 0;
    static AFEmode_t previousAFEMode = AFE_NORMAL;

    // turn off the measurements if they were on because of the mode change
    measurementState = batManagement_updateMeasurementsOn(false);

    // check if the measurements were on
    if(measurementState == 0)
    {
        // yield the task to finish measurements
        sched_yield();
    }

    // check what to do
    switch(mode)
    {
        case AFE_NORMAL:

            // enable the SPI transmission
            spi_EnableTransmission(BCC_SPI_BUS, true);

            // reset i
            i = 0;

            // do this at least once
            do
            {
                // set the CS pin low
                // wait 80 us (might be reduced to 20 us if needed)
                // set the CS pin high
                spi_wakeUpBCC(BCC_SPI_BUS);

                // wait the wakeup time
                usleep(400);

                // do the verification
                // check if SPI is initialized
                lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);

                // increase i
                i++;

                // sleep for a short moment
                usleep(1);

            // do this while the verification went wrong with a maximum of 5 times
            }while(lvRetValue && i < 5);

            // check for errors
            if(lvRetValue)
            {
                // ouptut to the user 
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't wakeup BCC! %d\n", lvRetValue);
            }

            // disable the CSB wakeup for the fault pin
            batManagement_setCSbFltEnable(false);

            // set the cyclic measurement on
            lvRetValue |= bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR,
                BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_INTERVAL_NORMAL);

            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't set measurements! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                1, &retRegVal);

            //cli_printf("SYS_CFG1: 0x%x\n", retRegVal);

            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't read SYS_CFG1! %d\n", lvRetValue);
            }

            if((retRegVal & BCC_RW_CYCLIC_TIMER_MASK) != BCC_CYCLIC_TIMER_INTERVAL_NORMAL)
            {
                // output to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't update measurments!\n");
            }

            // turn off slow current measurements (@ t-meas)
            gSlowCurrentMeasurements = false;

            // check if the measurements were on
            if(measurementState == 0)
            {
                // turn on the measurements again
                batManagement_updateMeasurementsOn(true);
            }
            
        break;
        case AFE_SLOW:

            // enable the SPI transmission
            spi_EnableTransmission(BCC_SPI_BUS, true);

            // reset i
            i = 0;

            // do this at least once
            do
            {
                // set the CS pin low
                // wait 80 us (might be reduced to 20 us if needed)
                // set the CS pin high
                spi_wakeUpBCC(BCC_SPI_BUS);

                // wait the wakeup time
                usleep(400);

                // do the verification
                // check if SPI is initialized
                lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);

                // increase i
                i++;

                // sleep for a short moment
                usleep(1);

            // do this while the verification went wrong with a maximum of 5 times
            }while(lvRetValue && i < 5);

            // check for errors
            if(lvRetValue)
            {
                // ouptut to the user 
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't wakeup BCC! %d\n", lvRetValue);
            }

            // disable the CSB wakeup for the fault pin
            batManagement_setCSbFltEnable(false);

            // set the cyclic measurement on
            lvRetValue |= bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR,
                BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_INTERVAL_SLOW);

            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't set measurements! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                1, &retRegVal);

            // check for errors
            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't read SYS_CFG1! %d\n", lvRetValue);
            }

            // Check for errors
            if((retRegVal & BCC_RW_CYCLIC_TIMER_MASK) != BCC_CYCLIC_TIMER_INTERVAL_SLOW)
            {
                // output to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't update measurments!\n");
            }

            // since the AFE is not measuring constantly, you don't need to get the current constantly
            // turn on slow current measurements (@ t-meas)
            gSlowCurrentMeasurements = true;

            // check if the measurements were not on
            if(measurementState == 0)
            {
                // turn on the measurements again
                batManagement_updateMeasurementsOn(true);
            }

        break;
        case AFE_SLEEP_MEAS:

            // check if it needs to be woken
            if(BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1))
            {
                // wake it first
                // reset i
                i = 0;

                // do this at least once
                do
                {
                    // increase i
                    i++;

                    cli_printf("waking up BCC!\n");

                    // set the CS pin low
                    // wait 80 us (might be reduced to 20 us if needed)
                    // set the CS pin high
                    spi_wakeUpBCC(BCC_SPI_BUS);

                    // wait the wakeup time
                    usleep(400);

                    // do the verification
                    // check if SPI is initialized
                    lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);

                    if(lvRetValue)
                    {
                        cli_printfError("can't verify com: %d!\n", i);
                    }

                    // sleep for a short moment
                    usleep(1);

                // do this while the verification went wrong with a maximum of 5 times
                }while(lvRetValue && i < 5);

                // check for errors
                if(lvRetValue)
                {
                    // ouptut to the user 
                    cli_printfError("batManagement_setAFEMode ERROR: Couldn't wakeup BCC! %d\n", lvRetValue);
                }
            }

            // set the cyclic measurement on
            lvRetValue = bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_INTERVAL_SLEEP);

            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't set measurements on! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                1, &retRegVal);

            // check for errors
            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't read SYS_CFG1! %d\n", lvRetValue);
            }

            if((retRegVal & BCC_RW_CYCLIC_TIMER_MASK) != BCC_CYCLIC_TIMER_INTERVAL_SLEEP)
            {
                // output to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't update measurments!\n");
            }

            // enable the CSB wakeup for the fault pin
            batManagement_setCSbFltEnable(true);

            // reset i
            i = 0;

            // loop until the BCC is a sleep with a max of 5
            do
            {
                // increase i
                i++;

                // check if not the first time
                if(i > 1)
                {
                    // sleep for a longer period to give the BCC time to handle things
                    usleep(1000);
                }

                cli_printf("Setting BCC to sleep\n");

                // set the BCC to sleep
                lvRetValue = BCC_Sleep(&gBccDrvConfig);

                // check if succeeded
                if(lvRetValue)
                {
                    cli_printfError("batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! try: %d/5 error: %d\n", 
                        i, lvRetValue);
                }

            // loop while the BCC is not in sleep mode yet with a max of 5 
            }while((lvRetValue) && (i < 5));

            // check if not succeeded
            if(lvRetValue)
            {
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! %d error: %d\n", 
                    i, lvRetValue);

                // error
                lvRetValue = -1;
            }
            else
            {
                // it is OK
                lvRetValue = 0;

                // disable the SPI transmission
                spi_EnableTransmission(BCC_SPI_BUS, false);
            }
            
        break;
        case AFE_SLEEP:

            // check if the previous mode was not the normal mode
            if(previousAFEMode != AFE_NORMAL)
            {
                // reset i
                i = 0;

                // do this at least once
                do
                {

                    // set the CS pin low
                    // wait 80 us (might be reduced to 20 us if needed)
                    // set the CS pin high
                    spi_wakeUpBCC(BCC_SPI_BUS);

                    // wait the wakeup time
                    usleep(400);

                    // do the verification
                    // check if SPI is initialized
                    lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);

                    // increase i
                    i++;

                    // sleep for a short moment
                    usleep(1);

                // do this while the verification went wrong with a maximum of 5 times
                }while(lvRetValue && i < 5);

                // check for errors
                if(lvRetValue)
                {
                    // ouptut to the user 
                    cli_printfError("batManagement_setAFEMode ERROR: Couldn't wakeup BCC! %d\n", lvRetValue);
                }
            }

            
            // set the cyclic measurement off
            lvRetValue = bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                BCC_RW_CYCLIC_TIMER_MASK, BCC_CYCLIC_TIMER_DISABLED);

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 
                1, &retRegVal);

            if(lvRetValue)
            {
                // ouptut to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't read SYS_CFG1! %d\n", lvRetValue);
            }

            if((retRegVal & BCC_RW_CYCLIC_TIMER_MASK) != BCC_CYCLIC_TIMER_DISABLED)
            {
                // output to the user
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't update measurments!\n");
            }

            // enable the CSB wakeup for the fault pin
            batManagement_setCSbFltEnable(true);
            
            // reset i
            i = 0;

            // loop until the BCC is a sleep with a max of 5
            do
            {
                // increase i
                i++;

                // check if not the first time
                if(i > 1)
                {
                    // sleep for a longer period to give the BCC time to handle things
                    usleep(100);
                }

                // set the BCC to sleep
                lvRetValue = BCC_Sleep(&gBccDrvConfig);

                // check if succeeded
                if(lvRetValue)
                {
                    cli_printfError("batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! try: %d/5 error: %d\n", 
                        i, lvRetValue);
                }

            // loop while the BCC is not in sleep mode yet with a max of 5 
            }while((lvRetValue) && (i < 5));

            // check if not succeeded
            if(lvRetValue)
            {
                cli_printfError("batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! error: %d\n", 
                    i, lvRetValue);

                // error
                lvRetValue = -1;
            }
            else
            {
                // disable the SPI transmission
                spi_EnableTransmission(BCC_SPI_BUS, false);

                // it is OK
                lvRetValue = 0;
            }
            
        break;
    }

    // save the previous AFE mode 
    previousAFEMode = mode;

    // return 
    return lvRetValue;
} 

/*!
 * @brief   This function could be used to configure a new configuration
 *          It could also calculate a new value based on the input
 *          like if remaining capacity changes, the new SoC is calculated.
 *          it will make sure the system adjusts this variable.
 *          this function should be called when a configuration variable changes.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   changedParam this is the parameter that changed, from the parameterKind_t enum
 * @param   newValue the new value that was set
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
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
            // check if the max peak current isn't exceeded
            // get current and the max current
            lvFloatValue1 = *(float*)newValue;

            // get the max peak current
            dataReturn = (int32_t*)data_getParameter(I_PEAK_MAX, &lvFloatValue2, NULL);
            if(dataReturn == NULL)
            {
                cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                return lvRetValue;
            }

            // compare to check for an error
            if(fabs(lvFloatValue1) > lvFloatValue2)
            {

#ifdef DEBUG_OUTPUT_ERROR_CURRENT
                cli_printf("error current: %.3fA\n", lvFloatValue1);
#endif
                // Set the bit in the variable to indicate a peak overcurrent
                gSWFaultVariable |= BMS_SW_PEAK_OVER_CURRENT;

                // call the callbackfunction for an overcurrent
                g_swMeasuredFaultCallbackFunctionfp(true);
            }
            // if there is not a peak overcurrent
            else
            {
                // clear the bit of the peak current
                gSWFaultVariable &= ~(BMS_SW_PEAK_OVER_CURRENT);
            }

        break;

        // in case it is the (1s?) average current 
        case I_BATT_AVG:

            lvRetValue = -1;
            // check if the max isn't exceeded
            // get the average current
            lvFloatValue1 = *(float*)newValue;

            // check if the in flight parameter should be true
            // get the in flight status boolean 
            if(data_getParameter(S_IN_FLIGHT, &lvInt32Value1, NULL) == NULL)
            {
               cli_printfError("batManagement_changedParameter ERROR: getting in flight status went wrong!\n");
               lvInt32Value1 = S_IN_FLIGHT_DEFAULT;
            } 

            // limit the value 
            lvInt32Value1 &= UINT8_MAX;

            // get the flight mode enable variable 
            if(data_getParameter(FLIGHT_MODE_ENABLE, &lvInt32Value2, NULL) == NULL)
            {
               cli_printfError("batManagement_changedParameter ERROR: getting flight mode enable went wrong!\n");
               lvInt32Value2 = FLIGHT_MODE_ENABLE_DEFAULT;
            } 

            // limit the value 
            lvInt32Value2 &= UINT8_MAX;

            // check if the flight mode enable parameter is true
            // and if the in flight status is false
            if(lvInt32Value2 && !lvInt32Value1)
            {
                // get the i flight mode and the i out max
                // get the flight mode current
                dataReturn = (int32_t*)data_getParameter(I_FLIGHT_MODE, &lvInt32Value2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }

                // limit to uint8
                lvInt32Value2 &= UINT8_MAX;

                // get the max pack output current
                dataReturn = (int32_t*)data_getParameter(I_OUT_MAX, &lvFloatValue2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }

                // check if the 1s avg current is more than the flight mode current 
                // and if the 1s avg current is less than the i-out-max current
                if((((lvFloatValue1 * -1)) > ((float)lvInt32Value2)) && 
                    ((lvFloatValue1 * -1) < lvFloatValue2))
                {
                    // set in flight status to true
                    lvInt32Value1 = 1;

                    // Set the in flight parameter
                    if(data_setParameter(S_IN_FLIGHT, &lvInt32Value1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                        return lvRetValue;
                    }
                }
            }
            // check if the s-in-flight is true
            else if(lvInt32Value1)
            {
                // get the sleep overcurrent 
                if(data_getParameter(I_SLEEP_OC, &lvInt32Value2, NULL) == NULL)
                {
                   cli_printfError("batManagement_changedParameter ERROR: getting sleep overcurrent went wrong!\n");
                   lvInt32Value2 = I_SLEEP_OC_DEFAULT;
                }

                // check if the battery is being charged
                // if the 1s avg current minus the sleep overcurrent is more than 0
                if((lvFloatValue1 - (float)(lvInt32Value2/1000.0)) > 0)
                {
                    // set in flight status to false
                    lvInt32Value1 = 0;

                    // Set the in flight parameter
                    if(data_setParameter(S_IN_FLIGHT, &lvInt32Value1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                        return lvRetValue;
                    }
                }
            }

            // check if not a charging current 
            if(lvFloatValue1 <= 0.0)
            {
                // get the max pack output current
                dataReturn = (int32_t*)data_getParameter(I_OUT_MAX, &lvFloatValue2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            // if charging current
            else
            {
                // get the max charge current
                dataReturn = (int32_t*)data_getParameter(I_CHARGE_MAX, &lvFloatValue2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }

                // check if needs to check for end of CB charge
                if((batManagement_SetNReadEndOfCBCharge(false, 0) & 1) != 1)
                {
                    // get the end of charge current
                    if(data_getParameter(I_CHARGE_FULL, &lvInt32Value1, NULL) == NULL)
                    {
                       cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
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

                        // trigger the main loop, but no fault
                        g_swMeasuredFaultCallbackFunctionfp(false);
                    }
                }
            }

            // compare to check for an error
            if(fabs(lvFloatValue1) > lvFloatValue2)
            {
                // Check if the bit is not set in the fault variable or 
                // if the system is not in the fault mode
                if((!(gSWFaultVariable & BMS_SW_AVG_OVER_CURRENT)) ||
                    data_getMainState() != FAULT)
                {

#ifdef DEBUG_OUTPUT_ERROR_CURRENT
                    cli_printf("error avg current: %.3fA\n", lvFloatValue1);
#endif
                    // Set the bit to indicate an avg overcurrent
                    gSWFaultVariable |= BMS_SW_AVG_OVER_CURRENT;

                    // call the callbackfunction for an overcurrent
                    g_swMeasuredFaultCallbackFunctionfp(true);
                }
            }
            // if there is no overcurrent
            else
            {
                // Set the bit to indicate an avg overcurrent
                gSWFaultVariable &= ~(BMS_SW_AVG_OVER_CURRENT);
            }

        break;

        // in case it is the 10s average current 
        case I_BATT_10S_AVG:

            // get the in flight status boolean 
            if(data_getParameter(S_IN_FLIGHT, &lvInt32Value1, NULL) == NULL)
            {
               cli_printfError("batManagement_changedParameter ERROR: getting in flight status went wrong!\n");
               lvInt32Value1 = S_IN_FLIGHT_DEFAULT;
            } 

            // limit the value 
            lvInt32Value1 &= UINT8_MAX;

            // check if the in flight status is true
            if(lvInt32Value1)
            {
                // get the 10s average current
                lvFloatValue1 = *(float*)newValue;

                // get the flight mode current
                dataReturn = (int32_t*)data_getParameter(I_FLIGHT_MODE, &lvInt32Value2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }

                // limit to uint8
                lvInt32Value2 &= UINT8_MAX;

                // Get the 1s average current
                dataReturn = (int32_t*)data_getParameter(I_BATT_AVG, &lvFloatValue2, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }

                // check if the 10s avg current is less than the flight mode current
                // And the (1s?) avg current is lower than the flight mode current
                if((((lvFloatValue1 * -1)) < ((float)lvInt32Value2)) && 
                    (((lvFloatValue2 * -1)) < ((float)lvInt32Value2)))
                {
                    // make the in flight status variable false again
                    lvInt32Value1 = 0;

                    // Set the in flight parameter
                    if(data_setParameter(S_IN_FLIGHT, &lvInt32Value1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                        return lvRetValue;
                    }
                }
            }

        break;

        // in case it is the flight mode enable
        case FLIGHT_MODE_ENABLE:

            // get the value
            lvInt32Value1 = (*(uint8_t*)newValue) & UINT8_MAX;

            // check if the flight mode enable is now false
            if(!lvInt32Value1)
            {
                // make the in flight status variable false again
                lvInt32Value2 = 0;

                // Set the in flight parameter
                if(data_setParameter(S_IN_FLIGHT, &lvInt32Value2))
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }

        break;

        // in case it is the remaining capacity or the full charge capacity 
        case A_REM:

            // get the value
            lvFloatValue1 = *(float*)newValue;

            // set the boolvalue true to indicate it is remaining capacity
            lvBoolVal = true;

        case A_FULL:
            // calculate the new state of charge 

            // check which parameter to get
            // get the remaining capacity and full charge capacity 
            if(!lvBoolVal)
            {
                dataReturn = (int32_t*)data_getParameter(A_REM, &lvFloatValue1, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
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
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }

            // check for the limits
            if(lvFloatValue2 < lvFloatValue1)
            {
                // set the remaining capacity to be the full_charge_capacity
                if(data_setParameter(A_REM, &lvFloatValue2))
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
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
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }

            // if it is full charge capacity
            if(!lvBoolVal)
            {
                // get the factory capacity
                dataReturn = (int32_t*)data_getParameter(A_FACTORY, &lvFloatValue1, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                    return lvRetValue;
                }
                
                // calculate the new State of health
                lvInt32Value1 = ((int32_t)((lvFloatValue2 / lvFloatValue1) * 100)) & UINT8_MAX;

                // set the new state of health
                if(data_setParameter(S_HEALTH, &lvInt32Value1))
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
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
                cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", changedParam);
                return lvRetValue;
            }
            
            // calculate the new State of health
            lvInt32Value1 = ((int32_t)((lvFloatValue2 / lvFloatValue1) * 100)) & UINT8_MAX;

            // set the new state of health
            if(data_setParameter(S_HEALTH, &lvInt32Value1))
            {
                cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                return lvRetValue;
            }

        break;

        case T_MEAS:

            // get the value
            lvInt32Value1 = (*(uint16_t*)newValue) & UINT16_MAX;

            // check if is not a whole division of T_MEAS_MAX
            if(T_MEAS_MAX % lvInt32Value1)
            {
                cli_printf("inserted T_meas: %d not a whole division of %d\n", lvInt32Value1, T_MEAS_MAX);
                
                // check if the inserted value is less than 1
                if(lvInt32Value1 < 1)
                {
                    // set the boolvalue to true
                    lvBoolVal = true;
                }

                // calculate lower measurement ratio
                while(T_MEAS_MAX % lvInt32Value1)
                {
                    // check if decrease is needed to find the next whole division
                    if(!lvBoolVal)
                    {
                        // decrease the update ratio
                        lvInt32Value1--;

                        if(lvInt32Value1 == 0)
                        {
                            // set the value
                            lvBoolVal = true;

                            //increase once for equation
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
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            else
            {
                // calculate the new value
                batManagement_calcSendInterval((uint16_t)lvInt32Value1);
            }
        break;

        // if it is one of the cell voltages
        case V_CELL1:
        case V_CELL2:
        case V_CELL3:
        case V_CELL4:
        case V_CELL5:
        case V_CELL6:

            // save the value
            lvFloatValue1 = *(float*)newValue;

            // check if the voltage does not exceed the OV
            // check if end of charge voltage needs to be checked and if charging to storage voltage
            if((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) != 2 &&
                batManagement_SetNReadChargeToStorage(false, 0))
            {
                // get the cell storage voltage
                if(data_getParameter(V_STORAGE, &lvFloatValue2, NULL) == NULL)
                {
                   cli_printfError("BatManagment ERROR: getting storage voltage went wrong!\n");
                   lvFloatValue2 = V_STORAGE_DEFAULT;
                }

                // get the cell voltage margin
                if(data_getParameter(V_CELL_MARGIN, &lvInt32Value1, NULL) == NULL)
                {
                   cli_printfError("BatManagment ERROR: getting storage voltage went wrong!\n");
                   lvInt32Value1 = V_CELL_MARGIN_DEFAULT;
                }

                // limit the value 
                lvInt32Value1 &= UINT8_MAX;

                // add the cell voltage margin to the storage voltage variable 
                // to charge a little bit more than the storage voltage
                lvFloatValue2 = lvFloatValue2 + (float)((float)lvInt32Value1 / 1000.0);
            }
            // if normal or charge to cell ov 
            else
            {
                // get the cell over voltage
                if(data_getParameter(V_CELL_OV, &lvFloatValue2, NULL) == NULL)
                {
                   cli_printfError("BatManagment ERROR: getting cell over voltage went wrong!\n");
                   lvFloatValue2 = V_CELL_OV_DEFAULT;
                }
            }

            // check if the cell voltage is higher than the cell overvoltage
            if(lvFloatValue1 >= lvFloatValue2)
            {
                // Check if charging to storage voltage
                if(batManagement_setNGetChargingState(false, 0) && 
                    ((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) != 2))
                {
                    // output the voltage to the user
                    cli_printf("End of charge voltage! cell%d %.3f >= %.3f\n", 
                        ((changedParam - V_CELL1)+1), lvFloatValue1, lvFloatValue2);

                    // set the end of charge variable
                    batManagement_SetNReadEndOfCBCharge(true, 2);

                    // trigger the main loop, but no fault
                    g_swMeasuredFaultCallbackFunctionfp(false);
                }

                // check if the bit is not already set or it is not in the fault mode
                if((!(gSWFaultVariable & (BMS_SW_CELL1_OV << (changedParam - V_CELL1)))) || 
                    data_getMainState() != FAULT)
                {
                    // Set the bit in the variable to indicate a cell overvoltage
                    gSWFaultVariable |= (BMS_SW_CELL1_OV << (changedParam - V_CELL1));

                    // call the callbackfunction for an overcurrent
                    g_swMeasuredFaultCallbackFunctionfp(true);
                }
            }
            // if there is not an cell overvoltage
            else
            {
                // clear the bit of the cell overvoltage
                gSWFaultVariable &= ~(BMS_SW_CELL1_OV << (changedParam - V_CELL1));
            }
            
        break;

        // in case of a new cell over voltage threshold
        case V_CELL_OV:

            // get the new value
            lvFloatValue1 = *(float*)newValue;

            // set the new upper cell voltage threshold in the BCC 
            bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, 
                NULL, &lvFloatValue1);
        break;

        // in case of a new cell under voltage threshold
        case V_CELL_UV:

            // get the new value
            lvFloatValue1 = *(float*)newValue;

            // set the new upper cell voltage threshold in the BCC
            bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, 
                &lvFloatValue1, NULL);

        break;

        // in case of the new PCB temperature thresholds
        case C_PCB_OT:
            // save the value in a float
            lvFloatValue1 = *(float*)newValue;

            // make the temperature bits (0b1101)
            lvInt32Value1 = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

            // seting PCB temperatuer threshold registers for upper threshold
            lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                (uint8_t)lvInt32Value1, NULL, &lvFloatValue1);

             // check for error
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                    changedParam);

                // return
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
            lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                (uint8_t)lvInt32Value1, &lvFloatValue1, NULL);

            // check for error
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                    changedParam);
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
                lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                    (uint8_t)lvInt32Value1, NULL, &lvFloatValue1); 

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                        changedParam);
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
                lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                    (uint8_t)lvInt32Value1, &lvFloatValue1, NULL); 

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                        changedParam);
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
                lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                    (uint8_t)lvInt32Value1, NULL, &lvFloatValue1); 

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                        changedParam);
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
                lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, 
                    (uint8_t)lvInt32Value1, &lvFloatValue1, NULL); 

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                        changedParam);
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
                cli_printfError("batManagement_changedParameter ERROR: couldn't get parameter in %d\n", 
                    changedParam);
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
                cli_printfError("batManagement_changedParameter ERROR: couldn't set parameter in %d\n", 
                    changedParam);
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
                cli_printfError("batManagement_changedParameter ERROR: failed to set batt temperature measurement: %d\n", 
                    lvRetValue);
                return lvRetValue;
            }
    break;

        // if the user changed the battery type
        case BATTERY_TYPE:
            // get the value and limit
            lvInt32Value1 = *(uint8_t*)newValue & UINT8_MAX;

            // sleep a little bit for the nsh> (for CLI output)
            usleep(1);

            // check what kind of battery it changed to
            switch(lvInt32Value1)
            {
                // if it has changed to a LiPo battery
                case 0:
                {
                    // change the cell overvoltage, cell undervoltage and storage voltage
                    lvFloatValue1 = LIPO_V_CELL_OV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-ov to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_OV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-ov\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIPO_V_CELL_UV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-uv to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_UV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-uv\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIPO_V_STORAGE_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-storage to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_STORAGE, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-storage\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIPO_V_CELL_NOMINAL_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-nominal to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_NOMINAL, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-nominal\n");
                        return lvRetValue;
                    }

                    lvRetValue = 0;
                }
                break;
                // if it has changed to a LiFePo4 battery
                case 1:
                {
                    // change the cell overvoltage, cell undervoltage and storage voltage
                    lvFloatValue1 = LIFEPO4_V_CELL_OV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-ov to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_OV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-ov\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEPO4_V_CELL_UV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-uv to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_UV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-uv\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEPO4_V_STORAGE_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-storage to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_STORAGE, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-storage\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEPO4_V_CELL_NOMINAL_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-nominal to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_NOMINAL, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-nominal\n");
                        return lvRetValue;
                    }

                    lvRetValue = 0;
                }
                break;
                // if it has changed to a LiFeYPo4 battery
                case 2:
                {
                    // change the cell overvoltage, cell undervoltage and storage voltage
                    lvFloatValue1 = LIFEYPO4_V_CELL_OV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-ov to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_OV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-ov\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEYPO4_V_CELL_UV_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-uv to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_UV, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-uv\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEYPO4_V_STORAGE_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-storage to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_STORAGE, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-storage\n");
                        return lvRetValue;
                    }

                    lvFloatValue1 = LIFEYPO4_V_CELL_NOMINAL_DEFAULT;

                    // output to the user
                    cli_printfWarning("WARNING: Setting v-cell-nominal to %.3f\n", lvFloatValue1);

                    // set the overvoltage
                    if(data_setParameter(V_CELL_NOMINAL, &lvFloatValue1))
                    {
                        cli_printfError("batManagement_changedParameter ERROR: couldn't set v-cell-nominal\n");
                        return lvRetValue;
                    }

                    lvRetValue = 0;
                }
                break;
                default:
                {
                    cli_printfError("ERROR: add new battery type correctly!!\n");
                }
                break;
            }

        break;

        // in case of the sleep current
        case I_SLEEP_OC:
            // get the sleep current
            lvInt32Value1 = *(uint8_t*)newValue & UINT8_MAX;

            // set the new sleep current
            if(bcc_configuration_changeSleepITH(&gBccDrvConfig, BCC_CID_DEV1, lvInt32Value1))
            {
                cli_printfError("ERROR: Couldn't set sleep current in register!\n");
            }
            else
            {
                lvRetValue = 0;
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
 * @brief   This function is used to enable the cyclic measurements. 
 *          It will read the T_meas time and start the measurements task.
 *          This task will measue and calculate the voltages, temperatures, current 
 *          and estimate the SoC, SoH and average current.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). 1 if it already had the value, negative for an error. 
 */
int batManagement_updateMeasurementsOn(bool on)
{
    int lvRetValue = 0;
    //int errcode;
    int semValueArr[2] = {0, 0};
    uint16_t measValue;
    static bool measurementsOn = false;

    // check if semaphores are not initialized
    if(!gMeasureSemInitialized || !gCalcOtherSemInitialized || 
        !gSkipMeasureWaitSemInitialized)
    {
        // error
        cli_printfError("batManagement_updateMeasurementsOn ERROR: semaphores not initialzed!\n");
        lvRetValue = -1;
        return lvRetValue;
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

        // do this and loop until the semaphore values are max 1
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

        // loop while this
        }while(semValueArr[0] > 1 || semValueArr[1] > 1);
    }
    // if measurements should be off but are on
    else if (!on && measurementsOn)
    {
        // read the semaphores
        sem_getvalue(&gMeasureSem, &semValueArr[0]);
        sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

        // if the semaphore is at least 1, decrease
        if(semValueArr[0] > 0)
        {
            // decrease the semaphore
            sem_wait(&gMeasureSem);
        }
        if(semValueArr[1] > 0)
        {
            // decrease the semaphore
            sem_wait(&gCalcOtherSem);
        }

        // measurements off
        measurementsOn = false;

        // make sure the semaphore are at least 0 (not less then 0)
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

        // make sure the semaphores are not more than 0
        do 
        {
            // do a task switch (it could be that an other task will increase the semaphore)
            sched_yield();

            // read the semaphores
            sem_getvalue(&gMeasureSem, &semValueArr[0]);
            sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

            // if the semaphore is at least 1, decrease
            if(semValueArr[0] > 0)
            {
                // decrease the semaphore
                sem_wait(&gMeasureSem);
            }
            if(semValueArr[1] > 0)
            {
                // decrease the semaphore
                sem_wait(&gCalcOtherSem);
            }
        }
        while(semValueArr[0] > 0 || semValueArr[1] > 0);

        // check the do measure sem value
        sem_getvalue(&gSkipMeasureWaitSem, &semValueArr[0]);

        // check if not more than 0
        if(semValueArr[0] < 1)
        {
            // post the semaphore so the task will wait on the next sem_wait()
            sem_post(&gSkipMeasureWaitSem);
        }

    }
    // if it already is in this state
    else
    {
        // set it to 1 to indicate already in this state
        lvRetValue = 1;
    }

    // read the semaphores
    // sem_getvalue(&gMeasureSem, &semValueArr[0]);
    // sem_getvalue(&gCalcOtherSem, &semValueArr[1]);

    // cli_printf("semaphore values after: %d, %d\n", semValueArr[0], semValueArr[1]);

    return lvRetValue;
}

/*!
 * @brief   This function is used to get the status of the measurement
 *          
 * @param   on the address of the variable to indicate the measurment status.
 *          If true the measurement task is still running, false otherwise.
 *
 * @return  If successful, the function will return zero (OK), negative for an error. 
 */
int batManagement_getMeasurementsStatus(bool *on)
{
    int lvRetValue = -1;
    int semValue[2] = {0, 0};

    // check if not a NULL pointer
    if(on != NULL)
    {
        // get the measurment semaphore value
        sem_getvalue(&gMeasureSem, &semValue[0]);

        // get the other calc semaphore value
        sem_getvalue(&gCalcOtherSem, &semValue[1]);

        // check if the thread is waiting on the semaphore
        if((semValue[0] == -1) && (semValue[1] == -1))
        {
            // the measurements are disabled
            *on = false;
        }
        else
        {
            // if not, the measurements are enabled
            *on = true;
        }

        // return OK
        lvRetValue = 0;
    }
    else
    {
        cli_printfError("batManagement_getMeasurementsStatus ERROR: NULL pointer!\n");
    }

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is used to enable the cyclic diagnostics. 
 *          It will read the T_ftti time and start the diagnostics task.
 *          The diagnostics will check if everything is ok and if not
 *          it will report to the callback function diagnosticsFail
 *          batManagement_initialize should be called before calling this function
 *          still needs to be implemented!!!!!
 *          
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_doDiagnosticsOn(bool on)
{
    int lvRetValue = 0;
    return lvRetValue;
}

/*!
 * @brief   This function is used to do a measurement
 *          This function will start a conversion and wait (blocking)
 *          until the BCC is done with the measurement.
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
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
        cli_printfError("batManagement_doMeasurement ERROR: failed to do measurement: %d\n", error);
    }

    // save the error
    lvRetValue = (int)error;

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is used to check if n-cells is in line with the measurements
 * @note    A measurements should be done first.
 *
 * @param   nCellsOK the address of the variable to become 1 if the cells are OK.
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkNCells(bool *nCellsOK)
{
    int lvRetValue;

    // check if n cells is ok
    lvRetValue = bcc_monitoring_checkNCells(&gBccDrvConfig, nCellsOK);

    return lvRetValue;
}

/*!
 * @brief   This function is used to get the output voltage and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   none
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_getOutputVoltage(void)
{
    int lvRetValue;

    // get the output voltage
    lvRetValue = bcc_monitoring_getOutputVoltage(&gBccDrvConfig);

    return lvRetValue;
}

/*!
 * @brief   This function is used to get the cell voltages and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   none
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_getCellVoltages(void)
{
    int lvRetValue;

    // get the cell voltages
    lvRetValue = bcc_monitoring_getCellVoltages(&gBccDrvConfig);

    return lvRetValue;
}

/*!
 * @brief   This function is used to get the battery current and set it in the data struct
 * @note    A measurements should be done first.
 *
 * @param   none
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_getBattCurrent(void)
{
    int lvRetValue;

    lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, SHUNT_RESISTOR_UOHM);

    return lvRetValue;
}

/*!
 * @brief   This function is used to start or stop the charging sequence. 
 *          Normally it will stop after completion of the charging sequence, but a fault could occur.
 *          It will start the charging task.
 *          This will implement the charging state machine.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_startCharging(bool on)
{
    int lvRetValue = 0;

    // set the charging state on
    batManagement_setNGetChargingState(true, on);

    return lvRetValue;
}

/*!
 * @brief   This function is used to start or stop the self discharging sequence. 
 *          it will self-discharge each cell to the storage voltage.
 *          Normally it will stop after completion of the charging sequence, but a fault could occur.
 *          It will start the self discharging task.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   on if true it will start the task and if false it will delete the task.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
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
        lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

        // check for errors
        if(lvBccStatus == BCC_STATUS_SUCCESS)
        {
            lvRetValue = 0;
        }
        else
        {
            lvRetValue = -1;
            cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(1)! %d\n",
                lvBccStatus);
        }

        // loop through them
        for(i = 0; i < 6; i++)
        {
            //cli_printf("setting CB %d off\n", i+1);
            // set every individual cell balance driver off
            lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // set the return value
                lvRetValue = -1;

                // output to the user
                cli_printfError("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
            }   
        }   

        // set the value to 0
        if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
        {
            cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 0);
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

    return lvRetValue;
}

/*!
 * @brief   This function is used to start or stop cell balancing. 
 *          it will discharge the higher cells to the lowest cell.
 *          Normally it will stop after completion of the charging sequence, but a fault could occur.
 *          batManagement_initialize should be called before calling this function
 *          
 * @param   on if true it will start balancing, with false it will end.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
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
        lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);
        if(lvBccStatus == BCC_STATUS_SUCCESS)
        {
            lvRetValue = 0;
        }
        else
        {
            lvRetValue = -1;
            cli_printfError("batManagement_setBalancing ERROR: couldn't turn off CB driver! %d\n", lvBccStatus);
            return lvRetValue;
        }       

        // loop through them
        for(i = 0; i < 6; i++)
        {
            //cli_printf("setting CB %d off CB\n", i);
            // set every individual cell balance driver off
            lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // set the return value
                lvRetValue = -1;

                // output to the user
                cli_printfError("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", i+1, lvBccStatus);
            }   
        }   

        // if error
        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            // could not turn off the CB drivers
            return lvRetValue;
        }

        // set the active balance cells variable to 0 
        if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
        {
            cli_printf("batManagement_setBalancing ERROR: can't set the activeBalancingCells with %d!\n", 0);
        }
    }
    // if it should be off
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
 * @brief   This function is used to get the status of the self discharge (sdchar) task
 *          
 * @param   on the address of the variable to indicate the sdchar task status.
 *          If true the measurement task is still running, false otherwise.
 *
 * @return  If successful, the function will return zero (OK), negative for an error. 
 */
int batManagement_getSDChargeStatus(bool *on)
{
    int lvRetValue = -1;
    int semValue = 0;

    // check if not a NULL pointer
    if(on != NULL)
    {
        // get the measurment semaphore value
        sem_getvalue(&gSdChargeSem, &semValue);

        // check if the thread is waiting on the semaphore
        if(semValue == -1)
        {
            // the measurements are disabled
            *on = false;
        }
        else
        {
            // if not, the measurements are enabled
            *on = true;
        }

        // return OK
        lvRetValue = 0;
    }
    else
    {
        cli_printfError("batManagement_getMeasurementsStatus ERROR: NULL pointer!\n");
    }

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is used to check if cell balancing is active. 
 *          
 * @return  0 if cell balancing is not active, a positive number if so
 *          if error a negative number will be returned to indicate the error. 
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
 * @brief   This function is used to check if the output voltage is at least OUTPUT_ON_VOLTAGE
 *          
 * @return  1 if output is active (>= OUTPUT_ON_VOLTAGE)
 *          0 if not active, -1 if something went wrong
 */
int batManagement_checkOutputVoltageActive(void)
{
    float outputVoltage;
    bool lvRetValue = -1;

    // get the output voltage 
    if(data_getParameter(V_OUT, &outputVoltage, NULL) == NULL)
    {
       cli_printfError("batManagement_checkOutputVoltageActive ERROR: getting ouput voltage went wrong!\n");
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
 * @brief   This function is used to get the highest cell voltage
 *          
 * @return  the highest cell voltage
 */
float batManagement_getHighestCellV(void)
{
    float lvRetValue = 0, cellVoltage;
    uint8_t nCells;
    int i;

    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
       cli_printfError("batManagement_getHighestCellV ERROR: getting cell count went wrong!\n");
       nCells = N_CELLS_DEFAULT;
    } 

    // check for which cell it needs to do this
    for(i = 0; i < nCells; i++)
    {
        // get the cell voltage 
        if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
        {
           cli_printfError("batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i+1);
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
 * @brief   This function is used to get the lowest cell voltage
 *          
 * @return  the lowest cell voltage
 */
float batManagement_getLowestCellV(void)
{
    float lvRetValue = V_CELL1_MAX, cellVoltage;
    uint8_t nCells;
    int i;

    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
       cli_printfError("batManagement_getHighestCellV ERROR: getting cell count went wrong!\n");
       nCells = N_CELLS_DEFAULT;
    } 

    // check for which cell it needs to do this
    for(i = 0; i < nCells; i++)
    {
        // get the cell voltage 
        if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
        {
           cli_printfError("batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i+1);
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
 * @brief   This function is used to check if the end of cell balancing charge is there
 * 
 * @param   set true if the newValue needs to be set, false if read
 * @param   newValue if set is true, this is the new value
 *          
 * @return  0 if it is not done, 1 if so with current and 2 if ended with voltage
 *          if error, a negative number will be returned to indicate the error. 
 */
int batManagement_SetNReadEndOfCBCharge(bool set, int newValue)
{
    int lvRetValue = -1;
    static bool endOfChargeVariable = true;

    // check if not initialized 
    if(!gEndOfChargeValueMutexInitialized)
    {
        cli_printfError("batManagement_SetNReadEndOfCBCharge ERROR mutex not initialzed!\n");
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gEndOfChargeValueMutex);

    // set the variable if needed 
    if(set)
    {
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
 * @brief   This function is used to check if the charger should charge to storage voltage
 * 
 * @param   set true if the newValue needs to be set, false if read
 * @param   newValue if set is true, this is the new value
 *          
 * @return  0 if not charging to storage, 1 if so, -1 if error
 */
int batManagement_SetNReadChargeToStorage(bool set, bool newValue)
{
    int lvRetValue = -1;
    static bool chargeToStorageVariable = false;

    // check if not initialized 
    if(!gChargeToStorageVarMutexInitialized)
    {
        cli_printfError("batManagement_SetNReadEndOfCBCharge ERROR mutex not initialzed!\n");
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gChargeToStorageVarMutex);

    // set the variable if needed 
    if(set)
    {
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
 * @brief   This function is to save the remaining capacity to the full charge capacity
 *          this will be used when it is at the end of the charge cycle
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_saveFullChargeCap(void)
{
    int lvRetValue = -1;
    float remainingCap;

    // get the remaining capacity 
    if(data_getParameter(A_REM, &remainingCap, NULL) == NULL)
    {
       cli_printfError("batManagement_saveFullChargeCap ERROR: getting A_REM went wrong!\n");
       return lvRetValue;
    }

    // save the full charge capacity
    if(data_setParameter(A_FULL, &remainingCap))
    {
        cli_printfError("batManagement_saveFullChargeCap ERROR: couldn't set full charge cap\n");
        return lvRetValue;
    }

    // set the return variable
    lvRetValue = 0;

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is to calculate the remaining capacity 
 *          this will be used when an CC overflow occures
 * @note    it will read and reset the CC registers
 * @param   clearingCCOverflow If not NULL this will be set true if the CC register is cleared.
 *          
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_calcRemaningCharge(bool *clearingCCOverflow)
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
       cli_printfError("batManagement_saveFullChargeCap ERROR: getting A_REM went wrong!\n");
       return lvRetValue;
    }

    // calculate dCharge and reset CC registors
    lvRetValue = bcc_monitoring_calcDCharge(&gBccDrvConfig, &samples, &avgCurrent, &deltaCharge, true);

    // check for errors
    if(lvRetValue)
    {
        // output and return
        cli_printfError("batManagement_calcRemaningCharge ERROR: couldn't get delta charge!\n");
        return lvRetValue;
    }

    // calc new remaining charge 
    remainingCap = remainingCap + deltaCharge;

    // set the new remaining charge 
    if(data_setParameter(A_REM, &remainingCap))
    {
       cli_printfError("batManagement_saveFullChargeCap ERROR: setting A_REM went wrong! %.3f\n", remainingCap);
       return lvRetValue;
    }

    // ouput to the user
    //cli_printf("calcRemaningCharge: Avg current: %.3fA samples: %d\n", avgCurrent, avgCurrent);

    // get the BCC fault
    lvBccStatus = bcc_spiwrapper_BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);

    // check for errors
    if(lvBccStatus != BCC_STATUS_SUCCESS)
    {
        // return error 
        cli_printfError("batManagement_saveFullChargeCap ERROR: getting fault went wrong! %.3f\n", remainingCap);
        return lvRetValue;
    }

    // check for a CC overflow
    if(lvBccFaultStatus[BCC_FS_FAULT3] & BCC_R_CC_OVR_FLT_MASK)
    {
        // check for NULL pointer
        if(clearingCCOverflow != NULL)
        {
            // set it true
            *clearingCCOverflow = true;
        }
        //cli_printf("clearing CC overflow\n");
        
        // remove the overflow bits and write the 

        // reset the CC by writing the CC_OVT and CC_P_OVF, CC_N_OVF, SAMP_OVF bits
        lvBccStatus = bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_ADC2_OFFSET_COMP_ADDR, 
            BCC_R_CC_OVT_MASK + BCC_R_SAMP_OVF_MASK + BCC_R_CC_N_OVF_MASK + BCC_R_CC_P_OVF_MASK, 0);

        // check for errors
        if(lvBccStatus != BCC_STATUS_SUCCESS)
        {
            // return error 
            cli_printfError("batManagement_saveFullChargeCap ERROR: clearing CC went wrong! %.3f\n", remainingCap);
            return lvRetValue;
        }

        //read the register
        lvBccStatus |= bcc_spiwrapper_BCC_Reg_Read(&gBccDrvConfig, BCC_CID_DEV1,
         (BCC_REG_ADC2_OFFSET_COMP_ADDR), 1, &retRegVal);

        // get the BCC fault
        lvBccStatus = bcc_spiwrapper_BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);
        if(lvBccFaultStatus[BCC_FS_FAULT3] & BCC_R_CC_OVR_FLT_MASK)
        {
            // clear the fault
            lvBccStatus = bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, BCC_FS_FAULT3);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // return error 
                cli_printfError("batManagement_saveFullChargeCap ERROR: clearing fault went wrong! %.3f\n", remainingCap);
                return lvRetValue;
            }
        }       
    }
    else
    {
        // check for NULL pointer
        if(clearingCCOverflow != NULL)
        {
            // set it to false
            *clearingCCOverflow = false;
        }
    }

    // return 
    lvRetValue = 0;
    return lvRetValue;
}

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
int batManagement_calibrateStateOfCharge(bool calibrateARem)
{
    // do the calibration with the current check on
    return bcc_monitoring_calibrateSoC(calibrateARem, true);
}

/*!
 * @brief   This function is used to output the cell voltages
 *          
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_outputCellVoltages(void)
{
    int lvRetValue = 0;
    uint8_t nCells, i;
    float cellVoltage; 

    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
       cli_printfError("main ERROR: getting n-cells went wrong! \n");
       nCells = N_CELLS_DEFAULT;
       lvRetValue = -1;
    } 

    // limit the value 
    nCells &= UINT8_MAX;

    // lock the printfmutex
    cli_printLock(true);

    // loop throught the amount of cells
    for(i = 0; i < nCells; i++)
    {
        // get the cell voltage
        if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
        {
           cli_printfError("main ERROR: getting cell%d voltage went wrong!\n", i+1);
           cellVoltage = 0.0;
           lvRetValue |= lvRetValue - 2;
        } 

        // output the cell voltage
        cli_printfTryLock("Cell%d voltage: %.3fV\n", i+1, cellVoltage);
    }

    // unlock the printfmutex
    cli_printLock(false);

    // return
    return lvRetValue;
}

/*!
 * @brief   This function is used to enable or disable the CC_OVR_FLT mask, 
 *          to set it the fault pin needs to be set or not with this fault.
 *
 * @param   enable if this fault needs to be enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setCCOvrFltEnable(bool enable)
{
    int lvRetValue;

    // enable or disable it
    lvRetValue = bcc_configuration_setCCOvrFltEnable(&gBccDrvConfig, BCC_CID_DEV1, enable);

    return lvRetValue;
}

/*!
 * @brief   This function is used to update the CSB_WUP_FLT bit in fault mask1 of the BCC,
 *          to set if the fault pin needs to be set or not with this fault.
 *
 * @param   enable if this fault needs to be enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_setCSbFltEnable(bool enable)
{
    int lvRetValue;

    // enable or disable it
    lvRetValue = bcc_configuration_setCSbFltEnable(&gBccDrvConfig, BCC_CID_DEV1, enable);

    return lvRetValue;
}

/*!
 * @brief   This function is used to check if the sleep current threshold mask is enabled, 
 *
 * @param   enabled address of the variable to be true if enabled.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error. 
 */
int batManagement_checkSleepCurrentTh(bool *enabled)
{
    int lvRetValue; 

    // check the register
    lvRetValue = bcc_configuration_checkSleepCurrentTh(&gBccDrvConfig, BCC_CID_DEV1, enabled);

    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief   function to do the meanual measurements, calculate current
 *          and if the semaphore is availeable read the rest and do the calculations
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_MeasTaskFunc(int argc, char *argv[])
{
    int intValue;
    bcc_status_t bcc_status;
    // make the wait time
    struct timespec waitTime;

    // endless loop
    while(1)
    {
        // wait for the semaphore
        sem_wait(&gMeasureSem);

        // check if the current measurement should be done constantly 
        if(!gSlowCurrentMeasurements)
        {
            // get the semaphore value
            sem_getvalue(&gMeasureSem, &intValue);

            // check if the sem is not 1
            if(intValue < 1)
            {
                // post a new semaphore to keep measuring if needed
                sem_post(&gMeasureSem);
            }
        }

        // do the measurements
        bcc_status = bcc_monitoring_updateMeasurements(&gBccDrvConfig, SHUNT_RESISTOR_UOHM, 
            &gLowestCellVoltage, &gGateLock);

        // check for errors
        if(bcc_status != BCC_STATUS_SUCCESS && bcc_status != SEM_OCCUPIED_ERROR)
        {
            cli_printfError("batManagement ERROR: failed to update measurements! error: %d\n", bcc_status);
        }

        // if it did all the measurements
        else if(bcc_status == BCC_STATUS_SUCCESS)
        {

            // callback that data needs to be send
            g_newMeasurementsCallbackFunctionfp();

            //cli_printf("lowest cell voltage: %.3f\n", gLowestCellVoltage);

            // get the semaphore value
            sem_getvalue(&gSdChargeSem, &intValue);

            // check if the sem is not 1
            if(intValue < 1)
            {
                // increase the balance semaphore
                sem_post(&gSdChargeSem);
            }
        }

        // get the current time           
        if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
        {
            cli_printfError("main ERROR: failed to get currentTime!\n");
        }

        // make the 1ms wait time in the current time
        waitTime.tv_sec += (waitTime.tv_nsec + 1000000) / (MAX_NSEC + 1);
        waitTime.tv_nsec = (waitTime.tv_nsec + 1000000) % (MAX_NSEC + 1);

        // TODO make with time difference!

        // wait x ms or until everything needs to be measured 
        sem_timedwait(&gSkipMeasureWaitSem, &waitTime);
    }

    // for compiler
    return -1;
}

/*!
 * @brief function to increase the semaphore at the right time to calculate the other measurements
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_OtherCalcTaskFunc(int argc, char *argv[])
{
    int ret;
    bcc_status_t bcc_status;
    struct timespec  waitTime;
    int32_t intValue;

    // get the T_meas value
    if(data_getParameter(T_MEAS, &intValue, NULL) == NULL)
    {
        cli_printfError("batManagement_OtherCalcTaskFunc ERROR: couldn't get t-meas!\n");
        intValue = T_MEAS_DEFAULT;
    }

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

        // check if the current measurement should be done at this (slow) rate
        // instead of constatly measuring
        if(gSlowCurrentMeasurements)
        {
            // post a new semaphore to start the measurement
            sem_post(&gMeasureSem);
        }

        // lock mutex
        pthread_mutex_lock(&gMeasureTimeMutex);

        //cli_printf("updating measurements!\n");

        // update the measurements
        bcc_status = bcc_monitoring_doAllCalculations();

        // get the sem value
        // get the semaphore value
        sem_getvalue(&gSkipMeasureWaitSem, &intValue);

        // check if the sem is not 1
        if(intValue < 1)
        {
            // post the gSkipMeasureWaitSem as well to skip the wait
            sem_post(&gSkipMeasureWaitSem);
        }

        // check for errors
        if(bcc_status != BCC_STATUS_SUCCESS)
        {
            cli_printfError("batManagement ERROR: failed to update measurements! error: %d\n", bcc_status);
        }

        // make the new gTargetTime time
        gTargetTime.tv_sec += (((gTargetTime.tv_nsec/1000) + gMeasCycleTime*1000) / ((MAX_NSEC/1000) + 1));
        gTargetTime.tv_nsec = ((((gTargetTime.tv_nsec/1000) + gMeasCycleTime*1000) % ((MAX_NSEC/1000) + 1))*1000);

        // get the new time interval
        if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
        {
            cli_printfError("batManagement ERROR: failed to get waitTime time!\n");
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
                cli_printfError("batManagement ERROR: failed to get waitTime time2!\n");
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
            cli_printfError("batManagement ERROR: meas nanosleep went wrong! error: %d time: %ds %dus\n", 
                errno, waitTime.tv_sec, waitTime.tv_nsec/1000);
        }
    }

    // shouldn't come here
    return -1;
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
    bool balancingDone = false;
    uint8_t cellBalanceTimes[6] = {0, 0, 0, 0, 0, 0};
    static bool driverOn = true;
    static bool goToStorageVoltage = false;

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
                cli_printfError("batManagement_SelfDischargeTaskFunc ERROR: \nsem_trywait(&gDoSdChargeSem) error: %d\n", error);
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
                   cli_printfError("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
                   nCells = N_CELLS_DEFAULT;
                } 

                // check for errors
                if(nCells < 3 || 6 < nCells)
                {
                    cli_printfError("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
                    nCells = N_CELLS_DEFAULT;
                }

                // get the storage voltage
                if(data_getParameter(V_STORAGE, &dischargeVoltage, NULL) == NULL)
                {
                   cli_printfError("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
                   dischargeVoltage = V_STORAGE_DEFAULT;
                } 

                // get the cell margin in mv
                if(data_getParameter(V_CELL_MARGIN, &cellMarginMv, NULL) == NULL)
                {
                   cli_printfError("batManagement_selfDischarge ERROR: getting cell margin went wrong!\n");
                   cellMarginMv = V_CELL_MARGIN_DEFAULT;
                } 

                // get the OCV slope
                if(data_getParameter(OCV_SLOPE, &ocvSlope, NULL) == NULL)
                {
                   cli_printfError("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
                   ocvSlope = OCV_SLOPE_DEFAULT;
                } 

                //cli_printf("setting driver OFF\n");

                // turn off the driver
                lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                // check for errors
                if(lvBccStatus == BCC_STATUS_SUCCESS)
                {
                    // clear the variable if ok
                    driverOn = false;
                }
                else
                {
                    cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(2)! %d\n", 
                        lvBccStatus);
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
                    if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
                    {
                       cli_printfError("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i);
                       cellVoltage = 3.5;
                    } 

                    // output equation to the user
                    cli_printf("Discharge when cell%d voltage: %.3f > %.3f\n", i+1, cellVoltage, (dischargeVoltage + ((float)cellMarginMv/1000)));

                    // reset the amout of balance times
                    cellBalanceTimes[i] = 0;

                    // check if the CB driver should be on for this cell
                    if(cellVoltage > (dischargeVoltage + ((float)cellMarginMv/1000)))
                    {
                        // calculate the CB timer
                        balanceMin = ((cellVoltage - dischargeVoltage)*RBAL)/(cellVoltage*ocvSlope/1000);
                        
                        cli_printf("Estimated cell%d balance minutes: %dmin\n", i+1, balanceMin);

                        // check if it is larger than the max
                        while(balanceMin > 511)
                        {
                            // decrease balanceMin with 511 
                            balanceMin -= 511;

                            // increase the amount of times for this cell
                            cellBalanceTimes[i]++;
                        }

                        // write the cell register
                        lvBccStatus =  bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, true, balanceMin);
                        
                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
                        }

                        // increase the balancing variable 
                        balancingCells |= (1<<i);
                    }
                    else
                    {
                        // write the cell register
                        lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, 
                            index, false, 0xFF);

                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", 
                                i, lvRetValue);
                        }

                        // clear the bit in the balancing variable 
                        balancingCells &= ~(1<<i);
                    }
                }

                // check if cells need to be balanced
                if(balancingCells)
                {
                    cli_printf("Setting cell self discharge on for cell: ");
                }

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
                        cli_printfError("batManagement_selfDischarge ERROR: couldn't turn on CB driver!\n");
                    }           

                    // set the global variable 
                    // get them
                    balancingCells |= batManagement_setNGetActiveBalancingCells(false, 0);

                    // set themn
                    if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 
                            balancingCells);
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
                    lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                    // check for errors
                    if(lvBccStatus == BCC_STATUS_SUCCESS)
                    {
                        // clear the variable if ok
                        driverOn = false;
                    }
                    else
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(3)! %d\n",
                            lvBccStatus);
                    }       

                    // loop through them
                    for(i = 0; i < 6; i++)
                    {
                        //cli_printf("setting CB %d off SD2\n", i+1);
                        // set every individual cell balance driver off
                        lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            // set the return value
                            lvRetValue = -1;

                            // output to the user
                            cli_printfError("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", 
                                i+1, lvBccStatus);
                        }   
                    }   

                    // set the global variable 
                    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 
                            0);
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
                cli_printfError("batManagement_SelfDischargeTaskFunc ERROR: \nsem_trywait(&gDoCellBalanceSem) error: %d\n", 
                    error);
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
                   cli_printfError("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
                   nCells = N_CELLS_DEFAULT;
                } 

                // get the cell margin in mv
                if(data_getParameter(V_CELL_MARGIN, &cellMarginMv, NULL) == NULL)
                {
                   cli_printfError("batManagement_selfDischarge ERROR: getting cell margin went wrong!\n");
                   cellMarginMv = V_CELL_MARGIN_DEFAULT;
                } 

                // get the OCV slope
                if(data_getParameter(OCV_SLOPE, &ocvSlope, NULL) == NULL)
                {
                   cli_printfError("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
                   ocvSlope = OCV_SLOPE_DEFAULT;
                } 

                //cli_printf("setting CB driver OFF CBon\n");

                // turn off the driver
                lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                // check for errors
                if(lvBccStatus == BCC_STATUS_SUCCESS)
                {
                    // clear the variable if ok
                    driverOn = false;
                }
                else
                {
                    cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(4)! %d\n",
                        lvBccStatus);
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
                       cli_printfError("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i+1);
                       cellVoltage = V_CELL1_DEFAULT;
                    } 

                    // output equation to the user
                    cli_printf("Balancing will be enabled for cell%d if %.3f > %.3f\n", i+1, cellVoltage, (gLowestCellVoltage+((float)cellMarginMv/1000)));

                    // reset the amout of balance times
                    cellBalanceTimes[i] = 0;

                    // check if the CB driver should be on for this cell
                    if(cellVoltage > (gLowestCellVoltage+((float)cellMarginMv/1000)))
                    {
                        // TODO use this formula? and check it?
                        // calculate the CB timer
                        balanceMin = ((cellVoltage - gLowestCellVoltage)*RBAL)/(cellVoltage*ocvSlope/1000);
                        //balanceMin = 0xFF;

                        cli_printf("Estimated cell%d balance minutes: %dmin\n", i+1, balanceMin);

                        // check if it is larger than the max
                        while(balanceMin > 511)
                        {
                            // decrease balanceMin with 511 
                            balanceMin -= 511;

                            // increase the amount of times for this cell
                            cellBalanceTimes[i]++;
                        }

                        // write the cell register
                        lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, true, balanceMin);
                        
                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
                        }

                        // increase the balancing variable 
                        balancingCells |= (1<<i);
                    }
                    else
                    {

                        // write the cell register
                        lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, false, 0xFF);
                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            cli_printfError("batManagement_selfDischarge ERROR: couldnt turn off cell%d balance: %d\n", i, lvRetValue);
                        }

                        // clear the bit in the balancing variable 
                        balancingCells &= ~(1<<i);
                    }
                }

                // check if cells are balanced
                if(balancingCells)
                {
                    // lock the mutex to print this together
                    cli_printLock(true);

                    // output to the user
                    cli_printfTryLock("Setting cell balance on for cell: ");

                    // check which cells have CB on
                    for (i = 0; i < 6; i++)
                    {
                        // check if on
                        if(balancingCells & (1 << i))
                        {
                            // print the cell number
                            cli_printfTryLock("%d, ", i+1);
                        }
                    }

                    // remove the ","
                    cli_printfTryLock("\b\b \n");

                    // unlock the mutex to print this together
                    cli_printLock(false);
                }
                // if no cells are balanced
                else
                {
                    // output that no cells need to be balanced
                    cli_printf("No cells need to be balanced\n");
                }  

                // check if balancing is on
                if(balancingCells)
                {
                    // let the LED blink blue to indicate balancing
                    g_changeLedColorCallbackBatFuntionfp(BLUE, OFF, LED_BLINK_ON);

                    // turn on the driver
                    if(BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, true) == BCC_STATUS_SUCCESS)
                    {
                        // clear the variable if ok
                        driverOn = true;
                    }
                    else
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: couldn't turn on CB driver!\n");
                    }           

                    // set the global variable 
                    balancingCells |= batManagement_setNGetActiveBalancingCells(false, 0);

                    if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", balancingCells);
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
                    // turn off the driver
                    lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                    // check for errors
                    if(lvBccStatus == BCC_STATUS_SUCCESS)
                    {
                        // clear the variable if ok
                        driverOn = false;
                    }
                    else
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(5)! %d \n",
                            lvBccStatus);
                    }   

                    // loop through them
                    for(i = 0; i < 6; i++)
                    {

                        //cli_printf("setting CB %d OFF CBoff\n", i+1);
                        // set every individual cell balance driver off
                        lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, i, false, 0xFF);

                        // check for errors
                        if(lvBccStatus != BCC_STATUS_SUCCESS)
                        {
                            // set the return value
                            lvRetValue = -1;

                            // output to the user
                            cli_printfError("batManagement_setBalancing ERROR: couldn't turn off driver for %d! %d\n", 
                                i+1, lvBccStatus);
                        }   
                    }   

                    // set the active balance variable 
                    if(batManagement_setNGetActiveBalancingCells(true, 0) == UINT8_MAX)
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells with %d!\n", 0);
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
                   cli_printfError("batManagement_selfDischarge ERROR: getting storage voltage went wrong!\n");
                   dischargeVoltage = V_STORAGE_DEFAULT;
                } 
            }
            else
            {
                // save the lowest cell voltage in dischargeVoltage for balancing during charging
                dischargeVoltage = gLowestCellVoltage;
            }

            // go through each ell
            for(i = 0; i < 6; i++)
            {
                // check if this cell has cell balancing active
                if(batManagement_setNGetActiveBalancingCells(false, 0) & (1<<i))
                {
                    // get the number of cells
                    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
                    {
                       cli_printfError("batManagement_selfDischarge ERROR: getting cell count went wrong!\n");
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

                    // check if the balance time has timed out
                    if(bcc_monitoring_checkBalancingDone(&gBccDrvConfig, index, &balancingDone))
                    {
                        cli_printfError("batManagement_selfDischarge ERROR: could not check if balancing is done\n");
                        cli_printf("Setting balancing to be done for cell%d\n", i+1);
                        balancingDone = true;
                    }

                    // check if balancing time is done
                    if(balancingDone)
                    {
                        // check if the balancing needs to be on for more minutes
                        if(cellBalanceTimes[i])
                        {
                            // turn on cell balancing for 511 minutes for that cell
                            lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, 
                                index, true, 511);
                        
                            // check for errors
                            if(lvBccStatus != BCC_STATUS_SUCCESS)
                            {
                                cli_printfError("batManagement_selfDischarge ERROR: couldnt turn on cell%d balance: %d\n", i, lvRetValue);
                            }

                            // decrease the number of cell balance times
                            cellBalanceTimes[i]--;
                        }
                        else
                        {
                            // output to the user
                            cli_printf("Balancing done for cell%d\n", i+1);

                            // clear the bit in the variable
                            balancingCells = batManagement_setNGetActiveBalancingCells(false, 0) & ~(1<<i);

                            // set the new variable
                            if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
                            {
                                cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells2 with %d!\n", balancingCells);
                            }
                        }                   
                    }
                    // if the balance timer is not done
                    else
                    {
                        // get the cell voltage
                        if(data_getParameter((parameterKind_t)(V_CELL1+i), &cellVoltage, NULL) == NULL)
                        {
                           cli_printfError("batManagement_selfDischarge ERROR: getting cell%d voltage went wrong!\n", i);
                           cellVoltage = V_CELL1_DEFAULT;
                        } 

                        // check if the cell voltage is not higher than the to discharge to voltage
                        if(cellVoltage <= dischargeVoltage)
                        {
                            // output to the user
                            cli_printf("Balancing done for cell%d due to voltage reached\n", i+1);

                            // clear the bit in the variable
                            balancingCells = batManagement_setNGetActiveBalancingCells(false, 0) & ~(1<<i);

                            // set the new variable
                            if(batManagement_setNGetActiveBalancingCells(true, balancingCells) == UINT8_MAX)
                            {
                                cli_printfError("batManagement_selfDischarge ERROR: can't set the activeBalancingCells2 with %d!\n", balancingCells);
                            }

                            // turn off cell balancing for that cell
                            lvBccStatus = bcc_spiwrapper_BCC_CB_SetIndividual(&gBccDrvConfig, BCC_CID_DEV1, index, false, 0xFF);
                            if(lvBccStatus != BCC_STATUS_SUCCESS)
                            {
                                cli_printfError("batManagement_selfDischarge ERROR: could not set cell CB %d\n", lvBccStatus);
                            }
                        }
                    }
                }
            }

            // check if no cell balancing is active, turn on the driver
            if(!batManagement_setNGetActiveBalancingCells(false, 0))
            {
                // turn off the driver
                lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                // check for errors
                if(lvBccStatus == BCC_STATUS_SUCCESS)
                {
                    // clear the variable if ok
                    driverOn = false;
                }
                else
                {
                    cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(6)! %d\n",
                        lvBccStatus);
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
                lvBccStatus = bcc_spiwrapper_BCC_CB_Enable(&gBccDrvConfig, BCC_CID_DEV1, false);

                // check for errors
                if(lvBccStatus == BCC_STATUS_SUCCESS)
                {
                    // clear the variable if ok
                    driverOn = false;
                }
                else
                {
                    cli_printfError("batManagement_selfDischarge ERROR: couldn't turn off CB driver(7)! %d\n",
                        lvBccStatus);
                }               
            }
            
        }
    }

    return lvRetValue;
}

/*!
 * @brief   function used to calculate the new measureinterval to read and calculate 
 *          voltages and temperatures
 *          it will set the variable gMeasCycleTime with the right value
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
            cli_printfError("batManagement ERROR: failed to get gTargetTime time!\n");
        }

        // make the cycletime for ms
        gMeasCycleTime = measMs;

        // set the next measurement target
        gTargetTime.tv_sec--;

        // set the nsec to max to make sure there is no negative number 
        gTargetTime.tv_nsec = MAX_NSEC;

        // unlock mutex
        pthread_mutex_unlock(&gMeasureTimeMutex);
    }
}

/*!
 * @brief   function to initialize the BCC
 *          could return without init if communication failed!
 * 
 * @param none
 * @return the bcc status from bcc_status_t
 */
static bcc_status_t BatManagement_initializeBCC(void)
{
    int lvRetValue = -1, i;
    float lowerTH, upperTH;
    uint16_t PCBOV, PCBUV;
    uint8_t configBits, nCells, measCycle, sleepCurrent;
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
    g_isenseFilterComp.rShunt = 500U;        /* R_SHUNT 0.5mOhm */
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
        lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);
        
        i++;
    }while(lvRetValue != BCC_STATUS_SUCCESS || i == 3);
    
    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to verify com: %d\n", lvRetValue);
        return lvRetValue;
    }

    /* Initialize BCC device */
    lvRetValue = bcc_spiwrapper_BCC_Init(&gBccDrvConfig, BCC_INIT_CONF);

     // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to initalize BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the battery temperature enable variable 
    dataReturn = (int32_t*)data_getParameter(SENSOR_ENABLE, &nCells, NULL);

    // check for error 
    if(dataReturn == NULL)
    {
        // set the default value
        nCells = SENSOR_ENABLE_DEFAULT;

        cli_printfError("BatManagement_initializeBCC ERROR: couldn't get SENSOR_ENABLE! setting default\n");
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
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set batt temperature measurement: %d\n", lvRetValue);
        return lvRetValue;
    }

    // check if disabled
    if(!nCells)
    {
        // output to the user
        cli_printfWarning("WARNING: battery temperature sensor is disabled!\n");
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
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set PCB temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the PCB temperatures 
    data_getParameter(V_MIN, &PCBUV, NULL);
    data_getParameter(V_MAX, &PCBOV, NULL);

    // make the temperature bits (0b1101)
    configBits = (1 << ANX_V_OUT);

    // make it millivolt and div by 11
    PCBUV = 0;//((PCBUV & 0xFF) * 1000)/(float)VOLTDIV_BATT_OUT;
    PCBOV = ((PCBOV & 0xFF) * 1000)/(float)VOLTDIV_BATT_OUT;

    // seting PCB voltage threshold registers
    lvRetValue = bcc_configuration_changeANxVTH(&gBccDrvConfig, BCC_CID_DEV1, configBits, 
        &PCBUV, &PCBOV);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set PCB temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // set the normal state
    batManagement_setNGetChargingState(true, false);

    // set the battery temperature thresholds
    lvRetValue = batManagement_setBatTempTHState(false);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set BAT temp BCC: %d\n", lvRetValue);
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
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set cell TH BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // TODO set AN4 threshold value (bat output threshold) (PCB threshold?)

    // get the measurement cycle for in the sleep mode
    data_getParameter(T_CYCLIC, &measCycle, NULL);

    // set the cyclic timer
    lvRetValue = bcc_configuration_changeCyclicTimer(&gBccDrvConfig, BCC_CID_DEV1, measCycle);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the number of cells
    data_getParameter(N_CELLS, &nCells, NULL);

    // set the odd or even
    lvRetValue = bcc_configuration_changeCellCount(&gBccDrvConfig, BCC_CID_DEV1, nCells);
    
    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the sleep current threshold 
    if(data_getParameter(I_SLEEP_OC, &sleepCurrent, NULL) == NULL)
    {
         cli_printfError("BatManagement_initializeBCC ERROR: failed to get sleep current\n");
         sleepCurrent = I_SLEEP_OC_DEFAULT;
    }

    // set the sleep current threshold for sleep mode
    if(bcc_configuration_changeSleepITH(&gBccDrvConfig, BCC_CID_DEV1, sleepCurrent))
    {
        cli_printfError("BatManagement_initializeBCC ERROR: Couldn't set sleep current in register!\n");
        lvRetValue = -1;
    }

    // return to user
    return lvRetValue;
}

/*!
 * @brief   function to set the battery under and over temperature threshold
 * 
 * @param   chargingMode true if the temperature thresholds are set for the charging mode.
 *          false if the temperature thresholds are set for the normal mode
 * @return  the bcc status
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
 * @brief   function to set of get the chargingMode variable
 * 
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the chargingMode variable, or -1 if error
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
        cli_printfError("batManagement_setNGetChargingState ERROR: mutex not initialzed!\n");
        while(1);
    }

    // lock mutex
    pthread_mutex_lock(&chargingStateMutex);

    // if the value needs to be set
    if(set)
    {
        // set the variable
        chargingMode = setValue;
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
            cli_printfError("batManagement_setNGetChargingState ERROR: couldn't set the right temp TH: %d\n", lvBccStatus);
        }
    }

    // return
    return lvRetValue;
}

/*!
 * @brief   function to set of get the active balancing cells the battery under and over temperature threshold
 *          this function can be used by multiple threads
 * 
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the activeBalancingCells, UINT8_MAX if failed
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
 * @brief   function to set of get the dischargingHandshakeVariable variable
 * 
 * @param   forSelfDischarge if this is true it is for the self discharge handshake, 
            otherwise it is for the balancing handshake
 * @param   set true if the value is being set, false if get
 * @param   setValue if set is true, this is the new value of it
 *
 * @return  the value of the dischargingHandshakeVariable variable or UINT8_MAX for error
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
