/****************************************************************************
 * nxp_bms/BMS_v1/src/batManagement.c
 *
 * BSD 3-Clause License
 *
 * Copyright 2020-2023 NXP
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
#include <time.h>
#include <semaphore.h>
#include <math.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <assert.h>

#include "batManagement.h"

#include "gpio.h"
#include "balancing.h"

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


#define RBAL                 82   //!< [Ohm] balancing resistor (84 Ohm for the Drone BMS)
#define BAT_MANAG_PRIORITY   120  //!< the priority for the bat management task
#define BAT_MANAG_STACK_SIZE 2048 //!< the needed stack size for the bat management task
#define MEASURE_CURRENT_US   3000
#define MAX_SEC              0xFFFFFFFF

/****************************************************************************
 * Types
 ****************************************************************************/
/*!
 *  @brief  Enumeration to check what triggered a (software) fault.
 *  @note   This is checked with the new measurement from the BCC.
 */
typedef enum
{
    BMS_SW_CELL1_OV = (1 << BMS_FAULT_CELL1_BIT_SHIFT), /*!< there is a SW OV fault with cell 1 */
    BMS_SW_CELL2_OV = (1 << BMS_FAULT_CELL2_BIT_SHIFT), /*!< there is a SW OV fault with cell 2 */
    BMS_SW_CELL3_OV = (1 << BMS_FAULT_CELL3_BIT_SHIFT), /*!< there is a SW OV fault with cell 3 */
    BMS_SW_CELL4_OV = (1 << BMS_FAULT_CELL4_BIT_SHIFT), /*!< there is a SW OV fault with cell 4 */
    BMS_SW_CELL5_OV = (1 << BMS_FAULT_CELL5_BIT_SHIFT), /*!< there is a SW OV fault with cell 5 */
    BMS_SW_CELL6_OV = (1 << BMS_FAULT_CELL6_BIT_SHIFT), /*!< there is a SW OV fault with cell 6 */
    BMS_SW_AVG_OVER_CURRENT =
        (1 << BMS_FAULT_AVG_OVER_CURRENT_BIT_SHIFT), /*!< there is a SW average overcurrent */
    BMS_SW_PEAK_OVER_CURRENT =
        (1 << BMS_FAULT_PEAK_OVER_CURRENT_BIT_SHIFT), /*!< there is a SW peak overcurrent */
} BMSSWFault_t;
/****************************************************************************
 * private data
 ****************************************************************************/
// the callback functions
/*! @brief  callback function to report there is an overcurrent */
swMeasuredFaultCallbackFunction g_swMeasuredFaultCallbackFunctionfp;

/*! @brief  callback function to change the LED color */
changeLedColorCallbackBatFuntion g_changeLedColorCallbackBatFuntionfp;

/*! @brief this callback function is used to check for new state transitions on the measured current */
checkForTransitionCurrentCallbackFunction g_checkForTransitionCurrentCallbackFunctionfp;

/*! @brief  callback function to act on new setted measurements */
newMeasurementsCallbackFunction g_newMeasurementsCallbackFunctionfp;

/*! @brief  semaphore for the battery management task*/
static sem_t gBatManagementSem;
/*! @brief  semaphore for skipping the wait in the battery management task*/
static sem_t gSkipBatManagementWaitSem;
/*! @brief  semaphore for the continues charging task*/
static sem_t gChargeSem;

/*! @brief  mutex for controlling the gate */
static pthread_mutex_t gGateLock;
/*! @brief  mutex for the measureTime */
static pthread_mutex_t gMeasureTimeMutex;
/*! @brief  mutex for the balancing value */
static pthread_mutex_t gEndOfChargeValueMutex;
/*! @brief  mutex for the balancing value */
static pthread_mutex_t gChargeToStorageVarMutex;
/*! @brief  mutex for the chargingstate variable */
static pthread_mutex_t chargingStateMutex;

/*! @brief  Variable to set the measurement cycle time */
static uint32_t gMeasCycleTime = 1000;

/*! @brief  Variable to set the target time for the bat manag task to sleep */
static struct timespec gTargetTime;

/*! @brief  variable to slow down the current measurement to t-meas
            can be used to reduce MCU load */
static bool gSlowCurrentMeasurements = false;

/*! @brief variable to keep track of the gate state */
bool gGateState = false;

/*! @brief  [-]         BCC configuration data */
// bcc_data_t      g_bccData;
bcc_drv_config_t gBccDrvConfig;

/*! @brief  [-]         BCC CT filter configuration data */
ct_filter_t g_ctFilterComp;

/*! @brief  [-]         BCC I-sense filter configuration data */
isense_filter_t g_isenseFilterComp;

/*! @brief  [-]         BCC NTC configuration data */
ntc_config_t g_ntcConfig;

/*! @brief  variable to keep track of the lowest cell voltage */
float gLowestCellVoltage = V_CELL_OV_DEFAULT;

/*! @brief  Variable to keep track of a sw defined fault using the BMSSWFault_t enum */
uint32_t gSWFaultVariable = 0;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief   function to do the meanual measurements, calculate current
 *          and if needed it will read the rest and do the calculations
 *          Then it will check the new values if there is anything wrong it
 *          will trigger the main to react on it.
 *          Afterwards it will check balancing and trigger to update measurements.
 *
 * @param   argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param   argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_batManagTaskFunc(int argc, char *argv[]);

/*
 * @brief   This function checks the current for peak over current.
 *          If there is an overcurrent it will set the BMS_SW_PEAK_OVER_CURRENT
 *          bit in gSWFaultVariable and trigger a sw measured fault.
 *
 * @param   current The current as a floating point variable
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int checkCurrentMeasurement(float current);

/*
 * @brief   This function checks the measurements and calculations for faults,
 *          s-in-flight and end of charge. If there is a fault, it will set the bit in
 *          gSWFaultVariable and trigger a sw measured fault. If there is an end of charge
 *          it will set the bit with batManagement_SetNReadEndOfCBCharge().
 *
 * @warning Make sure the average currents are calculated as well in pCommonBatteryVariables!
 *
 * @param   pCommonBatteryVariables The address of the local commonBatteryVariables_t struct (just filled in).
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int checkAllMeasurements(commonBatteryVariables_t *pCommonBatteryVariables);

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

/****************************************************************************
 * main
 ****************************************************************************/
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
 *          indicate the error.
 */
int batManagement_initialize(swMeasuredFaultCallbackFunction p_swMeasuredFaultCallbackFunction,
    changeLedColorCallbackBatFuntion                         p_changeLedColorCallbackBatFuntion,
    checkForTransitionCurrentCallbackFunction                p_checkForTransitionCurrentCallbackFunction,
    newMeasurementsCallbackFunction p_newMeasurementsCallbackFunction, bool skipSelfTest)
{
    int     lvRetValue;
    int     error, errcode;
    bool    boolValue      = 1;
    float   current        = 0.0;
    uint8_t sleepCurrentmA = 0;
    uint8_t doGateCheck    = GATE_CHECK_ENABLE_DEFAULT;

    // connect the callback functions
    g_swMeasuredFaultCallbackFunctionfp           = p_swMeasuredFaultCallbackFunction;
    g_changeLedColorCallbackBatFuntionfp          = p_changeLedColorCallbackBatFuntion;
    g_checkForTransitionCurrentCallbackFunctionfp = p_checkForTransitionCurrentCallbackFunction;
    g_newMeasurementsCallbackFunctionfp           = p_newMeasurementsCallbackFunction;

    // initialze the mutex
    pthread_mutex_init(&gGateLock, NULL);
    pthread_mutex_init(&gMeasureTimeMutex, NULL);
    pthread_mutex_init(&gEndOfChargeValueMutex, NULL);
    pthread_mutex_init(&gChargeToStorageVarMutex, NULL);
    pthread_mutex_init(&chargingStateMutex, NULL);

    // initialize the monitoring part
    error = bcc_monitoring_initialize();
    if(error)
    {
        cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", error);
        return error;
    }

    // initialize the balancing part
    error = balancing_initialize(&gBccDrvConfig);
    if(error)
    {
        cli_printfError("batManagement ERROR: failed to initialze balancing! error: %d\n", error);
        return error;
    }

    // initialize the semaphore
    error = sem_init(&gBatManagementSem, 0, 0);
    if(error)
    {
        // output to user
        cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", error);
    }
    else
    {
        sem_setprotocol(&gBatManagementSem, SEM_PRIO_NONE);
    }

    // initialize the semaphore
    error = sem_init(&gSkipBatManagementWaitSem, 0, 0);
    if(error)
    {
        // output to user
        cli_printfError("batManagement ERROR: failed to initialze sem! error: %d\n", error);
    }
    else
    {
        sem_setprotocol(&gSkipBatManagementWaitSem, SEM_PRIO_NONE);
    }

    // initialize the semaphore
    error = sem_init(&gChargeSem, 0, 0);
    if(error)
    {
        // output to user
        cli_printfError("batManagement ERROR: failed to initialze charge sem! error: %d\n", error);
    }
    else
    {
        sem_setprotocol(&gChargeSem, SEM_PRIO_NONE);
    }

    // create the task
    lvRetValue = task_create(
        "batManag", BAT_MANAG_PRIORITY, BAT_MANAG_STACK_SIZE, batManagement_batManagTaskFunc, NULL);
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

    // reset the BCC
    // write the reset pin
    lvRetValue = gpio_writePin(BCC_RESET, 1);
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
    if(lvRetValue)
    {
        cli_printfError("batManagement ERROR: writing BCC_RESET low went wrong!\n");
        cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
        return lvRetValue;
    }

    // check if the reset does not come from the external watchdog
    if(!(skipSelfTest))
    {
        // turn off the gate
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
        // errcode = errno;
        cli_printfError("batManagement ERROR: Failed to initialze BCC: %d\n", lvRetValue);

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

        // get the variable to do the gate check
        if(data_getParameter(GATE_CHECK_ENABLE, &doGateCheck, NULL) == NULL)
        {
            cli_printfError("batManagement ERROR: getting gate check enable went wrong!\n");
            doGateCheck = GATE_CHECK_ENABLE_DEFAULT;
        }

        // if you need to do the gate check
        if(doGateCheck)
        {
            // do the Gate check at least once
            do
            {
                cli_printf("SELF-TEST GATE: START\n");

                // turn off the gate
                batManagement_setGatePower(GATE_OPEN);

                // check the gate output to verify if it works
                // do a measurement
                lvRetValue = bcc_monitoring_doBlockingMeasurement(&gBccDrvConfig);
                // check for errors
                if(lvRetValue != 0)
                {
                    // inform user
                    // errcode = errno;
                    cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", lvRetValue);
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
                    // errcode = errno;
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

                    // lvRetValue = BCC_STATUS_DIAG_FAIL;

                    // set the LED color to indicate the charger is connected and is wrong
                    g_changeLedColorCallbackBatFuntionfp(RED, BLUE, LED_BLINK_ON);

                    // loop until the button is not pushed
                    while(!gpio_readPin(SBC_WAKE))
                    {
                        // wait for a little while
                        usleep(1000 * 10);
                    }

                    // loop until the button is pushed
                    while(gpio_readPin(SBC_WAKE))
                    {
                        // wait for a little while
                        usleep(1000 * 10);
                    }
                }

                // loop while the output is high
            } while(boolValue == 1);

            // change the LED color to RED again
            g_changeLedColorCallbackBatFuntionfp(RED, OFF, LED_BLINK_OFF);

            cli_printf("SELF-TEST GATE: \e[32mPASS\e[39m\n");
        }
        else // do gate check
        {
            // still turn off the gate
            batManagement_setGatePower(GATE_OPEN);

            cli_printfWarning("SELF-TEST GATE: TURNED OFF!\n");
        }

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
            cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", lvRetValue);
            cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // get the ISENSE pin open load value
        lvRetValue = bcc_monitoring_getIsenseOpenLoad(&gBccDrvConfig, &boolValue);

        // check for errors
        if(lvRetValue != 0)
        {
            // inform user
            // errcode = errno;
            cli_printfError("batManagement ERROR: Failed to open load value: %d\n", lvRetValue);
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
        lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, SHUNT_RESISTOR_UOHM, &current);

        // check for errors
        if(lvRetValue != 0)
        {
            // inform user
            cli_printfError("batManagement ERROR: Failed to read current: %d\n", lvRetValue);
            cli_printf("SELF-TEST CURRENT_SENSE: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }

        // check if the abs current is higher than the sleepcurrent or the overcurrent pin is high
        if(((int)(fabs(current) * 1000) > sleepCurrentmA) || (gpio_readPin(OVERCURRENT)))
        {
            // why did it happend?
            if(((int)(fabs(current) * 1000) > sleepCurrentmA))
            {
                cli_printfError("BatManagement ERROR: current: |%.3fA| > %.3fA\n", current,
                    (float)sleepCurrentmA / 1000.0);
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
            cli_printfError("batManagement ERROR: Failed to do measurement: %d\n", lvRetValue);
        }

        // set the current in the struct
        lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, SHUNT_RESISTOR_UOHM, NULL);

        // check for errors
        if(lvRetValue != 0)
        {
            // inform user
            cli_printfError("batManagement ERROR: Failed to read current: %d\n", lvRetValue);
        }
    }

    // change the return value
    lvRetValue = OK;

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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setGatePower(bool on)
{
    int lvRetValue = -1;

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
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t
 *          enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @param   BMSFault this is the address of the uint32_t variable to store the error (from the BMSFault_t
 *          enum)
 * @param   resetFaultPin this will reset the BCC_FAULT pin if it is up.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_checkFault(uint32_t *BMSFault, bool resetFaultPin)
{
    int                lvRetValue = -1, i;
    bcc_status_t       lvBccStatus;
    bcc_fault_status_t resetBCCFaultValue;
    uint16_t           lvBccFaultStatus[BCC_STAT_CNT];
    uint8_t            NumCells     = 0;
    uint8_t            checkBCCCell = 0;
    uint8_t            dataBMSFault;
    int32_t *          dataReturn         = NULL;
    uint16_t           maskedFaultReg     = 0;
    static uint8_t     previousCommErrors = 0;

    // reset the fault value
    *BMSFault = 0;

    // check for a software fault (sw cell ov, peak overcurrent, overcurrent)
    if(gSWFaultVariable)
    {
        // set the fault
        *BMSFault |= gSWFaultVariable;

        // check if there was a cell ov
        if(gSWFaultVariable &
            (BMS_SW_CELL1_OV + BMS_SW_CELL2_OV + BMS_SW_CELL3_OV + BMS_SW_CELL4_OV + BMS_SW_CELL5_OV +
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
    dataReturn = (int32_t *)data_getParameter(N_CELLS, &NumCells, NULL);
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
            checkBCCCell = (6 - NumCells) + i;
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
            *BMSFault |= (1 << i);
            *BMSFault |= BMS_CELL_OV;

#ifdef DEBUG_OV_UV
            cli_printf("OV on: cell%d\n", i + 1);

            dataReturn = (int32_t *)data_getParameter((parameterKind_t)(V_CELL1 + i), &lvCurrent, NULL);
            if(dataReturn == NULL)
            {
                cli_printfError("batManagement_checkFault ERROR: couldn't get cell voltage\n");
            }
            else
            {
                cli_printf("cell %d voltage: %.3f\n", i + 1, lvCurrent);
            }
#endif
        }

        // check for cell undervoltage
        if((lvBccFaultStatus[BCC_FS_CELL_UV] >> checkBCCCell) & 1)
        {
            // set the cell and undervoltage
            *BMSFault |= (1 << i);
            *BMSFault |= BMS_CELL_UV;
#ifdef DEBUG_OV_UV
            cli_printf("UV on: cell%d\n", i + 1);

            dataReturn = (int32_t *)data_getParameter((parameterKind_t)(V_CELL1 + i), &lvCurrent, NULL);
            if(dataReturn == NULL)
            {
                cli_printfError("batManagement_checkFault ERROR: couldn't get cell voltage\n");
            }
            else
            {
                cli_printf("cell %d voltage: %.3f\n", i + 1, lvCurrent);
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
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // TODO check PCB voltage (AN4?)
    // check for an under and over temperature
    if(lvBccFaultStatus[BCC_FS_AN_OT_UT])
    {
#ifdef DEBUG_OT_UT
        if(lvBccFaultStatus[BCC_FS_AN_OT_UT] & ((1 << 4) + (1 << (4 + 8))))
        {

            // get the batt voltage
            dataReturn = (int32_t *)data_getParameter(V_OUT, &lvCurrent, NULL);
            if(dataReturn == NULL)
            {
                cli_printfError("batManagement_checkFault ERROR: couldn't get voltage\n");
            }
            else
            {
                // get the batt voltage
                dataReturn = (int32_t *)data_getParameter(F_V_OUT_DIVIDER_FACTOR, &VoutVoltageFactor, NULL);
                if(dataReturn == NULL)
                {
                    cli_printfError("batManagement_checkFault: Using default v-out voltage factor (%f)\n",
                        F_V_OUT_DIVIDER_FACTOR_DEFAULT);

                    // set the default, could be slightly off
                    VoutVoltageFactor = F_V_OUT_DIVIDER_FACTOR_DEFAULT;
                }

                // output the actual voltage
                cli_printf("AN4 voltage: %.3f\n", lvCurrent / VoutVoltageFactor);
            }
        }
#endif
        // check for an undertemperature
        if(lvBccFaultStatus[BCC_FS_AN_OT_UT] & 0xF)
        {
            // set the fault
            *BMSFault |= (lvBccFaultStatus[BCC_FS_AN_OT_UT] & 0xF) << BMS_FAULT_TEMPERATURE_BITS_SHIFT;
            *BMSFault |= BMS_UT;
        }

        // check for an overtemperature
        if((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8) & 0xF)
        {
            // set the fault
            *BMSFault |= ((lvBccFaultStatus[BCC_FS_AN_OT_UT] >> 8) & 0xF) << BMS_FAULT_TEMPERATURE_BITS_SHIFT;
            *BMSFault |= BMS_OT;
        }

        // set the new value
        resetBCCFaultValue = BCC_FS_AN_OT_UT;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
    if(lvBccFaultStatus[BCC_FS_CB_OPEN])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError(
            "BCC fault error: fault in BCC_FS_CB_OPEN reg: %d: \n", lvBccFaultStatus[BCC_FS_CB_OPEN]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_CB_OPEN;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // check the BCC_FS_CB_SHORT fault register
    if(lvBccFaultStatus[BCC_FS_CB_SHORT])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError(
            "BCC fault error: fault in BCC_FS_CB_SHORT reg: %d: \n", lvBccFaultStatus[BCC_FS_CB_SHORT]);
#endif

        // set the new value
        resetBCCFaultValue = BCC_FS_CB_SHORT;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // check the BCC_FS_GPIO_STATUS fault register
    if(lvBccFaultStatus[BCC_FS_GPIO_STATUS])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError(
            "BCC fault error: fault in BCC_FS_GPIO_STATUS reg: %d: \n", lvBccFaultStatus[BCC_FS_GPIO_STATUS]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_GPIO_STATUS;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // check the BCC_FS_GPIO_SHORT fault register
    if(lvBccFaultStatus[BCC_FS_GPIO_SHORT])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError(
            "BCC fault error: fault in BCC_FS_GPIO_SHORT reg: %d: \n", lvBccFaultStatus[BCC_FS_GPIO_SHORT]);
#endif
        // set the new value
        resetBCCFaultValue = BCC_FS_GPIO_SHORT;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
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
        cli_printfError(
            "BCC fault error: fault in BCC_FS_COMM reg: %d: \n", (lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF);
#endif
        // check if there are communication errors
        if(lvBccFaultStatus[BCC_FS_COMM])
        {
            // if we already stated this amount of comm errors
            if(previousCommErrors != ((lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF))
            {
                // set the new amount of errors
                previousCommErrors = ((lvBccFaultStatus[BCC_FS_COMM] >> 8) & 0xFF);

                // output the amount of errors
                cli_printfError("%d communication error(s) detected!\n", previousCommErrors);
            }
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
                lvRetValue = bcc_spiwrapper_BCC_Reg_Write(
                    &gBccDrvConfig, BCC_CID_DEV1, BCC_REG_COM_STATUS_ADDR, 0, NULL);

                // check for errors
                if(lvBccStatus != BCC_STATUS_SUCCESS)
                {
                    // output to the user
                    cli_printfError(
                        "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
                }
            }
            else
            {
                cli_printfWarning("not resetting communication errors, max (255) not reached\n",
                    lvBccFaultStatus[BCC_FS_COMM]);
            }
        }
    }

    // make the masked fault register
    maskedFaultReg = lvBccFaultStatus[BCC_FS_FAULT1] &
        ~(BCC_RW_I2C_ERR_FLT_MASK + BCC_RW_IS_OC_FLT_MASK + BCC_R_AN_OT_FLT_MASK + BCC_R_AN_UT_FLT_MASK +
            BCC_R_CT_OV_FLT_MASK + BCC_R_CT_UV_FLT_MASK + BCC_RW_CSB_WUP_FLT_MASK);

    if(maskedFaultReg) //& 0xFFBF)
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
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    if(lvBccFaultStatus[BCC_FS_FAULT2])
    {
        // set the fault
        *BMSFault |= BMS_OTHER_FAULT;

#ifdef OUTPUT_OTHER_FAULT
        // output to the user
        cli_printfError(
            "BCC fault error: fault in BCC_FS_FAULT2 reg: %d: \n", lvBccFaultStatus[BCC_FS_FAULT2]);
#endif

        // set the new value
        resetBCCFaultValue = BCC_FS_FAULT2;

        // check if the pin needs to be resetted
        if(resetFaultPin)
        {
            // clear the fault
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // make the masked fault register
    maskedFaultReg = lvBccFaultStatus[BCC_FS_FAULT3] & ~(BCC_R_CC_OVR_FLT_MASK);

    // check for the other faults
    if(maskedFaultReg)
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
            lvBccStatus =
                bcc_spiwrapper_BCC_Fault_ClearStatus(&gBccDrvConfig, BCC_CID_DEV1, resetBCCFaultValue);

            // check for errors
            if(lvBccStatus != BCC_STATUS_SUCCESS)
            {
                // output to the user
                cli_printfError(
                    "batManagement_checkFault ERROR: couldn't reset fault: %d\n", resetBCCFaultValue);
            }
        }
    }

    // Fill in the fault variable for the data part
    dataBMSFault = ((((*BMSFault) & BMS_CELL_OV) >> (BMS_FAULT_CELL_OV_BIT_SHIFT - 0)) |
        (((*BMSFault) & BMS_CELL_UV) >> (BMS_FAULT_CELL_UV_BIT_SHIFT - 1)) |
        (((*BMSFault) & BMS_OT) >> (BMS_FAULT_OT_BIT_SHIFT - 2)) |
        (((*BMSFault) & BMS_UT) >> (BMS_FAULT_UT_BIT_SHIFT - 3)) |
        (((*BMSFault) & BMS_AVG_OVER_CURRENT) >> (BMS_FAULT_AVG_OVER_CURRENT_BIT_SHIFT - 4)) |
        (((*BMSFault) & BMS_PEAK_OVER_CURRENT) >> (BMS_FAULT_PEAK_OVER_CURRENT_BIT_SHIFT - 4)));

    // set the fault variable in data
    data_setBmsFault(dataBMSFault);

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
 * @warning In the AFE_NORMAL mode the batManagement_UpdateMeasurements should be on to check for an over
 *          current!
 *
 * @param   mode The desired mode to set the AFE to.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_setAFEMode(AFEmode_t mode)
{
    int              lvRetValue = 0;
    int              i;
    uint16_t         retRegVal;
    int8_t           measurementState = 0;
    static AFEmode_t previousAFEMode  = AFE_NORMAL;

    // turn off the measurements if they were on because of the mode change
    measurementState = batManagement_enableBatManagementTask(false);

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
            } while(lvRetValue && i < 5);

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
                cli_printfError(
                    "batManagement_setAFEMode ERROR: Couldn't set measurements! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(
                &gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, &retRegVal);

            // cli_printf("SYS_CFG1: 0x%x\n", retRegVal);

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
                batManagement_enableBatManagementTask(true);
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
            } while(lvRetValue && i < 5);

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
                cli_printfError(
                    "batManagement_setAFEMode ERROR: Couldn't set measurements! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(
                &gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, &retRegVal);

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
                batManagement_enableBatManagementTask(true);
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
                } while(lvRetValue && i < 5);

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
                cli_printfError(
                    "batManagement_setAFEMode ERROR: Couldn't set measurements on! %d\n", lvRetValue);
            }

            // check if succeeded
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(
                &gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, &retRegVal);

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
                    cli_printfError(
                        "batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! try: %d/5 error: %d\n", i,
                        lvRetValue);
                }

                // loop while the BCC is not in sleep mode yet with a max of 5
            } while((lvRetValue) && (i < 5));

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
                } while(lvRetValue && i < 5);

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
            lvRetValue = bcc_spiwrapper_BCC_Reg_Read(
                &gBccDrvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, &retRegVal);

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
                    cli_printfError(
                        "batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! try: %d/5 error: %d\n", i,
                        lvRetValue);
                }

                // loop while the BCC is not in sleep mode yet with a max of 5
            } while((lvRetValue) && (i < 5));

            // check if not succeeded
            if(lvRetValue)
            {
                cli_printfError(
                    "batManagement_setAFEMode ERROR: Couldn't set bcc to sleep! error: %d\n", i, lvRetValue);

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
 * @warning In this, or underlying functions the data_setParameter() and data_getParameter() may not be used.
 *
 * @param   changedParam this is the parameter that changed, from the parameterKind_t enum
 * @param   newValue the new value that was set
 * @param   extraValue Address of the extra value that can be used, may be NULL if not used.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_changedParameter(parameterKind_t changedParam, void *newValue, void *extraValue)
{
    int             lvRetValue = -1;
    variableTypes_u variable1;

    // check what needs to be done
    switch(changedParam)
    {
        // if a new parameter is added, add it to handleParamaterChange() in main.c as well
        case T_MEAS:

            // check if is a whole division of T_MEAS_MAX
            if((T_MEAS_MAX % ((*(uint16_t *)extraValue) & UINT16_MAX)) == 0)
            {
                // calculate the new value
                batManagement_calcSendInterval((uint16_t)((*(uint16_t *)extraValue) & UINT16_MAX));

                // return OK
                lvRetValue = 0;
            }
            break;

        // in case of a new cell over voltage threshold
        case V_CELL_OV:

            // set the new upper cell voltage threshold in the BCC
            lvRetValue =
                (int)bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, NULL, (float *)newValue);
            break;

        // in case of a new cell under voltage threshold
        case V_CELL_UV:

            // set the new upper cell voltage threshold in the BCC
            lvRetValue =
                (int)bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, (float *)newValue, NULL);

            break;

        // in case of the new PCB temperature thresholds
        case C_PCB_OT:

            // make the temperature bits (0b1101)
            variable1.uint8Var = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

            // seting PCB temperatuer threshold registers for upper threshold
            lvRetValue = (int)bcc_configuration_changeTempTH(
                &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, NULL, (float *)newValue);

            // check for error
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                cli_printfError(
                    "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);

                // return
                return lvRetValue;
            }
            break;

        // in case of the new PCB temperature thresholds
        case C_PCB_UT:

            // make the temperature bits (0b1101)
            variable1.uint8Var = (1 << ANX_C_R) + (1 << ANX_C_AFE) + (1 << ANX_C_T);

            // seting PCB temperatuer threshold registers for upper threshold
            lvRetValue = (int)bcc_configuration_changeTempTH(
                &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, (float *)newValue, NULL);

            // check for error
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                cli_printfError(
                    "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                return lvRetValue;
            }
            break;

        // in case of the new cell temperature thresholds
        case C_CELL_OT:

            // check if not charging mode
            if(!batManagement_setNGetChargingState(false, 0))
            {
                // set the upper threshold
                // make the temperature bits
                variable1.uint8Var = 1 << ANX_C_BATT;

                // set the right temperature
                // seting PCB temperatuer threshold registers
                lvRetValue = (int)bcc_configuration_changeTempTH(
                    &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, NULL, (float *)newValue);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError(
                        "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            else
            {
                // return OK
                lvRetValue = 0;
            }

            break;

        // in case of the new cell temperature thresholds
        case C_CELL_UT:

            // check if not charging mode
            if(!batManagement_setNGetChargingState(false, 0))
            {
                // set the lower threshold
                // make the temperature bits
                variable1.uint8Var = 1 << ANX_C_BATT;

                // set the right temperature
                // seting PCB temperatuer threshold registers
                lvRetValue = (int)bcc_configuration_changeTempTH(
                    &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, (float *)newValue, NULL);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError(
                        "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            else
            {
                // return OK
                lvRetValue = 0;
            }

            break;

        // in case of the new cell temperature thresholds
        case C_CELL_OT_CHARGE:

            // check if charging mode
            if(batManagement_setNGetChargingState(false, 0))
            {
                // set the upper threshold
                // make the temperature bits
                variable1.uint8Var = 1 << ANX_C_BATT;

                // set the right temperature
                // seting PCB temperatuer threshold registers
                lvRetValue = (int)bcc_configuration_changeTempTH(
                    &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, NULL, (float *)newValue);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError(
                        "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            else
            {
                // return OK
                lvRetValue = 0;
            }

            break;

        // in case of the new cell temperature thresholds
        case C_CELL_UT_CHARGE:

            // check if charging mode
            if(batManagement_setNGetChargingState(false, 0))
            {
                // set the lower threshold
                // make the temperature bits
                variable1.uint8Var = 1 << ANX_C_BATT;

                // set the right temperature
                // seting PCB temperatuer threshold registers
                lvRetValue = (int)bcc_configuration_changeTempTH(
                    &gBccDrvConfig, BCC_CID_DEV1, variable1.uint8Var, (float *)newValue, NULL);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    cli_printfError(
                        "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                    return lvRetValue;
                }
            }
            else
            {
                // return OK
                lvRetValue = 0;
            }

            break;

        // if the number of cells change
        case N_CELLS:
            // call the function to change the cell count in the BCC
            lvRetValue =
                (int)bcc_configuration_changeCellCount(&gBccDrvConfig, BCC_CID_DEV1, *(uint8_t *)newValue);

            // check for errors
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                // output error
                cli_printfError(
                    "batManagement_changedParameter ERROR: couldn't set parameter in %d\n", changedParam);
                return lvRetValue;
            }

            break;

        // if the user want to disable or enable the temperature measurement
        case SENSOR_ENABLE:
            // enable or disable the battery temperature sensor analog input
            lvRetValue = (int)bcc_configuration_disableNEnableANx(
                &gBccDrvConfig, BCC_CID_DEV1, (1 << ANX_C_BATT), (!(bool)((*(uint8_t *)newValue) & 1)));

            // check for error
            if(lvRetValue != BCC_STATUS_SUCCESS)
            {
                cli_printfError(
                    "batManagement_changedParameter ERROR: failed to set batt temperature measurement: %d\n",
                    lvRetValue);
                return lvRetValue;
            }
            break;

        // in case of the sleep current
        case I_SLEEP_OC:
            // set the new sleep current
            if(bcc_configuration_changeSleepITH(
                   &gBccDrvConfig, BCC_CID_DEV1, (*(uint8_t *)newValue) & UINT8_MAX))
            {
                cli_printfError("ERROR: Couldn't set sleep current in register!\n");
            }
            else
            {
                lvRetValue = 0;
            }

            break;

        default:
            // If a new parameter is added, add it to  handleParamaterChange() function in the main as well
            lvRetValue = 0;
            break;
    }

    // return to the user
    return lvRetValue;
}

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
int batManagement_enableBatManagementTask(bool enable)
{
    int         retValue = 0;
    int         semValue = 0;
    uint16_t    measTime;
    static bool batteryManagementOn = false;

    // check if the task needs to be started
    if(enable && !batteryManagementOn)
    {
        // get the T_meas value
        data_getParameter(T_MEAS, &measTime, NULL);

        // calculate the new time
        batManagement_calcSendInterval(measTime);

        // post the semaphores
        sem_post(&gBatManagementSem);

        // battery management on
        batteryManagementOn = true;

        // do this and loop until the semaphore value is max 1
        do
        {
            // read the semaphore
            sem_getvalue(&gBatManagementSem, &semValue);

            // cli_printf("semaphore value before: %d\n", semValue);

            // check if there are more than one
            if(semValue > 1)
            {
                // get the semaphore
                sem_wait(&gBatManagementSem);
            }

            // loop while this is more than one
        } while(semValue > 1);
    }
    // if battery management should be off but are on
    else if(!enable && batteryManagementOn)
    {
        // read the semaphore
        sem_getvalue(&gBatManagementSem, &semValue);

        // if the semaphore is at least 1, decrease
        if(semValue > 0)
        {
            // decrease the semaphore
            sem_wait(&gBatManagementSem);
        }

        // battery management off
        batteryManagementOn = false;

        // make sure the semaphore is at least 0 (not less then 0)
        do
        {
            // read the semaphore
            sem_getvalue(&gBatManagementSem, &semValue);

            // cli_printf("semaphore value before: %d\n", semValue);

            if(semValue < 0)
            {
                // post the semaphore
                sem_post(&gBatManagementSem);
            }
        } while(semValue < 0);

        // make sure the semaphore is not more than 0
        do
        {
            // do a task switch (it could be that an other task will increase the semaphore)
            sched_yield();

            // read the semaphore
            sem_getvalue(&gBatManagementSem, &semValue);

            // if the semaphore is at least 1, decrease
            if(semValue > 0)
            {
                // decrease the semaphore
                sem_wait(&gBatManagementSem);
            }
        } while(semValue > 0);

        // check the sem value
        sem_getvalue(&gSkipBatManagementWaitSem, &semValue);

        // check if not more than 0
        if(semValue < 1)
        {
            // post the semaphore so the task will wait on the next sem_wait()
            sem_post(&gSkipBatManagementWaitSem);
        }
    }
    // if it already is in this state
    else
    {
        // set it to 1 to indicate already in this state
        retValue = 1;
    }

    // read the semaphore
    // sem_getvalue(&gBatManagementSem, &semValue);

    // cli_printf("semaphore value after: %d\n", semValue);

    return retValue;
}

/*!
 * @brief   This function is used to get the status of the battery management task
 *
 * @param   on the address of the variable to indicate the battery management status.
 *          If true the measurement task is still running, false otherwise.
 *
 * @return  If successful, the function will return zero (OK), negative for an error.
 */
int batManagement_getBatManagementStatus(bool *on)
{
    int lvRetValue = -1;
    int semValue   = 0;

    // check if not a NULL pointer in debug mode
    DEBUGASSERT(on != NULL);

    // get the battery management semaphore value
    sem_getvalue(&gBatManagementSem, &semValue);

    // check if the thread is waiting on the semaphore
    if(semValue == -1)
    {
        // the battery management is disabled
        *on = false;
    }
    else
    {
        // if not, the battery management is enabled
        *on = true;
    }

    // return OK
    lvRetValue = 0;

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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_doMeasurement(void)
{
    int          lvRetValue = -1;
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @param   checkCurrent if true, it will check the current as well for an overcurrent fault.
 *          It will set the overcurrent bit and trigger the main to act on it.
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_getBattCurrent(bool checkCurrent)
{
    int   lvRetValue;
    float currentA;

    // read the battery current from the BCC and set it in the struct
    lvRetValue = bcc_monitoring_getBattCurrent(&gBccDrvConfig, SHUNT_RESISTOR_UOHM, &currentA);

    // check if the current check should be done
    if(checkCurrent)
    {
        // check the current for a peak over current fault
        if(checkCurrentMeasurement(checkCurrent))
        {
            cli_printfError("batManagement_getBattCurrent ERROR: failed to check current!\n");
        }
    }

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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_startCharging(bool on)
{
    int lvRetValue = 0;

    // set the charging state on
    batManagement_setNGetChargingState(true, on);

    return lvRetValue;
}

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
int batManagement_setBalanceState(balanceState_t newBalanceState)
{
    // call the function from balancing.h
    return balancing_setBalanceState(newBalanceState);
}

/*!
 * @brief   this function will check the balance state.
 *
 * @param   none
 *
 * @return  The current balancing state from the balanceState_t enum.
 *          If an error occurs, it will return BALANCE_ERROR
 */
balanceState_t batManagement_getBalanceState(void)
{
    // call the function from balancing.h
    return balancing_getBalanceState();
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
    bool  lvRetValue = -1;

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
    float   lvRetValue = 0, cellVoltage;
    uint8_t nCells;
    int     i;

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
        if(data_getParameter((parameterKind_t)(V_CELL1 + i), &cellVoltage, NULL) == NULL)
        {
            cli_printfError(
                "batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i + 1);
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
    float   lvRetValue = V_CELL1_MAX, cellVoltage;
    uint8_t nCells;
    int     i;

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
        if(data_getParameter((parameterKind_t)(V_CELL1 + i), &cellVoltage, NULL) == NULL)
        {
            cli_printfError(
                "batManagement_getHighestCellV ERROR: getting cell%d voltage went wrong!\n", i + 1);
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
    int         lvRetValue          = -1;
    static bool endOfChargeVariable = true;

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
    int         lvRetValue              = -1;
    static bool chargeToStorageVariable = false;

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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_saveFullChargeCap(void)
{
    int   lvRetValue = -1;
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_calcRemaningCharge(bool *clearingCCOverflow)
{
    int          lvRetValue = -1;
    float        remainingCap, avgCurrent, deltaCharge;
    bcc_status_t lvBccStatus;
    uint16_t     lvBccFaultStatus[BCC_STAT_CNT];
    uint16_t     retRegVal;

    // get the remaining capacity
    if(data_getParameter(A_REM, &remainingCap, NULL) == NULL)
    {
        cli_printfError("batManagement_calcRemaningCharge ERROR: getting A_REM went wrong!\n");
        return lvRetValue;
    }

    // calculate dCharge and reset CC registors
    lvRetValue = bcc_monitoring_calcDCharge(&gBccDrvConfig, &avgCurrent, &deltaCharge, true);

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
        cli_printfError(
            "batManagement_calcRemaningCharge ERROR: setting A_REM went wrong! %.3f\n", remainingCap);
        return lvRetValue;
    }

    // get the BCC fault
    lvBccStatus = bcc_spiwrapper_BCC_Fault_GetStatus(&gBccDrvConfig, BCC_CID_DEV1, lvBccFaultStatus);

    // check for errors
    if(lvBccStatus != BCC_STATUS_SUCCESS)
    {
        // return error
        cli_printfError(
            "batManagement_calcRemaningCharge ERROR: getting fault went wrong! %.3f\n", remainingCap);
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
        // remove the overflow bits and write the

        // reset the CC by writing the CC_OVT and CC_P_OVF, CC_N_OVF, SAMP_OVF bits
        lvBccStatus =
            bcc_spiwrapper_BCC_Reg_Update(&gBccDrvConfig, BCC_CID_DEV1, BCC_REG_ADC2_OFFSET_COMP_ADDR,
                BCC_R_CC_OVT_MASK + BCC_R_SAMP_OVF_MASK + BCC_R_CC_N_OVF_MASK + BCC_R_CC_P_OVF_MASK, 0);

        // check for errors
        if(lvBccStatus != BCC_STATUS_SUCCESS)
        {
            // return error
            cli_printfError(
                "batManagement_calcRemaningCharge ERROR: clearing CC went wrong! %.3f\n", remainingCap);
            return lvRetValue;
        }

        // read the register
        lvBccStatus |= bcc_spiwrapper_BCC_Reg_Read(
            &gBccDrvConfig, BCC_CID_DEV1, (BCC_REG_ADC2_OFFSET_COMP_ADDR), 1, &retRegVal);

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
                cli_printfError("batManagement_calcRemaningCharge ERROR: clearing fault went wrong! %.3f\n",
                    remainingCap);
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
 */
int batManagement_outputCellVoltages(void)
{
    int     lvRetValue = 0;
    uint8_t nCells, i;
    float   cellVoltage;

    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
        cli_printfError("main ERROR: getting n-cells went wrong! \n");
        nCells     = N_CELLS_DEFAULT;
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
        if(data_getParameter((parameterKind_t)(V_CELL1 + i), &cellVoltage, NULL) == NULL)
        {
            cli_printLock(false);
            cli_printfError("main ERROR: getting cell%d voltage went wrong!\n", i + 1);
            cellVoltage = 0.0;
            lvRetValue |= lvRetValue - 2;
            cli_printLock(true);
        }

        // output the cell voltage
        cli_printfTryLock("Cell%d voltage: %.3fV\n", i + 1, cellVoltage);
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to
 *          indicate the error.
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
 *          and if needed it will read the rest and do the calculations
 *          Then it will check the new values if there is anything wrong it
 *          will trigger the main to react on it.
 *          Afterwards it will check balancing and trigger to update measurements.
 *
 * @param   argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param   argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int batManagement_batManagTaskFunc(int argc, char *argv[])
{
    int          intValue;
    bool         measureEverything = true, increaseTargetTime = false;
    bcc_status_t bcc_status;
    // make the wait time
    struct timespec          waitTime, measureTime;
    struct timespec          oldMeasureAllTime = { 0, 0 };
    commonBatteryVariables_t commonBatteryVariables;

    // get the T_meas value
    if(data_getParameter(T_MEAS, &intValue, NULL) == NULL)
    {
        cli_printfError("batManagement_batManagTaskFunc ERROR: couldn't get t-meas!\n");
        intValue = T_MEAS_DEFAULT;
    }

    // limit the value
    intValue &= 0xFFFF;

    // calculate gMeasCycleTime and set the gTargetTime
    batManagement_calcSendInterval((uint16_t)intValue);

    // endless loop
    while(1)
    {
        // wait for the semaphore, this is how the battery management could be stopped
        sem_wait(&gBatManagementSem);

        // post a new semaphore to keep measuring, calculating, checking ...
        sem_post(&gBatManagementSem);

        // get the current time and save it as measuretime
        if(clock_gettime(CLOCK_REALTIME, &measureTime) == -1)
        {
            cli_printfError("batManagement_batManagTaskFunc ERROR: failed to get measureTime!\n");
        }

        // check if it should measure everything in the next measurement
        // This could be when it hasn't measured for too long
        if((!measureEverything) &&
            ((data_getUsTimeDiff(measureTime, oldMeasureAllTime) / 1000) > (gMeasCycleTime)))
        {
            // make sure it will measure everything
            measureEverything = true;
        }

        // if everything will be measured
        if(measureEverything)
        {
            // sav the current time
            oldMeasureAllTime.tv_sec  = measureTime.tv_sec;
            oldMeasureAllTime.tv_nsec = measureTime.tv_nsec;
        }

        // update the measurements in the local commonBatteryVariables struct
        // if measureEverything is true process everything, otherwise only the current
        bcc_status = bcc_monitoring_updateMeasurements(&gBccDrvConfig, SHUNT_RESISTOR_UOHM,
            &gLowestCellVoltage, measureEverything, &commonBatteryVariables);

        // set measureEverything to false to not keep measuring and processing everything.
        measureEverything = false;

        // check for errors and not returning because of only measuring the current
        if(bcc_status != BCC_STATUS_SUCCESS && bcc_status != ONLY_CURRENT_RETURN)
        {
            cli_printfError("batManagement ERROR: failed to update measurements! error: %d\n", bcc_status);
        }
        // if it did all the measurements
        else if(bcc_status == BCC_STATUS_SUCCESS)
        {
            // calculate the rest of the variables and set this in commonBatteryVariables and data
            bcc_monitoring_calculateVariables(
                &gBccDrvConfig, &gGateLock, gLowestCellVoltage, &commonBatteryVariables);

            // set the common battery variables in the data struct
            // do this before the check as the main loop will get the variables from the data struct
            if(data_setCommonBatteryVariables(&commonBatteryVariables))
            {
                cli_printfError("batManagement ERROR: failed to set new measurements!\n");
            }

            // check all the measurements for faults
            if(checkAllMeasurements(&commonBatteryVariables))
            {
                cli_printfError("batManagement ERROR: failed to check measurements!\n");
            }

            // make sure the main state checks transitions based on the new current
            if(g_checkForTransitionCurrentCallbackFunctionfp(&(commonBatteryVariables.I_batt)))
            {
                cli_printfError("batManagement ERROR: failed to check for current transitions!\n");
            }

            // handle the cell balancing
            if(balancing_handleCellBalancing(&commonBatteryVariables, gLowestCellVoltage))
            {
                cli_printfError("batManagement ERROR: failed to handle cell balancing!\n");
            }

            // callback that data needs to be send
            g_newMeasurementsCallbackFunctionfp();
        }
        // if it only measured the current
        else
        {
            // Since only the current has changed, set only the current
            if(data_setParameter(I_BATT, &(commonBatteryVariables.I_batt)))
            {
                cli_printfError(
                    "batManagement ERROR: Couldn't set i-batt! %.3f \n", commonBatteryVariables.I_batt);
            }

            // check the current for a peak over current fault
            if(checkCurrentMeasurement(commonBatteryVariables.I_batt))
            {
                cli_printfError("batManagement ERROR: failed to check current!\n");
            }

            // make sure the main state checks transitions based on the new current
            if(g_checkForTransitionCurrentCallbackFunctionfp(&(commonBatteryVariables.I_batt)))
            {
                cli_printfError(
                    "batManagement ERROR: failed to check for current transitions with current meas!\n");
            }
        }

        // lock mutex
        pthread_mutex_lock(&gMeasureTimeMutex);

        // get the current time
        if(clock_gettime(CLOCK_REALTIME, &waitTime) == -1)
        {
            cli_printfError("batManagement_batManagTaskFunc ERROR: failed to get waitTime!\n");
        }

        // check if the current time is more than the (old) target time
        if((waitTime.tv_sec > gTargetTime.tv_sec) ||
            ((waitTime.tv_sec == gTargetTime.tv_sec) && (waitTime.tv_nsec > gTargetTime.tv_nsec)))
        {
            // increase target time after the calculation
            increaseTargetTime = true;
        }

        // check if the slow measurement should be done
        // or the targetTime is too close to the current time (within 2*MEASURE_CURRENT_US)
        if(gSlowCurrentMeasurements ||
            (data_getUsTimeDiff(gTargetTime, waitTime) < (MEASURE_CURRENT_US << 1)))
        {
            // save the targetTime in waitTime to wait until the target time
            waitTime.tv_sec  = gTargetTime.tv_sec;
            waitTime.tv_nsec = gTargetTime.tv_nsec;

            // make sure it will measure everything
            measureEverything = true;
        }
        // otherwise prepare for the next current measurement
        else
        {
            // get the difference with the last measure time
            intValue = data_getUsTimeDiff(waitTime, measureTime);

            // make the remaining wait time in ns
            intValue = (MEASURE_CURRENT_US - intValue) * 1000;

            // make the wait time in the wait time variable
            waitTime.tv_sec += (waitTime.tv_nsec + intValue) / (MAX_NSEC + 1);
            waitTime.tv_nsec = (waitTime.tv_nsec + intValue) % (MAX_NSEC + 1);
        }

        // check if the target time needs to be increased
        if(increaseTargetTime || measureEverything)
        {
            // set the variable to false to not keep doing this
            increaseTargetTime = false;

            // keep in mind that if the measurements are enabled, that gTargetTime is reset to the current
            // time.

            // make the new target time based on the gMeasCycleTime (in ms)
            gTargetTime.tv_sec += (waitTime.tv_nsec + (gMeasCycleTime * 1000000)) / (MAX_NSEC + 1);
            gTargetTime.tv_nsec = (waitTime.tv_nsec + (gMeasCycleTime * 1000000)) % (MAX_NSEC + 1);
        }

        // unlock mutex
        pthread_mutex_unlock(&gMeasureTimeMutex);

        // wait the specified time until next measurement or until it is triggered
        intValue = sem_timedwait(&gSkipBatManagementWaitSem, &waitTime);

        // check if there is no error, meaning the semaphore got increased to measure right away
        if(!intValue)
        {
            // make sure it will measure everything
            measureEverything = true;
        }
    }

    // for compiler, shouldn't come here
    return -1;
}

/*
 * @brief   This function checks the current for peak over current.
 *          If there is an overcurrent it will set the BMS_SW_PEAK_OVER_CURRENT
 *          bit in gSWFaultVariable and trigger a sw measured fault.
 *
 * @param   current The current as a floating point variable
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int checkCurrentMeasurement(float current)
{
    variableTypes_u variable1;
    int             lvRetValue = 0;

    // get the max peak current
    if(data_getParameter(I_PEAK_MAX, &(variable1.floatVar), NULL) == NULL)
    {
        cli_printfError("checkCurrentMeasurement ERROR: couldn't get i-peak-max\n");

        // trigger an error
        variable1.floatVar = -1;
        lvRetValue         = -1;
    }

    // compare to check for an error
    if(fabs(current) > variable1.floatVar)
    {
        // trigger a current error
        // Set the bit in the variable to indicate a peak overcurrent
        gSWFaultVariable |= BMS_SW_PEAK_OVER_CURRENT;

        // trigger the fault
        g_swMeasuredFaultCallbackFunctionfp(true);
    }
    // if there is not a peak overcurrent
    else
    {
        // clear the bit of the peak current
        gSWFaultVariable &= ~(BMS_SW_PEAK_OVER_CURRENT);
    }

    // return the state
    return lvRetValue;
}

/*
 * @brief   This function checks the measurements and calculations for faults,
 *          s-in-flight and end of charge. If there is a fault, it will set the bit in
 *          gSWFaultVariable and trigger a sw measured fault. If there is an end of charge
 *          it will set the bit with batManagement_SetNReadEndOfCBCharge().
 *
 * @warning Make sure the average currents are calculated as well in pCommonBatteryVariables!
 *
 * @param   pCommonBatteryVariables The address of the local commonBatteryVariables_t struct (just filled in).
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int checkAllMeasurements(commonBatteryVariables_t *pCommonBatteryVariables)
{
    variableTypes_u variable1;
    variableTypes_u variable2;
    int             lvRetValue = 0, i;

    // check the current
    if(checkCurrentMeasurement(pCommonBatteryVariables->I_batt))
    {
        cli_printfError("checkAllMeasurements ERROR: Couldn't check current measurement!\n");

        // set the return variable to -1
        lvRetValue = -1;
    }

    // check for the s-in-flight parameter before checking the rest
    // check if the in flight parameter should be true
    // get the in flight status boolean
    if(data_getParameter(S_IN_FLIGHT, &(variable1.uint8Var), NULL) == NULL)
    {
        cli_printfError("checkAllMeasurements ERROR: getting in flight status went wrong!\n");

        // set it to true, just to be sure
        variable1.uint8Var = 1;

        // set the return variable to -1
        lvRetValue = -1;
    }

    // get the flight mode enable variable
    if(data_getParameter(FLIGHT_MODE_ENABLE, &(variable2.uint8Var), NULL) == NULL)
    {
        cli_printfError("checkAllMeasurements ERROR: getting flight mode enable went wrong!\n");

        // set it to false, just to be sure
        variable2.uint8Var = 0;

        // set the return variable to -1
        lvRetValue = -1;
    }

    // check if the flight mode enable parameter is true
    // and if the in flight status is false
    if(variable2.uint8Var && !variable1.uint8Var)
    {
        // get the flight mode current
        if(data_getParameter(I_FLIGHT_MODE, &(variable1.uint8Var), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting flight mode current went wrong!\n");

            // set it to the max just ot be sure
            variable1.uint8Var = UINT8_MAX;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // get the max pack output current
        if(data_getParameter(I_OUT_MAX, &(variable2.floatVar), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting i-out-max went wrong!\n");

            // set it to 0, just to be sure
            variable2.floatVar = 0;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // check if the 1s avg current is more than the flight mode current
        // and if the 1s avg current is less than the i-out-max current
        if((((pCommonBatteryVariables->I_batt_avg * -1)) > ((float)variable1.uint8Var)) &&
            ((pCommonBatteryVariables->I_batt_avg * -1) < variable2.floatVar))
        {
            // set s-in-flight to 1
            variable1.uint8Var = 1;

            // Set the in flight parameter
            if(data_setParameter(S_IN_FLIGHT, &variable1.uint8Var))
            {
                cli_printfError("checkAllMeasurements ERROR: couldn't set s-in-flight\n");

                // set the return variable to -1
                lvRetValue = -1;
            }
        }
    }
    // if the s in flight is true
    else if(variable1.uint8Var)
    {
        // check if it should be false

        // get the sleep overcurrent
        if(data_getParameter(I_SLEEP_OC, &(variable1.uint8Var), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting sleep overcurrent went wrong!\n");

            // set it to the max just in case
            variable1.uint8Var = UINT8_MAX;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // check if the battery is being charged
        // if the 1s avg current minus the sleep overcurrent is more than 0
        if((pCommonBatteryVariables->I_batt_avg - (float)(variable1.uint8Var / 1000.0)) > 0)
        {
            // set in flight status to false
            variable1.uint8Var = 0;

            // Set the in flight parameter
            if(data_setParameter(S_IN_FLIGHT, &(variable1.uint8Var)))
            {
                cli_printfError("checkAllMeasurements ERROR: couldn't set parameter s-in-flight\n");

                // set the return variable to -1
                lvRetValue = -1;
            }
        }
        // if the 1s avg current is not the reason to set s-in-flight to false
        else
        {
            // check if the 10s avg is the reason to make s-in-flight false
            // get the flight mode current
            if(data_getParameter(I_FLIGHT_MODE, &(variable1.uint8Var), NULL) == NULL)
            {
                cli_printfError("checkAllMeasurements ERROR: getting flight mode current went wrong!\n");

                // set it to max just to be sure to reset s-in-flight
                variable1.uint8Var = UINT8_MAX;

                // set the return variable to -1
                lvRetValue = -1;
            }

            // check if the 10s avg current is less than the flight mode current
            // And the (1s) avg current is lower than the flight mode current
            if((((pCommonBatteryVariables->I_batt_10s_avg * -1)) < ((float)variable1.uint8Var)) &&
                (((pCommonBatteryVariables->I_batt_avg * -1)) < ((float)variable1.uint8Var)))
            {
                // make the in flight status variable false again
                variable1.uint8Var = 0;

                // Set the in flight parameter
                if(data_setParameter(S_IN_FLIGHT, &(variable1.uint8Var)))
                {
                    cli_printfError(
                        "checkAllMeasurements ERROR: couldn't set s-in-flight to %d\n", variable1.uint8Var);

                    // set the return variable to -1
                    lvRetValue = -1;
                }
            }
        }
    }

    // check the average current for faults
    // check if not a charging current
    if(pCommonBatteryVariables->I_batt_avg <= 0.0)
    {
        // get the max pack output current
        if(data_getParameter(I_OUT_MAX, &(variable1.floatVar), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: couldn't get i-out-max\n");

            // trigger fault
            variable1.floatVar = -1;

            // set the return variable to -1
            lvRetValue = -1;
        }
    }
    // if charging current
    else
    {
        // get the max charge current
        if(data_getParameter(I_CHARGE_MAX, &(variable1.floatVar), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: couldn't get i-charge-max\n");

            // trigger fault
            variable1.floatVar = -1;

            // set the return variable to -1
            lvRetValue = -1;
        }
    }

    // compare to check for an error
    if(fabs(pCommonBatteryVariables->I_batt_avg) > variable1.floatVar)
    {
        // Check if the bit is not set in the fault variable or
        // if the system is not in the fault mode
        if((!(gSWFaultVariable & BMS_SW_AVG_OVER_CURRENT)) ||
            ((data_getMainState() != FAULT_OFF) && (data_getMainState() != FAULT_ON)))
        {
            // Set the bit to indicate an avg overcurrent
            gSWFaultVariable |= BMS_SW_AVG_OVER_CURRENT;

            // call the callbackfunction for an overcurrent
            g_swMeasuredFaultCallbackFunctionfp(true);
        }
    }
    // if there is no overcurrent
    else
    {
        // Clear the bit to indicate an avg overcurrent
        gSWFaultVariable &= ~(BMS_SW_AVG_OVER_CURRENT);
    }

    // check if the voltage does not exceed the OV

    // check if end of charge voltage needs to be checked and if charging to storage voltage
    if((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) != 2 &&
        batManagement_SetNReadChargeToStorage(false, 0))
    {
        // get the cell storage voltage
        if(data_getParameter(V_STORAGE, &(variable1.floatVar), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting storage voltage went wrong!\n");
            variable1.floatVar = V_STORAGE_DEFAULT;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // get the cell voltage margin
        if(data_getParameter(V_CELL_MARGIN, &variable2.uint8Var, NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting margin voltage went wrong!\n");
            variable2.uint8Var = V_CELL_MARGIN_DEFAULT;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // add the cell voltage margin to the storage voltage variable
        // to charge a little bit more than the storage voltage
        variable1.floatVar = variable1.floatVar + (float)((float)(variable2.uint8Var) / 1000.0);
    }
    // if normal or charge to cell ov
    else
    {
        // get the cell over voltage
        if(data_getParameter(V_CELL_OV, &(variable1.floatVar), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: getting cell over voltage went wrong!\n");

            // set the overvoltage to 0 to trigger a fault
            variable1.floatVar = 0;

            // set the return variable to -1
            lvRetValue = -1;
        }
    }

    // check the voltages
    // loop through all the voltages
    for(i = 0; i < pCommonBatteryVariables->N_cells; i++)
    {
        // check if the cell voltage is more or equal as the cell overvoltage
        if(pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] >= variable1.floatVar)
        {
            // Check if charging to end voltage
            if(batManagement_setNGetChargingState(false, 0) &&
                ((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) != 2))
            {
                // output the end voltage to the user
                cli_printf("End of charge voltage! cell%d %.3f >= %.3f\n", (i + 1),
                    pCommonBatteryVariables->V_cellVoltages.V_cellArr[i], variable1.floatVar);

                // set the end of charge voltage variable
                batManagement_SetNReadEndOfCBCharge(true, 2);

                // trigger the main loop, but no fault
                g_swMeasuredFaultCallbackFunctionfp(false);
            }

            // check if the bit is not already set or it is not in the fault mode
            if((!(gSWFaultVariable & (BMS_SW_CELL1_OV << i))) ||
                (data_getMainState() != FAULT_OFF && data_getMainState() != FAULT_ON))
            {
                // Set the bit in the variable to indicate a cell overvoltage
                gSWFaultVariable |= (BMS_SW_CELL1_OV << i);

                // call the callbackfunction for an overcurrent
                g_swMeasuredFaultCallbackFunctionfp(true);
            }
        }
        // if there is not a cell overvoltage for that cell
        else
        {
            // clear the bit of the cell overvoltage
            gSWFaultVariable &= ~(BMS_SW_CELL1_OV << i);
        }
    }

    // check if needs to check for end of CB charge current
    if(pCommonBatteryVariables->I_batt_avg > 0 && ((batManagement_SetNReadEndOfCBCharge(false, 0) & 1) != 1))
    {
        // get the end of charge current
        if(data_getParameter(I_CHARGE_FULL, &(variable1.uint16Var), NULL) == NULL)
        {
            cli_printfError("checkAllMeasurements ERROR: couldn't get i-charge-full!\n");
            variable1.uint16Var = I_CHARGE_FULL_DEFAULT;

            // set the return variable to -1
            lvRetValue = -1;
        }

        // check if the current is lower or equal than that current
        if(pCommonBatteryVariables->I_batt_avg <= ((float)variable1.uint16Var / 1000))
        {
            cli_printf("End of charge current %.3fA <= %.3fA\n", pCommonBatteryVariables->I_batt_avg,
                ((float)variable1.uint16Var / 1000));

            // set the variable to indicate end of charge current
            batManagement_SetNReadEndOfCBCharge(true, 1);

            // trigger the main loop, but no fault
            g_swMeasuredFaultCallbackFunctionfp(false);
        }
    }

    // return
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
    uint8_t period   = T_MEAS_MAX / measMs;
    uint8_t elements = 10;

    // lock mutex
    pthread_mutex_lock(&gMeasureTimeMutex);

    // set the new average interval to every second if possible
    if(measMs <= 1000)
    {
        // calculate the new amount of elements if there is rest
        while(period % elements)
        {
            // decrease the elements
            elements--;
        }

        // set the interval to make sure every second the moving avg is done
        bcc_monitoring_setAverageInterval((uint8_t)period / elements, elements);
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

/*!
 * @brief   function to initialize the BCC
 *          could return without init if communication failed!
 *
 * @param none
 * @return the bcc status from bcc_status_t
 */
static bcc_status_t BatManagement_initializeBCC(void)
{
    int      lvRetValue = -1, i;
    float    lowerTH, upperTH, VoutVoltageFactor;
    uint16_t PCBOV, PCBUV;
    uint8_t  configBits, nCells, measCycle, sleepCurrent;
    // bool enableBatTemp;
    void *dataReturn;

    /* Calculate BCC diagnostic time constants (g_bccData.diagTimeConst). */
    /* CT filter components. */
    g_ctFilterComp.rLpf1 = 3000U; /* R_LPF-1 3kOhm */
    g_ctFilterComp.rLpf2 = 2000U; /* R_LPF-2 2kOhm */
    g_ctFilterComp.cLpf  = 100U;  /* C_LPF 100nF */
    g_ctFilterComp.cIn   = 47U;   /* C_IN 47nF */

    /* ISENSE filter components. */
    g_isenseFilterComp.rLpfi  = 127U;   /* R_LPFI 127Ohm */
    g_isenseFilterComp.cD     = 2200U;  /* C_D 2.2uF */
    g_isenseFilterComp.cLpfi  = 47U;    /* C_LPFI 47nF */
    g_isenseFilterComp.rShunt = 500U;   /* R_SHUNT 0.5mOhm */
    g_isenseFilterComp.iMax   = 24570U; /* I_MAX 24.57A */

    // bcc_diag_calcDiagConst(&g_ctFilterComp, &g_isenseFilterComp, &g_bccData.diagTimeConst);

    /* Initialize BCC driver configuration structure (gBccDrvConfig). */
    gBccDrvConfig.drvInstance = BCC_INITIAL_DRIVER_INSTANCE;
    gBccDrvConfig.devicesCnt  = BCC_DEVICES;

    gBccDrvConfig.device[BCC_FIRST_INDEX]  = BCC_DEVICE_MC33772;
    gBccDrvConfig.cellCnt[BCC_FIRST_INDEX] = BCC_DEFAULT_CELLCNT;
    gBccDrvConfig.commMode                 = BCC_MODE_SPI;

    /* Precalculate NTC look up table for fast temperature measurement. */
    g_ntcConfig.rntc    = NTC_PULL_UP;  /* NTC pull-up 10kOhm */
    g_ntcConfig.refTemp = NTC_REF_TEMP; /* NTC resistance 10kOhm at 25 degC */
    g_ntcConfig.refRes  = NTC_REF_RES;  /* NTC resistance 10kOhm at 25 degC */
    g_ntcConfig.beta    = NTC_BETA;
    bcc_monitoring_fillNtcTable(&g_ntcConfig);

    i = 0;

    // do the verification
    do
    {
        // check if SPI is initialized
        lvRetValue = bcc_spiwrapper_BCC_VerifyCom(&gBccDrvConfig, BCC_CID_DEV1);

        i++;
    } while(lvRetValue != BCC_STATUS_SUCCESS || i == 3);

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
    dataReturn = (int32_t *)data_getParameter(SENSOR_ENABLE, &nCells, NULL);

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
    lvRetValue = bcc_configuration_disableNEnableANx(&gBccDrvConfig, BCC_CID_DEV1, configBits, !nCells);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set batt temperature measurement: %d\n",
            lvRetValue);
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
    lvRetValue = bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, configBits, &lowerTH, &upperTH);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set PCB temp BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the min and max voltages
    data_getParameter(V_MIN, &PCBUV, NULL);
    data_getParameter(V_MAX, &PCBOV, NULL);

    // make the temperature bits (0b1101)
    configBits = (1 << ANX_V_OUT);

    // get the v-out voltage divider variable and place it in outputVoltage for now
    if(data_getParameter(F_V_OUT_DIVIDER_FACTOR, &VoutVoltageFactor, NULL) == NULL)
    {
        cli_printfWarning("BatManagement_initializeBCC: Using default v-out voltage factor (%f)\n",
            F_V_OUT_DIVIDER_FACTOR_DEFAULT);

        // set the default value
        VoutVoltageFactor = F_V_OUT_DIVIDER_FACTOR_DEFAULT;
    }

    // NOTE: for now the PCBOV will only be corrected with a save and reboot.
    // TODO: Make that it will update bcc_configuration_changeANxVTH(ANX_V_OUT) when F_V_OUT_DIVIDER_FACTOR
    // changes

    // make it millivolt and div by the voltage factor
    PCBUV = 0; //((PCBUV & 0xFF) * 1000) / VoutVoltageFactor;
    PCBOV = ((PCBOV & 0xFF) * 1000) / VoutVoltageFactor;

    // seting PCB voltage threshold registers
    lvRetValue = bcc_configuration_changeANxVTH(&gBccDrvConfig, BCC_CID_DEV1, configBits, &PCBUV, &PCBOV);

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
    lvRetValue = bcc_configuration_ChangeCellVTH(&gBccDrvConfig, BCC_CID_DEV1, &lowerTH, &upperTH);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("BatManagement_initializeBCC ERROR: failed to set cell TH BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // TODO set AN4 threshold value (bat output threshold) (PCB threshold?)

    // get the measurement cycle for in the sleep mode
    data_getParameter(T_BCC_SLEEP_CYCLIC, &measCycle, NULL);

    // set the cyclic timer
    lvRetValue = bcc_configuration_changeCyclicTimer(&gBccDrvConfig, BCC_CID_DEV1, measCycle);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError(
            "BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
        return lvRetValue;
    }

    // get the number of cells
    data_getParameter(N_CELLS, &nCells, NULL);

    // set the odd or even
    lvRetValue = bcc_configuration_changeCellCount(&gBccDrvConfig, BCC_CID_DEV1, nCells);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError(
            "BatManagement_initializeBCC ERROR: failed to set cell odd or even BCC: %d\n", lvRetValue);
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
    float        lowerTH, upperTH;
    uint8_t      temperatureBits;

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
    lvRetValue =
        bcc_configuration_changeTempTH(&gBccDrvConfig, BCC_CID_DEV1, temperatureBits, &lowerTH, &upperTH);

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
    bool         lvRetValue = 0;
    bcc_status_t lvBccStatus;
    static bool  chargingMode    = false;
    static bool  oldChargingMode = true;

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
            cli_printfError("batManagement_setNGetChargingState ERROR: couldn't set the right temp TH: %d\n",
                lvBccStatus);
        }
    }

    // return
    return lvRetValue;
}

//#endif
