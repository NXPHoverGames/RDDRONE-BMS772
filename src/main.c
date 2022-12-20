/****************************************************************************
 * nxp_bms/BMS_v1/src/main.c
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

#include <stdio.h>
#include <stdlib.h>
#include <arch/board/board.h>
#include <sched.h>
#include <semaphore.h>
#include <math.h>

#include "data.h"
#include "cli.h"
#include "ledState.h"
#include "gpio.h"
#include "batManagement.h"
#include "uavcan.h"
#include "sbc.h"
#include "nfc.h"
#include "a1007.h"
#include "spi.h"
#include "SMBus.h"
#include "i2c.h"
#include "power.h"
#include "display.h"

//#warning each semaphore used to signal a task needs to call sem_setprotocol(&sem, SEM_PRIO_NONE); after sem_init!!!

/****************************************************************************
 * Defines
 ****************************************************************************/
#define BMS_VERSION_STRING "bms5.1-10.1"
//#define DONT_DO_UAVCAN

#ifndef MCU_POWER_DOMAIN
    #define MCU_POWER_DOMAIN    0
#endif

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

//! this define is used with macro BYTE_TO_BINARYto print a byte value to binary
// #ifndef BYTE_TO_BINARY_PATTERN
// #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
// #endif

#define DEFAULT_PRIORITY                100
#define MAIN_LOOP_PRIORITY              140
#define DATA_CHANGED_HANDLER_PRIORITY   130
#define UPDATER_PRIORITY                50
#define MAIN_LOOP_STACK_SIZE            2048
#define DATA_HANDLER_STACK_SIZE         1024 + 512  
#define DEFAULT_UPDATER_STACK_SIZE      1024 + 512 + 256

#define CHANGED_DATA_ARRAY_ELEMENTS     20

#define CHARGE_DETECTED_MARGIN          0.010 // [A]

//! this define is used to make sure it doesn't keep going in the charge with CB (from relaxation) when done
#define AMOUNT_CB_CHARGE_CYCLES_MAX     5

#define BUTTON_TIME_FOR_DEEP_SLEEP      5   // [s]
#define SELF_DISCHARGE_WAIT_TIME        10  // [s]

//! this define is used so the CC overflow message isn't outputted every second 
#define CC_OVERFLOW_MESS_TIMEOUT_TIME   120 // [s]

#define MAIN_LOOP_WAIT_TIME_MS          100 // [ms]
#define MAIN_LOOP_LONG_WAIT_TIME_S      2   // [s]

// check if PM module is configured correctly to go to VLPR mode 
#if (!defined(CONFIG_VLPR_STANDBY)) || (!defined(CONFIG_VLPR_SLEEP)) 
  #error configure PM correctly, see/set defconfig
#endif

/****************************************************************************
 * Types
 ****************************************************************************/
//! @brief This enum can be used to set or get the transition.
typedef enum{
    SLEEP_VAR,
    CHAR_VAR,
    DISCHAR_VAR
}transitionVars_t;

// the string array for the states
static const char *gStatesArray[] = 
{
    //"INIT", "NORMAL", "CHARGE", "SLEEP", "OCV", "FAULT", "SELF_DISCHARGE", "DEEP_SLEEP"
    FOR_EACH_STATE(GENERATE_STRING)
};

/****************************************************************************
 * private data
 ****************************************************************************/
// make a mutex lock for the main state 
pthread_mutex_t gStateLock;
bool gStateLockInitialized = false;

// make a mutex lock for the charge state
pthread_mutex_t gChargeStateLock;
bool gChargeStateLockInitialized = false;

pthread_mutex_t gTransVarLock;
bool gTransVarInitialized = false;

//! mutex for the state commands
pthread_mutex_t gStateCommandLock;
bool gStateCommandInitialized = false;

//! mutex for enabling or disabling the UAVCAN messages
pthread_mutex_t gSetUavcanMessagesLock;
bool gSetUavcanMessagesInitialized = false;

//! mutex for enabling or disabling the NFC Update
pthread_mutex_t gSetNfcUpdateLock;
bool gSetNfcUpdateLockInitialized = false;

//! mutex for enabling or disabling the display Update
pthread_mutex_t gSetDisplayUpdateLock;
bool gSetDisplayUpdateLockInitialized = false;

/*! @brief  semaphore to do the main loop*/
static sem_t gMainLoopSem;

static sem_t gDataChangedSem;
static bool gChangedParameterTaskStarted = false;

/*! @brief  semaphore to update the NFC information*/
static sem_t gUpdaterSem;
/*! @brief  to indicate the semaphore and the task are initialized*/
static bool gUpdaterInitialized = false;

parameterKind_t gChangedParametersArr[CHANGED_DATA_ARRAY_ELEMENTS];// = {NONE, NONE, NONE, NONE};
uint32_t gChangedDataArr[CHANGED_DATA_ARRAY_ELEMENTS];// = {NULL, NULL, NULL, NULL};

states_t gCurrentState = SELF_TEST;
charge_states_t gCurrentChargeState = CHARGE_START;

static bool gMainLoopStarted = false;       // to indicate the main loop is started

static bool gDischargeDetected = true;
static bool gChargeDetected = false;
static bool gSleepDetected = false;

static bool gBCCRisingFlank = false; 
static bool gButtonRisingFlank = false; 
static bool gButtonPressFlank = false;
static bool gSInFlightChangedFalse = false;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief this function will implement the main state machine
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int mainTaskFunc(int argc, char *argv[]);

/*!
 * @brief function for a task to handle a parameter change if semaphore is available
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int handleParamChangeFunc(int argc, char *argv[]);

/*!
 * @brief function for a task to handle the update of the NFC, SMBus and display
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int updaterTaskFunc(int argc, char *argv[]);

/*!
 * @brief function to handle a parameter change, will set semaphore available
 * 
 * @param changedParameter the parameter that changed
 * @param newValue the new value that is set
 */
int parameterChangeFunction(parameterKind_t changedParameter, void *newValue);

/*!
 * @brief function that will will be called if there is an overcurrent fault  
 * 
 * @param triggerFault boolean if a fault should be triggered.
 */
void swMeasuredFaultFunction(bool triggerFault);

/*!
 * @brief function that will will be called to change the LED
 * 
 * @param newColor the new LED color
 * @param newAltColor the new alternating color
 * @param blinkPeriodms false leds will not blink
 */
void changeLedColor(LEDColor_t newColor, LEDColor_t newAltColor, uint16_t blinkPeriodms);

/*!
 * @brief function to be called when new data is set
 * 
 */
void newMeasurementsFunction(void);

/*!
 * @brief Function that will be called with an interrupt occurs on a GPIO pin.
 *        Handling of the pin ISR should be done in this function
 * 
 * @param signo the signal number that triggers this ISR
 * @param siginfo signal information from the siginfo_t struct
 * @param context the context pointer
 */
void gpioIsrFunction(int signo, siginfo_t *siginfo, void *context);

/*!
 * @brief function that will be called when it needs to process a cli command when the CLI can't do this
 * 
 * @param command       the command that needs to be processed 
 * @param value         if 1 that CLI_SHOW command is enabled, if 0 it is disabled
 */
void processCLICommand(commands_t command, uint8_t value);

/*!
 * @brief function that will return the main state, but it will use the mutex 
 * 
 * @return the state
 */
states_t getMainState(void);

/*!
 * @brief function that will set the main state, but it will use the mutex 
 * 
 * @param newState the new state
 */
static int setMainState(states_t newState);

/*!
 * @brief   function that will return the charge state, but it will use the mutex 
 * 
 * @return  the state
 */
charge_states_t getChargeState(void);

/*!
 * @brief   function that will set the charge state, but it will use the mutex 
 * 
 * @param   newState the new state
 */
static int setChargeState(charge_states_t newState);

/*!
 * @brief   function that will return one of the transition variables
 * 
 * @param   variable the variable to get
 *
 * @return  the value of the variable
 */
static bool getTransitionVariable(transitionVars_t variable);

/*!
 * @brief   function that will set one of the transition variables
 * 
 * @param   variable the variable to set
 * @param   newValue the new value to set
 *
 * @return  0 if ok
 */
static int setTransitionVariable(transitionVars_t variable, bool newValue);

/*!
 * @brief   function that will set one of the transition variables
 *
 * @param   setNotGet if true it is used to set the variable, false to get the variable
 * @param   newValue if setNotGet is true, this is the new value
 *
 * @return  the state command variable, 0 if none is set, CMD_ERROR if error
 */
static stateCommands_t setNGetStateCommandVariable(bool setNotGet, stateCommands_t newValue);

/*!
 * @brief   function that will calculate and return the OCV period time
 * 
 * @param   newTime the address of the variable to become the OCV timer period
 * @param   oldState the oldState of the state machine
 * @warning Keep in mind that this function needs to be called before 
 *          the oldState is set with the current state (lvOldState = getMainState())
 *          when the OCV timer needs to be increased
 *
 * @return  0 if ok
 */
static int getOcvPeriodTime(int32_t *newTime, states_t oldState);

/*!
 * @brief   function that is used to enable or disable the UAVCAN messages 
 *          or get the value if it should output the UAVCAN messages.
 * 
 * @param   setNGet if true it will set the value if the UAVCAN messages should be outputted.
 *          if false it will return 1 if it should be outputted, 0 otherwise.
 * @param   enable if setNGet is true, if enable true the UAVCAN messages are enabled 
 *          to send after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableUavcanMessages(bool setNGet, bool enable);

/*!
 * @brief   function that is used to enable or disable the NFC update 
 *          or get the value if it should update the NFC.
 * 
 * @param   setNGet if true it will set the value if the NFC should be updated.
 *          if false it will return 1 if it should be updated, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the NFC update is enabled 
 *          to update after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableNFCUpdates(bool setNGet, bool enable);

/*!
 * @brief   function that is used to enable or disable the display update 
 *          or get the value if it should update the display.
 * 
 * @param   setNGet if true it will set the value if the display should be updated.
 *          if false it will return 1 if it should be updated, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the display update is enabled 
 *          to update after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableDisplayUpdates(bool setNGet, bool enable);

/*!
 * @brief   Function that is used to call a usleep (task switch as well)
 *          but it will kick the watchdog first. 
 * 
 * @param   usec the number of microseconds to wait.
 *
 * @return  On successful completion, usleep() returns 0. Otherwise, it returns -1
 *          and sets errno to indicate the error.
 */
static int usleepMainLoopWatchdog(useconds_t usec);

/*!
 * @brief   Function that is used to check if the tasks are done
 *          Like the measurement task, updater task and other tasks
 * 
 * @param   done the address of the varible to become true if the tasks are done, 
 *          false otherwise.
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int checkSequenceTaskStatus(bool *done);

/*!
 * @brief   Function that is used to get out of the sem_timedwait()
 * 
 * @param   none
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int escapeMainLoopWait(void);

/****************************************************************************
 * main
 ****************************************************************************/
// keep in mind that this program can be called more than once to enter parameters
int bms_main(int argc, char *argv[])
{
    // clear the screen from cursor down
    cli_printf("\e[0J");

    // reset the color if there was any color
    cli_printf("\e[39m");
    
    // the messages for the tasks
    int lvRetValue;
    int errcode;
    int i;
    bool resetCauseExWatchdog = 0;

    if(!gStateLockInitialized)
    {
        cli_printf("BMS version: %s\n", BMS_VERSION_STRING);

// output a warning if debug assertions are enabled
#ifdef CONFIG_DEBUG_ASSERTIONS
        cli_printfWarning("WARNING: Debug assertions are enabled!\n");
#endif
        
        // output the state 
        cli_printf("SELF_TEST mode\n");

        // sleep to make sure the CLI doesn't print things over each other
        // is only needed until the CLI is initialized
        usleep(1000);

        // initialze the mutex
        pthread_mutex_init(&gStateLock, NULL);
        gStateLockInitialized = true;

        // Check if the reset cause is the watchdog
        resetCauseExWatchdog = RESET_CAUSE_EXT_PIN & data_getResetCause();

        // Check if the reset cause is the watchdog
        if(resetCauseExWatchdog) 
        {
            cli_printfWarning("WARNING: Not doing self-test due to watchdog reset!\n");
        }
    } 

    // check if not initialized 
    if(!gChargeStateLockInitialized)
    {
        // initialze the mutex
        pthread_mutex_init(&gChargeStateLock, NULL);
        gChargeStateLockInitialized = true;
    }

    // check if not initialized 
    if(!gTransVarInitialized)
    {
        // initialize the transition variable mutex
        pthread_mutex_init(&gTransVarLock, NULL);
        gTransVarInitialized = true;
    }

    // check if not initialized 
    if(!gStateCommandInitialized)
    {
        // initialize the state command lock
        pthread_mutex_init(&gStateCommandLock, NULL);
        gStateCommandInitialized = true;
    }

    // check if not initialized 
    if(!gSetUavcanMessagesInitialized)
    {
        // initialize the state command lock
        pthread_mutex_init(&gSetUavcanMessagesLock, NULL);
        gSetUavcanMessagesInitialized = true;
    }

    // check if not initialized 
    if(!gSetNfcUpdateLockInitialized)
    {
        // initialize the state command lock
        pthread_mutex_init(&gSetNfcUpdateLock, NULL);
        gSetNfcUpdateLockInitialized = true;
    }

    // check if not initialized 
    if(!gSetDisplayUpdateLockInitialized)
    {
        // initialize the state command lock
        pthread_mutex_init(&gSetDisplayUpdateLock, NULL);
        gSetDisplayUpdateLockInitialized = true;
    }

    // initialize the functions

    // initialize the LED and make it RED
    lvRetValue = ledState_initialize(RED, resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize leds! code %d\n", 
            lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST LEDs: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        } 
        
    }

    // initialize data structure
    // create the changed parameter task if not started
    if(!gChangedParameterTaskStarted)
    {
        // initialize the changed parameter semaphore
        sem_init(&gDataChangedSem, 0, 0);
        // initialize the arrays
        for(i = 0; i < CHANGED_DATA_ARRAY_ELEMENTS; i++)
        {
            gChangedParametersArr[i] = NONE;
            gChangedDataArr[i] = INT32_MAX;
        }       

        // create the task to handle a parameter change
        lvRetValue = task_create("changed data handler", DATA_CHANGED_HANDLER_PRIORITY, 
            DATA_HANDLER_STACK_SIZE, handleParamChangeFunc, NULL);
        if (lvRetValue < 0)
        {
            errcode = errno;

            cli_printfError("main ERROR: Failed to start changed data handler task: %d\n", errcode);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return 0;
            }
        }
    }

    // check if the task is initialized
    if(!gUpdaterInitialized)
    {
        // initialize the changed parameter semaphore
        sem_init(&gUpdaterSem, 0, 0);

        // create the updater task
        lvRetValue = task_create("updater", UPDATER_PRIORITY,
            DEFAULT_UPDATER_STACK_SIZE, updaterTaskFunc, NULL);
        if (lvRetValue < 0)
        {
            errcode = errno;

            cli_printfError("main ERROR: Failed to start updater task: %d\n", errcode);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return 0;
            }
        }
    }

    // initialzie the data part
    lvRetValue = data_initialize(&parameterChangeFunction, &getMainState, 
        &getChargeState);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize data! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            return lvRetValue;
        }
    }

    // initialize the cli
    cli_initialize(&processCLICommand);

    // initialze the power part
    lvRetValue = power_initialize();
    if(lvRetValue)
    {
        cli_printfError("main ERROR: failed to initialize power! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            return lvRetValue;
        }
    }

    // initialize the GPIO
    lvRetValue = gpio_init(resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize gpio! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }
    }

    // initialize the SPI
    lvRetValue = spi_initialize();
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize SPI! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            return lvRetValue;
        }
    }
    
    // initialze the SBC
    lvRetValue = sbc_initialize(resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize SBC! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST SBC: \e[31mFAIL\e[39m\n");
            return lvRetValue;
        }
    }

#ifndef DONT_DO_UAVCAN

    // initialize the UAVACAN
    lvRetValue = uavcan_initialize();
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize UAVCAN! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            return lvRetValue;
        }
    }

#endif

    // initialize the battery management
    lvRetValue = batManagement_initialize(&swMeasuredFaultFunction, 
        &changeLedColor, &newMeasurementsFunction, resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize batManagement! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            return lvRetValue;
        }
    }

    // initialize the I2C
    lvRetValue = i2c_initialize();
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize i2c! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            //return lvRetValue;
        }
    }

    // initialze the NFC
    lvRetValue = nfc_initialize(resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize nfc! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST NFC: \e[31mFAIL\e[39m\n");
        }
    }

    // initialze the A1007
    lvRetValue = a1007_initialize(resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize A1007! code %d\n", lvRetValue);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST A1007: \e[31mFAIL\e[39m\n");
        }
    }

    // initialize the SMBus
    lvRetValue = SMBus_initialize();
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("main ERROR: failed to initialize SMBus! code %d\n", lvRetValue);
    }

    // initialze the display
    lvRetValue = display_initialize(resetCauseExWatchdog);
    if(lvRetValue)
    {
        // output to the user
        cli_printfWarning("main WARNING: failed to initialize display! code %d\n", lvRetValue);
        cli_printf("Could be that there is no display\n");

        //Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            cli_printf("SELF-TEST DISPLAY: \e[31mFAIL\e[39m\n");
        }
    }

    // check if a command was given
    if(argc > 1)
    {
        // TODO make parameters only changeable in normal and sleep, (self_discharge, fault)
        
        // start the cli task
        cli_processCommands(argc, argv);

        // return 
        return 0;
    }

    // check if the main loop is not started
    if(!gMainLoopStarted)
    {
        // initialize the main loop semaphore
        sem_init(&gMainLoopSem, 0, 0);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            // if no error occured with a GPIO, the GPIO SELF-TEST has passed
            cli_printf("SELF-TEST GPIO: \e[32mPASS\e[39m\n");

            // output to the user
            cli_printfGreen("ALL SELF-TESTS PASSED!\n");
        }

        // go to the INIT state
        setMainState(INIT);

        // create the main loop task
        lvRetValue = task_create("mainLoop", MAIN_LOOP_PRIORITY, MAIN_LOOP_STACK_SIZE, mainTaskFunc, NULL);
        if (lvRetValue < 0)
        {
            // get the error
            errcode = errno;

            // error
            cli_printfError("main ERROR: Failed to start main loop task: %d\n", errcode);
            return 0;
        }
    }

    // return
    return 0;
}


static int mainTaskFunc(int argc, char *argv[])
{   
    gMainLoopStarted = true;
    uint32_t BMSFault;
    int lvRetValue;
    charge_states_t oldChargeState = CHARGE_COMPLETE;
    struct timespec sampleTime;
    struct timespec buttonPressedTime;
    struct timespec selfDischargeTime;
    struct timespec currentTime;
    struct timespec cellUnderVoltageTime;
    struct timespec lastMessageTime = {0, 0};
    struct timespec sampleTime2;
    int32_t int32tVal;
    float floatVal;
    uint8_t amountOfCBChargeCycles = 0;
    uint16_t amountOfMissedMessages = 0;
    bool onlyOnce = false;
    bool chargeToStorage = false;
    bool boolValue;
    bool outputtedFirstMessage = false;
    bool deepsleepTimingOn = false;
    bool firstTimeStartup = true;
    bool cellUnderVoltageDetected = false;
    mcuPowerModes_t mcuPowerMode;
    bool buttonState = false;
    bool oldButtonState = true;
    bool didNotDisconnectPower = false;

    // get the variables if the fault happend
    gBCCRisingFlank = gpio_readPin(BCC_FAULT);
    
    // get the variables if the button press happend
    gButtonRisingFlank = gpio_readPin(SBC_WAKE);
    gButtonPressFlank = !gButtonRisingFlank;

    // register the pins for the ISR
    lvRetValue = gpio_registerISR((uint16_t) 
        ((1 << BCC_FAULT)), gpioIsrFunction);

    // check for errors
    if(lvRetValue)
    {
        // output the error
        cli_printfError("main ERROR: GPIO register ISR went wrong! %d\n", lvRetValue);

        // return
        return lvRetValue;
    }

    // make sure that if there was an interrupt it is read
    if(!gBCCRisingFlank && gpio_readPin(BCC_FAULT))
    {
        // set it
        gBCCRisingFlank = true;
    }

    // make sure that if there was an interrupt it is read
    if(!gButtonRisingFlank && gpio_readPin(SBC_WAKE))
    {
        // set it
        gButtonRisingFlank = true;
        gButtonPressFlank = false;
    }

    buttonState = gpio_readPin(SBC_WAKE);
    oldButtonState = !buttonState;

    // make the oldState different
    states_t lvOldState = !getMainState();

    // start the task
    cli_printf("BMS main loop!\n");

    // loop in the state machine
    while(1)
    {
        // get the buttonstate
        buttonState = gpio_readPin(SBC_WAKE);

        // check if changed
        if(buttonState != oldButtonState)
        {
            // check if the button pin is high
            if(buttonState)
            {
                // set the variable high (release)
                gButtonRisingFlank = true;
            }
            // if it is low
            else
            {
                // set the variable high (press)
                gButtonPressFlank = true;
            }

            // set the old one
            oldButtonState = buttonState;
        }

        // check if the BCC fualt pin is high and the variable is not high
        // if it was not noticed
        if((getMainState() != FAULT) && (getMainState() != INIT) && (getMainState() != CHARGE) &&
          (getMainState() != DEEP_SLEEP) && (!gBCCRisingFlank) && (gpio_readPin(BCC_FAULT)))
        {
            // output to user why
            cli_printf("Rising edge BCC pin not noticed, setting fault now!\n");

            // set the variable high to go through the error handler
            gBCCRisingFlank = true;
        }

        // check if the BCC pin is high or there is an other fault
        if(gBCCRisingFlank)
        {
            // check the MCU power mode and check for errors
            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
            if(mcuPowerMode == ERROR_VALUE)
            {
                // error
                cli_printfError("main ERROR: Could not get MCU power mode 1\n");
            }

            // check if the runmode needs to be changed to RUN
            if((mcuPowerMode == STANDBY_MODE) || 
                (mcuPowerMode == VLPR_MODE) || 
                (mcuPowerMode == ERROR_VALUE))
            {
                // change the run mode
                if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                {
                    // power mode switch went wrong
                    cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                    
                    // check if charging
                    if(getMainState() == CHARGE)
                    {
                        // disconnect the battery!
                        // open the gate
                        if(batManagement_setGatePower(GATE_OPEN) != 0)
                        {
                            cli_printfError("main ERROR: Failed to open gate\n");
                        }
                    }

                    // extra error message
                    cli_printfError("main ERROR: main loop will stop!\n");

                    // endless loop
                    while(1)
                    {
                        // don't continue
                        usleep(1);
                    }
                }
            }

            // check if in the sleep, OCV, init or the charge-relaxation state 
            if(getMainState() == SLEEP || getMainState() == OCV || getMainState() == INIT || 
                (getMainState() == CHARGE /*&& getChargeState() == RELAXATION*/))
            {
                // set the AFE mode to normal
                batManagement_setAFEMode(AFE_NORMAL);

                // set the SBC to normal mode
                if(sbc_setSbcMode(SBC_NORMAL))
                {
                    // error
                    cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                }

                // check for sleep
                if(getMainState() == SLEEP)
                {
                    // enable CC overflow on the fault pin
                    batManagement_setCCOvrFltEnable(true);

                    // go to the INIT state
                    setMainState(INIT);
                }
            }
            
            // check the BCC fault
            batManagement_checkFault(&BMSFault, 0);
            batManagement_checkFault(&BMSFault, 0);

            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime in gBCCRisingFlank!\n");
            }

            // don't output Rising edge BCC pin! when first start-up
            if(!firstTimeStartup)
            {
                // check if it is not CC overflow
                if(!(BMSFault & BMS_CC_OVERFLOW))
                {
                    // output the message for the rising edge on the BCC_FAULT pin 
                    cli_printf("Rising edge BCC pin!\n");

                    // just in case
                    int32tVal = 1;
                }
                // if it is the CC overflow and the time between the messages is long enough or the max is reached
                else if((BMSFault & BMS_CC_OVERFLOW) && 
                       (((lastMessageTime.tv_sec + CC_OVERFLOW_MESS_TIMEOUT_TIME) < currentTime.tv_sec) ||
                       (amountOfMissedMessages == UINT16_MAX) || (!outputtedFirstMessage)))
                {
                    // check if there are missed messages
                    if(amountOfMissedMessages)
                    {
                        // output the risting edge trigger and the amount of missed messages
                        cli_printf("Rising edge BCC pin! Repeated %d times\n", amountOfMissedMessages);
                    }
                    else
                    {
                        // output the risting edge trigger 
                        cli_printf("Rising edge BCC pin!\n");
                    }

                    // set the value to 1 to indicate it needs to be outputted
                    int32tVal = 1;
                    outputtedFirstMessage = true;
                }
                else
                {
                    // increase the missed messages
                    amountOfMissedMessages++;

                    // set the value to 0 to indicate it doesn't need to be outputted
                    int32tVal = 0;
                }
            }

            // // check if a fault occured
            if(BMSFault && !firstTimeStartup)
            {
                // change the status flags if needed 
                // check for temperature errors
                if(BMSFault & (BMS_UT | BMS_OT))
                {
                    // set the temperature bit
                    data_statusFlagBit(STATUS_TEMP_ERROR_BIT, 1);
                }

                // check for overload bit
                if(BMSFault & STATUS_OVERLOAD_BIT)
                {
                    // set the temperature bit
                    data_statusFlagBit(STATUS_TEMP_ERROR_BIT, 1);
                }
                
                // check if there is a sleep overcurrent
                if(BMSFault & BMS_SLEEP_OC)
                {
                    cli_printf("BMS sleep overcurrent detected!\n");

                    // wake up if sleep
                    if(getMainState() == SLEEP)
                    {
                        // go to the init state 
                        setMainState(INIT);
                    }
                }

                // check if need to react 
                if(BMSFault & (BMS_CELL_UV + BMS_CELL_OV  + BMS_SW_CELL_OV+ BMS_PCB_UV + 
                    BMS_PCB_OV + BMS_UT + BMS_OT + BMS_AVG_OVER_CURRENT + BMS_PEAK_OVER_CURRENT))
                {
                    // check for output to the user
                    if(BMSFault & BMS_AVG_OVER_CURRENT)
                    {
                        cli_printfError("Avg overcurrent detected!\n");
                    }
                    if(BMSFault & BMS_PEAK_OVER_CURRENT)
                    {
                        cli_printfError("Peak overcurrent detected!\n");
                    }
                    if(BMSFault & BMS_CELL_UV)
                    {
                        cli_printfError("Cell undervoltage detected!\n");

                        // set the variable
                        cellUnderVoltageDetected = true;
                    }
                    if(BMSFault & BMS_UT)
                    {
                        cli_printfError("BMS undertemperature detected!\n");
                    }
                    if(BMSFault & BMS_OT)
                    {
                        cli_printfError("BMS overtemperature detected!\n");
                    }

                    // check for a cell overvoltage in the charge with CB or relaxation 
                    if((((BMSFault & (BMS_SW_CELL_OV)) == BMS_SW_CELL_OV) ||
                        ((BMSFault & (BMS_CELL_OV)) == BMS_CELL_OV)) &&
                        (getMainState() == CHARGE) && ((getChargeState() == CHARGE_CB) || 
                        (getChargeState() == RELAXATION)) && (!(BMSFault & 
                        (BMS_AVG_OVER_CURRENT + BMS_PEAK_OVER_CURRENT + BMS_UT + BMS_OT))))
                    {
                        // set the relaxation state
                        setChargeState(RELAXATION);

                        // clear the fault (not an overvoltage, but end of charge voltage)
                        batManagement_checkFault(&BMSFault, true);

                        // indicate that you are going to the relaxation state
                        cli_printf("Going to relaxation state!\n");
                    }
                    else
                    {
                        // Check for a software cell overvoltage
                        if(BMSFault & BMS_SW_CELL_OV)
                        {
                            // indicate that a cell overvoltage measurement has been detected
                            cli_printfError("Cell overvoltage measurement detected!\n");
                        }
                        if(BMSFault & BMS_CELL_OV)
                        {
                            // indicate that an cell overvoltage has been detected by the BCC
                            cli_printfError("Cell overvoltage detected!\n");
                        }

                        // go to the FAULT state
                        setMainState(FAULT);

                        // check if the old state is the fault state
                        if(lvOldState == FAULT)
                        {
                            // check if the fault is a peak overcurrent fault
                            // this could happen with flight mode, but there should still
                            // be a check on the peak overcurrent
                            if(BMSFault & BMS_PEAK_OVER_CURRENT)
                            {
                                // set the old state to the self-test state to re-do the fault state
                                lvOldState = SELF_TEST;
                            }
                        }
                    }
                }

                // check if there is a CC overflow
                if(BMSFault & BMS_CC_OVERFLOW)
                {
                    // read and reset the CC registers by calculating a new remaining charge
                    lvRetValue = batManagement_calcRemaningCharge(&boolValue);

                    // check if outputting message is needed
                    if(boolValue)
                    {
                        // check if it needs to be outputted
                        if(int32tVal)
                        {
                            // check if there are missed messages
                            if(amountOfMissedMessages)
                            {
                                // output the output the message and the amount of missed messages
                                cli_printf("clearing CC overflow, repeated %d times\n", amountOfMissedMessages);
                            }
                            else
                            {
                                // output the message 
                                cli_printf("clearing CC overflow\n");
                            }

                            // reset the amount of missed messages
                            amountOfMissedMessages = 0;

                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &lastMessageTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get lastMessageTime in gBCCRisingFlank!\n");
                            }
                        }
                    }

                    // check for errors
                    if(lvRetValue)
                    {
                        // output to user
                        cli_printfError("main ERROR: could not reset CC register when overflow: %d\n", lvRetValue);
                    }
                }

                // check if there was CSB wakeup in the OCV state
                // while disregarding other faults and the CC overflow
                if((BMSFault & ~(BMS_OTHER_FAULT + BMS_CC_OVERFLOW)) == BMS_CSB_WAKEUP && 
                    getMainState() == OCV)
                {
                    cli_printf("clearing BCC pin pin due to CSB wakeup fault\n");

                    // clear the fault 
                    batManagement_checkFault(&BMSFault, true);
                }
            }

            // to make sure you only enter this once
            gBCCRisingFlank = false;

        // end of if(gBCCRisingFlank)    
        }

        // check for a buttonpress
        if(gButtonPressFlank)
        {
            if(getMainState() == SLEEP || getMainState() == NORMAL || getMainState() == CHARGE)
            {
                // get the time 
                if(clock_gettime(CLOCK_REALTIME, &buttonPressedTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get buttonPressedTime!\n");
                }

                // set the variable true
                deepsleepTimingOn = true;
            }
            
            // make sure it will only do this once
            gButtonPressFlank = false;
        }

        // check the button ISR value or the fault should be reset
        if(gButtonRisingFlank || setNGetStateCommandVariable(false, CMD_ERROR) == CMD_RESET)
        {
            // check if it is in the FAULT or SLEEP state
            if(getMainState() == FAULT || getMainState() == SLEEP || getMainState() == SELF_DISCHARGE)
            {
                // if self discharge check if the time passed 
                if(getMainState() == SELF_DISCHARGE)
                {
                    // get the current time 
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get currentTime in gButtonRisingFlank!\n");
                    }

                    // check if the right amount of time has passed 
                    if(((selfDischargeTime.tv_sec + SELF_DISCHARGE_WAIT_TIME) < currentTime.tv_sec) || 
                      (((selfDischargeTime.tv_sec + SELF_DISCHARGE_WAIT_TIME) == currentTime.tv_sec) && selfDischargeTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // go to the INIT state
                        setMainState(INIT);
                    }
                }
                else
                {
                    // go the the init state with a button press
                    setMainState(INIT);
                }
            }

            // set the variable to false to only do this once
            gButtonRisingFlank = false;

            // reset the command variable
            setNGetStateCommandVariable(true, CMD_NONE);

            // set the variable false
            deepsleepTimingOn = false;
        }

        // check if the pin is low, but the overcurrent happend
        if(getMainState() != FAULT && gpio_readPin(OVERCURRENT))
        {
            // output to the users
            cli_printfError("main ERROR: hardware overcurrent detected!\n");

            // go to the FAULT state
            setMainState(FAULT);
        }

        // get the emergency button enable value
        if(data_getParameter(EMERGENCY_BUTTON_ENABLE, &int32tVal, NULL) == NULL)
        {
            cli_printfError("main ERROR: getting emergency button enable value went wrong!\n");

            // set the default one just in case
            int32tVal = EMERGENCY_BUTTON_ENABLE_DEFAULT;
        }

        // check if the emergency button is used and pressed and it is not already in the fault state
        if((int32tVal & 1) && gpio_readPin(PTE8) && 
            getMainState() != FAULT)
        {
            // output to the users
            cli_printfError("main ERROR: emergency button pressed!\n");

            // go to the FAULT state
            setMainState(FAULT);
        }
        
        // check the state variable
        switch(getMainState())
        {
            // in case of the init state
            case INIT:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // set the LED to green 
                    ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                    // change the MCU to run mode
                    if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                    {
                        // error message 
                        cli_printfError("main ERROR: failed to put the system in RUN mode!\n");

                        // extra message
                        cli_printfError("main ERROR: main loop will stop!\n");

                        // endless loop
                        while(1)
                        {
                            // don't continue
                            usleep(1);
                        }
                    }

                    // set the AFE mode to normal
                    batManagement_setAFEMode(AFE_NORMAL);

                    // set the SBC to normal mode
                    if(sbc_setSbcMode(SBC_NORMAL))
                    {
                        // error
                        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                    }

                    // Enable the I2C
                    if(i2c_enableTransmission(true))
                    {
                        cli_printfError("main ERROR: failed to enable I2C!\n");
                    }

                    // check configuration and reset faults
                    batManagement_checkAFE(&BMSFault, true);

                    // reset the undervoltage variable
                    cellUnderVoltageDetected = false;

                    // check the fault variable
                    if(BMSFault)
                    {
                        // get the time
                        if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get sampleTime!\n");
                        }

                        // wait until the pin is low again or 1sec timeout
                        do
                        {
                            // get the time to check for an overflow
                            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get currentTime!\n");
                            }

                            // sleep for 10ms with a watchdog reset
                            usleepMainLoopWatchdog(10*1000UL);

                        } while (gpio_readPin(BCC_FAULT) && 
                            (((currentTime.tv_sec - sampleTime.tv_sec) >= 1) 
                                && (currentTime.tv_nsec >= sampleTime.tv_nsec)));

                        // check if timeout happend
                        if(gpio_readPin(BCC_FAULT))
                        {
                            // set the fault pin flank true to go to fault state
                            gBCCRisingFlank = true;
                        }

                        // start a manual measurement and wait until the measurement is done
                        lvRetValue = batManagement_doMeasurement();

                        // get the battery current and save it in the data struct
                        batManagement_getBattCurrent();

                        // wait 50ms to make sure the pin should be high again
                        usleepMainLoopWatchdog(50*1000UL);

                        // check if the interrupt happend
                        if(gBCCRisingFlank)
                        {
                            // make sure to do the whole init state next time
                            lvOldState = SELF_TEST;

                            // break to skip the rest of the init (don't close the switch)
                            break;
                        }
                    }

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode false
                    batManagement_startCharging(false);

                    // start a manual measurement and wait until the measurement is done
                    lvRetValue = batManagement_doMeasurement();

                    // check for error
                    if(lvRetValue)
                    {
                        // set the fault pin flank true to go to fault state
                        gBCCRisingFlank = true;

                        // make sure to do the whole init state next time
                        lvOldState = SELF_TEST;

                        // break to escape the rest of the init
                        break;
                    }

                    // check if the inserted n-cells is ok
                    lvRetValue = batManagement_checkNCells(&boolValue);

                    // check for errors
                    if(lvRetValue != 0)
                    {
                        // inform user
                        cli_printfError("main ERROR: Failed get n-cells ok: %d\n", lvRetValue);
                        
                        // set the fault pin flank true to go to fault state
                        gBCCRisingFlank = true;

                        // make sure to do the whole init state next time
                        lvOldState = SELF_TEST;
                        
                        // break to escape the rest of the init
                        break;
                    }

                    // check if the output is low
                    if(boolValue != 1)
                    {
                        // go to the FAULT state
                        setMainState(FAULT);

                        // make sure to do the whole init state next time
                        lvOldState = SELF_TEST;
                        
                        // break to escape the rest of the init
                        break;
                    }

                    // close the gate
                    if(batManagement_setGatePower(GATE_CLOSE) != 0)
                    {
                        cli_printfError("main ERROR: Failed to open gate\n");
                    }

                    // do another measurement
                    batManagement_doMeasurement();

                    // get the battery current and save it in the data struct
                    batManagement_getBattCurrent();

                    // wait 1ms to make sure the current is set
                    usleep(1*1000UL);

                    // turn on the measurements 
                    batManagement_updateMeasurementsOn(true);

                    // turn on the UAVCAN messages
                    if(setNGetEnableUavcanMessages(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable UAVCAN messages!\n");
                    }

                    // turn on the NFC update 
                    if(setNGetEnableNFCUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable NFC update!\n");
                    }

                    // turn on the display update
                    if(setNGetEnableDisplayUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable display update!\n");
                    }

                    // enable CC overflow on the fault pin
                    batManagement_setCCOvrFltEnable(true);

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    // reset the sleep variable
                    setTransitionVariable(SLEEP_VAR, false);

                    cli_printf("INIT mode\n");
                }

                // check for discharge
                if(!getTransitionVariable(CHAR_VAR))
                {
                    // go to the normal state
                    setMainState(NORMAL);
                }
                else
                {
                    // go to the charge state
                    setMainState(CHARGE);
                }

            break;
            case NORMAL:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // turn on the gate 
                    if(batManagement_setGatePower(GATE_CLOSE) != 0)
                    {
                        cli_printfError("main ERROR: Failed to open gate\n");
                    }

                    // get the battery current and save it in the data struct
                    batManagement_getBattCurrent();

                    // turn on the measurements if not on
                    batManagement_updateMeasurementsOn(true);

                    // reset consumed power 
                    floatVal = 0;

                    // set the consumed power to 0
                    if(data_setParameter(E_USED, &floatVal))
                    {
                        cli_printfError("main ERROR: couldn't reset consumed power!\n");
                    }

                    // get the state of charge 
                    if(data_getParameter(S_CHARGE, &int32tVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting state of charge went wrong!\n");

                       // set the default value just in case
                       int32tVal = S_CHARGE_DEFAULT;
                    } 

                    // set the value to calculate the state indication
                    ledState_calcStateIndication((uint8_t)int32tVal);

                    // set the LED to green blinking
                    ledState_setLedColor(GREEN, OFF, LED_BLINK_ON);

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode
                    batManagement_startCharging(false);

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    cli_printf("NORMAL mode\n");

                    // TODO CLI and NFC is allowed

                    // TODO enable diagnostics
                }

                // check if the button is pressed
                if(deepsleepTimingOn)
                {
                    // check if the current stays low 
                    if(!getTransitionVariable(DISCHAR_VAR))
                    {
                        // get the current time 
                        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get currentTime in sleep!\n");
                        }

                        // check if the right amount of time has passed 
                        if(((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) || 
                          (((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) && buttonPressedTime.tv_nsec <= currentTime.tv_nsec))
                        {
                            // go to the deep sleep state
                            setMainState(SELF_DISCHARGE);
                        }
                    }
                    else
                    {
                        // set the variable false
                        deepsleepTimingOn = 0;
                    }
                }

                //check for charge 
                if(getTransitionVariable(CHAR_VAR))
                {
                    // go to the charge state 
                    setMainState(CHARGE);
                }

                // check for sleep
                if(getTransitionVariable(SLEEP_VAR) || 
                    setNGetStateCommandVariable(false, CMD_ERROR) == CMD_GO_2_SLEEP)
                {
                    setMainState(SLEEP);
                }


            break;
            case CHARGE:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // turn on the gate 
                    if(batManagement_setGatePower(GATE_CLOSE) != 0)
                    {
                        cli_printfError("main ERROR: Failed to open gate\n");
                    }

                    // turn on the measurements if not on
                    batManagement_updateMeasurementsOn(true);

                    // set the charge mode
                    batManagement_startCharging(true);

                    // set the LED to blue
                    ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                    cli_printf("CHARGE mode\n");

                    // set the charge state to the first state
                    setChargeState(CHARGE_START);

                    // set the old charge state to the last state
                    oldChargeState = CHARGE_COMPLETE;

                    // reset the variable for the CB charge cycles
                    amountOfCBChargeCycles = 0;

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    // reset the charge to storage variable
                    batManagement_SetNReadChargeToStorage(true, 0);

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the variable to false
                    chargeToStorage = false;

                    // TODO CLI and NFC is allowed

                    // TODO enable diagnostics
                }

                // check if the charge state changed
                if(getChargeState() != oldChargeState)
                {
                    // check which charge state it is in
                    switch(getChargeState())
                    {
                        // in case of the beginning
                        case CHARGE_START:

                            // set the end of charge variable to 1 to make sure it 
                            // will only check for voltage
                            batManagement_SetNReadEndOfCBCharge(true, 1);

                            // start the charging timer
                            // check the time the charging begins
                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get sampleTime!\n");
                            }

                            cli_printf("Charge start %ds %dms\n", 
                                sampleTime.tv_sec, sampleTime.tv_nsec/1000000);

                        break;

                        // in case of charging with balancing
                        case CHARGE_CB:

                            // turn off the gate 
                            if(batManagement_setGatePower(GATE_CLOSE) != 0)
                            {
                                cli_printfError("main ERROR: Failed to open gate\n");
                            }

                            // enable cell balancing
                            batManagement_setBalancing(true);

                            // set the end of charge variable to 0 to check for current and voltage
                            batManagement_SetNReadEndOfCBCharge(true, 0);

                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get sampleTime! \n");
                            }

                            cli_printf("Charge with CB %ds %dms\n", 
                                sampleTime.tv_sec, sampleTime.tv_nsec/1000000);

                            // increase the counter
                            amountOfCBChargeCycles++;

                            // make sure it will only output CB done once
                            boolValue = true;

                            // get the n-charges-full value and increment it once 
                            if(data_getParameter(N_CHARGES, &int32tVal, NULL) == NULL)
                            {
                               cli_printfError("main ERROR: getting n-charges went wrong! \n");

                               // set it to the default value just in case
                               int32tVal = N_CHARGES_DEFAULT;
                            } 

                            // increament and limit it 
                            int32tVal = (int32tVal + 1) & UINT16_MAX;

                            // set the number of charger with the new value
                            if(data_setParameter(N_CHARGES, &int32tVal))
                            {
                                cli_printfError("main ERROR: couldn't set n-charges!\n");
                            }

                            //cli_printf("charge wth CB %d\n", amountOfCBChargeCycles);

                        break;

                        // in case of relaxing
                        case RELAXATION:

                            // set the power switches open
                            // turn off the gate 
                            if(batManagement_setGatePower(GATE_OPEN) != 0)
                            {
                                cli_printfError("main ERROR: Failed to open gate\n");
                            }

                            // start the relax time
                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get sampleTime! \n");
                            }

                            // make sure it doens't keep checking 
                            batManagement_SetNReadEndOfCBCharge(true, 3);

                            cli_printf("Charge RELAXATION %ds %dms\n", 
                                sampleTime.tv_sec, sampleTime.tv_nsec/1000000);

                            // make sure it will only output CB done once
                            boolValue = true;

                            // make sure it will output the end time once
                            onlyOnce = true;

                            // turn off the NFC update and enter outdated message
                            if(setNGetEnableNFCUpdates(true, false) < 0)
                            {
                                // output error
                                cli_printfError("main ERROR: Could not disable NFC update!\n");
                            }

                            // turn off the display update
                            if(setNGetEnableDisplayUpdates(true, false) < 0)
                            {
                                // output error
                                cli_printfError("main ERROR: Could not disable display update!\n");
                            }

                            // set the SMBus current to 0
                            SMBus_updateInformation(true);

                            cli_printf("Putting BCC to measure at 10Hz\n");

                            // set the AFE mode to slow (since gate is open anyway)
                            batManagement_setAFEMode(AFE_SLOW);

                            cli_printf("Putting MCU in VLPR mode and turning CAN (5V) off\n");

                            // set the SBC to standby mode
                            if(sbc_setSbcMode(SBC_STANDBY))
                            {
                                // error
                                cli_printfError("main ERROR: failed to set SBC to standby mode!\n");
                            }

                            // set the watchdog in slow mode
                            sbc_setWatchdogMode(WD_TIMEOUT, SLOW_WATCHDOG);

                            // sleep for a small amount of time to output the things on the CLI
                            usleepMainLoopWatchdog(1);

                            // change the MCU to standby mode (VLPR+)
                            if(power_setNGetMcuPowerMode(true, STANDBY_MODE) == ERROR_VALUE)
                            {
                                cli_printfError("main ERROR: failed to put the system in STANDBY mode!\n");
                                cli_printfWarning("main ERROR: putting system back to RUN mode!\n");

                                // change the MCU to RUN mode
                                if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                                {
                                    // error
                                    cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                                    cli_printfError("main ERROR: stopping main loop!\n");

                                    // loop endlessly
                                    while(1)
                                    {
                                        usleep(1);
                                    }
                                }
                            }

                            //cli_printf("charge relaxing %ds %dms\n", sampleTime.tv_sec, sampleTime.tv_nsec/1000000);

                        break;

                        // in case the charging is complete
                        case CHARGE_COMPLETE:

                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get sampleTime! \n");
                            }

                            // set the LED to green 
                            ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                            // turn off cell balancing function
                            batManagement_setBalancing(false);

                            // make sure it doens't keep checking 
                            batManagement_SetNReadEndOfCBCharge(true, 3);

                            cli_printf("Charge complete %ds %dms\n", 
                                sampleTime.tv_sec, sampleTime.tv_nsec/1000000);

                            // check if charging to storage is not on
                            // Only save the full-charge capacity and increment 
                            // The number of full charges if it is a full charge
                            if(!chargeToStorage)
                            {
                                // get the n-charges-full value and increment it once 
                                if(data_getParameter(N_CHARGES_FULL, &int32tVal, NULL) == NULL)
                                {
                                   cli_printfError("main ERROR: getting n-charges-full went wrong! \n");
                                   int32tVal = N_CHARGES_FULL_DEFAULT;
                                } 

                                // increament and limit it 
                                int32tVal =  (int32tVal + 1) & UINT16_MAX;

                                // set the incremented one
                                if(data_setParameter(N_CHARGES_FULL, &int32tVal))
                                {
                                    cli_printfError("main ERROR: couldn't set n-charges-full!\n");
                                }

                                // calibrate the a-full based on the state of charge
                                if(batManagement_calibrateStateOfCharge(false))
                                {
                                    // output error
                                    cli_printfError("main ERROR: failed to calibrate state of charge!\n");
                                }
                            }

                            // set the power switches open
                            // turn off the gate 
                            if(batManagement_setGatePower(GATE_OPEN) != 0)
                            {
                                cli_printfError("main ERROR: Failed to open gate\n");
                            }

                        break;
                    }

                    // save the state, so it wont be entered again
                    oldChargeState = getChargeState();
                }

                // check for charge state transistions              
                switch(getChargeState())
                {
                    // in case of the beginning
                    case CHARGE_START:

                        // if the cell is full the state needs to change 
                        if((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) == 2)
                        {
                            // make sure it doens't keep checking 
                            batManagement_SetNReadEndOfCBCharge(true, 3);

                            // set the next charge state
                            setChargeState(CHARGE_CB);
                        }

                        // get the CB begin time
                        if(data_getParameter(T_CB_DELAY, &int32tVal, NULL) == NULL)
                        {
                           cli_printfError("main ERROR: getting CB delay went wrong! \n");
                           int32tVal = T_CB_DELAY_DEFAULT;
                        } 

                        // make sure it is uint8
                        int32tVal &= UINT8_MAX;

                        // start the charging timer
                        // check the current time
                        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get currentTime! \n");
                        }

                        // check if the charge time ended and the charge is begon
                        if((((sampleTime.tv_sec + int32tVal) == currentTime.tv_sec) && 
                          (sampleTime.tv_nsec <= currentTime.tv_nsec)) ||
                          ((sampleTime.tv_sec + int32tVal) < currentTime.tv_sec))
                        {
                            //cli_printf("ended charge start %ds %dms\n", currentTime.tv_sec, currentTime.tv_nsec/1000000);
                            // set the next charge state
                            setChargeState(CHARGE_CB);
                        }

                    break;

                    // in case of charging with balancing
                    case CHARGE_CB:

                        // check current and cell voltages
                        if(batManagement_SetNReadEndOfCBCharge(false, 0))
                        {
                            // check if it is done due to the voltage requirement
                            if(batManagement_SetNReadEndOfCBCharge(false, 0) == 2)
                            {
                                // output all the voltages 
                                batManagement_outputCellVoltages();
                            }

                            // make sure it doens't keep checking 
                            batManagement_SetNReadEndOfCBCharge(true, 3);

                            // go to the relaxation state
                            setChargeState(RELAXATION);
                        }

                        // check if cell balancing is done to change the led color
                        if(!batManagement_checkBalancing())
                        {
                            // set the LED to blue
                            ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                            // check if the output has been done
                            if(boolValue)
                            {
                                // output to the user
                                cli_printf("CB is done\n");

                                // make sure it will only do this once 
                                boolValue = false;
                            }
                        }

                    break;

                    // in case of relaxing
                    case RELAXATION:

                        // check if the charger is removed
                        if(!batManagement_checkOutputVoltageActive())
                        {
                            // go to the sleep state by setting the transion variable true
                            setTransitionVariable(SLEEP_VAR, true);
                
                            // escape the transition switch
                            break;
                        }

                        // check for CB is done
                        if(!batManagement_checkBalancing())
                        {
                            // set the LED to blue
                            ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                            // check if the output has been done
                            if(boolValue)
                            {
                                // output to the user
                                cli_printf("CB is done\n");

                                // make sure it will only do this once 
                                boolValue = false;
                            }
                        }

                        // check for timeout 
                        // get the relax time
                        if(data_getParameter(T_CHARGE_RELAX, &int32tVal, NULL) == NULL)
                        {
                           cli_printfError("main ERROR: getting relax time went wrong! \n");
                           int32tVal = T_CHARGE_RELAX_DEFAULT;
                        } 

                        // make sure it is uint16
                        int32tVal &= UINT16_MAX;

                        // check the current time
                        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get currentTime! \n");
                        }

                        // check if the charge time ended and the charge is begon
                        if((((sampleTime.tv_sec + int32tVal) == currentTime.tv_sec) && 
                          (sampleTime.tv_nsec <= currentTime.tv_nsec)) ||
                          ((sampleTime.tv_sec + int32tVal) < currentTime.tv_sec))
                        {
                            // check for CB is done
                            if(!batManagement_checkBalancing())
                            {
                                // check if onlyOnce is still true
                                if(onlyOnce)
                                {
                                    onlyOnce = false;
                                    cli_printf("Ended relaxing! %ds %dms\n", 
                                        currentTime.tv_sec, currentTime.tv_nsec/1000000);
                                }

                                // get the cell margin in mv
                                if(data_getParameter(V_CELL_MARGIN, &int32tVal, NULL) == NULL)
                                {
                                   cli_printfError("main ERROR: getting cell margin went wrong! \n");
                                   int32tVal = V_CELL_MARGIN_DEFAULT;
                                } 

                                // make sure it is uint8
                                int32tVal &= UINT8_MAX;

                                // check if charge to storage is not enabled
                                if(!chargeToStorage)
                                {
                                    // get the cell over voltage
                                    if(data_getParameter(V_CELL_OV, &floatVal, NULL) == NULL)
                                    {
                                       cli_printfError("main ERROR: getting cell over voltage went wrong! \n");
                                       floatVal = V_CELL_OV_DEFAULT;
                                    } 
                                }
                                // if it should charge to storage voltage
                                else
                                {
                                    // get the storage voltage
                                    if(data_getParameter(V_STORAGE, &floatVal, NULL) == NULL)
                                    {
                                       cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                                       floatVal = V_STORAGE_DEFAULT;
                                    } 
                                }
                               
                                // check check if highest cell voltage is smaller than the to charge voltage - margin
                                if((amountOfCBChargeCycles < AMOUNT_CB_CHARGE_CYCLES_MAX) && 
                                  (batManagement_getHighestCellV() < 
                                  (floatVal - ((float)(int32tVal)/1000))))
                                {
                                    // output the equation to the user why it did go back
                                    cli_printf("Continue charging: highest cell: %.3f < %.3f\n", 
                                        batManagement_getHighestCellV(),
                                        (floatVal - ((float)(int32tVal)/1000)));

                                    // kick the watchdog before the task yield
                                    if(sbc_kickTheWatchdog())
                                    {
                                        cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                                    }

                                    // reset the intvalue 
                                    int32tVal = 0;

                                    // get the status of the sequence tasks
                                    lvRetValue = checkSequenceTaskStatus(&boolValue);

                                    // make sure it finished stuff
                                    while(!lvRetValue && boolValue && int32tVal < 100)
                                    {
                                        // sleep for a little while to make sure the other tasks can do their things
                                        usleep(1*1000UL);

                                        // get the status of the sequence tasks
                                        lvRetValue = checkSequenceTaskStatus(&boolValue);

                                        // increase the intvalue
                                        int32tVal++;
                                    }

                                    cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                                    // change the MCU to RUN mode
                                    if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                                    {
                                        // error
                                        cli_printfError("main ERROR: failed to put the system in NORMAL mode!\n");
                                        cli_printfError("main ERROR: stopping main loop!\n");

                                        // loop endlessly
                                        while(1)
                                        {
                                            usleep(1);
                                        }
                                    }

                                    // set the SBC to normal mode 
                                    // and watchdog back to fast mode
                                    if(sbc_setSbcMode(SBC_NORMAL))
                                    {
                                        // error
                                        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                                    }

                                    // set the AFE mode back to normal (since gate will be closed)
                                    batManagement_setAFEMode(AFE_NORMAL);

                                    // turn on the NFC update 
                                    if(setNGetEnableNFCUpdates(true, true) < 0)
                                    {
                                        // output error
                                        cli_printfError("main ERROR: Could not enable NFC update!\n");
                                    }

                                    // turn on the display update
                                    if(setNGetEnableDisplayUpdates(true, true) < 0)
                                    {
                                        // output error
                                        cli_printfError("main ERROR: Could not enable display update!\n");
                                    }

                                    // go back to charge with CB
                                    setChargeState(CHARGE_CB);
                                } 
                                else
                                {
                                    // check if 5 times has passed and that is why it transitioned
                                    if(amountOfCBChargeCycles >= AMOUNT_CB_CHARGE_CYCLES_MAX)
                                    {
                                        // send the message to the user 
                                        cli_printf("%d charge cycles done, skipping voltage requirement! highest cell: %.3fV\n", 
                                            amountOfCBChargeCycles, batManagement_getHighestCellV());
                                    }

                                    // kick the watchdog before the task yield
                                    if(sbc_kickTheWatchdog())
                                    {
                                        cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                                    }

                                    // reset the intvalue 
                                    int32tVal = 0;

                                    // get the status of the sequence tasks
                                    lvRetValue = checkSequenceTaskStatus(&boolValue);

                                    // make sure it finished stuff
                                    while(!lvRetValue && boolValue && int32tVal < 100)
                                    {
                                        // sleep for a little while to make sure the other tasks can do their things
                                        usleep(1*1000UL);

                                        // get the status of the sequence tasks
                                        lvRetValue = checkSequenceTaskStatus(&boolValue);

                                        // increase the intvalue
                                        int32tVal++;
                                    }

                                    cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                                    // change the MCU to RUN mode
                                    if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                                    {
                                        // error
                                        cli_printfError("main ERROR: failed to put the system in NORMAL mode!\n");
                                        cli_printfError("main ERROR: stopping main loop!\n");

                                        // loop endlessly
                                        while(1)
                                        {
                                            usleep(1);
                                        }
                                    }

                                    // set the SBC to normal mode
                                    if(sbc_setSbcMode(SBC_NORMAL))
                                    {
                                        // error
                                        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                                    }

                                    // set the AFE mode back to normal (since gate will be closed)
                                    batManagement_setAFEMode(AFE_NORMAL);

                                    // turn on the NFC update again
                                    if(setNGetEnableNFCUpdates(true, true) < 0)
                                    {
                                        // output error
                                        cli_printfError("main ERROR: Could not enable NFC update!\n");
                                    }

                                    // turn on the display update
                                    if(setNGetEnableDisplayUpdates(true, true) < 0)
                                    {
                                        // output error
                                        cli_printfError("main ERROR: Could not enable display update!\n");
                                    }

                                    // go to charging complete
                                    setChargeState(CHARGE_COMPLETE);
                                }
                            }
                            // if cell balancing is not done
                            else
                            {
                                // only re-calculate the 
                                if(onlyOnce)
                                {
                                    // reset the variable to only do this once
                                    onlyOnce = false;

                                    // end of relaxation time output
                                    cli_printf("End of relax time, re-estimating balance minutes\n");

                                    // enable cell balancing again
                                    batManagement_setBalancing(true);

                                    // sleep for a small amount to make sure it started
                                    usleepMainLoopWatchdog(1000);
                                }
                            }
                        }

                    break;

                    // in case the charging is complete
                    case CHARGE_COMPLETE:

                        // check if the charger is removed
                        if(!batManagement_checkOutputVoltageActive())
                        {
                            // go to the sleep state by setting the transion variable true
                            setTransitionVariable(SLEEP_VAR, true);
                        }

                        // check if charging to storage is not on
                        if(!chargeToStorage)
                        {
                            // get the v-recharge margin
                            if(data_getParameter(V_RECHARGE_MARGIN, &int32tVal, NULL) == NULL)
                            {
                               cli_printfError("main ERROR: getting v-recharge-margin went wrong!\n");
                               int32tVal = V_RECHARGE_MARGIN_DEFAULT;
                            }

                            // limit the value
                            int32tVal &= INT16_MAX;

                            // get the v-cell-ov
                            if(data_getParameter(V_CELL_OV, &floatVal, NULL) == NULL)
                            {
                               cli_printfError("main ERROR: getting v-cell-ov went wrong! \n");
                               floatVal = V_CELL_OV_DEFAULT;
                            } 

                            // check if the lowest cell voltage is 
                            // then the cell overvoltage minus this margin
                            if(batManagement_getLowestCellV() < 
                                (floatVal - ((float)int32tVal/1000.0)))
                            {
                                // start charging again
                                cli_printf("Recharging: lowest cell: %.3fv < v-cell-ov - (v-recharge-margin: %dmV)\n",
                                    batManagement_getLowestCellV(), int32tVal);

                                // set the charge state to the charge with CB state
                                setChargeState(CHARGE_CB);

                                // reset the variable for the CB charge cycles
                                amountOfCBChargeCycles = 0;
                            }
                        }
                        // if charging to storage is on
                        else
                        {
                            // go to the self discharge state
                            setMainState(SELF_DISCHARGE);
                        }

                    break;
                }

                // check for discharge
                if(getTransitionVariable(DISCHAR_VAR))
                {
                    // go to the normal state
                    setMainState(NORMAL);
                }
                else if(getTransitionVariable(SLEEP_VAR))
                {
                    // go to the sleep state
                    setMainState(SLEEP);
                }

                // check for deep_sleep
                if((!chargeToStorage) && (setNGetStateCommandVariable(false, CMD_ERROR) == CMD_GO_2_DEEPSLEEP))
                {
                    // get the storage voltage
                    if(data_getParameter(V_STORAGE, &floatVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                       floatVal = V_STORAGE_DEFAULT;
                    } 

                    // check if the lowest cell voltage is higher or equal than the storage voltage
                    if(batManagement_getLowestCellV() >= floatVal)
                    {
                        // go to the self_discharge state
                        setMainState(SELF_DISCHARGE);
                    }
                    else
                    {
                        // set the charge to storage variable
                        batManagement_SetNReadChargeToStorage(true, 1);

                        // set the variable to charge to the storage voltage
                        chargeToStorage = true;

                        cli_printf("Charging until storage voltage\n");
                    }
                }
                // check if not already on and and if the button is pressed by the user
                else if((!chargeToStorage) && (deepsleepTimingOn))
                {
                    // get the current time 
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
                    }

                    // check if the right amount of time has passed 
                    if(((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) || 
                        (((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) && 
                        buttonPressedTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // get the storage voltage
                        if(data_getParameter(V_STORAGE, &floatVal, NULL) == NULL)
                        {
                           cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                           floatVal = V_STORAGE_DEFAULT;
                        } 
                    
                        // check if the lowest cell voltage is higher or equal than the storage voltage
                        if(batManagement_getLowestCellV() >= floatVal)
                        {
                            // go to the self_discharge state
                            setMainState(SELF_DISCHARGE);
                        }
                        else
                        {
                            // set the charge to storage variable
                            batManagement_SetNReadChargeToStorage(true, 1);

                            // set the variable to charge to the storage voltage
                            chargeToStorage = true;

                            cli_printf("Charging until storage voltage\n");
                        }
                    }
                }

                // check if the state changes
                if(getMainState() != CHARGE)
                {
                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode
                    batManagement_startCharging(false);

                    // make sure it doens't keep checking 
                    batManagement_SetNReadEndOfCBCharge(true, 3);

                    // reset the charge to storage variable
                    batManagement_SetNReadChargeToStorage(true, 0);

                    // check the pm state and check for errors
                    mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                    if(mcuPowerMode == ERROR_VALUE)
                    {
                        // get the error 
                        lvRetValue = errno; 

                        // error
                        cli_printfError("main ERROR: Could not get MCU power mode 2: %d\n", lvRetValue);
                    }

                    // check if the power mode is not NORMAL mode
                    if((mcuPowerMode == STANDBY_MODE) ||
                        (mcuPowerMode == VLPR_MODE) ||
                        (mcuPowerMode == ERROR_VALUE))
                    {
                        // kick the watchdog before the task yield
                        if(sbc_kickTheWatchdog())
                        {
                            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                        }

                        // reset the intvalue 
                        int32tVal = 0;

                        // get the status of the sequence tasks
                        lvRetValue = checkSequenceTaskStatus(&boolValue);

                        // make sure it finished stuff
                        while(!lvRetValue && boolValue && int32tVal < 100)
                        {
                            // sleep for a little while to make sure the other tasks can do their things
                            usleep(1*1000UL);

                            // get the status of the sequence tasks
                            lvRetValue = checkSequenceTaskStatus(&boolValue);

                            // increase the intvalue
                            int32tVal++;
                        }

                        cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                        // change the MCU to RUN mode
                        if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                        {
                            // error
                            cli_printfError("main ERROR: failed to put the system in NORMAL mode!\n");
                            cli_printfError("main ERROR: stopping main loop!\n");

                            // loop endlessly
                            while(1)
                            {
                                usleep(1);
                            }
                        }

                        // set the SBC to normal mode
                        if(sbc_setSbcMode(SBC_NORMAL))
                        {
                            // error
                            cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                        }

                        // set the AFE mode back to normal (since gate will be closed)
                        batManagement_setAFEMode(AFE_NORMAL);

                        // turn on the NFC update 
                        if(setNGetEnableNFCUpdates(true, true) < 0)
                        {
                            // output error
                            cli_printfError("main ERROR: Could not enable NFC update!\n");
                        }

                        // turn on the display update
                        if(setNGetEnableDisplayUpdates(true, true) < 0)
                        {
                            // output error
                            cli_printfError("main ERROR: Could not enable display update!\n");
                        }
                    }
                }

            break;
            case SLEEP:

                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // calculate and get the right OCV timer value
                    if(getOcvPeriodTime(&int32tVal, lvOldState))
                    {
                        cli_printfError("main ERROR: failed to calculate new OCV time! \n");
                    }

                    // check if the oldState is not OCV
                    // because this shouldn't be resetted when going to the OCV state
                    if(lvOldState != OCV)
                    {
                        // get the time that it first entered the sleep state
                        if(clock_gettime(CLOCK_REALTIME, &sampleTime2) == -1)
                        {
                            cli_printfError("main ERROR: failed to get sleep sampleTime! \n");
                        }
                    }

                    //cli_printf("time: %ds\n", int32tVal);

                    // save the old value
                    lvOldState = getMainState();

                    // get the time for the sleep timeout time
                    if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get sampleTime! \n");
                    }

                    // turn on the gate 
                    if(batManagement_setGatePower(GATE_CLOSE) != 0)
                    {
                        cli_printfError("main ERROR: Failed to open gate\n");
                    }

                    // turn the LED off
                    ledState_setLedColor(OFF, OFF, LED_BLINK_OFF);

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode off
                    batManagement_startCharging(false);

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    // disable CC overflow for the fault pin  
                    batManagement_setCCOvrFltEnable(false);

                    // turn off the measurements 
                    batManagement_updateMeasurementsOn(false);

                    // turn off the UAVCAN messages
                    if(setNGetEnableUavcanMessages(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not disable UAVCAN messages!\n");
                    }

                    // turn off the NFC update and enter outdated message
                    if(setNGetEnableNFCUpdates(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not disable NFC update!\n");
                    }

                    // turn on the display update
                    if(setNGetEnableDisplayUpdates(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable display update!\n");
                    }

                    // set the SMBus current to 0
                    SMBus_updateInformation(true);

                    // check if the measurements are done
                    lvRetValue = batManagement_getMeasurementsStatus(&boolValue);

                    // reset the intvalue 
                    int32tVal = 0;

                    // wait until the measurements are done
                    // or at least 100ms have passed
                    while(!lvRetValue && boolValue && (int32tVal < 100))
                    {
                        // sleep a little bit so it can be done (1ms)
                        usleep(1*1000UL);

                        // check if the measurements are done
                        lvRetValue = batManagement_getMeasurementsStatus(&boolValue);

                        // increment the int value
                        int32tVal++;
                    }

                    // check if there was an error
                    if(lvRetValue)
                    {
                        cli_printfError("main ERROR: couldn't get measurement status, waiting for some time\n");
                        // sleep long enough so the measurements are done
                        usleepMainLoopWatchdog(7500);
                    }

                    // check if the timeout happend
                    if(int32tVal >= 100)
                    {
                        cli_printfError("main ERROR: measurement status timed out after +100ms!\n");
                    }

                    // check if the sleep current threshold is enabled 
                    if(batManagement_checkSleepCurrentTh(&boolValue))
                    {
                        cli_printfError("Getting sleep current th mask went wrong!\n");
                    }

                    // check if not enabled
                    if(!boolValue)
                    {
                        cli_printfError("Sleep Current th disabled!\n");
                    }

                    // reset the intvalue 
                    int32tVal = 0;

                    // get the status of the sequence tasks
                    lvRetValue = checkSequenceTaskStatus(&boolValue);

                    // make sure it finished stuff
                    while(!lvRetValue && boolValue && int32tVal < 100)
                    {
                        // sleep for a little while to make sure the other tasks can do their things
                        usleep(1*1000UL);

                        // get the status of the sequence tasks
                        lvRetValue = checkSequenceTaskStatus(&boolValue);

                        // increase the intvalue
                        int32tVal++;
                    }

                    // check if there was an error
                    if(lvRetValue)
                    {
                        cli_printfError("main ERROR: couldn't get measurement status, waiting for some time\n");
                        // sleep long enough so the measurements are done
                        usleepMainLoopWatchdog(7500);
                    }

                    // check if the timeout happend
                    if(int32tVal >= 100)
                    {
                        cli_printfError("main ERROR: task status timed out after +100ms!\n");
                    }

                    // BCC sleep meas mode
                    batManagement_setAFEMode(AFE_SLEEP_MEAS);

                    // Disable the I2C
                    if(i2c_enableTransmission(false))
                    {
                        cli_printfError("main ERROR: failed to disable I2C!\n");
                    }

                    //cli_printfWarning("AFE not to sleep mode\n");

                    // set the SBC to standby mode
                    if(sbc_setSbcMode(SBC_STANDBY))
                    {
                        // error
                        cli_printfError("main ERROR: failed to set SBC to standby mode!\n");
                    }

                    //cli_printfWarning("SBC not to sleep mode\n");

                    cli_printf("SLEEP mode\n");

                    // sleep for a small amount of time to output the things on the CLI
                    usleepMainLoopWatchdog(1);

                    // change the MCU to VLPR mode (VLPR with only UART console enabled)
                    if(power_setNGetMcuPowerMode(true, VLPR_MODE) == ERROR_VALUE)
                    {
                        // if error
                        cli_printfError("main ERROR: failed to put the system in SLEEP mode!\n");
                        while(1)
                        {
                            usleep(1);
                        }
                    }
                }

                // get the OCV cyclic timer time
                if(getOcvPeriodTime(&int32tVal, lvOldState))
                {
                    cli_printfError("main ERROR: failed to get OCV time!\n");
                }

                // get the current time
                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
                }

                // check for the OCV state transition
                if((currentTime.tv_sec - sampleTime.tv_sec) > int32tVal)
                {
                    //cli_printf("curr: %d, sample: %d, time: %d\n", currentTime.tv_sec, sampleTime.tv_sec, int32tVal);

                    // go to the OCV state
                    setMainState(OCV);
                }

                // get the sleep timeout variable
                if(data_getParameter(T_SLEEP_TIMEOUT, &int32tVal, NULL) == NULL)
                {
                   cli_printfError("main ERROR: getting sleep timeout went wrong!\n");
                   int32tVal = T_SLEEP_TIMEOUT_DEFAULT;
                }

                // limit the value
                int32tVal &= UINT8_MAX;

                // check if the sleep timeout shouldn't be skipped
                if(int32tVal != 0)
                {
                    // check if the timtout time has passed
                    if((sampleTime2.tv_sec + (int32tVal*60*60)) < currentTime.tv_sec) 
                    {
                        // output to the user
                        cli_printf("sleep timeout happend after %d hours, going to deepsleep %ds\n", 
                            int32tVal, currentTime.tv_sec);

                        // go to the self discharge state
                        setMainState(SELF_DISCHARGE);
                    }
                }

                // check if the NFC is active
                if(gpio_readPin(NFC_ED) == NFC_ED_PIN_ACTIVE)
                {
                    // print to the user
                    cli_printf("NFC activity detected!\n");

                    // wake up to update the measurements
                    setMainState(INIT);
                }

                // check for current
                if(!getTransitionVariable(SLEEP_VAR))
                {
                    // go to the init state
                    setMainState(INIT);
                }

                // check if the go to deep sleep command has been given
                if(setNGetStateCommandVariable(false, CMD_ERROR) == CMD_GO_2_DEEPSLEEP)
                {
                    // go to the deep sleep state
                    setMainState(SELF_DISCHARGE);
                }
                else if(setNGetStateCommandVariable(false, CMD_ERROR) == CMD_WAKE)
                {
                    // go to the init state
                    setMainState(INIT);
                }

                // check if the user is pressing the button
                if(deepsleepTimingOn)
                {
                    // get the current time 
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
                    }

                    // check if the right amount of time has passed 
                    if(((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) || 
                        (((buttonPressedTime.tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) && 
                        buttonPressedTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // go to the deep sleep state
                        setMainState(SELF_DISCHARGE);
                    }
                }

            break;
            case OCV:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // set the leds to be green blinking, wake up 
                    ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                    cli_printf("OCV mode\n");

                    // change the MCU to RUN mode 
                    if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                    {
                        cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                        while(1)
                        {
                            usleep(1);
                        }
                    }

                    // set the AFE mode to normal
                    batManagement_setAFEMode(AFE_NORMAL);

                    // Enable the I2C
                    if(i2c_enableTransmission(true))
                    {
                        cli_printfError("main ERROR: failed to enable I2C!\n");
                    }

                    // read and reset the CC registers by calculating a new remaining charge
                    lvRetValue = batManagement_calcRemaningCharge(NULL);

                    // do a blocking measurement 
                    batManagement_doMeasurement();

                    // do measurement and save it.
                    batManagement_getCellVoltages();

                    // calibrate the state of charge
                    if(batManagement_calibrateStateOfCharge(true))
                    {
                        // output error
                        cli_printfError("main ERROR: failed to calibrate state of charge!\n");
                    }

                    // break to not make the state to sleep before the fault pin has been handled
                    break;
                }

                // go to the sleep state
                setMainState(SLEEP);

            break;
            case FAULT:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // reset the variable 
                    didNotDisconnectPower = false;

                    // get the in flight status variable
                    if(data_getParameter(S_IN_FLIGHT, &int32tVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting flight mode enable went wrong!\n");
                       int32tVal = S_IN_FLIGHT_DEFAULT;
                    } 

                    // limit it
                    int32tVal &= UINT8_MAX;

                    // check if in flight (with flight-mode enabled) is enabled 
                    // to not disable the power if there is no peak overcurrent
                    if(int32tVal)
                    {
                        // check if there are faults
                        batManagement_checkFault(&BMSFault, 0);

                        // check if there is a peak over current  
                        if(BMSFault & BMS_PEAK_OVER_CURRENT)
                        {
                            // turn off the gate 
                            if(batManagement_setGatePower(GATE_OPEN) != 0)
                            {
                                cli_printfError("main ERROR: Failed to close gate\n");

                                // set the LED to red
                                ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

                                // state that the power is not disconnected
                                didNotDisconnectPower = true;
                            }
                            else
                            {
                                // set the LED to red blinking
                                ledState_setLedColor(RED, OFF, LED_BLINK_ON);

                                cli_printf("Disconnecting power: flight mode enabled but peak overcurrent\n");
                            }
                        }
                        // there is no peak overcurrent fault
                        else
                        {
                            // state that the power is not disconnected
                            didNotDisconnectPower = true;

                            // get the s-out parameter
                            if(data_getParameter(S_OUT, &int32tVal, NULL) == NULL)
                            {
                               cli_printfError("main ERROR: getting flight mode enable went wrong!\n");
                               int32tVal = S_OUT_DEFAULT;
                            } 

                            // limit it
                            int32tVal &= UINT8_MAX;

                            // check if the output power is enabled
                            if(int32tVal)
                            {
                                // output warning to the user
                                cli_printfWarning("WARNING: Couldn't disconnect power: flight mode enabled and in flight\n");

                                // set the LED to red
                                ledState_setLedColor(RED, OFF, LED_BLINK_OFF);
                            }
                        }
                    }
                    // if the system is not in flight (or flight mode is not enabled)
                    else
                    {
                        // turn off the gate 
                        if(batManagement_setGatePower(GATE_OPEN) != 0)
                        {
                            cli_printfError("main ERROR: Failed to close gate\n");

                            // set the LED to red
                            ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

                            // state that the power is not disconnected
                            didNotDisconnectPower = true;
                        }
                        else
                        {
                            // set the LED to red blinking
                            ledState_setLedColor(RED, OFF, LED_BLINK_ON);
                        }
                    }

                    // check if the MCU is not in run mode 
                    if(power_setNGetMcuPowerMode(false, ERROR_VALUE) != RUN_MODE)
                    {
                        // change the MCU to run mode
                        if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                        {
                            // error message 
                            cli_printfError("main ERROR: failed to put the system in RUN mode!\n");

                            // extra message
                            cli_printfError("main ERROR: main loop will stop!\n");

                            // endless loop
                            while(1)
                            {
                                // don't continue
                                usleep(1);
                            }
                        }
                    }
                    
                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode
                    batManagement_startCharging(false);

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    // set the AFE mode to normal
                    batManagement_setAFEMode(AFE_NORMAL);

                    // set the SBC to normal mode
                    if(sbc_setSbcMode(SBC_NORMAL))
                    {
                        // error
                        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                    }

                    // Enable the I2C
                    if(i2c_enableTransmission(true))
                    {
                        cli_printfError("main ERROR: failed to enable I2C!\n");
                    }

                    // turn on the measurements 
                    batManagement_updateMeasurementsOn(true);

                    // turn on the UAVCAN messages
                    if(setNGetEnableUavcanMessages(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable UAVCAN messages!\n");
                    }

                    // turn on the NFC update 
                    if(setNGetEnableNFCUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable NFC update!\n");
                    }

                    // turn on the display update
                    if(setNGetEnableDisplayUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable display update!\n");
                    }

                    cli_printf("FAULT mode\n");

                    // set the bool value to output the message only once
                    boolValue = true;

                    // check if there was a cell undervoltage
                    if(cellUnderVoltageDetected)
                    {
                        // get the cellUnderVoltageTime
                        if(clock_gettime(CLOCK_REALTIME, &cellUnderVoltageTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get cellUnderVoltageTime in fault!\n");
                        }
                    }
                }

                // check if the s-in-flight changed to false and the gate is not disconnected
                if(gSInFlightChangedFalse && didNotDisconnectPower)
                {
                    // reset both variables
                    gSInFlightChangedFalse = false;
                    didNotDisconnectPower = false;

                    // turn off the gate (disconnect power)
                    if(batManagement_setGatePower(GATE_OPEN) != 0)
                    {
                        cli_printfError("main ERROR: Failed to close gate\n");

                        // set the LED to red
                        ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

                        // state that the power is not disconnected
                        didNotDisconnectPower = true;
                    }
                    else
                    {
                        // set the LED to red blinking
                        ledState_setLedColor(RED, OFF, LED_BLINK_ON);

                        cli_printf("Disconnecting power: not in flight any more!\n");
                    }
                }

                // check if there is no cell undervoltage
                if(!cellUnderVoltageDetected)
                {
                    // check if there are faults
                    batManagement_checkFault(&BMSFault, 0);

                    // check if there is an undervoltage 
                    if(BMSFault & BMS_CELL_UV)
                    {
                        // get the cellUnderVoltageTime
                        if(clock_gettime(CLOCK_REALTIME, &cellUnderVoltageTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get cellUnderVoltageTime in fault!\n");
                        }

                        // set the undervoltage variable true
                        cellUnderVoltageDetected = true;

                        // output to the user
                        cli_printfError("cell undervoltage detected!\n");
                    }
                }

                // check if a cell undervoltage occured
                if(cellUnderVoltageDetected)
                {
                    // get the fault timeout time
                    if(data_getParameter(T_FAULT_TIMEOUT, &int32tVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting sleepcurrent went wrong!\n");
                       int32tVal = T_FAULT_TIMEOUT_DEFAULT;
                    } 

                    // limit the uint16_t value
                    int32tVal &= UINT16_MAX;

                    // check if the fault timeout is not 0
                    if(int32tVal != 0)
                    {
                        // check if it needs to be outputted 
                        if(boolValue)
                        {
                            // output to the user
                            cli_printfWarning("WARNING: starting fault timer to go to deepsleep after %ds\n", int32tVal);

                            // set the boolValue false to only output this once
                            boolValue = false;
                        }

                        // get the current time 
                        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                        {
                            cli_printfError("main ERROR: failed to get currentTime in fault!\n");
                        }

                        // check if the right amount of time has passed 
                        if(((cellUnderVoltageTime.tv_sec + int32tVal) < currentTime.tv_sec) || 
                            (((cellUnderVoltageTime.tv_sec + int32tVal) == currentTime.tv_sec) && 
                            cellUnderVoltageTime.tv_nsec <= currentTime.tv_nsec))
                        {
                            // go to the INIT state
                            setMainState(DEEP_SLEEP);
                        }
                    }
                    else
                    {
                        // check if it needs to be outputted 
                        if(boolValue)
                        {
                            // output to the user
                            cli_printf("fault timer disabled, t-fault-timeout: %ds\n", int32tVal);
                            
                            // set the boolValue false to only output this once
                            boolValue = false;
                        }
                    }
                }

                // other transitions are done from the button press check part
                
            break;
            case SELF_DISCHARGE:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // check if the MCU is not in run mode 
                    if(power_setNGetMcuPowerMode(false, ERROR_VALUE) != RUN_MODE)
                    {
                        // change the MCU to run mode
                        if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                        {
                            // error message 
                            cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                        }
                    }

                    // BCC normal mode
                    batManagement_setAFEMode(AFE_NORMAL);

                    // set the SBC to normal mode 
                    if(sbc_setSbcMode(SBC_NORMAL))
                    {
                        // error
                        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
                    }

                    // check if the self discharge should be done

                    // get the self discharge enable parameter
                    if(data_getParameter(SELF_DISCHARGE_ENABLE, &int32tVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting self discharge var went wrong! \n");
                       int32tVal = SELF_DISCHARGE_ENABLE_DEFAULT;
                    } 

                    // make sure it is a bool
                    int32tVal &= 1;

                    // check if it should be done
                    if(!int32tVal)
                    {
                        // go to the deepsleep state
                        setMainState(DEEP_SLEEP);

                        break;
                    }

                    // get the self discharge start time
                    if(clock_gettime(CLOCK_REALTIME, &selfDischargeTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get selfDischargeTime! \n");
                    } 
                
                    // turn off the gate
                    if(batManagement_setGatePower(GATE_OPEN) != 0)
                    {
                        cli_printfError("main ERROR: Failed to open gate\n");
                    }

                    // Enable the I2C
                    if(i2c_enableTransmission(true))
                    {
                        cli_printfError("main ERROR: failed to enable I2C!\n");
                    }

                    // set the LED to magenta(purple) blinking
                    ledState_setLedColor(BLUE, OFF, LED_BLINK_ON);

                    // turn on the NFC update 
                    if(setNGetEnableNFCUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable NFC update!\n");
                    }

                    // turn on the display update
                    if(setNGetEnableDisplayUpdates(true, true) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not enable display update!\n");
                    }

                    // turn on the measurements if not on
                    batManagement_updateMeasurementsOn(true);

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode
                    batManagement_startCharging(false);

                    // wait until the balancing is done
                    while(batManagement_checkBalancing())
                    {
                        // kick the watchdog and sleep for 100us
                        usleepMainLoopWatchdog(100);
                    }

                    // set self discharge on
                    batManagement_selfDischarge(true);

                    //cli_printf("self discharge time: %ds %dus\n", selfDischargeTime.tv_sec, selfDischargeTime.tv_nsec/1000);

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    cli_printf("SELF_DISCHARGE mode\n");

                    // get the bms timeout variable
                    if(data_getParameter(T_BMS_TIMEOUT, &int32tVal, NULL) == NULL)
                    {
                       cli_printfError("main ERROR: getting bms timeout var went wrong! \n");
                       int32tVal = T_BMS_TIMEOUT_DEFAULT;
                    }

                    // store it in the floatvalue so it doesn't get overwritten
                    floatVal = (float)int32tVal;
                }

                // get the current time        
                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get currentTime!\n");
                }

                // check if the right amount of time has passed (precision is not needed)
                if((selfDischargeTime.tv_sec + (int)(round(floatVal))) < currentTime.tv_sec)
                {
                    // calibrate the state of charge
                    if(batManagement_calibrateStateOfCharge(true))
                    {
                        // output error
                        cli_printfError("main ERROR: failed to calibrate state of charge!\n");
                    }

                    // add one second to it for the next calibration
                    floatVal = floatVal + 1;
                }
                
                // get the self discharge enable parameter
                if(data_getParameter(SELF_DISCHARGE_ENABLE, &int32tVal, NULL) == NULL)
                {
                   cli_printfError("main ERROR: getting self discharge var went wrong! \n");
                   int32tVal = SELF_DISCHARGE_ENABLE_DEFAULT;
                } 

                // make sure it is a bool
                int32tVal &= 1;

                // check if it should be done
                if(!int32tVal)
                {
                    // go to the deepsleep state
                    setMainState(DEEP_SLEEP);
                }

                // check if cell balancing is done 
                if(!batManagement_checkBalancing())
                {
                    // go to the deepsleep state
                    setMainState(DEEP_SLEEP);
                }

                // check if the go to sleep command has been given
                if(setNGetStateCommandVariable(false, CMD_ERROR) == CMD_GO_2_SLEEP)
                {
                    // go to the sleep state
                    setMainState(SLEEP);
                }
                
            break;
            case DEEP_SLEEP:
                // check if the state has changed to not do this everytime
                if(getMainState() != lvOldState)
                {
                    // save the old value
                    lvOldState = getMainState();

                    // set the LED to white
                    ledState_setLedColor(WHITE, OFF, LED_BLINK_OFF);

                    // turn off the gate 
                    if(batManagement_setGatePower(GATE_OPEN) != 0)
                    {
                        cli_printfError("main ERROR: Failed to close gate\n");
                    }

                    // check if the MCU is not in run mode 
                    if(power_setNGetMcuPowerMode(false, ERROR_VALUE) != RUN_MODE)
                    {
                        // change the MCU to run mode
                        if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                        {
                            // error message 
                            cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                        }
                    }

                    // disable cell balancing
                    batManagement_setBalancing(false);

                    // set the charge mode
                    batManagement_startCharging(false);

                    // turn off the measurements 
                    batManagement_updateMeasurementsOn(false);

                    // Enable the I2C
                    if(i2c_enableTransmission(true))
                    {
                        cli_printfError("main ERROR: failed to enable I2C!\n");
                    }

                    // turn off the NFC update and enter outdated message
                    if(setNGetEnableNFCUpdates(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not disable NFC update!\n");
                    }

                    // turn off the display update
                    if(setNGetEnableDisplayUpdates(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not disable display update!\n");
                    }

                    // set the SMBus current to 0
                    SMBus_updateInformation(true);

                    // Disable I2C
                    if(i2c_enableTransmission(false))
                    {
                        cli_printfError("main ERROR: failed to disable I2C!\n");
                    }

                    // reset the command variable
                    setNGetStateCommandVariable(true, CMD_NONE);

                    cli_printf("DEEP_SLEEP mode\n");
                }

                // break until the button is released
                if(deepsleepTimingOn)
                {
                    break;
                }

                // wait for 1s

                // kick the watchdog and sleep for 500ms (in case 1s watchdog)
                usleepMainLoopWatchdog(500*1000);

                // kick the watchdog and sleep for 500ms
                usleepMainLoopWatchdog(500*1000);

                // kick the watchdog again
                if(sbc_kickTheWatchdog())
                {
                    cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                }

                // BCC sleep 
                // set the AFE mode to normal
                cli_printf("Setting the BCC to sleep!\n");
                batManagement_setAFEMode(AFE_SLEEP);
                // wakeup with CSB from low to high

                cli_printf("Saving parameters before sleep\n");

                // save the variables
                lvRetValue = data_saveParameters();

                // turn the LED off
                ledState_setLedColor(OFF, OFF, LED_BLINK_OFF);

                //cli_printf("setting SBC to standby!\n");
                cli_printf("setting SBC to sleep!\n");

                // MCU (3.3V and 5V) off mode
                lvRetValue = sbc_setSbcMode(SBC_SLEEP);

                // go to the init state if possible (if you come here the sleep didn't work)
                setMainState(INIT);

            break;

            // if it is the SELF_TEST state
            case (SELF_TEST):

                // shouldn't come here
                cli_printfError("main ERROR: Main loop is in SLEF_TEST state\n");
                cli_printf("setting init mode\n");

                // set the init mode 
                setMainState(INIT);
            break;
        }

        // kick the watchdog
        if(sbc_kickTheWatchdog())
        {
            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        }

        // get the current time           
        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
        {
            cli_printfError("main ERROR: failed to get currentTime!\n");
        }

        // check if in charge relaxation
        if(getMainState() == CHARGE && getChargeState() == RELAXATION)
        {
            // add the 2s, for 2s wait
            currentTime.tv_sec += MAIN_LOOP_LONG_WAIT_TIME_S;
        }
        else
        { 
            // make the 100ms wait time in the current time for the normal mode
            currentTime.tv_sec += (currentTime.tv_nsec + 
                MAIN_LOOP_WAIT_TIME_MS * 1000000) / (MAX_NSEC + 1);
            currentTime.tv_nsec = (currentTime.tv_nsec + 
                MAIN_LOOP_WAIT_TIME_MS * 1000000) % (MAX_NSEC + 1);
        }

        // wait for 100ms or the semaphore is posted (with a fault)
        // the semaphore is posted to trigger this task when it needs to react on things
        sem_timedwait(&gMainLoopSem, &currentTime);

        // kick the watchdog
        if(sbc_kickTheWatchdog())
        {
            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        }

        // make sure the first time startup value is false
        firstTimeStartup = false;
    }

    return 0;
}

/*!
 * @brief function to handle a parameter change
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int handleParamChangeFunc(int argc, char *argv[])
{
    gChangedParameterTaskStarted = true;
    int lvRetValue = 0;
    static uint8_t handleChangedDataArrayIndex = 0;
    static bool timeOutTimeStarted = false;
    static bool chargeTimeStarted = false;
    static struct timespec timeOutBeginTime;
    static struct timespec chargeBeginTime;
    struct timespec currentTime;
    static uint8_t sleepCurrent = 0;
    static uint16_t timeoutTime = 0;
    static uint8_t chargeDetectTime = 0;
    void* voidPointer;
    float currentmA;
    float floatValue;//, lvFloatValue2;
    uint32_t uintValue = 0;

    // wait for the semaphore
    // semaphore is free if a data is changed
    lvRetValue = sem_wait(&gDataChangedSem);

    // post it again to make sure it does handle the change
    lvRetValue = sem_post(&gDataChangedSem);
    if (lvRetValue != 0)
    {
        cli_printfError("handleParamChangeFunc: ERROR sem_post failed\n");
    }

    // get the sleepcurrent, timeout time and charge detect time
    if(data_getParameter(I_SLEEP_OC, &sleepCurrent, NULL) == NULL)
    {
        sleepCurrent = I_SLEEP_OC_DEFAULT;
        cli_printfError("handleParamChangeFunc ERROR: getting sleep current went wrong!\n");
    }
    if(data_getParameter(T_BMS_TIMEOUT, &timeoutTime, NULL) == NULL)
    {
        timeoutTime = T_BMS_TIMEOUT_DEFAULT;
        cli_printfError("handleParamChangeFunc ERROR: getting timeoutTime went wrong!\n");
    } 
    if(data_getParameter(T_CHARGE_DETECT, &chargeDetectTime, NULL) == NULL)
    {
        chargeDetectTime = T_CHARGE_DETECT_DEFAULT;
        cli_printfError("handleParamChangeFunc ERROR: getting chargeDetectTime went wrong!\n");
    }

    // stay in this loop
    while(1)
    {
        // wait for the semaphore
        // semaphore is free if a data is changed
        lvRetValue = sem_wait(&gDataChangedSem);
        if (lvRetValue != 0)
        {
            cli_printfError("handleParamChangeFunc ERROR: sem_wait failed\n");
        }

        // check which parameter changed
        switch(gChangedParametersArr[handleChangedDataArrayIndex])
        {
            // check if the current changed
            case I_BATT:

                // get the variable
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];

                // get the current
                currentmA = *(float*)voidPointer;
                currentmA *= 1000;
                
                // check for a discharge
                if((currentmA) <= -sleepCurrent)
                {
                    //cli_printf("discharge! %fmA\n", (currentmA));
                    // reset the charge variables
                    setTransitionVariable(CHAR_VAR, false);
                    chargeTimeStarted = false;

                    // reset the sleep variable
                    setTransitionVariable(SLEEP_VAR, false);

                    // reset the timeoutstarted variable 
                    timeOutTimeStarted = false;

                    // set the discharge variable
                    setTransitionVariable(DISCHAR_VAR, true);

                }
                // check for a charge
                else if((currentmA) >= sleepCurrent)
                {
                    //cli_printf("charge! %fmA\n", (currentmA));
                    // reset the timeoutstarted variable 
                    timeOutTimeStarted = false;

                    // reset the sleep variable
                    setTransitionVariable(SLEEP_VAR, false);

                    // reset the discharge variable
                    setTransitionVariable(DISCHAR_VAR, false);

                    // check if the battery is in the normal state
                    if(getMainState() == NORMAL)
                    {
                        // check if the charge time has started
                        if(!chargeTimeStarted)
                        {
                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &chargeBeginTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get chargeBeginTime! \n");
                            }

                            //cli_printf("chargebegintime: %ds %dus\n", chargeBeginTime.tv_sec, chargeBeginTime.tv_nsec/1000);

                            // set the variable 
                            chargeTimeStarted = true;
                        }
                        else if(chargeTimeStarted && !getTransitionVariable(CHAR_VAR))
                        {
                            // check if the time passed
                            // check the current time
                            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get chargeBeginTime! \n");
                            }

                            // check if the time passed
                            if(((chargeBeginTime.tv_sec + chargeDetectTime) < currentTime.tv_sec) || 
                                (((chargeBeginTime.tv_sec + chargeDetectTime) == currentTime.tv_sec) && 
                                chargeBeginTime.tv_nsec <= currentTime.tv_nsec))
                            {
                                //cli_printf("chargeEndTime: %ds %dus\n", currentTime.tv_sec, currentTime.tv_nsec/1000);
                                // set the variable 
                                setTransitionVariable(CHAR_VAR, true);
                            }
                        }
                    }

                    // if not in normal state
                    else
                    {
                        // reset the variable 
                        chargeTimeStarted = false;
                        setTransitionVariable(CHAR_VAR, false);
                    }

                }
                // check for a sleep current if abs(i-batt) < i-sleep-oc  
                else
                {
                    // reset the charge variables
                    chargeTimeStarted = false;

                    // reset the discharge variable
                    setTransitionVariable(DISCHAR_VAR, false);

                    // check if the battery is in the normal state
                    if(getMainState() == NORMAL)
                    {
                        // check if the timeout time has started
                        if(!timeOutTimeStarted)
                        {
                            // save the time
                            if(clock_gettime(CLOCK_REALTIME, &timeOutBeginTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get timeoutBeginTime! \n");
                            }

                            //cli_printf("sleepbegintime: %ds %dus\n", timeOutBeginTime.tv_sec, timeOutBeginTime.tv_nsec/1000);

                            // set the variable 
                            timeOutTimeStarted = true;
                        }
                        else if(timeOutTimeStarted && !getTransitionVariable(SLEEP_VAR))
                        {
                            // check if the time passed
                            // check the current time
                            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                            {
                                cli_printfError("main ERROR: failed to get timeoutBeginTime! \n");
                            }

                            // check if the time passed
                            if(((timeOutBeginTime.tv_sec + timeoutTime) < currentTime.tv_sec) || 
                                (((timeOutBeginTime.tv_sec + timeoutTime) == currentTime.tv_sec) && 
                                timeOutBeginTime.tv_nsec <= currentTime.tv_nsec))
                            {

                                //cli_printf("sleepEndtime: %ds %dus\n", currentTime.tv_sec, currentTime.tv_nsec/1000);
                                // set the variable 
                                setTransitionVariable(SLEEP_VAR, true);
                            }
                        }
                    }
                    // // when in the charge mode, make sure to transition to sleep mode if the charge current is 0 or less
                    else if(getMainState() == CHARGE)
                    {
                        // check if the current is 0 or less (no charging / charger disconnected) 
                        // but only in the charge states where the gate is closed
                        if((currentmA <= 0) && 
                        ((getChargeState() == CHARGE_START) || (getChargeState() == CHARGE_CB)))
                        {
                            // check if the gate is already closed (comming from relaxation)
                            // this variable is set to 0 when the charge_CB state has entered and 
                            // after the gate is closed, if it isn't 0 is should sleep anyway.
                            if((batManagement_SetNReadEndOfCBCharge(false, 0) == 0) || 
                                (getChargeState() != CHARGE_CB))
                            {
                                // set the sleep variable to go to the sleep state if the charger 
                                /// is disconnected 
                                setTransitionVariable(SLEEP_VAR, true);
                                timeOutTimeStarted = true;
                                cli_printf("No charge current: %.3fA <= 0A\n", currentmA/1000);

                                // increase the main loop semaphore
                                if(escapeMainLoopWait())
                                {
                                    cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
                                }
                            }
                        }
                        else
                        {
                            // reset the sleep variable 
                            timeOutTimeStarted = false;
                            setTransitionVariable(SLEEP_VAR, false);
                        }
                    }
                    // make sure to reset the sleep variable if in any other state (except OCV)
                    else if(getMainState() != SLEEP && getMainState() != OCV)
                    {
                        // set the variable 
                        timeOutTimeStarted = false;
                        setTransitionVariable(SLEEP_VAR, false);
                    }
                }
                // I_BATT
            break;
            // in case of the s-in-flight
            case S_IN_FLIGHT:
                // get the parameter
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];
                uintValue = *(uint32_t*)voidPointer; 
                uintValue &= UINT8_MAX;

                // check if not set before and the parameter is 0 and the state is the FAULT state
                if(!gSInFlightChangedFalse && uintValue == 0 && (getMainState() == FAULT))
                {
                    // set the variable true to make sure to enter the fault state again
                    gSInFlightChangedFalse = true;

                    // trigger the mainloop to react on it
                    swMeasuredFaultFunction(false);
                }

            break;
                
            case T_BMS_TIMEOUT:
                // save the value 
                timeoutTime = (uint16_t)gChangedDataArr[handleChangedDataArrayIndex] & UINT16_MAX;

            break;
            case I_SLEEP_OC:
                // save the value 
                sleepCurrent = (uint8_t)gChangedDataArr[handleChangedDataArrayIndex] & UINT8_MAX;
            break;

            case T_CHARGE_DETECT:
                chargeDetectTime = (uint8_t)gChangedDataArr[handleChangedDataArrayIndex] & UINT8_MAX;
            break;

            // in case of the state of charge
            case S_CHARGE:
                // get the state of charge
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];
                uintValue = *(uint32_t*)voidPointer; 
                uintValue &= UINT8_MAX;

                // call the funtion set the led blink pattern
                ledState_calcStateIndication((uint8_t)uintValue);

            break;
            case A_FACTORY:
                // get the factory capacity
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];
                floatValue = *(float*)voidPointer;

                // get the state of health
                if(data_getParameter(S_HEALTH, &uintValue, NULL) == NULL)
                {
                   cli_printfError("main ERROR: getting state of health went wrong!\n");
                   uintValue = S_HEALTH_DEFAULT;
                   //return lvRetValue;
                }

                // check if state of health is more than 100% (undefined)
                if(uintValue > 100)
                {
                    // set it to the max value
                    uintValue = 100;
                } 

                // calculate the a-full and place it in currentmA
                currentmA = (floatValue*uintValue)/100;

                // sleep for 1us to output nsh> first
                usleep(1);

                // output to the user
                cli_printf("Setting a-full with %d%% (SoH) of a-factory(%.3f): %.3f\n", 
                    uintValue, floatValue, currentmA); 

                // set the new a-full
                if(data_setParameter(A_FULL, &currentmA))
                {
                    cli_printfError("main ERROR: couldn't set a-full!\n");
                }

                // calculate the new i-charge-full
                uintValue = (int)(floatValue*10);

                // output to the user
                cli_printf("Setting i-charge-full with 1%% of a-factory(%.3f): %d\n", 
                    floatValue, uintValue);

                // set the i-charge-full variable to 1% of it 
                if(data_setParameter(I_CHARGE_FULL, &uintValue))
                {
                    cli_printfError("main ERROR: couldn't set i-charge-full!\n");
                }           
            break;
            // if one of the subject IDs has changed
            case UAVCAN_ES_SUB_ID:
            case UAVCAN_BS_SUB_ID:
            case UAVCAN_BP_SUB_ID:

                // get the value
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];
                uintValue = *(uint32_t*)voidPointer; 
                uintValue &= UINT16_MAX;

                // check if it is more than the maximum allowed value
                if(uintValue > UAVCAN_MAX_SUB_ID && (uintValue != UAVCAN_UNSET_SUB_ID))
                {
                    // sleep to output the nsh> message first
                    usleep(1);

                    // output to the user
                    cli_printfWarning("WARNING: Entered subject id: %d > (max) %d!\n", uintValue, UAVCAN_MAX_SUB_ID);
                    cli_printf("Setting this subject ID (par: %d) to the unset value: %d\n", 
                        gChangedParametersArr[handleChangedDataArrayIndex], UAVCAN_UNSET_SUB_ID);
                    
                    // set the subject ID value to UAVCAN_UNSET_SUB_ID
                    uintValue = UAVCAN_UNSET_SUB_ID;
                    if(data_setParameter(gChangedParametersArr[handleChangedDataArrayIndex], &uintValue))
                    {
                        cli_printfError("main ERROR: couldn't set subject id (par %d)!\n", 
                            gChangedParametersArr[handleChangedDataArrayIndex]);
                    }       
                }   

            break;
            // in case of the emergency button enable variable
            case EMERGENCY_BUTTON_ENABLE:

                // get the emergency button enable value
                voidPointer = &gChangedDataArr[handleChangedDataArrayIndex];
                uintValue = *(uint32_t*)voidPointer; 
                uintValue &= UINT8_MAX;

                // sleep for a short while to output the nsh> message first
                usleep(1);

                // check if the variable is set
                if(uintValue)
                {
                    // output to the user
                    cli_printfWarning("WARNING: setting PTE8 as input pull-up!\n");

                    // set the GPIO as INPUT_PULLUP
                    lvRetValue = gpio_changePinType(PTE8, INPUT_PULL_UP);
                }
                // if the variable is not set
                else
                {
                    // output to the user
                    cli_printfWarning("WARNING: removing pull-up on PTE8!\n");

                    // set the GPIO as INPUT
                    lvRetValue = gpio_changePinType(PTE8, INPUT_INTERRUPT);
                }

                // check for errors
                if(lvRetValue)
                {
                    cli_printfError("main ERROR: failed to change emergency GPIO!\n");

                    // change back the emergency button enable variable to what it was
                    uintValue = (!uintValue) & 1;

                    // output to the user that you change it back.
                    cli_printfWarning("WARNING: changing emergency button enable to %d!\n",
                        uintValue);

                    // change the variable
                    if(data_setParameter(gChangedParametersArr[handleChangedDataArrayIndex], 
                        &uintValue))
                    {
                        // output error
                        cli_printfError("main ERROR: couldn't set emergency button enable (par %d)!\n", 
                            gChangedParametersArr[handleChangedDataArrayIndex]);
                    }       
                }

            break;
            default:
            break;
        }

        // handle the change in the batManagmenet part
        if(batManagement_changedParameter(gChangedParametersArr[handleChangedDataArrayIndex], 
            (void*)&gChangedDataArr[handleChangedDataArrayIndex]))
        {
            cli_printfError("main ERROR: batManagement_changedParameter went wrong! with %d\n", 
                gChangedParametersArr[handleChangedDataArrayIndex]);
        }

        // set the array to none to indicate it was handled
        gChangedParametersArr[handleChangedDataArrayIndex] = NONE;

        // increase the arrayIndex
        handleChangedDataArrayIndex = (handleChangedDataArrayIndex + 1) % CHANGED_DATA_ARRAY_ELEMENTS;
    }

    // never come here
    return lvRetValue;
}

/*!
 * @brief function for a task to handle the update of the NFC, SMBus and display
 * 
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int updaterTaskFunc(int argc, char *argv[])
{
    int error; 
    uint8_t uint8Val; 

    // it is initialized
    gUpdaterInitialized = true;

    // loop endlessly 
    while(1)
    {
        // wait until the semaphore is available
        sem_wait(&gUpdaterSem);

#ifndef DONT_DO_UAVCAN
        // check if the message needs to be send
        if(setNGetEnableUavcanMessages(false, 0))
        {
            // send data over UAVCAN
            error = uavcan_sendBMSStatus();

            // check error
            if(error)
            {
                // output to user
                cli_printf("updater ERROR: Coulnd't send UAVCAN!\n");
            }
        }
#endif
        // check if SMBus needs to be done
        // get the SMBus enable value 
        if(data_getParameter(SMBUS_ENABLE, &uint8Val, NULL) == NULL)
        {
           cli_printfError("updater ERROR: getting SMBus enable went wrong!\n");
           uint8Val = SMBUS_ENABLE_DEFAULT;
           //return lvRetValue;
        } 

        // limit
        uint8Val &= 1;

        // check if it needs to be updated 
        if(uint8Val)
        {
            // update the SMBUs data 
            error = SMBus_updateInformation(false);

            // check error
            if(error)
            {
                // output to user
                cli_printf("updater ERROR: Coulnd't update SMBus!\n");
            }

            // sleep for a little while
            usleep(1);
        }

        // check if the NFC needs to be updated
        if(setNGetEnableNFCUpdates(false, 0))
        {
            // update the NFC and check for errors
            if(nfc_updateBMSStatus(false, false))
            {
                // output to the user
                cli_printfError("updater ERROR: Can't update NFC!\n");
            }
        }

        // TODO make CLI lower prio

        // Update the CLI data
        if(cli_updateData())
        {
            // output to the user
            cli_printfError("updater ERROR: Can't update CLI!\n");
        }

        // TODO add display on even lower priority

        // check if the display needs to be updated
        if(setNGetEnableDisplayUpdates(false, 0) &&
            !(getMainState() == CHARGE && getChargeState() == RELAXATION))
        {
            // Update the display data
            if(display_updateValues())
            {
                // output to the user
                cli_printfError("updater ERROR: Can't update display, uninitializing!\n");

                // uninitialize the display
                display_uninitialize();
            }
        }
    }

    // for compiler
    return -1;
}

/*!
 * @brief function to handle a parameter change, will set semaphore available
 * 
 * @param changedParameter the parameter that changed
 * @param newValue the new value that is set
 */
int parameterChangeFunction(parameterKind_t changedParameter, void *newValue)
{
    int lvRetValue = 0;
    uint16_t i = 0;
    static uint8_t changedDataArrayIndex = 0;
    static parameterKind_t timeoutParameter = NONE;
    static void *timoutValue = NULL;
    bool savingParameter = false;
    bool currentParameterSet = false;
    static bool parameterSaved = false;
    bool outputToUserDone = false;

    // loop until currentParameterSet or savingParameter 
    do
    {
        // wait until the array is writable
        while(gChangedParametersArr[changedDataArrayIndex] != NONE && i < 1000)
        {
            // check if the output has not been done yet
            if(!outputToUserDone)
            {
                // make sure it only happens one time
                outputToUserDone = true;

                // output status to the user
                cli_printf("please increase CHANGED_DATA_ARRAY_ELEMENTS with 1!\n");
            }
            // sleep for 1us to give the changed data handler time to handle its change
            usleep(1);

            // increase the index for the time out
            i++;
        }

        // check for timeout
        if(i >= 1000)
        {
            cli_printf("Timeout happend!\n");
            timeoutParameter = changedParameter;
            timoutValue = newValue;
            savingParameter = true;
        }
        else
        {
            // check if the saved parameter needs to be set
            if(parameterSaved)
            {
                // set the new changed data parameter
                gChangedParametersArr[changedDataArrayIndex] = timeoutParameter;
                gChangedDataArr[changedDataArrayIndex] = *(uint32_t*)timoutValue;

                // remember that the parameter is set
                parameterSaved = false;
            }
            else
            {
                // set the new changed data parameter
                gChangedParametersArr[changedDataArrayIndex] = changedParameter;
                gChangedDataArr[changedDataArrayIndex] = *(uint32_t*)newValue;

                // to go out of the while loop
                currentParameterSet = true;
            }

            // increase the semaphore so the task will handle the change
            lvRetValue = sem_post(&gDataChangedSem);
            if (lvRetValue != 0)
            {
                cli_printfError("parameterChangeFunction: ERROR sem_post failed\n");
            }

            // increase the arrayIndex
            changedDataArrayIndex = (changedDataArrayIndex + 1) % CHANGED_DATA_ARRAY_ELEMENTS;
        }
        //cli_printf("in array index: %d\n", changedDataArrayIndex);
    }while(!currentParameterSet && !savingParameter);

    // set the saved parameter to true if needed
    if(savingParameter)
    {
        parameterSaved = true;
    }

    // return
    return lvRetValue;
}

/*!
 * @brief function that will will be called if there is an overcurrent fault  
 * 
 * @param triggerFault boolean if a fault should be triggered.
 */
void swMeasuredFaultFunction(bool triggerFault)
{
    // check if the fault should be triggered
    if(triggerFault)
    {
        // set the variable high
        gBCCRisingFlank = true;
    }

    // increase the main loop semaphore
    if(escapeMainLoopWait())
    {
        cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
    }
}

/*!
 * @brief function that will will be called to change the LED
 * 
 * @param newColor the new LED color
 * @param newAltColor the new alternating color
 * @param blinkPeriodms false leds will not blink
 */
void changeLedColor(LEDColor_t newColor, LEDColor_t newAltColor, uint16_t blinkPeriodms)
{
    // set the new LED color
    ledState_setLedColor(newColor, newAltColor, blinkPeriodms);
}

/*!
 * @brief function to be called when new data is set
 * 
 */
void newMeasurementsFunction(void)
{
    int error = 0, semValue;

     // check if initialized
    if(!gUpdaterInitialized)
    {
        // output to the user
        cli_printfError("new measurement ERROR: updater not initialized!\n");
    }
    else
    {
        // check if semaphore needs to be posted
        sem_getvalue(&gUpdaterSem, &semValue);

        // check posting the semaphore is needed 
        if(semValue < 1)
        {
            // increase the semaphore
            error = sem_post(&gUpdaterSem);

            // check for errors
            if(error)
            {
              cli_printfError("new measurement ERROR: couldn't post sem! %d\n", error);
            }
        }
    }
}

/*!
 * @brief Function that will be called with an interrupt occurs on a GPIO pin.
 *        Handling of the pin ISR should be done in this function
 * 
 * @param signo the signal number that triggers this ISR
 * @param siginfo signal information from the siginfo_t struct
 * @param context the context pointer
 */
void gpioIsrFunction(int signo, siginfo_t *siginfo, void *context)
{
    int pinNumber = (*siginfo).si_value.sival_int;
    //int intValue;
    //cli_printf("GPIO ISR: sig: %d, pin: %d value %d\n", signo, pinNumber, gpio_readPin(pinNumber);

    //cli_printf("ISR pin: %d\n", pinNumber);

    // check which pin it is
    switch(pinNumber)
    {
        // in case of the BCC fault pin
        case BCC_FAULT:

            // check if the variable is not high
            if(!gBCCRisingFlank)
            {
                // check if the pin is high
                if(gpio_readPin(pinNumber))
                {
                    // set the variable high to react on it
                    gBCCRisingFlank = true;

                    // up the main loop semaphore
                    if(escapeMainLoopWait())
                    {
                        cli_printfError("gpioIsrFunction ERROR: Couldn't up mainloop sem!\n");
                    }
                }
            }

        break;
        // in case of the button pin 
        case SBC_WAKE:

            // check which transaction
            if(gpio_readPin(pinNumber))
            {
                //cli_printf("Button ISR!\n");
                // set the variable high (release)
                gButtonRisingFlank = true;
            }
            else
            {
                // set the variable high (press)
                gButtonPressFlank = true;
            }

            // up the main loop semaphore
            if(escapeMainLoopWait())
            {
                cli_printfError("gpioIsrFunction ERROR: Couldn't up mainloop sem!\n");
            }
        break;
        // in case of any other pins
        default:
            // output to the user that this pin ISR isn't implemented
            cli_printfError("gpioIsrFunction ERROR: This pin's (%d) ISR isn't implemented!\n", 
                pinNumber);
        break;
    }

}

/*!
 * @brief function that will be called when it needs to process a cli command when the CLI can't do this
 * 
 * @param command       the command that needs to be processed 
 * @param value         if 1 that CLI_SHOW command is enabled, if 0 it is disabled
 */
void processCLICommand(commands_t command, uint8_t value)
{
    int returnValue;
    states_t currentState = getMainState();
    struct timespec currentTime;
    struct timespec sampleTime;

    mcuPowerModes_t mcuPowerMode;

    // check what command it is 
    // if it is the reset command
    if(command == CLI_RESET)
    {
        // check for state
        if(currentState == FAULT)
        {
            // set the rest variable
            setNGetStateCommandVariable(true, CMD_RESET);

            // increase the main loop semaphore
            if(escapeMainLoopWait())
            {
                cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
            }
        }
        else
        {
            // output to the user
            cli_printf("Can not reset, not in FAULT mode! %s\n", gStatesArray[(int)currentState]);
        }
    }
    else if(command == CLI_SLEEP)
    {
        // check what the current state is to help the user understand why it won't change
        if(currentState == NORMAL || currentState == SELF_DISCHARGE)
        {
            // set the sleep variable
            setTransitionVariable(SLEEP_VAR, true);

            // it can transition
            setNGetStateCommandVariable(true, CMD_GO_2_SLEEP);

            // increase the main loop semaphore
            if(escapeMainLoopWait())
            {
                cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
            }
        }
        else if(currentState == SLEEP || currentState == OCV)
        {
            // inform user 
            cli_printf("Already in sleep state (or OCV)!\n");
        }
        else
        {
            // inform user
            cli_printf("Can not go to sleep state by command in this state: %s!\n", 
                gStatesArray[(int)currentState]);
        }
    }
    else if(command == CLI_WAKE)
    {
        if(currentState == SLEEP)
        {
            // transition
            setNGetStateCommandVariable(true, CMD_WAKE);

            // increase the main loop semaphore
            if(escapeMainLoopWait())
            {
                cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
            }
        }
        else
        {
            // can't wake
            cli_printf("Can not wake in this state: %s\n", gStatesArray[(int)currentState]);
        }
    }
    else if(command == CLI_DEEP_SLEEP)
    {
        if(currentState == SLEEP || currentState == CHARGE)
        {
            // transition
            setNGetStateCommandVariable(true, CMD_GO_2_DEEPSLEEP);

            // increase the main loop semaphore
            if(escapeMainLoopWait())
            {
                cli_printfError("swMeasuredFaultFunction ERROR: Couldn't up mainloop sem!\n");
            }
        }
        else
        {
            cli_printf("Can not go to deep sleep in this state: %s\n", gStatesArray[(int)currentState]);
        }
    }
    else if(command == CLI_SAVE)
    {
        // check the MCU power mode and check for errors
        mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
        if(mcuPowerMode == ERROR_VALUE)
        {
            // error
            cli_printfError("main ERROR: Could not get MCU power mode 3\n");
        }

        // check if the power mode is not NORMAL mode
        if((mcuPowerMode == STANDBY_MODE) ||
            (mcuPowerMode == VLPR_MODE) ||
            (mcuPowerMode == ERROR_VALUE))
        {
            cli_printf("Flash may not be written in VLPR mode\n");
            cli_printf("Waking up the BMS... \n");

            // sample the time
            if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
            {
                cli_printfError("processCLICommand ERROR: failed to get sampleTime!\n");
                //cli_printfError("processCLICommand ERROR: Flash may not be written in sleep mode!\n");
                //cli_printf("Wake the BMS first with \"bms wake\"\n");
            }

            // set the current time to the sample time
            currentTime.tv_sec = sampleTime.tv_sec;
            //currentTime.tv_nsec = sampleTime.tv_nsec;

            // wake up the BMS
            setNGetStateCommandVariable(true, CMD_WAKE);

            // check the MCU power mode and check for errors
            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
            if(mcuPowerMode == ERROR_VALUE)
            {
                // error
                cli_printfError("main ERROR: Could not get MCU power mode 4\n");
            }

            // wait until the BMS is not in VLPR mode 
            // or have a 2-3 second timeout
            while(((mcuPowerMode == STANDBY_MODE) ||
                (mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE)) && 
                ((sampleTime.tv_sec + 3) > currentTime.tv_sec))
            {
                // sleep so other processes can continue
                usleep(1000);

                // get the current time
                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                {
                    cli_printfError("processCLICommand ERROR: failed to get currentTime!\n");
                }

                // check the MCU power mode and check for errors
                mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
                if(mcuPowerMode == ERROR_VALUE)
                {
                    // error
                    cli_printfError("main ERROR: Could not get MCU power mode 5\n");
                }
            }

            // check if the timeout happend
            if(((sampleTime.tv_sec + 3) < currentTime.tv_sec))
            {
                cli_printfError("processCLICommand ERROR: timeout happend on saving parameters!\n");
                cli_printfError("Waking up the BMS failed!\n");
                //cli_printfWarning("Not saving parameters!");
            }
        }

        // check the MCU power mode and check for errors
        mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
        if(mcuPowerMode == ERROR_VALUE)
        {
            // error
            cli_printfError("main ERROR: Could not get MCU power mode 6\n");
        }

        // make sure the parameters may be saved
        if((mcuPowerMode != STANDBY_MODE) &&
            (mcuPowerMode != VLPR_MODE) &&
            (mcuPowerMode != ERROR_VALUE))
        {
            // save the parameters to the flash
            cli_printf("Saving parameters to flash!\n");

            // save them 
            returnValue = data_saveParameters();

            // check for error
            if(returnValue)
            {
                cli_printfError("processCLICommand ERROR: saving par went wrong!%d\n", returnValue);
            }
        }
        else
        {
            cli_printfError("processCLICommand ERROR: Flash may not be written while MCU is not in RUN mode!\n");
        }           
    }
    else if(command == CLI_LOAD)
    {
        // load the parameters from the flash
        cli_printf("Loading parameters from flash!\n");

        // save them 
        returnValue = data_loadParameters();

        // check for error
        if(returnValue)
        {
            cli_printfError("processCLICommand ERROR: saving par went wrong!%d\n", returnValue);

            // set the default values
            if(data_setDefaultParameters())
            {
                // output
                cli_printfError("processCLICommand ERROR: could not set default values!\n");
            }
            else
            {
                // save the default values
                returnValue = data_saveParameters();

                // check for error
                if(returnValue)
                {
                    cli_printfError("processCLICommand ERROR: saving par went wrong!%d\n", returnValue);
                }
            }
        }
    }
    else if(command == CLI_DEFAULT)
    {
        // set the default parameters 
        cli_printf("Setting default parameters!\n");

        // set the default values and check for error
        if(data_setDefaultParameters())
        {
            // output
            cli_printfError("processCLICommand ERROR: could not set default values!\n");
        }
    }
    else
    {
        // error
        cli_printfError("processCLICommand ERROR: wrong command: %d!\n", command);
    }
}

/*!
 * @brief function that will return the main state, but it will use the mutex 
 * 
 * @return the state
 */
states_t getMainState(void)
{
    states_t lvRetValue = INIT; 

    // lock the mutex
    pthread_mutex_lock(&gStateLock);

    // save the state
    lvRetValue = gCurrentState;

    // unlock the mutex
    pthread_mutex_unlock(&gStateLock);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief function that will set the main state, but it will use the mutex 
 * 
 * @param newState the new state
 */
static int setMainState(states_t newState)
{
    int lvRetValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gStateLock);

    // set the state 
    gCurrentState = newState;

    // unlock the mutex
    pthread_mutex_unlock(&gStateLock);

    return lvRetValue;
}

/*!
 * @brief function that will return the charge state, but it will use the mutex 
 * 
 * @return the state
 */
charge_states_t getChargeState(void)
{
    charge_states_t lvRetValue = INIT; 

    // lock the mutex
    pthread_mutex_lock(&gChargeStateLock);

    // save the state
    lvRetValue = gCurrentChargeState;

    // unlock the mutex
    pthread_mutex_unlock(&gChargeStateLock);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief function that will set the charge state, but it will use the mutex 
 * 
 * @param newState the new state
 */
static int setChargeState(charge_states_t newState)
{
    int lvRetValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gChargeStateLock);

    // set the state 
    gCurrentChargeState = newState;

    // unlock the mutex
    pthread_mutex_unlock(&gChargeStateLock);

    return lvRetValue;
}

/*!
 * @brief function that will return one of the transition variables
 * 
 * @param variable the variable to get
 *
 * @return the value of the variable
 */
static bool getTransitionVariable(transitionVars_t variable)
{
    bool lvRetValue;

    // lock the mutex
    pthread_mutex_lock(&gTransVarLock);

    // check which variable to return
    switch(variable)
    {
        case DISCHAR_VAR:
            // save the state
            lvRetValue = gDischargeDetected;
        break;
        case CHAR_VAR:
            // save the variable 
            lvRetValue = gChargeDetected;
        break;
        case SLEEP_VAR:
            // save the variable 
            lvRetValue = gSleepDetected;
        break;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gTransVarLock);

    // return to the user
    return lvRetValue;  
}

/*!
 * @brief function that will set one of the transition variables
 * 
 * @param variable the variable to set
 * @param newValue the new value to set
 *
 * @return 0 if ok
 */
static int setTransitionVariable(transitionVars_t variable, bool newValue)
{
    int lvRetValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gTransVarLock);

    // check which variable to return
    switch(variable)
    {
        case DISCHAR_VAR:
            // set the new value
            gDischargeDetected = newValue;
        break;
        case CHAR_VAR:
            // set the new value
            gChargeDetected = newValue;
        break;
        case SLEEP_VAR:
            // set the new value
            gSleepDetected = newValue;
        break;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gTransVarLock);

    // return to the user
    return lvRetValue;  
}

/*!
 * @brief   function that will set one of the transition variables
 *
 * @param   setNotGet if true it is used to set the variable, false to get the variable
 * @param   newValue if setNotGet is true, this is the new value
 *
 * @return  the state command variable, 0 if none is set, CMD_ERROR if error
 */
static stateCommands_t setNGetStateCommandVariable(bool setNotGet, stateCommands_t newValue)
{
    stateCommands_t lvRetValue = CMD_ERROR;
    static stateCommands_t stateCommand = CMD_NONE;

    // lock the mutex
    pthread_mutex_lock(&gStateCommandLock);

    // check if write or read
    // if set
    if(setNotGet)
    {
        // set the new variable
        stateCommand = newValue;
    }
    else
    // if get
    {
        // save the variable 
        lvRetValue = stateCommand;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gStateCommandLock);

    // return the value
    return lvRetValue;
}

/*!
 * @brief   function that will calculate and return the OCV period time
 * 
 * @param   newTime the address of the variable to become the OCV timer period
 * @param   oldState the oldState of the state machine
 * @warning Keep in mind that this function needs to be called before 
 *          the oldState is set with the current state (lvOldState = getMainState())
 *          when the OCV timer needs to be increased
 *
 * @return  0 if ok
 */
static int getOcvPeriodTime(int32_t *newTime, states_t oldState)
{
    int lvRetValue = -1;
    static int32_t ocvPeriod = 0;
    int32_t cyclic;

    // check for NULL pointer
    if(newTime == NULL)
    {
        // output and return
        cli_printfError("main getOcvPeriodTime ERROR: newTime is NULL pointer!\n");
        return lvRetValue;
    }

    // check if the oldstate is not the OCV state and it is the first time in the SLEEP mode
    if((oldState != OCV) && (oldState != SLEEP))
    {
        // get the starting cyclic time and place in the ocv period
        if(data_getParameter(T_OCV_CYCLIC0, &ocvPeriod, NULL) == NULL)
        {
           cli_printfError("main getOcvPeriodTime ERROR: getting t-ocv-cyclic0 went wrong!\n");
           ocvPeriod = T_OCV_CYCLIC0_DEFAULT;
        }      

        // get the OCV measurement cyclic timer stop
        if(data_getParameter(T_OCV_CYCLIC1, &cyclic, NULL) == NULL)
        {
           cli_printfError("main getOcvPeriodTime ERROR: getting t-ocv-cyclic1 went wrong!\n");
           cyclic = T_OCV_CYCLIC1_DEFAULT;
        }

        // check for an overflow
        if(ocvPeriod > cyclic)
        {
            // limit it 
            ocvPeriod = cyclic;
        }
    }
    // check if it entered SLEEP mode from OCV mode
    else if(oldState != SLEEP)
    {
        // increase the ocvPeriod with 50%
        ocvPeriod = (int32_t)((float)ocvPeriod * 1.5);

        // get the OCV measurement cyclic timer stop
        if(data_getParameter(T_OCV_CYCLIC1, &cyclic, NULL) == NULL)
        {
           cli_printfError("main getOcvPeriodTime ERROR: getting t-ocv-cyclic1 went wrong!\n");
           cyclic = T_OCV_CYCLIC1_DEFAULT;
        }

        // check for an overflow
        if(ocvPeriod > cyclic)
        {
            // limit it 
            ocvPeriod = cyclic;
        }
    }

    // set the timer period in the new time variable
    *newTime = ocvPeriod;

    // succesfull
    lvRetValue = 0;
    return lvRetValue;
}

/*!
 * @brief   function that is used to enable or disable the UAVCAN messages 
 *          or get the value if it should output the UAVCAN messages.
 * 
 * @param   setNGet if true it will set the value if the UAVCAN messages should be outputted.
 *          if false it will return 1 if it should be outputted, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the UAVCAN messages are enabled 
 *          to send after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableUavcanMessages(bool setNGet, bool enable)
{
    int lvRetValue = -1;
    static bool enableUavcanMessages = false;

    // check if mutex is not initialized
    if(!gSetUavcanMessagesInitialized)
    {
        // output error
        cli_printfError("setNGetEnableUavcanMessages ERROR: Mutex not initialized!\n");

        // return
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gSetUavcanMessagesLock);

    // check if it needs to set the value
    if(setNGet)
    {
        // set the message
        enableUavcanMessages = enable;
    }

    // make the return value
    lvRetValue = enableUavcanMessages;

    // unlock the mutex
    pthread_mutex_unlock(&gSetUavcanMessagesLock);

    // return
    return lvRetValue;
}

/*!
 * @brief   function that is used to enable or disable the NFC update 
 *          or get the value if it should update the NFC.
 * 
 * @param   setNGet if true it will set the value if the NFC should be updated.
 *          if false it will return 1 if it should be updated, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the NFC update is enabled 
 *          to update after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableNFCUpdates(bool setNGet, bool enable)
{
    int lvRetValue = -1;
    static bool enableNfcUpdate = false;

    // check if mutex is not initialized
    if(!gSetNfcUpdateLockInitialized)
    {
        // output error
        cli_printfError("setNGetEnableNFCUpdates ERROR: Mutex not initialized!\n");

        // return
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gSetNfcUpdateLock);

    // check if it needs to set the value
    if(setNGet)
    {
        // set the message
        enableNfcUpdate = enable;

        // check if the NFC is turned off 
        if(!enable)
        {
            // check which state it is in
            if(getMainState() == SLEEP)
            {
                // put the wakeup data notice in the NTAG
                nfc_updateBMSStatus(true, true);
            }
            else
            {
                // put the charge relaxation data notice in the NTAG
                nfc_updateBMSStatus(true, false);
            }
           
        }
    }

    // make the return value
    lvRetValue = enableNfcUpdate;

    // unlock the mutex
    pthread_mutex_unlock(&gSetNfcUpdateLock);

    // return
    return lvRetValue;
}

/*!
 * @brief   function that is used to enable or disable the display update 
 *          or get the value if it should update the display.
 * 
 * @param   setNGet if true it will set the value if the display should be updated.
 *          if false it will return 1 if it should be updated, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the display update is enabled 
 *          to update after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableDisplayUpdates(bool setNGet, bool enable)
{
    int lvRetValue = -1;
    static bool enableDisplayUpdate = false;

    // check if mutex is not initialized
    if(!gSetDisplayUpdateLockInitialized)
    {
        // output error
        cli_printfError("setNGetEnableDisplayUpdates ERROR: Mutex not initialized!\n");

        // return
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gSetDisplayUpdateLock);

    // check if it needs to set the value
    if(setNGet)
    {
        // set the message
        enableDisplayUpdate = enable;
    }

    // make the return value
    lvRetValue = enableDisplayUpdate;

    // unlock the mutex
    pthread_mutex_unlock(&gSetDisplayUpdateLock);

    // return
    return lvRetValue;
}

/*!
 * @brief   Function that is used to call a usleep (task switch as well)
 *          but it will kick the watchdog first. 
 * 
 * @param   usec the number of microseconds to wait.
 *
 * @return  On successful completion, usleep() returns 0. Otherwise, it returns -1
 *          and sets errno to indicate the error.
 */
static int usleepMainLoopWatchdog(useconds_t usec)
{
    int lvRetValue = 0;

    // kick the watchdog
    if(sbc_kickTheWatchdog())
    {
        cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        lvRetValue = -1;
    }

    // sleep for a little time
    usleep(usec);

    // return
    return lvRetValue;
}

/*!
 * @brief   Function that is used to check if the tasks are on
 *          Like the measurement, other calc, updater and the sdchar task
 * 
 * @param   on the address of the varible to become true if the tasks are on, 
 *          false otherwise.
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int checkSequenceTaskStatus(bool *on)
{
    int lvRetValue = -1;
    int semValue = 0;

    // check if no NULL pointer
    if(on != NULL)
    {
        // check if the measurement tasks are running
        lvRetValue = batManagement_getMeasurementsStatus(on);

        // check if not running
        if(!(*on))
        {
            // check the updater task
            // get the measurment semaphore value
            sem_getvalue(&gUpdaterSem, &semValue);

            // check if not running 
            if(semValue == -1)
            {
                // check the sdchar task
                lvRetValue = batManagement_getSDChargeStatus(on);
            }
        }
    }
   
    // return
    return lvRetValue;
}

/*!
 * @brief   Function that is used to get out of the sem_timedwait()
 * 
 * @param   none
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int escapeMainLoopWait(void)
{
    int semValue = 0;

    // get the main loop semaphore value
    sem_getvalue(&gMainLoopSem, &semValue);

    // check if the sempahore is not already posted
    if(semValue < 1)
    {
        // increase the semaphore so the main loop will react on it
        semValue = sem_post(&gMainLoopSem);
        if(semValue != 0)
        {
            cli_printfError("escapeMainLoopWait: ERROR sem_post failed\n");
            return -1;
        }
    }

    // return OK
    return 0;
}
