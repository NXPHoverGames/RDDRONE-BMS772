/****************************************************************************
 * nxp_bms/BMS_v1/src/main.c
 *
 * BSD 3-Clause License
 *
 * Copyright 2020-2024 NXP
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
#include <assert.h>

#include "data.h"
#include "cli.h"
#include "ledState.h"
#include "gpio.h"
#include "batManagement.h"
#include "cyphalcan.h"
#include "dronecan.h"
#include "sbc.h"
#include "nfc.h"
#include "a1007.h"
#include "spi.h"
#include "SMBus.h"
#include "i2c.h"
#include "power.h"
#include "display.h"

#warning setting default string in dronecan will not work yet.

/****************************************************************************
 * Defines
 ****************************************************************************/
#define BMS_VERSION_STRING_ADDITION "11.0\n"
//#define DONT_DO_CAN

//! @brief This is the mainLoop task priority (highest priority)
#define MAIN_LOOP_PRIORITY 130

//! @brief This is the updater task priority (lowest priority)
#define UPDATER_PRIORITY 50

//! @brief The needed stack for the mainLoop task
#define MAIN_LOOP_STACK_SIZE 2048 + 512

//! @brief The needed stack for the updater task
#define DEFAULT_UPDATER_STACK_SIZE 2048

/*!
 * @brief this define is used to make sure it doesn't keep going in the charge with CB (from relaxation) when
 * done
 */
#define AMOUNT_CB_CHARGE_CYCLES_MAX 5

//! @brief the amount of seconds the button should be pressed to go to deepsleep
#define BUTTON_TIME_FOR_DEEP_SLEEP 5 // [s]

//! @brief The time the button whould be pressed in the SELF_DISCHARGE state to go back to the init state.
#define SELF_DISCHARGE_WAIT_TIME 10 // [s]

//! @brief this define is used so the CC overflow message isn't send every second
#define CC_OVERFLOW_MESS_TIMEOUT_TIME 120 // [s]

//! @brief These defines are the times the mainloop will wait (as long as no int.) before looping again.
#define MAIN_LOOP_WAIT_TIME_MS 100 // [ms]
//! @brief Same as above, but in sleep mode.
#define MAIN_LOOP_LONG_WAIT_TIME_S 2 // [s]

// check if PM module is configured correctly to go to VLPR mode
#if(!defined(CONFIG_VLPR_STANDBY)) || (!defined(CONFIG_VLPR_SLEEP))
#    if(!defined(DISABLE_PM))
#        error configure PM correctly or set DISABLE_PM in power.h, see/set defconfig
#    else
#        warning configure PM correctly or set DISABLE_PM in power.h, see/set defconfig
#    endif
#endif

/****************************************************************************
 * Types
 ****************************************************************************/
//! @brief This enum can be used to set or get the transition.
typedef enum
{
    SLEEP_VAR,
    CHAR_VAR,
    DISCHAR_VAR
} transitionVars_t;

/****************************************************************************
 * private data
 ****************************************************************************/
//! Variable to indicate if the bms is initialzed
static bool gBmsInitialized = false;

//! make a mutex lock for the main state
pthread_mutex_t gStateLock;

//! make a mutex lock for the charge state
pthread_mutex_t gChargeStateLock;

//! make a mutex lock for the transition variables
pthread_mutex_t gTransVarLock;

//! mutex for the state commands
pthread_mutex_t gStateCommandLock;

//! mutex for enabling or disabling the CAN messages
pthread_mutex_t gSetCanMessagesLock;

//! mutex for enabling or disabling the NFC Update
pthread_mutex_t gSetNfcUpdateLock;

//! mutex for enabling or disabling the display Update
pthread_mutex_t gSetDisplayUpdateLock;

/*! @brief  semaphore to do the main loop*/
static sem_t gMainLoopSem;

/*! @brief  semaphore to update the NFC information*/
static sem_t gUpdaterSem;

/*! @brief  Variables to keep track of the state machine states */
states_t        gCurrentState       = SELF_TEST;
charge_states_t gCurrentChargeState = CHARGE_START;

/*! @brief  Variables to keep track of transitions in the state machine */
static bool gDischargeDetected = true;
static bool gChargeDetected    = false;
static bool gSleepDetected     = false;

/*! @brief  Variables to keep track of GPIO edges */
static bool gBCCRisingEdge    = false;
static bool gButtonRisingEdge = false;
static bool gButtonPressEdge  = false;

/*! @brief  Variables to indicate that the s-in-flight parameter changed */
static bool gSInFlightChangedFalse = false;

//! bool to indicate updater can mode
static int gCanModeOFFDroneCANCyphalCAN = CAN_OFF_NUM; // 0 = off, 1 = DroneCAN, 2 = CyphalCAN

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
 * @brief function for a task to handle the update of the NFC, SMBus and display
 *
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int updaterTaskFunc(int argc, char *argv[]);

/*!
 * @brief   This Function will check all kinds of inputs and could take care of state transitions.
 *          It will take care of the button, bcc pin and emergency button (if enabled) and act accordingly.
 *          It will check for faults if there are faults (from BCC or batManag task)
 *
 * @param   pSelfDischargeTime address of the timespec struct what is the selfDischargeTimestruct
 * @param   pButtonPressedTime address of the timespec struct to become the time of the button press
 * @param   pDeepsleepTimingOn address of the variable to become the value if the deepsleep timing is on
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  None
 */
static void checkInputsAndStateTransitions(struct timespec *pSelfDischargeTime,
    struct timespec *pButtonPressedTime, bool *pDeepsleepTimingOn, bool *pCellUnderVoltageDetected,
    states_t *pOldState);

/*!
 * @brief   Function that is used to output messages on the CLI
 *          This is mainly for the "Rising edge BCC pin!" or "clearing CC overflow" message
 *          It will make sure it will not output the latter message too often
 *
 * @param   risingEdgeMessage if true, it will output Rising edge BCC pin if needed.
 *          if false, it will check if clearing CC overflow needs to be send.
 * @param   BMSFault The value of the BMSFault, which can be retreived with batManagement_checkFault()
 * @param   pCurrentTime The address of the struct timespec of the current time.
 *
 * @return  none
 */
static void bmsOutputRisingEdgeOrCCOverflow(
    bool risingEdgeMessage, uint32_t BMSFault, struct timespec *pCurrentTime);

/*!
 * @brief   Function that is used handle the noticed fault, it could change the state to FAULT_ON, INIT or
 * RELAXATION It could output the needed messages on the CLI and it could clear the fault.
 *
 * @param   BMSFault The BMSFault that is retreived from batManagement_checkFault()
 * @param   pCurrentTime address of the currentTime variable (after getting the current time)
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  none
 */
static void bmsHandleFault(
    uint32_t BMSFault, struct timespec *pCurrentTime, bool *pCellUnderVoltageDetected, states_t *pOldState);

/*!
 * @brief   Function that is used take care of the main state machine.
 *
 * @param   pSelfDischargeTime address of the timespec struct what is the selfDischargeTimestruct
 * @param   pButtonPressedTime address of the timespec struct to become the time of the button press
 * @param   pDeepsleepTimingOn address of the variable to become the value if the deepsleep timing is on
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  None
 */
static void mainStateMachine(struct timespec *pSelfDischargeTime, struct timespec *pButtonPressedTime,
    bool *pDeepsleepTimingOn, bool *pCellUnderVoltageDetected, states_t *pOldState);

/*!
 * @brief   Function that is used take care of the charge state machine.
 *
 * @param   pOldState address of the oldState variable
 * @param   chargeToStorage the variable that indicates if charge to storage is active
 *
 * @return  none
 */
static void chargeStateMachine(charge_states_t *pOldChargeState, bool chargeToStorage);

/*!
 * @brief   Function to check for new state transistions based on the current
 *
 * @param   currentA Address of the just measured battery current in A.
 *          If a NULL pointer is given, it will reset the timers and variables.
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int checkForTransitionCurrent(float *currentA);

/*!
 * @brief   Function to handle a parameter change which is not accesable from data.c
 *
 * @warning May only be called from data.c.
 * @warning Do not use the data_setParameter() and data_getParameter() functions.
 *
 * @param   parameter The parameter that changed.
 * @param   value Address of the variable containing the new value.
 * @param   extraValue Address of the extra value that can be used, may be NULL if not used.
 *
 * @return  0 if succeeded, false otherwise
 */
int handleParamaterChange(parameterKind_t parameter, void *value, void *extraValue);

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
 * @param command the command that needs to be processed
 */
int processCLICommand(commands_t command);

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
 *          the oldState is set with the current state (oldState = getMainState())
 *          when the OCV timer needs to be increased
 *
 * @return  0 if ok
 */
static int getOcvPeriodTime(int32_t *newTime, states_t oldState);

/*!
 * @brief   function that is used to enable or disable the CAN messages
 *          or get the value if it should output the CAN messages.
 *
 * @param   setNGet if true it will set the value if the CAN messages should be outputted.
 *          if false it will return 1 if it should be outputted, 0 otherwise.
 * @param   enable if setNGet is true, if enable true the CAN messages are enabled
 *          to send after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableCanMessages(bool setNGet, bool enable);

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
 * @brief   Function that is used to check if the tasks are on
 *          Like the batmanag task, updater task and other tasks
 *
 * @param   on the address of the varible to become true if the tasks are on,
 *          false otherwise.
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int checkSequenceTaskStatus(bool *on);

/*!
 * @brief   Function that is used to check if the updater tasks is on
 *
 * @param   none
 *
 * @return  true if the updater task is running, false otherwise
 */
static bool getUpdaterTaskStatus(void);

/*!
 * @brief   Function that is used to get out of the sem_timedwait()
 *
 * @param   none
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int escapeMainLoopWait(void);

/*!
 * @brief   Function that is used to stop the main loop (error)
 *
 * @param   none
 *
 * @return  none
 */
static void stopMainLoop(void);

/*!
 * @brief   Function that will enable or disable I2C, CAN, NFC, Display updates and
 *          The battery management task if setBatManagement is true.
 * @note    When disabling it will set the SMBUS current to 0.
 *
 * @param   enable If the updates or task need to be enabled (true) or disabled (false).
 * @param   setBatManagement if the enable or disable should be done for the battery management task as well.
 *
 * @return  none
 */
static void enableUpdatesAndBatManagementTask(bool enable, bool setBatManagement);

/*!
 * @brief   Function that will reset the common variables
 *          It will disable cell balancing, turn off the charge mode and reset the command variable
 *
 * @param   none
 *
 * @return  none
 */
static void resetVariables(void);

/*!
 * @brief   Function that will check if the MCU is in run mode
 *          If not, it will set the MCU to run mode.
 *          If it failed to set the MCU to run mode, it will stop.
 *          It will set the SBC (system basic chip)
 *          and AFE (Analog front end (BCC)) to normal.
 *
 * @param   none
 *
 * @return  none
 */
static int setMcuAfeSbcToNormalMode(bool returnWhenError);

/****************************************************************************
 * main
 ****************************************************************************/
// keep in mind that this program can be called more than once to enter parameters
int bms_main(int argc, char *argv[])
{
    // the messages for the tasks
    int  retValue;
    int  errcode;
    bool resetCauseExWatchdog = 0;

    // clear the screen from cursor down
    cli_printf("\e[0J");

    // reset the color if there was any color
    cli_printf("\e[39m");

    // check if the initialization is not done yet.
    // Otherwise with each "bms" call, this is done again.
    // now the use can call "bms help" to get the help
    if(!gBmsInitialized)
    {
        // output the bms version once
        cli_printf("BMS version: BMS%d.%d-%s\n", BMS_MAJOR_VERSION_NUMBER, BMS_MINOR_VERSION_NUMBER,
            BMS_VERSION_STRING_ADDITION);

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

        // Check if the reset cause is the watchdog
        resetCauseExWatchdog = RESET_CAUSE_EXT_PIN & data_getResetCause();

        // Check if the reset cause is the watchdog
        if(resetCauseExWatchdog)
        {
            cli_printfWarning("WARNING: Not doing self-test due to watchdog reset!\n");
        }
        else
        {
            cli_printf("NOTE: Safety library for the BCC is available under NDA\n");
        }

        // initialze all mutex
        pthread_mutex_init(&gChargeStateLock, NULL);
        pthread_mutex_init(&gTransVarLock, NULL);
        pthread_mutex_init(&gStateCommandLock, NULL);
        pthread_mutex_init(&gSetCanMessagesLock, NULL);
        pthread_mutex_init(&gSetNfcUpdateLock, NULL);
        pthread_mutex_init(&gSetDisplayUpdateLock, NULL);

        // initialize the LED and make it RED
        retValue = ledState_initialize(RED, resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize leds! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST LEDs: \e[31mFAIL\e[39m\n");
                return retValue;
            }
        }

        // initialize the cli to make sure things aren't printed at the same time
        cli_initialize(&processCLICommand);

        // initialize the changed parameter semaphore
        sem_init(&gUpdaterSem, 0, 0);

        // create the updater task
        retValue =
            task_create("updater", UPDATER_PRIORITY, DEFAULT_UPDATER_STACK_SIZE, updaterTaskFunc, NULL);
        if(retValue < 0)
        {
            errcode = errno;

            cli_printfError("main ERROR: Failed to start updater task: %d\n", errcode);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return 0;
            }
        }

        // initialzie the data part
        retValue = data_initialize(&handleParamaterChange, &getMainState, &getChargeState);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize data! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return retValue;
            }
        }

        // initialze the power part
        retValue = power_initialize();
        if(retValue)
        {
            cli_printfError("main ERROR: failed to initialize power! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return retValue;
            }
        }

        // initialize the GPIO
        retValue = gpio_init(resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize gpio! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
                return retValue;
            }
        }

        // initialize the SPI
        retValue = spi_initialize();
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize SPI! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return retValue;
            }
        }

        // initialze the SBC
        retValue = sbc_initialize(resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize SBC! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST SBC: \e[31mFAIL\e[39m\n");
                return retValue;
            }
        }

#ifndef DONT_DO_CAN
        {
            char     can_mode[STRING_MAX_CHARS];
            uint16_t can_mode_size = 0;
            void *   dataReturn;

            dataReturn              = (int32_t *)data_getParameter(CAN_MODE, can_mode, &can_mode_size);
            can_mode[can_mode_size] = 0x0;

            if(dataReturn != NULL)
            {
                // if no CAN mode is enabled
                if(strncasecmp(can_mode, "OFF", sizeof("OFF")) == 0)
                {
                    // turn off the CAN messages
                    if(setNGetEnableCanMessages(true, false) < 0)
                    {
                        // output error
                        cli_printfError("main ERROR: Could not disable CAN messages!\n");
                    }

                    // Set the droneCAN mode to OFF
                    gCanModeOFFDroneCANCyphalCAN = CAN_OFF_NUM;
                }
                else if(strncasecmp(can_mode, "DRONECAN", sizeof("DRONECAN")) == 0)
                {
                    retValue = dronecan_initialize();
                    if(retValue)
                    {
                        // output to the user
                        cli_printfError("main ERROR: failed to initialize DroneCAN! code %d\n", retValue);

                        // Check if the reset cause is not the watchdog
                        if(!resetCauseExWatchdog)
                        {
                            return retValue;
                        }
                    }
                    // Set the droneCAN mode to DroneCAN
                    gCanModeOFFDroneCANCyphalCAN = DRONECAN_NUM;
                }
                else if(strncasecmp(can_mode, "CYPHAL", sizeof("DRONECAN")) == 0)
                {
                    retValue = cyphalcan_initialize();
                    if(retValue)
                    {
                        // output to the user
                        cli_printfError("main ERROR: failed to initialize CAN! code %d\n", retValue);

                        // Check if the reset cause is not the watchdog
                        if(!resetCauseExWatchdog)
                        {
                            return retValue;
                        }
                    }

                    // Set the droneCAN mode to CyphalCAN
                    gCanModeOFFDroneCANCyphalCAN = CYPHALCAN_NUM; // Indicate Cyphal moder for updater
                }
                else
                {
                    cli_printfError(
                        "main ERROR: failed to set can-mode! name: \"%s\" and size: %i not supported\n", can_mode, can_mode_size);
                    cli_printf("Valid options are: %s, %s and %s\n\n", CAN_MODE_OFF, CAN_MODE_DRONECAN, CAN_MODE_CYPHAL);
                }

                cli_printf("can-mode is set to \"%s\"\n", can_mode);
            }
            else
            {
                cli_printfError("main ERROR: failed to get CAN mode!\n");
            }
        }
#endif

        // initialize the battery management
        retValue = batManagement_initialize(&swMeasuredFaultFunction, &changeLedColor,
            &checkForTransitionCurrent, &newMeasurementsFunction, resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize batManagement! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                return retValue;
            }
        }

        // initialize the I2C
        retValue = i2c_initialize();
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize i2c! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                // return retValue;
            }
        }

        // set the SBC mode to NORMAL to enable 5V in case something is there connected to the I2C
        if(sbc_setSbcMode(SBC_NORMAL))
        {
            cli_printfError("main ERROR: Could not set the SBC to NORMAL mode!\r\n");
        }

        // initialze the NFC
        retValue = nfc_initialize(resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize nfc! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST NFC: \e[31mFAIL\e[39m\n");
            }

            // disable NFC for now
            nfc_disableNFC(true);
        }

        // initialze the A1007
        retValue = a1007_initialize(resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize A1007! code %d\n", retValue);

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST A1007: \e[31mFAIL\e[39m\n");
            }
        }

        // initialize the SMBus
        retValue = SMBus_initialize();
        if(retValue)
        {
            // output to the user
            cli_printfError("main ERROR: failed to initialize SMBus! code %d\n", retValue);
        }

        // initialze the display
        retValue = display_initialize(resetCauseExWatchdog);
        if(retValue)
        {
            // output to the user
            cli_printfWarning("main WARNING: failed to initialize display! code %d\n", retValue);
            cli_printf("Could be that there is no display\n");

            // Check if the reset cause is not the watchdog
            if(!resetCauseExWatchdog)
            {
                cli_printf("SELF-TEST DISPLAY: \e[31mFAIL\e[39m\n");
            }
        }

        // initialize the main loop semaphore
        sem_init(&gMainLoopSem, 0, 0);

        // Check if the reset cause is not the watchdog
        if(!resetCauseExWatchdog)
        {
            // if no error occured with a GPIO, the GPIO SELF-TEST has passed
            cli_printf("SELF-TEST GPIO: \e[32mPASS\e[39m\n");

            // output to the user
            cli_printfGreen("ALL CRITICAL SELF-TESTS PASSED!\n");
        }

        // go to the INIT state
        setMainState(INIT);

        // create the main loop task
        retValue = task_create("mainLoop", MAIN_LOOP_PRIORITY, MAIN_LOOP_STACK_SIZE, mainTaskFunc, NULL);
        if(retValue < 0)
        {
            // get the error
            errcode = errno;

            // error
            cli_printfError("main ERROR: Failed to start main loop task: %d\n", errcode);
            return 0;
        }
        else
        {
            gBmsInitialized = true;
        }
    }
    // if the bms is already initialized, process the command
    else
    {
        // check if a command was given
        if(argc > 1)
        {
            // TODO make parameters only changeable in normal and sleep, (self_discharge, fault)

            // start the cli task
            cli_processCommands(argc, argv);

            // return
            return 0;
        }
    }

    // return
    return 0;
}

/*!
 * @brief this function will implement the main state machine
 *
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int mainTaskFunc(int argc, char *argv[])
{
    int             retValue;
    struct timespec bmsWaitTime;
    struct timespec buttonPressedTime;
    struct timespec selfDischargeTime;
    bool            deepsleepTimingOn        = false;
    bool            cellUnderVoltageDetected = false;

    // get the variables if the fault happend
    gBCCRisingEdge = gpio_readPin(BCC_FAULT);

    // get the variables if the button press happend
    gButtonRisingEdge = gpio_readPin(SBC_WAKE);
    gButtonPressEdge  = !gButtonRisingEdge;

    // register the pins for the ISR
    retValue = gpio_registerISR((uint16_t)((1 << BCC_FAULT)), gpioIsrFunction);

    // check for errors
    if(retValue)
    {
        // output the error
        cli_printfError("main ERROR: GPIO register ISR went wrong! %d\n", retValue);

        // return
        return retValue;
    }

    // make sure that if there was an interrupt it is read
    if(!gBCCRisingEdge && gpio_readPin(BCC_FAULT))
    {
        // set it
        gBCCRisingEdge = true;
    }

    // make sure that if there was an interrupt it is read
    if(!gButtonRisingEdge && gpio_readPin(SBC_WAKE))
    {
        // set it
        gButtonRisingEdge = true;
        gButtonPressEdge  = false;
    }

    // make the oldState different
    states_t oldState = !getMainState();

    // start the task
    cli_printf("BMS main loop!\n");

    // loop in the state machine
    while(1)
    {
        // this function will check inputs like the button, the bcc pin, faults, emergency button (if enabled)
        // It will output message on the CLI and change the main or charge state.
        // it will check the inputs and faults and change the main or charge state if needed.
        checkInputsAndStateTransitions(
            &selfDischargeTime, &buttonPressedTime, &deepsleepTimingOn, &cellUnderVoltageDetected, &oldState);

        // do the things as described in the main state machine diagram
        // this is where the actual logic of the state diagram is
        mainStateMachine(
            &selfDischargeTime, &buttonPressedTime, &deepsleepTimingOn, &cellUnderVoltageDetected, &oldState);

        // get the current time
        if(clock_gettime(CLOCK_REALTIME, &bmsWaitTime) == -1)
        {
            cli_printfError("main ERROR: failed to get bmsWaitTime!\n");
        }

        // check if in charge relaxation where the BMS is doing a lot in very low power run mode
        // meaning that the 100ms wait time may not be sufficient
        if(getMainState() == CHARGE && getChargeState() == RELAXATION)
        {
            // add the 2s, for 2s wait
            bmsWaitTime.tv_sec += MAIN_LOOP_LONG_WAIT_TIME_S;
        }
        else
        {
            // make the 100ms wait time in the current time for the normal mode
            bmsWaitTime.tv_sec += (bmsWaitTime.tv_nsec + MAIN_LOOP_WAIT_TIME_MS * 1000000) / (MAX_NSEC + 1);
            bmsWaitTime.tv_nsec = (bmsWaitTime.tv_nsec + MAIN_LOOP_WAIT_TIME_MS * 1000000) % (MAX_NSEC + 1);
        }

        // kick the watchdog before the timed wait
        if(sbc_kickTheWatchdog())
        {
            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        }

        // wait for 100ms or the semaphore is posted (with a fault)
        // the semaphore is posted to trigger this task when it needs to react on things
        sem_timedwait(&gMainLoopSem, &bmsWaitTime);

        // kick the watchdog after the timed wait
        if(sbc_kickTheWatchdog())
        {
            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        }
    }

    return 0;
}

/*!
 * @brief function for a task to handle the update of the NFC, SMBus and display
 *
 * @param argc the amount of arguments there are in argv (if the last argument is NULL!)
 * @param argv a character pointer array with the arguments, first is the taskname than the arguments
 */
static int updaterTaskFunc(int argc, char *argv[])
{
    int                      error;
    uint8_t                  uint8Val;
    commonBatteryVariables_t updaterCommonBatteryVars;
    calcBatteryVariables_t   updaterCalcBatteryVars;

    // loop endlessly
    while(1)
    {
        // wait until the semaphore is available
        sem_wait(&gUpdaterSem);

        // get the variables in the local copy of the struct to make sure
        // Every update uses the same data
        if(data_getCommonBatteryVariables(&updaterCommonBatteryVars) ||
            data_getCalcBatteryVariables(&updaterCalcBatteryVars, false))
        {
            // output to user
            cli_printfError("updater ERROR: Could not get battery or calc vars, skipping update!\n");
        }
        else
        {
#ifndef DONT_DO_CAN
            // check if the message needs to be send
            if(setNGetEnableCanMessages(false, 0))
            {
                // send data over CAN
                if(gCanModeOFFDroneCANCyphalCAN == CAN_OFF_NUM)
                {
                    error = 0;
                }
                else if(gCanModeOFFDroneCANCyphalCAN == DRONECAN_NUM)
                {
                    error = dronecan_sendBMSStatus();
                }
                else if(gCanModeOFFDroneCANCyphalCAN == CYPHALCAN_NUM)
                {
                    error = cyphalcan_sendBMSStatus();
                }
                //&updaterCommonBatteryVars, &updaterCalcBatteryVars); //FIXME SWITCH between both

                // check error
                if(error)
                {
                    // output to user
                    cli_printfError("updater ERROR: Could not send CAN!\n");
                }
            }
#endif
            // check if SMBus needs to be done
            // get the SMBus enable value
            if(data_getParameter(SMBUS_ENABLE, &uint8Val, NULL) == NULL)
            {
                cli_printfError("updater ERROR: getting SMBus enable went wrong!\n");
                uint8Val = SMBUS_ENABLE_DEFAULT;
                // return retValue;
            }

            // limit
            uint8Val &= 1;

            // check if it needs to be updated
            if(uint8Val)
            {
                // update the SMBUs data
                error = SMBus_updateInformation(false, &updaterCommonBatteryVars, &updaterCalcBatteryVars);

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
                if(nfc_updateBMSStatus(false, false, &updaterCommonBatteryVars, &updaterCalcBatteryVars))
                {
                    // disable NFC for now
                    nfc_disableNFC(true);

                    // output to the user
                    cli_printfError("updater ERROR: Can't update NFC!\n");
                }
            }

            // TODO make CLI lower prio

            // Update the CLI data
            if(cli_updateData(&updaterCommonBatteryVars, &updaterCalcBatteryVars))
            {
                // output to the user
                cli_printfError("updater ERROR: Can't update CLI!\n");
            }

            // TODO add display on even lower priority

            // check if the display needs to be updated
            if(setNGetEnableDisplayUpdates(false, 0) &&
                !(getMainState() == CHARGE && getChargeState() == RELAXATION))
            {
                // Check if the display power is off
                if(!display_getPower())
                {
                    // Turn on the display
                    if(display_setPower(true))
                    {
                        cli_printfError("updater ERROR: Could not turn on display, uninitializing!\n");

                        // uninitialize the display
                        display_uninitialize();
                    }
                }

                // Update the display data
                if(display_updateValues(&updaterCommonBatteryVars, &updaterCalcBatteryVars))
                {
                    // output to the user
                    cli_printfError("updater ERROR: Can't update display, uninitializing!\n");

                    // uninitialize the display
                    display_uninitialize();
                }
            }

            // update the LED blink
            ledState_calcStateIndication(updaterCalcBatteryVars.s_charge);
        }
    }

    // for compiler
    return -1;
}

/*!
 * @brief   This Function will check all kinds of inputs and could take care of state transitions.
 *          It will take care of the button, bcc pin and emergency button (if enabled) and act accordingly.
 *          It will check for faults if there are faults (from BCC or batManag task)
 *
 * @param   pSelfDischargeTime address of the timespec struct what is the selfDischargeTimestruct
 * @param   pButtonPressedTime address of the timespec struct to become the time of the button press
 * @param   pDeepsleepTimingOn address of the variable to become the value if the deepsleep timing is on
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  None
 */
static void checkInputsAndStateTransitions(struct timespec *pSelfDischargeTime,
    struct timespec *pButtonPressedTime, bool *pDeepsleepTimingOn, bool *pCellUnderVoltageDetected,
    states_t *pOldState)
{
    static bool     firstTime      = true;
    static int      oldButtonState = 0;
    int             buttonState;
    uint32_t        BMSFault;
    uint8_t         emergencyButtonEnable;
    struct timespec currentTime;
    states_t        mainState = getMainState();

    // check for NULL pointers in debug mode
    DEBUGASSERT(pButtonPressedTime != NULL);
    DEBUGASSERT(pDeepsleepTimingOn != NULL);

    // get the buttonstate
    buttonState = gpio_readPin(SBC_WAKE);

    // check if the oldBUttonState needs to be initialized
    if(firstTime)
    {
        // make sure it is not equal to check it.
        oldButtonState = !buttonState;
    }

    // check if changed
    if(buttonState != oldButtonState)
    {
        // check if the button pin is high
        if(buttonState)
        {
            // set the variable high (release)
            gButtonRisingEdge = true;
        }
        // if it is low
        else
        {
            // set the variable high (press)
            gButtonPressEdge = true;
        }

        // set the old one
        oldButtonState = buttonState;
    }

    // check if the BCC fualt pin is high and the variable is not high
    // if it was not noticed
    if((mainState != FAULT_ON && mainState != FAULT_OFF) && (mainState != INIT) && (mainState != CHARGE) &&
        (mainState != DEEP_SLEEP) && (!gBCCRisingEdge) && (gpio_readPin(BCC_FAULT)))
    {
        // output to user why
        cli_printf("Rising edge BCC pin not noticed, setting fault now!\n");

        // set the variable high to go through the error handler
        gBCCRisingEdge = true;
    }

    // check if the BCC pin is high or there is an other fault
    // This could be because a fault (OV, UV, ..), there is a sleep overcurrent or
    // The batManag task discovered a fault the SW should act on
    if(gBCCRisingEdge)
    {
        // make sure MCU is in normal mode, set SBC and AFE to normal mode
        if(setMcuAfeSbcToNormalMode(true))
        {
            // check if charging
            if(mainState == CHARGE)
            {
                // disconnect the battery!
                // open the gate
                if(batManagement_setGatePower(GATE_OPEN) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");
                }
            }

            // error and stop main loop
            stopMainLoop();
        }

        // check for sleep
        if(mainState == SLEEP)
        {
            // enable CC overflow on the fault pin
            batManagement_setCCOvrFltEnable(true);

            // go to the INIT state
            setMainState(INIT);
            mainState = INIT;
        }

        // check the BCC fault, 2 times to make sure there is no false alarm.
        batManagement_checkFault(&BMSFault, 0);
        batManagement_checkFault(&BMSFault, 0);

        // get the current time
        if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
        {
            cli_printfError("main ERROR: failed to get currentTime in gBCCRisingEdge!\n");
        }

        // don't output Rising edge BCC pin! when first start-up
        if(!firstTime)
        {
            // output the rising edge BCC pin! message
            // if it is because of BMS_CC_OVERFLOW, check if it needs to be send again.
            bmsOutputRisingEdgeOrCCOverflow(true, BMSFault, &currentTime);

            // check if a fault occured
            if(BMSFault)
            {
                // handle the fault
                // this function could set the state to FAULT_ON, INIT or RELAXATION
                // It will output the error to the user and is able to clear the fault
                bmsHandleFault(BMSFault, &currentTime, pCellUnderVoltageDetected, pOldState);
            }
        }

        // to make sure you only enter this once
        gBCCRisingEdge = false;

        // end of if(gBCCRisingEdge)
    }

    // check for a buttonpress
    if(gButtonPressEdge)
    {
        // check if the state is a state where the button does something and the time needs to be retreived
        if(mainState == SLEEP || mainState == NORMAL || mainState == CHARGE)
        {
            // get the time
            if(clock_gettime(CLOCK_REALTIME, pButtonPressedTime) == -1)
            {
                cli_printfError("main ERROR: failed to get buttonPressedTime!\n");
            }

            // set the variable true
            *pDeepsleepTimingOn = true;
        }

        // make sure it will only do this once
        gButtonPressEdge = false;
    }

    // check the button ISR value or the fault should be reset
    if(gButtonRisingEdge || setNGetStateCommandVariable(false, CMD_ERROR) == CMD_RESET)
    {
        // check if it is in the FAULT_ON, FAULT_OFF or SLEEP state
        // because then the button will reset to init.
        if(mainState == FAULT_ON || mainState == FAULT_OFF || mainState == SLEEP)
        {
            // go the the init state with a button press
            setMainState(INIT);
            mainState = INIT;
        }
        // in in self discharge, the button press could make it go to init as well after the elapsed time
        else if(mainState == SELF_DISCHARGE)
        {
            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime in gButtonRisingEdge!\n");
            }

            // check if the right amount of time has passed
            if(((pSelfDischargeTime->tv_sec + SELF_DISCHARGE_WAIT_TIME) < currentTime.tv_sec) ||
                (((pSelfDischargeTime->tv_sec + SELF_DISCHARGE_WAIT_TIME) == currentTime.tv_sec) &&
                    pSelfDischargeTime->tv_nsec <= currentTime.tv_nsec))
            {
                // go to the INIT state
                setMainState(INIT);
                mainState = INIT;
            }
        }

        // set the variable to false to only do this once
        gButtonRisingEdge = false;

        // reset the command variable
        setNGetStateCommandVariable(true, CMD_NONE);

        // set the variable false
        *pDeepsleepTimingOn = false;
    }

    // check if the pin is low, but the overcurrent happend
    if(mainState != FAULT_ON && mainState != FAULT_OFF && gpio_readPin(OVERCURRENT))
    {
        // output to the users
        cli_printfError("main ERROR: hardware overcurrent detected!\n");

        // go to the FAULT_OFF state since the hardware overcurrent has turned off the system
        setMainState(FAULT_OFF);
        mainState = FAULT_OFF;
    }

    // get the emergency button enable value
    if(data_getParameter(EMERGENCY_BUTTON_ENABLE, &emergencyButtonEnable, NULL) == NULL)
    {
        cli_printfError("main ERROR: getting emergency button enable value went wrong!\n");

        // set the default one just in case
        emergencyButtonEnable = EMERGENCY_BUTTON_ENABLE_DEFAULT;
    }

    // check if the emergency button is used and pressed and it is not already in the fault state
    if((emergencyButtonEnable & 1) && (mainState != FAULT_OFF && mainState != DEEP_SLEEP) &&
        gpio_readPin(PTE8))
    {
        // output to the users
        cli_printfError("main ERROR: emergency button pressed!\n");

        // go to the FAULT_OFF state
        setMainState(FAULT_OFF);
    }

    // it is not the first time anymore
    firstTime = false;

    return;
}

/*!
 * @brief   Function that is used to output messages on the CLI
 *          This is mainly for the "Rising edge BCC pin!" or "clearing CC overflow" message
 *          It will make sure it will not output the latter message too often
 *
 * @param   risingEdgeMessage if true, it will output Rising edge BCC pin if needed.
 *          if false, it will check if clearing CC overflow needs to be send.
 * @param   BMSFault The value of the BMSFault, which can be retreived with batManagement_checkFault()
 * @param   pCurrentTime The address of the struct timespec of the current time.
 *
 * @return  none
 */
static void bmsOutputRisingEdgeOrCCOverflow(
    bool risingEdgeMessage, uint32_t BMSFault, struct timespec *pCurrentTime)
{
    static bool            outputCCOverflowMessage = false;
    static bool            outputtedFirstMessage   = false;
    static uint16_t        amountOfMissedMessages  = 0;
    static struct timespec lastMessageTime         = { 0, 0 };

    // what should be send
    if(risingEdgeMessage)
    {
        // check if it is not only the CC overflow
        if((BMSFault & (~BMS_CC_OVERFLOW)))
        {
            // output the message for the rising edge on the BCC_FAULT pin
            cli_printf("Rising edge BCC pin!\n");

            // just in case
            outputCCOverflowMessage = true;
        }
        // if it is the CC overflow and the time between the messages is long enough or the max is reached
        else if((BMSFault & BMS_CC_OVERFLOW) &&
            (((lastMessageTime.tv_sec + CC_OVERFLOW_MESS_TIMEOUT_TIME) < pCurrentTime->tv_sec) ||
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
            outputCCOverflowMessage = true;
            outputtedFirstMessage   = true;
        }
        else
        {
            // increase the missed messages
            amountOfMissedMessages++;

            // set the value to 0 to indicate it doesn't need to be outputted
            outputCCOverflowMessage = false;
        }
    }
    else
    {
        // check if it needs to be outputted
        if(outputCCOverflowMessage)
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
                cli_printfError("main ERROR: failed to get lastMessageTime in gBCCRisingEdge!\n");
            }
        }
    }

    return;
}

/*!
 * @brief   Function that is used handle the noticed fault, it could change the state to FAULT_ON, INIT or
 * RELAXATION It could output the needed messages on the CLI and it could clear the fault.
 *
 * @param   BMSFault The BMSFault that is retreived from batManagement_checkFault()
 * @param   pCurrentTime address of the currentTime variable (after getting the current time)
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  none
 */
static void bmsHandleFault(
    uint32_t BMSFault, struct timespec *pCurrentTime, bool *pCellUnderVoltageDetected, states_t *pOldState)
{
    bool            isCCRegisterCleared;
    int             ret;
    charge_states_t chargeState;
    states_t        mainState = getMainState();

    // check for NULL pointers only in debug mode
    DEBUGASSERT(pCellUnderVoltageDetected != NULL);
    DEBUGASSERT(pOldState != NULL);

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
        if(mainState == SLEEP)
        {
            // go to the init state
            setMainState(INIT);
        }
    }

    // check if need to react
    if(BMSFault &
        (BMS_CELL_UV + BMS_CELL_OV + BMS_SW_CELL_OV + BMS_PCB_UV + BMS_PCB_OV + BMS_UT + BMS_OT +
            BMS_AVG_OVER_CURRENT + BMS_PEAK_OVER_CURRENT))
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
            *pCellUnderVoltageDetected = true;
        }
        if(BMSFault & BMS_UT)
        {
            cli_printfError("BMS undertemperature detected!\n");
        }
        if(BMSFault & BMS_OT)
        {
            cli_printfError("BMS overtemperature detected!\n");
        }

        // get the charge state
        chargeState = getChargeState();

        // check for a cell overvoltage in the charge with CB or relaxation
        if((((BMSFault & (BMS_SW_CELL_OV)) == BMS_SW_CELL_OV) ||
               ((BMSFault & (BMS_CELL_OV)) == BMS_CELL_OV)) &&
            (mainState == CHARGE) && ((chargeState == CHARGE_CB) || (chargeState == RELAXATION)) &&
            (!(BMSFault & (BMS_AVG_OVER_CURRENT + BMS_PEAK_OVER_CURRENT + BMS_UT + BMS_OT))))
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

            // go to the FAULT_ON state
            setMainState(FAULT_ON);

            // check if the old state is the FAULT_ON state
            if(*pOldState == FAULT_ON)
            {
                // check if the fault is a peak overcurrent fault
                // this could happen with flight mode, but there should still
                // be a check on the peak overcurrent
                if(BMSFault & BMS_PEAK_OVER_CURRENT)
                {
                    // set the old state to the self-test state to re-do the FAULT_ON state first check
                    *pOldState = SELF_TEST;
                }
            }
        }
    }

    // check if there is a CC overflow
    if(BMSFault & BMS_CC_OVERFLOW)
    {
        // read and reset the CC registers by calculating a new remaining charge
        ret = batManagement_calcRemaningCharge(&isCCRegisterCleared);

        // check if outputting message is needed
        if(isCCRegisterCleared)
        {
            // output CC overflow message if needed
            bmsOutputRisingEdgeOrCCOverflow(false, BMSFault, pCurrentTime);
        }

        // check for errors
        if(ret)
        {
            // output to user
            cli_printfError("main ERROR: could not reset CC register when overflow: %d\n", ret);
        }
    }

    // check if there was CSB wakeup in the OCV state
    if((BMSFault & ~(BMS_OTHER_FAULT + BMS_CC_OVERFLOW)) == BMS_CSB_WAKEUP && mainState == OCV)
    {
        cli_printf("clearing BCC pin pin due to CSB wakeup fault\n");

        // clear the fault
        batManagement_checkFault(&BMSFault, true);
    }

    return;
}

/*!
 * @brief   Function that is used take care of the main state machine.
 *
 * @param   pSelfDischargeTime address of the timespec struct what is the selfDischargeTimestruct
 * @param   pButtonPressedTime address of the timespec struct to become the time of the button press
 * @param   pDeepsleepTimingOn address of the variable to become the value if the deepsleep timing is on
 * @param   pCellUnderVoltageDetected address of the variable that is used to keep track of an undervoltage
 * @param   pOldState address of the oldState variable
 *
 * @return  None
 */
static void mainStateMachine(struct timespec *pSelfDischargeTime, struct timespec *pButtonPressedTime,
    bool *pDeepsleepTimingOn, bool *pCellUnderVoltageDetected, states_t *pOldState)
{
    static struct timespec sampleTime, sampleTime2;
    static struct timespec cellUnderVoltageTime;
    static bool            chargeToStorage       = false;
    static bool            outputMessageOnlyOnce = false;
    static charge_states_t oldChargeState        = CHARGE_COMPLETE;
    static uint16_t        bmsTimeoutTime        = 0;
    int                    retValue;
    uint32_t               BMSFault;
    struct timespec        currentTime;
    variableTypes_u        tempVariable1, tempVariable2;
    mcuPowerModes_t        mcuPowerMode;
    states_t               mainState = getMainState();

    // check the state variable which state it is
    switch(mainState)
    {
        // in case of the init state
        case INIT:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // set the LED to green
                ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                // make sure MCU is in normal mode, set SBC and AFE to normal mode
                setMcuAfeSbcToNormalMode(false);

                // Enable the I2C
                if(i2c_enableTransmission(true))
                {
                    cli_printfError("main ERROR: failed to enable I2C!\n");
                }

                // check configuration and reset faults
                batManagement_checkAFE(&BMSFault, true);

                // reset the undervoltage variable
                *pCellUnderVoltageDetected = false;

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
                        usleepMainLoopWatchdog(10 * 1000UL);

                    } while(gpio_readPin(BCC_FAULT) &&
                        (((currentTime.tv_sec - sampleTime.tv_sec) >= 1) &&
                            (currentTime.tv_nsec >= sampleTime.tv_nsec)));

                    // check if timeout happend
                    if(gpio_readPin(BCC_FAULT))
                    {
                        // set the fault pin flank true to go to fault state
                        gBCCRisingEdge = true;
                    }

                    // start a manual measurement and wait until the measurement is done
                    retValue = batManagement_doMeasurement();

                    // get the battery current and save it in the data struct
                    batManagement_getBattCurrent(true);

                    // wait 50ms to make sure the pin should be high again
                    usleepMainLoopWatchdog(50 * 1000UL);

                    // check if the interrupt happend
                    if(gBCCRisingEdge)
                    {
                        // make sure to do the whole init state next time
                        *pOldState = SELF_TEST;

                        // break to skip the rest of the init (don't close the switch)
                        break;
                    }
                }

                // turn off balancing, charge mode and reset the command variable
                resetVariables();

                // start a manual measurement and wait until the measurement is done
                // If there is a fault that the BCC could measure, it will set the pin high
                retValue = batManagement_doMeasurement();

                // check for error
                if(retValue)
                {
                    // set the fault pin flank true to go to fault state
                    gBCCRisingEdge = true;

                    // make sure to do the whole init state next time
                    *pOldState = SELF_TEST;

                    // break to escape the rest of the init
                    break;
                }

                // check if the inserted n-cells is ok
                retValue = batManagement_checkNCells(&tempVariable1.boolVar);

                // check for errors
                if(retValue != 0)
                {
                    // inform user
                    cli_printfError("main ERROR: Failed get n-cells ok: %d\n", retValue);

                    // set the fault pin flank true to go to fault state
                    gBCCRisingEdge = true;

                    // make sure to do the whole init state next time
                    *pOldState = SELF_TEST;

                    // break to escape the rest of the init
                    break;
                }

                // check if the output is low
                if(tempVariable1.boolVar != 1)
                {
                    // go to the FAULT_ON state
                    setMainState(FAULT_ON);

                    // make sure to do the whole init state next time
                    *pOldState = SELF_TEST;

                    // break to escape the rest of the init
                    break;
                }

                // close the gate
                if(batManagement_setGatePower(GATE_CLOSE) != 0)
                {
                    cli_printfError("main ERROR: Failed to close gate\n");
                }

                // do another measurement
                batManagement_doMeasurement();

                // get the battery current and save it in the data struct
                batManagement_getBattCurrent(true);

                // reset the transistion timers and variable
                checkForTransitionCurrent(NULL);

                // wait 1ms to make sure the current is set
                usleep(1 * 1000UL);

                // enable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(true, true);

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
        }

        break;
        case NORMAL:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // turn on the gate
                if(batManagement_setGatePower(GATE_CLOSE) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");
                }

                // get the battery current and save it in the data struct
                // batManagement_getBattCurrent();

                // reset the transistion timers and variable
                checkForTransitionCurrent(NULL);

                // turn on the measurements if not on
                batManagement_enableBatManagementTask(true);

                // reset consumed power
                tempVariable1.floatVar = 0;

                // set the consumed power to 0
                if(data_setParameter(E_USED, &tempVariable1.floatVar))
                {
                    cli_printfError("main ERROR: couldn't reset consumed power!\n");
                }

                // get the state of charge
                if(data_getParameter(S_CHARGE, &tempVariable1.uint8Var, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting state of charge went wrong!\n");

                    // set the default value just in case
                    tempVariable1.uint8Var = S_CHARGE_DEFAULT;
                }

                // set the value to calculate the state indication
                ledState_calcStateIndication(tempVariable1.uint8Var);

                // set the LED to green blinking
                ledState_setLedColor(GREEN, OFF, LED_BLINK_ON);

                // reset the command variable, balancing and charge mode
                resetVariables();

                cli_printf("NORMAL mode\n");

                // TODO CLI and NFC is allowed

                // TODO enable diagnostics
            }

            // check if the button is pressed
            if(*pDeepsleepTimingOn)
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
                    if(((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) ||
                        (((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) &&
                            pButtonPressedTime->tv_nsec <= currentTime.tv_nsec))
                    {
                        // go to the deep sleep state
                        setMainState(SELF_DISCHARGE);
                    }
                }
                else
                {
                    // set the variable false
                    *pDeepsleepTimingOn = 0;
                }
            }

            // check for charge
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
        }

        break;
        case CHARGE:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // turn on the measurements if not on
                batManagement_enableBatManagementTask(true);

                // turn on the gate
                if(batManagement_setGatePower(GATE_CLOSE) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");
                }

                // set the LED to blue
                ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                cli_printf("CHARGE mode\n");

                // set the charge state to the first state
                setChargeState(CHARGE_START);

                // set the old charge state to the last state
                oldChargeState = CHARGE_COMPLETE;

                // reset the charge to storage variable
                batManagement_SetNReadChargeToStorage(true, 0);

                // reset the command variable, balancing and charge mode
                resetVariables();

                // set the charge mode
                batManagement_startCharging(true);

                // set the variable to false
                chargeToStorage = false;

                // TODO CLI and NFC is allowed

                // TODO enable diagnostics
            }

            // do the charge state machine
            chargeStateMachine(&oldChargeState, chargeToStorage);

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
                if(data_getParameter(V_STORAGE, &tempVariable1.floatVar, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                    tempVariable1.floatVar = V_STORAGE_DEFAULT;
                }

                // check if the lowest cell voltage is higher or equal than the storage voltage
                if(batManagement_getLowestCellV() >= tempVariable1.floatVar)
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
            else if((!chargeToStorage) && (*pDeepsleepTimingOn))
            {
                // get the current time
                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
                }

                // check if the right amount of time has passed
                if(((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) ||
                    (((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) &&
                        pButtonPressedTime->tv_nsec <= currentTime.tv_nsec))
                {
                    // get the storage voltage
                    if(data_getParameter(V_STORAGE, &tempVariable1.floatVar, NULL) == NULL)
                    {
                        cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                        tempVariable1.floatVar = V_STORAGE_DEFAULT;
                    }

                    // check if the lowest cell voltage is higher or equal than the storage voltage
                    if(batManagement_getLowestCellV() >= tempVariable1.floatVar)
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

            // save the new main state
            mainState = getMainState();

            // check if the main state changed
            if(mainState != CHARGE)
            {
                // disable cell balancing
                batManagement_setBalanceState(BALANCE_OFF);

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
                    retValue = errno;

                    // error
                    cli_printfError("main ERROR: Could not get MCU power mode 2: %d\n", retValue);
                }

                // check if the power mode is not NORMAL mode
                if((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) ||
                    (mcuPowerMode == ERROR_VALUE))
                {
                    // kick the watchdog before the task yield
                    if(sbc_kickTheWatchdog())
                    {
                        cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                    }

                    // reset the intvalue
                    tempVariable1.uint16Var = 0;

                    // get the status of the sequence tasks
                    retValue = checkSequenceTaskStatus(&tempVariable2.boolVar);

                    // make sure it finished stuff
                    while(!retValue && tempVariable2.boolVar && tempVariable1.uint16Var < 100)
                    {
                        // sleep for a little while to make sure the other tasks can do their things
                        usleep(1 * 1000UL);

                        // get the status of the sequence tasks
                        retValue = checkSequenceTaskStatus(&tempVariable2.boolVar);

                        // increase the intvalue
                        tempVariable1.uint16Var++;
                    }

                    cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                    // make sure MCU is in normal mode, set SBC and AFE to normal mode
                    setMcuAfeSbcToNormalMode(false);

                    // enable the updates (I2C, NFC, CAN, display)
                    enableUpdatesAndBatManagementTask(true, false);
                }
            }
        }

        break;
        case SLEEP:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // calculate and get the right OCV timer value
                if(getOcvPeriodTime(&tempVariable1.int32Var, *pOldState))
                {
                    cli_printfError("main ERROR: failed to calculate new OCV time! \n");
                }

                // check if the *pOldState is not OCV
                // because this shouldn't be resetted when going to the OCV state
                if(*pOldState != OCV)
                {
                    // get the time that it first entered the sleep state
                    if(clock_gettime(CLOCK_REALTIME, &sampleTime2) == -1)
                    {
                        cli_printfError("main ERROR: failed to get sleep sampleTime! \n");
                    }
                }

                // cli_printf("time: %ds\n", tempVariable1.int32Var);

                // save the old value
                *pOldState = mainState;

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

                // reset the command variable, balancing and charge mode
                resetVariables();

                // disable CC overflow for the fault pin
                batManagement_setCCOvrFltEnable(false);

                // disable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(false, true);

                // check if the measurements are done
                retValue = batManagement_getBatManagementStatus(&tempVariable2.boolVar);

                // reset the intvalue
                tempVariable1.uint16Var = 0;

                // wait until the measurements are done
                // or at least 100ms have passed
                while(!retValue && tempVariable2.boolVar && (tempVariable1.uint16Var < 100))
                {
                    // sleep a little bit so it can be done (1ms)
                    usleep(1 * 1000UL);

                    // check if the measurements are done
                    retValue = batManagement_getBatManagementStatus(&tempVariable2.boolVar);

                    // increment the int value
                    tempVariable1.uint16Var++;
                }

                // check if there was an error
                if(retValue)
                {
                    cli_printfError("main ERROR: couldn't get measurement status, waiting for some time\n");
                    // sleep long enough so the measurements are done
                    usleepMainLoopWatchdog(7500);
                }

                // check if the timeout happend
                if(tempVariable1.uint16Var >= 100)
                {
                    cli_printfError("main ERROR: measurement status timed out after +100ms!\n");
                }

                // check if the sleep current threshold is enabled
                if(batManagement_checkSleepCurrentTh(&tempVariable1.boolVar))
                {
                    cli_printfError("Getting sleep current th mask went wrong!\n");
                }

                // check if not enabled
                if(!tempVariable1.boolVar)
                {
                    cli_printfError("Sleep Current th disabled!\n");
                }

                // reset the intvalue
                tempVariable1.uint16Var = 0;

                // get the status of the sequence tasks
                retValue = checkSequenceTaskStatus(&tempVariable2.boolVar);

                // make sure it finished stuff
                while(!retValue && tempVariable2.boolVar && tempVariable1.uint16Var < 100)
                {
                    // sleep for a little while to make sure the other tasks can do their things
                    usleep(1 * 1000UL);

                    // get the status of the sequence tasks
                    retValue = checkSequenceTaskStatus(&tempVariable2.boolVar);

                    // increase the intvalue
                    tempVariable1.uint16Var++;
                }

                // check if there was an error
                if(retValue)
                {
                    cli_printfError("main ERROR: couldn't get measurement status, waiting for some time\n");
                    // sleep long enough so the measurements are done
                    usleepMainLoopWatchdog(7500);
                }

                // check if the timeout happend
                if(tempVariable1.uint16Var >= 100)
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

                // cli_printfWarning("AFE not to sleep mode\n");

                // set the SBC to standby mode
                if(sbc_setSbcMode(SBC_STANDBY))
                {
                    // error
                    cli_printfError("main ERROR: failed to set SBC to standby mode!\n");
                }

                // cli_printfWarning("SBC not to sleep mode\n");

                cli_printf("SLEEP mode\n");

                // sleep for a small amount of time to output the things on the CLI
                usleepMainLoopWatchdog(1);

                // change the MCU to VLPR mode (VLPR with only UART console enabled)
                if(power_setNGetMcuPowerMode(true, VLPR_MODE) == ERROR_VALUE)
                {
                    // if error
                    cli_printfError("main ERROR: failed to put the system in SLEEP mode!\n");
                    // error and stop main loop
                    stopMainLoop();
                }
            }

            // get the OCV cyclic timer time
            if(getOcvPeriodTime(&tempVariable1.int32Var, *pOldState))
            {
                cli_printfError("main ERROR: failed to get OCV time!\n");
            }

            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
            }

            // check for the OCV state transition
            if((currentTime.tv_sec - sampleTime.tv_sec) > tempVariable1.int32Var)
            {
                // go to the OCV state
                setMainState(OCV);
            }

            // get the sleep timeout variable
            if(data_getParameter(T_SLEEP_TIMEOUT, &tempVariable1.uint8Var, NULL) == NULL)
            {
                cli_printfError("main ERROR: getting sleep timeout went wrong!\n");
                tempVariable1.uint8Var = T_SLEEP_TIMEOUT_DEFAULT;
            }

            // check if the sleep timeout shouldn't be skipped
            if(tempVariable1.uint8Var != 0)
            {
                // check if the timtout time has passed
                if((sampleTime2.tv_sec + (tempVariable1.uint8Var * 60 * 60)) < currentTime.tv_sec)
                {
                    // output to the user
                    cli_printf("sleep timeout happend after %d hours, going to deepsleep %ds\n",
                        tempVariable1.uint8Var, currentTime.tv_sec);

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
            if(*pDeepsleepTimingOn)
            {
                // get the current time
                if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get currentTime in sleep! \n");
                }

                // check if the right amount of time has passed
                if(((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) < currentTime.tv_sec) ||
                    (((pButtonPressedTime->tv_sec + BUTTON_TIME_FOR_DEEP_SLEEP) == currentTime.tv_sec) &&
                        pButtonPressedTime->tv_nsec <= currentTime.tv_nsec))
                {
                    // go to the deep sleep state
                    setMainState(SELF_DISCHARGE);
                }
            }
        }

        break;
        case OCV:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // set the leds to be green blinking, wake up
                ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                cli_printf("OCV mode\n");

                // change the MCU to RUN mode
                if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
                {
                    cli_printfError("main ERROR: failed to put the system in RUN mode!\n");
                    // error and stop main loop
                    stopMainLoop();
                }

                // set the AFE mode to normal
                batManagement_setAFEMode(AFE_NORMAL);

                // Enable the I2C
                if(i2c_enableTransmission(true))
                {
                    cli_printfError("main ERROR: failed to enable I2C!\n");
                }

                // read and reset the CC registers by calculating a new remaining charge
                batManagement_calcRemaningCharge(NULL);

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
        }

        break;
        case FAULT_ON:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // get the in flight status variable
                if(data_getParameter(S_IN_FLIGHT, &tempVariable1.uint8Var, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting flight mode enable went wrong!\n");
                    tempVariable1.uint8Var = S_IN_FLIGHT_DEFAULT;
                }

                // check if in flight (with flight-mode enabled) is enabled
                // to not disable the power if there is no peak overcurrent
                if(tempVariable1.uint8Var)
                {
                    // check if there are faults
                    batManagement_checkFault(&BMSFault, 0);

                    // check if there is a peak over current
                    if(BMSFault & BMS_PEAK_OVER_CURRENT)
                    {
                        // go to fault state to disconnect switch
                        setMainState(FAULT_OFF);

                        // increase the main loop semaphore to not wait
                        if(escapeMainLoopWait())
                        {
                            cli_printfError("main ERROR: Couldn't up mainloop sem!\n");
                        }

                        // break the remaining FAULT_ON sequence
                        break;
                    }
                    else
                    {
                        // get the s-out parameter
                        if(data_getParameter(S_OUT, &tempVariable1.uint8Var, NULL) == NULL)
                        {
                            cli_printfError("main ERROR: getting flight mode enable went wrong!\n");
                            tempVariable1.uint8Var = S_OUT_DEFAULT;
                        }

                        // check if the output power is enabled
                        if(tempVariable1.uint8Var)
                        {
                            // output warning to the user
                            cli_printfWarning(
                                "WARNING: Couldn't disconnect power: flight mode enabled and in flight\n");
                        }
                    }
                }
                // if not in flight (or flight mode is not enabled)
                else
                {
                    // if not in flight, disconnect switch
                    // go to fault state to disconnect switch
                    setMainState(FAULT_OFF);

                    // increase the main loop semaphore to not wait
                    if(escapeMainLoopWait())
                    {
                        cli_printfError("main ERROR: Couldn't up mainloop sem!\n");
                    }

                    // break the remaining FAULT_ON sequence
                    break;
                }

                // set the LED to red
                ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

                // make sure MCU is in normal mode, set SBC and AFE to normal mode
                setMcuAfeSbcToNormalMode(false);

                // reset the command variable, balancing and charge mode
                resetVariables();

                // enable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(true, true);

                cli_printf("FAULT_ON mode\n");

                // set the outputMessageOnlyOnce value to output the message only once
                outputMessageOnlyOnce = true;

                // check if there was a cell undervoltage
                if(*pCellUnderVoltageDetected)
                {
                    // get the cellUnderVoltageTime
                    if(clock_gettime(CLOCK_REALTIME, &cellUnderVoltageTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get cellUnderVoltageTime in FAULT_ON!\n");
                    }
                }

            } // if mainState != *pOldState

            // check if the s-in-flight changed to false
            if(gSInFlightChangedFalse)
            {
                // reset both variables
                gSInFlightChangedFalse = false;

                // go to fault state to disconnect switch
                setMainState(FAULT_OFF);

                // increase the main loop semaphore to not wait
                if(escapeMainLoopWait())
                {
                    cli_printfError("main ERROR: Couldn't up mainloop sem!\n");
                }

                // break the remaining FAULT_ON sequence
                break;
            }

            // check if there is no cell undervoltage
            if(!*pCellUnderVoltageDetected)
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
                    *pCellUnderVoltageDetected = true;

                    // output to the user
                    cli_printfError("cell undervoltage detected!\n");
                }
            }

            // check if a cell undervoltage occured
            if(*pCellUnderVoltageDetected)
            {
                // get the fault timeout time
                if(data_getParameter(T_FAULT_TIMEOUT, &tempVariable1.uint16Var, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting sleepcurrent went wrong!\n");
                    tempVariable1.uint16Var = T_FAULT_TIMEOUT_DEFAULT;
                }

                // check if the fault timeout is not 0
                if(tempVariable1.uint16Var != 0)
                {
                    // check if it needs to be outputted
                    if(outputMessageOnlyOnce)
                    {
                        // output to the user
                        cli_printfWarning("WARNING: starting fault timer to go to deepsleep after %ds\n",
                            tempVariable1.uint16Var);

                        // set the outputMessageOnlyOnce false to only output this once
                        outputMessageOnlyOnce = false;
                    }

                    // get the current time
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get currentTime in fault!\n");
                    }

                    // check if the right amount of time has passed
                    if(((cellUnderVoltageTime.tv_sec + tempVariable1.uint16Var) < currentTime.tv_sec) ||
                        (((cellUnderVoltageTime.tv_sec + tempVariable1.uint16Var) == currentTime.tv_sec) &&
                            cellUnderVoltageTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // go to the DEEP_SLEEP state
                        setMainState(DEEP_SLEEP);
                    }
                }
                else
                {
                    // check if it needs to be outputted
                    if(outputMessageOnlyOnce)
                    {
                        // output to the user
                        cli_printf("fault timer disabled!\n");

                        // set the outputMessageOnlyOnce false to only output this once
                        outputMessageOnlyOnce = false;
                    }
                }
            }
            // other transitions are done from the button press check part
        }
        break;
        case FAULT_OFF:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // Turn off the power
                if(batManagement_setGatePower(GATE_OPEN) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");

                    // go to the correct state
                    setMainState(FAULT_ON);

                    // set the LED to red
                    ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

                    // escape the rest of the FAULT_OFF state
                    break;
                }
                else
                {
                    // set the LED to red blinking
                    ledState_setLedColor(RED, OFF, LED_BLINK_ON);
                }

                // make sure MCU is in normal mode, set SBC and AFE to normal mode
                setMcuAfeSbcToNormalMode(false);

                // reset the command variable, balancing and charge mode
                resetVariables();

                // enable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(true, true);

                cli_printf("FAULT_OFF mode\n");

                // set the outputMessageOnlyOnce value to output the message only once
                outputMessageOnlyOnce = true;

                // check if there was a cell undervoltage
                if(*pCellUnderVoltageDetected)
                {
                    // get the cellUnderVoltageTime
                    if(clock_gettime(CLOCK_REALTIME, &cellUnderVoltageTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get cellUnderVoltageTime in FAULT_OFF!\n");
                    }
                }

            } // if mainState != *pOldState

            // check for cell undervoltage
            // check if there is no cell undervoltage
            if(!*pCellUnderVoltageDetected)
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
                    *pCellUnderVoltageDetected = true;

                    // output to the user
                    cli_printfError("cell undervoltage detected!\n");
                }
            }

            // check if a cell undervoltage occured
            if(*pCellUnderVoltageDetected)
            {
                // get the fault timeout time
                if(data_getParameter(T_FAULT_TIMEOUT, &tempVariable1.uint16Var, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting sleepcurrent went wrong!\n");
                    tempVariable1.uint16Var = T_FAULT_TIMEOUT_DEFAULT;
                }

                // check if the fault timeout is not 0
                if(tempVariable1.uint16Var != 0)
                {
                    // check if it needs to be outputted
                    if(outputMessageOnlyOnce)
                    {
                        // output to the user
                        cli_printfWarning("WARNING: starting fault timer to go to deepsleep after %ds\n",
                            tempVariable1.uint16Var);

                        // set the outputMessageOnlyOnce false to only output this once
                        outputMessageOnlyOnce = false;
                    }

                    // get the current time
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("main ERROR: failed to get currentTime in fault!\n");
                    }

                    // check if the right amount of time has passed
                    if(((cellUnderVoltageTime.tv_sec + tempVariable1.uint16Var) < currentTime.tv_sec) ||
                        (((cellUnderVoltageTime.tv_sec + tempVariable1.uint16Var) == currentTime.tv_sec) &&
                            cellUnderVoltageTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // go to the DEEP_SLEEP state
                        setMainState(DEEP_SLEEP);
                    }
                }
                else
                {
                    // check if it needs to be outputted
                    if(outputMessageOnlyOnce)
                    {
                        // output to the user
                        cli_printf("fault timer disabled!\n");

                        // set the outputMessageOnlyOnce false to only output this once
                        outputMessageOnlyOnce = false;
                    }
                }
            }
            // other transitions are done from the button press check part
        }
        break;
        case SELF_DISCHARGE:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

                // make sure MCU is in normal mode, set SBC and AFE to normal mode
                setMcuAfeSbcToNormalMode(false);

                // check if the self discharge should be done

                // get the self discharge enable parameter
                if(data_getParameter(SELF_DISCHARGE_ENABLE, &tempVariable1.uint8Var, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting self discharge var went wrong! \n");
                    tempVariable1.uint8Var = SELF_DISCHARGE_ENABLE_DEFAULT;
                }

                // check if it should be done
                if(!tempVariable1.uint8Var)
                {
                    // go to the DEEP_SLEEP state
                    setMainState(DEEP_SLEEP);

                    break;
                }

                // get the self discharge start time
                if(clock_gettime(CLOCK_REALTIME, pSelfDischargeTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get selfDischargeTime! \n");
                }

                // turn off the gate
                if(batManagement_setGatePower(GATE_OPEN) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");
                }

                // enable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(true, true);

                // set the LED to magenta(purple) blinking
                ledState_setLedColor(BLUE, OFF, LED_BLINK_ON);

                // reset the command variable, balancing and charge mode
                resetVariables();

                // wait until the balancing is done
                while(batManagement_getBalanceState() != BALANCE_OFF)
                {
                    // kick the watchdog and sleep for 100us
                    usleepMainLoopWatchdog(100);
                }

                // set self discharge to storage on
                batManagement_setBalanceState(BALANCE_TO_STORAGE);

                cli_printf("SELF_DISCHARGE mode\n");

                // get the bms timeout variable
                if(data_getParameter(T_BMS_TIMEOUT, &bmsTimeoutTime, NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting bms timeout var went wrong! \n");
                    bmsTimeoutTime = T_BMS_TIMEOUT_DEFAULT;
                }
            }

            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime!\n");
            }

            // check if the right amount of time has passed (precision is not needed)
            if((pSelfDischargeTime->tv_sec + bmsTimeoutTime) < currentTime.tv_sec)
            {
                // calibrate the state of charge
                if(batManagement_calibrateStateOfCharge(true))
                {
                    // output error
                    cli_printfError("main ERROR: failed to calibrate state of charge!\n");
                }

                // add one second to it for the next calibration
                bmsTimeoutTime = bmsTimeoutTime + 1;
            }

            // get the self discharge enable parameter
            if(data_getParameter(SELF_DISCHARGE_ENABLE, &tempVariable1.uint8Var, NULL) == NULL)
            {
                cli_printfError("main ERROR: getting self discharge var went wrong! \n");
                tempVariable1.uint8Var = SELF_DISCHARGE_ENABLE_DEFAULT;
            }

            // check if it should be done
            if(!tempVariable1.uint8Var)
            {
                // go to the deepsleep state
                setMainState(DEEP_SLEEP);
            }

            // check if cell balancing is done
            if(batManagement_getBalanceState() == BALANCE_OFF)
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
        }
        break;
        case DEEP_SLEEP:
        {
            // check if the state has changed to not do this everytime
            if(mainState != *pOldState)
            {
                // save the old value
                *pOldState = mainState;

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

                // reset the command variable, balancing and charge mode
                resetVariables();

                // disable the battery management task and the updates (I2C, NFC, CAN, display)
                enableUpdatesAndBatManagementTask(false, true);

                cli_printf("DEEP_SLEEP mode\n");
            }

            // break until the button is released
            if(*pDeepsleepTimingOn)
            {
                break;
            }

            // wait for 1s

            // kick the watchdog and sleep for 500ms (in case 1s watchdog)
            usleepMainLoopWatchdog(500 * 1000);

            // kick the watchdog and sleep for 500ms
            usleepMainLoopWatchdog(500 * 1000);

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
            retValue = data_saveParameters();

            // turn the LED off
            ledState_setLedColor(OFF, OFF, LED_BLINK_OFF);

            // cli_printf("setting SBC to standby!\n");
            cli_printf("setting SBC to sleep!\n");

            // MCU (3.3V and 5V) off mode
            retValue = sbc_setSbcMode(SBC_SLEEP);

            // go to the init state if possible (if you come here the sleep didn't work)
            setMainState(INIT);
        }
        break;

        // if it is the SELF_TEST state
        case(SELF_TEST):
        {

            // shouldn't come here
            cli_printfError("main ERROR: Main loop is in SLEF_TEST state\n");
            cli_printf("setting init mode\n");

            // set the init mode
            setMainState(INIT);
        }
        break;
        case NUMBER_OF_MAIN_STATES: // do nothing, for compiler
        {
            cli_printfError("main ERROR: Main loop is in NUMBER_OF_MAIN_STATES state\n");
            // set the init mode
            setMainState(INIT);
        }
        break;
    }
    return;
}

/*!
 * @brief   Function that is used take care of the charge state machine.
 *
 * @param   pOldState address of the oldState variable
 * @param   chargeToStorage the variable that indicates if charge to storage is active
 *
 * @return  none
 */
static void chargeStateMachine(charge_states_t *pOldChargeState, bool chargeToStorage)
{
    static struct timespec sampleTime             = { 0 };
    static uint8_t         amountOfCBChargeCycles = 0;
    static bool            onlyOnce               = false;
    struct timespec        currentTime;
    bool                   outputCBStatus = false;
    variableTypes_u        tempVariable1, tempVariable2, tempVariable3;
    charge_states_t        chargeState = getChargeState();

    // check for null pointers in debug mode
    DEBUGASSERT(pOldChargeState != NULL);

    // check if the charge state changed
    if(chargeState != *pOldChargeState)
    {
        // check which charge state it is in
        switch(chargeState)
        {
            // in case of the beginning
            case CHARGE_START:
            {

                // reset the variable for the CB charge cycles
                amountOfCBChargeCycles = 0;

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

                cli_printf("Charge start %ds %dms\n", sampleTime.tv_sec, sampleTime.tv_nsec / 1000000);
            }
            break;

            // in case of charging with balancing
            case CHARGE_CB:
            {
                // turn off the gate
                if(batManagement_setGatePower(GATE_CLOSE) != 0)
                {
                    cli_printfError("main ERROR: Failed to open gate\n");
                }

                // enable cell balancing
                batManagement_setBalanceState(BALANCE_TO_LOWEST_CELL);

                // set the LED to blue blinking
                ledState_setLedColor(BLUE, OFF, LED_BLINK_ON);

                // set the end of charge variable to 0 to check for current and voltage
                batManagement_SetNReadEndOfCBCharge(true, 0);

                // save the time
                if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get sampleTime! \n");
                }

                cli_printf("Charge with CB %ds %dms\n", sampleTime.tv_sec, sampleTime.tv_nsec / 1000000);

                // increase the counter
                amountOfCBChargeCycles++;

                // make sure it will only output CB done once
                outputCBStatus = true;

                // get the n-charges-full value and increment it once
                if(data_getParameter(N_CHARGES, &(tempVariable1.uint16Var), NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting n-charges went wrong! \n");

                    // set it to the default value just in case
                    tempVariable1.uint16Var = N_CHARGES_DEFAULT;
                }

                // increament and limit it
                tempVariable1.uint16Var = (tempVariable1.uint16Var + 1);

                // set the number of charger with the new value
                if(data_setParameter(N_CHARGES, &(tempVariable1.uint16Var)))
                {
                    cli_printfError("main ERROR: couldn't set n-charges!\n");
                }
            }
            break;

            // in case of relaxing
            case RELAXATION:
            {
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

                cli_printf("Charge RELAXATION %ds %dms\n", sampleTime.tv_sec, sampleTime.tv_nsec / 1000000);

                // make sure it will only output CB done once
                outputCBStatus = true;

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
                SMBus_updateInformation(true, NULL, NULL);

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

                // wait until the updater task is done
                // reset the variable
                tempVariable1.uint16Var = 0;

                // make sure it finished stuff in the updater task
                while(getUpdaterTaskStatus() && tempVariable1.uint16Var < 100)
                {
                    // sleep for a little while to make sure the other tasks can do their things
                    usleep(1 * 1000UL);

                    // increase the intvalue, to have a timeout
                    tempVariable1.uint16Var++;
                }

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
                        // error and stop main loop
                        stopMainLoop();
                    }
                }
            }
            break;

            // in case the charging is complete
            case CHARGE_COMPLETE:
            {
                // save the time
                if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                {
                    cli_printfError("main ERROR: failed to get sampleTime! \n");
                }

                // set the LED to green
                ledState_setLedColor(GREEN, OFF, LED_BLINK_OFF);

                // turn off cell balancing function
                batManagement_setBalanceState(BALANCE_OFF);

                // make sure it doens't keep checking
                batManagement_SetNReadEndOfCBCharge(true, 3);

                cli_printf("Charge complete %ds %dms\n", sampleTime.tv_sec, sampleTime.tv_nsec / 1000000);

                // check if charging to storage is not on
                // Only save the full-charge capacity and increment
                // The number of full charges if it is a full charge
                if(!chargeToStorage)
                {
                    // get the n-charges-full value and increment it once
                    if(data_getParameter(N_CHARGES_FULL, &(tempVariable1.uint16Var), NULL) == NULL)
                    {
                        cli_printfError("main ERROR: getting n-charges-full went wrong! \n");
                        tempVariable1.uint16Var = N_CHARGES_FULL_DEFAULT;
                    }

                    // increament and limit it
                    tempVariable1.uint16Var = (tempVariable1.uint16Var + 1);

                    // set the incremented one
                    if(data_setParameter(N_CHARGES_FULL, &(tempVariable1.uint16Var)))
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
            }
            break;
            case NUMBER_OF_CHARGE_STATES:
            {
                cli_printfError("main ERROR: Not in correct charge state!!\n");
            }
            break;
        }

        // save the state, so it wont be entered again
        *pOldChargeState = chargeState;
    }

    // check for charge state transistions
    switch(chargeState)
    {
        // in case of the beginning
        case CHARGE_START:
        {
            // if the cell is full the state needs to change
            if((batManagement_SetNReadEndOfCBCharge(false, 0) & 2) == 2)
            {
                // make sure it doens't keep checking
                batManagement_SetNReadEndOfCBCharge(true, 3);

                // set the next charge state
                setChargeState(CHARGE_CB);
            }

            // get the CB begin time
            if(data_getParameter(T_CB_DELAY, &(tempVariable1.uint8Var), NULL) == NULL)
            {
                cli_printfError("main ERROR: getting CB delay went wrong! \n");
                tempVariable1.uint8Var = T_CB_DELAY_DEFAULT;
            }

            // start the charging timer
            // check the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime! \n");
            }

            // check if the charge time ended and the charge is begon
            if((((sampleTime.tv_sec + tempVariable1.uint8Var) == currentTime.tv_sec) &&
                   (sampleTime.tv_nsec <= currentTime.tv_nsec)) ||
                ((sampleTime.tv_sec + tempVariable1.uint8Var) < currentTime.tv_sec))
            {
                // cli_printf("ended charge start %ds %dms\n", currentTime.tv_sec,
                // currentTime.tv_nsec/1000000);
                // set the next charge state
                setChargeState(CHARGE_CB);
            }
        }
        break;

        // in case of charging with balancing
        case CHARGE_CB:
        {
            // check if the current or cell voltage is met
            tempVariable1.int32Var = batManagement_SetNReadEndOfCBCharge(false, 0);

            // check current and cell voltages
            if(tempVariable1.int32Var)
            {
                // check if it is done due to the voltage requirement
                if(tempVariable1.int32Var == 2)
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
            if(batManagement_getBalanceState() == BALANCE_OFF)
            {
                // set the LED to blue
                ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                // check if the output has been done
                if(outputCBStatus)
                {
                    // output to the user
                    cli_printf("CB is done\n");

                    // make sure it will only do this once
                    outputCBStatus = false;
                }
            }
        }
        break;

        // in case of relaxing
        case RELAXATION:
        {
            // check if the charger is removed
            if(!batManagement_checkOutputVoltageActive())
            {
                // go to the sleep state by setting the transion variable true
                setTransitionVariable(SLEEP_VAR, true);

                // escape the transition switch
                break;
            }

            // check for CB is done
            if(batManagement_getBalanceState() == BALANCE_OFF)
            {
                // set the LED to blue
                ledState_setLedColor(BLUE, OFF, LED_BLINK_OFF);

                // check if the output has been done
                if(outputCBStatus)
                {
                    // output to the user
                    cli_printf("CB is done\n");

                    // make sure it will only do this once
                    outputCBStatus = false;
                }
            }

            // check for timeout
            // get the relax time
            if(data_getParameter(T_CHARGE_RELAX, &(tempVariable1.uint16Var), NULL) == NULL)
            {
                cli_printfError("main ERROR: getting relax time went wrong! \n");
                tempVariable1.uint16Var = T_CHARGE_RELAX_DEFAULT;
            }

            // check the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("main ERROR: failed to get currentTime! \n");
            }

            // check if the charge time ended and the charge is begon
            if((((sampleTime.tv_sec + tempVariable1.uint16Var) == currentTime.tv_sec) &&
                   (sampleTime.tv_nsec <= currentTime.tv_nsec)) ||
                ((sampleTime.tv_sec + tempVariable1.uint16Var) < currentTime.tv_sec))
            {
                // check if CB is done
                if(batManagement_getBalanceState() == BALANCE_OFF)
                {
                    // check if onlyOnce is still true
                    if(onlyOnce)
                    {
                        onlyOnce = false;
                        cli_printf(
                            "Ended relaxing! %ds %dms\n", currentTime.tv_sec, currentTime.tv_nsec / 1000000);
                    }

                    // get the cell margin in mv
                    if(data_getParameter(V_CELL_MARGIN, &(tempVariable1.uint8Var), NULL) == NULL)
                    {
                        cli_printfError("main ERROR: getting cell margin went wrong! \n");
                        tempVariable1.uint8Var = V_CELL_MARGIN_DEFAULT;
                    }

                    // check if charge to storage is not enabled
                    if(!chargeToStorage)
                    {
                        // get the cell over voltage
                        if(data_getParameter(V_CELL_OV, &(tempVariable2.floatVar), NULL) == NULL)
                        {
                            cli_printfError("main ERROR: getting cell over voltage went wrong! \n");
                            tempVariable2.floatVar = V_CELL_OV_DEFAULT;
                        }
                    }
                    // if it should charge to storage voltage
                    else
                    {
                        // get the storage voltage
                        if(data_getParameter(V_STORAGE, &(tempVariable2.floatVar), NULL) == NULL)
                        {
                            cli_printfError("main ERROR: getting storage voltage went wrong! \n");
                            tempVariable2.floatVar = V_STORAGE_DEFAULT;
                        }
                    }

                    // check check if highest cell voltage is smaller than the to charge voltage - margin
                    if((amountOfCBChargeCycles < AMOUNT_CB_CHARGE_CYCLES_MAX) &&
                        (batManagement_getHighestCellV() <
                            (tempVariable2.floatVar - ((float)(tempVariable1.uint8Var) / 1000))))
                    {
                        // output the equation to the user why it did go back
                        cli_printf("Continue charging: highest cell: %.3f < %.3f\n",
                            batManagement_getHighestCellV(),
                            (tempVariable2.floatVar - ((float)(tempVariable1.uint8Var) / 1000)));

                        // kick the watchdog before the task yield
                        if(sbc_kickTheWatchdog())
                        {
                            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                        }

                        // turn off the measurements to be able to do a switch
                        batManagement_enableBatManagementTask(false);

                        // reset the intvalue
                        tempVariable1.uint8Var = 0;

                        // get the status of the sequence tasks
                        tempVariable2.int32Var = checkSequenceTaskStatus(&tempVariable3.boolVar);

                        // make sure it finished stuff
                        while(
                            !tempVariable2.int32Var && tempVariable3.boolVar && tempVariable1.uint8Var < 100)
                        {
                            // sleep for a little while to make sure the other tasks can do their things
                            usleep(1 * 1000UL);

                            // get the status of the sequence tasks
                            tempVariable2.int32Var = checkSequenceTaskStatus(&tempVariable3.boolVar);

                            // increase the intvalue
                            tempVariable1.uint8Var++;
                        }

                        cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                        // make sure MCU is in normal mode, set SBC and AFE to normal mode
                        setMcuAfeSbcToNormalMode(false);

                        // enable the battery management task and the updates (I2C, NFC, CAN, display)
                        enableUpdatesAndBatManagementTask(true, true);

                        // go back to charge with CB
                        setChargeState(CHARGE_CB);
                    }
                    else
                    {
                        // check if 5 times has passed and that is why it transitioned
                        if(amountOfCBChargeCycles >= AMOUNT_CB_CHARGE_CYCLES_MAX)
                        {
                            // send the message to the user
                            cli_printf(
                                "%d charge cycles done, skipping voltage requirement! highest cell: %.3fV\n",
                                amountOfCBChargeCycles, batManagement_getHighestCellV());
                        }

                        // kick the watchdog before the task yield
                        if(sbc_kickTheWatchdog())
                        {
                            cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
                        }

                        // turn off the measurements to be able to do a switch
                        batManagement_enableBatManagementTask(false);

                        // reset the intvalue
                        tempVariable1.uint8Var = 0;

                        // get the status of the sequence tasks
                        tempVariable2.int32Var = checkSequenceTaskStatus(&tempVariable3.boolVar);

                        // make sure it finished stuff
                        while(
                            !tempVariable2.int32Var && tempVariable3.boolVar && tempVariable1.uint8Var < 100)
                        {
                            // sleep for a little while to make sure the other tasks can do their things
                            usleep(1 * 1000UL);

                            // get the status of the sequence tasks
                            tempVariable2.int32Var = checkSequenceTaskStatus(&tempVariable3.boolVar);

                            // increase the intvalue
                            tempVariable1.uint8Var++;
                        }

                        cli_printf("Setting MCU back to RUN mode and CAN (5V) on\n");

                        // make sure MCU is in normal mode, set SBC and AFE to normal mode
                        setMcuAfeSbcToNormalMode(false);

                        // enable the battery management task and the updates (I2C, NFC, CAN, display)
                        enableUpdatesAndBatManagementTask(true, true);

                        // go to charging complete
                        setChargeState(CHARGE_COMPLETE);
                    }
                }
                // if cell balancing is not done
                else
                {
                    // only re-calculate the Balance minutes after the relax time
                    if(onlyOnce)
                    {
                        // reset the variable to only do this once
                        onlyOnce = false;

                        // end of relaxation time output
                        cli_printf("End of relax time, re-estimating balance minutes\n");

                        // enable cell balancing again
                        batManagement_setBalanceState(BALANCE_TO_LOWEST_CELL);

                        // set the LED to blue blinking
                        ledState_setLedColor(BLUE, OFF, LED_BLINK_ON);

                        // sleep for a small amount to make sure it started
                        usleepMainLoopWatchdog(1000);
                    }
                }
            }
        }
        break;

        // in case the charging is complete
        case CHARGE_COMPLETE:
        {
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
                if(data_getParameter(V_RECHARGE_MARGIN, &(tempVariable1.uint16Var), NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting v-recharge-margin went wrong!\n");
                    tempVariable1.uint16Var = V_RECHARGE_MARGIN_DEFAULT;
                }

                // get the v-cell-ov
                if(data_getParameter(V_CELL_OV, &(tempVariable2.floatVar), NULL) == NULL)
                {
                    cli_printfError("main ERROR: getting v-cell-ov went wrong! \n");
                    tempVariable2.floatVar = V_CELL_OV_DEFAULT;
                }

                // check if the lowest cell voltage is
                // then the cell overvoltage minus this margin
                if(batManagement_getLowestCellV() <
                    (tempVariable2.floatVar - ((float)tempVariable1.uint16Var / 1000.0)))
                {
                    // start charging again
                    cli_printf("Recharging: lowest cell: %.3fv < v-cell-ov - (v-recharge-margin: %dmV)\n",
                        batManagement_getLowestCellV(), tempVariable1.uint16Var);

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
        }
        break;
        case NUMBER_OF_CHARGE_STATES:
        {
            cli_printfError("main ERROR: not in a correct charge state!\n");
        }
        break;
    }
}

/*!
 * @brief   Function to check for new state transistions based on the current
 *
 * @param   currentA Address of the just measured battery current in A.
 *          If a NULL pointer is given, it will reset the timers and variables.
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int checkForTransitionCurrent(float *currentA)
{
    static bool            timeOutTimeStarted = false;
    static bool            chargeTimeStarted  = false;
    static struct timespec savedTime          = { .tv_sec = 0, .tv_nsec = 0 };
    int                    retValue           = 0;
    struct timespec        currentTime;
    variableTypes_u        variable1;

    // lock the mutex
    pthread_mutex_lock(&gTransVarLock);
    // do not use setTransitionVariable(CHAR_VAR, true), setTransitionVariable(SLEEP_VAR, true),
    // setTransitionVariable(DISCHAR_VAR, true) since the mutex is locked

    // check if a NULL pointer is given to reset the variables
    if(currentA == NULL)
    {
        // reset the charge variables
        gChargeDetected   = false;
        chargeTimeStarted = false;

        // reset the sleep variable
        gSleepDetected = false;

        // reset the timeoutstarted variable
        timeOutTimeStarted = false;

        // set the discharge variable
        gDischargeDetected = false;
    }
    else
    {
        // get the sleep current
        if(data_getParameter(I_SLEEP_OC, &(variable1.uint8Var), NULL) == NULL)
        {
            cli_printfError("checkForTransitionCurrent ERROR: getting i-sleep-oc went wrong!\n");

            // set it with default, just in case
            variable1.uint8Var = I_SLEEP_OC_DEFAULT;

            // set the return variable to -1
            retValue = -1;
        }

        // check for a discharge
        if((((*currentA) * 1000)) <= -(variable1.uint8Var))
        {
            // reset the charge variables
            gChargeDetected   = false;
            chargeTimeStarted = false;

            // reset the sleep variable
            gSleepDetected = false;

            // reset the timeoutstarted variable
            timeOutTimeStarted = false;

            // set the discharge variable
            gDischargeDetected = true;
        }
        // check for a charge
        else if((((*currentA) * 1000)) >= variable1.uint8Var)
        {
            // reset the timeoutstarted variable
            timeOutTimeStarted = false;

            // reset the sleep variable
            gSleepDetected = false;

            // reset the discharge variable
            gDischargeDetected = false;

            // check if the battery is in the normal state
            if(getMainState() == NORMAL)
            {
                // check if the charge time has started
                if(!chargeTimeStarted)
                {
                    // save the time in the savedTime variable
                    if(clock_gettime(CLOCK_REALTIME, &savedTime) == -1)
                    {
                        cli_printfError("checkForTransitionCurrent ERROR: failed to get chargeBeginTime! \n");
                    }

                    // set the variable
                    chargeTimeStarted = true;
                }
                else if(chargeTimeStarted && !gChargeDetected)
                {
                    // get the charge detect time
                    if(data_getParameter(T_CHARGE_DETECT, &(variable1.uint8Var), NULL) == NULL)
                    {
                        cli_printfError(
                            "checkForTransitionCurrent ERROR: getting t-charge-detect went wrong!\n");

                        // set it with default, just in case
                        variable1.uint8Var = T_CHARGE_DETECT_DEFAULT;

                        // set the return variable to -1
                        retValue = -1;
                    }

                    // check if the time passed
                    // check the current time
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError("checkForTransitionCurrent ERROR: failed to get current time! \n");
                    }

                    // check if the time passed
                    if(((savedTime.tv_sec + variable1.uint8Var) < currentTime.tv_sec) ||
                        (((savedTime.tv_sec + variable1.uint8Var) == currentTime.tv_sec) &&
                            savedTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // set the variable
                        gChargeDetected = true;
                    }
                }
            }

            // if not in normal state
            else
            {
                // reset the variable
                chargeTimeStarted = false;
                gChargeDetected   = false;
            }
        }
        // check for a sleep current if abs(i-batt) < i-sleep-oc
        else
        {
            // reset the charge variables
            chargeTimeStarted = false;

            // reset the discharge variable
            gDischargeDetected = false;

            // check if the battery is in the normal state
            if(getMainState() == NORMAL)
            {
                // check if the timeout time has started
                if(!timeOutTimeStarted)
                {
                    // save the time
                    if(clock_gettime(CLOCK_REALTIME, &savedTime) == -1)
                    {
                        cli_printfError(
                            "checkForTransitionCurrent ERROR: failed to get timeoutBeginTime! \n");
                    }

                    // set the variable
                    timeOutTimeStarted = true;
                }
                else if(timeOutTimeStarted && !gSleepDetected)
                {
                    // get time bms timeoutTime
                    if(data_getParameter(T_BMS_TIMEOUT, &(variable1.uint16Var), NULL) == NULL)
                    {
                        cli_printfError(
                            "checkForTransitionCurrent ERROR: getting t-bms-timeout went wrong!\n");

                        // set it with default, just in case
                        variable1.uint16Var = T_BMS_TIMEOUT_DEFAULT;

                        // set the return variable to -1
                        retValue = -1;
                    }

                    // check if the time passed
                    // check the current time
                    if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
                    {
                        cli_printfError(
                            "checkForTransitionCurrent ERROR: failed to get timeoutBeginTime! \n");
                    }

                    // check if the time passed
                    if(((savedTime.tv_sec + variable1.uint16Var) < currentTime.tv_sec) ||
                        (((savedTime.tv_sec + variable1.uint16Var) == currentTime.tv_sec) &&
                            savedTime.tv_nsec <= currentTime.tv_nsec))
                    {
                        // set the variable
                        gSleepDetected = true;
                    }
                }
            }
            // // when in the charge mode, make sure to transition to sleep mode if the charge current is 0 or
            // less
            else if(getMainState() == CHARGE)
            {
                // check if the current is 0 or less (no charging / charger disconnected)
                // but only in the charge states where the gate is closed
                if((((*currentA) * 1000) <= 0) &&
                    ((getChargeState() == CHARGE_START) || (getChargeState() == CHARGE_CB)))
                {
                    // check if the gate is already closed (comming from relaxation)
                    // this variable is set to 0 when the charge_CB state has entered and
                    // after the gate is closed, if it isn't 0 is should sleep anyway.
                    if((batManagement_SetNReadEndOfCBCharge(false, 0) == 0) ||
                        (getChargeState() != CHARGE_CB))
                    {
                        // set the sleep variable to go to the sleep state if the charger
                        // is disconnected
                        gSleepDetected     = true;
                        timeOutTimeStarted = true;
                        cli_printf("No charge current: %.3fA <= 0A\n", (*currentA));

                        // unlock the mutex to not stall the main task
                        pthread_mutex_unlock(&gTransVarLock);

                        // increase the main loop semaphore to not wait on it
                        if(escapeMainLoopWait())
                        {
                            cli_printfError("checkForTransitionCurrent ERROR: Couldn't up mainloop sem!\n");
                        }

                        // lock the mutex again to unlock later
                        pthread_mutex_lock(&gTransVarLock);
                    }
                }
                else
                {
                    // reset the sleep variable
                    timeOutTimeStarted = false;
                    gSleepDetected     = false;
                }
            }
            // make sure to reset the sleep variable if in any other state (except OCV)
            else if(getMainState() != SLEEP && getMainState() != OCV)
            {
                // set the variable
                timeOutTimeStarted = false;
                gSleepDetected     = false;
            }
        }
    }

    // unlock the mutex
    pthread_mutex_unlock(&gTransVarLock);

    return retValue;
}


/*!
 * @brief   Function to handle a parameter change which is not accesable from data.c
 *
 * @warning May only be called from data.c.
 * @warning Do not use the data_setParameter() and data_getParameter() functions.
 *
 * @param   parameter The parameter that changed.
 * @param   value Address of the variable containing the new value.
 * @param   extraValue Address of the extra value that can be used, may be NULL if not used.
 *
 * @return  0 if succeeded, false otherwise
 */
int handleParamaterChange(parameterKind_t parameter, void *value, void *extraValue)
{
    int retValue = 0;

    // check which parameter it is
    switch(parameter)
    {
        case FLIGHT_MODE_ENABLE:

            // check if it needs to handle the change after setting s-in-flight
            if(*((bool *)extraValue))
            {
                // handle the change on s-in-flight
                // check if not set before and the parameter is 0 and the state is the FAULT_ON state
                if(!gSInFlightChangedFalse && (data_getMainState() == FAULT_ON))
                {
                    // set the variable true to make sure to enter the fault state again
                    gSInFlightChangedFalse = true;

                    // trigger the mainloop to react on it
                    swMeasuredFaultFunction(false);
                }
            }

        case S_IN_FLIGHT:

            // check if not set before and the parameter is 0 and the state is the FAULT_ON state
            if(!gSInFlightChangedFalse && *(uint8_t *)value == 0 && (data_getMainState() == FAULT_ON))
            {
                // set the variable true to make sure to check it again
                gSInFlightChangedFalse = true;

                // trigger the mainloop to react on it
                swMeasuredFaultFunction(false);
            }

            break;
        case S_CHARGE:
            // call the funtion set the led blink pattern
            ledState_calcStateIndication((*(uint8_t *)value));
            break;
        case A_FULL:
        case A_REM:
            // call the funtion set the led blink pattern
            ledState_calcStateIndication((*(uint8_t *)extraValue));
            break;
        // in case of the emergency button enable variable
        case EMERGENCY_BUTTON_ENABLE:

            // check if the pin change handle needs to be done
            if(*(uint8_t *)extraValue == 1)
            {
                // check if the variable is set
                if(*(uint8_t *)value)
                {
                    // output to the user
                    cli_printfWarning("WARNING: setting PTE8 as input pull-up!\n");

                    // set the GPIO as INPUT_PULLUP
                    retValue = gpio_changePinType(PTE8, INPUT_PULL_UP);
                }
                // if the variable is not set
                else
                {
                    // output to the user
                    cli_printfWarning("WARNING: removing pull-up on PTE8!\n");

                    // set the GPIO as INPUT
                    retValue = gpio_changePinType(PTE8, INPUT_INTERRUPT);
                }
            }
            break;

        // check it if needs to be handled in the batmangement part
        case T_MEAS:
        case V_CELL_OV:
        case V_CELL_UV:
        case C_PCB_OT:
        case C_PCB_UT:
        case C_CELL_OT:
        case C_CELL_UT:
        case C_CELL_OT_CHARGE:
        case C_CELL_UT_CHARGE:
        case N_CELLS:
        case SENSOR_ENABLE:
        case I_SLEEP_OC:
            // call the function to handle it
            retValue = batManagement_changedParameter(parameter, value, extraValue);
        default:
            break;
    }

    return retValue;
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
        gBCCRisingEdge = true;
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

    // check if semaphore needs to be posted
    sem_getvalue(&gUpdaterSem, &semValue);

    // check posting the semaphore is needed
    if(semValue < 0)
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
    // cli_printf("GPIO ISR: sig: %d, pin: %d value %d\n", signo, pinNumber, gpio_readPin(pinNumber);

    // cli_printf("ISR pin: %d\n", pinNumber);

    // check which pin it is
    switch(pinNumber)
    {
        // in case of the BCC fault pin
        case BCC_FAULT:

            // check if the variable is not high
            if(!gBCCRisingEdge)
            {
                // check if the pin is high
                if(gpio_readPin(pinNumber))
                {
                    // set the variable high to react on it
                    gBCCRisingEdge = true;

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
                // cli_printf("Button ISR!\n");
                // set the variable high (release)
                gButtonRisingEdge = true;
            }
            else
            {
                // set the variable high (press)
                gButtonPressEdge = true;
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
            cli_printfError("gpioIsrFunction ERROR: This pin's (%d) ISR isn't implemented!\n", pinNumber);
            break;
    }
}

/*!
 * @brief function that will be called when it needs to process a cli command when the CLI can't do this
 *
 * @param command the command that needs to be processed
 */
int processCLICommand(commands_t command)
{
    int             returnValue;
    states_t        currentState = getMainState();
    struct timespec currentTime;
    struct timespec sampleTime;
    int             ret = 0;

    mcuPowerModes_t mcuPowerMode;

    // check what command it is
    // if it is the reset command
    switch(command)
    {
        case CLI_RESET:
            // check for state
            if(currentState == FAULT_ON || currentState == FAULT_OFF)
            {
                // set the rest variable
                setNGetStateCommandVariable(true, CMD_RESET);

                // increase the main loop semaphore
                if(escapeMainLoopWait())
                {
                    cli_printfError("processCLICommand ERROR: Couldn't up mainloop sem!\n");
                }
            }
            else
            {
                // set the return value to fail.
                ret = -1;
                // output to the user
                cli_printf("Can not reset, not in FAULT_ON or FAULT_OFF mode! %s\n",
                    cli_getStateString(true, (int)currentState, NULL));
            }
            break;
        case CLI_SLEEP:
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
                    cli_printfError("processCLICommand ERROR: Couldn't up mainloop sem!\n");
                }
            }
            else if(currentState == SLEEP || currentState == OCV)
            {
                // inform user
                cli_printf("Already in sleep state (or OCV)!\n");

                ret = -1;
            }
            else
            {
                // inform user
                cli_printf("Can not go to sleep state by command in this state: %s!\n",
                    cli_getStateString(true, (int)currentState, NULL));

                ret = -1;
            }
            break;
        case CLI_WAKE:
            if(currentState == SLEEP)
            {
                // transition
                setNGetStateCommandVariable(true, CMD_WAKE);

                // increase the main loop semaphore
                if(escapeMainLoopWait())
                {
                    cli_printfError("processCLICommand ERROR: Couldn't up mainloop sem!\n");
                }
            }
            else
            {
                // can't wake
                cli_printf(
                    "Can not wake in this state: %s\n", cli_getStateString(true, (int)currentState, NULL));
                ret = -1;
            }
            break;
        case CLI_DEEP_SLEEP:
            if(currentState == SLEEP || currentState == CHARGE)
            {
                // transition
                setNGetStateCommandVariable(true, CMD_GO_2_DEEPSLEEP);

                // increase the main loop semaphore
                if(escapeMainLoopWait())
                {
                    cli_printfError("processCLICommand ERROR: Couldn't up mainloop sem!\n");
                }
            }
            else
            {
                cli_printf("Can not go to deep sleep in this state: %s\n",
                    cli_getStateString(true, (int)currentState, NULL));

                ret = -1;
            }
            break;
        case CLI_SAVE:
            // check the MCU power mode and check for errors
            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
            if(mcuPowerMode == ERROR_VALUE)
            {
                // error
                cli_printfError("processCLICommand ERROR: Could not get MCU power mode 3\n");
                ret = -1;
            }

            // check if the power mode is not NORMAL mode
            if((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE))
            {
                cli_printf("Flash may not be written in VLPR mode\n");
                cli_printf("Waking up the BMS... \n");

                // sample the time
                if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                {
                    cli_printfError("processCLICommand ERROR: failed to get sampleTime!\n");
                }

                // set the current time to the sample time
                currentTime.tv_sec = sampleTime.tv_sec;

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
                while(((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) ||
                          (mcuPowerMode == ERROR_VALUE)) &&
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
            if((mcuPowerMode != STANDBY_MODE) && (mcuPowerMode != VLPR_MODE) && (mcuPowerMode != ERROR_VALUE))
            {
                // save the parameters to the flash
                cli_printf("Saving parameters to flash!\n");

                // save them
                returnValue = data_saveParameters();

                // check for error
                if(returnValue)
                {
                    cli_printfError("processCLICommand ERROR: saving par went wrong!%d\n", returnValue);
                    ret = -1;
                }
                else
                {
                    ret = 0;
                }
            }
            else
            {
                cli_printfError(
                    "processCLICommand ERROR: Flash may not be written while MCU is not in RUN mode!\n");
                ret = -1;
            }
            break;
        case CLI_LOAD:
            // load the parameters from the flash
            cli_printf("Loading parameters from flash!\n");

            // save them
            returnValue = data_loadParameters();

            // check for error
            if(returnValue)
            {
                ret = -1;
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
            break;
        case CLI_DEFAULT:
            // check the MCU power mode and check for errors
            mcuPowerMode = power_setNGetMcuPowerMode(false, ERROR_VALUE);
            if(mcuPowerMode == ERROR_VALUE)
            {
                // error
                cli_printfError("processCLICommand ERROR: Could not get MCU power mode 3\n");
            }

            // check if the power mode is not NORMAL mode
            if((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) || (mcuPowerMode == ERROR_VALUE))
            {
                cli_printf("Flash may not be written in VLPR mode\n");
                cli_printf("Waking up the BMS... \n");

                // sample the time
                if(clock_gettime(CLOCK_REALTIME, &sampleTime) == -1)
                {
                    cli_printfError("processCLICommand ERROR: failed to get sampleTime!\n");
                }

                // set the current time to the sample time
                currentTime.tv_sec = sampleTime.tv_sec;

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
                while(((mcuPowerMode == STANDBY_MODE) || (mcuPowerMode == VLPR_MODE) ||
                          (mcuPowerMode == ERROR_VALUE)) &&
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
            if((mcuPowerMode != STANDBY_MODE) && (mcuPowerMode != VLPR_MODE) && (mcuPowerMode != ERROR_VALUE))
            {
                cli_printf("Setting default parameters!\n");

                // set the default values and check for error
                if(data_setDefaultParameters())
                {
                    // output
                    cli_printfError("processCLICommand ERROR: could not set default values!\n");
                    ret = -1;
                }
                else
                {
                    ret = 0;
                }
            }
            else
            {
                cli_printfError(
                    "processCLICommand ERROR: Flash may not be written while MCU is not in RUN mode!\n");
                ret = -1;
            }
            break;
        default:
            // error
            ret = -1;
            cli_printfError("processCLICommand ERROR: wrong command: %d!\n", command);
            break;
    }

    return ret;
}

/*!
 * @brief function that will return the main state, but it will use the mutex
 *
 * @return the state
 */
states_t getMainState(void)
{
    states_t retValue = INIT;

    // lock the mutex
    pthread_mutex_lock(&gStateLock);

    // save the state
    retValue = gCurrentState;

    // unlock the mutex
    pthread_mutex_unlock(&gStateLock);

    // return to the user
    return retValue;
}

/*!
 * @brief function that will set the main state, but it will use the mutex
 *
 * @param newState the new state
 */
static int setMainState(states_t newState)
{
    int retValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gStateLock);

    // set the state
    gCurrentState = newState;

    // unlock the mutex
    pthread_mutex_unlock(&gStateLock);

    return retValue;
}

/*!
 * @brief function that will return the charge state, but it will use the mutex
 *
 * @return the state
 */
charge_states_t getChargeState(void)
{
    charge_states_t retValue = INIT;

    // lock the mutex
    pthread_mutex_lock(&gChargeStateLock);

    // save the state
    retValue = gCurrentChargeState;

    // unlock the mutex
    pthread_mutex_unlock(&gChargeStateLock);

    // return to the user
    return retValue;
}

/*!
 * @brief function that will set the charge state, but it will use the mutex
 *
 * @param newState the new state
 */
static int setChargeState(charge_states_t newState)
{
    int retValue = 0;

    // lock the mutex
    pthread_mutex_lock(&gChargeStateLock);

    // set the state
    gCurrentChargeState = newState;

    // unlock the mutex
    pthread_mutex_unlock(&gChargeStateLock);

    return retValue;
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
    bool retValue;

    // lock the mutex
    pthread_mutex_lock(&gTransVarLock);

    // check which variable to return
    switch(variable)
    {
        case DISCHAR_VAR:
            // save the state
            retValue = gDischargeDetected;
            break;
        case CHAR_VAR:
            // save the variable
            retValue = gChargeDetected;
            break;
        case SLEEP_VAR:
            // save the variable
            retValue = gSleepDetected;
            break;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gTransVarLock);

    // return to the user
    return retValue;
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
    int retValue = 0;

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
    return retValue;
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
    stateCommands_t        retValue     = CMD_ERROR;
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
    // if get
    else
    {
        // save the variable
        retValue = stateCommand;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gStateCommandLock);

    // return the value
    return retValue;
}

/*!
 * @brief   function that will calculate and return the OCV period time
 *
 * @param   newTime the address of the variable to become the OCV timer period
 * @param   oldState the oldState of the state machine
 * @warning Keep in mind that this function needs to be called before
 *          the oldState is set with the current state (oldState = getMainState())
 *          when the OCV timer needs to be increased
 *
 * @return  0 if ok
 */
static int getOcvPeriodTime(int32_t *newTime, states_t oldState)
{
    int            retValue  = -1;
    static int32_t ocvPeriod = 0;
    int32_t        cyclic;

    // check for NULL pointer
    if(newTime == NULL)
    {
        // output and return
        cli_printfError("main getOcvPeriodTime ERROR: newTime is NULL pointer!\n");
        return retValue;
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
    retValue = 0;
    return retValue;
}

/*!
 * @brief   function that is used to enable or disable the CAN messages
 *          or get the value if it should output the CAN messages.
 *
 * @param   setNGet if true it will set the value if the CAN messages should be outputted.
 *          if false it will return 1 if it should be outputted, 0 otherwise.
 * @param   enable if setNGet is true, if enable is true the CAN messages are enabled
 *          to send after the new measurements, if enable is false otherwise.
 * @note    this function can be safely called from multiple threads.
 *
 * @return  < 0 if not OK, >= 0 if OK.
 */
static int setNGetEnableCanMessages(bool setNGet, bool enable)
{
    int         retValue          = -1;
    static bool enableCanMessages = false;

    // lock the mutex
    pthread_mutex_lock(&gSetCanMessagesLock);

    // check if it needs to set the value
    if(setNGet)
    {
        // set the message
        enableCanMessages = enable;
    }

    // make the return value
    retValue = enableCanMessages;

    // unlock the mutex
    pthread_mutex_unlock(&gSetCanMessagesLock);

    // return
    return retValue;
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
    int         retValue        = -1;
    static bool enableNfcUpdate = false;

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
                nfc_updateBMSStatus(true, true, NULL, NULL);
            }
            else
            {
                // put the charge relaxation data notice in the NTAG
                nfc_updateBMSStatus(true, false, NULL, NULL);
            }
        }
    }

    // make the return value
    retValue = enableNfcUpdate;

    // unlock the mutex
    pthread_mutex_unlock(&gSetNfcUpdateLock);

    // return
    return retValue;
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
    int         retValue            = -1;
    static bool enableDisplayUpdate = false;

    // lock the mutex
    pthread_mutex_lock(&gSetDisplayUpdateLock);

    // check if it needs to set the value
    if(setNGet)
    {
        // set the message
        enableDisplayUpdate = enable;
    }

    // make the return value
    retValue = enableDisplayUpdate;

    // unlock the mutex
    pthread_mutex_unlock(&gSetDisplayUpdateLock);

    // return
    return retValue;
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
    int retValue = 0;

    // kick the watchdog
    if(sbc_kickTheWatchdog())
    {
        cli_printfError("main ERROR: Couldn't kick the watchdog!\n");
        retValue = -1;
    }

    // sleep for a little time
    usleep(usec);

    // return
    return retValue;
}

/*!
 * @brief   Function that is used to check if the tasks are on
 *          Like the batmanag task, updater task and other tasks
 *
 * @param   on the address of the varible to become true if the tasks are on,
 *          false otherwise.
 *
 * @return  On successful completion, returns 0. Otherwise, it returns -1
 */
static int checkSequenceTaskStatus(bool *on)
{
    int retValue = -1;

    // check if no NULL pointer
    if(on != NULL)
    {
        // check if the measurement tasks are running
        retValue = batManagement_getBatManagementStatus(on);
    }
    // if the bat management task is not running
    if(!(*on))
    {
        // check the updater task
        *on = getUpdaterTaskStatus();
    }

    // return
    return retValue;
}

/*!
 * @brief   Function that is used to check if the updater tasks is on
 *
 * @param   none
 *
 * @return  true if the updater task is running, false otherwise
 */
static bool getUpdaterTaskStatus(void)
{
    int  semValue;
    bool returnValue;

    // check the updater task
    // get the measurment semaphore value
    sem_getvalue(&gUpdaterSem, &semValue);

    // check if the updater task is running
    if(semValue != -1)
    {
        // make sure to state that the task is running
        returnValue = true;
    }
    else
    {
        // it is not running
        returnValue = false;
    }

    return returnValue;
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

/*!
 * @brief   Function that is used to stop the main loop (error)
 *
 * @param   none
 *
 * @return  none
 */
static void stopMainLoop(void)
{
    // set the LED to RED to inform the user
    ledState_setLedColor(RED, OFF, LED_BLINK_OFF);

    // extra error message
    cli_printfError("main ERROR: main loop will stop!\n");

    // endless loop
    while(1)
    {
        // don't continue
        usleep(1);
    }
}

/*!
 * @brief   Function that will enable or disable I2C, xCAN, NFC, Display updates and
 *          The battery management task if setBatManagement is true.
 * @note    When disabling it will set the SMBUS current to 0.
 *
 * @param   enable If the updates or task need to be enabled (true) or disabled (false).
 * @param   setBatManagement if the enable or disable should be done for the battery management task as well.
 *
 * @return  none
 */
static void enableUpdatesAndBatManagementTask(bool enable, bool setBatManagement)
{
    // check if the updates needs to be enabled
    if(enable)
    {
        // Enable the I2C
        if(i2c_enableTransmission(true))
        {
            cli_printfError("main ERROR: failed to enable I2C!\n");
        }

        // turn on the CAN messages
        if(setNGetEnableCanMessages(true, true) < 0)
        {
            // output error
            cli_printfError("main ERROR: Could not enable CAN messages!\n");
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

        // the display will be turned on in the updater task.

        // check if the measurements needs to be enabled
        if(setBatManagement)
        {
            // enable the battery management task
            batManagement_enableBatManagementTask(true);
        }
    }
    else //! enable
    {
        // check if the measurements needs to be disabled
        if(setBatManagement)
        {
            // disable the battery management task
            batManagement_enableBatManagementTask(false);
        }

        // turn off the NFC update
        if(setNGetEnableNFCUpdates(true, false) < 0)
        {
            // output error
            cli_printfError("main ERROR: Could not enable NFC update!\n");
        }

        // turn off the display update
        if(setNGetEnableDisplayUpdates(true, false) < 0)
        {
            // output error
            cli_printfError("main ERROR: Could not enable display update!\n");
        }

        // turn off the display
        if(display_setPower(false))
        {
            // output error
            cli_printfError("main ERROR: Could not turn off display!\n");
        }

        // turn off the CAN messages
        if(setNGetEnableCanMessages(true, false) < 0)
        {
            // output error
            cli_printfError("main ERROR: Could not enable CAN messages!\n");
        }

        // set the SMBus current to 0
        SMBus_updateInformation(true, NULL, NULL);

        // disable the I2C
        if(i2c_enableTransmission(false))
        {
            cli_printfError("main ERROR: failed to enable I2C!\n");
        }
    }

    return;
}

/*!
 * @brief   Function that will reset the common variables
 *          It will disable cell balancing, turn off the charge mode and reset the command variable
 *
 * @param   none
 *
 * @return  none
 */
static void resetVariables(void)
{
    // disable cell balancing
    batManagement_setBalanceState(BALANCE_OFF);

    // set the charge mode off
    batManagement_startCharging(false);

    // reset the command variable
    setNGetStateCommandVariable(true, CMD_NONE);

    return;
}

/*!
 * @brief   Function that will check if the MCU is in run mode
 *          If not, it will set the MCU to run mode.
 *          If it failed to set the MCU to run mode, it will stop.
 *          It will set the SBC (system basic chip)
 *          and AFE (Analog front end (BCC)) to normal.
 *
 * @param   none
 *
 * @return  none
 */
static int setMcuAfeSbcToNormalMode(bool returnWhenError)
{
    // check if the MCU is not in run mode
    if(power_setNGetMcuPowerMode(false, ERROR_VALUE) != RUN_MODE)
    {
        // change the MCU to run mode
        if(power_setNGetMcuPowerMode(true, RUN_MODE) == ERROR_VALUE)
        {
            // error message
            cli_printfError("main ERROR: failed to put the system in RUN mode!\n");

            // check if it should return when an error happens
            if(returnWhenError)
            {
                return -1;
            }
            else
            {
                cli_printfWarning("WARNING: Stopping main loop!\n");

                // error and stop main loop
                stopMainLoop();
            }
        }
    }

    // Set the BCC (AFE) to normal mode
    if(batManagement_setAFEMode(AFE_NORMAL))
    {
        // error
        cli_printfError("main ERROR: failed to set AFE to normal mode!\n");
    }

    // set the SBC to normal mode
    if(sbc_setSbcMode(SBC_NORMAL))
    {
        // error
        cli_printfError("main ERROR: failed to set SBC to normal mode!\n");
    }

    return 0;
}

// EOF
