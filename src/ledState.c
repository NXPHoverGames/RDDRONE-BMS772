/****************************************************************************
 * nxp_bms/BMS_v1/src/ledState.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>

#include <arch/board/board.h>
#include <nuttx/leds/userled.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <sched.h>
#include <errno.h>

#include "ledState.h"
#include "cli.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Defines
 ****************************************************************************/

#define DEFAULT_LED_PRIORITY    50
#define DEFAULT_LED_STACK_SIZE  1024

#ifndef MAX_NSEC
#define MAX_NSEC                999999999
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/

static bool gLEDInitialized = false;    // to check if the LEDs are initialized
static LEDColor_t gLEDColor = OFF;      // the color of LEDs 
static LEDColor_t gAltLEDColor = OFF;   // the alternating color of LEDs 
static uint16_t gOnOffTimems = 1000;    // the blinkperiod of the LEDs

static pthread_mutex_t gLedsLock;       // mutex for the LEDS

/*! @brief  semaphore for the continue measure task*/
static sem_t gBlinkerSem;

/*! @brief  semaphore for the continue measure task*/
static sem_t gBlinkerWaitSem;

static uint8_t gLedStateIndicationNumber = 1;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief function to be used by a task to enable blinking of the LEDs
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int ledBlinkTaskFunc(int argc, char *argv[]);

/****************************************************************************
 * main
 ****************************************************************************/

/*!
 * @brief   this function initialized the RGB LED and it will set the LED to green blinking
 *          
 * @param   startColor the starting color from the LEDColor_t enum
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_initialize(RED, false))
 *          {
 *              // do something with the error
 *          }
 */
int ledState_initialize(LEDColor_t startColor, bool skipSelfTest)
{
    int lvRetValue = !gLEDInitialized;
    int errcode;
    uint32_t realLedState = 0;

    // check if already configured
    if(!gLEDInitialized)
    {
        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST LEDs: START\n");
        }

        /* Configure LED GPIOs for output */
        board_userled_initialize();

        // initialze the mutex
        pthread_mutex_init(&gLedsLock, NULL);

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            // set the leds off
            gLEDColor = OFF;

            // set the leds 
            board_userled_all((uint8_t)gLEDColor);

            // read the read LED state
            board_userled_checkall(&realLedState);

            // check if the LEDS are not off
            if(realLedState != OFF)
            {
                // output to user
                cli_printfError("ledState ERROR: failed to turn all the LEDs off!\n");

                // check which LED is on and output to the user
                if(realLedState & RED)
                {
                    cli_printfError("ledState ERROR: RED LED is on!\n");
                }
                if(realLedState & GREEN)
                {
                    cli_printfError("ledState ERROR: GREEN LED is on!\n");
                }
                if(realLedState & BLUE)
                {
                    cli_printfError("ledState ERROR: BLUE LED is on!\n");
                }

                // return with a fault
                return lvRetValue;
            }

            // set the LED to RED
            gLEDColor = RED;

            // set the LEDs 
            board_userled_all((uint8_t)gLEDColor);

            // read the read LED state
            board_userled_checkall(&realLedState);

            // check if the LED is not RED
            if(realLedState != RED)
            {
                // output to user
                cli_printfError("ledState ERROR: failed to make the LED RED!\n");
                return lvRetValue;
            }

            // set the LED to GREEN
            gLEDColor = GREEN;

            // set the LEDs 
            board_userled_all((uint8_t)gLEDColor);

            // read the read LED state
            board_userled_checkall(&realLedState);

            // check if the LED is not GREEN
            if(realLedState != GREEN)
            {
                // output to user
                cli_printfError("ledState ERROR: failed to make the LED GREEN!\n");
                return lvRetValue;
            }

            // set the LED to BLUE
            gLEDColor = BLUE;

            // set the LEDs 
            board_userled_all((uint8_t)gLEDColor);

            // read the read LED state
            board_userled_checkall(&realLedState);

            // check if the LED is not BLUE
            if(realLedState != BLUE)
            {
                // output to user
                cli_printfError("ledState ERROR: failed to make the LED BLUE!\n");
                return lvRetValue;
            }
        }

        // set that the color to the start color
        gLEDColor = startColor;

        // set the LEDs to the start color
        board_userled_all((uint8_t)gLEDColor);

        // initialize the semaphores
        lvRetValue = sem_init(&gBlinkerSem, 0, 0);
        sem_setprotocol(&gBlinkerSem, SEM_PRIO_NONE);
        
        if(lvRetValue)
        {
            // output to user
            cli_printfError("ledState ERROR: failed to initialze blinker sem! error: %d\n", lvRetValue);
            return lvRetValue;
        }

        lvRetValue = sem_init(&gBlinkerWaitSem, 0, 0);
        sem_setprotocol(&gBlinkerWaitSem, SEM_PRIO_NONE);
        
        if(lvRetValue)
        {
            // output to user
            cli_printfError("ledState ERROR: failed to initialze wait sem! error: %d\n", lvRetValue);
            return lvRetValue;
        }

        // create the blinking task
        lvRetValue = task_create("blinker", DEFAULT_LED_PRIORITY, DEFAULT_LED_STACK_SIZE, 
            ledBlinkTaskFunc, NULL);
        if(lvRetValue < 0)
        {
            // inform user
            errcode = errno;
            cli_printfError("ledState ERROR: Failed to start task: %d\n", errcode);
            return lvRetValue;
        }

        // set that the LED's are configured
        gLEDInitialized = true;

        // change the return value
        lvRetValue = !gLEDInitialized;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST LEDs: \e[32mPASS\e[39m\n");
        }
    }

    // return the value
    return lvRetValue;
}

/*!
 * @brief   This function will set the color of the LED that can be entered from the LEDColor_t enum.
 *          This function can also be used to set the LED off. using the OFF enum.
 *          Using this function the blink period, alternating color can be configured if blinkPeriodms is true (!= 0)
 *          ledState_initialize(); should be called before using this function
 *          This function can be called from multiple threads
 *          
 * @param   newColor The new color is should have from the LEDColor_t enum, it will change imidiatly 
 * @param   newAltColor The alternating color if blinkPeriodms is true. This could be OFF to blink or an other color
 * @param   blinkPeriodms The blinkperiod of the LED, in ms. If 0 blinking is disabled
 *
 * @return  If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_setLedColor(GREEN, OFF, 2000)) // led from GREEN-OFF will take 2s (period)
 *          {
 *              // do something with the error
 *          }
 */
int ledState_setLedColor(LEDColor_t newColor, LEDColor_t newAltColor, uint16_t blinkPeriodms)
{
    int lvRetValue = !gLEDInitialized;
    int semValue;
    static bool blinking = false;           // if the blinking task has begon

    // check if initialized 
    if(!gLEDInitialized)
    {
        // error
        cli_printfError("ledState ERROR: Leds not initialized, pleaze initialze\n");
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gLedsLock);

    // check if it already has the color
    if(((newColor == gLEDColor) && ((bool)blinkPeriodms == blinking) && ((bool)blinkPeriodms == false)) || 
        (((bool)blinkPeriodms == true) && (newAltColor == gAltLEDColor) && (blinkPeriodms == gOnOffTimems)))
    {
        // unlock the mutex
        pthread_mutex_unlock(&gLedsLock);

        // it already has this color
        return lvRetValue;
    }

    // set the new colors and time
    gLEDColor = newColor;
    gAltLEDColor = newAltColor;
    gOnOffTimems = blinkPeriodms/2;

    // check the semaphore 
    sem_getvalue(&gBlinkerSem, &semValue);

    // check if the blinker was running and should end wait to set and start with the new color
    if(blinking && (bool)blinkPeriodms)
    {
        //cli_printf("restarting!\n");
        // increase the semaphore to get out of the timed wait
        sem_post(&gBlinkerWaitSem);
    }

    // stop the blinker task if it was running
    if(semValue > 0 && blinking && !(bool)blinkPeriodms)
    {
        // decrease the semaphore
        sem_wait(&gBlinkerSem);

        // set the variable
        blinking = false;
    }

    // set the color
    board_userled_all((uint8_t)newColor);

    // check if the LED needs to blink and is not blinking
    if(semValue < 1 && !blinking && (bool)blinkPeriodms)
    {
        // increase the semaphore
        sem_post(&gBlinkerSem);

        // set the variable
        blinking = true;
    }

    // unlock the mutex
    pthread_mutex_unlock(&gLedsLock);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief function to be used by a task to enable blinking of the LEDs
 * 
 * @param the amount of arguments there are in argv (if the last argument is NULL!)
 * @param a character pointer array with the arguments, first is the taskname than the arguments
 */
static int ledBlinkTaskFunc(int argc, char *argv[])
{
    struct timespec waitTime;
    bool lvBlinkingOff = false;
    int semState = 0;
    uint8_t greenBlinkCounts = 0;

    // loop
    while(1)
    {
        // wait for the semaphore to pause the blinker task, or go through if need to run
        sem_wait(&gBlinkerSem);

        // increase the semaphore to keep running
        sem_post(&gBlinkerSem);

        // toggle the led
        // set the color
        if(lvBlinkingOff)
        {
            // set the standart color
            board_userled_all((uint8_t)gLEDColor);
        }
        else
        {
            // set the to the alternating color
            board_userled_all(gAltLEDColor);
        }

        // toggle the blinking variable 
        lvBlinkingOff = !lvBlinkingOff;

        // get the time 
        if(clock_gettime(CLOCK_REALTIME, &waitTime))
        {
            cli_printfError("ledState ERROR: failed to get time! errno: %d\n", errno);

            // than just sleep
            usleep(gOnOffTimems*1000);
        }
        // if you got the current time
        else
        {
            // check when in normal mode, to indicate the state of charge 
            if(gLEDColor == GREEN)
            {
                // check if the LED is off 
                if(lvBlinkingOff)
                {
                    // increase the blink counter and check if equal to the amount of blinks
                    if(++greenBlinkCounts >= ledState_getStateIndication())
                    {
                        // reset the counter 
                        greenBlinkCounts = 0;

#ifdef LED_INDICATION_DEFINED_WAIT
                        // make a wait time to make sure it waits LED_DEFINED_WAIT
                        waitTime.tv_sec += LED_DEFINED_WAIT;
#else
                        // make a wait time to make sure it is off until LED_PERIOD_INDICATION_S total period has passed
                        waitTime.tv_sec += (LED_PERIOD_INDICATION_S - ledState_getStateIndication());
#endif
                    }
                    else
                    {
                        // make the wait time
                        waitTime.tv_sec += ((int)LED_BLINK_TIME_MS / 1000) + (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) / (int)(MAX_NSEC+1);
                        waitTime.tv_nsec = (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) % (MAX_NSEC+1);
                    }
                }
                else
                {
                    waitTime.tv_sec += ((int)LED_BLINK_TIME_MS / 1000) + (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) / (int)(MAX_NSEC+1);
                    waitTime.tv_nsec = (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) % (MAX_NSEC+1);
                }               
            }
            // if it is any other color blink
            else
            {
                // add the blinktime
                //waitTime.tv_sec   += LED_BLINK_TIME_S;
                waitTime.tv_sec += ((int)gOnOffTimems / 1000) + (waitTime.tv_nsec + ((gOnOffTimems % 1000) *1000*1000)) / (int)(MAX_NSEC+1);
                waitTime.tv_nsec = (waitTime.tv_nsec + ((gOnOffTimems % 1000) *1000*1000)) % (MAX_NSEC+1);
            }

            // wait for the time to expire or continue when semaphore is available
            semState = sem_timedwait(&gBlinkerWaitSem, &waitTime);

            // check if the sem was available
            if(!semState)
            {
                // set the blinking variable on
                lvBlinkingOff = true;
            }
        }
    }

    // should not come here
    return 1;
}

/*!
 * @brief   This function will used to set the blinking indication, the number of blinks
 *          
 * @param   newValue the newvalue, this needs to be at least 1 less than LED_PERIOD_INDICATION_S
 *
 * @return  the new state indication number of blinks, UINT8_MAX if error  
 */
uint8_t ledState_calcStateIndication(uint8_t newValue)
{
    uint8_t lvRetValue = UINT8_MAX;
    static uint8_t oldStateIndication = 0;

    // check if initialized 
    if(!gLEDInitialized)
    {
        // error
        cli_printfError("ledState ERROR: Leds not initialized, pleaze initialze\n");
        return lvRetValue;
    }

    // check the input 
    if(newValue > 100)
    {
        // error
        cli_printfError("ledState ERROR: enter valid input: %d > 100\n", newValue);
        return lvRetValue;
    }

    // calc the new blink value 
    if(newValue >= LED_4_BLINK_PERCENTAGE)
    {
        // set the value
        lvRetValue = 4;
    }
    else if(newValue >= LED_3_BLINK_PERCENTAGE)
    {
        // set the value
        lvRetValue = 3;
    }
    else if(newValue >= LED_2_BLINK_PERCENTAGE)
    {
        // set the value
        lvRetValue = 2;
    }
    else 
    {
        // set the value
        lvRetValue = 1;
    }

    // check if nothing changed
    if(lvRetValue == oldStateIndication)
    {
        // return with the value 
        return lvRetValue;
    }

#ifndef LED_INDICATION_DEFINED_WAIT

    // check the define 
    if(lvRetValue >= LED_PERIOD_INDICATION_S)
    {
        // error
        cli_printfError("ledState ERROR: LED_PERIOD_INDICATION_S is not valid: %d >= %d\n", 
            lvRetValue, LED_PERIOD_INDICATION_S);
        return lvRetValue;
    }

#endif

    // lock the mutex
    pthread_mutex_lock(&gLedsLock);

    // save the value
    gLedStateIndicationNumber = lvRetValue;

    // unlock the mutex
    pthread_mutex_unlock(&gLedsLock);

    // set the old value
    oldStateIndication = lvRetValue;

    // return the value 
    return lvRetValue;
}

/*!
 * @brief   This function will used to get the blinking indication, the number of blinks
 *          
 * @param   none
 *
 * @return  the new state indication number of blinks, UINT8_MAX if error 
 */
uint8_t ledState_getStateIndication(void)
{
    uint8_t lvRetValue = UINT8_MAX;

    // check if initialized 
    if(!gLEDInitialized)
    {
        // error
        cli_printfError("ledState ERROR: Leds not initialized, pleaze initialze\n");
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gLedsLock);

    // save the value
    lvRetValue = gLedStateIndicationNumber;

    // unlock the mutex
    pthread_mutex_unlock(&gLedsLock);

    // return the value 
    return lvRetValue;
}

#endif
