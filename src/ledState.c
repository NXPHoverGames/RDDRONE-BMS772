/****************************************************************************
 * nxp_bms/BMS_v1/src/ledState.c
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

#define DEFAULT_LED_PRIORITY 	100
#define DEFAULT_LED_STACK_SIZE 	1024

#ifndef MAX_NSEC
#define MAX_NSEC	 				999999999
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/

static bool gLEDInitialized = false; 	// to check if the LEDs are initialized
static LEDColor_t gLEDColor = OFF;		// the color of LEDs 

//static pid_t gPreviousBlinkTaskPid = 0; 	// the PID of the blinking task

static pthread_mutex_t gLedsLock;		// mutex for the LEDS

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
 * @brief 	this function initialized the RGB LED and it will set the LED to green blinking
 * 			
 * @param 	None
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_initialize())
 *			{
 *				// do something with the error
 *			}
 */
int ledState_initialize(void)
{
	int lvRetValue = !gLEDInitialized;
	int errcode;

	// check if already configured
	if(!gLEDInitialized)
	{
		cli_printf("SELF-TEST START: LEDs\n");

		/* Configure LED GPIOs for output */
	  	board_userled_initialize();

	  	// initialze the mutex
		pthread_mutex_init(&gLedsLock, NULL);

	 	// start the leds with OFF
	 	board_userled_all((uint8_t)OFF);

	 	// set that the color is off
	 	gLEDColor = OFF;

	 	// initialize the semaphores
		lvRetValue = sem_init(&gBlinkerSem, 0, 0);
		sem_setprotocol(&gBlinkerSem, SEM_PRIO_NONE);
		
		if(lvRetValue)
		{
			// output to user
			cli_printf("ledState ERROR: failed to initialze blinker sem! error: %d\n", lvRetValue);
			return lvRetValue;
		}

		lvRetValue = sem_init(&gBlinkerWaitSem, 0, 0);
		sem_setprotocol(&gBlinkerWaitSem, SEM_PRIO_NONE);
		
		if(lvRetValue)
		{
			// output to user
			cli_printf("ledState ERROR: failed to initialze wait sem! error: %d\n", lvRetValue);
			return lvRetValue;
		}

	 	// create the blinking task
		lvRetValue = task_create("blinker", DEFAULT_LED_PRIORITY, DEFAULT_LED_STACK_SIZE, ledBlinkTaskFunc, NULL);
		if(lvRetValue < 0)
	    {
	    	// inform user
	    	errcode = errno;
	      	cli_printf("ledState ERROR: Failed to start task: %d\n", errcode);
	      	return lvRetValue;
	    }

	    // set that the LED's are configured
	 	gLEDInitialized = true;

	 	// change the return value
	 	lvRetValue = !gLEDInitialized;

	 	cli_printf("SELF-TEST PASS:  LEDs\n");
	}

	// return the value
	return lvRetValue;
}


/*!
 * @brief 	This function will set the color of the LED that can be entered from the LEDColor_t enum.
 * 			This function can also be used to set the LED off. using the OFF enum.
 * 			ledState_initialize(); should be called before using this function
 * 			
 * @param 	The new color is should have from the LEDColor_t enum, it will change imidiatly 
 * @param 	if the LED should blink at the rate of BLINK_TIME, LED_BLINK_ON or LED_BLINK_OFF could be used
 *
 * @return 	If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example if(ledState_setLedColor(GREEN, LED_BLINK_ON))
 *			{
 *				// do something with the error
 *			}
 */
int ledState_setLedColor(LEDColor_t newColor, bool BlinkOn)
{
	int lvRetValue = !gLEDInitialized;
	int semValue;
	static bool blinking = false;			// if the blinking task has begon

	// check if initialized 
	if(!gLEDInitialized)
	{
		// error
		cli_printf("ERROR: Leds not initialized, pleaze initialze\n");
		return lvRetValue;
	}

	// lock the mutex
	pthread_mutex_lock(&gLedsLock);

	// check if it already has the color
	if((newColor == gLEDColor) && (BlinkOn == blinking))
	{
		// unlock the mutex
		pthread_mutex_unlock(&gLedsLock);

		// it already has this color
		return lvRetValue;
	}

	// set the new color
	gLEDColor = newColor;

	// check the semaphore 
	sem_getvalue(&gBlinkerSem, &semValue);

	// check if the blinker was running and should end wait to set and start with the new color
	if(blinking && BlinkOn)
	{
		//cli_printf("restarting!\n");
		// increase the semaphore to get out of the timed wait
		sem_post(&gBlinkerWaitSem);
	}

	// delete the previous task if it was running
	if(semValue > 0 && blinking && !BlinkOn)
	{
		// decrease the semaphore
		sem_wait(&gBlinkerSem);

		// set the variable
		blinking = false;
	}

	// set the color
	board_userled_all((uint8_t)newColor);

	// check if the LED needs to blink and is not blinking
	if(semValue < 1 && !blinking && BlinkOn)
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
			// check if it is the red blue combination
			if(gLEDColor == RED_BLUE)
			{
				// set the color
				board_userled_all((uint8_t)RED);
			}
			else
			{
				// set the color
				board_userled_all((uint8_t)gLEDColor);
			}
		}
		else
		{
			// check if it is the red blue combination
			if(gLEDColor == RED_BLUE)
			{
				// set the color
				board_userled_all((uint8_t)BLUE);
			}
			else
			{
				// set the LED off
				board_userled_all(OFF);
			}
		}

		// toggle the blinking variable	
		lvBlinkingOff = !lvBlinkingOff;

		// get the time 
		if(clock_gettime(CLOCK_REALTIME, &waitTime))
        {
            cli_printf("ledState ERROR: failed to get time! errno: %d\n", errno);

            // than use sleep for seconds
            usleep(LED_BLINK_TIME_MS*1000);
        }
        else
        {
         	// check when in normal mode, to indicate the state of charge 
			if(gLEDColor == GREEN)
			{
				// check if the LED is off 
				if(lvBlinkingOff)
				{
					//cli_printf("ledcounts: %d\n", greenBlinkCounts);

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
						waitTime.tv_sec 	+= ((int)LED_BLINK_TIME_MS / 1000) + (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) / (int)(MAX_NSEC+1);
						waitTime.tv_nsec  	= (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) % (MAX_NSEC+1);
					}
				}
				else
				{
					// make the wait time
					waitTime.tv_sec 	+= ((int)LED_BLINK_TIME_MS / 1000) + (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) / (int)(MAX_NSEC+1);
					waitTime.tv_nsec  	= (waitTime.tv_nsec + ((LED_BLINK_TIME_MS % 1000) *1000*1000)) % (MAX_NSEC+1);
				}				
			}
			// if ledColor is not green
			else
			{
				// add the blinktime
				waitTime.tv_sec 	+= LED_BLINK_TIME_S;
			}

			// wait for the time to expire or continue when semaphore is available
			semState = sem_timedwait(&gBlinkerWaitSem, &waitTime);

			// check if the sem was available
			if(!semState)
			{
				//cli_printf("Went out of the sleep!\n" );

				// set the blinking variable on
				lvBlinkingOff = true;
			}
		}
	}

	// should not come here
	return 1;
}

/*!
 * @brief 	This function will used to set the blinking indication, the number of blinks
 * 			
 * @param 	newValue the newvalue, this needs to be at least 1 less than LED_PERIOD_INDICATION_S
 *
 * @return 	the new state indication number of blinks, UINT8_MAX if error  
 */
uint8_t ledState_calcStateIndication(uint8_t newValue)
{
	uint8_t lvRetValue = UINT8_MAX;
	static uint8_t oldStateIndication = 0;

	// check if initialized 
	if(!gLEDInitialized)
	{
		// error
		cli_printf("ERROR: Leds not initialized, pleaze initialze\n");
		return lvRetValue;
	}

	// check the input 
	if(newValue > 100)
	{
		// error
		cli_printf("ERROR: enter valid input: %d > 100\n", newValue);
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
		cli_printf("ERROR: LED_PERIOD_INDICATION_S is not valid: %d >= %d\n", lvRetValue, LED_PERIOD_INDICATION_S);
		return lvRetValue;
 	}

#endif

	// lock the mutex
	pthread_mutex_lock(&gLedsLock);

	// save the value
	gLedStateIndicationNumber = lvRetValue;

	// unlock the mutex
	pthread_mutex_unlock(&gLedsLock);

	//cli_printf("new ledStateindicator %d\n", lvRetValue);

	// set the old value
	oldStateIndication = lvRetValue;

	// return the value 
	return lvRetValue;
}

/*!
 * @brief 	This function will used to get the blinking indication, the number of blinks
 * 			
 * @param 	none
 *
 * @return 	the new state indication number of blinks, UINT8_MAX if error 
 */
uint8_t ledState_getStateIndication(void)
{
	uint8_t lvRetValue = UINT8_MAX;

	// check if initialized 
	if(!gLEDInitialized)
	{
		// error
		cli_printf("ERROR: Leds not initialized, pleaze initialze\n");
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



