/****************************************************************************
 * nxp_bms/BMS_v1/inc/gpio.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
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
 ** ###################################################################
 **     Filename    : gpio.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-04-03
 **     Abstract    :
 **        gpio module.
 **        This module contains all functions needed for using gpio
 **
 ** ###################################################################*/
/*!
 ** @file gpio.h
 **
 ** @version 01.00
 **
 ** @brief
 **        gpio module. this module contains the functions to control the GPIO pins
 **
 */
#ifndef GPIO_H_
#define GPIO_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <sched.h>

/*******************************************************************************
 * defines
 ******************************************************************************/
#define START_OUTPUT_PIN_NUMBER 	0
#define START_INTERRRUPT_PIN_NUMBER 7
#define END_INTERRUPT_PIN_NUMBER 	13

#define GPIO_SIG_OFFSET 			100
/*******************************************************************************
 * types
 ******************************************************************************/
typedef enum {	
	GATE_CTRL_CP,
	GATE_CTRL_D,
	BCC_RESET,
	NFC_HPD,
	AUTH_WAKE,
	EXT_OUT1,
	BCC_CS,
	OVERCURRNT,
	SBC_WAKE,
	GATE_RS, 
	SBC_LIMP,
	BCC_FAULT,
	NFC_ED,
	RST_N
}pinEnum_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief 	This function will read the state a gpio pin.
 * 			it can be used to see if the pin is high or low
 * 			the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 * 			and it needs to be added to the array in <chip>_gpio.c
 * 		
 * @return 	1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 * 			if(lvGpioReadVal == -1)
 *			{
 *				// do something with the error
 *			}
 * 			else if(lvGpioReadVal)
 *			{
 *				// do something
 *			}
 */
int gpio_init(void);

/*!
 * @brief 	This function will read the state a gpio pin.
 * 			it can be used to see if the pin is high or low
 * 			the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 * 			and it needs to be added to the array in <chip>_gpio.c
 * 			
 * @param 	which pin to read from the pinEnum_t enum. 
 *
 * @return 	1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 * 			if(lvGpioReadVal == -1)
 *			{
 *				// do something with the error
 *			}
 * 			else if(lvGpioReadVal)
 *			{
 *				// do something
 *			}
 */
int gpio_readPin(pinEnum_t pin);

/*!
 * @brief 	This function write a value to a gpio pin.
 * 			it can be used to set the pin high or low
 * 			the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 * 			and it needs to be added to the array in <chip>_gpio.c 	
 *		
 * @param 	which pin to set from the pinEnum_t enum. 
 * @param 	the new value of the pin, 1 is high and 0 is low 
 *
 * @return 	0 if succesfull, -1 if there is an error
 * @example if(gpio_writePin(GATE_CTRL_CP, 1))
 *			{
 *				// do something with the error
 *			}
 */
int gpio_writePin(pinEnum_t pin, bool newValue);

/*!
 * @brief   This function can be used to register a function as 
 *      	interrupt service routine (ISR)
 * @warning This function should be called in a thread that is running (not ended) when the interrupt occurs!
 *      	if the thread pid doesn't exist any more, it will not go to the ISR 
 * @warning only 2 pins can be registered as interrupt
 *    
 * @param   which pin to set the ISR for from the pinEnum_t enum. 
 * @param   The ISR handle function
 *      	pinISRHandler = void handler(int sig);
 * @param 	num which pin is registered 0 or 1 
 * 			note: 0 = SIGUSR1 and 1 = SIGUSR2
 * 			note: the previous signal to ISR is gone 
 *
 * @return  0 if succesfull, -1 if there is an error
 * @example if(gpio_registerISR(SBC_WAKE, handler))
 *     	 	{
 *      	  // do something with the error
 *      	}
 *      
 *      	void handler(int sig)
 *      	{
 *      	  printf("pin %d value: %d\n", sig, gpio_readPin(sig));
 *      	}
 */
int gpio_registerISR(pinEnum_t pin, _sa_handler_t  pinISRHandler, bool num);


/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* GPIO_H_ */
