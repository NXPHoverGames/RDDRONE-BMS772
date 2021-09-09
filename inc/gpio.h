/****************************************************************************
 * nxp_bms/BMS_v1/inc/gpio.h
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
 ** ###################################################################
 **     Filename    : gpio.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-04-03
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
#define START_OUTPUT_PIN_NUMBER     0
#define START_INTERRRUPT_PIN_NUMBER 5
#define END_INTERRUPT_PIN_NUMBER    11

#define NFC_ED_PIN_ACTIVE           0
#define NFC_ED_PIN_INACTIVE         1
/*******************************************************************************
 * types
 ******************************************************************************/
/*!
 *   @brief this enum could be used to drive the GPIOs of the BMS and reflect the GPIOs from the 
 *          rddrone-bms772.h file.
 *          With START_OUTPUT_PIN_NUMBER stating where the GPIO output pins start
 *          and START_INTERRRUPT_PIN_NUMBER stating where the GPIO (input) interrupt pins start
 */
typedef enum {  
    GATE_CTRL_CP    = 0,
    GATE_CTRL_D     = 1,
    BCC_RESET       = 2,
    NFC_HPD         = 3,
    AUTH_WAKE       = 4,
    PTE8            = 5,
    OVERCURRENT     = 6,
    SBC_WAKE        = 7,
    GATE_RS         = 8, 
    SBC_LIMP        = 9,
    BCC_FAULT       = 10,
    NFC_ED          = 11
}pinEnum_t;

/*!
 *   @brief this enum could be used to change the GPIO input pintype
 */
typedef enum{
    INPUT_INTERRUPT,
    INPUT_PULL_UP,
    INPUT_PULL_DOWN,
    INPUT_PIN_CONFIGURATIONS
}inputPinTypes_t;

/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief   This function will initialze the GPIO pins
 *          it will open devices for each pin and the file descriptor will be 
 *          saved, to ensure quick acces when reading or writing
 *
 * @param   skipSelfTest if this is true it will skip the self-test
 *    
 * @return  0 if there is no error, otherwise the error number will indicate the error
 * @example if(gpio_init(false))
 *          {
 *            // do something with the error
 *          }
 *      
 */
int gpio_init(bool skipSelfTest);

/*!
 * @brief   This function will read the state a gpio pin.
 *          it can be used to see if the pin is high or low
 *          the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *          and it needs to be added to the array in <chip>_gpio.c
 *          
 * @param   pin which pin to read from the pinEnum_t enum. 
 *
 * @return  1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 *          if(lvGpioReadVal == -1)
 *          {
 *              // do something with the error
 *          }
 *          else if(lvGpioReadVal)
 *          {
 *              // do something
 *          }
 */
int gpio_readPin(pinEnum_t pin);

/*!
 * @brief   This function write a value to a gpio pin.
 *          it can be used to set the pin high or low
 *          the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *          and it needs to be added to the array in <chip>_gpio.c  
 *      
 * @param   pin which pin to set from the pinEnum_t enum. 
 * @param   newValue the new value of the pin, 1 is high and 0 is low 
 *
 * @return  0 if succesfull, -1 if there is an error
 * @example if(gpio_writePin(GATE_CTRL_CP, 1))
 *          {
 *              // do something with the error
 *          }
 */
int gpio_writePin(pinEnum_t pin, bool newValue);

/*!
 * @brief   This function can be used to register a function as 
 *          interrupt service routine (ISR) for multiple pins. 
 * @warning This function should be called in a thread that is running (not ended) when the interrupt occurs!
 *          if the thread pid doesn't exist any more, it will not go to the ISR 
 * @note    It will use SIGUSR1 for the pin interrupt 
 * @note    Multiple pins will have the same ISR. 
 *
 * @param   IsrPins Bitfield of the pins which to set the ISR for from the pinEnum_t enum. 
 * @param   pinISRHandler The ISR handle function
 *          pinISRHandler = void handler(int signo, FAR siginfo_t *siginfo, FAR void *context);
 *
 * @return  0 if succesfull, otherwise a number to indicate an error
 * @example if(gpio_registerISR((uint16_t) ((1 << SBC_WAKE) + (1 <<BCC_FAULT)), handler);
 *          {
 *            // do something with the error
 *          }
 *        
 *          void handler(int signo, siginfo_t *siginfo, void *context)
 *          {
 *            int pinNumber = (*siginfo).si_value.sival_int;
 *            cli_printf("cli_printf("GPIO ISR: sig: %d, pin: %d value %d\n", signo, pinNumber, gpio_readPin(pinNumber);
 * 
 *            // do something with the pin
 *          }
 */
int gpio_registerISR(uint16_t IsrPins, _sa_sigaction_t  pinISRHandler);

/*!
 * @brief   This function can be used to register an input pin as a different pin type
 *
 * @param   pin which pin to set from the pinEnum_t enum. 
 * @param   newPinType To which the pin type should change from the inputPinTypes_t enum.
 *
 * @return  0 if succesfull, otherwise a number to indicate an error
 * @example if(gpio_changePinType(PTE8, INPUT_PULL_UP);
 *          {
 *            // do something with the error
 *          }
 */
int gpio_changePinType(pinEnum_t pin, inputPinTypes_t newPinType);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* GPIO_H_ */
