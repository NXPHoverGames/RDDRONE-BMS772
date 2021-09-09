/****************************************************************************
 * nxp_bms/BMS_v1/src/gpio.c
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

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

#include <nuttx/ioexpander/gpio.h>
#include "gpio.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

#define DEVPATH_LENGHT_TO_99    12

#define ERROR_MALLOC_DEVPATH    1
#define ERROR_MALLOC_NUB_BUFFER 2
#define ERROR_OPEN_DEV          4
#define ERROR_READ_DEV          8
#define ERROR_WRITE_DEV         16
#define ERROR_WRONG_PIN         32
#define ERROR_REGISTER_INT      64
#define ERROR_REGISTER_ISR      128
#define ERROR_GET_PINTYPE       256
#define ERROR_SET_PINTYPE       512

/****************************************************************************
 * Private Variables
 ****************************************************************************/
static bool gGPIOInitialized = false;

/*! @brief value to keep track of the pins registered for the interrupt.
 *  @note bitmask with the pinEnum_t
 */
static uint16_t gInterruptPinsISR = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
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
int gpio_init(bool skipSelfTest)
{
    int lvRetValue = 0;

    // check if initialized
    if(!gGPIOInitialized)
    {
        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST GPIO: START\n");
        }

        lvRetValue = -1;

        // set the initialzed variable
        gGPIOInitialized = true;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            /* Only test this pin OUTPUT if it is configured as output */

            if(PTE8 < START_INTERRRUPT_PIN_NUMBER)
            {
                cli_printfWarning("WARNING: Toggling PTE8 to high and back to low!\n");

                // check if the PTE8 pin can be written
                // write the PTE8 pin to high
                lvRetValue = gpio_writePin(PTE8, 1);

                // check if it went wrong
                if(lvRetValue)
                {
                    cli_printfError("GPIO ERROR: writing PTE8 high went wrong!\n");
                    cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
                    return lvRetValue;
                }

                // write the PTE8 pin to low
                lvRetValue = gpio_writePin(PTE8, 0);

                // check if it went wrong
                if(lvRetValue)
                {
                    cli_printfError("GPIO ERROR: writing PTE8 low went wrong!\n");
                    cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
                    return lvRetValue;
                }
            }
            /* If the GPIO is an input */
            else
            {
                // it went OK
                lvRetValue = 0;
            }
        }
        else
        {
            // it went OK
            lvRetValue = 0;
        }
        
        /* other pins will be tested during the initialize of the other parts */
    }

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   This function will read the state a gpio pin.
 *          it can be used to see if the pin is high or low
 *          the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *          and it needs to be added to the array in <chip>_gpio.c
 *      
 * @param   which pin to read from the pinEnum_t enum. 
 *
 * @return  1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 *          if(lvGpioReadVal == -1)
 *          {
 *            // do something with the error
 *          }
 *          else if(lvGpioReadVal)
 *          {
 *            // do something
 *          }
 */
int gpio_readPin(pinEnum_t pin)
{
    int lvRetValue = -1, lvError = 0;
    bool lvInvalue;
    int lvFd = -1;
    int lvErrorCode;
    char devPath[DEVPATH_LENGHT_TO_99 + 8];

    // make the dev path
    // check what kind of pin it is
    if(pin < START_OUTPUT_PIN_NUMBER)
    {
        // it is an input pin
        sprintf(devPath, "/dev/gpin%u", pin);
    }
    else if(pin < START_INTERRRUPT_PIN_NUMBER)
    {
        // it is an output pin
        sprintf(devPath,"/dev/gpout%u", pin);
    }
    else if(pin <= END_INTERRUPT_PIN_NUMBER)
    {
        // it is an interrupt pin
        sprintf(devPath, "/dev/gpint%u", pin);
    }
    else
    {
        cli_printfError("GPIO ERROR: read pin number not in range: %d\n", pin);
        return lvRetValue;
    }

    //Open the pin driver
    lvFd = open(devPath, O_RDONLY);

    // Read the pin value
    if(lvFd >= 0)
    {
        lvRetValue = ioctl(lvFd, GPIOC_READ, (unsigned long)((uintptr_t)&lvInvalue));
        if(lvRetValue < 0)
        {
            lvErrorCode = errno;
            cli_printfError("GPIO ERROR: Failed to read value from %d: %d\n", pin, lvErrorCode);
            
            lvError += ERROR_READ_DEV;
        }
    }
    else
    {
        // get the error code
        lvErrorCode = errno;
        // error 
        lvError += ERROR_OPEN_DEV;
        cli_printfError("GPIO ERROR: Failed to open %s: %d\n", devPath, lvErrorCode);
    }

    // Check if there are no errors
    if(!lvError)
    {
        // set the return value
        lvRetValue = lvInvalue;
    }

    // close the fd
    close(lvFd);

    // return the value
    return lvRetValue;
}

/*!
 * @brief This function write a value to a gpio pin.
 *        it can be used to set the pin high or low
 *        the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *        and it needs to be added to the array in <chip>_gpio.c  
 *    
 * @param which pin to set from the pinEnum_t enum. 
 * @param the new value of the pin, 1 is high and 0 is low 
 *
 * @return  0 if succesfull, -1 if there is an error
 * @example if(gpio_writePin(GATE_CTRL_CP, 1))
 *      {
 *        // do something with the error
 *      }
 */
int gpio_writePin(pinEnum_t pin, bool newValue)
{
    int lvRetValue = -1, lvError = 0;
    bool lvInvalue;
    int lvFd = -1;
    int lvErrorCode;
    char devPath[DEVPATH_LENGHT_TO_99 + 8];

    // check if the pin is not in the correct range
    if(pin < START_OUTPUT_PIN_NUMBER || pin >= START_INTERRRUPT_PIN_NUMBER)
    {
        // Error
        cli_printfError("GPIO ERROR: write pin number not in range: %d\n", pin);
        return lvRetValue;
    }

    // make the dev path
    sprintf(devPath,"/dev/gpout%u", pin);

    //Open the pin driver 
    lvFd = open(devPath, O_RDONLY);

    // check for no errors
    if(lvFd >= 0)
    {
        // write the GPIO pin
        lvRetValue = ioctl(lvFd, GPIOC_WRITE, (unsigned long)newValue);
        if(lvRetValue < 0)
        {
            lvErrorCode = errno;
            cli_printfError("GPIO ERROR: Failed to write value %u from %d: %d\n",
                (unsigned int)newValue, pin, lvErrorCode);

            // set the error value
            lvError += ERROR_WRITE_DEV;
        }
    }
    else
    {
        lvError += ERROR_OPEN_DEV;
        cli_printfError("GPIO ERROR: Failed to open %s: %d\n", devPath, lvFd);
    }
     
    // if there are no errors
    if(!lvError)
    {
        // read the pin value 
        lvRetValue = ioctl(lvFd, GPIOC_READ, (unsigned long)((uintptr_t)&lvInvalue));

        // Check for an error
        if(lvRetValue < 0)
        {
            lvErrorCode = errno;
            cli_printfError("GPIO ERROR: Failed to read value from %d: %d\n", pin, lvErrorCode);
            
            // set the error variable
            lvError += ERROR_READ_DEV;
        }
    }

    // check if it went right
    if(!lvError && (lvInvalue == newValue))
    {
        // set the return value
        lvRetValue = 0;
    }
    // if the read value is not equal to the written value or there is an error
    else
    {
        // set the return value to -1
        lvRetValue = -1;
    }

    // close the fd 
    close(lvFd);

    // return the value
    return lvRetValue;
}

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
int gpio_registerISR(uint16_t IsrPins, _sa_sigaction_t  pinISRHandler)
{
    int lvRetValue = -1, lvError = 0;
    int lvFd = -1, i;
    int lvErrorCode;
    struct sigevent notify;
    struct sigaction act;
    struct sigaction oact1;
    enum gpio_pintype_e pinType;
    char devPath[DEVPATH_LENGHT_TO_99 + 8];

    // loop through all the bits set in the IsrPins variable 
    for(i = 0; IsrPins >> i; i++)
    {
        // check if the bit is set
        if((IsrPins >> i) & 1)
        {
            // check if there is already an error 
            if(lvError)
            {
                // output to the user
                cli_printfError("GPIO ERROR: not initializing pin ISR %d, there is already an error\n", i);
            }

            // check if the IsrPins bit is not in the correct range
            if((i < START_INTERRRUPT_PIN_NUMBER) || (i > END_INTERRUPT_PIN_NUMBER))
            {
                // set the error and output
                lvError += ERROR_WRONG_PIN;
                cli_printfError("GPIO ERROR: pin not in range: %d <= %d <= %d\n", 
                    START_INTERRRUPT_PIN_NUMBER, i, END_INTERRUPT_PIN_NUMBER);
            }

            // check for errors
            if(!lvError)
            {
                // make the dev path
                // it is an interrupt pin
                sprintf(devPath, "/dev/gpint%u", i);
                
                //Open the pin driver 
                lvFd = open(devPath, O_RDONLY);

                // check for errors
                if(lvFd < 0)
                {
                    lvError += ERROR_OPEN_DEV;
                    cli_printfError("GPIO ERROR: Failed to get file descriptor %s: %d\n", lvFd, i);
                }

                // check if it all went ok
                if(!lvError)
                {
                    // get the pintype
                    lvRetValue = ioctl(lvFd, GPIOC_PINTYPE, (unsigned long)&pinType);
                    if(lvRetValue < 0)
                    {
                        lvErrorCode = errno;
                        cli_printfError("GPIO ERROR: Failed to get pintype for %d: %d\n",
                            i, lvErrorCode);

                        // set the error value
                        lvError += ERROR_GET_PINTYPE;
                    }

                    // check if the pintype is not an interrupt pin
                    // get the pintype
                    if(pinType < GPIO_INTERRUPT_PIN || pinType > GPIO_INTERRUPT_BOTH_PIN)
                    {
                        // set the error and output
                        lvError += ERROR_WRONG_PIN;
                        cli_printfError("GPIO ERROR: pin %d is not an interrupt pin: %d \n", 
                            i, pinType);
                    }

                    // check if no error
                    if(!lvError)
                    {
                        //cli_printf("setting notify %d!\n", i);
                        // set the notify signal
                        notify.sigev_notify = SIGEV_SIGNAL;
                        notify.sigev_signo  = SIGUSR1;

                        // add the pin number to be extracted
                        notify.sigev_value.sival_int = i;

                        // register the interrupt 
                        lvRetValue = ioctl(lvFd, GPIOC_REGISTER, (unsigned long)&notify);
                        if(lvRetValue < 0)
                        {
                            lvErrorCode = errno;
                            cli_printfError("GPIO ERROR: Failed to register interrupt for %d: %d\n",
                                i, lvErrorCode);

                            // set the error value
                            lvError += ERROR_REGISTER_INT;
                        }
                    }
                }

                // close the fd
                close(lvFd);

                // check if it all went ok
                if(!lvError)
                {
                    // Set up so that pinISRHandler will respond to SIGUSR1 
                    memset(&act, 0, sizeof(struct sigaction));
                    act.sa_sigaction = pinISRHandler;
                    act.sa_flags     = SA_SIGINFO;

                    // empty the mask
                    sigemptyset(&act.sa_mask);

                    // register the handler
                    lvError += sigaction(SIGUSR1, &act, &oact1);
                    
                    // check for errors
                    if(lvError)
                    {
                        // check errno for the error
                        lvErrorCode = errno;
                        cli_printfError("GPIO ERROR: Failed to register signal to ISR pin: %d sig: %d err:%d\n",
                            i, (SIGUSR1), lvErrorCode);

                        // set the errorvalue
                        lvError += ERROR_REGISTER_ISR;
                    }

                    // add the pin to the variable that keeps track of the ISR pins
                    gInterruptPinsISR += 1 << i;
                }
            }
        }
    }

    // set the error value
    lvRetValue = lvError;

    // return
    return lvRetValue;
}


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
int gpio_changePinType(pinEnum_t pin, inputPinTypes_t newPinType)
{
    int lvRetValue = 0, lvFd, lvErrorCode;
    enum gpio_pintype_e pinTypeIoctl;
    char devPath[DEVPATH_LENGHT_TO_99 + 8];

    // check if the pin is not in range
    if(pin < START_INTERRRUPT_PIN_NUMBER)
    {
        // error and return
        cli_printfError("GPIO ERROR: pin is not an input pin: %d max: %d\n", 
            pin, START_INTERRRUPT_PIN_NUMBER);

        // set the correct error value
        lvRetValue += ERROR_WRONG_PIN;
    }

    // check if the newPinType is in rage
    if(newPinType >= INPUT_PIN_CONFIGURATIONS)
    {
        // error and return
        cli_printfError("GPIO ERROR: newPinType is not in range: %d max: %d\n", 
            newPinType, INPUT_PIN_CONFIGURATIONS);

        // set the correct error value
        lvRetValue += ERROR_WRONG_PIN;
    }

    // check if the pin is not already used in the ISR
    if(gInterruptPinsISR & 1 << pin)
    {
        // error and return
        cli_printfError("GPIO ERROR: pin %d is already used for ISR\n", 
            pin);

        // set the correct error value
        lvRetValue += ERROR_WRONG_PIN;
    }

    // check if there is no error
    if(!lvRetValue)
    {
        // it is an interrupt pin
        sprintf(devPath, "/dev/gpint%u", pin);

        //Open the pin driver 
        lvFd = open(devPath, O_RDONLY);
        if(lvFd < 0)
        {
            lvErrorCode = errno;
            cli_printfError("GPIO ERROR: Failed to open %s: %d\n", devPath, lvErrorCode);
            lvRetValue += ERROR_OPEN_DEV;
        }

        // check if it all went ok
        if(!lvRetValue)
        {
            // check to which type to set it
            switch(newPinType)
            {
                // in case of the 
                case INPUT_INTERRUPT:
                    // set the correct IOCTL pintype
                    pinTypeIoctl = GPIO_INTERRUPT_BOTH_PIN;
                break;
                case INPUT_PULL_UP:
                    // set the correct IOCTL pintype
                    pinTypeIoctl = GPIO_INPUT_PIN_PULLUP;
                break;
                case INPUT_PULL_DOWN:
                    // set the correct IOCTL pintype
                    pinTypeIoctl = GPIO_INPUT_PIN_PULLDOWN;
                break;
                default:
                    // error
                    cli_printfError("GPIO ERROR: Wrong pintype!!! %d\n", lvFd, newPinType);
                    lvRetValue += ERROR_WRONG_PIN;
                break;
            }

            // Check if no errors
            if(!lvRetValue)
            {
                // register the new pintype
                lvRetValue = ioctl(lvFd, GPIOC_SETPINTYPE, (unsigned long)pinTypeIoctl);
                if (lvRetValue != 0)
                {
                    // error
                    lvErrorCode = errno;
                    cli_printfError("GPIO ERROR: Failed to set new pintype for %d: %d\n",
                         pin, newPinType, lvErrorCode);

                    // set the error value
                    lvRetValue += ERROR_SET_PINTYPE;
                }
            }
        }

        // close the FD
        close(lvFd);
    }

    // return
    return lvRetValue;
}
