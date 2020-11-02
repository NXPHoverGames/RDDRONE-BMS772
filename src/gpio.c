/****************************************************************************
 * nxp_bms/BMS_v1/src/gpio.c
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

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/*! @brief  mutex for the filedescriptor */
static pthread_mutex_t gFileDescriptorLock;
static bool gFileDescriptorLockInitialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*!
 * @brief   This function will be used to set or get a file descriptor that is open
 *          This file descriptor can be used in the IOCTL call.
 *
 * @param   setNGet true if you want to set the file descriptio, false to get it
 * @param   pin for which pin you want to get or set the descriptor from the pinEnum_t
 * @param   newFileDescriptor if setNGet == true, this wil be the new file descriptor
 *          This needs to be an opened file descriptor
 *    
 * @return  Returns the file descriptor when setNGet == false, -1 if error
 *      
 */
int setNGetFileDescriptior(bool setNGet, pinEnum_t pin, int newFileDescriptor);

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the GPIO pins
 *          it will open devices for each pin and the file descriptor will be 
 *          saved, to ensure quick acces when reading or writing
 *    
 * @return  0 if there is no error, otherwise the error number will indicate the error
 * @example if(gpio_init())
 *          {
 *            // do something with the error
 *          }
 *      
 */
int gpio_init(void)
{
  int lvRetValue = 0;
  char devPath[DEVPATH_LENGHT_TO_99 + 8];
  int i, lvFd, lvErrorCode;

  // check if initialized
  if(!gFileDescriptorLockInitialized)
  {
    cli_printf("SELF-TEST START: GPIO\n");

    lvRetValue = -1;

    // initialize the mutex
    pthread_mutex_init(&gFileDescriptorLock, NULL);

    // go through each pin
    for(i = 0; i < END_INTERRUPT_PIN_NUMBER+1; i++)
    {
      // check what kind of pin it is
      if(i < START_OUTPUT_PIN_NUMBER)
      {
        // it is an input pin
        sprintf(devPath, "/dev/gpin%u", i);
      }
      else if (i < START_INTERRRUPT_PIN_NUMBER)
      {
        // it is an output pin
        sprintf(devPath,"/dev/gpout%u", i);
      }
      else
      {
        // it is an interrupt pin
        sprintf(devPath, "/dev/gpint%u", i);
      }

      //Open the pin driver 
      lvFd = open(devPath, O_RDWR);
      if (lvFd < 0)
      {
        lvErrorCode = errno;
        cli_printf("gpio_init ERROR: Failed to open %s: %d\n", devPath, lvErrorCode);
        return lvRetValue;
      }

      // save the file desriptor
      if(setNGetFileDescriptior(true, i, lvFd))
      {
        // there is an error 
        cli_printf("gpio_init ERROR: Failed to save file descriptor %s: %d\n", devPath, i);
        return lvRetValue;
      }

    }

    // set the return value to 0
    lvRetValue = 0;

    // set the initialzed variable
    gFileDescriptorLockInitialized = true;

    cli_printf("SELF-TEST PASS:  GPIO\n");
  }

  // return to the user
  return lvRetValue;

}

/*!
 * @brief   This function will read the state a gpio pin.
 *      it can be used to see if the pin is high or low
 *      the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *      and it needs to be added to the array in <chip>_gpio.c
 *      
 * @param   which pin to read from the pinEnum_t enum. 
 *
 * @return  1 if the pin is high, 0 if the pin in low, -1 if there is an error
 * @example lvGpioReadVal = gpio_readPin(GATE_RS);
 *      if(lvGpioReadVal == -1)
 *      {
 *        // do something with the error
 *      }
 *      else if(lvGpioReadVal)
 *      {
 *        // do something
 *      }
 */
int gpio_readPin(pinEnum_t pin)
{
  int lvRetValue = -1, lvError = 0;
  bool lvInvalue;
  int lvFd = -1;
  int lvErrorCode;

  // get the file descriptor
  lvFd = setNGetFileDescriptior(false, pin, -1);

  // Read the pin value 
  if(lvFd >= 0)
  {
    lvRetValue = ioctl(lvFd, GPIOC_READ, (unsigned long)((uintptr_t)&lvInvalue));
    if (lvRetValue < 0)
    {
      lvErrorCode = errno;
      cli_printf("GPIO ERROR: Failed to read value from %d: %d\n", pin, lvErrorCode);
      //close(lvFd);
      lvError += ERROR_READ_DEV;
      //return lvRetValue;
    }
  }
  else
  {
    // error 
    lvError += ERROR_OPEN_DEV;
    cli_printf("GPIO ERROR: Failed to get file descriptor %d: %d\n", pin, lvFd);
  }

  if(!lvError)
  {
    // set the return value
    lvRetValue = lvInvalue;
  }

  // return the value
  return lvRetValue;
}

/*!
 * @brief   This function write a value to a gpio pin.
 *      it can be used to set the pin high or low
 *      the pin needs to be defined in the specific board file (like rrdrone-bms772.h)
 *      and it needs to be added to the array in <chip>_gpio.c  
 *    
 * @param   which pin to set from the pinEnum_t enum. 
 * @param   the new value of the pin, 1 is high and 0 is low 
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

  // get the file descriptor
  lvFd = setNGetFileDescriptior(false, pin, -1);

  // check for an error
  if(lvFd >= 0)
  {
    // write the GPIO pin
    lvRetValue = ioctl(lvFd, GPIOC_WRITE, (unsigned long)newValue);
    if (lvRetValue < 0)
    {
      lvErrorCode = errno;
      cli_printf("GPIO ERROR: Failed to write value %u from %d: %d\n",
         (unsigned int)newValue, pin, lvErrorCode);

      // set the error value
      lvError += ERROR_WRITE_DEV;
    }
  }
  else
  {
    lvError += ERROR_OPEN_DEV;
    cli_printf("GPIO ERROR: Failed to get file descriptor %s: %d\n", pin, lvFd);
  }
   
  // read the pin value 
  if(!lvError)
  {
    lvRetValue = ioctl(lvFd, GPIOC_READ, (unsigned long)((uintptr_t)&lvInvalue));
    if (lvRetValue < 0)
    {
      lvErrorCode = errno;
      cli_printf("GPIO ERROR: Failed to read value from %d: %d\n", pin, lvErrorCode);

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

  // return the value
  return lvRetValue;
}

/*!
 * @brief   This function can be used to register a function as 
 *          interrupt service routine (ISR)
 * @warning This function should be called in a thread that is running (not ended) when the interrupt occurs!
 *          if the thread pid doesn't exist any more, it will not go to the ISR 
 * @warning only 2 pins can be registered as interrupt
 *    
 * @param   which pin to set the ISR for from the pinEnum_t enum. 
 * @param   The ISR handle function
 *          pinISRHandler = void handler(int sig);
 * @param   num which pin is registered 0 or 1 
 *          note: 0 = SIGUSR1 and 1 = SIGUSR2
 *          note: the previous signal to ISR is gone 
 *
 * @return  0 if succesfull, -1 if there is an error
 * @example if(gpio_registerISR(SBC_WAKE, handler))
 *          {
 *            // do something with the error
 *          }
 *        
 *          void handler(int sig)
 *          {
 *            cli_printf("pin %d value: %d\n", sig, gpio_readPin(sig));
 *          }
 */
int gpio_registerISR(pinEnum_t pin, _sa_handler_t  pinISRHandler, bool num)
{
  int signalNumber = SIGUSR1;
  int lvRetValue = -1, lvError = 0;
  int lvFd = -1;
  int lvErrorCode;
  struct sigevent notify;
  _sa_handler_t sigRetValue;

  // check if the signalnumber needs to change
  if(num)
  {
    // change the number
    signalNumber = SIGUSR2;
  }

  // get the file descriptor
  lvFd = setNGetFileDescriptior(false, pin, -1);

  // check for errors
  if(lvFd < 0)
  {
    lvError += ERROR_OPEN_DEV;
    cli_printf("GPIO ERROR: Failed to get file descriptor %s: %d\n", pin, lvFd);
  }

  // check if it all went ok
  if(!lvError)
  {

    //cli_printf("setting notify %d!\n", pin);

    // set the notify signal
    notify.sigev_notify = SIGEV_SIGNAL;
    notify.sigev_signo  = signalNumber;//pin;

    // register the interrupt 
    lvRetValue = ioctl(lvFd, GPIOC_REGISTER, (unsigned long)&notify);
    if (lvRetValue < 0)
    {
      lvErrorCode = errno;
      cli_printf("GPIO ERROR: Failed to register interrupt for %d: %d\n",
         pin, lvErrorCode);

      // set the error value
      lvError += ERROR_REGISTER_INT;
    }

  }

  // check if it all went ok
  if(!lvError)
  {
    // register the handler
    sigRetValue = signal(signalNumber, pinISRHandler);
    
    // check for errors
    if(sigRetValue == SIG_ERR)
    {
      // check errno for the error
      lvErrorCode = errno;
      cli_printf("GPIO ERROR: Failed to register signal to ISR pin: %d sig: %d err:%d\n", pin, (pin+GPIO_SIG_OFFSET), lvErrorCode);

      // set the errorvalue
      lvError += ERROR_REGISTER_ISR;
    }
  }

  // return
  return lvRetValue;
}

/*!
 * @brief   This function will be used to set or get a file descriptor that is open
 *          This file descriptor can be used in the IOCTL call.
 *
 * @param   setNGet true if you want to set the file descriptio, false to get it
 * @param   pin for which pin you want to get or set the descriptor from the pinEnum_t
 * @param   newFileDescriptor if setNGet == true, this wil be the new file descriptor
 *          This needs to be an opened file descriptor
 *    
 * @return  Returns the file descriptor when setNGet == false, -1 if error
 *      
 */
int setNGetFileDescriptior(bool setNGet, pinEnum_t pin, int newFileDescriptor)
{
  int lvRetValue = -1;
  static int descriptorArr[END_INTERRUPT_PIN_NUMBER+1], i;
  static bool firstTime = true;

  // check if first time
  if(firstTime)
  {
    // initialize the array
    for(i = 0; i < END_INTERRUPT_PIN_NUMBER+1; i++)
    {
      // set the error value
      descriptorArr[i] = -1;
    }

    // set the firstTime value off
    firstTime = false;
  }

  // check if it is get or set
  // if get
  if(!setNGet)
  {
    // output the descriptor 
    // lock the mutex
    pthread_mutex_lock(&gFileDescriptorLock);

    // save the descriptor
    lvRetValue = descriptorArr[pin];

    // unlock the mutex
    pthread_mutex_unlock(&gFileDescriptorLock);
  }
  // if set
  else
  {
    // check for errors
    if(newFileDescriptor < 0)
    {
      cli_printf("GPIO ERROR: newFileDescriptor < 0!\n");
      return lvRetValue;
    }
    // write the descriptor
    // lock the mutex
    pthread_mutex_lock(&gFileDescriptorLock);

    // save the new descriptor
    descriptorArr[pin] = newFileDescriptor;

    // unlock the mutex
    pthread_mutex_unlock(&gFileDescriptorLock);

    // set lvretvalue to 0
    lvRetValue = 0;
  }

  // return to the user
  return lvRetValue;
}
