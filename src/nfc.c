/****************************************************************************
 * nxp_bms/BMS_v1/src/nfc.c
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

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include "nfc.h"
#include "cli.h"
#include "gpio.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define NTAG5_SLAVE_ADR           0x54 
#define SCL_FREQ                  400000

#define I2C_SLAVE_CONF_REG_ADR1   0x10
#define I2C_SLAVE_CONF_REG_ADR2   0xA9
#define I2C_SLAVE_CONF_REG_BYTE   0x0 


/****************************************************************************
 * Private Variables
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gNfcInitialized = false;  
const char i2c_path_nfc[] = "/dev/i2c0";  

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the NFC
 *          it will test the i2C connection with the chip and read the slave address
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int nfc_initialize(void)
{
  int lvRetValue = 0;
  uint8_t regVal[1] = {0}, writeVal[3]; 
  int fd;
  struct i2c_msg_s i2c_msg[2];  
  struct i2c_transfer_s i2c_transfer;  
  //uint8_t i = 0; 

  if(!gNfcInitialized)
  {
    cli_printf("SELF-TEST NFC: START\n");

    // set the register address of the i2c slave configuration
    writeVal[0] = I2C_SLAVE_CONF_REG_ADR1;
    writeVal[1] = I2C_SLAVE_CONF_REG_ADR2;
    writeVal[2] = I2C_SLAVE_CONF_REG_BYTE;

    // open the i2c device 
    fd = open(i2c_path_nfc, O_RDONLY);  
    
    // check for errors
    if (fd < 0)  
    { 
      // get the error 
      lvRetValue = -errno;  

      // output to the user
      cli_printfError("nfc ERROR: Can't open i2c device, error: %d\n", lvRetValue);

      // return error
      return lvRetValue; 
    }  
    
    // make the 2 part write message with the register address to read from
    i2c_msg[0].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 3;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the read message
    i2c_msg[1].addr   = NTAG5_SLAVE_ADR;  
    i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
    i2c_msg[1].buffer = regVal;  
    i2c_msg[1].length = 1;  
    i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
    i2c_transfer.msgc = 2;  
      
    // the read register should be equal to the slave address
    /* do the i2C transfer to read the register */  
    lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

    // check for errors
    if(lvRetValue < 0)  
    {  
      // output to the user
      cli_printfError("nfc ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

      // close the file descriptor
      close(fd);  

      // return to the user
      return lvRetValue;  
    }

    // there is a device that reacts to the slave address

    // close the file descriptor
    close(fd);  

    // check if the value is not equal to the slave address
    if((regVal[0] & 0x7F) != NTAG5_SLAVE_ADR)
    {
      // output to the user
      cli_printfError("nfc ERROR: slave address is not equal!\n");

      cli_printfError("Can't verify NFC chip!\n");

      // set the returnvalue 
      lvRetValue = -1;

      // the expected registervalue is not what it should be

      // return to the user
      return lvRetValue;
    }

    //cli_printf("NFC chip (NTAG5) I2C communication verified!\n");

    // say it is initialized
    gNfcInitialized = true;

    cli_printf("SELF-TEST NFC: \e[32mPASS\e[39m\n");

    // set the NFC not in HPD mode to test the GPIO
    lvRetValue = nfc_setHPD(false);

    // check if there is an error
    if(lvRetValue)
    {
      // output that it failed
      cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }

    // put the NFC chip in hard power-down mode because it is not yet used 
    cli_printfWarning("WARNING: putting NFC in hard power-down mode\n");
    lvRetValue = nfc_setHPD(true);

    // check if there is an error
    if(lvRetValue)
    {
      // output that it failed
      cli_printf("SELF-TEST GPIO: \e[31mFAIL\e[39m\n");
    }
  }

  // return to the user
  return lvRetValue;
}

/*!
 * @brief   This function can be used to set the hard power-down (HPD) mode of the NFC 
 *    
 * @param   HPD if true, the microcontroller will set the NFC chip in hard power-down mode. 
 *          if false, it will disable this mode.
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(nfc_setHPD())
 *          {
 *            // do something with the error
 *          }
 */
int nfc_setHPD(bool HPD)
{
  int lvRetValue;

  // set the HPD (hard power-down) pin of the NFC high to consume power
  lvRetValue = gpio_writePin(NFC_HPD, HPD);

  // check for errors
  if(lvRetValue)
  {
    cli_printfError("nfc ERROR: could not set the HPD pin to %d!\n", HPD);
  } 

  // return
  return lvRetValue;
}
