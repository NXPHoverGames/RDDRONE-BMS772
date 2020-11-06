/****************************************************************************
 * nxp_bms/BMS_v1/src/a1007.c
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
#include "a1007.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define A1007_SLAVE_ADR             0x50 
#define SCL_FREQ                    400000

#define STATUS_COMMAND_REG_ADR1     0x09
#define STATUS_COMMAND_REG_ADR2     0x00 

#define STATUS_COMMAND_REG_VAL_MASK 0x0F

/****************************************************************************
 * Private Variables
 ****************************************************************************/
static bool gA1007Initialized = false;  
const char i2c_path_a1007[] = "/dev/i2c0";  

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*!
 * @brief   This function will initialze the a1007
 *          it will test the i2C connection with the chip 
 *    
 * @return  0 if ok, -1 if there is an error
 * @example 
 *          if(a1007_initialize())
 *          {
 *            // do something with the error
 *          }
 */
int a1007_initialize(void)
{
  int lvRetValue = 0;
  uint8_t regVal[2] = {0, 0}, writeVal[2]; 
  int fd;
  struct i2c_msg_s i2c_msg[2];  
  struct i2c_transfer_s i2c_transfer;  
  //uint8_t i = 0; 

  // check if not initialized 
  if(!gA1007Initialized)
  {
    cli_printf("SELF-TEST START: A1007\n");

    // set the register address of the i2c slave configuration
    writeVal[0] = STATUS_COMMAND_REG_ADR1;
    writeVal[1] = STATUS_COMMAND_REG_ADR2;

    // open the i2c device 
    fd = open(i2c_path_a1007, O_RDONLY);  
    
    // check for errors
    if (fd < 0)  
    { 
      // get the error 
      lvRetValue = -errno;  

      // output to the user
      cli_printf("A1007 ERROR: Can't open i2c device, error: %d\n", lvRetValue);

      // return error
      return lvRetValue; 
    }  
    
    // make the 2 part write message with the register address to read from
    i2c_msg[0].addr   = A1007_SLAVE_ADR;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 2;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the read message
    i2c_msg[1].addr   = A1007_SLAVE_ADR;  
    i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
    i2c_msg[1].buffer = regVal;  
    i2c_msg[1].length = 2;  
    i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  
      
    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
    i2c_transfer.msgc = 2;  
      
    // for (trytime = 0; trytime < 20; trytime++)  
    // {      
    /* do the i2C transfer to read the register */  
    lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

    // check for errors
    if (lvRetValue < 0)  
    {  

      // output to the user
      cli_printf("A1007 ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

      // close the file descriptor
      close(fd);  

      // return to the user
      return lvRetValue;  
    }

    // close the file descriptor
    close(fd);  

    // check if there are no wrong status bits
    if((regVal[1] & STATUS_COMMAND_REG_VAL_MASK) != 0)
    {
      // output to the user
      cli_printf("A1007 ERROR: status command has wrong bits!\n");

      cli_printf("Can't verify A1007 chip!\n");

      // set the returnvalue 
      lvRetValue = -1;

      return lvRetValue;
    }

    //cli_printf("A1007 I2C communication verified!\n");
    cli_printf("SELF-TEST PASS:  A1007\n");

    // set that it is initialzed
    gA1007Initialized = true;
  }

  // return to the user
  return lvRetValue;
}
