/****************************************************************************
 * nxp_bms/BMS_v1/src/i2c.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2021 NXP
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

#include "cli.h"
#include "i2c.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/* Device naming */
#define I2C_PATH    "/dev/i2c0"

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/

/*! @brief variable to indicate of it is initialized */
static bool gI2cInitialized = false;  

// this variable is used to determain if the I2C bus may be used
bool gEnableI2cBus = false;

/*! @brief  mutex for the i2c bus */
static pthread_mutex_t gI2cBusLock;
/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

/*!
 * @brief   This function is used to initialize the I2C 
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_initialize(void)
{
    int lvRetValue = 0;

    // check if already initialzed
    if(!gI2cInitialized)
    {
        // initialize the mutex
        pthread_mutex_init(&gI2cBusLock, NULL);

        // set the bus to be enabled
        gEnableI2cBus = true;
    
        // it is now initialzed
        gI2cInitialized = true;
    }

    lvRetValue = !gI2cInitialized;

    // return
    return lvRetValue;
}

/*!
 * @brief   This function can be used to read a data via the I2C
 *    
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   readReg address of the variable to become the read value
 * @param   readBytes the amount of bytes to read max 255
 * @param   useRestart if this is high it will use a restart instead of a stop and start.
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_readData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* readReg, uint8_t readBytes, 
    bool useRestart)
{
    uint8_t writeVal[2]; 
    struct i2c_msg_s i2c_msg[2];  
    struct i2c_transfer_s i2c_transfer;  
    int lvRetValue = 0, fd;

    // check if not initialzed 
    if(!gI2cInitialized)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: i2c not initialzed!\n");
        return -1;
    }

    // check for NULL pointer
    if(readReg == NULL)
    {
        // wrong readReg
        cli_printfError("i2c ERROR: readReg has NULL pointer!\n");
        return -1;
    }

    // make the array with the register addresses
    writeVal[0] = (regAdr >> 8) & 0xFF;
    writeVal[1] = regAdr & 0xFF;

    // make the 2 part write message with the register address to read from
    i2c_msg[0].addr   = slaveAdr;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 2;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  

    // check if restart is needed
    if(useRestart)
    {
        // construct the read transfer
        i2c_msg[1].addr   = slaveAdr;  
        i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
        i2c_msg[1].buffer = readReg;  
        i2c_msg[1].length = readBytes;  
        i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  

        // set the message count to 2
        i2c_transfer.msgc = 2;  
    }
    else
    {
        // set the message count to 1
        i2c_transfer.msgc = 1;  
    }

    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;

    // lock the i2c mutex
    pthread_mutex_lock(&gI2cBusLock);

    // open the i2c device 
    fd = open(I2C_PATH, O_RDONLY);  
    
    // check for errors
    if(fd < 0)  
    { 
      // get the error 
      lvRetValue = -2;  

      // output to the user
      cli_printfError("i2c ERROR: Can't open i2c device, error: %d\n", lvRetValue);
    }  

    // check if the bus is not enabled
    if(!gEnableI2cBus)
    {
        // output to the user
        cli_printfError("i2c ERROR: Bus is not enabled!\n");

        // set the error value
        lvRetValue = -1;
    }

    // check for no errors
    if(!lvRetValue)
    {
        /* do the i2C transfer to read the register */  
        lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

        // check for errors
        if(lvRetValue < 0)  
        {  
            // output to the user
            cli_printfError("i2c read ERROR: Can't do i2c tranfer 1, error: %d\n", 
                lvRetValue);

            // error
            lvRetValue = -1;  
        }
        else
        {
            // If it went OK
            lvRetValue = 0;
        }

        // check if restart is not needed and for no errors
        if(!useRestart && !lvRetValue)
        {
            // make the read message
            i2c_msg[0].addr   = slaveAdr;  
            i2c_msg[0].flags  = I2C_M_READ; /* Write command then sequence read data */  
            i2c_msg[0].buffer = readReg;  
            i2c_msg[0].length = readBytes;  
            i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  

            // make the i2C tranfer 
            i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
            i2c_transfer.msgc = 1;  

            /* do the i2C transfer to read the register */  
            lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

            // check for errors
            if(lvRetValue < 0)  
            {  
                // output to the user
                cli_printfError("i2c read ERROR: Can't do i2c tranfer 2, error: %d\n", 
                    lvRetValue);

                // set the error value
                lvRetValue = -1;
            }
            else
            {
                // if it went OK, set to 0
                lvRetValue = 0;
            }
        } 
    }   

    // close the fd
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&gI2cBusLock);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   This function can be used to write a data via the I2C
 *    
 * @param   slaveAdr the slave address of the I2C device
 * @param   regAdr the address of the register to write to (2 bytes)
 * @param   writeReg address of the variable to write
 * @param   writeBytes the amount of bytes to write max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_writeData(uint8_t slaveAdr, uint16_t regAdr, uint8_t* writeReg, uint8_t writeBytes)
{
    uint8_t writeVal[2 + writeBytes]; 
    struct i2c_msg_s i2c_msg[1];  
    struct i2c_transfer_s i2c_transfer;  
    int lvRetValue = 0, i, fd;

    // check if not initialzed 
    if(!gI2cInitialized)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: i2c not initialzed!\n");
        return -1;
    }

    // check for NULL pointer
    if(writeReg == NULL)
    {
        // wrong writeReg
        cli_printfError("i2c ERROR: writeReg has NULL pointer!\n");
        return -1;
    }

    // make the array with the register addresses and the data to write
    writeVal[0] = (regAdr >> 8) & 0xFF;
    writeVal[1] = regAdr & 0xFF;

    // loop throught the write values
    for(i = 0; i < writeBytes; i++)
    {
        // add the data to write
        writeVal[2+i] = writeReg[i];
    }

    // make the 2 part write message with the register address to write to and the values
    i2c_msg[0].addr   = slaveAdr;  
    i2c_msg[0].flags  = 0;  
    i2c_msg[0].buffer = writeVal;  
    i2c_msg[0].length = 2 + writeBytes;  /* Write address of where we want to read to AT24 */  
    i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  

    // make the i2C tranfer 
    i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
    i2c_transfer.msgc = 1;

    // lock the i2c mutex
    pthread_mutex_lock(&gI2cBusLock);

    // open the i2c device 
    fd = open(I2C_PATH, O_RDONLY);  
    
    // check for errors
    if(fd < 0)  
    { 
      // get the error 
      lvRetValue = -2;  

      // output to the user
      cli_printfError("i2c ERROR: Can't open i2c device, error: %d\n", lvRetValue);
    }  

    // check if the bus is not enabled
    if(!gEnableI2cBus)
    {
        // output to the user
        cli_printfError("i2c ERROR: Bus is not enabled!\n");

        lvRetValue = -1;
    }

    // check for no errors
    if(!lvRetValue)
    {
        /* do the i2C transfer to read the register */  
        lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  

        // check for errors
        if(lvRetValue < 0)  
        {  
            // output to the user
            cli_printfError("i2c write ERROR: Can't do i2c tranfer, error: %d\n", 
                lvRetValue);

            // set the error value
            lvRetValue = -1;
        }
        else
        {
            // If it went OK
            lvRetValue = 0;
        }
    }

    // close the fd
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&gI2cBusLock);

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   This function can be used to read a data byte from the NFC chip session register
 *    
 * @param   slaveAdr the slave address of the I2C NFC device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   byteNum the numer of byte to read from (0-3)
 * @param   readReg address of the variable to become the read value
 * @param   readBytes the amount of bytes to read max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_nfcReadSessionRegByte(uint8_t slaveAdr, uint16_t regAdr, uint8_t byteNum,
  uint8_t* readReg, uint8_t readBytes)
{
    uint8_t writeVal[3]; 
    struct i2c_msg_s i2c_msg[2];  
    struct i2c_transfer_s i2c_transfer;  
    int lvRetValue, fd;
    uint8_t i;

    // check if not initialzed 
    if(!gI2cInitialized)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: i2c not initialzed!\n");
        return -1;
    }

    // check byteNum for valid value
    if(byteNum < 0 || byteNum > 3)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: wrong byteNum (0-3 is allowed): %d\n", 
        byteNum);
        return -1;
    }

    // check for NULL pointer
    if(readReg == NULL)
    {
        // wrong readReg
        cli_printfError("i2c ERROR: readReg has NULL pointer!\n");
        return -1;
    }

    // loop through the register to read them
    for(i = 0; i < readBytes; i++)
    {
        regAdr += (int)(byteNum + i) / (int)4;
        writeVal[0] = (regAdr >> 8) & 0xFF;
        writeVal[1] = regAdr & 0xFF;
        writeVal[2] = (byteNum + i) % 4;

        // make the 2 part write message with the register address to read from
        i2c_msg[0].addr   = slaveAdr;  
        i2c_msg[0].flags  = 0;  
        i2c_msg[0].buffer = writeVal;  
        i2c_msg[0].length = 3;  /* Write address of where we want to read to AT24 */  
        i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  

        // make the read message
        i2c_msg[1].addr   = slaveAdr;  
        i2c_msg[1].flags  = I2C_M_READ; /* Write command then sequence read data */  
        i2c_msg[1].buffer = &readReg[i];  
        i2c_msg[1].length = 1;  
        i2c_msg[1].frequency = SCL_FREQ;  /* 400K bsp */  

        // make the i2C tranfer 
        i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
        i2c_transfer.msgc = 2;      

        // lock the i2c mutex
        pthread_mutex_lock(&gI2cBusLock);

        // open the i2c device 
        fd = open(I2C_PATH, O_RDONLY);  
        
        // check for errors
        if(fd < 0)  
        { 
          // get the error 
          lvRetValue = -1;  

          // output to the user
          cli_printfError("i2c ERROR: Can't open i2c device, error: %d\n", lvRetValue);
        }  

        /* do the i2C transfer to read the register */  
        lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer); 

        // close the fd
        close(fd); 

        // unlock the mutex
        pthread_mutex_unlock(&gI2cBusLock);

        // check for errors
        if(lvRetValue < 0)  
        {  
            // output to the user
            cli_printfError("i2c read ses ERROR: Can't do i2c tranfer, error: %d\n", lvRetValue);

            // return to the user
            return lvRetValue;  
        }
        else
        {
            // If it went OK
            lvRetValue = 0;
        }
    }

  // return to the user
  return lvRetValue;
}

/*!
 * @brief   This function can be used to write data to a byte of the NFC chip session register
 *    
 * @param   slaveAdr the slave address of the I2C NFC device
 * @param   regAdr the address of the register to read from (2 bytes)
 * @param   byteNum the numer of byte to write to (0-3)
 * @param   writeReg address of the variable to write
 * @param   mask 8-bit control register bit mask. Only if corresponding control bit is set to 1b,
 *          the register bit will be overwritten.
 * @param   writeBytes the amount of bytes to write max 255
 *
 * @return  0 if ok, -1 if there is an error
 */
int i2c_nfcWriteSessionRegByte(uint8_t slaveAdr, uint16_t regAdr, uint8_t byteNum,
  uint8_t* writeReg, uint8_t mask, uint8_t writeBytes)
{
    uint8_t writeVal[4 + writeBytes]; 
    struct i2c_msg_s i2c_msg[1];  
    struct i2c_transfer_s i2c_transfer;  
    int lvRetValue = 0, fd;
    uint8_t i;

    // check if not initialzed 
    if(!gI2cInitialized)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: i2c not initialzed!\n");
        lvRetValue = -1;
        return lvRetValue;
    }

    // check byteNum for valid value
    if(byteNum < 0 || byteNum > 3)
    {
        // wrong byteNum
        cli_printfError("i2c ERROR: wrong byteNum (0-3 is allowed): %d\n", 
        byteNum);
        lvRetValue = -1;
        return lvRetValue;
    }

    // check for NULL pointer
    if(writeReg == NULL)
    {
        // wrong writeReg
        cli_printfError("i2c ERROR: writeReg has NULL pointer!\n");
        lvRetValue = -1;
        return lvRetValue;
    }

    // loop through the register to read them
    for(i = 0; i < writeBytes; i++)
    {
        regAdr += (int)(byteNum + i) / (int)4;
        writeVal[0] = (regAdr >> 8) & 0xFF;
        writeVal[1] = regAdr & 0xFF;
        writeVal[2] = (byteNum + i) % 4;
        writeVal[3] = mask;
        writeVal[4] = writeReg[i]; 

        // make the 2 part write message with the register address to read from
        i2c_msg[0].addr   = slaveAdr;  
        i2c_msg[0].flags  = 0;  
        i2c_msg[0].buffer = writeVal;  
        i2c_msg[0].length = 5;  /* Write address of where we want to read to AT24 */  
        i2c_msg[0].frequency = SCL_FREQ;  /* 400K bsp */  

        // make the i2C tranfer 
        i2c_transfer.msgv = (struct i2c_msg_s *)i2c_msg;  
        i2c_transfer.msgc = 1;

        // lock the i2c mutex
        pthread_mutex_lock(&gI2cBusLock);

        // open the i2c device 
        fd = open(I2C_PATH, O_RDONLY);  
        
        // check for errors
        if(fd < 0)  
        { 
          // get the error 
          lvRetValue = -1;  

          // output to the user
          cli_printfError("i2c ERROR: Can't open i2c device, error: %d\n", lvRetValue);
        }  

        // check if there is no error
        if(!lvRetValue)
        {
            /* do the i2C transfer to read the register */  
            lvRetValue = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&i2c_transfer);  
        }

        // close the fd
        close(fd);

        // unlock the mutex
        pthread_mutex_unlock(&gI2cBusLock);

        // check for errors
        if(lvRetValue < 0)  
        {  
            // output to the user
            cli_printfError("i2c write ERROR: Can't do i2c tranfer, error: %d\n", 
                lvRetValue);

            // return to the user
            return lvRetValue;  
        }
        else
        {
            // If it went OK
            lvRetValue = 0;
        }
    }

  // return to the user
  return lvRetValue;
}

/*!
 * @brief   this function will be used to configure if an I2C transmission may be done
 *          
 * @param   enable If this is true the I2C transmision of the bus is enabled, disabled otherwise 
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 * @example if(i2c_enableTransmission(false))
 *          {
 *              // do something with the error
 *          }
 */
int i2c_enableTransmission(bool enable)
{
    int lvRetValue = -1;

    // check if not initialized
    if(!gI2cInitialized)
    {
        // return error
        cli_printfError("i2c ERROR: Isn't initialized!\n");
        return lvRetValue;
    }

    // lock the i2c mutex
    pthread_mutex_lock(&gI2cBusLock);

    // set the variable 
    gEnableI2cBus = enable;

    // unlock the mutex
    pthread_mutex_unlock(&gI2cBusLock);

    // it went ok 
    lvRetValue = 0;

    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/
