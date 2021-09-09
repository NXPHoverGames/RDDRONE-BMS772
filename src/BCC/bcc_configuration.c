/*
 * Copyright 2016 - 2021 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ** ###################################################################
 **     Filename    : bcc_configuration.c
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K118
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Battery Cell Controller (BCC) module - configuration file.
 **         This module contains all functions linked to configuration of BCC6 chip.
 **
 **         Note: INIT register is initialized automatically by the BCC driver.
 **         Note: SYS_CFG_GLOBAL register contains only command GO2SLEEP (no initialization needed).
 **         Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are not initialized.
 **
 ** ###################################################################*/
/*!
 ** @file bcc_configuration.c
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - configuration file.
 **         This module contains all functions linked to configuration of BCC6 chip. \n
 **
 **         Note: INIT register is initialized automatically by the BCC driver. \n
 **         Note: SYS_CFG_GLOBAL register contains only command GO2SLEEP (no initialization needed). \n
 **         Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are not initialized. \n
 **
 ** @note
 **         This module was adapted from BCC SW examples by C. van Mierlo.
 **
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_configuration.h"                  // Include header file
#include "bcc_spiwrapper.h"
#include "cli.h"
#include <math.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*! @brief 0 degree Celsius converted to Kelvin. */
#define NTC_DEGC_0          273.15

#define LAST_CELL_BITVALUE  5
#define ALL_CELL_MASK       0x3F

/*******************************************************************************
 * Private functions declerations
 ******************************************************************************/
/*!  @brief this function can be used to calculate the register value from the temperature 
 *   @param temp the temperature the user wants the register value from as a double or float
 *   @param upperTh true if it is the upper threshold
 *   @return it will return the register value as an uint16
 */
static uint16_t CalcRegFromTemp(float temp, bool upperTh);

/*******************************************************************************
 * Public functions
 ******************************************************************************/
/*!
 * @brief   This function is used to configure the temperature thresholds for the 
 *          TH_ANx_OT and TH_ANx_UT registers. 
 *
 * @warning this should be set after initialization!
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   ANxbits which bit it set represents the ANx to set this threshold for 
 *          for example 0b1101 set the threshold for AN0, AN2 and AN3. max is 0b1111111
 * @param   lowerTH address of the floating point value in Celcius to set the this 
 *          as the lower threshold value. if this is NULL, it will not be set.
 * @param   lowerTH address of the floating point value in Celcius to set the this 
 *          as the upper threshold value. if this is NULL, it will not be set.
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeTempTH(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t ANxbits, float *lowerTH, float *upperTH)
 {
    bcc_status_t lvRetValue = BCC_STATUS_SUCCESS;
    int i;
    uint16_t regVal;
    uint16_t retRegVal;

    // there are 7 AN registers
    for(i = 0; i < 7; i++)
    {
        // check if the bit is set
        if(ANxbits & (1<<i))
        {
            // set the lower threshold if not NULL
            if(lowerTH != NULL)
            {
                // calculate the lower temperature threshold register vaulue 
                regVal = CalcRegFromTemp(*lowerTH, false);

                // write the lower temperature threshold
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, (BCC_REG_TH_AN0_UT_ADDR - i), regVal, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't write lower threshold! %d \n", lvRetValue);
                }

                // read the register
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, (BCC_REG_TH_AN0_UT_ADDR - i), 1, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't read lower threshold! %d \n", lvRetValue);
                }

                // check if it didn't work
                if(regVal != retRegVal)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't set the lower temp threshold! %d != %d \n", regVal, retRegVal);
                }
            }

            // set the upper threshold if not NULL
            if(upperTH != NULL)
            {
                // calculate the lower temperature threshold register vaulue 
                regVal = CalcRegFromTemp(*upperTH, true);

                // write the lower temperature threshold
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, (BCC_REG_TH_AN0_OT_ADDR - i), regVal, &retRegVal);
                
                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't write upper threshold! %d \n", lvRetValue);
                }

                // read the register
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, (BCC_REG_TH_AN0_OT_ADDR - i), 1, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't read upper threshold! %d \n", lvRetValue);
                }

                // check if it didn't work
                if(regVal != retRegVal)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't set the upper temp threshold! %d != %d \n", regVal, retRegVal);
                }
            }
        }
    }

    // return to the user
    return lvRetValue;
 }

/*!
 * @brief   This function is used to enable or disable a GPIO.  
 *          It will set the pin as a digital input when disabled and 
 *          as analog input for ratiometric measurement if enabled
 *
 * @warning this should be set after initialization!  
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   ANxbits which bit it set represents the ANx to enable or disable
 *          for example 0b1101 will change AN0, AN2 and AN3. max is 0b1111111 
 * @param   disable if false it will configure it as analog input for ratiometric measurement
 *          If true it will set it as digital input
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_disableNEnableANx(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t ANxbits, bool disable)
 {
    bcc_status_t lvRetValue = BCC_STATUS_SUCCESS;
    int i;
    uint16_t regVal = 0, regMask = 0;
    uint16_t retRegVal;

    // there are 7 AN registers
    for(i = 0; i < 7; i++)
    {
        // check if the bit is set
        if(ANxbits & (1<<i))
        {
            // update the regval
            regVal |= ((disable<<1) << (i*2));

            // make the mask
            regMask |= (3 << (i*2));
        }
    }

    // update the register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, BCC_REG_GPIO_CFG1_ADDR, regMask, regVal);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: couldn't update GPIO_CFG1! %d \n", lvRetValue);
    }

    // read the register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_GPIO_CFG1_ADDR, 1, &retRegVal);

    // check for error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: couldn't read GPIO_CFG1! %d \n", lvRetValue);
    }

    // check if it didn't work
    if(regVal != (retRegVal & regMask))
    {
        // set the return value to an error
        lvRetValue = BCC_STATUS_PARAM_RANGE;
        
        // report to user
        cli_printfError("bcc_configuration ERROR: couldn't set GPIO_CFG1! %d != %d \n", regVal, (retRegVal &regMask));
    }

    return lvRetValue;
 }

 /*!
 * @brief   This function is used to configure the threshold for an ANx input
 *
 * @warning this should be set after initialization!  
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   ANxbits which bit it set represents the ANx to set this threshold for 
 *          for example 0b1101 set the threshold for AN0, AN2 and AN3. max is 0b1111111
 * @param   lowerTH address of the uint16_t value in millivoltage to set the this 
 *          as the lower threshold value. if this is NULL, it will not be set.
 * @param   lowerTH address of the uint16_t value in millivoltage to set the this 
 *          as the upper threshold value. if this is NULL, it will not be set.
 *
 * @warning an OV will appear as an UT and UV will appear as an OT
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeANxVTH(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t ANxbits, uint16_t *lowerTH, uint16_t *upperTH)
 {
    bcc_status_t lvRetValue = BCC_STATUS_SUCCESS;
    int i;
    uint16_t regVal;
    uint16_t retRegVal;

    // there are 7 AN registers
    for(i = 0; i < 7; i++)
    {
        // check if the bit is set
        if(ANxbits & (1<<i))
        {
            // set the lower threshold if not NULL
            if(lowerTH != NULL)
            {
                // calculate the lower temperature threshold register vaulue 
                regVal = BCC_SET_ANX_OT_TH(*lowerTH);

                // write the lower temperature threshold
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, (BCC_REG_TH_AN0_OT_ADDR - i), regVal, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't write lower threshold! %d \n", lvRetValue);
                }

                // read the register
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, (BCC_REG_TH_AN0_OT_ADDR - i), 1, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't read lower threshold! %d \n", lvRetValue);
                }

                // check if it didn't work
                if(regVal != retRegVal)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't set the lower temp threshold! %d != %d \n", regVal, retRegVal);
                }

            }

            // set the upper threshold if not NULL
            if(upperTH != NULL)
            {
                // calculate the lower temperature threshold register vaulue 
                regVal = BCC_SET_ANX_UT_TH(*upperTH);

                // write the lower temperature threshold
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, (BCC_REG_TH_AN0_UT_ADDR - i), regVal, &retRegVal);
                
                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't write upper threshold! %d \n", lvRetValue);
                }

                // read the register
                lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, (BCC_REG_TH_AN0_UT_ADDR - i), 1, &retRegVal);

                // check for error
                if(lvRetValue != BCC_STATUS_SUCCESS)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't read upper threshold! %d \n", lvRetValue);
                }

                // check if it didn't work
                if(regVal != retRegVal)
                {
                    // report to user
                    cli_printfError("bcc_configuration ERROR: couldn't set the upper temp threshold! %d != %d \n", regVal, retRegVal);
                }
            }
        }
    }

    // return to the user
    return lvRetValue;
 }

 /*!
 * @brief   This function is used to configure the temperature thresholds for the 
 *          BCC_REG_TH_ALL_CT register (all (commom) cell thresholds).  
 *
 * @warning this should be set after initialization!
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   lowerTH address of the floating point value in volt to set the this 
 *          as the lower threshold value. if this is NULL, it will not be set.
 * @param   lowerTH address of the floating point value in volt to set the this 
 *          as the upper threshold value. if this is NULL, it will not be set.
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_ChangeCellVTH(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    float *lowerTH, float *upperTH)
 {
    bcc_status_t lvRetValue = BCC_STATUS_SUCCESS;
    uint16_t regVal;
    uint16_t retRegVal;
    uint32_t intValue;

    // get the previous register value
    lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_TH_ALL_CT_ADDR, 1, &regVal);

    // check if it didn't work
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: couldn't read cell voltage thresholds! %d \n", lvRetValue);
    }
    else
    {
        // set the lower threshold if not NULL
        if(lowerTH != NULL)
        {
            //remove the theshold bits from the register value
            regVal &= ~((uint16_t)(0xFF << BCC_RW_ALL_CT_UV_TH_SHIFT));

            // convert to integer *1000
            intValue = (uint32_t)(round((*lowerTH)*1000));

            // calculate the lower voltage threshold register value and set the bits
            regVal |= BCC_SET_ALL_CT_UV_TH((uint16_t)intValue);
        }

        // set the upper threshold if not NULL
        if(upperTH != NULL)
        {
            //remove the theshold bits from the register value
            regVal &= ~((uint16_t)(0xFF << BCC_RW_ALL_CT_OV_TH_SHIFT));

            // convert to integer * 1000 (mV)
            intValue = (uint32_t)(round((*upperTH)*1000));

            // calculate the higher voltage threshold register value (19,53mV/LSB)
            intValue = (uint32_t)(((intValue) * 100U) / 1953U);

            // check if the wanted TH value is higher than the average with the next point (19,53mV steps)
            if(((((uint32_t)(round((*upperTH)*100000))) - (1953*intValue)) / 1953.0) > 0.5)
            {
                // Add 2 to be sure to not trigger too soon on the cell ov 
                intValue = intValue + 2;
            }
            // if the value to be set just a little lower than the wanted value
            else
            {
                // add 1 to be sure the threshold for the BCC is a little higher 
                intValue = intValue + 1;
            }

            // shift the bits correctly and add it to the registervalue
            regVal |= (uint16_t)intValue << BCC_RW_ALL_CT_OV_TH_SHIFT;

            // output to the user
            cli_printf("BCC overvoltage set to %dmV\n", (intValue * 1953) / 100);
        }

        // write the lower temperature threshold
        lvRetValue |= bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, (BCC_REG_TH_ALL_CT_ADDR), 
            regVal, &retRegVal);

        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            // report to user
            cli_printfError("bcc_configuration ERROR: write TH_CTx failed! error: %d \n", lvRetValue);
        }

        // read the register
        lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, (BCC_REG_TH_ALL_CT_ADDR), 
            1, &retRegVal);

        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            // report to user
            cli_printfError("bcc_configuration ERROR: read TH_CTx failed! error: %d \n", lvRetValue);
        }

        // check if it didn't work
        if(regVal != retRegVal)
        {
            // report to user
            cli_printfError("bcc_configuration ERROR: couldn't set the cell voltage thresholds! %d != %d \n", regVal, retRegVal);
        }
    }

    // return to the user
    return lvRetValue;
 }

/*!
 * @brief   This function is used to configure the current thresholds in sleep mode.  
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   sleepCurrentmA the overcurrent threshold in sleep mode [mA]
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeSleepITH(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t sleepCurrentmA)
 {
    bcc_status_t lvRetValue;
    uint16_t regVal, retRegVal;

    // write the registervalue
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_TH_ISENSE_OC_ADDR, 1, &retRegVal);

    // calculate the registervalue
    regVal = BCC_SET_TH_ISENSE_OC(sleepCurrentmA*SHUNT_RESISTOR);

    // write the registervalue
    lvRetValue = bcc_spiwrapper_BCC_Reg_Write(drvConfig, cid, BCC_REG_TH_ISENSE_OC_ADDR, regVal, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: write TH_ISENSE failed! error: %d \n", lvRetValue);
    }

    // write the registervalue
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_TH_ISENSE_OC_ADDR, 1, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: read TH_ISENSE failed! error: %d \n", lvRetValue);
    }

    // check if it didn't work
    if(regVal != retRegVal)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: couldn't set the I sense threshold! %d != %d \n", regVal, retRegVal);
    }

    // return to the user
    return lvRetValue;
 }

 /*!
 * @brief   This function is used to configure the CYCLIC_TIMER time
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   newTimeS [s] the new cyclic timer value (can be 0, 1, 2, 4 or 8s) will be rounded down
 *          if it is not one of those. 0 is continuous measurements
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeCyclicTimer(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t newTimeS)
 {
    bcc_status_t lvRetValue;
    uint16_t regVal = BCC_CYCLIC_TIMER_CONTINOUS;

    // calculate the new time
    if(newTimeS == 1)
    {
        // set to 1s
        regVal = BCC_CYCLIC_TIMER_1S;
    }
    else if (newTimeS < 4)
    {
        // set to 2s
        regVal = BCC_CYCLIC_TIMER_2S;
    }
    else if (newTimeS < 8)
    {
        // set to 4s
        regVal = BCC_CYCLIC_TIMER_4S;
    }
    else 
    {
        // set to 8s
        regVal = BCC_CYCLIC_TIMER_8S;
    }

    // update the register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, BCC_REG_SYS_CFG1_ADDR, BCC_RW_CYCLIC_TIMER_MASK, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: update cyclic_timer failed! error: %d \n", lvRetValue);
    }

    // return to the user
    return lvRetValue;
 }

  /*!
 * @brief   This function is used to configure the odd or even cell count and should be used when
 *          a new cell count is entered
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   newCellCount the new amount of cells
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeCellCount(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t newCellCount)
 {
    bcc_status_t lvRetValue;
    uint16_t regVal, retRegVal; 
    int i;

    // check for error input
    if(newCellCount < 0 || newCellCount > 6)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    // calc if odd or even 
    if (newCellCount % 2)
    {
        // if it is odd
        regVal = BCC_ODD_CELLS;
    }
    else
    {
        // is it is even
        regVal = BCC_EVEN_CELLS;
    }

    // update the register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, 
        BCC_REG_SYS_CFG2_ADDR, BCC_RW_NUMB_ODD_MASK, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: write ODD cells failed! error: %d \n", lvRetValue);
    }

    // read the register to check it
    lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_SYS_CFG2_ADDR, 1, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: read ODD failed! error: %d \n", lvRetValue);
    }

    // check if worked
    if(regVal != (retRegVal & BCC_RW_NUMB_ODD_MASK))
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: update ODD cells failed! %d != %d\n", 
            regVal, (retRegVal &BCC_RW_NUMB_ODD_MASK));
    }

    // reset the regval
    regVal = 0;

    // set the new th enable register 
    for(i = 0; i < newCellCount; i++)
    {
        // check if it is the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            regVal |= 1 << ((6-newCellCount) + i);
        }
        else
        {
            // it is the first 2 cells
            regVal |= 1 << i;
        }       
    }

    //cli_printf("bccCells: %d\n", regVal);

    // update the register 
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, BCC_REG_OV_UV_EN_ADDR, ALL_CELL_MASK, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: write th en cells failed! error: %d \n", 
            lvRetValue);
    }

    // read the register to check it
    lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_OV_UV_EN_ADDR, 1, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: read th en failed! error: %d \n", lvRetValue);
    }

    // check if worked
    if(regVal != (retRegVal & 0x3F))
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: update th en failed! %d != %d\n", 
            regVal, (retRegVal &0x3F));
    }

    // return to the user
    return lvRetValue;
 }

 /*!
 * @brief   This function is used to enable or disable the CC_OVR_FLT mask, 
 *          to set it the fault pin needs to be set or not with this fault.
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   enable if this fault needs to be enabled.
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_setCCOvrFltEnable(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool enable)
{
    bcc_status_t lvRetValue;
    uint16_t regVal, retRegVal;

    // check if it needs to be enabled
    if(enable)
    {
        // set the registervalue
        regVal = BCC_CC_OVR_FLT_EN;
    }
    else
    {
        // set the registervalue
        regVal = BCC_CC_OVR_FLT_DIS;
    }
    
    // update the FAULT_MASK3 register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, BCC_REG_FAULT_MASK3_ADDR, BCC_CC_OVR_FLT_DIS, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: write BCC_CC_OVR_FLT failed! error: %d \n", lvRetValue);
    }

    // read the register to check it
    lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_FAULT_MASK3_ADDR, 1, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: read BCC_CC_OVR_FLT failed! error: %d \n", lvRetValue);
    }

    // check if worked
    if(regVal != (retRegVal & BCC_CC_OVR_FLT_DIS))
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: update BCC_CC_OVR_FLT failed! %d != %d\n", 
            regVal, (retRegVal & BCC_CC_OVR_FLT_DIS));
    }

    // return
    return lvRetValue;
}

 /*!
 * @brief   This function is used to enable or disable the CSB_WUP_FLT mask, 
 *          to set it the fault pin needs to be set or not with this fault.
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   enable if this fault needs to be enabled.
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_setCSbFltEnable(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool enable)
 {
    bcc_status_t lvRetValue;
    uint16_t regVal, retRegVal;

    // check if it needs to be enabled
    if(enable)
    {
        // set the registervalue
        regVal = BCC_CSB_WUP_FLT_EN;
    }
    else
    {
        // set the registervalue
        regVal = BCC_CSB_WUP_FLT_DIS;
    }
    
    // update the FAULT_MASK3 register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Update(drvConfig, cid, BCC_REG_FAULT_MASK1_ADDR, BCC_CSB_WUP_FLT_DIS, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: write BCC_CSB_WUP_FLT failed! error: %d \n", lvRetValue);
    }

    // read the register to check it
    lvRetValue |= bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_FAULT_MASK1_ADDR, 1, &retRegVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: read BCC_CSB_WUP_FLT failed! error: %d \n", lvRetValue);
    }

    // check if worked
    if(regVal != (retRegVal & BCC_CSB_WUP_FLT_DIS))
    {
        // report to user
        cli_printfError("bcc_configuration ERROR: update BCC_CSB_WUP_FLT failed! %d != %d\n", 
            regVal, (retRegVal & BCC_CSB_WUP_FLT_DIS));
    }

    // return
    return lvRetValue;
 }

/*!
 * @brief   his function is used to check if the sleep current threshold mask is enabled, 
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   enabled address of the variable to be true if enabled.
 *
 * @return  BCC error status.
 */
bcc_status_t bcc_configuration_checkSleepCurrentTh(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool *enabled)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t retRegVal[3];

    // check for NULL pointer
    if(enabled == NULL)
    {
        // error
        cli_printfError("bcc_configuration_checkSleepCurrentTh ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // read the register to check it
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_WAKEUP_MASK1_ADDR, 3, retRegVal);

    if(retRegVal[0] & BCC_IS_OC_WAKEUP_DIS)
    {
        cli_printfError("Wake up on IS OC not enabled!\n");
    }

    // read the register to check it
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, cid, BCC_REG_FAULT_MASK1_ADDR, 3, retRegVal);

    // check if the IS_OC mask is enabled and if there was no error getting it
    if((!(retRegVal[0] & BCC_IS_OC_FLT_DIS)) && !lvRetValue)
    {
        // set the variable to true
        *enabled = true;
    }
    else
    {
        // set the variable to false
        *enabled = false;
    }

    return lvRetValue;
}

 /*******************************************************************************
 * Private functions
 ******************************************************************************/
/*!  @brief this function can be used to calculate the register value from the temperature 
 *   @param temp the temperature the user wants the register value from as a double or float
 *   @param upperTh true if it is the upper threshold
 *   @return it will return the register value as an uint16
 */
static uint16_t CalcRegFromTemp(float temp, bool upperTh)
{
    uint16_t lvRetValue;
    float lvmVolt;

    // calculate the NTC voltage in mV
    lvmVolt = (exp(NTC_BETA * ((1.0 / (NTC_DEGC_0 + temp)) -  \
        (1.0 / (NTC_DEGC_0 + NTC_REF_TEMP)))) * NTC_REF_RES);

    lvmVolt= ((NTC_VCOM * 1000 * lvmVolt) / (lvmVolt + NTC_PULL_UP));

    // check if it is the lower or upper threshold
    if(upperTh)
    {
        // set the upper th
        lvRetValue = BCC_SET_ANX_OT_TH(lvmVolt);
    }
    else
    {
        // set the lower
        lvRetValue = BCC_SET_ANX_UT_TH(lvmVolt);
    }
    
    // return the registervalue
    return lvRetValue;
}


 /*******************************************************************************
 * EOF
 ******************************************************************************/
