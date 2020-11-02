/*
 * Copyright 2016 - 2020 NXP
 * All rights reserved.
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
 * ###################################################################
 **     Filename    : bcc_monitoring.c
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K118
 **     Version     : 1.00
 **     Compiler    : GNU C Compiler
 **     Date/Time   : 2020-04-01, 11:00, # CodeGen: 0
 **     Abstract    :
 **         Battery Cell Controller (BCC) module - monitoring functions.
 **         This module contains functions linked to monitoring on BCC6 chip.
 **
 ** ###################################################################*/
/*!
 ** @file bcc_monitoring.c
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - monitoring functions.
 **         This module contains functions linked to monitoring on BCC6 chip. \n
 ** @note
 ** 		This module was adapted from BCC SW examples by C. van Mierlo.
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_monitoring.h" 					// Include header file
#include "data.h"
#include "gpio.h"
#include "cli.h"
#include <errno.h>

//#include <math.h>
#include <float.h>
#include "bcc_configuration.h"
#include <semaphore.h>

#include <nuttx/vt100.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//#define OUTPUT_CURRENT_MEAS_DOT
//#define OUTPUT_UPDATE_OTHER_MEAS

//#define OUTPUT_STACK_VOLTAGE
//#define OUTPUT_BAT_VOLTAGE
//#define OUTPUT_OUTPUT_STATUS
//#define OUTPUT_CURRENT
//#define OUTPUT_TEMP
//#define OUTPUT_CELL_VOLTAGE

//#define OUTPUT_AVG_CURRENT
//#define OUTPUT_AVG_TIME
//#define OUTPUT_AVG_POWER
//#define OUTPUT_STATE_OF_CHARGE
//#define OUTPUT_ENERGY_CONSUMED
//#define OUTPUT_REMAINING_CHARGE

//#define DONT_DO_LAST

#define BCC_REG_MEAS_AN4_ADDR 0x43

/*! @brief Size of NTC look-up table. */
#define NTC_TABLE_SIZE        (NTC_MAXTEMP - NTC_MINTEMP + 1)
/* ! @brief 0 degree Celsius converted to Kelvin. 
// #define NTC_DEGC_0            273.15*/

/*! @brief 0 degree Celsius converted to Kelvin. */
#define NTC_DEGC_0      273.15

/*! @brief miliAmpere converted to A with a division. */
#define MA_TO_A         1000.0

/*! @brief microvolt to volt with a division. */
#define UV_TO_V         1000000.0

/*! @brief 10*temperature to temperature with a division. */
#define T10_TO_T         10.0

#define V2RES            0.6 //!< [μV/LSB] Current sense user register resolution
#define V2RES_DIV_RSHUNT V2RES / SHUNT_RESISTOR_UOHM / 1000000 //!< [(μV/LSB)/uOhm] this times the coulumb count value is current

#define STANDARD_MOVING_AVG_SIZE 10   

/*!
 * @brief Calculates final temperature value.
 *
 * @param tblIdx Index of value in NTC table which is close
 *        to the register value provided by user.
 * @param degTenths Fractional part of temperature value.
 * @return Temperature.
 */
#define NTC_COMP_TEMP(tblIdx, degTenths) \
    ((((tblIdx) + NTC_MINTEMP) * 10) + (degTenths))

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/

static const char gSaveCursor[]         = VT100_SAVECURSOR;
static const char gRestoreCursor[]      = VT100_RESTORECURSOR;

/**
 * NTC look up table intended for resistance to temperature conversion. After
 * table initialization, array item contains raw value from a register.
 * Index of the item is temperature value.
 */
static uint16_t g_ntcTable[NTC_TABLE_SIZE];

/*! @brief this semaphore will be used to read all measurements */
static sem_t gGetOtherMeasurementSem;
/*! @brief to indicate if this semaphore has been initialized */
static bool gGetOtherMeasSemInitialized = false;

static uint8_t gMeasurementCounter = 1;
static uint8_t gMeasurementCounterEndValue = 100;

static float *gMovingAvgArr;
static uint8_t gMovingAvgArrElements = 0; 
static uint8_t gMovingAvgArrIndex = 0; 

uint16_t gShowMeasurements = 0; 
bool gDoTop = false;

/*! @brief  mutex for the calc delta charge function */
static pthread_mutex_t gDChargeFuncMutex;   
static bool dChargeFuncMutexInitialized = false;

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

/*!
 * @brief This function calculates temperature from raw value of MEAS_ANx
 * register. It uses precalculated values stored in g_ntcTable table.
 * You can use function BCC_Meas_GetRawValues to get values of measurement
 * registers.
 *
 * @param regVal Value of MEAS_ANx register.
 * @param temp Temperature value in deg. of Celsius * 10.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t getNtcCelsius(uint16_t regVal, int16_t* temp);


/*******************************************************************************
 * Private functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : getNtcCelsius
 * Description   : This function calculates temperature from raw value of
 *                 MEAS_ANx register.
 *
 *END**************************************************************************/
static bcc_status_t getNtcCelsius(uint16_t regVal, int16_t* temp)
{
    int16_t left = 0;    /* Pointer (index) to the left border of interval
                            (NTC table). */
    int16_t right = NTC_TABLE_SIZE - 1; /* Pointer (index) to the right border
                                           of interval (NTC table). */
    int16_t middle;      /* Pointer (index) to the middle of interval
                            (NTC table). */
    int8_t degTenths;    /* Fractional part of temperature value. */

    BCC_MCU_Assert(temp != NULL);

    /* Check range of NTC table. */
    if (g_ntcTable[NTC_TABLE_SIZE - 1] > regVal)
    {
        *temp = NTC_COMP_TEMP(NTC_TABLE_SIZE - 1, 0);
        return BCC_STATUS_PARAM_RANGE;
    }
    if (g_ntcTable[0] < regVal)
    {
        *temp = NTC_COMP_TEMP(0, 0);
        return BCC_STATUS_PARAM_RANGE;
    }

    regVal &= BCC_GET_MEAS_RAW(regVal);

    /* Search for an array item which is close to the register value provided
    * by user (regVal). Used method is binary search in sorted array. */
    while ((left + 1) != right)
    {
        /* Split interval into halves. */
        middle = (left + right) >> 1U;
        if (g_ntcTable[middle] <= regVal)
        {
            /* Select right half (array items are in descending order). */
            right = middle;
        }
        else
        {
            /* Select left half. */
            left = middle;
        }
    }

    /* Notes: found table item (left) is less than the following item in the
    * table (left + 1).
    * The last item cannot be found (algorithm property). */

    /* Calculate fractional part of temperature. */
    degTenths = (g_ntcTable[left] - regVal) /
            ((g_ntcTable[left] - g_ntcTable[left + 1]) / 10);
    (*temp) = NTC_COMP_TEMP(left, degTenths);

    return BCC_STATUS_SUCCESS;

}


/*******************************************************************************
 * API
 ******************************************************************************/

/*
 * @brief   This function will initialize the semaphore 
 *          this function should be called before bcc_monitoring_updateMeasurements 
 *          Developed by C. van Mierlo. \n
 *
 * @param   none
 *
 * @return  int Error code (of the semaphore)
 */
int bcc_monitoring_initializeSem(void)
{
    int retVal = 0;

    // check if initialized 
    if(!gGetOtherMeasSemInitialized)
    {
        // initialize the semaphore
        retVal = sem_init(&gGetOtherMeasurementSem, 0, 0);
        sem_setprotocol(&gGetOtherMeasurementSem, SEM_PRIO_NONE);

        // check if not succeeded 
        if(retVal)
        {
           // return to the user 
            return retVal;
        }

        // allocate the moving array standard lenght
        gMovingAvgArr = (float*)calloc(STANDARD_MOVING_AVG_SIZE, sizeof(float));

        // set the global value
        gMovingAvgArrElements = STANDARD_MOVING_AVG_SIZE;

        // reset the value 
        gShowMeasurements = 0;
        gDoTop = false;

        // set the gShowMeasurements value
#ifdef OUTPUT_STACK_VOLTAGE 
        gShowMeasurements |= (1<<CLI_STACK_VOLTAGE);
#endif
#ifdef OUTPUT_BAT_VOLTAGE
        gShowMeasurements |= (1<<CLI_BAT_VOLTAGE);
#endif
#ifdef OUTPUT_OUTPUT_STATUS
        gShowMeasurements |= (1<<CLI_OUTPUT_STATUS);
#endif
#ifdef OUTPUT_CURRENT
        gShowMeasurements |= (1<<CLI_CURRENT);
#endif
#ifdef OUTPUT_TEMP
        gShowMeasurements |= (1<<CLI_TEMPERATURE);
#endif
#ifdef OUTPUT_CELL_VOLTAGE
        gShowMeasurements |= (1<<CLI_CELL_VOLTAGE);
#endif
#ifdef OUTPUT_AVG_CURRENT
        gShowMeasurements |= (1<<CLI_AVG_CURRENT);
#endif
#ifdef OUTPUT_AVG_POWER
        gShowMeasurements |= (1<<CLI_AVG_POWER);
#endif
#ifdef OUTPUT_ENERGY_CONSUMED 
        gShowMeasurements |= (1<<CLI_ENERGY_CONSUMED);
#endif
#ifdef OUTPUT_STATE_OF_CHARGE 
        gShowMeasurements |= (1<<CLI_STATE_OF_CHARGE);
#endif        
#ifdef OUTPUT_REMAINING_CHARGE
        gShowMeasurements |= (1<<CLI_REMAINING_CAP);
#endif
        // set the variable
        gGetOtherMeasSemInitialized = true;
    }

    // check if not initialized 
    if(!dChargeFuncMutexInitialized)
    {
        // initialize the mutex
        pthread_mutex_init(&gDChargeFuncMutex, NULL);

        // set the variable to true
        dChargeFuncMutexInitialized = true;
    }

    // return to the user 
    return retVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_FillNtcTable
 * Description   : This function fills the NTC look up table.
 *
 *END**************************************************************************/
void bcc_monitoring_fillNtcTable(const ntc_config_t* const ntcConfig)
{
    double ntcVal, expArg;
    uint16_t i = 0;
    int32_t temp;

    for (temp = NTC_MINTEMP; temp <= NTC_MAXTEMP; temp++)
    {
        // calculate the new exponent arguement (beta*(1/temp(k) - 1/tempRef(k)))
        expArg = ntcConfig->beta * ((1.0 / (NTC_DEGC_0 + temp)) -
                (1.0 / (NTC_DEGC_0 + ntcConfig->refTemp)));

        // calculate the NTC value e^expArg * Rref voltage is ((NTC_VCOM * ntcVal) / (ntcVal + ntcConfig->rntc))
        ntcVal = exp(expArg) * ntcConfig->refRes;

        //calculate the registervalue for each temperature
        g_ntcTable[i] = (uint16_t)round(((NTC_VCOM * ntcVal) /
                (ntcVal + ntcConfig->rntc)) / NTC_REGISTER_RES);

        // increase the index
        i++;
    }
}

/*
 * @brief   This function reads values measured and provided via SPI
 *          by BCC device (ISENSE, cell voltages, temperatures). \n
 *          it will set the values in the data struct
 *          Developed by M. Musak and adapted by C. van Mierlo. \n
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   rShunt Shunt resistor for ISENSE in [uOhm].  
 * @param   lowestCellVoltageAdr this function will set the lowest cell voltage in this variable
 * @param   gateMutexAdr this is the address of the gate mutex, to check a pin for the output status
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_updateMeasurements(bcc_drv_config_t* const drvConfig, uint32_t rShunt, 
    float *lowestCellVoltageAdr, pthread_mutex_t *gateMutexAdr)
{
	/* Variables *****************************************************************/
    static bool firstPowerup = true;
    bcc_status_t error;																			// Error status.
    uint16_t measurements[BCC_MEAS_CNT]; 														// Array needed to store all measured values.
    bool completed = false; 
    float newFloatValue = 0;																	// Conversion complete flag.
    int i = 0, bccIndex, errnoNum, lineCounterVal = 0;
    parameterKind_t parameter;
    int16_t lvTemp = 0;
    static float sumOfMovAvg = 0;
    float newSumCurrent, avgCurrent, dCharge, batVoltage, lowerstCellVoltage;//, remainingCap;
    int32_t int32Val = 0;
    uint8_t uint8Val = 0;

    /* Start ADC conversion ******************************************************/
    error = bcc_monitoring_doBlockingMeasurement(drvConfig);

    // check for an error
    if (error != BCC_STATUS_SUCCESS)
    {
        cli_printf("bcc_monitoring ERROR: Couldn't do measurement error: %d\n", error);
        return error;
    }

    // get the current measument
    error = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_ISENSE1_ADDR,
                             2, &measurements[BCC_MSR_ISENSE1]);

    // check for an error
    if (error != BCC_STATUS_SUCCESS)
    {
        cli_printf("Couldn't get current error: %d\n", error);
        return error;
    }

    /* Mask bits. */
    /* Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2 registers. */
    measurements[BCC_MSR_ISENSE1] &= BCC_R_MEAS1_I_MASK;
    measurements[BCC_MSR_ISENSE2] &= BCC_R_MEAS2_I_MASK;

    // convert the battery current to a float                                                                           // Measured ISENSE in [mA]. Value of shunt resistor is used.
    newFloatValue = BCC_GET_ISENSE_AMP(rShunt,
                                            measurements[BCC_MSR_ISENSE1],
                                            measurements[BCC_MSR_ISENSE2]) / MA_TO_A;

    // set the new battery current
    parameter = I_BATT;
    if(data_setParameter(parameter, &newFloatValue))
    {
       cli_printf("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
    }

#ifdef OUTPUT_CURRENT_MEAS_DOT
    // to indicate the current is set
    cli_printf(".");
#endif

    // check if semaphore is initialzed
    if(!gGetOtherMeasSemInitialized)
    {
        // set the error 
        error = BCC_STATUS_SPI_INIT;

        // error to user
        cli_printf("bcc_monitoring ERROR: semaphore isn't initialized!\n");

        // return error
        return error;
    }

    // check the semaphore if the other measurements need to be done
    if(sem_trywait(&gGetOtherMeasurementSem))
    {
        // save errno
        errnoNum = errno;

        // check if the semaphore is occupied, than stop after this measurements
        if(errnoNum == EAGAIN)
        {
            // reset the error and return
            return SEM_OCCUPIED_ERROR;
        }
        else if (errnoNum)
        {
            cli_printf("bcc_monitoring ERROR: sem_trywait error: %d\n", errnoNum);
            //cli_printf("EAGAIN = %d, EDEADLK = %d, EINTR = %d, EINVAL = &d\n", EAGAIN, EDEADLK, EINTR, EINVAL);
            return BCC_STATUS_PARAM_RANGE;
        }
    }

    
#ifdef OUTPUT_CURRENT_MEAS_DOT

    cli_printf("\n");
#endif

    if(gShowMeasurements && gDoTop)
    {
        // reset the lineCounterVal
        lineCounterVal = 1;
    }
    if(gShowMeasurements & (1<<CLI_CURRENT))
    {
        // check if top is on
        if(gDoTop)
        {
            // lock the cli lock
            cli_printLock(true);

            // save the cursor position
            cli_printfNoLock("%s \e[%d;1H \e[2K \ri-batt: \t %8.3f A\n" , &gSaveCursor, lineCounterVal, 
                newFloatValue);
        }
        else
        {
             // clear the line and write the value
            cli_printf("\e[2K");

            // print it
            cli_printf("i-batt: \t %8.3f A\n", newFloatValue);
        }

        // increase the line counter
        lineCounterVal++;
    }
   
#ifdef OUTPUT_UPDATE_OTHER_MEAS
    cli_printfTryLock("Updating other meas!\n");
#endif

    // get stack voltage
    error = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_STACK_ADDR, 1,
                         (uint16_t *)(measurements + ((uint8_t)BCC_MSR_STACK_VOLT)));

    // check for an error
    if (error != BCC_STATUS_SUCCESS)
    {
        cli_printfTryLock("bcc_monitoring ERROR: Couldn't get stack voltage error: %d\n", error);
        return error;
    }

    // get the cell voltages and the used analog channels 
    error = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_CELLX_ADDR_MC33772_START,
                         (BCC_REG_MEAS_ANX_ADDR_END - BCC_REG_MEAS_CELLX_ADDR_MC33772_START) + 1,
                         (uint16_t *)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));

    // check for an error
    if (error != BCC_STATUS_SUCCESS)
    {
        cli_printfTryLock("bcc_monitoring ERROR: Couldn't other measurements error: %d\n", error);
        return error;
    }
    

    /* Mask the other registers (starting at 5th register). */
    for (i = 5U; i < BCC_MEAS_CNT; i++)
    {
        measurements[i] &= BCC_R_MEAS_MASK;
    }


    // calculate the rest

    /* Voltages calculations ******************************************************/
      
    // get the amount of cells
    parameter = N_CELLS;
    if(data_getParameter(parameter, &int32Val, NULL) == NULL)
    {
       cli_printfTryLock("bcc_monitoring ERROR: getting new data went wrong in the update! par %d val %d\n", parameter, int32Val);
    } 

    // limit the value
    int32Val &= UINT8_MAX;

    // reset the battery voltage
    batVoltage = 0;
    
    // get the cell voltages
    for(i = 0; i < int32Val; i++)
    {
        // check if it is the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            bccIndex = (6-int32Val) + i;
        }
        else
        {
            // it is the first 2 cells
            bccIndex = i;
        }

        // convert the cell voltage to a float
        newFloatValue = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1 - bccIndex]) / UV_TO_V; 

        // if the first
        if(i == 0)
        {
            // set the first voltage
            lowerstCellVoltage = newFloatValue;
        }

        // set the new battery cell voltage
        parameter = (parameterKind_t)(V_CELL1 + i);

        // set the cell voltage 
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }    

        // add all the cell voltages to the battery voltage
        batVoltage += newFloatValue;

        // check if the lowest
        if(lowerstCellVoltage > newFloatValue)
        {
            // set the new lowest cell voltage
            lowerstCellVoltage = newFloatValue;
        }

        if(gShowMeasurements & (1<<CLI_CELL_VOLTAGE))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the cell voltage
                cli_printfNoLock("\e[%d;1H \e[2K \rv-cell%d:\t %8.3f V\n",/* &gSaveCursor,*/ lineCounterVal, 
                    i+1, newFloatValue);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                
                // print the cell voltage
                cli_printfTryLock("v-cell%d:\t %8.3f V\n", i+1, newFloatValue);
            }      
            
            // increase the linecounterval
            lineCounterVal++;
        }
    }

    // save the lowest cell voltage
    if(lowestCellVoltageAdr != NULL)
        *lowestCellVoltageAdr = lowerstCellVoltage;

    // convert the battery voltage to a float
    newFloatValue = BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]) / UV_TO_V;         // Stack voltage in [uV].

    // check if the battery stack voltage is almost the sum of the cells for redundancy
    if(((batVoltage - newFloatValue) > (STACK_VOLTAGE_DIFFERENCE_ERROR * int32Val)) ||
       ((batVoltage - newFloatValue) < -(STACK_VOLTAGE_DIFFERENCE_ERROR * int32Val)))
    {
        // output error
        cli_printfTryLock("bcc_monitoring ERROR: stackvoltage too different from sum of cells! stack: %8.3fV cells: %8.3fV\n",
            newFloatValue, batVoltage);

        // set the floatvalue
        batVoltage = newFloatValue;

        // set the error in the status flags
        data_statusFlagBit(STATUS_BMS_ERROR_BIT, 1);

        // TODO add error if this happens? go to FAULT state?

        #warning TODO add error if this happens?
    }

    // set the new battery voltage
    parameter = V_BATT;
    if(data_setParameter(parameter, &batVoltage))
    {
       cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
    }  
    if(gShowMeasurements & (1<<CLI_STACK_VOLTAGE))
    {
        // check if top is on
        if(gDoTop)
        {
            // print the battery voltage
            cli_printfNoLock("\e[%d;1H \e[2K \rv-batt:\t\t %8.3f V\n",/* &gSaveCursor,*/ lineCounterVal,
                batVoltage);
        }
        else
        {
            // clear the line and write the value
            cli_printfTryLock("\e[2K");
            cli_printfTryLock("v-batt:\t\t %8.3f V\n", batVoltage);
        }

        // increase the line counter
        lineCounterVal++;
    }

    // convert the battery voltage to a float
    batVoltage = VOLTDIV_BATT_OUT*BCC_GET_VOLT(measurements[BCC_MSR_AN4]) / UV_TO_V;

    // set the battery voltage
    parameter = V_OUT;
    if(data_setParameter(parameter, &batVoltage))
    {
       cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
    }

    // if the battery voltage needs to be shown
    if(gShowMeasurements & (1<<CLI_BAT_VOLTAGE))
    {
        // check if top is on
        if(gDoTop)
        {
            // print the output voltage
            cli_printfNoLock("\e[%d;1H \e[2K \rv-out: \t\t %8.3f V\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                batVoltage);
        }
        else
        {
            // clear the line and write the value
            cli_printfTryLock("\e[2K");
            
            // print it
            cli_printfTryLock("v-out: \t\t %8.3f V\n", batVoltage);
        }

        // increase the line counter
        lineCounterVal++;
    }

    // check if the voltage is lower than the defined on voltage
    if(batVoltage < OUTPUT_ON_VOLTAGE)
    {
        // set the output_status value off
        uint8Val = 0;
    }
    else
    {
        // lock the gate mutex
        pthread_mutex_lock(gateMutexAdr);

        // read the gate CTRL D pin
        uint8Val = (!gpio_readPin(GATE_CTRL_D)) & 1;

        // unlock the gate mutex
        pthread_mutex_unlock(gateMutexAdr);

        // check if gate is enabled in software
        if(uint8Val)
        {
            // check if the shortcut pin (overcurrent) is not high
            uint8Val = (!gpio_readPin(OVERCURRNT)) & 1;
        }
    }

    // set the battery voltage
    parameter = S_OUT;
    if(data_setParameter(parameter, &uint8Val))
    {
       cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
    }

    if(gShowMeasurements & (1<<CLI_OUTPUT_STATUS))
    {
        // check if top is on
        if(gDoTop)
        {
            // print the output status
            cli_printfNoLock("\e[%d;1H \e[2K \rs-out \t\t\t%d\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                uint8Val);
        }
        else
        {
            // clear the line and write the value
            cli_printfTryLock("\e[2K");
            cli_printfTryLock("s-out \t\t\t%d\n", uint8Val);
        }

        // increase the line counter
        lineCounterVal++;
    }

     // if the first time
    // oldsample is the newsmaple, to make sure there is a difference in time of 0
    if(firstPowerup)
    {
        completed = false;

        //cli_printfTryLock("lowest voltage: %8.3fv ", lowerstCellVoltage);

        // find the initial state of charge
        for(i = 0; i < ((sizeof(cellmvVsSOCLookupTable)/sizeof(mvSoC_t)) - 1); i++)
        {
            // check the cell voltage
            if((lowerstCellVoltage*1000) > ((cellmvVsSOCLookupTable[i].milliVolt - cellmvVsSOCLookupTable[i+1].milliVolt)/2) + cellmvVsSOCLookupTable[i+1].milliVolt)
            {
                //cli_printfTryLock(" > %dmv\n", ((cellmvVsSOCLookupTable[i].milliVolt - cellmvVsSOCLookupTable[i+1].milliVolt)/2) + cellmvVsSOCLookupTable[i+1].milliVolt);
                // set the state of charge
                uint8Val = (cellmvVsSOCLookupTable[i]).SoC;

                //cli_printfTryLock("i = %d, SoC: %d == %d\n", i, uint8Val, (cellmvVsSOCLookupTable[i]).SoC);

                // say it is found
                completed = true;

                // escape the for loop
                break;
            }
        }

        // if didn't found one, set to 0
        if(!completed)
        {
            // set to 0
            uint8Val = 0;
        }

        // set a minimum and max
        if(uint8Val < 1)
        {
            uint8Val = 1;
        }
        else if(uint8Val > 100)
        {
            uint8Val = 100;
        }
        
        // set the initial charge
        if(data_getParameter(A_FULL, &dCharge, NULL) == NULL)
        {
            cli_printfTryLock("bcc_monitoring ERROR: couldn't get A_FULL!\n");
            dCharge = A_FULL_DEFAULT;
            //return lvRetValue;
        }

        // calculate the remaining capacity by multiplying the full charge capacity with the state of charge
        newFloatValue = (dCharge * uint8Val)/100;

        // TODO make better look up table or calibration

        //cli_printfTryLock("remaining capacity %8.3fAh state of charge %d\n", newFloatValue, uint8Val);

        // set the initial capacity
        // set the new remaining capacity
        parameter = A_REM;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d %8.3f\n", parameter, newFloatValue);
        }     

        // it is not the first power up anymore
        firstPowerup = false;
    }


    /* GPIO calculations  **********************************************************/

    error = 0;

    // get the variable to know if the battery temperature measurement should be done
    if(data_getParameter(SENSOR_ENABLE, &uint8Val, NULL) == NULL)
    {
        cli_printfTryLock("bcc_monitoring ERROR: couldn't get SENSOR_ENABLE!\n");
        uint8Val = SENSOR_ENABLE_DEFAULT;
        //return lvRetValue;
    }

    // check if the battery temperature measurement should be done
    if(uint8Val)
    {
        // get the battery temperature
        error = getNtcCelsius(measurements[BCC_MSR_AN1], &lvTemp);

        // check for errors
        if(error)
        {
            cli_printfTryLock("bcc_monitoring ERROR: failed getting batt temp error: %d\n", error);
            cli_printfTryLock("Maybe disable the measurement? with \"enable_batt_temp\"\n");
        }
        else
        {
            // make it a floating point value
            newFloatValue = lvTemp/T10_TO_T;

            // set the temperature value
            parameter = C_BATT;
            if(data_setParameter(parameter, &newFloatValue))
            {
               cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
            }

            if(gShowMeasurements & (1<<CLI_TEMPERATURE))
            {
                // check if top is on
                if(gDoTop)
                {
                    // print the battery temperature
                    cli_printfNoLock("\e[%d;1H \e[2K \rc-batt: \t %8.3f C\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                        newFloatValue);
                }
                else
                {
                    // clear the line and write the value
                    cli_printfTryLock("\e[2K");
                    cli_printfTryLock("c-batt: \t %8.3f C\n", newFloatValue);
                }          

                // increase the line counter
                lineCounterVal++;
            }
        }
    }

    // get the AFE temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN3], &lvTemp);

    // check for errors
    if(error)
    {
        cli_printfTryLock("bcc_monitoring ERROR: failed getting AFE temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value
        newFloatValue = lvTemp/T10_TO_T;

        // set the temperature value
        parameter = C_AFE;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }

        if(gShowMeasurements & (1<<CLI_TEMPERATURE))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the afe temperature
                cli_printfNoLock("\e[%d;1H \e[2K \rc-afe: \t\t %8.3f C\n",/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                cli_printfTryLock("c-afe: \t\t %8.3f C\n", newFloatValue);
            }

            // increase the line counter
            lineCounterVal++;
        }
    }

    // get the transistor temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN2], &lvTemp);

    // check for errors
    if(error)
    {
        cli_printfTryLock("bcc_monitoring ERROR: failed getting T temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value
        newFloatValue = lvTemp/T10_TO_T;

        // set the temperature value
        parameter = C_T;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }

        if(gShowMeasurements & (1<<CLI_TEMPERATURE))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the transistor temperature
                cli_printfNoLock("\e[%d;1H \e[2K \rc-t: \t\t %8.3f C\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                cli_printfTryLock("c-t: \t\t %8.3f C\n", newFloatValue);
            }

            // increase the line counter
            lineCounterVal++;
        }
    } 

    // get the sense resistor temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN0], &lvTemp);

    // check for errors
    if(error)
    {
        cli_printfTryLock("bcc_monitoring ERROR: failed getting R temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value
        newFloatValue = lvTemp/T10_TO_T;

        // set the temperature value
        parameter = C_R;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }

        if(gShowMeasurements & (1<<CLI_TEMPERATURE))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the resistor temperature
                cli_printfNoLock("\e[%d;1H \e[2K \rc-r: \t\t %8.3f C\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                cli_printfTryLock("c-r: \t\t %8.3f C\n", newFloatValue);
            }

            // increase the line counter
            lineCounterVal++;
        }
 
    } 

    //cli_printfTryLock("counter: %d max: %d\n", gMeasurementCounter, gMeasurementCounterEndValue);

    // check if the CC needs to be read
    if(gMeasurementCounter >= gMeasurementCounterEndValue)
    {
        // calculate the average current and the delta charge
        i = bcc_monitoring_calcDCharge(drvConfig, &measurements[BCC_MSR_CC_NB_SAMPLES], &avgCurrent, &dCharge, false);

        // check for errors
        if(i)
        {
            // return error
            cli_printfTryLock("bcc_monitoring ERROR: failed to calc Dcharge! %d\n", i);
            return BCC_STATUS_PARAM_RANGE;
        }

        // save the average current
        parameter = I_BATT_AVG;
        if(data_setParameter(parameter, &avgCurrent))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d %8.3f\n", parameter, newFloatValue);
        }   

        if(gShowMeasurements & (1<<CLI_AVG_CURRENT))  
        {  
            // check if top is on
            if(gDoTop)
            {
                // print the average current and samples
                cli_printfNoLock("\e[%d;1H \e[2K \ri-avg: \t\t %8.3f A, smpls: %d\n" ,/* &gSaveCursor,*/ lineCounterVal,
                    avgCurrent, measurements[BCC_MSR_CC_NB_SAMPLES]);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                // output the average current if needed
                cli_printfTryLock("i-avg: \t\t %8.3f A, smpls: %d\n", avgCurrent, measurements[BCC_MSR_CC_NB_SAMPLES]);
            }

            // increase the line counter
            lineCounterVal++; 
        }
        
        // get the consumed power
        parameter = E_USED;
        if(data_getParameter(parameter, &newFloatValue, NULL) == NULL)
        {
             cli_printfTryLock("bcc_monitoring ERROR: getting value went wrong in the update! par: %d\n", parameter);
        }

        // add the difference in charge in Ah times the battery output voltage
        newFloatValue += (-dCharge * batVoltage);

        // set the new energy consumed
        parameter = E_USED;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d %8.3f\n", parameter, newFloatValue);
        }   

        // check if outputing the energy consumed is needed 
        if(gShowMeasurements & (1<<CLI_ENERGY_CONSUMED))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the energy used
                cli_printfNoLock("\e[%d;1H \e[2K \re-used: \t %8.3f Wh\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                cli_printfTryLock("e-used: \t %8.3f Wh\n", newFloatValue);
            }

            // increase the line counter
            lineCounterVal++;
        }

        // get the capacity 
        parameter = A_REM;
        if(data_getParameter(parameter, &newFloatValue, NULL) == NULL)
        {
             cli_printfTryLock("bcc_monitoring ERROR: getting value went wrong in the update! par: %d\n", parameter);
        }

        // calculate the new remaining capacity with the difference in charge
        newFloatValue = (newFloatValue + dCharge);   

        // output remaining and Dcharge if needed
        if(gShowMeasurements & (1<<CLI_REMAINING_CAP))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the remaning capacity
                cli_printfNoLock("\e[%d;1H \e[2K \ra-rem: \t\t %8.3f Ah\n"/*, DCharge: %8.3f\n" */,/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);//, dCharge);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                cli_printfTryLock("a-rem: \t\t %8.3f Ah\n"/*, DCharge: %8.3f\n"*/, newFloatValue, dCharge);
            }

            // increase the line counter
            lineCounterVal++;
        }

        // limit the remaining capacity if less than 0
        if(newFloatValue < 0)
        {
            // set the remaining capacity to 0
            newFloatValue = 0;
        }

        // set the new remaining capacity
        parameter = A_REM;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d %8.3f\n", parameter, newFloatValue);
        }   

        // if state of charge needs to be outputted
        if(gShowMeasurements & (1<<CLI_STATE_OF_CHARGE))
        {
            // get the FCC 
            parameter = A_FULL;
            if(data_getParameter(parameter, &dCharge, NULL) == NULL)
            {
                 cli_printfTryLock("bcc_monitoring ERROR: getting value went wrong in the update! par: %d\n", parameter);
            }

            // calcultate the state of charge
            uint8Val = (uint8_t)round((newFloatValue/dCharge)*100);

            // check if top is on
            if(gDoTop)
            {
                // print the state of charge
                cli_printfNoLock("\e[%d;1H \e[2K \rs-charge:\t      %3d %%\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                    uint8Val);
            }
            else
            {
                 // clear the line and write the value
                cli_printfTryLock("\e[2K");
                // output the state of charge
                cli_printfTryLock("s-charge:\t%d %%\n", uint8Val);
            }

            // increase the line counter
            lineCounterVal++;
        }

        //save the remaining capacity in dCharge
        dCharge = newFloatValue;

        // remove the old value from the sum
        sumOfMovAvg -= gMovingAvgArr[gMovingAvgArrIndex];

        // add the new value to the sum 
        sumOfMovAvg += avgCurrent;

        // divide by the amount of elements
        newFloatValue = sumOfMovAvg/gMovingAvgArrElements;

         // set the new value in the array
        gMovingAvgArr[gMovingAvgArrIndex] = avgCurrent;

        // increase the index 
        gMovingAvgArrIndex = (gMovingAvgArrIndex + 1) % gMovingAvgArrElements;

        // get the full charge capacity in newSumCurrent
        parameter = A_FULL;
        if(data_getParameter(parameter, &newSumCurrent, NULL) == NULL)
        {
             cli_printfTryLock("bcc_monitoring ERROR: getting value went wrong in the update! par: %d\n", parameter);
        }

        // get the max charge Current
        parameter = I_CHARGE_MAX;
        if(data_getParameter(parameter, &avgCurrent, NULL) == NULL)
        {
             cli_printfTryLock("bcc_monitoring ERROR: getting value went wrong in the update! par: %d\n", parameter);
        }

        // check if the current is higher than the max charge current
        if(newFloatValue > avgCurrent || newFloatValue < 0)
        {
            // calculate the time left to charge
            // keep in mind that dCharge is the remaining capacity and newSumCurrent is the Full charge capacity
            dCharge = (newSumCurrent - dCharge)/avgCurrent;
        }
        else
        {
            // calculate the time left to charge
            // keep in mind that dCharge is the remaining capacity and newSumCurrent is the Full charge capacity
            dCharge = (newSumCurrent - dCharge)/fabs(newFloatValue);
        }

        // check for limits 
        if(dCharge < 0.001)
        {
            // set it to the minimum limit
            dCharge = 0.001;
        }

        // set the time to charge
        parameter = T_FULL;

        //cli_printfTryLock("hours to full charge: %8.3f\n", dCharge);
        if(data_setParameter(parameter, &dCharge))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }   

        // this calculate the avg power V*Iavg
        newFloatValue = batVoltage * newFloatValue;
        
        if(gShowMeasurements & (1<<CLI_AVG_POWER))
        {
            // check if top is on
            if(gDoTop)
            {
                // print the average power
                cli_printfNoLock("\e[%d;1H \e[2K \rp-avg: \t\t %8.3f W\n" ,/* &gSaveCursor,*/ lineCounterVal, 
                    newFloatValue);

                // restore the cursor position
                cli_printfNoLock("%s", &gRestoreCursor);
            }
            else
            {
                // clear the line and write the value
                cli_printfTryLock("\e[2K");
                // output the average power
                cli_printfTryLock("p-avg: \t\t %8.3f W\n", newFloatValue);
            }            

            // increase the line counter 
            lineCounterVal++;
        }

        // set the average power
        parameter = P_AVG;
        if(data_setParameter(parameter, &newFloatValue))
        {
           cli_printfTryLock("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", parameter);
        }   

        // reset the measurement counter
        gMeasurementCounter = 0;
    }

    // unlock the printlock mutex
    cli_printLock(false);

    // increase the measurementCounter 
    gMeasurementCounter++;

    return error;
}

/*
 * @brief   This function is used to do a meaurement
 *          This function is blocking and will wait until the measurement is done
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_doBlockingMeasurement(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t error;
    int i = 0;
    bool completed = false;

    // do the measuremetns until no error is given for 5 tries 
    do
    {
        //cli_printf("starting conversion! %d\n", i+1);
        // do the measurements
        error = BCC_Meas_StartConversion(drvConfig, BCC_CID_DEV1);                      // Error verification.

        // increase i to only loop an amount of time
        i++;
    }while(error != BCC_STATUS_SUCCESS && i < 5);
    
    // check for errors
    if (error != BCC_STATUS_SUCCESS)
    {
        // return error
        cli_printf("bcc_monitoring_doBlockingMeasurement: Couldn't start conversion! error: %d\n", error);
        return error;
    }

    // wait until the conversion is complete
    do                                                                                          // Conversion.
    {
        error = BCC_Meas_IsConverting(drvConfig, BCC_CID_DEV1, &completed);
        if (error != BCC_STATUS_SUCCESS)
        {
            // return error
            cli_printf("bcc_monitoring_doBlockingMeasurement: Couldn't check conversion! error: %d\n", error);
            return error;
        }
    } while (!completed);

    // return to the user
    return error;
}

/*
 * @brief   This function is used to check the output, without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   output the address of the variable to become 1 if the output high and 0 if low.
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_checkOutput(bcc_drv_config_t* const drvConfig, bool *output)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t regVal;
    float batVoltage;

    // check if the address is not NULL
    if(output == NULL)
    {
        cli_printf("bcc_monitoring_checkOutput ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // get the AN4 measurement
    lvRetValue = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_AN4_ADDR),
        1, &regVal); 

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("bcc_monitoring_checkOutput ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask the measurement
    regVal &= BCC_R_MEAS_MASK;

    // convert the battery voltage to a float
    batVoltage = VOLTDIV_BATT_OUT*BCC_GET_VOLT(regVal) / UV_TO_V;

    // check if the voltage is lower than the defined on voltage
    if(batVoltage < OUTPUT_ON_VOLTAGE)
    {
        // set the output_status value off
        *output = 0;
    }
    else
    {
        // set the output_status value on
        *output = 1;
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function is used to check if the number of cells are OK
 *          it will read the cell voltages and the stack voltage, without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   nCellsOk the address of the variable to become 1 if the cells are OK.
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_checkNCells(bcc_drv_config_t* const drvConfig, bool *nCellsOk)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t regVal;
    float cellVoltage, cellVoltageTotal;
    float stackVoltage;
    int bccIndex, i;
    uint8_t n_cells;

    // check if the address is not NULL
    if(nCellsOk == NULL)
    {
        cli_printf("bcc_monitoring_checkNCells ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // set it to not ok
    *nCellsOk = 0;

    // get the amount of cells
    if(data_getParameter(N_CELLS, &n_cells, NULL) == NULL)
    {
       cli_printf("bcc_monitoring_checkNCells ERROR: getting N_CELLS went wrong! val %d\n", n_cells);
    } 

    // reset the battery voltage
    cellVoltageTotal = 0;

    //n_cells = 6;
    
    // get the cell voltages
    for(i = 0; i < n_cells; i++)
    {
        // check if it is the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            bccIndex = (6-n_cells) + i;
        }
        else
        {
            // it is the first 2 cells
            bccIndex = i;
        }

        // read the cell voltage
        lvRetValue = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_CELLX_ADDR_END - bccIndex),
        1, &regVal); 

        // check for errors
        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            cli_printf("bcc_monitoring_checkOutput ERROR: reading AN4 failed: %d\n", lvRetValue);
            return lvRetValue;
        }

        // mask it 
        regVal &= BCC_R_MEAS_MASK;

        // convert the cell voltage to a float
        cellVoltage = BCC_GET_VOLT(regVal) / UV_TO_V; 

        //cli_printf("cell%d V: %8.3f of BCCindex: %d\n", i, cellVoltage, bccIndex);

        // add all the cell voltages to the battery voltage
        cellVoltageTotal += cellVoltage;
    }

    // read the stack voltage
    lvRetValue = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_STACK_ADDR,
        1, &regVal); 

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("bcc_monitoring_checkOutput ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask it 
    regVal &= BCC_R_MEAS_MASK;

    // convert the battery voltage to a float
    stackVoltage = BCC_GET_STACK_VOLT(regVal) / UV_TO_V;         // Stack voltage in [uV].

    // check if the battery stack voltage is almost the sum of the cells for redundancy
    if(((cellVoltageTotal - stackVoltage) > (STACK_VOLTAGE_DIFFERENCE_ERROR * n_cells)) ||
       ((cellVoltageTotal - stackVoltage) < -(STACK_VOLTAGE_DIFFERENCE_ERROR * n_cells)))
    {
        // output error
        cli_printf("bcc_monitoring ERROR: stackvoltage too different from sum of cells! stack: %8.3fV cells: %8.3fV\n",
            stackVoltage, cellVoltageTotal);


        // set the error in the status flags
        data_statusFlagBit(STATUS_BMS_ERROR_BIT, 1);

        // set the variable to not OK
        *nCellsOk = 0;    
    }
    else
    {
        // set the variable to OK
        *nCellsOk = 1;
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function is used to get the output voltage
 *          it will read the output voltage without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_getOutputVoltage(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t regVal;
    float outputVoltage;

    // read the output voltage (AN4)
    lvRetValue = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_ANX_ADDR_END - 4),
    1, &regVal); 

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printf("bcc_monitoring_getOutputVoltage ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask it 
    regVal &= BCC_R_MEAS_MASK;

    // convert the cell voltage to a float
    outputVoltage = VOLTDIV_BATT_OUT*BCC_GET_VOLT(regVal) / UV_TO_V; 

    //cli_printf("output voltage: %8.3f\n", outputVoltage);

    // set the output voltage
    if(data_setParameter(V_OUT, &outputVoltage))
    {
       cli_printf("bcc_monitoring ERROR: setting new data went in the update wrong! par %d\n", 
        V_OUT);

       // set an error
       lvRetValue = BCC_STATUS_PARAM_RANGE;
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function increases the semaphore so the bcc_monitoring_updateMeasurements 
 *          function will do the rest of the calculations
 *          Developed by C. van Mierlo. \n
 *
 * @param   none
 *
 * @return  int Error code (of the semaphore)
 */
int bcc_monitoring_doAllCalculations(void)
{
    int error = 0;
    int semValue;

     // check if semaphore is initialzed
    if(!gGetOtherMeasSemInitialized)
    {
        // set the error 
        error = BCC_STATUS_SPI_INIT;

        // error to user
        cli_printf("bcc_monitoring ERROR: semaphore isn't initialized!\n");

        // return error
        return error;
    }

    // check if semaphore needs to be posted
    error = sem_getvalue(&gGetOtherMeasurementSem, &semValue);

    // check for errors
    if(error)
    {
        error = errno;
        cli_printf("bcc_monitoring ERROR: other meas semaphore value error: %d\n", error);
        //usleep(100);
        return error;
    }

    //cli_printf("sem: %d\n", semValue);
    //usleep(100);

    // check if needed 
    while(semValue < 1 && error == 0)
    {
        // increase the semaphore
        error = sem_post(&gGetOtherMeasurementSem);
        //cli_printf("sem2: %d\n", semValue);

        // check for error
        if(error)
        {
            error = errno;

             // check if semaphore needs to be posted
            sem_getvalue(&gGetOtherMeasurementSem, &semValue);

            // get the error to the user
            cli_printf("bcc_monitoring ERROR: other meas semaphore error: %d sem: %d\n", error, semValue);

        }

        // check if semaphore needs to be posted
        error = sem_getvalue(&gGetOtherMeasurementSem, &semValue);

        // check for errors
        if(error)
        {
            error = errno;
            cli_printf("bcc_monitoring ERROR: other meas semaphore value2 error: %d\n", error);
            return error;
        }        
    }

    // return 
    return error;
}

/*
 * @brief   This function will set the end value (gMeasurementCounterEndValue)
 *          this will be used to check if the coulomb counter should be read and reset
 *          this function will also reset the counter
 *
 * @param   updateAverageInterval the amount of measurements with bcc_monitoring_doAllMeasurements
 *          should be done before updating the 10 elements moving average
 *             
 * @param   elements the amount of elements in the moving average max 10 min 1
 *          it will set it to 10 or 1 if out of range!
 *
 * @return  none
 */
void bcc_monitoring_setAverageInterval(uint8_t updateAverageInterval, uint8_t elements)
{
    // check for min and max and limit
    if(elements < 1)
    {
        elements = 1;
    }
    else if(elements > 10)
    {
        elements = 10;
    }

    // check if changed 
    if(elements != gMovingAvgArrElements)
    {
        // free the previous array
        free((void*)gMovingAvgArr);

        // allocate the new array
        gMovingAvgArr = (float*)calloc(elements, sizeof(float));

        // set the global value
        gMovingAvgArrElements = elements;

        // reset the index 
        gMovingAvgArrIndex = 0;
    }

    // set the max value
    gMeasurementCounterEndValue = updateAverageInterval;

    // reset the counter value
    gMeasurementCounter = 1;
}

/*
 * @brief   This function is used to calculate the delta charge with the coulomb counter registers
 *          it will read and therefor reset the CC registers
 * @note    can be called from mulitple threads
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   samplesAdr address of the variable to become the samples in the CC register
 * @param   avgCurrentAdr address of the variable to become the average current
 * @param   deltaChargeAdr address of the variable to become the delta charge
 * @param   resetCC if this is true, it will reset the CC registers by writing CC_RST true 
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int bcc_monitoring_calcDCharge(bcc_drv_config_t* const drvConfig, uint16_t *samplesAdr, 
    float *avgCurrentAdr, float *deltaChargeAdr, bool resetCC)
{
    bcc_status_t error;                                                                         // Error status.
    uint16_t measurements[BCC_MEAS_CNT];                                                        // Array needed to store all measured values.
    int lvRetValue = -1;
    float newSumCurrent;
    int32_t dt;

    // to get the sampletime, keep in mind that static struct timespec oldSampleTime is initialized later!
    struct timespec newSampletime;

    // make the variable for the oldtime and initialze once as the same as the sampletime
    static struct timespec oldSampleTime = {.tv_nsec = 0, .tv_sec = 0 };

    // check if mutex not is initialized 
    if(!dChargeFuncMutexInitialized)
    {
        // return 
        cli_printf("bcc_monitoring ERROR: mutex not initialzed1\n");
        return lvRetValue;
    }

    // lock the mutex
    pthread_mutex_lock(&gDChargeFuncMutex);

    if(samplesAdr == NULL || avgCurrentAdr == NULL || deltaChargeAdr == NULL)
    {
        cli_printf("bcc_monitoring ERROR: gave NULL pointer!\n");
        return lvRetValue;
    }

    // read the CC registers and reset them
    error = BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_CC_NB_SAMPLES_ADDR,
                             ((BCC_REG_COULOMB_CNT2_ADDR - BCC_REG_CC_NB_SAMPLES_ADDR) + 1), measurements);

    // get the sample time
    if(clock_gettime(CLOCK_REALTIME, &newSampletime) == -1)
    {
        cli_printf("bcc_monitoring ERROR: failed to get newSampletime time!\n");
        return lvRetValue;
    }

    // check if first time
    if(oldSampleTime.tv_sec == 0 && oldSampleTime.tv_nsec == 0)
    {
        // make sure the difference is 0
        oldSampleTime.tv_sec = newSampletime.tv_sec;
        oldSampleTime.tv_nsec = newSampletime.tv_nsec;
    }

    // check for an error
    if (error != BCC_STATUS_SUCCESS)
    {
        cli_printf("bcc_monitoring ERROR: Couldn't get CC registers error: %d\n", error);
        return error;
    }

     // maybe set CC_RST of ADC_CFG to reset CC or check CC_RST_CFG from ADC2_OFFSET_COMP
    if(resetCC)
    {
        // reset the CC by writing the CC_RST bit in ADC_CFG
        error = BCC_Reg_Update(drvConfig, BCC_CID_DEV1, BCC_REG_ADC_CFG_ADDR,  BCC_W_CC_RST_MASK,  0xFF);

        // check for errors
        if (error != BCC_STATUS_SUCCESS)
        {
            cli_printf("bcc_monitoring ERROR: Couldn't reset CC registers error: %d\n", error);
            //return error;
        }

        //cli_printf("CCmsb: %d CClsb: %d samples: %d\n", measurements[BCC_MSR_COULOMB_CNT1], measurements[BCC_MSR_COULOMB_CNT2], measurements[BCC_MSR_CC_NB_SAMPLES]);
    }

    // calulate the average power

    /* capacity and SoC measurements ********************************************************/

    // (coulombcountTotalk)*V2res/Rshint[uOhm] = ACCk [uA]
    // (coulombcountTotalk-1)*V2res/Rshint[uOhm] = ACCk-1 [uA]
    // CC_NB_SAMPLESk - CC_NB_SAMPLESk-1 = dNk
    // dt = tk - tk-1

    // dCharge = ((ACCk – ACCk-1) / dNk) * dt 
    //chargek = chargek-1 + dCharge

    // get the sum of the current in A
    newSumCurrent = (((measurements[BCC_MSR_COULOMB_CNT1] << 16) +  measurements[BCC_MSR_COULOMB_CNT2]) * V2RES_DIV_RSHUNT);

    // get the average current in A over the 
    *avgCurrentAdr = ((newSumCurrent)/(measurements[BCC_MSR_CC_NB_SAMPLES]));
    
    // get the difference in time in ms (could use T_meas)
    dt = ((newSampletime.tv_nsec / 1000000) + (newSampletime.tv_sec * 1000)) - ((oldSampleTime.tv_nsec / 1000000) + (oldSampleTime.tv_sec * 1000));

    // check for limits
    if(dt < 0)
    {
        // set it to 0
        dt = 0;
    }

    //cli_printf("dt: %dms\n", dt);
    
    // get the difference in charge in Ah
    *deltaChargeAdr = *avgCurrentAdr * (float)dt/(3600000);

    // copy the samples to the variable
    *samplesAdr = measurements[BCC_MSR_CC_NB_SAMPLES];

#ifdef OUTPUT_AVG_TIME
        cli_printf("dt: %dms\n", dt);
#endif

    // save the old time
    oldSampleTime.tv_nsec = newSampletime.tv_nsec;
    oldSampleTime.tv_sec = newSampletime.tv_sec;

    // unlock the mutex
    pthread_mutex_unlock(&gDChargeFuncMutex);

    // return to the user
    lvRetValue = 0;
    return lvRetValue;
}

/*
 * @brief   This function is used to enable or disable the display of the measurements
 *
 * @param   showCommand which measurements to enable or disable
 * @param   value the new value of that bit
 *
 * @return  none
 */
void bcc_monitoring_setShowMeas(showCommands_t showCommand, bool value)
{
    int i;
    int32_t int32Val;
    int lineCounterVal = 0;

    // check if a bit
    if(showCommand < CLI_ALL)
    {
        // check the value
        if(value)
        {
            // set the bit
            gShowMeasurements |= (1 << showCommand);

            // reset the screen
            if(gDoTop)
            {
                cli_printf("\e[2J");
            }
        }
        else
        {
            // clear the bit
            gShowMeasurements &= ~(1 << showCommand);
        }
    }
    else
    {
        if(showCommand == CLI_ALL)
        {
            // reset the value
            if(!value)
            {
                gShowMeasurements = 0;
            }
            else
            {
                // set each bit
                for(i = 0; i < CLI_ALL; i++)
                {
                    // set the value
                    gShowMeasurements |= (1<<i);
                }
            }

            if(gDoTop)
            {
                // clear the screen 
                cli_printf("\e[2J");

                for(i = 0; i < CLI_ALL; i++)
                {
                    if(gShowMeasurements & (1<<i))
                    {
                        // increase the linecounter value
                        lineCounterVal++;

                        if(i == CLI_CELL_VOLTAGE)
                        {
                            // get the cell count
                            if(data_getParameter(N_CELLS, &int32Val, NULL) == NULL)
                            {
                               cli_printf("bcc_monitoring_setShowMeas ERROR: getting cell count went wrong!\n");
                            } 
                            // add some more 
                            lineCounterVal += ((int32Val & 0xFF) - 1);
                        }
                        else if(i == CLI_TEMPERATURE)
                        {
                            // add some more 
                            lineCounterVal += 3;
                        }
                    }
                }

                // set the line 
                cli_printf("\e[%d;0H", ++lineCounterVal);

                // save the value
                //cli_printf("%s", &gSaveCursor);
            }
        }
        if(showCommand == CLI_TOP)
        {
            // set the topvariable
            gDoTop = value;

            // check if reset is needed 
            if(value)
            {

               cli_printf("\e[2J");

                lineCounterVal = 0;

                for(i = 0; i < CLI_ALL; i++)
                {
                    if(gShowMeasurements & (1<<i))
                    {
                        lineCounterVal++;
                        if(i == CLI_CELL_VOLTAGE)
                        {
                            // get the cell count
                            if(data_getParameter(N_CELLS, &int32Val, NULL) == NULL)
                            {
                               cli_printf("bcc_monitoring_setShowMeas ERROR: getting cell count went wrong!\n");
                            } 
                            // add some more 
                            lineCounterVal += ((int32Val & 0xFF) - 1);
                        }
                        else if(i == CLI_TEMPERATURE)
                        {
                            // add some more 
                            lineCounterVal += 3;
                        }
                    }
                }

                // set the line 
                cli_printf("\e[%d;0H", ++lineCounterVal);
                //cli_printf("this line %d\n", lineCounterVal);

                 // save the line
                //cli_printf("%c[2J", 7);
                //cli_printf("%s", &gSaveCursor);
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
