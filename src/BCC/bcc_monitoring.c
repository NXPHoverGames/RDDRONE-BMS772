/*
 * Copyright 2016 - 2024 NXP
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
 **         This module was adapted from BCC SW examples by C. van Mierlo.
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_monitoring.h" // Include header file
#include "data.h"
#include "gpio.h"
#include "cli.h"
#include "spi.h"
#include <errno.h>
#include <assert.h>

//#include <math.h>
#include <float.h>
#include "bcc_configuration.h"
#include "bcc_spiwrapper.h"
#include <semaphore.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//#define OUTPUT_CURRENT_MEAS_DOT
//#define OUTPUT_UPDATE_OTHER_MEAS

//#define DONT_DO_LAST

//#define DEBUG_TIMING

#define BCC_REG_MEAS_AN4_ADDR       0x43

/*! @brief Size of NTC look-up table. */
#define NTC_TABLE_SIZE              (NTC_MAXTEMP - NTC_MINTEMP + 1)

/*! @brief 0 degree Celsius converted to Kelvin. */
#define NTC_DEGC_0                  273.15

/*! @brief miliAmpere converted to A with a division. */
#define MA_TO_A                     1000.0

/*! @brief microvolt to volt with a division. */
#define UV_TO_V                     1000000.0

/*! @brief 10*temperature to temperature with a division. */
#define T10_TO_T                    10.0

#define V2RES                       0.6 //!< [μV/LSB] Current sense user register resolution
#define V2RES_DIV_RSHUNT            V2RES / SHUNT_RESISTOR_UOHM / 1000000 //!< [(μV/LSB)/uOhm] this times the coulumb count value is current

#define STANDARD_MOVING_AVG_SIZE    10   

/*!
 * @brief Calculates final temperature value.
 *
 * @param tblIdx Index of value in NTC table which is close
 *        to the register value provided by user.
 * @param degTenths Fractional part of temperature value.
 * @return Temperature.
 */
#define NTC_COMP_TEMP(tblIdx, degTenths) ((((tblIdx) + NTC_MINTEMP) * 10) + (degTenths))

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/
/*!
 *  @brief this is the OCV/SoC table for a LiPo battery
 *  @note Change this table to the specification of the used LiPo battery
 */
static const mvSoC_t cellmvVsSOCLiPoLookupTable[60] =
{
    { 4200, 100},
    { 4187, 99},
    { 4168, 96},
    { 4149, 94},
    { 4131, 92},
    { 4113, 90},
    { 4095, 88},
    { 4079, 86},
    { 4065, 84},
    { 4047, 82},
    { 4028, 80},
    { 4011, 78},
    { 3997, 75},
    { 3984, 73},
    { 3972, 72},
    { 3959, 70},
    { 3945, 68},
    { 3930, 66},
    { 3912, 64},
    { 3895, 62},
    { 3881, 60},
    { 3869, 58},
    { 3859, 56},
    { 3850, 54},
    { 3841, 52},
    { 3833, 51},
    { 3826, 49},
    { 3819, 47},
    { 3813, 45},
    { 3807, 43},
    { 3802, 41},
    { 3796, 40},
    { 3792, 38},
    { 3787, 36},
    { 3783, 34},
    { 3778, 32},
    { 3771, 31},
    { 3765, 29},
    { 3758, 27},
    { 3751, 25},
    { 3745, 24},
    { 3740, 22},
    { 3734, 20},
    { 3727, 18},
    { 3718, 17},
    { 3710, 15},
    { 3698, 13},
    { 3692, 12},
    { 3688, 11},
    { 3686, 10},
    { 3683, 9},
    { 3680, 8},
    { 3675, 7},
    { 3666, 6},
    { 3642, 5},
    { 3605, 4},
    { 3568, 3},
    { 3503, 2},
    { 3417, 1},
    { 3313, 0}
};

/*!
 *  @brief this is the OCV/SoC table for a LiFePO4 battery
 *  @note Change this table to the specification of the used LiFePO4 battery
 */
static const mvSoC_t cellmvVsSOCLiFePO4LookupTable[] =
{
    {3650, 100},            
    {3610, 99},         
    {3460, 95},         
    {3320, 90},         
    {3310, 80},         
    {3300, 70},         
    {3290, 60},         
    {3280, 50},         
    {3270, 40},         
    {3250, 30},         
    {3220, 20},         
    {3200, 17},         
    {3120, 14},         
    {3000,  9},         
    {2500,  0}  
};

/*!
 *  @brief this is the OCV/SoC table for a LiFeYPO4 battery
 *  @note Change this table to the specification of the used LiFeYPO4 battery
 */
static const mvSoC_t cellmvVsSOCLiFeYPO4LookupTable[] =
{
    {3600, 100},
    {3500, 99},
    {3300, 95},
    {3285, 90},
    {3270, 80},
    {3260, 70},
    {3250, 60},
    {3225, 50},
    {3200, 40},
    {3180, 30},
    {3170, 20},
    {3100, 17},
    {3050, 14},
    {3000, 9},
    {2800, 0}
};

/*!
 *  @brief this is the OCV/SoC table for a NMC LiPo (LiNiMnCoO2) battery
 *  @note Change this table to the specification of the used NMC LIPO battery
 */
static const mvSoC_t cellmvVsSOCLiPoNMCLookupTable[] =
{
    {4192, 100},
    {4142, 97},
    {4113, 95},
    {4087, 92},
    {4061, 89},
    {4042, 87},
    {4013, 84},
    {3973, 81},
    {3947, 79},
    {3928, 76},
    {3913, 74},
    {3895, 71},
    {3878, 68},
    {3861, 66},
    {3845, 63},
    {3830, 60},
    {3816, 58},
    {3803, 55},
    {3790, 52},
    {3779, 50},
    {3768, 47},
    {3757, 44},
    {3748, 42},
    {3740, 39},
    {3733, 36},
    {3726, 34},
    {3719, 31},
    {3712, 28},
    {3704, 26},
    {3696, 23},
    {3685, 20},
    {3672, 18},
    {3660, 15},
    {3646, 12},
    {3630, 10},
    {3567, 7},
    {3450, 6},
    {3350, 5},
    {3203, 4},
    {3150, 3},
    {3114, 2},
    {3000, 0},
};

/*!
 *  @brief this is the OCV/SoC table for a sodium-ion (Na-Ion) battery
 *  @note Change this table to the specification of the used sodium-ion (Na-ion) battery
 */
static const mvSoC_t cellmvVsSOCNaIonLookupTable[] =
{
    {4100, 100},
    {4000, 100},
    {3962, 98},
    {3924, 96},
    {3886, 94},
    {3848, 92},
    {3810, 90},
    {3772, 88},
    {3734, 86},
    {3696, 84},
    {3658, 82},
    {3620, 80},
    {3582, 78},
    {3544, 76},
    {3506, 74},
    {3468, 72},
    {3430, 70},
    {3392, 68},
    {3354, 66},
    {3316, 64},
    {3278, 62},
    {3240, 60},
    {3202, 58},
    {3164, 56},
    {3126, 54},
    {3088, 52},
    {3050, 50},
    {3012, 48},
    {2974, 46},
    {2936, 44},
    {2898, 42},
    {2860, 40},
    {2822, 38},
    {2784, 36},
    {2746, 34},
    {2708, 32},
    {2670, 30},
    {2632, 28},
    {2594, 26},
    {2556, 24},
    {2518, 22},
    {2480, 20},
    {2442, 18},
    {2404, 16},
    {2366, 14},
    {2328, 12},
    {2290, 10},
    {2252, 8},
    {2214, 6},
    {2176, 4},
    {2138, 2},
    {2100, 0},
    {1500, 0}
};


/*!
 *  @brief this array contains the pointers to the OCV/SoC tables
 *  @note use with BATTERY_TYPE variable
 *  @example cellmvVsSOCLookupTableAll[batteryType][row].milliVolt
 */
static const mvSoC_t* cellmvVsSOCLookupTableAll[] = { &cellmvVsSOCLiPoLookupTable[0],
    &cellmvVsSOCLiFePO4LookupTable[0], &cellmvVsSOCLiFeYPO4LookupTable[0], 
    &cellmvVsSOCLiPoNMCLookupTable[0], &cellmvVsSOCNaIonLookupTable[0]};

/*!
 *  @brief this array contains the size of the OCV/SoC tables
 *  @note use with BATTERY_TYPE variable
 */
static const uint8_t cellmvVsSOCLookupTableAllSize[] = { sizeof(cellmvVsSOCLiPoLookupTable) / sizeof(mvSoC_t),
    sizeof(cellmvVsSOCLiFePO4LookupTable) / sizeof(mvSoC_t),
    sizeof(cellmvVsSOCLiFeYPO4LookupTable) / sizeof(mvSoC_t),
    sizeof(cellmvVsSOCLiPoNMCLookupTable) /sizeof(mvSoC_t),
    sizeof(cellmvVsSOCNaIonLookupTable) / sizeof(mvSoC_t)};

/**
 * NTC look up table intended for resistance to temperature conversion. After
 * table initialization, array item contains raw value from a register.
 * Index of the item is temperature value.
 */
static uint16_t g_ntcTable[NTC_TABLE_SIZE];

static uint8_t gMeasurementCounter         = 1;
static uint8_t gMeasurementCounterEndValue = 100;

static float*  gMovingAvgArr;
static uint8_t gMovingAvgArrElements = 0;
static uint8_t gMovingAvgArrIndex    = 0;

/*! @brief  mutex for the calc delta charge function */
static pthread_mutex_t gDChargeFuncMutex;
static bool            gDChargeFuncMutexInitialized = false;

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

/*!
 * @brief This function calculates temperature from raw value of MEAS_ANx
 *        register. It uses precalculated values stored in g_ntcTable table.
 *        You can use function BCC_Meas_GetRawValues to get values of measurement
 *        registers.
 *
 * @param regVal Value of MEAS_ANx register.
 * @param temp Temperature value in deg. of Celsius * 10.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t getNtcCelsius(uint16_t regVal, int16_t* temp);

/*
 * @brief   This function calculates the state of charge (SoC) and state of health (SoH)
 *
 * @param   calcBatteryVariables Pointer to the local calcBatteryVariables_t struct to get the information
 *          from and set the new s-charge in.
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int calculateSoCAndSoH(calcBatteryVariables_t* calcBatteryVariables);

/*
 * @brief   This function can be used to get the state of charge (SoC) from the open cell voltage
 * @note    A predefined table and the lowest cell voltage will be used for this
 * @warning The battery (voltage) needs to be relaxed before this is used!
 *
 * @param   batteryType The battery-type (BATTERY_TYPE)
 * @param   CurrentCheck if true the current check will be done,
 *          Keep this default on true, except for a power-up check
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
static uint8_t getSoCBasedOnOCV(uint8_t batteryType, uint16_t lowestCellmV);

/*******************************************************************************
 * API
 ******************************************************************************/

/*
 * @brief   This function will initialize the moving average and the mutex
 *          this function should be called before bcc_monitoring_updateMeasurements
 *          Developed by C. van Mierlo. \n
 *
 * @param   none
 *
 * @return  int Error code (of the mutex)
 */
int bcc_monitoring_initialize(void)
{
    int retVal = gDChargeFuncMutexInitialized;

    // check if not initialized
    if(!gDChargeFuncMutexInitialized)
    {
        // initialize the mutex
        retVal = pthread_mutex_init(&gDChargeFuncMutex, NULL);

        // allocate the moving array standard lenght
        gMovingAvgArr = (float*)calloc(STANDARD_MOVING_AVG_SIZE, sizeof(float));

        // set the global value
        gMovingAvgArrElements = STANDARD_MOVING_AVG_SIZE;

        // set the variable to true
        gDChargeFuncMutexInitialized = true;
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
    double   ntcVal, expArg;
    uint16_t i = 0;
    int32_t  temp;

    for(temp = NTC_MINTEMP; temp <= NTC_MAXTEMP; temp++)
    {
        // calculate the new exponent arguement (beta*(1/temp(k) - 1/tempRef(k)))
        expArg = ntcConfig->beta * ((1.0 / (NTC_DEGC_0 + temp)) - (1.0 / (NTC_DEGC_0 + ntcConfig->refTemp)));

        // calculate the NTC value e^expArg * Rref voltage is ((NTC_VCOM * ntcVal) / (ntcVal +
        // ntcConfig->rntc))
        ntcVal = exp(expArg) * ntcConfig->refRes;

        // calculate the registervalue for each temperature
        g_ntcTable[i] =
            (uint16_t)round(((NTC_VCOM * ntcVal) / (ntcVal + ntcConfig->rntc)) / NTC_REGISTER_RES);

        // increase the index
        i++;
    }
}

/*
 * @brief   This function reads values measured and provided via SPI
 *          by BCC device (ISENSE, cell voltages, temperatures). \n
 *          it will set the values in pCommonBatteryVariables
 *          Developed by M. Musak and adapted by C. van Mierlo. \n
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   rShunt Shunt resistor for ISENSE in [uOhm].
 * @param   lowestCellVoltageAdr this function will set the lowest cell voltage in this variable
 * @param   measureEverything if true, it will measure everything
 * @param   pCommonBatteryVariables Pointer to the local commonBatteryVariables_t struct to store the results
 *          in
 *
 * @return  bcc_status_t Error code or ONLY_CURRENT_RETURN when only current measurement has been done
 */
bcc_status_t bcc_monitoring_updateMeasurements(bcc_drv_config_t* const drvConfig, uint32_t rShunt,
    float* lowestCellVoltageAdr, bool measureEverything, commonBatteryVariables_t* pCommonBatteryVariables)
{
    /* Variables *****************************************************************/
    bcc_status_t    error;                      // Error status.
    uint16_t        measurements[BCC_MEAS_CNT]; // Array needed to store all measured values.
    variableTypes_u variable1;
    int             i = 0;
    float           lowestCellVoltage;
    charge_states_t chargeState = data_getChargeState();

#ifdef DEBUG_TIMING
    struct timespec firstTime, currentTime;

    if(clock_gettime(CLOCK_REALTIME, &firstTime) == -1)
    {
        cli_printfError("bcc_monitoring ERROR: failed to get newSampletime time!\n");
    }

    // cli_printf("time: %ds %dus\n", firstTime.tv_sec, firstTime.tv_nsec/1000);
#endif

    // check for NULL pointer, but only in debug mode
    DEBUGASSERT(drvConfig != NULL);
    DEBUGASSERT(lowestCellVoltageAdr != NULL);
    DEBUGASSERT(pCommonBatteryVariables != NULL);

    // lock the BCC SPI until the measurement is done and read
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't lock BCC SPI\n");
    }

    /* Start ADC conversion ******************************************************/
    error = bcc_monitoring_doBlockingMeasurement(drvConfig);

    // check for an error
    if(error != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring ERROR: Couldn't do measurement error: %d\n", error);

        // unlock the BCC SPI
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
        }

        return error;
    }

    // get the current measument
    error = bcc_spiwrapper_BCC_Reg_Read(
        drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_ISENSE1_ADDR, 2, &measurements[BCC_MSR_ISENSE1]);

    // unlock the BCC SPI
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
    }

    // check for an error
    if(error != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring ERROR: couldn't get current error: %d\n", error);
        return error;
    }

    /* Mask bits. */
    /* Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2 registers. */
    measurements[BCC_MSR_ISENSE1] &= BCC_R_MEAS1_I_MASK;
    measurements[BCC_MSR_ISENSE2] &= BCC_R_MEAS2_I_MASK;

    // convert the battery current to a float in mA
    // Measured ISENSE in [mA]. Value of shunt resistor is used.
    pCommonBatteryVariables->I_batt =
        BCC_GET_ISENSE_AMP(rShunt, measurements[BCC_MSR_ISENSE1], measurements[BCC_MSR_ISENSE2]);

    // Get the system current
    if(data_getParameter(I_SYSTEM, &(variable1.uint8Var), NULL) == NULL)
    {
        cli_printfError("bcc_monitoring ERROR: Couldn't get i-system\n");
    }

    // Check if substracting own board current during charging is needed
    // Check if the current is positive (charging) (including board current)
    if(((int)pCommonBatteryVariables->I_batt) >= (variable1.uint8Var))
    {
        // Get the state and check if it is in the charging state (CHARGE_START or CHARGE_CB)
        if(data_getMainState() == CHARGE && (chargeState == CHARGE_START || chargeState == CHARGE_CB))
        {
            // Substract the system current because that is measured as well
            pCommonBatteryVariables->I_batt = pCommonBatteryVariables->I_batt - (float)(variable1.uint8Var);
        }
    }

    // convert to A
    pCommonBatteryVariables->I_batt = pCommonBatteryVariables->I_batt / MA_TO_A;


#ifdef OUTPUT_CURRENT_MEAS_DOT
    // to indicate the current is set
    cli_printf(".");
#endif

    // check if it should not measure everything
    if(!measureEverything)
    {
        // Do not save or handle the changed parameter here, this will be done in batManagement

        // return with only current return
        return ONLY_CURRENT_RETURN;
    }

    // check if not initialized
    if(!gDChargeFuncMutexInitialized)
    {
        // set the error
        error = BCC_STATUS_SPI_INIT;

        // error to user
        cli_printfError("bcc_monitoring ERROR: mutex isn't initialized!\n");

        // return error
        return error;
    }

#ifdef OUTPUT_CURRENT_MEAS_DOT

    cli_printf("\n");
#endif

#ifdef OUTPUT_UPDATE_OTHER_MEAS
    cli_printf("Updating other meas!\n");
#endif

    // get the sensor enable and the n-cells in the struct
    // get the amount of cells
    if(data_getParameter(N_CELLS, &(pCommonBatteryVariables->N_cells), NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_updateMeasurements ERROR: Couldn't get n-cells!\n");

        // set the default value
        pCommonBatteryVariables->N_cells = N_CELLS_DEFAULT;
    }

    // get the variable to know if the battery temperature measurement should be done
    if(data_getParameter(SENSOR_ENABLE, &(pCommonBatteryVariables->sensor_enable), NULL) == NULL)
    {
        cli_printfError("bcc_monitoring ERROR: Couldn't get sensor-enable!\n");

        // set the default value
        pCommonBatteryVariables->sensor_enable = SENSOR_ENABLE_DEFAULT;
    }

    // lock the BCC SPI until the measurement is done and read
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't lock BCC SPI\n");
    }

    // get stack voltage
    error = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_STACK_ADDR, 1,
        (uint16_t*)(measurements + ((uint8_t)BCC_MSR_STACK_VOLT)));

    // check for an error
    if(error != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring ERROR: Couldn't get stack voltage error: %d\n", error);
        return error;
    }

    // get the cell voltages and the used analog channels
    error = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_CELLX_ADDR_MC33772_START,
        (BCC_REG_MEAS_ANX_ADDR_END - BCC_REG_MEAS_CELLX_ADDR_MC33772_START) + 1,
        (uint16_t*)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));

    // unlock the BCC SPI
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
    }

    // check for an error
    if(error != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring ERROR: Couldn't other measurements error: %d\n", error);
        return error;
    }


    /* Mask the other registers (starting at 5th register). */
    for(i = 5U; i < BCC_MEAS_CNT; i++)
    {
        measurements[i] &= BCC_R_MEAS_MASK;
    }

    // calculate the rest

    /* Voltages calculations ******************************************************/

    // reset the battery voltage
    pCommonBatteryVariables->V_batt = 0;

    // get the cell voltages
    for(i = 0; i < pCommonBatteryVariables->N_cells; i++)
    {
        // map the cell number to the bcc cell number
        // check if it is not the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            variable1.uint8Var = (6 - pCommonBatteryVariables->N_cells) + i;
        }
        else
        {
            // it is the first 2 cells
            variable1.uint8Var = i;
        }

        // convert the measured cell voltage to a float variable cell voltage
        pCommonBatteryVariables->V_cellVoltages.V_cellArr[i] =
            BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1 - variable1.uint8Var]) / UV_TO_V;

        // if the first
        if(i == 0)
        {
            // set the first voltage
            lowestCellVoltage = pCommonBatteryVariables->V_cellVoltages.V_cellArr[i];
        }

        // add all the cell voltages to the battery voltage
        pCommonBatteryVariables->V_batt += pCommonBatteryVariables->V_cellVoltages.V_cellArr[i];

        // check if the lowest
        if(lowestCellVoltage > pCommonBatteryVariables->V_cellVoltages.V_cellArr[i])
        {
            // set the new lowest cell voltage
            lowestCellVoltage = pCommonBatteryVariables->V_cellVoltages.V_cellArr[i];
        }
    }

    // save the lowest cell voltage
    // set the lowest cell voltage in the lowest cell voltage address
    *lowestCellVoltageAdr = lowestCellVoltage;

    // convert the measured battery stack voltage to a float voltage
    variable1.floatVar =
        BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]) / UV_TO_V; // Stack voltage in [uV].

    // check if the battery stack voltage is too different than the sum of the cells for redundancy
    if(((pCommonBatteryVariables->V_batt - variable1.floatVar) >
           (STACK_VOLTAGE_DIFFERENCE_ERROR * pCommonBatteryVariables->N_cells)) ||
        ((pCommonBatteryVariables->V_batt - variable1.floatVar) <
            -(STACK_VOLTAGE_DIFFERENCE_ERROR * pCommonBatteryVariables->N_cells)))
    {
        // check if not in the fault state
        if(data_getMainState() != FAULT_ON && data_getMainState() != FAULT_OFF)
        {
            // output error
            cli_printfError("bcc_monitoring ERROR: stackvoltage too different from sum of cells! stack: "
                            "%8.3fV cells: %8.3fV\n",
                variable1.floatVar, pCommonBatteryVariables->V_batt);
        }

        // set the measured stack voltage as battery voltage
        pCommonBatteryVariables->V_batt = variable1.floatVar;

        // set the error in the status flags
        data_statusFlagBit(STATUS_BMS_ERROR_BIT, 1);

        // TODO add error if this happens? go to FAULT state?
    }

    // get the v-out voltage divider variable
    if(data_getParameter(F_V_OUT_DIVIDER_FACTOR, &(variable1.floatVar), NULL) == NULL)
    {
        cli_printfWarning(
            "bcc_monitoring: Using default v-out voltage factor (%f)\n", F_V_OUT_DIVIDER_FACTOR_DEFAULT);

        // set the default value
        variable1.floatVar = F_V_OUT_DIVIDER_FACTOR_DEFAULT;
    }

    // convert the output voltage to a float
    pCommonBatteryVariables->V_out = variable1.floatVar * BCC_GET_VOLT(measurements[BCC_MSR_AN4]) / UV_TO_V;

    // check if the battery temperature measurement should be done
    if(pCommonBatteryVariables->sensor_enable & 1)
    {
        // get the battery temperature
        error = getNtcCelsius(measurements[BCC_MSR_AN1], &(variable1.int16Var));

        // check for errors
        if(error)
        {
            cli_printfError("bcc_monitoring ERROR: failed getting batt temp error: %d\n", error);
            cli_printf("Maybe disable the measurement? with \"bms set sensor-enable 0\"\n");
        }
        else
        {
            // make it a floating point value and set the value
            pCommonBatteryVariables->C_batt = variable1.int16Var / T10_TO_T;
        }
    }

    // get the AFE temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN3], &(variable1.int16Var));

    // check for errors
    if(error)
    {
        cli_printfError("bcc_monitoring ERROR: failed getting AFE temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value and set the value
        pCommonBatteryVariables->C_AFE = variable1.int16Var / T10_TO_T;
    }

    // get the transistor temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN2], &(variable1.int16Var));

    // check for errors
    if(error)
    {
        cli_printfError("bcc_monitoring ERROR: failed getting T temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value and set the value
        pCommonBatteryVariables->C_T = variable1.int16Var / T10_TO_T;
    }

    // get the sense resistor temperature
    error = getNtcCelsius(measurements[BCC_MSR_AN0], &(variable1.int16Var));

    // check for errors
    if(error)
    {
        cli_printfError("bcc_monitoring ERROR: failed getting R temp error: %d\n", error);
    }
    else
    {
        // make it a floating point value and set the value
        pCommonBatteryVariables->C_R = variable1.int16Var / T10_TO_T;
    }

    // the struct is copied back in data after the currents are calculated (in batManagement.c)

    return error;
}

/*
 * @brief   This function calculates the battery variables
 *          It will use the calcBatteryVariables_t struct to get and set them.
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   gateMutexAdr this is the address of the gate mutex, to check a pin for the output status
 * @param   lowestCellVoltage The lowest cell voltage in V as a float
 * @param   pCommonBatteryVariables Pointer to the local commonBatteryVariables_t struct to store the results
 *          in
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
int bcc_monitoring_calculateVariables(bcc_drv_config_t* const drvConfig, pthread_mutex_t* gateMutexAdr,
    float lowestCellVoltage, commonBatteryVariables_t* pCommonBatteryVariables)
{
    static bool            firstPowerup = true;
    static float           sumOfMovAvg  = 0;
    variableTypes_u        variable1;
    variableTypes_u        variable2;
    calcBatteryVariables_t calcBatteryVariable;

    // check for NULL pointer in debug mode
    DEBUGASSERT(drvConfig != NULL);
    DEBUGASSERT(gateMutexAdr != NULL);
    DEBUGASSERT(pCommonBatteryVariables != NULL);

    // mutex on calcBatteryVariables struct
    if(data_lockMutex())
    {
        cli_printfError("bcc_monitoring_calculateVariables ERROR: Could not lock on data!\n");
    }

    // copy the calcBatteryVariables struct in calcBatteryVariable
    // and use try lock since this task has the lock
    variable2.int32Var = data_getCalcBatteryVariables(&calcBatteryVariable, true);
    if(variable2.int32Var)
    {
        cli_printfError(
            "bcc_monitoring_calculateVariables ERROR: Could not get batt vars! %d\n", variable2.int32Var);
    }

    // calc everything
    // check if the voltage is lower than the defined on voltage
    if(pCommonBatteryVariables->V_out < OUTPUT_ON_VOLTAGE)
    {
        // set the output_status value off
        calcBatteryVariable.s_out = 0;
    }
    else
    {
        // lock the gate mutex
        pthread_mutex_lock(gateMutexAdr);

        // read the gate CTRL D pin
        calcBatteryVariable.s_out = (!gpio_readPin(GATE_CTRL_D)) & 1;

        // unlock the gate mutex
        pthread_mutex_unlock(gateMutexAdr);

        // check if gate is enabled in software
        if(calcBatteryVariable.s_out)
        {
            // check if the shortcut pin (overcurrent) is not high
            calcBatteryVariable.s_out = (!gpio_readPin(OVERCURRENT)) & 1;
        }
    }

    // if the first time
    // oldsample is the newsmaple, to make sure there is a difference in time of 0
    if(firstPowerup)
    {
        // get the battery type
        variable1.uint8Var = *(uint8_t*)data_getAdr(BATTERY_TYPE);

        // get the state of charge based on the lowest cell voltage
        calcBatteryVariable.s_charge =
            getSoCBasedOnOCV(variable1.uint8Var, (uint16_t)(lowestCellVoltage * 1000));

        // calculate the new remaining capacity
        calcBatteryVariable.A_rem = (calcBatteryVariable.A_full / 100) * calcBatteryVariable.s_charge;

        // it is not the first power up anymore
        firstPowerup = false;
    }

    // check if the CC needs to be read
    // This interval is the (T_MEAS_MAX/t-meas)/elements for t-meas <= 1000
    // If t-meas is 1000ms, it will be (10000/1000)/10 = 1s.
    if(gMeasurementCounter >= gMeasurementCounterEndValue)
    {
        // calculate the average current and the delta charge (in variable2.floatVar)
        variable1.int32Var = bcc_monitoring_calcDCharge(
            drvConfig, &(pCommonBatteryVariables->I_batt_avg), &(variable2.floatVar), true);

        // check for errors
        if(variable1.int32Var)
        {
            // return error
            cli_printfError("bcc_monitoring ERROR: failed to calc Dcharge! %d\n", variable1.int32Var);

            // set the returnvalue
            variable1.int32Var = BCC_STATUS_PARAM_RANGE;
        }
        else
        {
            // get the system current
            // do this via data_getAdr since the mutex is locked
            variable1.uint8Var = *(uint8_t*)data_getAdr(I_SYSTEM);

            // Check if substracting own board current during charging is needed
            // Check if the current is positive (charging) (including board current)
            if((pCommonBatteryVariables->I_batt_avg * 1000) >= (variable1.uint8Var))
            {
                // Get the state and check if it is in the charging state (CHARGE_START or CHARGE_CB)
                if(data_getMainState() == CHARGE &&
                    (data_getChargeState() == CHARGE_START || data_getChargeState() == CHARGE_CB))
                {
                    // Substract the system current because that is measured as well
                    pCommonBatteryVariables->I_batt_avg =
                        pCommonBatteryVariables->I_batt_avg - (float)(variable1.uint8Var / 1000.0);
                }
            }

            // calculate the new energy used
            // add the difference in charge in Ah times the battery output voltage
            calcBatteryVariable.E_used += (-variable2.floatVar * pCommonBatteryVariables->V_out);

            // calculate the new remaining capacity with the difference in charge
            calcBatteryVariable.A_rem = (calcBatteryVariable.A_rem + variable2.floatVar);

            // limit the remaining capacity if less than 0
            if(calcBatteryVariable.A_rem < 0)
            {
                // set the remaining capacity to 0
                calcBatteryVariable.A_rem = 0;
            }

            // remove the old value from the sum
            sumOfMovAvg -= gMovingAvgArr[gMovingAvgArrIndex];

            // add the new value to the sum
            sumOfMovAvg += pCommonBatteryVariables->I_batt_avg;

            // divide the sum of currents by the amount of elements
            // To get the average current over the whole moving average (10s)
            pCommonBatteryVariables->I_batt_10s_avg = sumOfMovAvg / gMovingAvgArrElements;

            // set the new value in the array
            gMovingAvgArr[gMovingAvgArrIndex] = pCommonBatteryVariables->I_batt_avg;

            // increase the index
            gMovingAvgArrIndex = (gMovingAvgArrIndex + 1) % gMovingAvgArrElements;

            // calculate the state of charge
            if(calculateSoCAndSoH(&calcBatteryVariable))
            {
                cli_printfError("bcc_monitoring ERROR: Could not calculate s-charge or s-health!\n");
            }

            // the change on s-charge will be handled in the updater task

            // get the battery eol percentage
            // do this via data_getAdr since the mutex is locked
            variable1.uint8Var = *(uint8_t*)data_getAdr(BATT_EOL);

            // check if the battery is end of life (EOL)
            if(calcBatteryVariable.s_health <= variable1.uint8Var)
            {
                // get the s-flags
                variable2.uint8Var = *(uint8_t*)data_getAdr(S_FLAGS);

                // set the bad battery bit
                variable2.uint8Var |= 1 << STATUS_BAD_BATTERY_BIT;

                // set the new s-flags
                *(uint8_t*)data_getAdr(S_FLAGS) = variable2.uint8Var;
            }

            // get the max charge Current
            // do this via data_getAdr since the mutex is locked
            variable1.floatVar = *(float*)data_getAdr(I_CHARGE_MAX);

            // check if the current is higher than the max charge current or it is not charging
            if(pCommonBatteryVariables->I_batt_10s_avg < 0)
            {
                // set the time to 0 since it is not charging
                calcBatteryVariable.t_full = 0;
            }
            else if(pCommonBatteryVariables->I_batt_10s_avg > variable1.floatVar)
            {
                // calculate the time left to charge based on max charge current
                calcBatteryVariable.t_full =
                    (calcBatteryVariable.A_full - calcBatteryVariable.A_rem) / variable1.floatVar;
            }
            else
            {
                // calculate the time left to charge based on the avg current
                calcBatteryVariable.t_full = (calcBatteryVariable.A_full - calcBatteryVariable.A_rem) /
                    pCommonBatteryVariables->I_batt_10s_avg;
            }
            // TODO add balancing time

            // check for limits
            if(calcBatteryVariable.t_full < 0.001)
            {
                // set it to the minimum limit
                calcBatteryVariable.t_full = 0.001;
            }

            // this calculate the avg power Vout*Iavg10s
            calcBatteryVariable.P_avg =
                pCommonBatteryVariables->V_out * pCommonBatteryVariables->I_batt_10s_avg;

            // reset the measurement counter
            gMeasurementCounter = 0;
            // Return OK
            variable1.int32Var = 0;

#ifdef DEBUG_TIMING

            // get the current time
            if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
            {
                cli_printfError("bcc_monitoring ERROR: failed to get newSampletime time!\n");
            }

            // calculate the difference in time
            variable2.int32Var = (currentTime.tv_sec * 1000000 + currentTime.tv_nsec / 1000) -
                (firstTime.tv_sec * 1000000 + firstTime.tv_nsec / 1000);

            cli_printf("dtime: %dus\n", variable2.int32Var);
#endif
        }
    }

    // increase the measurementCounter
    gMeasurementCounter++;

    // set calcBatteryVariables struct with calcBatteryVariable
    // and use try lock since this task has the lock
    variable2.int32Var = data_setCalcBatteryVariables(&calcBatteryVariable, true);
    if(variable2.int32Var)
    {
        cli_printfError(
            "bcc_monitoring_calculateVariables ERROR: Could not set batt vars! %d\n", variable2.int32Var);
    }

    // unlock mutex
    if(data_unlockMutex())
    {
        cli_printfError("bcc_monitoring_calculateVariables ERROR: Could not unlock on data!\n");
    }

    // return
    return variable1.int32Var;
}

/*
 * @brief   This function is used to do a meaurement
 *          This function is blocking and will wait until the measurement is done
 *
 * @note    Even blocks task switch
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_doBlockingMeasurement(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t error;
    int          errorCounter = 0;
    bool         completed    = false;

    // lock on this thread (no task switch!)
    sched_lock();

    // lock the BCC SPI until the measurement is done and read
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't lock BCC SPI\n");
    }

    // do the measuremetns until no error is given for 5 tries
    do
    {
        // do the measurements
        error = bcc_spiwrapper_BCC_Meas_StartConversion(drvConfig, BCC_CID_DEV1); // Error verification.

        // increase i to only loop an amount of time
        errorCounter++;

    } while(error != BCC_STATUS_SUCCESS && errorCounter < 5);

    // check for errors
    if(error != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
        }

        // return error
        cli_printfError(
            "bcc_monitoring_doBlockingMeasurement ERROR: Couldn't start conversion! error: %d i = %d\n",
            error, errorCounter - 1);

        // unlock this thread (enable task switch!)
        sched_unlock();

        return error;
    }

    // reset the error counter
    errorCounter = 0;

    // wait until the conversion is complete
    do
    {
        // check if converting with error check
        error = bcc_spiwrapper_BCC_Meas_IsConverting(drvConfig, BCC_CID_DEV1, &completed);
        if(error != BCC_STATUS_SUCCESS)
        {
            // increment the error counter
            errorCounter++;
        }
    } while(!completed && (errorCounter < 5));

    // check if it returned because of the error
    if(!completed)
    {
        // output error
        cli_printfError(
            "bcc_monitoring_doBlockingMeasurement ERROR: Couldn't check conversion! error: %d i amount: %d\n",
            error, errorCounter);
    }

    // unlock the BCC SPI
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("bcc_monitoring_update ERROR: couldn't unlock BCC SPI\n");
    }

    // unlock this thread (enable task switch!)
    sched_unlock();

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
bcc_status_t bcc_monitoring_checkOutput(bcc_drv_config_t* const drvConfig, bool* output)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t     regVal;
    float        batVoltage;

    // check if the address is not NULL
    if(output == NULL)
    {
        cli_printfError("bcc_monitoring_checkOutput ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // get the AN4 measurement
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_AN4_ADDR), 1, &regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring_checkOutput ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask the measurement
    regVal &= BCC_R_MEAS_MASK;

    // get the v-out voltage divider variable and place it in batVoltage for now
    if(data_getParameter(F_V_OUT_DIVIDER_FACTOR, &(batVoltage), NULL) == NULL)
    {
        cli_printfWarning("bcc_monitoring_checkOutput: Using default v-out voltage factor (%f)\n",
            F_V_OUT_DIVIDER_FACTOR_DEFAULT);

        // set the default value
        batVoltage = F_V_OUT_DIVIDER_FACTOR_DEFAULT;
    }

    // convert the battery voltage to a float
    batVoltage = batVoltage * BCC_GET_VOLT(regVal) / UV_TO_V;

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
bcc_status_t bcc_monitoring_checkNCells(bcc_drv_config_t* const drvConfig, bool* nCellsOk)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t     regVal;
    float        cellVoltage, cellVoltageTotal;
    float        stackVoltage;
    int          bccIndex, i;
    uint8_t      nCells, nCellsProb;

    // check if the address is not NULL
    if(nCellsOk == NULL)
    {
        cli_printfError("bcc_monitoring_checkNCells ERROR: NULL pointer!\n");
        return lvRetValue;
    }

    // set it to not ok
    *nCellsOk = 0;

    // get the amount of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_checkNCells ERROR: getting N_CELLS went wrong! val %d\n", nCells);
        nCells = N_CELLS_DEFAULT;
    }

    // reset the battery voltage
    cellVoltageTotal = 0;

    // get the cell voltages
    for(i = 0; i < nCells; i++)
    {
        // check if it is the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            bccIndex = (6 - nCells) + i;
        }
        else
        {
            // it is the first 2 cells
            bccIndex = i;
        }

        // read the cell voltage
        lvRetValue = bcc_spiwrapper_BCC_Reg_Read(
            drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_CELLX_ADDR_END - bccIndex), 1, &regVal);

        // check for errors
        if(lvRetValue != BCC_STATUS_SUCCESS)
        {
            cli_printfError("bcc_monitoring_checkNCells ERROR: reading AN4 failed: %d\n", lvRetValue);
            return lvRetValue;
        }

        // mask it
        regVal &= BCC_R_MEAS_MASK;

        // convert the cell voltage to a float
        cellVoltage = BCC_GET_VOLT(regVal) / UV_TO_V;

        // cli_printf("cell%d V: %8.3f of BCCindex: %d\n", i, cellVoltage, bccIndex);

        // add all the cell voltages to the battery voltage
        cellVoltageTotal += cellVoltage;
    }

    // read the stack voltage
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_STACK_ADDR, 1, &regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring_checkNCells ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask it
    regVal &= BCC_R_MEAS_MASK;

    // convert the battery voltage to a float
    stackVoltage = BCC_GET_STACK_VOLT(regVal) / UV_TO_V; // Stack voltage in [uV].

    // check if the battery stack voltage is almost the sum of the cells for redundancy
    if(((cellVoltageTotal - stackVoltage) > (STACK_VOLTAGE_DIFFERENCE_ERROR * nCells)) ||
        ((cellVoltageTotal - stackVoltage) < -(STACK_VOLTAGE_DIFFERENCE_ERROR * nCells)))
    {
        // output error
        cli_printfError("bcc_monitoring ERROR: stackvoltage too different from sum of cells!\nstack: %.3fV "
                        "cells: %.3fV\n",
            stackVoltage, cellVoltageTotal);

        // set the error in the status flags
        data_statusFlagBit(STATUS_BMS_ERROR_BIT, 1);

        // set the variable to not OK
        *nCellsOk = 0;

        // calculate the possible number of cells
        nCellsProb = round(stackVoltage / cellVoltage);

        // limit upper
        if(nCellsProb > 6)
        {
            // 6 is the max
            nCellsProb = 6;
        }
        // limit lower
        else if(nCellsProb < 3)
        {
            // 3 is the min
            nCellsProb = 3;
        }

        // output error
        cli_printfError("bcc_monitoring ERROR: wrong n-cells!\n");
        cli_printf("n-cells is currently %d\n", nCells);

        // output to the user
        cli_printf(
            "n-cells probably needs to be %d, if so, type \"bms set n-cells %d\"\n", nCellsProb, nCellsProb);
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
    uint16_t     regVal;
    float        outputVoltage;

    // read the output voltage (AN4)
    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_ANX_ADDR_END - 4), 1, &regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring_getOutputVoltage ERROR: reading AN4 failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // mask it
    regVal &= BCC_R_MEAS_MASK;

    // get the v-out voltage divider variable and place it in outputVoltage for now
    if(data_getParameter(F_V_OUT_DIVIDER_FACTOR, &(outputVoltage), NULL) == NULL)
    {
        cli_printfWarning("bcc_monitoring_getOutputVoltage: Using default v-out voltage factor (%f)\n",
            F_V_OUT_DIVIDER_FACTOR_DEFAULT);

        // set the default value
        outputVoltage = F_V_OUT_DIVIDER_FACTOR_DEFAULT;
    }

    // convert the cell voltage to a float
    outputVoltage = outputVoltage * BCC_GET_VOLT(regVal) / UV_TO_V;

    // cli_printf("output voltage: %8.3f\n", outputVoltage);

    // set the output voltage
    if(data_setParameter(V_OUT, &outputVoltage))
    {
        cli_printfError("bcc_monitoring ERROR: setting new data went in the update wrong! par %d: %.3f \n",
            V_OUT, outputVoltage);

        // set an error
        lvRetValue = BCC_STATUS_PARAM_RANGE;
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function is used to get the cell voltages
 *          it will read the cell voltages without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_getCellVoltages(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t    lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint8_t         nCells, i, bccIndex;
    uint16_t        regVal[6];
    float           cellVoltage;
    parameterKind_t parameter;

    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_getCellVoltages ERROR: getting n-cells went wrong!\n");
        nCells = N_CELLS_DEFAULT;
    }

    // read the cell voltages
    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, (BCC_REG_MEAS_CELLX_ADDR_END - 5), 6, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError(
            "bcc_monitoring_getCellVoltages ERROR: reading cell voltages failed: %d\n", lvRetValue);
        return lvRetValue;
    }

    // loop throught the cells
    for(i = 0; i < nCells; i++)
    {
        // check if it isn't is one of the first 2 cells
        if(i >= 2)
        {
            // calculate the BCC pin index
            bccIndex = (6 - nCells) + i;
        }
        else
        {
            // it is one of the first 2 cells
            bccIndex = i;
        }

        // Mask the registers
        regVal[5 - bccIndex] &= BCC_R_MEAS_MASK;

        // convert the cell voltage to a float
        cellVoltage = BCC_GET_VOLT(regVal[5 - bccIndex]) / UV_TO_V;

        // set the new battery cell voltage
        parameter = (parameterKind_t)(V_CELL1 + i);

        // set the cell voltage
        if(data_setParameter(parameter, &cellVoltage))
        {
            cli_printfError("bcc_monitoring_getCellVoltages ERROR: setting new data went in the update "
                            "wrong! par %d: %.3f \n",
                parameter, cellVoltage);

            // set an error
            lvRetValue = BCC_STATUS_PARAM_RANGE;
        }
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function is used to get the current
 *          it will read the current without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   rShunt the value of the shunt resistor in uOhm
 * @param   currentA The address of the variable that will become the current in A if not a NULL pointer
 *          This may be NULL
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_getBattCurrent(
    bcc_drv_config_t* const drvConfig, uint32_t rShunt, float* currentA)
{
    bcc_status_t    lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t        regVal[2];
    float           current;
    charge_states_t chargeState = data_getChargeState();

    // get the current measument
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_MEAS_ISENSE1_ADDR, 2, regVal);

    // check for an error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        cli_printfError("bcc_monitoring_getBattCurrent ERROR: couldn't get current error: %d\n", lvRetValue);
        return lvRetValue;
    }

    /* Mask bits. */
    regVal[0] &= BCC_R_MEAS1_I_MASK;
    regVal[1] &= BCC_R_MEAS2_I_MASK;

    // convert the battery current to a float in mA
    // Measured ISENSE in [mA]. Value of shunt resistor is used.
    current = BCC_GET_ISENSE_AMP(rShunt, regVal[0], regVal[1]);

    // Get the system current
    if(data_getParameter(I_SYSTEM, &regVal[0], NULL) == NULL)
    {
        cli_printfError("bcc_monitoring ERROR: getting new data went wrong in the update! par %d val %d\n",
            I_SYSTEM, regVal[0]);
    }

    // Limit the value
    regVal[0] &= UINT8_MAX;

    // Check if substracting own board current during charging is needed
    // Check if the current is positive (charging) (including board current)
    if((int)(current) >= (regVal[0]))
    {
        // Get the state and check if it is in the charging state (CHARGE_START or CHARGE_CB)
        if(data_getMainState() == CHARGE && (chargeState == CHARGE_START || chargeState == CHARGE_CB))
        {
            // Substract the system current because that is measured as well
            current = current - (float)(regVal[0]);
        }
    }

    // Convert to A
    current = current / MA_TO_A;

    // set the current
    if(data_setParameter(I_BATT, &current))
    {
        cli_printfError(
            "bcc_monitoring_getBattCurrent ERROR: setting new data went in the update wrong! par %d: %.3f \n",
            I_BATT, current);

        cli_printf("Measured current: %.3fA\n", current);

        // set an error
        lvRetValue = BCC_STATUS_PARAM_RANGE;
    }

    // if not a NULL pointer
    if(currentA != NULL)
    {
        // set the current
        *currentA = current;
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function is used to get the value for the ISense pins open load detected
 * @note    This function will not contain much imformation/comments
 *          for more information see the MC3372B_SafetyManual on docstore.nxp.com
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   openLoadDetected the address of the value that is 1 if there is an open load
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_getIsenseOpenLoad(bcc_drv_config_t* const drvConfig, bool* openLoadDetected)
{
    bcc_status_t lvRetValue = BCC_STATUS_PARAM_RANGE;
    uint16_t     regVal[1], retRegVal[1];
    uint8_t      timeout;

    // lock the BCC SPI until done
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't lock BCC SPI\n");
    }

    // read the register
    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, regVal);

    // check for errors
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    timeout = (regVal[0]) >> 10;
    regVal[0] &= ~(7680);
    regVal[0] |= 5184;

    // write the registers and check for errors
    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Write(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, regVal[0], retRegVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't write SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad: couldn't get SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    regVal[0] |= 1024;

    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Write(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, regVal[0], retRegVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't write SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    // sleep for 20ms
    usleep(20 * 1000UL);

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_FAULT1_STATUS_ADDR, 1, regVal);

    // check for an error
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError(
            "bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get fault 1 error: %d\n", lvRetValue);
        return lvRetValue;
    }

    regVal[0] &= BCC_RW_IS_OL_FLT_MASK;

    if(regVal[0])
    {
        *openLoadDetected = true;
    }
    else
    {
        *openLoadDetected = false;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    regVal[0] &= ~(1024);

    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Write(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, regVal[0], retRegVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't write SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_DIAG_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_DIAG: %d\n", lvRetValue);
        return lvRetValue;
    }

    usleep(6 * 1000UL);

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    regVal[0] &= ~(7232);
    regVal[0] |= (512 + (timeout << 10));

    lvRetValue =
        bcc_spiwrapper_BCC_Reg_Write(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, regVal[0], retRegVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't write SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    lvRetValue = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG1_ADDR, 1, regVal);
    if(lvRetValue != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI until done
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
        }

        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't get SYS_CFG1: %d\n", lvRetValue);
        return lvRetValue;
    }

    // unlock the BCC SPI until done
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("bcc_monitoring_getIsenseOpenLoad ERROR: couldn't unlock BCC SPI\n");
    }

    // return to the user
    return lvRetValue;
}

/*
 * @brief   This function will set the end value (gMeasurementCounterEndValue)
 *          this will be used to check if the coulomb counter should be read and reset
 *          this function will also reset the counter
 *
 * @param   updateAverageInterval the amount of all measurements
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
 * @param   avgCurrentAdr address of the variable to become the average current
 * @param   deltaChargeAdr address of the variable to become the delta charge
 * @param   resetCC if this is true, it will reset the CC registers by writing CC_RST true
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int bcc_monitoring_calcDCharge(
    bcc_drv_config_t* const drvConfig, float* avgCurrentAdr, float* deltaChargeAdr, bool resetCC)
{
    bcc_status_t error;
    // Array of 3 needed to store CC measured values.
    uint16_t        ccMeasurements[((BCC_REG_COULOMB_CNT2_ADDR - BCC_REG_CC_NB_SAMPLES_ADDR) + 1)];
    variableTypes_u variable;

    // to get the sampletime, keep in mind that static struct timespec oldSampleTime is initialized later!
    struct timespec newSampletime;

    // make the variable for the oldtime and initialze once as the same as the sampletime
    static struct timespec oldSampleTime = { .tv_nsec = 0, .tv_sec = 0 };

    // check for NULL pointers but only in debug mode
    DEBUGASSERT(drvConfig != NULL);
    DEBUGASSERT(avgCurrentAdr != NULL);
    DEBUGASSERT(deltaChargeAdr != NULL);

    // check if mutex not is initialized
    if(!gDChargeFuncMutexInitialized)
    {
        // return
        cli_printfError("bcc_monitoring ERROR: mutex not initialzed!\n");
        return -1;
    }

    // lock the mutex
    pthread_mutex_lock(&gDChargeFuncMutex);

    // lock the BCC SPI until the measurement is done and read
    if(spi_lockNotUnlockBCCSpi(true))
    {
        cli_printfError("bcc_monitoring ERROR: couldn't lock BCC SPI\n");
    }

    // read the CC registers and reset them
    error = bcc_spiwrapper_BCC_Reg_Read(drvConfig, BCC_CID_DEV1, BCC_REG_CC_NB_SAMPLES_ADDR,
        ((BCC_REG_COULOMB_CNT2_ADDR - BCC_REG_CC_NB_SAMPLES_ADDR) + 1), ccMeasurements);

    // get the sample time
    if(clock_gettime(CLOCK_REALTIME, &newSampletime) == -1)
    {
        // unlock the BCC SPI
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring ERROR: couldn't unlock BCC SPI\n");
        }

        // unlock the mutex
        pthread_mutex_unlock(&gDChargeFuncMutex);

        cli_printfError("bcc_monitoring ERROR: failed to get newSampletime time!\n");
        return -1;
    }

    // check if first time
    if(oldSampleTime.tv_sec == 0 && oldSampleTime.tv_nsec == 0)
    {
        // make sure the difference is 0
        oldSampleTime.tv_sec  = newSampletime.tv_sec;
        oldSampleTime.tv_nsec = newSampletime.tv_nsec;
    }

    // check for an error
    if(error != BCC_STATUS_SUCCESS)
    {
        // unlock the BCC SPI
        if(spi_lockNotUnlockBCCSpi(false))
        {
            cli_printfError("bcc_monitoring ERROR: couldn't unlock BCC SPI\n");
        }

        // unlock the mutex
        pthread_mutex_unlock(&gDChargeFuncMutex);

        cli_printfError("bcc_monitoring ERROR: Couldn't get CC registers error: %d\n", error);
        return error;
    }

    // maybe set CC_RST of ADC_CFG to reset CC or check CC_RST_CFG from ADC2_OFFSET_COMP
    if(resetCC)
    {
        // reset the CC by writing the CC_RST bit in ADC_CFG
        error = bcc_spiwrapper_BCC_Reg_Update(
            drvConfig, BCC_CID_DEV1, BCC_REG_ADC_CFG_ADDR, BCC_W_CC_RST_MASK, 0xFF);

        // check for errors
        if(error != BCC_STATUS_SUCCESS)
        {
            cli_printfError("bcc_monitoring ERROR: Couldn't reset CC registers error: %d\n", error);
        }
    }

    // calulate the currents and capacities

    // (coulombcountTotalk)*V2res/Rshint[uOhm] = ACCk [uA]
    // (coulombcountTotalk-1)*V2res/Rshint[uOhm] = ACCk-1 [uA]
    // CC_NB_SAMPLESk - CC_NB_SAMPLESk-1 = dNk
    // dt = tk - tk-1

    // dCharge = ((ACCk – ACCk-1) / dNk) * dt
    // chargek = chargek-1 + dCharge

    // get the sum of the current in A
    variable.floatVar =
        (((ccMeasurements[BCC_MSR_COULOMB_CNT1] << 16) + ccMeasurements[BCC_MSR_COULOMB_CNT2]) *
            V2RES_DIV_RSHUNT);

    // get the average current in A over the samples
    *avgCurrentAdr = ((variable.floatVar) / (ccMeasurements[BCC_MSR_CC_NB_SAMPLES]));

    // get the difference in time in ms (could use T_meas)
    variable.int32Var = ((newSampletime.tv_nsec / 1000000) + (newSampletime.tv_sec * 1000)) -
        ((oldSampleTime.tv_nsec / 1000000) + (oldSampleTime.tv_sec * 1000));

    // check for limits
    if(variable.int32Var < 0)
    {
        // set it to 0
        variable.int32Var = 0;
    }

    // get the difference in charge in Ah
    *deltaChargeAdr = *avgCurrentAdr * (float)(variable.int32Var) / (3600000);

#ifdef OUTPUT_AVG_TIME
    cli_printf("dt: %dms\n", variable.int32Var);
#endif

    // save the old time
    oldSampleTime.tv_nsec = newSampletime.tv_nsec;
    oldSampleTime.tv_sec  = newSampletime.tv_sec;

    // unlock the BCC SPI
    if(spi_lockNotUnlockBCCSpi(false))
    {
        cli_printfError("bcc_monitoring ERROR: couldn't unlock BCC SPI\n");
    }

    // unlock the mutex
    pthread_mutex_unlock(&gDChargeFuncMutex);

    // return OK to the user
    return 0;
}

/*
 * @brief   This function can be used to calibrate the state of charge (SoC)
 * @note    A measurement needs to be done first and the cell voltages need to be saved
 * @note    A predefined table and the lowest cell voltage will be used for this
 * @note    Can be called from mulitple threads
 * @warning The battery (voltage) needs to be relaxed before this is used!
 *
 * @param   calibrateARem if true, it will set the a-rem to calibrate SoC (mostly needed).
 *          If false, it will calibrate a-full based on a-rem with correct SoC (charge complete).
 * @param   CurrentCheck if true the current check will be done,
 *          Keep this default on true, except for a power-up check
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int bcc_monitoring_calibrateSoC(bool calibrateARem, bool currentCheck)
{
    int     lvRetValue = -1;
    float   lowestCellVoltage, cellVoltageOrCapacity;
    uint8_t nCells, i, StateOfCharge, batteryType;

    // check if the current check needs to be done
    if(currentCheck)
    {
        // get the sleepcurrent in mA and place it in nCells
        if(data_getParameter(I_SLEEP_OC, &nCells, NULL) == NULL)
        {
            cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting sleep current went wrong!\n");
        }
        // if it went good
        else
        {
            // get the battery current and place it in cellVoltageOrCapacity
            if(data_getParameter(I_BATT, &cellVoltageOrCapacity, NULL) == NULL)
            {
                cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting current went wrong!\n");
            }
            // if it went good
            else
            {
                // check if the battery current is higher than the sleepcurrent
                // the battery should be relaxed for at least a couple of minutes
                // drawing more than the sleepcurrent means that the battery is not relaxed
                if(abs((int)(cellVoltageOrCapacity * 1000)) > nCells)
                {
                    // output the error message
                    cli_printfError("bcc_monitoring_calibrateSoC ERROR: Battery is not relaxed!\n");
                    cli_printf("Abs battery current %.0fmA > %dmA sleepcurrent\n",
                        abs((int)(cellVoltageOrCapacity * 1000)), nCells);

                    // return
                    return lvRetValue;
                }
            }
        }
    }

    // get the lowest cell voltage voltage
    // get the number of cells
    if(data_getParameter(N_CELLS, &nCells, NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting cell count went wrong!\n");
        return lvRetValue;
    }

    // get the first cell voltage
    if(data_getParameter(V_CELL1, &lowestCellVoltage, NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting cell1 voltage went wrong!\n");
        return lvRetValue;
    }

    // cli_printf("Cell1 voltage: %.3f\n", lowestCellVoltage);

    // loop through the other cells
    for(i = 0; i < nCells; i++)
    {
        // get the first cell voltage
        if(data_getParameter((parameterKind_t)(V_CELL1 + i), &cellVoltageOrCapacity, NULL) == NULL)
        {
            cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting cell%d voltage went wrong!\n", i + 1);
            return lvRetValue;
        }

        // compare if it is less than the lowest
        if(cellVoltageOrCapacity < lowestCellVoltage)
        {
            // set the new lowest cell voltage
            lowestCellVoltage = cellVoltageOrCapacity;
        }
    }

    // make the lowest cell voltage a mV value
    lowestCellVoltage = lowestCellVoltage * 1000;

    // get the battery type variable
    if(data_getParameter(BATTERY_TYPE, &batteryType, NULL) == NULL)
    {
        cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting battery type went wrong!\n");
        batteryType = BATTERY_TYPE_DEFAULT;
    }

    // get the state of charge based on the lowest cell voltage
    StateOfCharge = getSoCBasedOnOCV(batteryType, (uint16_t)lowestCellVoltage);

    // check if a-rem needs to be calibrated to calibrate the SoC
    if(calibrateARem)
    {
        // get the full charge capacity
        if(data_getParameter(A_FULL, &cellVoltageOrCapacity, NULL) == NULL)
        {
            cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting a-full went wrong!\n");
            return lvRetValue;
        }

        // calculate the new remaining capacity
        cellVoltageOrCapacity = (cellVoltageOrCapacity / 100) * StateOfCharge;

        // set the new remaining capacity
        // NOTE: the state of charge is calculated with a change of the remaining capacity or the full charge
        // capacity
        if(data_setParameter(A_REM, &cellVoltageOrCapacity))
        {
            cli_printfError(
                "bcc_monitoring_calibrateSoC ERROR: setting new remaining capacity went wrong!\n");
            return lvRetValue;
        }
    }
    else
    {
        cli_printf("Re-calibrating a-full\n");

        // get the remaining capacity
        if(data_getParameter(A_REM, &cellVoltageOrCapacity, NULL) == NULL)
        {
            cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting a-rem went wrong!\n");
            return lvRetValue;
        }

        // calculate the new full capacity based on the SoC
        cellVoltageOrCapacity = (cellVoltageOrCapacity / StateOfCharge) * 100;

        // get the factory capacity
        if(data_getParameter(A_FACTORY, &lowestCellVoltage, NULL) == NULL)
        {
            cli_printfError("bcc_monitoring_calibrateSoC ERROR: getting a-factory went wrong!\n");
            return lvRetValue;
        }

        // check if the new full capacity is larger than the factory capacity
        if(cellVoltageOrCapacity > lowestCellVoltage)
        {
            // limit the full capacity to the factory capacity
            cellVoltageOrCapacity = lowestCellVoltage;

            // set the new remaining capacity
            // NOTE: the state of charge is calculated with a change of the remaining capacity or the full
            // charge capacity
            if(data_setParameter(A_FULL, &cellVoltageOrCapacity))
            {
                cli_printfError("bcc_monitoring_calibrateSoC ERROR: setting new full capacity went wrong!\n");
                return lvRetValue;
            }

            // Correct the remaining capacity
            // calculate the new remaining capacity
            cellVoltageOrCapacity = (cellVoltageOrCapacity / 100) * StateOfCharge;

            // set the new remaining capacity
            // NOTE: the state of charge is calculated with a change of the remaining capacity or the full
            // charge capacity
            if(data_setParameter(A_REM, &cellVoltageOrCapacity))
            {
                cli_printfError(
                    "bcc_monitoring_calibrateSoC ERROR: setting new remaining capacity went wrong!\n");
                return lvRetValue;
            }

            cli_printf("Calculated a-full is > a-factory\nSetting a-full at %.3fAh and a-rem at %.3fAh\n",
                lowestCellVoltage, cellVoltageOrCapacity);
        }
        // if the full charge capacity is OK
        else
        {
            // set the new remaining capacity
            // NOTE: the state of charge is calculated with a change of the remaining capacity or the full
            // charge capacity
            if(data_setParameter(A_FULL, &cellVoltageOrCapacity))
            {
                cli_printfError("bcc_monitoring_calibrateSoC ERROR: setting new full capacity went wrong!\n");
                return lvRetValue;
            }
        }
    }

    // set the returnvalue to 0
    lvRetValue = 0;

    return lvRetValue;
}

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
    int16_t left = 0;                   /* Pointer (index) to the left border of interval
                                           (NTC table). */
    int16_t right = NTC_TABLE_SIZE - 1; /* Pointer (index) to the right border
                                           of interval (NTC table). */
    int16_t middle;                     /* Pointer (index) to the middle of interval
                                           (NTC table). */
    int8_t degTenths;                   /* Fractional part of temperature value. */

    BCC_MCU_Assert(temp != NULL);

    /* Check range of NTC table. */
    if(g_ntcTable[NTC_TABLE_SIZE - 1] > regVal)
    {
        *temp = NTC_COMP_TEMP(NTC_TABLE_SIZE - 1, 0);
        return BCC_STATUS_PARAM_RANGE;
    }
    if(g_ntcTable[0] < regVal)
    {
        *temp = NTC_COMP_TEMP(0, 0);
        return BCC_STATUS_PARAM_RANGE;
    }

    regVal &= BCC_GET_MEAS_RAW(regVal);

    /* Search for an array item which is close to the register value provided
     * by user (regVal). Used method is binary search in sorted array. */
    while((left + 1) != right)
    {
        /* Split interval into halves. */
        middle = (left + right) >> 1U;
        if(g_ntcTable[middle] <= regVal)
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
    degTenths = (g_ntcTable[left] - regVal) / ((g_ntcTable[left] - g_ntcTable[left + 1]) / 10);

    (*temp) = NTC_COMP_TEMP(left, degTenths);

    return BCC_STATUS_SUCCESS;
}

/*
 * @brief   This function calculates the state of charge (SoC) and state of health (SoH)
 *
 * @param   calcBatteryVariables Pointer to the local calcBatteryVariables_t struct to get the information
 *          from and set the new s-charge in.
 *
 * @return  0 if succesfull, otherwise it will indicate the error.
 */
static int calculateSoCAndSoH(calcBatteryVariables_t* calcBatteryVariables)
{
    // Check if the calcBatteryVariables are given
    // check for NULL pointer, but only in debug mode
    DEBUGASSERT(calcBatteryVariables != NULL);

    // use the variable of calcBatteryVariables to calculate the s-charge and set it in the struct

    // Check if a-rem is more than a-full
    if(calcBatteryVariables->A_rem > calcBatteryVariables->A_full)
    {
        // check if a-rem is more than a-factory
        if(calcBatteryVariables->A_rem > calcBatteryVariables->A_factory)
        {
            // correct a-rem with a-factory
            calcBatteryVariables->A_rem = calcBatteryVariables->A_factory;
        }

        // correct a-full
        calcBatteryVariables->A_full = calcBatteryVariables->A_rem;

        // set the s-charge to 100%
        calcBatteryVariables->s_charge = 100;
    }
    else
    {
        // calculate the new s-charge
        calcBatteryVariables->s_charge =
            ((uint8_t)((calcBatteryVariables->A_rem / calcBatteryVariables->A_full) * 100)) & UINT8_MAX;
    }

    // calculate the new s-health
    calcBatteryVariables->s_health =
        ((uint8_t)((calcBatteryVariables->A_full / calcBatteryVariables->A_factory) * 100)) & UINT8_MAX;

    // return OK
    return OK;
}

/*
 * @brief   This function can be used to get the state of charge (SoC) from the open cell voltage
 * @note    A predefined table and the lowest cell voltage will be used for this
 * @warning The battery (voltage) needs to be relaxed before this is used!
 *
 * @param   batteryType The battery-type (BATTERY_TYPE)
 * @param   CurrentCheck if true the current check will be done,
 *          Keep this default on true, except for a power-up check
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
static uint8_t getSoCBasedOnOCV(uint8_t batteryType, uint16_t lowestCellmV)
{
    int     i;
    uint8_t StateOfCharge;

    // find the initial state of charge
    for(i = 0; i < ((cellmvVsSOCLookupTableAllSize[batteryType]) - 1); i++)
    {
        // check if the lowest cell voltage is higher than the next voltage
        if((lowestCellmV) > cellmvVsSOCLookupTableAll[batteryType][i + 1].milliVolt)
        {
            // set the state of charge using linearly interpolation
            StateOfCharge = (cellmvVsSOCLookupTableAll[batteryType][i + 1]).SoC +
                (int)round((float)(lowestCellmV - (cellmvVsSOCLookupTableAll[batteryType][i + 1]).milliVolt) *
                    (float)((float)((cellmvVsSOCLookupTableAll[batteryType][i]).SoC -
                                (cellmvVsSOCLookupTableAll[batteryType][i + 1]).SoC) /
                        ((cellmvVsSOCLookupTableAll[batteryType][i]).milliVolt -
                            (cellmvVsSOCLookupTableAll[batteryType][i + 1]).milliVolt)));

            // escape the for loop
            break;
        }
        // check if it couldn't find it
        else if(i == ((cellmvVsSOCLookupTableAllSize[batteryType]) - 2))
        {
            // set to 0
            StateOfCharge = 0;
        }
    }

    // set a minimum and max
    if(StateOfCharge < 1)
    {
        // set to min value
        StateOfCharge = 1;
    }
    else if(StateOfCharge > 100)
    {
        // set to max value
        StateOfCharge = 100;
    }

    return StateOfCharge;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
