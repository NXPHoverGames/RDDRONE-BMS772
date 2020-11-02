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
 **     Filename    : bcc_monitoring.h
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
 ** @file bcc_monitoring.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - monitoring functions.
 **         This module contains functions linked to monitoring on BCC6 chip. \n
 ** @note
 ** 		This module was adapted from BCC SW examples by C. van Mierlo.
 */

#ifndef BCC_MONITORING_H_
#define BCC_MONITORING_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

// #include "../define.h"							// Include definitions and external functions
// #include "../ext_global.h"						// Include all global variables

/* Modules */

//#include "bcc_common.h"							// Include BCC common functions

/* Other */

//#include <math.h>

#include "bcc_define.h"
#include "BMS_data_types.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef SEM_OCCUPIED_ERROR
#define SEM_OCCUPIED_ERROR 255
#endif
/* NTC precomputed table configuration. */
/*! @brief Minimal temperature in NTC table.
 *
 * It directly influences size of the NTC table (number of precomputed values).
 * Specifically lower boundary.
 */
#define NTC_MINTEMP           (-40)

/*! @brief Maximal temperature in NTC table.
 *
 * It directly influences size of the NTC table (number of precomputed values).
 * Specifically higher boundary.
 */
#define NTC_MAXTEMP           (120)

/*! if the output voltage is lower than this voltage, the output is defined as off. 
    keep in mind that the output is > 0.5V if no load is attached while the switch is open*/
#define OUTPUT_ON_VOLTAGE   3.0  

#define STACK_VOLTAGE_DIFFERENCE_ERROR 0.4


typedef struct
{
	uint16_t 	milliVolt;
	uint8_t 	SoC;
}mvSoC_t;
/*******************************************************************************
 * API
 ******************************************************************************/
/*
 * @brief 	This function will initialize the semaphore 
 * 			this function should be called before bcc_monitoring_updateMeasurements 
 * 			Developed by C. van Mierlo. \n
 *
 * @param   none
 *
 * @return 	int Error code (of the semaphore)
 */
int bcc_monitoring_initializeSem(void);

/*!
 * @brief This function fills the NTC look up table.
 *
 * NTC look up table is intended for resistance to temperature conversion.
 * An array item contains raw value from a register. Index of the item is
 * temperature value.
 *
 * ArrayItem = (Vcom * Rntc) / (0.00015258789 * (NTC + Rntc))
 * Where:
 *  - ArrayItem is an item value of the table,
 *  - Vcom is maximal voltage (5V),
 *  - NTC is the resistance of NTC thermistor (Ohm),
 *  - 0.00015258789 is resolution of measured voltage in Volts
 *    (V = 152.58789 uV * Register_value),
 *  - Rntc is value of a resistor connected to Vcom (see MC3377x datasheet,
 *    section MC3377x PCB components).
 *
 * Beta formula used to calculate temperature based on NTC resistance:
 *   1 / T = 1 / T0 + (1 / Beta) * ln(Rt / R0)
 * Where:
 *  - R0 is the resistance (Ohm) at temperature T0 (Kelvin),
 *  - Beta is material constant (Kelvin),
 *  - T is temperature corresponding to resistance of the NTC thermistor.
 *
 * Equation for NTC value is given from the Beta formula:
 *   NTC = R0 * exp(Beta * (1/T - 1/T0))
 *
 * @param ntcConfig Pointer to NTC components configuration.
 */
void bcc_monitoring_fillNtcTable(const ntc_config_t* const ntcConfig);


/*
 * @brief   This function reads values measured and provided via SPI
 *          by BCC device (ISENSE, cell voltages, temperatures). \n
 *          it will set the values in the data struct
 *          Developed by M. Musak and adapted by C. van Mierlo. \n
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   rShunt Shunt resistor for ISENSE in [uOhm].  
 * @param   lowestCellVoltageAdr this function will set the lowest cell voltage in this variable
 * @param 	gateMutexAdr this is the address of the gate mutex, to check a pin for the output status
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_updateMeasurements(bcc_drv_config_t* const drvConfig, uint32_t rShunt, 
	float *lowestCellVoltageAdr, pthread_mutex_t *gateMutexAdr);

/*
 * @brief   This function is used to do a meaurement
 * 			This function is blocking and will wait until the measurement is done
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_doBlockingMeasurement(bcc_drv_config_t* const drvConfig);

/*
 * @brief   This function is used to check the output, without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   output the address of the variable to become 1 if the output high and 0 if low.
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_checkOutput(bcc_drv_config_t* const drvConfig, bool *output);

/*
 * @brief   This function is used to check if the number of cells are OK
 * 			it will read the cell voltages and the stack voltage, without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   nCellsOk the address of the variable to become 1 if the cells are OK.
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_checkNCells(bcc_drv_config_t* const drvConfig, bool *nCellsOk);

/*
 * @brief   This function is used to get the output voltage
 * 			it will read the output voltage without reading the other measurements
 * @note    A measurement should be done first
 *
 * @param   drvConfig the address the BCC driver configuration
 *
 * @return  bcc_status_t Error code
 */
bcc_status_t bcc_monitoring_getOutputVoltage(bcc_drv_config_t* const drvConfig);

/*
 * @brief 	This function increases the semaphore so the bcc_monitoring_updateMeasurements 
 * 			function will do the rest of the calculations
 * 			Developed by C. van Mierlo. \n
 *
 * @param   none
 *
 * @return 	int Error code (of the semaphore)
 */
int bcc_monitoring_doAllCalculations(void);

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
void bcc_monitoring_setAverageInterval(uint8_t updateAverageInterval, uint8_t elements);

/*
 * @brief   This function is used to calculate the delta charge with the coulomb counter registers
 *          it will read and therefor reset the CC registers
 * @note    can be called from mulitple threads
 *
 * @param   drvConfig the address the BCC driver configuration
 * @param   samplesAdr address of the variable to become the samples in the CC register
 * @param   avgCurrentAdr address of the variable to become the average current
 * @param   deltaChargeAdr address of the variable to become the delta charge
 * @param 	resetCC if this is true, it will reset the CC registers by writing CC_RST true 
 *
 * @return  0 if succesfull, otherwise it will indicate the error
 */
int bcc_monitoring_calcDCharge(bcc_drv_config_t* const drvConfig, uint16_t *samplesAdr, 
    float *avgCurrentAdr, float *deltaChargeAdr, bool resetCC);

/*
 * @brief   This function is used to enable or disable the display of the measurements
 *
 * @param   showCommand which measurements to enable or disable
 * @param   value the new value of that bit
 *
 * @return  none
 */
void bcc_monitoring_setShowMeas(showCommands_t showCommand, bool value);

static const mvSoC_t cellmvVsSOCLookupTable[] =
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
	//{ 3700, 13},
	//{ 3699, 13},
	{ 3698, 13},
	//{ 3697, 13},
	//{ 3695, 13},
	//{ 3694, 12},
	//{ 3693, 12},
	{ 3692, 12},
	//{ 3691, 12},
	//{ 3690, 12},
	//{ 3690, 11},
	//{ 3689, 11},
	//{ 3689, 11},
	{ 3688, 11},
	//{ 3688, 11},
	//{ 3687, 11},
	//{ 3687, 10},
	//{ 3687, 10},
	//{ 3686, 10},
	{ 3686, 10},
	//{ 3685, 10},
	//{ 3685, 10},
	//{ 3685, 9},
	//{ 3684, 9},
	//{ 3683, 9},
	{ 3683, 9},
	//{ 3682, 9},
	//{ 3682, 9},
	//{ 3682, 8},
	//{ 3681, 8},
	//{ 3680, 8},
	{ 3680, 8},
	//{ 3679, 8},
	//{ 3678, 8},
	//{ 3677, 7},
	//{ 3677, 7},
	//{ 3676, 7},
	{ 3675, 7},
	//{ 3674, 7},
	//{ 3673, 7},
	//{ 3671, 6},
	//{ 3670, 6},
	//{ 3668, 6},
	{ 3666, 6},
	//{ 3663, 6},
	//{ 3660, 6},
	//{ 3657, 5},
	//{ 3652, 5},
	//{ 3647, 5},
	{ 3642, 5},
	//{ 3637, 5},
	//{ 3631, 5},
	//{ 3625, 4},
	//{ 3618, 4},
	//{ 3612, 4},
	{ 3605, 4},
	//{ 3598, 4},
	//{ 3591, 4},
	//{ 3584, 3},
	//{ 3576, 3},
	{ 3568, 3},
	//{ 3560, 3},
	//{ 3551, 3},
	//{ 3542, 3},
	//{ 3533, 2},
	//{ 3523, 2},
	//{ 3514, 2},
	{ 3503, 2},
	//{ 3493, 2},
	//{ 3481, 2},
	//{ 3470, 2},
	//{ 3457, 1},
	//{ 3444, 1},
	//{ 3431, 1},
	{ 3417, 1},
	//{ 3402, 1},
	//{ 3386, 1},
	//{ 3370, 1},
	//{ 3352, 0},
	//{ 3333, 0},
	{ 3313, 0},
	//{ 3300, 0},
};

/*******************************************************************************
 * EOF
 ******************************************************************************/
#endif /* BCC_MONITORING_H_ */
