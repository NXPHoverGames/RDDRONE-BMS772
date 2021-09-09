/****************************************************************************
 * nxp_bms/BMS_v1/inc/BMS_data_limits.h
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
 **     Filename    : BMS_data_limits.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        BMS data types module.
 **        This module contains all data limits used in shared memory
 **
 ** ###################################################################*/
/*!
 ** @file BMS_data_limits.h
 **
 ** @version 01.00
 **
 ** @brief
 **        BMS_data_limits module. this module contains the limits for the shared memory
 **
 */
#ifndef BMS_DATA_LIMITS_H_
#define BMS_DATA_LIMITS_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

#include <stdio.h>
#include <stdlib.h>
#include <float.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/
// some defined functions to use with the limits:
// to check
#define RANGE_OK(min, value, max)           if(min <= value && value <= max)
#define RANGE_OK_HIGH(value, max)           if(value <= max)
#define RANGE_OK_LOW(min, value)            if(min <= value)

// to limit
#define LIMIT_VALUE(value, min, max)        (value > max ? max : (value < min ? min : value));

// limitis by the type
#ifndef UINT7_MAX
#define UINT7_MAX                           127
#endif
#ifndef UINT8_MAX
#define UINT8_MAX                           255
#endif
#ifndef UINT11_MAX
#define UINT11_MAX                          2047
#endif  
#ifndef UINT16_MAX
#define UINT16_MAX                          65535
#endif
#ifndef INT16_MAX
#define INT16_MAX                           65535
#endif
#ifndef FLOAT16_MAX
#define FLOAT16_MAX                         65504
#endif
#ifndef INT32_MAX
#define INT32_MAX                           2147483647
#endif
#ifndef UNT64_MAX
#define UNT64_MAX                           18446744073709551615 
#endif

#ifndef MAX_NSEC
#define MAX_NSEC                            999999999
#endif

// minimum and maximum values
// WARNING max value is INT32_MAX to limit on otherwise don't check for max!
#define C_BATT_MAX                          100                 //!< [C]
#define C_BATT_MIN                          (-40)               //!< [C] 
#define V_OUT_MAX                           30                  //!< [V]
#define V_OUT_MIN                           0                   //!< [V] 
#define V_BATT_MAX                          V_OUT_MAX           //!< [V] 
#define V_BATT_MIN                          V_OUT_MIN           //!< [V] 
#define I_BATT_MAX                          200                 //!< [A]
#define I_BATT_MIN                          (-200)              //!< [A]   
#define I_BATT_AVG_MAX                      I_BATT_MAX          //!< [A]
#define I_BATT_AVG_MIN                      I_BATT_MIN          //!< [A]   
#define I_BATT_10S_AVG_MAX                  I_BATT_MAX          //!< [A]
#define I_BATT_10S_AVG_MIN                  I_BATT_MIN          //!< [A]   
#define S_OUT_MAX                           1                   //!< [-]
#define S_OUT_MIN                           0                   //!< [-]
#define S_IN_FLIGHT_MAX                     1                   //!< [-]
#define S_IN_FLIGHT_MIN                     0                   //!< [-]
#define P_AVG_MAX                           3780                //!< [W]
#define P_AVG_MIN                           (-3780)             //!< [W]    
#define E_USED_MAX                          500                 //!< [Wh]
#define E_USED_MIN                          (-500)              //!< [Wh]    
#define A_REM_MAX                           1000                //!< [Ah] // was FLOAT16_MAX
#define A_REM_MIN                           0                   //!< [Ah]
#define A_FULL_MAX                          A_REM_MAX           //!< [Ah] // was FLOAT16_MAX
#define A_FULL_MIN                          0.00001             //!< [Ah]  
#define T_FULL_MAX                          INT16_MAX           //!< [h]  // was FLOAT16_MAX
#define T_FULL_MIN                          0                   //!< [h]   
#define S_FLAGS_MAX                         UINT8_MAX           //!< [-]
#define S_FLAGS_MIN                         0                   //!< [-]    
#define S_HEALTH_MAX                        UINT7_MAX           //!< [%]
#define S_HEALTH_MIN                        0                   //!< [%]   
#define S_CHARGE_MAX                        100                 //!< [%]
#define S_CHARGE_MIN                        0                   //!< [%]   
//#define   S_CHARGE_STDEV_MAX              100                 //!< [%]
//#define   S_CHARGE_STDEV_MIN              0                   //!< [%]  
#define BATT_ID_MAX                         UINT8_MAX           //!< [-]
#define BATT_ID_MIN                         0                   //!< [-]
#define MODEL_ID_MAX                        INT32_MAX//UNT64_MAX            //!< [-]  // was INT32_MAX
#define MODEL_ID_MIN                        0                   //!< [-]
#define MODEL_NAME_MAX                      0                   //!< [-]  // for the generic macro
#define MODEL_NAME_MIN                      0                   //!< [-]  // for the generic macro
#define MODEL_NAME_MAX_CHARS                32                  //!< [-]
                    
#define V_CELL1_MAX                         5                   //!< [V]
#define V_CELL1_MIN                         0                   //!< [V] 
#define V_CELL2_MAX                         5                   //!< [V]
#define V_CELL2_MIN                         0                   //!< [V] 
#define V_CELL3_MAX                         5                   //!< [V]
#define V_CELL3_MIN                         0                   //!< [V] 
#define V_CELL4_MAX                         5                   //!< [V]
#define V_CELL4_MIN                         0                   //!< [V] 
#define V_CELL5_MAX                         5                   //!< [V]
#define V_CELL5_MIN                         0                   //!< [V] 
#define V_CELL6_MAX                         5                   //!< [V]
#define V_CELL6_MIN                         0                   //!< [V] 
#define C_AFE_MAX                           100                 //!< [C]
#define C_AFE_MIN                           (-40)               //!< [C] 
#define C_T_MAX                             100                 //!< [C]
#define C_T_MIN                             (-40)               //!< [C] 
#define C_R_MAX                             100                 //!< [C]
#define C_R_MIN                             (-40)               //!< [C] 
#define N_CHARGES_MAX                       UINT16_MAX          //!< [-]
#define N_CHARGES_MIN                       0                   //!< [-] 
#define N_CHARGES_FULL_MAX                  UINT16_MAX          //!< [-]
#define N_CHARGES_FULL_MIN                  0                   //!< [-] 

#define N_CELLS_MAX                         6                   //!< [-]
#define N_CELLS_MIN                         3                   //!< [-]
#define T_MEAS_MAX                          10000               //!< [ms]
#define T_MEAS_MIN                          100                 //!< [ms]
#define T_FTTI_MAX                          UINT16_MAX          //!< [ms]
#define T_FTTI_MIN                          1                   //!< [ms]
#define T_CYCLIC_MAX                        UINT8_MAX           //!< [s] 
#define T_CYCLIC_MIN                        1                   //!< [s] 
#define I_SLEEP_OC_MAX                      UINT8_MAX           //!< [mA]
#define I_SLEEP_OC_MIN                      1                   //!< [mA]
#define V_CELL_OV_MAX                       4.9725              //!< [V] 
#define V_CELL_OV_MIN                       2 // TODO 3                 //!< [V] 
#define V_CELL_UV_MAX                       4                   //!< [V] 
#define V_CELL_UV_MIN                       0                   //!< [V] 
#define V_CELL_NOMINAL_MAX                  V_CELL_OV_MAX       //!< [V] 
#define V_CELL_NOMINAL_MIN                  V_CELL_UV_MIN       //!< [V] 
#define C_CELL_OT_MAX                       100                 //!< [C] 
#define C_CELL_OT_MIN                       0                   //!< [C] 
#define C_CELL_OT_CHARGE_MAX                100                 //!< [C] 
#define C_CELL_OT_CHARGE_MIN                20                  //!< [C] 
#define C_CELL_UT_MAX                       20                  //!< [C] 
#define C_CELL_UT_MIN                       (-40)               //!< [C] 
#define C_CELL_UT_CHARGE_MAX                20                  //!< [C] 
#define C_CELL_UT_CHARGE_MIN                (-40)               //!< [C] 
#define A_FACTORY_MAX                       A_REM_MAX           //!< [Ah]
#define A_FACTORY_MIN                       0.00001             //!< [Ah]
#define T_BMS_TIMEOUT_MAX                   UINT16_MAX          //!< [s] 
#define T_BMS_TIMEOUT_MIN                   1                   //!< [s]        
#define T_FAULT_TIMEOUT_MAX                 UINT16_MAX          //!< [s] 
#define T_FAULT_TIMEOUT_MIN                 0                   //!< [s] 
#define T_SLEEP_TIMEOUT_MAX                 UINT8_MAX           //!< [h]
#define T_SLEEP_TIMEOUT_MIN                 0                   //!< [h]
#define T_CHARGE_DETECT_MAX                 UINT8_MAX           //!< [s] 
#define T_CHARGE_DETECT_MIN                 1                   //!< [s] 
#define T_CB_DELAY_MAX                      UINT8_MAX           //!< [s] 
#define T_CB_DELAY_MIN                      1                   //!< [s] 
#define T_CHARGE_RELAX_MAX                  UINT16_MAX          //!< [s] 
#define T_CHARGE_RELAX_MIN                  1                   //!< [s] 
#define I_CHARGE_FULL_MAX                   UINT16_MAX          //!< [mA]
#define I_CHARGE_FULL_MIN                   1                   //!< [mA]
#define I_SYSTEM_MAX                        UINT8_MAX           //!< [mA]
#define I_SYSTEM_MIN                        0                   //!< [mA]
#define I_CHARGE_MAX_MAX                    100                 //!< [A]
#define I_CHARGE_MAX_MIN                    0                   //!< [A]
#define I_CHARGE_NOMINAL_MAX                I_CHARGE_MAX_MAX    //!< [A]                    
#define I_CHARGE_NOMINAL_MIN                0                   //!< [A]                    
#define I_OUT_MAX_MAX                       300                 //!< [A]
#define I_OUT_MAX_MIN                       0                   //!< [A]
#define I_PEAK_MAX_MAX                      300                 //!< [A]
#define I_PEAK_MAX_MIN                      0                   //!< [A]
#define I_OUT_NOMINAL_MAX                   I_OUT_MAX_MAX       //!< [A]                    
#define I_OUT_NOMINAL_MIN                   0                   //!< [A]                    
#define I_FLIGHT_MODE_MAX                   UINT8_MAX           //!< [A]
#define I_FLIGHT_MODE_MIN                   0                   //!< [A]
#define V_CELL_MARGIN_MAX                   UINT8_MAX           //!< [mV]
#define V_CELL_MARGIN_MIN                   1                   //!< [mV]
#define V_RECHARGE_MARGIN_MAX               500                 //!< [mV]
#define V_RECHARGE_MARGIN_MIN               10                  //!< [mV]
#define T_OCV_CYCLIC0_MAX                   INT32_MAX           //!< [s] //was INT32_MAX
#define T_OCV_CYCLIC0_MIN                   1                   //!< [s] 
#define T_OCV_CYCLIC1_MAX                   INT32_MAX           //!< [s] // was INT32_MAX
#define T_OCV_CYCLIC1_MIN                   1                   //!< [s] 
#define C_PCB_UT_MAX                        30 // TODO 20                   //!< [C]
#define C_PCB_UT_MIN                        (-40)               //!< [C] 
#define C_PCB_OT_MAX                        100                 //!< [C] 
#define C_PCB_OT_MIN                        20                  //!< [C] 
#define V_STORAGE_MAX                       5                   //!< [V]
#define V_STORAGE_MIN                       0                   //!< [V]
#define OCV_SLOPE_MAX                       100                 //!< [mV/A.min]
#define OCV_SLOPE_MIN                       0.1                 //!< [mV/A.min]
#define BATT_EOL_MAX                        100                 //!< [%]
#define BATT_EOL_MIN                        0                   //!< [%]
#define BATTERY_TYPE_MAX                    2                   //!< [-]
#define BATTERY_TYPE_MIN                    0                   //!< [-]
#define SENSOR_ENABLE_MAX                   1                   //!< [-]
#define SENSOR_ENABLE_MIN                   0                   //!< [-]
#define SELF_DISCHARGE_ENABLE_MAX           1                   //!< [-]
#define SELF_DISCHARGE_ENABLE_MIN           0                   //!< [-]
#define FLIGHT_MODE_ENABLE_MAX              1                   //!< [-]
#define FLIGHT_MODE_ENABLE_MIN              0                   //!< [-]
#define EMERGENCY_BUTTON_ENABLE_MAX         1                   //!< [-]
#define EMERGENCY_BUTTON_ENABLE_MIN         0                   //!< [-]
#define SMBUS_ENABLE_MAX                    1                   //!< [-]
#define SMBUS_ENABLE_MIN                    0                   //!< [-]
#define UAVCAN_NODE_STATIC_ID_MAX           255                 //!< [-]
#define UAVCAN_NODE_STATIC_ID_MIN           0                   //!< [-]
#define UAVCAN_ES_SUB_ID_MAX                UINT16_MAX          //!< [-]
#define UAVCAN_ES_SUB_ID_MIN                0                   //!< [-]
#define UAVCAN_BS_SUB_ID_MAX                UINT16_MAX          //!< [-]
#define UAVCAN_BS_SUB_ID_MIN                0                   //!< [-]
#define UAVCAN_BP_SUB_ID_MAX                UINT16_MAX          //!< [-]
#define UAVCAN_BP_SUB_ID_MIN                0                   //!< [-]
#define UAVCAN_LEGACY_BI_SUB_ID_MAX         UINT16_MAX          //!< [-]    
#define UAVCAN_LEGACY_BI_SUB_ID_MIN         0                   //!< [-]    
#define UAVCAN_FD_MODE_MAX                  1                   //!< [-]
#define UAVCAN_FD_MODE_MIN                  0                   //!< [-]
#define UAVCAN_BITRATE_MAX                  1000000             //!< [bit/s]
#define UAVCAN_BITRATE_MIN                  50000               //!< [bit/s]
#define UAVCAN_FD_BITRATE_MAX               5000000             //!< [bit/s]
#define UAVCAN_FD_BITRATE_MIN               50000               //!< [bit/s]

#define V_MIN_MAX                           9                   //!< [v]
#define V_MIN_MIN                           3                   //!< [v]
#define V_MAX_MAX                           30                  //!< [v]
#define V_MAX_MIN                           11                  //!< [v]
#define I_RANGE_MAX_MAX                     300                 //!< [A]
#define I_RANGE_MAX_MIN                     1                   //!< [A]
#define I_MAX_MAX                           150                 //!< [A]
#define I_MAX_MIN                           1                   //!< [A] 
#define I_SHORT_MAX                         600                 //!< [A] 
#define I_SHORT_MIN                         500                 //!< [A] 
#define T_SHORT_MAX                         UINT8_MAX           //!< [us]
#define T_SHORT_MIN                         1                   //!< [us]   
#define I_BAL_MAX                           UINT8_MAX           //!< [mA]
#define I_BAL_MIN                           0                   //!< [mA]
#define M_MASS_MAX                          100                 //!< [kg]
#define M_MASS_MIN                          0                   //!< [kg]

/*******************************************************************************
 * Types
 ******************************************************************************/

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BMS_DATA_LIMITS_H_ */
