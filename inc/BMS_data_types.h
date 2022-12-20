/****************************************************************************
 * nxp_bms/BMS_v1/inc/BMS_data_types.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020-2022 NXP
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
 **     Filename    : BMS_data_types.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date        : 2020-03-17
 **     Abstract    :
 **        BMS data types module.
 **        This module contains all data types used in shared memory
 **
 ** ###################################################################*/
/*!
 ** @file BMS_data_types.h
 **
 ** @version 01.00
 **
 ** @brief
 **        BMS_data_types module. this module contains the data types for the shared memory
 **
 */
#ifndef BMS_DATA_TYPES_H_
#define BMS_DATA_TYPES_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>

#include "BMS_data_limits.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/
#define LIPO_V_CELL_OV_DEFAULT              4.2         //!< [V] the default cell overvoltage level of a LiPo battery cell
#define LIPO_V_CELL_UV_DEFAULT              3           //!< [V] the default cell undervoltage level of a LiPo battery cell
#define LIPO_V_STORAGE_DEFAULT              3.8         //!< [V] the default cell storage voltage level of a LiPo battery cell
#define LIPO_V_CELL_NOMINAL_DEFAULT         3.7         //!< [V] the nominal cell voltage of a LiPo battery cell

#define LIFEPO4_V_CELL_OV_DEFAULT           3.6         //!< [V] the default cell overvoltage level of a LiFePo4 battery cell
#define LIFEPO4_V_CELL_UV_DEFAULT           2.5         //!< [V] the default cell undervoltage level of a LiFePo4 battery cell
#define LIFEPO4_V_STORAGE_DEFAULT           3.3         //!< [V] the default cell storage voltage level of a LiFePo4 battery cell
#define LIFEPO4_V_CELL_NOMINAL_DEFAULT      3.3         //!< [V] the nominal cell voltage of a LiFePo4 battery cell

#define LIFEYPO4_V_CELL_OV_DEFAULT          3.6         //!< [V] the default cell overvoltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_CELL_UV_DEFAULT          2.8         //!< [V] the default cell undervoltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_STORAGE_DEFAULT          3.2         //!< [V] the default cell storage voltage level of a LiFeYPo4 battery cell
#define LIFEYPO4_V_CELL_NOMINAL_DEFAULT     3.3         //!< [V] the nominal cell voltage of a LiFeYPo4 battery cell

#define S_HEALTH_UNKNOWN                    127

#define STATUS_ASK_PARS_BIT                 0           //!< There is a change in the extra parameters so these should be asked
#define STATUS_TEMP_ERROR_BIT               1           //!< Battery temperature limit failure, the temperature is either too high or too low
#define STATUS_OVERLOAD_BIT                 2           //!< Safe operating area violation, the controller should look at drawing less current
#define STATUS_BAD_BATTERY_BIT              3           //!< This battery should not be used anymore (e.g. low SoH)
#define STATUS_NEED_SERVICE_BIT             4           //!< This battery requires maintenance (e.g. balancing, full recharge)
#define STATUS_BMS_ERROR_BIT                5           //!< Battery management system/controller error, smart battery interface error
#define STATUS_OPTIONAL1_BIT                6           //!< To be applied to another status
#define STATUS_OPTIONAL2_BIT                7           //!< To be applied to another status
#define STATUS_HIGHEST_BIT                  STATUS_OPTIONAL2_BIT //!< the highest bit of the status flags

#define S_FLAGS_UKNOWN                      255         //!< When the status is unknown

// default values
#define C_BATT_DEFAULT                      0           //!< [C] 
#define V_OUT_DEFAULT                       0           //!< [V]  
#define V_BATT_DEFAULT                      0           //!< [V] 
#define I_BATT_DEFAULT                      0           //!< [A] 
#define I_BATT_AVG_DEFAULT                  0           //!< [A] 
#define I_BATT_10S_AVG_DEFAULT              0           //!< [A] 
#define S_OUT_DEFAULT                       0           //!< [-]   
#define S_IN_FLIGHT_DEFAULT                 0           //!< [-]   
#define P_AVG_DEFAULT                       0           //!< [W]    
#define E_USED_DEFAULT                      0           //!< [Wh]    
#define A_REM_DEFAULT                       0           //!< [Ah]
#define A_FULL_DEFAULT                      4.6         //!< [Ah]  
#define T_FULL_DEFAULT                      0           //!< [h]   
#define S_FLAGS_DEFAULT                     S_FLAGS_UKNOWN  //!< [-]    
#define S_HEALTH_DEFAULT                    S_HEALTH_UNKNOWN    //!< [%]   
#define S_CHARGE_DEFAULT                    0           //!< [%]   
//#define   S_CHARGE_STDEV_DEFAULT          0           //!< [%]  
#define BATT_ID_DEFAULT                     0           //!< [-]
#define MODEL_ID_DEFAULT                    0           //!< [-]
#define MODEL_NAME_DEFAULT                  "BMS test"  //!< [-] 
                    
#define V_CELL1_DEFAULT                     0           //!< [V] 
#define V_CELL2_DEFAULT                     0           //!< [V] 
#define V_CELL3_DEFAULT                     0           //!< [V] 
#define V_CELL4_DEFAULT                     0           //!< [V] 
#define V_CELL5_DEFAULT                     0           //!< [V] 
#define V_CELL6_DEFAULT                     0           //!< [V] 
#define C_AFE_DEFAULT                       0           //!< [C] 
#define C_T_DEFAULT                         0           //!< [C] 
#define C_R_DEFAULT                         0           //!< [C] 
#define N_CHARGES_DEFAULT                   0           //!< [-]  
#define N_CHARGES_FULL_DEFAULT              0           //!< [-] 

#define N_CELLS_DEFAULT                     3           //!< [-]
#define T_MEAS_DEFAULT                      1000        //!< [ms]
#define T_FTTI_DEFAULT                      1000        //!< [ms]
#define T_CYCLIC_DEFAULT                    1           //!< [s] 
#define I_SLEEP_OC_DEFAULT                  30          //!< [mA]
#define V_CELL_OV_DEFAULT                   LIPO_V_CELL_OV_DEFAULT //!< [V] 
#define V_CELL_UV_DEFAULT                   LIPO_V_CELL_UV_DEFAULT //!< [V] 
#define V_CELL_NOMINAL_DEFAULT              LIPO_V_CELL_NOMINAL_DEFAULT //!<[V]
#define C_CELL_OT_DEFAULT                   45          //!< [C] 
#define C_CELL_OT_CHARGE_DEFAULT            40          //!< [C] 
#define C_CELL_UT_DEFAULT                   (-20)       //!< [C] 
#define C_CELL_UT_CHARGE_DEFAULT            0           //!< [C]
#define A_FACTORY_DEFAULT                   A_FULL_DEFAULT  //!< [Ah] 
#define T_BMS_TIMEOUT_DEFAULT               600         //!< [s]        
#define T_FAULT_TIMEOUT_DEFAULT             60          //!< [s] 
#define T_SLEEP_TIMEOUT_DEFAULT             12          //!< [h]
#define T_CHARGE_DETECT_DEFAULT             1           //!< [s] 
#define T_CB_DELAY_DEFAULT                  120         //!< [s] 
#define T_CHARGE_RELAX_DEFAULT              300         //!< [s] 
#define I_CHARGE_FULL_DEFAULT               50          //!< [mA]
#define I_SYSTEM_DEFAULT                    40          //!< [mA]
#define I_CHARGE_MAX_DEFAULT                4.6         //!< [A]
#define I_CHARGE_NOMINAL_DEFAULT            I_CHARGE_MAX_DEFAULT //!< [A]
#define I_OUT_MAX_DEFAULT                   60          //!< [A]
#define I_PEAK_MAX_DEFAULT                  200         //!< [A]
#define I_OUT_NOMINAL_DEFAULT               I_OUT_MAX_DEFAULT //!< [A]
#define I_FLIGHT_MODE_DEFAULT               5           //!< [A]
#define V_CELL_MARGIN_DEFAULT               50          //!< [mV]
#define V_RECHARGE_MARGIN_DEFAULT           200         //!< [mV]
#define T_OCV_CYCLIC0_DEFAULT               300         //!< [s] 
#define T_OCV_CYCLIC1_DEFAULT               86400       //!< [s] 
#define C_PCB_UT_DEFAULT                    (-20)       //!< [C] 
#define C_PCB_OT_DEFAULT                    45          //!< [C] 
#define V_STORAGE_DEFAULT                   LIPO_V_STORAGE_DEFAULT //!< [V]
#define OCV_SLOPE_DEFAULT                   5.3         //!< [mV/A.min]
#define BATT_EOL_DEFAULT                    80          //!< [%]
#define BATTERY_TYPE_DEFAULT                0           //!< [-]
#define SENSOR_ENABLE_DEFAULT               0           //!< [-]
#define SELF_DISCHARGE_ENABLE_DEFAULT       1           //!< [-]
#define FLIGHT_MODE_ENABLE_DEFAULT          0           //!< [-]
#define EMERGENCY_BUTTON_ENABLE_DEFAULT     0           //!< [-]
#define SMBUS_ENABLE_DEFAULT                0           //!< [-]
#define UAVCAN_NODE_STATIC_ID_DEFAULT       255         //!< [-] 
#define UAVCAN_ES_SUB_ID_DEFAULT            4096        //!< [-] 
#define UAVCAN_BS_SUB_ID_DEFAULT            4097        //!< [-] 
#define UAVCAN_BP_SUB_ID_DEFAULT            4098        //!< [-] 
#define UAVCAN_LEGACY_BI_SUB_ID_DEFAULT     UINT16_MAX  //!< [-]
#define UAVCAN_FD_MODE_DEFAULT              0           //!< [-] 
#define UAVCAN_BITRATE_DEFAULT              1000000     //!< [bit/s]
#define UAVCAN_FD_BITRATE_DEFAULT           4000000     //!< [bit/s]

#define V_MIN_DEFAULT                       6           //!< [v]
#define V_MAX_DEFAULT                       26          //!< [v]
#define I_RANGE_MAX_DEFAULT                 300         //!< [A]
#define I_MAX_DEFAULT                       60          //!< [A] 
#define I_SHORT_DEFAULT                     500         //!< [A] 
#define T_SHORT_DEFAULT                     20          //!< [uA]   
#define I_BAL_DEFAULT                       50          //!< [mA]
#define M_MASS_DEFAULT                      0           //!< [kg]

/*! @brief function to generate an enum from FOR_EACH_.. */
#define GENERATE_ENUM(ENUM)     ENUM,       

/*! @brief function to generate a string array from FOR_EACH_.. */
#define GENERATE_STRING(STRING) #STRING,    
/*******************************************************************************
 * Types
 ******************************************************************************/

/*! @brief  define the values for the state enum 
*           this can be used to create an enum and a string array for these values */
#define FOR_EACH_STATE(STATE)       \
        STATE(SELF_TEST)            \
        STATE(INIT)                 \
        STATE(NORMAL)               \
        STATE(CHARGE)               \
        STATE(SLEEP)                \
        STATE(OCV)                  \
        STATE(FAULT)                \
        STATE(SELF_DISCHARGE)       \
        STATE(DEEP_SLEEP)           

/*! @brief define the state enum */
//typedef enum{INIT, NORMAL, CHARGE, SLEEP, OCV, FAULT, SELF_DISCHARGE, DEEP_SLEEP}states_t;
typedef enum{
    FOR_EACH_STATE(GENERATE_ENUM)
}states_t;


/*! @brief define the values for the charge state enum 
 * this can be used to create an enum and a string array for these values */
#define FOR_EACH_CHARGE_STATE(STATE)    \
                                        \
        STATE(CHARGE_START)             \
        STATE(CHARGE_CB)                \
        STATE(RELAXATION)               \
        STATE(CHARGE_COMPLETE)                  

/*! @brief define the charge state enum */
typedef enum{
    FOR_EACH_CHARGE_STATE(GENERATE_ENUM)
}charge_states_t;

/*! @brief  this enum consists of each variable name, can be used to get or set variables
 *          if this enum changes, it will automatically change the cli.c gGetSetParameters string array
 *          this define will be used to generate an enum (to get the data) and string values of that enum (for the cli)
 *          this should consist of each of the BMSParameterValues_t value names in capitals
 * @warning if this enum changes, change the s_parameters struct and the data_setDefaultParameters function in data.c
 *          and add the default, min and max value, add it to the parametersTypes array in cli.c and parameterUnits array in data.c!
 */
#define FOR_EACH_PARAMETER(PARAMETER)       \
        PARAMETER(C_BATT)                   /*0*/\
        PARAMETER(V_OUT)                    \
        PARAMETER(V_BATT)                   \
        PARAMETER(I_BATT)                   \
        PARAMETER(I_BATT_AVG)               \
        PARAMETER(I_BATT_10S_AVG)           \
        PARAMETER(S_OUT)                    \
        PARAMETER(S_IN_FLIGHT)              \
        PARAMETER(P_AVG)                    \
        PARAMETER(E_USED)                   \
        PARAMETER(A_REM)                    /*10*/\
        PARAMETER(A_FULL)                   \
        PARAMETER(T_FULL)                   \
        PARAMETER(S_FLAGS)                  \
        PARAMETER(S_HEALTH)                 \
        PARAMETER(S_CHARGE)                 \
        /*PARAMETER(S_CHARGE_STDEV)*/       \
        PARAMETER(BATT_ID)                  \
        PARAMETER(MODEL_ID)                 \
        PARAMETER(MODEL_NAME)               \
                                            \
        PARAMETER(V_CELL1)                  \
        PARAMETER(V_CELL2)                  /*20*/\
        PARAMETER(V_CELL3)                  \
        PARAMETER(V_CELL4)                  \
        PARAMETER(V_CELL5)                  \
        PARAMETER(V_CELL6)                  \
        PARAMETER(C_AFE)                    \
        PARAMETER(C_T)                      \
        PARAMETER(C_R)                      \
        PARAMETER(N_CHARGES)                \
        PARAMETER(N_CHARGES_FULL)           \
                                            \
        PARAMETER(N_CELLS)                  /*30*/\
        PARAMETER(T_MEAS)                   \
        PARAMETER(T_FTTI)                   \
        PARAMETER(T_CYCLIC)                 \
        PARAMETER(I_SLEEP_OC)               \
        PARAMETER(V_CELL_OV)                \
        PARAMETER(V_CELL_UV)                \
        PARAMETER(V_CELL_NOMINAL)           \
        PARAMETER(C_CELL_OT)                \
        PARAMETER(C_CELL_OT_CHARGE)         \
        PARAMETER(C_CELL_UT)                /*40*/\
        PARAMETER(C_CELL_UT_CHARGE)         \
        PARAMETER(A_FACTORY)                \
        PARAMETER(T_BMS_TIMEOUT)            \
        PARAMETER(T_FAULT_TIMEOUT)          \
        PARAMETER(T_SLEEP_TIMEOUT)          \
        PARAMETER(T_CHARGE_DETECT)          \
        PARAMETER(T_CB_DELAY)               \
        PARAMETER(T_CHARGE_RELAX)           \
        PARAMETER(I_CHARGE_FULL)            \
        PARAMETER(I_SYSTEM)                 /*50*/\
        PARAMETER(I_CHARGE_MAX)             \
        PARAMETER(I_CHARGE_NOMINAL)         \
        PARAMETER(I_OUT_MAX)                \
        PARAMETER(I_PEAK_MAX)               \
        PARAMETER(I_OUT_NOMINAL)            \
        PARAMETER(I_FLIGHT_MODE)            \
        PARAMETER(V_CELL_MARGIN)            \
        PARAMETER(V_RECHARGE_MARGIN)        \
        PARAMETER(T_OCV_CYCLIC0)            \
        PARAMETER(T_OCV_CYCLIC1)            /*60*/\
        PARAMETER(C_PCB_UT)                 \
        PARAMETER(C_PCB_OT)                 \
        PARAMETER(V_STORAGE)                \
        PARAMETER(OCV_SLOPE)                \
        PARAMETER(BATT_EOL)                 \
        PARAMETER(BATTERY_TYPE)             \
        PARAMETER(SENSOR_ENABLE)            \
        PARAMETER(SELF_DISCHARGE_ENABLE)    \
        PARAMETER(FLIGHT_MODE_ENABLE)       \
        PARAMETER(EMERGENCY_BUTTON_ENABLE)  /*70*/\
        PARAMETER(SMBUS_ENABLE)             \
        PARAMETER(UAVCAN_NODE_STATIC_ID)    \
        PARAMETER(UAVCAN_ES_SUB_ID)         \
        PARAMETER(UAVCAN_BS_SUB_ID)         \
        PARAMETER(UAVCAN_BP_SUB_ID)         \
        PARAMETER(UAVCAN_LEGACY_BI_SUB_ID)  \
        PARAMETER(UAVCAN_FD_MODE)           \
        PARAMETER(UAVCAN_BITRATE)           \
        PARAMETER(UAVCAN_FD_BITRATE)        \
                                            \
        PARAMETER(V_MIN)                    /*80*/\
        PARAMETER(V_MAX)                    \
        PARAMETER(I_RANGE_MAX)              \
        PARAMETER(I_MAX)                    \
        PARAMETER(I_SHORT)                  \
        PARAMETER(T_SHORT)                  \
        PARAMETER(I_BAL)                    \
        PARAMETER(M_MASS)                   
        //PARAMETER(NONE)                   /*88*/  /* needs to be last! */

/*! @brief  generate the enum fromt he defined parameters, this should include all the parameters in BMSParameterValues_t in capitals 
 *          this enum consists of each variable name, can be used to get or set variables
 *          if this enum changes, it will automatically change the cli.c gGetSetParameters string array
 *          this define will be used to generate an enum (to get the data) and string values of that enum (for the cli)
 *          this should consist of each of the BMSParameterValues_t value names in capitals
 * @warning if this enum changes, change the s_parameters struct and the data_setDefaultParameters function in data.c
 *          and add the default, min and max value, add it to the parametersTypes array in cli.c and parameterUnits array in data.c!
 */
typedef enum
{
    FOR_EACH_PARAMETER(GENERATE_ENUM)
    NONE            
}parameterKind_t;

// TODO make output_status with a bool, but this needs to be made in data.c as well

/*! @brief this struct consists of the variables needed in the UAVCAN protol 
 *          keep in mind that the float needs to be a float16   
 *          the uint8 of the SoH, SoC and SoC std needs to be a uint7
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!! 
 */
typedef struct
{
    float               C_batt;                             //!< float16!   [C] the temperature of the external battey temperature sensor 
    float               V_out;                              //!< float16!   [V] the voltage of the BMS output
    float               V_batt;                             //!< float16!   [V] the voltage of the battery pack
    float               I_batt;                             //!< float16!   [A] the last recorded current of the battery
    float               I_batt_avg;                         //              [A] the average current since the last measurement (period T_meas (default 1s))
    float               I_batt_10s_avg;                     //              [A] the 10s rollling average current, updated each 1s with T_meas 1000 (ms)
    uint8_t             s_out;                              //!< bool!      [-] this is true if the output power is enabled
    uint8_t             s_in_flight;                        //!< bool!      [-] this is true if the system thinks it is in flight (with flight-mode-enable and i-flight-mode)
    float               P_avg;                              //!< float16!   [W] average power consumption over the last 10 seconds
    float               E_used;                             //!< float16!   [Wh] power consumption since device boot
    float               A_rem;                              //!< float16!   [Ah] remaining capacity in the battery
    float               A_full;                             //!< float16!   [Ah] predicted battery capacity when it is fully charged. falls with aging, full charge capacity
    float               t_full;                             //!< float16!   [h] charging is expected to complete in this time; zero if not charging
    uint8_t             s_flags;                            //!< uint11!    [-] this contains the status flags as discribed in BMS_status_flags_t
    uint8_t             s_health;                           //!< uint7!     [%] state of health, health of the battery in percentage use S_HEALTH_UNKNOWN = 127 if cannot be estimated
    uint8_t             s_charge;                           //!< uint7!     [%] state of charge, precent of hte full charge [0, 100]. this field is required.
    /*uint8_t               state_of_charge_stdev;              // < uint7!     [%] SOC error standard deviation, use best guess if unkown*/
    uint8_t             batt_id;                            //!<            [-] identifies the battery within this vehicle, e.g. 0 - primariy battery.
    uint64_t            model_id;                           //!<            [-] set to 0 if not applicable
    char                model_name[MODEL_NAME_MAX_CHARS];   //!<            [-] battery model name, model name is a human-radable string that normally should include the vendor name, model name and chemistry
}BMSBasicVariables_t;

/*! @brief this struct consists of the additional variables
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct 
{
    float               V_cell1;                            //!< [V] the voltage of cell 1
    float               V_cell2;                            //!< [V] the voltage of cell 2
    float               V_cell3;                            //!< [V] the voltage of cell 3
    float               V_cell4;                            //!< [V] the voltage of cell 4
    float               V_cell5;                            //!< [V] the voltage of cell 5
    float               V_cell6;                            //!< [V] the voltage of cell 6
    float               C_AFE;                              //!< [C] the temperature of the analog front end
    float               C_T;                                //!< [C] the temperature of the transitor
    float               C_R;                                //!< [C] the temperature of the sense resistor
    uint16_t            N_charges;                          //!< [-] the number of charges done 
    uint16_t            N_charges_full;                     //!< [-] the number of complete charges
}BMSAdditionalVariables_t;

/*! @brief  this struct consists of the configuration parameters
 * @warning if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct 
{
    uint8_t             N_cells;                            //!< [-] number of cells used in the BMS board
    uint16_t            t_meas;                             //!< [ms] cycle of the battery to perform a complete battery measurement and SOC estimation can only be 10000 or a whole division of 10000 (For example: 5000, 1000, 500)
    uint16_t            t_ftti;                             //!< [ms] cycle of the battery to perform diagnostics (Fault Tolerant Time Interval) 
    uint8_t             t_cyclic;                           //!< [s] wake up cyclic timing of the AFE (after front end) during sleep mode 
    uint8_t             I_sleep_oc;                         //!< [mA] overcurrent threshold detection in sleep mode that will wake up the battery and also the threshold to detect the battery is not in use
    float               V_cell_ov;                          //!< [V] battery maximum allowed voltage for one cell. exceeding this voltage, the battery will go to fault mode.
    float               V_cell_uv;                          //!< [V] Battery minimum allowed voltage for one cell. Going below this voltage, the BMS will go to fault (maybe deep_sleep) mode
    float               V_cell_nominal;                     //!< [V] Battery nominal voltage for one cell. will be used for energy calculation.
    float               C_cell_ot;                          //!< [C] Overtemperature threshold for the Cells during discharging. Going over this threshold and the battery will go to FAULT mode 
    float               C_cell_ot_charge;                   //!< [C] Overtemperature threshold for the Cells during charging. Going over this threshold and the battery will go to FAULT mode 
    float               C_cell_ut;                          //!< [C] Under temperature threshold for the Cells. Going under this threshold and the battery will go to FAULT mode 
    float               C_cell_ut_charge;                   //!< [C] Under temperature threshold for the Cells during charging. Going under this threshold during charging and the battery will go to FAULT mode 
    float               A_factory;                          //!< [Ah] battery capacity stated by the factory
    uint16_t            t_bms_timeout;                      //!< [s] Timeout for the BMS to go to SLEEP mode when the battery is not used. 
    uint16_t            t_fault_timeout;                    //!< [s] After this timeout, with an undervoltage fault the battery will go to DEEPSLEEP mode to preserve power. 0 sec is disabled.
    uint8_t             t_sleep_timeout;                    //!< [h] When the BMS is in sleep mode for this period it will go to the self discharge mode, 0 if disabled. 
    uint8_t             t_charge_detect;                    //!< [s] During NORMAL mode, is the battery current is positive for more than this time, then the battery will go to CHARGE mode 
    uint8_t             t_cb_delay;                         //!< [s] Time for the cell balancing function to start after entering the CHARGE mode 
    uint16_t            t_charge_relax;                     //!< [s] Relaxation after the charge is complete before going to another charge round. 
    uint16_t            I_charge_full;                      //!< [mA] Current threshold to detect end of charge sequence 
    uint8_t             I_system;                           //!< [mA] Current of the BMS board itself, this is measured (as well) during charging, so this needs to be substracted
    float               I_charge_max;                       //!< [A] Maximum current threshold to open the switch during charging
    float               I_charge_nominal;                   //!< [A] Nominal charge current (informative only) 
    float               I_out_max;                          //!< [A] Maximum average current threshold to open the switch during normal operation, if not overrulled
    float               I_peak_max;                         //!< [A] Maximum peak current threshold to open the switch during normal operation, can't be overrulled
    float               I_out_nominal;                      //!< [A] Nominal discharge current (informative only) 
    uint8_t             I_flight_mode;                      //!< [A] current threshold to not disable the power in flight mode
    uint8_t             V_cell_margin;                      //!< [mV] Cell voltage charge margin to decide or not to go through another topping charge cycle 
    uint16_t            V_recharge_margin;                  //!< [mV] Cell voltage charge complete margin to decide or not to do a battery re-charge, to keep the cell voltages at max this much difference with the cell-ov
    int32_t             t_ocv_cyclic0;                      //!< [s] OCV measurement cyclic timer start (timer is increase by 50% at each cycle)
    int32_t             t_ocv_cyclic1;                      //!< [s] OCV measurement cyclic timer final (timer is increase by 50% at each cycle)
    float               C_pcb_ut;                           //!< [C] PCB Ambient temperature under temperature threshold
    float               C_pcb_ot;                           //!< [C] PCB Ambient temperature over temperature threshold
    float               V_storage;                          //!< [V] The voltage what is specified as storage voltage for a cell
    float               ocv_slope;                          //!< [mV/A.min] The slope of the OCV curve  
    uint8_t             batt_eol;                           //!< [%] perentage at which the battery is a bad battery and shouldn't be used typical between 90%-50% default is 80%
    uint8_t             battery_type;                       //!< [-] The type of battery attached to it. 0 = LiPo, 1 = LiFePo4, 2 = LiFeYPo4. Could be extended. Will change OV, UV, v-storage, OCV/SoC table if changed runtime.
    uint8_t             sensor_enable;                      //!< [-] This variable is used to enable or disable the battery temperature sensor, 0 is disabled
    uint8_t             self_discharge_enable;              //!< [-] This variable is used to enable or disable the SELF_DISCHARGE state, 0 is disabled
    uint8_t             flight_mode_enable;                 //!< [-] This variable is used to enable or disable flight mode, is used together with i-flight-mode
    uint8_t             emergency_button_enable;            //!< [-] This variable is used to enable or disable the emergency button on PTE8
    uint8_t             smbus_enable;                       //!< [-] This variable is used to enable or disable the SMBus update. 
    uint8_t             Uavcan_node_static_id;              //!< [-] This is the node ID of the UAVCAN message 
    uint16_t            Uavcan_es_sub_id;                   //!< [-] This is the subject ID of the energy source UAVCAN message (1...100Hz)
    uint16_t            Uavcan_bs_sub_id;                   //!< [-] This is the subject ID of the battery status UAVCAN message (1Hz)
    uint16_t            Uavcan_bp_sub_id;                   //!< [-] This is the subject ID of the battery parameters UAVCAN message (0.2Hz)
    uint16_t            Uavcan_legacy_bi_sub_id;            //!< [-] This is the subject ID of the battery info legacy UAVCAN message (0.2 ~ 1Hz)
    uint8_t             Uavcan_fd_mode;                     //!< [-] If true CANFD is used, otherwise classic CAN is used, only during startup check!
    int32_t             Uavcan_bitrate;                     //!< [bit/s] the bitrate of classical can or CAN FD arbitratration bitrate
    int32_t             Uavcan_fd_bitrate;                  //!< [bit/s] the bitrate of CAN FD data bitrate
}BMSConfigurationVariables_t;

/*! @brief   this struct contains the hardware parameters (these are dependent by the hardware)
 * @warning  if this changes, change FOR_EACH_PARAMETER aswell!!
 */
typedef struct 
{
    uint8_t             V_min;                              //!< [V] Minimum stack voltage for the BMS board to be fully functional
    uint8_t             V_max;                              //!< [V] Maximum stack voltage allowed by the BMS board
    uint16_t            I_range_max;                        //!< [A] Maximum current that can be measured by the BMS board
    uint8_t             I_max;                              //!< [A] Maximum DC current allowed in the BMS board (limited by power dissipation in the MOSFETs)
    uint16_t            I_short;                            //!< [A] short circuit current threshold (typical: 550A, min: 500A, max: 600A)
    uint8_t             t_short;                            //!< [us] Blanking time for the short circuit detection 
    uint8_t             I_bal;                              //!< [mA] Cell balancing current under 4.2V with cell balancing resistors of 82 ohms
    float               m_mass;                             //!< [kg] The total mass of the (smart) battery
}BMSHardwareVariables_t;

/*! @brief  this struct contains all the variables
 */
typedef struct 
{
    BMSBasicVariables_t             basicVariables;         //!< the basic variables needed in the UAVCAN protol 
    BMSAdditionalVariables_t        additionalVariables;    //!< the additional variables
    BMSConfigurationVariables_t     configurationVariables; //!< the configuration parameters
    BMSHardwareVariables_t          hardwareVariables;      //!< the hardware parameters
}BMSParameterValues_t;

/*! @brief      union to be used with the max and min value
    @warning    for integers, INT32_MAX is the maximum value to check on and INT32_MIN the minimum value
 */
typedef union
{
    uint8_t         U8;                                     //!< the uint8 value
    uint16_t        U16;                                    //!< the uint16 value
    int32_t         I32;                                    //!< the int32 value    
    //int16_t       FX10;                                   //!< the float x 10  and int16 value
    //int64_t       I64F;                                   //!< the 64 bit value and the float value
    float           FLTVAL;                                 //!< the float value

}types_t;

/*! @brief  enum to define what data type it is
 */
typedef enum 
{
    UINT8VAL,                                               //!< it is a uint8
    UINT16VAL,                                              //!< it is a uint16
    INT32VAL,                                               //!< it is an int32
    UINT64VAL,                                              //!< it is a uint64
    FLOATVAL,                                               //!< it is a float value
    STRINGVAL/*,*/                                          //!< it is a string value (character pointer)
    //BOOLVAL                                               //!< it is a bool
}valueType_t;

/*! @brief  enum to define what data type it is
 */
typedef struct 
{
    valueType_t             type;                           //!< the type of the parameter
    bool                    checkMax;                       //!< if there is a maximum limit
    bool                    checkMin;                       //!< if there is a maximum limit
    types_t                 max;                            //!< the maximum value of the parameter
    types_t                 min;                            //!< the minimum value of the parameter
    void*                   parameterAdr;                   //!< the address of the parameter
    char                    *parameterUnit;                 //!< the unit of the parameter as a string if any, "-" otherwise
    char                    *parameterType;                 //!< the parameter type of the variable in a sting              
}BMSparametersInfo_t;   


/*! @brief  this struct contains all the parameter values and information of each parameter in an array
 */
typedef struct 
{
    BMSparametersInfo_t         parametersInfo[NONE];       //!< the BMS parameters info 
    BMSParameterValues_t        parameters;                 //!< the BMS parameter 
}BMSparameters_t;

/*! @brief these are the possible transition commands */
typedef enum 
{
    CMD_NONE            = 0,
    CMD_GO_2_SLEEP      = 1,    
    CMD_WAKE            = 2,
    CMD_GO_2_DEEPSLEEP  = 3,
    CMD_RESET           = 4,
    CMD_ERROR           
}stateCommands_t;

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BMS_DATA_TYPES_H_ */
