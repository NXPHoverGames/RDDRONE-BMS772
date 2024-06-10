/****************************************************************************
 * nxp_bms/BMS_v1/src/data.c
 *
 * BSD 3-Clause License
 *
 * Copyright 2020-2024 NXP
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
#include <pthread.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/boardctl.h>

#include "data.h"
#include "cli.h"

#include "BMS_data_types.h"
#include "BMS_data_limits.h"


/****************************************************************************
 * Defines
 ****************************************************************************/
#ifndef CONFIG_S32K1XX_RESETCAUSE_PROCFS
#    error enable CONFIG_S32K1XX_RESETCAUSE_PROCFS in menuconfig (Board Selection -> enable reset cause as a proc fs)
#endif

#define RESET_CAUSE_BYTES 10

//! @brief Define to indicate no task PID has the lock
#define NO_PID -999

//! @brief macro to initialze the BMSparametersInfo_t value for integers (and strings) (no floating point values)
#define SET_DEFAULT_INT(typeT, maxOn, minOn, userReadOnlyOn, stringUnit, stringType, par, parVal_t) \
                                                                                                                            .type                   = typeT, \
                                                                                                                            .checkMax               = maxOn, \
                                                                                                                            .checkMin               = minOn, \
                                                                                                                            .userReadOnly           = userReadOnlyOn, \
                                                                                                                            .parameterUnit          = stringUnit, \
                                                                                                                            .parameterType          = stringType, \
                                                                                                                            .max.I32                = (int32_t)par##_MAX, \
                                                                                                                            .min.I32                = (int32_t)par##_MIN, \
                                                                                                                            .defaultVal.I32         = (int32_t)par##_DEFAULT, \
                                                                                                                            .parameterAdr           = &s_parameters.parVal_t, 

//! @brief macro to initialze the BMSparametersInfo_t value for floating point values
#define SET_DEFAULT_FLT(typeT, maxOn, minOn, userReadOnlyOn, stringUnit, stringType, par, parVal_t) \
                                                                                                                            .type                   = typeT, \
                                                                                                                            .checkMax               = maxOn, \
                                                                                                                            .checkMin               = minOn, \
                                                                                                                            .userReadOnly           = userReadOnlyOn, \
                                                                                                                            .parameterUnit          = stringUnit, \
                                                                                                                            .parameterType          = stringType, \
                                                                                                                            .max.FLTVAL             = (float)par##_MAX, \
                                                                                                                            .min.FLTVAL             = (float)par##_MIN, \
                                                                                                                            .defaultVal.FLTVAL      = (float)par##_DEFAULT, \
                                                                                                                            .parameterAdr           = &s_parameters.parVal_t, 

//! @brief This macro will check if the setting has changed compared to the inNewValue, if so it will set lvChanged true
#define CHECK_CHANGED(setting, valueType)                     (setting != (*(valueType*)inNewValue)) ? (lvChanged = true) : (lvChanged = false)

//! @brief This macro will assign the setting with inNewValue
#define ASSIGN(setting, valueType)                            (setting = (*(valueType*)inNewValue));

//! @brief This macro will first check if the value has changed and set lvChanged if so, it will assign the value if it doesn't exceed the both limits
#define CHECK_ASSIGN_BOTH(max, min, setting, valueType)       CHECK_CHANGED(setting, valueType); RANGE_OK(min, (*(valueType*)inNewValue), max) {ASSIGN(setting, valueType) ret = 0;} else{ret = -1;}

//! @brief This macro will first check if the value has changed and set lvChanged if so, it will assign the value if it doesn't exceed the max limit
#define CHECK_ASSIGN_MAX(max, setting, valueType)             CHECK_CHANGED(setting, valueType); RANGE_OK_HIGH((*(valueType*)inNewValue), max) {ASSIGN(setting, valueType) ret = 0;} else{ret = -1;}

//! @brief This macro will first check if the value has changed and set lvChanged if so, it will assign the value if it doesn't exceed the min limit
#define CHECK_ASSIGN_MIN(min, setting, valueType)             CHECK_CHANGED(setting, valueType); RANGE_OK_LOW(min, (*(valueType*)inNewValue))  {ASSIGN(setting, valueType) ret = 0;} else{ret = -1;}

//! @brief This macro will first check if the value has changed and set lvChanged if so, it will assign the value 
#define CHECK_ASSIGN_NONE(setting, valueType)                 CHECK_CHANGED(setting, valueType); ASSIGN(setting, valueType) ret = 0; 

/****************************************************************************
 * Types
 ****************************************************************************/
//! @brief  enum to indicate what limit needs to be checked
typedef enum
{
    CHECK_NONE = 0, //!< no limit needs to be checked
    CHECK_LOW  = 1, //!< the lower limit needs to be checked
    CHECK_HIGH = 2, //!< the upper limit needs to be checked
    CHECK_BOTH = 3  //!< both upper and lower limit needs to be checked
} checkLimit_t;

/****************************************************************************
 * private data
 ****************************************************************************/

// the callback functions
/*! @brief  callback function to handle the parameter change */
parameterChangeCallbackFunction       gParameterChangeCallbackFunctionfp;

/*! @brief  callback function to get the main state */
getMainStateCallbackBatFuntion        gGetMainStateCallbackBatFuntionfp;

/*! @brief  callback function to get the charge state */
getChargeStateCallbackBatFuntion      gGetChargeStateCallbackBatFuntionfp;

//! the data mutex
pthread_mutex_t dataLock;

//! the eeprom mutex
pthread_mutex_t flashLock;
//! to indicate data is initialized
bool gFlashInitialized = false;

//! to indicate a configuration parameter has changed, and it should be saved to flashed
static bool gSavableParameterChanged = false;

//! to indicate if a task has the data lock via data_lockMutex()
static int gLockOwnerPID = NO_PID;

//! Variable to indicate the active BMS fault.
uint8_t gBMSFault = 0;

/*!
 * @brief the struct containing all the data with the default values, the default values are set
 *        this struct
 */
BMSParameterValues_t s_parameters = 
{
    .commonBatteryVariables.V_out                   = V_OUT_DEFAULT,
    .commonBatteryVariables.V_batt                  = V_BATT_DEFAULT,
    .commonBatteryVariables.N_cells                 = N_CELLS_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[0] = V_CELL1_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[1] = V_CELL2_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[2] = V_CELL3_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[3] = V_CELL4_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[4] = V_CELL5_DEFAULT,
    .commonBatteryVariables.V_cellVoltages.V_cellArr[5] = V_CELL6_DEFAULT,
    .commonBatteryVariables.I_batt                  = I_BATT_DEFAULT,
    .commonBatteryVariables.I_batt_avg              = I_BATT_AVG_DEFAULT,
    .commonBatteryVariables.I_batt_10s_avg          = I_BATT_10S_AVG_DEFAULT,
    .commonBatteryVariables.sensor_enable           = SENSOR_ENABLE_DEFAULT,
    .commonBatteryVariables.C_batt                  = C_BATT_DEFAULT,
    .commonBatteryVariables.C_AFE                   = C_AFE_DEFAULT,
    .commonBatteryVariables.C_T                     = C_T_DEFAULT,
    .commonBatteryVariables.C_R                     = C_R_DEFAULT,

    .calcBatteryVariables.P_avg                     = P_AVG_DEFAULT,
    .calcBatteryVariables.E_used                    = E_USED_DEFAULT,
    .calcBatteryVariables.t_full                    = T_FULL_DEFAULT,
    .calcBatteryVariables.A_rem                     = A_REM_DEFAULT,
    .calcBatteryVariables.A_full                    = A_FULL_DEFAULT,
    .calcBatteryVariables.A_factory                 = A_FACTORY_DEFAULT,
    .calcBatteryVariables.s_charge                  = S_CHARGE_DEFAULT,
    .calcBatteryVariables.s_health                  = S_HEALTH_DEFAULT,
    .calcBatteryVariables.s_out                     = S_OUT_DEFAULT,
    .calcBatteryVariables.s_in_flight               = S_IN_FLIGHT_DEFAULT,
    .calcBatteryVariables.batt_id                   = BATT_ID_DEFAULT,

    .additionalVariables.V_cell_ov                  = V_CELL_OV_DEFAULT,
    .additionalVariables.V_cell_uv                  = V_CELL_UV_DEFAULT,
    .additionalVariables.V_cell_nominal             = V_CELL_NOMINAL_DEFAULT,
    .additionalVariables.V_storage                  = V_STORAGE_DEFAULT,
    .additionalVariables.V_cell_margin              = V_CELL_MARGIN_DEFAULT,
    .additionalVariables.V_recharge_margin          = V_RECHARGE_MARGIN_DEFAULT,
    .additionalVariables.I_peak_max                 = I_PEAK_MAX_DEFAULT,
    .additionalVariables.I_out_max                  = I_OUT_MAX_DEFAULT,
    .additionalVariables.I_out_nominal              = I_OUT_NOMINAL_DEFAULT,
    .additionalVariables.I_flight_mode              = I_FLIGHT_MODE_DEFAULT,
    .additionalVariables.I_sleep_oc                 = I_SLEEP_OC_DEFAULT,
    .additionalVariables.I_system                   = I_SYSTEM_DEFAULT,
    .additionalVariables.I_charge_max               = I_CHARGE_MAX_DEFAULT,
    .additionalVariables.I_charge_nominal           = I_CHARGE_NOMINAL_DEFAULT,
    .additionalVariables.I_charge_full              = I_CHARGE_FULL_DEFAULT,
    .additionalVariables.C_cell_ot                  = C_CELL_OT_DEFAULT,
    .additionalVariables.C_cell_ut                  = C_CELL_UT_DEFAULT,
    .additionalVariables.C_pcb_ot                   = C_PCB_OT_DEFAULT,
    .additionalVariables.C_pcb_ut                   = C_PCB_UT_DEFAULT,
    .additionalVariables.C_cell_ot_charge           = C_CELL_OT_CHARGE_DEFAULT,
    .additionalVariables.C_cell_ut_charge           = C_CELL_UT_CHARGE_DEFAULT,
    .additionalVariables.N_charges                  = N_CHARGES_DEFAULT,
    .additionalVariables.N_charges_full             = N_CHARGES_FULL_DEFAULT,
    .additionalVariables.ocv_slope                  = OCV_SLOPE_DEFAULT,
    .additionalVariables.battery_type               = BATTERY_TYPE_DEFAULT,

    .configurationVariables.t_meas                  = T_MEAS_DEFAULT,
    .configurationVariables.t_ftti                  = T_FTTI_DEFAULT,
    .configurationVariables.t_bms_timeout           = T_BMS_TIMEOUT_DEFAULT,
    .configurationVariables.t_fault_timeout         = T_FAULT_TIMEOUT_DEFAULT,
    .configurationVariables.t_bcc_sleep_cyclic      = T_BCC_SLEEP_CYCLIC_DEFAULT,
    .configurationVariables.t_sleep_timeout         = T_SLEEP_TIMEOUT_DEFAULT,
    .configurationVariables.t_ocv_cyclic0           = T_OCV_CYCLIC0_DEFAULT,
    .configurationVariables.t_ocv_cyclic1           = T_OCV_CYCLIC1_DEFAULT,
    .configurationVariables.t_charge_detect         = T_CHARGE_DETECT_DEFAULT,
    .configurationVariables.t_cb_delay              = T_CB_DELAY_DEFAULT,
    .configurationVariables.t_charge_relax          = T_CHARGE_RELAX_DEFAULT,
    .configurationVariables.batt_eol                = BATT_EOL_DEFAULT,
    .configurationVariables.s_flags                 = S_FLAGS_DEFAULT,
    .configurationVariables.self_discharge_enable   = SELF_DISCHARGE_ENABLE_DEFAULT,
    .configurationVariables.flight_mode_enable      = FLIGHT_MODE_ENABLE_DEFAULT,
    .configurationVariables.emergency_button_enable = EMERGENCY_BUTTON_ENABLE_DEFAULT,
    .configurationVariables.smbus_enable            = SMBUS_ENABLE_DEFAULT,
    .configurationVariables.gate_check_enable       = GATE_CHECK_ENABLE_DEFAULT,
    .configurationVariables.model_id                = MODEL_ID_DEFAULT,
    .configurationVariables.model_name              = MODEL_NAME_DEFAULT,

    .canVariables.Cyphal_node_static_id             = CYPHAL_NODE_STATIC_ID_DEFAULT,
    .canVariables.Cyphal_es_sub_id                  = CYPHAL_ES_SUB_ID_DEFAULT,
    .canVariables.Cyphal_bs_sub_id                  = CYPHAL_BS_SUB_ID_DEFAULT,
    .canVariables.Cyphal_bp_sub_id                  = CYPHAL_BP_SUB_ID_DEFAULT,
    .canVariables.Cyphal_legacy_bi_sub_id           = CYPHAL_LEGACY_BI_SUB_ID_DEFAULT,
    .canVariables.DroneCAN_node_static_id           = DRONECAN_NODE_STATIC_ID_DEFAULT,
    .canVariables.DroneCAN_bat_continuous           = DRONECAN_BAT_CONTINUOUS_DEFAULT,
    .canVariables.DroneCAN_bat_periodic             = DRONECAN_BAT_PERIODIC_DEFAULT,
    .canVariables.DroneCAN_bat_cells                = DRONECAN_BAT_CELLS_DEFAULT,
    .canVariables.DroneCAN_bat_info                 = DRONECAN_BAT_INFO_DEFAULT,
    .canVariables.DroneCAN_bat_info_aux             = DRONECAN_BAT_INFO_AUX_DEFAULT,
    .canVariables.can_mode                          = CAN_MODE_DEFAULT,
    .canVariables.can_fd_mode                       = CAN_FD_MODE_DEFAULT,
    .canVariables.can_bitrate                       = CAN_BITRATE_DEFAULT,
    .canVariables.can_fd_bitrate                    = CAN_FD_BITRATE_DEFAULT,

    .hardwareVariables.V_min                        = V_MIN_DEFAULT,
    .hardwareVariables.V_max                        = V_MAX_DEFAULT,
    .hardwareVariables.I_range_max                  = I_RANGE_MAX_DEFAULT,
    .hardwareVariables.I_max                        = I_MAX_DEFAULT,
    .hardwareVariables.I_short                      = I_SHORT_DEFAULT,
    .hardwareVariables.t_short                      = T_SHORT_DEFAULT,
    .hardwareVariables.I_bal                        = I_BAL_DEFAULT,
    .hardwareVariables.m_mass                       = M_MASS_DEFAULT,
    .hardwareVariables.f_v_out_divider_factor       = F_V_OUT_DIVIDER_FACTOR_DEFAULT
};

/*! @brief  This struct array contains all the parameter info variables like every type, minOn, maxOn, min,
 * max, address the default values are set using the SET_DEFAULT_FLT or the SET_DEFAULT_INT macro
 */
const BMSparametersInfo_t s_parametersInfo[NONE] = 
{
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_OUT, commonBatteryVariables.V_out) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_BATT, commonBatteryVariables.V_batt) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "uint8",  N_CELLS, commonBatteryVariables.N_cells) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL1, commonBatteryVariables.V_cellVoltages.V_cellArr[0]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL2, commonBatteryVariables.V_cellVoltages.V_cellArr[1]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL3, commonBatteryVariables.V_cellVoltages.V_cellArr[2]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL4, commonBatteryVariables.V_cellVoltages.V_cellArr[3]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL5, commonBatteryVariables.V_cellVoltages.V_cellArr[4]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "V",  "float",  V_CELL6, commonBatteryVariables.V_cellVoltages.V_cellArr[5]) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "A",  "float",  I_BATT, commonBatteryVariables.I_batt) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "A",  "float",  I_BATT_AVG, commonBatteryVariables.I_batt_avg) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "A",  "float",  I_BATT_10S_AVG, commonBatteryVariables.I_batt_10s_avg) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   SENSOR_ENABLE, commonBatteryVariables.sensor_enable) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "C",  "float",  C_BATT, commonBatteryVariables.C_batt) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "C",  "float",  C_AFE, commonBatteryVariables.C_AFE) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "C",  "float",  C_T, commonBatteryVariables.C_T) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "C",  "float",  C_R, commonBatteryVariables.C_R) },

    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "W",  "float",  P_AVG, calcBatteryVariables.P_avg) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, true,   "Wh", "float",  E_USED, calcBatteryVariables.E_used) },
    { SET_DEFAULT_FLT(FLOATVAL,  false, true, true,   "h",  "float",  T_FULL, calcBatteryVariables.t_full) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "Ah", "float",  A_REM, calcBatteryVariables.A_rem) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "Ah", "float",  A_FULL, calcBatteryVariables.A_full) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "Ah", "float",  A_FACTORY, calcBatteryVariables.A_factory) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  false, true,  "%",  "uint8",  S_CHARGE, calcBatteryVariables.s_charge) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  false, true,  "%",  "uint8",  S_HEALTH, calcBatteryVariables.s_health) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  false, true,  "-",  "bool",   S_OUT, calcBatteryVariables.s_out) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  false, true,  "-",  "bool",   S_IN_FLIGHT, calcBatteryVariables.s_in_flight) },
    { SET_DEFAULT_INT(UINT8VAL,  false, false, false, "-",  "uint8",  BATT_ID, calcBatteryVariables.batt_id) },

    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "V",  "float",  V_CELL_OV, additionalVariables.V_cell_ov) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "V",  "float",  V_CELL_UV, additionalVariables.V_cell_uv) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "V",  "float",  V_CELL_NOMINAL, additionalVariables.V_cell_nominal) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "V",  "float",  V_STORAGE, additionalVariables.V_storage) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "mV", "uint8",  V_CELL_MARGIN, additionalVariables.V_cell_margin) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "mV", "uint16", V_RECHARGE_MARGIN, additionalVariables.V_recharge_margin) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "A",  "float",  I_PEAK_MAX, additionalVariables.I_peak_max) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "A",  "float",  I_OUT_MAX, additionalVariables.I_out_max) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "A",  "float",  I_OUT_NOMINAL, additionalVariables.I_out_nominal) },
    { SET_DEFAULT_INT(UINT8VAL,  true, true, false,   "A",  "uint8",  I_FLIGHT_MODE, additionalVariables.I_flight_mode) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "mA", "uint8",  I_SLEEP_OC, additionalVariables.I_sleep_oc) },  
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "mA", "uint8",  I_SYSTEM, additionalVariables.I_system) }, 
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "A",  "float",  I_CHARGE_MAX, additionalVariables.I_charge_max) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "A",  "float",  I_CHARGE_NOMINAL, additionalVariables.I_charge_nominal) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "mA", "uint16", I_CHARGE_FULL, additionalVariables.I_charge_full) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_CELL_OT, additionalVariables.C_cell_ot) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_CELL_UT, additionalVariables.C_cell_ut) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_PCB_OT, additionalVariables.C_pcb_ot) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_PCB_UT, additionalVariables.C_pcb_ut) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_CELL_OT_CHARGE, additionalVariables.C_cell_ot_charge) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "C",  "float",  C_CELL_UT_CHARGE, additionalVariables.C_cell_ut_charge) },    
    { SET_DEFAULT_INT(UINT16VAL, false, false, false, "-",  "uint16", N_CHARGES, additionalVariables.N_charges) },
    { SET_DEFAULT_INT(UINT16VAL, false, false, false, "-",  "uint16", N_CHARGES_FULL, additionalVariables.N_charges_full) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "mV/A.min", "float", OCV_SLOPE, additionalVariables.ocv_slope) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "uint8",  BATTERY_TYPE, additionalVariables.battery_type) },

    { SET_DEFAULT_INT(UINT16VAL, false, true, false,  "ms", "uint16", T_MEAS, configurationVariables.t_meas) },
    { SET_DEFAULT_INT(UINT16VAL, false, true, false,  "ms", "uint16", T_FTTI, configurationVariables.t_ftti) },
    { SET_DEFAULT_INT(UINT16VAL, false, true, false,  "s",  "uint16", T_BMS_TIMEOUT, configurationVariables.t_bms_timeout) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "s",  "uint16", T_FAULT_TIMEOUT, configurationVariables.t_fault_timeout) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "s",  "uint8",  T_BCC_SLEEP_CYCLIC, configurationVariables.t_bcc_sleep_cyclic) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "h",  "uint8",  T_SLEEP_TIMEOUT, configurationVariables.t_sleep_timeout) },
    { SET_DEFAULT_INT(INT32VAL,  false, true, false,  "s",  "int32",  T_OCV_CYCLIC0, configurationVariables.t_ocv_cyclic0) },
    { SET_DEFAULT_INT(INT32VAL,  false, true, false,  "s",  "int32",  T_OCV_CYCLIC1, configurationVariables.t_ocv_cyclic1) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "s",  "uint8",  T_CHARGE_DETECT, configurationVariables.t_charge_detect) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "s",  "uint8",  T_CB_DELAY, configurationVariables.t_cb_delay) },
    { SET_DEFAULT_INT(UINT16VAL, false, true, false,  "s",  "uint16", T_CHARGE_RELAX, configurationVariables.t_charge_relax) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  false, false, "%",  "uint8",  BATT_EOL, configurationVariables.batt_eol) },
    { SET_DEFAULT_INT(UINT8VAL,  false, false, true,  "-",  "uint8",  S_FLAGS, configurationVariables.s_flags) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   SELF_DISCHARGE_ENABLE, configurationVariables.self_discharge_enable) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   FLIGHT_MODE_ENABLE, configurationVariables.flight_mode_enable) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   EMERGENCY_BUTTON_ENABLE, configurationVariables.emergency_button_enable) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   SMBUS_ENABLE, configurationVariables.smbus_enable) }, 
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   GATE_CHECK_ENABLE, configurationVariables.gate_check_enable) },
    { SET_DEFAULT_INT(UINT64VAL, false, false, false, "-",  "uint64", MODEL_ID, configurationVariables.model_id) },
    { SET_DEFAULT_INT(STRINGVAL, false, false, false, "-",  "char[32]", MODEL_NAME, configurationVariables.model_name) },

    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "uint8",  CYPHAL_NODE_STATIC_ID, canVariables.Cyphal_node_static_id) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "-",  "uint16", CYPHAL_ES_SUB_ID, canVariables.Cyphal_es_sub_id) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "-",  "uint16", CYPHAL_BS_SUB_ID, canVariables.Cyphal_bs_sub_id) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "-",  "uint16", CYPHAL_BP_SUB_ID, canVariables.Cyphal_bp_sub_id) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "-",  "uint16", CYPHAL_LEGACY_BI_SUB_ID, canVariables.Cyphal_legacy_bi_sub_id) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "uint8",  DRONECAN_NODE_STATIC_ID, canVariables.DroneCAN_node_static_id) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   DRONECAN_BAT_CONTINUOUS, canVariables.DroneCAN_bat_continuous) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   DRONECAN_BAT_PERIODIC, canVariables.DroneCAN_bat_periodic) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   DRONECAN_BAT_CELLS, canVariables.DroneCAN_bat_cells) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   DRONECAN_BAT_INFO, canVariables.DroneCAN_bat_info) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "bool",   DRONECAN_BAT_INFO_AUX, canVariables.DroneCAN_bat_info_aux) },
    { SET_DEFAULT_INT(STRINGVAL, false, false, false, "-",  "char[10]",  CAN_MODE, canVariables.can_mode) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "-",  "uint8",  CAN_FD_MODE, canVariables.can_fd_mode) },
    { SET_DEFAULT_INT(INT32VAL,  true,  true, false,  "bit/s", "int32", CAN_BITRATE, canVariables.can_bitrate) },
    { SET_DEFAULT_INT(INT32VAL,  true,  true, false,  "bit/s", "int32", CAN_FD_BITRATE, canVariables.can_fd_bitrate) },

    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "V",  "uint8",  V_MIN, hardwareVariables.V_min) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "V",  "uint8",  V_MAX, hardwareVariables.V_max) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "A",  "uint16", I_RANGE_MAX, hardwareVariables.I_range_max) },
    { SET_DEFAULT_INT(UINT8VAL,  true,  true, false,  "A",  "uint8",  I_MAX, hardwareVariables.I_max) },
    { SET_DEFAULT_INT(UINT16VAL, true,  true, false,  "A",  "uint16", I_SHORT, hardwareVariables.I_short) },
    { SET_DEFAULT_INT(UINT8VAL,  false, true, false,  "us", "uint8",  T_SHORT, hardwareVariables.t_short) },
    { SET_DEFAULT_INT(UINT8VAL,  false, false, false, "mA", "uint8",  I_BAL, hardwareVariables.I_bal) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "kg", "float",  M_MASS, hardwareVariables.m_mass) },
    { SET_DEFAULT_FLT(FLOATVAL,  true,  true, false,  "-",  "float",  F_V_OUT_DIVIDER_FACTOR, hardwareVariables.f_v_out_divider_factor) }
};

/****************************************************************************
 * private Function prototypes
 ****************************************************************************/
/*!
 * @brief       function to get a certain parameter from the data struct in data.c
 *              this function could be used after the data_setParameter function
 *              there are 2 ways to get the parameter, either with the outData parameter
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initialize should have been called once
 *
 * @param       parameterKind the parameter value it wants, from the parameterKind enum in BMS_data_types.h
 * @param       outData pointer to the value that needs to become the parameter value, could be int32_t, float
 *              or char*
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes)
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      a void pointer to the value
 */
static void* getParameterNoLock(parameterKind_t parameterKind, void* outData, uint16_t* outLength);

/*!
 * @brief       function to set a certain parameter in the data struct
 *              after this, the value can be read with the data_getParameter function
 *              this function only sets the new value if it is within the range
 *              of the parameter as discribed in BMS_data_limits.h
 *              data_initialize should have been called once
 *
 *              If it is changed, it will call handleParamaterChange() to handle the change
 *
 * @param       parameterKind the setting it wants to set from the parameterKind enum in BMS_data_types.h
 * @param       inNewValue a pointer to the value it will be
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 */
static int setParameterNoLock(parameterKind_t parameterKind, void* inNewValue);

/*!
 * @brief   Function to handle a parameter change.
 * @note    Can be called from multiple threads.
 *
 * @param   parameter The parameter that changed.
 * @param   value Address of the variable containing the new value.
 *
 * @return  0 if succeeded, false otherwise
 */
static int handleParamaterChange(parameterKind_t parameter, void* value);

/****************************************************************************
 * public Functions
 ****************************************************************************/

/*!
 * @brief     this function is needed to use the data_setParameter and data_getParameter
 *            it will initialize the mutex and return the outcome of the pthread_mutex_init function
 *
 * @param     p_parameterChangeCallbackFunction the address of the function to start a task to handle a
 *            parameter change
 * @param     p_getChargeStateCallbackBatFuntion the address of the function to get the main state
 * @param     p_userCommandCallbackBatFuntion the address of function to get the charge state
 *
 * @return    If successful, the function will return zero (OK). Otherwise, an error number will be returned
 *            to indicate the error:
 * @example   if(data_initialize())
 *        {
 *          // do something with the error
 *        }
 */
int data_initialize(parameterChangeCallbackFunction p_parameterChangeCallbackFunction,
    getMainStateCallbackBatFuntion                  p_getMainStateCallbackBatFuntion,
    getChargeStateCallbackBatFuntion                p_getChargeStateCallbackBatFuntion)
{
    int     ret        = -1;
    uint8_t flightMode = 0;

    // connect the callback functions
    gParameterChangeCallbackFunctionfp  = p_parameterChangeCallbackFunction;
    gGetMainStateCallbackBatFuntionfp   = p_getMainStateCallbackBatFuntion;
    gGetChargeStateCallbackBatFuntionfp = p_getChargeStateCallbackBatFuntion;

    // initialize the mutex lock, otherwise it will not work
    ret = pthread_mutex_init(&dataLock, NULL);

    if(ret != 0)
    {
        cli_printfError("data_initialize ERROR: couldn't init data mutex!\n");
        return ret;
    }

    // initialize flash mutex
    ret = pthread_mutex_init(&flashLock, NULL);

    // check for errors
    if(ret != 0)
    {
        cli_printfError("data_initialize ERROR: couldn't init flash mutex!\n");
        return ret;
    }

    // set the value to true
    gFlashInitialized = true;

    // try to load the parameters, it will go wrong the first time and load the default ones
    if(data_loadParameters())
    {
        cli_printf("nothing/wrong saved!\n");
    }

    // get the enable value
    if(data_getParameter(FLIGHT_MODE_ENABLE, &flightMode, NULL) == NULL)
    {
        cli_printfError("data ERROR: couldn't get flight mode variable!\n");
        flightMode = 1;
    }

    // check if flight mode is on
    if(flightMode)
    {
        // Send a warning to the user
        cli_printfWarning("WARNING: flight-mode-enable is on during startup!\n");
    }

    return ret;
}

/*!
 * @brief     function to the type a certain parameter from the data struct in data.c
 *            this function could be used before the data_setParameter or data_getParameter function
 *
 * @param     parameterKind the parameter of which the type should be returned, from the parameterKind enum in
 * BMS_data_types.h
 *
 * @retval    the type of the parameter as a valueType_t enum value, defined in BMS_data_types.h
 *            it will return STRINGVAL if the parameterKind was NONE
 *
 * @example   valueType_t paramType;
 *            switch(paramType)
 *            {
 *              case FLOATVAL: // so somthing with floating points values
 *              break;
 *              case STRINGVAL: // do somthing with string values
 *              break;
 *              case default: // something with int32_t values (uint16 and uint8 can be set and get with
 * int32) break;
 *            }
 *
 */
valueType_t data_getType(parameterKind_t parameterKind)
{
    // use the most not used value as an error
    valueType_t ret = STRINGVAL;

    // check if not NONE
    if(parameterKind != NONE)
    {
        // get the type
        ret = s_parametersInfo[parameterKind].type;
    }

    // return the value
    return ret;
}

/*!
 * @brief       function to get a certain parameter from the data struct in data.c
 *              this function could be used after the data_setParameter function
 *              there are 2 ways to get the parameter, either with the outData parameter
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initialize should have been called once
 *
 * @param       parameterKind the parameter value it wants, from the parameterKind enum in BMS_data_types.h
 * @param       outData pointer to the value that needs to become the parameter value, could be int32_t, float
 *              or char*
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes)
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      a void pointer to the value
 * @example     int32_t lvGetParam;
 *              uint8_t *pReadUint8_tData = NULL;
 *              pReadUint8_tData = data_getParameter(S_CHARGE, &lvGetParam, NULL);
 *              if(pReadUint8_tData != NULL)
 *              {
 *                // do something with value
 *              }
 *              else
 *              {
 *                // something went wrong!
 *              }
 *              other example:
 *              char  *GetString = malloc(sizeof(char[32]));
 *              uint16_t *Lenght = malloc(sizeof(uint16_t));
 *              data_getParameter(MODEL_NAME, GetString, Lenght);
 *              free(GetString);
 *              free(Lenght);
 */
void* data_getParameter(parameterKind_t parameterKind, void* outData, uint16_t* outLength)
{
    // make a return variable
    void* ret = NULL;
    int   errorCode;

    // check if wrong input
    if(parameterKind == NONE)
    {
        cli_printfError("data ERROR: wrong input!\n");
        return ret;
    }

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: pthread_mutex_lock failed %d\n", errorCode);
        return ret;
    }

    // get the parameter
    ret = getParameterNoLock(parameterKind, outData, outLength);

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: pthread_mutex_unlock failed %d\n", errorCode);
        return NULL;
    }

    return ret;
}

/*!
 * @brief       function to set a certain parameter in the data struct
 *              after this, the value can be read with the data_getParameter function
 *              this function only sets the new value if it is within the range
 *              of the parameter as discribed in BMS_data_limits.h
 *              data_initialize should have been called once
 *
 * @note        If the new parameter changes, it will take action to handle the change
 *
 * @param       parameterKind the setting it wants to set from the parameterKind enum in BMS_data_types.h
 * @param       inNewValue a pointer to the value it will be
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 * @example:    int32_t newValue = 3; or float newValue = 3.3
 *              if(data_setParameter(S_CHARGE, &newValue))
 *              {
 *                // it went wrong!
 *              }
 *
 *              for model name:
 *              char newModelName[] = "New model name\0";
 *              if(data_setParameter(MODEL_NAME, newModelName))
 *              {
 *                // it went wrong!
 *              }
 */
int data_setParameter(parameterKind_t parameterKind, void* inNewValue)
{
    // the return varaiable
    int ret = -1;

    // check for a void pointer and right input
    if(inNewValue == NULL || parameterKind == NONE)
    {
        cli_printfError("data ERROR: wrong input!\n");
        return ret;
    }

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");
        return ret;
    }

    // set the parameter
    ret = setParameterNoLock(parameterKind, inNewValue);

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        ret = -1;
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
        return ret;
    }

    // return the value
    return ret;
}

/*!
 * @brief       function to get the min and max value of a certain parameter from the data struct in data.c
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initializeData should have been called once
 *
 * @param       parameterKind the parameter value it wants the min and max from,
 *              from the parameterKind enum in BMS_data_types.h
 * @param       outMin pointer to the value that needs to become the parameter minimum value,
 *              could be int64_t or float.
 * @param       outMax pointer to the value that needs to become the parameter maximum value,
 *              could be int64_t or float.
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 */
int data_getParameterMinMax(parameterKind_t parameterKind, void* outMin, void* outMax)
{
    int ret = -1;

    // check if not NONE
    if(parameterKind >= NONE || outMin == NULL || outMax == NULL)
    {
        cli_printfError("data_getParameterMinMax ERROR: wrong input!\n");
        return ret;
    }

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data_getParameterMinMax ERROR: pthread_mutex_lock failed\n");
        return ret;
    }

    // check the type
    switch(s_parametersInfo[parameterKind].type)
    {
        // in case it is a floatvalue
        case FLOATVAL: // set the returnvalue to the address of the parameter
        {
            *(float*)outMin = s_parametersInfo[parameterKind].min.FLTVAL;
            *(float*)outMax = s_parametersInfo[parameterKind].max.FLTVAL;
            ret             = 0;
        }
        break;
        // in case it is a uint8_t value
        case UINT8VAL:
        {
            *(uint8_t*)outMin = s_parametersInfo[parameterKind].min.U8;
            *(uint8_t*)outMax = s_parametersInfo[parameterKind].max.U8;
            ret               = 0;
        }
        break;
        // in case it is a uint16_t value
        case UINT16VAL:
        {
            *(uint16_t*)outMin = s_parametersInfo[parameterKind].min.U16;
            *(uint16_t*)outMax = s_parametersInfo[parameterKind].max.U16;
            ret                = 0;
        }
        break;
        // in case it is a int32_t value
        case INT32VAL:
        {
            *(int32_t*)outMin = s_parametersInfo[parameterKind].min.I32;
            *(int32_t*)outMax = s_parametersInfo[parameterKind].max.I32;
            ret               = 0;
        }
        break;
        // in case it is a uint64_t value
        case UINT64VAL:
        {
            *(int32_t*)outMin = s_parametersInfo[parameterKind].min.I32;
            *(int32_t*)outMax = s_parametersInfo[parameterKind].max.I32;
            ret               = 0;
        }
        break;
        // in case it is a string value
        case STRINGVAL:
        {
            ret               = 0;
            *(int32_t*)outMin = 0;
            *(int32_t*)outMax = 0;
            // will not get min, max
        }
        break;
    }

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        ret = -1;
        cli_printfError("data_getParameterMinMax ERROR: pthread_mutex_unlock failed\n");
        return ret;
    }

    // return the value
    return ret;
}

/*!
 * @brief       function to get the default value of a certain parameter from the data struct in data.c
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initializeData should have been called once
 *
 * @param       parameterKind the parameter value it wants the default value from,
 *              from the parameterKind enum in BMS_data_types.h
 * @param       outDefaultData pointer to the value that needs to become the parameter default value,
 *              could be int64_t, float char*.
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes)
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 */
int data_getParameterDefault(parameterKind_t parameterKind, void* outDefaultData, uint16_t* outLength)
{
    // make a return variable
    int ret = -1;

    // check if not NONE
    if(parameterKind >= NONE || outDefaultData == NULL)
    {
        cli_printfError("data_getParameterDefault ERROR: wrong input!\n");
        return ret;
    }

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data_getParameterDefault ERROR: pthread_mutex_lock failed\n");
        return ret;
    }

    // this switch will check which type it is
    // it will assign the lenght of the data
    // it will assign outDefaultValue with the default value
    switch(s_parametersInfo[parameterKind].type)
    {
        // in case it is a floatvalue
        case FLOATVAL: // set the returnvalue to the address of the parameter
        {
            *(float*)outDefaultData = s_parametersInfo[parameterKind].defaultVal.FLTVAL;
            ret                     = 0;
        }
        break;
        // in case it is a uint8_t value
        case UINT8VAL:
        {
            *(uint8_t*)outDefaultData = s_parametersInfo[parameterKind].defaultVal.U8;
            ret                       = 0;
        }

        break;
        // in case it is a uint16_t value
        case UINT16VAL:
        {
            *(uint16_t*)outDefaultData = s_parametersInfo[parameterKind].defaultVal.U16;
            ret                        = 0;
        }

        break;
        // in case it is a int32_t value
        case INT32VAL:
        {
            *(int32_t*)outDefaultData = s_parametersInfo[parameterKind].defaultVal.I32;
            ret                       = 0;
        }
        break;
        // in case it is a uint64_t value
        case UINT64VAL:
        {
            if(parameterKind == MODEL_ID)
            {
                *(uint64_t*)outDefaultData = MODEL_ID_DEFAULT;
                ret                        = 0;
            }
            else
            {
                cli_printfError(
                    "data_getParameterDefault ERROR: this uint64 not supported yet, please add param: %d\n",
                    parameterKind);
            }
        }

        break;
        // in case it is a string value
        case STRINGVAL:
        {
            // check which string parameter
            switch(parameterKind)
            {
                case MODEL_NAME:
                {
                    // if outlenght isn't NULL
                    if(outLength != NULL)
                    {
                        // set the outlenght to the size of the value
                        *outLength = strlen(MODEL_NAME_DEFAULT);
                    }

                    // set outDefaultData to the needed value
                    strncpy(((char*)outDefaultData), MODEL_NAME_DEFAULT, STRING_MAX_CHARS);
                    ret = 0;
                }

                break;
                case CAN_MODE:
                {
                    // if outlenght isn't NULL
                    if(outLength != NULL)
                    {
                        // set the outlenght to the size of the value
                        *outLength = strlen(CAN_MODE_DEFAULT);
                    }

                    // set outDefaultData to the needed value
                    strncpy(((char*)outDefaultData), CAN_MODE_DEFAULT, STRING_MAX_CHARS);
                    ret = 0;
                }
                break;
                default:
                    cli_printfError(
                        "data_getParameterDefault ERROR: not supported, please add parameter %d to this\n",
                        parameterKind);
                    break;
            }
        }
        break;
    }

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        ret = -1;
        cli_printfError("data_getParameterDefault ERROR: pthread_mutex_unlock failed\n");
        return ret;
    }

    return ret;
}

/*!
 * @brief       function to get if a certain parameter is read only for a user from the data struct in data.c
 *              for setting parameters via CLI or CAN.
 *              data_initializeData should have been called once
 *
 * @param       parameterKind the parameter value it wants the read only state of,
 *              from the parameterKind enum in BMS_data_types.h
 *
 * @retval      is -1 when something went wrong, 0 if the user may change this parameter, 1 if it is read only
 *              for a user.
 */
int data_getParameterIfUserReadOnly(parameterKind_t parameterKind)
{
    int ret = -1;

    /* Check for wrong user input */
    if(parameterKind > NONE)
    {
        return ret;
    }
    else if(parameterKind == NONE)
    {
        ret = 1;
        return ret;
    }

    // return if it is read only
    return (int)s_parametersInfo[parameterKind].userReadOnly;
}

/*
 * @brief   Function to be called when the parameter change needs to be handled.
 *          Should be used with data_setCalcBatteryVariables()
 *
 * @param   parameterKind the setting it wants to handle from the parameterKind enum in BMS_data_types.h
 * @param   inNewValue A pointer to the value it will handle
 *
 * @return  0 if succeeded, false otherwise
 */
int data_handleParameterChange(parameterKind_t parameter, void* inNewValue)
{
    int ret;

    pthread_mutex_lock(&dataLock);

    // handle the change
    ret = handleParamaterChange(parameter, inNewValue);

    pthread_mutex_unlock(&dataLock);

    return ret;
}

/*!
 * @brief     function to lock the mutex (make sure that no other tasks/threads can acces/change the data)
 *            make sure the mutex is unlocked again with data_unlockMutex
 *            should always be used with data_unlockMutex
 *
 * @param     none
 *
 * @retval    0 if succeeded, otherwise the error of pthread_mutex_lock
 *
 * @example   uint8_t stateOfCharge;
 *            data_lockMutex();
 *            stateOfCharge = *(data_getAdr(S_CHARGE));
 *            data_unlockMutex();
 *
 */
int data_lockMutex(void)
{
    int ret;

    // get the lock
    ret = pthread_mutex_lock(&dataLock);

    // check if the lock can and may be locked
    if(gLockOwnerPID == NO_PID && !(ret))
    {
        // save the process ID
        gLockOwnerPID = getpid();
    }
    else
    {
        // unlock and error
        pthread_mutex_unlock(&dataLock);
        return -1;
    }

    return ret;
}

/*!
 * @brief   function to unlock the mutex (make sure that other tasks/threads can acces/change the data)
 *          make sure the mutex is locked with the data_lockMutex function before calling this funciton
 *          should always be used with the data_lockMutex
 *
 * @param   none
 *
 * @retval  0 if succeeded, otherwise the error of pthread_mutex_unlock
 *
 * @example uint8_t stateOfCharge;
 *          data_lockMutex();
 *          stateOfCharge = *(data_getAdr(S_CHARGE));
 *          data_unlockMutex();
 *
 */
int data_unlockMutex(void)
{
    // check if it may unlock
    if(gLockOwnerPID == getpid())
    {
        // reset the variable and unlock
        gLockOwnerPID = NO_PID;
        return pthread_mutex_unlock(&dataLock);
    }

    // return error
    return -1;
}

/*!
 * @brief   function to get the address of a certain parameter from the data struct in data.c
 *          this function is faster than the get or set function
 *          this function should be used with the data_lockMutex() and data_unlockMutex() functions
 *
 *
 * @param   parameterKind the parameter of which the address should be returned, from the parameterKind enum
 *          in BMS_data_types.h
 *
 * @retval  an void pointer addres to the value
 * @warning be sure to lock te mutex before this function and unlock it after.
 *          shouldn't be used to set a variable!
 *          use with caution, a variable could be changed with this function but there are no limit checks
 *          there is alno no check if the varaible has changed (the signal for other functions)
 *
 * @example uint8_t stateOfCharge;
 *          data_lockMutex();
 *          stateOfCharge = *(data_getAdr(S_CHARGE));
 *          data_unlockMutex();
 *
 */
void* data_getAdr(parameterKind_t parameterKind)
{
    // check the input
    if(parameterKind != NONE)
    {
        // return the parameter address
        return s_parametersInfo[parameterKind].parameterAdr;
    }
    // else
    else
    {
        // return NULL when out of range
        return NULL;
    }
}

/*!
 * @brief   function to get the unit as a strings of a certain parameter
 *
 * @param   parameterKind the parameter of which the unit should be returned,
 *          from the parameterKind enum in BMS_data_types.h
 *
 * @retval  a char pointer (string) of the value
 *
 */
char* data_getUnit(parameterKind_t parameterKind)
{
    char* noUnit = "-";

    // check if out of range
    if(parameterKind >= NONE)
    {
        // return the unit -
        return noUnit;
    }
    // if in range
    else
    {
        // return the unit
        return (char*)s_parametersInfo[parameterKind].parameterUnit;
    }
}

/*!
 * @brief   function to get the type as a strings of a certain parameter
 *
 * @param   parameterKind the parameter of which the type should be returned,
 *          from the parameterKind enum in BMS_data_types.h
 *
 * @retval  a char pointer (string) of the value
 *
 */
char* data_getTypeString(parameterKind_t parameterKind)
{
    char* noUnit = "-";

    // check if out of range
    if(parameterKind >= NONE)
    {
        // return the unit -
        return noUnit;
    }
    // if in range
    else
    {
        // return the unit
        return (char*)s_parametersInfo[parameterKind].parameterType;
    }
}

/*!
 * @brief   function that will return the main state, it will use a mutex
 * @note  Could be called from multiple threads
 *
 * @param   none
 *
 * @return  the state of the main state machine.
 */
states_t data_getMainState(void)
{
    // return the main state
    return gGetMainStateCallbackBatFuntionfp();
}

/*!
 * @brief   function that will return the charge state, it will use a mutex
 * @note  Could be called from multiple threads
 *
 * @param   none
 *
 * @return the state of the charge state machine.
 */
charge_states_t data_getChargeState(void)
{
    // return the main state
    return gGetChargeStateCallbackBatFuntionfp();
}

/*!
 * @brief   function that will copy the commonBatteryVariables_t struct
 *          From the struct saved in data to the destination struct
 *          It will use the mutex for data protection
 *
 * @param   destination The address of the pointer to the
 *          commonBatteryVariables_t struct to copy the struct in.
 *
 * @return  0 if succeeded, -1 otherwise
 */
int data_getCommonBatteryVariables(commonBatteryVariables_t* destination)
{
    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");
        return -1;
    }

    // copy the data struct to the destination struct
    memcpy(destination, &(s_parameters.commonBatteryVariables), sizeof(commonBatteryVariables_t));

    // unlock the mutex after it is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
        return -1;
    }

    return 0;
}

/*!
 * @brief   function that will copy the commonBatteryVariables_t struct
 *          From the source struct to the saved struct in data
 *          It will use the mutex for data protection
 * @note    n-cells and sensor-enable will not be overwritten!
 *
 * @param   source The source pointer address that copies to the data struct
 *
 * @return  0 if succeeded, -1 otherwise
 */
int data_setCommonBatteryVariables(commonBatteryVariables_t* source)
{
    uint8_t nCells, sensorEnable;

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");
        return -1;
    }

    // save the n-cells and sensor-enable (these should maintain the latest value)
    nCells       = s_parameters.commonBatteryVariables.N_cells;
    sensorEnable = s_parameters.commonBatteryVariables.sensor_enable;

    // copy the data struct to the destination struct
    memcpy(&(s_parameters.commonBatteryVariables), source, sizeof(commonBatteryVariables_t));

    // copy back the ncells and sensorenable
    s_parameters.commonBatteryVariables.N_cells       = nCells;
    s_parameters.commonBatteryVariables.sensor_enable = sensorEnable;

    // unlock the mutex after it is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
        return -1;
    }

    return 0;
}

/*!
 * @brief   function that will copy the calcBatteryVariables_t struct
 *          From the struct saved in data to the destination struct
 *          It will use the mutex for data protection
 *
 * @param   destination The address of the pointer to the
 *          calcBatteryVariables_t struct to copy the struct in.
 * @param   gotLock Indication if the task already has the lock via data_lockMutex().
 *          Keep false by default.
 *
 * @return  0 if succeeded, otherwise it will indicate the error
 */
int data_getCalcBatteryVariables(calcBatteryVariables_t* destination, bool gotLock)
{
    int retVal = 0;

    // check if trylock is on
    if(!gotLock || (gLockOwnerPID != getpid()))
    {
        // lock the mutex(with error check)
        if((pthread_mutex_lock(&dataLock)) != 0)
        {
            cli_printfError("data ERROR: pthread_mutex_lock failed\n");
            retVal = -1;
            return retVal;
        }
    }

    // copy the data struct to the destination struct
    memcpy(destination, &(s_parameters.calcBatteryVariables), sizeof(calcBatteryVariables_t));

    // check if trylock is off or the mutex was locked
    if(!gotLock || (gLockOwnerPID != getpid()))
    {
        // unlock the mutex after it is done
        if((pthread_mutex_unlock(&dataLock)) != 0)
        {
            cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
            retVal = -1;
        }
    }

    return retVal;
}

/*!
 * @brief   function that will copy the calcBatteryVariables_t struct
 *          From the source struct to the saved struct in data
 *          It will use the mutex for data protection
 *
 * @warning Make sure to handle the changed parameter with data_handleParameterChange()
 *
 * @param   source The source pointer address that copies to the data struct
 * @param   gotLock Indication if the task already has the lock via data_lockMutex().
 *          Keep false by default.
 *
 * @return  0 if succeeded, otherwise it will indicate the error
 */
int data_setCalcBatteryVariables(calcBatteryVariables_t* source, bool gotLock)
{
    int retVal = 0;

    // check if trylock is on
    if(!gotLock || (gLockOwnerPID != getpid()))
    {
        // lock the mutex(with error check)
        if((pthread_mutex_lock(&dataLock)) != 0)
        {
            cli_printfError("data ERROR: pthread_mutex_lock failed\n");
            retVal = -1;
            return retVal;
        }
    }

    // copy the data struct to the destination struct
    memcpy(&(s_parameters.calcBatteryVariables), source, sizeof(calcBatteryVariables_t));

    // check if trylock is off or the mutex was locked
    if(!gotLock || (gLockOwnerPID != getpid()))
    {
        // unlock the mutex after it is done
        if((pthread_mutex_unlock(&dataLock)) != 0)
        {
            cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
            retVal = -1;
        }
    }

    return retVal;
}

/*!
 * @brief   function to set a bit in the status flags (s_flags)
 * @note    if the flags are S_FLAGS_UKNOWN, clear the s_flags first.
 *
 * @param     bit the bit that needs to change in the s_flags variable (0-7) use the STATUS_*_BIT defines for
 *            it
 * @param     value the new value of this bit as a bool
 *
 * @retval    0 if succeeded, otherwise the error of pthread_mutex_unlock
 *
 * @example   if(data_statusFlagBit(STATUS_OVERLOAD_BIT, 1))
 *        {
 *          // do something with the error
 *        }
 *
 */
int data_statusFlagBit(uint8_t bit, bool value)
{
    int ret = 1;

    // check the input
    if(bit > STATUS_HIGHEST_BIT)
    {
        // return and output to user
        cli_printfError("data_statusFlagBit ERROR: input bit: %d > %d\n", bit, STATUS_HIGHEST_BIT);
        return ret;
    }

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");
        return ret;
    }

    // check if the flags need to be cleared (if it was unknown)
    if(s_parameters.configurationVariables.s_flags == S_FLAGS_UKNOWN)
    {
        // reset the flags
        s_parameters.configurationVariables.s_flags = 0;
    }

    // TODO reset the STATUS_ASK_PARS_BIT when the service request is send or reset is send

    // check if setting the bit or clearing it
    if(value)
    {
        // set the new bit
        s_parameters.configurationVariables.s_flags |= 1 << bit;
    }
    else
    {
        // clear the bit
        s_parameters.configurationVariables.s_flags &= ~(1 << bit);
    }

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");
        ret = -1;
        return ret;
    }

    // return
    return ret;
}

/*!
 * @brief     function to save the parameters in flash
 * @note      The eeeprom is used for this (4KB)
 * @note       Multi-thread protected
 *
 * @retval    0 if it went OK, negative otherwise
 */
int data_saveParameters(void)
{
    int            ret = 0;
    int            fd, writtenBytes, i;
    uint32_t       CRC = 1;
    uint8_t*       CRCCalc;
    const uint32_t parSize = sizeof(s_parameters) / sizeof(int8_t);

    // check if initialized
    if(!gFlashInitialized)
    {
        // output to user
        cli_printfError("data_saveParameters ERROR: not initialized!\n");

        ret -= 1;

        // return erro
        return ret;
    }

    // lock the datalock
    pthread_mutex_lock(&dataLock);

    // check if it does not needs to save anything
    if(!gSavableParameterChanged)
    {
        // output to user
        cli_printf("data_saveParameters: parameters didn't change, nothing will be saved\n");

        ret = 0;

        // unlock the datalock
        pthread_mutex_unlock(&dataLock);

        // return erro
        return ret;
    }

    // unlock the datalock
    pthread_mutex_unlock(&dataLock);

    // lock the mutex
    pthread_mutex_lock(&flashLock);

    // Open the eeprom device
    fd = open("/dev/eeeprom0", O_WRONLY);

    // check if open worked in debug mode
    DEBUGASSERT(fd >= 0);

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");

        // close the filedescriptor
        close(fd);

        // unlock the mutex
        pthread_mutex_unlock(&flashLock);

        ret -= 2;

        // return
        return ret;
    }

    // write the struct
    writtenBytes = write(fd, &s_parameters, parSize);

    // cli_printf("writtenBytes: %d\n", writtenBytes);

    // check for error
    if(writtenBytes != parSize)
    {
        // output to user
        cli_printfError(
            "data_saveParameters ERROR: could not write parameters! %d != %d\n", writtenBytes, parSize);

        ret -= 4;

        // return erro
        ret--;
    }
    else
    {
        // reset the save variable, because new variable have been saved
        gSavableParameterChanged = false;
    }

    // set the crcclat to the beginning of the parameters
    CRCCalc = (uint8_t*)(&s_parameters);

    // calculate CRC
    for(i = 0; i < parSize; i++)
    {
        // sum all the values
        CRC += *CRCCalc;

        // increment the address of the pointer
        CRCCalc++;
    }

    // unlock the mutex(with error check)
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");

        // close the filedescriptor
        close(fd);

        // unlock the mutex
        pthread_mutex_unlock(&flashLock);

        ret -= 8;

        // return
        return ret;
    }

    // cli_printf("CRC: %d\n", CRC);

    // write the CRC
    writtenBytes = write(fd, &CRC, sizeof(CRC) / sizeof(uint8_t));

    // check for error
    if(writtenBytes != sizeof(CRC) / sizeof(uint8_t))
    {
        // output to user
        cli_printfError("data_saveParameters ERROR: could not write CRC! %d != %d\n", writtenBytes,
            sizeof(CRC) / sizeof(uint8_t));

        // return error
        ret -= 16;

        // reset eeprom?
    }

    // close the filedescriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    // return to user
    return ret;
}

/*!
 * @brief     function to load the parameters from flash
 * @note      The eeeprom is used for this (4KB)
 * @note      Multi-thread protected
 *
 * @retval    0 if it went OK, negative otherwise
 */
int data_loadParameters(void)
{
    int ret = 0;

    int                  fd, i, readBytes;
    uint32_t             CRCR = 1, CRCW = 1;
    uint8_t*             CRCCalc;
    const uint32_t       parSize = sizeof(s_parameters) / sizeof(uint8_t);
    BMSParameterValues_t oldParameters;

    // check if initialized
    if(!gFlashInitialized)
    {
        // output to user
        cli_printfError("data_saveParameters ERROR: not initialized!\n");

        ret -= 1;

        // return erro
        return ret;
    }

    // lock the mutex
    pthread_mutex_lock(&flashLock);

    // make read only
    fd = open("/dev/eeeprom0", O_RDONLY);

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_lock failed\n");

        // close file descriptor
        close(fd);

        // unlock the mutex
        pthread_mutex_unlock(&flashLock);

        ret -= 2;

        // return to user
        return ret;
    }

    // save the old values
    oldParameters = s_parameters;

    // read the paramters
    readBytes = read(fd, &s_parameters, parSize);

    // cli_printf("readBytes %d\n", readBytes);

    // check if the bytes to read is ok
    if(readBytes != parSize)
    {
        // output to user
        cli_printfError("data_saveParameters ERROR: could not read parameters!\n");

        // return erro
        ret -= 8;

        // reset eeprom and save default values in call
    }

    // // check the CRC
    readBytes = read(fd, &CRCR, sizeof(CRCR) / sizeof(uint8_t));

    // cli_printf("CRCR: %d readBytes %d\n", CRCR, readBytes);

    // check if the bytes to read is ok
    if(readBytes != sizeof(CRCR) / sizeof(uint8_t))
    {
        // output to user
        cli_printfError("data_saveParameters ERROR: could not read CRC!\n");

        // return erro
        ret -= 16;
        // reset eeprom and save default values in call
    }

    CRCCalc = (uint8_t*)&s_parameters;

    // calculate CRC
    for(i = 0; i < parSize; i++)
    {
        // sum all the values
        CRCW += *CRCCalc;

        // increment the address of the pointer
        CRCCalc++;
    }

    // cli_printf("CRCW: %d\n", CRCW);

    // check CRC
    if(CRCR != CRCW)
    {
        // output to user
        cli_printf("CRC of saved data doesn't match!\n");

        // return erro
        ret -= 32;

        cli_printf("Setting old values!\n");

        // save the old values
        s_parameters = oldParameters;
    }

    // unlock the mutex(with error check)
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        cli_printfError("data ERROR: pthread_mutex_unlock failed\n");

        // close file descriptor
        close(fd);

        // unlock the mutex
        pthread_mutex_unlock(&flashLock);

        ret -= 4;

        // return to user
        return ret;
    }

    // close file descriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    // return to user
    return ret;
}

/*!
 * @brief     this function will set the default values of the BMS to the data struct
 * @note      Multi-thread protected
 *
 * @param     none
 *
 * @return    0 if succeeded, negative otherwise
 */
int data_setDefaultParameters(void)
{
    int ret = 0;
    int i;
    variableTypes_u variable;

    // set the default value for all parameters
    for(i = 0; i < NONE; i++)
    {
        /* Get the parameter type */
        switch(data_getType((parameterKind_t)i))
        {
            case UINT8VAL:
            {
                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, &variable.uint8Var, NULL))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, &variable.uint8Var))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
            case UINT16VAL:
            {
                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, &variable.uint16Var, NULL))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, &variable.uint16Var))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
            case INT32VAL:
            {
                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, &variable.int32Var, NULL))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, &variable.int32Var))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
            case UINT64VAL:
            {
                uint64_t variable64 = 0;
                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, &variable64, NULL))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, &variable64))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
            case FLOATVAL:
            {
                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, &variable.floatVar, NULL))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, &variable.floatVar))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
            case STRINGVAL:
            {
                char     charVal[STRING_MAX_CHARS];
                uint16_t stringSize = 0;

                /* Get the default varibale */
                if(data_getParameterDefault((parameterKind_t)i, charVal, &stringSize))
                {
                    cli_printfError("data_setDefaultParameters ERROR: could not get param: %d\n", i);
                    ret = -1;
                }
                else
                {
                    // check if size is more than the max
                    if(stringSize > STRING_MAX_CHARS)
                    {
                        // limit
                        stringSize = (STRING_MAX_CHARS);
                    }
                    // check for 0
                    else if(stringSize <= 0)
                    {
                        // set to minimum size
                        stringSize = 1;
                    }

                    // add null term
                    charVal[stringSize - 1] = '\0';

                    /* Set the default variable */
                    if(data_setParameter((parameterKind_t)i, charVal))
                    {
                        cli_printfError("data_setDefaultParameters: ERROR: Could not set param: %d\n", i);
                        ret = -1;
                    }
                }
            }
            break;
        }

        // sleep for a small bit to offload MCU
        usleep(10);
    }

    // return to the user
    return ret;
}

/*!
 * @brief     this function will return the difference of 2 timespec structs
 *
 * @param     timeHigh  The high time (later sampled)
 * @param     timeLow   The low time (earlier sampled)
 *
 * @return    The difference in between timeHigh and timeLow in us
 */
int data_getUsTimeDiff(struct timespec timeHigh, struct timespec timeLow)
{
    return (((timeHigh.tv_sec * 1000000) + (timeHigh.tv_nsec / 1000)) -
        ((timeLow.tv_sec * 1000000) + (timeLow.tv_nsec / 1000)));
}

/*!
 * @brief   function to get the MCU reset cause
 *
 * @param   none
 *
 * @return  the value of the reset cause register of the MCU, 0 if error
 */
unsigned int data_getResetCause(void)
{
    unsigned int resetCause = 0;
    int          fd         = 0, errorCode;
    ssize_t      ret;
    char         readData[RESET_CAUSE_BYTES];

    // open de proc fs of the reset cause
    fd = open("/proc/resetcause", O_RDONLY);

    // check for errors
    if(fd < 0)
    {
        // error
        errorCode = errno;
        cli_printfError("data_getResetCause ERROR: Can't open resetcause fs, %d\n", errorCode);
    }
    // if no error
    else
    {
        // read the reset cause
        ret = read(fd, readData, RESET_CAUSE_BYTES);

        // check for errors
        if(ret <= 0)
        {
            cli_printfError("data_getResetCause ERROR: Can't read resetcause fs\n");
        }
        else
        {
            // convert reset cause string to number
            resetCause = (unsigned int)strtol(readData, NULL, 0);
        }
    }

    // close the device
    close(fd);

    // return
    return resetCause;
}

/****************************************************************************
 * Private functions
 ****************************************************************************/
/*!
 * @brief       function to get a certain parameter from the data struct in data.c
 *              this function could be used after the data_setParameter function
 *              there are 2 ways to get the parameter, either with the outData parameter
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initialize should have been called once
 *
 * @param       parameterKind the parameter value it wants, from the parameterKind enum in BMS_data_types.h
 * @param       outData pointer to the value that needs to become the parameter value, could be int32_t, float
 *              or char*
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes)
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      a void pointer to the value
 */
static void* getParameterNoLock(parameterKind_t parameterKind, void* outData, uint16_t* outLength)
{
    // make a return variable
    void* ret = NULL;

    // this switch will check which type it is
    // it will assign the lenght of the data
    // it will set the pointer of ret to the right data of the s_parameters struct
    // it will assign outData with the value of ret
    switch(s_parametersInfo[parameterKind].type)
    {
        // in case it is a floatvalue
        case FLOATVAL: // set the returnvalue to the address of the parameter
            ret = (float*)s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = sizeof(*ret);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                (*(float*)outData = *(float*)ret);
            }
            break;
        // in case it is a uint8_t value
        case UINT8VAL: // set the returnvalue to the address of the parameter
            ret = (uint8_t*)s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = sizeof(*ret);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                (*(uint8_t*)outData = *(uint8_t*)ret);
            }

            break;
        // in case it is a uint16_t value
        case UINT16VAL: // set the returnvalue to the address of the parameter
            ret = (uint16_t*)s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = sizeof(*ret);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                (*(uint16_t*)outData = *(uint16_t*)ret);
            }

            break;
        // in case it is a int32_t value
        case INT32VAL: // set the returnvalue to the address of the parameter
            ret = (int32_t*)s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = sizeof(*ret);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                (*(int32_t*)outData = *(int32_t*)ret);
            }

            break;
        // in case it is a uint64_t value
        case UINT64VAL: // set the returnvalue to the address of the parameter
            ret = (uint64_t*)s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = sizeof(*ret);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                (*(uint64_t*)outData = *(uint64_t*)ret);
            }

            break;
        // in case it is a string value
        case STRINGVAL: // set the returnvalue to the address of the parameter
            ret = s_parametersInfo[parameterKind].parameterAdr;

            // if outlenght isn't NULL
            if(outLength != NULL)
            {
                // set the outlenght to the size of the value
                *outLength = strlen(s_parametersInfo[parameterKind].parameterAdr);
            }

            // if outData isn't NULL
            if(outData != NULL)
            {
                // set outData to the needed value
                strncpy(((char*)outData), s_parametersInfo[parameterKind].parameterAdr, STRING_MAX_CHARS);
            }

            break;
        // just in case
        default:
            break;
    }

    return ret;
}

/*!
 * @brief       function to set a certain parameter in the data struct
 *              after this, the value can be read with the data_getParameter function
 *              this function only sets the new value if it is within the range
 *              of the parameter as discribed in BMS_data_limits.h
 *              data_initialize should have been called once
 *
 *              If it is changed, it will call handleParamaterChange() to handle the change
 *
 * @param       parameterKind the setting it wants to set from the parameterKind enum in BMS_data_types.h
 * @param       inNewValue a pointer to the value it will be
 *
 * @retval      is -1 when something went wrong, 0 when it went right
 */
static int setParameterNoLock(parameterKind_t parameterKind, void* inNewValue)
{
    int ret;

    // variable to check if the variable has changed
    bool lvChanged = false;

    // make the enum for the switch
    checkLimit_t lvCheckLimit = (checkLimit_t)(
        (s_parametersInfo[parameterKind].checkMax << 1) | (s_parametersInfo[parameterKind].checkMin));

    // this switch will check if the parameter is within its limits as stated in "BMS_data_limits.h"
    // if is is within the limits it will assign the value to the s_parameters struct
    // and it will set ret to 0, meaning it went right
    // before setting the parameter it will check if the old en new parameters are different, if so lvChanged
    // = true;
    switch(s_parametersInfo[parameterKind].type)
    {
        // in case it is a floatvalue
        case FLOATVAL:

            // check if there is a limit on it
            // this switch will check what check needs to be done with the limit
            // it will assign the right value
            // it will set lvChanged high if the data is different
            // it will set ret if succeeded
            switch(lvCheckLimit)
            {
                // if the limit check need to be done on both high and low limit
                case CHECK_BOTH:
                    CHECK_ASSIGN_BOTH((float)(s_parametersInfo[parameterKind].max.FLTVAL),
                        (float)(s_parametersInfo[parameterKind].min.FLTVAL),
                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                    break;
                // if the limit check need to be done on low limit
                case CHECK_LOW:
                    CHECK_ASSIGN_MIN((float)s_parametersInfo[parameterKind].min.FLTVAL,
                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                    break;
                // if the limit check need to be done on high limit
                case CHECK_HIGH:
                    CHECK_ASSIGN_MAX((float)s_parametersInfo[parameterKind].max.FLTVAL,
                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                    break;
                // if the limit check shouldn't be done
                case CHECK_NONE:
                    CHECK_ASSIGN_NONE(*(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                    break;
            }

            break;
        // in case it is a uint8_t value
        case UINT8VAL: // check if there is a limit on it
            // this switch will check what check needs to be done with the limit
            // it will assign the right value
            // it will set lvChanged high if the data is different
            // it will set ret if succeeded
            switch(lvCheckLimit)
            {
                // if the limit check need to be done on both high and low limit
                case CHECK_BOTH:
                    CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.U8,
                        s_parametersInfo[parameterKind].min.U8,
                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                    break;
                // if the limit check need to be done on low limit
                case CHECK_LOW:
                    CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.U8,
                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                    break;
                // if the limit check need to be done on high limit
                case CHECK_HIGH:
                    CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.U8,
                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                    break;
                // if the limit check shouldn't be done
                case CHECK_NONE:
                    CHECK_ASSIGN_NONE(*(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                    break;
            }
            break;
        // in case it is a uint16_t value
        case UINT16VAL: // check if there is a limit on it
            // this switch will check what check needs to be done with the limit
            // it will assign the right value
            // it will set lvChanged high if the data is different
            // it will set ret if succeeded
            switch(lvCheckLimit)
            {
                // if the limit check need to be done on both high and low limit
                case CHECK_BOTH:
                    CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.U16,
                        s_parametersInfo[parameterKind].min.U16,
                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                    break;
                // if the limit check need to be done on low limit
                case CHECK_LOW:
                    CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.U16,
                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                    break;
                // if the limit check need to be done on high limit
                case CHECK_HIGH:
                    CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.U16,
                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                    break;
                // if the limit check shouldn't be done
                case CHECK_NONE:
                    CHECK_ASSIGN_NONE(*(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                    break;
            }

            break;
        // in case it is a int32_t value
        case INT32VAL: // check if there is a limit on it
            // this switch will check what check needs to be done with the limit
            // it will assign the right value
            // it will set lvChanged high if the data is different
            // it will set ret if succeeded
            switch(lvCheckLimit)
            {
                // if the limit check need to be done on both high and low limit
                case CHECK_BOTH:
                    CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.I32,
                        s_parametersInfo[parameterKind].min.I32,
                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                    break;
                // if the limit check need to be done on low limit
                case CHECK_LOW:
                    CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.I32,
                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                    break;
                // if the limit check need to be done on high limit
                case CHECK_HIGH:
                    CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.I32,
                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                    break;
                // if the limit check shouldn't be done
                case CHECK_NONE:
                    CHECK_ASSIGN_NONE(*(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                    break;
            }

            break;
        // in case it is a uint64_t value
        // WARNING max value to check on is INT32_MAX and min value is INT32_MIN
        case UINT64VAL: // check if there is a limit on it
            // this switch will check what check needs to be done with the limit
            // it will assign the right value
            // it will set lvChanged high if the data is different
            // it will set ret if succeeded
            // WARNING max value to check on is INT32_MAX and min value is INT32_MIN
            switch(lvCheckLimit)
            {
                // if the limit check need to be done on both high and low limit
                case CHECK_BOTH:
                    CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.I32,
                        s_parametersInfo[parameterKind].min.I32,
                        *(uint64_t*)(s_parametersInfo[parameterKind].parameterAdr), uint64_t)
                    break;
                // if the limit check need to be done on low limit
                case CHECK_LOW:
                    CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.I32,
                        *(uint64_t*)(s_parametersInfo[parameterKind].parameterAdr), uint64_t)
                    break;
                // if the limit check need to be done on high limit
                case CHECK_HIGH:
                    CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.I32,
                        *(uint64_t*)(s_parametersInfo[parameterKind].parameterAdr), uint64_t)
                    break;
                // if the limit check shouldn't be done
                case CHECK_NONE:
                    CHECK_ASSIGN_NONE(*(uint64_t*)(s_parametersInfo[parameterKind].parameterAdr), uint64_t)
                    break;
            }

            break;

        // in case it is a string value
        case STRINGVAL: // do a stringcompare on the model name and if it is not equal
            if(strcmp(s_parametersInfo[parameterKind].parameterAdr, ((char*)inNewValue)))
            {
                // set the change value to true
                lvChanged = true;
            }

            // string copy the new value in the struct
            strncpy(s_parametersInfo[parameterKind].parameterAdr, ((char*)inNewValue), STRING_MAX_CHARS);

            // set the return value to true
            ret = 0;
            break;
        // just in case
        default:
            break;
    }

    // check if a parameter has changed
    if(lvChanged && ret == 0)
    {
        // call the callback function
        ret = handleParamaterChange(parameterKind, inNewValue);

        // check which parameter has been changed if it needs to be saved
        // Things that are measured should not be saved
        if((!gSavableParameterChanged) &&
            ((parameterKind == N_CELLS) || (parameterKind == SENSOR_ENABLE) || (parameterKind == A_FULL) ||
                (parameterKind == A_FACTORY) || (parameterKind == S_HEALTH) || (BATT_ID <= parameterKind)))
        {
            // cli_printf("this parameter changed: %d", parameterKind);
            // set the savable parameter true
            gSavableParameterChanged = true;
        }
    }

    return ret;
}

/*!
 * @brief   Function to handle a parameter change.
 * @note    Can be called from multiple threads.
 *
 * @param   parameter The parameter that changed.
 * @param   value Address of the variable containing the new value.
 *
 * @return  0 if succeeded, false otherwise
 */
static int handleParamaterChange(parameterKind_t parameter, void* value)
{
    int             ret = -1;
    variableTypes_u variable1;
    variableTypes_u variable2;
    variableTypes_u variable3;

    // which parameter changed
    switch(parameter)
    {
        // check if should not handle the change
        // see checkCurrentMeasurement() and checkAllMeasurements() in batmangement
        case V_OUT:
        case V_BATT:
        case V_CELL1:
        case V_CELL2:
        case V_CELL3:
        case V_CELL4:
        case V_CELL5:
        case V_CELL6:
        case I_BATT:
        case I_BATT_AVG:
        case I_BATT_10S_AVG:
        case C_BATT:
        case C_AFE:
        case C_T:
        case C_R:
            // just return OK
            return 0;
            break;

        // in case it is the flight mode enable
        case FLIGHT_MODE_ENABLE:

            // set variable1 to false to indicate that it should not be checked in the main
            variable1.boolVar = false;

            // check if the flight mode enable is now false
            if(!((*(uint8_t*)value) & UINT8_MAX))
            {
                // make the in flight status variable false again
                variable1.uint8Var = 0;

                // Set the in flight parameter
                if(setParameterNoLock(S_IN_FLIGHT, &variable1.uint8Var))
                {
                    cli_printfError("handleParamaterChange ERROR: couldn't set parameter in %d\n", parameter);
                    return ret;
                }

                // set variable1 to true
                variable1.boolVar = true;
            }

            // make sure to return OK
            ret = 0;

            break;

        case A_FULL:
        case A_REM:
            // check a-full, a-rem with each other and a-factory.
            // calc new s-charge and s-health.
            // check for bad battery (batt-eol).
            // will set new values in the data struct

            // check if it is A-full
            if(parameter == A_FULL)
            {
                // get a-full
                variable2.floatVar = *(float*)value;

                // Get the remaining capacity a-rem
                if(getParameterNoLock(A_REM, &(variable1.floatVar), NULL) == NULL)
                {
                    cli_printfError("handleParameterChange ERROR: couldn't get a-rem\n");
                    return ret;
                }
            }
            // if the a-rem changed
            else
            {
                // Get the full charge capacity a-full
                if(getParameterNoLock(A_FULL, &(variable2.floatVar), NULL) == NULL)
                {
                    cli_printfError("handleParameterChange ERROR: couldn't get a-full\n");
                    return ret;
                }

                // Get the remaining capacity a-rem
                variable1.floatVar = *(float*)value;
            }

            // Check if a-rem is more than a-full
            if(variable1.floatVar > variable2.floatVar)
            {
                // Get the factory capacity a-factory
                if(getParameterNoLock(A_FACTORY, &(variable3.floatVar), NULL) == NULL)
                {
                    cli_printfError("handleParameterChange ERROR: couldn't get a-factory\n");
                    return ret;
                }

                // check if a-rem is more than a-factory
                if(variable1.floatVar > variable3.floatVar)
                {
                    // limit a-rem with a-factory
                    variable1.floatVar = variable3.floatVar;

                    // set a-rem with the corrected value
                    if(setParameterNoLock(A_REM, &(variable1.floatVar)))
                    {
                        cli_printfError("handleParameterChange ERROR: couldn't set a-full\n");
                        return ret;
                    }

                    // set the s-health to 100%
                    variable3.uint8Var = 100;

                    // set the new state of health
                    if(setParameterNoLock(S_HEALTH, &(variable3.uint8Var)))
                    {
                        cli_printfError("handleParameterChange ERROR: couldn't set s-health\n");
                        return ret;
                    }
                }
                // if a-rem is not more than a-factory
                else
                {
                    // calculate SoH with A-full (will be set with a-rem) / A-factory
                    variable3.uint8Var =
                        (uint8_t)((variable1.floatVar / variable3.floatVar) * 100) & UINT8_MAX;

                    // set the new SoH
                    if(setParameterNoLock(S_HEALTH, &(variable3.uint8Var)))
                    {
                        cli_printfError("handleParameterChange ERROR: couldn't set s-health\n");
                        return ret;
                    }
                }

                // set a-full with a-rem to correct it
                if(setParameterNoLock(A_FULL, &(variable1.floatVar)))
                {
                    cli_printfError("handleParameterChange ERROR: couldn't set a-full\n");
                    return ret;
                }

                // set the s-charge to 100% since it is equal to a-full
                variable1.uint8Var = 100;
            }
            // if a-rem is less than a-full
            else
            {
                // calculate the new s-charge (a-rem / a-full)
                variable1.uint8Var = (uint8_t)((variable1.floatVar / variable2.floatVar) * 100) & UINT8_MAX;

                // check if the s-health needs to be calculated as well
                if(parameter == A_FULL)
                {
                    // Get the factory capacity a-factory
                    if(getParameterNoLock(A_FACTORY, &(variable3.floatVar), NULL) == NULL)
                    {
                        cli_printfError("handleParameterChange ERROR: couldn't get a-factory\n");
                        return ret;
                    }

                    // calculate SoH with A-full / A-factory
                    variable3.uint8Var =
                        (uint8_t)((variable2.floatVar / variable3.floatVar) * 100) & UINT8_MAX;

                    // set the new state of health
                    if(setParameterNoLock(S_HEALTH, &(variable3.uint8Var)))
                    {
                        cli_printfError("handleParameterChange ERROR: couldn't set s-health\n");
                        return ret;
                    }
                }
                else
                {
                    // set the s-health to 100%
                    variable3.uint8Var = 100;
                }
            }

            // set the new state of charge
            if(setParameterNoLock(S_CHARGE, &(variable1.uint8Var)))
            {
                cli_printfError("handleParameterChange ERROR: couldn't set s-charge\n");
                return ret;
            }

            // the new led blink will be set via main.c and variable1.uint8Var

            // check for batt-eol
            // get the bad battery threshold
            if(getParameterNoLock(BATT_EOL, &(variable2.uint8Var), NULL) == NULL)
            {
                cli_printfError("handleParameterChange ERROR: couldn't get batt-eol\n");
                return ret;
            }

            // check if the new s-health is worse or equal than the batt-eol
            if(variable3.uint8Var <= variable2.uint8Var)
            {
                // check if the flags need to be cleared (if it was unknown)
                if(s_parameters.configurationVariables.s_flags == S_FLAGS_UKNOWN)
                {
                    // reset the flags
                    s_parameters.configurationVariables.s_flags = 0;
                }

                // set the new bad battery bit
                s_parameters.configurationVariables.s_flags |= 1 << STATUS_BAD_BATTERY_BIT;
            }

            // make sure to return OK
            ret = 0;

            break;

        // in case of a-factory, set a-full based on a-health
        // and i-charge-full based on a-factory
        case A_FACTORY:

            // get the state of health
            if(getParameterNoLock(S_HEALTH, &(variable1.uint8Var), NULL) == NULL)
            {
                cli_printfError("handleParamaterChange ERROR: getting state of health went wrong!\n");
                return ret;
            }

            // check if state of health is more than 100% (undefined)
            if(variable1.uint8Var > 100)
            {
                // set it to the max value
                variable1.uint8Var = 100;
            }

            // calculate the a-full and place it in currentmA
            variable2.floatVar = (*(float*)value * variable1.uint8Var) / 100;

            // // sleep for 1us to output nsh> first
            // usleep(1);

            // output to the user
            cli_printf("Setting a-full with %d%% (SoH) of a-factory(%.3f): %.3f\n", variable1.uint8Var,
                *(float*)value, variable2.floatVar);

            // set the new a-full
            if(setParameterNoLock(A_FULL, &variable2.floatVar))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set a-full!\n");
                return ret;
            }

            // calculate the new i-charge-full
            variable1.uint16Var = (int)(*(float*)value * 10);

            // output to the user
            cli_printf("Setting i-charge-full with 1%% of a-factory(%.3f): %d\n", *(float*)value,
                variable1.uint16Var);

            // set the i-charge-full variable to 1% of it
            if(setParameterNoLock(I_CHARGE_FULL, &variable1.uint16Var))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set i-charge-full!\n");
                return ret;
            }

            // make sure to return OK
            ret = 0;

            break;
        // if one of the subject IDs has changed
        case CYPHAL_ES_SUB_ID:
        case CYPHAL_BS_SUB_ID:
        case CYPHAL_BP_SUB_ID:

            // check if it is more than the maximum allowed value
            if(*(uint16_t*)value > CYPHAL_MAX_SUB_ID && (*(uint16_t*)value != CYPHAL_UNSET_SUB_ID))
            {
                // // sleep to output the nsh> message first
                // usleep(1);

                // output to the user
                cli_printfWarning(
                    "WARNING: Entered subject id: %d > (max) %d!\n", *(uint16_t*)value, CYPHAL_MAX_SUB_ID);
                cli_printf("Setting this subject ID (par: %d) to the unset value: %d\n", parameter,
                    CYPHAL_UNSET_SUB_ID);

                // set the subject ID value to CYPHAL_UNSET_SUB_ID
                variable1.uint16Var = CYPHAL_UNSET_SUB_ID;
                if(setParameterNoLock(parameter, &(variable1.uint16Var)))
                {
                    cli_printfError(
                        "handleParamaterChange ERROR: couldn't set subject id (par %d)!\n", parameter);
                    return ret;
                }
            }

            // make sure to return OK
            ret = 0;

            break;
        case CAN_MODE:

            // check if this is the correct CAN name
            if((strncasecmp((char*)value, CAN_MODE_OFF, sizeof(CAN_MODE_OFF)) != 0) && 
                (strncasecmp((char*)value, CAN_MODE_DRONECAN, sizeof(CAN_MODE_DRONECAN)) != 0) && 
                (strncasecmp((char*)value, CAN_MODE_CYPHAL, sizeof(CAN_MODE_CYPHAL)) != 0)) 
            {
                cli_printfError("ERROR: Entered variable %s is not one of the CAN options!\n", (char*)value);
                cli_printf("Valid options are: %s, %s and %s\n\n", CAN_MODE_OFF, CAN_MODE_DRONECAN, CAN_MODE_CYPHAL);
                cli_printfWarning("WARNING: setting can-mode to OFF\n");

                // setting the can-mode to off
                setParameterNoLock(CAN_MODE, CAN_MODE_OFF);
            }
            else
            {
                // make sure to return OK
                ret = 0;
                cli_printfWarning("WARNING: New CAN mode (%s) will only take effect after \"bms save\" followed by \"reboot\"\n", (char*)value);
            }
            
            break;
        // in case of the emergency button enable variable
        case EMERGENCY_BUTTON_ENABLE:

            // // sleep for a short while to output the nsh> message first
            // usleep(1);

            // set the variable to 1 to handle the pinchange
            variable1.uint8Var = 1;

            // call the callback function to handle the pin change
            ret = gParameterChangeCallbackFunctionfp(parameter, value, (void*)&variable1);

            // check for errors
            if(ret)
            {
                cli_printfError("handleParamaterChange ERROR: failed to change emergency GPIO!\n");

                // change back the emergency button enable variable to what it was
                variable1.uint8Var = (!(*(uint8_t*)value)) & 1;

                // output to the user that you change it back.
                cli_printfWarning("WARNING: changing emergency button enable to %d!\n", variable1.uint8Var);

                // change the variable
                if(setParameterNoLock(parameter, &(variable1.uint8Var)))
                {
                    // output error
                    cli_printfError(
                        "handleParamaterChange ERROR: couldn't set emergency button enable (par %d)!\n",
                        parameter);
                    return ret;
                }
            }

            // set the extra variable to 0 to not handle the pin change anymore
            variable1.uint8Var = 0;

            break;

        // if the user changed the battery type
        case BATTERY_TYPE:

            // // sleep a little bit for the nsh> (for CLI output)
            // usleep(1);

            // check what kind of battery it changed to
            // and set the default v-cell-ov and v-cell-uv
            switch(*(uint8_t*)value)
            {
                // if it has changed to a LiPo battery
                case 0:
                case 3:
                {
                    variable1.floatVar = LIPO_V_CELL_OV_DEFAULT;
                    variable2.floatVar = LIPO_V_CELL_UV_DEFAULT;
                }
                break;
                // if it has changed to a LiFePo4 battery
                case 1:
                {
                    variable1.floatVar = LIFEPO4_V_CELL_OV_DEFAULT;
                    variable2.floatVar = LIFEPO4_V_CELL_UV_DEFAULT;
                }
                break;
                // if it has changed to a LiFeYPo4 battery
                case 2:
                {
                    variable1.floatVar = LIFEYPO4_V_CELL_OV_DEFAULT;
                    variable2.floatVar = LIFEYPO4_V_CELL_UV_DEFAULT;
                }
                break;
                // if it has changed to a sodium-ion (Na-ion) battery
                case 4:
                {
                    variable1.floatVar = SODIUM_ION_V_CELL_OV_DEFAULT;
                    variable2.floatVar = SODIUM_ION_V_CELL_UV_DEFAULT;

                    cli_printfWarning("WARNING: Check if these sodium-ion parameters are OK for your cell!\n");
                    cli_printfWarning("WARNING: Make sure that the BMS voltage does not go below 6V!, check amount of cells (n-cells) and the v-cell-uv\n");
                    cli_printf("NOTE: Please set the over- and undertemperatures for your cell: \nc_cell_ot, c-cell-ut, c-pcb-ot, c-pcb-ut, c-cell-ot-charge and c-cell-ut-charge\n");
                }
                break;
                default:
                    cli_printfError("ERROR: add new battery type correctly!!\n");
                    break;
            }

            // set the 2 new values and handle the change
            // output to the user
            cli_printfWarning("WARNING: Setting v-cell-ov to %.3f\n", variable1.floatVar);

            // set the overvoltage
            if(setParameterNoLock(V_CELL_OV, &variable1.floatVar))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set v-cell-ov\n");
                return ret;
            }
            // if no error
            else
            {
                // handle the changed parameter in the battery management part
                ret = gParameterChangeCallbackFunctionfp(V_CELL_OV, (void*)&variable1.floatVar, NULL);
                if(ret)
                {
                    cli_printfError(
                        "handleParamaterChange ERROR: couldn't handle change for v-cell-ov: %.3f\n",
                        variable1.floatVar);
                    return ret;
                }
            }

            // output to the user
            cli_printfWarning("WARNING: Setting v-cell-uv to %.3f\n", variable2.floatVar);

            // set the overvoltage
            if(setParameterNoLock(V_CELL_UV, &variable2.floatVar))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set v-cell-uv\n");
                ret = -1;
                return ret;
            }
            // if no error
            else
            {
                // handle the changed parameter in the battery management part
                ret = gParameterChangeCallbackFunctionfp(V_CELL_UV, (void*)&variable2.floatVar, NULL);
                if(ret)
                {
                    cli_printfError(
                        "handleParamaterChange ERROR: couldn't handle change for v-cell-uv: %.3f\n",
                        variable2.floatVar);
                    return ret;
                }
            }

            // check what kind of battery it changed to
            // and set the default v-storage and v-cell-margin
            switch(*(uint8_t*)value)
            {
                // if it has changed to a LiPo battery
                case 0:
                case 3:
                {
                    variable1.floatVar = LIPO_V_STORAGE_DEFAULT;
                    variable2.floatVar = LIPO_V_CELL_NOMINAL_DEFAULT;
                }
                break;
                // if it has changed to a LiFePo4 battery
                case 1:
                {
                    variable1.floatVar = LIFEPO4_V_STORAGE_DEFAULT;
                    variable2.floatVar = LIFEPO4_V_CELL_NOMINAL_DEFAULT;
                }
                break;
                // if it has changed to a LiFeYPo4 battery
                case 2:
                {
                    variable1.floatVar = LIFEYPO4_V_STORAGE_DEFAULT;
                    variable2.floatVar = LIFEYPO4_V_CELL_NOMINAL_DEFAULT;
                }
                break;
                // if it has changed to a sodium-ion (Na-ion) battery
                case 4:
                {
                    variable1.floatVar = SODIUM_ION_V_STORAGE_DEFAULT;
                    variable2.floatVar = SODIUM_ION_V_CELL_NOMINAL_DEFAULT;

                    cli_printfWarning("WARNING: Check if these sodium-ion parameters are OK for your cell!\n");
                    cli_printfWarning("WARNING: The sodium-ion OCV curve in bcc_monitorin.c is not correct yet!\n");
                }
                break;
                default:
                    cli_printfError("ERROR: add new battery type correctly!!\n");
                    break;
            }

            // set the 2 new values and handle the change
            // output to the user
            cli_printfWarning("WARNING: Setting v-storage to %.3f\n", variable1.floatVar);

            // set the overvoltage
            if(setParameterNoLock(V_STORAGE, &variable1.floatVar))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set v-storage\n");
                ret = -1;
                return ret;
            }
            // this variable will not handled in batManagement

            // output to the user
            cli_printfWarning("WARNING: Setting v-cell-nominal to %.3f\n", variable2.floatVar);

            // set the overvoltage
            if(setParameterNoLock(V_CELL_NOMINAL, &variable2.floatVar))
            {
                cli_printfError("handleParamaterChange ERROR: couldn't set v-cell-nominal\n");
                ret = -1;
                return ret;
            }

            // return OK
            ret = 0;

            break;
        case T_MEAS:

            // get the value
            variable1.uint16Var = (*(uint16_t*)value) & UINT16_MAX;

            // check if is not a whole division of T_MEAS_MAX
            if(T_MEAS_MAX % variable1.uint16Var)
            {
                cli_printf(
                    "inserted T_meas: %d not a whole division of %d\n", variable1.uint16Var, T_MEAS_MAX);

                // check if the inserted value is less than 1
                if(variable1.uint16Var < 1)
                {
                    // set the boolvalue to true
                    variable2.boolVar = true;
                }
                else
                {
                    // set the boolvalue to false
                    variable2.boolVar = false;
                }

                // calculate lower measurement ratio
                while(T_MEAS_MAX % variable1.uint16Var)
                {
                    // check if decrease is needed to find the next whole division
                    if(!variable2.boolVar)
                    {
                        // decrease the update ratio
                        variable1.uint16Var--;

                        if(variable1.uint16Var == 0)
                        {
                            // set the value
                            variable2.boolVar = true;

                            // increase once for equation
                            variable1.uint16Var++;
                        }
                    }
                    else
                    {
                        // increase the update ratio
                        variable1.uint16Var++;
                    }
                }

                // output to user
                cli_printf("setting new T_meas: %dms\n", variable1.uint16Var);

                // set the new, faster ratio
                // save the new state of charge
                if(setParameterNoLock(T_MEAS, &variable1.uint16Var))
                {
                    cli_printfError("handleParamaterChange ERROR: couldn't set parameter in %d\n", parameter);
                    return ret;
                }
            }

            // return OK
            ret = 0;

            // variable1 will be used to calculate the new interval

            break;
        default:
            // to return OK
            ret = 0;
            break;
    }

    // callback to main to handle parameter change, without using data_get, data_set functions
    // needed to set for example the new BCC parameter in the BCC, or calc new intervals
    ret |= gParameterChangeCallbackFunctionfp(parameter, value, (void*)&variable1);

    // return
    return ret;
}

/*!
 * @brief   function to get the MCU unique id
 *
 * @param   pointer of data, and size of target data
 *
 * @return  0 if succeeded, negative otherwise
 */
int data_getUniqueid(uintptr_t uniqueid, size_t size)
{
    if(size < CONFIG_BOARDCTL_UNIQUEID_SIZE)
    {
        return -1;
    }

    if(boardctl(BOARDIOC_UNIQUEID, uniqueid) != OK)
    {
        return -2;
    }

    return 0;
}

/*!
 * @brief   function to set the BMS fault
 *
 * @param   New fault variable
 *
 * @return  0 if succeeded, negative otherwise
 */
int data_setBmsFault(uint8_t BMSFault)
{
    int retVal = -1, errorCode;

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: data_setBmsFault pthread_mutex_lock failed %d\n", errorCode);
        return retVal;
    }

    // set the new fault variable
    gBMSFault = BMSFault;

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: data_setBmsFault pthread_mutex_unlock failed %d\n", errorCode);
        return -1;
    }

    return retVal;
}

/*!
 * @brief   function to get the BMS fault
 *
 * @param   none
 *
 * @return  The active BMS fault, negative if error.
 */
int data_getBmsFault(void)
{
    int retVal = -1, errorCode;

    // lock the mutex(with error check)
    if((pthread_mutex_lock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: data_getBmsFault pthread_mutex_lock failed %d\n", errorCode);
        return retVal;
    }

    // get the parameter
    retVal = (int)gBMSFault;

    // unlock the mutex after writing is done
    if((pthread_mutex_unlock(&dataLock)) != 0)
    {
        errorCode = errno;
        cli_printfError("data ERROR: data_getBmsFault pthread_mutex_unlock failed %d\n", errorCode);
        return -1;
    }

    return retVal;
}

// EOF
