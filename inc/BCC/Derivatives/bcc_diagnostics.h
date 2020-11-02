/*
 * Copyright 2016 - 2019 NXP
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
 */

/*!
 * @file bcc_diagnostics.h
 *
 * Diagnostics part of Battery cell controller SW driver V1.1.
 * Supports boards based on MC33771B and MC33772B.
 *
 * This module is common for all supported models.
 */

#ifndef __BCC_DIAGNOSTICS_H__
#define __BCC_DIAGNOSTICS_H__

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc.h"

/*******************************************************************************
 * User definitions
 ******************************************************************************/

/*! @brief Number of averaged cell voltage measurements in Cell voltage channel
 *  functional verification. BCC_DIAG_CVFV_MEAS_NUM should be at least 6U
 *  according to the safety manual. */
#define BCC_DIAG_CVFV_MEAS_NUM    6U

/*! @brief Number of averaged cell voltage measurements in Cell Terminal Leakage
 *  Diagnostics. BCC_DIAG_LEAK_MEAS_NUM should be at least 4 according to the
 *  safety manual. */
#define BCC_DIAG_LEAK_MEAS_NUM    4U

/*! @brief Minimal measured voltage [uV] of contacts with abnormally high
 *  resistance. To avoid false alarms the following constraint shall be met:
 *  BCC_DIAG_DV >= R_connector(max) * I_cell_balance(max). */
#define BCC_DIAG_DV               30000

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief R_PD (CT open load detection pull-down resistor) resistance
 *  in [Ohm]. */
#define BCC_RPD                   950U

/** Diagnostic thresholds. **/
/*! @brief ISENSE diagnostic common mode offset voltage in [uV],
 *  maximal value. */
#define BCC_DIAG_VOFF_MAX         37U
/*! @brief MC33771 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4. */
#define BCC_DIAG_VREF_MC33771     127000
/*! @brief MC33771 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4, minimal value. */
#define BCC_DIAG_VREF_MIN_MC33771 124000
/*! @brief MC33771 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4, maximal value. */
#define BCC_DIAG_VREF_MAX_MC33771 130000
/*! @brief MC33772 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4. */
#define BCC_DIAG_VREF_MC33772     126500
/*! @brief MC33772 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4, minimal value. */
#define BCC_DIAG_VREF_MIN_MC33772 123500
/*! @brief MC33772 ISENSE diagnostic reference in [uV]
 *  with PGA having gain 4, maximal value. */
#define BCC_DIAG_VREF_MAX_MC33772 129500

/*! @brief Cell terminal open load V detection threshold [uV],
 *  Type T (1.5 V <= V_CELL <= 2.7 V). */
#define BCC_DIAG_V_OL_DETECT_T    50000U
/*! @brief Cell terminal open load V detection threshold [uV],
 *  Type F (2.5 V <= V_CELL <= 3.7 V). */
#define BCC_DIAG_V_OL_DETECT_F    100000U
/*! @brief Cell terminal open load V detection threshold [uV],
 *  Type N (2.5 V <= V_CELL <= 4.3 V). */
#define BCC_DIAG_V_OL_DETECT_N    150000U
/*! @brief Undervoltage functional verification threshold [mV],
 *  Type T (1.5 V <= V_CELL <= 2.7 V). */
#define BCC_DIAG_CTX_UV_TH_T      390U
/*! @brief Undervoltage functional verification threshold [mV],
 *  Type F (2.5 V <= V_CELL <= 3.7 V). */
#define BCC_DIAG_CTX_UV_TH_F      650U
/*! @brief Undervoltage functional verification threshold [mV],
 *  Type N (2.5 V <= V_CELL <= 4.3 V). */
#define BCC_DIAG_CTX_UV_TH_N      1200U
/*! @brief Overvoltage functional verification threshold [mV],
 *  Type T (1.5 V <= V_CELL <= 2.7 V). */
#define BCC_DIAG_CTX_OV_TH_T      1800U
/*! @brief Overvoltage functional verification threshold [mV],
 *  Type F (2.5 V <= V_CELL <= 3.7 V). */
#define BCC_DIAG_CTX_OV_TH_F      4000U
/*! @brief Overvoltage functional verification threshold [mV],
 *  Type N (2.5 V <= V_CELL <= 4.3 V). */
#define BCC_DIAG_CTX_OV_TH_N      4000U

/*! @brief Number of measurements in ADC1-A and ADC1-B functional verification. */
#define BCC_DIAG_ADC1_MEAS_NUM    6U
/*! @brief Voltage reference [uV] used in ADC1-A,B functional verification. */
#define BCC_DIAG_V_BGP            1180000U
/*! @brief Maximum tolerance [uV] between ADC1-A, B and diagnostic reference
 *  (1.5 V <= VCELL <= 4.3 V) in ADC1-A,B functional verification, absolute value. */
#define BCC_DIAG_ADC1X_FV         5250U

/*! @brief Cell voltage channel functional verification allowable error
 *  in CT verification measurement, minimal value in [uV]. */
#define BCC_DIAG_VCVFV_MIN        -22000
/*! @brief Cell voltage channel functional verification allowable error
 *  in CT verification measurement, maximal value in [uV]. */
#define BCC_DIAG_VCVFV_MAX        6000

/*! @brief Waiting time in [ms] in Cell Terminal Leakage diagnostics containing
 *  the cell balance delay and cell terminal settling time. */
#define BCC_DIAG_TCT_SETTLE       5
/*! @brief MC33771 Cell terminal leakage detection level in [uV].
 *  This value can be used also as negative. */
#define BCC_DIAG_VLEAK_MC33771    27000U
/*! @brief MC33772 Cell terminal leakage detection level in [uV].
 *  This value can be used also as negative. */
#define BCC_DIAG_VLEAK_MC33772    15000U
/*! @brief ISENSE Open Load injected current [uA]. */
#define BCC_DIAG_ISENSE_OL        130U
/*! @brief ISENSE Open Load detection threshold [uV]. */
#define BCC_DIAG_V_ISENSE_OL      460000U
/*! @} */

/* Enum types definition. */
/*!
 * @addtogroup enum_group
 * @{
 */
/*! @brief Selection between Cell terminal and Cell balancing diagnostic
    switches. */
typedef enum
{
    BCC_SWITCH_SEL_CT         = 0U, /*!< Cell terminal switches. */
    BCC_SWITCH_SEL_CB         = 1U  /*!< Cell balancing switches. */
} bcc_diag_switch_sel_t;

/*! @brief Selection between opened and closed diagnostic switches. */
typedef enum
{
    BCC_SWITCH_POS_OPEN       = 0U,  /*!< Opened switches. */
    BCC_SWITCH_POS_CLOSED     = 1U   /*!< Closed switches. */
} bcc_diag_switch_pos_t;

/*! @brief Selection of diagnostic type and source of ADC2 for Current
 *  measurement diagnostics. */
typedef enum
{
    BCC_DCM_AMP_INP_GND       = 0U,  /*!< Diagnostic of measurement chain offset,
                                          amplifier inputs are grounded. */
    BCC_DCM_VREF_GAIN4        = 1U,  /*!< Diagnostic of measurement chain with a gain 4,
                                          ADC is set to calibrated internal reference. */
    BCC_DCM_AN5AN6            = 2U   /*!< Diagnostic of external open and short or
                                          leaking devices, ADC is set to GPIO5 and GPIO6. */
} bcc_diag_current_meas_t;

/*! @brief Battery type. */
typedef enum
{
    BCC_BATT_T                = 0U, /*!< Type T (1.5 V <= V_CELL <= 2.7 V). */
    BCC_BATT_F                = 1U, /*!< Type F (2.5 V <= V_CELL <= 3.7 V). */
    BCC_BATT_N                = 2U  /*!< Type N (2.5 V <= V_CELL <= 4.3 V). */
} bcc_battery_type_t;

/*! @} */

/* Configure struct types definition. */
/*!
 * @addtogroup struct_group
 * @{
 */
/*!
 * @brief Diagnostic time constants.
 */
typedef struct {
    uint32_t tau;             /*!< Measurement time constant tau (in [us]).
                                   See equations 1 - 2 in MC3377x datasheet. */
    uint32_t tauDiag;         /*!< Tau_diag time constant (in [us]).
                                   See equations 6 - 11 in MC3377x datasheet. */
    uint32_t tauDiagN;        /*!< Tau_diag,n time constant (in [us])
                                   See equations 3 - 5 in MC33771x datasheet. */
    uint32_t tauI;            /*!< Current measurement time constant tau_I (in [us]).
                                   See equation 12 in MC33771x datasheet. */
    uint32_t tDiag;           /*!< Diagnostic time t_diag (in [us]) to detect an
                                   open from the shunt to the current filter.
                                   See equation 14 in MC33771x datasheet. */
} bcc_diag_const_t;

/*!
 * @brief Result of ADC1-A and ADC1-B functional verification.
 */
typedef struct
{
    uint32_t adc1aAvg;        /*!< Average of ADC1-A measured values in [uV]. */
    uint32_t adc1bAvg;        /*!< Average of ADC1-B measured values in [uV]. */
    bool error;               /*!< True if error detected, False otherwise. */
} bcc_diag_adc1x_res_t;

/*!
 * @brief Result of overvoltage and undervoltage functional verification.
 */
typedef struct
{
    uint16_t ovOdd;           /*!< Content of CELL_OV_FLT register, OV fault is expected on odd cells. */
    uint16_t uvEven;          /*!< Content of CELL_UV_FLT register, UV fault is expected on even cells. */
    uint16_t ovEven;          /*!< Content of CELL_OV_FLT register, OV fault is expected on even cells. */
    uint16_t uvOdd;           /*!< Content of CELL_UV_FLT register, UV fault is expected on odd cells. */
    bool error;               /*!< True if error detected, False otherwise. */
} bcc_diag_ov_uv_res_t;

/*!
 * @brief Result of the CTx open detect and open detect functional verification.
 */
typedef struct
{
    uint32_t measPreClosure[BCC_MAX_CELLS]; /*!< Measured cell voltages in [uV] before closing any
                                                 CT open terminal switch.
                                                 MC33771: [0] CT14, .., [13] CT1.
                                                 MC33772: [0] CT6, .., [5] CT1. */
    uint32_t measOddClosed[BCC_MAX_CELLS];  /*!< Measured cell voltages in [uV] when odd CT open
                                                 terminal switches are closed.
                                                 MC33771: [0] CT14, .., [13] CT1.
                                                 MC33772: [0] CT6, .., [5] CT1. */
    uint32_t measEvenClosed[BCC_MAX_CELLS]; /*!< Measured cell voltages in [uV] when even CT open
                                                 terminal switches are closed.
                                                 MC33771: [0] CT14, .., [13] CT1.
                                                 MC33772: [0] CT6, .., [5] CT1. */
    uint16_t ctxOpen;                       /*!< Bit map representing open terminal status.
                                                 MC33771: 0th bit: CT1, ..., 13th bit: CT14.
                                                 MC33772: 0th bit: CT1, ..., 5th bit: CT6.
                                                 Bit value 0: normal condition. Bit value 1: open condition.
                                                 If ctxOpen is zero, no CTx was detected open.
                                                 Faults at CTx of unused cells are ignored. */
} bcc_diag_ctx_open_res_t;

/*!
 * @brief Result of the cell voltage channel functional verification.
 */
typedef struct {
    uint32_t measCellVolt[BCC_MAX_CELLS];  /*!< Measurements of diagnostics cell voltages in [uV].
                                                MC33771: [0] CT14, .., [13] CT1.
                                                MC33772: [0] CT6, .., [5] CT1. */
    int32_t vErrX[BCC_MAX_CELLS - 2];      /*!< Computed errors V_err_x in [uV].
                                                MC33771: [0] V_err_3, .., [11] V_err_14.
                                                MC33772: [0] V_err_3, [1] V_err_5. */
    uint16_t result;                       /*!< Bit map representing errors detected in V_err_x.
                                                 MC33771: 0th bit: V_err_3, .., 11th bit: V_err_14.
                                                 MC33772: 0th bit: V_err_3, 1st bit: V_err_5.
                                                 Bit value 0: OK. Bit value 1: Error detected.
                                                 If result is zero, no error was detected. */
} bcc_diag_cell_volt_res_t;

/*!
 * @brief Result of the cell terminal leakage diagnostics.
 */
typedef struct
{
    uint16_t conResistance;                  /*!< Bit map representing connectors having
                                                  abnormally high contact resistance.
                                                  MC33771: 0th bit: connector 1, .., 13th bit: connector 14.
                                                  MC33772: 0th bit: connector 1, .., 5th bit: connector 6.
                                                  Bit value 0: Normal resistance. Bit value 1: High resistance.
                                                  If conResistance is zero, no error was detected. */
    uint32_t vleak_avx[BCC_MAX_CELLS + 1U];  /*!< Average of Vleak_x in [uV].
                                                  MC33771: [0] CT_REF (Vleak_av1), [1] CT1 (Vleak1), ..., [14] CT14 (Vleak_av15).
                                                  MC33772: [0] CT_REF (Vleak_av1), [1] CT1 (Vleak1), ..., [6] CT6 (Vleak_av7). */
    uint16_t leakStatus;                     /*!< Bit map representing leakage status on CTx terminals.
                                                  MC33771: 0th bit: CT_REF, 1st bit: CT_1, .., 14th bit: CT14.
                                                  MC33772: 0th bit: CT_REF, 1st bit: CT_1, .., 6th bit: CT6.
                                                  Bit value 0: CT not leaky. Bit value 1: CT is leaky.
                                                  If leakStatus is zero, no error was detected. */
} bcc_diag_ctx_leak_res_t;

/*!
 * @brief Result of GPIOx OT/UT functional verification.
 */
typedef struct
{
    uint16_t untStat;      /*!< Contains value of AN_OT_UT_FLT register when
                                under-temperature is expected for all GPIOs). */
    uint16_t ovtStat;      /*!< Contains value of AN_OT_UT_FLT register when
                                over-temperature is expected for all GPIOs). */
    bool error;            /*!< True if error detected, False otherwise. */
} bcc_diag_gpiox_otut_res_t;

/*!
 * @brief Result of the cell balance fault diagnostics.
 */
typedef struct
{
    uint16_t cbxOpenStatusEven;  /*!< Contains CB_OPEN_FLT register when even
                                      CB open detection switches are closed. */
    uint16_t cbxOpenStatusOdd;   /*!< Contains CB_OPEN_FLT register when odd
                                      CB open detection switches are closed. */
    bool error;                  /*!< True if error detected, False otherwise.
                                      Faults at CBx of unused cells are ignored. */
} bcc_diag_cbx_open_res_t;
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */
/*!
 * @brief This function implements the ADC1-A and ADC1-B functional verification.
 * Six on-demand conversions are performed and measured values from
 * MEAS_VBG_DIAG_ADC1A and MEAS_VBG_DIAG_ADC1B registers are compared with the
 * voltage reference.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_ADC1(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_diag_adc1x_res_t *results);

/*!
 * @brief This function implements OV/UV functional verification through digital
 * comparators against tunable thresholds. This can be done by forcing an OV/UV
 * condition on cell terminal pins with use of diagnostic switches.
 *
 * Note that OV/UV thresholds are temporary changed to CTx_OV_TH and CTx_UV_TH
 * values from BCC datasheets according to the battery type.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param battType Battery type.
 * @param diagTimeConst Pointer to structure with diagnostic time constants.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_OvUv(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_battery_type_t battType, const bcc_diag_const_t* const diagTimeConst,
    bcc_diag_ov_uv_res_t* results);

/*!
 * @brief This function implements CTx open detection and functional
 * verification. Open detection is achieved by taking an ADC reading of the cell
 * terminal voltages before and after closing the open detection switches.
 * Comparison of measured values with expected results gives status of cell
 * terminals. It checks normal condition and open condition (not the CTx
 * terminated and SWx Failed Short, CTx terminated and SWx Failed Open).
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param battType Battery type.
 * @param diagTimeConst Pointer to structure with diagnostic time constants.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_CTxOpen(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_battery_type_t battType, const bcc_diag_const_t* const diagTimeConst,
    bcc_diag_ctx_open_res_t* results);

/*!
 * @brief This function implements Cell Voltage Channel functional verification.
 * Purpose of this verification is to check that gain variations introduced by
 * multiplexers (used to route CTx pins to ADC1-A,B) are small compared to the
 * unity. The diagnostic disconnects the cell terminal input circuitry and
 * places a precision zener reference on the input to each differential
 * amplifier attenuator to verify the integrity of the level shifting
 * differential amplifier, attenuator and multiplexer chain. Unused cell
 * voltage channels are skipped.
 *
 * See the datasheet for assumption of minimum cell voltage for this
 * verification.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_CellVolt(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_diag_cell_volt_res_t* results);

/*!
 * @brief This function implements Cell Terminal Leakage diagnostic.
 * The present safety mechanism is made up of two procedures. One of them, is
 * thought to detect a connector having an abnormally high contact resistance
 * The other is to detect cell terminals and cell balancing terminals
 * leakage current. Leakage detection is achieved by taking an ADC
 * reading of the cell terminals referenced to cell balance terminals. Inverted
 * and non-inverted measurement is used to detect possible current sourcing or
 * sinking.
 *
 * Note that this function disables all CB drivers. If CB was utilised before
 * calling of this function, it needs to be set-up again.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_CellLeak(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_diag_ctx_leak_res_t* results);

/*!
 * @brief This function implements current measurement diagnostics. It verifies
 * integrity of current measurement chain. It consists of three different
 * diagnostic types, namely: Amplifier inputs grounded (measurement chain
 * offset), VREF_DIAG reference (measurement chain with known reference and
 * a gain of 4) and GPIO5, GPIO6 (external open and short or leaking devices).
 *
 * Note that this diagnostic resets the coulomb counter. Read coulomb counter
 * COULOMB_CNT to retain count information before calling this function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param sel Selection between different diagnostic types related to
 *            current measurement. See definition of this enumeration
 *            in bcc.h header file.
 * @param current Measured ISENSE voltage in [uV].
 * @param fault True (faulted condition) / false (un-faulted condition).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_CurrentMeas(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bcc_diag_current_meas_t sel, int32_t* current, bool* fault);

/*!
 * @brief This function verifies whether the shunt resistor is properly
 * connected to the current channel low-pass filter.
 *
 * Note that this diagnostic resets the coulomb counter. Read coulomb counter
 * COULOMB_CNT to retain count information before calling this function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param diagTimeConst Pointer to structure with diagnostic time constants.
 * @param shuntConn True (shunt resistor is connected) / false (not connected).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_ShuntConn(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, const bcc_diag_const_t* const diagTimeConst, bool* shuntConn);

/*!
 * @brief This function implements GPIOx OT/UT functional verification. All
 * GPIOs are forced to analog input RM mode. Driving pin to low/high simulates
 * OT/UT condition. Note that programmed OT/UT thresholds are used to verify
 * functionality.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file above.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_GPIOxOtUt(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bcc_diag_gpiox_otut_res_t* results);

/*!
 * @brief This function implements GPIOx open terminal diagnostics. To detect
 * open terminals on the GPIO pins, a weak internal pull-down is commanded ON
 * and OFF. Voltages below the VOL(TH) threshold are considered open terminals.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param openStatus Open terminal status for each GPIOx terminal (ANx_OPEN bits
 *                   in OPEN GPIO_SHORT_ANx_OPEN_STS register). If the value is
 *                   zero, no GPIOx terminal is opened.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_GPIOxOpen(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t* openStatus);

/*!
 * @brief This function implements Cell balance open load detection. To detect
 * open load on the cell balance terminals, Rpd_cb resistor is applied between
 * the CBx outputs and their common terminal. Voltages below the Vout(FLT_TH)
 * activate the CB_OPEN_FLT register bits. Note that results for short detection
 * are not part of this diagnostic. It is diagnosed continuously with the cell
 * balance FET active.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param results Pointer to structure with results of diagnostic. See
 *                definition of this structure in this header file above.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Diag_CBxOpen(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bcc_diag_cbx_open_res_t* results);

/*! @} */

#endif /* __BCC_DIAGNOSTICS_H__ */
/*******************************************************************************
 * EOF;
 ******************************************************************************/
