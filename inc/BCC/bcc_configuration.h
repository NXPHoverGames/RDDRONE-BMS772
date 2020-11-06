/*
 * Copyright 2019 - 2020 NXP
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
 ** ###################################################################
 **     Filename    : bcc_configuration.h
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
 ** 		Note: SYS_CFG_GLOBAL register contains only command GO2SLEEP (no initialization needed).
 ** 		Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are not initialized.
 **
 ** ###################################################################*/
/*!
 ** @file bcc_configuration.h
 **
 ** @version 01.00
 **
 ** @brief
 **         Battery Cell Controller (BCC) module - configuration file.
 **         This module contains all functions linked to configuration of BCC6 chip. \n
 **
 **         Note: INIT register is initialized automatically by the BCC driver. \n
 ** 		Note: SYS_CFG_GLOBAL register contains only command GO2SLEEP (no initialization needed). \n
 ** 		Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are not initialized. \n
 **
 ** @note
 ** 		This module was adapted from BCC SW examples by C. van Mierlo.
 **
 */

#ifndef BCC_CONFIGURATION_H_
#define BCC_CONFIGURATION_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/* Global */

/* Modules */
#include "bcc.h"
#include "bcc_mc3377x.h"
#include <math.h>

#include "BMS_data_types.h"

/*******************************************************************************
 * Global variable
 ******************************************************************************/
#define BCC_INITIAL_DRIVER_INSTANCE 0
#define BCC_DEVICES                 1
#define BCC_FIRST_INDEX             0
#define BCC_DEFAULT_CELLCNT         6

#define SHUNT_RESISTOR              0.5 //[mOhm] 0.5mOhm
#define SHUNT_RESISTOR_UOHM         SHUNT_RESISTOR * 1000 

/* Number of configurable registers. */
#define REG_CONF_CNT_MC33772        44

/* Precalculate NTC look up table for fast temperature measurement. */
#define NTC_PULL_UP                 10000U  /*!< NTC pull-up 10kOhm */
#define NTC_REF_TEMP                25U     /*!< NTC resistance 10kOhm at 25 degC */
#define NTC_REF_RES                 10000U  /*!< NTC resistance 10kOhm at 25 degC */
#define NTC_BETA                    3900U   /*!< NTC beta value */

/*! @brief Maximal voltage (5V). */
#define NTC_VCOM                    5.0

/*! @brief Resolution of measured voltage in Volts (U = 152.58789 uV *
 *  register_value), with 5V maximal voltage. */
#define NTC_REGISTER_RES            0.00015258789

#define ANX_C_R                     0   /*!< @brief ANx, the Rsense temperature input number */
#define ANX_C_BATT                  1   /*!< @brief ANx, the battery temperature input number */
#define ANX_C_AFE                   2   /*!< @brief ANx, the analog front end temperature input number */
#define ANX_C_T                     3   /*!< @brief ANx, the power switch transistor temperature input number */
#define ANX_V_OUT                   4   /*!< @brief ANx, the Vout/11 voltage input number */

#define BIT_MASK_3_CELL             0b00000111  /*!< @brief these bits represent a 3 cell battery */
#define BIT_MASK_4_CELL             0b00001111  /*!< @brief these bits represent a 4 cell battery */
#define BIT_MASK_6_CELL             0b00111111  /*!< @brief these bits represent a 6 cell battery */

/*! @brief Voltage divider coeficient for Vbatt-out measured value.  */
#define VOLTDIV_BATT_OUT            11

/*******************************************************************************
 * Initial register configuration
 ******************************************************************************/

/* Note: INIT register is initialized automatically by the BCC driver. */
/* Note: SYS_CFG_GLOBAL register contains only command GO2SLEEP (no initialization needed). */
/* Note: EEPROM_CTRL, FUSE_MIRROR_DATA and FUSE_MIRROR_CNTL registers are not initialized. */

/* Initial value of SYS_CFG1 register. */
#define BCC_CONF1_SYS_CFG1_VALUE ( \
    BCC_CYCLIC_TIMER_CONTINOUS /*BCC_CYCLIC_TIMER_0_1S*/ | \
    BCC_DIAG_TIMEOUT_1S | \
    BCC_I_MEAS_ENABLED | \
    BCC_CB_AUTO_PAUSE_DISABLED /*BCC_CB_AUTO_PAUSE_ENABLED*/ | \
    BCC_CB_DRV_DISABLED | \
    BCC_DIAG_MODE_DISABLED | \
    BCC_CB_MAN_PAUSE_DISABLED | \
    BCC_SW_RESET_DISABLED | \
    BCC_FAULT_WAVE_DISABLED | \
    BCC_WAVE_DC_500US \
)

/* Initial value of SYS_CFG2 register. */
#define BCC_CONF1_SYS_CFG2_VALUE ( \
    BCC_FLT_RST_CFG_OSC | \
    BCC_TIMEOUT_COMM_128MS | \
    BCC_ODD_CELLS | \
    BCC_HAMM_ENCOD_DECODE \
)

/* Initial value of ADC_CFG register. */
#define BCC_CONF1_ADC_CFG_VALUE ( \
    /* Note: TAG_ID is zero. */ \
    /* Note: SOC is disable (i.e. do not initiate on-demand conversion now). */ \
    BCC_ADC2_PGA_AUTO | \
    /* Note: CC_RST is not set (do not reset CC now). */ \
    BCC_CHAR_COMP_ENABLED | \
    BCC_ADC1_A_RES_16BIT | \
    BCC_ADC1_B_RES_16BIT | \
    BCC_ADC2_RES_16BIT \
)

/* Initial value of ADC2_OFFSET_COMP register. */
#define BCC_CONF1_ADC2_OFFSET_COMP_VALUE (\
    BCC_READ_CC_RESET | \
    BCC_FREE_CC_CLAMP | \
    BCC_GET_ADC2_OFFSET(0) /* ADC2 offset compensation value. */ \
)

/* Initial value of OV_UV_EN register. */
#define BCC_CONF1_OV_UV_EN_VALUE ( \
    BCC_CTX_OV_TH_COMMON | \
    BCC_CTX_UV_TH_COMMON | \
    /* CTs OV and UV enable (bit is 1) or disable (bit is 0). */ \
    0x003FU/*0x003FU*/ \
)

/* Initial value of CELL_OV_FLT register. */
#define BCC_CONF1_CELL_OV_FLT_VALUE    0x0000U

/* Initial value of CELL_UV_FLT register. */
#define BCC_CONF1_CELL_UV_FLT_VALUE    0x0000U

/* Initial value of CBx_CFG registers. */
#define BCC_CONF1_CB1_CFG_VALUE ( \
    BCC_CB_ENABLED /*BCC_CB_DISABLED*/ | \
    0xFFU /* Cell balance timer in minutes. */ \
)

#define BCC_CONF1_CB2_CFG_VALUE        BCC_CONF1_CB1_CFG_VALUE
#define BCC_CONF1_CB3_CFG_VALUE        BCC_CONF1_CB1_CFG_VALUE
#define BCC_CONF1_CB4_CFG_VALUE        BCC_CONF1_CB1_CFG_VALUE
#define BCC_CONF1_CB5_CFG_VALUE        BCC_CONF1_CB1_CFG_VALUE
#define BCC_CONF1_CB6_CFG_VALUE        BCC_CONF1_CB1_CFG_VALUE

/* Initial value of CB_SHORT_FLT register. */
#define BCC_CONF1_CB_SHORT_FLT_VALUE   0x0000U

/* Initial value of GPIO_CFG1 register. */
#define BCC_CONF1_GPIO_CFG1_VALUE ( \
    BCC_GPIOX_DIG_IN(6U) | \
    BCC_GPIOX_DIG_IN(5U) | \
    BCC_GPIOX_AN_IN_ABS_MEAS(4U) | \
    BCC_GPIOX_AN_IN_RM_MEAS(3U) | \
    BCC_GPIOX_AN_IN_RM_MEAS(2U) | \
    BCC_GPIOX_AN_IN_RM_MEAS(1U) | \
    BCC_GPIOX_AN_IN_RM_MEAS(0U) \
)

/* Initial value of GPIO_CFG2 register. */
#define BCC_CONF1_GPIO_CFG2_VALUE ( \
    BCC_GPIO2_ADC_TRG_DISABLED | \
    BCC_GPIO0_NO_WAKE_UP | \
    BCC_GPIO0_INP_HIGH_FP_NACT \
    /* Note: GPIOx_DR are initialized to zero (low output level). */ \
)

/* Initial value of GPIO_STS register. */
#define BCC_CONF1_GPIO_STS_VALUE       0x0000U

/* Initial value of AN_OT_UT_FLT register. */
#define BCC_CONF1_AN_OT_UT_FLT_VALUE   0x0000U

/* Initial value of GPIO_SHORT_ANx_OPEN_STS register. */
#define BCC_CONF1_GPIO_SHORT_VALUE     0x0000U

/* Initial value of FAULT3_STATUS register. */
#define BCC_CONF1_FAULT1_STATUS_VALUE  0x0000U

/* Initial value of FAULT3_STATUS register. */
#define BCC_CONF1_FAULT2_STATUS_VALUE  0x0000U

/* Initial value of FAULT3_STATUS register. */
#define BCC_CONF1_FAULT3_STATUS_VALUE  0x0000U

#warning maybe more need to be enabled in the fault masks
/* Initial value of FAULT_MASK1 register. */
#define BCC_CONF1_FAULT_MASK1_VALUE ( \
    BCC_VPWR_OV_FLT_DIS | \
    BCC_VPWR_LV_FLT_DIS | \
    BCC_COM_LOSS_FLT_DIS | \
    BCC_COM_ERR_FLT_DIS | \
    BCC_CSB_WUP_FLT_DIS | \
    BCC_GPIO0_WUP_FLT_DIS | \
    BCC_I2C_ERR_FLT_DIS | \
    BCC_IS_OL_FLT_DIS | \
    BCC_IS_OC_FLT_DIS | \
    BCC_AN_OT_FLT_EN | \
    BCC_AN_UT_FLT_EN | \
    BCC_CT_OV_FLT_EN | \
    BCC_CT_UV_FLT_EN \
)

/* Initial value of FAULT_MASK2 register. */
#define BCC_CONF1_FAULT_MASK2_VALUE ( \
    BCC_VCOM_OV_FLT_DIS | \
    BCC_VCOM_UV_FLT_DIS | \
    BCC_VANA_OV_FLT_DIS | \
    BCC_VANA_UV_FLT_DIS | \
    BCC_ADC1_B_FLT_DIS | \
    BCC_ADC1_A_FLT_DIS | \
    BCC_GND_LOSS_FLT_DIS | \
    BCC_AN_OPEN_FLT_DIS | \
    BCC_GPIO_SHORT_FLT_DIS | \
    BCC_CB_SHORT_FLT_DIS | \
    BCC_CB_OPEN_FLT_DIS | \
    BCC_OSC_ERR_FLT_DIS | \
    BCC_DED_ERR_FLT_DIS | \
    BCC_FUSE_ERR_FLT_DIS \
)

#warning set BCC_CC_OVR_FLT_EN and the other faults to enable
/* Initial value of FAULT_MASK3 register. */
#define BCC_CONF1_FAULT_MASK3_VALUE ( \
    BCC_CC_OVR_FLT_EN  | \
    BCC_DIAG_TO_FLT_EN | \
    /* CBx timeout detection (EOT_CBx bits). */ \
    BCC_EOT_CBX_FLT_DIS(1U) |                  /* CB1. */  \
    BCC_EOT_CBX_FLT_DIS(2U) |                  /* CB2. */  \
    BCC_EOT_CBX_FLT_DIS(3U) |                  /* CB3. */  \
    BCC_EOT_CBX_FLT_DIS(4U) |                  /* CB4. */  \
    BCC_EOT_CBX_FLT_DIS(5U) |                  /* CB5. */  \
    BCC_EOT_CBX_FLT_DIS(6U)                    /* CB6. */  \
)

#warning maybe more need to be enabled in the wakeup masks
/* Initial value of WAKEUP_MASK1 register. */
#define BCC_CONF1_WAKEUP_MASK1_VALUE ( \
    BCC_VPWR_OV_WAKEUP_DIS | \
    BCC_VPWR_LV_WAKEUP_DIS | \
    BCC_CSB_WUP_WAKEUP_DIS | \
    BCC_GPIO0_WUP_WAKEUP_DIS | \
    BCC_IS_OC_WAKEUP_EN | \
    BCC_AN_OT_WAKEUP_EN | \
    BCC_AN_UT_WAKEUP_EN | \
    BCC_CT_OV_WAKEUP_EN | \
    BCC_CT_UV_WAKEUP_EN \
)

/* Initial value of WAKEUP_MASK2 register. */
#define BCC_CONF1_WAKEUP_MASK2_VALUE ( \
    BCC_VCOM_OV_WAKEUP_EN | \
    BCC_VCOM_UV_WAKEUP_EN | \
    BCC_VANA_OV_WAKEUP_EN | \
    BCC_VANA_UV_WAKEUP_EN | \
    BCC_ADC1_B_WAKEUP_EN | \
    BCC_ADC1_A_WAKEUP_EN | \
    BCC_GND_LOSS_WAKEUP_EN | \
    BCC_IC_TSD_WAKEUP_EN | \
    BCC_GPIO_SHORT_WAKEUP_EN | \
    BCC_CB_SHORT_WAKEUP_EN | \
    BCC_OSC_ERR_WAKEUP_EN | \
    BCC_DED_ERR_WAKEUP_EN \
)

/* Initial value of WAKEUP_MASK3 register. */
#define BCC_CONF1_WAKEUP_MASK3_VALUE ( \
    BCC_CC_OVR_FLT_EN | \
    BCC_DIAG_TO_FLT_DIS | \
    /* CBx timeout detection (EOT_CBx bits). */ \
    BCC_EOT_CBX_WAKEUP_EN |                  /* CB1. */  \
    BCC_EOT_CBX_WAKEUP_EN |                  /* CB2. */  \
    BCC_EOT_CBX_WAKEUP_EN |                  /* CB3. */  \
    BCC_EOT_CBX_WAKEUP_EN |                  /* CB4. */  \
    BCC_EOT_CBX_WAKEUP_EN |                  /* CB5. */  \
    BCC_EOT_CBX_WAKEUP_EN                    /* CB6. */  \
)


/* Initial value of TH_ALL_CT register. */
#define BCC_CONF1_TH_ALL_CT_VALUE ( \
    BCC_SET_ALL_CT_OV_TH(4195U) /*BCC_SET_ALL_CT_OV_TH(1510U)   CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
    BCC_SET_ALL_CT_UV_TH(2509U) /*BCC_SET_ALL_CT_UV_TH(1000U)   CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)

/* Initial value of TH_CTx registers. */
#define BCC_CONF1_TH_CT1_VALUE ( \
    BCC_SET_CTX_OV_TH(4195U)  /* CT OV threshold is 4195 mV. It is enabled/disabled through OV_UV_EN register. */ | \
    BCC_SET_CTX_UV_TH(2509U)  /* CT UV threshold is 2509 mV. It is enabled/disabled through OV_UV_EN register. */ \
)

#define BCC_CONF1_TH_CT2_VALUE         BCC_CONF1_TH_CT1_VALUE
#define BCC_CONF1_TH_CT3_VALUE         BCC_CONF1_TH_CT1_VALUE
#define BCC_CONF1_TH_CT4_VALUE         BCC_CONF1_TH_CT1_VALUE
#define BCC_CONF1_TH_CT5_VALUE         BCC_CONF1_TH_CT1_VALUE
#define BCC_CONF1_TH_CT6_VALUE         BCC_CONF1_TH_CT1_VALUE

/* Initial value of TH_ANx_OT registers. */
#define BCC_CONF1_TH_AN0_OT_VALUE ( \
    BCC_SET_ANX_OT_TH(0)  /* AN OT threshold is 1160 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)

#define BCC_CONF1_TH_AN1_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE
#define BCC_CONF1_TH_AN2_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE
#define BCC_CONF1_TH_AN3_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE
#define BCC_CONF1_TH_AN4_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE
#define BCC_CONF1_TH_AN5_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE
#define BCC_CONF1_TH_AN6_OT_VALUE      BCC_CONF1_TH_AN0_OT_VALUE

/* Initial value of TH_ANx_UV registers. */
#define BCC_CONF1_TH_AN0_UT_VALUE ( \
    BCC_SET_ANX_UT_TH(4900)  /* AN UT threshold is 3820 mV. It is enabled/disabled through FAULT_MASK1 register. */ \
)

#define BCC_CONF1_TH_AN1_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE
#define BCC_CONF1_TH_AN2_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE
#define BCC_CONF1_TH_AN3_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE
#define BCC_CONF1_TH_AN4_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE
#define BCC_CONF1_TH_AN5_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE
#define BCC_CONF1_TH_AN6_UT_VALUE      BCC_CONF1_TH_AN0_UT_VALUE

/* Initial value of TH_ISENSE_OC register. */
#define BCC_CONF1_TH_ISENSE_OC_VALUE ( \
    /* ISENSE OC threshold is 24576 mA (2458 uV using 100 uOhm resistor). It is enabled/disabled through FAULT_MASK1 and WAKEUP_MASK1 register. */ \
    BCC_SET_TH_ISENSE_OC(I_SLEEP_OC_DEFAULT*SHUNT_RESISTOR)/*2458U)*/ \
)

/* Initial value of TH_COULOMB_CNT_MSB register. */
#define BCC_CONF1_TH_COULOMB_CNT_MSB_VALUE ( \
    0x7FFF/*BCC_SET_TH_COULOMB_CNT_MSB(0x7FFF)  Higher 16 bits of over Coulomb threshold (2's complement representation). */ \
)

/* Initial value of TH_COULOMB_CNT_LSB register. */
#define BCC_CONF1_TH_COULOMB_CNT_LSB_VALUE ( \
  0xFFFF /* BCC_SET_TH_COULOMB_CNT_LSB(0xFFFF) Lower 16 bits of over Coulomb threshold (2's complement representation). */ \
)

/*******************************************************************************
 * Global variable
 ******************************************************************************/

/*******************************************************************************
 * Initial BCC configuration
 ******************************************************************************/

/*! @brief  Initial configuration of Battery Cell Controller devices. 
 *          see BCC_INIT_CONF_REG_ADDR from bcc.c for more information 
 */
static const uint16_t BCC_INIT_CONF[1][BCC_INIT_CONF_REG_CNT] = {
    {
        BCC_CONF1_GPIO_CFG1_VALUE,          /*!< The GPIO_CFG1 register programs the individual GPIO port as a ratiometric, single ended, input or output port.*/
        BCC_CONF1_GPIO_CFG2_VALUE,          /*!< on GPIO0 disable trigger, no wake-up, does not activate fault pin. and GPIOx could be set high or low */
        BCC_CONF1_TH_ALL_CT_VALUE,          /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for all cell terminals */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        BCC_CONF1_TH_CT6_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_CT5_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_CT4_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_CT3_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_CT2_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_CT1_VALUE,             /*!< 2509 - 4195 mv overvoltage and undervoltage threshold for this cell terminal */
        BCC_CONF1_TH_AN6_OT_VALUE,          /*!< not used contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN5_OT_VALUE,          /*!< not used contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN4_OT_VALUE,          /*!< V_sense out battery output/11 contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN3_OT_VALUE,          /*!< AFE_temperature (analog front end) contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN2_OT_VALUE,          /*!< T_temperature (power switch) contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN1_OT_VALUE,          /*!< BATT_temperature (battery) contain the individually programmed overtemperature value for the analog input.*/
        BCC_CONF1_TH_AN0_OT_VALUE,          /*!< R_temperature (Rsense) contain the individually programmed overtemperature value for the analog input. */
        BCC_CONF1_TH_AN6_UT_VALUE,          /*!< contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN5_UT_VALUE,          /*!< contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN4_UT_VALUE,          /*!< V_sense out battery output/11 contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN3_UT_VALUE,          /*!< T_temperature (power switch) contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN2_UT_VALUE,          /*!< AFE_temperature (analog front end) T_temperature (power switch) contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN1_UT_VALUE,          /*!< BATT_temperature (battery) contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_AN0_UT_VALUE,          /*!< R_temperature (Rsense) contain the individually programmed undertemperature value for the analog input.*/
        BCC_CONF1_TH_ISENSE_OC_VALUE,       /*!< Registers TH_ISENSE_OC contains the programmed overcurrent threshold in sleep mode. should be set with the I*Rshunt (uV) */
        BCC_CONF1_TH_COULOMB_CNT_MSB_VALUE, /*!< 0xEFFF The coulomb counter threshold in sleep mode is given by these two registers*/
        BCC_CONF1_TH_COULOMB_CNT_LSB_VALUE, /*!< 0xEFFF The coulomb counter threshold in sleep mode is given by these two registers*/
        BCC_CONF1_CB1_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        BCC_CONF1_CB2_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        BCC_CONF1_CB3_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        BCC_CONF1_CB4_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        BCC_CONF1_CB5_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        BCC_CONF1_CB6_CFG_VALUE,            /*!< CB on and 0xFF The cell balance configuration register holds the operating parameters of the cell balance output drivers.*/
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        0x0000,                             /*!< the MC33772 only has 6 cells */
        BCC_CONF1_OV_UV_EN_VALUE,           /*!< OV and UV to common and all TH on.*/
        BCC_CONF1_SYS_CFG1_VALUE,           /*!< cyclic cont. and diag 1s, I_meas enabled CB disabled and auto pause, diag, CB man pause, soft rst and fault wave (500us) disabled The SYS_CFG1 register contains control bits and register settings that allow the user to adapt the 33772 to specific applications and system requirements. */
        BCC_CONF1_SYS_CFG2_VALUE,           /*!< OSC fault monitoring and reset enabled, BCC timeout com 128 ms, even number of cells!, DED HAMM Decode*/
        BCC_CONF1_ADC_CFG_VALUE,            /*!< tag id 0, SOC disable (dont activate on demand conversion now), automatic PGA, no CC rst (dont rst coulomb counter now), set all ADC res to 16 bit */
        BCC_CONF1_ADC2_OFFSET_COMP_VALUE,   /*!< read and reset CC, CC clamp to max/min, no ADC2 offset*/
        BCC_CONF1_FAULT_MASK1_VALUE,        /*!< all faults are enabled to activate the fault pin*/
        BCC_CONF1_FAULT_MASK2_VALUE,        /*!< all faults are enabled to activate the fault pin*/
        BCC_CONF1_FAULT_MASK3_VALUE,        /*!< all faults are enabled to activate the fault pin*/
        BCC_CONF1_WAKEUP_MASK1_VALUE,       /*!< enable all wake-up events*/
        BCC_CONF1_WAKEUP_MASK2_VALUE,       /*!< enable all wake-up events*/
        BCC_CONF1_WAKEUP_MASK3_VALUE,       /*!< enable all wake-up events except BCC_DIAG_TO_FLT_DIS */
        BCC_CONF1_CELL_OV_FLT_VALUE,        /*!< 0 reset the register*/
        BCC_CONF1_CELL_UV_FLT_VALUE,        /*!< 0 reset the register*/
        BCC_CONF1_AN_OT_UT_FLT_VALUE,       /*!< 0 reset the register*/
        BCC_CONF1_CB_SHORT_FLT_VALUE,       /*!< 0 reset the register The cell balance short detection register holds the cell balance shorted load status.*/
        BCC_CONF1_GPIO_STS_VALUE,           /*!< 0 reset the register*/
        BCC_CONF1_GPIO_SHORT_VALUE,         /*!< 0 reset the register*/
        BCC_CONF1_FAULT1_STATUS_VALUE,      /*!< 0 reset the register*/
        BCC_CONF1_FAULT2_STATUS_VALUE,      /*!< 0 reset the register*/
        BCC_CONF1_FAULT3_STATUS_VALUE,      /*!< 0 reset the register*/
    },
};

/*******************************************************************************
 * Function prototypes
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
    uint8_t ANxbits, float *lowerTH, float *upperTH);

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
    uint8_t ANxbits, bool disable);

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
    uint8_t ANxbits, uint16_t *lowerTH, uint16_t *upperTH);

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
    float *lowerTH, float *upperTH);
    //uint8_t cellBits, float *lowerTH, float *upperTH);

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
    uint8_t sleepCurrentmA);

/*!
 * @brief   This function is used to configure the CYCLIC_TIMER time
 *
 * @warning this should be called after the initialization
 *
 * @param   drvConfig Pointer to driver instance configuration.
 * @param   cid Cluster Identification Address.
 * @param   newTimeS [s] the new cyclic timer value (can be 1, 2, 4 or 8s) will be rounded down
 *          if it is not one of those
 *
 * @return  BCC error status.
 */
 bcc_status_t bcc_configuration_changeCyclicTimer(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t newTimeS);

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
    uint8_t newCellCount);

/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* BCC_CONFIGURATION_H_ */
