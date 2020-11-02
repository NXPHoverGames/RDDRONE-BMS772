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
 * @file bcc_mc3377x.h
 *
 * This header file contains register map for the MC33771B and MC33772B
 * Battery Cell Controllers.
 */

#ifndef BCC_MC3377X_H_
#define BCC_MC3377X_H_

/* General bit operations. */

/**
 * Macro for setting value to 1 of the bit given by the mask.
 *
 * @param reg Register to be modified.
 * @param mask Bit selection in register.
 */
#define BCC_REG_SET_BIT_VALUE(reg, mask) ((reg) | (mask))

/**
 * Macro for setting value to 0 of the bit given by the mask.
 *
 * @param reg Register to be modified.
 * @param mask Bit selection in register.
 */
#define BCC_REG_UNSET_BIT_VALUE(reg, mask) ((reg) & ~(mask))

/**
 * Macro for getting value of the bit given by the mask.
 *
 * @param reg Register to be read.
 * @param mask Bit selection in register.
 * @param shift Bit shift in register.
 */
#define BCC_REG_GET_BIT_VALUE(reg, mask, shift) (((reg) & (mask)) >> (shift))

/**
 * Macro for setting value of bits given by the mask.
 *
 * @param reg Register value to be modified.
 * @param mask Bits selection in register.
 * @param shift Bits shift in register.
 * @param val Value to be applied.
 * @param range Admissible range of value.
 */
#define BCC_REG_SET_BITS_VALUE(reg, mask, shift, val, range) \
    (((reg) & ~(mask)) | (((val) & (range)) << (shift)))

/**
 * Macro for getting value of bits given by the mask.
 *
 * @param reg Register to be read.
 * @param mask Bits selection in register.
 * @param shift Bits shift in register.
 */
#define BCC_REG_GET_BITS_VALUE(reg, mask, shift) (((reg) & (mask)) >> (shift))

/* Address of last register. */
#define BCC_MAX_REG_ADDR       0x7FU

/* Maximal address of fuse mirror data. */
#define BCC_MAX_FUSE_ADDR      0x1FU

/* Maximal address of EEPROM data. */
#define BCC_MAX_EEPROM_ADDR    0x7FU

/******************************************************************************/
/* $01 INIT - Device initialization. */
/******************************************************************************/
#define BCC_REG_INIT_ADDR       0x01U
#define BCC_REG_INIT_DEFAULT    0x0000U

#define BCC_RW_CID_MASK         0x000FU
#define BCC_RW_BUS_SW_MASK      0x0010U
#define BCC_RW_RTERM_MASK       0x0020U

#define BCC_RW_CID_SHIFT        0x00U
#define BCC_RW_BUS_SW_SHIFT     0x04U
#define BCC_RW_RTERM_SHIFT      0x05U

/* Bus switch control. */
#define BCC_BUS_SWITCH_DISABLED (0x00U << BCC_RW_BUS_SW_SHIFT)
#define BCC_BUS_SWITCH_ENABLED  (0x01U << BCC_RW_BUS_SW_SHIFT)

/* Control of the internal termination resistor connection to the communication bus. */
#define BCC_RTERM_COMM_SW    (0x00U << BCC_RW_RTERM_SHIFT) /* Depends on the status of the communication switch. */
#define BCC_RTERM_CONNECTED  (0x01U << BCC_RW_RTERM_SHIFT) /* Connected regardless of the bus switch status. */

/**
 * Sets cluster identifier.
 *
 * @param reg Register value to be modified.
 * @param cid Cluster Identifier [0x00 - 0x0F].
 */
#define BCC_SET_CID(reg, cid) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_CID_MASK, BCC_RW_CID_SHIFT, cid, 0x0FU)

/******************************************************************************/
/* $02 SYS_CFG_GLOBAL - Global system configuration. */
/******************************************************************************/
#define BCC_REG_SYS_CFG_GLOBAL_ADDR    0x02U
#define BCC_REG_SYS_CFG_GLOBAL_DEFAULT 0x0000U

#define BCC_W_GO2SLEEP_MASK            0x0001U
#define BCC_W_GO2SLEEP_SHIFT           0x00U

/* Go to SLEEP command. */
#define BCC_GO2SLEEP_DISABLED          (0x00U << BCC_W_GO2SLEEP_SHIFT)
#define BCC_GO2SLEEP_ENABLED           (0x01U << BCC_W_GO2SLEEP_SHIFT)

/******************************************************************************/
/* $03 SYS_CFG1 - System configuration. */
/******************************************************************************/
#define BCC_REG_SYS_CFG1_ADDR        0x03U
#define BCC_REG_SYS_CFG1_DEFAULT     0x9001U

#define BCC_RW_WAVE_DC_MASK          0x0006U
#define BCC_RW_FAULT_WAVE_MASK       0x0008U
#define BCC_W_SOFT_RST_MASK          0x0010U
#define BCC_RW_CB_MANUAL_PAUSE_MASK  0x0020U
#define BCC_W_GO2DIAG_MASK           0x0040U
#define BCC_R_DIAG_ST_MASK           0x0040U
#define BCC_RW_CB_DRVEN_MASK         0x0080U
#define BCC_RW_CB_AUTO_PAUSE_MASK    0x0100U
#define BCC_RW_I_MEAS_EN_MASK        0x0200U
#define BCC_RW_DIAG_TIMEOUT_MASK     0x1C00U
#define BCC_RW_CYCLIC_TIMER_MASK     0xE000U

#define BCC_RW_WAVE_DC_SHIFT         0x01U
#define BCC_RW_FAULT_WAVE_SHIFT      0x03U
#define BCC_W_SOFT_RST_SHIFT         0x04U
#define BCC_RW_CB_MANUAL_PAUSE_SHIFT 0x05U
#define BCC_W_GO2DIAG_SHIFT          0x06U
#define BCC_R_DIAG_ST_SHIFT          0x06U
#define BCC_RW_CB_DRVEN_SHIFT        0x07U
#define BCC_RW_CB_AUTO_PAUSE_SHIFT   0x08U
#define BCC_RW_I_MEAS_EN_SHIFT       0x09U
#define BCC_RW_DIAG_TIMEOUT_SHIFT    0x0AU
#define BCC_RW_CYCLIC_TIMER_SHIFT    0x0DU

/* Controls the off time of the heart beat pulse. */
#define BCC_WAVE_DC_500US            (0x00U << BCC_RW_WAVE_DC_SHIFT)
#define BCC_WAVE_DC_1MS              (0x01U << BCC_RW_WAVE_DC_SHIFT)
#define BCC_WAVE_DC_10MS             (0x02U << BCC_RW_WAVE_DC_SHIFT)
#define BCC_WAVE_DC_100MS            (0x03U << BCC_RW_WAVE_DC_SHIFT)

/* FAULT pin wave form control bit. */
/* Fault pin has high (fault is present) or low (no fault) level behavior. */
#define BCC_FAULT_WAVE_DISABLED      (0x00U << BCC_RW_FAULT_WAVE_SHIFT)
/* Fault pin has heart beat wave when no fault is present. Pulse high time is 
 * fixed at 500us. */
#define BCC_FAULT_WAVE_ENABLED       (0x01U << BCC_RW_FAULT_WAVE_SHIFT)

/* Software Reset. */
#define BCC_SW_RESET_DISABLED        (0x00U << BCC_W_SOFT_RST_SHIFT)
#define BCC_SW_RESET_ENABLED         (0x01U << BCC_W_SOFT_RST_SHIFT)

/* Cell balancing manual pause. */
/* CB switches can be normally commanded on/off by the dedicated logic functions. */
#define BCC_CB_MAN_PAUSE_DISABLED    (0x00U << BCC_RW_CB_MANUAL_PAUSE_SHIFT)
/* CB switches are forced off, CB counters are not frozen. */
#define BCC_CB_MAN_PAUSE_ENABLED     (0x01U << BCC_RW_CB_MANUAL_PAUSE_SHIFT)

/* Commands the device to DIAG Mode. Re-writing the GO2DIAG bit restarts the DIAG_TIMEOUT. */
#define BCC_DIAG_MODE_DISABLED       (0x00U << BCC_W_GO2DIAG_SHIFT)
#define BCC_DIAG_MODE_ENABLED        (0x01U << BCC_W_GO2DIAG_SHIFT)

/* General enable or disable for all cell balance drivers. */
#define BCC_CB_DRV_DISABLED          (0x00U << BCC_RW_CB_DRVEN_SHIFT)
/* Each cell balance driver can be individually switched on/off by CB_xx_CFG register. */
#define BCC_CB_DRV_ENABLED           (0x01U << BCC_RW_CB_DRVEN_SHIFT)

/* Disables Cell Balance for ADC1-A and ADC1-B during the conversion cycle. */
#define BCC_CB_AUTO_PAUSE_DISABLED   (0x00U << BCC_RW_CB_AUTO_PAUSE_SHIFT)
/* CB switches are forced off each time a cyclic measurement is performed, no 
 * impact on CB counters. */
#define BCC_CB_AUTO_PAUSE_ENABLED    (0x01U << BCC_RW_CB_AUTO_PAUSE_SHIFT)

/* Enable for current measurement chain. */
#define BCC_I_MEAS_DISABLED          (0x00U << BCC_RW_I_MEAS_EN_SHIFT)
#define BCC_I_MEAS_ENABLED           (0x01U << BCC_RW_I_MEAS_EN_SHIFT)

/* DIAG Mode Timeout. Length of time the device is allowed to be in DIAG Mode 
 * before being forced to Normal mode. */
#define BCC_DIAG_TIMEOUT_NO_TIMER    (0x00U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_05S       (0x01U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_1S        (0x02U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_0_2S        (0x03U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_1S          (0x04U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_2S          (0x05U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_4S          (0x06U << BCC_RW_DIAG_TIMEOUT_SHIFT)
#define BCC_DIAG_TIMEOUT_8S          (0x07U << BCC_RW_DIAG_TIMEOUT_SHIFT)

/* Timer to trigger cyclic measurements in Normal mode or Sleep mode. */
/* Cyclic timer is disabled, whatever the mode. */
#define BCC_CYCLIC_TIMER_DISABLED    (0x00U << BCC_RW_CYCLIC_TIMER_SHIFT)
/* Continuous measurements. */
#define BCC_CYCLIC_TIMER_CONTINOUS   (0x01U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_0_1S        (0x02U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_0_2S        (0x03U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_1S          (0x04U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_2S          (0x05U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_4S          (0x06U << BCC_RW_CYCLIC_TIMER_SHIFT)
#define BCC_CYCLIC_TIMER_8S          (0x07U << BCC_RW_CYCLIC_TIMER_SHIFT)

/******************************************************************************/
/* $04 SYS_CFG2 - System configuration. */
/******************************************************************************/
#define BCC_REG_SYS_CFG2_ADDR            0x04U
#define BCC_REG_SYS_CFG2_DEFAULT         0x0334U

#define BCC_RW_HAMM_ENCOD_MASK           0x0001U
#define BCC_RW_NUMB_ODD_MASK             0x0002U
#define BCC_R_VPRE_UV_MASK               0x0004U
#define BCC_RW_TIMEOUT_COMM_MASK         0x0030U
#define BCC_RW_FLT_RST_CFG_MASK          0x03C0U
#define BCC_R_PREVIOUS_STATE_MASK        0x1C00U

#define BCC_RW_HAMM_ENCOD_SHIFT          0x00U
#define BCC_RW_NUMB_ODD_SHIFT            0x01U
#define BCC_R_VPRE_UV_SHIFT              0x02U
#define BCC_RW_TIMEOUT_COMM_SHIFT        0x04U
#define BCC_RW_FLT_RST_CFG_SHIFT         0x06U
#define BCC_R_PREVIOUS_STATE_SHIFT       0x0AU

/* Hamming encoders. */
/* Decode - the DED Hamming decoder performs. */
#define BCC_HAMM_ENCOD_DECODE           (0x00U << BCC_RW_HAMM_ENCOD_SHIFT)
/* Encode - DED Hamming decoders generate the redundancy bits. */
#define BCC_HAMM_ENCOD_ENCODE           (0x01U << BCC_RW_HAMM_ENCOD_SHIFT)

/* Odd number of cells in the cluster (useful for open-load diagnosis). */
#define BCC_EVEN_CELLS                  (0x00U << BCC_RW_NUMB_ODD_SHIFT)
#define BCC_ODD_CELLS                   (0x01U << BCC_RW_NUMB_ODD_SHIFT)

/* VPRE undervoltage detection. */
#define BCC_VPRE_UV_NO_UNDERVOLTAGE     (0x00U << BCC_R_VPRE_UV_SHIFT)
#define BCC_VPRE_UV_DETECTED            (0x01U << BCC_R_VPRE_UV_SHIFT)

/* No communication timeout - Set FAULT1_STATUS[COMM_LOSS_FLT] bit indicating 
 * no valid communication has been received for a period greater than 
 * TIMEOUT_COMM while in Normal mode. */
#define BCC_TIMEOUT_COMM_32MS           (0x00U << BCC_RW_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_64MS           (0x01U << BCC_RW_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_128MS          (0x02U << BCC_RW_TIMEOUT_COMM_SHIFT)
#define BCC_TIMEOUT_COMM_256MS          (0x03U << BCC_RW_TIMEOUT_COMM_SHIFT)

/* Fault reset configuration. */
/* Disabled COM timeout (1024 ms) reset and OSC fault monitoring and reset. */
#define BCC_FLT_RST_CFG_RESET_DIS       (0x03U << BCC_RW_FLT_RST_CFG_SHIFT)
/* Enabled OSC fault monitoring. */
#define BCC_FLT_RST_CFG_OSC_MON         (0x05U << BCC_RW_FLT_RST_CFG_SHIFT)
/* Enabled OSC fault monitoring and reset. */
#define BCC_FLT_RST_CFG_OSC             (0x06U << BCC_RW_FLT_RST_CFG_SHIFT)
/* Enabled COM timeout (1024 ms) reset. */
#define BCC_FLT_RST_CFG_COM             (0x09U << BCC_RW_FLT_RST_CFG_SHIFT)
/* Enabled COM timeout (1024 ms) reset and OSC fault monitoring. */
#define BCC_FLT_RST_CFG_COM_OSC_MON     (0x0AU << BCC_RW_FLT_RST_CFG_SHIFT)
/* Enable COM timeout (1024 ms) reset and OSC fault monitoring and reset (reset value). */
#define BCC_FLT_RST_CFG_COM_OSC         (0x0CU << BCC_RW_FLT_RST_CFG_SHIFT)

/******************************************************************************/
/* $05 SYS_DIAG - System diagnostic. */
/******************************************************************************/
#define BCC_REG_SYS_DIAG_ADDR       0x05U
#define BCC_REG_SYS_DIAG_DEFAULT    0x0000U

#define BCC_RW_CB_OL_EVEN_MASK      0x0001U
#define BCC_RW_CB_OL_ODD_MASK       0x0002U
#define BCC_RW_CT_OL_EVEN_MASK      0x0004U
#define BCC_RW_CT_OL_ODD_MASK       0x0008U
#define BCC_RW_CT_OV_UV_MASK        0x0010U
#define BCC_RW_CT_LEAK_DIAG_MASK    0x0020U
#define BCC_RW_POLARITY_MASK        0x0040U
#define BCC_RW_DA_DIAG_MASK         0x0080U
#define BCC_RW_ANX_TEMP_DIAG_MASK   0x0100U
#define BCC_RW_ANX_OLDIAG_MASK      0x0200U
#define BCC_RW_ISENSE_OL_DIAG_MASK  0x0400U
#define BCC_RW_I_MUX_MASK           0x1800U
#define BCC_RW_FAULT_DIAG_MASK      0x8000U

#define BCC_RW_CB_OL_EVEN_SHIFT     0x00U
#define BCC_RW_CB_OL_ODD_SHIFT      0x01U
#define BCC_RW_CT_OL_EVEN_SHIFT     0x02U
#define BCC_RW_CT_OL_ODD_SHIFT      0x03U
#define BCC_RW_CT_OV_UV_SHIFT       0x04U
#define BCC_RW_CT_LEAK_DIAG_SHIFT   0x05U
#define BCC_RW_POLARITY_SHIFT       0x06U
#define BCC_RW_DA_DIAG_SHIFT        0x07U
#define BCC_RW_ANX_TEMP_DIAG_SHIFT  0x08U
#define BCC_RW_ANX_OLDIAG_SHIFT     0x09U
#define BCC_RW_ISENSE_OL_DIAG_SHIFT 0x0AU
#define BCC_RW_I_MUX_SHIFT          0x0BU
#define BCC_RW_FAULT_DIAG_SHIFT     0x0FU

/* Control bit used to control the cell balance open load EVEN detection switches. */
#define BCC_CB_OL_EVEN_OPEN         (0x00U << BCC_RW_CB_OL_EVEN_SHIFT)
#define BCC_CB_OL_EVEN_CLOSED       (0x01U << BCC_RW_CB_OL_EVEN_SHIFT)

/* Control bit used to control the cell balance open load ODD detection switches. */
#define BCC_CB_OL_ODD_OPEN          (0x00U << BCC_RW_CB_OL_ODD_SHIFT)
#define BCC_CB_OL_ODD_CLOSED        (0x01U << BCC_RW_CB_OL_ODD_SHIFT)

/* Control bit used to control the even numbered cell terminal open detect switches. */
#define BCC_CT_OL_EVEN_OPEN         (0x00U << BCC_RW_CT_OL_EVEN_SHIFT)
/* May be set only when CT_OL_ODD is logic 0. */
#define BCC_CT_OL_EVEN_CLOSED       (0x01U << BCC_RW_CT_OL_EVEN_SHIFT)

/* Control bit used to control the odd numbered cell terminal open detect switches. */
#define BCC_CT_OL_ODD_OPEN          (0x00U << BCC_RW_CT_OL_ODD_SHIFT)
/* May be set only when CT_OL_EVEN is logic 0. */
#define BCC_CT_OL_ODD_CLOSED        (0x01U << BCC_RW_CT_OL_ODD_SHIFT)

/* OV & UV diagnostic is enabled (required for CT14, CT13, CT2 and CT1). This 
 * bit must be set to logic 0 when performing CT open load diagnostic. */
#define BCC_CT_OV_UV_DISABLED       (0x00U << BCC_RW_CT_OV_UV_SHIFT)
#define BCC_CT_OV_UV_ENABLED        (0x01U << BCC_RW_CT_OV_UV_SHIFT)

/* Control bit used in terminal leakage detection. Commands the MUX to route the 
 * CTx/CBx pin to
   ADC1-A,B converters. This bit must be exclusive vs. DA_DIAG. */
/* Normal operation, CTx are MUXed to converter. */
#define BCC_CT_LEAK_DIAG_NORMAL     (0x00U << BCC_RW_CT_LEAK_DIAG_SHIFT)
/* Difference between CT & CB pins are routed to the Analog Front End to be 
 * converted. */
#define BCC_CT_LEAK_DIAG_DIFF       (0x01U << BCC_RW_CT_LEAK_DIAG_SHIFT)

/* Control bit used in terminal leakage detection. Controls the polarity between 
 * the level shifter and the ADC1-A and ADC1-B converters. */
#define BCC_POL_NON_INVERTED        (0x00U << BCC_RW_POLARITY_SHIFT)
#define BCC_POL_INVERTED            (0x01U << BCC_RW_POLARITY_SHIFT)

/* Differential Amplifier Diagnostic. DIAGNOSTIC MODE FUNCTION ONLY. */
#define BCC_DA_DIAG_NO_CHECK        (0x00U << BCC_RW_DA_DIAG_SHIFT)
/* Check is enabled (Floating Zener conversion, Ground Zener measurement added, 
 * comparison). */
#define BCC_DA_DIAG_CHECK           (0x01U << BCC_RW_DA_DIAG_SHIFT)

/* Control bit used to activate the high-side or low-side switch on inputs 
 * configured as analog. */
/* Diagnostic switches are open. */
#define BCC_ANX_DIAG_SW_OPEN        (0x00U << BCC_RW_ANX_TEMP_DIAG_SHIFT)
/* Diagnostic switches are closed. */
#define BCC_ANX_DIAG_SW_CLOSED      (0x01U << BCC_RW_ANX_TEMP_DIAG_SHIFT)

/* ANx Open Load Diagnostic Control Bit. Used to activate the pull-down on GPIO 
 * input pins. */
#define BCC_ANX_OL_DIAG_DISABLED    (0x00U << BCC_RW_ANX_OLDIAG_SHIFT)
#define BCC_ANX_OL_DIAG_ENABLED     (0x01U << BCC_RW_ANX_OLDIAG_SHIFT)

/* ISENSE Open Load Diagnostic Control bit. Enables or disables internal pull up 
 * resistors on the ISENSE input pins. */
#define BCC_ISENSE_OL_DIAG_DISABLED (0x00U << BCC_RW_ISENSE_OL_DIAG_SHIFT)
#define BCC_ISENSE_OL_DIAG_ENABLED  (0x01U << BCC_RW_ISENSE_OL_DIAG_SHIFT)

/* Allows user to select between various inputs to PGA to be converted by ADC2. */
/* ISENSE+, ISENSE- */
#define BCC_IMUX_ISENSE             (0x00U << BCC_RW_I_MUX_SHIFT)
/* GPIO5, GPIO6 */
#define BCC_IMUX_GPIO5_6            (0x01U << BCC_RW_I_MUX_SHIFT)
/* Calibrated internal reference (diff ref). */
#define BCC_IMUX_DIFF_REF           (0x02U << BCC_RW_I_MUX_SHIFT)
/* PGA Zero (PGA differential inputs terminated to ground). */
#define BCC_IMUX_PGA_ZERO           (0x03U << BCC_RW_I_MUX_SHIFT)

/* FAULT Pin driver command. */
/* No FAULT pin drive, FAULT pin is under command of the pack controller. */
#define BCC_FAULT_PIN_PACK_CTRL     (0x00U << BCC_RW_FAULT_DIAG_SHIFT)
/* FAULT pin is forced to high level. */
#define BCC_FAULT_PIN_FORCED_HIGH   (0x01U << BCC_RW_FAULT_DIAG_SHIFT)

/******************************************************************************/
/* $06 ADC_CFG - ADC configuration. */
/******************************************************************************/
#define BCC_REG_ADC_CFG_ADDR      0x06U
#define BCC_REG_ADC_CFG_DEFAULT   0x0417U

#define BCC_RW_ADC2_DEF_MASK      0x0003U
#define BCC_RW_ADC1_B_DEF_MASK    0x000CU
#define BCC_RW_ADC1_A_DEF_MASK    0x0030U
#define BCC_RW_DIS_CH_COMP_MASK   0x0040U
#define BCC_W_CC_RST_MASK         0x0080U
#define BCC_W_PGA_GAIN_MASK       0x0700U
#define BCC_R_PGA_GAIN_S_MASK     0x0700U
#define BCC_W_SOC_MASK            0x0800U
#define BCC_R_EOC_N_MASK          0x0800U
#define BCC_RW_TAG_ID_MASK        0xF000U

#define BCC_RW_ADC2_DEF_SHIFT     0x00U
#define BCC_RW_ADC1_B_DEF_SHIFT   0x02U
#define BCC_RW_ADC1_A_DEF_SHIFT   0x04U
#define BCC_RW_DIS_CH_COMP_SHIFT  0x06U
#define BCC_W_CC_RST_SHIFT        0x07U
#define BCC_W_PGA_GAIN_SHIFT      0x08U
#define BCC_R_PGA_GAIN_S_SHIFT    0x08U
#define BCC_W_SOC_SHIFT           0x0BU
#define BCC_R_EOC_N_SHIFT         0x0BU
#define BCC_RW_TAG_ID_SHIFT       0x0CU

/* ADC2 Measurement Resolution. */
#define BCC_ADC2_RES_13BIT        (0x00U << BCC_RW_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_14BIT        (0x01U << BCC_RW_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_15BIT        (0x02U << BCC_RW_ADC2_DEF_SHIFT)
#define BCC_ADC2_RES_16BIT        (0x03U << BCC_RW_ADC2_DEF_SHIFT)

/* ADC1_B Measurement Resolution. */
#define BCC_ADC1_B_RES_13BIT      (0x00U << BCC_RW_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_14BIT      (0x01U << BCC_RW_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_15BIT      (0x02U << BCC_RW_ADC1_B_DEF_SHIFT)
#define BCC_ADC1_B_RES_16BIT      (0x03U << BCC_RW_ADC1_B_DEF_SHIFT)

/* ADC1_A Measurement Resolution. */
#define BCC_ADC1_A_RES_13BIT      (0x00U << BCC_RW_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_14BIT      (0x01U << BCC_RW_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_15BIT      (0x02U << BCC_RW_ADC1_A_DEF_SHIFT)
#define BCC_ADC1_A_RES_16BIT      (0x03U << BCC_RW_ADC1_A_DEF_SHIFT)

/* This bit enables or disables a compensation block for pins CT1, CT2. It 
 * compensates for the charge transfer from ADC1-A to these pins. If disabled, 
 * cell1 and cell2 accuracies may be impacted by a few mV. */
#define BCC_CHAR_COMP_ENABLED     (0x00U << BCC_RW_DIS_CH_COMP_SHIFT)
#define BCC_CHAR_COMP_DISABLED    (0x01U << BCC_RW_DIS_CH_COMP_SHIFT)

/* Control bit used to reset the value of the Coulomb counter to 0. */
/* Reset Coulomb counter registers COULOMB_CNT1 and COULOMB_CNT2 and the 
 * CC_NB_SAMPLES registers. */
#define BCC_CC_RESET              (0x01U << BCC_W_CC_RST_SHIFT)

/* Define the gain of the ADC2 Programmable Gain Amplifier. */
#define BCC_ADC2_PGA_4            (0x00U << BCC_W_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_16           (0x01U << BCC_W_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_64           (0x02U << BCC_W_PGA_GAIN_SHIFT)
#define BCC_ADC2_PGA_256          (0x03U << BCC_W_PGA_GAIN_SHIFT)
/* Automatic Gain Selection (internally adjusted). */
#define BCC_ADC2_PGA_AUTO         (0x04U << BCC_W_PGA_GAIN_SHIFT)

/* Control bit to command the BCC to initiate a conversion sequence. */
/* Initiate conversion sequence. Bit remains logic 1 during conversion cycle. 
 * User may not write logic 0 to this bit. */
#define BCC_INIT_CONV_SEQ         (0x01U << BCC_W_SOC_SHIFT)

/**
 * The TAG_ID is provided by the system controller during each conversion request. 
 * Tag ID should be incremented for each conversion request sent by the system 
 * controller. When reading the data for the requested conversion the tag field
 * contains the TAG_ID.
 *
 * @param reg Register to be modified.
 * @param tagID Tag ID provided in conversion [0x00 - 0x0F].
 */
#define BCC_SET_TAG_ID(reg, tagID) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_TAG_ID_MASK, BCC_RW_TAG_ID_SHIFT, tagID, 0x0FU)

/******************************************************************************/
/* $07 ADC2_OFFSET_COMP - ADC2 offset compensation. */
/******************************************************************************/
#define BCC_REG_ADC2_OFFSET_COMP_ADDR    0x07U
#define BCC_REG_ADC2_OFFSET_COMP_DEFAULT 0x4000U

#define BCC_RW_ADC2_OFFSET_COMP_MASK     0x00FFU
#define BCC_R_CC_OVT_MASK                0x0400U
#define BCC_R_SAMP_OVF_MASK              0x0800U
#define BCC_R_CC_N_OVF_MASK              0x1000U
#define BCC_R_CC_P_OVF_MASK              0x2000U
#define BCC_RW_FREE_CNT_MASK             0x4000U
#define BCC_RW_CC_RST_CFG_MASK           0x8000U

#define BCC_RW_ADC2_OFFSET_COMP_SHIFT    0x00U
#define BCC_R_CC_OVT_SHIFT               0x0AU
#define BCC_R_SAMP_OVF_SHIFT             0x0BU
#define BCC_R_CC_N_OVF_SHIFT             0x0CU
#define BCC_R_CC_P_OVF_SHIFT             0x0DU
#define BCC_RW_FREE_CNT_SHIFT            0x0EU
#define BCC_RW_CC_RST_CFG_SHIFT          0x0FU

/* Offset value, signed two's complement with 0.6uV resolution. It can be used 
 * to compensate for a PCB offset. 
 *
 * @param reg Value of a register (ORed with offset value).
 * @param offset Raw value of the offset.
 */
#define BCC_SET_ADC2_OFFSET(reg, offset) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_ADC2_OFFSET_COMP_MASK, \
            BCC_RW_ADC2_OFFSET_COMP_SHIFT, offset, 0xFFU)

/* Offset signed (2s complement) value with 0.6 uV resolution. Returned value 
 * is prepared to be placed in a register.
 *
 * @param offset Offset value in uV.
 */
#define BCC_GET_ADC2_OFFSET(offset) \
    (BCC_SET_ADC2_OFFSET(0x00U, (uint16_t)(((offset) * 10U) / 6U)))
      
/* Configuration of the free running Coulomb counters. */
#define BCC_FREE_CC_CLAMP                (0x00U << BCC_RW_FREE_CNT_SHIFT)
#define BCC_FREE_CC_ROLL_OVER            (0x01U << BCC_RW_FREE_CNT_SHIFT)

/* Configuration of the action linked to the read of Coulomb counters results. */
#define BCC_READ_CC_NO_ACTION            (0x00U << BCC_RW_CC_RST_CFG_SHIFT)
/* Reading any CC register (from @ $2D to @ $2F) also resets the Coulomb counters. */
#define BCC_READ_CC_RESET                (0x01U << BCC_RW_CC_RST_CFG_SHIFT)

/******************************************************************************/
/* $08 OV_UV_EN - CT measurement selection. */
/******************************************************************************/
#define BCC_REG_OV_UV_EN_ADDR              0x08U
#define BCC_REG_OV_UV_EN_DEFAULT           0x3FFFU

/* Note MC33772 does not have CT7_OVUV_EN, ..., CT14_OVUV_EN registers. */
/* Represents CT[1-14]_OVUV_EN mask. */
#define BCC_RW_CTX_OVUV_EN_MASK(ctNumber) \
    (0x0001U << ((ctNumber) - 1U))

#define BCC_RW_COMMON_UV_TH_MASK           0x4000U
#define BCC_RW_COMMON_OV_TH_MASK           0x8000U

/* Represents CT[1-14]_OVUV_EN shift. */
#define BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber) ((ctNumber) - 1U)

#define BCC_RW_COMMON_UV_TH_SHIFT          0x0EU
#define BCC_RW_COMMON_OV_TH_SHIFT          0x0FU

/* Control bit used to Enable or disable ADC data to be compared with thresholds 
 * for OV/UV. If Disabled no OVUV fault is set. */
#define BCC_CTX_OVUV_DISABLED(ctNumber)    (0x00U << BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber))
#define BCC_CTX_OVUV_ENABLED(ctNumber)     (0x01U << BCC_RW_CTX_OVUV_EN_SHIFT(ctNumber))

/* All CTx measurement use the common or individual under-voltage threshold 
 * register for comparison. */
#define BCC_CTX_UV_TH_INDIVIDUAL           (0x00U << BCC_RW_COMMON_UV_TH_SHIFT)
#define BCC_CTX_UV_TH_COMMON               (0x01U << BCC_RW_COMMON_UV_TH_SHIFT)

/* All CTx measurement use the common or individual over-voltage threshold 
 * register for comparison. */
#define BCC_CTX_OV_TH_INDIVIDUAL           (0x00U << BCC_RW_COMMON_OV_TH_SHIFT)
#define BCC_CTX_OV_TH_COMMON               (0x01U << BCC_RW_COMMON_OV_TH_SHIFT)

/******************************************************************************/
/* $09 CELL_OV_FLT - the overvoltage fault status of each cell. */
/******************************************************************************/
#define BCC_REG_CELL_OV_FLT_ADDR  0x09U

/* Bit mask for desired cell (overvoltage fault status).
 *
 * @param ctNumber Cell number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_CTX_OV_FLT_MASK(ctNumber) \
    (0x0001U << ((ctNumber) - 1U))
  
/******************************************************************************/
/* $0A CELL_UV_FLT - the undervoltage fault status of each cell. */
/******************************************************************************/
#define BCC_REG_CELL_UV_FLT_ADDR  0x0AU

/* Bit mask for desired cell (undervoltage fault status).
 *
 * @param ctNumber Cell number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_CTX_UV_FLT_MASK(ctNumber) \
    (0x0001U << ((ctNumber) - 1U))
  
/******************************************************************************/
/* $0C~19 CBx_CFG - CB configuration for cell 1~14. */
/******************************************************************************/
#define BCC_REG_CB1_CFG_ADDR    0x0CU
#define BCC_REG_CB2_CFG_ADDR    0x0DU
#define BCC_REG_CB3_CFG_ADDR    0x0EU
#define BCC_REG_CB4_CFG_ADDR    0x0FU
#define BCC_REG_CB5_CFG_ADDR    0x10U
#define BCC_REG_CB6_CFG_ADDR    0x11U

/* Following 8 registers are reserved in MC33772. */
#define BCC_REG_CB7_CFG_ADDR    0x12U
#define BCC_REG_CB8_CFG_ADDR    0x13U
#define BCC_REG_CB9_CFG_ADDR    0x14U
#define BCC_REG_CB10_CFG_ADDR   0x15U
#define BCC_REG_CB11_CFG_ADDR   0x16U
#define BCC_REG_CB12_CFG_ADDR   0x17U
#define BCC_REG_CB13_CFG_ADDR   0x18U
#define BCC_REG_CB14_CFG_ADDR   0x19U

#define BCC_REG_CBX_CFG_DEFAULT 0x0000U

#define BCC_RW_CB_TIMER_MASK    0x01FFU
#define BCC_W_CB_EN_MASK        0x0200U
#define BCC_R_CB_STS_MASK       0x0200U

#define BCC_RW_CB_TIMER_SHIFT   0x00U
#define BCC_W_CB_EN_SHIFT       0x09U
#define BCC_R_CB_STS_SHIFT      0x09U

/* Cell Balance Timer in minutes. */
#define BCC_SET_CB_TIMER(reg, minutes) \
    BCC_REG_SET_BITS_VALUE(reg, BCC_RW_CB_TIMER_MASK, BCC_RW_CB_TIMER_SHIFT, minutes, 0x01FFU)

/* Cell Balance enable. */
#define BCC_CB_DISABLED         (0x00U << BCC_W_CB_EN_SHIFT)
/* Enabled or re-launched if overwritten (restarts the timer count from zero 
 * and enables the driver). */
#define BCC_CB_ENABLED          (0x01U << BCC_W_CB_EN_SHIFT)

/******************************************************************************/
/* $1A CB_OPEN_FLT - the open CB fault. */
/******************************************************************************/
#define BCC_REG_CB_OPEN_FLT_ADDR  0x1AU

/* Bit mask for desired cell (cell balancing open load detection).
 *
 * @param cbNumber Cell number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_CBX_OPEN_FLT_MASK(cbNumber) \
    (0x0001U << ((cbNumber) - 1U))
  
/******************************************************************************/
/* $1B CB_SHORT_FLT - the cell balance short detection register. */
/******************************************************************************/
#define BCC_REG_CB_SHORT_FLT_ADDR 0x1BU

/* Bit mask for desired cell (cell balancing shorted load fault detection).
 *
 * @param cbNumber Cell number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_CBX_SHORT_FLT_MASK(cbNumber) \
    (0x0001U << ((cbNumber) - 1U))
  
/******************************************************************************/
/* $1C CB_DRV_STS - the CB driver status. */
/******************************************************************************/
#define BCC_REG_CB_DRV_STS_ADDR  0x1CU

/* Bit mask for desired cell balance driver (state of the cell balance driver).
 *
 * @param cbNumber CB driver number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_R_CBX_STS_MASK(cbNumber) \
    (0x0001U << ((cbNumber) - 1U))
  
/******************************************************************************/
/* $1D GPIO_CFG1 - GPIO configuration. */
/******************************************************************************/
#define BCC_REG_GPIO_CFG1_ADDR     0x1DU
#define BCC_REG_GPIO_CFG1_DEFAULT  0x0000U

/* Represents GPIO[0-6]_CFG mask. */
#define BCC_RW_GPIOX_CFG_MASK(GPIONumber) \
    (0x0003U << ((GPIONumber) * 2))

/* Represents GPIO[1-6]_CFG shift. */
#define BCC_GPIOX_CFG_SHIFT(GPIONumber)   \
    ((GPIONumber) * 2)

/* Register controls the configuration of the GPIO port. */
/* GPIOx configured as Analog Input for Ratiometric Measurement. */
#define BCC_GPIOX_AN_IN_RM_MEAS(gpioNumber)   \
    (0x00U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as Analog Input for Absolute Measurement. */
#define BCC_GPIOX_AN_IN_ABS_MEAS(gpioNumber)  \
    (0x01U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as digital input. */
#define BCC_GPIOX_DIG_IN(gpioNumber)          \
    (0x02U << BCC_GPIOX_CFG_SHIFT(gpioNumber))
/* GPIOx configured as digital output. */
#define BCC_GPIOX_DIG_OUT(gpioNumber)         \
    (0x03U << BCC_GPIOX_CFG_SHIFT(gpioNumber))

/******************************************************************************/
/* $1E GPIO_CFG2 - GPIO configuration. */
/******************************************************************************/
#define BCC_REG_GPIO_CFG2_ADDR     0x1EU
#define BCC_REG_GPIO_CFG2_DEFAULT  0x0000U

/* Represents GPIO[0-6]_DR mask. */
#define BCC_RW_GPIOX_DR_MASK(gpioNumber) \
    (0x0001U << (gpioNumber))

#define BCC_RW_GPIO0_FLT_ACT_MASK  0x0080U
#define BCC_RW_GPIO0_WU_MASK       0x0100U
#define BCC_RW_GPIO2_SOC_MASK      0x0200U

#define BCC_RW_GPIO0_FLT_ACT_SHIFT 0x07U
#define BCC_RW_GPIO0_WU_SHIFT      0x08U
#define BCC_RW_GPIO2_SOC_SHIFT     0x09U

/* GPIOx pin drive. Valid only when GPIOx_CFG = 11 (Normal mode), Functional 
 * in Diagnostic mode for OT/UT diagnostics. */
/* Drive GPIOx to low level. */
#define BCC_GPIOx_LOW(GPIONumber)  (0x00U << (GPIONumber))
/* Drive GPIOx to high level. */
#define BCC_GPIOx_HIGH(GPIONumber) (0x01U << (GPIONumber))

/* GPIO0 Activate Fault Output pin. Valid only when GPIO0_CFG = 10. */
/* Does not activate Fault pin when GPIO0 is configured as an input and is logic 1. */
#define BCC_GPIO0_INP_HIGH_FP_NACT (0x00U << BCC_RW_GPIO0_FLT_ACT_SHIFT)
/* Activates the Fault pin when GPIO0 is configured as an input and is logic 1. */
#define BCC_GPIO0_INP_HIGH_FP_ACT  (0x01U << BCC_RW_GPIO0_FLT_ACT_SHIFT)

/* GPIO0 wake-up capability. Valid only when GPIO0_CFG = 10. */
/* No wake-up capability. */
#define BCC_GPIO0_NO_WAKE_UP       (0x00U << BCC_RW_GPIO0_WU_SHIFT)
/* Wake-up on any edge, transitioning the system from Sleep to Normal. */
#define BCC_GPIO0_WAKE_UP          (0x01U << BCC_RW_GPIO0_WU_SHIFT)

/* GPIO2 used as ADC1_A/ADC1_B start-of-conversion. Requires GPIO2_CFG = 10. */
/* GPIO2 port ADC Trigger is disabled. */
#define BCC_GPIO2_ADC_TRG_DISABLED (0x00U << BCC_RW_GPIO2_SOC_SHIFT)
/* GPIO2 port ADC Trigger is enabled. A rising edge on GPIO2 triggers an ADC1-A 
 * and ADC1-B conversion - only when in Normal mode. */
#define BCC_GPIO2_ADC_TRG_ENABLED  (0x01U << BCC_RW_GPIO2_SOC_SHIFT)

/******************************************************************************/
/* $1F GPIO_STS - GPIO diagnostic. */
/******************************************************************************/
#define BCC_REG_GPIO_STS_ADDR  0x1FU

/* Bit mask for desired GPIO (recognition of low to high to low transitions events).
 *
 * @param gpioNumber GPIO number ranging from 0 to 6 (no check performed).
 * @return Bit mask for desired GPIO. 
 */ 
#define BCC_R_GPIOX_ST_MASK(gpioNumber) \
    (0x0001U << (gpioNumber))
  
/* Bit mask for desired GPIO (real time GPIO status).
 *
 * @param gpioNumber GPIO number ranging from 0 to 6 (no check performed).
 * @return Bit mask for desired GPIO. 
 */ 
#define BCC_RW_GPIOX_H_MASK(gpioNumber) \
    (0x0001U << ((gpioNumber) + 8U))
  
/******************************************************************************/
/* $20 AN_OT_UT_FLT - AN over and undertemp. */
/******************************************************************************/
#define BCC_REG_AN_OT_UT_FLT_ADDR  0x20U

/* Bit mask for desired AN (undertemperature detection).
 *
 * @param anNumber AN number ranging from 0 to 6 (no check performed).
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_ANX_UT_MASK(anNumber) \
    (0x0001U << (anNumber))

/* Bit mask of all ANx UT bits. */
#define BCC_RW_AN_UT_MASK          0x007F

/* Bit mask for desired AN (overtemperature detection).
 *
 * @param anNumber AN number ranging from 0 to 6 (no check performed).
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_ANX_OT_MASK(anNumber) \
    (0x0001U << ((anNumber) + 8U))
  
/* Bit mask of all ANx OT bits. */
#define BCC_RW_AN_OT_MASK          0x7F00

/******************************************************************************/
/* $21 GPIO_SHORT_ANx_OPEN_STS - short GPIO/Open AN diagnostic. */
/******************************************************************************/
#define BCC_REG_GPIO_SHORT_ADDR  0x21U

/* Bit mask for desired AN (analog inputs open load detection).
 *
 * @param anNumber AN number ranging from 0 to 6 (no check performed).
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_ANX_OPEN_MASK(anNumber) \
    (0x0001U << (anNumber))

/* Bit mask of all ANx OPEN bits. */
#define BCC_RW_AN_OPEN_MASK      0x007F

/* Bit mask for desired GPIO (GPIO short detection).
 *
 * @param GPIONumber GPIO number ranging from 0 to 6 (no check performed). 
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_GPIOX_SH_MASK(gpioNumber) \
    (0x0001U << ((gpioNumber) + 8U))

/******************************************************************************/
/* $22 I_STATUS - PGA DAC value. */
/******************************************************************************/
#define BCC_REG_I_STATUS_ADDR  0x22U

#define BCC_R_PGA_DAC_MASK     0xFF00U

#define BCC_R_PGA_DAC_SHIFT    0x08U

/******************************************************************************/
/* $23 COM_STATUS - number of CRC error counted. */
/******************************************************************************/
#define BCC_REG_COM_STATUS_ADDR   0x23U

#define BCC_R_COM_ERR_COUNT_MASK  0xFF00U

#define BCC_R_COM_ERR_COUNT_SHIFT 0x08U

/**
 * This macro returns COM_ERR_COUNT bits from raw value of COM_STATUS register.
 *
 * @param reg Unsigned 16 bit raw value of COM_STATUS register.
 * @return Unsigned 8 bit value containing number of communication errors
 * detected.
 */
#define BCC_GET_COM_ERR_COUNT(reg) \
    (BCC_REG_GET_BITS_VALUE(reg, BCC_R_COM_ERR_COUNT_MASK, \
            BCC_R_COM_ERR_COUNT_SHIFT))

/******************************************************************************/
/* $24 FAULT1_STATUS - fault status. */
/******************************************************************************/
#define BCC_REG_FAULT1_STATUS_ADDR  0x24U

#define BCC_R_CT_UV_FLT_MASK        0x0001U
#define BCC_R_CT_OV_FLT_MASK        0x0002U
#define BCC_R_AN_UT_FLT_MASK        0x0004U
#define BCC_R_AN_OT_FLT_MASK        0x0008U
#define BCC_RW_IS_OC_FLT_MASK       0x0010U
#define BCC_RW_IS_OL_FLT_MASK       0x0020U
#define BCC_RW_I2C_ERR_FLT_MASK     0x0040U
#define BCC_RW_GPIO0_WUP_FLT_MASK   0x0080U
#define BCC_RW_CSB_WUP_FLT_MASK     0x0100U
#define BCC_RW_COM_ERR_FLT_MASK     0x0200U
#define BCC_RW_COM_LOSS_FLT_MASK    0x0400U
#define BCC_RW_VPWR_LV_FLT_MASK     0x0800U
#define BCC_RW_VPWR_OV_FLT_MASK     0x1000U
#define BCC_RW_COM_ERR_OVR_FLT_MASK 0x2000U
#define BCC_RW_RESET_FLT_MASK       0x4000U
#define BCC_RW_POR_MASK             0x8000U

/******************************************************************************/
/* $25 FAULT2_STATUS - fault status. */
/******************************************************************************/
#define BCC_REG_FAULT2_STATUS_ADDR 0x25U

#define BCC_RW_FUSE_ERR_FLT_MASK   0x0001U
#define BCC_RW_DED_ERR_FLT_MASK    0x0002U
#define BCC_RW_OSC_ERR_FLT_MASK    0x0004U
#define BCC_R_CB_OPEN_FLT_MASK     0x0008U
#define BCC_R_CB_SHORT_FLT_MASK    0x0010U
#define BCC_R_GPIO_SHORT_FLT_MASK  0x0020U
#define BCC_R_AN_OPEN_FLT_MASK     0x0040U
#define BCC_RW_IDLE_MODE_FLT_MASK  0x0080U
#define BCC_RW_IC_TSD_FLT_MASK     0x0100U
#define BCC_RW_GND_LOSS_FLT_MASK   0x0200U
#define BCC_RW_ADC1_A_FLT_MASK     0x0400U
#define BCC_RW_ADC1_B_FLT_MASK     0x0800U
#define BCC_RW_VANA_UV_FLT_MASK    0x1000U
#define BCC_RW_VANA_OV_FLT_MASK    0x2000U
#define BCC_RW_VCOM_UV_FLT_MASK    0x4000U
#define BCC_RW_VCOM_OV_FLT_MASK    0x8000U

/******************************************************************************/
/* $26 FAULT3_STATUS - fault status. */
/******************************************************************************/
#define BCC_REG_FAULT3_STATUS_ADDR  0x26U

/* Bit mask for desired cell (end of time cell balancing notification).
 *
 * @param cbNumber CB number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_RW_EOT_CBX_MASK(cbNumber) \
  (0x0001U << ((cbNumber) - 1U))

#define BCC_R_VCP_UV_MASK       0x2000U /* MC33772 only. */
#define BCC_R_DIAG_TO_FLT_MASK  0x4000U
#define BCC_R_CC_OVR_FLT_MASK   0x8000U

/******************************************************************************/
/* $27 FAULT_MASK1 - FAULT pin mask. */
/******************************************************************************/
/* Mask a fault from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK1_ADDR    0x27U
#define BCC_REG_FAULT_MASK1_DEFAULT 0x0000U

/* Note: you can use bit masks defined for FAULT1_STATUS register (the same 
 * order of bits). */
#define BCC_CT_UV_FLT_EN        0x0000U
#define BCC_CT_UV_FLT_DIS       0x0001U
#define BCC_CT_OV_FLT_EN        0x0000U
#define BCC_CT_OV_FLT_DIS       0x0002U
#define BCC_AN_UT_FLT_EN        0x0000U
#define BCC_AN_UT_FLT_DIS       0x0004U
#define BCC_AN_OT_FLT_EN        0x0000U
#define BCC_AN_OT_FLT_DIS       0x0008U
#define BCC_IS_OC_FLT_EN        0x0000U
#define BCC_IS_OC_FLT_DIS       0x0010U
#define BCC_IS_OL_FLT_EN        0x0000U
#define BCC_IS_OL_FLT_DIS       0x0020U
#define BCC_I2C_ERR_FLT_EN      0x0000U
#define BCC_I2C_ERR_FLT_DIS     0x0040U
#define BCC_GPIO0_WUP_FLT_EN    0x0000U
#define BCC_GPIO0_WUP_FLT_DIS   0x0080U
#define BCC_CSB_WUP_FLT_EN      0x0000U
#define BCC_CSB_WUP_FLT_DIS     0x0100U
#define BCC_COM_ERR_FLT_EN      0x0000U
#define BCC_COM_ERR_FLT_DIS     0x0200U
#define BCC_COM_LOSS_FLT_EN     0x0000U
#define BCC_COM_LOSS_FLT_DIS    0x0400U
#define BCC_VPWR_LV_FLT_EN      0x0000U
#define BCC_VPWR_LV_FLT_DIS     0x0800U
#define BCC_VPWR_OV_FLT_EN      0x0000U
#define BCC_VPWR_OV_FLT_DIS     0x1000U

/******************************************************************************/
/* $28 FAULT_MASK2 - FAULT pin mask. */
/******************************************************************************/
/* Mask a fault from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK2_ADDR    0x28U
#define BCC_REG_FAULT_MASK2_DEFAULT 0x0000U

/* Note: you can use bit masks defined for FAULT2_STATUS register (the same 
 * order of bits). */
#define BCC_FUSE_ERR_FLT_EN     0x0000U
#define BCC_FUSE_ERR_FLT_DIS    0x0001U
#define BCC_DED_ERR_FLT_EN      0x0000U
#define BCC_DED_ERR_FLT_DIS     0x0002U
#define BCC_OSC_ERR_FLT_EN      0x0000U
#define BCC_OSC_ERR_FLT_DIS     0x0004U
#define BCC_CB_OPEN_FLT_EN      0x0000U
#define BCC_CB_OPEN_FLT_DIS     0x0008U
#define BCC_CB_SHORT_FLT_EN     0x0000U
#define BCC_CB_SHORT_FLT_DIS    0x0010U
#define BCC_GPIO_SHORT_FLT_EN   0x0000U
#define BCC_GPIO_SHORT_FLT_DIS  0x0020U
#define BCC_AN_OPEN_FLT_EN      0x0000U
#define BCC_AN_OPEN_FLT_DIS     0x0040U
#define BCC_GND_LOSS_FLT_EN     0x0000U
#define BCC_GND_LOSS_FLT_DIS    0x0200U
#define BCC_ADC1_A_FLT_EN       0x0000U
#define BCC_ADC1_A_FLT_DIS      0x0400U
#define BCC_ADC1_B_FLT_EN       0x0000U
#define BCC_ADC1_B_FLT_DIS      0x0800U
#define BCC_VANA_UV_FLT_EN      0x0000U
#define BCC_VANA_UV_FLT_DIS     0x1000U
#define BCC_VANA_OV_FLT_EN      0x0000U
#define BCC_VANA_OV_FLT_DIS     0x2000U
#define BCC_VCOM_UV_FLT_EN      0x0000U
#define BCC_VCOM_UV_FLT_DIS     0x4000U
#define BCC_VCOM_OV_FLT_EN      0x0000U
#define BCC_VCOM_OV_FLT_DIS     0x8000U

/******************************************************************************/
/* $29 FAULT_MASK3 - FAULT pin mask. */
/******************************************************************************/
/* Mask out the cell balance timer from activating the FAULT pin output. */
#define BCC_REG_FAULT_MASK3_ADDR    0x29U
#define BCC_REG_FAULT_MASK3_DEFAULT 0x0000U

/* Note: you can use bit masks defined for FAULT3_STATUS register (the same 
 * order of bits). */

/* Disable fault detection for desired cell (end of time cell balancing notification).
 *
 * @param CBNumber cb number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_EOT_CBX_FLT_DIS(cbNumber) \
    (0x0001U << ((cbNumber) - 1U))
  
/* Enable fault detection for desired cell (end of time cell balancing notification). */ 
#define BCC_EOT_CBX_FLT_EN      0x0000U

#define BCC_VCP_UV_EN           0x0000U /* MC33772 only. */
#define BCC_VCP_UV_DIS          0x2000U /* MC33772 only. */
#define BCC_DIAG_TO_FLT_EN      0x0000U
#define BCC_DIAG_TO_FLT_DIS     0x4000U
#define BCC_CC_OVR_FLT_EN       0x0000U
#define BCC_CC_OVR_FLT_DIS      0x8000U

/******************************************************************************/
/* $2A WAKEUP_MASK1 - Wake-up events mask. */
/******************************************************************************/
/* Mask events from waking up device and transition to Normal mode. */
#define BCC_REG_WAKEUP_MASK1_ADDR    0x2AU
#define BCC_REG_WAKEUP_MASK1_DEFAULT 0x0000U

#define BCC_CT_UV_WAKEUP_EN       0x0000U
#define BCC_CT_UV_WAKEUP_DIS      0x0001U
#define BCC_CT_OV_WAKEUP_EN       0x0000U
#define BCC_CT_OV_WAKEUP_DIS      0x0002U
#define BCC_AN_UT_WAKEUP_EN       0x0000U
#define BCC_AN_UT_WAKEUP_DIS      0x0004U
#define BCC_AN_OT_WAKEUP_EN       0x0000U
#define BCC_AN_OT_WAKEUP_DIS      0x0008U
#define BCC_IS_OC_WAKEUP_EN       0x0000U
#define BCC_IS_OC_WAKEUP_DIS      0x0010U
#define BCC_GPIO0_WUP_WAKEUP_EN   0x0000U
#define BCC_GPIO0_WUP_WAKEUP_DIS  0x0080U
#define BCC_CSB_WUP_WAKEUP_EN     0x0000U
#define BCC_CSB_WUP_WAKEUP_DIS    0x0100U
#define BCC_VPWR_LV_WAKEUP_EN     0x0000U
#define BCC_VPWR_LV_WAKEUP_DIS    0x0800U
#define BCC_VPWR_OV_WAKEUP_EN     0x0000U
#define BCC_VPWR_OV_WAKEUP_DIS    0x1000U

/******************************************************************************/
/* $2B WAKEUP_MASK2 - Wake-up events mask. */
/******************************************************************************/
/* Mask events from waking up device and transition to Normal mode. */
#define BCC_REG_WAKEUP_MASK2_ADDR    0x2BU
#define BCC_REG_WAKEUP_MASK2_DEFAULT 0x0000U

#define BCC_DED_ERR_WAKEUP_EN     0x0000U
#define BCC_DED_ERR_WAKEUP_DIS    0x0002U
#define BCC_OSC_ERR_WAKEUP_EN     0x0000U
#define BCC_OSC_ERR_WAKEUP_DIS    0x0004U
#define BCC_CB_SHORT_WAKEUP_EN    0x0000U
#define BCC_CB_SHORT_WAKEUP_DIS   0x0010U
#define BCC_GPIO_SHORT_WAKEUP_EN  0x0000U
#define BCC_GPIO_SHORT_WAKEUP_DIS 0x0020U
#define BCC_IC_TSD_WAKEUP_EN      0x0000U
#define BCC_IC_TSD_WAKEUP_DIS     0x0100U
#define BCC_GND_LOSS_WAKEUP_EN    0x0000U
#define BCC_GND_LOSS_WAKEUP_DIS   0x0200U
#define BCC_ADC1_A_WAKEUP_EN      0x0000U
#define BCC_ADC1_A_WAKEUP_DIS     0x0400U
#define BCC_ADC1_B_WAKEUP_EN      0x0000U
#define BCC_ADC1_B_WAKEUP_DIS     0x0800U
#define BCC_VANA_UV_WAKEUP_EN     0x0000U
#define BCC_VANA_UV_WAKEUP_DIS    0x1000U
#define BCC_VANA_OV_WAKEUP_EN     0x0000U
#define BCC_VANA_OV_WAKEUP_DIS    0x2000U
#define BCC_VCOM_UV_WAKEUP_EN     0x0000U
#define BCC_VCOM_UV_WAKEUP_DIS    0x4000U
#define BCC_VCOM_OV_WAKEUP_EN     0x0000U
#define BCC_VCOM_OV_WAKEUP_DIS    0x8000U

/******************************************************************************/
/* $2C WAKEUP_MASK3 - Wake-up events mask. */
/******************************************************************************/
/* Mask out the cell balance timeout in Sleep mode from activating Normal mode. */
#define BCC_REG_WAKEUP_MASK3_ADDR    0x2CU
#define BCC_REG_WAKEUP_MASK3_DEFAULT 0x0000U

/* Disable wake-up for desired cell (end of time cell balancing notification).
 *
 * @param cbNumber CB number ranging from 1 to 6 (MC33772) or 14 (MC33771)
 *                 No check performed.
 * @return Bit mask for desired cell. 
 */ 
#define BCC_EOT_CBX_WAKEUP_DIS(cbNumber) \
  (0x0001U << ((cbNumber) - 1U))
  
/* Enable fault detection for desired cell (end of time cell balancing notification). */ 
#define BCC_EOT_CBX_WAKEUP_EN    0x0000U

#define BCC_VCP_UV_WAKEUP_EN     0x0000U /* MC33772 only. */
#define BCC_VCP_UV_WAKEUP_DIS    0x2000U /* MC33772 only. */

#define BCC_DIAG_TO_WAKEUP_EN    0x0000U
#define BCC_DIAG_TO_WAKEUP_DIS   0x4000U

#define BCC_CC_OVR_WAKEUP_EN     0x0000U
#define BCC_CC_OVR_WAKEUP_DIS    0x8000U

/******************************************************************************/
/* $2D CC_NB_NB_SAMPLES - number of samples taken for the Coulomb count. */
/******************************************************************************/
#define BCC_REG_CC_NB_SAMPLES_ADDR 0x2DU

#define BCC_R_CC_NB_SAMPLES_MASK   0xFFFFU

/******************************************************************************/
/* $2E COULOMB_CNT1 - Coulomb counting accumulator. */
/******************************************************************************/
#define BCC_REG_COULOMB_CNT1_ADDR  0x2EU

#define BCC_R_COULOMB_CNT_MSB_MASK 0xFFFFU

/******************************************************************************/
/* $2F COULOMB_CNT2 - Coulomb counting accumulator. */
/******************************************************************************/
#define BCC_REG_COULOMB_CNT2_ADDR  0x2FU

#define BCC_R_COULOMB_CNT_LSB_MASK 0xFFFFU

/**
 * This macro returns 32 bit signed value of COULOMB_CNT.
 * Note: COULOMB_CNT1 register represents higher part of final value
 * (MSB). COULOMB_CNT2 is lower part (LSB).
 *
 * @param coulombCnt1 Content of register COULOMB_CNT1.
 * @param coulombCnt2 Content of register COULOMB_CNT2.
 */
#define BCC_GET_COULOMB_CNT(coulombCnt1, coulombCnt2) \
  ((int32_t)(((uint32_t)((coulombCnt1) & BCC_R_COULOMB_CNT_MSB_MASK) << 0xFU) | \
  (((uint32_t)(coulombCnt2) & BCC_R_COULOMB_CNT_LSB_MASK))))

/******************************************************************************/
/* $30 MEAS_ISENSE1 - On Demand value of measured current. */
/******************************************************************************/
#define BCC_REG_MEAS_ISENSE1_ADDR  0x30U

#define BCC_R_MEAS1_I_MASK    0x7FFFU

/******************************************************************************/
/* $31 MEAS_ISENSE2 - On Demand value of measured current. */
/******************************************************************************/
#define BCC_REG_MEAS_ISENSE2_ADDR  0x31U

#define BCC_R_MEAS2_I_MASK      0x000FU
#define BCC_RW_PGA_GCHANGE_MASK 0x0040U
#define BCC_RW_ADC2_SAT_MASK    0x0080U
#define BCC_R_PGA_GAIN_MASK     0x0300U

/**
 * This macro returns 32 bit unsigned value of ISENSE.
 * Note: MEAS_I from MEAS_ISENSE1 register represents higher part of final value
 * (MSB). MEAS_I from MEAS_ISENSE2 is lower part (LSB).
 *
 * @param measISense1 Content of register MEAS_ISENSE1.
 * @param measISense2 Content of register MEAS_ISENSE2.
 */
#define BCC_GET_ISENSE_RAW(measISense1, measISense2) \
    ((((uint32_t)(measISense1) & BCC_R_MEAS1_I_MASK) << 4U) | \
      ((uint32_t)(measISense2) & BCC_R_MEAS2_I_MASK))

/**
 * This macro converts two 19 bit value (result of BCC_GET_ISENSE_RAW macro)
 * to signed 32 bit value. Sign extension is performed.
 *
 * @param iSenseRaw Raw value of measured current (result of
 * BCC_GET_ISENSE_RAW macro).
 */
#define BCC_GET_ISENSE_RAW_SIGN(iSenseRaw) \
    ((int32_t)(((iSenseRaw) & 0x040000U) ? ((iSenseRaw) | 0xFFF80000U) : (iSenseRaw)))

/******************************************************************************/
/* $32 MEAS_STACK - Stack voltage measurement. */
/******************************************************************************/    
/* Register addresses. */
#define BCC_REG_MEAS_STACK_ADDR  0x32U

/* Bit masks common for MEAS_STACK, MEAS_CELL14~1, MEAS_AN6~0, MEAS_IC_TEMP,
 * MEAS_VBG_DIAG_ADC1A and MEAS_VBG_DIAG_ADC1A registers.
 */
#define BCC_R_MEAS_MASK     0x7FFFU
#define BCC_R_DATA_RDY_MASK 0x8000U

/**
 * This macro masks register value and returns raw measured value.
 *
 * @param reg Value from a measurement register (MEAS_STACK, MEAS_CELL14~1,
 * MEAS_AN6~0, MEAS_IC_TEMP, MEAS_VBG_DIAG_ADC1A and MEAS_VBG_DIAG_ADC1A
 * registers).
 */
#define BCC_GET_MEAS_RAW(reg) \
    ((reg) & BCC_R_MEAS_MASK)

/******************************************************************************/
/* $33~40 MEAS_CELL14~1 - Cell 1~14 voltage measurement. */
/******************************************************************************/    
#define BCC_REG_MEAS_CELLX_ADDR_MC33771_START  0x33U
#define BCC_REG_MEAS_CELLX_ADDR_MC33772_START  0x3BU
#define BCC_REG_MEAS_CELLX_ADDR_END            0x40U

/******************************************************************************/
/* $41~47 MEAS_AN6~0 - AN 0~6 voltage measurement. */
/******************************************************************************/    
#define BCC_REG_MEAS_ANX_ADDR_START       0x41U
#define BCC_REG_MEAS_ANX_ADDR_END         0x47U

/******************************************************************************/
/* $48 MEAS_IC_TEMP - IC temperature measurement. */
/******************************************************************************/    
#define BCC_REG_MEAS_IC_TEMP_ADDR         0x48U

/******************************************************************************/
/* $49 MEAS_VBG_DIAG_ADC1A - ADCIA band gap reference measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_VBG_DIAG_ADC1A_ADDR  0x49U

/******************************************************************************/
/* $4A MEAS_VBG_DIAG_ADC1B - ADCIB band gap reference measurement. */
/******************************************************************************/
#define BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR  0x4AU

/******************************************************************************/
/* $4B TH_ALL_CT - CTx over and undervoltage threshold. */
/******************************************************************************/
#define BCC_REG_TH_ALL_CT_ADDR    0x4BU
#define BCC_REG_TH_ALL_CT_DEFAULT 0xD780U

#define BCC_RW_ALL_CT_UV_TH_MASK  0x00FFU
#define BCC_RW_ALL_CT_OV_TH_MASK  0xFF00U

#define BCC_RW_ALL_CT_UV_TH_SHIFT 0x00U
#define BCC_RW_ALL_CT_OV_TH_SHIFT 0x08U

/* Undervoltage threshold setting for all cell terminals. Returned value is 
 * prepared to be placed in register. Enabled through register OV_UV_EN.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ALL_CT_UV_TH(threshold) \
    ((uint8_t)(((threshold) * 10U) / 195U) << BCC_RW_ALL_CT_UV_TH_SHIFT)
/* Default value is 2.5 V. */
#define BCC_ALL_CT_UV_TH_DEFAULT  0x80U

/* Overvoltage threshold setting for all cell terminals. Returned value is 
 * prepared to be placed in register. Enabled through register OV_UV_EN.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ALL_CT_OV_TH(threshold) \
    ((uint16_t)(((threshold) * 10U) / 195U) << BCC_RW_ALL_CT_OV_TH_SHIFT)

/* Default value is 2.5 V. */
#define BCC_ALL_CT_OV_TH_DEFAULT  0xD7U

/******************************************************************************/
/* $4C~59 TH_CTx - CTx over and undervoltage threshold. */
/******************************************************************************/
/* Following 8 registers are reserved in MC33772. */
#define BCC_REG_TH_CT14_ADDR   0x4CU
#define BCC_REG_TH_CT13_ADDR   0x4DU
#define BCC_REG_TH_CT12_ADDR   0x4EU
#define BCC_REG_TH_CT11_ADDR   0x4FU
#define BCC_REG_TH_CT10_ADDR   0x50U
#define BCC_REG_TH_CT9_ADDR    0x51U
#define BCC_REG_TH_CT8_ADDR    0x52U
#define BCC_REG_TH_CT7_ADDR    0x53U

#define BCC_REG_TH_CT6_ADDR    0x54U
#define BCC_REG_TH_CT5_ADDR    0x55U
#define BCC_REG_TH_CT4_ADDR    0x56U
#define BCC_REG_TH_CT3_ADDR    0x57U
#define BCC_REG_TH_CT2_ADDR    0x58U
#define BCC_REG_TH_CT1_ADDR    0x59U

#define BCC_REG_TH_CTX_DEFAULT 0xD780U

#define BCC_RW_CTX_UV_TH_MASK  0x00FFU
#define BCC_RW_CTX_OV_TH_MASK  0xFF00U

#define BCC_RW_CTX_UV_TH_SHIFT 0x00U
#define BCC_RW_CTX_OV_TH_SHIFT 0x08U

/* Undervoltage threshold setting for all cell terminals. Returned value is 
 * prepared to be placed in register. Register [COMMON_UV_TH] must be logic 0 
 * and [CTx_OVUV_EN] must be logic 1 to use TH_CTx register as threshold.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_CTX_UV_TH(threshold)                                  \
    ((uint16_t)(((((threshold) * 10U) / 196U) > 0x00FF) ?             \
    0x0FF : (((threshold) * 10U) / 196U)) << BCC_RW_CTX_UV_TH_SHIFT)

/* Default value is 2.5 V. */
#define BCC_CTX_UV_TH_DEFAULT  0x80U

/* Overvoltage threshold setting for all cell terminals. Returned value is 
 * prepared to be placed in register. Register [COMMON_OV_TH] must be logic 0 
 * and [CTx_OVUV_EN] must be logic 1 to use TH_CTx register as threshold.
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_CTX_OV_TH(threshold)                                  \
    ((uint16_t)(((((threshold) * 10U) / 196U) > 0x00FF) ?             \
    0x0FF : (((threshold) * 10U) / 196U)) << BCC_RW_CTX_OV_TH_SHIFT)


/* Default value is 2.5 V. */
#define BCC_CTX_OV_TH_DEFAULT  0xD7U

/******************************************************************************/
/* $5A~60 TH_ANx_OT - ANx overtemp threshold. */
/******************************************************************************/
#define BCC_REG_TH_AN6_OT_ADDR    0x5AU
#define BCC_REG_TH_AN5_OT_ADDR    0x5BU
#define BCC_REG_TH_AN4_OT_ADDR    0x5CU
#define BCC_REG_TH_AN3_OT_ADDR    0x5DU
#define BCC_REG_TH_AN2_OT_ADDR    0x5EU
#define BCC_REG_TH_AN1_OT_ADDR    0x5FU
#define BCC_REG_TH_AN0_OT_ADDR    0x60U

#define BCC_REG_TH_ANX_OT_DEFAULT 0x00EDU

#define BCC_RW_ANX_OT_TH_MASK     0x03FFU
#define BCC_RW_ANX_OT_TH_SHIFT    0x00U

/* Overtemperature threshold setting for analog input x. Returned value is 
 * prepared to be placed in register. 
 *
 * @param threshold Threshold value in mV.
 */
#define BCC_SET_ANX_OT_TH(threshold)                                                   \
    ((uint16_t)((((((uint32_t)(threshold)) * 100U) / 488U) > BCC_RW_ANX_OT_TH_MASK) ?  \
    BCC_RW_ANX_OT_TH_MASK : ((((uint32_t)(threshold)) * 100U) / 488U)) << BCC_RW_ANX_OT_TH_SHIFT)

/* Default value set to 1.16V. */
#define BCC_ANX_OT_TH_DEFAULT     0x0EDU

/******************************************************************************/
/* $61~67 TH_ANx_UT - ANx undertemp threshold. */
/******************************************************************************/
#define BCC_REG_TH_AN6_UT_ADDR    0x61U
#define BCC_REG_TH_AN5_UT_ADDR    0x62U
#define BCC_REG_TH_AN4_UT_ADDR    0x63U
#define BCC_REG_TH_AN3_UT_ADDR    0x64U
#define BCC_REG_TH_AN2_UT_ADDR    0x65U
#define BCC_REG_TH_AN1_UT_ADDR    0x66U
#define BCC_REG_TH_AN0_UT_ADDR    0x67U

#define BCC_REG_TH_ANX_UT_DEFAULT 0x030EU

#define BCC_RW_ANX_UT_TH_MASK     0x03FFU
#define BCC_RW_ANX_UT_TH_SHIFT    0x00U

/* Undertemperature threshold setting for analog input x. Returned value is 
 * prepared to be placed in register.
 * @param threshold Threshold value in mV.
 */ 
#define BCC_SET_ANX_UT_TH(threshold)                                                   \
    ((uint16_t)((((((uint32_t)(threshold)) * 100U) / 488U) > BCC_RW_ANX_UT_TH_MASK) ?  \
    BCC_RW_ANX_UT_TH_MASK : ((((uint32_t)(threshold)) * 100U) / 488U)) << BCC_RW_ANX_UT_TH_SHIFT)

/* Default value set to 3.82V. */
#define BCC_ANX_UT_TH_DEFAULT     0x30EU

/******************************************************************************/
/* $68 TH_ISENSE_OC - ISENSE over current threshold. */
/******************************************************************************/
#define BCC_REG_TH_ISENSE_OC_ADDR    0x68U

#define BCC_REG_TH_ISENSE_OC_DEFAULT 0x00U

#define BCC_RW_TH_ISENSE_OC_MASK     0x0FFFU

#define BCC_RW_TH_ISENSE_OC_SHIFT    0x00U

/* Overvoltage threshold setting for individual cell terminals. Register 
 * [COMMON_OV_TH] must be logic 0 and [CTx_OVUV_EN] must be logic 1 to use 
 * TH_CTx register as threshold. Returned value is prepared to be placed      
 * in register. 
 *
 * @param threshold Threshold value in uV.
 */
#define BCC_SET_TH_ISENSE_OC(threshold)                                  \
    (((uint16_t)(((threshold) * 5) / 6) << BCC_RW_TH_ISENSE_OC_SHIFT) &  \
            BCC_RW_TH_ISENSE_OC_MASK)

/******************************************************************************/
/* $69 TH_COULOMB_CNT_MSB - Coulomb counter threshold. */
/******************************************************************************/
#define BCC_REG_TH_COULOMB_CNT_MSB_ADDR 0x69U

#define BCC_RW_TH_COULOMB_CNT_MSB_MASK  0xFFFFU

#define BCC_RW_TH_COULOMB_CNT_MSB_SHIFT 0x00U

/* Over Coulomb counting threshold  setting. Returned value is prepared to be       
 * placed in a register. 
 *
 * @param Threshold threshold value in uV.
 */
#define BCC_SET_TH_COULOMB_CNT_MSB(threshold) \
    ((uint16_t)((((threshold) >> 16U) * 10U) / 6U) << BCC_RW_TH_COULOMB_CNT_MSB_SHIFT)
  
/******************************************************************************/
/* $6A TH_COULOMB_CNT_LSB - Coulomb counter threshold. */
/******************************************************************************/
#define BCC_REG_TH_COULOMB_CNT_LSB_ADDR 0x6AU

#define BCC_RW_TH_COULOMB_CNT_LSB_MASK  0xFFFFU

#define BCC_RW_TH_COULOMB_CNT_LSB_SHIFT 0x00U

/* Over Coulomb counting threshold  setting. Returned value is prepared to be       
 * placed in a register. 
 *
 * @param threshold Threshold value in uV.
 */
#define BCC_SET_TH_COULOMB_CNT_LSB(threshold) \
  ((uint16_t)((((threshold) && 0xFFFF) * 10) / 6) << BCC_RW_TH_COULOMB_CNT_LSB_SHIFT)

/******************************************************************************/
/* $6B SILICON_REV - Silicon revision. */
/******************************************************************************/
#define BCC_REG_SILICON_REV_ADDR  0x6BU

#define BCC_R_MREV_MASK  0x0007U
#define BCC_R_FREV_MASK  0x0038U

/******************************************************************************/
/* $6C EEPROM_CTRL - EEPROM transfer control. */
/******************************************************************************/
#define BCC_REG_EEPROM_CTRL_ADDR  0x6CU

/* Bit masks for reading from the register. */
#define BCC_R_READ_DATA_MASK      0x00FFU
#define BCC_R_EE_PRESENT_MASK     0x2000U
#define BCC_R_ERROR_MASK          0x4000U
#define BCC_R_BUSY_MASK           0x8000U
/* Bit masks for writing into the register. */
#define BCC_W_DATA_TO_WRITE_MASK  0x00FFU
#define BCC_W_EEPROM_ADD_MASK     0x7F00U
#define BCC_W_RW_MASK             0x8000U

/* Shifting values for reading from the register. */
#define BCC_R_READ_DATA_SHIFT     0x00U
#define BCC_R_EE_PRESENT_SHIFT    0x0DU
#define BCC_R_ERROR_SHIFT         0x0EU
#define BCC_R_BUSY_SHIFT          0x0FU
/* Shifting values for writing into the register. */
#define BCC_W_DATA_TO_WRITE_SHIFT 0x00U
#define BCC_W_EEPROM_ADD_SHIFT    0x08U
#define BCC_W_RW_SHIFT            0x0FU

/* Read/write bit, directs the 3377x to read or write from EEPROM. */
#define BCC_EEPROM_RW_W           (0x00U << BCC_W_RW_SHIFT)
#define BCC_EEPROM_RW_R           (0x01U << BCC_W_RW_SHIFT)

/******************************************************************************/
/* $6D DED_ENCODE1 - ECC signature 1. */
/******************************************************************************/
#define BCC_REG_DED_ENCODE1_ADDR 0x6DU

#define BCC_R_DED_HAMMING1_MASK  0xFFFFU

/******************************************************************************/
/* $6E DED_ENCODE2 - ECC signature 2. */
/******************************************************************************/
#define BCC_REG_DED_ENCODE2_ADDR 0x6EU

#define BCC_R_DED_HAMMING2_MASK  0xFF00U

/******************************************************************************/
/* $6F FUSE_MIRROR_DATA - Fuse mirror data. */
/******************************************************************************/
#define BCC_REG_FUSE_MIRROR_DATA_ADDR 0x6FU

#define BCC_RW_FMR_DATA_MASK          0xFFFFU

/******************************************************************************/
/* $70 FUSE_MIRROR_CTRL - Fuse mirror address. */
/******************************************************************************/
#define BCC_REG_FUSE_MIRROR_CTRL_ADDR  0x70U

/* Bit masks. */
#define BCC_R_FST_ST_MASK       0x0007U
#define BCC_RW_FMR_ADDR_MASK    0x1F00U
#define BCC_R_SEC_ERR_FLT_MASK  0x8000U

#define BCC_W_FST_MASK          0x0007U
#define BCC_W_FSTM_MASK         0x0008U

/* Shifting values. */
#define BCC_R_FST_ST_SHIFT      0x00U
#define BCC_RW_FMR_ADDR_SHIFT   0x08U
#define BCC_R_SEC_ERR_FLT_SHIFT 0x0FU

#define BCC_W_FST_SHIFT         0x00U
#define BCC_W_FSTM_SHIFT        0x03U

/* Control of write access to the FST bits. */
#define BCC_FSTM_WRITE_EN       (0x01U << BCC_W_FSTM_SHIFT)
#define BCC_FSTM_WRITE_DIS      (0x00U << BCC_W_FSTM_SHIFT)

/* Fuse state control. Write to this register controls the switching of the fuse
 * state machine. */
#define BCC_FST_EN_SPI_WRITE    (0x00U << BCC_W_FST_SHIFT)
#define BCC_FST_LP              (0x04U << BCC_W_FST_SHIFT)

/******************************************************************************/
/* Fuse bank. */
/******************************************************************************/

/* Traceability */
#define BCC_FUSE_TR_0_ADDR_MC33771   0x18U
#define BCC_FUSE_TR_1_ADDR_MC33771   0x19U
#define BCC_FUSE_TR_2_ADDR_MC33771   0x1AU

#define BCC_FUSE_TR_0_ADDR_MC33772   0x10U
#define BCC_FUSE_TR_1_ADDR_MC33772   0x11U
#define BCC_FUSE_TR_2_ADDR_MC33772   0x12U

#define BCC_FUSE_TR_0_MASK           0xFFFFU
#define BCC_FUSE_TR_1_MASK           0xFFFFU
#define BCC_FUSE_TR_2_MASK           0x001FU

#endif /* __BCC_MC3377X_H__ */
/*******************************************************************************
 * EOF;
 ******************************************************************************/
