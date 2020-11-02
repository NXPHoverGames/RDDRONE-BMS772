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
 * @file bcc.c
 *
 * Battery cell controller SW driver V1.1.
 * Supports boards based on MC33771B and MC33772B.
 *
 * This module is common for all supported models.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_spi.h"
#include "bcc_tpl.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Cell map for 7 cells connected to MC33771. */
#define BCC_CM_MC33771_7CELLS     0x380FU
/*! @brief Cell map for 3 cells connected to MC33772. */
#define BCC_CM_MC33772_3CELLS     0x0023U

/*! @brief Time after VPWR connection for the IC to be ready for initialization
 *  (t_VPWR(READY)) in [ms]. */
#define BCC_T_VPWR_READY_MS       5U

/*! @brief CSB_TX LOW period in CSB_TX wake-up pulse sequence (t_1, typ.) in [us]. */
#define BCC_T_WAKE_T1_US          21U

/*! @brief CSB_TX HIGH period in CSB_TX wake-up pulse sequence (t_2, typ.) in [us]. */
#define BCC_T_WAKE_T2_US          600U

/*! @brief EN LOW to HIGH transition to INTB verification pulse
 * (t_INTB_PULSE_DELAY, maximum) in [us]. */
#define BCC_T_INTB_PULSE_DELAY_US 100U

/*! @brief INTB verification pulse duration (t_INTB_PULSE, typ.) in [us]. */
#define BCC_T_INTB_PULSE_US       100U

/*! @brief RESET de-glitch filter (t_RESETFLT, typ.) in [us]. */
#define BCC_T_RESETFLT_US         100U

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/

/** Addresses of configurable registers.
 *
 * Note that MC33772 does not have CB7_CFG - CB14_CFG (0x12 - 0x19) and
 * TH_CT14 - TH_CT7 (0x4c - 0x53) registers. These values are ignored
 * in the initialization. */
static const uint8_t BCC_INIT_CONF_REG_ADDR[BCC_INIT_CONF_REG_CNT] = {
    /* Note: INIT register is initialized automatically. SYS_CFG_GLOBAL register
     *       contains only command GO2SLEEP (no initialization needed). */

    BCC_REG_GPIO_CFG1_ADDR,
    BCC_REG_GPIO_CFG2_ADDR,
    BCC_REG_TH_ALL_CT_ADDR,
    BCC_REG_TH_CT14_ADDR,
    BCC_REG_TH_CT13_ADDR,
    BCC_REG_TH_CT12_ADDR,
    BCC_REG_TH_CT11_ADDR,
    BCC_REG_TH_CT10_ADDR,
    BCC_REG_TH_CT9_ADDR,
    BCC_REG_TH_CT8_ADDR,
    BCC_REG_TH_CT7_ADDR,
    BCC_REG_TH_CT6_ADDR,
    BCC_REG_TH_CT5_ADDR,
    BCC_REG_TH_CT4_ADDR,
    BCC_REG_TH_CT3_ADDR,
    BCC_REG_TH_CT2_ADDR,
    BCC_REG_TH_CT1_ADDR,
    BCC_REG_TH_AN6_OT_ADDR,
    BCC_REG_TH_AN5_OT_ADDR,
    BCC_REG_TH_AN4_OT_ADDR,
    BCC_REG_TH_AN3_OT_ADDR,
    BCC_REG_TH_AN2_OT_ADDR,
    BCC_REG_TH_AN1_OT_ADDR,
    BCC_REG_TH_AN0_OT_ADDR,
    BCC_REG_TH_AN6_UT_ADDR,
    BCC_REG_TH_AN5_UT_ADDR,
    BCC_REG_TH_AN4_UT_ADDR,
    BCC_REG_TH_AN3_UT_ADDR,
    BCC_REG_TH_AN2_UT_ADDR,
    BCC_REG_TH_AN1_UT_ADDR,
    BCC_REG_TH_AN0_UT_ADDR,
    BCC_REG_TH_ISENSE_OC_ADDR,
    BCC_REG_TH_COULOMB_CNT_MSB_ADDR,
    BCC_REG_TH_COULOMB_CNT_LSB_ADDR,
    BCC_REG_CB1_CFG_ADDR,
    BCC_REG_CB2_CFG_ADDR,
    BCC_REG_CB3_CFG_ADDR,
    BCC_REG_CB4_CFG_ADDR,
    BCC_REG_CB5_CFG_ADDR,
    BCC_REG_CB6_CFG_ADDR,
    BCC_REG_CB7_CFG_ADDR,
    BCC_REG_CB8_CFG_ADDR,
    BCC_REG_CB9_CFG_ADDR,
    BCC_REG_CB10_CFG_ADDR,
    BCC_REG_CB11_CFG_ADDR,
    BCC_REG_CB12_CFG_ADDR,
    BCC_REG_CB13_CFG_ADDR,
    BCC_REG_CB14_CFG_ADDR,
    BCC_REG_OV_UV_EN_ADDR,
    BCC_REG_SYS_CFG1_ADDR,
    BCC_REG_SYS_CFG2_ADDR,
    BCC_REG_ADC_CFG_ADDR,
    BCC_REG_ADC2_OFFSET_COMP_ADDR,
    BCC_REG_FAULT_MASK1_ADDR,
    BCC_REG_FAULT_MASK2_ADDR,
    BCC_REG_FAULT_MASK3_ADDR,
    BCC_REG_WAKEUP_MASK1_ADDR,
    BCC_REG_WAKEUP_MASK2_ADDR,
    BCC_REG_WAKEUP_MASK3_ADDR,
    BCC_REG_CELL_OV_FLT_ADDR,
    BCC_REG_CELL_UV_FLT_ADDR,
    BCC_REG_AN_OT_UT_FLT_ADDR,
    BCC_REG_CB_SHORT_FLT_ADDR,
    BCC_REG_GPIO_STS_ADDR,
    BCC_REG_GPIO_SHORT_ADDR,
    BCC_REG_FAULT1_STATUS_ADDR,
    BCC_REG_FAULT2_STATUS_ADDR,
    BCC_REG_FAULT3_STATUS_ADDR,
};

/*******************************************************************************
 * Prototypes of internal functions
 ******************************************************************************/

/*!
 * @brief This function does a transition of CSB from low to high.
 *
 * CSB -> 0 for 21 us
 * CSB -> 1 for 600 us
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
static inline void BCC_WakeUpPatternSpi(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function does two consecutive transitions of CSB_TX from low to
 * high.
 *
 * CSB_TX -> 0 for 21 us
 * CSB_TX -> 1 for 600 us
 * CSB_TX -> 0 for 21 us
 * CSB_TX -> 1 for 21 us
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
static inline void BCC_WakeUpPatternTpl(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function initializes a BCC device or all devices in daisy chain.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param devConf Initialization values of BCC device registers.
 *                devConf[0][x] belongs to device with CID 1,
 *                devConf[1][x] belongs to device with CID 2, etc.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t BCC_InitRegisters(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]);

/*!
 * @brief This function assigns CID to a BCC device that has CID equal to zero.
 * It closes bus switch to allow communication with the next BCC.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster ID of BCC device.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t BCC_AssignCid(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid);

/*!
 * @brief This function wakes device(s) and resets them (if needed), assigns
 * CIDs, and initializes registers of device(s).
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param devConf Initialization values of BCC device registers.
 *                devConf[0] belongs to device with CID 1,
 *                devConf[1] belongs to device with CID 2, etc.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t BCC_InitDevices(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]);

/*******************************************************************************
 * Internal function
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WakeUpPatternSpi
 * Description   : This function does a transition of CSB from low to high.
 *
 *END**************************************************************************/
#pragma GCC push_options
#pragma GCC optimize ("-O2")
static inline void BCC_WakeUpPatternSpi(const bcc_drv_config_t* const drvConfig)
{
    /* A transition of CSB from low to high. */
    /* CSB low. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 0);
    /* Wait for t1; 20 us < t1. */
    BCC_MCU_WaitUs(BCC_T_WAKE_T1_US);

    /* CSB high. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 1);
    /* Wait for t2; 500 us < t2. */
    BCC_MCU_WaitUs(BCC_T_WAKE_T2_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WakeUpPatternTpl
 * Description   : This function does two consecutive transitions of CSB_TX from
 *                 low to high.
 *
 *END**************************************************************************/
static inline void BCC_WakeUpPatternTpl(const bcc_drv_config_t* const drvConfig)
{
    /* CSB_TX low. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 0);
    /* Wait for t1; 20 us < t1. */
    BCC_MCU_WaitUs(BCC_T_WAKE_T1_US);

    /* CSB_TX high. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 1);
    /* Wait for t2; 500 us < t2. */
    BCC_MCU_WaitUs(BCC_T_WAKE_T2_US);

    /* CSB_TX low. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 0);
    /* Wait for t1; 20 us < t1. */
    BCC_MCU_WaitUs(BCC_T_WAKE_T1_US);

    /* CSB_TX high. */
    BCC_MCU_WriteCsbPin(drvConfig->drvInstance, 1);
    /* Time to switch Sleep mode to normal mode after TPL bus wake-up. */
    BCC_MCU_WaitUs(1000U);
}
#pragma GCC pop_options

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_InitRegisters
 * Description   : This function initializes a BCC device or all devices in
 *                 daisy chain.
 *
 *END**************************************************************************/
static bcc_status_t BCC_InitRegisters(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT])
{
    uint8_t i, cid;
    bcc_status_t error;

    /* Initialize all registers according to according to the user values. */
    for (cid = 1; cid <= drvConfig->devicesCnt; cid++)
    {
        for (i = 0; i < BCC_INIT_CONF_REG_CNT; i++)
        {
            if (drvConfig->device[cid - 1U] == BCC_DEVICE_MC33772)
            {
                if (BCC_IS_IN_RANGE(BCC_INIT_CONF_REG_ADDR[i], BCC_REG_CB7_CFG_ADDR, BCC_REG_CB14_CFG_ADDR) ||
                    BCC_IS_IN_RANGE(BCC_INIT_CONF_REG_ADDR[i], BCC_REG_TH_CT14_ADDR, BCC_REG_TH_CT7_ADDR))
                {
                    continue;
                }
            }

            error = BCC_Reg_Write(drvConfig, (bcc_cid_t)cid, BCC_INIT_CONF_REG_ADDR[i],
                                  devConf[cid - 1U][i], NULL);
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
        }
    }

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_AssignCid
 * Description   : This function assigns CID to a BCC device that has CID equal
 *                 to zero.
 *
 *END**************************************************************************/
static bcc_status_t BCC_AssignCid(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid)
{
    uint16_t writeVal, readVal;
    bcc_status_t error;

    /* Check if unassigned node replies. This is the first reading after device
     * reset. */
    error = BCC_Reg_Read(drvConfig, BCC_CID_UNASSIG, BCC_REG_INIT_ADDR, 1U, &readVal);

    /* Note: in SPI communication mode the device responds with all zero and the
     * correct CRC (null response) during the very first message. */
    if ((error != BCC_STATUS_SUCCESS) && (error != BCC_STATUS_NULL_RESP))
    {
        return error;
    }

    /* Assign CID and close the bus switch to be able to initialize next BCC
     * device. Bus switch of the last BCC in chain stays opened. In SPI mode,
     * just CID needs to be written.
     * Note: It is forbidden to use global write command to assign CID (writing
     * into INIT register). */
    if ((uint8_t)cid < drvConfig->devicesCnt)
    {
        writeVal = BCC_SET_CID(readVal, (uint8_t)cid) | BCC_BUS_SWITCH_ENABLED | BCC_RTERM_COMM_SW;
    }
    else
    {
        writeVal = BCC_SET_CID(readVal, (uint8_t)cid) | BCC_BUS_SWITCH_DISABLED | BCC_RTERM_COMM_SW;
    }

    error = BCC_Reg_Write(drvConfig, BCC_CID_UNASSIG, BCC_REG_INIT_ADDR, writeVal, NULL);
    if (error == BCC_STATUS_SUCCESS)
    {
        /* Check if assigned node replies. */
        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_INIT_ADDR, 1U, &readVal);
    }

    if (error != BCC_STATUS_SUCCESS)
    {
        /* Wait and try to assign CID once again. */
        BCC_MCU_WaitUs(750U);

        error = BCC_Reg_Write(drvConfig, BCC_CID_UNASSIG, BCC_REG_INIT_ADDR, writeVal, NULL);
        if (error == BCC_STATUS_SUCCESS)
        {
            error = BCC_Reg_Read(drvConfig, cid, BCC_REG_INIT_ADDR, 1U, &readVal);
        }
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_InitDevices
 * Description   : This function wakes device(s) and resets them (if needed),
 *                 assigns CIDs, and initializes registers of device(s).
 *
 *
 *END**************************************************************************/
static bcc_status_t BCC_InitDevices(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT])
{
    uint8_t cid;
    bcc_status_t error = BCC_STATUS_SUCCESS;

    /* Wake-up all configured devices (in case of SLEEP mode) or:
     * Move the first device (closest to MC33664) from IDLE to NORMAL mode
     * (if it is in IDLE mode). */
    BCC_WakeUp(drvConfig);

    /* Reset (soft reset) all configured devices (in case CID was already assigned). */
    (void)BCC_SoftwareReset(drvConfig, (drvConfig->commMode == BCC_MODE_TPL) ? BCC_CID_UNASSIG : BCC_CID_DEV1);

    /* Wait for 5 ms - for the IC to be ready for initialization. */
    BCC_MCU_WaitMs(BCC_T_VPWR_READY_MS);

    /* Assign CIDs and close bus switches. */
    error = BCC_AssignCid(drvConfig, BCC_CID_DEV1);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    for (cid = 2U; cid <= drvConfig->devicesCnt; cid++)
    {
        BCC_MCU_WaitMs(2U);

        /* Move the following device from IDLE to NORMAL mode (if it is in IDLE mode). */
        BCC_WakeUpPatternTpl(drvConfig);

        error = BCC_AssignCid(drvConfig, (bcc_cid_t)cid);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    /* Initialize registers of device(s) according to user values. */
    if (devConf != NULL)
    {
        error = BCC_InitRegisters(drvConfig, devConf);
    }

    return error;
}

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Init
 * Description   : This function initializes the Battery Cell Controller
 *                 device(s), configures its registers, assigns CID and
 *                 initializes internal driver data.
 *
 *END**************************************************************************/
bcc_status_t BCC_Init(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT])
{
    uint8_t dev;
    uint8_t cell;
    uint8_t cid;
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    /* Check the drvConfig structure. */
    if ((drvConfig->devicesCnt == 0) || (drvConfig->devicesCnt >
            ((drvConfig->commMode == BCC_MODE_SPI) ? BCC_DEVICE_CNT_MAX_SPI : BCC_DEVICE_CNT_MAX_TPL)))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    for (dev = 0; dev < drvConfig->devicesCnt; dev++)
    {
        if (drvConfig->device[dev] == BCC_DEVICE_MC33771)
        {
            if (!BCC_IS_IN_RANGE(drvConfig->cellCnt[dev], BCC_MIN_CELLS_MC33771, BCC_MAX_CELLS_MC33771))
            {
                return BCC_STATUS_PARAM_RANGE;
            }
        }
        else
        {
            if (!BCC_IS_IN_RANGE(drvConfig->cellCnt[dev], BCC_MIN_CELLS_MC33772, BCC_MAX_CELLS_MC33772))
            {
                return BCC_STATUS_PARAM_RANGE;
            }
        }
    }

    /* Initialize driver variables. */
    for (dev = 0; dev < drvConfig->devicesCnt; dev++)
    {
        if (drvConfig->device[dev] == BCC_DEVICE_MC33771)
        {
            drvConfig->drvData.cellMap[dev] = BCC_CM_MC33771_7CELLS;
            for (cell = BCC_MIN_CELLS_MC33771; cell < drvConfig->cellCnt[dev]; cell++)
            {
                drvConfig->drvData.cellMap[dev] |= (0x0400U >> (cell - BCC_MIN_CELLS_MC33771));
            }
        }
        else
        {
            drvConfig->drvData.cellMap[dev] = BCC_CM_MC33772_3CELLS;
            for (cell = BCC_MIN_CELLS_MC33772; cell < drvConfig->cellCnt[dev]; cell++)
            {
                drvConfig->drvData.cellMap[dev] |= (0x0010U >> (cell - BCC_MIN_CELLS_MC33772));
            }
        }
    }

    /* Initialize TAG ID. */
    for (cid = 0; cid < drvConfig->devicesCnt; cid++)
    {
        drvConfig->drvData.rcTbl[cid] = 0U;
        drvConfig->drvData.tagId[cid] = 0U;
    }

    /* RESET -> 0. */
    BCC_MCU_WriteRstPin(drvConfig->drvInstance, 0);

    if (drvConfig->commMode == BCC_MODE_TPL)
    {
        /* Enable MC33664 device. */
        if ((error = BCC_TPL_Enable(drvConfig)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    /* Wake-up BCC (if case of idle/sleep mode), resets them, assigns CID,
     * initialize registers and check communication with configured devices. */
    return BCC_InitDevices(drvConfig, devConf);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_VerifyCom
 * Description   : This function uses No Operation command of BCC to verify
 *                 communication with device specified by CID without performing
 *                 any operation.
 *
 *END**************************************************************************/
bcc_status_t BCC_VerifyCom(bcc_drv_config_t* const drvConfig, bcc_cid_t cid)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        return BCC_VerifyComSpi(drvConfig, cid);
    }
    else
    {
        return BCC_VerifyComTpl(drvConfig, cid);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Sleep
 * Description   : This function sets sleep mode to all Battery Cell Controller
 *                 devices.
 *
 *END**************************************************************************/
bcc_status_t BCC_Sleep(bcc_drv_config_t* const drvConfig)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        return BCC_Reg_Write(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG_GLOBAL_ADDR,
                             BCC_GO2SLEEP_ENABLED, NULL);
    }
    else
    {
        return BCC_Reg_WriteGlobal(drvConfig, BCC_REG_SYS_CFG_GLOBAL_ADDR,
                                    BCC_GO2SLEEP_ENABLED);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WakeUp
 * Description   : This function sets normal mode to all Battery Cell Controller
 *                 devices.
 *
 *END**************************************************************************/
void BCC_WakeUp(const bcc_drv_config_t* const drvConfig)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        BCC_WakeUpPatternSpi(drvConfig);
    }
    else
    {
        BCC_WakeUpPatternTpl(drvConfig);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_SoftwareReset
 * Description   : This function resets BCC device using software reset. It
 *                 enters reset via SPI or TPL interface.
 *
 *END**************************************************************************/
bcc_status_t BCC_SoftwareReset(bcc_drv_config_t* const drvConfig, bcc_cid_t cid)
{
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    /* Note: it is not necessary to read content of SYS_CFG1 register
    * to change only RST bit, because registers are set to default values. */
    if ((((uint8_t)cid) > drvConfig->devicesCnt) ||
            ((cid == BCC_CID_UNASSIG) && (drvConfig->commMode == BCC_MODE_SPI)))
    {
        return BCC_STATUS_PARAM_RANGE;
    }
    else if (cid == BCC_CID_UNASSIG)
    {
        /* TPL Global reset command. */
        error = BCC_Reg_WriteGlobal(drvConfig, BCC_REG_SYS_CFG1_ADDR, BCC_W_SOFT_RST_MASK);
    }
    else
    {
        error = BCC_Reg_Write(drvConfig, cid, BCC_REG_SYS_CFG1_ADDR, BCC_W_SOFT_RST_MASK, NULL);
        if (error == BCC_STATUS_COM_TIMEOUT)
        {
            /* Device does not respond after reset - normal condition. */
            error = BCC_STATUS_SUCCESS;
        }
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_HardwareReset
 * Description   : This function resets BCC device using GPIO pin.
 *
 *END**************************************************************************/
void BCC_HardwareReset(const bcc_drv_config_t* const drvConfig)
{
    BCC_MCU_Assert(drvConfig != NULL);

    BCC_MCU_WriteRstPin(drvConfig->drvInstance, 1);
    /* Wait at least t_RESETFLT (100 us). */
    BCC_MCU_WaitUs(BCC_T_RESETFLT_US);
    BCC_MCU_WriteRstPin(drvConfig->drvInstance, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_TPL_Enable
 * Description   : This function enables MC33664 TPL device. It uses EN and
 *                 INTB pins.
 *
 *END**************************************************************************/
bcc_status_t BCC_TPL_Enable(const bcc_drv_config_t* const drvConfig)
{
    int32_t timeout;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(drvConfig->commMode == BCC_MODE_TPL);

    /* Set normal state (transition from low to high). */
    BCC_MCU_WriteEnPin(drvConfig->drvInstance, 0);
    /* Wait at least 100 us. */
    BCC_MCU_WaitUs(150);
    BCC_MCU_WriteEnPin(drvConfig->drvInstance, 1);

    /* Note: MC33664 has time t_Ready/t_INTB_PULSE_DELAY (max. 100 us) to take effect.
     * Wait for INTB transition from high to low (max. 100 us). */
    timeout = BCC_T_INTB_PULSE_DELAY_US;
    while ((BCC_MCU_ReadIntbPin(drvConfig->drvInstance) > 0) && (timeout > 0))
    {
        timeout -= 5;
        BCC_MCU_WaitUs(5U);
    }
    if (timeout <= 0)
    {
        return BCC_STATUS_COM_TIMEOUT;
    }

    /* Wait for INTB transition from low to high (typ. 100 us).
     * Wait for at most 200 us. */
    timeout = BCC_T_INTB_PULSE_US * 2;
    while ((BCC_MCU_ReadIntbPin(drvConfig->drvInstance) == 0) && (timeout > 0))
    {
        timeout -= 10;
        BCC_MCU_WaitUs(10U);
    }
    if (timeout <= 0)
    {
        return BCC_STATUS_COM_TIMEOUT;
    }

    /* Now the device should be in normal mode (i.e. after INTB low to high
    * transition). For sure wait for 150 us. */
    BCC_MCU_WaitUs(150U);

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_TPL_Disable
 * Description   : This function sets MC33664 device into sleep mode.
 *
 *END**************************************************************************/
void BCC_TPL_Disable(const bcc_drv_config_t* const drvConfig)
{
    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(drvConfig->commMode == BCC_MODE_TPL);

    BCC_MCU_WriteEnPin(drvConfig->drvInstance, 0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_Read
 * Description   : This function reads a value from addressed register (or
 *                 desired number of registers) of selected Battery Cell
 *                 Controller device.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        return BCC_Reg_ReadSpi(drvConfig, cid, regAddr, regCnt, regVal);
    }
    else
    {
        return BCC_Reg_ReadTpl(drvConfig, cid, regAddr, regCnt, regVal);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_Write
 * Description   : This function writes a value to addressed register of
 *                 selected Battery Cell Controller device.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_Write(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regVal, uint16_t* retReg)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        return BCC_Reg_WriteSpi(drvConfig, cid, regAddr, regVal, retReg);
    }
    else
    {
        return BCC_Reg_WriteTpl(drvConfig, cid, regAddr, regVal, retReg);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_WriteGlobal
 * Description   : This function writes a value to addressed register of all
 *                 configured BCC devices. Intended for TPL mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_WriteGlobal(bcc_drv_config_t* const drvConfig,
     uint8_t regAddr, uint16_t regVal)
{
    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(drvConfig->commMode == BCC_MODE_TPL);

    return BCC_Reg_WriteGlobalTpl(drvConfig, regAddr, regVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Reg_Update
 * Description   : This function updates content of a selected register. It
 *                 affects bits specified by a bit mask only.
 *
 *END**************************************************************************/
bcc_status_t BCC_Reg_Update(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regMask, uint16_t regVal)
{
    uint16_t regValTemp;
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if (((uint8_t)cid) > drvConfig->devicesCnt)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    error = BCC_Reg_Read(drvConfig, cid, regAddr, 1U, &regValTemp);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Update register value. */
    regValTemp = BCC_REG_UNSET_BIT_VALUE(regValTemp, regMask);
    regValTemp = BCC_REG_SET_BIT_VALUE(regValTemp, (regVal & regMask));

    return BCC_Reg_Write(drvConfig, cid, regAddr, regValTemp, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Meas_StartConversion
 * Description   : This function starts ADC conversion. It sets Start of
 *                 Conversion bit and new value of TAG ID in ADC_CFG register.
 *
 *END**************************************************************************/
bcc_status_t BCC_Meas_StartConversion(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid)
{
    uint16_t regVal;     /* Value of ADC_CFG register. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_ADC_CFG_ADDR, 1U, &regVal);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Increment TAG ID (4 bit value). */
    drvConfig->drvData.tagId[(uint8_t)cid - 1] = (drvConfig->drvData.tagId[(uint8_t)cid - 1] + 1U) & 0x0FU;

    /* Set new TAG ID and Start of Conversion bit. */
    regVal = BCC_SET_TAG_ID(regVal, drvConfig->drvData.tagId[(uint8_t)cid - 1]);
    regVal = BCC_REG_SET_BIT_VALUE(regVal, BCC_W_SOC_MASK);

    return BCC_Reg_Write(drvConfig, cid, BCC_REG_ADC_CFG_ADDR, regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Meas_StartConversionGlobal
 * Description   : This function starts ADC conversion for all devices in TPL
 *                 chain. It uses a Global Write command to set ADC_CFG
 *                 register. Intended for TPL mode only!
 *
 *END**************************************************************************/
bcc_status_t BCC_Meas_StartConversionGlobal(bcc_drv_config_t* const drvConfig,
    uint16_t adcCfgValue)
{
    uint8_t dev;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(drvConfig->commMode == BCC_MODE_TPL);

    /* Increment & Use TAG ID (4 bit value) of the first node. */
    drvConfig->drvData.tagId[0] = (drvConfig->drvData.tagId[0] + 1U) & 0x0FU;

    /* Set Tag ID to all BCCs in the driver configuration structure. */
    for (dev = 1; dev < drvConfig->devicesCnt; dev++)
    {
        drvConfig->drvData.tagId[dev] = drvConfig->drvData.tagId[0];
    }

    /* Set new TAG ID and Start of Conversion bit. */
    adcCfgValue = BCC_SET_TAG_ID(adcCfgValue, drvConfig->drvData.tagId[0]);
    adcCfgValue = BCC_REG_SET_BIT_VALUE(adcCfgValue, BCC_W_SOC_MASK);

    return BCC_Reg_WriteGlobal(drvConfig, BCC_REG_ADC_CFG_ADDR, adcCfgValue);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Meas_IsConverting
 * Description   : This function checks status of conversion defined by End of
 *                 Conversion bit in ADC_CFG register.
 *
 *END**************************************************************************/
bcc_status_t BCC_Meas_IsConverting(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bool* completed)
{
    uint16_t regVal;     /* Value of ADC_CFG register. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(completed != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_ADC_CFG_ADDR, 1U, &regVal);

    regVal = regVal & BCC_R_EOC_N_MASK;
    *(completed) = (regVal == 0x00U);

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Meas_GetRawValues
 * Description   : This function reads the measurement registers and returns raw
 *                 values. You can use macros defined in BCC header file to
 *                 perform correct unit conversion.
 *
 *END**************************************************************************/
bcc_status_t BCC_Meas_GetRawValues(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t measurements[])
{
    bcc_status_t error;
    uint8_t i;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(measurements != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Read all the measurement registers.
    * Note: the order and number of registers conforms to the order of measured
    * values in Measurements array, see enumeration bcc_measurements_t. */
    if (drvConfig->device[(uint8_t)cid - 1] == BCC_DEVICE_MC33771)
    {
        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_CC_NB_SAMPLES_ADDR,
                             BCC_MEAS_CNT, measurements);
    }
    else
    {
        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_CC_NB_SAMPLES_ADDR,
                             (BCC_REG_MEAS_STACK_ADDR - BCC_REG_CC_NB_SAMPLES_ADDR) + 1, measurements);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        /* Skip the reserved registers. */
        measurements[BCC_MSR_CELL_VOLT14] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT13] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT12] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT11] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT10] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT9] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT8] = 0x0000;
        measurements[BCC_MSR_CELL_VOLT7] = 0x0000;

        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_MEAS_CELLX_ADDR_MC33772_START,
                             (BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR - BCC_REG_MEAS_CELLX_ADDR_MC33772_START) + 1,
                             (uint16_t *)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));
    }

    /* Mask bits. */
    /* Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2 registers. */
    measurements[BCC_MSR_ISENSE1] &= BCC_R_MEAS1_I_MASK;
    measurements[BCC_MSR_ISENSE2] &= BCC_R_MEAS2_I_MASK;

    /* Mask the other registers (starting at 5th register). */
    for (i = 5U; i < BCC_MEAS_CNT; i++)
    {
        measurements[i] &= BCC_R_MEAS_MASK;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Fault_GetStatus
 * Description   : This function reads the status registers and returns raw
 *                 values. You can use constants defined in bcc_mc3377x.h file.
 *
 *END**************************************************************************/
bcc_status_t BCC_Fault_GetStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t status[])
{
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(status != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Read CELL_OV_FLT and CELL_UV_FLT. */
    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_CELL_OV_FLT_ADDR, 2U, &status[BCC_FS_CELL_OV]);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Read CB_OPEN_FLT, CB_SHORT_FLT. */
    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_CB_OPEN_FLT_ADDR, 2U, &status[BCC_FS_CB_OPEN]);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Read GPIO_STS, AN_OT_UT_FLT, GPIO_SHORT_Anx_OPEN_STS. */
    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_GPIO_STS_ADDR, 3U, &status[BCC_FS_GPIO_STATUS]);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Read COM_STATUS, FAULT1_STATUS, FAULT2_STATUS and FAULT3_STATUS. */
    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_COM_STATUS_ADDR, 4U, &status[BCC_FS_COMM]);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Fault_ClearStatus
 * Description   : This function clears selected fault status register.
 *
 *END**************************************************************************/
bcc_status_t BCC_Fault_ClearStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bcc_fault_status_t statSel)
{
    /* This array is intended for conversion of bcc_fault_status_t value to
     * a BCC register address. */
    const uint8_t REG_ADDR_MAP[BCC_STAT_CNT] = {
        BCC_REG_CELL_OV_FLT_ADDR, BCC_REG_CELL_UV_FLT_ADDR,
        BCC_REG_CB_OPEN_FLT_ADDR, BCC_REG_CB_SHORT_FLT_ADDR,
        BCC_REG_GPIO_STS_ADDR, BCC_REG_AN_OT_UT_FLT_ADDR,
        BCC_REG_GPIO_SHORT_ADDR, BCC_REG_COM_STATUS_ADDR,
        BCC_REG_FAULT1_STATUS_ADDR, BCC_REG_FAULT2_STATUS_ADDR,
        BCC_REG_FAULT3_STATUS_ADDR
    };

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }
    /* Note: COM_STATUS register is read only. */
    if (((uint32_t)statSel >= BCC_STAT_CNT) || (statSel == BCC_FS_COMM))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    return BCC_Reg_Write(drvConfig, cid, REG_ADDR_MAP[statSel], 0x00U, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_GPIO_SetOutput
 * Description   : This function sets output value of one BCC GPIO pin.
 *
 *END**************************************************************************/
bcc_status_t BCC_GPIO_SetOutput(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t gpioSel, bool val)
{
    uint16_t regVal;    /* Value of GPIO_CFG2 register. */
    bcc_status_t error;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt) || (gpioSel >= BCC_GPIO_INPUT_CNT))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* Read and update content of GPIO_CFG2 register. */
    error = BCC_Reg_Read(drvConfig, cid, BCC_REG_GPIO_CFG2_ADDR, 1U, &regVal);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Set GPIO output value. */
    regVal &= ~((uint16_t)BCC_RW_GPIOX_DR_MASK(gpioSel));
    regVal |= (uint16_t)((val) ? BCC_GPIOx_HIGH(gpioSel) : BCC_GPIOx_LOW(gpioSel));

    return BCC_Reg_Write(drvConfig, cid, BCC_REG_GPIO_CFG2_ADDR, regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CB_Enable
 * Description   : This function enables or disables the cell balancing via
 *                 SYS_CFG1[CB_DRVEN] bit.
 *
 *END**************************************************************************/
bcc_status_t BCC_CB_Enable(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool enable)
{
    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    return BCC_Reg_Update(drvConfig, cid, BCC_REG_SYS_CFG1_ADDR, BCC_RW_CB_DRVEN_MASK,
                          enable ? BCC_CB_DRV_ENABLED : BCC_CB_DRV_DISABLED);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CB_SetIndividual
 * Description   : This function sets state of individual cell balancing driver
 *                 and sets its timer.
 *
 *END**************************************************************************/
bcc_status_t BCC_CB_SetIndividual(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t cellIndex, bool enable, uint16_t timer)
{
    uint16_t regVal;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (cellIndex > (BCC_MAX_CELLS_DEV(drvConfig->device[(uint8_t)cid - 1]) - 1))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (timer > BCC_RW_CB_TIMER_MASK)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    regVal = enable ? BCC_CB_ENABLED : BCC_CB_DISABLED;
    regVal |= (timer & BCC_RW_CB_TIMER_MASK) << BCC_RW_CB_TIMER_SHIFT;

    return BCC_Reg_Write(drvConfig, cid, BCC_REG_CB1_CFG_ADDR + cellIndex, regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CB_Pause
 * Description   : This function can be used to manual pause cell balancing
 *                 before on demand conversion.
 *
 *END**************************************************************************/
bcc_status_t BCC_CB_Pause(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool pause)
{
    uint16_t regVal = (pause) ? BCC_CB_MAN_PAUSE_ENABLED : BCC_CB_MAN_PAUSE_DISABLED;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    return BCC_Reg_Update(drvConfig, cid, BCC_REG_SYS_CFG1_ADDR,
                          BCC_RW_CB_MANUAL_PAUSE_MASK, regVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_FuseMirror_Read
 * Description   : This function reads a fuse mirror register of a BCC device
 *                 specified by CID.
 *
 *END**************************************************************************/
bcc_status_t BCC_FuseMirror_Read(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t fuseAddr, uint16_t* const value)
{
    bcc_status_t error = BCC_STATUS_SUCCESS;
    uint16_t regVal;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(value != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (fuseAddr > BCC_MAX_FUSE_ADDR)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    regVal = (fuseAddr << BCC_RW_FMR_ADDR_SHIFT) | BCC_FSTM_WRITE_DIS | BCC_FST_EN_SPI_WRITE;
    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_FUSE_MIRROR_CTRL_ADDR, regVal, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    return BCC_Reg_Read(drvConfig, cid, BCC_REG_FUSE_MIRROR_DATA_ADDR, 1, value);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_FuseMirror_Write
 * Description   : This function writes a fuse mirror register of a BCC device
 *                 specified by CID.
 *
 *END**************************************************************************/
bcc_status_t BCC_FuseMirror_Write(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t fuseAddr, uint16_t value)
{
    bcc_status_t error = BCC_STATUS_SUCCESS;
    uint16_t regVal;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (fuseAddr > BCC_MAX_FUSE_ADDR)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* FUSE_MIRROR_CNTL to enable writing. */
    regVal = BCC_FSTM_WRITE_EN | BCC_FST_EN_SPI_WRITE;
    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_FUSE_MIRROR_CTRL_ADDR, regVal, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    regVal = (fuseAddr << BCC_RW_FMR_ADDR_SHIFT) | BCC_FSTM_WRITE_EN | BCC_FST_EN_SPI_WRITE;
    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_FUSE_MIRROR_CTRL_ADDR, regVal, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_FUSE_MIRROR_DATA_ADDR, value, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* FUSE_MIRROR_CNTL to low power. */
    regVal = BCC_FSTM_WRITE_EN | BCC_FST_LP;
    return BCC_Reg_Write(drvConfig, cid, BCC_REG_FUSE_MIRROR_CTRL_ADDR, regVal, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_GUID_Read
 * Description   : This function reads an unique serial number of the BCC device
 *                 from the content of mirror registers.
 *
 *END**************************************************************************/
bcc_status_t BCC_GUID_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint64_t* const guid)
{
    uint8_t addrMc33771[3] = {BCC_FUSE_TR_0_ADDR_MC33771,
            BCC_FUSE_TR_1_ADDR_MC33771, BCC_FUSE_TR_2_ADDR_MC33771};
    uint8_t addrMc33772[3] = {BCC_FUSE_TR_0_ADDR_MC33772,
            BCC_FUSE_TR_1_ADDR_MC33772, BCC_FUSE_TR_2_ADDR_MC33772};
    uint8_t const *readAddr;
    uint16_t readData[3];
    uint8_t i;
    bcc_status_t error = BCC_STATUS_SUCCESS;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(guid != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    readAddr = (drvConfig->device[(uint8_t)cid - 1] == BCC_DEVICE_MC33771) ? addrMc33771 : addrMc33772;

    for (i = 0; i < 3; i++)
    {
        error = BCC_FuseMirror_Read(drvConfig, cid, readAddr[i], &(readData[i]));
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    *guid = (((uint64_t)(readData[0] & BCC_FUSE_TR_0_MASK)) << 21) |
            (((uint64_t)(readData[1] & BCC_FUSE_TR_1_MASK)) << 5) |
            ((uint64_t)(readData[2] & BCC_FUSE_TR_2_MASK));

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_EEPROM_Read
 * Description   : This function reads a byte from specified address of EEPROM
 *                 memory connected to BCC device via I2C bus.
 *
 *END**************************************************************************/
bcc_status_t BCC_EEPROM_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t addr, uint8_t* const data)
{
    bcc_status_t error = BCC_STATUS_SUCCESS;
    uint16_t regVal;

    BCC_MCU_Assert(drvConfig != NULL);
    BCC_MCU_Assert(data != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (addr > BCC_MAX_EEPROM_ADDR)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* EEPROM Read command. */
    regVal = BCC_EEPROM_RW_R | ((addr << BCC_W_EEPROM_ADD_SHIFT) & BCC_W_EEPROM_ADD_MASK);
    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_EEPROM_CTRL_ADDR, regVal, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Wait while data is read from EEPROM. */
    regVal = BCC_R_BUSY_MASK;
    while (regVal & BCC_R_BUSY_MASK)
    {
        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_EEPROM_CTRL_ADDR, 1U, &regVal);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    if (regVal & BCC_R_EE_PRESENT_MASK)
    {
        return BCC_STATUS_EEPROM_PRESENT;
    }

    if (regVal & BCC_R_ERROR_MASK)
    {
        return BCC_STATUS_EEPROM_ERROR;
    }

    /* Store read data to memory space defined by the pointer. */
    *data = (uint8_t)(regVal & BCC_R_READ_DATA_MASK);

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_EEPROM_Write
 * Description   : This function writes a byte to specified address of EEPROM
 *                 memory connected to BCC device via I2C bus.
 *
 *END**************************************************************************/
bcc_status_t BCC_EEPROM_Write(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t addr, uint8_t data)
{
    bcc_status_t error = BCC_STATUS_SUCCESS;
    uint16_t regVal;

    BCC_MCU_Assert(drvConfig != NULL);

    if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > drvConfig->devicesCnt))
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    if (addr > BCC_MAX_EEPROM_ADDR)
    {
        return BCC_STATUS_PARAM_RANGE;
    }

    /* EEPROM Write command. */
    regVal = BCC_EEPROM_RW_W |
            ((addr << BCC_W_EEPROM_ADD_SHIFT) & BCC_W_EEPROM_ADD_MASK) |
            ((data << BCC_W_DATA_TO_WRITE_SHIFT) & BCC_W_DATA_TO_WRITE_MASK);
    error = BCC_Reg_Write(drvConfig, cid, BCC_REG_EEPROM_CTRL_ADDR, regVal, NULL);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Wait while data is written to EEPROM. */
    regVal = BCC_R_BUSY_MASK;
    while (regVal & BCC_R_BUSY_MASK)
    {
        error = BCC_Reg_Read(drvConfig, cid, BCC_REG_EEPROM_CTRL_ADDR, 1U, &regVal);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    if (regVal & BCC_R_EE_PRESENT_MASK)
    {
        return BCC_STATUS_EEPROM_PRESENT;
    }

    if (regVal & BCC_R_ERROR_MASK)
    {
        return BCC_STATUS_EEPROM_ERROR;
    }

    return BCC_STATUS_SUCCESS;
}
