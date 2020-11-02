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
 * @file bcc.h
 *
 * Battery cell controller SW driver V1.1.
 * Supports boards based on MC33771B and MC33772B.
 *
 * This module is common for all supported models.
 */

#ifndef __BCC_H__
#define __BCC_H__

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "bcc_mc3377x.h"

/*******************************************************************************
 * User definitions
 ******************************************************************************/

/*! @brief Use \#define BCC_MSG_BIGEND for big-endian format of the TX/RX SPI
 *  buffer ([0] DATA_H, [1] DATA_L, ..., [4] CRC). If BCC_MSG_BIGEND is not
 *  defined, little-endian is used ([0] CRC, ..., [3] DATA_L, [4] DATA_H) */
//#define BCC_MSG_BIGEND

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Maximal number of Battery Cell Controller devices in SPI mode. */
#define BCC_DEVICE_CNT_MAX_SPI    1U
/*! @brief Maximal number of Battery Cell Controller devices in TPL mode. */
#define BCC_DEVICE_CNT_MAX_TPL    15U
/*! @brief Maximal number of Battery Cell Controller devices. */
#define BCC_DEVICE_CNT_MAX        BCC_DEVICE_CNT_MAX_TPL

/*! @brief Minimal battery cell count connected to MC33771. */
#define BCC_MIN_CELLS_MC33771     7U
/*! @brief Maximal battery cell count connected to MC33771. */
#define BCC_MAX_CELLS_MC33771     14U
/*! @brief Minimal battery cell count connected to MC33772. */
#define BCC_MIN_CELLS_MC33772     3U
/*! @brief Maximal battery cell count connected to MC33772. */
#define BCC_MAX_CELLS_MC33772     6U
/*! @brief Maximal battery cell count connected to any BCC device. */
#define BCC_MAX_CELLS             14U

/*! @brief Maximal battery cell count connected to MC33772.
 *
 * @param dev BCC device type.
 */
#define BCC_MAX_CELLS_DEV(dev)                             \
    ((dev == BCC_DEVICE_MC33771) ? BCC_MAX_CELLS_MC33771 : BCC_MAX_CELLS_MC33772)

/*!
 * @brief Returns a non-zero value when desired cell (cellNo) is connected
 * to the BCC specified by CID. Otherwise returns zero.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param cellNo Number of a cell (range is {1, ..., 14} for MC33771 and
 *               {1, ..., 6} for MC33772).
 * @return Non-zero value if cell is connected, zero otherwise.
 */
#define BCC_IS_CELL_CONN(drvConfig, cid, cellNo) \
    ((drvConfig)->drvData.cellMap[(cid) - 1U] & (1U << ((cellNo) - 1U)))

/*! @brief Maximal frequency of SPI clock in SPI mode. */
#define BCC_SPI_FREQ_MC3377x_MAX  4200000U
/*! @brief Minimal frequency of SPI_TX clock in TPL mode. */
#define BCC_SPI_FREQ_MC33664_MIN  1900000U
/*! @brief Maximal frequency of SPI_TX clock in TPL mode. */
#define BCC_SPI_FREQ_MC33664_MAX  2100000U

/*! @brief  Number of MC33771 registers configured in the initialization with
 * user values.
 *
 * Note that the number of configured registers of MC33772 is 52 only.
 * See BCC_INIT_CONF_REG_ADDR in bcc.c for more details. */
#define BCC_INIT_CONF_REG_CNT     68U

/*! @brief  Number of MC33771 measurement registers.
 *
 * Note MC33772 contains 22 measurement registers. For compliance
 * with bcc_measurements_t indexes, BCC_Meas_GetRawValues function
 * requires 30 x uint16_t array for both BCC devices. */
#define BCC_MEAS_CNT              30U

/*! @brief  Number of BCC status registers. */
#define BCC_STAT_CNT              11U

/*! @brief Message size in bytes. */
#define BCC_MSG_SIZE              5U

/*! @brief Max. number of frames that can be read at once in TPL mode. */
#define BCC_RX_LIMIT_TPL          0x7FU

/*! @brief Size of buffer that is used for receiving via SPI in TPL mode. */
#define BCC_RX_BUF_SIZE_TPL \
    (BCC_MSG_SIZE * (BCC_RX_LIMIT_TPL + 1U))

/*! @brief Number of GPIO/temperature sensor inputs. */
#define BCC_GPIO_INPUT_CNT        7U

/*!
 * @brief Calculates ISENSE value in [uV]. Resolution is
 * 0.6 uV/LSB. Result is int32_t type.
 * Note: Vind (Differential Input Voltage Range) is min. -150 mV
 * and max. 150 mV (see datasheet).
 *
 * @param iSense1 Content of register MEAS_ISENSE1.
 * @param iSense2 Content of register MEAS_ISENSE2.
 * @return ISENSE voltage in [uV].
 */
#define BCC_GET_ISENSE_VOLT(iSense1, iSense2) \
    ((BCC_GET_ISENSE_RAW_SIGN(BCC_GET_ISENSE_RAW(iSense1, iSense2)) * 6) / 10)

/*!
 * @brief This macro calculates ISENSE value in [mA]. Resolution is
 * (600/R_SHUNT) mA/LSB (V2Res / Rshunt = 1000 * 0.6 uV / rShunt uOhm).
 *
 * @param rShunt Resistance of Shunt resistor in [uOhm].
 * @param iSense1 Content of register MEAS_ISENSE1.
 * @param iSense2 Content of register MEAS_ISENSE2.
 * @return ISENSE current in [mA].
 */
#define BCC_GET_ISENSE_AMP(rShunt, iSense1, iSense2) ( \
    (BCC_GET_ISENSE_RAW_SIGN(BCC_GET_ISENSE_RAW(iSense1, iSense2)) * 600) /  (int32_t)(rShunt)                                                       \
)

/*!
 * @brief This macro converts value of the MEAS_STACK register to [uV].
 * Resolution is 2.4414 mV/LSB. Result is in range 0 - 80 000 000 uV.
 *
 * @param reg Value of the MEAS_STACK register.
 * @return Converted value in [uV].
 */
#define BCC_GET_STACK_VOLT(reg) \
    ((uint32_t)BCC_GET_MEAS_RAW(reg) * 24414U / 10U)

/*!
 * @brief Converts value of a register to [uV]. Resolution is 152.59 uV/LSB.
 * Result is in range 0 - 5 000 000 uV.
 * This macro is intended for the following registers: MEAS_CELLx, MEAS_ANx,
 * MEAS_IC_TEMP, MEAS_VBG_DIAG_ADC1A and MEAS_VBG_DIAG_ADC1B.
 *
 * @param reg Value of a measurement register.
 * @return Converted value in [uV].
 */
#define BCC_GET_VOLT(reg) \
    (((uint32_t)BCC_GET_MEAS_RAW(reg) * 15259U) / 100U)

/*!
 * @brief Converts value of the MEAS_IC_TEMP register to degrees Celsius.
 * Resolution is 0.032 Kelvin/LSB.
 *
 * @param reg Value of the MEAS_IC_TEMP register.
 * @return Converted value in [deg C] multiplied by 10
 *         (i.e. resolution of 0.1 deg C).
 */
#define BCC_GET_IC_TEMP(reg) \
    ((((int32_t)(BCC_GET_MEAS_RAW(reg))) * 32 - 273150) / 100)

/*!
 * Returns true if value VAL is in the range defined by MIN and MAX values
 * (range includes the border values).
 *
 * @param val Comparison value.
 * @param min Minimal value of the range.
 * @param max Maximal value of the range.
 * @return True if value is the range. False otherwise.
 */
#define BCC_IS_IN_RANGE(val, min, max)   (((val) >= (min)) && ((val) <= (max)))
/*! @} */

/* Enum types definition. */
/*!
 * @addtogroup enum_group
 * @{
 */
/*! @brief Error codes. */
typedef enum
{
    BCC_STATUS_SUCCESS        = 0U,   /*!< No error. */
    BCC_STATUS_SPI_INIT       = 1U,   /*!< SPI initialization failure. */
    BCC_STATUS_SPI_BUSY       = 2U,   /*!< SPI instance is busy. */
    BCC_STATUS_PARAM_RANGE    = 4U,   /*!< Parameter out of range. */
    BCC_STATUS_CRC            = 5U,   /*!< Wrong CRC. */
    BCC_STATUS_COM_TAG_ID     = 6U,   /*!< Response Tag ID does not match with provided ID. */
    BCC_STATUS_COM_RC         = 7U,   /*!< Response Rolling Counter (RC) value does not match
                                           with expected RC. */
    BCC_STATUS_COM_TIMEOUT    = 8U,   /*!< Communication timeout. */
    BCC_STATUS_DIAG_FAIL      = 9U,   /*!< It is not allowed to enter diagnostic mode. */
    BCC_STATUS_EEPROM_ERROR   = 10U,  /*!< An error occurred during the communication to EEPROM. */
    BCC_STATUS_EEPROM_PRESENT = 11U,  /*!< No EEPROM detected. */
    BCC_STATUS_NULL_RESP      = 12U   /*!< Response frame of BCC device is equal to zero
                                           (except CRC). This occurs only in SPI communication
                                           mode during the very first message. */
} bcc_status_t;

/*! @brief Cluster Identification Address.
 *
 * Note that SPI communication mode uses one cluster/device only.
 * The maximum number of clusters/devices for TPL mode is 15.  */
typedef enum
{
    BCC_CID_UNASSIG       = 0U,    /*!< ID of uninitialized BCC device. */
    BCC_CID_DEV1          = 1U,    /*!< Cluster ID of device 1. In TPL mode,
                                       this is the first device in daisy
                                       chain (connected directly to MC33664). */
    BCC_CID_DEV2          = 2U,    /*!< Cluster ID of device 2. */
    BCC_CID_DEV3          = 3U,    /*!< Cluster ID of device 3. */
    BCC_CID_DEV4          = 4U,    /*!< Cluster ID of device 4. */
    BCC_CID_DEV5          = 5U,    /*!< Cluster ID of device 5. */
    BCC_CID_DEV6          = 6U,    /*!< Cluster ID of device 6. */
    BCC_CID_DEV7          = 7U,    /*!< Cluster ID of device 7. */
    BCC_CID_DEV8          = 8U,    /*!< Cluster ID of device 8. */
    BCC_CID_DEV9          = 9U,    /*!< Cluster ID of device 9. */
    BCC_CID_DEV10         = 10U,   /*!< Cluster ID of device 10. */
    BCC_CID_DEV11         = 11U,   /*!< Cluster ID of device 11. */
    BCC_CID_DEV12         = 12U,   /*!< Cluster ID of device 12. */
    BCC_CID_DEV13         = 13U,   /*!< Cluster ID of device 13. */
    BCC_CID_DEV14         = 14U,   /*!< Cluster ID of device 14. */
    BCC_CID_DEV15         = 15U    /*!< Cluster ID of device 15. */
} bcc_cid_t;

/*! @brief BCC communication mode.  */
typedef enum
{
    BCC_MODE_SPI          = 0U,    /*!< SPI communication mode. */
    BCC_MODE_TPL          = 1U     /*!< TPL communication mode. */
} bcc_mode_t;

/*! @brief BCC device.  */
typedef enum
{
    BCC_DEVICE_MC33771    = 0U,    /*!< MC33771B. */
    BCC_DEVICE_MC33772    = 1U     /*!< MC33772B. */
} bcc_device_t;

/*! @brief Measurements provided by Battery Cell Controller.
 *
 * Note that MC33772 doesn't have MEAS_CELL7, ..., MEAS_CELL14 registers.
 * Function BCC_Meas_GetRawValues returns 0x0000 at these positions.
 */
typedef enum
{
    BCC_MSR_CC_NB_SAMPLES = 0U,   /*!< Number of samples in Coulomb counter (register CC_NB_SAMPLES). */
    BCC_MSR_COULOMB_CNT1  = 1U,   /*!< Coulomb counting accumulator (register COULOMB__CNT1). */
    BCC_MSR_COULOMB_CNT2  = 2U,   /*!< Coulomb counting accumulator (register COULOMB__CNT2). */
    BCC_MSR_ISENSE1       = 3U,   /*!< ISENSE measurement (register MEAS_ISENSE1). */
    BCC_MSR_ISENSE2       = 4U,   /*!< ISENSE measurement (register MEAS_ISENSE2). */
    BCC_MSR_STACK_VOLT    = 5U,   /*!< Stack voltage measurement (register MEAS_STACK). */
    BCC_MSR_CELL_VOLT14   = 6U,   /*!< Cell 14 voltage measurement (register MEAS_CELL14). */
    BCC_MSR_CELL_VOLT13   = 7U,   /*!< Cell 13 voltage measurement (register MEAS_CELL13). */
    BCC_MSR_CELL_VOLT12   = 8U,   /*!< Cell 12 voltage measurement (register MEAS_CELL12). */
    BCC_MSR_CELL_VOLT11   = 9U,   /*!< Cell 11 voltage measurement (register MEAS_CELL11). */
    BCC_MSR_CELL_VOLT10   = 10U,  /*!< Cell 10 voltage measurement (register MEAS_CELL10). */
    BCC_MSR_CELL_VOLT9    = 11U,  /*!< Cell 9 voltage measurement (register MEAS_CELL9). */
    BCC_MSR_CELL_VOLT8    = 12U,  /*!< Cell 8 voltage measurement (register MEAS_CELL8). */
    BCC_MSR_CELL_VOLT7    = 13U,  /*!< Cell 7 voltage measurement (register MEAS_CELL7). */
    BCC_MSR_CELL_VOLT6    = 14U,  /*!< Cell 6 voltage measurement (register MEAS_CELL6). */
    BCC_MSR_CELL_VOLT5    = 15U,  /*!< Cell 5 voltage measurement (register MEAS_CELL5). */
    BCC_MSR_CELL_VOLT4    = 16U,  /*!< Cell 4 voltage measurement (register MEAS_CELL4). */
    BCC_MSR_CELL_VOLT3    = 17U,  /*!< Cell 3 voltage measurement (register MEAS_CELL3). */
    BCC_MSR_CELL_VOLT2    = 18U,  /*!< Cell 2 voltage measurement (register MEAS_CELL2). */
    BCC_MSR_CELL_VOLT1    = 19U,  /*!< Cell 1 voltage measurement (register MEAS_CELL1). */
    BCC_MSR_AN6           = 20U,  /*!< Analog input 6 voltage measurement (register MEAS_AN6). */
    BCC_MSR_AN5           = 21U,  /*!< Analog input 5 voltage measurement (register MEAS_AN5). */
    BCC_MSR_AN4           = 22U,  /*!< Analog input 4 voltage measurement (register MEAS_AN4). */
    BCC_MSR_AN3           = 23U,  /*!< Analog input 3 voltage measurement (register MEAS_AN3). */
    BCC_MSR_AN2           = 24U,  /*!< Analog input 2 voltage measurement (register MEAS_AN2). */
    BCC_MSR_AN1           = 25U,  /*!< Analog input 1 voltage measurement (register MEAS_AN1). */
    BCC_MSR_AN0           = 26U,  /*!< Analog input 0 voltage measurement (register MEAS_AN0). */
    BCC_MSR_ICTEMP        = 27U,  /*!< IC temperature measurement (register MEAS_IC_TEMP). */
    BCC_MSR_VBGADC1A      = 28U,  /*!< ADCIA Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1A). */
    BCC_MSR_VBGADC1B      = 29U   /*!< ADCIB Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1B). */
} bcc_measurements_t;

/*! @brief Status provided by Battery Cell Controller. */
typedef enum
{
    BCC_FS_CELL_OV        = 0U,   /*!< CT overvoltage fault (register CELL_OV_FLT). */
    BCC_FS_CELL_UV        = 1U,   /*!< CT undervoltage fault (register CELL_UV_FLT). */
    BCC_FS_CB_OPEN        = 2U,   /*!< Open CB fault (register CB_OPEN_FLT). */
    BCC_FS_CB_SHORT       = 3U,   /*!< Short CB fault (register CB_SHORT_FLT). */
    BCC_FS_GPIO_STATUS    = 4U,   /*!< GPIO status (register GPIO_STS). */
    BCC_FS_AN_OT_UT       = 5U,   /*!< AN undertemperature and overtemperature (register AN_OT_UT_FLT). */
    BCC_FS_GPIO_SHORT     = 6U,   /*!< GPIO short and analog inputs open load detection
                                       (register GPIO_SHORT_Anx_OPEN_STS). */
    BCC_FS_COMM           = 7U,   /*!< Number of communication errors detected (register COM_STATUS). */
    BCC_FS_FAULT1         = 8U,   /*!< Fault status (register FAULT1_STATUS). */
    BCC_FS_FAULT2         = 9U,   /*!< Fault status (register FAULT2_STATUS). */
    BCC_FS_FAULT3         = 10U   /*!< Fault status (register FAULT3_STATUS). */
} bcc_fault_status_t;

/*! @brief Fault pin behavior. */
typedef enum
{
    BCC_FAULT_FIXEDLEVEL  = 0U,   /*!< Fault pin has fixed level behavior. Either
                                       high (fault is present) or low (no fault). */
    BCC_FAULT_HEARTBEAT   = 1U    /*!< Fault pin is used to propagate heart-beat
                                       wave when no fault is present. */
} bcc_fault_beh_t;

/*! @} */

/* Configure struct types definition. */
/*!
 * @addtogroup struct_group
 * @{
 */
/*!
 * @brief Driver internal data.
 *
 * Note that it is initialized in BCC_Init function by the driver
 * and the user mustn't change it at any time.
 */
typedef struct
{
    uint16_t cellMap[BCC_DEVICE_CNT_MAX]; /*!< Bit map of used cells of each BCC device. */
    uint8_t rcTbl[BCC_DEVICE_CNT_MAX];    /*!< Rolling counter index (0-4). */
    uint8_t tagId[BCC_DEVICE_CNT_MAX];    /*!< TAG IDs of BCC devices. */
    uint8_t rxBuf[BCC_RX_BUF_SIZE_TPL];   /*!< Buffer for receiving data in TPL mode. */
} bcc_drv_data_t;

/*!
 * @brief Driver configuration.
 *
 * This structure contains all information needed for proper functionality of
 * the driver, such as used communication mode, BCC device(s) configuration or
 * internal driver data.
 */
typedef struct {
    uint8_t drvInstance;                     /*!< BCC driver instance. Passed to the external functions
                                                  defined by the user. */

    bcc_mode_t commMode;                     /*!< BCC communication mode. */
    uint8_t devicesCnt;                      /*!< Number of BCC devices. SPI mode allows one device only,
                                                  TPL mode allows up to 15 devices. */
    bcc_device_t device[BCC_DEVICE_CNT_MAX]; /*!< BCC device type of
                                                  [0] BCC with CID 1, [1] BCC with CID 2, etc. */
    uint16_t cellCnt[BCC_DEVICE_CNT_MAX];    /*!< Number of connected cells to each BCC.
                                                  [0] BCC with CID 1, [1] BCC with CID 2, etc. */

    bcc_drv_data_t drvData;                  /*!< Internal driver data. */
} bcc_drv_config_t;
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief This function initializes the Battery Cell Controller device(s),
 * configures its registers, assigns CID and initializes internal driver data.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param devConf Initialization values of BCC device registers specified
 *                by BCC_INIT_CONF_REG_ADDR. If NULL, registers are not
 *                initialized.
 *                devConf[0][x] belongs to device with CID 1,
 *                devConf[1][x] belongs to device with CID 2, etc.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Init(bcc_drv_config_t* const drvConfig,
    const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]);

/*!
 * @brief This function uses No Operation command of BCC to verify communication
 * with device specified by CID without performing any operation.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_VerifyCom(bcc_drv_config_t* const drvConfig, bcc_cid_t cid);

/*!
 * @brief This function sets sleep mode to all Battery Cell Controller devices.
 *
 * In case of TPL mode MC33664TL goes to sleep mode automatically.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Sleep(bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function sets normal mode to all Battery Cell Controller devices.
 *
 * In case of TPL mode, MC33664 goes to normal mode automatically.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return bcc_status_t Error code.
 */
void BCC_WakeUp(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function resets BCC device using software reset. It enters reset
 * via SPI or TPL interface.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_SoftwareReset(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid);

/*!
 * @brief This function resets BCC device using GPIO pin.
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
void BCC_HardwareReset(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function enables MC33664 device (sets a normal mode).
 * Intended for TPL mode only!
 *
 * During the driver initialization (BCC_Init function), normal mode of MC33664
 * is set automatically. This function can be used e.g. when sleep mode of both
 * BCC device(s) and MC33664 is required. The typical function flow can be then:
 * BCC_Sleep -> BCC_TPL_Disable -> ... -> BCC_TPL_Enable -> BCC_WakeUp.
 *
 * @param drvConfig Pointer to driver instance configuration.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_TPL_Enable(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function sets MC33664 device into sleep mode.
 * Intended for TPL mode only!
 *
 * This function can be (optionally) used after BCC_Sleep function. Function
 * BCC_TPL_Enable must be then called before BCC_WakeUp function!
 *
 * @param drvConfig Pointer to driver instance configuration.
 */
void BCC_TPL_Disable(const bcc_drv_config_t* const drvConfig);

/*!
 * @brief This function reads a value from addressed register (or desired
 * number of registers) of selected Battery Cell Controller device.
 *
 * In case of simultaneous read of more registers, address is incremented
 * in ascending manner.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regCnt Number of registers to read.
 * @param regVal Pointer to memory where content of selected 16 bit registers
 *               is stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint8_t regCnt, uint16_t* regVal);

/*!
 * @brief This function writes a value to addressed register of selected Battery
 * Cell Controller device.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 * @param retReg Automatic response of BCC, which contains updated register
 *               (TPL mode) or a register addressed in previous access
 *               (SPI mode). You can pass NULL when you do not care about the
 *               response.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_Write(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regVal, uint16_t* retReg);

/*!
 * @brief This function writes a value to addressed register of all configured
 * BCC devices. Intended for TPL mode only!
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regVal New value of selected register.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_WriteGlobal(bcc_drv_config_t* const drvConfig,
    uint8_t regAddr, uint16_t regVal);

/*!
 * @brief This function updates content of a selected register. It affects bits
 * specified by a bit mask only.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param regAddr Register address. See BCC header file with register map for
 *                possible values.
 * @param regMask Bit mask. Bits set to 1 will be updated.
 * @param regVal  New value of register bits defined by bit mask.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Reg_Update(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t regAddr, uint16_t regMask, uint16_t regVal);

/*!
 * @brief This function starts ADC conversion. It sets Start of Conversion bit
 * and new value of TAG ID in ADC_CFG register.
 *
 * TAG ID is incremented for each conversion. You can use function
 * BCC_Meas_IsConverting to check conversion status.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Meas_StartConversion(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid);

/*!
  * @brief This function starts ADC conversion for all devices in TPL chain. It
  * uses a Global Write command to set ADC_CFG register. Intended for TPL mode
  * only!
  *
  * As a TAG ID, incremented TAG ID of the first device is used. You can use
  * function BCC_Meas_IsConverting to check conversion status.
  *
  * @param drvConfig Pointer to driver instance configuration.
  * @param adcCfgValue Value of ADC_CFG register to be written to all devices in
  *                    the chain. Note that TAG_ID and SOC bits are
  *                    automatically added by this function.
  *
  * @return bcc_status_t Error code.
  */
bcc_status_t BCC_Meas_StartConversionGlobal(bcc_drv_config_t* const drvConfig,
    uint16_t adcCfgValue);

/*!
 * @brief This function checks status of conversion defined by End of Conversion
 * bit in ADC_CFG register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param completed Pointer to check result. True if a conversion is complete.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Meas_IsConverting(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bool *completed);

/*!
 * @brief This function reads the measurement registers and returns raw values.
 * Macros defined in BCC header file can be used to perform correct unit
 * conversion.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param measurements Array containing all values measured by BCC. Indexes into
 *                     the array are defined in enumeration bcc_measurements_t
 *                     placed in BCC header file. For required size of the array
 *                     see BCC_MEAS_CNT constant defined in BCC header file).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Meas_GetRawValues(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t measurements[]);

/*!
 * @brief This function reads the status registers and returns raw values.
 * You can use constants defined in bcc_mc3377x.h file.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param status Array containing all fault status information provided by BCC.
 *               Indexes into the array are defined in bcc_fault_status_t
 *               enumeration placed in BCC header file. Required size of the
 *               array is 11. You can use macro BCC_STAT_CNT defined in BCC
 *               header file, which contains appropriate value.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Fault_GetStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint16_t status[]);

/*!
 * @brief This function clears selected fault status register.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param statSel Selection of a fault status register to be cleared. See
 *                definition of this enumeration in BCC header file.
 *                COM_STATUS register is read only and cannot be cleared.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_Fault_ClearStatus(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, bcc_fault_status_t statSel);

/*!
 * @brief This function sets output value of one BCC GPIO pin. This function
 * should be used only when at least one GPIO is in output mode. Resets BCC
 * device using GPIO pin.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param gpioSel Index of GPIO output to be set. Index starts at 0 (GPIO 0).
 * @param val Output value. Possible values are FALSE (logical 0, low level)
 *            and TRUE (logical 1, high level).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_GPIO_SetOutput(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t gpioSel, bool val);

/*!
 * @brief This function enables or disables the cell balancing via
 * SYS_CFG1[CB_DRVEN] bit.
 *
 * Note that each cell balancing driver needs to be setup separately, e.g. by
 * BCC_CB_SetIndividual function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param enable Drivers state. False (all drivers are disabled) or true
 *               (drivers are enabled).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_CB_Enable(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool enable);

/*!
 * @brief This function enables or disables cell balancing for a specified cell
 * and sets its timer.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param cellIndex Index of the cell. Note the cells are indexed from 0.
 * @param enable True for enabling of CB, False otherwise.
 * @param timer Timer for enabled CB driver in minutes. Note that a zero
 *              value represents 30 seconds.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_CB_SetIndividual(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t cellIndex, bool enable, uint16_t timer);

/*!
 * @brief This function can be used to manual pause cell balancing before on
 * demand conversion. As a result more precise measurement can be done. Note
 * that it is user obligation to re-enable cell balancing after measurement
 * ends.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param pause True (pause) / false (unpause).
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_CB_Pause(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    bool pause);

/*!
 * @brief This function reads a fuse mirror register of a BCC device specified
 * by CID.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param fuseAddr Address of a fuse mirror register to be read.
 * @param value Pointer to memory where the read value will be stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_FuseMirror_Read(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t fuseAddr, uint16_t* const value);

/*!
 * @brief This function writes a fuse mirror register of a BCC device specified
 * by CID.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param fuseAddr Address of a fuse mirror register to be written.
 * @param value Value to be written.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_FuseMirror_Write(bcc_drv_config_t* const drvConfig,
    bcc_cid_t cid, uint8_t fuseAddr, uint16_t value);

/*!
 * @brief This function reads an unique serial number of the BCC device from the
 * content of mirror registers.
 *
 * GUID is created according to the following table:
 * |    Device    | GUID [36:21] | GUID [20:5] | GUID [4:0] |
 * |:------------:|:------------:|:-----------:|:----------:|
 * |   MC33771B   | 0x18 [15:0]  | 0x19 [15:0] | 0xA1 [4:0] |
 * | fuse address |  (16 bit)    |  (16 bit)   |  (5 bit)   |
 * |:------------:|:------------:|:-----------:|:----------:|
 * |   MC33772B   | 0x10 [15:0]  | 0x11 [15:0] | 0x12 [4:0] |
 * | fuse address |  (16 bit)    |  (16 bit)   |  (5 bit)   |
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address.
 * @param guid Pointer to memory where 37b unique ID will be stored.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_GUID_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint64_t* const guid);

/*!
 * @brief This function reads a byte from specified address of EEPROM memory
 * connected to BCC device via I2C bus.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address of BCC device the EEPROM memory is
 *                    connected to.
 * @param addr Address of EEPROM data will be read from. The admission range is
 *             from 0 to 127.
 * @param data Data read from specified address of EEPROM memory.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_EEPROM_Read(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t addr, uint8_t* const data);

/*!
 * @brief This function writes a byte to specified address of EEPROM memory
 * connected to BCC device via I2C bus.
 *
 * Note that the EEPROM write time (depends on device selection) is usually
 * around 5 ms. Therefore, another EEPROM (write & read) operations to the same
 * EEPROM memory cannot be done 5 ms after end of BCC_EEPROM_Write function.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid Cluster Identification Address of BCC device the EEPROM memory is
 *                    connected to.
 * @param addr Address of EEPROM data will be written to. The admission range is
 *             from 0 to 127.
 * @param data Data written to specified address of EEPROM memory.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_EEPROM_Write(bcc_drv_config_t* const drvConfig, bcc_cid_t cid,
    uint8_t addr, uint8_t data);

/*******************************************************************************
 * Platform specific functions
 ******************************************************************************/

/*!
 * @brief Waits for specified amount of milliseconds. This function needs to be
 * implemented for specified MCU by the user.
 *
 * @param delay - Number of milliseconds to wait.
 */
extern void BCC_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds. This function needs to be
 * implemented for specified MCU by the user.
 *
 * @param delay - Number of microseconds to wait.
 */
extern void BCC_MCU_WaitUs(uint32_t delay);

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
extern void BCC_MCU_Assert(bool x);

/*!
 * @brief This function performs one 40b transfer via SPI bus. Intended for SPI
 * mode only. This function needs to be implemented for specified MCU by the
 * user.
 *
 * The byte order of buffers is given by BCC_MSG_BIGEND macro (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param transBuf Pointer to 40b data buffer to be sent.
 * @param recvBuf Pointer to 40b data buffer for received data.
 *
 * @return bcc_status_t Error code.
 */
extern bcc_status_t BCC_MCU_TransferSpi(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[]);

/*!
 * @brief This function sends and receives data via TX and RX SPI buses.
 * Intended for TPL mode only. This function needs to be implemented for
 * specified MCU by the user.
 *
 * TX SPI bus always performs only one 40b SPI transfer. Expected number of RX
 * transfers is passed as a parameter. The byte order of buffers is given by
 * BCC_MSG_BIGEND macro (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param transBuf Pointer to 40b data buffer to be sent.
 * @param recvBuf Pointer to buffer for received data. Its size must be at least
 *                (5 * recvTrCnt) bytes.
 * @param recvTrCnt Number of 40b transfers to be received.
 *
 * @return bcc_status_t Error code.
 */
extern bcc_status_t BCC_MCU_TransferTpl(uint8_t drvInstance, uint8_t transBuf[],
    uint8_t recvBuf[], uint16_t recvTrCnt);

/*!
 * @brief Writes logic 0 or 1 to the CSB pin (or CSB_TX in case of TPL mode).
 * This function needs to be implemented by the user.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to CSB (CSB_TX) pin.
 */
extern void BCC_MCU_WriteCsbPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Writes logic 0 or 1 to the RST pin. This function needs to be
 * implemented by the user. If no RST pin is used, keep the function body empty.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to RST pin.
 */
extern void BCC_MCU_WriteRstPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Writes logic 0 or 1 to the EN pin of MC33664. This function is
 * called only in the TPL and it needs to be implemented by the user.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value - Zero or one to be set to EN pin.
 */
extern void BCC_MCU_WriteEnPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Reads logic value of INTB pin of MC33664. This function is
 * called in the TPL mode only and it needs to be implemented by the user.
 *
 * @param drvInstance Instance of BCC driver.
 *
 * @return Zero value for logic zero, non-zero value otherwise.
 */
extern uint32_t BCC_MCU_ReadIntbPin(uint8_t drvInstance);

/*! @} */

#endif /* __BCC_H__ */
/*******************************************************************************
 * EOF;
 ******************************************************************************/
