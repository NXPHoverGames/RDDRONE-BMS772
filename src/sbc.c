/****************************************************************************
 * nxp_bms/BMS_v1/src/sbc.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>
#include <sys/ioctl.h>
#include "sbc.h"
#include "cli.h"

#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>

#include "data.h"
#include "spi.h"
#include "gpio.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

//#define DISABLE_WATCHDOG
//#define DEBUG_WATCHDOG_TIME

#define READ_BIT                        1
#define WRITE_BIT                       0

#define UJA1169TK_F_3_ID                0xE9

#define WATCHDOG_CTRL_REG_ADR           0x00
#define MODE_CTRL_REG_ADR               0x01
#define MAIN_STATUS_REG_ADR             0x03
#define WATCHDOG_STATUS_REG_ADR         0x05
#define REGULATOR_CTRL_REG_ADR          0x10 
#define CAN_CTRL_REG_ADR                0x20
#define WAKE_PIN_STATUS_REG_ADR         0x4B
#define WAKE_PIN_ENABLE_REG_ADR         0x4C
#define GLOBAL_EVENT_STAT_REG_ADR       0x60
#define SYS_EVENT_STAT_REG_ADR          0x61
#define SUPPLY_EVENT_STAT_REG_ADR       0x62
#define TRANSC_EVENT_STAT_REG_ADR       0x63
#define WAKE_PIN_EVENT_STAT_REG_ADR     0x64
#define MTPNV_STATUS_REG_ARD            0x70
#define START_UP_CTRL_REG_ADR           0x73
#define SBC_CONF_CTRL_REG_ADR           0x74
#define MTPNV_CRC_CTRL_REG_ADR          0x75
#define IDENTIFICATION_REG_ADR          0x7E

#define WATCHDOG_CTRL_NWP_BIT           0
#define WATCHDOG_CTRL_WMC_BIT           5
#define WATCHDOG_STATUS_WDS_BIT         0
#define WATCHDOG_STATUS_SDMS_BIT        2
#define WATCHDOG_STATUS_FNMS_BIT        3
#define REGULATOR_CTRL_PDC_BIT          6
#define REGULATOR_CTRL_V2C_BIT          2
#define CAN_CTRL_CFDC_BIT               6
#define CAN_CTRL_PNCOK_BIT              5
#define CAN_CTRL_CPNC_BIT               4
#define CAN_CTRL_CMC_BIT                0
#define WAKE_PIN_ENABLE_WPFE_BIT        0
#define WAKE_PIN_ENABLE_WPRE_BIT        1
#define GLOBAL_EVENT_STAT_WPE_BIT       3
#define GLOBAL_EVENT_STAT_TRXE_BIT      2
#define GLOBAL_EVENT_STAT_SUPE_BIT      1
#define GLOBAL_EVENT_STAT_SYSE_BIT      0
#define SYS_EVENT_STAT_PO_BIT           4
#define SYS_EVENT_STAT_OTW_BIT          2
#define SYS_EVENT_STAT_SPIF_BIT         1
#define SYS_EVENT_STAT_WDF_BIT          0
#define SUPPLY_EVENT_STAT_V20_BIT       2
#define SUPPLY_EVENT_STAT_V2U_BIT       1
#define SUPPLY_EVENT_STAT_V1U_BIT       0
#define TRANSC_EVENT_STAT_PNFDE_BIT     5
#define TRANSC_EVENT_STAT_CBS_BIT       4
#define TRANSC_EVENT_STAT_CF_BIT        1
#define TRANSC_EVENT_STAT_CW_BIT        0
#define WAKE_PIN_EVENT_STAT_WPR_BIT     1
#define WAKE_PIN_EVENT_STAT_WPF_BIT     0
#define MTPNV_STATUS_NVMPS_BIT          0
#define START_UP_CTRL_V2SUC_BIT         3
#define START_UP_CTRL_RLC_BIT           4
#define SBC_CONF_CTRL_SLPC_BIT          0
#define SBC_CONF_CTRL_SDMC_BIT          2
#define SBC_CONF_CTRL_FNMC_BIT          3

// watchdog register can only be written in standby mode!
#define WATCHDOG_CTRL_WMC_AUTONOM       0x1     
#define WATCHDOG_CTRL_WMC_TIMEOUT       0x2
#define WATCHDOG_CTRL_WMC_WINDOW        0x4
#define WATCHDOG_CTRL_WMC_MASK          0x7
#define WATCHDOG_CTRL_NWP_8             0x8
#define WATCHDOG_CTRL_NWP_16            0x1
#define WATCHDOG_CTRL_NWP_32            0x2
#define WATCHDOG_CTRL_NWP_64            0xB
#define WATCHDOG_CTRL_NWP_128           0x4
#define WATCHDOG_CTRL_NWP_256           0xD
#define WATCHDOG_CTRL_NWP_1024          0xE
#define WATCHDOG_CTRL_NWP_4096          0x7
#define WATCHDOG_CTRL_NWP_MASK          0xF

#define MODE_CONTROL_SLEEP_MODE         0x1
#define MODE_CONTROL_STANDBY_MODE       0x4
#define MODE_CONTROL_NORMAL_MODE        0x7

#define WATCHDOG_STATUS_WDS_MASK        0x3
#define WATCHDOG_STATUS_WDS_WD_OFF      0
#define WATCHDOG_STATUS_SDMS_MASK       0x1
#define WATCHDOG_STATUS_FNMS_MASK       0x1

#define REGULATOR_CTRL_PDC_DEFAULT      0
#define REGULATOR_CTRL_V2C_OFF          0
#define REGULATOR_CTRL_V2C_NORMAL       1
#define REGULATOR_CTRL_V2C_ALL_NRST     2
#define REGULATOR_CTRL_V2C_ALL_RST      3
#define REGULATOR_CTRL_V2C_MASK         0x0C

#define CAN_CTRL_CFDC_CANFD_DIS         0
#define CAN_CTRL_CFDC_CANFD_EN          1
#define CAN_CTRL_PNCOK_INV              0
#define CAN_CTRL_PNCOK_SUC              1
#define CAN_CTRL_CPNC_DIS               0
#define CAN_CTRL_CPNC_EN                1
#define CAN_CTRL_CMC_OFF                0
#define CAN_CTRL_CMC_ACT_UV_EN          1
#define CAN_CTRL_CMC_ACT_UV_DIS         2
#define CAN_CTRL_CMC_LIS                3

// set CAN FD OFF, disable CAN selective wake-up, Active mode (when the SBC is in Normal mode); 
// CAN supply undervoltage detection disabled
#define CAN_CTRL_REG_VAL                (CAN_CTRL_CFDC_CANFD_DIS << CAN_CTRL_CFDC_BIT) + \
                                        (CAN_CTRL_PNCOK_INV << CAN_CTRL_PNCOK_BIT) + \
                                        (CAN_CTRL_CPNC_DIS << CAN_CTRL_CPNC_BIT) + \
                                        (CAN_CTRL_CMC_ACT_UV_DIS << CAN_CTRL_CMC_BIT)

// enable the wake with high to low on the wake pin
#define WAKE_PIN_ENABLE_REG_VAL         1 << WAKE_PIN_ENABLE_WPFE_BIT

#define START_UP_CTRL_V2SUC_OFF         0
#define START_UP_CTRL_V2SUC_ON          1
#define START_UP_CTRL_RLC_20MS          0
#define START_UP_CTRL_RLC_10MS          1
#define START_UP_CTRL_RLC_3_6MS         2
#define START_UP_CTRL_RLC_1MS           3

// set the reset pulse width to min 20ms and turn off V2
#define START_UP_CTRL_REG_VAL           (START_UP_CTRL_RLC_20MS << START_UP_CTRL_RLC_BIT) + \
                                        (START_UP_CTRL_V2SUC_OFF << START_UP_CTRL_V2SUC_BIT)

#define SBC_CONF_CTRL_FNMC_OFF          0
#define SBC_CONF_CTRL_FNMC_ON           1
#define SBC_CONF_CTRL_SDMC_OFF          0
#define SBC_CONF_CTRL_SDMC_ON           1
#define SBC_CONF_CTRL_SLPC_ACC          0
#define SBC_CONF_CTRL_SLPC_IGN          1       

// set the Forced normal mode off, to go in the other modes
// set the Software Development mode on to disable the watchdog
// set the sleep command accept on
#define SBC_CONF_CTRL_REG_VAL           (SBC_CONF_CTRL_FNMC_OFF << SBC_CONF_CTRL_FNMC_BIT) + \
                                        (SBC_CONF_CTRL_SDMC_ON << SBC_CONF_CTRL_SDMC_BIT) + \
                                        (SBC_CONF_CTRL_SLPC_ACC << SBC_CONF_CTRL_SLPC_BIT)

#define WATCHDOG_CTRL_WRITE_MASK        0xEF
#define MODE_CONTROL_WRITE_MASK         0x7
#define GLOBAL_EVENT_STAT_MASK          0xF
#define SYS_EVENT_STAT_MASK             0x17
#define SUPPLY_EVENT_STAT_MASK          0x7
#define TRANSC_EVENT_STAT_MASK          0x33
#define WAKE_PIN_EVENT_STAT_MASK        0x3
#define WAKE_PIN_EN_WRITE_MASK          0x3
#define SBC_CONF_CTRL_REG_WRITE_MASK    0x3D 
#define START_UP_CTRL_REG_WRITE_MASK    0x38
#define CAN_CTRL_WRITE_MASK             0x73

//! @brief use this define to specify the fast watchdog period.
#define WATCHDOG_PERIOD_FAST            WATCHDOG_CTRL_NWP_4096 //WATCHDOG_CTRL_NWP_1024 //

//! @brief use this define to specify the slow watchdog period.
#define WATCHDOG_PERIOD_SLOW            WATCHDOG_CTRL_NWP_4096

#if WATCHDOG_PERIOD_FAST == WATCHDOG_PERIOD_SLOW
#   warning WATCHDOG_PERIOD_FAST is not faster then the WATCHDOG_PERIOD_SLOW
#endif

#warning TODO check traces or add internal (MCU) watchdog of 2s.

#define NRST_CHECK_BYTES                4

#ifndef CONFIG_NRST_CHECK_PROC_FS
  #error enable NRST_CHECK_PROC_FS in menuconfig (Board Selection -> enable nrst check as a proc fs)
#endif

// output a warning if debug assertions are enabled
#ifdef CONFIG_DEBUG_ASSERTIONS
#   ifdef DISABLE_WATCHDOG
        #warning Watchdog is disabled!! 
#   endif
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gSbcInitialized = false;

/*! @brief  mutex for controlling the watchdog */
static pthread_mutex_t gWatchdogLock;   

/*! @brief variable to indicate of it is initialized */
static bool gWatchdogLockInitialized = false;

/****************************************************************************
 * private Functions
 ****************************************************************************/
/*! 
 * @brief   this function is used to program the START_UP_CTRL_REG, SBC_CONF_CTRL_REG, MTPNV_CRC_CTRL
 *          Make sure it can be programmed
 */
int programNVMPSRegisters(void);

/*! 
 * @brief   this function is used to check if the NRST pin is connected to the SBC
 *
 * @param   none
 *
 * @return  1 if OK, 0 if not OK and negative if error 
 */
int checkNrstLine(void);

/****************************************************************************
 * public functions
 ****************************************************************************/
/*!
 * @brief   this function is used to initialze the SBC 
 *          
 * @param   skipSelfTest if this is true it will skip the self-test
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_initialize(bool skipSelfTest)
{
    int lvRetValue;
    uint8_t txData[2];
    uint8_t rxData[2], rxData2[2];

    // check if initialized 
    if(!gSbcInitialized)
    {

#ifdef DISABLE_WATCHDOG
        cli_printfWarning("WARNING: watchdog is disabled!!\n");
#endif
#ifdef DEBUG_WATCHDOG_TIME
        cli_printfWarning("WARNING: watchdog time ouput is enabled!\n");
#endif
#if WATCHDOG_PERIOD_FAST == WATCHDOG_CTRL_NWP_1024
        cli_printfWarning("WARNING: watchdog is at 1s, increase to 4s at release!!\n");
#endif

        // check if the watchdog is not initialized
        if(!gWatchdogLockInitialized)
        {
            // initialze the mutex
            pthread_mutex_init(&gWatchdogLock, NULL);
            gWatchdogLockInitialized = true;
        }

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST SBC: START\n");

            // check the NRST line
            lvRetValue = checkNrstLine();

            // check if it is not OK
            if(lvRetValue != 1)
            {
                // output to the user and return
                cli_printfError("SBC ERROR: Failed to verify the NRST line!\n");
                cli_printf("Check the NRST line, jumper J6 needs to be closed!\n");
                
                // return error
                lvRetValue = -1;
                return lvRetValue;
            }

            // check the SBC 
            if(sbc_verifySbc())
            {
                // output to the user and return
                cli_printfError("SBC ERROR: Failed to verify the SBC!\n");
                
                // return error
                lvRetValue = -1;
                return lvRetValue;
            }
        }

        // check if the NVMPS registors have the right values (SBC_CONF_CTRL_REG_ADR and START_UP_CTRL_REG_ADR)
        // read the SBC_CONF_CTRL_REG 
        txData[0] = (SBC_CONF_CTRL_REG_ADR << 1) + READ_BIT;
        txData[1] = 0;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

#ifdef DEBUG_SBC_SPI
        cli_printf("SBC conf ctrl R: RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to read SBC_CONF_CTRL_REG! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

        // set the start-up control register to 20ms reset pulse width and V2 off
        txData[0] = (START_UP_CTRL_REG_ADR << 1) + READ_BIT;
        txData[1] = START_UP_CTRL_REG_VAL;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData2);

#ifdef DEBUG_SBC_SPI
        cli_printf("START_UP_CTRL R: RX0: %d, RX1: %d\n", rxData2[0], rxData2[1]);
#endif

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to write START_UP_CTRL_REG! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

        // check if the values are not ok
        if(!((((rxData[1] & SBC_CONF_CTRL_REG_WRITE_MASK) ==  SBC_CONF_CTRL_REG_VAL) && 
            ((rxData2[1] & START_UP_CTRL_REG_WRITE_MASK) ==  START_UP_CTRL_REG_VAL))))
        {
            cli_printf("NVMS registers don't have the right value!\n");

            if((rxData[1] & SBC_CONF_CTRL_REG_WRITE_MASK) !=  SBC_CONF_CTRL_REG_VAL)
            {
                cli_printf("SBC_CONF: %d != %d\n", (rxData[1] & SBC_CONF_CTRL_REG_WRITE_MASK), SBC_CONF_CTRL_REG_VAL);
            }
            if((rxData2[1] & START_UP_CTRL_REG_WRITE_MASK) !=  START_UP_CTRL_REG_VAL)
            {
                cli_printf("START_UP: %d != %d \n", (rxData2[1] & START_UP_CTRL_REG_WRITE_MASK), START_UP_CTRL_REG_VAL);
            }

            // check the MTPNV status register
            txData[0] = (MTPNV_STATUS_REG_ARD << 1) + READ_BIT;
            txData[1] = 0;

            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            cli_printf("MTPNV_STATUS: RX0: %d, RX1: %d\n", rxData[0], rxData[1]);

            // check if the NVMPS can not be overwritten
            if(!(rxData[1] & (1 << MTPNV_STATUS_NVMPS_BIT)))
            {
                // output to the user
                cli_printfError("SBC ERROR: NVMPS can't be overwritten!!!\n");

                cli_printf("Please apply the next things continuously for at least 1.1sec \n");
                cli_printf("during battery power-up:\n");
                cli_printf("pin RSTN is held LOW\n");
                cli_printf("CANH is pulled up to VBAT\n");
                cli_printf("CANL is pulled down to GND\n");

                // return error
                lvRetValue = -1;
                return lvRetValue;
            }

            // overwritten it
            cli_printf("overwritting NVMPS registers\n");

            // program the NVMPS registers
            lvRetValue = programNVMPSRegisters();

            // check for error
            if(lvRetValue)
            {
                // return the error
                return lvRetValue;
            }
        }       

        // sleep for (20us) (Tto(SPI))
        usleep(20);
        
        // set the local wakeup from low to high
        txData[0] = (WAKE_PIN_ENABLE_REG_ADR << 1) + WRITE_BIT;
        txData[1] = WAKE_PIN_ENABLE_REG_VAL;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("WAKE_PIN_EN W RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        // check the value
        // check the wake pin status and enable
        txData[0] = (WAKE_PIN_ENABLE_REG_ADR << 1) + READ_BIT;
        txData[1] = 0;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("WAKE_PIN_EN R RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        // check if the write went not ok
        if(WAKE_PIN_ENABLE_REG_VAL != (rxData[1] & WAKE_PIN_EN_WRITE_MASK))
        {
            // print to the user
            cli_printfError("SBC ERROR: failed to verify wake status! %d != %d\n", WAKE_PIN_ENABLE_REG_VAL, (rxData[1] & WAKE_PIN_EN_WRITE_MASK));

            // return error
            lvRetValue = -1;
            return lvRetValue;
        }

        // get the CAN FD mode 
        if(data_getParameter(UAVCAN_FD_MODE, &rxData2[0], NULL) == NULL)
        {
           cli_printfError("SBC ERROR: couldn't get CANFD mode\n");
           rxData2[0] = UAVCAN_FD_MODE_DEFAULT;
        } 
        // write the CAN control register
        txData[0] = (CAN_CTRL_REG_ADR << 1) + WRITE_BIT;
        txData[1] = ((CAN_CTRL_REG_VAL) | (rxData2[0] << CAN_CTRL_CFDC_BIT));

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("CAN_CTRL W RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        txData[0] = (CAN_CTRL_REG_ADR << 1) + READ_BIT;
        txData[1] = 0;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("CAN_CTRL R RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        // check if the write went not ok
        if(((CAN_CTRL_REG_VAL) | (rxData2[0] << CAN_CTRL_CFDC_BIT)) != 
           (rxData[1] & CAN_CTRL_WRITE_MASK))
        {
            // print to the user
            cli_printfError("SBC ERROR: failed to verify CAN control! %d != %d\n", 
                ((CAN_CTRL_REG_VAL) | (rxData2[0] << CAN_CTRL_CFDC_BIT)), 
                (rxData[1] & CAN_CTRL_WRITE_MASK));

            // return error
            lvRetValue = -1;
            return lvRetValue;
        }

        // set V2 on in normal mode 
        // read the Regulator control register
        txData[0] = (REGULATOR_CTRL_REG_ADR << 1) + READ_BIT;
        txData[1] = 0;

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to read REGULATOR_CTRL_REG! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("REGULATOR_CTRL_REG R RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

        // check if not ok 
        if((rxData[1] & REGULATOR_CTRL_V2C_MASK) != 
            (REGULATOR_CTRL_V2C_NORMAL << REGULATOR_CTRL_V2C_BIT))
        {
            // write the right value
            // clear the V2C bits
            txData[1] = rxData[1] & ~(REGULATOR_CTRL_V2C_MASK); 

            // make sure V2 is on in normal mode 
            txData[1] |= (REGULATOR_CTRL_V2C_NORMAL << REGULATOR_CTRL_V2C_BIT);
            txData[0] = (REGULATOR_CTRL_REG_ADR << 1) + WRITE_BIT;

            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                // output to the user
                cli_printfError("SBC ERROR: failed to read REGULATOR_CTRL_REG! %d\n", lvRetValue);

                // return the error
                return lvRetValue;
            }

            // read the register again to verify it 
            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                // output to the user
                cli_printfError("SBC ERROR: failed to write REGULATOR_CTRL_REG! %d\n", lvRetValue);

                // return the error
                return lvRetValue;
            }

#ifdef DEBUG_SBC_SPI
            cli_printf("REGULATOR_CTRL_REG R RX0: %d, RX1: %d\n", rxData[0], rxData[1]);
#endif

            // check if not ok 
            if((rxData[1] & REGULATOR_CTRL_V2C_MASK) != 
                (REGULATOR_CTRL_V2C_NORMAL << REGULATOR_CTRL_V2C_BIT))
            {
                // error
                // output to the user
                cli_printfError("SBC ERROR: failed to verify REGULATOR_CTRL_REG! %d\n", lvRetValue);

                // return the error
                lvRetValue = -1;
                return lvRetValue;
            }
        }
        
        // turn off the watchdog until the mode is set in the main loop
        // set the watchdog off
        lvRetValue = sbc_setWatchdogMode(WD_AUTONOMOUS, FAST_WATCHDOG);

        // check for errors
        if(lvRetValue)
        {
            cli_printfError("SBC ERROR: Couldn't set the watchdog to autonomous mode (off) 1\n");
        }

        cli_printf("Setting SBC to normal mode!\n");

        // lock the mutex 
        pthread_mutex_lock(&gWatchdogLock);

        // set the SBC to normal mode, without setting the watchdog on
        // otherwise replace the beneith code with sbc_setSbcMode(SBC_NORMAL)

        // set the normal mode
        txData[1] = MODE_CONTROL_NORMAL_MODE;

        // set the address and set it in write mode 
        txData[0] = (MODE_CTRL_REG_ADR << 1) + WRITE_BIT;

        // write the data to the SBC
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

#ifdef DEBUG_SBC_SPI
        cli_printf("Mode W rx: 0: %d 1: %d\n", rxData[0], rxData[1]);
#endif

        // check for errors
        if(lvRetValue)
        {
            cli_printfError("SBC ERROR: failed to set mode! %d\n", lvRetValue);
        }

        // check the mode 
        // set the address and set it in read mode 
        txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

        // read the data from the SBC
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            cli_printfError("SBC ERROR: failed to get mode! %d\n", lvRetValue);
        }

#ifdef DEBUG_SBC_SPI
        cli_printf("Mode R rx: 0: %d 1: %d\n", rxData[0], rxData[1]);
#endif

        // check for errors
        if((rxData[1] & MODE_CONTROL_WRITE_MASK) != txData[1])
        {
            cli_printfError("SBC ERROR: Couldn't verify SBC mode! %d != %d\n", (rxData[1] & MODE_CONTROL_WRITE_MASK), txData[1]);
        }

        // unlock the mutex 
        pthread_mutex_unlock(&gWatchdogLock);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to set SBC to normal mode! %d\n", lvRetValue);

            // return the error
            return lvRetValue;
        }

        lvRetValue = 0;

        // the SBC is initialized 
        gSbcInitialized = true;

        // Check if the self-test shouldn't be skipped
        if(!skipSelfTest)
        {
            cli_printf("SELF-TEST SBC: \e[32mPASS\e[39m\n");
        }
    }
    else
    {
        lvRetValue = 0;
    }

    // return 
    return lvRetValue;
}

/*!
 * @brief   This function is used to verify the SBC using SPI 
 *          It will check the Device identification register (0x7E)
 *          
 * @param   None
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_verifySbc(void)
{
    int lvRetValue = -1;
    uint8_t txData[2];
    uint8_t rxData[2];

    // set the first byte to the identification register address with a read bit
    txData[0] = (IDENTIFICATION_REG_ADR << 1) + READ_BIT;
    txData[1] = 0;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to read ID! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    // make it be wrong again
    lvRetValue = -1;

    // check if the returned ID is expected
    if(rxData[1] == UJA1169TK_F_3_ID)
    {
        // output to the user 
        //cli_printf("SBC ID verified!\n");

        // set the returnvalue to 0 (OK)
        lvRetValue = 0;
    }

    // return to the user 
    return lvRetValue;
}

/*!
 * @brief   this function is used to set the SBC mode
 * @note    Multi-thread protected 
 *          
 * @param   newMode the new mode from the sbc_mode_t enum
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setSbcMode(sbc_mode_t newMode)
{
    int lvRetValue = -1;
    uint8_t txData[2];
    uint8_t rxData[2];
    uint8_t globalEventStat;

    // lock the mutex 
    pthread_mutex_lock(&gWatchdogLock);

    // calculate the tx data
    switch(newMode)
    {
        // in case of sleep mode 
        case SBC_SLEEP: 

            // unlock the mutex 
            pthread_mutex_unlock(&gWatchdogLock);

            // turn off the watchdog
            if(sbc_setWatchdogMode(WD_AUTONOMOUS, FAST_WATCHDOG))
            {
                cli_printfError("SBC ERROR: Couldn't set the watchdog to autonomous mode (off) 2\n");
            }

            // lock the mutex 
            pthread_mutex_lock(&gWatchdogLock);

            // check if wake-up is enabled
            // check the wake pin enable
            txData[0] = (WAKE_PIN_ENABLE_REG_ADR << 1) + READ_BIT;
            txData[1] = 0;

            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                // output to the user
                cli_printfError("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

                // unlock the mutex 
                pthread_mutex_unlock(&gWatchdogLock);

                // return the error
                return lvRetValue;
            }

            if((rxData[1] & 3) == 0)
            {
                // error 
                cli_printfError("SBC ERROR: Wake isn't configured!\n");
                lvRetValue = -1;

                // unlock the mutex 
                pthread_mutex_unlock(&gWatchdogLock);

                // return
                return lvRetValue;
            }

            // check if SLPC = 0
            // read the SBC_CONF_CTRL_REG 
            txData[0] = (SBC_CONF_CTRL_REG_ADR << 1) + READ_BIT;
            txData[1] = 0;

            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                // output to the user
                cli_printfError("SBC ERROR: failed to read SBC_CONF_CTRL_REG! %d\n", lvRetValue);

                // unlock the mutex 
                pthread_mutex_unlock(&gWatchdogLock);

                // return the error
                return lvRetValue;
            }

            // check the SLPC bit
            if(rxData[1] & (1 << SBC_CONF_CTRL_SLPC_BIT))
            {
                // error
                cli_printfError("SBC ERROR: SBC CONF isn't configured!\n");
                lvRetValue = -1;

                // unlock the mutex 
                pthread_mutex_unlock(&gWatchdogLock);

                // return
                return lvRetValue;
            }

            // read the GLOBAL_EVENT_STAT_REG 
            txData[0] = (GLOBAL_EVENT_STAT_REG_ADR << 1) + READ_BIT;
            txData[1] = 0;

            // write the data to the SBC and receive data
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                // output to the user
                cli_printfError("SBC ERROR: failed to read GLOBAL_EVENT_STAT_REG_ADR! %d\n", lvRetValue);

                // unlock the mutex 
                pthread_mutex_unlock(&gWatchdogLock);

                // return the error
                return lvRetValue;
            }

#ifdef DEBUG_SBC_SPI
            cli_printf("Glob stat rx0: %d rx1: %d\n", rxData[0], rxData[1]);
#endif
            // save the global event status
            globalEventStat = rxData[1] & GLOBAL_EVENT_STAT_MASK;

            // check if there is a wakeup pending
            if(globalEventStat)
            {
                // check if it is the wake pin event
                if(globalEventStat & (1 << GLOBAL_EVENT_STAT_WPE_BIT))
                {
                    // read the WAKE_PIN_EVENT_STAT register
                    txData[0] = (WAKE_PIN_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                    txData[1] = 0;

                    // write the data to the SBC and receive data
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check for errors
                    if(lvRetValue)
                    {
                        // output to the user
                        cli_printfError("SBC ERROR: failed to read WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                            lvRetValue);

                        // unlock the mutex 
                        pthread_mutex_unlock(&gWatchdogLock);

                        // return the error
                        return lvRetValue;
                    }

#ifdef DEBUG_SBC_SPI
                    cli_printf("wake pin event stat: %d\n", rxData[1]);
#endif                  
                    // check if there is an error
                    if(rxData[1] & WAKE_PIN_EVENT_STAT_MASK)
                    {
                        cli_printf("clearing wake pin event status register!\n");

                        // reset the error
                        // write the WAKE_PIN_EVENT_STAT register
                        txData[0] = (WAKE_PIN_EVENT_STAT_REG_ADR << 1) + WRITE_BIT;
                        txData[1] = rxData[1] & WAKE_PIN_EVENT_STAT_MASK;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // read the WAKE_PIN_EVENT_STAT register
                        txData[0] = (WAKE_PIN_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                        txData[1] = 0;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to read WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // check if the register was cleared
                        // check if there is an error
                        if(rxData[1] & WAKE_PIN_EVENT_STAT_MASK)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to reset WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                                rxData[1] & WAKE_PIN_EVENT_STAT_MASK);

                            lvRetValue = -1;

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }
                    }
                }

                // check if there is tranceiver event wake-up
                if(globalEventStat & (1 << GLOBAL_EVENT_STAT_TRXE_BIT))
                {
                    // read the TRANSC_EVENT_STAT register
                    txData[0] = (TRANSC_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                    txData[1] = 0;

                    // write the data to the SBC and receive data
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check for errors
                    if(lvRetValue)
                    {
                        // output to the user
                        cli_printfError("SBC ERROR: failed to read TRANSC_EVENT_STAT_REG_ADR! %d\n", 
                            lvRetValue);

                        // unlock the mutex 
                        pthread_mutex_unlock(&gWatchdogLock);

                        // return the error
                        return lvRetValue;
                    }

#ifdef DEBUG_SBC_SPI
                    cli_printf("tranceiver event stat: %d\n", rxData[1]);
#endif                  
                    // check if there is an error
                    if(rxData[1] & TRANSC_EVENT_STAT_MASK)
                    {
                        cli_printf("clearing transceiver event status register!\n");

                        // reset the error
                        // write the WAKE_PIN_EVENT_STAT register
                        txData[0] = (WAKE_PIN_EVENT_STAT_REG_ADR << 1) + WRITE_BIT;
                        txData[1] = rxData[1] & TRANSC_EVENT_STAT_MASK;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // read the TRANSC_EVENT_STAT register
                        txData[0] = (TRANSC_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                        txData[1] = 0;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to read TRANSC_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // check if the register was resetted
                        // check if there is an error
                        if(rxData[1] & TRANSC_EVENT_STAT_MASK)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to reset TRANSC_EVENT_STAT_REG_ADR! %d\n", 
                                rxData[1] & TRANSC_EVENT_STAT_MASK);

                            lvRetValue = -1;

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }
                    }
                }

                // check if there is tranceiver event wake-up
                if(globalEventStat & (1 << GLOBAL_EVENT_STAT_SUPE_BIT))
                {
                    // read the SUPPLY_EVENT_STAT_REG_ADR register
                    txData[0] = (SUPPLY_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                    txData[1] = 0;

                    // write the data to the SBC and receive data
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check for errors
                    if(lvRetValue)
                    {
                        // output to the user
                        cli_printfError("SBC ERROR: failed to read SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
                            lvRetValue);

                        // unlock the mutex 
                        pthread_mutex_unlock(&gWatchdogLock);

                        // return the error
                        return lvRetValue;
                    }

#ifdef DEBUG_SBC_SPI
                    cli_printf("supply event stat: %d\n", rxData[1]);
#endif                  
                    // check if there is an error
                    if(rxData[1] & SUPPLY_EVENT_STAT_MASK)
                    {
                        cli_printf("clearing supply event status register!\n");

                        // reset the error
                        // write the WAKE_PIN_EVENT_STAT register
                        txData[0] = (SUPPLY_EVENT_STAT_REG_ADR << 1) + WRITE_BIT;
                        txData[1] = rxData[1] & SUPPLY_EVENT_STAT_MASK;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to write SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // read the register once more
                        // read the SUPPLY_EVENT_STAT_REG_ADR register
                        txData[0] = (SUPPLY_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                        txData[1] = 0;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to read SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // check if the register has not been cleared
                        if(rxData[1] & SUPPLY_EVENT_STAT_MASK)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to reset SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
                                rxData[1] & SUPPLY_EVENT_STAT_MASK);

                            lvRetValue = -1;

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }
                    }
                }

                // check if there is tranceiver event wake-up
                if(globalEventStat & (1 << GLOBAL_EVENT_STAT_SYSE_BIT))
                {
                    // read the SYS_EVENT_STAT_REG_ADR register
                    txData[0] = (SYS_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                    txData[1] = 0;

                    // write the data to the SBC and receive data
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check for errors
                    if(lvRetValue)
                    {
                        // output to the user
                        cli_printfError("SBC ERROR: failed to read SYS_EVENT_STAT_REG_ADR! %d\n", 
                            lvRetValue);

                        // unlock the mutex 
                        pthread_mutex_unlock(&gWatchdogLock);

                        // return the error
                        return lvRetValue;
                    }

#ifdef DEBUG_SBC_SPI
                    cli_printf("sys event stat: %d\n", rxData[1]);
#endif                  
                    // check if there is an error
                    if(rxData[1] & SYS_EVENT_STAT_MASK)
                    {
                        cli_printf("clearing sys event status register!\n");

                        // reset the error
                        // write the SYS_EVENT_STAT_REG_ADR register
                        txData[0] = (SYS_EVENT_STAT_REG_ADR << 1) + WRITE_BIT;
                        txData[1] = rxData[1] & SYS_EVENT_STAT_MASK;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // read the SYS_EVENT_STAT_REG_ADR register
                        txData[0] = (SYS_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                        txData[1] = 0;

                        // write the data to the SBC and receive data
                        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                        // check for errors
                        if(lvRetValue)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to read SYS_EVENT_STAT_REG_ADR! %d\n", 
                                lvRetValue);

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);

                            // return the error
                            return lvRetValue;
                        }

                        // check if there is still an error
                        if(rxData[1] & SYS_EVENT_STAT_MASK)
                        {
                            // output to the user
                            cli_printfError("SBC ERROR: failed to reset SYS_EVENT_STAT_REG_ADR! %d\n", 
                                rxData[1] & SYS_EVENT_STAT_MASK);

                            lvRetValue = -1;

                            // unlock the mutex 
                            pthread_mutex_unlock(&gWatchdogLock);
                            
                            // return the error
                            return lvRetValue;
                        }
                    }
                }

                // read the global register again
                // read the GLOBAL_EVENT_STAT_REG 
                txData[0] = (GLOBAL_EVENT_STAT_REG_ADR << 1) + READ_BIT;
                txData[1] = 0;

                // write the data to the SBC and receive data
                lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                // check for errors
                if(lvRetValue)
                {
                    // output to the user
                    cli_printfError("SBC ERROR: failed to read GLOBAL_EVENT_STAT_REG_ADR! %d\n", 
                        lvRetValue);

                    // unlock the mutex 
                    pthread_mutex_unlock(&gWatchdogLock);

                    // return the error
                    return lvRetValue;
                }

                // check if there is still a wake-up pending
                if(rxData[1] & GLOBAL_EVENT_STAT_MASK)
                {
                    // output to the user
                    cli_printfError("SBC ERROR: failed to reset GLOBAL_EVENT_STAT_REG_ADR! %d\n", 
                        rxData[1] & GLOBAL_EVENT_STAT_MASK);

                    lvRetValue = -1;

                    // unlock the mutex 
                    pthread_mutex_unlock(&gWatchdogLock);

                    // return the error
                    return lvRetValue;
                }
            }

            // to make sure the latest messages are send
            usleep(10);

            // set the sleep mode
            txData[1] = MODE_CONTROL_SLEEP_MODE;
    
        break;

        // in case of the standby mode 
        case SBC_STANDBY:

            // set the sleep mode
            txData[1] = MODE_CONTROL_STANDBY_MODE;

        break;

        // in case of the standby mode 
        case SBC_NORMAL:

            // set the sleep mode
            txData[1] = MODE_CONTROL_NORMAL_MODE;

        break;
    }

    // set the address and set it in write mode 
    txData[0] = (MODE_CTRL_REG_ADR << 1) + WRITE_BIT;

    // write the data to the SBC
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

#ifdef DEBUG_SBC_SPI
    cli_printf("Mode W rx: 0: %d 1: %d\n", rxData[0], rxData[1]);
#endif

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("SBC ERROR: failed to set mode! %d\n", lvRetValue);
    }

    // check the mode 
    // set the address and set it in read mode 
    txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

    // read the data from the SBC
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("SBC ERROR: failed to get mode! %d\n", lvRetValue);
    }

#ifdef DEBUG_SBC_SPI
    cli_printf("Mode R rx: 0: %d 1: %d\n", rxData[0], rxData[1]);
#endif

    // check for errors
    if((rxData[1] & MODE_CONTROL_WRITE_MASK) != txData[1])
    {
        cli_printfError("SBC ERROR: Couldn't verify SBC mode! %d != %d\n", (rxData[1] & MODE_CONTROL_WRITE_MASK), txData[1]);
    }

    // unlock the mutex 
    pthread_mutex_unlock(&gWatchdogLock);

    // check if the watchdog needs to be off
    if(!lvRetValue && (newMode == SBC_SLEEP))
    {
        // set the watchdog off
        if(sbc_setWatchdogMode(WD_AUTONOMOUS, FAST_WATCHDOG))
        {
            cli_printfError("SBC ERROR: Couldn't set the watchdog to autonomous mode (off) 3\n");
        }
    }
    // if the watchdog needs to be on
    else
    {
#ifndef DISABLE_WATCHDOG
        // set the watchdog to timeout mode
        if(sbc_setWatchdogMode(WD_TIMEOUT, FAST_WATCHDOG))
        {
            cli_printfError("SBC ERROR: Couldn't set the watchdog to timeout mode\n");
        }
#else
        // set the watchdog off
        if(sbc_setWatchdogMode(WD_AUTONOMOUS, FAST_WATCHDOG))
        {
            cli_printfError("SBC ERROR: Couldn't set the watchdog to autonomous mode (off) 3\n");
        }
#endif
    }   

    // return to the user
    return lvRetValue;
}

/*!
 * @brief   this function is used to get the SBC mode 
 * @note    Multi-thread protected
 *          
 * @param   None
 *
 * @return  If successful, the function will return the new mode from the sbc_mode_t enum. 
 *          Otherwise negative (-1 or -2)
 */
int sbc_getSbcMode(void)
{
    int lvRetValue = -1;
    uint8_t txData[2];
    uint8_t rxData[2];

    // check the mode 
    // set the address and set it in read mode 
    txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

    // read the data from the SBC
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        cli_printfError("SBC ERROR: failed to get mode! %d\n", lvRetValue);
        // set return value to -1 
        lvRetValue = -1;
    }
    else
    {
        // check the mode
        switch(rxData[1] & MODE_CONTROL_WRITE_MASK)
        {
            case MODE_CONTROL_SLEEP_MODE:
                // set the return value 
                lvRetValue = SBC_SLEEP;
            break;
            case MODE_CONTROL_STANDBY_MODE:
                // set the return value 
                lvRetValue = SBC_STANDBY;
            break;
            case MODE_CONTROL_NORMAL_MODE:
                // set the return value 
                lvRetValue = SBC_NORMAL;
            break;
            default:
                cli_printfError("Wrong mode: Rx: 0x%x and 0x%x\n", rxData[0], rxData[1]);
                // set the return value 
                lvRetValue = -2;
            break;
        }
    }

    // return the mode
    return lvRetValue;
}

/*!
 * @brief   this function is used to set the CAN FD mode on or off 
 *          
 * @param   on if true, CAN FD will be tollerated, if false it will not
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setCANFDMode(bool on)
{
    int lvRetValue = -1;
    uint8_t txData[2], rxData[2];

    // read the CAN control register
    txData[0] = (CAN_CTRL_REG_ADR << 1) + READ_BIT;
    txData[1] = 0;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to read CAN CTRL! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    // write the CAN control register
    txData[0] = (CAN_CTRL_REG_ADR << 1) + WRITE_BIT;
    txData[1] = rxData[1] | (on << CAN_CTRL_CFDC_BIT);

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    return lvRetValue;
}

/*!
 * @brief   this function is used to kick the watchdog, which will reset it. 
 * @note    Multi-thread protected 
 *          
 * @param   none
 *
 * @return  If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_kickTheWatchdog(void)
{
    int lvRetValue = -1;
    uint8_t txData[2], rxData[2];

#ifdef DEBUG_WATCHDOG_TIME
    struct timespec currentTime;
    uint32_t timeDifference;
    static maxTimeDifference = 0;
    static bool firstTime = true;
    static struct timespec oldTime  =   {.tv_sec = 2, 
                                        .tv_nsec = 0};
#endif

    // check if not initialzed
    if(!gWatchdogLockInitialized)
    {
        // output to the user
        cli_printfError("SBC ERROR: watchdog mutex not initialized!\n");

        // return the error
        return lvRetValue;
    }

    // lock the mutex 
    pthread_mutex_lock(&gWatchdogLock);

    // read the watchdog control register to get the mode and the period
    txData[0] = (WATCHDOG_CTRL_REG_ADR << 1) + READ_BIT;
    txData[1] = 0;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to read WATCHDOG CTRL! %d\n", lvRetValue);
    }
    // if it went ok
    else
    {
        // make the watchdog control register value 
        // and write the register again to reset the watchdog
        txData[0] = (WATCHDOG_CTRL_REG_ADR << 1) + WRITE_BIT;
        txData[1] = rxData[1];

        // write the data to the SBC and receive data
        lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

        // check for errors
        if(lvRetValue)
        {
            // output to the user
            cli_printfError("SBC ERROR: failed to write WATCHDOG CTRL! %d\n", lvRetValue);
        }
    }

#ifdef DEBUG_WATCHDOG_TIME

    // check if the first time
    if(firstTime)
    {
        // make sure to only do it once
        firstTime = false;

        // get the time of oldTime
        if(clock_gettime(CLOCK_REALTIME, &oldTime))
        {
            cli_printfError("SBC ERROR: failed to get old time! errno: %d\n", errno);
        }
    }

    // get the current time
    if(clock_gettime(CLOCK_REALTIME, &currentTime))
    {
        cli_printfError("SBC ERROR: failed to get current time! errno: %d\n", errno);
    }

    // get the time difference
    timeDifference = data_getUsTimeDiff(currentTime, oldTime);

    // check if the new difference is more than the max value
    if(timeDifference > maxTimeDifference)
    {
        // save the new max
        maxTimeDifference = timeDifference;

        // output the differnece
        cli_printf("New max watchdog trigger time: %dms\n", timeDifference/1000);
    }

    // save the current time in the old time
    oldTime.tv_sec = currentTime.tv_sec;
    oldTime.tv_nsec = currentTime.tv_nsec;

    // check if in the INIT
    if(INIT == data_getMainState())
    {
        // reset the max time
        maxTimeDifference = 0;
    }

#endif

    // unlock the mutex 
    pthread_mutex_unlock(&gWatchdogLock);

    // return
    return lvRetValue;
}

/*! 
 * @brief   this function is used to set a new watchdog mode in the SBC. 
 * @warning keep in mind that change the watchdog mode means disabling 5V (CAN tranceiver) briefly 
 * @note    Multi-thread protected 
 *
 * @param   newMode The new watchdog mode from the watchdogModes_t enum.
 * @param   slow True if the watchdog needs to be slow, false otherwise
 *
 * @return  0 if succeeded, -1 otherwise.
 */
int sbc_setWatchdogMode(watchdogMode_t newMode, bool slow)
{
    int lvRetValue = -1;
    bool alreadyInThisMode = false;
    uint8_t txData[2], rxData[2], newWDModeReg = 0;
    uint8_t oldSBCModeReg = MODE_CONTROL_STANDBY_MODE;
    uint8_t watchdogspeed = 0;

    // check if not initialzed
    if(!gWatchdogLockInitialized)
    {
        // output to the user
        cli_printfError("SBC ERROR: watchdog mutex not initialized!\n");

        // return the error
        return lvRetValue;
    }

    // lock the mutex 
    pthread_mutex_lock(&gWatchdogLock);

    // check if the watchdog needs to be slow
    if(slow)
    {
        // set the slow speed 
        watchdogspeed = WATCHDOG_PERIOD_SLOW;
        // cli_printf("WatchdogMode slow\n");

        // check if the slow mode is more than the fast mode
        if(WATCHDOG_PERIOD_SLOW != WATCHDOG_PERIOD_FAST)
        {
            cli_printf("Setting watchdog to slow mode\n");
        }
    }
    // if it needs to be fase
    else
    {
        // set the fast speed 
        watchdogspeed = WATCHDOG_PERIOD_FAST;
        // cli_printf("WatchdogMode fast\n");
    }

    // read the watchdog control register to get the mode and the period
    txData[0] = (WATCHDOG_CTRL_REG_ADR << 1) + READ_BIT;
    txData[1] = 0;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to read WATCHDOG CTRL! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }
    else
    {
        // check if the watchdog is already in this mode
        // if not set the new registerdata
        switch(newMode)
        {
            // in case of autonomous mode
            case WD_AUTONOMOUS:

                // check if already in this mode and if the watchdog time is the same
                if((((rxData[1] >> WATCHDOG_CTRL_WMC_BIT) & WATCHDOG_CTRL_WMC_MASK) == WATCHDOG_CTRL_WMC_AUTONOM) && 
                    (((rxData[1] >> WATCHDOG_CTRL_NWP_BIT) & WATCHDOG_CTRL_NWP_MASK) == watchdogspeed))
                {                   
                    // state that you are already in this mode
                    alreadyInThisMode = true;
                }
                else
                {
                    // make the new register value
                    newWDModeReg = (WATCHDOG_CTRL_WMC_AUTONOM << WATCHDOG_CTRL_WMC_BIT) + watchdogspeed;
                }

            break;

            // in case of timout mode
            case WD_TIMEOUT:

                // check if already in this mode and if the watchdog time is the same
                if((((rxData[1] >> WATCHDOG_CTRL_WMC_BIT) & WATCHDOG_CTRL_WMC_MASK) == WATCHDOG_CTRL_WMC_TIMEOUT) && 
                    (((rxData[1] >> WATCHDOG_CTRL_NWP_BIT) & WATCHDOG_CTRL_NWP_MASK) == watchdogspeed))
                {
                    // state that you are already in this mode
                    alreadyInThisMode = true;
                }
                else
                {
                    // make the new register value
                    newWDModeReg = (WATCHDOG_CTRL_WMC_TIMEOUT << WATCHDOG_CTRL_WMC_BIT) + watchdogspeed;
                }

            break;

            // in case of window mode
            case WD_WINDOW:

                // check if already in this mode and if the watchdog time is the same
                if((((rxData[1] >> WATCHDOG_CTRL_WMC_BIT) & WATCHDOG_CTRL_WMC_MASK) == WATCHDOG_CTRL_WMC_WINDOW) && 
                    (((rxData[1] >> WATCHDOG_CTRL_NWP_BIT) & WATCHDOG_CTRL_NWP_MASK) == watchdogspeed))
                {
                    // state that you are already in this mode
                    alreadyInThisMode = true;
                }
                else
                {                   
                    // make the new register value
                    newWDModeReg = (WATCHDOG_CTRL_WMC_WINDOW << WATCHDOG_CTRL_WMC_BIT) + watchdogspeed;
                }

            break;
        }

        // check if not already in this mode
        if(!alreadyInThisMode)
        {
            // read the SBC mode
            // set the address and set it in read mode 
            txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

            // write the data to the SBC
            lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

            // check for errors
            if(lvRetValue)
            {
                cli_printfError("SBC ERROR: failed to get mode! %d\n", lvRetValue);
            }

            // check if not STANDBY, set standby mode
            if((rxData[1] & MODE_CONTROL_WRITE_MASK) != MODE_CONTROL_STANDBY_MODE)
            {
                // save the old mode register 
                oldSBCModeReg = rxData[1] & MODE_CONTROL_WRITE_MASK;

                cli_printfWarning("NOTICE: Disabling 5V regulator (CAN transceiver) briefly!\n");

                // set the mode to standby 
                // set the address and set it in write mode 
                txData[0] = (MODE_CTRL_REG_ADR << 1) + WRITE_BIT;
                txData[1] = MODE_CONTROL_STANDBY_MODE;

                // write the data to the SBC
                lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                // check for errors
                if(lvRetValue)
                {
                    cli_printfError("SBC ERROR: failed to set mode! %d\n", lvRetValue);
                }

                // check the mode 
                // read the SBC mode
                // set the address and set it in read mode 
                txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

                // write the data to the SBC
                lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                // check if not STANDBY, set standby mode
                if((rxData[1] & MODE_CONTROL_WRITE_MASK) != MODE_CONTROL_STANDBY_MODE)
                {
                    // set the error value
                    lvRetValue = -1;
                }

                // check for errors
                if(lvRetValue)
                {
                    cli_printfError("SBC ERROR: failed to get or set mode! %d\n", lvRetValue);
                }
            }

            // check for no errors
            if(!lvRetValue)
            {
                // set the new watchdog mode
                txData[0] = (WATCHDOG_CTRL_REG_ADR << 1) + WRITE_BIT;
                txData[1] = newWDModeReg;

#ifdef DEBUG_SBC_SPI
                cli_printf("writing 0x%x to watchdog control reg\n", newWDModeReg);
#endif
                // output to the user
                //cli_printf("Writing new watchdog mode from enum: 0x%x (0x%x)\n", newMode, newWDModeReg);

                // write the data to the SBC
                lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                // check for errors
                if(lvRetValue)
                {
                    cli_printfError("SBC ERROR: failed to set WD mode! %d\n", lvRetValue);
                }
                else
                {
                    // read the WD mode 
                    txData[0] = (WATCHDOG_CTRL_REG_ADR << 1) + READ_BIT;

                    // write the data to the SBC
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check if it went not ok
                    if(lvRetValue || (rxData[1] != newWDModeReg))
                    {
                        cli_printfError("SBC ERROR: failed to set or read WD mode! %d 0x%x != 0x%x\n", 
                            lvRetValue, rxData[1], newWDModeReg);

                        // set the error value
                        lvRetValue = -1;
                    }
                }
            }

            // check if a new mode was set
            if(oldSBCModeReg != MODE_CONTROL_STANDBY_MODE)
            {
                // return to old SBC mode 
                txData[0] = (MODE_CTRL_REG_ADR << 1) + WRITE_BIT;
                txData[1] = oldSBCModeReg;

                // write the data to the SBC
                lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                // check for errors
                if(lvRetValue)
                {
                    cli_printfError("SBC ERROR: failed to set old mode! %d\n", lvRetValue);
                }
                else
                {
                    // check if the mode is set
                    txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

                    // write the data to the SBC
                    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

                    // check if not STANDBY, set standby mode
                    if((rxData[1] & MODE_CONTROL_WRITE_MASK) != (oldSBCModeReg & MODE_CONTROL_WRITE_MASK))
                    {
                        // set the error value
                        lvRetValue = -1;
                    }

                    // check for errors
                    if(lvRetValue)
                    {
                        cli_printfError("SBC ERROR: failed to get or set old mode! %d\n", lvRetValue);
                    }
                }
            }
        }
    }

    // unlock the mutex 
    pthread_mutex_unlock(&gWatchdogLock);

    // return
    return lvRetValue;
}

/****************************************************************************
 * private Functions
 ****************************************************************************/

/*! 
 * @brief   this function is used to program the START_UP_CTRL_REG, SBC_CONF_CTRL_REG, MTPNV_CRC_CTRL
 *          Make sure it can be programmed
*/
int programNVMPSRegisters(void)
{
    int lvRetValue = -1;
    uint8_t txData[2], rxData[2];
    uint8_t crc, data;
    uint16_t i, j;

    // set the start-up control register to 20ms reset pulse width and V2 off
    txData[0] = (START_UP_CTRL_REG_ADR << 1) + WRITE_BIT;
    txData[1] = START_UP_CTRL_REG_VAL;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    cli_printf("START_UP_CTRL W: RX0: %d, RX1: %d\n", rxData[0], rxData[1]);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to write START_UP_CTRL_REG! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    // write the SBC configuration register 
    txData[0] = (SBC_CONF_CTRL_REG_ADR << 1) + WRITE_BIT;
    txData[1] = SBC_CONF_CTRL_REG_VAL;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    cli_printf("SBC conf ctrl W: RX0: %d, RX1: %d\n", rxData[0], rxData[1]);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to write SBC_CONF_CTRL_REG! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    cli_printf("Restarting!\n");

    usleep(10*1000);

    // reset the CRC variables
    data = 0; // unsigned byte
    crc = 0xFF;

    // loop through the 2 data registers
    for(i = 0; i < 2; i++)
    {
        // check which register it is about
        if(i == 0)
        {
            // make the data with the first register (0x73)
            data = ((START_UP_CTRL_REG_VAL) ^ crc);
        }
        else
        {
            // make the data with the second register (0x74)
            data = ((SBC_CONF_CTRL_REG_VAL) ^ crc);
        }

        // loop though the byte
        for(j = 0; j < 8; j++)
        {
            // check if larger or equal than 128
            if(data >= 128)
            {
                // shift left by 1
                data = (data * 2);
                // xor
                data = (data ^ 0x2F);
            }
            else
            {
                // shift left by 1
                data = (data * 2);
            }
        }

        // set the CRC
        crc = data;
    }

    // set the CRC with xor
    crc = (crc ^ 0xFF);

    // write the CRC value
    // write the SBC configuration register
    txData[0] = ((MTPNV_CRC_CTRL_REG_ADR << 1) + WRITE_BIT);
    txData[1] = crc;

    // write the data to the SBC and receive data
    lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

    cli_printf("CRC W: RX0: %d, RX1: %d\n", rxData[0], rxData[1]);

    // check for errors
    if(lvRetValue)
    {
        // output to the user
        cli_printfError("SBC ERROR: failed to write CRC! %d\n", lvRetValue);

        // return the error
        return lvRetValue;
    }

    return lvRetValue;
}

/*! 
 * @brief   this function is used to check if the NRST pin is connected to the SBC
 *
 * @param   none
 *
 * @return  1 if OK, 0 if not OK and negative if error 
 */
int checkNrstLine(void)
{
    int nrstOke = -1;
    int fd = 0, errorCode;
    ssize_t ret;
    char readData[NRST_CHECK_BYTES];

    // open de proc fs of the reset cause
    fd = open("/proc/nrstcheck", O_RDONLY);

    // check for errors
    if(fd < 0)
    {
        // error
        errorCode = errno;
        cli_printfError("SBC ERROR: Can't open nrstcheck fs, %d\n",
            errorCode);

        // set the error code
        nrstOke = fd;
    }
    // if no error
    else
    {
        // read the reset cause
        ret = read(fd, readData, NRST_CHECK_BYTES);

        // check for errors
        if(ret <= 0)
        {
            cli_printfError("SBC ERROR: Can't read nrstcheck fs: %d\n", ret);
            
            // if ret == 0
            if(ret == 0)
            {
                // set a negative value
                nrstOke = ret - 1;
            }
            else
            {
                // set the error code
                nrstOke = ret;
            }
        }
        else
        {
            // convert reset cause string to number
            nrstOke = (int)strtol(readData, NULL, 0);

            // check for error input
            if(nrstOke != 0 && nrstOke != 1)
            {
                // error ouptut 
                cli_printfError("SBC ERROR: Wrong conversion? %s -> %d\n", 
                    readData, nrstOke);

                // check if larger than 1
                if(nrstOke > 1)
                {
                    // make negative 
                    nrstOke *= -1;
                }
            }
        }
    }

    // close the device
    close(fd);

    // return
    return nrstOke;
}

//EOF
