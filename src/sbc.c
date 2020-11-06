/****************************************************************************
 * nxp_bms/BMS_v1/src/sbc.c
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
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
#include "sbc.h"
#include "cli.h"

#include <time.h>
#include <errno.h>

#include "data.h"
#include "spi.h"
#include "gpio.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define SBC_SPI_BUS 0
#define READ_BIT 					1
#define WRITE_BIT 					0

#define UJA1169TK_F_3_ID			0xE9

#define WATCHDOG_CTRL_REG_ADR		0x00
#define MODE_CTRL_REG_ADR 			0x01
#define MAIN_STATUS_REG_ADR			0x03
#define REGULATOR_CTRL_REG_ADR		0x10 
#define CAN_CTRL_REG_ADR			0x20
#define WAKE_PIN_STATUS_REG_ADR 	0x4B
#define WAKE_PIN_ENABLE_REG_ADR		0x4C
#define GLOBAL_EVENT_STAT_REG_ADR	0x60
#define SYS_EVENT_STAT_REG_ADR 		0x61
#define SUPPLY_EVENT_STAT_REG_ADR	0x62
#define TRANSC_EVENT_STAT_REG_ADR	0x63
#define WAKE_PIN_EVENT_STAT_REG_ADR 0x64
#define MTPNV_STATUS_REG_ARD		0x70
#define START_UP_CTRL_REG_ADR		0x73
#define SBC_CONF_CTRL_REG_ADR		0x74
#define MTPNV_CRC_CTRL_REG_ADR		0x75
#define IDENTIFICATION_REG_ADR		0x7E

#define WATCHDOG_CTRL_NWP_BIT 		0
#define WATCHDOG_CTRL_WMC_BIT 		5
#define REGULATOR_CTRL_PDC_BIT 		6
#define REGULATOR_CTRL_V2C_BIT 		2
#define CAN_CTRL_CFDC_BIT			6
#define CAN_CTRL_PNCOK_BIT			5
#define CAN_CTRL_CPNC_BIT			4
#define CAN_CTRL_CMC_BIT			0
#define WAKE_PIN_ENABLE_WPFE_BIT	0
#define WAKE_PIN_ENABLE_WPRE_BIT	1
#define GLOBAL_EVENT_STAT_WPE_BIT   3
#define GLOBAL_EVENT_STAT_TRXE_BIT 	2
#define GLOBAL_EVENT_STAT_SUPE_BIT	1
#define GLOBAL_EVENT_STAT_SYSE_BIT	0
#define SYS_EVENT_STAT_PO_BIT		4
#define SYS_EVENT_STAT_OTW_BIT		2
#define SYS_EVENT_STAT_SPIF_BIT		1
#define SYS_EVENT_STAT_WDF_BIT		0
#define SUPPLY_EVENT_STAT_V20_BIT	2
#define SUPPLY_EVENT_STAT_V2U_BIT	1
#define SUPPLY_EVENT_STAT_V1U_BIT	0
#define TRANSC_EVENT_STAT_PNFDE_BIT	5
#define TRANSC_EVENT_STAT_CBS_BIT	4
#define TRANSC_EVENT_STAT_CF_BIT	1
#define TRANSC_EVENT_STAT_CW_BIT	0
#define WAKE_PIN_EVENT_STAT_WPR_BIT	1
#define WAKE_PIN_EVENT_STAT_WPF_BIT	0
#define MTPNV_STATUS_NVMPS_BIT 		0
#define START_UP_CTRL_V2SUC_BIT 	3
#define START_UP_CTRL_RLC_BIT		4
#define SBC_CONF_CTRL_SLPC_BIT 		0
#define SBC_CONF_CTRL_SDMC_BIT 		2
#define SBC_CONF_CTRL_FNMC_BIT 		3

// watchdog register can only be written in standby mode!
#define WATCHDOG_CTRL_WMC_AUTONOM	1 	
#define WATCHDOG_CTRL_WMC_TIMEOUT	2
#define WATCHDOG_CTRL_WMC_WINDOW	4
#define WATCHDOG_CTRL_NWP_8			0x8
#define WATCHDOG_CTRL_NWP_16		0x1
#define WATCHDOG_CTRL_NWP_32		0x2
#define WATCHDOG_CTRL_NWP_64		0xB
#define WATCHDOG_CTRL_NWP_128		0x4
#define WATCHDOG_CTRL_NWP_256		0xD
#define WATCHDOG_CTRL_NWP_1024		0xE
#define WATCHDOG_CTRL_NWP_4096		0x7

#define MODE_CONTROL_SLEEP_MODE 	1
#define MODE_CONTROL_STANDBY_MODE 	4
#define MODE_CONTROL_NORMAL_MODE 	7

#define REGULATOR_CTRL_PDC_DEFAULT 	0
#define REGULATOR_CTRL_V2C_OFF 		0
#define REGULATOR_CTRL_V2C_NORMAL	1
#define REGULATOR_CTRL_V2C_ALL_NRST	2
#define REGULATOR_CTRL_V2C_ALL_RST 	3
#define REGULATOR_CTRL_V2C_MASK		0x0C

#define CAN_CTRL_CFDC_CANFD_DIS		0
#define CAN_CTRL_CFDC_CANFD_EN		1
#define CAN_CTRL_PNCOK_INV			0
#define CAN_CTRL_PNCOK_SUC			1
#define CAN_CTRL_CPNC_DIS			0
#define CAN_CTRL_CPNC_EN			1
#define CAN_CTRL_CMC_OFF			0
#define CAN_CTRL_CMC_ACT_UV_EN		1
#define CAN_CTRL_CMC_ACT_UV_DIS		2
#define CAN_CTRL_CMC_LIS			3

// set CAN FD OFF, disable CAN selective wake-up, Active mode (when the SBC is in Normal mode); 
// CAN supply undervoltage detection disabled
#define CAN_CTRL_REG_VAL 			(CAN_CTRL_CFDC_CANFD_DIS << CAN_CTRL_CFDC_BIT) + \
									(CAN_CTRL_PNCOK_INV << CAN_CTRL_PNCOK_BIT) + \
									(CAN_CTRL_CPNC_DIS << CAN_CTRL_CPNC_BIT) + \
									(CAN_CTRL_CMC_ACT_UV_DIS << CAN_CTRL_CMC_BIT)

// enable the wake with high to low on the wake pin
#define WAKE_PIN_ENABLE_REG_VAL    	1 << WAKE_PIN_ENABLE_WPFE_BIT

#define START_UP_CTRL_V2SUC_OFF		0
#define START_UP_CTRL_V2SUC_ON		1
#define START_UP_CTRL_RLC_20MS 		0
#define START_UP_CTRL_RLC_10MS 		1
#define START_UP_CTRL_RLC_3_6MS		2
#define START_UP_CTRL_RLC_1MS 		3

// set the reset pulse width to min 20ms and turn off V2
#define START_UP_CTRL_REG_VAL 		(START_UP_CTRL_RLC_20MS << START_UP_CTRL_RLC_BIT) + \
									(START_UP_CTRL_V2SUC_OFF << START_UP_CTRL_V2SUC_BIT)

#define SBC_CONF_CTRL_FNMC_OFF		0
#define SBC_CONF_CTRL_FNMC_ON		1
#define SBC_CONF_CTRL_SDMC_OFF		0
#define SBC_CONF_CTRL_SDMC_ON		1
#define SBC_CONF_CTRL_SLPC_ACC		0
#define SBC_CONF_CTRL_SLPC_IGN		1		

// set the Forced normal mode off, to go in the other modes
// set the Software Development mode on to disable the watchdog
// set the sleep command accept on
#define SBC_CONF_CTRL_REG_VAL		(SBC_CONF_CTRL_FNMC_OFF << SBC_CONF_CTRL_FNMC_BIT) + \
									(SBC_CONF_CTRL_SDMC_ON << SBC_CONF_CTRL_SDMC_BIT) + \
									(SBC_CONF_CTRL_SLPC_ACC << SBC_CONF_CTRL_SLPC_BIT)

#define WATCHDOG_CTRL_WRITE_MASK		0xEF
#define MODE_CONTROL_WRITE_MASK			0x7
#define GLOBAL_EVENT_STAT_MASK			0xF
#define SYS_EVENT_STAT_MASK				0x17
#define SUPPLY_EVENT_STAT_MASK			0x7
#define TRANSC_EVENT_STAT_MASK			0x33
#define WAKE_PIN_EVENT_STAT_MASK		0x3
#define WAKE_PIN_EN_WRITE_MASK 			0x3
#define SBC_CONF_CTRL_REG_WRITE_MASK 	0x3D 
#define START_UP_CTRL_REG_WRITE_MASK	0x38
#define CAN_CTRL_WRITE_MASK 			0x73

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
/*! @brief variable to indicate of it is initialized */
static bool gSbcInitialized = false;	
/****************************************************************************
 * private Functions
 ****************************************************************************/
/*! 
 * @brief 	this function is used to program the START_UP_CTRL_REG, SBC_CONF_CTRL_REG, MTPNV_CRC_CTRL
 * 			Make sure it can be programmed
*/
int programNVMPSRegisters(void);

/****************************************************************************
 * public functions
 ****************************************************************************/
/*!
 * @brief 	this function is used to initialze the SBC 
 * 			
 * @param 	none
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_initialize(void)
{
	int lvRetValue = -1;
	uint8_t txData[2];
	uint8_t rxData[2], rxData2[2];
	struct timespec currentTime, sampleTime;

	// check if initialized 
	if(!gSbcInitialized)
	{
		cli_printf("SELF-TEST START: SBC\n");
		//cli_printf("initializing SBC!\n");

		// check the SBC 
		if(sbc_verifySbc())
		{
			// output to the user and return
			cli_printf("SBC ERROR: Failed to verify the SBC!\n");
			
			// return error
			lvRetValue = -1;
			return lvRetValue;
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
			cli_printf("SBC ERROR: failed to read SBC_CONF_CTRL_REG! %d\n", lvRetValue);

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
			cli_printf("SBC ERROR: failed to write START_UP_CTRL_REG! %d\n", lvRetValue);

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
				cli_printf("NVMPS can't be overwitten!!!\n");

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

		// get the time 
		if(clock_gettime(CLOCK_REALTIME, &sampleTime))
        {
            cli_printf("SBC ERROR: failed to get sample time! errno: %d\n", errno);

            // than use sleep for 20 miliseconds
            sleep(1000*20);
        }

        // wait until the pin is high or 1sec timeout
		do
		{
			// get the time to check for an overflow
			if(clock_gettime(CLOCK_REALTIME, &currentTime) == -1)
			{
				cli_printf("SBC ERROR: failed to get currentTime! errno: %d\n", errno);

				// return error
				lvRetValue = -1;
				return lvRetValue;
			}

			// sleep a little bit
			usleep(1);

		} while ((!gpio_readPin(RST_N)) && (((currentTime.tv_sec - sampleTime.tv_sec) >= 1) 
				&& (currentTime.tv_nsec >= sampleTime.tv_nsec)));

		// check if timeout happend
		if(!gpio_readPin(RST_N))
		{
			// return error
			cli_printf("SBC ERROR: Timeout happend! RST_N pin: %d\n", gpio_readPin(RST_N));

			// return error
			lvRetValue = -1;
			return lvRetValue;
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
			cli_printf("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

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
			cli_printf("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

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
			cli_printf("SBC ERROR: failed to verify wake status! %d != %d\n", WAKE_PIN_ENABLE_REG_VAL, (rxData[1] & WAKE_PIN_EN_WRITE_MASK));

			// return error
			lvRetValue = -1;
			return lvRetValue;
		}

		// get the CAN FD mode 
		if(data_getParameter(UAVCAN_FD_MODE, &rxData2[0], NULL) == NULL)
	    {
	       cli_printf("SBC ERROR: couldn't get CANFD mode\n");
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
			cli_printf("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

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
			cli_printf("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

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
			cli_printf("SBC ERROR: failed to verify CAN control! %d != %d\n", 
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
			cli_printf("SBC ERROR: failed to read REGULATOR_CTRL_REG! %d\n", lvRetValue);

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
				cli_printf("SBC ERROR: failed to read REGULATOR_CTRL_REG! %d\n", lvRetValue);

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
				cli_printf("SBC ERROR: failed to write REGULATOR_CTRL_REG! %d\n", lvRetValue);

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
				cli_printf("SBC ERROR: failed to verify REGULATOR_CTRL_REG! %d\n", lvRetValue);

				// return the error
				lvRetValue = -1;
				return lvRetValue;
			}
		}

		cli_printf("Setting SBC to normal mode!\n");

		// set the mode to normal
		lvRetValue = sbc_setSbcMode(SBC_NORMAL);

		// check for errors
		if(lvRetValue)
		{
			// output to the user
			cli_printf("SBC ERROR: failed to set SBC to normal mode! %d\n", lvRetValue);

			// return the error
			return lvRetValue;
		}

		lvRetValue = 0;

		// the SBC is initialized 
		gSbcInitialized = true;

		cli_printf("SELF-TEST PASS:  SBC\n");
	}
	else
	{
		lvRetValue = 0;
	}

	// return 
	return lvRetValue;
}

/*!
 * @brief 	This function is used to verify the SBC using SPI 
 * 			It will check the Device identification register (0x7E)
 * 			
 * @param 	None
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
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
		cli_printf("SBC ERROR: failed to read ID! %d\n", lvRetValue);

		// return the error
		return lvRetValue;
	}

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
 * @brief 	this function is used to set the SBC mode 
 * 			
 * @param 	newMode the new mode from the sbc_mode_t enum
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 */
int sbc_setSbcMode(sbc_mode_t newMode)
{
	int lvRetValue = -1;
	uint8_t txData[2];
	uint8_t rxData[2];
	uint8_t globalEventStat;

	// calculate the tx data
	switch(newMode)
	{
		// in case of sleep mode 
		case SBC_SLEEP:	

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
				cli_printf("SBC ERROR: failed to read wake status! %d\n", lvRetValue);

				// return the error
				return lvRetValue;
			}

			if((rxData[1] & 3) == 0)
			{
				// error 
				cli_printf("SBC error: Wake isn't configured!\n");
				lvRetValue = -1;

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
				cli_printf("SBC ERROR: failed to read SBC_CONF_CTRL_REG! %d\n", lvRetValue);

				// return the error
				return lvRetValue;
			}

			// check the SLPC bit
			if(rxData[1] & (1 << SBC_CONF_CTRL_SLPC_BIT))
			{
				// error
				cli_printf("SBC error: SBC CONF isn't configured!\n");
				lvRetValue = -1;

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
				cli_printf("SBC ERROR: failed to read GLOBAL_EVENT_STAT_REG_ADR! %d\n", lvRetValue);

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
						cli_printf("SBC ERROR: failed to read WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
							lvRetValue);

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
							cli_printf("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

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
							cli_printf("SBC ERROR: failed to read WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

							// return the error
							return lvRetValue;
						}

						// check if the register was cleared
						// check if there is an error
						if(rxData[1] & WAKE_PIN_EVENT_STAT_MASK)
						{
							// output to the user
							cli_printf("SBC ERROR: failed to reset WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
								rxData[1] & WAKE_PIN_EVENT_STAT_MASK);

							lvRetValue = -1;

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
						cli_printf("SBC ERROR: failed to read TRANSC_EVENT_STAT_REG_ADR! %d\n", 
							lvRetValue);

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
							cli_printf("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

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
							cli_printf("SBC ERROR: failed to read TRANSC_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

							// return the error
							return lvRetValue;
						}

						// check if the register was resetted
						// check if there is an error
						if(rxData[1] & TRANSC_EVENT_STAT_MASK)
						{
							// output to the user
							cli_printf("SBC ERROR: failed to reset TRANSC_EVENT_STAT_REG_ADR! %d\n", 
								rxData[1] & TRANSC_EVENT_STAT_MASK);

							lvRetValue = -1;

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
						cli_printf("SBC ERROR: failed to read SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
							lvRetValue);

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
							cli_printf("SBC ERROR: failed to write SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

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
							cli_printf("SBC ERROR: failed to read SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

							// return the error
							return lvRetValue;
						}

						// check if the register has not been cleared
						if(rxData[1] & SUPPLY_EVENT_STAT_MASK)
						{
							// output to the user
							cli_printf("SBC ERROR: failed to reset SUPPLY_EVENT_STAT_REG_ADR! %d\n", 
								rxData[1] & SUPPLY_EVENT_STAT_MASK);

							lvRetValue = -1;

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
						cli_printf("SBC ERROR: failed to read SYS_EVENT_STAT_REG_ADR! %d\n", 
							lvRetValue);

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
							cli_printf("SBC ERROR: failed to write WAKE_PIN_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

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
							cli_printf("SBC ERROR: failed to read SYS_EVENT_STAT_REG_ADR! %d\n", 
								lvRetValue);

							// return the error
							return lvRetValue;
						}

						// check if there is still an error
						if(rxData[1] & SYS_EVENT_STAT_MASK)
						{
							// output to the user
							cli_printf("SBC ERROR: failed to reset SYS_EVENT_STAT_REG_ADR! %d\n", 
								rxData[1] & SYS_EVENT_STAT_MASK);

							lvRetValue = -1;
							
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
					cli_printf("SBC ERROR: failed to read GLOBAL_EVENT_STAT_REG_ADR! %d\n", 
						lvRetValue);

					// return the error
					return lvRetValue;
				}

				// check if there is still a wake-up pending
				if(rxData[1] & GLOBAL_EVENT_STAT_MASK)
				{
					// output to the user
					cli_printf("SBC ERROR: failed to reset GLOBAL_EVENT_STAT_REG_ADR! %d\n", 
						rxData[1] & GLOBAL_EVENT_STAT_MASK);

					lvRetValue = -1;

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
		cli_printf("SBC ERROR: failed to set mode! %d\n", lvRetValue);
	}

	// check the mode 
	// set the address and set it in read mode 
	txData[0] = (MODE_CTRL_REG_ADR << 1) + READ_BIT;

	// write the data to the SBC
	lvRetValue = spi_BMSTransferData(SBC_SPI_BUS, txData, rxData);

	// check for errors
	if(lvRetValue)
	{
		cli_printf("SBC ERROR: failed to set mode! %d\n", lvRetValue);
	}

#ifdef DEBUG_SBC_SPI
	cli_printf("Mode R rx: 0: %d 1: %d\n", rxData[0], rxData[1]);
#endif

	// check for errors
	if((rxData[1] & MODE_CONTROL_WRITE_MASK) != txData[1])
	{
		cli_printf("SBC ERROR: Couldn't verify SBC mode! %d != %d\n", (rxData[1] & MODE_CONTROL_WRITE_MASK), txData[1]);
	}

	// return to the user
	return lvRetValue;
}

/*!
 * @brief 	this function is used to set the CAN FD mode on or off 
 * 			
 * @param 	on if true, CAN FD will be tollerated, if false it will not
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
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
		cli_printf("SBC ERROR: failed to read CAN CTRL! %d\n", lvRetValue);

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
		cli_printf("SBC ERROR: failed to write CAN CTRL! %d\n", lvRetValue);

		// return the error
		return lvRetValue;
	}

	return lvRetValue;
}


/****************************************************************************
 * private Functions
 ****************************************************************************/

/*! 
 * @brief 	this function is used to program the START_UP_CTRL_REG, SBC_CONF_CTRL_REG, MTPNV_CRC_CTRL
 * 			Make sure it can be programmed
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
		cli_printf("SBC ERROR: failed to write START_UP_CTRL_REG! %d\n", lvRetValue);

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
		cli_printf("SBC ERROR: failed to write SBC_CONF_CTRL_REG! %d\n", lvRetValue);

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
		cli_printf("SBC ERROR: failed to write CRC! %d\n", lvRetValue);

		// return the error
		return lvRetValue;
	}

	return lvRetValue;
}


//EOF