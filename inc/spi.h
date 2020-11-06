/****************************************************************************
 * nxp_bms/BMS_v1/inc/spi.h
 *
 * BSD 3-Clause License
 * 
 * Copyright 2020 NXP
 * All rights reserved.
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
 **     Filename    : spi.h
 **     Project     : SmartBattery_RDDRONE_BMS772
 **     Processor   : S32K144
 **     Version     : 1.00
 **     Date   		: 2020-03-17
 **     Abstract    :
 **        spi module.
 **        This module contains all functions needed for spi
 **
 ** ###################################################################*/
/*!
 ** @file spi.h
 **
 ** @version 01.00
 **
 ** @brief
 **        spi module. this module contains the functions for spi
 **
 */
#ifndef SPI_H_
#define SPI_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
//#include "BMS_data_types.h"
#include <stdio.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#ifndef CONFIG_SPI_TYPE_DEFAULT
#define  CONFIG_SPI_TYPE_DEFAULT SPIDEVTYPE_USER
#endif

#ifndef CONFIG_SPI_CSN_DEFAULT
#define  CONFIG_SPI_CSN_DEFAULT 0
#endif

#ifndef CONFIG_SPI_MODE_DEFAULT
#define  CONFIG_SPI_MODE_DEFAULT  1 /* CPOL=0, CHPHA=1 */
#endif

#ifndef CONFIG_SPI_WIDTH_DEFAULT
#define  CONFIG_SPI_WIDTH_DEFAULT 16
#endif

#ifndef CONFIG_BCC_SPI_WIDTH_DEFAULT
#define  CONFIG_BCC_SPI_WIDTH_DEFAULT 40
#endif

#ifndef CONFIG_SPI_FREQ_DEFAULT
#define  CONFIG_SPI_FREQ_DEFAULT 4000000 
#endif

#ifndef CONFIG_SPI_UDELAY_DEFAULT
#define  CONFIG_SPI_UDELAY_DEFAULT 0
#endif

#ifndef CONFIG_SPI_NWORDS_DEFAULT
#define  CONFIG_SPI_NWORDS_DEFAULT 1
#endif


#ifndef CONFIG_SPI_MINBUS_DEFAULT
#define  CONFIG_SPI_MINBUS_DEFAULT 0
#endif

#ifndef CONFIG_SPI_MAXBUS_DEFAULT
#define  CONFIG_SPI_MAXBUS_DEFAULT 1
#endif

/*******************************************************************************
 * Types
 ******************************************************************************/
// the struct to configure the SPI
typedef struct 
{
	uint32_t 	devtype;    /* DevType (see spi_devtype_e) 	*/	
	uint32_t 	csn;        /* Chip select number for devtype  */
	uint8_t 	mode;       /* Mode to use for transfer       	*/
	uint8_t 	width;      /* is the data width (8 or 16)   	*/
	uint32_t 	freq;       /* SPI frequency                  	*/
	useconds_t 	udelay;   	/* Delay in uS after transfer   	*/
	uint32_t 	nwords;     /* No of words to exchange       	*/
}spiStruct_s;
/*******************************************************************************
 * public functions
 ******************************************************************************/
/*!
 * @brief 	this function is the SPI application driver 
 * 			it will take care of SPI transfers
 * 			SPI transmits or receives can be done with this function
 * 			
 * @param 	spiBus Which spi bus to use. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param 	txDataBuf The transmit data buffer to transmit (max 40 bits / 5 bytes), this may be NULL if not used
 * @param 	rxDataBuf The receive data buffer to receive (max 40 bits / 5 bytes), this may be NULL if not used
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 * @example for(i = 0; i < transferSize; i++)
 * 			{
 * 				txDataBuf[i] = i; // set the right data 
 * 			}	
 * 			if(spi_BMSTransferData(SPIBUS_BCC, txDataBuf, rxDataBuf, transferSize))
 *			{
 *				// do something with the error
 *			}
 * 
 * 			// do something with the rxDataBuf
 */
int spi_BMSTransferData(uint8_t  spiBus, uint8_t *txDataBuf, uint8_t *rxDataBuf);

/*!
 * @brief 	This function configures the SPI. 
 * 			it will set the source file global spiStruct_s struct with new values
 * 			in startup this is configured with the default values
 * 			
 * @param 	newSpiConfiguration the new spiStruct_s struct to set the value
 * @param 	BCCconfiguration if this is true it will set the BCC configuration
 *
 * @return 	If successful, the function will return zero (OK). Otherwise -1
 * @example if(spi_configure(newSpiConfiguration))
 * 			{
 *				// do something with the error
 *			}	
 */
int spi_configure(spiStruct_s newSpiConfiguration, bool BCCconfiguration);


/*******************************************************************************
 * EOF
 ******************************************************************************/

#endif /* SPI_H_ */
