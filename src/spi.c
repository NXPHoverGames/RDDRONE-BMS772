/****************************************************************************
 * nxp_bms/BMS_v1/src/spi.c
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
#include <sys/ioctl.h>
#include <fcntl.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi_transfer.h>

#include "spi.h"
#include "cli.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
/* Device naming */
#define DEVNAME_FMT    "/dev/spi%d"
#define DEVNAME_FMTLEN (8 + 3 + 1)

#define MAX_BCC_BUFFER_SIZE 8
#define MAX_BUFFER_SIZE 1


/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * private data
 ****************************************************************************/
static char gDevname[DEVNAME_FMTLEN];

// make the struct to configure the SPI transfer for the BCC
static spiStruct_s gBCCSpiStruct = 
{
	.devtype 	= CONFIG_SPI_TYPE_DEFAULT,
	.csn 		= CONFIG_SPI_CSN_DEFAULT, 
	.mode 		= CONFIG_SPI_MODE_DEFAULT, 
	.width 		= CONFIG_BCC_SPI_WIDTH_DEFAULT, 
	.freq 		= CONFIG_SPI_FREQ_DEFAULT,
	.udelay 	= CONFIG_SPI_UDELAY_DEFAULT, 
	.nwords 	= CONFIG_SPI_NWORDS_DEFAULT
};

// make the struct to configure the SPI transfer for other SPI transfers
static spiStruct_s gSpiStruct = 
{
	.devtype 	= CONFIG_SPI_TYPE_DEFAULT,
	.csn 		= CONFIG_SPI_CSN_DEFAULT, 
	.mode 		= CONFIG_SPI_MODE_DEFAULT, 
	.width 		= CONFIG_SPI_WIDTH_DEFAULT, 
	.freq 		= CONFIG_SPI_FREQ_DEFAULT,
	.udelay 	= CONFIG_SPI_UDELAY_DEFAULT, 
	.nwords 	= CONFIG_SPI_NWORDS_DEFAULT
};

/****************************************************************************
 * private Functions declerations 
 ****************************************************************************/
// insert the number of the SPI bus, will be 0 or 1 for the rddrone-bms772
// it will return the devpath "/dev/spi<bus>"
FAR char *spi_path(int bus);

// this function will open de spi device with O_RDONLY 
int spi_open(int bus);

// this function is used to transfer on the SPI bus
// it will use the ioctl function with SPIIOC_TRANSFER
int spi_ioctlTransfer(int fd, FAR struct spi_sequence_s *seq);

/****************************************************************************
 * main
 ****************************************************************************/

/*!
 * @brief 	this function is the SPI application driver 
 * 			it will take care of SPI transfers
 * 			SPI transmits or receives can be done with this function
 * 			
 * @param 	Which spi bus to use. this could for example be 0 if LPSPI0 is used. 1 if LPSPI1 is used
 * @param 	The transmit data buffer to transmit (max 40 bits / 5 bytes), this may be NULL if not used
 * @param 	The receive data buffer to receive (max 40 bits / 5 bytes), this may be NULL if not used
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
int spi_BMSTransferData(uint8_t  spiBus, uint8_t *txDataBuf, uint8_t *rxDataBuf)
{
	int lvRetValue = -1;
	int lvFd;
	struct spi_trans_s lvTrans;
	struct spi_sequence_s lvSeq;

	// make the transmit buffer
	uint8_t txdataBuffer[MAX_BCC_BUFFER_SIZE] =
	{
		0x00, 0x00, 0x00, 0x10, 0x1C
	};

	uint8_t *txdata = txdataBuffer;

	// check if write buffer isn't NULL
	if(txDataBuf != NULL)
	{
		txdata = txDataBuf;
	}

	// make the receive buffer
	uint8_t rxdataBuffer[MAX_BCC_BUFFER_SIZE] =
	{
		0x00
	};

	uint8_t *rxdata = rxdataBuffer;

	// check if receive buffer isn't null
	if(rxDataBuf != NULL)
	{
		rxdata = rxDataBuf;
	}

	// open the SPI device
	lvFd = spi_open(spiBus);
	if (lvFd < 0)
	{
		cli_printfError("SPI ERROR: failed to get bus %d\n", spiBus);
		return lvRetValue;
	}

	// check which bus it is
	if(spiBus == 1)
	{
		/* Set up the transfer profile */
		lvSeq.dev = SPIDEV_ID(gBCCSpiStruct.devtype, gBCCSpiStruct.csn);
		lvSeq.mode = gBCCSpiStruct.mode;
		lvSeq.nbits = gBCCSpiStruct.width;
		lvSeq.frequency = gBCCSpiStruct.freq;
		lvSeq.ntrans = 1; 				
		lvSeq.trans = &lvTrans;

		lvTrans.deselect = false;//true; 	/* De-select after transfer */
		lvTrans.delay = gBCCSpiStruct.udelay;
		lvTrans.nwords = gBCCSpiStruct.nwords;
	}
	else
	{
		/* Set up the transfer profile */
		lvSeq.dev = SPIDEV_ID(gSpiStruct.devtype, gSpiStruct.csn);
		lvSeq.mode = gSpiStruct.mode;
		lvSeq.nbits = gSpiStruct.width;
		lvSeq.frequency = gSpiStruct.freq;
		lvSeq.ntrans = 1; 				
		lvSeq.trans = &lvTrans;

		lvTrans.deselect = false;//true; 	/* De-select after transfer */
		lvTrans.delay = gSpiStruct.udelay;
		lvTrans.nwords = gSpiStruct.nwords;
	}

	// the transmit buffers
	lvTrans.txbuffer = txdata;
	lvTrans.rxbuffer = rxdata;

	// transfer on SPI
	lvRetValue = ioctl(lvFd, SPIIOC_TRANSFER, &lvSeq);//spidev_transfer(lvFd, &lvSeq);
	if(lvRetValue)
	{
		cli_printfError("SPI ERROR: failed to transfer! error: %d\n", lvRetValue);
	}

	// close the device
	close(lvFd);

	// return to the user
	return lvRetValue;
}

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
int spi_configure(spiStruct_s newSpiConfiguration, bool BCCconfiguration)
{
	int lvRetValue = -1;

	// check which configuration needs to be written
	if(BCCconfiguration)
	{
		// set the new struct
		gBCCSpiStruct = newSpiConfiguration;
	}
	else
	{
		// set the new struct
		gSpiStruct = newSpiConfiguration;
	}

	lvRetValue = 0;

	// return 
	return lvRetValue;
}
/****************************************************************************
 * private Functions
 ****************************************************************************/

// insert the number of the SPI bus, will be 0 or 1 for the rddrone-bms772
// it will return the devpath "/dev/spi<bus>"
FAR char *spi_path(int bus)
{
	// make the device path
  	snprintf(gDevname, DEVNAME_FMTLEN, DEVNAME_FMT, bus);

  	// return it
  	return gDevname;
}

// this function will open de spi device with O_RDONLY 
int spi_open(int bus)
{
 	FAR char *devpath;

 	// Get the device path 
 	devpath = spi_path(bus);

 	// Open the file for read-only access (we need only IOCTLs)
 	return open(devpath, O_RDONLY);
}

// this function is used to transfer on the SPI bus
// it will use the ioctl function with SPIIOC_TRANSFER
int spi_ioctlTransfer(int fd, FAR struct spi_sequence_s *seq)
{
   	//Perform the IOCTL 
  	return ioctl(fd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)seq));
}


//#endif
