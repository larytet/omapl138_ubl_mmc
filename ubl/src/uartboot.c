/*
 * uartboot.c
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/* --------------------------------------------------------------------------
  FILE        : uartboot.c                                                   
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Implementation of the UART boot mode for the UBL
 ----------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

#include "device_sdmmc.h"
#include "sdmmc_mem.h"
// UART driver
#include "uart.h"

// Misc. utility function include
#include "util.h"

// Project specific debug functionality
#include "debug.h"

// Main UBL module
#include "ubl.h"

// Device module
#include "device.h"

// This module's header file
#include "uartboot.h"

/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern __FAR__ Uint32 DDR_SIZE, DDR_START;


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

void cpywords(Uint32* dest, Uint32* src, int words);

/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/


/************************************************************
* Global Function Definitions                               *
************************************************************/

Uint32 UARTBOOT_copy(UART_InfoHandle hUartInfo)
{
	Uint32 status = E_FAIL;
	Uint32 descriptor[5];
	Uint32 magicNum;
	SDMMC_Boot sdMMCBootDesc;
	SDMMC_MEM_InfoHandle hSDMMCMemInfo;
	Uint32 dataBytesPerBlk;
	Uint32 len, bytes2read;

	do
	{
		hSDMMCMemInfo = SDMMC_MEM_open(0,NULL);
		if (hSDMMCMemInfo == NULL)
		{
			DEBUG_printString("Failed to initialize MMC\r\n");
		    break;
		}
		dataBytesPerBlk = hSDMMCMemInfo->hSDMMCInfo->dataBytesPerBlk;

		// read from the UART first 5 words
		len = sizeof(descriptor);
		status = UART_recvStringN(hUartInfo, (char*)descriptor, &len, 0);
		if ((status == E_FAIL) ||(len != sizeof(descriptor)))
		{
			DEBUG_printString("Failed to read descriptor\r\n");
			break;
		}

		magicNum = descriptor[0];
		if((magicNum & 0xFFFFFF00) != (UBL_MAGIC_BIN_IMG & 0xFFFFFF00))
		{
			DEBUG_printString("Magic number failed\r\n");
			break;
		}

		sdMMCBootDesc.magicNum = magicNum;
		sdMMCBootDesc.entryPoint = descriptor[1];
		sdMMCBootDesc.numBlock = descriptor[2];
		sdMMCBootDesc.startBlock = descriptor[3];
		sdMMCBootDesc.ldAddress = descriptor[4];

		DEBUG_printString("Got descriptor\r\n");

		bytes2read = dataBytesPerBlk*sdMMCBootDesc.numBlock;
		len = bytes2read;
		status = UART_recvStringN(hUartInfo, ((char*)&DDR_START), &len, 0);
		if ((status == E_FAIL) || (len != bytes2read))
		{
			DEBUG_printString("Failed to read data to the external RAM\r\n");
			break;
		}

		// all the data is in the RAM - program the MMC
		status = SDMMC_MEM_writeBytes (hSDMMCMemInfo, dataBytesPerBlk*sdMMCBootDesc.startBlock, bytes2read, (Uint8*)&DDR_START);
		if (status == E_FAIL)
		{
			DEBUG_printString("Failed to write MMC\r\n");
			break;
		}


		status = E_PASS;

	}
	while (0);


	return status;
}


/************************************************************
* Local Function Definitions                                *
************************************************************/

void cpywords(Uint32* dst, Uint32* src, int words)
{
	int i;
	for (i = 0;i < words;i++)
	{
		*dst = *src;
		dst++;
		src++;
	}

}


/************************************************************
* End file                                                  *
************************************************************/


