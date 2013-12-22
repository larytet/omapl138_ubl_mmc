/*
 * sdmmcboot.c
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
  FILE        : sdmmcboot.c
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Module to boot the from the SD/MMC peripheral by finding the
                application (usually U-boot) and loading it to RAM.
----------------------------------------------------------------------------- */

#ifdef UBL_SDMMC

// General type include
#include "tistdtypes.h"

// Debug I/O module
#include "debug.h"

// Misc utility module
#include "util.h"

// Main UBL module
#include "ubl.h"

// SD/MMC driver functions
#include "sdmmc.h"
#include "sdmmc_mem.h"

// This module's header file
#include "sdmmcboot.h"

// Device specific file
#include "device_sdmmc.h"

/************************************************************
* Explicit External Declarations                            *
************************************************************/

// Entrypoint for application we are decoding out of flash
extern Uint32 gEntryPoint;
extern __FAR__ Uint32 EXTERNAL_RAM_SIZE, EXTERNAL_RAM_START, EXTERNAL_RAM_END;
extern __FAR__ Uint32 INTERNAL_RAM_SIZE, INTERNAL_RAM_START, INTERNAL_RAM_END;
extern __FAR__ Uint32 STACK_START, STACK_SIZE;

/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

// structure for holding details about UBL stored in SDMMC
volatile SDMMCBOOT_HeaderObj  gSDMMCBoot;
Uint8 SDMMCBOOT(SDMMC_MEM_InfoHandle hSDMMCMemInfo);

/************************************************************
* Global Function Definitions                               *
************************************************************/

// Function to find out where the application is and copy to RAM
Uint32 SDMMCBOOT_copy()
{
  SDMMC_MEM_InfoHandle hSDMMCMemInfo;

  DEBUG_printString("Starting SDMMC Copy...\r\n");
  
  // SDMMC Initialization
  hSDMMCMemInfo = SDMMC_MEM_open(0,NULL);

  if (hSDMMCMemInfo == NULL)
  {
    DEBUG_printString("Failed to initialize MMC\r\n");
    return E_FAIL;
  }

   if( SDMMCBOOT(hSDMMCMemInfo) != E_PASS)
	   return E_FAIL;

  return E_PASS;
}

void cpywords(Uint32* dst, Uint32* src, int words);

Uint8 SDMMCBOOT(SDMMC_MEM_InfoHandle hSDMMCMemInfo)
{	Uint32 count,blockNum = DEVICE_SDMMC_UBL_SEARCH_START_BLOCK;
	Uint32 magicNum,magicNumFound;   
	Uint8 *rxBuf;		/* receive buffer where the data read from NAND will be stored */
	Uint32 readError = E_FAIL;
	Uint8 retry = 0;
	Uint32 dataBytesPerBlk = hSDMMCMemInfo->hSDMMCInfo->dataBytesPerBlk;
	Uint32 command;
	Uint32 ldAddress;
	Uint32 dataLength;
	Uint32 commandOffset;
	void *memPtr;

	// Allocate memory for one block
	memPtr = UTIL_getCurrMemPtr();
	rxBuf = (Uint8*)UTIL_allocMem(dataBytesPerBlk);

  
SDMMC_tryAgain:

	retry++;	
    // if retry value is > 2, MMCSD boot fails and starts USB boot mode 
	if(retry > 4)
	{
	  DEBUG_printString("Failed to find magic number\r\n");
	  return E_FAIL;
	}

	/* read data about UBL from the block 1(to MMCSD_TRY_BLOCK_NUM+1)*/
	magicNumFound = 0;
	for(count=blockNum; count <= DEVICE_SDMMC_UBL_SEARCH_END_BLOCK; count++) {
		// reading 512 though only 32 is used
		readError = SDMMC_MEM_readBytes(hSDMMCMemInfo, count * dataBytesPerBlk, dataBytesPerBlk, &rxBuf[0]);
		/* Read Error has occured */
		if(readError != E_PASS)
			continue;

		magicNum = *(((Uint32 *)rxBuf));

		/* Magic number found */
		if(magicNum == 0x41504954)
		{
			// next block
			blockNum = count+1;
			magicNumFound = 1;
			break;
		}
	}
	
	if(readError != E_PASS)
		goto SDMMC_tryAgain; /* MMC/SD boot failed.. Retry */

	/* When readError == E_PASS, check if magicNum is found */
	if(magicNumFound == 0) {
		goto SDMMC_tryAgain; /* MMC/SD boot failed.. Retry */
	}

	// Simple parser of AIS script - I ignore everything until SECTION_LOAD command
	commandOffset = 4;
	while (1)
	{
		command = *(((Uint32 *)(&rxBuf[commandOffset])));
		commandOffset += 4;
		if (command == 0x58535907)
		{
			commandOffset += 4*4;
			DEBUG_printString("SET skipped\r\n");
		}
		else if (command == 0x5853590D)
		{
			Uint32 temp;
			temp = *(((Uint32 *)(&rxBuf[commandOffset])));
			commandOffset += 4+((temp >> 16) & 0xFFFF)*4;
			DEBUG_printString("FUNCTIONEXEC skipped\r\n");
		}
		else if (command == 0x58535901)
		{
			DEBUG_printString("SECTION_LOAD found\r\n");
			break;
		}
		else
		{
			DEBUG_printString("Not supported AIS command\r\n");
			goto SDMMC_tryAgain;
		}
	}

	// I ignore last JUMP_CLOSE command and any other command after SECTION_LOAD
    // I assume that ldAddress == entryPoint

	ldAddress = *(((Uint32 *)(&rxBuf[commandOffset])));
	commandOffset += 4;
	dataLength = *(((Uint32 *)(&rxBuf[commandOffset])));
	commandOffset += 4;

	// copy data from the first read MMC block to the ldAddress
    cpywords((Uint32*)ldAddress, (Uint32*)(rxBuf+commandOffset), (dataBytesPerBlk-commandOffset)/4);

    // free memory
    UTIL_setCurrMemPtr(memPtr);

    // Set the copy location to final run location
    rxBuf = (Uint8 *)ldAddress + (dataBytesPerBlk-commandOffset);
    
    readError = SDMMC_MEM_readBytes(hSDMMCMemInfo, (blockNum*dataBytesPerBlk), dataLength-(dataBytesPerBlk-commandOffset), &rxBuf[0]);
	if(readError != E_PASS)
		goto SDMMC_tryAgain; /* MMC/SD boot failed.. Retry */
		
	// Application was read correctly, so set entrypoint
	gEntryPoint = ldAddress;
    return E_PASS;
}

/************************************************************
* Local Function Definitions                                *
************************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/
#endif  // #ifdef UBL_SDMMC

