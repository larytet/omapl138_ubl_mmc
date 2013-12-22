/*
 * spi_memboot.c
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
  FILE        : spi_memboot.c                                                   
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Module to boot the from an SPI flash device by finding the
                application (usually U-boot) and loading it to RAM.
----------------------------------------------------------------------------- */

#ifdef UBL_SPI_MEM

// General type include
#include "tistdtypes.h"
// Debug I/O module
#include "debug.h"

// Device specific functions
#include "device.h"

// Misc utility module
#include "util.h"

// Main UBL module
#include "ubl.h"

// SPI driver functions
#include "spi.h"
#include "spi_mem.h"

// Device specific SPI info
#include "device_spi.h"

// This module's header file
#include "spi_memboot.h"

/************************************************************
* Explicit External Declarations                            *
************************************************************/

// Entrypoint for application we are getting out of flash
extern Uint32 gEntryPoint;


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

// Structure for holding details about image stored in SPI memory
volatile SPI_MEM_BOOT_HeaderObj  gSpiMemBoot;


/************************************************************
* Global Function Definitions                               *
************************************************************/

// Function to find out where the application is and copy to RAM
Uint32 SPI_MEM_BOOT_copy()
{
  SPI_MEM_InfoHandle hSpiMemInfo;
  
  Uint32 currMemAddr = 0;

  DEBUG_printString("Starting SPI Memory Copy...\r\n");
  
  // Do device specific init for SPI 
  DEVICE_SPIInit(DEVICE_SPIBOOT_PERIPHNUM);
  
  // SPI Memory Initialization
  hSpiMemInfo = SPI_MEM_open(DEVICE_SPIBOOT_PERIPHNUM, DEVICE_SPIBOOT_CSNUM, hDEVICE_SPI_config);
  if (hSpiMemInfo == NULL)
    return E_FAIL;
    
  // Read data about Application starting at start of memory and searching
  // at the start of each memory block
  while (currMemAddr < hSpiMemInfo->hMemParams->memorySize)
  {
    currMemAddr += hSpiMemInfo->hMemParams->blockSize;
  
    SPI_MEM_readBytes(hSpiMemInfo, currMemAddr, sizeof(SPI_MEM_BOOT_HeaderObj), (Uint8 *) &gSpiMemBoot);
  
    if (gSpiMemBoot.magicNum == UBL_MAGIC_BINARY_BOOT)
    {
      // Valid magic number found
      DEBUG_printString("Valid magicnum, ");
      DEBUG_printHexInt(gSpiMemBoot.magicNum);
      DEBUG_printString(", found at offset ");
      DEBUG_printHexInt(currMemAddr);
      DEBUG_printString(".\r\n");
      break;
    } 
  }

  if (currMemAddr >= hSpiMemInfo->hMemParams->memorySize)
  {
    DEBUG_printString("No magic number found.\r\n");
	DEBUG_printString("Looking for: ");
      DEBUG_printHexInt(UBL_MAGIC_BINARY_BOOT);
      DEBUG_printString(", but read ");
      DEBUG_printHexInt(gSpiMemBoot.magicNum);
      DEBUG_printString(".\r\n");
    return E_FAIL;
  }
  
  if (SPI_MEM_readBytes(hSpiMemInfo, gSpiMemBoot.memAddress, gSpiMemBoot.appSize, (Uint8 *)gSpiMemBoot.ldAddress) != E_PASS)
  {
    DEBUG_printString("Application image reading failed.\r\n");
    return E_FAIL;
  }
  
  // Application was read correctly, so set entrypoint
  gEntryPoint = gSpiMemBoot.entryPoint;

  return E_PASS;
}

/************************************************************
* Local Function Definitions                                *
************************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/
#endif  // #ifdef UBL_SPI_MEM
