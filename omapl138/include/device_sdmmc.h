/*
 * device_sdmmc.h
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
  FILE        : device_sdmmc.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Provides device differentiation for the project files. This
                file MUST be modified to match the device specifics.
----------------------------------------------------------------------------- */

#ifndef _DEVICE_SDMMC_H_
#define _DEVICE_SDMMC_H_

#include "tistdtypes.h"
#include "sdmmc.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/***********************************************************
* Global Macro Declarations                                *
***********************************************************/

#define DEVICE_SDMMC_TIMEOUT            (10240)
#define DEVICE_SDMMCBOOT_PERIPHNUM          (0)

// Define the size of the FIFO for high and low levels
#define DEVICE_SDMMC_FIFO_LEVEL_LOW_COUNT     (32)
#define DEVICE_SDMMC_FIFO_LEVEL_LOW_SHIFT     (5)
#define DEVICE_SDMMC_FIFO_LEVEL_HIGH_COUNT    (64)
#define DEVICE_SDMMC_FIFO_LEVEL_HIGH_SHIFT    (6)

// Defines which SDMMC blocks the RBL will search in for a UBL image
#define DEVICE_SDMMC_RBL_SEARCH_START_BLOCK     (1)
#define DEVICE_SDMMC_RBL_SEARCH_END_BLOCK       (24)

// Defines which SDMMC blocks are valid for writing the APP data
#define DEVICE_SDMMC_UBL_SEARCH_START_BLOCK     (48)
#define DEVICE_SDMMC_UBL_SEARCH_END_BLOCK       (100)

// Used by UBL when doing UART boot, UBL Nor Boot, NAND boot or MMC_SD boot
#define UBL_MAGIC_BIN_IMG           (0xA1ACED66)		/* Execute in place supported*/



/***********************************************************
* Global Typedef Declarations                              *
***********************************************************/
typedef struct
{
  Uint32 magicNum;    // Expected magic number
  Uint32 entryPoint;  // Entry point of the user boot loader (should be in 0x2000-0x3FFF region
  Uint32 numBlock;	  // Number of blocks where User boot loader is stored
  Uint32 startBlock;  // starting block number where User boot loader is stored
  Uint32 ldAddress;   // Starting RAM address where image is to copied - XIP Mode
}
SDMMC_Boot;


/***********************************************************
* Global Variable Declarations                             *
***********************************************************/

extern __FAR__ SDMMC_ConfigHandle const hDEVICE_SDMMC_config;


/***********************************************************
* Global Function Declarations                             *
***********************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // End _DEVICE_SDMMC_H_

