/*
 * device_sdmmc.c
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
    FILE        : device_sdmmc.c 				                             	 	        
    PROJECT     : TI Booting and Flashing Utilities
    AUTHOR      : Daniel Allred
    DESC        : This file descibes and implements various device-specific NAND
                  functionality.
-------------------------------------------------------------------------- */ 

// General type include
#include "tistdtypes.h"

// Device specific CSL
#include "device.h"

// Device specific SDMMC details
#include "device_sdmmc.h"

// Generic SDMMC header file
#include "sdmmc.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
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

const Uint32 DEVICE_SDMMC_baseAddr[SDMMC_PERIPHERAL_CNT] =
{
  (Uint32) SDMMC0,
  (Uint32) SDMMC1
};

const SDMMC_ConfigObj DEVICE_SDMMC_config = 
{
  SDMMC_LITTLE_ENDIAN,              // writeEndian
  SDMMC_LITTLE_ENDIAN,              // readEndian
  SDMMC_DAT3_EDGE_DETECT_DISABLE,   // dat3Detect
#if (1)
  249,                              // initModeClockRate (target 300 KHz with 150 MHz input to module)
  7,                                // dataModeClockRate (target 10 MHz with 150 MHz input to module)  
#else
  19,                               // initModeClockRate (target 300 KHz with 12 MHz input to module)
  0,                                // dataModeClockRate (target 3 MHz with 6 MHz input to module) 
#endif  
  SDMMC_4BIT_DATABUS,               // busWidth
  0xFF,                             // timeoutResponse (none)
  0x0000FFFF,                       // timeoutData
  SDMMC_FIFO_LEVEL_HIGH             // fifoThreshold
};

// Set SDMMC config to NULL to use SDMMC driver defaults
//SDMMC_ConfigHandle const hDEVICE_SDMMC_config = NULL;
SDMMC_ConfigHandle const hDEVICE_SDMMC_config = (SDMMC_ConfigHandle) &DEVICE_SDMMC_config;


/************************************************************
* Global Function Definitions                               *
************************************************************/


/************************************************************
* Local Function Definitions                                *
************************************************************/


/************************************************************
* End file                                                  *
************************************************************/


