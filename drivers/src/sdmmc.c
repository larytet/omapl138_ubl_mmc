/*
 * sdmmc.c
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
  FILE      : sdmmc.c
  PROJECT   : TI Boot and Flash Utilities
  AUTHOR    : Daniel Allred
  DESC      : Generic SDMMC peripheral driver
-------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// Device specific CSL
#include "device.h"

// Util functions
#include "util.h"

// This module's header
#include "sdmmc.h"
#include "device_sdmmc.h"
#include "debug.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern Uint32 DEVICE_SDMMC_baseAddr[];


/************************************************************
* Local Macro Declarations                                  *
************************************************************/

#define SDMMC_BLOCK_LENGTH     (512)


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

#ifdef USE_IN_ROM
SDMMC_InfoObj gSDMMCInfo;
#endif

// Default SPI Config strcuture
const SDMMC_ConfigObj DEFAULT_SDMMC_CONFIG = 
{
  SDMMC_LITTLE_ENDIAN,              // writeEndian
  SDMMC_LITTLE_ENDIAN,              // readEndian
  SDMMC_DAT3_EDGE_DETECT_DISABLE,   // dat3Detect
  19,                               // initModeClockRate (target 300 KHz with 12 MHz input to module)
  3,                                // dataModeClockRate (target 6 MHz with 12 MHz input to module)
  SDMMC_8BIT_DATABUS,               // busWidth
  (Uint16)0xFF,                     // timeoutResponse
  0x03FFFFFF,                       // timeoutData
  SDMMC_FIFO_LEVEL_HIGH             // fifoThreshold
};

SDMMC_ConfigHandle const hDEFAULT_SDMMC_CONFIG = (SDMMC_ConfigHandle) &DEFAULT_SDMMC_CONFIG;


/************************************************************
* Global Function Definitions                               *
************************************************************/

extern Uint32 DEVICE_SDMMCInit(Uint8 periphNum);

// Initialze SDMMC interface
SDMMC_InfoHandle SDMMC_open(Uint32 sdmmcPeripheralNum, SDMMC_ConfigHandle hSDMMCCfg)
{
  DEVICE_SDMMCRegs *SDMMC;
  SDMMC_InfoHandle hSDMMCInfo;
  Uint32 mmcctl = 0;
  
  // Do device level init (pinmux, power domain, etc.)
  if (DEVICE_SDMMCInit(sdmmcPeripheralNum) != E_PASS)
  {
    return NULL;
  }

  // Set SDMMCInfo handle
#ifdef USE_IN_ROM
  hSDMMCInfo = (SDMMC_InfoHandle) &gSDMMCInfo;
#else
  hSDMMCInfo = (SDMMC_InfoHandle) UTIL_allocMem(sizeof(SDMMC_InfoObj));
#endif

  // Assign the correct register base
  hSDMMCInfo->regs = (void *) DEVICE_SDMMC_baseAddr[sdmmcPeripheralNum];
  SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;

  if (hSDMMCCfg == NULL)
  {
    hSDMMCInfo->hSDMMCCfg = hDEFAULT_SDMMC_CONFIG;
  }
  else
  {
    hSDMMCInfo->hSDMMCCfg = hSDMMCCfg;
  }
  
  // Set up MMCCTL register

  // Put SDMMC peripheral in reset
  SDMMC->MMCCTL = (SDMMC_MMCCTL_DATRST_MASK & (1u << SDMMC_MMCCTL_DATRST_SHIFT)) |
                  (SDMMC_MMCCTL_CMDRST_MASK & (1u << SDMMC_MMCCTL_CMDRST_SHIFT));

  // Set endianness for reads and writes
  if (hSDMMCInfo->hSDMMCCfg->writeEndian == SDMMC_BIG_ENDIAN)
  {
    mmcctl |= (SDMMC_MMCCTL_PERMDX_MASK & (1u <<SDMMC_MMCCTL_PERMDX_SHIFT));
  }
  if (hSDMMCInfo->hSDMMCCfg->readEndian == SDMMC_BIG_ENDIAN)
  {
    mmcctl |= (SDMMC_MMCCTL_PERMDR_MASK & (1u <<SDMMC_MMCCTL_PERMDR_SHIFT));
  }
  // Set DAT3 edge detection
  mmcctl |= (SDMMC_MMCCTL_DATEG_MASK & (hSDMMCInfo->hSDMMCCfg->dat3Detect << SDMMC_MMCCTL_DATEG_SHIFT));
  
  // Set the data buswidth
  mmcctl |= (SDMMC_MMCCTL_WIDTH0_MASK & ((hSDMMCInfo->hSDMMCCfg->busWidth & 0x1) << SDMMC_MMCCTL_WIDTH0_SHIFT));
  mmcctl |= (SDMMC_MMCCTL_WIDTH1_MASK & ((hSDMMCInfo->hSDMMCCfg->busWidth & 0x2) << SDMMC_MMCCTL_WIDTH1_SHIFT));
  
  // Apply register changes
  SDMMC->MMCCTL = mmcctl;
  
  // Disable all interrupts
  SDMMC->MMCIM = 0x00000000;
  
  // Set timeouts
  SDMMC_setResponseTimeout(hSDMMCInfo, hSDMMCInfo->hSDMMCCfg->timeoutResponse);
  SDMMC_setDataTimeout(hSDMMCInfo, hSDMMCInfo->hSDMMCCfg->timeoutData);
  
  // Set and enable the clock
  SDMMC_setClockRate(hSDMMCInfo, hSDMMCInfo->hSDMMCCfg->initModeClockRate);
  
  // Set the block size
  SDMMC_setBlockSize(hSDMMCInfo, SDMMC_BLOCK_LENGTH);
  
  // Set FIFO level
  SDMMC_setFifoLevel(hSDMMCInfo, hSDMMCInfo->hSDMMCCfg->fifoThreshold);
  
  return hSDMMCInfo;
}

void SDMMC_setResponseTimeout(SDMMC_InfoHandle hSDMMCInfo, Uint32 timeout)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  
  SDMMC->MMCTOR &= (~SDMMC_MMCTOR_TOR_MASK);
  SDMMC->MMCTOR |= (SDMMC_MMCTOR_TOR_MASK & (timeout << SDMMC_MMCTOR_TOR_SHIFT));
}

void SDMMC_setDataTimeout(SDMMC_InfoHandle hSDMMCInfo, Uint32 timeout)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  
  SDMMC->MMCTOR &= (~SDMMC_MMCTOR_TOD_25_16_MASK);
  SDMMC->MMCTOR |= (SDMMC_MMCTOR_TOD_25_16_MASK & ((timeout>>16) << SDMMC_MMCTOR_TOD_25_16_SHIFT));
  SDMMC->MMCTOD  = (SDMMC_MMCTOD_TOD_15_0_MASK  & (timeout << SDMMC_MMCTOD_TOD_15_0_SHIFT));
}

void SDMMC_setBlockSize(SDMMC_InfoHandle hSDMMCInfo, Uint16 size)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  Uint32 localDataBytesPerBlk = 0;  
  
  if (size == 0)
  {
    return;
  }
  
  hSDMMCInfo->dataBytesPerBlk = size;

  localDataBytesPerBlk = hSDMMCInfo->dataBytesPerBlk;
  hSDMMCInfo->dataBytesPerBlkPower2 = 0;
  while(localDataBytesPerBlk > 1)
  {
    localDataBytesPerBlk >>= 1;
    hSDMMCInfo->dataBytesPerBlkPower2++;
  }
  
  SDMMC->MMCBLEN = size & 0x0FFF;
}

void SDMMC_setBlockCount(SDMMC_InfoHandle hSDMMCInfo, Uint16 numBlks)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  SDMMC->MMCNBLK = numBlks;
}

void SDMMC_setDataWidth(SDMMC_InfoHandle hSDMMCInfo, SDMMC_DatabusWidth width)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  Uint32 mmcctl = SDMMC->MMCCTL & ~(SDMMC_MMCCTL_WIDTH0_MASK | SDMMC_MMCCTL_WIDTH1_MASK);
  
  // Put SDMMC peripheral in reset  
  SDMMC->MMCCTL = (SDMMC_MMCCTL_DATRST_MASK & (1u << SDMMC_MMCCTL_DATRST_SHIFT)) |
                  (SDMMC_MMCCTL_CMDRST_MASK & (1u << SDMMC_MMCCTL_CMDRST_SHIFT));
                      
  // Set the data buswidth
  mmcctl |= (SDMMC_MMCCTL_WIDTH0_MASK & ((width & 0x1) << SDMMC_MMCCTL_WIDTH0_SHIFT));
  mmcctl |= (SDMMC_MMCCTL_WIDTH1_MASK & ((width & 0x2) << SDMMC_MMCCTL_WIDTH1_SHIFT));
  
  // Clear the reset bits
  mmcctl &= ~(SDMMC_MMCCTL_DATRST_MASK | SDMMC_MMCCTL_CMDRST_MASK);
    
  // Apply register changes and take out of reset
  SDMMC->MMCCTL = mmcctl;
}

void SDMMC_setClockRate(SDMMC_InfoHandle hSDMMCInfo, Uint8 rate)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;

  // Disable the clock pin
  SDMMC->MMCCLK &= ~(SDMMC_MMCCLK_CLKEN_MASK);
  
  // Change the divider and re-enable the clock pin
  SDMMC->MMCCLK = (SDMMC_MMCCLK_CLKEN_MASK & (1u << SDMMC_MMCCLK_CLKEN_SHIFT)) |
                  (SDMMC_MMCCLK_CLKRT_MASK & (rate << SDMMC_MMCCLK_CLKRT_SHIFT));

  // Wait for clock to stabilize  
  UTIL_waitLoop(10000);
}

// Assumes the block size has been set prior to calling this
void SDMMC_setFifoLevel(SDMMC_InfoHandle hSDMMCInfo, SDMMC_FifoThreshold level)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  
  if (SDMMC_FIFO_LEVEL_LOW == level)
  {
    hSDMMCInfo->dataBytesPerOp = DEVICE_SDMMC_FIFO_LEVEL_LOW_COUNT;
    hSDMMCInfo->dataBytesPerOpPower2 = DEVICE_SDMMC_FIFO_LEVEL_LOW_SHIFT;
    hSDMMCInfo->numOpsPerBlk = hSDMMCInfo->dataBytesPerBlk >> DEVICE_SDMMC_FIFO_LEVEL_LOW_SHIFT;
  }
  else if (SDMMC_FIFO_LEVEL_HIGH == level)
  {
    hSDMMCInfo->dataBytesPerOp = DEVICE_SDMMC_FIFO_LEVEL_HIGH_COUNT;
    hSDMMCInfo->dataBytesPerOpPower2 = DEVICE_SDMMC_FIFO_LEVEL_HIGH_SHIFT;
    hSDMMCInfo->numOpsPerBlk = hSDMMCInfo->dataBytesPerBlk >> DEVICE_SDMMC_FIFO_LEVEL_HIGH_SHIFT;
  }
  
  // Clear and then set FIFO level bit
  SDMMC->MMCFIFOCTL &= (~SDMMC_MMCFIFOCTL_FIFOLEV_MASK);
  if (level == SDMMC_FIFO_LEVEL_HIGH)
  {
    SDMMC->MMCFIFOCTL |= (SDMMC_MMCFIFOCTL_FIFOLEV_MASK & (1u << SDMMC_MMCFIFOCTL_FIFOLEV_SHIFT));
  }
}

// Set FIFO access direction
void SDMMC_setFifoDirection(SDMMC_InfoHandle hSDMMCInfo, SDMMC_FifoDirection direction)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  
  // Reset the FIFO
  SDMMC->MMCFIFOCTL |= (SDMMC_MMCFIFOCTL_FIFORST_MASK & (0x1 << SDMMC_MMCFIFOCTL_FIFORST_SHIFT)); 
  
  UTIL_waitLoop(10);

  // Set the transfer direction for the FIFO
  if (SDMMC_FIFO_DIRECTION_TX == direction)
  {
    SDMMC->MMCFIFOCTL |= SDMMC_MMCFIFOCTL_FIFODIR_MASK;
  }
  else if (SDMMC_FIFO_DIRECTION_RX == direction)
  {
    SDMMC->MMCFIFOCTL &= ~(SDMMC_MMCFIFOCTL_FIFODIR_MASK);
  }
}

// clear the response
void SDMMC_clearResponse(SDMMC_InfoHandle hSDMMCInfo)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;

  SDMMC->MMCRSP01 = 0x0;
  SDMMC->MMCRSP23 = 0x0;
  SDMMC->MMCRSP45 = 0x0;
  SDMMC->MMCRSP67 = 0x0;
  SDMMC->MMCCIDX &= 0xFFC0;
}

// Get SDMMC response
void SDMMC_getResponse(SDMMC_InfoHandle hSDMMCInfo, SDMMC_ResponseHandle hResponse)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  SDMMC_ResponseHandle hMyResponse;
  
  hMyResponse = (hResponse == NULL) ? &(hSDMMCInfo->response) : hResponse; 
  
  hMyResponse->responseData[0]    = SDMMC->MMCRSP01;
  hMyResponse->responseData[1]    = SDMMC->MMCRSP23;
  hMyResponse->responseData[2]    = SDMMC->MMCRSP45;
  hMyResponse->responseData[3]    = SDMMC->MMCRSP67;
  
  hMyResponse->commandIndex       = (SDMMC->MMCCIDX  & 0x003F);
  
  SDMMC_clearResponse(hSDMMCInfo);
}

// Send a command to a connected device
Uint32 SDMMC_sendCmd(SDMMC_InfoHandle hSDMMCInfo, Uint32 command, Uint32 argument, Uint32 timeOut)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  Uint32 statusRegBits = 0x0;
  volatile Uint32 dummyread;

  // Dummy read
  dummyread = SDMMC->MMCST0;

  // Clear response registers
  SDMMC_clearResponse( hSDMMCInfo );

  // Setup the Argument Register and send CMD
  SDMMC->MMCARGHL = argument;
  SDMMC->MMCCMD   = command;
  
  // Delay loop allowing cards time to respond
  UTIL_waitLoop(1000);
        
  // If a software timeout is given then we should wait checking for status
  if (timeOut > 0)
  {
    /* Wait for RspDne; exit on RspTimeOut or RspCRCErr
     bit 2 - RSPDNE
     bit 3 - 7 Error/timeout */
    while( ((statusRegBits & (0x00FC)) == 0) && (timeOut > 0))
    {
      statusRegBits = SDMMC->MMCST0;
      --timeOut;
    }
    
    // We got a software timeout
    if (timeOut == 0)
    {
      return E_TIMEOUT;
    }
    
    if ((statusRegBits & SDMMC_MMCST0_RSPDNE_MASK) == SDMMC_MMCST0_RSPDNE_MASK)
    {
      return E_PASS;
    }
    else
    {
      return statusRegBits;
    }
  }

  return E_PASS;
}


Uint32 SDMMC_readNWords(SDMMC_InfoHandle hSDMMCInfo, Uint32 *data, Uint32 numofBytes)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  Uint16 i=0,j=0;
  Uint32 status;
  Uint32 fifoReadItrCount, wordItrCount;

  // Setup counters for fifo iterations and word iterations
  fifoReadItrCount= numofBytes >> hSDMMCInfo->dataBytesPerOpPower2;
  wordItrCount = hSDMMCInfo->dataBytesPerOp >> 2;

  for(i=0; i<fifoReadItrCount; i++)
  {
    // Wait for DRRDY or some kind of error/timeout
    do
    {
      status=SDMMC->MMCST0;
      if (status & (SDMMC_MMCST0_CRCRD_MASK | SDMMC_MMCST0_TOUTRD_MASK))
      {
        return E_FAIL;
      }
    }
    while( (status & SDMMC_MMCST0_DRRDY_MASK) == 0 );

    // Read the data, one word at a time
    for(j=0; j<wordItrCount; j++)
    {
      *data++ = SDMMC->MMCDRR;
    }
  }
  return E_PASS;
}

#ifndef USE_IN_ROM
Uint32 SDMMC_writeNWords(SDMMC_InfoHandle hSDMMCInfo, Uint32 *data, Uint32 numofBytes)
{
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) hSDMMCInfo->regs;
  Uint16 i=0,j=0;
  Uint32 status;
  Uint32 fifoReadItrCount, wordItrCount;

  // Setup counters for fifo iterations and word iterations
  fifoReadItrCount= numofBytes >> hSDMMCInfo->dataBytesPerOpPower2;
  wordItrCount = hSDMMCInfo->dataBytesPerOp >> 2;

  for(i=0; i < fifoReadItrCount;i++)
  {
    // Write the data, one word at a time
    for(j=0; j<wordItrCount; j++)
    {
      SDMMC->MMCDXR= (Uint32)*data++;
    }

    if(i != (fifoReadItrCount-1))
    {
      // Wait for DXRDY timeout
      do
      {
        status=SDMMC->MMCST0;
        if (status & 0xf8)
        {
          return (E_FAIL);
        }
      }
      while( (status & SDMMC_MMCST0_DXRDY_MASK) == 0 );
    }
  }
  return E_PASS;
}
#endif



/************************************************************
* Local Function Definitions                                *
************************************************************/


/***********************************************************
* End file                                                 *
***********************************************************/

