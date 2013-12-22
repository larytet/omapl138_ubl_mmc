/*
 * sdmmc_mem.c
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
  FILE      : sdmmc_mem.c
  PROJECT   : TI Boot and Flash Utilities
  AUTHOR    : Daniel Allred
  DESC      : Generic SDMMC memory driver
-------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// Device specific register definitions
#include "device.h"

// Util functions
#include "util.h"

// The generic SD/MMC module driver 
#include "sdmmc.h"

// Platform/device specific SDMMC info
#include "device_sdmmc.h"

// This module's header
#include "sdmmc_mem.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

static Uint32 LOCAL_detectSDCard(SDMMC_MEM_InfoHandle hSDMMCMemInfo);
static Uint32 LOCAL_detectMMC(SDMMC_MEM_InfoHandle hSDMMCMemInfo);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

#ifdef USE_IN_ROM
  SDMMC_MEM_InfoObj     gSDMMCMemInfo;
#endif


/************************************************************
* Global Function Definitions                               *
************************************************************/

// Initialze SDMMC interface and find the details of the SD/MMC card used
SDMMC_MEM_InfoHandle SDMMC_MEM_open(Uint32 sdmmcPeripheralNum, SDMMC_ConfigHandle hSDMMCCfg)
{
  SDMMC_MEM_InfoHandle hSDMMCMemInfo = NULL;

  // Set SDMMC_MEM_Info handle
#ifdef USE_IN_ROM
  hSDMMCMemInfo = (SDMMC_MEM_InfoHandle) &gSDMMCMemInfo;
#else
  hSDMMCMemInfo = (SDMMC_MEM_InfoHandle) UTIL_allocMem(sizeof(SDMMC_MEM_InfoObj));
#endif

  // Open SDMMC peripheral
  hSDMMCMemInfo->hSDMMCInfo = SDMMC_open
  (
    sdmmcPeripheralNum,
    hSDMMCCfg
  );

  // Try to open memory device as SD card
  if (E_PASS != LOCAL_detectSDCard(hSDMMCMemInfo))
  {
    // That didn't work, so try it as MMC
    if (E_PASS != LOCAL_detectMMC(hSDMMCMemInfo))
    {
      // That didn't work either, so we return NULL as we don't appear to
      // have a valid SDMMC type of memory
      DEBUG_printString("This is not MMC device\r\n");
      return NULL;
    }
  }
  
  // Increase the clock to for the data mode usage
  SDMMC_setClockRate(hSDMMCMemInfo->hSDMMCInfo, hSDMMCMemInfo->hSDMMCInfo->hSDMMCCfg->dataModeClockRate);

  return hSDMMCMemInfo;
}

// Read bytes from SDMMC memory
Uint32 SDMMC_MEM_readBytes (SDMMC_MEM_InfoHandle hSDMMCMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *dest)
{
  SDMMC_InfoHandle hSDMMCInfo = hSDMMCMemInfo->hSDMMCInfo;
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) (hSDMMCMemInfo->hSDMMCInfo->regs);
  Uint32 status = E_PASS, retVal;
  Uint32 numBlks;
  Uint32 timeOut;
  
  if (byteCnt == 0)
  {
    return E_FAIL;
  }

  // Find out number of blocks to be written (i.e. round up to nearest block)
  numBlks = (byteCnt + hSDMMCInfo->dataBytesPerBlk - 1) >> hSDMMCInfo->dataBytesPerBlkPower2;
  
  // Update byteCnt to be multiple of block length
  byteCnt = numBlks * hSDMMCInfo->dataBytesPerBlk;

  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SET_BLOCKLEN, hSDMMCInfo->dataBytesPerBlk, 0);
  if(status != E_PASS)
  {
    return  E_FAIL;
  }
      
  SDMMC_clearResponse(hSDMMCInfo);

  SDMMC_setBlockCount(hSDMMCInfo, numBlks);

  SDMMC_setFifoDirection(hSDMMCInfo,SDMMC_FIFO_DIRECTION_RX);

  // Send read block command
  if (hSDMMCMemInfo->capacity == SDMMC_MEM_CAPACITY_TYPE_HIGH)
  {
    if (numBlks == 1)
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2000 | SDMMC_READ_SINGLE_BLOCK, addr >> hSDMMCInfo->dataBytesPerBlkPower2, 0);
    }
    else
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0xA080 | SDMMC_READ_MULTIPLE_BLOCK, addr >> hSDMMCInfo->dataBytesPerBlkPower2, 0);
    }
  }
  else
  {
    if (numBlks == 1)
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2000 | SDMMC_READ_SINGLE_BLOCK, addr, 0);
    }
    else
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0xA080 | SDMMC_READ_MULTIPLE_BLOCK, addr, 0);
    }
  }
  
  if(status != E_PASS)
  {
    return E_FAIL;
  }
  
  // Perform data read
  status=SDMMC_readNWords(hSDMMCInfo, (Uint32*)dest, byteCnt);
  if(status !=E_PASS)
  {
    return E_FAIL; 
  }
  
  // Delay needed for safety
  if (numBlks == 1)
  {
    UTIL_waitLoop(100); 
    timeOut = 3000;
  }
  else
  {
    UTIL_waitLoop(10000);
    timeOut = 50000;
  }
  
  do
  {
    if(SDMMC->MMCST0 & SDMMC_MMCST0_DATDNE_MASK)
    {
      retVal = E_PASS;
      break;
    }

    if( --timeOut == 0 )
    {
      retVal = E_TIMEOUT;
      break;
    }
  }
  while(1);

  // Send the Stop Tx Command
  if (numBlks > 1)
  {
    if(retVal == E_PASS)
    {
      status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_STOP_TRANSMISSION, addr, 0);
      if(status != E_PASS)
      {
        retVal = E_FAIL;
      }
    }
  }
  return retVal;
}

Uint32 SDMMC_MEM_goIdle(SDMMC_MEM_InfoHandle hSDMMCMemInfo)
{
  return SDMMC_sendCmd(hSDMMCMemInfo->hSDMMCInfo, (SDMMC_MMCCMD_INITCK_MASK | SDMMC_GO_IDLE_STATE), 0, 0);
}

// Defining this macro for the build will cause write (flash) ability to be removed
// This can be used for using this driver as read-only for ROM code
#ifndef USE_IN_ROM    

// Global Erase NOR Flash
Uint32 SDMMC_MEM_globalErase(SDMMC_MEM_InfoHandle hSDMMCMemInfo)
{
  return E_PASS;
}

// NAND Flash erase block function
Uint32 SDMMC_MEM_eraseBytes(SDMMC_MEM_InfoHandle hSDMMCMemInfo, Uint32 startAddr, Uint32 byteCnt)
{  
  return E_PASS;
}

// Verify bytes written into SDMMC memory
Uint32 SDMMC_MEM_verifyBytes (SDMMC_MEM_InfoHandle hSDMMCMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *src, Uint8 *dest)
{
  int i;
  Uint32 *wDest, *wSrc;
  
  wDest = (Uint32 *) dest; wSrc = (Uint32 *) src;
  
  if (SDMMC_MEM_readBytes(hSDMMCMemInfo, addr, byteCnt, dest) != E_PASS)
  {
    return E_FAIL;
  }

  for (i=0; i<(byteCnt>>2); i++)
  {
    if ( wDest[i] != wSrc[i] )
    {
	    return E_FAIL;
    }
  }
  
  return E_PASS;
}

// Write bytes to SDMMC memory
Uint32 SDMMC_MEM_writeBytes (SDMMC_MEM_InfoHandle hSDMMCMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *src)
{
  SDMMC_InfoHandle hSDMMCInfo = hSDMMCMemInfo->hSDMMCInfo;
  DEVICE_SDMMCRegs *SDMMC = (DEVICE_SDMMCRegs *) (hSDMMCMemInfo->hSDMMCInfo->regs);
  Uint32 status = E_PASS;
  Uint32 numBlks;
  
  if (byteCnt == 0)
  {
    return E_FAIL;
  }

  // Find out number of blocks to be written (i.e. round up to nearest block)
  numBlks = (byteCnt + hSDMMCInfo->dataBytesPerBlk - 1) >> hSDMMCInfo->dataBytesPerBlkPower2;
  
  // Update byteCnt to be multiple of block length
  byteCnt = numBlks * hSDMMCInfo->dataBytesPerBlk;

  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SET_BLOCKLEN, hSDMMCInfo->dataBytesPerBlk, 0);
  if(status != E_PASS)
  {
    return  E_FAIL;
  }
      
  SDMMC_clearResponse(hSDMMCInfo);

  SDMMC_setBlockCount(hSDMMCInfo, numBlks);

  SDMMC_setFifoDirection(hSDMMCInfo,SDMMC_FIFO_DIRECTION_TX);

  
  // Prime the FIFO with data
  status = SDMMC_writeNWords(hSDMMCInfo, (Uint32*)src, hSDMMCInfo->dataBytesPerOp);
  byteCnt -= hSDMMCInfo->dataBytesPerOp;
  src += hSDMMCInfo->dataBytesPerOp;
  
  // Write Data, every time MMCDXR Reg full
  if (SDMMC_MEM_CAPACITY_TYPE_HIGH == hSDMMCMemInfo->capacity)
  {
    // Issue the write command to the device
    if(numBlks == 1)
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2800 | SDMMC_WRITE_BLOCK, addr >> hSDMMCInfo->dataBytesPerBlkPower2, 0);
    }
    else
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2880 | SDMMC_WRITE_MULTIPLE_BLOCK, addr >> hSDMMCInfo->dataBytesPerBlkPower2, 0);
    }
  }
  else
  {
   
    if(numBlks == 1)
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2800 | SDMMC_WRITE_BLOCK, addr, 0);
    }
    else
    {
      status = SDMMC_sendCmd(hSDMMCInfo, 0x2880 | SDMMC_WRITE_MULTIPLE_BLOCK, addr, 0);
    }
  }
  
  // Finish the transfer of the data
  status = SDMMC_writeNWords(hSDMMCInfo, (Uint32*)src, byteCnt);
     
  // Delay required
  UTIL_waitLoop(100);
   
  if(status != E_PASS)
  {
    return E_FAIL;
  }
  
  // Wait until status shows done
  while ((SDMMC->MMCST0 & SDMMC_MMCST0_DATDNE_MASK) == 0);

  if (numBlks > 1)
  {
    status = SDMMC_sendCmd(hSDMMCInfo, 0xA180 | SDMMC_STOP_TRANSMISSION, 0, SDMMC_RESPONSE_TIMEOUT);
  }

  return status;
}
#endif


/************************************************************
* Local Function Definitions                                *
************************************************************/
/**
  MMC/SD interface selection, Controller and Card Initialization
*/
static Uint32 LOCAL_detectSDCard(SDMMC_MEM_InfoHandle hSDMMCMemInfo)
{
  SDMMC_InfoHandle hSDMMCInfo = hSDMMCMemInfo->hSDMMCInfo;
  Uint32 status, opTimeOut, temp, *cardReg;

  // Place all cards in idle state
  SDMMC_MEM_goIdle(hSDMMCMemInfo);

  // Need an arbitrary(?) wait loop here to give time for cards
  // to finish their own reset procedures
  UTIL_waitLoop(SDMMC_RESPONSE_TIMEOUT);

  // Send CMD8 for SDHC or SDXC detection
  status = SDMMC_sendCmd(hSDMMCInfo, SDHC_SEND_IF_COND, (SDHC_CMD8_HIGH_VOLTAGE_RANGE|SDHC_CMD8_CHECK_PATTERN), SDMMC_RESPONSE_TIMEOUT);

  if( status != E_PASS )
  {
    // We got no response to CMD8, which means one of three things:
    //  1) Ver2.00 or later SD Memory Card with voltage mismatch OR
    //  2) Ver1.X SD Memory Card OR
    //  3) Not an SD Memory Card

    // Repeat sending ACMD41 with HCS = 0 until we get no response
    // or the device responds that it is not busy
    opTimeOut = SDMMC_OP_TIMEOUT;
    do
    {
      // Send ACMD41 with HCS = 0
      status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_APP_CMD, 0, SDMMC_RESPONSE_TIMEOUT);
      if ( status != E_PASS )
      {
        continue;
      }
      status = SDMMC_sendCmd(hSDMMCInfo, SD_SEND_OP_COND, (SD_ACMD41_HCS_LOW | SDMMC_MEM_VDD_27_36), SDMMC_RESPONSE_TIMEOUT);
      if( status != E_PASS )
      {
        // Not an SD Card, let's go try MMC initialization
        return E_FAIL;
      }

      SDMMC_getResponse(hSDMMCInfo, NULL);
      temp = (hSDMMCInfo->response.responseData[3] & 0xC0000000);
    }
    while ( (--opTimeOut > 0) && ((temp & 0x80000000) == 0) );

    if (opTimeOut == 0)
    {
      return E_FAIL;
    }

    // If we get here than the card returned ready and it is a
    // v1.x standard capacity SD card
    hSDMMCMemInfo->capacity = SDMMC_MEM_CAPACITY_TYPE_STANDARD;
    hSDMMCMemInfo->memType  = SDMMC_MEM_TYPE_SD;

  }
  else
  {
    // We got a response to CMD8, which should mean one thing:
    //  1) Ver2.00 or later SD Memory Card (no voltage mismatch)
    SDMMC_getResponse(hSDMMCInfo, NULL);

    // Validate the check pattern
    if ( (hSDMMCInfo->response.responseData[3] & 0x000000FF) != SDHC_CMD8_CHECK_PATTERN)
    {
      // Check pattern failed, which probably just means this card is not
      // usable, but let's go try MMC initialization
      return E_FAIL;
    }

    // Repeat sending ACMD41 with HCS = 1 until we get no response
    // or the device responds that it is not busy
    opTimeOut = SDMMC_OP_TIMEOUT;
    do
    {
      // Send ACMD41 with HCS = 1
      status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_APP_CMD, 0, 0xFFFF);
      if ( status != E_PASS )
      {
        continue;
      }
      status = SDMMC_sendCmd(hSDMMCInfo, SD_SEND_OP_COND, (SD_ACMD41_HCS_HIGH | SDMMC_MEM_VDD_27_36), 0xFFFF);
      if( status != E_PASS )
      {
        // Not an SD Card, let's go try MMC initialization
        return E_FAIL;
      }

      SDMMC_getResponse(hSDMMCInfo, NULL);
      temp = (hSDMMCInfo->response.responseData[3] & 0xC0000000);
    }
    while ( (--opTimeOut > 0) && ((temp & 0x80000000) == 0) );

    if (opTimeOut == 0)
    {
      return E_FAIL;
    }

    // If we get here than the card returned ready and it is a
    // v2.x SD card, check CCS bit for capacity
    if (temp & 0x40000000)
    {
      hSDMMCMemInfo->capacity = SDMMC_MEM_CAPACITY_TYPE_HIGH;
    }
    else
    {
      hSDMMCMemInfo->capacity = SDMMC_MEM_CAPACITY_TYPE_STANDARD;
    }
    hSDMMCMemInfo->memType = SDMMC_MEM_TYPE_SD;
  }

  // Check if we timed out from any of the above operations
  if (opTimeOut == 0)
  {
    return E_FAIL;
  }

  // Ask all cards to send their CIDs
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_ALL_SEND_CID, 0, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }

  SDMMC_getResponse(hSDMMCInfo, NULL);

  // Read Relative Address from the card and store it in variable rca
  status = SDMMC_sendCmd(hSDMMCInfo, SD_SEND_RELATIVE_ADDR, 0, 0xFFFF);
  if( status != E_PASS )
  {
    return E_FAIL;
  }

  // RCA received in upper 16 bits of response data
  SDMMC_getResponse(hSDMMCInfo, NULL);
  hSDMMCMemInfo->relCardAddress = (hSDMMCInfo->response.responseData[3]>>16);

  // Read the CSD structure for MMC/SD
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SEND_CSD, hSDMMCMemInfo->relCardAddress<<16, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }

  // Check whether the mode of operation is native MMC/SD or SPI
  SDMMC_getResponse(hSDMMCInfo, NULL);

  // Read CSD from response R2
  cardReg = hSDMMCInfo->response.responseData;

  // Fill the local structure with information required for other commands used in the driver
  hSDMMCMemInfo->csdRegInfo.permWriteProtect    = (cardReg[0]>>13) & 0x1;
  hSDMMCMemInfo->csdRegInfo.tmpWriteProtect     = (cardReg[0]>>12) & 0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkPartial     = (cardReg[0]>>21) & 0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkLenBytes    = 1<<((cardReg[0]>>22) & 0xF);
  hSDMMCMemInfo->csdRegInfo.wpGrpEnable         = (cardReg[0]>>31) & 0x1;
  hSDMMCMemInfo->csdRegInfo.wpGrpSize           = (cardReg[1] & 0x7F) + 1;      // Extracting 7 bits: For MMC - 5 bits reqd; For SD - 7 bits reqd. (have to be taken care by user)
  hSDMMCMemInfo->csdRegInfo.dsrImp              = (cardReg[2]>>12) &0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkMisalign     = (cardReg[2]>>13) &0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkMisalign    = (cardReg[2]>>14) &0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkPartial      = (cardReg[2]>>15) & 0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkLenBytes     = 1<<((cardReg[2]>>16) & 0xF);
  hSDMMCMemInfo->csdRegInfo.sysSpecVersion      = (cardReg[3]>>26) & 0xF;       // These bits are reserved in the case of SD card

  // Select the Card which responded
  // The rca value is sent as the upper 16 bits of the command argument
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SELECT_CARD, (hSDMMCMemInfo->relCardAddress)<<16, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }

  if ( (hSDMMCMemInfo->relCardAddress) > 0 )
  {
    // The rca value is sent as the upper 16 bits of the command argument
    status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SEND_STATUS, (hSDMMCMemInfo->relCardAddress)<<16, 0xFFFF);
    if( status == E_PASS )
    {
      SDMMC_getResponse(hSDMMCInfo, NULL);
      // Response expected: R1 in native mode
      hSDMMCMemInfo->cardStatus.currentState = (SDMMC_MEM_CardStates) ( (hSDMMCInfo->response.responseData[3] >> 9) & 0xf );
      if( (hSDMMCMemInfo->cardStatus.currentState != SDMMC_CARD_STATE_TRAN) &&
          (hSDMMCMemInfo->cardStatus.currentState != SDMMC_CARD_STATE_PRG ) )
      {
        return E_FAIL;
      }
    }
  }

  // Setting bus width as 4-bits for SD cards
  opTimeOut = SDMMC_OP_TIMEOUT;
  do
  {
    /* Send CMD55 */
    status = SDMMC_sendCmd( hSDMMCInfo, SDMMC_APP_CMD, (hSDMMCMemInfo->relCardAddress)<<16, 0xFFFF);
    if ( status != E_PASS )
    {
      return E_FAIL;
    }
    status = SDMMC_sendCmd( hSDMMCInfo, SD_SET_BUS_WIDTH, 2, 0xFFFF);
  }
  while( (status != E_PASS) && --opTimeOut );

  if ( opTimeOut == 0)
  {
    return E_FAIL;
  }

  SDMMC_setDataWidth( hSDMMCInfo,SDMMC_4BIT_DATABUS);

  return status;
}


static Uint32 LOCAL_detectMMC(SDMMC_MEM_InfoHandle hSDMMCMemInfo)
{
  SDMMC_InfoHandle hSDMMCInfo = hSDMMCMemInfo->hSDMMCInfo;
  Uint32 status, opTimeOut, response, temp, *cardReg;
  
  // Place all cards in idle state
  SDMMC_MEM_goIdle(hSDMMCMemInfo);

  // MMC needs some delay between two commands
  UTIL_waitLoop(10000);

  opTimeOut = SDMMC_OP_TIMEOUT;
  do
  {
    // Format and send cmd: Volt. window is usually (3.4-3.2v)
    //  Safe window is between 2.6V to 3.4V
    status = SDMMC_sendCmd(hSDMMCInfo, MMC_SEND_OP_COND, (MMC_OCR_ACCESS_MODE_SECTOR | SDMMC_MEM_VDD_27_36), 0xFFFF);
    if (status != E_PASS)
    {
      continue;
    }
    
    SDMMC_getResponse(hSDMMCInfo, NULL);
    temp = (hSDMMCInfo->response.responseData[3] & (SDMMC_OCR_BUSY_MASK | MMC_OCR_ACCESS_MODE_SECTOR));
  }
  while( (--opTimeOut > 0) && ((temp & SDMMC_OCR_BUSY_MASK) == 0) );
  
  // Check if we timed out in the above operations
  if (opTimeOut == 0)
  {
    return E_FAIL;
  }

  if(temp & MMC_OCR_ACCESS_MODE_SECTOR)
  {
    hSDMMCMemInfo->capacity = SDMMC_MEM_CAPACITY_TYPE_HIGH;
  }
  else
  {
    hSDMMCMemInfo->capacity = SDMMC_MEM_CAPACITY_TYPE_STANDARD;
  }
  hSDMMCMemInfo->memType = SDMMC_MEM_TYPE_MMC;
  
  // At this point we think we have an MMC card
  UTIL_waitLoop(5000);    
  
  // Ask all cards to send their CIDs
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_ALL_SEND_CID, 0, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }

  SDMMC_getResponse(hSDMMCInfo, NULL);
   
  // Set Relative Address for the card. As we are using only one card
  // currently setting the card RCA to MMC_CARD_ID = 2
  // We cannot use 1 because that is the default for all the SanDisk cards
  hSDMMCMemInfo->relCardAddress = 2;
  status = SDMMC_sendCmd(hSDMMCInfo, MMC_SET_RELATIVE_ADDR, hSDMMCMemInfo->relCardAddress<<16, 0xFFFF);
  if (status != E_PASS)
  {
    return E_FAIL;
  }

  // Read card status from response
  response = hSDMMCInfo->response.responseData[3];
  hSDMMCMemInfo->cardStatus.appSpecific   = ( response & 0xFF );
  hSDMMCMemInfo->cardStatus.ready         = ( (response>> 8) & 1 );
  hSDMMCMemInfo->cardStatus.currentState  = (SDMMC_MEM_CardStates) ( (response >> 9) & 0xF );
  hSDMMCMemInfo->cardStatus.eraseReset    = ( (response >> 13) & 1 );
  hSDMMCMemInfo->cardStatus.eccDisabled   = ( (response >> 14) & 1 );
  hSDMMCMemInfo->cardStatus.wpEraseSkip   = ( (response >> 15) & 1 );
  hSDMMCMemInfo->cardStatus.errorFlags    = ( (response >> 16) & 0xFFFF );

  // Send Status may be required at this point
   
  // Read the CSD structure for MMC/SD
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SEND_CSD, hSDMMCMemInfo->relCardAddress<<16, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }  

  // Check whether the mode of operation is native MMC/SD or SPI
  SDMMC_getResponse(hSDMMCInfo, NULL);

  // Read CSD from response R2
  cardReg = hSDMMCInfo->response.responseData;
  
  // Fill the local structure with information required for other commands used in the driver
  hSDMMCMemInfo->csdRegInfo.permWriteProtect    = (cardReg[0]>>13) & 0x1;
  hSDMMCMemInfo->csdRegInfo.tmpWriteProtect     = (cardReg[0]>>12) & 0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkPartial     = (cardReg[0]>>21) & 0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkLenBytes    = 1<<((cardReg[0]>>22) & 0xF);
  hSDMMCMemInfo->csdRegInfo.wpGrpEnable         = (cardReg[0]>>31) & 0x1;
  hSDMMCMemInfo->csdRegInfo.wpGrpSize           = (cardReg[1] & 0x7F) + 1;        // Extracting 7 bits: For MMC - 5 bits reqd; For SD - 7 bits reqd. (have to be taken care by user)
  hSDMMCMemInfo->csdRegInfo.dsrImp              = (cardReg[2]>>12) &0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkMisalign     = (cardReg[2]>>13) &0x1;
  hSDMMCMemInfo->csdRegInfo.writeBlkMisalign    = (cardReg[2]>>14) &0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkPartial      = (cardReg[2]>>15) & 0x1;
  hSDMMCMemInfo->csdRegInfo.readBlkLenBytes     = 1<<((cardReg[2]>>16) & 0xF);
  hSDMMCMemInfo->csdRegInfo.sysSpecVersion      = (cardReg[3]>>26) & 0xF;         // These bits are reserved in the case of SD card

  // Select the Card which responded
  // The rca value is sent as the upper 16 bits of the command argument
  status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SELECT_CARD, (hSDMMCMemInfo->relCardAddress)<<16, 0xFFFF);
  if( status != E_PASS )
  {
    return status;
  }

  if ( (hSDMMCMemInfo->relCardAddress) > 0 )
  {
    status = SDMMC_sendCmd(hSDMMCInfo, SDMMC_SEND_STATUS, (hSDMMCMemInfo->relCardAddress)<<16, 0xFFFF);
    if( status == E_PASS )
    {
      SDMMC_getResponse(hSDMMCInfo, NULL);
      // Response expected: R1 in native mode
      hSDMMCMemInfo->cardStatus.currentState = (SDMMC_MEM_CardStates) ( (hSDMMCInfo->response.responseData[3] >> 9) & 0xF );
      if( (hSDMMCMemInfo->cardStatus.currentState != SDMMC_CARD_STATE_TRAN) &&
          (hSDMMCMemInfo->cardStatus.currentState != SDMMC_CARD_STATE_PRG ) ) 
      {
        return E_FAIL;
      }
    }
  }

  // Reset the timeout value
  SDMMC_setDataWidth( hSDMMCInfo, SDMMC_1BIT_DATABUS);
  
  return status;
}



/***********************************************************
* End file                                                 *
***********************************************************/
