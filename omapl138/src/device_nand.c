/*
 * device_nand.c
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
    FILE        : device_nand.c 				                             	 	        
    PROJECT     : TI Booting and Flashing Utilities
    AUTHOR      : Daniel Allred
    DESC        : This file descibes and implements various device-specific NAND
                  functionality.
-------------------------------------------------------------------------- */ 

// General type include
#include "tistdtypes.h"

// Device specific CSL
#include "device.h"

// Debug functions for non-ROMed version
#include "debug.h"

// Device specific NAND details
#include "device_nand.h"

// Generic NAND header file
#include "nand.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
************************************************************/

// ECC offset defines
#define DEVICE_NAND_ECC_START_OFFSET (6)


/************************************************************
* Local Function Declarations                               *
************************************************************/

// Required implementation for NAND_ECC_Info struct
static void DEVICE_NAND_ECC_calculate(NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *calcECC);
static void DEVICE_NAND_ECC_store(NAND_InfoHandle hNandInfo, Uint8 *spareBytes, Uint8 opNum, Uint8 *calcECC);
static void DEVICE_NAND_ECC_enable(NAND_InfoHandle hNandInfo);
static void DEVICE_NAND_ECC_disable(NAND_InfoHandle hNandInfo);
static void DEVICE_NAND_ECC_read(NAND_InfoHandle hNandInfo, Uint8 *spareBytes, Uint8 opNum, Uint8 *readECC);
static Uint32 DEVICE_NAND_ECC_correct(NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *readECC);

// Required implementation for NAND_BB_Info struct
static void DEVICE_NAND_BB_markSpareBytes(NAND_InfoHandle hNandInfo, Uint8 *spareBytes);
static Uint32 DEVICE_NAND_BB_checkSpareBytes(NAND_InfoHandle hNandInfo, Uint8 *spareBytes);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

// Specfic ECC info to support ROM of device
const NAND_ECC_InfoObj DEVICE_NAND_ECC_info = 
{
  TRUE,         // ECCEnable
  16,           // calcECCByteCnt
  10,           // storedECCByteCnt
  &(DEVICE_NAND_ECC_calculate),
  &(DEVICE_NAND_ECC_store),
  &(DEVICE_NAND_ECC_enable),
  &(DEVICE_NAND_ECC_disable),
  &(DEVICE_NAND_ECC_read),
  &(DEVICE_NAND_ECC_correct)
};

const NAND_BB_InfoObj DEVICE_NAND_BB_info = 
{
  TRUE,
  TRUE,
  &(DEVICE_NAND_BB_markSpareBytes),
  &(DEVICE_NAND_BB_checkSpareBytes)
};

// Specific Page layout to support ROM of device
const NAND_PAGE_LayoutObj DEVICE_NAND_PAGE_layout = 
{
  // Data region definition
  {
    NAND_REGION_DATA,           // Region Type
    NAND_OFFSETS_RELTODATA,     // Offsets relative type
    DEVICE_NAND_MAX_BYTES_PER_OP,      // bytesPerOp
    { 0*(512) ,1*(512) ,2*(512) ,3*(512) ,    // dataOffsets
      4*(512) ,5*(512) ,6*(512) ,7*(512) ,
      8*(512) ,9*(512) ,10*(512),11*(512),
      12*(512),13*(512),14*(512),15*(512) }
  },
  
  // Spare region definition
  {
    NAND_REGION_SPARE,          // Region Type
    NAND_OFFSETS_RELTOSPARE,    // Offsets relative type
    DEVICE_NAND_MAX_SPAREBYTES_PER_OP, // bytesPerOp  
    { 0*16  ,1*16  ,2*16  ,3*16  ,      // spareOffsets
      4*16  ,5*16  ,6*16  ,7*16  ,
      8*16  ,9*16  ,10*16 ,11*16 ,
      12*16 ,13*16 ,14*16 ,15*16  }
  }
};

// Table of ROM supported NAND devices
const NAND_CHIP_InfoObj DEVICE_NAND_CHIP_infoTable[] = 
{// devID,  numBlocks,  pagesPerBlock,  bytesPerPage
  { 0xE3,   512,        16,             512+16},	// 4 MB
  { 0xE5,   512,        16,             512+16},	// 4 MB
  
  { 0x39,   1024,       16,             512+16},	// 8 MB
  { 0xE6,   1024,       16,             512+16},	// 8 MB
  { 0x49,   1024,       16,             512+16},	// 8 MB x16
  { 0x59,   1024,       16,             512+16},	// 8 MB x16
  
  { 0x6B,   1024,       16,             512+16},	// 8 MB
  
  { 0x33,   1024,       32,             512+16},	// 16 MB
  { 0x73,   1024,       32,             512+16},	// 16 MB
  { 0x43,   1024,       32,             512+16},	// 16 MB x16
  { 0x53,   1024,       32,             512+16},	// 16 MB x16
  
  { 0x75,   2048,       32,             512+16},	// 32 MB
  { 0x35,   2048,       32,             512+16},	// 32 MB
  { 0xF5,   2048,       32,             512+16},  // 32 MB  

  { 0x76,   4096,       32,             512+16},	// 64 MB
  { 0x36,   4096,       32,             512+16},	// 64 MB
  { 0x46,   4096,       32,             512+16},  // 64 MB x16
  { 0x56,   4096,       32,             512+16},  // 64 MB x16

  { 0x78,   8192,       32,             512+16},  // 128 MB
  { 0x79,   8192,       32,             512+16},	// 128 MB
  { 0x72,   8192,       32,             512+16},  // 128 MB x16
  { 0x74,   8192,       32,             512+16},  // 128 MB x16

  { 0x71,   16384,      32,             512+16},	// 256 MB  

  { 0xA1,   1024,       64,             2048+64},	// 128 MB
  { 0xB1,   1024,       64,             2048+64}, // 128 MB x16
  { 0xC1,   1024,       64,             2048+64}, // 128 MB x16
  { 0xF1,   1024,       64,             2048+64}, // 128 MB  

  { 0xAA,   2048,       64,             2048+64},	// 256 MB (4th ID byte will be checked)
  { 0xBA,   2048,       64,             2048+64}, // 256 MB x16 (4th ID byte will be checked) 
  { 0xCA,   2048,       64,             2048+64}, // 256 MB x16 (4th ID byte will be checked)
  { 0xDA,   2048,       64,             2048+64},	// 256 MB (4th ID byte will be checked)

  { 0xAC,   4096,       64,             2048+64}, // 512 MB (4th ID byte will be checked)
  { 0xBC,   4096,       64,             2048+64}, // 512 MB x16 (4th ID byte will be checked)
  { 0xCC,   4096,       64,             2048+64}, // 512 MB x16 (4th ID byte will be checked)
  { 0xDC,   4096,       64,             2048+64},	// 512 MB (4th ID byte will be checked)

  { 0xA3,   8192,       64,             2048+64}, // 1 GB (4th ID byte will be checked)
  { 0xB3,   8192,       64,             2048+64}, // 1 GB x16 (4th ID byte will be checked) 
  { 0xC3,   8192,       64,             2048+64}, // 1 GB x16 (4th ID byte will be checked) 
  { 0xD3,   8192,       64,             2048+64}, // 1 GB (4th ID byte will be checked)

  { 0xA5,  16384,       64,             2048+64}, // 2 GB (4th ID byte will be checked)
  { 0xB5,  16384,       64,             2048+64}, // 2 GB x16 (4th ID byte will be checked)
  { 0xC5,  16384,       64,             2048+64}, // 2 GB x16 (4th ID byte will be checked)
  { 0xD5,  16384,       64,             2046+64}, // 2 GB (4th ID byte will be checked)
   
  { 0x00,      0,        0,                   0}	// Dummy null entry to indicate end of table
};


/************************************************************
* Global Function Definitions                               *
************************************************************/

static void DEVICE_NAND_ECC_calculate(NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *calcECC)
{
  Uint32 *eccValue = (Uint32 *) calcECC;

  eccValue[0] = (AEMIF->NAND4BITECC1 & 0x03FF03FF);
  eccValue[1] = (AEMIF->NAND4BITECC2 & 0x03FF03FF);
  eccValue[2] = (AEMIF->NAND4BITECC3 & 0x03FF03FF);
  eccValue[3] = (AEMIF->NAND4BITECC4 & 0x03FF03FF);
}

// Do conversion from 8 10-bit ECC values from HW (in 16 byte input) to 10 8-bit continguous values for storage in spare bytes
static void DEVICE_NAND_ECC_store(NAND_InfoHandle hNandInfo, Uint8 *spareBytes, Uint8 opNum, Uint8 *calcECC)
{
  Uint32 out[3],in[4],i;
  
  for (i=0; i <DEVICE_NAND_ECC_info.calcECCByteCnt; i++)
  {
    ((Uint8 *)in)[i] = ((Uint8 *)calcECC)[i];
  }
  
  out[0] = ((in[1]&0x00030000) <<14) | ((in[1]&0x000003FF) <<20) |
           ((in[0]&0x03FF0000) >> 6) | ((in[0]&0x000003FF)     );
  out[1] = ((in[3]&0x0000000F) <<28) | ((in[2]&0x03FF0000) << 2) |
           ((in[2]&0x000003FF) << 8) | ((in[1]&0x03FC0000) >>18);
  out[2] = ((out[2]&0xFFFF0000)    ) | ((in[3]&0x03FF0000) >>10) |
           ((in[3]&0x000003F0) >> 4);
  
  for (i=0; i <DEVICE_NAND_ECC_info.storedECCByteCnt; i++)
  {
    spareBytes[i + DEVICE_NAND_ECC_START_OFFSET + hNandInfo->spareBytesPerOp*opNum] = ((Uint8 *)out)[i];
  }
}

static void DEVICE_NAND_ECC_enable(NAND_InfoHandle hNandInfo)
{
  VUint32 dummy;

  // Select appropriate CS for ECC usage
  AEMIF->NANDFCR &= ~DEVICE_EMIF_NANDFCR_4BITECC_SEL_MASK;
  AEMIF->NANDFCR |= (hNandInfo->CSOffset << DEVICE_EMIF_NANDFCR_4BITECC_SEL_SHIFT) & DEVICE_EMIF_NANDFCR_4BITECC_SEL_MASK;

  // Write appropriate bit to start ECC calcualtions (bit 12 for four bit ECC)
  AEMIF->NANDFCR |= 0x1 << DEVICE_EMIF_NANDFCR_4BITECC_START_SHIFT;
  
  // Dummy read on CFG bus to guarantee ECC bit is set before we do anything else
  dummy = AEMIF->ERCSR;
}

static void DEVICE_NAND_ECC_disable(NAND_InfoHandle hNandInfo)
{
  VUint32 dummy;

  // Dummy read of a different CS region (flush data writes)
  if ( (hNandInfo->CSOffset+1) == DEVICE_EMIF_NUMBER_CE_REGION )
    dummy = *((VUint32*)(((VUint8*)hNandInfo->flashBase) - DEVICE_EMIF_INTER_CE_REGION_SIZE));
  else
    dummy = *((VUint32*)(((VUint8*)hNandInfo->flashBase) + DEVICE_EMIF_INTER_CE_REGION_SIZE));
  
  // Read any ECC register to end the calculation and clear the ECC start bit
  dummy = AEMIF->NAND4BITECC1;
}

// Read 10 8-bit values from pagebuffer and convert to 8 10-bit values
static void DEVICE_NAND_ECC_read(NAND_InfoHandle hNandInfo, Uint8 *spareBytes, Uint8 opNum, Uint8 *readECC)
{
  Uint32 in[3],out[4],i;
  
  for (i=0; i <DEVICE_NAND_ECC_info.storedECCByteCnt; i++)
  {
    ((Uint8 *)in)[i] = spareBytes[i + DEVICE_NAND_ECC_START_OFFSET + hNandInfo->spareBytesPerOp*opNum];
  }
  
  out[0] = ((in[0]&0x000FFC00) << 6) | ((in[0]&0x000003FF)     );
  out[1] = ((in[1]&0x000000FF) <<18) | ((in[0]&0xC0000000) >>14) |
           ((in[0]&0x3FF00000) >>20);
  out[2] = ((in[1]&0x0FFC0000) >> 2) | ((in[1]&0x0003FF00) >> 8);
  out[3] = ((in[2]&0x0000FFC0) <<10) | ((in[2]&0x0000003F) << 4) |
           ((in[1]&0xF0000000) >>28);
  
  for (i=0; i <DEVICE_NAND_ECC_info.calcECCByteCnt; i++)
  {
    readECC[i] = ((Uint8 *)out)[i];
  }
}

static Uint32 DEVICE_NAND_ECC_correct(NAND_InfoHandle hNandInfo, Uint8 *data, Uint8 *readECC)
{
  VUint32 temp, corrState, numE;
  Uint32 i;
  Uint16 addOffset, corrValue;
  Uint16* syndrome10 = (Uint16 *)readECC;

  // Load the syndrome10 (from 7 to 0) values
  for(i=8;i>0;i--)
  {
    AEMIF->NAND4BITECCLOAD = (syndrome10[i-1] & 0x000003FF);
  }
  
  // Read the EMIF status and version (dummy call) 
  temp = AEMIF->ERCSR;
  
  // Check if error is detected
  temp = (AEMIF->NAND4BITECC1 & 0x03FF03FF) | (AEMIF->NAND4BITECC2 & 0x03FF03FF) |
         (AEMIF->NAND4BITECC3 & 0x03FF03FF) | (AEMIF->NAND4BITECC4 & 0x03FF03FF);
  if(temp == 0)
  {
    return E_PASS;
  }

  // Start calcuating the correction addresses and values
  AEMIF->NANDFCR |= (0x1U << DEVICE_EMIF_NANDFCR_4BITECC_ADD_CALC_START_SHIFT);
  
  // Wait until ECC HW goes into correction state
  do 
  {
    corrState = (AEMIF->NANDFSR & DEVICE_EMIF_NANDFSR_ECC_STATE_MASK)>>DEVICE_EMIF_NANDFSR_ECC_STATE_SHIFT;
  } while (corrState < 4);

  // Loop until timeout or the ECC calculations are complete ( 0<=corrState<=3 )
  i = NAND_TIMEOUT;
  do
  {
    corrState = (AEMIF->NANDFSR & DEVICE_EMIF_NANDFSR_ECC_STATE_MASK)>>DEVICE_EMIF_NANDFSR_ECC_STATE_SHIFT;
    i--;
  }
  while((i>0) && (corrState > 0x3));
  
  if ((corrState == 1) || (corrState > 3))
  {
    // Clear 4BITECC_ADD_CALC_START bit in NANDFCR
    temp = AEMIF->NANDERRVAL1;
    return E_FAIL;
  }
  else if (corrState == 0)
  {
    temp = AEMIF->NANDERRVAL1;
    return E_PASS;
  }
  else
  {
    // Error detected and address calculated
    // Number of errors corrected 17:16
    numE = (AEMIF->NANDFSR & DEVICE_EMIF_NANDFSR_ECC_ERRNUM_MASK) >> DEVICE_EMIF_NANDFSR_ECC_ERRNUM_SHIFT;

    switch( numE )
    {
      case 3:     // Four errors
        addOffset = 519 - ( (AEMIF->NANDERRADD2 & (0x03FF0000))>>16 );
        if (addOffset > 511) return E_FAIL;
        corrValue = (AEMIF->NANDERRVAL2 & (0x03FF0000))>>16;
        data[addOffset] ^= (Uint8)corrValue;
        // Fall through to case 2
      case 2:     // Three errors
        addOffset = 519 - (AEMIF->NANDERRADD2 & (0x000003FF));
        if (addOffset > 511) return E_FAIL;
        corrValue = AEMIF->NANDERRVAL2 & (0x000003FF);
        data[addOffset] ^= (Uint8)corrValue;
        // Fall through to case 1
      case 1:     // Two errors
        addOffset = 519 - ( (AEMIF->NANDERRADD1 & (0x03FF0000))>>16 );
        if (addOffset > 511) return E_FAIL;
        corrValue = (AEMIF->NANDERRVAL1 & (0x03FF0000))>>16;
        data[addOffset] ^= (Uint8)corrValue;        
        // Fall through to case 0
      case 0:     // One error
        addOffset = 519 - (AEMIF->NANDERRADD1 & (0x000003FF));
        if (addOffset > 511) return E_FAIL;
        corrValue = AEMIF->NANDERRVAL1 & (0x3FF);
        data[addOffset] ^= (Uint8)corrValue;
        break;
    }
    return E_PASS;
  }
}

// Function to manipulate the spare bytes to mark as a bad block
static void DEVICE_NAND_BB_markSpareBytes(NAND_InfoHandle hNandInfo, Uint8 *spareBytes)
{
  Uint32 i,j;
  // Mark all the free bytes (non-ECC bytes) as 0x00
  for (j = 0; j< hNandInfo->numOpsPerPage; j++)
  {
    for (i=0; i<DEVICE_NAND_ECC_START_OFFSET; i++)
    {
      spareBytes[i+hNandInfo->spareBytesPerOp*j] = 0x00;
    }
  }
}

// Function to determine if the spare bytes indicate a bad block
static Uint32 DEVICE_NAND_BB_checkSpareBytes(NAND_InfoHandle hNandInfo, Uint8 *spareBytes)
{
  Uint32 i,j;
  // Check all the free bytes (non-ECC bytes) for 0xFF
  for (j = 0; j< hNandInfo->numOpsPerPage; j++)
  {
    for (i=0; i<DEVICE_NAND_ECC_START_OFFSET; i++)
    {
      if (spareBytes[i+hNandInfo->spareBytesPerOp*j] & 0x00FF != 0xFF)
        return E_FAIL;
    }
  }

  return E_PASS;
}

/***********************************************************
* End file                                                 *
***********************************************************/


