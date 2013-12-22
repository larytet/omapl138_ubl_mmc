/*
 * i2c_mem.h
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
  FILE        : i2c_mem.h
  PROJECT     : DaVinci User Boot-Loader and Flasher
  AUTHOR      : Texas Instruments Incorporated
  DESC        : Generic I2C memory device driver header file
 ----------------------------------------------------------------------------- */

#ifndef _I2C_MEM_H_
#define _I2C_MEM_H_

#include "tistdtypes.h"

#include "i2c.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/


/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

// I2C_MEM Config Structure
typedef struct _I2C_MEM_CONFIG_
{
  Uint32 i2cMemAddr;
  Uint32 pageSize;
  Uint32 addrWidth;
  Uint32 memorySize;
}
I2C_MemConfigObj, *I2C_MemConfigHandle;

// I2C_MEM Driver Structure
typedef struct _I2C_MEM_INFO_
{
  I2C_InfoHandle hI2CInfo;
  I2C_MemConfigHandle hI2CMemCfg;
}
I2C_MemInfoObj, *I2C_MemInfoHandle;



/************************************************************
* Global Function Declarations                              *
************************************************************/

extern I2C_MemInfoHandle I2C_MEM_open(Uint32 i2cPeripheralNum, I2C_ConfigHandle hI2cCfg, I2C_MemConfigHandle hI2CMemCfg);
extern Uint32 I2C_MEM_readBytes(I2C_MemInfoHandle hI2CMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *dest);
extern Uint32 I2C_MEM_writeBytes(I2C_MemInfoHandle hI2CMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *src);
extern Uint32 I2C_MEM_verifyBytes(I2C_MemInfoHandle hI2CMemInfo, Uint32 addr, Uint32 byteCnt, Uint8 *src, Uint8* dest);
extern Uint32 I2C_MEM_globalErase(I2C_MemInfoHandle hI2CMemInfo);
extern Uint32 I2C_MEM_eraseBytes(I2C_MemInfoHandle hI2CMemInfo, Uint32 startAddr, Uint32 byteCnt);


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // _I2C_H_
