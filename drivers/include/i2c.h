/*
 * i2c.h
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
  FILE        : i2c.h
  PROJECT     : DaVinci User Boot-Loader and Flasher
  AUTHOR      : Texas Instruments Incorporated
  DESC        : Generic I2C driver header file
 ----------------------------------------------------------------------------- */

#ifndef _I2C_H_
#define _I2C_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/

#define I2C_TIMEOUT       (0x10000u)

#define I2C_SLAVE_ADDR    (0x50u)
#define I2C_OWN_ADDR      (0x29u)


/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

// I2C Module Roles
typedef enum _I2C_ROLE_
{
  I2C_ROLE_MASTER = 0x01,
  I2C_ROLE_SLAVE = 0x02
}
I2C_Role;

// I2C Module Modes of Operation
typedef enum _I2C_MODE_
{
  I2C_MODE_MANUAL_STOP          = 0x01,
  I2C_MODE_REPEAT_MANUAL_STOP   = 0x02,
  I2C_MODE_AUTO_STOP            = 0x04,
  I2C_MODE_REPEAT_AUTO_STOP     = 0x08
}
I2C_Mode;

typedef enum _I2C_STATUS_
{
  I2C_STATUS_RESET = 0x01,
  I2C_STATUS_READY = 0x02,
  I2C_STATUS_BUSY  = 0x03
}
I2C_Status;

typedef enum _I2C_ADDRESSING_MODE_
{
  I2C_ADDRESSING_7BIT  = 0x01,
  I2C_ADDRESSING_10BIT = 0x02
}
I2C_AddressingMode;

typedef struct _I2C_CONFIG_
{
  Uint32              ownAddr;
  Uint8               i2cclkl;
  Uint8               i2cclkh;
  Uint8               prescalar;
  I2C_AddressingMode  addrMode;
}
I2C_ConfigObj, *I2C_ConfigHandle;

// I2C driver structure
typedef struct _I2C_INFO_
{
  Uint32      peripheralNum;
  void        *regs;
  Uint32      slaveAddr;
  I2C_Role    role;
  I2C_Mode    mode;
  I2C_Status  status;
}
I2C_InfoObj, *I2C_InfoHandle;


/************************************************************
* Global Function Declarations                              *
************************************************************/

extern I2C_InfoHandle I2C_open(Uint32 i2cPeripheralNum, I2C_Role role, I2C_Mode mode, I2C_ConfigHandle hConfig);
extern void I2C_assertReset(I2C_InfoHandle hI2CInfo);
extern void I2C_releaseReset(I2C_InfoHandle hI2CInfo);
void I2C_setStopCondition(I2C_InfoHandle hI2CInfo);
void I2C_setStartCondition(I2C_InfoHandle hI2CInfo, Uint32 i2cmdr);
extern Uint32 I2C_readBytes(I2C_InfoHandle hI2CInfo, Uint32 byteCnt, Uint8 *dest);
extern Uint32 I2C_writeBytes(I2C_InfoHandle hI2CInfo, Uint32 byteCnt, Uint8 *src);


/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif // _I2C_H_
