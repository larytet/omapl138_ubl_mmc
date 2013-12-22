/*
 * uart_irda.h
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
  FILE        : uart.h
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : UART driver header file
 ----------------------------------------------------------------------------- */

#ifndef _UART_H_
#define _UART_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/

#define MAXSTRLEN 256


/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

// UART Oversampling Mode
typedef enum _UART_OSM_MODE_
{
  UART_OSM_X16    = 0x01,
  UART_OSM_X13    = 0x02
}
UART_OsmMode;

// UART Parity Modes
typedef enum _UART_PARITY_MODE_
{
  UART_PARITY_NONE    = 0x01,
  UART_PARITY_EVEN    = 0x02,
  UART_PARITY_ODD     = 0x04
}
UART_ParityMode;

typedef enum _UART_STOP_BITS_
{
  UART_STOP_BITS_1    = 0x01,
  UART_STOP_BITS_1_5  = 0x02,
  UART_STOP_BITS_2    = 0x04
}
UART_StopBits;

typedef struct _UART_CONFIG_
{
  UART_OsmMode    osm;
  UART_ParityMode parity;
  UART_StopBits   stopBits;
  Uint8           charLen;
  Uint16          divider;
}
UART_ConfigObj, *UART_ConfigHandle;

// UART driver structure
typedef struct _UART_INFO_
{
  Uint32            peripheralNum;
  void              *regs;
  UART_ConfigHandle config;
}
UART_InfoObj, *UART_InfoHandle;


/***********************************************************
* Global Variable declarations                             *
***********************************************************/

extern __FAR__ UART_InfoHandle hUartInfo;


/************************************************************
* Global Function Declarations                              *
************************************************************/

extern __FAR__ UART_InfoHandle UART_open(Uint32 peripheralNum, UART_ConfigHandle config);
extern __FAR__ void UART_reset(UART_InfoHandle hUartInfo);
extern __FAR__ Uint32 UART_waitForTxEmpty(UART_InfoHandle hUartInfo, Uint32 timeout);

// Simple send/recv functions
extern __FAR__ Uint32 UART_sendString(UART_InfoHandle hUartInfo, String seq, Bool includeNull);
extern __FAR__ Uint32 UART_sendHexInt(UART_InfoHandle hUartInfo, Uint32 value);
extern __FAR__ Uint32 UART_recvString(UART_InfoHandle hUartInfo, String seq);
extern __FAR__ Uint32 UART_recvStringN(UART_InfoHandle hUartInfo, String seq, Uint32* len, Bool stopAtNull);

extern __FAR__ Uint32 UART_checkSequence(UART_InfoHandle hUartInfo, String seq, Bool includeNull);
extern __FAR__ Uint32 UART_recvHexData(UART_InfoHandle hUartInfo, Uint32 numBytes, Uint32* data);



/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
extern far "c" }
#endif

#endif // End _UART_H_

