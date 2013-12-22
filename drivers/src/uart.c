/*
 * uart.c
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
  FILE        : uart.c 				                             	 	        
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : Implementation of specific I/O functionality for the 
                UART peripheral
 ----------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// This module's header file 
#include "uart.h"

// Device specific CSL
#include "device.h"

// Misc. utility function module
#include "util.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern Uint32 DEVICE_UART_baseAddr[];


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

static Uint32 LOCAL_getStringLen(String seq);
static Uint32 LOCAL_setupMode(UART_InfoHandle hUartInfo);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

#ifdef USE_IN_ROM
UART_InfoObj gUartInfo;
#endif

// Default SPI Config strcuture
const UART_ConfigObj DEFAULT_UART_CONFIG = 
{
  UART_OSM_X13,         // osm
  UART_PARITY_NONE,     // parity
  UART_STOP_BITS_1,     // stopBits
  8,                    // charLen
  18                    // divider
};

UART_ConfigHandle const hDEFAULT_UART_CONFIG = (UART_ConfigHandle) &DEFAULT_UART_CONFIG;


/************************************************************
* Global Function Definitions                               *
************************************************************/

UART_InfoHandle UART_open(Uint32 uartPeripheralNum, UART_ConfigHandle config)
{
  UART_InfoHandle hUartInfo;
  volatile DEVICE_UARTRegs *UART;

  // Set NandInfo handle
#ifdef USE_IN_ROM
  hUartInfo = (UART_InfoHandle) &gUartInfo;
#else
  hUartInfo = (UART_InfoHandle) UTIL_allocMem(sizeof(UART_InfoObj));
#endif

  hUartInfo->peripheralNum = uartPeripheralNum;
  hUartInfo->regs = (void *) DEVICE_UART_baseAddr[uartPeripheralNum];
  UART = (volatile DEVICE_UARTRegs *) hUartInfo->regs;
  
  // If UART initialized wait until the UART is not currently transmitting data
  while (  (UART->LCR != 0) && ((UART->LSR & 0x40) == 0) );

  if (config == NULL)
  {
    hUartInfo->config = hDEFAULT_UART_CONFIG;
  }
  else
  {
    hUartInfo->config = config;
  }

  // Setup the UART state
  LOCAL_setupMode(hUartInfo);

  return hUartInfo;
}

void UART_reset(UART_InfoHandle hUartInfo)
{
  // Put UART in reset
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
  
  UART->PWREMU_MGMT = 0;
  UTIL_waitLoop(10000);
}

// Send specified number of bytes 
Uint32 UART_sendString(UART_InfoHandle hUartInfo, String seq, Bool includeNull)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
 
  Uint32 status = 0;
  Int32 i,numBytes;
  Uint32 timerStatus = 1;
	
  numBytes = includeNull?(LOCAL_getStringLen(seq)+1):(LOCAL_getStringLen(seq));
	
  for(i=0;i<numBytes;i++)
  {
    // Enable Timer one time
    DEVICE_TIMER0Start();
    do
    {
      status = (UART->LSR)&(0x20);
      timerStatus = DEVICE_TIMER0Status();
    }
    while (!status && timerStatus);

    if(timerStatus == 0)
      return E_TIMEOUT;
		
    // Send byte 
    (UART->THR) = seq[i];
  }
  return E_PASS;
}

Uint32 UART_sendHexInt(UART_InfoHandle hUartInfo, Uint32 value)
{
  char seq[9];
  Uint32 i,shift,temp;

  for( i = 0; i < 8; i++)
  {
    shift = ((7-i)*4);
    temp = ((value>>shift) & (0x0000000F));
    if (temp > 9)
    {
      temp = temp + 7;
    }
    seq[i] = temp + 48;	
  }
  seq[8] = 0;
  return UART_sendString(hUartInfo, (String)seq, FALSE);
}

Uint32 UART_recvString(UART_InfoHandle hUartInfo, String seq)
{
  Uint32 len = MAXSTRLEN;
  return UART_recvStringN(hUartInfo, seq,&len,TRUE);
}

void UART_flushRx(UART_InfoHandle hUartInfo)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
  Uint32 status = 0;
  char d;

  // Write RXCLR in FCR (same address as IIR)  - reset RX FIFO
  UART->IIR = 0x02;

  // Empty the shift register
  while (1)
  {
    status = (UART->LSR)&(0x01);
    if (! status)
    	break;
    d = (UART->RBR);
  }
}

Uint32 UART_checkRx(UART_InfoHandle hUartInfo)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
  Uint32 status = 0;

  status = (UART->LSR)&(0x01);
  if (status != 0)
	  return E_PASS;

  return E_FAIL;
}

// Receive data from UART 
Uint32 UART_recvStringN(UART_InfoHandle hUartInfo, String seq, Uint32* len, Bool stopAtNull)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
  
  Uint32 i, status = 0;
  Uint32 timerStatus = 1;
  	
  for(i=0;i<(*len);i++)
  {
    // Enable timer one time
    DEVICE_TIMER0Start();
    do
    {
      status = (UART->LSR)&(0x01);
      timerStatus = DEVICE_TIMER0Status();
    }
    while (!status && timerStatus);

    if(timerStatus == 0)
      return E_TIMEOUT;

    // Receive byte 
    seq[i] = (UART->RBR) & 0xFF;

    // Check status for errors
    if( ( (UART->LSR)&(0x1C) ) != 0 )
      return E_FAIL;

    if (stopAtNull && (seq[i] == 0x00))
    {
      *len = i;
      break;
    }
  }
  return E_PASS;
}

// More complex send / receive functions
Uint32 UART_checkSequence(UART_InfoHandle hUartInfo, String seq, Bool includeNull)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;
  
  Uint32 i, numBytes;
  Uint32 status = 0,timerStatus = 1;

  numBytes = includeNull?(LOCAL_getStringLen(seq)+1):(LOCAL_getStringLen(seq));

  for(i=0;i<numBytes;i++)
  {
    // Enable Timer one time
    DEVICE_TIMER0Start();
    do
    {
      status = (UART->LSR)&(0x01);
      timerStatus = DEVICE_TIMER0Status();
    }
    while (!status && timerStatus);

    if(timerStatus == 0)
      return E_TIMEOUT;

    if( ( (UART->RBR)&0xFF) != seq[i] )
      return E_FAIL;
  }
  return E_PASS;
}

Uint32 UART_recvHexData(UART_InfoHandle hUartInfo, Uint32 numBytes, Uint32* data)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;

  Uint32 i,j;
  Uint32 temp[8];
  Uint32 timerStatus = 1, status = 0;
  Uint32 numLongs, numAsciiChar, shift;
    
  if(numBytes == 2)
  {
    numLongs = 1;
    numAsciiChar = 4;
    shift = 12;
  }
  else
  {
    numLongs = numBytes/4;
    numAsciiChar = 8;
    shift = 28;
  }

  for(i=0;i<numLongs;i++)
  {
    data[i] = 0;
    for(j=0;j<numAsciiChar;j++)
    {
      /* Enable Timer one time */
      DEVICE_TIMER0Start();
      do
      {
        status = (UART->LSR)&(0x01);
        timerStatus = DEVICE_TIMER0Status();
      }
      while (!status && timerStatus);

      if(timerStatus == 0)
        return E_TIMEOUT;

      // Converting ascii to Hex
      temp[j] = ((UART->RBR)&0xFF)-48;
      if(temp[j] > 22)    // To support lower case a,b,c,d,e,f
        temp[j] = temp[j] - 39;
      else if(temp[j]>9)  // To support upper case A,B,C,D,E,F
        temp[j] = temp[j] - 7;

      // Checking for bit 1,2,3,4 for reception Error
      if( ( (UART->LSR)&(0x1C) ) != 0)
        return E_FAIL;

      data[i] |= (temp[j]<<(shift-(j*4)));
    }
  }
  return E_PASS;
}


/************************************************************
* Local Function Definitions                                *
************************************************************/

// Get string length by finding null terminating char
static Uint32 LOCAL_getStringLen(String seq)
{
  Uint32 i = 0;
  while ((seq[i] != 0) && (i<MAXSTRLEN)){ i++; }
  if (i == MAXSTRLEN)
    return ((Uint32)-1);
  else
    return i;
}

static Uint32 LOCAL_setupMode(UART_InfoHandle hUartInfo)
{
  DEVICE_UARTRegs *UART = (DEVICE_UARTRegs *) hUartInfo->regs;

  // Reset the UART
  UART_reset(hUartInfo);

  // Take out of reset, set free running
  UART->PWREMU_MGMT &= ~(0x00008000);
  UART->PWREMU_MGMT |= (0x1u << 15) & 0x00008000;
  //UART->PWREMU_MGMT = 0xE003;
  
  // Set oversampling correctly
  if (hUartInfo->config->osm == UART_OSM_X13)
  {
    // Not all devices have this selectable oversampling
    UART->MDR = 0x01;
  }
  else if (hUartInfo->config->osm == UART_OSM_X16)
  {
    // Not all devices have this selectable oversampling
    UART->MDR = 0x00;
  }
  else
  {
    return E_FAIL;
  }
  
  // Set DLAB bit - allows setting of clock divisors
  UART->LCR |= 0x80;
  UTIL_waitLoop(100);

  UART->DLL = hUartInfo->config->divider & 0xFF;
  UART->DLH = (hUartInfo->config->divider >> 8) & 0xFF;
  
  // Enable, clear and reset FIFOs
  UART->FCR = 0x0;
  UTIL_waitLoop(100);

  UART->FCR = 0xC7;
  UTIL_waitLoop(100);

  // Disable autoflow control
  UART->MCR = 0x00;
  UTIL_waitLoop(100);
                                                                               
  // Set word length to 8 bits, no parity, one stop bit, clear DLAB bit                                 
  UART->LCR = 0x03;
  UTIL_waitLoop(100); 
                                                                                
  // Enable interrupts                                                         
  UART->IER = 0x07;
  UTIL_waitLoop(100); 

  // Set to run free
  //UART->PWREMU_MGMT |= 0x6001;
  //UTIL_waitLoop(100); 

  if ( (UART->IIR & 0xC0) != 0xC0 )
  {
    return E_FAIL;
  }
                                                                               
  UTIL_waitLoop(100);
  
  return E_PASS;
}


/***********************************************************
* End file                                                 *
***********************************************************/
