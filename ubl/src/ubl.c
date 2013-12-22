/*
 * ubl.c
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
  FILE        : ubl.c                                                   
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : The main project file for the user boot loader
 ----------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// This module's header file 
#include "ubl.h"

// Device specific CSL
#include "device.h"

// Misc. utility function include
#include "util.h"

// Project specific debug functionality
#include "debug.h"

// Uart driver includes
#include "device_uart.h"
#include "uart.h"

#if defined(UBL_NOR)
// NOR driver include
#include "nor.h"
#include "norboot.h"

#elif defined(UBL_NAND)
// NAND driver include
#include "nand.h"
#include "nandboot.h"

#elif defined(UBL_ONENAND)
// OneNAND driver include
#include "onenand.h"
#include "onenandboot.h"

#elif defined(UBL_SDMMC)
// SD/MMC driver include
#include "sdmmc.h"
#include "sdmmcboot.h"
#include "uartboot.h"

#elif defined(UBL_SPI_MEM)
// SPI MEM driver include
#include "spi_mem.h"
#include "spi_memboot.h"

#elif defined(UBL_I2C_MEM)
// I2C MEM driver include
#include "i2c_mem.h"
#include "i2c_memboot.h"
#endif

/************************************************************
* Explicit External Declarations                            *
************************************************************/


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/

static Uint32 LOCAL_boot(void);
static void LOCAL_bootAbort(void);
static void (*APPEntry)(void);


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/

Uint32 gEntryPoint;
UART_InfoHandle hUartInfo;


/************************************************************
* Global Function Definitions                               *
************************************************************/

// Main entry point
void main(void)
{
  // Call to real boot function code
  if (LOCAL_boot() != E_PASS)
  {
    LOCAL_bootAbort();
  }
  else
  {
    APPEntry = (void (*)(void)) gEntryPoint;
    (*APPEntry)();  
  }
}


/************************************************************
* Local Function Definitions                                *
************************************************************/


static Uint32 LOCAL_bufcmp(char *s1, char *s2, int size)
{
	while (size--)
	{
		if (*(s1+size-1) != *(s2+size-1))
			return E_FAIL;
	}
	return E_PASS;
}

static int LOCAL_shiftRegisterReset(char *data, int dataMaxSize)
{
	int i;

	for (i = 0;i < dataMaxSize;i++)
	{
		data[i] = 0;
	}

	return 0;
}

static int LOCAL_shiftRegisterAddChar(char *data, int dataSize, int dataMaxSize, char c)
{
	int i;
	// if shift register is full - shift all characters left
	if (dataSize == dataMaxSize)
	{
		for (i = 0;i < (dataMaxSize-1);i++)
		{
			data[i] = data[i+1];
		}
	}
	else
	{
		dataSize += 1;
	}
	data[dataSize-1] = c;

	return dataSize;
}

static inline int LOCAL_shiftRegisterAddStr(char *data, int dataSize, int dataMaxSize, char *subStr, int subStrSize)
{
	int i, ret;

	for (i = 0;i < subStrSize;i++)
	{
		ret = LOCAL_shiftRegisterAddChar(data, dataSize, dataMaxSize, subStr[i]);
	}
	
	return ret;
}


static Uint32 LOCAL_shiftRegisterContains(char *data, int dataSize, char *subStr, int subStrSize)
{
	int i, j;
	Uint32 status = E_FAIL;
	for (i = 0;i < (dataSize-subStrSize+1);i++)
	{
		status = E_PASS;
		for (j = 0;j < subStrSize;j++)
		{
			if (subStr[j] != data[i+j])
			{
				status = E_FAIL;
				break;
			}
		}
		if (status == E_PASS)
			break;
	}
	
	
	return status;
}

static Uint32 LOCAL_boot(void)
{
  Uint32 status;
  char command[1] = {0};
  char commandShift[12] = {0};
  int commandShiftSize = 0;
  Uint32 commandLen;
  int loops = 0xFFFFF;

  // Set RAM pointer to beginning of RAM space
  UTIL_setCurrMemPtr(0);

  // Init device PLLs, PSCs, external memory, etc.
  status = DEVICE_init();
  
  // Open UART peripheral for sending out status
  if (status == E_PASS)
  {
    DEVICE_UARTInit(DEVICE_UART_PERIPHNUM);
    hUartInfo = UART_open(DEVICE_UART_PERIPHNUM, hDEVICE_UART_config);
    UART_reset(hUartInfo);
    DEVICE_UARTInit(DEVICE_UART_PERIPHNUM);
    hUartInfo = UART_open(DEVICE_UART_PERIPHNUM, hDEVICE_UART_config);
    DEBUG_printString((String)devString);
  }
  else
  {
    return E_FAIL;
  }


  // Send some information to host
  DEBUG_printString("\r\nUBL Version: ");
  DEBUG_printString(UBL_VERSION_STRING);
  DEBUG_printString("\r\n");
  UART_flushRx(hUartInfo);

  DEBUG_printString("UBL waits for user command boot/prog: ");
  while (1)
  {
	  commandLen = sizeof(command);
	  status = UART_checkRx(hUartInfo);
	  if (status == E_PASS)
	  {
		  status = UART_recvStringN(hUartInfo, command, &commandLen, 0);
	  }
	  if (--loops < 0)
	  {
		  DEBUG_printString("timeout");
	  }
  	  if ((status == E_PASS) || (loops < 0))
  	  {
  		  commandShiftSize = LOCAL_shiftRegisterAddStr(commandShift, commandShiftSize, sizeof(commandShift), command, sizeof(command));
  		  status = LOCAL_shiftRegisterContains(commandShift, commandShiftSize, "prog", 4);
  		  if (status == E_PASS)
  		  {
  	    	  commandShiftSize = LOCAL_shiftRegisterReset(commandShift, sizeof(commandShift));
  	    	  DEBUG_printString("\r\nProgram MMC\r\n");
  	    	  status = UARTBOOT_copy(hUartInfo);
  	    	  if (status != E_PASS)
  	    	  {
  	    		  DEBUG_printString("Program MMC failed\r\n");
  	    		  return E_FAIL;
  	    	  }
  	    	  DEBUG_printString("Program MMC done\r\n");
  		  }
  		  status = LOCAL_shiftRegisterContains(commandShift, commandShiftSize, "boot", 4);
  		  if ((status == E_PASS) || (loops < 0))
  		  {
  	    	  commandShiftSize = LOCAL_shiftRegisterReset(commandShift, sizeof(commandShift));
  	    	  DEBUG_printString("\r\nUBL boot from MMC\r\n");
  	    	  // Copy binary of application image from SD/MMC card to RAM
  	    	  status = SDMMCBOOT_copy();
  	    	  if (status != E_PASS)
  	    	  {
  	    		  DEBUG_printString("MMC Boot failed\r\n");
  	    		  return E_FAIL;
  	    	  }
  	    	  DEBUG_printString("Booting the image ...\r\n");
  	    	  break;
  		  }
  	  }
  }

  return E_PASS;    
}

static void LOCAL_bootAbort(void)
{
  DEBUG_printString("Aborting...\r\n");
  while (TRUE);
}

/************************************************************
* End file                                                  *
************************************************************/

