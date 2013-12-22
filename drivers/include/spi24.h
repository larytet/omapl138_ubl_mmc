/*
 * spi24.h
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
    FILE        : spi24.h
    PURPOSE     : SPI24 header file
    PROJECT     : DaVinci User Boot-Loader and Flasher
    AUTHOR      : Texas Instruments Incorporated
    DATE	    : May-15-2007

    HISTORY
        v1.00 completion
 	        - May-15-2007
 	        
    Copyright 2007 Texas Instruments Incorporated. 	          
 	        
 ----------------------------------------------------------------------------- */
#ifndef _SPI24_H_
#define _SPI24_H_

#include <tistdtypes.h>
  typedef struct {
       VUint32 fileSize;
  } SPI24_INFO, *PSPI24_INFO;

// ---------------- Prototypes of functions for SPI24 Eeprom Write --------------------
  
Uint32 SPI24_Init(void);
Uint32 SPI24_WriteHeaderAndData(Uint8 *ramPtr);
Uint32 SPI24_ReadByte(Uint32 eeprom_addr);
Uint32 SPI24_WriteByte(Uint32 eeprom_addr, Uint8 *byte);
Uint32 SPI24_bulkErase(void);


#define SPI_READ_COMMAND            0x03000000u // Read data from EEPROM
#define SPI_PROG_COMMAND            0x02000000u // Program data to EEPROM
#define SPI_WREN_COMMAND            0x06000000u // Set EEPROM Write Enable
#define SPI_WSR_COMMAND             0x01000000u // Write to EEPROM status register
#define SPI_RSR_COMMAND             0x05000000u // Read EEPROM status register
#define SPI_BE_COMMAND              0xC7000000u // Bulk Erase Command	  
#endif
