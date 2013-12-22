/*
 * sdmmc.h
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
  FILE      : sdmmc.h
  PROJECT   : TI Booting and Flashing Utilities
  AUTHOR    : Daniel Allred
  DESC      : Header file for the generic low-level SDMMC driver
-------------------------------------------------------------------------- */

#ifndef _SDMMC_H_
#define _SDMMC_H_

#include "tistdtypes.h"

// Prevent C++ name mangling
#ifdef __cplusplus
extern far "c" {
#endif

/************************************************************
* Global Macro Declarations                                 *
************************************************************/

// Direction of DMA Transfer
#define SDMMC_FROM_MMC          0           // MMCIF to SDRAM
#define SDMMC_TO_MMC            1           // SDRAM to MMCIF

// Command Macros
#define SDMMC_CMD0                      (0x0000u)
#define SDMMC_CMD1                      (0x0001u)
#define SDMMC_CMD2                      (0x0002u)
#define SDMMC_CMD3                      (0x0003u)
#define SDMMC_CMD4                      (0x0004u)
#define SDMMC_CMD5                      (0x0005u)
#define SDMMC_CMD6                      (0x0006u)
#define SDMMC_CMD7                      (0x0007u)
#define SDMMC_CMD8                      (0x0008u)
#define SDMMC_CMD9                      (0x0009u)
#define SDMMC_CMD10                     (0x000Au)
#define SDMMC_CMD11                     (0x000Bu)
#define SDMMC_CMD12                     (0x000Cu)
#define SDMMC_CMD13                     (0x000Du)
#define SDMMC_CMD14                     (0x000Eu)
#define SDMMC_CMD15                     (0x000Fu)
#define SDMMC_CMD16                     (0x0010u)
#define SDMMC_CMD17                     (0x0011u)
#define SDMMC_CMD18                     (0x0012u)
#define SDMMC_CMD19                     (0x0013u)
#define SDMMC_CMD20                     (0x0014u)
#define SDMMC_CMD21                     (0x0015u)
#define SDMMC_CMD22                     (0x0016u)
#define SDMMC_CMD23                     (0x0017u)
#define SDMMC_CMD24                     (0x0018u)
#define SDMMC_CMD25                     (0x0019u)
#define SDMMC_CMD26                     (0x001Au)
#define SDMMC_CMD27                     (0x001Bu)
#define SDMMC_CMD28                     (0x001Cu)
#define SDMMC_CMD29                     (0x001Du)
#define SDMMC_CMD30                     (0x001Eu)
#define SDMMC_CMD31                     (0x001Fu)
#define SDMMC_CMD32                     (0x0020u)
#define SDMMC_CMD33                     (0x0021u)
#define SDMMC_CMD34                     (0x0022u)
#define SDMMC_CMD35                     (0x0023u)
#define SDMMC_CMD36                     (0x0024u)
#define SDMMC_CMD37                     (0x0025u)
#define SDMMC_CMD38                     (0x0026u)
#define SDMMC_CMD39                     (0x0027u)
#define SDMMC_CMD40                     (0x0028u)
#define SDMMC_CMD41                     (0x0029u)
#define SDMMC_CMD42                     (0x002Au)
#define SDMMC_CMD43                     (0x002Bu)
#define SDMMC_CMD44                     (0x002Cu)
#define SDMMC_CMD45                     (0x002Du)
#define SDMMC_CMD46                     (0x002Eu)
#define SDMMC_CMD47                     (0x002Fu)
#define SDMMC_CMD48                     (0x0030u)
#define SDMMC_CMD49                     (0x0031u)
#define SDMMC_CMD50                     (0x0032u)
#define SDMMC_CMD51                     (0x0033u)
#define SDMMC_CMD52                     (0x0034u)
#define SDMMC_CMD53                     (0x0035u)
#define SDMMC_CMD54                     (0x0036u)
#define SDMMC_CMD55                     (0x0037u)
#define SDMMC_CMD56                     (0x0038u)
#define SDMMC_CMD57                     (0x0039u)
#define SDMMC_CMD58                     (0x003Au)
#define SDMMC_CMD59                     (0x003Bu)
#define SDMMC_CMD60                     (0x003Cu)
#define SDMMC_CMD61                     (0x003Du)
#define SDMMC_CMD62                     (0x003Eu)
#define SDMMC_CMD63                     (0x003Fu)

// Response Macros
#define SDMMC_RSPNONE                   (0x0000u)
#define SDMMC_RSP1                      (0x0200u)
#define SDMMC_RSP2                      (0x0400u)
#define SDMMC_RSP3                      (0x0600u)
#define SDMMC_RSP4                      SDMMC_RSP1
#define SDMMC_RSP5                      SDMMC_RSP1
#define SDMMC_RSP6                      SDMMC_RSP1


/***********************************************************
* Global Typedef declarations                              *
***********************************************************/

// Endianness Select
typedef enum _SDMMC_ENDIANNESS_
{
  SDMMC_LITTLE_ENDIAN = 0x00,
  SDMMC_BIG_ENDIAN    = 0x01
}
SDMMC_Endianness;

// Data Bus Width
typedef enum _SDMMC_DATABUS_WIDTH_
{
  SDMMC_1BIT_DATABUS = 0x00,
  SDMMC_4BIT_DATABUS = 0x01,
  SDMMC_8BIT_DATABUS = 0x02
}
SDMMC_DatabusWidth;

typedef enum _SDMMC_FIFO_THRESHOLD_
{
  SDMMC_FIFO_LEVEL_LOW =  0x00,  // 16 or 32 bytes
  SDMMC_FIFO_LEVEL_HIGH = 0x01   // 32 or 64 Bytes
}
SDMMC_FifoThreshold;

typedef enum _SDMMC_FIFO_DIRECTION_
{
  SDMMC_FIFO_DIRECTION_TX = 0x00,  // Transmit
  SDMMC_FIFO_DIRECTION_RX = 0x01   // Receive
}
SDMMC_FifoDirection;

typedef enum _SDMMC_MODES_
{
  SDMMC_MODE_NATIVE = 0x00,
  SDMMC_MODE_SPI    = 0x01
}
SDMMC_Mode;

// DAT3 Edge Detection select
typedef enum _SDMMC_DAT3_EDGE_DETECT_TYPE_
{
  SDMMC_DAT3_EDGE_DETECT_DISABLE    = 0,   // DAT3 edge detection is disabled
  SDMMC_DAT3_EDGE_DETECT_RISE       = 1,   // DAT3 rising edge detection is enabled
  SDMMC_DAT3_EDGE_DETECT_FALL       = 2,   // DAT3 falling edge detection is enabled
  SDMMC_DAT3_EDGE_DETECT_BOTH       = 3    // DAT3 both edge detection is enabled
}
SDMMC_Dat3EdgeDetectType;

// Status bits of Register - Status0
typedef enum _SDMMC_STATUS0_FLAGS_
{
  SDMMC_STATUS0_FLAG_DATDNE         = 0x0001,      // Data Done Status
  SDMMC_STATUS0_FLAG_BSYDNE         = 0x0002,      // Busy Done Status
  SDMMC_STATUS0_FLAG_RSPDNE         = 0x0004,      // Command / Response Done Status
  SDMMC_STATUS0_FLAG_TOUTRD         = 0x0008,      // Time-Out ( read data ) Status
  SDMMC_STATUS0_FLAG_TOUTRS         = 0x0010,      // Time-Out ( response ) Status
  SDMMC_STATUS0_FLAG_CRCWR          = 0x0020,      // CRC error ( write data ) Status
  SDMMC_STATUS0_FLAG_CRCRD          = 0x0040,      // CRC error ( read data ) Status
  SDMMC_STATUS0_FLAG_CRCRS          = 0x0080,      // CRC error ( response ) Status
  SDMMC_STATUS0_FLAG_SPIERR         = 0x0100,      // Data Error ( in SPI mode ) Status
  SDMMC_STATUS0_FLAG_DXRDY          = 0x0200,      // Data Transmit Ready Status
  SDMMC_STATUS0_FLAG_DRRDY          = 0x0400,      // Data Receive Ready Status
  SDMMC_STATUS0_FLAG_DATED          = 0x0800       // DAT3 Edge Detect Status
}
SDMMC_Status0Flags;

// Status bits of Register - Status1
typedef enum _SDMMC_STATUS1_FLAGS_
{
  SDMMC_STATUS1_FLAG_BUSY           = 0x0001,      // Busy Status
  SDMMC_STATUS1_FLAG_CLKSTP         = 0x0002,      // Clock Stop Status
  SDMMC_STATUS1_FLAG_DXEMP          = 0x0004,      // Data transmit empty Status
  SDMMC_STATUS1_FLAG_DRFUL          = 0x0008,      // Data receive full Status
  SDMMC_STATUS1_FLAG_DAT3ST         = 0x0010,      // DAT3 Status
  SDMMC_STATUS1_FLAG_FIFOEMP        = 0x0020,      // FIFO empty status
  SDMMC_STATUS1_FLAG_FIFOFULL       = 0x0040      // FIFO full status
}
SDMMC_Status1Flags;

// Response information received from SDMMC
typedef struct _SDMMC_RESPONSE_
{
  Uint32  responseData[4];     // Response of the command
  Uint8   commandIndex;        // Command Index
}
SDMMC_ResponseObj, *SDMMC_ResponseHandle;

typedef struct _SDMMC_CONFIG_
{
  SDMMC_Endianness          writeEndian;        // Endian select enable while writing
  SDMMC_Endianness          readEndian;         // Endian select enable while reading
  SDMMC_Dat3EdgeDetectType  dat3Detect;         // DAT3 Edge detection
  Uint8                     initModeClockRate;  // Clock Rate Divider for initialization procedures
  Uint8                     dataModeClockRate;  // Clock Rate Divider for data access procedures
  SDMMC_DatabusWidth        busWidth;           // Data bus width (1, 4, or 8 data lines)
  Uint32                    timeoutResponse;    // Timeout value for response, range 0 to 255
                                                //   MMC CLK clock cycles for Native mode, for
                                                //   SPI mode timeout value is equal to this
                                                //   value multiplied by 8 MMC CLK clock cycles
  Uint32                    timeoutData;        // Time out value for data read, range from
                                                //   0 to 65535 MMC CLK clock cycles in native mode,
                                                //   for SPI mode timeout value is equal to this
                                                //   value multiplied by 8 MMC CLK clock cycles
  SDMMC_FifoThreshold       fifoThreshold;      // To set the FIFO depth level (high or low)
}
SDMMC_ConfigObj, *SDMMC_ConfigHandle;

// _SDMMC_INFO_ structure - holds pertinent info for open driver instance
typedef struct _SDMMC_INFO_
{
  void                *regs;
  SDMMC_ResponseObj   response;
  Uint32              dataBytesPerBlk;
  Uint32              dataBytesPerBlkPower2;
  Uint8               dataBytesPerOp; 
  Uint8               dataBytesPerOpPower2;   
  Uint8               numOpsPerBlk;
  SDMMC_ConfigHandle  hSDMMCCfg;
} 
SDMMC_InfoObj, *SDMMC_InfoHandle;


/************************************************************
* Global Function Declarations                              *
************************************************************/

extern __FAR__ SDMMC_InfoHandle   SDMMC_open(Uint32 sdmmcPeripheralNum, SDMMC_ConfigHandle hSDMMCCfg);

extern __FAR__ Uint32             SDMMC_sendCmd(SDMMC_InfoHandle hSDMMCInfo, Uint32 command, Uint32 argument, Uint32 timeOut);

extern __FAR__ void               SDMMC_clearResponse(SDMMC_InfoHandle hSDMMCInfo);
extern __FAR__ void               SDMMC_getResponse(SDMMC_InfoHandle hSDMMCInfo, SDMMC_ResponseHandle hResponse);

extern __FAR__ void               SDMMC_setDataWidth(SDMMC_InfoHandle hSDMMCInfo, SDMMC_DatabusWidth width);
extern __FAR__ void               SDMMC_setBlockSize(SDMMC_InfoHandle hSDMMCInfo, Uint16 size);
extern __FAR__ void               SDMMC_setBlockCount(SDMMC_InfoHandle hSDMMCInfo, Uint16 numBlks);
extern __FAR__ void               SDMMC_setClockRate(SDMMC_InfoHandle hSDMMCInfo, Uint8 rate);
extern __FAR__ void               SDMMC_setFifoLevel(SDMMC_InfoHandle hSDMMCInfo, SDMMC_FifoThreshold level);
extern __FAR__ void               SDMMC_setFifoDirection(SDMMC_InfoHandle hSDMMCInfo, SDMMC_FifoDirection direction);
extern __FAR__ void               SDMMC_setResponseTimeout(SDMMC_InfoHandle hSDMMCInfo, Uint32 timeout);
extern __FAR__ void               SDMMC_setDataTimeout(SDMMC_InfoHandle hSDMMCInfo, Uint32 timeout);

extern __FAR__ Uint32             SDMMC_readNWords(SDMMC_InfoHandle hSDMMCInfo, Uint32 *data, Uint32 numofBytes);
#ifndef USE_IN_ROM
extern __FAR__ Uint32             SDMMC_writeNWords(SDMMC_InfoHandle hSDMMCInfo, Uint32 *data, Uint32 numofBytes);
#endif



/***********************************************************
* End file                                                 *
***********************************************************/

#ifdef __cplusplus
}
#endif

#endif //_SDMMC_H_

