/*
 * selfcopy.c
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
  FILE        : selfcopy.c                                                   
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : The XiP self-copy code for the user boot loader
 ----------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// The header for the boot entry point
#include "boot.h"

// This module's header file 
#include "selfcopy.h"

/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern __FAR__ Uint32 __selfcopysrc, __selfcopysrcend, __selfcopydest;


/************************************************************
* Local Macro Declarations                                  *
************************************************************/


/************************************************************
* Local Typedef Declarations                                *
************************************************************/


/************************************************************
* Local Function Declarations                               *
************************************************************/


/************************************************************
* Local Variable Definitions                                *
************************************************************/


/************************************************************
* Global Variable Definitions                               *
************************************************************/


/************************************************************
* Global Function Definitions                               *
************************************************************/

#if defined(__TMS470__)
  #pragma FUNC_EXT_CALLED(selfcopy);
  #pragma CODE_SECTION(selfcopy,".selfcopy");
  #pragma TASK(selfcopy);
#endif
void selfcopy(void)
{
  asm(" .global EXTERNAL_RAM_START");
  asm(" .global __selfcopysrc");
  asm(" .global __selfcopydest");
  asm(" .global __selfcopysrcend");
  asm(" .global _dramstart"); 
  asm(" .global _selfcopysrc"); 
  asm(" .global _selfcopydest"); 
  asm(" .global _selfcopysrcend"); 
  asm(" .global boot");  

  //Enable ITCM
  asm(" MRC p15,#0,r0,c9,c1,#1");
  asm(" MOV r0, #0x1");
  asm(" MCR p15,#0,r0,c9,c1,#1");
    
  //Enable DTCM
  asm(" MRC p15,#0,r0,c9,c1,#0");
  asm(" LDR r0,_dramstart");
  asm(" ORR r0,r0,#0x1D");
  asm(" MCR p15,#0,r0,c9,c1,#0");

  // Init copy variables
  asm(" LDR r0,_selfcopysrc");
  asm(" LDR r1,_selfcopydest");
  asm(" LDR r2,_selfcopysrcend");

  // Copy loop
  asm(" CMP  r2, r0");
  asm(" BLS  endcopyloop");
  asm(" SUB  r2, r2, r0");
  asm(" MOV  r2, r2, ASR #2");
  asm("copyloop:");
  asm(" LDR  r12, [R0], #0x4");
  asm(" SUBS r2, r2, #0x1");
  asm(" STR  r12, [r1], #0x4");
  asm(" BNE copyloop");
  asm("endcopyloop:");
  
  // Long jump to the normal entry point 
  asm(" LDR r0,_bootaddr");
  asm(" MOV pc,r0");

  asm("_dramstart: ");
  asm(" .word EXTERNAL_RAM_START");
  asm("_bootaddr: ");
  asm(" .word boot");
  asm("_selfcopysrc: ");
  asm(" .word __selfcopysrc");
  asm("_selfcopydest: ");
  asm(" .word __selfcopydest");
  asm("_selfcopysrcend: ");
  asm(" .word __selfcopysrcend");
}

/************************************************************
* Local Function Definitions                                *
************************************************************/


/************************************************************
* End file                                                  *
************************************************************/

