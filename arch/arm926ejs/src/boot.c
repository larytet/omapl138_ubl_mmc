/*
 * boot.c
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
  FILE        : boot.c                                                   
  PROJECT     : TI Booting and Flashing Utilities
  AUTHOR      : Daniel Allred
  DESC        : ARM926EJ-S entry point functionality
 ----------------------------------------------------------------------------- */

// General type include
#include "tistdtypes.h"

// Device specific header file
#include "device.h"

// This module's header file 
#include "boot.h"


/************************************************************
* Explicit External Declarations                            *
************************************************************/

extern void main(void);


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

// Boot entry point to setup the C environment
#if defined(__TMS470__)
  #pragma TASK(boot);
  #pragma NO_HOOKS(boot);
  #pragma CODE_SECTION(boot,".boot");
#endif
void boot(void)
{
  asm(" .global STACK_START");
  asm(" .global _stack");
  asm(" .global main");  
  asm("__start:");
  asm(" NOP");
  asm(" MRS  r0, cpsr");
  asm(" BIC  r0, r0, #0x1F");       // CLEAR MODES
  asm(" ORR  r0, r0, #0x13");       // SET SUPERVISOR mode
  asm(" ORR  r0, r0, #0xC0");       // Disable FIQ and IRQ
  asm(" MSR  cpsr, r0");
  asm(" NOP");
  
  // Set the IVT to low memory, leave MMU & caches disabled
  asm(" MRC  p15,#0,r0,c1,c0,#0");
  asm(" BIC  r0,r0,#0x00002300");
  asm(" BIC  r0,r0,#0x00000087");
  asm(" ORR  r0,r0,#0x00000002");
  asm(" ORR  r0,r0,#0x00001000");  
  asm(" MCR  p15,#0,r0,c1,c0,#0");
  asm(" NOP");
  
  // Setup the stack pointer
  asm(" LDR  sp,_stack");
  asm(" SUB  sp,sp,#4");
  asm(" BIC  sp, sp, #7");
  
  // Call to main entry point
  main();
  
  asm("_stack:");
  asm(" .word STACK_START");
}

/************************************************************
* Local Function Definitions                                *
************************************************************/


/************************************************************
* End file                                                  *
************************************************************/
