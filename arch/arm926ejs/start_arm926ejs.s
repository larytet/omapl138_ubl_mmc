/*
;
; start_arm926ejs.asm
;

;
; Copyright (C) 2007-2011 Texas Instruments Incorporated - http://www.ti.com/ 
;



;
;  Redistribution and use in source and binary forms, with or without 
;  modification, are permitted provided that the following conditions 
;  are met:
; 
;     Redistributions of source code must retain the above copyright 
;     notice, this list of conditions and the following disclaimer.
; 
;     Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the 
;     documentation and/or other materials provided with the   
;     distribution.
; 
;     Neither the name of Texas Instruments Incorporated nor the names of
;     its contributors may be used to endorse or promote products derived
;     from this software without specific prior written permission.
; 
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; 
*/

    .global stack
    .sect   ".text:.start"
    .global __STACK_SIZE
    .global main
    .global start

start:
	# set the cpu to SVC32 mode
    mrs r0,cpsr
    bic	r0,r0,#0x1f
    orr	r0,r0,#0xd3
    msr	cpsr,r0

stack_setup:
    # Set up the stack
    ldr	r0, stackptr
    ldr r1, stacksize
    add	r0, r0, r1
    sub	sp, r0, #4
    # eabi requires align to 8 bytes
    bic sp, sp, #7

    # Load the Kernel entry point address
    ldr	r0, main_entry

    # Jump to Entry Point
    mov pc, r0
    
abort:
    b   abort

stackptr:
    .word stack
stacksize:
    .word 0x800
main_entry:
    .word  main
