#
# Copyright (c) 2010 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ..

# Microcontroller core: ARM9
#-mcpu=arm9tdmi -march=armv4t 
MARCH = armv4t
MCPU = arm9tdmi


# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET = ubl.out

# C source files located from the top-level source directory
CSRCS = \
       ubl/src/uartboot.c                  \
       ubl/src/sdmmcboot.c                  \
       ubl/src/ubl.c                  \
       src/util.c                  \
       omapl138/src/device.c    \
       omapl138/src/device_uart.c \
       omapl138/src/device_sdmmc.c    \
       drivers/src/uart.c          \
       drivers/src/sdmmc.c          \
       drivers/src/sdmmc_mem.c          \
       gnu/src/debug.c          \
       
#       arch/arm926ejs/src/boot.c 
                
       
       
       


# Assembler source files located from the top-level source directory
ASSRCS = arch/arm926ejs/start_arm926ejs.s \
 

# Include path located from the top-level source directory
INC_PATH = include \
            ubl/include \
            omapl138/include \
            gnu/include \
            drivers/include \
			arch/arm926ejs/include \


# Library paths from the top-level source directory
LIB_PATH =  

# Libraries to link with the project
LIBS = 

# Additional options for debugging. By default the common Makefile.in will
# add -gdwarf-2.
DBGFLAGS = 


# Debug build level
BUILD_DEBUG_LEVEL = 3

# Optimization settings
OPTIMIZATION = -O0

# Extra flags used when creating an EEPROM Intel HEX file. By default the
# common Makefile.in will add -j .eeprom
# --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0.
EEPROMFLAGS = 

# Extra flags used when creating an Intel HEX file. By default the common
# Makefile.in will add -R .eeprom -R .usb_descriptor_table.
FLASHFLAGS = 

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = -D UBL_SDMMC 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
CPPFLAGS =

# Extra flags${ProjDirPath} to use when linking
LDFLAGS =  -T arm_ubl.lds  

