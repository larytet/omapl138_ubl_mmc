/*
 *  Desc: Linker command file sft application program.
 *  Auth: Daniel Allred
 */

ENTRY(start)
SECTIONS 
{
  PROVIDE(ASYNC_MEM_START     = 0x02000000);

  /* Internal SRAM for code and data (also used in the Makefile when AIS file is generated) */
  IRAM_START          = 0x80000000;
  IRAM_SIZE           = 0x00008000;

  /* 256MB of DDR total */
  PROVIDE(DDR_START           = 0xC0000000);
  PROVIDE(DDR_SIZE            = 0x10000000);


  /* 1MB in the end of DDR for dynamic allocation */

  PROVIDE(EXTERNAL_RAM_SIZE   = 0x00100000);
  PROVIDE(EXTERNAL_RAM_START  = (DDR_START + DDR_SIZE - EXTERNAL_RAM_SIZE));
  PROVIDE(EXTERNAL_RAM_END    = EXTERNAL_RAM_START + EXTERNAL_RAM_SIZE);

/*
  PROVIDE(EXTERNAL_RAM_SIZE   = 0x00004000);
  PROVIDE(EXTERNAL_RAM_START  = 0x80004000);
  PROVIDE(EXTERNAL_RAM_END    = EXTERNAL_RAM_START + EXTERNAL_RAM_SIZE);
*/
  
  PROVIDE(STACK_START         = 0x80008800);
  PROVIDE(STACK_SIZE          = 0x00000800);
  PROVIDE(stack                = STACK_START);
  
  . = IRAM_START;
  
  .text    : 
  {
    *(.text:.start)
    *(.text)
    . = ALIGN(4);
  }
  
  /* Move into DRAM for placing const and data sections */
  .rodata    :
  {
    *(.rodata*)
    *(.rodata)
    . = ALIGN(4);
  }  
  .data    : 
  {
    *(.data)
    . = ALIGN(4);
  }
        
  .bss    :
  {
    *(.bss) *(COMMON)
    . = ALIGN(4);
  }

  .init_array     :
  {
     PROVIDE_HIDDEN (__init_array_start = .);
     KEEP (*(SORT(.init_array.*)))
     KEEP (*(.init_array))
     PROVIDE_HIDDEN (__init_array_end = .);
  }

}

