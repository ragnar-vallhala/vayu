MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}



SECTIONS {
  
  .isr_vector : {
    KEEP(*(.isr_vector))
    . = ALIGN(4);   
  } > FLASH
  
  .text : {

    *(.text*)

    *(.rodata*)

  } > FLASH

  /* gcov (on-target coverage, C3): -fprofile-info-section emits each
   * instrumented TU's `struct gcov_info` here; coverage_dump.c walks the list
   * between these symbols. Empty (start == end) for non-coverage builds, so this
   * costs production nothing. The 64-bit counters are zero-init -> .bss/RAM. */
  .gcov_info : {
    . = ALIGN(4);
    PROVIDE(__gcov_info_start = .);
    KEEP(*(.gcov_info))
    PROVIDE(__gcov_info_end = .);
    . = ALIGN(4);
  } > FLASH

  __exidx_start = .;
    .ARM.exidx :
    {
      KEEP(*(.ARM.exidx*))
      KEEP(*(.gnu.linkonce.armexidx.*))
    } > FLASH
     __exidx_end = .;
  . = ALIGN(4);           
  .data :
  {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
    *(.init*)
    *(.fini*)
  } > RAM AT > FLASH

  .bss : {
    _sbss = .;
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = .;
    __bss_end__ = .;
    _heap_start = .;
  } > RAM
  _heap_end   = ORIGIN(RAM) + LENGTH(RAM);
  _sidata = LOADADDR(.data);
  _estack = ORIGIN(RAM) + LENGTH(RAM);
}


