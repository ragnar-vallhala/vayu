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

    /* Reset copy/zero tables walked by NavHAL's Reset_Handler (boot.c). NavHAL
       0.3.x moved to this CMSIS-style table-driven startup, so a linker script
       that omits them does not fail to boot -- it fails to LINK, on
       __copy_table_start__ and friends. Kept inside the read-only load segment
       so no spurious RW segment appears. Copy triples {flash-src, ram-dst,
       byte-len}; zero pairs {ram-dst, byte-len}. Matches NavHAL's own
       src/arch/armv7e-m/link/cortex-m4.ld. */
    . = ALIGN(4);
    __copy_table_start__ = .;
    LONG(_sidata); LONG(_sdata); LONG(_edata - _sdata);
    __copy_table_end__ = .;
    __zero_table_start__ = .;
    LONG(_sbss); LONG(_ebss - _sbss);
    __zero_table_end__ = .;

  } > FLASH

  /* Self-registering driver tables. Each entry is a const descriptor placed
   * by VAYU_SENSOR_DRIVER / VAYU_SENSOR_RIDE; the adapter walks the section
   * between the __start_/__stop_ symbols the linker provides for any section
   * with a C-identifier name (no leading dot -- that is what makes them
   * automatic, and it works the same on the host link, so SITL walks the same
   * table).
   *
   * The start/stop symbols are PROVIDEd by hand here, and must be: ld only
   * invents them for ORPHAN sections, so naming a section in SECTIONS at all
   * takes that service away. Placing it without providing them fails the link.
   *
   * Orphan placement would put these in FLASH on its own, so this block exists
   * for the KEEP: the moment anyone turns on --gc-sections (see the note by
   * VAYU_R12_WARN_FLAGS), nothing references these entries from code the
   * linker can see, and the registry would go quietly empty -- every sensor
   * "absent" at boot, with no build error anywhere. */
  .vayu_registries : {
    . = ALIGN(4);
    PROVIDE(__start_vayu_sensors = .);
    KEEP(*(vayu_sensors))
    PROVIDE(__stop_vayu_sensors = .);
    . = ALIGN(4);
    PROVIDE(__start_vayu_rides = .);
    KEEP(*(vayu_rides))
    PROVIDE(__stop_vayu_rides = .);
    . = ALIGN(4);
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


