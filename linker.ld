/*
	ITCM SRAM: 64k, core only, instructions only, 64-bit
	DTCM SRAM: 128k, core only, data only, 64-bit
	AXI SRAM: 512k, 64-bit access to MDMA and Cortex M7 core
	SRAM1: 128k, 32-bit (domain D2)
	SRAM2: 128k, 32-bit (domain D2)
	SRAM3: 32k, 32-bit  (domain D2)
	SRAM4: 64k, 32-bit  (domain D3)

	Backup SRAM: 4k
*/

MEMORY
{

  ram_axi    (rwx)      : ORIGIN = 0x24000000, LENGTH = 512K
  ram_sram1  (rwx)      : ORIGIN = 0x30000000, LENGTH = 128K
  ram_sram2  (rwx)      : ORIGIN = 0x30020000, LENGTH = 128K
  ram_sram3  (rwx)      : ORIGIN = 0x30040000, LENGTH = 32K
  ram_sram4  (rwx)      : ORIGIN = 0x38000000, LENGTH = 64K
  ram_dtcm   (rwx)      : ORIGIN = 0x20000000, LENGTH = 128K
  ram_itcm   (rwx)      : ORIGIN = 0x00000000, LENGTH = 64K
  ram_backup (rwx)      : ORIGIN = 0x38800000, LENGTH = 4K


  stack(rwx)            : ORIGIN = 0x2407fff8, LENGTH = 0K

  rom (rx)              : ORIGIN = 0x08000000, LENGTH = 896K
  rom_sect7 (rx)        : ORIGIN = 0x080e0000, LENGTH = 128K

}

SECTIONS
{
    .nvic_vector : 
    {
        *(.nvic_vector)    /* Vector table */
    } >rom

    . = ALIGN(8);

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)
    } >rom

    . = ALIGN(8);

    _TEXT_ITCM_I_BEGIN = LOADADDR(.text_itcm);

    .text_itcm :  /* Code run from 16K ITCM RAM block */ 
    {
        _TEXT_ITCM_BEGIN = .;
        *(.text_itcm)
	_TEXT_ITCM_END = .;
    } >ram_itcm AT>rom

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)
        *(.data.*)        
        _DATA_END = .;
    } >ram_main AT>rom

    . = ALIGN(8);

    _DTCM_DATA_I_BEGIN = LOADADDR(.dtcm_data);

    .dtcm_data : 
    {
        _DTCM_DATA_BEGIN = .;
        *(.dtcm_data)
        *(.dtcm_data.*)        
        _DTCM_DATA_END = .;
    } >ram_dtcm AT>rom

    . = ALIGN(8);

    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram_main 

    . = ALIGN(8);

    .dtcm_bss :
    {
        _DTCM_BSS_BEGIN = .;
        *(.dtcm_bss)
        *(.dtcm_bss.*)
        _DTCM_BSS_END = .;
    } >ram_dtcm

    . = ALIGN(8);

    .heap :
    {
        _HEAP = .;
    } >ram_main

    .stack :
    {
        _STACKTOP = .;
    } >stack

    . = ALIGN(8);

    _SETTINGS_I_BEGIN = LOADADDR(.settings);

    .settings :
    {
        _SETTINGS_BEGIN = .;
        *(.settings)
        *(.settings.*)
        _SETTINGS_END = .;
    } >ram_dtcm AT>rom_sect7

}  
