/*
	ITCM SRAM: 64k, core only, instructions only, 64-bit
	DTCM SRAM: 128k, core only, data only, 64-bit
	AXI SRAM: 512k, 64-bit access to MDMA and Cortex M7 core
	SRAM1: 128k, 32-bit (domain D2)
	SRAM2: 128k, 32-bit (domain D2)
	SRAM3: 32k, 32-bit  (domain D2)
	SRAM4: 64k, 32-bit  (domain D3)

	Backup SRAM: 4k  (always powered by battery)

	SRAM1,SRAM2,SRAM3 form a contiguous, linear address space, so they
	can be joined together for simple usage
*/

MEMORY
{

  ram_axi    (rwx)      : ORIGIN = 0x24000000, LENGTH = 512K
/*  ram_sram1  (rwx)      : ORIGIN = 0x30000000, LENGTH = 128K
  ram_sram2  (rwx)      : ORIGIN = 0x30020000, LENGTH = 128K */
  ram_sram12 (rwx)      : ORIGIN = 0x30000000, LENGTH = 256K
  ram_sram3  (rwx)      : ORIGIN = 0x30040000, LENGTH = 32K
  ram_sram4  (rwx)      : ORIGIN = 0x38000000, LENGTH = 64K
  ram_dtcm   (rwx)      : ORIGIN = 0x20000000, LENGTH = 127K
  ram_itcm   (rwx)      : ORIGIN = 0x00000000, LENGTH = 63K
  ram_vectors(rwx)      : ORIGIN = 0x0000FC00, LENGTH = 1K  /* last 1K of ITCM dedicated for relocated vector table*/
  ram_backup (rwx)      : ORIGIN = 0x38800000, LENGTH = 4K

  stack(rwx)            : ORIGIN = 0x2407fff8, LENGTH = 0K /* In AXI SRAM*/

  rom_b1s0 (rx)         : ORIGIN = 0x08000000, LENGTH = 128K
  rom_b1s1 (rx)         : ORIGIN = 0x08020000, LENGTH = 128K
  rom_b1s234567 (rx)    : ORIGIN = 0x08040000, LENGTH = 768K

  rom_b2s0 (rx)         : ORIGIN = 0x08100000, LENGTH = 128K
  rom_b2s1 (rx)         : ORIGIN = 0x08120000, LENGTH = 128K
  rom_b2s234567 (rx)    : ORIGIN = 0x08140000, LENGTH = 768K

}

SECTIONS
{
    .nvic_vector : /* Interrupt vector */
    {
        *(.nvic_vector)
    } >ram_vectors AT>rom_b1s0

    . = ALIGN(8);

    .text : /* Code run from flash (not specifically designed to run from anywhere else) - runs slower, especially if instruction cache turned off*/
    {
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
    } >rom_b1s0

    . = ALIGN(8);

    _TEXT_ITCM_I_BEGIN = LOADADDR(.text_itcm);

    .text_itcm : /* Super fast (400MHz) core-coupled RAM for instructions only.*/
    {
        _TEXT_ITCM_BEGIN = .;
        *(.text_itcm)
	_TEXT_ITCM_END = .;
    } >ram_itcm AT>rom_b1s0

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : /* Initialized data - catch-all for anything not specifically designated elsewhere*/
    {
        _DATA_BEGIN = .;
        *(.data)
        *(.data.*)        
        _DATA_END = .;
    } >ram_axi AT>rom_b1s0

    . = ALIGN(8);

    .bss : /* Uninitialized data - catch-all for anything not specifically designated elsewhere*/
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram_axi 

    . = ALIGN(8);

    _DTCM_DATA_I_BEGIN = LOADADDR(.dtcm_data);

    .dtcm_data : /* Initialized data in fast DTCM (core-coupled only) memory */
    {
        _DTCM_DATA_BEGIN = .;
        *(.dtcm_data)
        *(.dtcm_data.*)        
        _DTCM_DATA_END = .;
    } >ram_dtcm AT>rom_b1s0

    . = ALIGN(8);

    .dtcm_bss :
    {
        _DTCM_BSS_BEGIN = .;
        *(.dtcm_bss)
        *(.dtcm_bss.*)
        _DTCM_BSS_END = .;
    } >ram_dtcm








/* SRAM1&SRAM2 are joined together as .sram1_data and .sram1_bss */

    . = ALIGN(8);

    _SRAM1_DATA_I_BEGIN = LOADADDR(.sram1_data);

    .sram1_data : 
    {
        _SRAM1_DATA_BEGIN = .;
        *(.sram1_data)
        *(.sram1_data.*)        
        _SRAM1_DATA_END = .;
    } >ram_sram12 AT>rom_b1s0

    . = ALIGN(8);

    .sram1_bss :
    {
        _SRAM1_BSS_BEGIN = .;
        *(.sram1_bss)
        *(.sram1_bss.*)
        _SRAM1_BSS_END = .;
    } >ram_sram12





/*


    . = ALIGN(8);

    _SRAM2_DATA_I_BEGIN = LOADADDR(.sram2_data);

    .sram2_data : 
    {
        _SRAM2_DATA_BEGIN = .;
        *(.sram2_data)
        *(.sram2_data.*)        
        _SRAM2_DATA_END = .;
    } >ram_sram2 AT>rom_b1s0

    . = ALIGN(8);

    .sram2_bss :
    {
        _SRAM2_BSS_BEGIN = .;
        *(.sram2_bss)
        *(.sram2_bss.*)
        _SRAM2_BSS_END = .;
    } >ram_sram2


*/


    . = ALIGN(8);

    _SRAM3_DATA_I_BEGIN = LOADADDR(.sram3_data);

    .sram3_data : 
    {
        _SRAM3_DATA_BEGIN = .;
        *(.sram3_data)
        *(.sram3_data.*)        
        _SRAM3_DATA_END = .;
    } >ram_sram3 AT>rom_b1s0

    . = ALIGN(8);

    .sram3_bss :
    {
        _SRAM3_BSS_BEGIN = .;
        *(.sram3_bss)
        *(.sram3_bss.*)
        _SRAM3_BSS_END = .;
    } >ram_sram3





    . = ALIGN(8);

    _SRAM4_DATA_I_BEGIN = LOADADDR(.sram4_data);

    .sram4_data : 
    {
        _SRAM4_DATA_BEGIN = .;
        *(.sram4_data)
        *(.sram4_data.*)        
        _SRAM4_DATA_END = .;
    } >ram_sram4 AT>rom_b1s0

    . = ALIGN(8);

    .sram4_bss :
    {
        _SRAM4_BSS_BEGIN = .;
        *(.sram4_bss)
        *(.sram4_bss.*)
        _SRAM4_BSS_END = .;
    } >ram_sram4




    . = ALIGN(8);

    _RAM_BACKUP_DATA_I_BEGIN = LOADADDR(.ram_backup_data);

    .ram_backup_data : 
    {
        _RAM_BACKUP_DATA_BEGIN = .;
        *(.ram_backup_data)
        *(.ram_backup_data.*)        
        _RAM_BACKUP_DATA_END = .;
    } >ram_backup AT>rom_b1s0

    . = ALIGN(8);

    .ram_backup_bss :
    {
        _RAM_BACKUP_BSS_BEGIN = .;
        *(.ram_backup_bss)
        *(.ram_backup_bss.*)
        _RAM_BACKUP_BSS_END = .;
    } >ram_backup




    . = ALIGN(8);

    .heap :
    {
        _HEAP = .;
    } >ram_axi

    .stack :
    {
        _STACKTOP = .;
    } >stack

    . = ALIGN(8);

/*
    _SETTINGS_I_BEGIN = LOADADDR(.settings);

    .settings :
    {
        _SETTINGS_BEGIN = .;
        *(.settings)
        *(.settings.*)
        _SETTINGS_END = .;
    } >ram_dtcm AT>rom_sect7
*/
}  
