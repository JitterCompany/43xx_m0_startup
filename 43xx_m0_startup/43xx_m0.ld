INCLUDE "link.ld"

ENTRY(ResetISR)

SECTIONS
{
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data)) ;
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = . ;
        

        *(.after_vectors*)
        
    } >Flash_M0
    
    .text : ALIGN(4)    
    {
         *(.text*)
        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);
        
    } > Flash_M0

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    } > Flash_M0
    __exidx_start = .;
    
    .ARM.exidx : ALIGN(4)
    {
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > Flash_M0
    __exidx_end = .;
    
    _etext = .;
         
    
	/* MAIN DATA SECTION */
    

    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RAM_M0

	
	/* Main DATA section (RAM_M0) */
	.data : ALIGN(4)
	{
	   FILL(0xff)
	   _data = . ;
	   *(vtable)
	   *(.data*)
	   . = ALIGN(4) ;
	   _edata = . ;
	} > RAM_M0 AT>Flash_M0


    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RAM_M0
        
    
    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RAM_M0
    
    PROVIDE(_pvHeapStart = .);
    PROVIDE(_vStackTop = __top_RAM_M0 - 0);

    PROVIDE(__valid_user_code_checksum = 0 - 
        (_vStackTop 
        + (ResetISR + 1) 
        + (NMI_Handler + 1) 
        + (HardFault_Handler + 1)
        + (0)
        + (0)
        + (0)
        )
    );
}
