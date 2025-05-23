/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Entry Point */
ENTRY(Reset_Handler)

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

_min_heap_size = 0;
_min_stack_size = 256;

/* Memories definition */
MEMORY {
    FLASH	(xrw) : ORIGIN = 0x08000000,    LENGTH = 512K
    SRAM    (xrw) : ORIGIN = 0x20000000,    LENGTH = 128K
}

/* Sections */
SECTIONS {
    /* The startup code into "FLASH" ROM type memory */
    .vectors : {
        . = ALIGN(4);
        KEEP(*(.vectors))       				/* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* The program code and other data into "FLASH" ROM type memory */
    .text : {
        . = ALIGN(4);
        *(.text)                				/* .text sections (code) */
        *(.text*)               				/* .text* sections (code) */
        *(.glue_7)              				/* glue arm to thumb code */
        *(.glue_7t)             				/* glue thumb to arm code */
        *(.eh_frame)
    
        KEEP (*(.init))
        KEEP (*(.fini))
    
        . = ALIGN(4);
        _etext = .;             				/* define a global symbols at end of code */
    } >FLASH

    /* Constant data into "FLASH" ROM type memory */
    .rodata : {
        . = ALIGN(4);
        *(.rodata)              				/* .rodata sections (constants, strings, etc.) */
        *(.rodata*)             				/* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >FLASH

    /* The READONLY keyword is only supported in GCC11 and later */
    .ARM.extab (READONLY) : {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >FLASH

    .ARM (READONLY) : {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } >FLASH

    .preinit_array (READONLY) : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .init_array (READONLY) : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .fini_array (READONLY) : {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data sections into "SRAM" RAM type memory */
    .data : {
        . = ALIGN(4);
        _sdata = .;             				/* create a global symbol at data start */
        *(.data)                				/* .data sections */
        *(.data*)               				/* .data* sections */
        *(.RamFunc)             				/* .RamFunc sections */
        *(.RamFunc*)            				/* .RamFunc* sections */
    
        . = ALIGN(4);
        _edata = .;             				/* define a global symbol at data end */
    } >SRAM AT> FLASH

    /* Uninitialized data section into "SRAM" RAM type memory */
    . = ALIGN(4);
    
    .bss : {
        _sbss = .;              				/* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
    
        . = ALIGN(4);
        _ebss = .;              				/* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } >SRAM
    
    /* User_heap_stack section, used to check that there is enough "SRAM" RAM type memory left */
    ._user_heap_stack : {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _min_heap_size;
        . = . + _min_stack_size;
        . = ALIGN(8);
    } >SRAM

    /* Remove information from the compiler libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { 
    	*(.ARM.attributes) 
    }
}
