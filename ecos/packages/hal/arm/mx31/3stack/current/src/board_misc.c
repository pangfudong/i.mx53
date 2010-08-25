//==========================================================================
//
//      board_misc.c
//
//      HAL misc board support code for the board
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with eCos; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
// As a special exception, if other files instantiate templates or use macros
// or inline functions from this file, or you compile this file and link it
// with other works to produce a work based on this file, this file does not
// by itself cause the resulting work to be covered by the GNU General Public
// License. However the source code for this file must still be made available
// in accordance with section (3) of the GNU General Public License.
//
// This exception does not invalidate any other reasons why a work based on
// this file might be covered by the GNU General Public License.
//
// Alternative licenses for eCos may be arranged by contacting Red Hat, Inc.
// at http://sources.redhat.com/ecos/ecos-license/
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//========================================================================*/

#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include <redboot.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_arch.h>           // Register state info
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/hal_intr.h>           // Interrupt names
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_soc.h>            // Hardware definitions
#include <cyg/hal/fsl_board.h>          // Platform specifics

#include <cyg/infra/diag.h>             // diag_printf

// All the MM table layout is here:
#include <cyg/hal/hal_mm.h>

externC void* memset(void *, int, size_t);

void hal_mmu_init(void)
{
    unsigned long ttb_base = RAM_BANK0_BASE + 0x4000;
    unsigned long i;

    /*
     * Set the TTB register
     */
    asm volatile ("mcr  p15,0,%0,c2,c0,0" : : "r"(ttb_base) /*:*/);

    /*
     * Set the Domain Access Control Register
     */
    i = ARM_ACCESS_DACR_DEFAULT;
    asm volatile ("mcr  p15,0,%0,c3,c0,0" : : "r"(i) /*:*/);

    /*
     * First clear all TT entries - ie Set them to Faulting
     */
    memset((void *)ttb_base, 0, ARM_FIRST_LEVEL_PAGE_TABLE_SIZE);

    /*              Actual   Virtual  Size   Attributes                                                 Function  */
    /*              Base     Base     MB     cached?          buffered?         access permissions                */
    /*              xxx00000 xxx00000                                                                             */
    X_ARM_MMU_SECTION(0x000, 0xF00,   0x1,   ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* ROM */
    X_ARM_MMU_SECTION(0x1FF, 0x1FF,   0x1,   ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* IRAM */
    X_ARM_MMU_SECTION(0x300, 0x300,   0x1,   ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* L2CC */
    X_ARM_MMU_SECTION(0x43F, 0x43F,   0x3C1, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* Internal Regsisters upto SDRAM*/
    X_ARM_MMU_SECTION(0x800, 0x000,   0x80,  ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* SDRAM */
    X_ARM_MMU_SECTION(0x800, 0x800,   0x80,  ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* SDRAM */
    X_ARM_MMU_SECTION(0xA00, 0xA00,   0x20,  ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* Flash */
    X_ARM_MMU_SECTION(0xB40, 0xB40,   0x10,  ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* External I/O */
    X_ARM_MMU_SECTION(0xB50, 0xB50,   0x8,   ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* PSRAM */
    X_ARM_MMU_SECTION(0xB60, 0xB60,   0x10,  ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* External I/O */
    X_ARM_MMU_SECTION(0xB80, 0xB80,   0x10,  ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* EIM control*/
}

//
// Platform specific initialization
//

unsigned int g_clock_src;
extern int g_board_type;

void plf_hardware_init(void)
{
    unsigned long val = readl(CCM_BASE_ADDR + CLKCTL_CCMR);

    if ((val & 0x6) == 0x4) {
        g_clock_src = FREQ_26MHZ;
    } else if ((val & 0x6) == 0x2) {
        g_clock_src = FREQ_32768HZ;
    }

    /* Reset interrupt status reg */
    writew(0x1F, PBC_INT_REST);
    writew(0x00, PBC_INT_REST);
    writew(0xFFFF, PBC_INT_MASK);
    // UART1
    writel(0x1210, IOMUXC_BASE_ADDR + 0x80);

    g_board_type = BOARD_TYPE_3STACK;
}

#include CYGHWR_MEMORY_LAYOUT_H

typedef void code_fun(void);

void board_program_new_stack(void *func)
{
    register CYG_ADDRESS stack_ptr asm("sp");
    register CYG_ADDRESS old_stack asm("r4");
    register code_fun *new_func asm("r0");
    old_stack = stack_ptr;
    stack_ptr = CYGMEM_REGION_ram + CYGMEM_REGION_ram_SIZE - sizeof(CYG_ADDRESS);
    new_func = (code_fun*)func;
    new_func();
    stack_ptr = old_stack;
}

static void display_clock_src(void)
{
    diag_printf("\n");
    if (g_clock_src == FREQ_27MHZ) {
        diag_printf("Clock input is 27 MHz");
    } else if (g_clock_src == FREQ_26MHZ) {
        diag_printf("Clock input is 26 MHz");
    } else if (g_clock_src == FREQ_32768HZ) {
        diag_printf("Clock input is 32KHz");
    } else {
        diag_printf("Unknown clock input source. Something is wrong!");
    }
}
RedBoot_init(display_clock_src, RedBoot_INIT_LAST);

// ------------------------------------------------------------------------
