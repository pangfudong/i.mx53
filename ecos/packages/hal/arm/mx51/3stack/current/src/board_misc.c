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
#include <cyg/hal/hal_soc.h>         // Hardware definitions
#include <cyg/hal/fsl_board.h>             // Platform specifics

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

    /*              Actual   Virtual  Size   Attributes                                                    Function  */
    /*              Base     Base     MB     cached?           buffered?        access permissions                 */
    /*              xxx00000 xxx00000                                                                                */
    X_ARM_MMU_SECTION(0x000, 0x200,   0x200, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* ROM */
    X_ARM_MMU_SECTION(0x100, 0x100,   0x001, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* IRAM */
    X_ARM_MMU_SECTION(0x400, 0x000,   0x080, ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* SDRAM */
    X_ARM_MMU_SECTION(0x400, 0x400,   0x080, ARM_CACHEABLE,   ARM_BUFFERABLE,   ARM_ACCESS_PERM_RW_RW); /* SDRAM */
    X_ARM_MMU_SECTION(0x7ff, 0x7ff,   0x001, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* NAND Flash buffer */
    X_ARM_MMU_SECTION(0x800, 0x800,   0x020, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* IPUv3D */
    X_ARM_MMU_SECTION(0xB00, 0xB00,   0x400, ARM_UNCACHEABLE, ARM_UNBUFFERABLE, ARM_ACCESS_PERM_RW_RW); /* periperals */
}

//
// Platform specific initialization
//

void plf_hardware_init(void)
{
    unsigned int v;

    v = 0x0040174A; // modified
    writel(v, NFC_FLASH_CONFIG2_REG);

    writel(0xFFFF0000, UNLOCK_BLK_ADD0_REG);
    writel(0xFFFF0000, UNLOCK_BLK_ADD1_REG);
    writel(0xFFFF0000, UNLOCK_BLK_ADD2_REG);
    writel(0xFFFF0000, UNLOCK_BLK_ADD3_REG);
    
    v = NFC_WR_PROT_CS0 | NFC_WR_PROT_BLS_UNLOCK | NFC_WR_PROT_WPC;
    writel(v, NFC_WR_PROT_REG);

    writel(0, NFC_IPC_REG);

#if 0
    /* PBC setup */
    //Enable UART transceivers also reset the Ethernet/external UART
    temp = readw(PBC_BASE + PBC_BCTRL1);

    writew(0x8023, PBC_BASE + PBC_BCTRL1);

    for (i = 0; i < 100000; i++) {
    }

    // clear the reset, toggle the LEDs
    writew(0xDF, PBC_BASE + PBC_BCTRL1_CLR);

    for (i = 0; i < 100000; i++) {
    }

    dummy = readb(0xB4000008);
    dummy = readb(0xB4000007);
    dummy = readb(0xB4000008);
    dummy = readb(0xB4000007);
#endif

#if 0
    /* Reset interrupt status reg */
    writew(0x1F, PBC_INT_REST);
    writew(0x00, PBC_INT_REST);
    writew(0xFFFF, PBC_INT_MASK);
#endif
    // UART1
    //RXD
    writel(0x0, IOMUXC_BASE_ADDR + 0x15C);
    writel(0x4, IOMUXC_BASE_ADDR + 0x604);
    writel(0x1C5, IOMUXC_BASE_ADDR + 0x3BC);

    //TXD
    writel(0x0, IOMUXC_BASE_ADDR + 0x160);
    writel(0x1C5, IOMUXC_BASE_ADDR + 0x3C0);

    //RTS
    writel(0x0, IOMUXC_BASE_ADDR + 0x164);
    writel(0x4, IOMUXC_BASE_ADDR + 0x600);
    writel(0x1C4, IOMUXC_BASE_ADDR + 0x3C4);

    //CTS
    writel(0x0, IOMUXC_BASE_ADDR + 0x168);
    writel(0x1C4, IOMUXC_BASE_ADDR + 0x3C8);
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

