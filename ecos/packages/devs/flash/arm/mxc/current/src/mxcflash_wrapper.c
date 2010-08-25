//==========================================================================
//
//      mxcflash_wrapper.c
//
//      Flash programming wrapper to support both NOR and NAND flashes
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Kevin Zhang <k.zhang@freescale.com>
// Contributors: Kevin Zhang <k.zhang@freescale.com>
// Date:         2006-01-23
// Purpose:      
// Description:  
//              
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_misc.h>
#include <redboot.h>

extern void norflash_query(void* data);
extern int norflash_hwr_init(void);
extern int norflash_hwr_map_error(int e);
extern bool norflash_code_overlaps(void *start, void *end);
extern int norflash_erase_block(void* block, unsigned int size);
extern int norflash_program_buf(void* addr, void* data, int len);
extern int norflash_lock_block(void* block);
extern int norflash_unlock_block(void* block, int block_size, int blocks);

extern void nandflash_query(void* data);
extern int nandflash_hwr_init(void);
extern int nandflash_hwr_map_error(int e);
extern bool nandflash_code_overlaps(void *start, void *end);
extern int nandflash_erase_block(void* block, unsigned int size);
extern int nandflash_program_buf(void* addr, void* data, int len);
extern int nandflash_lock_block(void* block);
extern int nandflash_unlock_block(void* block, int block_size, int blocks);

static int mxc_flash_warning_done = 0;

void flash_query(void* data)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        norflash_query(data);
    } else if (IS_BOOTING_FROM_NAND() || IS_FIS_FROM_NAND()) {
        nandflash_query(data);
    } else {
        if (!mxc_flash_warning_done) {
            mxc_flash_warning_done = 1;
            diag_printf("1: Use \"factive [NOR|NAND]\" to select either NOR or NAND flash\n");
        }
    }
}

int flash_hwr_init(void)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_hwr_init();
    } else if (IS_BOOTING_FROM_NAND() || IS_FIS_FROM_NAND()) {
        return nandflash_hwr_init();
    } else {
        if (!mxc_flash_warning_done)
            mxc_flash_warning_done = 1;
        diag_printf("2: Use \"factive [NOR|NAND]\" to select either NOR or NAND flash\n");
        return -1;
    }
}

int flash_hwr_map_error(int e)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_hwr_map_error(e);
    } else {
        return nandflash_hwr_map_error(e);
    }
}

bool flash_code_overlaps(void *start, void *end)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_code_overlaps(start, end);
    } else {
        return nandflash_code_overlaps(start, end);
    }
}

int flash_erase_block(void* block, unsigned int size)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_erase_block(block, size);
    } else {
        return nandflash_erase_block(block, size);
    }
}

int flash_program_buf(void* addr, void* data, int len)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_program_buf(addr, data, len);
    } else {
        return nandflash_program_buf(addr, data, len);
    }
}

int flash_lock_block(void* block)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_lock_block(block);
    } else {
        return nandflash_lock_block(block);
    }
}

int flash_unlock_block(void* block, int block_size, int blocks)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        return norflash_unlock_block(block, block_size, blocks);
    } else {
        return nandflash_unlock_block(block, block_size, blocks);
    }
}

extern void mxc_nfc_print_info(void);

static void mxc_flash_print_info(void)
{
    if (IS_BOOTING_FROM_NOR()) {
        diag_printf("\nBooting from [NOR flash]\n");
        MXC_ASSERT_NOR_BOOT();
    } else if (IS_BOOTING_FROM_NAND()) {
        diag_printf("\nBooting from [NAND flash]\n");
        MXC_ASSERT_NAND_BOOT();
        mxc_nfc_print_info();
    } else if (IS_BOOTING_FROM_SDRAM()) {
        diag_printf("\nBooting from [SDRAM]\n");
    } else {
        diag_printf("\n!!!Warning: Unknown boot source !!!\n");
    }
    diag_printf("\n");
}

RedBoot_init(mxc_flash_print_info, RedBoot_INIT_LAST);
