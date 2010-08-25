//==========================================================================
//
//      redboot_cmds.c
//
//      Board [platform] specific RedBoot commands
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
#include <redboot.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/plf_mmap.h>
#include <cyg/hal/fsl_board.h>          // Platform specific hardware definitions

#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#include <flash_config.h>

#if (REDBOOT_IMAGE_SIZE != CYGBLD_REDBOOT_MIN_IMAGE_SIZE)
#error REDBOOT_IMAGE_SIZE != CYGBLD_REDBOOT_MIN_IMAGE_SIZE
#endif

RedBoot_config_option("Board specifics",
                      brd_specs,
                      ALWAYS_ENABLED,
                      true,
                      CONFIG_INT,
                      0
                     );
#endif  //CYGSEM_REDBOOT_FLASH_CONFIG

char HAL_PLATFORM_EXTRA[60] = "MX31 3-Stack (Freescale i.MX31 based) PASS 1.0 [x32 DDR]";

static void runImg(int argc, char *argv[]);

RedBoot_cmd("run",
            "Run an image at a location with MMU off",
            "[<virtual addr>]",
            runImg
           );

void launchRunImg(unsigned long addr)
{
    asm volatile ("mov r12, r0;");
    HAL_CLEAN_INVALIDATE_L2();
    HAL_DISABLE_L2();
    HAL_MMU_OFF();
    asm volatile (
                 "mov r0, #0;"
                 "mov r1, r12;"
                 "mov r11, #0;"
                 "mov r12, #0;"
                 "mrs r10, cpsr;"
                 "bic r10, r10, #0xF0000000;"
                 "msr cpsr_f, r10;"
                 "mov pc, r1"
                 );
}

extern unsigned long entry_address;

static void runImg(int argc,char *argv[])
{
    unsigned int virt_addr, phys_addr;

    // Default physical entry point for Symbian
    if (entry_address == 0xFFFFFFFF)
        virt_addr = 0x800000;
    else
    virt_addr = entry_address;

    if (!scan_opts(argc,argv,1,0,0,(void*)&virt_addr,
                   OPTION_ARG_TYPE_NUM, "virtual address"))
        return;

    if (entry_address != 0xFFFFFFFF)
        diag_printf("load entry_address=0x%lx\n", entry_address);
    HAL_VIRT_TO_PHYS_ADDRESS(virt_addr, phys_addr);

    diag_printf("virt_addr=0x%x\n",virt_addr);
    diag_printf("phys_addr=0x%x\n",phys_addr);

    launchRunImg(phys_addr);
}

#if defined(CYGSEM_REDBOOT_FLASH_CONFIG) && defined(CYG_HAL_STARTUP_ROMRAM)

RedBoot_cmd("romupdate",
            "Update Redboot with currently running image",
            "",
            romupdate
           );

extern int flash_program(void *_addr, void *_data, int len, void **err_addr);
extern int flash_erase(void *addr, int len, void **err_addr);
extern char *flash_errmsg(int err);
extern unsigned char *ram_end; //ram end is where the redboot starts FIXME: use PC value

#ifdef CYGPKG_IO_FLASH
void romupdate(int argc, char *argv[])
{
    void *err_addr, *base_addr;
    int stat;

    base_addr = (void*)0;
    diag_printf("Updating ROM in NAND flash\n");

    // Erase area to be programmed
    if ((stat = flash_erase((void *)base_addr,
                            CYGBLD_REDBOOT_MIN_IMAGE_SIZE,
                            (void **)&err_addr)) != 0) {
        diag_printf("Can't erase region at %p: %s\n",
                    err_addr, flash_errmsg(stat));
        return;
    }
    // Now program it
    if ((stat = flash_program((void *)base_addr, (void *)ram_end,
                              CYGBLD_REDBOOT_MIN_IMAGE_SIZE,
                              (void **)&err_addr)) != 0) {
        diag_printf("Can't program region at %p: %s\n",
                    err_addr, flash_errmsg(stat));
    }
}
RedBoot_cmd("factive",
            "Enable one flash media for Redboot",
            "[NOR | NAND]",
            factive
           );

void factive(int argc, char *argv[])
{
    unsigned long phys_addr;

    if (argc != 2) {
        diag_printf("Invalid factive cmd\n");
        return;
    }

    if (strcasecmp(argv[1], "NOR") == 0) {
#ifndef MXCFLASH_SELECT_NOR
        diag_printf("Not supported\n");
        return;
#else
        MXC_ASSERT_NOR_BOOT();
#endif
    } else if (strcasecmp(argv[1], "NAND") == 0) {
#ifndef MXCFLASH_SELECT_NAND
        diag_printf("Not supported\n");
        return;
#else
        MXC_ASSERT_NAND_BOOT();
#endif
    } else {
        diag_printf("Invalid command: %s\n", argv[1]);
        return;
    }
    HAL_VIRT_TO_PHYS_ADDRESS(ram_end, phys_addr);

    launchRunImg(phys_addr);
}
#endif //CYGPKG_IO_FLASH
#endif /* CYG_HAL_STARTUP_ROMRAM */
