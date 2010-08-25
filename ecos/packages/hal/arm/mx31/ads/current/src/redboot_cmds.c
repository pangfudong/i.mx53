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

char HAL_PLATFORM_EXTRA[60] = "MX31 ADS (Freescale i.MX31 based) PASS 1.0 [x32 DDR]";

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

#if CYGPKG_REDBOOT_NETWORKING
#define LAN_BASE    BOARD_CS_LAN_BASE

#define PP_EE_ADDR_W0   0x001C
#define PP_EE_ADDR_W1   0x001D
#define PP_EE_ADDR_W2   0x001E

extern cyg_uint16 read_eeprom(cyg_addrword_t base, cyg_uint16 offset);
extern void write_eeprom(cyg_addrword_t base, cyg_uint16 offset, cyg_uint16 data);

// Exported CLI function(s)
static void setMac(int argc, char *argv[]);
RedBoot_cmd("setmac",
            "Set Ethernet MAC address in EEPROM",
            "[0x##:0x##:0x##:0x##:0x##:0x##]",
            setMac
           );

const static unsigned short RESET_CONFIG_BLOCK[] = {
    0xA002,
    0x5E00,
};

#define DRIVER_CONFIG_BASE        0x1C   //Cirrus driver config base

static unsigned short g_drv_cfg_blk[] = {
    0xFFFF, //1C - MAC 4,5
    0xFFFF, //1D - MAC 2,3
    0xFFFF, //1E - MAC 0,1
    0x0000, //1F - ISA config
    0x0000, //20 - PP mem base
    0x0000, //21 - Boot PROM base
    0x0000, //22 - Boot PROM mask
    0x8040, //23 - Tx ctrl: Full duplex, media not required
    0x0021, //24 - Adapter config: 10Base-T, 10Base-T circuitry
    0x0001, //25 - EEPROM rev: 1.0
    0x0000, //26 - resvd
    0x0A2D, //27 - Mfg date
    0xFFFF, //28 - copy of 1C
    0xFFFF, //29 - copy of 1D
    0xFFFF, //2A - copy of 1E
    0x0000, //2B - resvd
    0x0000, //2C - resvd
    0x0000, //2D - resvd
    0x0000, //2E - resvd
    0x0000, //2F - Checksum
};

static void setMac(int argc,char *argv[])
{
    int i, ret, wsize = sizeof(g_drv_cfg_blk) / 2;  // word size
    unsigned char data[6];
    unsigned long temp;
    unsigned short ee_word[3];

    if (argc == 1) {
        ee_word[0] = read_eeprom(LAN_BASE, PP_EE_ADDR_W0 + 0);
        ee_word[1] = read_eeprom(LAN_BASE, PP_EE_ADDR_W0 + 1);
        ee_word[2] = read_eeprom(LAN_BASE, PP_EE_ADDR_W0 + 2);
        if (ee_word[0] == 0 && ee_word[1] == 0 && ee_word[2] == 0) {
            diag_printf("Can't read MAC address\n\n");
            return;
        }

        diag_printf("MAC address: ");
        diag_printf("0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n\n",
                    (ee_word[0] & 0x00FF), (ee_word[0] >> 8),
                    (ee_word[1] & 0x00FF), (ee_word[1] >> 8),
                    (ee_word[2] & 0x00FF), (ee_word[2] >> 8));
        return;
    }

    if (argc != 2) {
        ret = -1; goto error;
    }

    for (i = 0;  i < 6;  i++) {
        if (!parse_num(*(&argv[1]), &temp, &argv[1], ":")) {
            ret = -2; goto error;
        }
        if (temp > 0xFF) {
            ret = -3; goto error;
        }
        data[i] = temp & 0xFF;
    }

    g_drv_cfg_blk[0] = g_drv_cfg_blk[12] = *(unsigned short*)(&data[0]);
    g_drv_cfg_blk[1] = g_drv_cfg_blk[13] = *(unsigned short*)(&data[2]);
    g_drv_cfg_blk[2] = g_drv_cfg_blk[14] = *(unsigned short*)(&data[4]);

    // Calculate checksum
    temp = 0;
    for (i = 0; i < wsize-1; i++) {
        temp += g_drv_cfg_blk[i];
    }
    temp = (~temp + 1) & 0xFFFF;
    g_drv_cfg_blk[wsize-1] = temp;

    // Program the EEPROM
    // Reset config block first
    for (i = 0; i < sizeof(RESET_CONFIG_BLOCK)/2; i++) {
        write_eeprom(LAN_BASE, i, RESET_CONFIG_BLOCK[i]);
    }
    // Driver config block 2nd
    for (i = 0; i < wsize; i++) {
        write_eeprom(LAN_BASE, DRIVER_CONFIG_BASE+i,
                     g_drv_cfg_blk[i]);
    }
    return;
    error:
    diag_printf("Wrong value for setMac. Error=%d\n\n", ret);
}

//#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
RedBoot_cmd("eefun",
            "read/write a word into EEPROM",
            "[0-based IO Base offset:value]",
            eefun
           );

static void eefun(int argc,char *argv[])
{
    int i, ret;
    unsigned short data[2];
    unsigned int temp;

    if (argc != 2) {
        ret=-1; goto error;
    }

    for (i = 0;  i < 2;  i++) {
        if (!parse_num(*(&argv[1]), &temp, &argv[1], ":")) {
            ret=-2; goto error;
        }
        if (temp > 0xFFFF) {
            ret=-3; goto error;
        }
        data[i] = (unsigned short)temp;
    }

    if (data[0] >= 0x30 && data[0] != 0xFFFF) {
        ret=-4; goto error;
    }

    if (data[0] == 0xFFFF) {
        for (i = 0; i < 0x30; i++) {
            if (i % 8 == 0) diag_printf("0x%02x: ", i);
            diag_printf("%04x ", read_eeprom(LAN_BASE, i));
            if (i % 8 == 7) diag_printf("\n");
        }
        return;
    }

    diag_printf("writeEE() Offset: 0x%x, value=0x%x\n", data[0], data[1]);
    write_eeprom(LAN_BASE, data[0], data[1]);
    diag_printf("Reading back: 0x%x\n\n", read_eeprom(LAN_BASE, data[0]));
    return;
    error:
    diag_printf("Wrong value %d for writeEE\n\n", ret);
}
#endif //EEPROM_DEBUG

#endif //CYGPKG_REDBOOT_NETWORKING

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

    if (IS_FIS_FROM_NAND()) {
        base_addr = (void*)0;
        diag_printf("Updating ROM in NAND flash\n");
    } else if (IS_FIS_FROM_NOR()) {
        base_addr = (void*)BOARD_FLASH_START;
        diag_printf("Updating ROM in NOR flash\n");
    } else {
        diag_printf("romupdate not supported\n");
        diag_printf("Use \"factive [NOR|NAND]\" to select either NOR or NAND flash\n");
        return;
    }
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
