#ifndef _MXC_NFC_V2_H_
#define _MXC_NFC_V2_H_
//==========================================================================
//
//      mxc_nfc_v2.h
//
//      Flash programming to support NAND flash on Freescale MXC platforms
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

#include <pkgconf/devs_flash_onmxc.h>
#include "mxc_nand_specifics.h"

#define NFC_DEBUG_MIN   1
#define NFC_DEBUG_MED   2
#define NFC_DEBUG_MAX   3
#define NFC_DEBUG_DEF   NFC_DEBUG_MED

#define PG_2K_DATA_OP_MULTI_CYCLES()        false
extern int _mxc_boot;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned char u8;

#define ADDR_INPUT_SIZE                 8
//----------------------------------------------------------------------------
// Common device details.
#define FLASH_Read_ID                   (0x90)
#if CYGHWR_DEVS_FLASH_MXC_NAND_RESET_WORKAROUND
#define FLASH_Reset                     0xFFFF
#else
#define FLASH_Reset                     (0xFF)
#endif
#define FLASH_Read_Mode1                (0x00)
#define FLASH_Read_Mode1_LG             (0x30)
#define FLASH_Read_Mode2                (0x01)
#define FLASH_Read_Mode3                (0x50)
#define FLASH_Program                   (0x10)
#define FLASH_Send_Data                 (0x80)
#define FLASH_Status                    (0x70)
#define FLASH_Block_Erase               (0x60)
#define FLASH_Start_Erase               (0xD0)

#define NAND_MAIN_BUF0                  (NFC_BASE + 0x000)
#define NAND_MAIN_BUF1                  (NFC_BASE + 0x200)
#define NAND_MAIN_BUF2                  (NFC_BASE + 0x400)
#define NAND_MAIN_BUF3                  (NFC_BASE + 0x600)
#if defined (NFC_V2_0)
#define NAND_SPAR_BUF0                  (NFC_BASE + 0x800)
#define NAND_SPAR_BUF1                  (NFC_BASE + 0x810)
#define NAND_SPAR_BUF2                  (NFC_BASE + 0x820)
#define NAND_SPAR_BUF3                  (NFC_BASE + 0x830)
#define NAND_RESERVED                   (NFC_BASE + 0x840)
#elif defined (NFC_V2_1)
#define NAND_MAIN_BUF4                  (NFC_BASE + 0x800)
#define NAND_MAIN_BUF5                  (NFC_BASE + 0xA00)
#define NAND_MAIN_BUF6                  (NFC_BASE + 0xC00)
#define NAND_MAIN_BUF7                  (NFC_BASE + 0xE00)
#define NAND_SPAR_BUF0                  (NFC_BASE + 0x1000)
#define NAND_SPAR_BUF1                  (NFC_BASE + 0x1040)
#define NAND_SPAR_BUF2                  (NFC_BASE + 0x1080)
#define NAND_SPAR_BUF3                  (NFC_BASE + 0x10C0)
#define NAND_SPAR_BUF4                  (NFC_BASE + 0x1100)
#define NAND_SPAR_BUF5                  (NFC_BASE + 0x1140)
#define NAND_SPAR_BUF6                  (NFC_BASE + 0x1180)
#define NAND_SPAR_BUF7                  (NFC_BASE + 0x11C0)
#else
#error NOT supported
#endif

#define ECC_STATUS_RESULT_REG           (NAND_REG_BASE + 0x08)

enum nfc_internal_buf {
    RAM_BUF_0 = 0x0 << 4,
    RAM_BUF_1 = 0x1 << 4,
    RAM_BUF_2 = 0x2 << 4,
    RAM_BUF_3 = 0x3 << 4,
    RAM_BUF_4 = 0x4 << 4,
    RAM_BUF_5 = 0x5 << 4,
    RAM_BUF_6 = 0x6 << 4,
    RAM_BUF_7 = 0x7 << 4,
};

enum nfc_output_mode {
    FDO_PAGE_SPARE      = 0x0008,
    FDO_SPARE_ONLY      = 0x1008,  // LSB has to be 0x08
    FDO_FLASH_ID        = 0x0010,
    FDO_FLASH_STATUS    = 0x0020,
};

/*!
 * Defined the "complete" address input operations which may involve
 * more than one cycle of single address input operation.
 */
enum nfc_addr_ops {
    ADDRESS_INPUT_READ_ID,
    ADDRESS_INPUT_READ_PAGE,
    ADDRESS_INPUT_PROGRAM_PAGE,
    ADDRESS_INPUT_ERASE_BLOCK,
};

enum nfc_page_area {
    NFC_SPARE_ONLY,
    NFC_MAIN_ONLY,
};

enum {
    MXC_NAND_8_BIT = 8,
    MXC_NAND_16_BIT =  16,
};

enum {
    NAND_SLC = 0,
    NAND_MLC = 1,
};

// read column 464-465 byte but only 464 for bad block marker
#define BAD_BLK_MARKER_464          (NAND_MAIN_BUF3 + 464)
// read column 4-5 byte, but only 5 is used for swapped main area data
#define BAD_BLK_MARKER_SP_5         (NAND_SPAR_BUF3 + 4)

// Polls the NANDFC to wait for an operation to complete
#define wait_op_done()                                                              \
    do {                                                                            \
        while ((readl(NFC_IPC_REG) & NFC_IPC_INT) == 0)  \
            {} \
        write_nfc_ip_reg(0, NFC_IPC_REG); \
    } while (0)

static void write_nfc_ip_reg(u32 val, u32 reg)
{
    writel(NFC_IPC_CREQ, NFC_IPC_REG);
    while((readl(NFC_IPC_REG) & NFC_IPC_CACK) == 0);

    writel(val, reg);
    writel(0, NFC_IPC_REG);
}

/*!
 * NAND flash data output operation (reading data from NAND flash)
 * @param buf_no    internal ram buffer number that will contain data
 *                  to be outputted from the NAND flash after operation done
 * @param mode      one of the mode defined in enum nfc_output_mode
 * @param ecc_en    1 - ecc enabled; 0 - ecc disabled
 */
static void NFC_DATA_OUTPUT(enum nfc_internal_buf buf_no, enum nfc_output_mode mode,
                            int ecc_en)
{
    u32 v = readl(NFC_FLASH_CONFIG2_REG);

    if ((v & NFC_FLASH_CONFIG2_ECC_EN) != 0 && ecc_en == 0) {
        write_nfc_ip_reg(v & ~NFC_FLASH_CONFIG2_ECC_EN, NFC_FLASH_CONFIG2_REG);
    }
    if ((v & NFC_FLASH_CONFIG2_ECC_EN) == 0 && ecc_en != 0) {
        write_nfc_ip_reg(v | NFC_FLASH_CONFIG2_ECC_EN, NFC_FLASH_CONFIG2_REG);
    }

    v = readl(NAND_CONFIGURATION1_REG);

    if (mode == FDO_SPARE_ONLY) {
        v = (v & ~0x31) | buf_no | NAND_CONFIGURATION1_SP_EN;
    } else {
        v = (v & ~0x31) | buf_no;
    }

    writel(v, NAND_CONFIGURATION1_REG);

    writel(mode & 0xFF, NAND_LAUNCH_REG);
    wait_op_done();
}

static void NFC_CMD_INPUT(u32 cmd)
{
    writel(cmd & 0xFFFF, NAND_ADD_CMD_REG);
    writel(NAND_LAUNCH_FCMD, NAND_LAUNCH_REG);
    wait_op_done();
}

static u16 NFC_STATUS_READ(void)
{
    u16 flash_status;
    u16 saved = readw(NAND_MAIN_BUF0);

    NFC_CMD_INPUT(FLASH_Status);
    NFC_DATA_OUTPUT(RAM_BUF_0, FDO_FLASH_STATUS, 1);
    flash_status = readw(NAND_MAIN_BUF0) & 0x00FF;

    // restore
    writew(saved, NAND_MAIN_BUF0);

    return flash_status;
}

/*!
 * NAND flash data input operation (writing data to NAND flash)
 * @param buf_no    internal ram buffer number containing data to be
 *                  written into the NAND flash
 * @param area      NFC_SPARE_ONLY or NFC_MAIN_ONLY,
 * @param ecc_en    1 - ecc enabled; 0 - ecc disabled
 */
static void NFC_DATA_INPUT(enum nfc_internal_buf buf_no, enum nfc_page_area area,
                           int ecc_en)
{
    u32 v = readl(NFC_FLASH_CONFIG2_REG);

    // setup config2 register for ECC enable or not
    if ((v & NFC_FLASH_CONFIG2_ECC_EN) != 0 && ecc_en == 0) {
        write_nfc_ip_reg(v & ~NFC_FLASH_CONFIG2_ECC_EN, NFC_FLASH_CONFIG2_REG);
    }
    if ((v & NFC_FLASH_CONFIG2_ECC_EN) == 0 && ecc_en != 0) {
        write_nfc_ip_reg(v | NFC_FLASH_CONFIG2_ECC_EN, NFC_FLASH_CONFIG2_REG);
    }

    // setup config1 register for ram buffer number, spare-only or not
    v = readl(NAND_CONFIGURATION1_REG);

    if (area == NFC_SPARE_ONLY) {
        v = (v & ~0x31) | buf_no | NAND_CONFIGURATION1_SP_EN;
    } else {
        v = (v & ~0x31) | buf_no;
    }

    writel(v, NAND_CONFIGURATION1_REG);

    // start operation
    writel(NAND_LAUNCH_FDI, NAND_LAUNCH_REG);
    wait_op_done();
}

static void NFC_DATA_INPUT_2k(enum nfc_internal_buf buf_no)
{
    u32 v;

    // setup config1 register for ram buffer number, spare-only or not
    v = readl(NAND_CONFIGURATION1_REG);
    v = (v & ~0x30) | buf_no;
    writel(v, NAND_CONFIGURATION1_REG);

    // start operation
    writel(NAND_LAUNCH_FDI, NAND_LAUNCH_REG);
    wait_op_done();
}

/*!
 * The NFC has to be preset before performing any operation
 */
static void NFC_PRESET(u32 max_block_count)
{
    // not needed. It is done in plf_hardware_init()
}

/*!
 * Issue the address input operation
 * @param       addr    the address for the address input operation
 */
static void NFC_ADDR_INPUT(u32 addr)
{
    if (nfc_debug) {
        diag_printf("add = 0x%x, at 0x%x\n",
                    (addr & ((1 << ADDR_INPUT_SIZE) - 1)) << 16, NAND_ADD_CMD_REG);
        diag_printf("NAND_LAUNCH_FADD=%x, NAND_LAUNCH_REG=%x\n", NAND_LAUNCH_FADD, NAND_LAUNCH_REG);
    }
    writel((addr & ((1 << ADDR_INPUT_SIZE) - 1)) << 16, NAND_ADD_CMD_REG);
    writel(NAND_LAUNCH_FADD, NAND_LAUNCH_REG);
    wait_op_done();
}

#define NFC_ARCH_INIT()

#endif // _MXC_NFC_V2_H_
