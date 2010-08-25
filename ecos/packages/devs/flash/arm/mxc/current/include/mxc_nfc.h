#ifndef _MXC_NFC_H_
#define _MXC_NFC_H_
//==========================================================================
//
//      mxc_nfc.h
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

#if defined(NFC_V1_1)
#define PG_2K_DATA_OP_MULTI_CYCLES()        false
#else
#define PG_2K_DATA_OP_MULTI_CYCLES()        true
#endif

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
#if defined(NFC_V1_1)
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
#define NAND_SPAR_BUF0                  (NFC_BASE + 0x800)
#define NAND_SPAR_BUF1                  (NFC_BASE + 0x810)
#define NAND_SPAR_BUF2                  (NFC_BASE + 0x820)
#define NAND_SPAR_BUF3                  (NFC_BASE + 0x830)
#define NAND_RESERVED                   (NFC_BASE + 0x840)
#endif

#define NFC_BUFSIZE_REG                 (NAND_REG_BASE + 0x00)
#define RAM_BUFFER_ADDRESS_REG          (NAND_REG_BASE + 0x04)
#define NAND_FLASH_ADD_REG              (NAND_REG_BASE + 0x06)
#define NAND_FLASH_CMD_REG              (NAND_REG_BASE + 0x08)
#define NFC_CONFIGURATION_REG           (NAND_REG_BASE + 0x0A)
#define ECC_STATUS_RESULT_REG           (NAND_REG_BASE + 0x0C)
#define ECC_RSLT_MAIN_AREA_REG          (NAND_REG_BASE + 0x0E)
#define ECC_RSLT_SPARE_AREA_REG         (NAND_REG_BASE + 0x10)
#define NF_WR_PROT_REG                  (NAND_REG_BASE + 0x12)
#define NAND_FLASH_WR_PR_ST_REG         (NAND_REG_BASE + 0x18)
#define NAND_FLASH_CONFIG1_REG          (NAND_REG_BASE + 0x1A)
#define NAND_FLASH_CONFIG2_REG          (NAND_REG_BASE + 0x1C)
#if defined(NFC_V1_1)
#define UNLOCK_START_BLK_ADD_REG        (NAND_REG_BASE + 0x20)
#define UNLOCK_END_BLK_ADD_REG          (NAND_REG_BASE + 0x22)
#define UNLOCK_START_BLK_ADD1_REG       (NAND_REG_BASE + 0x24)
#define UNLOCK_END_BLK_ADD1_REG         (NAND_REG_BASE + 0x26)
#define UNLOCK_START_BLK_ADD2_REG       (NAND_REG_BASE + 0x28)
#define UNLOCK_END_BLK_ADD2_REG         (NAND_REG_BASE + 0x2A)
#define UNLOCK_START_BLK_ADD3_REG       (NAND_REG_BASE + 0x2C)
#define UNLOCK_END_BLK_ADD3_REG         (NAND_REG_BASE + 0x2E)
#else
#define UNLOCK_START_BLK_ADD_REG        (NAND_REG_BASE + 0x14)
#define UNLOCK_END_BLK_ADD_REG          (NAND_REG_BASE + 0x16)
#endif

enum nfc_internal_buf {
    RAM_BUF_0,
    RAM_BUF_1,
    RAM_BUF_2,
    RAM_BUF_3,
    RAM_BUF_4,
    RAM_BUF_5,
    RAM_BUF_6,
    RAM_BUF_7,
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
    NFC_MAIN_SPARE,
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
        volatile int mxc_nfc_wait_loop;                                                      \
        while ((readw(NAND_FLASH_CONFIG2_REG) & NAND_FLASH_CONFIG2_INT_DONE) == 0)  \
            {for (mxc_nfc_wait_loop = 0; mxc_nfc_wait_loop < 100; mxc_nfc_wait_loop++);} \
    } while (0)

/*!
 * NAND flash data output operation (reading data from NAND flash)
 * @param buf_no    internal ram buffer number that will contain data
 *                  to be outputted from the NAND flash after operation done
 * @param mode      one of the mode defined in enum nfc_output_mode
 */
static void NFC_DATA_OUTPUT(enum nfc_internal_buf buf_no, enum nfc_output_mode mode,
                            int ecc_en)
{
    u16 config1 = (ecc_en != 0) ? NAND_FLASH_CONFIG1_ECC_EN : 0;

    config1 |= readw(NAND_FLASH_CONFIG1_REG);

    if (mode == FDO_SPARE_ONLY) {
        config1 |= NAND_FLASH_CONFIG1_SP_EN;
    }

    writew(config1, NAND_FLASH_CONFIG1_REG);
    writew(buf_no, RAM_BUFFER_ADDRESS_REG);
    writew(mode & 0xFF, NAND_FLASH_CONFIG2_REG);
    wait_op_done();
}

static void NFC_CMD_INPUT(u32 cmd)
{
    writew(cmd, NAND_FLASH_CMD_REG);
    writew(NAND_FLASH_CONFIG2_FCMD_EN, NAND_FLASH_CONFIG2_REG);
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
    u16 config1 = (ecc_en != 0) ? NAND_FLASH_CONFIG1_ECC_EN : 0;

    config1 |= readw(NAND_FLASH_CONFIG1_REG);

    if (area == NFC_SPARE_ONLY) {
        config1 |= NAND_FLASH_CONFIG1_SP_EN;
#ifdef CYGPKG_HAL_ARM_MXC91221
        config1 &= ~NAND_FLASH_CONFIG1_ECC_EN;
#endif
    }

    writew(config1, NAND_FLASH_CONFIG1_REG);
    writew(buf_no, RAM_BUFFER_ADDRESS_REG);

    // start operation
    writew(NAND_FLASH_CONFIG2_FDI_EN, NAND_FLASH_CONFIG2_REG);
    wait_op_done();
}

static void NFC_DATA_INPUT_2k(enum nfc_internal_buf buf_no)
{
    writew(buf_no, RAM_BUFFER_ADDRESS_REG);
    writew(NAND_FLASH_CONFIG2_FDI_EN, NAND_FLASH_CONFIG2_REG);
    wait_op_done();
}

/*!
 * The NFC has to be preset before performing any operation
 */
static void NFC_PRESET(u32 max_block_count)
{
    // Unlock the internal RAM buffer
    writew(NFC_CONFIGURATION_UNLOCKED, NFC_CONFIGURATION_REG);
    // First Block to be unlocked
    writew(0, UNLOCK_START_BLK_ADD_REG);
    // Last Unlock Block
    writew(max_block_count, UNLOCK_END_BLK_ADD_REG);
    // Unlock Block Command
    writew(NF_WR_PROT_UNLOCK, NF_WR_PROT_REG);
}

/*!
 * Issue the address input operation
 * @param       addr    the address for the address input operation
 */
static void NFC_ADDR_INPUT(u32 addr)
{
    if (nfc_debug) {
        diag_printf("add = 0x%x, at 0x%x\n",
                    addr & ((1 << ADDR_INPUT_SIZE) - 1), NAND_FLASH_ADD_REG);
        diag_printf("NAND_FLASH_CONFIG2_REG=%x\n", NAND_FLASH_CONFIG2_REG);
    }
    writew(addr & ((1 << ADDR_INPUT_SIZE) - 1), NAND_FLASH_ADD_REG);
    writew(NAND_FLASH_CONFIG2_FADD_EN, NAND_FLASH_CONFIG2_REG);
    wait_op_done();
}

#if defined(NFC_V1_1)
#define NFC_ARCH_INIT()		\
		{		\
			unsigned int tmp, reg;	\
			tmp = flash_dev_info->page_size / 512; \
			if(flash_dev_info->spare_size) {\
				writew((flash_dev_info->spare_size>>1), \
						ECC_RSLT_SPARE_AREA_REG);\
			}	\
			writew(0x2, NFC_CONFIGURATION_REG);\
			reg = readw(NAND_FLASH_CONFIG1_REG)| 0x800; \
			if((flash_dev_info->spare_size / tmp) > 16) \
				reg &= ~1;	\
			else 	\
				reg |= 1; \
			writew(reg, NAND_FLASH_CONFIG1_REG); \
		}
#else
#define NFC_ARCH_INIT()
#endif /*NFC_V1_1*/
#endif // _MXC_NFC_H_
