//==========================================================================
//
//      mxc_nfc.c
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
// Date:         2006-01-23 Initial version
// Date:         2007-12-20 Update to support 4K page and bbt management.
// Purpose:
// Description:
//   -- Add bad block management according to Linux NAND MTD implementation.
//      Reference linux/drivers/mtd/nand/nand_bbt.c by Thomas Gleixner
//      Summary:
//         1. Last 4 blocks are reserved for one main BBT and one
//            mirror BBT (2 spare ones just in case a block turns bad.)
//         2. The main BBT block's spare area starts with "Bbt0" followed
//            by a version number starting from 1.
//         3. The mirror BBT block's spare area starts with "1tbB" followed
//            by a version number also starting from 1.
//         4. The actual main area, starting from first page in the BBT block,
//            is used to indicate if a block is bad or not through 2bit/block:
//              * The table uses 2 bits per block
//              * 11b: 	block is good
//              * 00b: 	block is factory marked bad
//              * 01b, 10b: 	block is marked bad due to wear
//      Redboot operations: During boot, it searches for the marker for
//                          either main BBT or mirror BBT based on the marker:
//         case 1: Neither table is found:
//                 Do the bad block scan of the whole flash with ECC off. Use
//                 manufactor marked BI field to decide if a block is bad and
//                 then build the BBT in RAM. Then write this table to both
//                 main BBT block and mirror BBT block.
//         case 2: Only one table is found:
//                 Load the BBT table from the flash and stored in the RAM.
//                 Then build the 2nd BBT table in the flash.
//         case 3: If both tables found, load the one with higher version in the
//                 RAM and then update the block with older BBT info with the
//                 newer one. If same version, just then read out the table in
//                 RAM.
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <redboot.h>
#include <stdlib.h>
static int nfc_debug = 0;

#include CYGHWR_MEMORY_LAYOUT_H

#include <cyg/hal/hal_io.h>
#define  _FLASH_PRIVATE_
#include <cyg/io/flash.h>

#if defined(NFC_V2_0) || defined(NFC_V2_1)
#include <cyg/io/mxc_nfc_v2.h>
#else
#include <cyg/io/mxc_nfc.h>
#endif

/* Search good / bad pattern on the first page only */
#define NAND_BBT_SCAN1STPAGE    0x00000001
/* Search good / bad pattern on the first and the second page */
#define NAND_BBT_SCAN2NDPAGE	0x00000002
/* Search good / bad pattern on the last page only */
#define NAND_BBT_SCANLSTPAGE	0x00000004
// todo: move to top
#define ECC_FORCE_ON    1
#define ECC_FORCE_OFF   2

enum blk_bad_type
{
    BLK_BAD_FACTORY = 0,
    BLK_BAD_RUNTIME = 1,
};

//#define diag_printf1    diag_printf
#define diag_printf1(fmt,args...)
#define MXC_UNLOCK_BLK_END      0xFFFF

extern unsigned int hal_timer_count(void);
int nfc_program_region(u32 addr, u32 buf, u32 len);
int nfc_erase_region(u32 addr, u32 len, u32 skip_bad, bool verbose);

static int nfc_write_pg_random(u32 adj_addr, u32 buf, enum nfc_page_area area,
                               u32 ecc_force);
static int nfc_read_pg_random(u32 adj_addr, u32 force);
static int nfc_erase_blk(u32 ra);
static void print_page (u32 addr, bool spare_only);
static int nfc_read_page(u32 addr);
static int nfc_read_page_sp(u32 addr);
static int mxc_nfc_scan(bool lowlevel);
static void read_nflash_id(void* id);
static int nfc_program_blk(u32 ra, u32 buf, u32 len);

// globals
static int nand_flash_index = -1;
static int g_ecc_enable = true;
static int g_spare_only_read_ok = true;
static int g_nfc_debug_level = NFC_DEBUG_MIN;
static bool g_nfc_debug_measure = false;
static bool g_is_2k_page = false;
static bool g_is_4k_page = false;
static unsigned int g_nfc_version = MXC_NFC_V1;

#define nfc_printf(level, args...)          \
    do {                                \
        if (g_nfc_debug_level >= level)     \
            diag_printf(args);          \
    } while(0)

#ifndef MXCFLASH_SELECT_MULTI
void flash_query(void* data)
#else
void nandflash_query(void* data)
#endif
{
    read_nflash_id(data);
    nfc_printf(NFC_DEBUG_MAX, "%s(ID=0x%x: 0x%x, 0x%x, 0x%x)\n",
               __FUNCTION__, *(u8*)(data), *(u8*)((u32)data+1),
	       *(u8*)((u32)data+2), *(u8*)((u32)data+3));
}

#ifndef MXCFLASH_SELECT_MULTI
int flash_program_buf(void* addr, void* data, int len)
#else
int nandflash_program_buf(void* addr, void* data, int len)
#endif
{
    return nfc_program_region((u32) addr, (u32) data, (u32) len);
}

#ifndef MXCFLASH_SELECT_MULTI
int flash_erase_block(void* block, unsigned int size)
#else
int nandflash_erase_block(void* block, unsigned int size)
#endif
{
    return nfc_erase_region((u32) block, size, 1, 0);
}

#ifndef MXCFLASH_SELECT_MULTI
bool flash_code_overlaps(void *start, void *end)
#else
bool nandflash_code_overlaps(void *start, void *end)
#endif
{
    extern unsigned char _stext[], _etext[];

    return ((((unsigned long)&_stext >= (unsigned long)start) &&
             ((unsigned long)&_stext < (unsigned long)end)) ||
            (((unsigned long)&_etext >= (unsigned long)start) &&
             ((unsigned long)&_etext < (unsigned long)end)));
}

#ifndef MXCFLASH_SELECT_MULTI
int flash_hwr_map_error(int e)
#else
int nandflash_hwr_map_error(int e)
#endif
{
    return e;
}

#ifndef MXCFLASH_SELECT_MULTI
int flash_lock_block(void* block)
#else
int nandflash_lock_block(void* block)
#endif
{
    // Not supported yet
    return 0;
}

#ifndef MXCFLASH_SELECT_MULTI
int flash_unlock_block(void* block, int block_size, int blocks)
#else
int nandflash_unlock_block(void* block, int block_size, int blocks)
#endif
{
    // Not supported yet
    return 0;
}

//----------------------------------------------------------------------------
// Now that device properties are defined, include magic for defining
// accessor type and constants.
#include <cyg/io/flash_dev.h>

// Information about supported devices
typedef struct flash_dev_info {
    cyg_uint16   device_id;
    cyg_uint16   device_id2;
    cyg_uint16   device_id3;
    cyg_uint16   device_id4;
    cyg_uint16   page_size;
    cyg_uint16   spare_size;
    cyg_uint32   pages_per_block;
    cyg_uint32   block_size;
    cyg_int32    block_count;
    cyg_uint32   base_mask;
    cyg_uint32   device_size;
    cyg_uint32   port_size;     // x8 or x16 IO
    cyg_uint32   type;          // SLC vs MLC
    cyg_uint32   options;
    cyg_uint32   fis_start_addr;
    cyg_uint32   bi_off;
    cyg_uint32   bbt_blk_max_nr;
    cyg_uint8    vendor_info[96];
} flash_dev_info_t;

static const flash_dev_info_t* flash_dev_info;
static const flash_dev_info_t supported_devices[] = {
#include <cyg/io/mxc_nand_parts.inl>
};
#define NUM_DEVICES (sizeof(supported_devices)/sizeof(flash_dev_info_t))

#define NF_PG_SZ                    flash_dev_info->page_size
#define NF_SPARE_SIZE               flash_dev_info->spare_size
#define NF_PG_PER_BLK               flash_dev_info->pages_per_block
#define NF_DEV_SZ                   flash_dev_info->device_size
#define NF_BLK_SZ                   flash_dev_info->block_size
#define NF_BLK_CNT                  flash_dev_info->block_count
#define NF_VEND_INFO                flash_dev_info->vendor_info
#define NF_OPTIONS                  flash_dev_info->options
#define NF_BBT_MAX_NR               flash_dev_info->bbt_blk_max_nr
#define NF_OPTIONS                  flash_dev_info->options
#define NF_BI_OFF                   flash_dev_info->bi_off

#define NAND_PG_SHIFT               ((g_is_2k_page) ? 12 : ((g_is_4k_page) ? 13 : 9))
#define NFC_ADJUST_ADDR(addr)       ((addr  / NF_PG_SZ) << NAND_PG_SHIFT)

// Mask off the higher bits representing linear address of the nand flash
#define MXC_NAND_LA_MASK            (NF_DEV_SZ - 1)

#define NFC_DEVICE_ALIGN(a)         ((a) & MXC_NAND_LA_MASK & (~(NF_DEV_SZ - 1)))
#define NFC_BLOCK_ALIGN(a)          ((a) & MXC_NAND_LA_MASK & (~(NF_BLK_SZ - 1)))
#define NFC_PAGE_ALIGN(a)           ((a) & MXC_NAND_LA_MASK & (~(NF_PG_SZ - 1)))

#define BLOCK_TO_OFFSET(blk)            (blk * NF_PG_PER_BLK * NF_PG_SZ)
#define BLOCK_TO_PAGE(blk)              (blk * NF_PG_PER_BLK)
#define BLOCK_PAGE_TO_OFFSET(blk, pge)  ((blk * NF_PG_PER_BLK + pge) * NF_PG_SZ)
#define OFFSET_TO_BLOCK(offset)         ((offset / NF_PG_SZ) / NF_PG_PER_BLK)
#define OFFSET_TO_PAGE(offset)          ((offset / NF_PG_SZ) % NF_PG_PER_BLK)

static u8 *g_bbt, *g_page_buf;
static u32 g_bbt_sz;
static u32 g_main_bbt_addr = 0, g_mirror_bbt_page = 0;
static u8 g_main_bbt_ver;
//static u8 g_mirror_bbt_ver;
static u8 g_main_bbt_des[] = "Bbt0";
//static u8 g_mirror_bbt_des[] = "1tbB";
static bool mxcnfc_init_ok = false;

//mxc_nand_fixup_t nand_page_sz_2k_fixup;
//mxc_nand_fixup_t nand_port_sz_x16_fixup;

// this callback allows the platform specific function to be called right
// after flash_dev_query()
nfc_setup_func_t *nfc_setup = NULL;

int
#ifndef MXCFLASH_SELECT_MULTI
flash_hwr_init(void)
#else
nandflash_hwr_init(void)
#endif
{
    cyg_uint16 id[4];
    int i;
    nfc_printf(NFC_DEBUG_MAX, "%s()\n", __FUNCTION__);
    NFC_CMD_INPUT(FLASH_Reset);

    // Look through table for device data
    flash_dev_info = supported_devices;
    for (i = 0; i < NUM_DEVICES; i++) {
        flash_dev_query(id);
        if ((flash_dev_info->device_id == id[0]) &&
            (flash_dev_info->device_id2 == 0xFFFF || flash_dev_info->device_id2 == id[1]))
            break;
        flash_dev_info++;
    }

    nfc_printf(NFC_DEBUG_MED, "%s(): %d out of NUM_DEVICES=%d, id=0x%x\n",
               __FUNCTION__, i, (u32)NUM_DEVICES, flash_dev_info->device_id);

    // Do we find the device? If not, return error.
    if (NUM_DEVICES == i) {
        diag_printf("Unrecognized NAND part: 0x%04x, 0x%04x, 0x%04x, 0x%04x\n",
                    id[0], id[1], id[2], id[3]);
        return FLASH_ERR_DRV_WRONG_PART;
    }

    if (NF_PG_SZ == 2048) {
        g_is_2k_page = true;
        g_spare_only_read_ok = false;
    }
    if (NF_PG_SZ == 4096) {
        g_is_4k_page = true;
        g_spare_only_read_ok = false;
    }
    if (nfc_setup) {
        g_nfc_version = nfc_setup(NF_PG_SZ, flash_dev_info->port_size,
                                  flash_dev_info->type);
    }

    NFC_ARCH_INIT();

    nand_flash_index = i;
    mxcnfc_init_ok = true;

    g_bbt_sz = NF_BLK_CNT / 4;
    g_bbt = (u8*)malloc(g_bbt_sz); // two bit for each block
    if (g_bbt == NULL) {
        diag_printf("%s(): malloc failed (%d)\n", __FUNCTION__, __LINE__);
        return FLASH_ERR_PROTOCOL;
    }

    g_page_buf = (u8*)malloc(NF_PG_SZ); // for programming less than one page size buffer
    if (g_page_buf == NULL) {
        diag_printf("%s(): malloc failed (%d)\n", __FUNCTION__, __LINE__);
        return FLASH_ERR_PROTOCOL;
    }
    memset(g_bbt, -1, g_bbt_sz);

    // Hard wired for now
    flash_info.block_size = NF_BLK_SZ;
    flash_info.blocks = NF_BLK_CNT;
    flash_info.start = (void *)0;
    flash_info.end = (void *)NF_DEV_SZ;

    diag_printf1("%s(g_bbt=%p)\n", __FUNCTION__, g_bbt);
    mxc_nfc_scan(false); // look for table

    diag_printf1("%s(): block_size=0x%x, blocks=0x%x, start=%p, end=%p\n",
               __FUNCTION__, flash_info.block_size, flash_info.blocks,
               flash_info.start, flash_info.end);

    return FLASH_ERR_OK;
}

// used by redboot/current/src/flash.c
int mxc_nand_fis_start(void)
{
    return flash_dev_info->fis_start_addr;
}

#define nfc_buf_mem_cpy     memcpy
#ifndef nfc_buf_mem_cpy
static void nfc_buf_mem_cpy(void *dst, void *src, u32 len)
{
    u16 *d = (u16*) dst, *s = (u16*) src;
    if ((len & 1) != 0 || ((u32)dst & 1) != 0 || ((u32)src & 1) != 0) {
        diag_printf("Error: len=0x%x, dst=%p, src=%p should be even number\n",
                    len, dst, src);
        return;
    }
    if (len == 0) {
        return;
    }
    do {
        *d++ = *s++;
        len -= 2;
    } while (len > 0);
}
#endif

/*!
 * Starts the address input cycles for different operations as defined in ops.
 *
 * @param ops           operations as defined in enum nfc_addr_ops
 * @param adj_addr      starting address
 * @param mask          mask for the full address range of the nand flash
 *                      For 64MB flash, the mask should be 0x03FFFFFF (64MB-1)
 * Note that the adj_addr argument is not linear address from user perspective,
 * i.e. it is not the original passed-in value from user. It is the "adjusted"
 * address from the the NAND flash perspective. The adjustment is done via
 * NAND_PG_SHIFT. So don't call this function without massaging the address
 * properly with NAND_PG_SHIFT.
 */
static void start_nfc_addr_ops(enum nfc_addr_ops ops, u32 adj_addr, u32 mask)
{
    u32 m = mask, a = adj_addr;

    switch (ops) {
    case ADDRESS_INPUT_READ_ID:
        NFC_ADDR_INPUT(0);
        return;
    case ADDRESS_INPUT_READ_PAGE:
    case ADDRESS_INPUT_PROGRAM_PAGE:
        if (g_is_2k_page) {
            NFC_ADDR_INPUT(a & 0xFF);
            NFC_ADDR_INPUT((a >> 8) & 0xF);
        } else if (g_is_4k_page) {
            NFC_ADDR_INPUT(a & 0xFF);
            NFC_ADDR_INPUT((a >> 8) & 0x1F);
        } else {
            NFC_ADDR_INPUT(a & 0xFF);
        }
        // don't break on purpose
    case ADDRESS_INPUT_ERASE_BLOCK:
        a >>= NAND_PG_SHIFT;
        m >>= (g_is_2k_page ? 11 : (g_is_4k_page ? 12 : 9));
        break;
    default:
        diag_printf("!!!!!! %s(): wrong ops: %d !!!!!\n", __FUNCTION__, ops);
        return;
    }

    do {
        NFC_ADDR_INPUT(a);
        m >>= ADDR_INPUT_SIZE;
        a >>= ADDR_INPUT_SIZE;
    } while (m != 0);
}

static void read_nflash_id(void* id)
{
    volatile u32 *ptr = (u32*) NAND_MAIN_BUF0;
    volatile u32 *id_32 = (u32*)id;

    nfc_printf(NFC_DEBUG_MAX, "%s()\n", __FUNCTION__);
    NFC_PRESET(MXC_UNLOCK_BLK_END);
    NFC_CMD_INPUT(FLASH_Read_ID);
    start_nfc_addr_ops(ADDRESS_INPUT_READ_ID, 0, 0);
    NFC_DATA_OUTPUT(RAM_BUF_0, FDO_FLASH_ID, g_ecc_enable);

    *id_32++ = *ptr++;
    *id_32++ = *ptr++;
}

/*!
 * Checks to see if a block is bad. If buf is not NULL, it indicates a valid
 * BBT in the RAM. In this case, it assumes to have 2-bit to represent each
 * block for good or bad
 *              * 11b: 	block is good
 *              * 00b: 	block is factory marked bad
 *              * 01b, 10b: 	block is marked bad due to wear
 * If buf is NULL, then it indicates a low level scan based on the certain
 * offset value in certain pages and certain offset to be non-0xFF. In this
 * case, the HW ECC will be turned off.
 *
 * @param block     0-based block number
 * @param buf       BBT buffer. Could be NULL (see above explanation)
 *
 * @return          1 if bad block; 0 otherwise
 */
static bool nfc_is_badblock(u32 block, u8 *buf)
{
    u32 off;       // byte offset
    u32 sft;       // bit shift 0, 2, 4, 6
    u32 addr;
    u16 temp;
    bool res = false;

    if (buf) {
        // use BBT
        off = block >> 2;       // byte offset
        sft = (block & 3) << 1;  // bit shift 0, 2, 4, 6
        if (((buf[off] >> sft) & 0x3) != 0x3) {
            res = true;
        }
        goto out;
    }
    // need to do low level scan with ECC off
    if (NF_OPTIONS & NAND_BBT_SCAN1STPAGE) {
        addr = block * NF_BLK_SZ; // TODO: overflow for over 4GB nand
        nfc_read_pg_random(NFC_ADJUST_ADDR(addr), ECC_FORCE_OFF); // no ecc
        if (g_is_2k_page || g_is_4k_page) {
            temp = readw(NAND_MAIN_BUF0 + NF_BI_OFF);
        } else {
            temp = readw(NAND_SPAR_BUF0 + 4) >> 8; // BI is at 5th byte in spare area
        }
        if ((temp & 0xFF) != 0xFF)
            res = true;
    }
    if (NF_OPTIONS & NAND_BBT_SCAN2NDPAGE) {
        addr = block * NF_BLK_SZ + NF_PG_SZ; // TODO: overflow for over 4GB nand
        nfc_read_pg_random(NFC_ADJUST_ADDR(addr), ECC_FORCE_OFF); // no ecc
        if (g_is_2k_page || g_is_4k_page) {
            temp = readw(NAND_MAIN_BUF0 + NF_BI_OFF);
        } else {
            temp = readw(NAND_SPAR_BUF0 + 4) >> 8; // BI is at 5th byte in spare area
        }
        if ((temp & 0xFF) != 0xFF)
            res = true;
    }
    if (NF_OPTIONS & NAND_BBT_SCANLSTPAGE) {
        if (g_is_4k_page || g_is_2k_page) {
            // TODO: overflow for over 4GB nand
            addr = (block + 1) * NF_BLK_SZ - NF_PG_SZ;
            // we don't do partial page read here. No ecc either
            nfc_read_pg_random(NFC_ADJUST_ADDR(addr), ECC_FORCE_OFF);
            temp = readw((u32)NAND_MAIN_BUF0 + NF_BI_OFF);
            if ((temp & 0xFF) != 0xFF)
                res = true;
        } else {
            diag_printf("only 2K/4K page is supported\n");
            // die here -- need to fix the SW
            while (1);
        }
    }
out:
    return res;
}

/*
 * Program g_bbt into the NAND block with offset at g_main_bbt_addr.
 * This assumes that the g_bbt has been built already.
 *
 * If g_main_bbt_addr is 0, search for a free block from the bottom 4 blocks (but make
 * sure not re-using the mirror block). If g_mirror_bbt_page is 0, do the same thing.
 * Otherwise, just use g_main_bbt_addr, g_mirror_bbt_page numbers to prgram the
 * g_bbt into those two blocks.
 * todo: need to do the version to see which one is newer.
 *
 * @return  0 if successful; -1 otherwise.
 */
static int program_bbt_to_flash(void)
{
    int i = 0;
    u32 addr, blk;

    if (g_main_bbt_addr) {
        // update the spare area before writing and version number
        g_main_bbt_ver++;
    } else {
        // no existing main bbt table in flash, build one.
        g_main_bbt_ver = 1; // first BBT version
        for (i = 0; i < NF_BBT_MAX_NR; i++) {
            blk = NF_BLK_CNT - i - 1;
            addr = blk * NF_BLK_SZ;
            if (g_mirror_bbt_page == addr || nfc_is_badblock(blk, g_bbt))
                continue;
            g_main_bbt_addr = addr;
            break;
        }
    }
    // todo: take care of bad block here if programming error?
    if (i < NF_BBT_MAX_NR) {
        nfc_erase_blk(g_main_bbt_addr);
        writel(*(u32*)g_main_bbt_des, NAND_SPAR_BUF0);
        writew(g_main_bbt_ver | 0xFF00, NAND_SPAR_BUF0 + 4);

        nfc_program_blk(g_main_bbt_addr, (u32)g_bbt, g_bbt_sz);

        diag_printf("\nWriting BBT at offset 0x%x size=%d\n", g_main_bbt_addr, g_bbt_sz);
    } else {
        diag_printf("Error: %s() failed to build main BBT in flash\n", __FUNCTION__);
        return -1;
    }
    return 0;
}

/*!
 * Unconditionally erase a block without checking the BI field.
 * Note that there is NO error checking for passed-in ra.
 *
 * @param ra        starting address in the raw address space (offset)
 *                  Must be block-aligned
 * @return          0 if successful; -1 otherwise
 */
static int nfc_erase_blk(u32 ra)
{
    u16 flash_status;

    NFC_CMD_INPUT(FLASH_Block_Erase);
    // todo: replace MXC_NAND_LA_MASK with number of addr cycles
    start_nfc_addr_ops(ADDRESS_INPUT_ERASE_BLOCK, NFC_ADJUST_ADDR(ra), MXC_NAND_LA_MASK);
    NFC_CMD_INPUT(FLASH_Start_Erase);

    flash_status = NFC_STATUS_READ();

    // check I/O bit 0 to see if it is 0 for success
    if((flash_status & 0x1) != 0) {
        return -1;
    }
    return 0;
}

/*!
 * Program a block of data in the flash. This function doesn't do
 * bad block checking. But if program fails, it return error.
 * Note: If "len" is less than a block it will program up to a page's
 *       boundary. If not within a page boundary, then it fills the
 *       rest of the page with 0xFF.
 *
 * @param ra        destination raw flash address
 * @param buf       source address in the RAM
 * @param len       len to be programmed
 *
 * @return          0 if successful; -1 otherwise
 */
static int nfc_program_blk(u32 ra, u32 buf, u32 len)
{
    for (; len >= NF_PG_SZ; len -= NF_PG_SZ) {
        if (nfc_write_pg_random(NFC_ADJUST_ADDR(ra), buf, NFC_MAIN_ONLY, 0) != 0) {
            return -1;
        }
        ra += NF_PG_SZ;
        buf += NF_PG_SZ;
    }
    if (len != 0) {
        memset(g_page_buf, 0xFF, NF_PG_SZ);
        memcpy(g_page_buf, (void *)buf, len);
        if (nfc_write_pg_random(NFC_ADJUST_ADDR(ra), (u32)g_page_buf, NFC_MAIN_ONLY, 0) != 0) {
            return -1;
        }
    }
    return 0;
}

static void mark_blk_bad(unsigned int block, unsigned char *buf,
                                enum blk_bad_type bad_type)
{
    unsigned int off = block >> 2;       // byte offset - each byte can hold status for 4 blocks
    unsigned int sft = (block & 3) << 1;  // bit shift 0, 2, 4, 6
    unsigned char val = buf[off];
    diag_printf1("buf[%d]=0x%x\n", off, buf[off]);

    val &= ~(3 << sft) | (bad_type << sft);
    buf[off] = val;
    diag_printf1("buf[%d]=0x%x\n", off, val);
}

/*!
 * Erase a range of NAND flash good blocks only.
 * It skips bad blocks and update the BBT once it sees new bad block due to erase.
 * @param addr          raw NAND flash address. it has to be block size aligned
 * @param len           number of bytes
 * @param skip_bad      if 1, don't erase bad block; otherwise, always erase
 * @param verbose       use true to print more messages
 *
 * @return              FLASH_ERR_OK (0) if successful; non-zero otherwise
 */
int nfc_erase_region(u32 addr, u32 len, u32 skip_bad, bool verbose)
{
    u32 sz, blk, update = 0, skip = 0, j = 0;

    nfc_printf(NFC_DEBUG_MED, "%s(addr=0x%x, len=0x%x)\n",
               __FUNCTION__, addr, len);

    if ((addr % NF_BLK_SZ) != 0 || (len % NF_BLK_SZ) != 0 || len == 0) {
        diag_printf("%s(): invalid value or not aligned with block boundry\n", __FUNCTION__);
        diag_printf("addr=0x%x, len=%d\n", addr, len);
        return FLASH_ERR_INVALID;
    }

    // now addr has to be block aligned
    for (sz = 0; sz < len; addr += NF_BLK_SZ, j++) {
        blk = OFFSET_TO_BLOCK(addr);
        if (skip_bad && nfc_is_badblock(blk, g_bbt)) {
            diag_printf("\nWarning: %s(addr=0x%x, block=%d): skipping bad\n",
                       __FUNCTION__, addr, blk);
            skip++;
            continue;
        }
        if (nfc_erase_blk(addr) != 0) {
            diag_printf("\nError: %s2(addr=0x%x, block=%d): run-time erase error!\n",
                       __FUNCTION__, addr, blk);
            mark_blk_bad(blk, g_bbt, BLK_BAD_RUNTIME);
            if (!skip_bad) {
                sz += NF_BLK_SZ;
            }
            // we don't need to update the table immediately here since even
            // with power loss now, we should see the same erase error again.
            update++;
            continue;
        }
        if (verbose) {
            if ((j % 0x20) == 0)
                diag_printf("\n%s 0x%08x: ", skip_bad ? "Erase" : "FORCE erase", addr);
            diag_printf(".");
        }
        sz += NF_BLK_SZ;
    }
    if (update) {
        if (program_bbt_to_flash() != 0) {
            diag_printf("ERROR: TOO BAD! What can I do?\n");
            return -1;
        }
        diag_printf("\n%s(new bad blocks=%d)\n\n", __FUNCTION__, update);
    }
    if (skip) {
        diag_printf("\n%s(skip bad blocks=%d\n\n", __FUNCTION__, skip);
    }
    return FLASH_ERR_OK;
}

/*!
 * Program a range of NAND flash in blocks only.
 * It skips bad blocks and update the BBT once it sees new bad block due to program.
 * @param addr          raw NAND flash address. it has to be block size aligned
 * @param len           number of bytes
 * @return              FLASH_ERR_OK (0) if successful; non-zero otherwise
 */
int nfc_program_region(u32 addr, u32 buf, u32 len)
{
    u32 sz, blk, update = 0, skip = 0;

    diag_printf1("%s(addr=0x%x, len=0x%x)\n", __FUNCTION__, addr, len);

    if ((addr % NF_BLK_SZ) != 0 || len == 0) {
        diag_printf("%s(): invalid value or not aligned with block boundry\n", __FUNCTION__);
        diag_printf("addr=0x%x, len=%d\n", addr, len);
        return FLASH_ERR_INVALID;
    }

    // now addr has to be block aligned
    for (; ; addr += NF_BLK_SZ) {
        blk = OFFSET_TO_BLOCK(addr);
        if (nfc_is_badblock(blk, g_bbt)) {
            diag_printf("\nWarning: %s(addr=0x%x, block=%d): skipping bad\n",
                       __FUNCTION__, addr, blk);
            skip++;
            continue;
        }
        sz = (len >= NF_BLK_SZ) ? NF_BLK_SZ : len;

        if (nfc_program_blk(addr, buf, sz) != 0) {
            diag_printf("\nError: %s2(addr=0x%x, block=%d): run-time program error!\n",
                       __FUNCTION__, addr, blk);
            mark_blk_bad(blk, g_bbt, BLK_BAD_RUNTIME);
            // we don't need to update the table immediately here since even
            // with power loss now, we should see the same program error again.
            update++;
            continue;
        }

        len -= NF_BLK_SZ;
        buf += NF_BLK_SZ;
        if (len == 0 || sz < NF_BLK_SZ)
            break;
    }
    if (update) {
        if (program_bbt_to_flash() != 0) {
            diag_printf("ERROR: TOO BAD! What can I do?\n");
            return -1;
        }
        diag_printf("\n%s(new bad blocks=%d\n", __FUNCTION__, update);
    }
    if (skip)
        diag_printf("\n%s(skip bad blocks=%d\n", __FUNCTION__, skip);

    return FLASH_ERR_OK;
}

/*! todo MXC_NAND_LA_MASK should go away
 * Read data from raw NAND flash address to memory. The MSB of the passed-
 * in flash address will be masked off inside the function.
 * It skips bad blocks and read good blocks of data for "len" bytes.
 *
 * @param addr          NAND flash address. it has to be page aligned
 * @param buf           memory buf where data will be copied to
 * @param len           number of bytes
 * @return              FLASH_ERR_OK (0) if successful; non-zero otherwise
 */
int nfc_read_region(u32 addr, u32 buf, u32 len)
{
    u32 sz, blk = 0, bad;

    // make sure 32-bit aligned
    len = (len + 3) & (~0x3);

    diag_printf1("\n%s(addr=0x%x, buf=0x%x, len=0x%x)\n",
               __FUNCTION__, addr, buf, len);

    if (addr < (u32)(flash_info.start) || (addr + len) > (u32)(flash_info.end) || len == 0) {
        diag_printf("\n%s(): Error: invalide address=0x%x, len=%d\n",
                    __FUNCTION__, addr, len);
        diag_printf("flash_info.start=%p, flash_info.end=%p\n", flash_info.start, flash_info.end);
        return FLASH_ERR_INVALID;
    }

    if ((addr % NF_PG_SZ) != 0) {
        diag_printf("\n%s(): invalid value or not aligned with page boundry\n", __FUNCTION__);
        diag_printf("addr=0x%x, len=%d\n", addr, len);
        return FLASH_ERR_INVALID;
    }

    for (sz = 0, bad = 0; sz < len;) {
        if ((addr % NF_BLK_SZ) == 0) {
            // only need to test block aligned page address
            blk = OFFSET_TO_BLOCK(addr);
            if (nfc_is_badblock(blk, g_bbt)) {
                diag_printf("\nWarning: %s(addr=0x%x, block=%d): skipping bad\n",
                           __FUNCTION__, addr, blk);
                addr += NF_BLK_SZ;
                if (bad++ >= (NF_BLK_CNT / 10)) {
                    diag_printf("Found too many bad blocks (%d). Abort\n", bad);
                    return FLASH_ERR_PROTOCOL;
                }
                continue;
            }
        }

        if (nfc_read_page(addr) != 0) {
            diag_printf("\nError: %s() can't handle read error at (addr=0x%x, block=%d)\n",
                       __FUNCTION__, addr, blk);
            return FLASH_ERR_INVALID;
        }
        // now do the copying
        nfc_buf_mem_cpy((void*)buf, (void*)(NAND_MAIN_BUF0), NF_PG_SZ);

        buf += NF_PG_SZ; sz += NF_PG_SZ; addr += NF_PG_SZ;
    }
    return FLASH_ERR_OK;
}

/*
 * Support only either program for main area only. Or spare-area only for 512B.
 * If one wants to write to the spare-area, then before calling this function,
 * the spare area NFC RAM buffer has to be setup already. This function doesn't touch
 * the spare area NFC RAM buffer.
 * Note that:
 *  1) If adj_addr's column address is all zero, then the ECC is always
 *     ON unless the ecc_force argument is non-zero, which is mainly for testing.
 *  2) If adj_addr's column address is non zeor, then the ECC is always
 *     OFF unless the ecc_force argument is non-zero, which is mainly for testing.
 *  3) Special treatment for barker code on certain platforms. The barker code
 *     always stored in the beginning of the 1st spare area.
 *
 * @param   adj_addr        raw flash byte offset that has been adjusted with NAND_PG_SHIFT
 * @param   buf             sdram buffer to be programmed into the NAND flash.
 * @return  0 if successful; non-zero otherwise
 */
static int nfc_write_pg_random(u32 adj_addr, u32 buf, enum nfc_page_area area,
                               u32 ecc_force)
{
    u16 flash_status;
    u32 ecc = 1;

    // the 2nd condition is to test for unaligned page address -- ecc has to be off.
    if (ecc_force == ECC_FORCE_OFF || (adj_addr & ((1 << NAND_PG_SHIFT) - 1)) != 0 ) {
        ecc = 0;
    }

    diag_printf1("%s(0x%x, 0x%x, %d)\n", __FUNCTION__, adj_addr, buf, area);

    switch (area) {
    case NFC_MAIN_ONLY:
        // No need to worry about the spare area
        nfc_buf_mem_cpy((void *)NAND_MAIN_BUF0, (void *)buf, NF_PG_SZ);
        break;
    case NFC_SPARE_ONLY:
        nfc_buf_mem_cpy((void *)NAND_SPAR_BUF0, (void *)buf, NF_SPARE_SIZE);
        break;
    case NFC_MAIN_SPARE:
        nfc_buf_mem_cpy((void *)NAND_MAIN_BUF0, (void *)buf, NF_PG_SZ);
        nfc_buf_mem_cpy((void *)NAND_SPAR_BUF0, (void *)(buf + NF_PG_SZ), NF_SPARE_SIZE);
        break;
    default:
        diag_printf("NOT supported yet!\n");
        return -1;
    }

    NFC_CMD_INPUT(FLASH_Send_Data);
    start_nfc_addr_ops(ADDRESS_INPUT_PROGRAM_PAGE, adj_addr, MXC_NAND_LA_MASK);

    NFC_DATA_INPUT(RAM_BUF_0, area, ecc);
    if (g_is_2k_page && PG_2K_DATA_OP_MULTI_CYCLES()) {
        NFC_DATA_INPUT_2k(RAM_BUF_1);
        NFC_DATA_INPUT_2k(RAM_BUF_2);
        NFC_DATA_INPUT_2k(RAM_BUF_3);
    }
    NFC_CMD_INPUT(FLASH_Program);

    flash_status = NFC_STATUS_READ();
    // check I/O bit 0 to see if it is 0 for success
    if((flash_status & 0x1) != 0) {
        return -1;
    }

    return 0;
}

/*!
 * Low level spare-only read. Only applies to 512 byte page NAND.
 *
 * @param addr          starting address to be read from the NAND flash
 * @param buf           one of the internal buffers
 * @return              0 if no error or 1-bit error; -1 otherwise
 */
static int nfc_sp_only_read_ll(u32 addr, enum nfc_internal_buf buf)
{
    volatile u16 temp;

    NFC_CMD_INPUT(FLASH_Read_Mode3);
    start_nfc_addr_ops(ADDRESS_INPUT_READ_PAGE, addr, MXC_NAND_LA_MASK);
    NFC_DATA_OUTPUT(buf, FDO_SPARE_ONLY, g_ecc_enable);
    temp = readw(ECC_STATUS_RESULT_REG);
    NFC_CMD_INPUT(FLASH_Read_Mode1);

    if (g_ecc_enable) {
        if ((temp & 0x2) != 0x0) {
            nfc_printf(NFC_DEBUG_MED, "\nError %d: %s(addr=0x%x): ECC status result reg=0x%x\n",
                        __LINE__, __FUNCTION__, addr, temp);
            return -1;
        }
    }

    return 0;
}

/*!
 * Read spare area from NAND flash to the 1st internal RAM buffer.
 * Not supported for 2kB page NAND.
 *
 * @param addr          starting address to be read from the NAND flash
 *
 * @return              0 if no error or 1-bit error; -1 otherwise
 */
static int nfc_read_page_sp(u32 addr)
{
    if (g_spare_only_read_ok) {
        if (g_is_2k_page || g_is_4k_page) {
            diag_printf("spare-only read for 2k page is not supported\n");
            return -1;
        }
        return nfc_sp_only_read_ll(addr, RAM_BUF_0);
    }
    return -1;
}

/*
 * The adj_addr must have been adjusted with NAND_PG_SHIFT.
 * See comments for start_nfc_addr_ops()
 * This function read a page from flash at adj_addr byte offset.
 * Note that:
 *  1) If adj_addr's column address is all zero, then the ECC is always
 *     ON unless the force argument is non-zero, which is mainly for testing.
 *  2) If adj_addr's column address is non zeor, then the ECC is always
 *     OFF unless the force argument is non-zero, which is mainly for testing.
 *  3) Special treatment for barker code on certain platforms. The barker code
 *     always copied from the beginning of the 1st spare area.
 */
static int nfc_read_pg_random(u32 adj_addr, u32 ecc_force)
{
    u32 t1, ecc = 1;
    u8 t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0, t8 = 0;
    int res = 0;

    if (ecc_force == ECC_FORCE_OFF || (adj_addr & ((1 << NAND_PG_SHIFT) - 1)) != 0 )
        ecc = 0;

    NFC_CMD_INPUT(FLASH_Read_Mode1);
    start_nfc_addr_ops(ADDRESS_INPUT_READ_PAGE, adj_addr, MXC_NAND_LA_MASK);
    if (g_is_2k_page || g_is_4k_page) {
        NFC_CMD_INPUT(FLASH_Read_Mode1_LG);
    }

    NFC_DATA_OUTPUT(RAM_BUF_0, FDO_PAGE_SPARE, ecc);
    if (g_nfc_version == MXC_NFC_V1) {
        t1 = readw(ECC_STATUS_RESULT_REG);
        if (g_is_2k_page && PG_2K_DATA_OP_MULTI_CYCLES()) {
            NFC_DATA_OUTPUT(RAM_BUF_1, FDO_PAGE_SPARE, ecc);
            t2 = readw(ECC_STATUS_RESULT_REG);
            NFC_DATA_OUTPUT(RAM_BUF_2, FDO_PAGE_SPARE, ecc);
            t3 = readw(ECC_STATUS_RESULT_REG);
            NFC_DATA_OUTPUT(RAM_BUF_3, FDO_PAGE_SPARE, ecc);
            t4 = readw(ECC_STATUS_RESULT_REG);
        }

        if (ecc && ((t1 & 0xA) != 0x0 || (t2 & 0xA) != 0x0 || (t3 & 0xA) != 0x0
            || (t4 & 0xA) != 0x0)) {
            diag_printf("\nError %d: %s(page=%d, col=%d): ECC status=0x%x:0x%x:0x%x:0x%x\n",
                        __LINE__, __FUNCTION__, adj_addr >> NAND_PG_SHIFT,
                        adj_addr & ((1 << NAND_PG_SHIFT) -1), t1, t2, t3, t4);
            res = -1;
            goto out;
        }
    } else if (g_nfc_version == MXC_NFC_V2) {
        if (g_is_2k_page && PG_2K_DATA_OP_MULTI_CYCLES()) {
            NFC_DATA_OUTPUT(RAM_BUF_1, FDO_PAGE_SPARE, ecc);
            NFC_DATA_OUTPUT(RAM_BUF_2, FDO_PAGE_SPARE, ecc);
            NFC_DATA_OUTPUT(RAM_BUF_3, FDO_PAGE_SPARE, ecc);
        }
        if (ecc) {
            t1 = readl(ECC_STATUS_RESULT_REG);
            if (g_is_2k_page || g_is_4k_page) {
                t2 = (t1 >> 4) & 0xF;
                t3 = (t1 >> 8) & 0xF;
                t4 = (t1 >> 12) & 0xF;
                if (g_is_4k_page) {
                    t5 = (t1 >> 16) & 0xF;
                    t6 = (t1 >> 20) & 0xF;
                    t7 = (t1 >> 24) & 0xF;
                    t8 = (t1 >> 28) & 0xF;
                }
            }
            if ((t1 = (t1 & 0xF)) > 4 || t2 > 4 || t3 > 4 || t4 > 4 || t5 > 4 || t6 > 4 || t7 > 4 || t8 > 4) {
                diag_printf("\nError %d: %s(page=%d, col=%d)\n",
                            __LINE__, __FUNCTION__, adj_addr >> NAND_PG_SHIFT,
                            adj_addr & ((1 << NAND_PG_SHIFT) -1));
                diag_printf("   ECC status=%x:%x:%x:%x:%x:%x:%x:%x\n", t1, t2, t3, t4, t5, t6, t7, t8);
                res = -1;
                goto out;
            }
        }
    } else {
        diag_printf("Unknown NFC version: %d\n", g_nfc_version);
        while (1);
    }
#ifdef BARKER_CODE_SWAP_LOC
    // To replace the data at offset BARKER_CODE_SWAP_LOC with the address of the NFC base
    // This is needed for certain platforms
    if ((u32)adj_addr <= BARKER_CODE_SWAP_LOC && (u32)(adj_addr + NF_PG_SZ - 1) > BARKER_CODE_SWAP_LOC) {
        diag_printf("\n[INFO]: copy back data from spare to 0x%x\n", BARKER_CODE_SWAP_LOC);
        writel(readl(NAND_SPAR_BUF0), NFC_BASE + BARKER_CODE_SWAP_LOC);
    }
#endif

out:
    return res;
}

/*!
 * Read a page's both main and spare area from NAND flash to the internal RAM buffer.
 * It always reads data to the internal buffer 0.
 *
 * @param ra          starting address to be read from the NAND flash; must be page-aligned
 *
 * @return              0 if no error or 1-bit error; -1 otherwise
 */
static int nfc_read_page(u32 ra)
{
    u32 flash_addr;

    if (ra % NF_PG_SZ) {
        diag_printf("Non page-aligned read not supported here: 0x%x\n", ra);
        return -1;
    }
    flash_addr = NFC_ADJUST_ADDR(ra);

    return nfc_read_pg_random(flash_addr, ECC_FORCE_ON);
}

// Read data into buffer
int flash_read_buf(void* addr, void* data, int len)
{
    if (IS_BOOTING_FROM_NOR() || IS_FIS_FROM_NOR()) {
        memcpy(data, addr, len);
        return 0;
    } else {
        return nfc_read_region((u32)addr, (u32)data, (u32)len);
    }
}

void mxc_nfc_print_info(void)
{
    diag_printf("[0x%08x bytes]: %d blocks of %d pages of %d bytes each.\n",
                NF_DEV_SZ, NF_BLK_CNT,
                NF_PG_PER_BLK, NF_PG_SZ);
}

/*
 * Look for the BBT table depending on the passed-in lowlevel value.
 * @param   lowlevel    If true, then it does a low level scan based on factory
 *                      marked BI(block info) field with ECC off to decide if a
 *                      block is bad.
 *                      If false, then it checks to see if an existing BBT in the
 *                      flash or not. If not, then it returns -1. If yes, it will
 *                      prints out the number of bad blocks.
 *
 * @return  number of bad blocks for the whole nand flash
 *
 * Note: For a brand new flash, this function has to be called with
 *       lowlevel=true.
 *
 *
 */
static int mxc_nfc_scan(bool lowlevel)
{
    u32 addr, bad = 0, i = 0, blk;
    u32 count1 = 0, count2 = 0;
    u8 *buf = 0;

    nfc_printf(NFC_DEBUG_MAX, "%s()\n", __FUNCTION__);

    if (g_nfc_debug_measure) {
        count1 = hal_timer_count();
    }
    // read out the last 4 blocks for marker
    // need to keep where is the td and md block number
    if (!lowlevel) {
        diag_printf("Searching for BBT table in the flash ...\n");
        g_main_bbt_addr = 0;
        for (i = 0; i < NF_BBT_MAX_NR; i++) {
            blk = NF_BLK_CNT - i - 1;
            addr = blk * NF_BLK_SZ;
            if (nfc_read_pg_random(NFC_ADJUST_ADDR(addr), ECC_FORCE_ON) != 0)
                continue;
            if (*(u32 *)g_main_bbt_des == *(u32 *)NAND_SPAR_BUF0) {
                diag_printf1("bingo\n");
                g_main_bbt_addr = addr;
                g_main_bbt_ver = readw(NAND_SPAR_BUF0 + 4);
                if (nfc_read_region(addr, (u32)g_bbt, g_bbt_sz) != 0) {
                    diag_printf("ERROR!!! Can't read BBT table at addr: 0x%x\n", addr);
                    return -1;
                }
                diag_printf("\nFound version %d Bbt0 at block %d (0x%x)\n",
                            g_main_bbt_ver, OFFSET_TO_BLOCK(g_main_bbt_addr), g_main_bbt_addr);
                break;
            }
            // todo: finish up the mirror block detection also
        }
        if (!g_main_bbt_addr) {
            diag_printf("No BBT table found. Need to do \"nand scan\" first\n");
            return -1;
        }
        buf = g_bbt;
    } else
        diag_printf("Do low level scan to construct BBT\n");

    // do some low level scan of each block and check for bad.
    for (i = 0; i < NF_BLK_CNT; i++) {
        if (nfc_is_badblock(i, buf)) {
            // construct the bad block table
            if (!buf)
                mark_blk_bad(i, g_bbt, BLK_BAD_FACTORY);
            bad++;
            diag_printf("Block %d is bad\n", i);
        }
    }

    diag_printf("Total bad blocks: %d\n", bad);
    if (g_nfc_debug_measure) {
        count2 = hal_timer_count();
        diag_printf("counter1=0x%x, counter2=0x%x, diff=0x%x\n",
                    count1, count2, count2 - count1);
        diag_printf("Using [diff * 1000000 / 32768] to get usec\n");
    }
    return bad;
}

////////////////////////// "nand" commands support /////////////////////////
// Image management functions
local_cmd_entry("info",
        "Show nand flash info (number of good/bad blocks)",
        "[-f <raw address>] [-l <length>]",
        nand_info,
        NAND_cmds
           );

local_cmd_entry("show",
        "Show a page main/spare areas or spare area only (-s)",
        "-f <raw page address> [-s]",
        nand_show,
        NAND_cmds
           );

local_cmd_entry("read",
        "Read data from nand flash into RAM",
        "-f <raw addr> -b <mem_load_addr> -l <byte len> [-c <col>]\n \
      Note -c is only for 2K-page for value <0, 2048+64-1>",
        nand_read,
        NAND_cmds
           );

local_cmd_entry("write",
        "Write data from RAM into nand flash",
        "-f <raw address> -b <memory_address> -l <image_length> [-c <col_addr>]",
        nand_write,
        NAND_cmds
           );

local_cmd_entry("erase",
        "Erase nand flash contents",
        "-f <raw address> -l <length> [-o] \n\
                -o: force erase (even for bad blocks)",
        nand_erase,
        NAND_cmds
           );

local_cmd_entry("scan",
        "Scan bad blocks and may also save bad block table into the NAND flash.",
        "[-o] [-r] \n\
                No argument: save existing bad block table (BBT) \n\
                -r: re-scan with ECC off and save BBT -- for brand NEW flash \n\
                -o: force erase all, reconstruct BBT (no ECC) and save BBT -- for development. ",
        nand_scan,
        NAND_cmds
           );

local_cmd_entry("debug",
        "Various NAND debug features ",
        "<0> min debug messages <default> \n\
             <1> med debug messages \n\
             <2> max debug messages \n\
             <3> enable(default)/disable h/w ECC for both r/w \n\
             <4> disable(default)/enalbe spare-only read \n\
             <9> enable/disable measurement \n\
             no parameter - display current debug setup",
        nand_debug_fun,
        NAND_cmds
           );

// Define table boundaries
CYG_HAL_TABLE_BEGIN( __NAND_cmds_TAB__, NAND_cmds);
CYG_HAL_TABLE_END( __NAND_cmds_TAB_END__, NAND_cmds);

extern struct cmd __NAND_cmds_TAB__[], __NAND_cmds_TAB_END__;

// CLI function
static cmd_fun do_nand_cmds;
RedBoot_nested_cmd("nand",
           "Utility function to NAND flash using raw address",
           "{cmds}",
           do_nand_cmds,
           __NAND_cmds_TAB__, &__NAND_cmds_TAB_END__
          );

static void nand_usage(char *why)
{
    diag_printf("*** invalid 'nand' command: %s\n", why);
    cmd_usage(__NAND_cmds_TAB__, &__NAND_cmds_TAB_END__, "nand ");
}

static u32 curr_addr;
static void nand_show(int argc, char *argv[])
{
    u32 ra;
    bool flash_addr_set = false;
    bool spar_only = false;
    struct option_info opts[2];

    init_opts(&opts[0], 'f', true, OPTION_ARG_TYPE_NUM,
          (void *)&ra, (bool *)&flash_addr_set, "NAND FLASH memory byte address");
    init_opts(&opts[1], 's', false, OPTION_ARG_TYPE_FLG,
          (void *)&spar_only, (bool *)0, "Spare only");

    if (!scan_opts(argc, argv, 2, opts, 2, 0, 0, "")) {
        return;
    }
    if (!flash_addr_set) {
        ra = curr_addr;
        curr_addr += NF_PG_SZ;
    } else {
        curr_addr = ra;
    }

    if (ra % NF_PG_SZ) {
        diag_printf("error: non-page aligned\n");
        return;
    }

    if (nfc_is_badblock(OFFSET_TO_BLOCK(ra), g_bbt)) {
        diag_printf("This is a bad block\n");
    }

    print_page(ra, spar_only);
}

/*!
 * For low level nand read command. It doesn't check for bad block or not
 */
static void nand_read(int argc, char *argv[])
{
    int len;
    u32 mem_addr, ra, col;
    bool mem_addr_set = false;
    bool flash_addr_set = false;
    bool length_set = false;
    bool col_set = false;
    struct option_info opts[4];
    int j = 0;
    bool ecc_status = g_ecc_enable;;

    init_opts(&opts[0], 'b', true, OPTION_ARG_TYPE_NUM,
              (void *)&mem_addr, (bool *)&mem_addr_set, "memory base address");
    init_opts(&opts[1], 'f', true, OPTION_ARG_TYPE_NUM,
              (void *)&ra, (bool *)&flash_addr_set, "FLASH memory base address");
    init_opts(&opts[2], 'l', true, OPTION_ARG_TYPE_NUM,
              (void *)&len, (bool *)&length_set, "image length [in FLASH]");
    init_opts(&opts[3], 'c', true, OPTION_ARG_TYPE_NUM,
              (void *)&col, (bool *)&col_set, "column addr");

    if (!scan_opts(argc, argv, 2, opts, 4, 0, 0, 0)) {
        nand_usage("invalid arguments");
        return;
    }

    if (!mem_addr_set || !flash_addr_set || !length_set) {
        nand_usage("required parameter missing");
        return;
    }
    if ((mem_addr < (CYG_ADDRESS)ram_start) ||
        ((mem_addr+len) >= (CYG_ADDRESS)ram_end)) {
        diag_printf("** WARNING: RAM address: %p may be invalid\n", (void *)mem_addr);
        diag_printf("   valid range is %p-%p\n", (void *)ram_start, (void *)ram_end);
    }

    // Safety check - make sure the address range is not within the code we're running
    if (flash_code_overlaps((void *)ra, (void *)(ra+len-1))) {
        diag_printf("Can't program this region - contains code in use!\n");
        return;
    }

    if (col_set) {
        u32 flash_addr;

        diag_printf("Random read at page %d, column %d\n",
                    ra / NF_PG_SZ, col);
        flash_addr = ((ra / NF_PG_SZ) << NAND_PG_SHIFT) + col;

        if (g_is_2k_page || g_is_4k_page) {
            g_ecc_enable = false;
        }
        nfc_read_pg_random(flash_addr, 0); // don't swap BI for 2k page
        if (g_is_2k_page || g_is_4k_page) {
            g_ecc_enable = ecc_status;
        }
        nfc_buf_mem_cpy((void *)mem_addr, (void *)NAND_MAIN_BUF0, NF_PG_SZ);
        return;
    }

    // insure integer multiple of page size
    len = (len + NF_PG_SZ - 1) & ~(NF_PG_SZ - 1);

    do {
        if (OFFSET_TO_BLOCK(ra) > (NF_BLK_CNT - 1)) {
            diag_printf("Out of range: addr=0x%x\n", ra);
            return;
        }
        if (nfc_read_page(ra) != 0) {
            diag_printf("Error %d: uncorrectable ECC at addr 0x%x\n", __LINE__, ra);
            diag_printf("should invoke bad block management to replace this block \n");
            diag_printf("and then mark this block \"bad\". But Redboot doesn't do it yet.\n");
        }
        if ((j++ % 0x20) == 0)
            diag_printf("\n%s 0x%08x: ", __FUNCTION__, ra);
        diag_printf(".");

        nfc_buf_mem_cpy((void *)mem_addr, (void *)NAND_MAIN_BUF0, NF_PG_SZ);

        ra += NF_PG_SZ;
        mem_addr += NF_PG_SZ;
        len -= NF_PG_SZ;
    } while (len > 0);
    diag_printf("\n");
}

static void nand_write(int argc, char *argv[])
{
    int len, j = 0;
    u32 mem_addr, ra, col;
    bool mem_addr_set = false;
    bool flash_addr_set = false;
    bool length_set = false;
    bool col_set = false;
    struct option_info opts[4];
    bool ecc_status = g_ecc_enable;;

    init_opts(&opts[0], 'b', true, OPTION_ARG_TYPE_NUM,
              (void *)&mem_addr, (bool *)&mem_addr_set, "memory base address");
    init_opts(&opts[1], 'f', true, OPTION_ARG_TYPE_NUM,
              (void *)&ra, (bool *)&flash_addr_set, "FLASH memory base address");
    init_opts(&opts[2], 'l', true, OPTION_ARG_TYPE_NUM,
              (void *)&len, (bool *)&length_set, "image length [in FLASH]");
    init_opts(&opts[3], 'c', true, OPTION_ARG_TYPE_NUM,
              (void *)&col, (bool *)&col_set, "column addr");
    if (!scan_opts(argc, argv, 2, opts, 4, 0, 0, 0))
    {
        nand_usage("invalid arguments");
        return;
    }

    if (!mem_addr_set || !flash_addr_set || !length_set) {
        nand_usage("required parameter missing");
        return;
    }

    if ((mem_addr < (CYG_ADDRESS)ram_start) ||
        ((mem_addr+len) >= (CYG_ADDRESS)ram_end)) {
        diag_printf("** WARNING: RAM address: %p may be invalid\n", (void *)mem_addr);
        diag_printf("   valid range is %p-%p\n", (void *)ram_start, (void *)ram_end);
    }

    if (col_set) {
        u32 flash_addr;

        diag_printf("Random write at page %d, column %d\n",
                    ra / NF_PG_SZ, col);
        flash_addr = ((ra / NF_PG_SZ) << NAND_PG_SHIFT) + col;

        if (g_is_2k_page || g_is_4k_page) {
            g_ecc_enable = false;
        }
        nfc_write_pg_random(flash_addr, mem_addr, NFC_MAIN_ONLY, 0);
        if (g_is_2k_page || g_is_4k_page) {
            g_ecc_enable = ecc_status;
        }
        return;
    }

    if ((len % NF_PG_SZ) != 0) {
        diag_printf("Not a full page write?\n\n");
    }

    do {
        if (OFFSET_TO_BLOCK(ra) > (NF_BLK_CNT - 1)) {
            diag_printf("Out of range: addr=0x%x\n", ra);
            return;
        }
        if (nfc_is_badblock(OFFSET_TO_BLOCK(ra), g_bbt)) {
            diag_printf("\nERROR: bad block at raw addr=0x%x(block=%d)\n",
                       ra, OFFSET_TO_BLOCK(ra));
            diag_printf("%s() failed\n", __FUNCTION__);
            return;
        }

        if (nfc_write_pg_random(NFC_ADJUST_ADDR(ra), mem_addr, NFC_MAIN_ONLY, 0) != 0) {
            if (g_nfc_debug_level >= NFC_DEBUG_DEF) {
                diag_printf("Error %d: program error at addr 0x%x\n", __LINE__, ra);
                diag_printf("should invoke bad block management to replace this block \n");
                diag_printf("and then mark this block \"bad\". But Redboot doesn't do it yet.\n");
            }
            return;
        }
        if ((j++ % 0x20) == 0)
            diag_printf("\nProgramming 0x%08x: ", ra);
        diag_printf(".");

        len -= NF_PG_SZ;
        ra += NF_PG_SZ;
        mem_addr += NF_PG_SZ;
    } while (len > 0);
    diag_printf("\n");
}

void nand_debug_fun(int argc, char *argv[])
{
    int opt;

    if (argc == 3) {
        opt = argv[2][0] - '0';
        switch (opt) {
        case 0:
            g_nfc_debug_level = NFC_DEBUG_MIN;
            break;
        case 1:
            g_nfc_debug_level = NFC_DEBUG_MED;
            break;
        case 2:
            g_nfc_debug_level = NFC_DEBUG_MAX;
            break;
        case 3:
            g_ecc_enable = g_ecc_enable? false: true;
            break;
        case 4:
            // toggle g_spare_only_read_ok
            g_spare_only_read_ok = g_spare_only_read_ok? false: true;
            break;
        case 9:
            g_nfc_debug_measure = g_nfc_debug_measure? false: true;
            break;

        default:
            diag_printf("%s(%s) not supported\n", __FUNCTION__, argv[2]);
            break;

        }
    }
    diag_printf("Current debug options are: \n");
    diag_printf("    h/w ECC: %s\n", g_ecc_enable ? "on":"off");
    diag_printf("    sp-only read: %s\n", g_spare_only_read_ok ? "on":"off");
    diag_printf("    measurement: %s\n", g_nfc_debug_measure ? "on":"off");
    diag_printf("    message level: %s\n", (g_nfc_debug_level == NFC_DEBUG_MIN) ? "min" : \
        ((g_nfc_debug_level == NFC_DEBUG_MED) ? "med" : "max"));
}

static void nand_erase(int argc, char *argv[])
{
    u32 len, ra;
    bool faddr_set = false;
    bool force_erase_set = false;
    bool length_set = false;
    struct option_info opts[4];

    init_opts(&opts[0], 'f', true, OPTION_ARG_TYPE_NUM,
          (void **)&ra, (bool *)&faddr_set, "FLASH memory base address");
    init_opts(&opts[1], 'l', true, OPTION_ARG_TYPE_NUM,
          (void **)&len, (bool *)&length_set, "length in bytes");
    init_opts(&opts[2], 'o', false, OPTION_ARG_TYPE_FLG,
          (void **)&force_erase_set, (bool *)&force_erase_set, "force erases block");

    if (!scan_opts(argc, argv, 2, opts, 4, 0, 0, 0)) {
        nand_usage("invalid arguments");
        return;
    }

    if (!faddr_set || !length_set) {
        nand_usage("missing argument");
        return;
    }
    if ((ra % NF_BLK_SZ) != 0 ||
        (len % NF_BLK_SZ) != 0 || len == 0) {
        diag_printf("Address or length is not block aligned or length is zero!\n");
        diag_printf("Block size is 0x%x\n", NF_BLK_SZ);
        return;
    }

    if (!verify_action("About to erase 0x%x bytes from nand offset 0x%x\n", len, ra)) {
        diag_printf("** Aborted\n");
        return;
    }

    // now ra is block aligned
    if (force_erase_set == true) {
        diag_printf("Force erase ...");
        nfc_erase_region(ra, len, 0, 1);
        diag_printf("\n");
    } else {
        nfc_erase_region(ra, len, 1, 1);
    }
    diag_printf("\n");
}

extern void romupdate(int argc, char *argv[]);
static void nand_scan(int argc, char *argv[])
{
    bool force_erase = false;
    bool force_rescan = false;
    struct option_info opts[2];

    init_opts(&opts[0], 'o', false, OPTION_ARG_TYPE_FLG,
          (void *)&force_erase, (bool *)0, "force erases block first");

    init_opts(&opts[1], 'r', false, OPTION_ARG_TYPE_FLG,
          (void *)&force_rescan, (bool *)0, "force low level re-scan");

    if (!scan_opts(argc, argv, 2, opts, 2, 0, 0, 0)) {
        nand_usage("invalid arguments");
        return;
    }

    if (!force_erase && !force_rescan && !g_main_bbt_addr) {
        diag_printf("Need to build BBT table first with \"nand scan [-o|-r]\"\n");
        return;
    }
    if (force_erase) {
        diag_printf("Force erase first ...\n");
        memset(g_bbt, -1, g_bbt_sz);
        // do force erase, including bad blocks. After this call, g_bbt should be re-built
        // for the whole NAND flash.
        nfc_erase_region(0, NF_DEV_SZ, 0, 1);
        g_main_bbt_addr = 0;
        diag_printf("\n");
    }
    if (force_rescan) {
        diag_printf("Force re-scan ...\n");
        memset(g_bbt, -1, g_bbt_sz);
        mxc_nfc_scan(true);
        g_main_bbt_addr = 0;
    }
    // program g_bbt into the flash
    diag_printf("Writing Bbt0 to flash\n");
    if (program_bbt_to_flash() != 0) {
        diag_printf("ERROR: TOO BAD! What can I do?\n");
    } else
        diag_printf("Format successful\n");

    if (force_erase) {
        romupdate(0, (char **)NULL);
    }
}

static void nand_info(int argc, char *argv[])
{
    u32 i, j = 0, len, ra;
    bool flash_addr_set = false;
    bool flash_len_set = false;
    struct option_info opts[2];

    init_opts(&opts[0], 'f', true, OPTION_ARG_TYPE_NUM,
          (void *)&ra, (bool *)&flash_addr_set, "NAND FLASH memory byte address");
    init_opts(&opts[1], 'l', true, OPTION_ARG_TYPE_NUM,
          (void *)&len, (bool *)&flash_len_set, "length");

    if (!scan_opts(argc, argv, 2, opts, 2, 0, 0, 0)) {
        nand_usage("invalid arguments");
        return;
    }

    if (nand_flash_index == -1) {
        diag_printf("Can't find valid NAND flash: %d\n", __LINE__);
        return;
    }

    if (argc == 2) {
        diag_printf("\nType:\t\t %s\n", NF_VEND_INFO);
        diag_printf("Total size:\t 0x%08x bytes (%d MB)\n", NF_DEV_SZ, NF_DEV_SZ/0x100000);
        diag_printf("Total blocks:\t 0x%x (%d)\n", NF_BLK_CNT, NF_BLK_CNT);
        diag_printf("Block size:\t 0x%x (%d)\n", NF_BLK_SZ, NF_BLK_SZ);
        diag_printf("Page size:\t 0x%x (%d)\n", NF_PG_SZ, NF_PG_SZ);
        diag_printf("Pages per block: 0x%x (%d)\n", NF_PG_PER_BLK, NF_PG_PER_BLK);
        return;
    }

    if (!flash_addr_set) {
        diag_printf("Wrong argument\n");
        return;
    }
    if (!flash_len_set) {
        len = NF_BLK_SZ;
    }

    if (ra % NF_BLK_SZ) {
        diag_printf("error: non-block aligned\n");
        return;
    }
    if (mxc_nfc_scan(false) == -1) {
        return;
    }
    diag_printf("\n");
    for (i = 0;
         i < ((len + NF_BLK_SZ - 1) / NF_BLK_SZ);
         i++) {

        if (nfc_is_badblock(OFFSET_TO_BLOCK(ra), g_bbt)) {
            diag_printf("block %d at offset 0x%x is bad\n",
                        OFFSET_TO_BLOCK(ra), ra);
            j++;
        }
        ra += NF_BLK_SZ;
    }
    diag_printf("==================================\n");
    diag_printf("Found %d bad block(s) out of %d\n\n", j, i);
}

static void do_nand_cmds(int argc, char *argv[])
{
    struct cmd *cmd;

    if (!mxcnfc_init_ok) {
        diag_printf("\nWarning:NAND flash hasn't been initialized. Try \"factive nand\" first\n\n");
        return;
    }

    if (argc < 2) {
        nand_usage("too few arguments");
        return;
    }
    if ((cmd = cmd_search(__NAND_cmds_TAB__, &__NAND_cmds_TAB_END__,
                          argv[1])) != (struct cmd *)0) {
        (cmd->fun)(argc, argv);
        return;
    }
    nand_usage("unrecognized command");
}

/*!
 * Display a memory region by 16-bit words
 * @param pkt   pointer to the starting address of the memory
 * @param len   byte length of the buffer to be displayed
 */
static void print_pkt_16(u16* pkt, u32 len)
{
    diag_printf("******************** %d bytes********************\n", len);
    u32 i = 0, tempLen = (len + 1) / 2;

    while(tempLen >= 0)
        {
        if(tempLen >= 8) {
            diag_printf("[%03x-%03x] ", i*2, ((i*2)+14));
            diag_printf("%04x %04x %04x %04x %04x %04x %04x %04x\n",
                        pkt[i], pkt[i+1], pkt[i+2], pkt[i+3],
                        pkt[i+4], pkt[i+5], pkt[i+6], pkt[i+7]);
        }
        else {
            if (tempLen == 0) {
                diag_printf("*************************************************\n");
                return;
            }
            diag_printf("[%03x-%03x] ", i*2, ((i*2)+14));
            switch(tempLen) {
                case 1:
                    diag_printf("%04x\n", pkt[i]);
                    break;
                case 2:
                    diag_printf("%04x %04x\n", pkt[i], pkt[i+1]);
                    break;
                case 3:
                    diag_printf("%04x %04x %04x\n", pkt[i], pkt[i+1], pkt[i+2]);
                    break;
                case 4:
                    diag_printf("%04x %04x %04x %04x\n", pkt[i],pkt[i+1], pkt[i+2],pkt[i+3]);
                    break;
                case 5:
                    diag_printf("%04x %04x %04x %04x %04x\n", pkt[i], pkt[i+1], pkt[i+2], pkt[i+3],pkt[i+4]);
                    break;
                case 6:
                    diag_printf("%04x %04x %04x %04x %04x %04x\n", pkt[i], pkt[i+1], pkt[i+2], pkt[i+3],pkt[i+4],
                             pkt[i+5]);
                    break;
                case 7:
                    diag_printf("%04x %04x %04x %04x %04x %04x %04x\n", pkt[i], pkt[i+1], pkt[i+2], pkt[i+3],pkt[i+4],
                             pkt[i+5], pkt[i+6]);
                    break;
            }
        }
        tempLen -= 8;
        i += 8;
    }
}

// addr = starting byte address within NAND flash
static void print_page (u32 addr, bool spare_only)
{
    u32 blk_num = OFFSET_TO_BLOCK(addr), pg_num = OFFSET_TO_PAGE(addr);

    if (spare_only) {
        if(nfc_read_page_sp(addr) != 0) {
            diag_printf("Error %d: uncorrectable. But still printing ...\n", __LINE__);
        }
    } else {
        if(nfc_read_page(addr) != 0) {
            diag_printf("Error %d: uncorrectable. But still printing ...\n", __LINE__);
        }
    }

    diag_printf("\n============ Printing block(%d) page(%d)  ==============\n",
                blk_num, pg_num);

    diag_printf("<<<<<<<<< spare area >>>>>>>>>\n");
    print_pkt_16((u16*)(NAND_SPAR_BUF0), 16);

    if (!spare_only) {
        diag_printf("<<<<<<<<< main area >>>>>>>>>\n");
        print_pkt_16((u16*)(NAND_MAIN_BUF0), NF_PG_SZ);
    }

    diag_printf("\n");
}

void rebuild_bbt(void)
{
    bool force_erase = true;
    bool force_rescan = true;

    if (!force_erase && !force_rescan && !g_main_bbt_addr) {
        diag_printf("Need to build BBT table first with \"nand scan [-o|-r]\"\n");
        return;
    }
    if (force_erase) {
        diag_printf("Force erase first ...\n");
        memset(g_bbt, -1, g_bbt_sz);
        // do force erase, including bad blocks. After this call, g_bbt should be re-built
        // for the whole NAND flash.
        nfc_erase_region(0, NF_DEV_SZ, 0, 1);
        g_main_bbt_addr = 0;
        diag_printf("\n");
    }
    if (force_rescan) {
        diag_printf("Force re-scan ...\n");
        memset(g_bbt, -1, g_bbt_sz);
        mxc_nfc_scan(true);
        g_main_bbt_addr = 0;
    }
    // program g_bbt into the flash
    diag_printf("Writing Bbt0 to flash\n");
    if (program_bbt_to_flash() != 0) {
        diag_printf("ERROR: TOO BAD! What can I do?\n");
    } else
        diag_printf("Format successful\n");
}

static int program_page_with_spare(u32 page_index, u32 buf)
{
    u32 ra = page_index * NF_PG_SZ;
    return nfc_write_pg_random(NFC_ADJUST_ADDR(ra), buf, NFC_MAIN_SPARE, 0);
}

#define NUM_PARTITIONS 5
static u32 g_boox_partitions[NUM_PARTITIONS + 1] = {0, 8, 16, 32, 56, 1336};
static u32 g_blocks_used[NUM_PARTITIONS] = {0, 0, 0, 0, 0};

void onyx_program_blk(u32 blk_index, u32 buf, u32 len, u8 with_spare)
{
    u8 i = 0;
    u32 next_good_block = 0;
    u32 partition_size = 0;

    for (i = 0; i < NUM_PARTITIONS; i++)
    {
        if (blk_index >= g_boox_partitions[i] && blk_index < g_boox_partitions[i+1])
        {
            break;
        }
    }

    /* Search next good block to write */
    partition_size = g_boox_partitions[i+1] - g_boox_partitions[i];
    while (g_blocks_used[i] < partition_size)
    {
        next_good_block = g_boox_partitions[i] + g_blocks_used[i];
        if (nfc_is_badblock(next_good_block, g_bbt))
        {
            diag_printf("Skip bad block %d\n", next_good_block);
            g_blocks_used[i]++;
        }
        else
        {
            break;
        }
    }

    if (g_blocks_used[i] == partition_size)
    {
        /* There is no available blocks in this partition */
        return;
    }

    diag_printf("Program block %d, len = %d\n", next_good_block, len);
    if (with_spare == 0)
    {
        nfc_program_blk(next_good_block * NF_BLK_SZ, buf, len);
    }
    else
    {
        u32 i, page_cnt = 0;
        if (len % (NF_PG_SZ + NF_SPARE_SIZE) != 0)
        {
            diag_printf("%s: invalid buffer length!\n", __FUNCTION__);
        }

        page_cnt = len / (NF_PG_SZ + NF_SPARE_SIZE);
        for (i = 0; i < page_cnt; i++)
        {
            program_page_with_spare(next_good_block * NF_PG_PER_BLK + i,
                                    buf + i * (NF_PG_SZ + NF_SPARE_SIZE));
        }
    }

    g_blocks_used[i]++;
}
