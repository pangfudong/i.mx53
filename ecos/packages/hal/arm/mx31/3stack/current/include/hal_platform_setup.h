#ifndef CYGONCE_HAL_PLATFORM_SETUP_H
#define CYGONCE_HAL_PLATFORM_SETUP_H

//=============================================================================
//
//      hal_platform_setup.h
//
//      Platform specific support for HAL (assembly code)
//
//=============================================================================
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
//===========================================================================

#include <pkgconf/system.h>             // System-wide configuration info
#include CYGBLD_HAL_VARIANT_H           // Variant specific configuration
#include CYGBLD_HAL_PLATFORM_H          // Platform specific configuration
#include <cyg/hal/hal_soc.h>            // Variant specific hardware definitions
#include <cyg/hal/hal_mmu.h>            // MMU definitions
#include <cyg/hal/fsl_board.h>          // Platform specific hardware definitions

#if defined(CYG_HAL_STARTUP_ROM) || defined(CYG_HAL_STARTUP_ROMRAM)
#define PLATFORM_SETUP1 _platform_setup1
#define CYGHWR_HAL_ARM_HAS_MMU

#ifdef CYG_HAL_STARTUP_ROMRAM
#define CYGSEM_HAL_ROM_RESET_USES_JUMP
#endif

//#define ARM_399MHZ
#define ARM_532MHZ

//#define NFC_2K_BI_SWAP
#define SDRAM_FULL_PAGE_BIT     0x100
#define SDRAM_FULL_PAGE_MODE    0x37
#define SDRAM_BURST_MODE        0x33

#define CYGHWR_HAL_ROM_VADDR    0x0

#if 0
#define UNALIGNED_ACCESS_ENABLE
#define SET_T_BIT_DISABLE
#define BRANCH_PREDICTION_ENABLE
#endif

//#define TURN_OFF_IMPRECISE_ABORT

// This macro represents the initial startup code for the platform
// r11 is reserved to contain chip rev info in this file
    .macro  _platform_setup1
FSL_BOARD_SETUP_START:
/*
 *       ARM1136 init
 *       - invalidate I/D cache/TLB and drain write buffer;
 *       - invalidate L2 cache
 *       - unaligned access
 *       - branch predictions
 */
#ifdef TURN_OFF_IMPRECISE_ABORT
    mrs r0, cpsr
    bic r0, r0, #0x100
    msr cpsr, r0
#endif

    mov r0, #0
    mcr 15, 0, r0, c7, c7, 0        /* invalidate I cache and D cache */
    mcr 15, 0, r0, c8, c7, 0        /* invalidate TLBs */
    mcr 15, 0, r0, c7, c10, 4       /* Drain the write buffer */

    /* Also setup the Peripheral Port Remap register inside the core */
    ldr r0, ARM_PPMRR        /* start from AIPS 2GB region */
    mcr p15, 0, r0, c15, c2, 4

    /* Reload data from spare area to 0x400 of main area if booting from NAND */
    mov r0, #NFC_BASE
    add r1, r0, #0x400
    cmp pc, r0
    blo 1f
    cmp pc, r1
    bhi 1f
#ifdef NFC_2K_BI_SWAP
    ldr r3, [r0, #0x7D0]    // load word at addr 464 of last 512 RAM buffer
    and r3, r3, #0xFFFFFF00 // mask off the LSB
    ldr r4, [r0, #0x834]    // load word at addr 4 of the 3rd spare area buffer
    mov r4, r4, lsr #8      // shift it to get the byte at addr 5
    and r4, r4, #0xFF       // throw away upper 3 bytes
    add r3, r4, r3          // construct the word
    str r3, [r0, #0x7D0]    // write back
#endif

1:
    /*** L2 Cache setup/invalidation/disable ***/
    /* Disable L2 cache first */
    mov r0, #L2CC_BASE_ADDR
    ldr r2, [r0, #L2_CACHE_CTL_REG]
    bic r2, r2, #0x1
    str r2, [r0, #L2_CACHE_CTL_REG]
    /*
     * Configure L2 Cache:
     * - 128k size(16k way)
     * - 8-way associativity
     * - 0 ws TAG/VALID/DIRTY
     * - 4 ws DATA R/W
     */
    ldr r1, [r0, #L2_CACHE_AUX_CTL_REG]
    and r1, r1, #0xFE000000
    ldr r2, L2CACHE_PARAM
    orr r1, r1, r2
    str r1, [r0, #L2_CACHE_AUX_CTL_REG]

    /* Invalidate L2 */
    mov r1, #0x000000FF
    str r1, [r0, #L2_CACHE_INV_WAY_REG]
L2_loop:
    /* Poll Invalidate By Way register */
    ldr r2, [r0, #L2_CACHE_INV_WAY_REG]
    cmp r2, #0
    bne L2_loop
    /*** End of L2 operations ***/

    mov r0, #SDRAM_NON_FLASH_BOOT
    ldr r1, AVIC_VECTOR0_ADDR_W
    str r0, [r1] // for checking boot source from nand or sdram
/*
 * End of ARM1136 init
 */
init_spba_start:
    init_spba
init_aips_start:
    init_aips
init_max_start:
    init_max
init_m3if_start:
    init_m3if

    ldr r11, =CHIP_REV_1_0
    ldr r0, IIM_SREV_REG_VAL
    ldr r1, [r0, #0x0]
    cmp r1, #0x0
    ldrne r11, =CHIP_REV_1_1
    init_drive_strength

    /* If SDRAM has been setup, bypass clock/WEIM setup */
    cmp pc, #SDRAM_BASE_ADDR
    blo init_clock_start
    cmp pc, #(SDRAM_BASE_ADDR + SDRAM_SIZE)
    blo HWInitialise_skip_SDRAM_setup

    mov r0, #NOR_FLASH_BOOT
    ldr r1, AVIC_VECTOR0_ADDR_W
    str r0, [r1]

init_cs5_start:
    init_cs5

init_clock_start:
    init_clock

    /* Based on chip rev, setup params for SDRAM controller */
    ldr r10, =0
    mov r4, #SDRAM_BURST_MODE

init_sdram_start:

    /* Assuming DDR memory first */
    setup_sdram ddr X32 DDR 0
#if 0
    beq HWInitialise_skip_SDRAM_setup
    setup_sdram ddr X16 DDR 0
    beq HWInitialise_skip_SDRAM_setup
    setup_sdram sdr X32 SDR 0
    beq HWInitialise_skip_SDRAM_setup
    setup_sdram sdr X16 SDR 0
    beq HWInitialise_skip_SDRAM_setup

    /* Reach hear means memory setup problem. Try to
     * increase the HCLK divider */
    ldr r0, CCM_BASE_ADDR_W
    ldr r1, [r0, #CLKCTL_PDR0]
    and r2, r1, #0x38
    cmp r2, #0x38
    beq loop_forever
    add r1, r1, #0x8
    str r1, [r0, #CLKCTL_PDR0]
    b init_sdram_start

loop_forever:
    b loop_forever  /* shouldn't get here */
#endif

HWInitialise_skip_SDRAM_setup:

    mov r0, #NFC_BASE
    add r2, r0, #0x800      // 2K window
    cmp pc, r0
    blo Normal_Boot_Continue
    cmp pc, r2
    bhi Normal_Boot_Continue
NAND_Boot_Start:
    /* Copy image from flash to SDRAM first */
    ldr r1, MXC_REDBOOT_ROM_START

1:  ldmia r0!, {r3-r10}
    stmia r1!, {r3-r10}
    cmp r0, r2
    blo 1b
    /* Jump to SDRAM */
    ldr r1, CONST_0x0FFF
    and r0, pc, r1     /* offset of pc */
    ldr r1, MXC_REDBOOT_ROM_START
    add r1, r1, #0x10
    add pc, r0, r1
    nop
    nop
    nop
    nop
NAND_Copy_Main:
    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)

    mov r0, #NAND_FLASH_BOOT
    ldr r1, AVIC_VECTOR0_ADDR_W
    str r0, [r1]
    mov r0, #MXCFIS_NAND
    ldr r1, AVIC_VECTOR1_ADDR_W
    str r0, [r1]

    mov r0, #NFC_BASE;   //r0: nfc base. Reloaded after each page copying
    mov r1, #0x800       //r1: starting flash addr to be copied. Updated constantly
    add r2, r0, #0x800   //r2: end of 3rd RAM buf. Doesn't change
    addeq r2, r0, #0x200   //r2: end of 1st RAM buf. Doesn't change
    add r12, r0, #0xE00  //r12: NFC register base. Doesn't change
    ldr r11, MXC_REDBOOT_ROM_START
    add r13, r11, #REDBOOT_IMAGE_SIZE //r13: end of SDRAM address for copying. Doesn't change
    add r11, r11, r1     //r11: starting SDRAM address for copying. Updated constantly

    //unlock internal buffer
    mov r3, #0x2
    strh r3, [r12, #0xA]

Nfc_Read_Page:
//  NFC_CMD_INPUT(FLASH_Read_Mode1);
    mov r3, #0x0
    nfc_cmd_input

    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)
    bne nfc_addr_ops_2kb
//    start_nfc_addr_ops(ADDRESS_INPUT_READ_PAGE, addr, nflash_dev_info->base_mask);
    mov r3, r1
    do_addr_input       //1st addr cycle
    mov r3, r1, lsr #9
    do_addr_input       //2nd addr cycle
    mov r3, r1, lsr #17
    do_addr_input       //3rd addr cycle
    mov r3, r1, lsr #25
    do_addr_input       //4th addr cycle
    b end_of_nfc_addr_ops

nfc_addr_ops_2kb:
//    start_nfc_addr_ops(ADDRESS_INPUT_READ_PAGE, addr, nflash_dev_info->base_mask);
    mov r3, #0
    do_addr_input       //1st addr cycle
    mov r3, #0
    do_addr_input       //2nd addr cycle
    mov r3, r1, lsr #11
    do_addr_input       //3rd addr cycle
    mov r3, r1, lsr #19
    do_addr_input       //4th addr cycle
    mov r3, r1, lsr #27
    do_addr_input       //5th addr cycle

//    NFC_CMD_INPUT(FLASH_Read_Mode1_2K);
    mov r3, #0x30
    nfc_cmd_input

end_of_nfc_addr_ops:
//    NFC_DATA_OUTPUT(buf, FDO_PAGE_SPARE_VAL);
//        writew(NAND_FLASH_CONFIG1_INT_MSK | NAND_FLASH_CONFIG1_ECC_EN,
//               NAND_FLASH_CONFIG1_REG);
    mov r8, #0
    bl nfc_data_output
    bl do_wait_op_done
    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)
    beq nfc_addr_data_output_done_512

// For 2K page - 2nd 512
    mov r8, #1
    bl nfc_data_output
    bl do_wait_op_done

// 3rd 512
    mov r8, #2
    bl nfc_data_output
    bl do_wait_op_done

// 4th 512
    mov r8, #3
    bl nfc_data_output
    bl do_wait_op_done
// end of 4th
#ifdef NFC_2K_BI_SWAP
    ldr r3, [r0, #0x7D0]    // load word at addr 464 of last 512 RAM buffer
    and r3, r3, #0xFFFFFF00 // mask off the LSB
    ldr r4, [r0, #0x834]    // load word at addr 4 of the 3rd spare area buffer
    mov r4, r4, lsr #8      // shift it to get the byte at addr 5
    and r4, r4, #0xFF       // throw away upper 3 bytes
    add r3, r4, r3          // construct the word
    str r3, [r0, #0x7D0]    // write back
#endif
    // check for bad block
    mov r3, r1, lsl #(32-17)    // get rid of block number
    cmp r3, #(0x800 << (32-17)) // check if not page 0 or 1
    b nfc_addr_data_output_done

nfc_addr_data_output_done_512:
    // check for bad block
    mov r3, r1, lsl #(32-5-9)    // get rid of block number
    cmp r3, #(512 << (32-5-9))   // check if not page 0 or 1

nfc_addr_data_output_done:
    bhi Copy_Good_Blk
    add r4, r0, #0x800  //r3 -> spare area buf 0
    ldrh r4, [r4, #0x4]
    and r4, r4, #0xFF00
    cmp r4, #0xFF00
    beq Copy_Good_Blk
    // really sucks. Bad block!!!!
    cmp r3, #0x0
    beq Skip_bad_block
    // even suckier since we already read the first page!
    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)

    subeq r11, r11, #512  //rewind 1 page for the sdram pointer
    subeq r1, r1, #512    //rewind 1 page for the flash pointer

    // for 2k page
    subne r11, r11, #0x800  //rewind 1 page for the sdram pointer
    subne r1, r1, #0x800    //rewind 1 page for the flash pointer

Skip_bad_block:
    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)

    addeq r1, r1, #(32*512)
    addne r1, r1, #(64*2048)

    b Nfc_Read_Page
Copy_Good_Blk:
    //copying page
1:  ldmia r0!, {r3-r10}
    stmia r11!, {r3-r10}
    cmp r0, r2
    blo 1b
    cmp r11, r13
    bge NAND_Copy_Main_done
    // Check if x16/2kb page
    ldr r7, CCM_BASE_ADDR_W
    ldr r7, [r7, #0xC]
    ands r7, r7, #(1 << 30)
    addeq r1, r1, #0x200
    addne r1, r1, #0x800
    mov r0, #NFC_BASE
    b Nfc_Read_Page

NAND_Copy_Main_done:

Normal_Boot_Continue:

#ifdef CYG_HAL_STARTUP_ROMRAM     /* enable running from RAM */
    /* Copy image from flash to SDRAM first */
    ldr r0, =0xFFFFF000
    and r0, r0, pc
    ldr r1, MXC_REDBOOT_ROM_START
    cmp r0, r1
    beq HWInitialise_skip_SDRAM_copy

    add r2, r0, #REDBOOT_IMAGE_SIZE

1:  ldmia r0!, {r3-r10}
    stmia r1!, {r3-r10}
    cmp r0, r2
    ble 1b
    /* Jump to SDRAM */
    ldr r1, =0xFFFF
    and r0, pc, r1         /* offset of pc */
    ldr r1, =(SDRAM_BASE_ADDR + SDRAM_SIZE - 0x100000 + 0x8)
    add pc, r0, r1
    nop
    nop
    nop
    nop
#endif /* CYG_HAL_STARTUP_ROMRAM */

HWInitialise_skip_SDRAM_copy:

NAND_ClockSetup:

/*
 * Note:
 *     IOMUX/PBC setup is done in C function plf_hardware_init() for simplicity
 */

STACK_Setup:
    // Set up a stack [for calling C code]
    ldr r1, =__startup_stack
    ldr r2, =RAM_BANK0_BASE
    orr sp, r1, r2

    // Create MMU tables
    bl hal_mmu_init

    // Enable MMU
    ldr r2, =10f
    mrc MMU_CP, 0, r1, MMU_Control, c0      // get c1 value to r1 first
    orr r1, r1, #7                          // enable MMU bit
    mcr MMU_CP, 0, r1, MMU_Control, c0
    mov pc,r2    /* Change address spaces */
    nop
    nop
    nop
10:

    // Save shadow copy of BCR, also hardware configuration
    ldr r1, =_board_BCR
    str r2, [r1]
    ldr r1, =_board_CFG
    str r9, [r1]                // Saved far above...

    .endm                       // _platform_setup1

do_wait_op_done:
    1:
        ldrh r3, [r12, #NAND_FLASH_CONFIG2_REG_OFF]
        ands r3, r3, #NAND_FLASH_CONFIG2_INT_DONE
        beq 1b
    bx lr     // do_wait_op_done

nfc_data_output:
    mov r3, #(NAND_FLASH_CONFIG1_INT_MSK | NAND_FLASH_CONFIG1_ECC_EN)
    strh r3, [r12, #NAND_FLASH_CONFIG1_REG_OFF]

    // writew(buf_no, RAM_BUFFER_ADDRESS_REG);
    strh r8, [r12, #RAM_BUFFER_ADDRESS_REG_OFF]
    // writew(FDO_PAGE_SPARE_VAL & 0xFF, NAND_FLASH_CONFIG2_REG);
    mov r3, #FDO_PAGE_SPARE_VAL
    strh r3, [r12, #NAND_FLASH_CONFIG2_REG_OFF]
    bx lr

#else // defined(CYG_HAL_STARTUP_ROM) || defined(CYG_HAL_STARTUP_ROMRAM)
#define PLATFORM_SETUP1
#endif

    /* Do nothing */
    .macro  init_spba
    .endm  /* init_spba */

    /* AIPS setup - Only setup MPROTx registers. The PACR default values are good.*/
    .macro init_aips
        /*
         * Set all MPROTx to be non-bufferable, trusted for R/W,
         * not forced to user-mode.
         */
        ldr r0, AIPS1_CTRL_BASE_ADDR_W
        ldr r1, AIPS1_PARAM_W
        str r1, [r0, #0x00]
        str r1, [r0, #0x04]
        ldr r0, AIPS2_CTRL_BASE_ADDR_W
        str r1, [r0, #0x00]
        str r1, [r0, #0x04]

        /*
         * Clear the on and off peripheral modules Supervisor Protect bit
         * for SDMA to access them. Did not change the AIPS control registers
         * (offset 0x20) access type
         */
        ldr r0, AIPS1_CTRL_BASE_ADDR_W
        ldr r1, =0x0
        str r1, [r0, #0x40]
        str r1, [r0, #0x44]
        str r1, [r0, #0x48]
        str r1, [r0, #0x4C]
        ldr r1, [r0, #0x50]
        and r1, r1, #0x00FFFFFF
        str r1, [r0, #0x50]

        ldr r0, AIPS2_CTRL_BASE_ADDR_W
        ldr r1, =0x0
        str r1, [r0, #0x40]
        str r1, [r0, #0x44]
        str r1, [r0, #0x48]
        str r1, [r0, #0x4C]
        ldr r1, [r0, #0x50]
        and r1, r1, #0x00FFFFFF
        str r1, [r0, #0x50]
    .endm /* init_aips */

    /* MAX (Multi-Layer AHB Crossbar Switch) setup */
    .macro init_max
        ldr r0, MAX_BASE_ADDR_W
        /* MPR - priority is M4 > M2 > M3 > M5 > M0 > M1 */
        ldr r1, MAX_PARAM1
        str r1, [r0, #0x000]        /* for S0 */
        str r1, [r0, #0x100]        /* for S1 */
        str r1, [r0, #0x200]        /* for S2 */
        str r1, [r0, #0x300]        /* for S3 */
        str r1, [r0, #0x400]        /* for S4 */
        /* SGPCR - always park on last master */
        ldr r1, =0x10
        str r1, [r0, #0x010]        /* for S0 */
        str r1, [r0, #0x110]        /* for S1 */
        str r1, [r0, #0x210]        /* for S2 */
        str r1, [r0, #0x310]        /* for S3 */
        str r1, [r0, #0x410]        /* for S4 */
        /* MGPCR - restore default values */
        ldr r1, =0x0
        str r1, [r0, #0x800]        /* for M0 */
        str r1, [r0, #0x900]        /* for M1 */
        str r1, [r0, #0xA00]        /* for M2 */
        str r1, [r0, #0xB00]        /* for M3 */
        str r1, [r0, #0xC00]        /* for M4 */
        str r1, [r0, #0xD00]        /* for M5 */
    .endm /* init_max */

    /* Clock setup */
    .macro    init_clock
        ldr r0, IPU_CTRL_BASE_ADDR_W
        ldr r1, =0x40
        str r1, [r0]

        ldr r0, CCM_BASE_ADDR_W
        ldr r2, CCM_CCMR_0x074B0BF5
        ldr r3, CCM_CCMR_0x074B0BFD
        ldr r4, CCM_CCMR_0x074B0B7D

        // Make sure to use CKIH
        ldr r1, [r0, #CLKCTL_CCMR]
        bic r1, r1, #0x8            // disable PLL first
        str r1, [r0, #CLKCTL_CCMR]
        str r2, [r0, #CLKCTL_CCMR]  // select CKIH (26MHz) as PLL ref clock
        ldr r1, =0x1000
    1:
        subs r1, r1, #0x1
        bne 1b
        str r3, [r0, #CLKCTL_CCMR]  // enable PLL
        str r4, [r0, #CLKCTL_CCMR]  // switch to PLL (SPLL for FIR)

        // 532-133-66.5
        ldr r1, CCM_PDR0_W
        str r1, [r0, #CLKCTL_PDR0]
        ldr r1, MPCTL_PARAM_W
        str r1, [r0, #CLKCTL_MPCTL]

        /* Set UPLL=240MHz, USB=60MHz */
        ldr r1, CCM_PDR1_0x49FCFE7F
        str r1, [r0, #CLKCTL_PDR1]
        ldr r1, CCM_UPCTL_PARAM_240
        str r1, [r0, #CLKCTL_UPCTL]
        // default CLKO to 1/8 of the ARM core
        mov r1, #0x000002C0
        add r1, r1, #0x00000006
        str r1, [r0, #CLKCTL_COSR]
    .endm /* init_clock */

    /* M3IF setup */
    .macro init_m3if
        /* Configure M3IF registers */
        ldr r1, M3IF_BASE_W
        /*
        * M3IF Control Register (M3IFCTL)
        * MRRP[0] = L2CC0 not on priority list (0 << 0)        = 0x00000000
        * MRRP[1] = L2CC1 not on priority list (0 << 0)        = 0x00000000
        * MRRP[2] = MBX not on priority list (0 << 0)        = 0x00000000
        * MRRP[3] = MAX1 not on priority list (0 << 0)        = 0x00000000
        * MRRP[4] = SDMA not on priority list (0 << 0)        = 0x00000000
        * MRRP[5] = MPEG4 not on priority list (0 << 0)       = 0x00000000
        * MRRP[6] = IPU1 on priority list (1 << 6)             = 0x00000040
        * MRRP[7] = IPU2 not on priority list (0 << 0)   = 0x00000000
        *                                                       ------------
        *                                                       0x00000040
        */
        ldr r0, =0x00000040
        str r0, [r1]  /* M3IF control reg */
    .endm /* init_m3if */

     /* CPLD on CS5 setup */
    .macro init_cs5
        ldr r0, WEIM_CTRL_CS5_W
        ldr r1, CS5_0x0000D843
        str r1, [r0, #CSCRU]
        ldr r1, CS5_0x22252521
        str r1, [r0, #CSCRL]
        ldr r1, CS5_0x22220A00
        str r1, [r0, #CSCRA]
    .endm /* init_cs5 */

    .macro setup_sdram, name, bus_width, mode, full_page
        /* It sets the "Z" flag in the CPSR at the end of the macro */
        ldr r0, ESDCTL_BASE_W
        mov r2, #SDRAM_BASE_ADDR
        ldr r1, SDRAM_0x0075E73A
        str r1, [r0, #0x4]
        ldr r1, =0x2            // reset
        str r1, [r0, #0x10]
        ldr r1, SDRAM_PARAM1_\mode
        str r1, [r0, #0x10]
        // Hold for more than 200ns
        ldr r1, =0x10000
1:
        subs r1, r1, #0x1
        bne 1b

        ldr r1, SDRAM_0x92100000
        str r1, [r0]
        ldr r1, =0x0
        ldr r12, SDRAM_PARAM2_\mode
        str r1, [r12]
        ldr r1, SDRAM_0xA2100000
        str r1, [r0]
        ldr r1, =0x0
        str r1, [r2]
        ldr r1, SDRAM_0xB2100000
        str r1, [r0]

        ldr r1, =0x0
        .if \full_page
        strb r1, [r2, #SDRAM_FULL_PAGE_MODE]
        .else
        strb r1, [r2, #SDRAM_BURST_MODE]
        .endif

        ldr r1, =0xFF
        ldr r12, =0x81000000
        strb r1, [r12]
        ldr r3, SDRAM_0x82116080
        ldr r4, SDRAM_PARAM3_\mode
        add r3, r3, r4
        ldr r4, SDRAM_PARAM4_\bus_width
        add r3, r3, r4
        .if \full_page
        add r3, r3, #0x100   /* Force to full page mode */
        .endif

        str r3, [r0]
        ldr r1, =0x0
        str r1, [r2]
        /* Below only for DDR */
        ldr r1, [r0, #0x10]
        ands r1, r1, #0x4
        ldrne r1, =0x0000000C
        strne r1, [r0, #0x10]
        /* Testing if it is truly DDR */
        ldr r1, SDRAM_0x55555555
        ldr r0, =SDRAM_BASE_ADDR
        str r1, [r0]
        ldr r2, SDRAM_0xAAAAAAAA
        str r2, [r0, #0x4]
        ldr r2, [r0]
        cmp r1, r2
    .endm

    .macro nfc_cmd_input
        strh r3, [r12, #NAND_FLASH_CMD_REG_OFF]
        mov r3, #NAND_FLASH_CONFIG2_FCMD_EN;
        strh r3, [r12, #NAND_FLASH_CONFIG2_REG_OFF]
        bl do_wait_op_done
    .endm   // nfc_cmd_input

    .macro do_addr_input
        and r3, r3, #0xFF
        strh r3, [r12, #NAND_FLASH_ADD_REG_OFF]
        mov r3, #NAND_FLASH_CONFIG2_FADD_EN
        strh r3, [r12, #NAND_FLASH_CONFIG2_REG_OFF]
        bl do_wait_op_done
    .endm   // do_addr_input

    /* To support 133MHz DDR */
    .macro  init_drive_strength
        /*
         * Disable maximum drive strength SDRAM/DDR lines by clearing DSE1 bits
         * in SW_PAD_CTL registers
         */

        // SDCLK
        ldr r1, IOMUXC_BASE_ADDR_W
        add r1, r1, #0x200
        // Now r1 = (IOMUX_BASE_ADDR + 0x200)
        ldr r0, [r1, #0x6C]
        bic r0, r0, #(1 << 12)
        str r0, [r1, #0x6C]

        // CAS
        ldr r0, [r1, #0x70]
        bic r0, r0, #(1 << 22)
        str r0, [r1, #0x70]

        // RAS
        ldr r0, [r1, #0x74]
        bic r0, r0, #(1 << 2)
        str r0, [r1, #0x74]

        // CS2 (CSD0)
        ldr r0, [r1, #0x7C]
        bic r0, r0, #(1 << 22)
        str r0, [r1, #0x7C]

        // DQM3
        ldr r0, [r1, #0x84]
        bic r0, r0, #(1 << 22)
        str r0, [r1, #0x84]

        // DQM2, DQM1, DQM0, SD31-SD0, A25-A0, MA10 (0x288..0x2DC)
        ldr r2, =22     // (0x2E0 - 0x288) / 4 = 22
pad_loop:
        ldr r0, [r1, #0x88]
        bic r0, r0, #(1 << 22)
        bic r0, r0, #(1 << 12)
        bic r0, r0, #(1 << 2)
        str r0, [r1, #0x88]
        add r1, r1, #4
        subs r2, r2, #0x1
        bne pad_loop
    .endm /* init_drive_strength */

#define PLATFORM_VECTORS         _platform_vectors
    .macro  _platform_vectors
        .globl  _board_BCR, _board_CFG
_board_BCR:   .long   0       // Board Control register shadow
_board_CFG:   .long   0       // Board Configuration (read at RESET)
    .endm

ARM_PPMRR:              .word   0x40000015
L2CACHE_PARAM:          .word   0x00030024
IIM_SREV_REG_VAL:       .word   IIM_BASE_ADDR + IIM_SREV_OFF
AIPS1_CTRL_BASE_ADDR_W: .word   AIPS1_CTRL_BASE_ADDR
AIPS2_CTRL_BASE_ADDR_W: .word   AIPS2_CTRL_BASE_ADDR
AIPS1_PARAM_W:          .word   0x77777777
MAX_BASE_ADDR_W:        .word   MAX_BASE_ADDR
MAX_PARAM1:             .word   0x00302154
CLKCTL_BASE_ADDR_W:     .word   CLKCTL_BASE_ADDR
ESDCTL_BASE_W:          .word   ESDCTL_BASE
M3IF_BASE_W:            .word   M3IF_BASE
SDRAM_PARAM1_DDR:	    .word	0x4
SDRAM_PARAM1_SDR:	    .word	0x0
SDRAM_PARAM2_DDR:	    .word	0x80000F00
SDRAM_PARAM2_SDR:	    .word	0x80000400
SDRAM_PARAM3_DDR:       .word   0x00100000
SDRAM_PARAM3_SDR:       .word   0x0
SDRAM_PARAM4_X32:       .word   0x00010000
SDRAM_PARAM4_X16:       .word   0x0
SDRAM_0x55555555:       .word   0x55555555
SDRAM_0xAAAAAAAA:       .word   0xAAAAAAAA
SDRAM_0x92100000:       .word   0x92100000
SDRAM_0xA2100000:       .word   0xA2100000
SDRAM_0xB2100000:       .word   0xB2100000
SDRAM_0x82116080:       .word   0x82116080
SDRAM_0x0075E73A:       .word   0x0075E73A
IOMUXC_BASE_ADDR_W:     .word   IOMUXC_BASE_ADDR
#ifdef ARM_399MHZ
CCM_PDR0_W:             .word   PDR0_399_133_66
MPCTL_PARAM_W:          .word   MPCTL_PARAM_399
#endif
#ifdef ARM_532MHZ
CCM_PDR0_W:             .word   PDR0_532_133_66
MPCTL_PARAM_W:          .word   MPCTL_PARAM_532
#endif

MPCTL_PARAM_532_27_W:   .word   MPCTL_PARAM_532_27
CCM_PDR1_0x49FCFE7F:    .word   0x49FCFE7F
CCM_UPCTL_PARAM_240:    .word   UPCTL_PARAM_240
CCM_UPCTL_PARAM_240_27: .word   UPCTL_PARAM_240_27
AVIC_VECTOR0_ADDR_W:    .word   MXCBOOT_FLAG_REG
AVIC_VECTOR1_ADDR_W:    .word   MXCFIS_FLAG_REG
MXC_REDBOOT_ROM_START:  .word   SDRAM_BASE_ADDR + SDRAM_SIZE - 0x100000
CONST_0x0FFF:           .word   0x0FFF
CCM_BASE_ADDR_W:        .word   CCM_BASE_ADDR
IPU_CTRL_BASE_ADDR_W:   .word   IPU_CTRL_BASE_ADDR
CCM_CCMR_0x074B0BF5:    .word   0x074B0BF5
CCM_CCMR_0x074B0BFD:    .word   0x074B0BFD
CCM_CCMR_0x074B0B7D:    .word   0x074B0B7D
WEIM_CTRL_CS5_W:    .word   WEIM_CTRL_CS5
CS5_0x0000D843:     .word   0x0000D843
CS5_0x22252521:     .word   0x22252521
CS5_0x22220A00:     .word   0x22220A00

/*---------------------------------------------------------------------------*/
/* end of hal_platform_setup.h                                               */
#endif /* CYGONCE_HAL_PLATFORM_SETUP_H */
