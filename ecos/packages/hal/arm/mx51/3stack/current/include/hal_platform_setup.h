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

//#define ENABLE_IMPRECISE_ABORT

// This macro represents the initial startup code for the platform
    .macro  _platform_setup1
FSL_BOARD_SETUP_START:
/*
 *       ARM1136 init
 *       - invalidate I/D cache/TLB and drain write buffer;
 *       - invalidate L2 cache
 *       - unaligned access
 *       - branch predictions
 */
    // mrc p15, 0, r0, c1, c1, 0 // Read Secure Configuration Register data. Why doesn't work???
    // mcr p15, 0, <Rd>, c1, c1, 0 ; Write Secure Configuration Register data
#ifdef ENABLE_IMPRECISE_ABORT
        mrs r1, spsr            // save old spsr
        mrs r0, cpsr            // read out the cpsr
        bic r0, r0, #0x100      // clear the A bit
        msr spsr, r0            // update spsr
        add lr, pc, #0x8        // update lr
        movs pc, lr             // update cpsr
        nop
        nop
        nop
        nop
        msr spsr, r1            // restore old spsr
#endif

    mov r0, #0
    mcr 15, 0, r0, c7, c7, 0        /* invalidate I cache and D cache */
    mcr 15, 0, r0, c8, c7, 0        /* invalidate TLBs */
    mcr 15, 0, r0, c7, c10, 4       /* Drain the write buffer */

    /* Also setup the Peripheral Port Remap register inside the core */
    ldr r0, ARM_PPMRR        /* start from AIPS 2GB region */
    mcr p15, 0, r0, c15, c2, 4

    /* Reload data from spare area to 0x400 of main area if booting from NAND */
    ldr r0, NFC_BASE_W
    cmp pc, r0
    blo 1f
    cmp pc, r1
    bhi 1f
#ifdef BARKER_CODE_SWAP_LOC
#if BARKER_CODE_SWAP_LOC != 0x404
#error FIXME: the following depends on barker code to be 0x404
#endif
    // Recover the word at 0x404 offset using the one stored in the spare area 0
    add r1, r0, #0x400
    add r1, r1, #0x4
    mov r3, #0x1000
    ldr r2, [r0, r3]
    str r2, [r1]
#endif
1:
#ifdef L2CC_ENABLED
    /*** L2 Cache setup/invalidation/disable ***/
    /* Disable L2 cache first */
    mov r0, #L2CC_BASE_ADDR
    mov r2, #0
    str r2, [r0, #L2_CACHE_CTL_REG]
    /*
     * Configure L2 Cache:
     * - 128k size(16k way)
     * - 8-way associativity
     * - 0 ws TAG/VALID/DIRTY
     * - 4 ws DATA R/W
     */
    mov r2, #0xFF000000
    add r2, r2, #0x00F00000
    ldr r1, [r0, #L2_CACHE_AUX_CTL_REG]
    and r1, r1, r2
    ldr r2, L2CACHE_PARAM
    orr r1, r1, r2
    str r1, [r0, #L2_CACHE_AUX_CTL_REG]

    /* Invalidate L2 */
    mov r1, #0xFF
    str r1, [r0, #L2_CACHE_INV_WAY_REG]
L2_loop:
    /* Poll Invalidate By Way register */
    ldr r2, [r0, #L2_CACHE_INV_WAY_REG]
    ands r2, r2, #0xFF
    bne L2_loop
    /*** End of L2 operations ***/
#endif

    /* Store the boot type, from NAND or SDRAM */
    mov r11, #SDRAM_NON_FLASH_BOOT
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
init_iomux_start:
    init_iomux

    /* If SDRAM has been setup, bypass clock/WEIM setup */
    cmp pc, #SDRAM_BASE_ADDR
    blo init_clock_start
    cmp pc, #(SDRAM_BASE_ADDR + SDRAM_SIZE)
    blo HWInitialise_skip_SDRAM_setup

init_clock_start:
    init_clock

    // disable wdog
    ldr r0, =0x30
    ldr r1, WDOG1_BASE_W
    strh r0, [r1]

    /* Based on chip rev, setup params for SDRAM controller */
    ldr r10, =0
    mov r4, #SDRAM_BURST_MODE

init_sdram_start:

    /* Assuming DDR memory first */
    setup_sdram

HWInitialise_skip_SDRAM_setup:
    ldr r0, NFC_BASE_W
    add r2, r0, #0x1000      // 4K window
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
    nop

NAND_Copy_Main:
    // Check if x16/2kb page
//    ldr r7, CCM_BASE_ADDR_W
//    ldr r7, [r7, #0xC]
//    ands r7, r7, #(1 << 30)
    ldr r0, NFC_BASE_W   //r0: nfc base. Reloaded after each page copying
    mov r1, #0x1000       //r1: starting flash addr to be copied. Updated constantly
                        // ???? should be dynamic based on the page size kevin todo
    add r2, r0, #0x1000   //r2: end of 3rd RAM buf. Doesn't change ?? dynamic

    ldr r11, NFC_IP_BASE_W  //r11: NFC IP register base. Doesn't change
    add r12, r0, #0x1E00  //r12: NFC AXI register base. Doesn't change
    ldr r14, MXC_REDBOOT_ROM_START
    add r13, r14, #REDBOOT_IMAGE_SIZE //r13: end of SDRAM address for copying. Doesn't change
    add r14, r14, r1     //r14: starting SDRAM address for copying. Updated constantly

    //unlock internal buffer
    mov r3, #0xFF000000
    add r3, r3, #0x00FF0000
    str r3, [r11, #0x4]
    str r3, [r11, #0x8]
    str r3, [r11, #0xC]
    str r3, [r11, #0x10]
    mov r3, #0x20000
    add r3, r3, #0x4
    str r3, [r11, #0x0]     // kevin - revist for multiple CS ??
    mov r3, #0
    str r3, [r11, #0x18]

Nfc_Read_Page:
//  writew(FLASH_Read_Mode1, NAND_ADD_CMD_REG);
    mov r3, #0x0;
    str r3, [r12, #0x0]
    mov r3, #NAND_LAUNCH_FCMD
    str r3, [r12, #0xC]

    do_wait_op_done
//    start_nfc_addr_ops(ADDRESS_INPUT_READ_PAGE, addr, nflash_dev_info->base_mask);
    mov r4, r1, lsl #1
    and r3, r4, #0xFF
    mov r3, r3, lsl #16
    do_addr_input       //1st addr cycle
    mov r3, r4, lsr #8
    and r3, r3, #0x1F
    mov r3, r3, lsl #16
    do_addr_input       //2nd addr cycle
    mov r3, r4, lsr #13
    and r3, r3, #0xFF
    mov r3, r3, lsl #16
    do_addr_input       //3rd addr cycle
    mov r3, r4, lsr #21
    and r3, r3, #0xFF
    mov r3, r3, lsl #16
    do_addr_input       //4th addr cycle
    mov r3, r4, lsr #29
    and r3, r3, #0xF
    mov r3, r3, lsl #16
    do_addr_input       //5th addr cycle TODO

//  writew(FLASH_Read_Mode1_2K, NAND_ADD_CMD_REG);
    mov r3, #0x30;
    str r3, [r12, #0x0]
    mov r3, #NAND_LAUNCH_FCMD
    str r3, [r12, #0xC]
    do_wait_op_done

// write RBA=0 to NFC_CONFIGURATION1
    mov r3, #0
    str r3, [r12, #0x4]

//    writel(mode & 0xFF, NAND_LAUNCH_REG);
    mov r3, #0x8
    str r3, [r12, #0xC]
//        wait_op_done();
    do_wait_op_done


Copy_Good_Blk:
    //copying page
1:  ldmia r0!, {r3-r10}
    stmia r14!, {r3-r10}
    cmp r0, r2
    blo 1b
    cmp r14, r13
    bge NAND_Copy_Main_done
    add r1, r1, #0x1000
    ldr r0, NFC_BASE_W
    b Nfc_Read_Page

NAND_Copy_Main_done:

    mov r11, #NAND_FLASH_BOOT

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
    ldr r1, =_mxc_boot
    str r11, [r1]

    .endm                       // _platform_setup1

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
#if 0
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
#endif
    .endm /* init_max */

    .macro    init_clock
        /*
         * Clock setup
         * After this step,

           Module           Freq (MHz)
           ===========================
           ARM core         532          ap_clk
           AHB              133          ahb_clk
           IP               66.5         ipg_clk
           EMI              133          ddr_clk

         * All other clocks can be figured out based on this.
         */
        /*
        * Step 1: Switch to step clock
        */
        ldr r0, CCM_BASE_ADDR_W
        mov r1, #0x00000104
        str r1, [r0, #CLKCTL_CCSR]

        /* Step 2: Setup PLL's */
        /* Set PLL1 to be 532MHz */
        ldr r0, PLL1_BASE_ADDR_W

        mov r1, #0x1200
        add r1, r1, #0x22
        str r1, [r0, #PLL_DP_CTL]     /* Set DPLL ON (set UPEN bit); BRMO=1 */
        ldr r1, =0x2
        str r1, [r0, #PLL_DP_CONFIG]  /* Enable auto-restart AREN bit */

        ldr r1, =0x50
        str r1, [r0, #PLL_DP_OP]
        ldr r1, =23
        str r1, [r0, #PLL_DP_MFD]
        ldr r1, =13
        str r1, [r0, #PLL_DP_MFN]

        ldr r1, =0x50
        str r1, [r0, #PLL_DP_HFS_OP]
        ldr r1, =23
        str r1, [r0, #PLL_DP_HFS_MFD]
        ldr r1, =13
        str r1, [r0, #PLL_DP_HFS_MFN]

        /* Now restart PLL 1 */
        ldr r1, PLL_VAL_0x1232
        str r1, [r0, #PLL_DP_CTL]
wait_pll0_lock:
        ldr r1, [r0, #PLL_DP_CTL]
        ands r1, r1, #0x1
        beq wait_pll0_lock

        /*
        * Step 2: Setup PLL2 to 665 MHz.
        */
        ldr r0, PLL2_BASE_ADDR_W

        ldr r1, =0x1200
        add r1, r1, #0x22
        str r1, [r0, #PLL_DP_CTL]     /* Set DPLL ON (set UPEN bit); BRMO=1 */
        ldr r1, =0x2
        str r1, [r0, #PLL_DP_CONFIG]  /* Enable auto-restart AREN bit */

        ldr r1, =0x60
        str r1, [r0, #PLL_DP_OP]
        ldr r1, =95
        str r1, [r0, #PLL_DP_MFD]
        ldr r1, =89
        str r1, [r0, #PLL_DP_MFN]

        ldr r1, =0x60
        str r1, [r0, #PLL_DP_HFS_OP]
        ldr r1, =95
        str r1, [r0, #PLL_DP_HFS_MFD]
        ldr r1, =89
        str r1, [r0, #PLL_DP_HFS_MFN]

        /* Now restart PLL 2 */
        ldr r1, PLL_VAL_0x1232
        str r1, [r0, #PLL_DP_CTL]
wait_pll1_lock:
        ldr r1, [r0, #PLL_DP_CTL]
        ands r1, r1, #0x1
        beq wait_pll1_lock

        /*
        * Set PLL 3 to 216MHz
        */
        ldr r0, PLL3_BASE_ADDR_W

        ldr r1, PLL_VAL_0x222
        str r1, [r0, #PLL_DP_CTL]     /* Set DPLL ON (set UPEN bit); BRMO=1 */
        ldr r1, =0x2
        str r1, [r0, #PLL_DP_CONFIG]  /* Enable auto-restart AREN bit */

        ldr r1, =0x91
        str r1, [r0, #PLL_DP_OP]
        ldr r1, =0x0
        str r1, [r0, #PLL_DP_MFD]
        ldr r1, =0x0
        str r1, [r0, #PLL_DP_MFN]

        ldr r1, =0x91
        str r1, [r0, #PLL_DP_HFS_OP]
        ldr r1, =0x0
        str r1, [r0, #PLL_DP_HFS_MFD]
        ldr r1, =0x0
        str r1, [r0, #PLL_DP_HFS_MFN]

        /* Now restart PLL 3 */
        ldr r1, PLL_VAL_0x232
        str r1, [r0, #PLL_DP_CTL]

wait_pll2_lock:
        ldr r1, [r0, #PLL_DP_CTL]
        ands r1, r1, #0x1
        beq wait_pll2_lock
        /* End of PLL 3 setup */

        /*
        * Step 3: switching to PLL 1 and restore default register values.
        */
        ldr r0, CCM_BASE_ADDR_W
        mov r1, #0x00000100
        str r1, [r0, #CLKCTL_CCSR]

        mov r1, #0x000A0000
        add r1, r1, #0x00000F0
        str r1, [r0, #CLKCTL_CCOSR]
        /* Use 133MHz for DDR clock */
        mov r1, #0x1C00
        str r1, [r0, #CLKCTL_CAMR]
        /* Use PLL 2 for UART's, get 66.5MHz from it */
        ldr r1, CCM_VAL_0xA5A6A020
        str r1, [r0, #CLKCTL_CSCMR1]
        ldr r1, CCM_VAL_0x01450321
        str r1, [r0, #CLKCTL_CSCDR1]

        mov r1, #0x1C
        str r1, [r0, #CLKCTL_CBCDR7]
        mov r1, #1
        str r1, [r0, #4]
    .endm /* init_clock */

    /* M3IF setup */
    .macro init_m3if
#if 0
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
#endif
    .endm /* init_m3if */

    .macro setup_sdram
        ldr r0, ESDCTL_BASE_W
        /* Set CSD0 */
        ldr r1, =0x80000000
        str r1, [r0, #ESDCTL_ESDCTL0]
        /* Precharge command */
        ldr r1, SDRAM_0x04008008
        str r1, [r0, #ESDCTL_ESDSCR]
        /* 2 refresh commands */
        ldr r1, SDRAM_0x00008010
        str r1, [r0, #ESDCTL_ESDSCR]
        str r1, [r0, #ESDCTL_ESDSCR]
        /* LMR with CAS=3 and BL=3 */
        ldr r1, SDRAM_0x00338018
        str r1, [r0, #ESDCTL_ESDSCR]
        /* 13 ROW, 10 COL, 32Bit, SREF=4 Micron Model */
        ldr r1, SDRAM_0xB2220000
        str r1, [r0, #ESDCTL_ESDCTL0]
        /* Timing parameters */
        ldr r1, SDRAM_0x899F6BBA
        str r1, [r0, #ESDCTL_ESDCFG0]
        /* MDDR enable, RLAT=2 */
        ldr r1, SDRAM_0x000A0104
        str r1, [r0, #ESDCTL_ESDMISC]
        /* Normal mode */
        ldr r1, =0x00000000
        str r1, [r0, #ESDCTL_ESDSCR]
    .endm

    .macro do_wait_op_done
    1:
        ldr r3, [r11, #0x18]
        ands r3, r3, #NFC_IPC_INT
        beq 1b
        mov r3, #0x0
        str r3, [r11, #0x18]
    .endm   // do_wait_op_done

    .macro do_addr_input
        str r3, [r12, #0x0]
        mov r3, #NAND_LAUNCH_FADD
        str r3, [r12, #0xC]
        do_wait_op_done
    .endm   // do_addr_input

    /* To support 133MHz DDR */
    .macro  init_iomux
        ldr r0, IOMUXC_BASE_ADDR_W
        // AP CKO/CKOH selected
        ldr r8, =0x1
        str r8, [r0, #0x228]

        // DDR signal setup for D16-D31 and drive strength
        ldr r8, =0x1
        add r1, r0, #8
        add r2, r0, #0x4C
    1:
        stmia r1!, {r8}
        cmp r1, r2
        bls 1b

        str r8, [r0, #0x5C]
        str r8, [r0, #0x60]

        ldr r8, =0x80
        add r1, r0, #230
        add r2, r0, #0x26C
    1:
        stmia r1!, {r8}
        cmp r1, r2
        bls 1b

    add r4, r0, #0x400
      ldr r8, =0x200
        str r8, [r4, #0xD8]
        str r8, [r4, #0xE8]
      ldr r8, =0
        str r8, [r4, #0xC4]
      ldr r8, =4
        str r8, [r4, #0xE4]
        str r8, [r4, #0xF4]

    add r2, r0, #0x200
      mov r8, #0x204
        str r8, [r2, #0xA8]
        str r8, [r2, #0xAC]
      mov r8, #0x2C4
        str r8, [r2, #0xBC]
        str r8, [r2, #0xC0]
    .endm /* init_iomux */

#define PLATFORM_VECTORS         _platform_vectors
    .macro  _platform_vectors
        .globl  _board_BCR, _board_CFG, _mxc_boot
_board_BCR:   .long   0       // Board Control register shadow
_board_CFG:   .long   0       // Board Configuration (read at RESET)
_mxc_boot:    .long   0       // Used to figure out boot type
    .endm

ARM_PPMRR:              .word   0x80000016
L2CACHE_PARAM:          .word   0x0003001B
WDOG1_BASE_W:           .word   WDOG1_BASE_ADDR
IIM_SREV_REG_VAL:       .word   IIM_BASE_ADDR + IIM_SREV_OFF
AIPS1_CTRL_BASE_ADDR_W: .word   AIPS1_CTRL_BASE_ADDR
AIPS2_CTRL_BASE_ADDR_W: .word   AIPS2_CTRL_BASE_ADDR
AIPS1_PARAM_W:          .word   0x77777777
MAX_BASE_ADDR_W:        .word   MAX_BASE_ADDR
MAX_PARAM1:             .word   0x00302154
ESDCTL_BASE_W:          .word   ESDCTL_BASE
M3IF_BASE_W:            .word   M4IF_BASE
NFC_BASE_W:             .word   NFC_BASE
NFC_IP_BASE_W:          .word   NFC_IP_BASE
SDRAM_0x04008008:       .word   0x04008008
SDRAM_0x00008010:       .word   0x00008010
SDRAM_0x00338018:       .word   0x00338018
SDRAM_0xB2220000:       .word   0xB2220000
SDRAM_0x899F6BBA:       .word   0x899F6BBA
SDRAM_0x000A0104:       .word   0x000A0104
IOMUXC_BASE_ADDR_W:     .word   IOMUXC_BASE_ADDR
MXC_REDBOOT_ROM_START:  .word   SDRAM_BASE_ADDR + SDRAM_SIZE - 0x100000
CONST_0x0FFF:           .word   0x0FFF
CCM_BASE_ADDR_W:        .word   CCM_BASE_ADDR
CCM_VAL_0x01450321:     .word   0x01450321
CCM_VAL_0xA5A6A020:     .word   0xA5A6A020
PLL_VAL_0x222:          .word   0x222
PLL_VAL_0x232:          .word   0x232
PLL1_BASE_ADDR_W:       .word   PLL1_BASE_ADDR
PLL2_BASE_ADDR_W:       .word   PLL2_BASE_ADDR
PLL3_BASE_ADDR_W:       .word   PLL3_BASE_ADDR
PLL_VAL_0x1232:         .word   0x1232

/*---------------------------------------------------------------------------*/
/* end of hal_platform_setup.h                                               */
#endif /* CYGONCE_HAL_PLATFORM_SETUP_H */
