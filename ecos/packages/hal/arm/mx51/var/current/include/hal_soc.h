//==========================================================================
//
//      hal_soc.h
//
//      SoC chip definitions
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
// Copyright (C) 2002 Gary Thomas
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

#ifndef __HAL_SOC_H__
#define __HAL_SOC_H__

#ifdef __ASSEMBLER__

#define REG8_VAL(a)          (a)
#define REG16_VAL(a)         (a)
#define REG32_VAL(a)         (a)

#define REG8_PTR(a)          (a)
#define REG16_PTR(a)         (a)
#define REG32_PTR(a)         (a)

#else /* __ASSEMBLER__ */

extern char HAL_PLATFORM_EXTRA[];
#define REG8_VAL(a)          ((unsigned char)(a))
#define REG16_VAL(a)         ((unsigned short)(a))
#define REG32_VAL(a)         ((unsigned int)(a))

#define REG8_PTR(a)          ((volatile unsigned char *)(a))
#define REG16_PTR(a)         ((volatile unsigned short *)(a))
#define REG32_PTR(a)         ((volatile unsigned int *)(a))
#define readb(a)             (*(volatile unsigned char *)(a))
#define readw(a)             (*(volatile unsigned short *)(a))
#define readl(a)             (*(volatile unsigned int *)(a))
#define writeb(v,a)          (*(volatile unsigned char *)(a) = (v))
#define writew(v,a)          (*(volatile unsigned short *)(a) = (v))
#define writel(v,a)          (*(volatile unsigned int *)(a) = (v))

#endif /* __ASSEMBLER__ */

/*
 * Default Memory Layout Definitions
 */

#define L2CC_BASE_ADDR          0xB0000000

/*
 * AIPS 1
 */
#define AIPS1_BASE_ADDR         0xC3F00000
#define AIPS1_CTRL_BASE_ADDR    AIPS1_BASE_ADDR
#define MAX_BASE_ADDR           0xC3F80000
#define GPIO1_BASE_ADDR         0xC3F84000
#define GPIO2_BASE_ADDR         0xC3F88000
#define GPIO3_BASE_ADDR         0xC3F8C000
#define KPP_BASE_ADDR           0xC3F94000
#define WDOG1_BASE_ADDR         0xC3F98000
#define WDOG_BASE_ADDR          WDOG1_BASE_ADDR
#define WDOG2_BASE_ADDR         0xC3F9C000
#define GPT1_BASE_ADDR          0xC3FA0000
#define RTC_BASE_ADDR           0xC3FA4000
#define IOMUXC_BASE_ADDR        0xC3FA8000
#define IIM_BASE_ADDR           0xC3FAC000
#define FEC_BASE_ADDR           0xC3FE8000

/*
 * SPBA
 */
#define MMC_SDHC1_BASE_ADDR     0xC0004000
#define MMC_SDHC2_BASE_ADDR     0xC0008000
#define UART3_BASE_ADDR         0xC000C000
#define CSPI2_BASE_ADDR         0xC0010000
#define SSI2_BASE_ADDR          0xC0014000
#define ATA_DMA_BASE_ADDR       0xC0034000
#define SPBA_CTRL_BASE_ADDR     0xC003C000

/*
 * AIPS 2
 */
#define AIPS2_BASE_ADDR         0xE3F00000
#define AIPS2_CTRL_BASE_ADDR    AIPS2_BASE_ADDR
#define PLL1_BASE_ADDR          0xE3F80000
#define PLL2_BASE_ADDR          0xE3F84000
#define PLL3_BASE_ADDR          0xE3F88000
#define CCM_BASE_ADDR           0xE3F8C000
#define SRC_BASE_ADDR           0xE3F94000
#define EPIT1_BASE_ADDR         0xE3F98000
#define EPIT2_BASE_ADDR         0xE3F9C000
#define CSPI3_BASE_ADDR         0xE3FA8000
#define CSPI1_BASE_ADDR         0xE3FAC000
#define UART1_BASE_ADDR         0xE3FB0000
#define UART2_BASE_ADDR         0xE3FBC000
#define I2C3_BASE_ADDR          0xE3FC0000
#define I2C2_BASE_ADDR          0xE3FC4000
#define I2C_BASE_ADDR           0xE3FC8000
#define SSI1_BASE_ADDR          0xE3FCC000
#define AUDMUX_BASE             0xE3FD0000

#define GPC_BASE_ADDR           (AIPS2_BASE_ADDR + 0x00090000)
#define GPC_CNTR_REG            (GPC_BASE_ADDR + 0)
#define GPC_PGR_REG             (GPC_BASE_ADDR + 4)
#define GPC_VCR_REG             (GPC_BASE_ADDR + 8)

#define PGC_BASE_VPU            (GPC_BASE_ADDR + 0x0240)
#define PGC_BASE_IPU            (GPC_BASE_ADDR + 0x0220)
#define GPC_PGR                 (GPC_BASE_ADDR + 0x000)
#define SRPGCR_ARM              (GPC_BASE_ADDR + 0x02A0 + 0x0000)
#define SRPGCR_EMI              (GPC_BASE_ADDR + 0x0280 + 0x0000)
#define PGC_PGCR_VPU            (PGC_BASE_VPU + 0x0000)
#define PGC_PGCR_IPU            (PGC_BASE_IPU + 0x0000)

#define PLATFORM_BASE_ADDR      0xB0404000
#define PLATFORM_LPC_REG        (PLATFORM_BASE_ADDR + 0x14)

/*
 * Interrupt controller
 */
#define INTC_BASE_ADDR          0xB0800000

/*
 * NAND, SDRAM, WEIM, M3IF, EMI controllers
 */
#define NFC_IP_BASE             0xE3FDB000
#define ESDCTL_BASE             0xE3FD9000
#define WEIM_BASE_ADDR          0xE3FDA000

#define WEIM_CTRL_CS0           WEIM_BASE_ADDR
#define WEIM_CTRL_CS1           (WEIM_BASE_ADDR + 0x18)
#define WEIM_CTRL_CS2           (WEIM_BASE_ADDR + 0x30)
#define WEIM_CTRL_CS3           (WEIM_BASE_ADDR + 0x48)
#define WEIM_CTRL_CS4           (WEIM_BASE_ADDR + 0x60)
#define WEIM_CTRL_CS5           (WEIM_BASE_ADDR + 0x78)
#define M4IF_BASE               0xE3FD8000

/*
 * Memory regions and CS
 */
#define CSD0_BASE_ADDR          0x40000000
#define CSD1_BASE_ADDR          0x50000000
#define CS0_BASE_ADDR           0x60000000
#define CS1_BASE_ADDR           0x68000000
#define CS2_BASE_ADDR           0x70000000

/*
 * IRQ Controller Register Definitions.
 */
#define INTC_NIMASK                     REG32_PTR(INTC_BASE_ADDR + (0x04))
#define INTC_INTTYPEH                   REG32_PTR(INTC_BASE_ADDR + (0x18))
#define INTC_INTTYPEL                   REG32_PTR(INTC_BASE_ADDR + (0x1C))

/* L210 */
#define L2_CACHE_LINE_SIZE              32
#define L2_CACHE_CTL_REG                0x100
#define L2_CACHE_AUX_CTL_REG            0x104
#define L2_CACHE_SYNC_REG               0x730
#define L2_CACHE_INV_LINE_REG           0x770
#define L2_CACHE_INV_WAY_REG            0x77C
#define L2_CACHE_CLEAN_LINE_PA_REG      0x7B0
#define L2_CACHE_CLEAN_LINE_WAY_REG     0x7B8
#define L2_CACHE_CLEAN_WAY_REG          0x7BC
#define L2_CACHE_CLEAN_INV_LINE_PA_REG  0x7F0
#define L2_CACHE_CLEAN_INV_LINE_WAY_REG 0x7F8
#define L2_CACHE_CLEAN_INV_WAY_REG      0x7FC

/* CCM */
#define CLKCTL_CCMR                     0x00
#define CLKCTL_PDR0                     0x04
#define CLKCTL_PDR1                     0x08

#define CLKCTL_CCSR                     0x0C
#define CLKCTL_CACRR                    0x10
#define CLKCTL_CBCDR2                   0x18
#define CLKCTL_CBCDR3                   0x1C
#define CLKCTL_CBCDR4                   0x20
#define CLKCTL_CBCDR5                   0x24
#define CLKCTL_CBCDR6                   0x28
#define CLKCTL_CBCDR7                   0x2C
#define CLKCTL_CAMR                     0x30
#define CLKCTL_PDR2                     0x64
#define CLKCTL_RCSR                     0x0C
#define CLKCTL_MPCTL                    0x10
#define CLKCTL_UPCTL                    0x14
#define CLKCTL_SPCTL                    0x18
#define CLKCTL_COSR                     0x1C
#define CLKCTL_CSCMR1                   0x34
#define CLKCTL_CSCDR1                   0x3C
#define CLKCTL_CS1CDR                   0x40
#define CLKCTL_CS2CDR                   0x44
#define CLKCTL_CSCDR2                   0x60
#define CLKCTL_CDCR                     0x6C
#define CLKCTL_CCOSR                    0x80

#define FREQ_24MHZ                      24000000
#define FREQ_32768HZ                    (32768 * 1024)
#define FREQ_38400HZ                    (38400 * 1024)
#define FREQ_32000HZ                    (32000 * 1024)
#define PLL_REF_CLK                     FREQ_24MHZ
//#define PLL_REF_CLK  FREQ_32768HZ
//#define PLL_REF_CLK  FREQ_32000HZ

/* WEIM registers */
#define CSGCR1                          0x00
#define CSGCR2                          0x04
#define CSRCR1                          0x08
#define CSRCR2                          0x0C
#define CSWCR1                          0x10

/* ESDCTL */
#define ESDCTL_ESDCTL0                  0x00
#define ESDCTL_ESDCFG0                  0x04
#define ESDCTL_ESDCTL1                  0x08
#define ESDCTL_ESDCFG1                  0x0C
#define ESDCTL_ESDMISC                  0x10
#define ESDCTL_ESDSCR                   0x14
#define ESDCTL_ESDCDLY1                 0x20
#define ESDCTL_ESDCDLY2                 0x24
#define ESDCTL_ESDCDLY3                 0x28
#define ESDCTL_ESDCDLY4                 0x2C
#define ESDCTL_ESDCDLY5                 0x30
#define ESDCTL_ESDCDLYGD                0x34

/* DPLL */
#define PLL_DP_CTL          0x00
#define PLL_DP_CONFIG       0x04
#define PLL_DP_OP           0x08
#define PLL_DP_MFD          0x0C
#define PLL_DP_MFN          0x10
#define PLL_DP_MFNMINUS     0x14
#define PLL_DP_MFNPLUS      0x18
#define PLL_DP_HFS_OP       0x1C
#define PLL_DP_HFS_MFD      0x20
#define PLL_DP_HFS_MFN      0x24
#define PLL_DP_TOGC         0x28
#define PLL_DP_DESTAT       0x2C

#define CHIP_REV_1_0            0x0      /* PASS 1.0 */
#define CHIP_REV_1_1            0x1      /* PASS 1.1 */
#define CHIP_REV_2_0            0x2      /* PASS 2.0 */
#define CHIP_LATEST             CHIP_REV_1_1

#define IIM_STAT_OFF            0x00
#define IIM_STAT_BUSY           (1 << 7)
#define IIM_STAT_PRGD           (1 << 1)
#define IIM_STAT_SNSD           (1 << 0)
#define IIM_STATM_OFF           0x04
#define IIM_ERR_OFF             0x08
#define IIM_ERR_PRGE            (1 << 7)
#define IIM_ERR_WPE         (1 << 6)
#define IIM_ERR_OPE         (1 << 5)
#define IIM_ERR_RPE         (1 << 4)
#define IIM_ERR_WLRE        (1 << 3)
#define IIM_ERR_SNSE        (1 << 2)
#define IIM_ERR_PARITYE     (1 << 1)
#define IIM_EMASK_OFF           0x0C
#define IIM_FCTL_OFF            0x10
#define IIM_UA_OFF              0x14
#define IIM_LA_OFF              0x18
#define IIM_SDAT_OFF            0x1C
#define IIM_PREV_OFF            0x20
#define IIM_SREV_OFF            0x24
#define IIM_PREG_P_OFF          0x28
#define IIM_SCS0_OFF            0x2C
#define IIM_SCS1_P_OFF          0x30
#define IIM_SCS2_OFF            0x34
#define IIM_SCS3_P_OFF          0x38

#define EPIT_BASE_ADDR          EPIT1_BASE_ADDR
#define EPITCR                  0x00
#define EPITSR                  0x04
#define EPITLR                  0x08
#define EPITCMPR                0x0C
#define EPITCNR                 0x10

#define GPT_BASE_ADDR           GPT1_BASE_ADDR
#define GPTCR                   0x00
#define GPTPR                   0x04
#define GPTSR                   0x08
#define GPTIR                   0x0C
#define GPTOCR1                 0x10
#define GPTOCR2                 0x14
#define GPTOCR3                 0x18
#define GPTICR1                 0x1C
#define GPTICR2                 0x20
#define GPTCNT                  0x24

/* Assuming 26MHz input clock */
/*                            PD             MFD              MFI          MFN */
#define MPCTL_PARAM_208     (((2-1) << 26) + ((1 -1) << 16) + (8  << 10) + (0  << 0))
#define MPCTL_PARAM_399     (((1-1) << 26) + ((52-1) << 16) + (7  << 10) + (35 << 0))
#define MPCTL_PARAM_532     (((1-1) << 26) + ((52-1) << 16) + (10 << 10) + (12 << 0))
#define MPCTL_PARAM_665     (((1-1) << 26) + ((52-1) << 16) + (12 << 10) + (41 << 0))
#define MPCTL_PARAM_532_27  (((1-1) << 26) + ((15-1) << 16) + (9  << 10) + (13 << 0))

/* UPCTL                      PD             MFD              MFI          MFN */
#define UPCTL_PARAM_288     (((1-1) << 26) + ((13-1) << 16) + (5  << 10) + (7  << 0))
#define UPCTL_PARAM_240     (((2-1) << 26) + ((13-1) << 16) + (9  << 10) + (3  << 0))
#define UPCTL_PARAM_240_27  (((2-1) << 26) + ((9 -1) << 16) + (8  << 10) + (8  << 0))

/* PDR0 */
#define PDR0_208_104_52     0xFF870D48  /* ARM=208MHz, HCLK=104MHz, IPG=52MHz */
#define PDR0_399_66_66      0xFF872B28  /* ARM=399MHz, HCLK=IPG=66.5MHz */
#define PDR0_399_133_66     0xFF871650  /* ARM=399MHz, HCLK=133MHz, IPG=66.5MHz */
#define PDR0_532_133_66     0xFF871D58  /* ARM=532MHz, HCLK=133MHz, IPG=66MHz */
#define PDR0_665_83_42      0xFF873B78  /* ARM=665MHz, HCLK=83MHz, IPG=42MHz */
#define PDR0_665_133_66     0xFF872560  /* ARM=665MHz, HCLK=133MHz, IPG=66MHz */

//#define BARKER_CODE_SWAP_LOC            0x404
#define BARKER_CODE_VAL                 0xB1
#define NFC_V2_1
#define NFC_BASE                        0x7FFF0000
#define NAND_REG_BASE                   (NFC_BASE + 0x1E00)

#define NAND_ADD_CMD_REG                (NAND_REG_BASE + 0x00)

#define NAND_CONFIGURATION1_REG         (NAND_REG_BASE + 0x04)
    #define NAND_CONFIGURATION1_NFC_RST     (1 << 2)
    #define NAND_CONFIGURATION1_NF_CE       (1 << 1)
    #define NAND_CONFIGURATION1_SP_EN       (1 << 0)

#define NAND_ECC_STATUS_RESULT_REG      (NAND_REG_BASE + 0x08)

#define NAND_LAUNCH_REG                 (NAND_REG_BASE + 0x0C)
    #define NAND_LAUNCH_FCMD                (1 << 0)
    #define NAND_LAUNCH_FADD                (1 << 1)
    #define NAND_LAUNCH_FDI                 (1 << 2)


#define NFC_WR_PROT_REG                 (NFC_IP_BASE + 0x00)
    #define NFC_WR_PROT_CS0              (0 << 20)
    #define NFC_WR_PROT_BLS_UNLOCK       (2 << 16)
    #define NFC_WR_PROT_WPC              (4 << 0)

#define UNLOCK_BLK_ADD0_REG             (NFC_IP_BASE + 0x04)

#define UNLOCK_BLK_ADD1_REG             (NFC_IP_BASE + 0x08)

#define UNLOCK_BLK_ADD2_REG             (NFC_IP_BASE + 0x0C)

#define UNLOCK_BLK_ADD3_REG             (NFC_IP_BASE + 0x10)

#define NFC_FLASH_CONFIG2_REG           (NFC_IP_BASE + 0x14)
    #define NFC_FLASH_CONFIG2_EDC0          (0 << 9)
    #define NFC_FLASH_CONFIG2_EDC1          (1 << 9)
    #define NFC_FLASH_CONFIG2_EDC2          (2 << 9)
    #define NFC_FLASH_CONFIG2_EDC3          (3 << 9)
    #define NFC_FLASH_CONFIG2_EDC4          (4 << 9)
    #define NFC_FLASH_CONFIG2_EDC5          (5 << 9)
    #define NFC_FLASH_CONFIG2_EDC6          (6 << 9)
    #define NFC_FLASH_CONFIG2_EDC7          (7 << 9)
    #define NFC_FLASH_CONFIG2_PPB_32        (0 << 7)
    #define NFC_FLASH_CONFIG2_PPB_64        (1 << 7)
    #define NFC_FLASH_CONFIG2_PPB_128       (2 << 7)
    #define NFC_FLASH_CONFIG2_PPB_256       (3 << 7)
    #define NFC_FLASH_CONFIG2_INT_MSK       (1 << 4)
    #define NFC_FLASH_CONFIG2_ECC_EN        (1 << 3)
    #define NFC_FLASH_CONFIG2_SYM           (1 << 2)

#define NFC_IPC_REG                     (NFC_IP_BASE + 0x18)
    #define NFC_IPC_INT                     (1 << 31)
    #define NFC_IPC_LPS                     (1 << 30)
    #define NFC_IPC_RB_B                    (1 << 29)
    #define NFC_IPC_CACK                    (1 << 1)
    #define NFC_IPC_CREQ                    (1 << 0)
#define NFC_AXI_ERR_ADD_REG             (NFC_IP_BASE + 0x1C)


#define NAND_FLASH_BOOT                 0x10000000
#define SDRAM_NON_FLASH_BOOT            0x20000000

#define MXCFIS_NOTHING                  0x00000000
#define MXCFIS_NAND                     0x10000000

#define IS_BOOTING_FROM_NAND()          (_mxc_boot == NAND_FLASH_BOOT)
// No NOR flash is supported under MX37 for booting
#define IS_BOOTING_FROM_NOR()           (0)
#define IS_BOOTING_FROM_SDRAM()         (_mxc_boot == SDRAM_NON_FLASH_BOOT)

#define IS_FIS_FROM_NAND()              1
#define IS_FIS_FROM_NOR()               0

/*
 * This macro is used to get certain bit field from a number
 */
#define MXC_GET_FIELD(val, len, sh)          ((val >> sh) & ((1 << len) - 1))

/*
 * This macro is used to set certain bit field inside a number
 */
#define MXC_SET_FIELD(val, len, sh, nval)    ((val & ~(((1 << len) - 1) << sh)) | (nval << sh))

#define L2CC_ENABLED
#define UART_WIDTH_32         /* internal UART is 32bit access only */

#if !defined(__ASSEMBLER__)
void cyg_hal_plf_serial_init(void);
void cyg_hal_plf_serial_stop(void);
void hal_delay_us(unsigned int usecs);
#define HAL_DELAY_US(n)     hal_delay_us(n)

enum plls {
    PLL1,
    PLL2,
    PLL3,
};

enum main_clocks {
        CPU_CLK,
        AHB_CLK,
        IPG_CLK,
        IPG_PER_CLK,
        DDR_CLK,
        NFC_CLK,
        USB_CLK,
};

enum peri_clocks {
        UART1_BAUD,
        UART2_BAUD,
        UART3_BAUD,
        SSI1_BAUD,
        SSI2_BAUD,
        CSI_BAUD,
        MSTICK1_CLK,
        MSTICK2_CLK,
        SPI1_CLK = CSPI1_BASE_ADDR,
        SPI2_CLK = CSPI2_BASE_ADDR,
};

unsigned int pll_clock(enum plls pll);

unsigned int get_main_clock(enum main_clocks clk);

unsigned int get_peri_clock(enum peri_clocks clk);

enum {
    MXC_NFC_V1,
    MXC_NFC_V2,
};

typedef unsigned int nfc_setup_func_t(unsigned int, unsigned int, unsigned int);

#endif //#if !defined(__ASSEMBLER__)

#define HAL_MMU_OFF() \
CYG_MACRO_START          \
    asm volatile (                                                      \
        "mcr p15, 0, r0, c7, c14, 0;"                                   \
        "mcr p15, 0, r0, c7, c10, 4;" /* drain the write buffer */      \
        "mcr p15, 0, r0, c7, c5, 0;" /* invalidate I cache */           \
        "mrc p15, 0, r0, c1, c0, 0;" /* read c1 */                      \
        "bic r0, r0, #0x7;" /* disable DCache and MMU */                \
        "bic r0, r0, #0x1000;" /* disable ICache */                     \
        "mcr p15, 0, r0, c1, c0, 0;" /*  */                             \
        "nop;" /* flush i+d-TLBs */                                     \
        "nop;" /* flush i+d-TLBs */                                     \
        "nop;" /* flush i+d-TLBs */                                     \
        :                                                               \
        :                                                               \
        : "r0","memory" /* clobber list */);                            \
CYG_MACRO_END

#endif /* __HAL_SOC_H__ */
