//==========================================================================
//
//      cmds.c
//
//      SoC [platform] specific RedBoot commands
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
#include <cyg/hal/plf_mmap.h>
#include <cyg/hal/hal_soc.h>         // Hardware definitions
#include <cyg/hal/hal_cache.h>

typedef unsigned long long  u64;
typedef unsigned int        u32;
typedef unsigned short      u16;
typedef unsigned char       u8;

#define SZ_DEC_1M       1000000
#define PLL_PD_MAX      16      //actual pd+1
#define PLL_MFI_MAX     15
#define PLL_MFI_MIN     5
#define PLL_MFD_MAX     1024    //actual mfd+1
#define PLL_MFN_MAX     511
#define PRESC_MAX       8
#define IPG_DIV_MAX     4
#define AHB_DIV_MAX     8
#define HSP_PODF_MAX    8
#define NFC_PODF_MAX    8

#define PLL_FREQ_MAX    (2 * PLL_REF_CLK * PLL_MFI_MAX)
#define PLL_FREQ_MIN    ((2 * PLL_REF_CLK * (PLL_MFI_MIN - 1)) / PLL_PD_MAX)
#define AHB_CLK_MAX     133333333
#define IPG_CLK_MAX     (AHB_CLK_MAX / 2)
#define NFC_CLK_MAX     25000000
// IPU-HSP clock is independent of the HCLK and can go up to 177MHz but requires
// higher voltage support. For simplicity, limit it to 133MHz
#define HSP_CLK_MAX     133333333

#define ERR_WRONG_CLK   -1
#define ERR_NO_MFI      -2
#define ERR_NO_MFN      -3
#define ERR_NO_PD       -4
#define ERR_NO_PRESC    -5
#define ERR_NO_AHB_DIV  -6

u32 pll_clock(enum plls pll);
u32 get_main_clock(enum main_clocks clk);
u32 get_peri_clock(enum peri_clocks clk);

static u32 pll_mfd_fixed;

static void clock_setup(int argc, char *argv[]);
static void clko(int argc, char *argv[]);
extern unsigned int g_clock_src;

RedBoot_cmd("clock",
            "Setup/Display clock (max AHB=133MHz, max IPG=66.5MHz)\nSyntax:",
            "[<core clock in MHz> [:<AHB-to-core divider>[:<IPG-to-AHB divider>]]] \n\n\
If a divider is zero or no divider is specified, the optimal divider values \n\
will be chosen. Examples:\n\
   [clock]         -> Show various clocks\n\
   [clock 532]     -> Core=532  AHB=133           IPG=66.5\n\
   [clock 399]     -> Core=399  AHB=133           IPG=66.5\n\
   [clock 532:8]   -> Core=532  AHB=66.5(Core/8)  IPG=66.5\n\
   [clock 532:8:2] -> Core=532  AHB=66.5(Core/8)  IPG=33.25(AHB/2)\n",
            clock_setup
           );

/*!
 * This is to calculate various parameters based on reference clock and
 * targeted clock based on the equation:
 *      t_clk = 2*ref_freq*(mfi + mfn/(mfd+1))/(pd+1)
 * This calculation is based on a fixed MFD value for simplicity.
 *
 * @param ref       reference clock freq
 * @param target    targeted clock in HZ
 * @param p_pd      calculated pd value (pd value from register + 1) upon return
 * @param p_mfi     calculated actual mfi value upon return
 * @param p_mfn     calculated actual mfn value upon return
 * @param p_mfd     fixed mfd value (mfd value from register + 1) upon return
 *
 * @return          0 if successful; non-zero otherwise.
 */
int calc_pll_params(u32 ref, u32 target, u32 *p_pd,
                    u32 *p_mfi, u32 *p_mfn, u32 *p_mfd)
{
    u64 pd, mfi, mfn, n_target = (u64)target, n_ref = (u64)ref;

    if (g_clock_src == FREQ_26MHZ) {
        pll_mfd_fixed = 26 * 16;
    } else if (g_clock_src == FREQ_27MHZ) {
        pll_mfd_fixed = 27 * 16;
    } else {
        pll_mfd_fixed = 1024;
    }

    // Make sure targeted freq is in the valid range. Otherwise the
    // following calculation might be wrong!!!
    if (target < PLL_FREQ_MIN || target > PLL_FREQ_MAX) {
        return ERR_WRONG_CLK;
    }
    // Use n_target and n_ref to avoid overflow
    for (pd = 1; pd <= PLL_PD_MAX; pd++) {
        mfi = (n_target * pd) / (2 * n_ref);
        if (mfi > PLL_MFI_MAX) {
            return ERR_NO_MFI;
        } else if (mfi < 5) {
            continue;
        }
        break;
    }
    // Now got pd and mfi already
    mfn = (((n_target * pd) / 2 - n_ref * mfi) * pll_mfd_fixed) / n_ref;
    // Check mfn within limit and mfn < denominator
    if (mfn > PLL_MFN_MAX || mfn >= pll_mfd_fixed) {
        return ERR_NO_MFN;
    }

    if (pd > PLL_PD_MAX) {
        return ERR_NO_PD;
    }
    *p_pd = (u32)pd;
    *p_mfi = (u32)mfi;
    *p_mfn = (u32)mfn;
    *p_mfd = pll_mfd_fixed;
    return 0;
}

/*!
 * This function assumes the expected core clock has to be changed by
 * modifying the PLL. This is NOT true always but for most of the times,
 * it is. So it assumes the PLL output freq is the same as the expected
 * core clock (presc=1) unless the core clock is less than PLL_FREQ_MIN.
 * In the latter case, it will try to increase the presc value until
 * (presc*core_clk) is greater than PLL_FREQ_MIN. It then makes call to
 * calc_pll_params() and obtains the values of PD, MFI,MFN, MFD based
 * on the targeted PLL and reference input clock to the PLL. Lastly,
 * it sets the register based on these values along with the dividers.
 * Note 1) There is no value checking for the passed-in divider values
 *         so the caller has to make sure those values are sensible.
 *      2) Also adjust the NFC divider such that the NFC clock doesn't
 *         exceed NFC_CLK_MAX.
 *      3) IPU HSP clock is independent of AHB clock. Even it can go up to
 *         177MHz for higher voltage, this function fixes the max to 133MHz.
 *      4) This function should not have allowed diag_printf() calls since
 *         the serial driver has been stoped. But leave then here to allow
 *         easy debugging by NOT calling the cyg_hal_plf_serial_stop().
 *
 * @param ref       pll input reference clock (32KHz or 26MHz)
 * @param core_clk  core clock in Hz
 * @param ahb_div   ahb divider to divide the core clock to get ahb clock
 *                  (ahb_div - 1) needs to be set in the register
 * @param ipg_div   ipg divider to divide the ahb clock to get ipg clock
 *                  (ipg_div - 1) needs to be set in the register
 # @return          0 if successful; non-zero otherwise
 */
int configure_clock(u32 ref, u32 core_clk, u32 ahb_div, u32 ipg_div)
{
    u32 pll, presc = 1, pd, mfi, mfn, mfd, brmo = 0, mpctl0, hsp_div;
    u32 pdr0, nfc_div;
    int ret, i;

    // assume pll default to core clock first
    pll = core_clk;
    // when core_clk >= PLL_FREQ_MIN, the presc can be 1.
    // Otherwise, need to calculate presc value below and adjust the targeted pll
    if (core_clk < PLL_FREQ_MIN) {
        for (presc = 1; presc <= PRESC_MAX; presc++) {
            if ((core_clk * presc) > PLL_FREQ_MIN) {
                break;
            }
        }
        if (presc == (PRESC_MAX + 1)) {
            diag_printf("can't make presc=%d\n", presc);
            return ERR_NO_PRESC;
        }
        pll = core_clk * presc;
    }
    // get hsp_div
    for (hsp_div = 1; hsp_div <= HSP_PODF_MAX; hsp_div++) {
        if ((pll / hsp_div) <= HSP_CLK_MAX) {
            break;
        }
    }
    if (hsp_div == (HSP_PODF_MAX + 1)) {
        diag_printf("can't make hsp_div=%d\n", hsp_div);
        return ERR_NO_PRESC;
    }

    // get nfc_div - make sure optimal NFC clock but less than NFC_CLK_MAX
    for (nfc_div = 1; nfc_div <= NFC_PODF_MAX; nfc_div++) {
        if ((pll / (ahb_div * nfc_div)) <= NFC_CLK_MAX) {
            break;
        }
    }

    // pll is now the targeted pll output. Use it along with ref input clock
    // to get pd, mfi, mfn, mfd
    if ((ret = calc_pll_params(ref, pll, &pd, &mfi, &mfn, &mfd)) != 0) {
        diag_printf("can't find pll parameters: %d\n", ret);
        return ret;
    }
#ifdef CMD_CLOCK_DEBUG
    diag_printf("ref=%d, pll=%d, pd=%d, mfi=%d,mfn=%d, mfd=%d\n",
                ref, pll, pd, mfi, mfn, mfd);
#endif

    // blindly increase divider first to avoid too fast ahbclk and ipgclk
    // in case the core clock increases too much
    pdr0 = readl(CCM_BASE_ADDR + CLKCTL_PDR0);
    pdr0 &= ~0x000000FF;
    // increase the dividers. should work even when core clock is 832 (26*2*16)MHz
    // which is unlikely true.
    pdr0 |= (1 << 6) | (6 << 3) | (0 << 0);
    writel(pdr0, CCM_BASE_ADDR + CLKCTL_PDR0);
    // calculate new pdr0
    pdr0 &= ~0x00003FFF;
    pdr0 |= ((hsp_div - 1) << 11) | ((nfc_div - 1) << 8) | ((ipg_div - 1) << 6) |
            ((ahb_div - 1) << 3) | ((presc - 1) << 0);

    // update PLL register
    if ((mfd >= (10 * mfn)) || ((10 * mfn) >= (9 * mfd)))
        brmo = 1;

    mpctl0 = readl(CCM_BASE_ADDR + CLKCTL_MPCTL);
    mpctl0 = (mpctl0 & 0x4000C000)  |
             (brmo << 31)           |
             ((pd - 1) << 26)       |
             ((mfd - 1) << 16)      |
             (mfi << 10)            |
             mfn;
    writel(mpctl0, CCM_BASE_ADDR + CLKCTL_MPCTL);
    writel(pdr0, CCM_BASE_ADDR + CLKCTL_PDR0);
    // add some delay for new values to take effect
    for (i = 0; i < 10000; i++);

    return 0;
}

static void clock_setup(int argc,char *argv[])
{
    u32 i, core_clk, ipg_div, data[3], temp, ahb_div, ahb_clk, ipg_clk;
    int ret;

    if (argc == 1)
        goto print_clock;
    if (g_clock_src == FREQ_27MHZ) {
        diag_printf("Error: clock setup is not supported for 27MHz source\n\n");
        return;
    }
    for (i = 0;  i < 3;  i++) {
        if (!parse_num(*(&argv[1]), (unsigned long *)&temp, &argv[1], ":")) {
            diag_printf("Error: Invalid parameter\n");
            return;
        }
        data[i] = temp;
    }

    core_clk = data[0] * SZ_DEC_1M;
    ahb_div = data[1];  // actual register field + 1
    ipg_div = data[2];  // actual register field + 1

    if (core_clk < (PLL_FREQ_MIN / PRESC_MAX) || core_clk > PLL_FREQ_MAX) {
        diag_printf("Targeted core clock should be within [%d - %d]\n",
                    PLL_FREQ_MIN / PRESC_MAX, PLL_FREQ_MAX);
        return;
    }

    // find the ahb divider
    if (ahb_div > AHB_DIV_MAX) {
        diag_printf("Invalid AHB divider: %d. Maximum value is %d\n",
                    ahb_div, AHB_DIV_MAX);
        return;
    }
    if (ahb_div == 0) {
        // no HCLK divider specified
        for (ahb_div = 1; ; ahb_div++) {
            if ((core_clk / ahb_div) <= AHB_CLK_MAX) {
                break;
            }
        }
    }
    if (ahb_div > AHB_DIV_MAX || (core_clk / ahb_div) > AHB_CLK_MAX) {
        diag_printf("Can't make AHB=%d since max=%d\n",
                    core_clk / ahb_div, AHB_CLK_MAX);
        return;
    }

    // find the ipg divider
    ahb_clk = core_clk / ahb_div;
    if (ipg_div > IPG_DIV_MAX) {
        diag_printf("Invalid IPG divider: %d. Maximum value is %d\n",
                    ipg_div, IPG_DIV_MAX);
        return;
    }
    if (ipg_div == 0) {
        ipg_div++;          // At least =1
        if (ahb_clk > IPG_CLK_MAX)
            ipg_div++;      // Make it =2
    }
    if (ipg_div > IPG_DIV_MAX || (ahb_clk / ipg_div) > IPG_CLK_MAX) {
        diag_printf("Can't make IPG=%d since max=%d\n",
                    (ahb_clk / ipg_div), IPG_CLK_MAX);
        return;
    }
    ipg_clk = ahb_clk / ipg_div;

    diag_printf("Trying to set core=%d ahb=%d ipg=%d...\n",
                core_clk, ahb_clk, ipg_clk);

    // stop the serial to be ready to adjust the clock
    hal_delay_us(100000);
    cyg_hal_plf_serial_stop();
    // adjust the clock
    ret = configure_clock(PLL_REF_CLK, core_clk, ahb_div, ipg_div);
    // restart the serial driver
    cyg_hal_plf_serial_init();
    hal_delay_us(100000);

    if (ret != 0) {
        diag_printf("Failed to setup clock: %d\n", ret);
        return;
    }
    diag_printf("\n<<<New clock setting>>>\n");

    // Now printing clocks
print_clock:
    diag_printf("\nMPLL\t\tUPLL\t\tSPLL\n");
    diag_printf("========================================\n");
    diag_printf("%-16d%-16d%-16d\n\n", pll_clock(MCU_PLL), pll_clock(USB_PLL),
                pll_clock(SER_PLL));
    diag_printf("CPU\t\tAHB\t\tIPG\t\tIPG_PER\n");
    diag_printf("========================================================\n");
    diag_printf("%-16d%-16d%-16d%-16d\n\n",
                get_main_clock(CPU_CLK),
                get_main_clock(AHB_CLK),
                get_main_clock(IPG_CLK),
                get_main_clock(IPG_PER_CLK));

    diag_printf("NFC\t\tUSB\t\tIPU-HSP\n");
    diag_printf("========================================\n");
    diag_printf("%-16d%-16d%-16d\n\n",
                get_main_clock(NFC_CLK),
                get_main_clock(USB_CLK),
                get_main_clock(HSP_CLK));

    diag_printf("UART1-5\t\tSSI1\t\tSSI2\t\tCSI\n");
    diag_printf("===========================================");
    diag_printf("=============\n");

    diag_printf("%-16d%-16d%-16d%-16d\n\n",
                get_peri_clock(UART1_BAUD),
                get_peri_clock(SSI1_BAUD),
                get_peri_clock(SSI2_BAUD),
                get_peri_clock(CSI_BAUD));

    diag_printf("FIRI\t\tSIM\t\tMSTICK1\t\tMSTICK2\n");
    diag_printf("===========================================");
    diag_printf("=============\n");

    diag_printf("%-16d%-16d%-16d%-16d\n\n",
                get_peri_clock(FIRI_BAUD),
                get_peri_clock(SIM_BAUD),
                get_peri_clock(MSTICK1_CLK),
                get_peri_clock(MSTICK2_CLK));
    diag_printf("IPG_PERCLK as baud clock for: UART1-5, I2C, SIM, OWIRE, SDHC");
    if (((readl(EPIT1_BASE_ADDR) >> 24) & 0x3) == 0x2) {
        diag_printf(", EPIT");
    }
    if (((readl(GPT1_BASE_ADDR) >> 6) & 0x7) == 0x2) {
        diag_printf("GPT,");
    }
    if (((readl(PWM_BASE_ADDR) >> 16) & 0x3) == 0x2) {
        diag_printf("PWM,");
    }
    diag_printf("\n");
}

/*!
 * This function returns the PLL output value in Hz based on pll.
 */
u32 pll_clock(enum plls pll)
{
    u64 mfi, mfn, mfd, pdf, ref_clk, pll_out, sign;
    u64 reg = readl(pll);

    pdf = (reg >> 26) & 0xF;
    mfd = (reg >> 16) & 0x3FF;
    mfi = (reg >> 10) & 0xF;
    mfi = (mfi <= 5) ? 5: mfi;
    mfn = reg & 0x3FF;
    sign = (mfn < 512) ? 0: 1;
    mfn = (mfn < 512) ? mfn: (1024 - mfn);

    ref_clk = g_clock_src;

    if (sign == 0) {
        pll_out = (2 * ref_clk * mfi + ((2 * ref_clk * mfn) / (mfd + 1))) /
                  (pdf + 1);
    } else {
        pll_out = (2 * ref_clk * mfi - ((2 * ref_clk * mfn) / (mfd + 1))) /
                  (pdf + 1);
    }

    return (u32)pll_out;
}

// The clocks are on by default. But need to setup the IOMUX
void clock_spi_enable(unsigned int spi_clk)
{
    u32 val;

    // Take care of SPI1 and SPI2
    val = readl(IOMUXC_BASE_ADDR + 0x80);
    writel((val & 0xFFFF) | 0x12120000, IOMUXC_BASE_ADDR + 0x80);

    writel(0x12123812, IOMUXC_BASE_ADDR + 0x84);

    val = readl(IOMUXC_BASE_ADDR + 0x88);
    writel((val & 0xFFFFFF00) | 0x12, IOMUXC_BASE_ADDR + 0x88);

    /* Select CSPI 1 signals */
    val = readl(IOMUXC_BASE_ADDR + 0x8);
    writel((val | 0x4), IOMUXC_BASE_ADDR + 0x8);

    val = readl(IOMUXC_BASE_ADDR + 0x78);
    writel((val & 0xFFFFFF) | 0x24000000, IOMUXC_BASE_ADDR + 0x78);

    val = readl(IOMUXC_BASE_ADDR + 0x7C);
    writel((val & 0xFFFFFF00) | 0x24, IOMUXC_BASE_ADDR + 0x7C);

    writel(0x12121212, IOMUXC_BASE_ADDR + 0x8C);

    val = readl(IOMUXC_BASE_ADDR + 0x88);
    writel((val & 0xFF) | 0x12121200, IOMUXC_BASE_ADDR + 0x88);
}

// The clocks are on by default. But need to setup the IOMUX
void mxc_i2c_init(unsigned int module_base)
{
    unsigned int val, reg;

    switch (module_base) {
    case I2C_BASE_ADDR:
        reg = IOMUXC_BASE_ADDR + 0xA0;
        val = (readl(reg) & 0xFFFF0000) | 0x1212; // func mode
        writel(val, reg);
        break;
    case I2C2_BASE_ADDR:
        reg = IOMUXC_BASE_ADDR + 0x88;
        val = (readl(reg) & 0xFFFFFF00) | 0x24; // alt mode 1
        writel(val, reg);
        reg = IOMUXC_BASE_ADDR + 0x84;
        val = (readl(reg) & 0x00FFFFFF) | 0x24000000; // alt mode 1
        writel(val, reg);
        break;
    case I2C3_BASE_ADDR:
        reg = IOMUXC_BASE_ADDR + 0x84;
        val = (readl(reg) & 0xFFFFFF00) | 0x24; // alt mode 1
        writel(val, reg);
        reg = IOMUXC_BASE_ADDR + 0x80;
        val = (readl(reg) & 0x00FFFFFF) | 0x24000000; // alt mode 1
        writel(val, reg);
        break;
    default:
        diag_printf("Invalide I2C base: 0x%x\n", module_base);
        return;
    }
}

/*!
 * This function returns the main clock value in Hz.
 */
u32 get_main_clock(enum main_clocks clk)
{
    u32 mcu_podf, max_pdf, ipg_pdf, nfc_pdf, hsp_podf, clk_sel;
    u32 pll, ret_val = 0, usb_prdf, usb_podf, pdf;
    u32 reg = readl(CCM_BASE_ADDR + CLKCTL_PDR0);
    u32 reg1 = readl(CCM_BASE_ADDR + CLKCTL_PDR1);
    u32 ccmr = readl(CCM_BASE_ADDR + CLKCTL_CCMR);
    u32 mpdr0 = readl(CCM_BASE_ADDR + CLKCTL_PDR0);

    switch (clk) {
    case CPU_CLK:
        mcu_podf = reg & 0x7;
        pll = pll_clock(MCU_PLL);
        ret_val = pll / (mcu_podf + 1);
        break;
    case AHB_CLK:
        max_pdf = (reg >> 3) & 0x7;
        pll = pll_clock(MCU_PLL);
        ret_val = pll / (max_pdf + 1);
        break;
    case HSP_CLK:
        hsp_podf = (reg >> 11) & 0x7;
        pll = pll_clock(MCU_PLL);
        ret_val = pll / (hsp_podf + 1);
        break;
    case MBX_CLK:
        max_pdf = (reg >> 3) & 0x7;
        pll = pll_clock(MCU_PLL);
        ret_val = (pll / (max_pdf + 1)) / 2;
        break;
    case IPG_CLK:
        max_pdf = (reg >> 3) & 0x7;
        ipg_pdf = (reg >> 6) & 0x3;
        pll = pll_clock(MCU_PLL);
        ret_val = pll / ((max_pdf + 1) * (ipg_pdf + 1));
        break;
    case IPG_PER_CLK:
        clk_sel = ccmr & (1 << 24);
        pdf = (mpdr0 >> 16) & 0x1F;
        if (clk_sel != 0) {
            // get the ipg_clk
            max_pdf = (reg >> 3) & 0x7;
            ipg_pdf = (reg >> 6) & 0x3;
            pll = pll_clock(MCU_PLL);
            ret_val = pll / ((max_pdf + 1) * (ipg_pdf + 1));
        } else {
            ret_val = pll_clock(USB_PLL) / (pdf + 1);
        }
        break;
    case NFC_CLK:
        nfc_pdf = (reg >> 8) & 0x7;
        max_pdf = (reg >> 3) & 0x7;
        pll = pll_clock(MCU_PLL);
        /* AHB/nfc_pdf */
        ret_val = pll / ((max_pdf + 1) * (nfc_pdf + 1));
        break;
    case USB_CLK:
        usb_prdf = reg1 >> 30;
        usb_podf = (reg1 >> 27) & 0x7;
        pll = pll_clock(USB_PLL);
        ret_val = pll / ((usb_prdf + 1) * (usb_podf + 1));
        break;
    default:
        diag_printf("Unknown clock: %d\n", clk);
        break;
    }

    return ret_val;
}

/*!
 * This function returns the peripheral clock value in Hz.
 */
u32 get_peri_clock(enum peri_clocks clk)
{
    u32 ret_val = 0, pdf, pre_pdf, clk_sel;
    u32 ccmr = readl(CCM_BASE_ADDR + CLKCTL_CCMR);
    u32 mpdr0 = readl(CCM_BASE_ADDR + CLKCTL_PDR0);
    u32 mpdr1 = readl(CCM_BASE_ADDR + CLKCTL_PDR1);
    u32 mpdr2 = readl(CCM_BASE_ADDR + CLKCTL_PDR2);

    switch (clk) {
    case UART1_BAUD:
    case UART2_BAUD:
    case UART3_BAUD:
    case UART4_BAUD:
    case UART5_BAUD:
    case SIM_BAUD:
        clk_sel = ccmr & (1 << 24);
        pdf = (mpdr0 >> 16) & 0x1F;
        ret_val = (clk_sel != 0) ? get_main_clock(IPG_CLK) :
                  pll_clock(USB_PLL) / (pdf + 1);
        break;
    case SSI1_BAUD:
        pre_pdf = (mpdr1 >> 6) & 0x7;
        pdf = mpdr1 & 0x3F;
        clk_sel = (ccmr >> 18) & 3;
        if (clk_sel == 0) {
            ret_val = pll_clock(MCU_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x1) {
            ret_val = pll_clock(USB_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x2) {
            ret_val = pll_clock(SER_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else {
            diag_printf("Error: Use reserved value for SSI1!\n");
            ret_val = 0;
        }
        break;
    case SSI2_BAUD:
        pre_pdf = (mpdr1 >> 15) & 0x7;
        pdf = (mpdr1 >> 9) & 0x3F;
        clk_sel = (ccmr >> 21) & 3;
        if (clk_sel == 0) {
            ret_val = pll_clock(MCU_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x1) {
            ret_val = pll_clock(USB_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x2) {
            ret_val = pll_clock(SER_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else {
            diag_printf("Error: Use reserved value for SSI2!\n");
            ret_val = 0;
        }
        break;
    case CSI_BAUD:
        clk_sel = ccmr & (1 << 25);
        pdf = (mpdr0 >> 23) & 0x1FF;
        ret_val = (clk_sel != 0) ? (pll_clock(SER_PLL) / (pdf + 1)) :
                  (pll_clock(USB_PLL) / (pdf + 1));
        break;
    case FIRI_BAUD:
        pre_pdf = (mpdr1 >> 24) & 0x7;
        pdf = (mpdr1 >> 18) & 0x3F;
        clk_sel = (ccmr >> 11) & 3;
        if (clk_sel == 0) {
            ret_val = pll_clock(MCU_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x1) {
            ret_val = pll_clock(USB_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else if (clk_sel == 0x2) {
            ret_val = pll_clock(SER_PLL) / ((pre_pdf + 1) * (pdf + 1));
        } else {
            diag_printf("Error: Use reserved value for SSI1!\n");
            ret_val = 0;
        }
        break;
    case MSTICK1_CLK:
        pdf = mpdr2 & 0x3F;
        ret_val = pll_clock(USB_PLL) / (pdf + 1);
        break;
    case MSTICK2_CLK:
        pdf = (mpdr2 >> 7) & 0x3F;
        ret_val = pll_clock(USB_PLL) / (pdf + 1);
        break;
    case SPI1_CLK:
    case SPI2_CLK:
        ret_val = get_main_clock(IPG_CLK);
        break;
    default:
        diag_printf("%s(): This clock: %d not supported yet \n",
                    __FUNCTION__, clk);
        break;
    }

    return ret_val;
}

RedBoot_cmd("clko",
            "Select clock source for CLKO (J11 on the CPU daughter card)",
            " Default is 1/8 of ARM core\n\
          <0> - display current clko selection \n\
          <1> - mpl_dpdgck_clk (MPLL) \n\
          <2> - ipg_clk_ccm (IPG) \n\
          <3> - upl_dpdgck_clk (UPLL) \n\
          <4> - pll_ref_clk \n\
          <5> - fpm_ckil512_clk \n\
          <6> - ipg_clk_ahb_arm (AHB) \n\
          <7> - ipg_clk_arm (ARM) \n\
          <8> - spl_dpdgck_clk (SPLL) \n\
          <9> - ckih \n\
          <10> - ipg_clk_ahb_emi_clk \n\
          <11> - ipg_clk_ipu_hsp \n\
          <12> - ipg_clk_nfc_20m \n\
          <13> - ipg_clk_perclk_uart1 (IPG_PER)",
            clko
           );

static u8* clko_name[] ={
    "NULL",
    "1/8 of mpl_dpdgck_clk (MPLL)",
    "ipg_clk_ccm (IPG)",
    "1/8 of upl_dpdgck_clk (UPLL)",
    "pll_ref_clk",
    "fpm_ckil512_clk",
    "ipg_clk_ahb_arm (AHB)",
    "1/8 of ipg_clk_arm (ARM)",
    "1/8 of spl_dpdgck_clk (SPLL)",
    "ckih",
    "ipg_clk_ahb_emi_clk",
    "ipg_clk_ipu_hsp",
    "ipg_clk_nfc_20m",
    "ipg_clk_perclk_uart1 (IPG_PER)",
};

#define CLKO_MAX_INDEX          (sizeof(clko_name) / sizeof(u8*))

static void clko(int argc,char *argv[])
{
    u32 action = 0, cosr;

    if (!scan_opts(argc, argv, 1, 0, 0, (void*) &action,
                   OPTION_ARG_TYPE_NUM, "action"))
        return;

    if (action >= CLKO_MAX_INDEX) {
        diag_printf("%d is not supported\n\n", action);
        return;
    }

    cosr = readl(CCM_BASE_ADDR + CLKCTL_COSR);

    if (action != 0) {
        cosr = (cosr & (~0x1FF)) + action - 1;
        if (action == 1 || action == 3 || action == 7 || action == 8) {
            cosr |= (0x3 << 6); // make it divided by 8
        }
        writel(cosr, CCM_BASE_ADDR + CLKCTL_COSR);
        diag_printf("Set clko to ");
    }

    cosr = readl(CCM_BASE_ADDR + CLKCTL_COSR);
    diag_printf("%s\n", clko_name[(cosr & 0xF) + 1]);
    diag_printf("COSR register[0x%x] = 0x%x\n",
                (CCM_BASE_ADDR + CLKCTL_COSR), cosr);
}

#ifdef L2CC_ENABLED
/*
 * This command is added for some simple testing only. It turns on/off
 * L2 cache regardless of L1 cache state. The side effect of this is
 * when doing any flash operations such as "fis init", the L2
 * will be turned back on along with L1 caches even though it is off
 * by using this command.
 */
RedBoot_cmd("L2",
            "L2 cache",
            "[ON | OFF]",
            do_L2_caches
           );

void do_L2_caches(int argc, char *argv[])
{
    u32 oldints;
    int L2cache_on=0;

    if (argc == 2) {
        if (strcasecmp(argv[1], "on") == 0) {
            HAL_DISABLE_INTERRUPTS(oldints);
            HAL_ENABLE_L2();
            HAL_RESTORE_INTERRUPTS(oldints);
        } else if (strcasecmp(argv[1], "off") == 0) {
            HAL_DISABLE_INTERRUPTS(oldints);
            HAL_CLEAN_INVALIDATE_L2();
            HAL_DISABLE_L2();
            HAL_RESTORE_INTERRUPTS(oldints);
        } else {
            diag_printf("Invalid L2 cache mode: %s\n", argv[1]);
        }
    } else {
        HAL_L2CACHE_IS_ENABLED(L2cache_on);
        diag_printf("L2 cache: %s\n", L2cache_on?"On":"Off");
    }
}
#endif //L2CC_ENABLED

#define IIM_ERR_SHIFT       8
#define POLL_FUSE_PRGD      (IIM_STAT_PRGD | (IIM_ERR_PRGE << IIM_ERR_SHIFT))
#define POLL_FUSE_SNSD      (IIM_STAT_SNSD | (IIM_ERR_SNSE << IIM_ERR_SHIFT))

static void fuse_op_start(void)
{
    /* Do not generate interrupt */
    writel(0, IIM_BASE_ADDR + IIM_STATM_OFF);
    // clear the status bits and error bits
    writel(0x3, IIM_BASE_ADDR + IIM_STAT_OFF);
    writel(0xFE, IIM_BASE_ADDR + IIM_ERR_OFF);
}

/*
 * The action should be either:
 *          POLL_FUSE_PRGD
 * or:
 *          POLL_FUSE_SNSD
 */
static int poll_fuse_op_done(int action)
{

    u32 status, error;

    if (action != POLL_FUSE_PRGD && action != POLL_FUSE_SNSD) {
        diag_printf("%s(%d) invalid operation\n", __FUNCTION__, action);
        return -1;
    }

    /* Poll busy bit till it is NOT set */
    while ((readl(IIM_BASE_ADDR + IIM_STAT_OFF) & IIM_STAT_BUSY) != 0 ) {
    }

    /* Test for successful write */
    status = readl(IIM_BASE_ADDR + IIM_STAT_OFF);
    error = readl(IIM_BASE_ADDR + IIM_ERR_OFF);

    if ((status & action) != 0 && (error & (action >> IIM_ERR_SHIFT)) == 0) {
        if (error) {
            diag_printf("Even though the operation seems successful...\n");
            diag_printf("There are some error(s) at addr=0x%x: 0x%x\n",
                        (IIM_BASE_ADDR + IIM_ERR_OFF), error);
        }
        return 0;
    }
    diag_printf("%s(%d) failed\n", __FUNCTION__, action);
    diag_printf("status address=0x%x, value=0x%x\n",
                (IIM_BASE_ADDR + IIM_STAT_OFF), status);
    diag_printf("There are some error(s) at addr=0x%x: 0x%x\n",
                (IIM_BASE_ADDR + IIM_ERR_OFF), error);
    return -1;
}

static void sense_fuse(int bank, int row, int bit)
{
    int addr, addr_l, addr_h, reg_addr;

    fuse_op_start();

    addr = ((bank << 11) | (row << 3) | (bit & 0x7));
    /* Set IIM Program Upper Address */
    addr_h = (addr >> 8) & 0x000000FF;
    /* Set IIM Program Lower Address */
    addr_l = (addr & 0x000000FF);

#ifdef IIM_FUSE_DEBUG
    diag_printf("%s: addr_h=0x%x, addr_l=0x%x\n",
                __FUNCTION__, addr_h, addr_l);
#endif
    writel(addr_h, IIM_BASE_ADDR + IIM_UA_OFF);
    writel(addr_l, IIM_BASE_ADDR + IIM_LA_OFF);
    /* Start sensing */
    writel(0x8, IIM_BASE_ADDR + IIM_FCTL_OFF);
    if (poll_fuse_op_done(POLL_FUSE_SNSD) != 0) {
        diag_printf("%s(bank: %d, row: %d, bit: %d failed\n",
                    __FUNCTION__, bank, row, bit);
    }
    reg_addr = IIM_BASE_ADDR + IIM_SDAT_OFF;
    diag_printf("fuses at (bank:%d, row:%d) = 0x%x\n", bank, row, readl(reg_addr));
}

void do_fuse_read(int argc, char *argv[])
{
    int bank, row;

    if (argc == 1) {
        diag_printf("Useage: fuse_read <bank> <row>\n");
        return;
    } else if (argc == 3) {
        if (!parse_num(*(&argv[1]), (unsigned long *)&bank, &argv[1], " ")) {
                diag_printf("Error: Invalid parameter\n");
            return;
        }
        if (!parse_num(*(&argv[2]), (unsigned long *)&row, &argv[2], " ")) {
                diag_printf("Error: Invalid parameter\n");
                return;
            }

        diag_printf("Read fuse at bank:%d row:%d\n", bank, row);
        sense_fuse(bank, row, 0);

    } else {
        diag_printf("Passing in wrong arguments: %d\n", argc);
        diag_printf("Useage: fuse_read <bank> <row>\n");
    }
}

/* Blow fuses based on the bank, row and bit positions (all 0-based)
*/
static int fuse_blow(int bank,int row,int bit)
{
    int addr, addr_l, addr_h, ret = -1;

    fuse_op_start();

    /* Disable IIM Program Protect */
    writel(0xAA, IIM_BASE_ADDR + IIM_PREG_P_OFF);

    addr = ((bank << 11) | (row << 3) | (bit & 0x7));
    /* Set IIM Program Upper Address */
    addr_h = (addr >> 8) & 0x000000FF;
    /* Set IIM Program Lower Address */
    addr_l = (addr & 0x000000FF);

#ifdef IIM_FUSE_DEBUG
    diag_printf("blowing addr_h=0x%x, addr_l=0x%x\n", addr_h, addr_l);
#endif

    writel(addr_h, IIM_BASE_ADDR + IIM_UA_OFF);
    writel(addr_l, IIM_BASE_ADDR + IIM_LA_OFF);
    /* Start Programming */
    writel(0x31, IIM_BASE_ADDR + IIM_FCTL_OFF);
    if (poll_fuse_op_done(POLL_FUSE_PRGD) == 0) {
        ret = 0;
    }

    /* Enable IIM Program Protect */
    writel(0x0, IIM_BASE_ADDR + IIM_PREG_P_OFF);
    return ret;
}

/*
 * This command is added for burning IIM fuses
 */
RedBoot_cmd("fuse_read",
            "read some fuses",
            "<bank> <row>",
            do_fuse_read
           );

RedBoot_cmd("fuse_blow",
            "blow some fuses",
            "<bank> <row> <value>",
            do_fuse_blow
           );

#define         INIT_STRING              "12345678"
static char ready_to_blow[] = INIT_STRING;

void quick_itoa(u32 num, char *a)
{
    int i, j, k;
    for (i = 0; i <= 7; i++) {
        j = (num >> (4 * i)) & 0xF;
        k = (j < 10) ? '0' : ('a' - 0xa);
        a[i] = j + k;
    }
}

void do_fuse_blow(int argc, char *argv[])
{
    int bank, row, value, i;

    if (argc == 1) {
        diag_printf("It is too dangeous for you to use this command.\n");
        return;
    } else if (argc == 2) {
        if (strcasecmp(argv[1], "nandboot") == 0) {
            quick_itoa(readl(EPIT_BASE_ADDR + EPITCNR), ready_to_blow);
            diag_printf("%s\n", ready_to_blow);
        }
        return;
    } else if (argc == 3) {
        if (strcasecmp(argv[1], "nandboot") == 0 &&
            strcasecmp(argv[2], ready_to_blow) == 0) {
#if defined(CYGPKG_HAL_ARM_MXC91131) || defined(CYGPKG_HAL_ARM_MX21) || defined(CYGPKG_HAL_ARM_MX27) || defined(CYGPKG_HAL_ARM_MX31)
            diag_printf("No need to blow any fuses for NAND boot on this platform\n\n");
#else
            diag_printf("Ready to burn NAND boot fuses\n");
            if (fuse_blow(0, 16, 1) != 0 || fuse_blow(0, 16, 7) != 0) {
                diag_printf("NAND BOOT fuse blown failed miserably ...\n");
            } else {
                diag_printf("NAND BOOT fuse blown successfully ...\n");
            }
        } else {
            diag_printf("Not ready: %s, %s\n", argv[1], argv[2]);
#endif
        }
    } else if (argc == 4) {
        if (!parse_num(*(&argv[1]), (unsigned long *)&bank, &argv[1], " ")) {
                diag_printf("Error: Invalid parameter\n");
                return;
        }
        if (!parse_num(*(&argv[2]), (unsigned long *)&row, &argv[2], " ")) {
                diag_printf("Error: Invalid parameter\n");
                return;
        }
        if (!parse_num(*(&argv[3]), (unsigned long *)&value, &argv[3], " ")) {
                diag_printf("Error: Invalid parameter\n");
                return;
        }

        diag_printf("Blowing fuse at bank:%d row:%d value:%d\n",
                    bank, row, value);
        for (i = 0; i < 8; i++) {
            if (((value >> i) & 0x1) == 0) {
                continue;
            }
            if (fuse_blow(bank, row, i) != 0) {
                diag_printf("fuse_blow(bank: %d, row: %d, bit: %d failed\n",
                            bank, row, i);
            } else {
                diag_printf("fuse_blow(bank: %d, row: %d, bit: %d successful\n",
                            bank, row, i);
            }
        }
        sense_fuse(bank, row, 0);

    } else {
        diag_printf("Passing in wrong arguments: %d\n", argc);
    }
    /* Reset to default string */
    strcpy(ready_to_blow, INIT_STRING);;
}

/* precondition: m>0 and n>0.  Let g=gcd(m,n). */
int gcd(int m, int n)
{
    int t;
    while(m > 0) {
        if(n > m) {t = m; m = n; n = t;} /* swap */
        m -= n;
    }
    return n;
}

#define CLOCK_SRC_DETECT_MS         100
#define CLOCK_IPG_DEFAULT           66500000
#define CLOCK_SRC_DETECT_MARGIN     500000
void mxc_show_clk_input(void)
{
    u32 c1, c2, diff, ipg_real, num = 0;
    u32 prcs = (readl(CCM_BASE_ADDR + CLKCTL_CCMR) >> 1) & 0x3;

    return;  // FIXME

    switch (prcs) {
    case 0x01:
        diag_printf("FPM enabled --> 32KHz input source\n");
        return;
    case 0x02:
        break;
    default:
        diag_printf("Error %d: unknown clock source %d\n", __LINE__, prcs);
        return;
    }

    // enable GPT with IPG clock input
    writel(0x241, GPT_BASE_ADDR + GPTCR);
    // prescaler = 1
    writel(0, GPT_BASE_ADDR + GPTPR);

    c1 = readl(GPT_BASE_ADDR + GPTCNT);
    // use 32KHz input clock to get the delay
    hal_delay_us(CLOCK_SRC_DETECT_MS * 1000);
    c2 = readl(GPT_BASE_ADDR + GPTCNT);
    diff = (c2 > c1) ? (c2 - c1) : (0xFFFFFFFF - c1 + c2);

    ipg_real = diff * (1000 / CLOCK_SRC_DETECT_MS);

    if (ipg_real > (CLOCK_IPG_DEFAULT + CLOCK_SRC_DETECT_MARGIN)) {
        if (g_clock_src != FREQ_27MHZ)
            num = 27;
    } else if (ipg_real < (CLOCK_IPG_DEFAULT - CLOCK_SRC_DETECT_MARGIN)) {
        if (g_clock_src != FREQ_26MHZ)
            num = 26;
    }
    if (num != 0) {
        diag_printf("Error: Actural clock input is %d MHz\n", num);
        diag_printf("       ipg_real=%d CLOCK_IPG_DEFAULT - CLOCK_SRC_DETECT_MARGIN=%d\n\n",
                    ipg_real, CLOCK_IPG_DEFAULT - CLOCK_SRC_DETECT_MARGIN);
        diag_printf("       But clock source defined to be %d\n\n", g_clock_src);
        hal_delay_us(2000000);
    } else {
        diag_printf("ipg_real=%d CLOCK_IPG_DEFAULT - CLOCK_SRC_DETECT_MARGIN=%d\n\n",
                    ipg_real, CLOCK_IPG_DEFAULT - CLOCK_SRC_DETECT_MARGIN);
        diag_printf("clock source defined to be %d\n\n", g_clock_src);
    }
}

RedBoot_init(mxc_show_clk_input, RedBoot_INIT_LAST);
