//==========================================================================
//
//      mxc_spi.c
//
//      SPI support on Freescale MXC platforms
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
// Contributors:
// Date:         2006-08-24
// Purpose:
// Description:
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <redboot.h>
#include <stdlib.h>
#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_io.h>

#include <cyg/hal/fsl_board.h>
#include <cyg/io/mxc_spi.h>

void clock_spi_enable(unsigned int spi_clk);

#undef MXC_SPI_DEBUG
//#define MXC_SPI_DEBUG

#ifdef MXC_SPI_DEBUG
#define diag_printf1    diag_printf
#else
#define diag_printf1(fmt,args...)
#endif

#if defined(MXC_SPI_VER_0_7) || defined(MXC_SPI_VER_0_4)
const unsigned int baud_rate_div[] = {
    4, 8, 16, 32, 64, 128, 256, 512,
};
static int version = 1;
#elif defined(MXC_SPI_VER_XX)
const unsigned int baud_rate_div[] = {
    3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256, 384, 512,
};
static int version = 2;
#elif defined(MXC_SPI_VER_2_3)
const unsigned int baud_rate_div[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
};
static int version = 3;
#else
#error No SPI version defined
#endif

const int BAUD_RATE_DIV_MAX = sizeof(baud_rate_div) / sizeof(unsigned int);

/*!
 * It is for the master mode operation to exchange a single word with
 * external device.
 *
 * @param   data        data to be transferred
 * @param   base        base address of the spi module
 *
 * @return              the value received from the Rx register
 */
unsigned int spi_xchg_single(unsigned int data, unsigned int base)
{
    volatile unsigned int cfg_reg = readl(base + SPI_CTRL_REG_OFF);

    hal_delay_us(100);

    writel(data, base + SPI_TX_REG_OFF);

    cfg_reg |= SPI_CTRL_REG_XCH_BIT;

    writel(cfg_reg, base + SPI_CTRL_REG_OFF);

    while ((readl(base + SPI_INT_STAT_REG_OFF) & SPI_INT_STAT_RR) == 0) {
    }

    return readl(base + SPI_RX_REG_OFF);
}

/*!
 * Initialize and enable a spi module
 *
 * @param   base        base address of spi module (also assigned for SPIx_CLK)
 * @param   baue        the desired data rate
 * @param   ctrl_val    control register value EXCEPT the data rate
 *
 * @return              0 if successful; non-zero otherwise
 */
int spi_init(unsigned int base, unsigned int baud, unsigned int ctrl_val)
{
    unsigned int clock = get_peri_clock(base);
    int i, div = clock / baud;

    clock_spi_enable(base);

    diag_printf1("base=0x%x, baue=%d, ctrl_val=0x%x, div=%d, clock=%d\n",
                base, baud, ctrl_val, div, clock);

    for (i = 0; i < BAUD_RATE_DIV_MAX; i++) {
        if (div <= baud_rate_div[i]) {
            break;
        }
    }
    if (i == BAUD_RATE_DIV_MAX) {
        diag_printf("Baud rate requested (%d) is too slow for spi(0x%x)\n",
                    baud, base);
        return -1;
    }

    // to adjust for differen spi versions
    if (version == 2) {
        ctrl_val |= ((i + 1) << SPI_CTRL_REG_RATE_SH);
    } else
        ctrl_val |= (i << SPI_CTRL_REG_RATE_SH);


    diag_printf1("ctrl_val=0x%x, i=%d, SPI_CTRL_REG_RATE_SH=%d\n",
                ctrl_val, i, SPI_CTRL_REG_RATE_SH);

    writel(SPI_CTRL_EN, base + SPI_CTRL_REG_OFF);

    writel(ctrl_val, base + SPI_CTRL_REG_OFF);

    writel(0, base + SPI_INT_CTRL_REG_OFF);
    diag_printf1("requested data rate is: %d, actual rate is: %d\n",
                 baud, clock / baud_rate_div[i]);

    return 0;
}

#ifdef PMIC_SPI_BASE
static void mxc_pmic_init(void)
{
    volatile unsigned int rev_id;
    unsigned int ctrl;

    ctrl = SPI_CTRL_REG_BIT_COUNT32 | SPI_CTRL_SSPOL_HIGH |
           SPI_CTRL_MODE_MASTER | SPI_CTRL_EN;
    ctrl |= PMIC_SPI_CHIP_SELECT_NO;

    spi_init(PMIC_SPI_BASE, 4000000,      // 4MHz data rate
             ctrl);

    rev_id = pmic_reg(7, 0, 0);
    diag_printf("PMIC ID: 0x%08x [Rev: ", rev_id);
    switch (rev_id & 0x1F) {
    case 0x1:
        diag_printf("1.0");
        break;
    case 0x9:
        diag_printf("1.1");
        break;
    case 0xA:
        diag_printf("1.2");
        break;
    case 0x10:
        diag_printf("2.0");
        break;
    case 0x11:
        diag_printf("2.1");
        break;
    case 0x18:
        diag_printf("3.0");
        break;
    case 0x19:
        diag_printf("3.1");
        break;
    case 0x1A:
        diag_printf("3.2");
        break;
    case 0x2:
        diag_printf("3.2A");
        break;
    case 0x1B:
        diag_printf("3.3");
        break;
    case 0x1D:
        diag_printf("3.5");
        break;
    default:
        diag_printf("unknown");
        break;
    }
    diag_printf("]\n");
}

RedBoot_init(mxc_pmic_init, RedBoot_INIT_PRIO(100));

static void do_pmic(int argc, char *argv[]);
RedBoot_cmd("pmic",
            "Read/Write internal PMIC register",
            "<reg num> [value to be written]",
            do_pmic
           );

static void do_pmic(int argc,char *argv[])
{
    unsigned int reg, temp, val = 0, write = 0;

    if (argc == 1) {
        diag_printf("\tRead:  pmic <reg num>\n");
        diag_printf("\tWrite: pmic <reg num> <value to be written>\n");
        return;
    }

    if (!parse_num(*(&argv[1]), (unsigned long *)&reg, &argv[1], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }

    if (argc == 3) {
        if (!parse_num(*(&argv[2]), (unsigned long *)&val, &argv[2], ":")) {
            diag_printf("Error: Invalid parameter\n");
            return;
        }
        write = 1;
    }

    temp = pmic_reg(reg, val, write);

    diag_printf("\tval: 0x%08x\n\n", temp);
}

/*!
 * To read/write to a PMIC register. For write, it does another read for the
 * actual register value.
 *
 * @param   reg         register number inside the PMIC
 * @param   val         data to be written to the register; don't care for read
 * @param   write       0 for read; 1 for write
 *
 * @return              the actual data in the PMIC register
 */
unsigned int pmic_reg(unsigned int reg, unsigned int val, unsigned int write)
{
    unsigned int temp;

    if (reg > 63 || write > 1 ) {
        diag_printf("<reg num> = %d is invalide. Should be less then 63\n", reg);
        return 0;
    }
    val = (write << 31) | (reg << 25) | (val & 0x00FFFFFF);
    diag_printf1("reg=0x%x, val=0x%08x\n", reg, val);

    temp = spi_xchg_single(val, PMIC_SPI_BASE);

    if (write) {
        val &= ~(1 << 31);
        temp = spi_xchg_single(val, PMIC_SPI_BASE);
    }

    return temp;
}
#endif // PMIC_SPI_BASE

#ifdef CPLD_SPI_BASE

unsigned int spi_cpld_xchg_single(unsigned int data, unsigned int data1, unsigned int base)
{
    volatile unsigned int cfg_reg = readl(base + SPI_CTRL_REG_OFF);
    unsigned int temp;

    /* Activate the SS signal */
    cfg_reg |= CPLD_SPI_CHIP_SELECT_NO;
    writel(cfg_reg, CPLD_SPI_BASE + SPI_CTRL_REG_OFF);

    /* Write the data */
    writel(data, base + SPI_TX_REG_OFF);
    writel(data1, base + SPI_TX_REG_OFF);

    cfg_reg |= SPI_CTRL_REG_XCH_BIT;
    writel(cfg_reg, base + SPI_CTRL_REG_OFF);

    while ((((cfg_reg = readl(base + SPI_TEST_REG_OFF)) &
              SPI_TEST_REG_RXCNT_MASK) >> SPI_TEST_REG_RXCNT_OFFSET) != 2) {
    }

    /* Deactivate the SS signal */
    cfg_reg = readl(base + SPI_CTRL_REG_OFF);
    cfg_reg &= ~SPI_CTRL_CS_MASK;
    writel(cfg_reg, base + SPI_CTRL_REG_OFF);

    /* Read from RX FIFO, second entry contains the data */
    temp = readl(base + SPI_RX_REG_OFF);
    temp = readl(base + SPI_RX_REG_OFF);
    return ((temp >> 6) & 0xffff);
}

static void mxc_cpld_spi_init(void)
{
    unsigned int ctrl;

    ctrl = SPI_CTRL_REG_BIT_COUNT46 | CPLD_SPI_CTRL_MODE_MASTER | SPI_CTRL_EN;

    spi_init(CPLD_SPI_BASE, 18000000,      // 54MHz data rate
             ctrl);
}

RedBoot_init(mxc_cpld_spi_init, RedBoot_INIT_PRIO(102));

static void do_cpld(int argc, char *argv[]);

RedBoot_cmd("spi_cpld",
            "Read/Write 16-bit internal CPLD register over CSPI",
            "<reg num> [16-bit value to be written]",
            do_cpld
           );

static void do_cpld(int argc,char *argv[])
{
    unsigned int reg, temp, val = 0, read = 1;

    if (argc == 1) {
        diag_printf("\tRead:  spi_cpld <reg num>\n");
        diag_printf("\tWrite: spi_cpld <reg num> <value to be written>\n");
        return;
    }

    if (!parse_num(*(&argv[1]), (unsigned long *)&reg, &argv[1], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }

    if (argc == 3) {
        if (!parse_num(*(&argv[2]), (unsigned long *)&val, &argv[2], ":")) {
            diag_printf("Error: Invalid parameter\n");
            return;
        }
        read = 0;
    }

    temp = cpld_reg(reg, val, read);

    diag_printf("\tval: 0x%04x\n\n", temp);
}

/*!
 * To read/write to a CPLD register.
 *
 * @param   reg         register number inside the CPLD
 * @param   val         data to be written to the register; don't care for read
 * @param   read        0 for write; 1 for read
 *
 * @return              the actual data in the CPLD register
 */
unsigned int cpld_reg_xfer(unsigned int reg, unsigned int val, unsigned int read)
{
    unsigned int local_val1, local_val2;

    reg >>= 1;

    local_val1 = (read << 13) | ((reg & 0x0001FFFF) >> 5) | 0x00001000;
    if (read) {
        //local_val1 = (read << 22) | (reg << 4) | 0x00200004;
        //local_val2 = 0x1F;
        local_val2 = ( ((reg & 0x0000001F) << 27) | 0x0200001f);

    } else {
        //local_val1 = (read << 22) | (reg << 4) | 0x00200007;
        //local_val2 = ((val & 0xFFFF) << 6) | 0x00400027;
        local_val2 = ( ((reg & 0x0000001F) << 27) | ((val & 0x0000FFFF) << 6) | 0x03C00027);

    }

    diag_printf1("reg=0x%x, val=0x%08x\n", reg, val);
    return spi_cpld_xchg_single(local_val1, local_val2, CPLD_SPI_BASE);
}

/*!
 * To read/write to a CPLD register. For write, it does another read for the
 * actual register value.
 *
 * @param   reg         register number inside the CPLD
 * @param   val         data to be written to the register; don't care for read
 * @param   read        0 for write; 1 for read
 *
 * @return              the actual data in the CPLD register
 */
unsigned int cpld_reg(unsigned int reg, unsigned int val, unsigned int read)
{
    unsigned int temp;

    if (reg > 0x20068 || read > 1 ) {
        diag_printf("<reg num> = %x is invalid. Should be less then 0x20068\n", reg);
        return 0;
    }

    temp = cpld_reg_xfer(reg, val, read);
    diag_printf1("reg=0x%x, val=0x%08x\n", reg, val);

    if (read == 0) {
        temp = cpld_reg_xfer(reg, val, 1);
    }

    return temp;
}

#endif // CPLD_SPI_BASE
