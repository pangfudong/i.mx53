//==========================================================================
//
//      mxc_spi.h
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

#ifndef __MXC_SPI_H__
#define __MXC_SPI_H__

#define SPI_RX_REG_OFF              0x0
#define SPI_TX_REG_OFF              0x4
#define SPI_CTRL_REG_OFF            0x8

#if defined(MXC_SPI_VER_0_4)
#define SPI_INT_CTRL_REG_OFF        0xC
#define SPI_DMA_REG_OFF             0x10
#define SPI_INT_STAT_REG_OFF        0x14
#define SPI_PERIOD_REG_OFF          0x18
#define SPI_TEST_REG_OFF            0x1C

#define SPI_INT_STAT_RR             (1 << 3)

#define SPI_CTRL_REG_XCH_BIT        (1 << 2)
#define SPI_CTRL_REG_RATE_SH        16  // start from bit 16
#define SPI_CTRL_REG_RATE_WD        3   // 3-bit width

#define SPI_CTRL_REG_BIT_COUNT32    (0x1F << 8)  // 32-bit xfer
#define SPI_CTRL_REG_BIT_COUNT23    (0x16 << 8)  // 23-bit xfer
#define SPI_CTRL_CS0                (0 << 24)
#define SPI_CTRL_CS1                (1 << 24)
#define SPI_CTRL_CS2                (2 << 24)
#define SPI_CTRL_CS3                (3 << 24)
#define SPI_CTRL_CS_MASK            (3 << 24)
#define SPI_CTRL_SSPOL_HIGH         (1 << 7)
#define SPI_CTRL_SSCTL_SET          (1 << 6)
#define SPI_CTRL_SCLK_POL_LOW       (1 << 4)
#define SPI_CTRL_MODE_MASTER        (1 << 1)
#define SPI_CTRL_EN                 (1 << 0)
#define SPI_TEST_REG_RXCNT_OFFSET   4
#define SPI_TEST_REG_RXCNT_MASK     (0xF << 4)

#elif defined(MXC_SPI_VER_0_7)
#define SPI_INT_CTRL_REG_OFF        0xC
#define SPI_DMA_REG_OFF             0x10
#define SPI_INT_STAT_REG_OFF        0x14
#define SPI_PERIOD_REG_OFF          0x18
#define SPI_TEST_REG_OFF            0x1C

#define SPI_INT_STAT_RR             (1 << 3)

#define SPI_CTRL_REG_XCH_BIT        (1 << 2)
#define SPI_CTRL_REG_RATE_SH        16  // start from bit 16
#define SPI_CTRL_REG_RATE_WD        3   // 3-bit width

#define SPI_CTRL_REG_BIT_COUNT46    (0x2D << 20)  // 48-bit xfer
#define SPI_CTRL_REG_BIT_COUNT32    (0x1F << 20)  // 32-bit xfer
#define SPI_CTRL_REG_BIT_COUNT23    (0x16 << 20)  // 23-bit xfer
#define SPI_CTRL_CS0                (0 << 12)
#define SPI_CTRL_CS1                (1 << 12)
#define SPI_CTRL_CS2                (2 << 12)
#define SPI_CTRL_CS3                (3 << 12)
#define SPI_CTRL_CS_MASK            (3 << 12)
#define SPI_CTRL_SSPOL_HIGH         (1 << 7)
#define SPI_CTRL_SSCTL_SET          (1 << 6)
#define SPI_CTRL_SCLK_POL_LOW       (1 << 4)
#define SPI_CTRL_MODE_MASTER        (1 << 1)
#define SPI_CTRL_EN                 (1 << 0)
#define SPI_TEST_REG_RXCNT_OFFSET   4
#define SPI_TEST_REG_RXCNT_MASK     (0xF << 4)


#elif defined(MXC_SPI_VER_2_3)
#define SPI_INT_CTRL_REG_OFF        0x10
#define SPI_DMA_REG_OFF             0x14
#define SPI_INT_STAT_REG_OFF        0x18
#define SPI_PERIOD_REG_OFF          0x1C
#define SPI_TEST_REG_OFF            0x20

#define SPI_INT_STAT_RR             (1 << 3)

#define SPI_CTRL_REG_XCH_BIT        (1 << 2)
#define SPI_CTRL_REG_RATE_SH        12  // start from bit 12
#define SPI_CTRL_REG_RATE_WD        4   // 3-bit width

#define SPI_CTRL_REG_BIT_COUNT32    (0x1F << 20)  // 32-bit xfer
#define SPI_CTRL_REG_BIT_COUNT23    (0x16 << 20)  // 23-bit xfer
#define SPI_CTRL_REG_BIT_COUNT46    (0x2D << 20)  // 46-bit xfer
#define SPI_CTRL_CS0                (0 << 18)
#define SPI_CTRL_CS1                (1 << 18)
#define SPI_CTRL_CS2                (2 << 18)
#define SPI_CTRL_CS3                (3 << 18)
#define SPI_CTRL_CS_MASK            (3 << 18)
#define SPI_CTRL_MODE_MASTER_0      (1 << 4)
#define SPI_CTRL_MODE_MASTER_1      (1 << 5)
#define SPI_CTRL_MODE_MASTER_2      (1 << 6)
#define SPI_CTRL_MODE_MASTER_3      (1 << 7)
#define SPI_CTRL_EN                 (1 << 0)
#define SPI_TEST_REG_RXCNT_OFFSET   8
#define SPI_TEST_REG_RXCNT_MASK     (0x7F << 8)
#else
// For MX27
#define SPI_INT_CTRL_REG_OFF        0xC
#define SPI_INT_STAT_REG_OFF        0xC
#define SPI_TEST_REG_OFF            0x10
#define SPI_PERIOD_REG_OFF          0x14
#define SPI_DMA_REG_OFF             0x18
#define SPI_RESET_REG_OFF           0x1C

#define SPI_INT_STAT_RR             (1 << 4)

#define SPI_CTRL_REG_XCH_BIT        (1 << 9)
#define SPI_CTRL_REG_RATE_SH        14  // start from bit 14
#define SPI_CTRL_REG_RATE_WD        5   // 5-bit width

#define SPI_CTRL_BURST_EN           (1 << 23)
#define SPI_CTRL_SDHC_SPI_EN        (1 << 22)
#define SPI_CTRL_SWAP_EN            (1 << 21)
#define SPI_CTRL_CS0                (0 << 19)
#define SPI_CTRL_CS1                (1 << 19)
#define SPI_CTRL_CS2                (2 << 19)
#define SPI_CTRL_CS_MASK            (3 << 19)
#define SPI_CTRL_SSPOL_HIGH         (1 << 8)
#define SPI_CTRL_SSCTL_SET          (1 << 7)
#define SPI_CTRL_SCLK_POL_LOW       (1 << 5)
#define SPI_CTRL_REG_BIT_COUNT32    0x1F   // 32-bit xfer
#define SPI_CTRL_REG_BIT_COUNT23    0x16   // 23-bit xfer
#define SPI_CTRL_MODE_MASTER        (1 << 11)
#define SPI_CTRL_EN                 (1 << 10)
#define SPI_TEST_REG_RXCNT_OFFSET   4
#define SPI_TEST_REG_RXCNT_MASK     (0xF << 4)

#endif

int spi_init(unsigned int base, unsigned int baud, unsigned int ctrl_val);
unsigned int pmic_reg(unsigned int reg, unsigned int val, unsigned int write);
unsigned int spi_xchg_single(unsigned int data, unsigned int base);

#ifdef CPLD_SPI_BASE
unsigned int spi_cpld_xchg_single(unsigned int data, unsigned int data1, unsigned int base);
unsigned int cpld_reg(unsigned int reg, unsigned int val, unsigned int read);
unsigned int cpld_reg_xfer(unsigned int reg, unsigned int val, unsigned int read);
#endif   /* CPLD_SPI_BASE */

#endif				/* __MXC_SPI_H__ */
