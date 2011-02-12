//==========================================================================
//
//      mxc_i2c.c
//
//      I2C support on Freescale MXC platforms
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
#include <stdlib.h>
#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <cyg/hal/hal_io.h>

#include <cyg/hal/fsl_board.h>
#include <cyg/io/mxc_i2c.h>

extern void mxc_i2c_init(unsigned int module_base);

//#define MXC_I2C_DEBUG
#undef MXC_I2C_DEBUG

#ifdef MXC_I2C_DEBUG
#define diag_printf1    diag_printf
#else
#define diag_printf1(fmt,args...)
#endif

struct clk_div_table {
    int reg_value;
    int div;
};

static const struct clk_div_table i2c_clk_table[] = {
    {0x20, 22}, {0x21, 24}, {0x22, 26}, {0x23, 28},
    {0, 30}, {1, 32}, {0x24, 32}, {2, 36},
    {0x25, 36}, {0x26, 40}, {3, 42}, {0x27, 44},
    {4, 48}, {0x28, 48}, {5, 52}, {0x29, 56},
    {6, 60}, {0x2A, 64}, {7, 72}, {0x2B, 72},
    {8, 80}, {0x2C, 80}, {9, 88}, {0x2D, 96},
    {0xA, 104}, {0x2E, 112}, {0xB, 128}, {0x2F, 128},
    {0xC, 144}, {0xD, 160}, {0x30, 160}, {0xE, 192},
    {0x31, 192}, {0x32, 224}, {0xF, 240}, {0x33, 256},
    {0x10, 288}, {0x11, 320}, {0x34, 320}, {0x12, 384},
    {0x35, 384}, {0x36, 448}, {0x13, 480}, {0x37, 512},
    {0x14, 576}, {0x15, 640}, {0x38, 640}, {0x16, 768},
    {0x39, 768}, {0x3A, 896}, {0x17, 960}, {0x3B, 1024},
    {0x18, 1152}, {0x19, 1280}, {0x3C, 1280}, {0x1A, 1536},
    {0x3D, 1536}, {0x3E, 1792}, {0x1B, 1920}, {0x3F, 2048},
    {0x1C, 2304}, {0x1D, 2560}, {0x1E, 3072}, {0x1F, 3840},
    {0, 0}
};

#define ERR_TX          -1
#define ERR_RX          -2
#define ERR_ARB_LOST    -3
#define ERR_NO_ACK      -4
#define ERR_XFER        -5
#define ERR_RX_ACK      -6

static inline int wait_till_busy(unsigned int base)
{
    int i = 10000;

    while(((readw(base + I2C_I2SR) & I2C_I2SR_IBB) == 0) && (--i > 0)) {
        if (readw(base + I2C_I2SR) & I2C_I2SR_IAL) {
            diag_printf1("Error: arbitration lost!\n");
            return ERR_ARB_LOST;
        }
    }

    if (i <= 0) {
        return -1;
    }

    return 0;
}

static unsigned int g_dev_addr_width, g_dev_data_width;
static unsigned char g_dev_value[4];
static unsigned int g_i2c_nr = -1;

static inline int is_bus_free(unsigned int base)
{
    return ((readw(base + I2C_I2SR) & I2C_I2SR_IBB) == 0);
}

#define ASSERT_NO_ARBITRATION_LOST(stat)  \
{ \
    if (stat & I2C_I2SR_IAL) { \
        diag_printf("Error %d: Arbitration lost\n", __LINE__); \
        return ERR_ARB_LOST; \
    } \
}

#define WAIT_RXAK_LOOPS     1000000

static inline unsigned short wait_op_done(unsigned int base, int is_tx)
{
    volatile unsigned short v;
    int i = WAIT_RXAK_LOOPS;

    while ((((v = readw(base + I2C_I2SR)) & I2C_I2SR_IIF) == 0 ||
           (v & I2C_I2SR_ICF) == 0) && --i > 0) {
        if (v & I2C_I2SR_IAL) {
            diag_printf1("Error %d: Arbitration lost\n", __LINE__);
            return ERR_ARB_LOST;
        }

    }
    if (i <= 0) {
        diag_printf1("I2C Error: timeout unexpected\n");
        return -1;
    }
    if (is_tx) {
        if (v & I2C_I2SR_IAL) {
            diag_printf1("Error %d: Arbitration lost\n", __LINE__);
            return ERR_ARB_LOST;
        }
    if (v & I2C_I2SR_RXAK) {
            diag_printf1("Error %d: no ack received\n", __LINE__);
            return -1;
    }
    }
    return 0;
}

//
// For master TX, always expect a RXAK signal to be set!
static int tx_byte(unsigned char *data, unsigned int base)
{
    diag_printf1("%s(data=0x%02x, base=0x%x)\n", __FUNCTION__, *data, base);

    // clear both IAL and IIF bits
    writew(0, base + I2C_I2SR);

    writew(*data, base + I2C_I2DR);
    
    if (wait_op_done(base, 1) != 0)
        return -1;

    return 0;
}

// For master RX
static int rx_bytes(unsigned char *data, unsigned int base, int sz)
{
    unsigned short i2cr;
    int i;

    for (i = 0; sz > 0; sz--, i++) {
        if (wait_op_done(base, 0) != 0)
            return -1;

        // clear both IAL and IIF bits
        writew(0, base + I2C_I2SR);

        // the next two if-statements setup for the next read control register value
        if (sz == 1) {
            // last byte --> generate STOP
            i2cr = readw(base + I2C_I2CR);
            writew(i2cr & ~(I2C_I2CR_MSTA | I2C_I2CR_MTX), base + I2C_I2CR);
        }
        if (sz == 2) {
            // 2nd last byte --> set TXAK bit to NOT generate ACK
            i2cr = readw(base + I2C_I2CR);
            writew(i2cr | I2C_I2CR_TXAK, base + I2C_I2CR);
        }

        // read the true data
        data[i] = readw(base + I2C_I2DR);
        diag_printf1("OK 0x%02x\n", data[i]);
    }
    return 0;
}

int i2c_xfer(unsigned int i2c_nr, struct mxc_i2c_request *rq, int dir)
{
    unsigned int base, reg;
    unsigned char i, data;
    unsigned short i2cr;
    int ret = 0;
    
    if ( rq == NULL || i2c_nr >= i2c_num) {
    diag_printf("Invalid request or invalid i2c port number\n");
    return -1;
    } 

    base = i2c_base_addr[i2c_nr];
    if (rq->reg_addr_sz == 0 || rq->buffer_sz == 0 || rq->buffer == NULL) {
        diag_printf("Invalid register address size=%x, buffer size=%x, buffer=%x\n",
                    rq->reg_addr_sz, rq->buffer_sz, (unsigned int)rq->buffer);
        return -1;
    }

    // reset and enable I2C
    writew(0, base + I2C_I2CR);

    writew(I2C_I2CR_IEN, base + I2C_I2CR);

    /* Need wait at least 2 cycles of per_clk*/
    for (i = 0; i < 16; i++) {
        asm("nop");
    }
    // Step 1: generate START signal
    // 1.1 make sure bus is free
    if (!is_bus_free(base)) {
        return -1;
    }
    // 1.2 clear both IAL and IIF bits
    writew(0, base + I2C_I2SR);

    // 1.3 assert START signal and also indicate TX mode
    i2cr = I2C_I2CR_IEN | I2C_I2CR_MSTA | I2C_I2CR_MTX;
    writew(i2cr, base + I2C_I2CR);

    // 1.4 make sure bus is busy after the START signal
    if (wait_till_busy(base) != 0) {
        return ERR_TX;
    }

    // Step 2: send slave address + read/write at the LSB
    data = (rq->dev_addr << 1) | I2C_WRITE;
    if (tx_byte(&data, base) != 0) {
        return -1;
    }

    // Step 3: send I2C device register address
    if (rq->reg_addr_sz > 4) {
        diag_printf("Warning register address size %d should less than 4\n",
                            rq->reg_addr_sz);
        rq->reg_addr_sz = 4;
    }
    reg = rq->reg_addr;

    for (i = 0; i <  rq->reg_addr_sz; i++, reg>>=8) {
        data = reg & 0xFF;
        diag_printf1("sending I2C=0x%x device register: data=0x%x, byte %d\n",
                     base, data, i);
        if (tx_byte(&data, base) != 0) {
            return -1;
        }
    }
    // Step 4: read/write data
    if (dir == I2C_READ) {
        // do repeat-start
        i2cr = readw(base + I2C_I2CR);
        writew(i2cr | I2C_I2CR_RSTA, base + I2C_I2CR);

        // send slave address again, but indicate read operation
        data = (rq->dev_addr << 1) | I2C_READ;
        if (tx_byte(&data, base) != 0) {
            return -1;
        }

        // change to receive mode
        i2cr = readw(base + I2C_I2CR);
        if (rq->buffer_sz == 1) {
            // if only one byte to read, make sure don't send ack
            i2cr |= I2C_I2CR_TXAK;
        }
        writew(i2cr & ~I2C_I2CR_MTX, base + I2C_I2CR);
        // dummy read
        readw(base + I2C_I2DR);

        // now reading ...
        if (rx_bytes(rq->buffer, base, rq->buffer_sz) != 0) {
            return -1;
        }
    } else {
        // I2C_WRITE
        for (i = 0; i < rq->buffer_sz; i++) {
            // send device register value
            data = rq->buffer[i];
            if ((ret = tx_byte(&data, base)) != 0) {
                break;
            }
        }
        // generate STOP by clearing MSTA bit
        writew(I2C_I2CR_IEN | I2C_I2CR_MTX, base + I2C_I2CR);
    }

    return ret;
}

/*!
 * Initialize and enable a i2c module -- mainly enable the I2C clock, module
 * itself and the I2C clock prescaler.
 *
 * @param   base        base address of i2c module (also assigned for I2Cx_CLK)
 * @param   baue        the desired data rate
 *
 * @return              0 if successful; non-zero otherwise
 */
int i2c_init(unsigned int base, unsigned int baud)
{
    unsigned int clock = get_main_clock(IPG_PER_CLK);
    int div = clock / baud;
    struct clk_div_table *p = (struct clk_div_table *)&i2c_clk_table[0];

    mxc_i2c_init(base);

    // reset and enable I2C
    writew(0, base + I2C_I2CR);
    writew(I2C_I2CR_IEN, base + I2C_I2CR);

    while (p->div != 0) {
        if (div <= p->div)
            break;
        p++;
    }
    
    if (p->div == 0) {
        diag_printf("Error: can't meet I2C baud rate request (%d) for 0x%x)\n",
                    baud, base);
        return -1;
    }

    diag_printf1("baud=%d, div=%d, reg_val=%d\n", baud, p->div, p->reg_value);

    writew(p->reg_value, base + I2C_IFDR);

    diag_printf1("requested data rate is: %d, actual rate is: %d\n",
                 baud, clock / p->div);

    return 0;
}

static void do_i2c(int argc, char *argv[]);
RedBoot_cmd("i2c",
            "i2c R/W operations as master",
            "<i2c slave addr> <register index> [<regisetr val>]]",
            do_i2c
           );


static void do_i2c(int argc,char *argv[])
{
    int dir = I2C_READ, i;
    unsigned long v;
    unsigned int dev_addr, dev_reg;
    struct mxc_i2c_request rq;
 
    if (g_i2c_nr == -1) {
        diag_printf("I2C module [%d] not initialized. Issue i2c_init first\n\n", g_i2c_nr);
        return;
    }
    if (argc == 1) {
        diag_printf("\tRead:  i2c <i2c_dev_addr> <dev_reg_addr>\n");
        diag_printf("\tWrite: i2c <i2c_dev_addr> <dev_reg_addr> <dev_reg_val>\n");
        return;
    }

    if (!parse_num(*(&argv[1]), (unsigned long *)&dev_addr, &argv[1], ":")) {
        diag_printf("Error: Invalid parameter %d\n", __LINE__);
        return;
    }

    if (!parse_num(*(&argv[2]), (unsigned long *)&dev_reg, &argv[2], ":")) {
        diag_printf("Error: Invalid parameter %d\n", __LINE__);
        return;
    }

    if (argc == 4) {
        if (!parse_num(*(&argv[3]), &v, &argv[3], ":")) {
            diag_printf("Error: Invalid parameter\n");
            return;
        }
        dir = I2C_WRITE;
        diag_printf("Writing I2C[%d] for addr 0x%x register 0x%x with value 0x%08lx\n",
                    g_i2c_nr, dev_addr, dev_reg, v);
        for (i = 0; i < g_dev_data_width; i++) {
            g_dev_value[i] = v >> (8 * (g_dev_data_width - i - 1)) & 0xff;
        }
        diag_printf1("testing reversed data: 0x%08x\n", *(unsigned int*)g_dev_value);

    } else {
        diag_printf("Reading I2C [%d] from slave addr [0x%x] register [0x%x]\n",
                    g_i2c_nr, dev_addr,  dev_reg);
    }

    rq.dev_addr = dev_addr;
    rq.reg_addr = dev_reg;
    rq.reg_addr_sz = g_dev_addr_width;
    rq.buffer = g_dev_value;
    rq.buffer_sz = g_dev_data_width;

    if (i2c_xfer(g_i2c_nr, &rq, dir) != 0) {
        diag_printf("Error I2C transfer!\n\n");
        return;
    }

    if (dir == I2C_READ) {
        diag_printf("--->  ");
        for (i = 0; i < g_dev_data_width; i++) {
            diag_printf("0x%02x ", g_dev_value[i]);
        }
        diag_printf("\n\n");
    }
}

static void do_i2c_init(int argc, char *argv[]);
RedBoot_cmd("i2c_init",
            "Initialize i2c (i2c_num is 0-indexed)",
            "<i2c_num> <frequency> <device addr width> <device reg width>",
            do_i2c_init
           );

static void do_i2c_init(int argc,char *argv[])
{
    unsigned freq;

    if (argc == 1 || argc != 5) {
        diag_printf("\ni2c_init <i2c_num> <frequency> <device addr width> <device data width>\n\n");
        return;
    }

    if (!parse_num(*(&argv[1]), (unsigned long *)&g_i2c_nr, &argv[1], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }
    
    if (g_i2c_nr > i2c_num - 1) {
        diag_printf("invalide i2c number: %d, max number is: %d\n", g_i2c_nr, i2c_num - 1);
        return;
    }
    diag_printf1("i2c max number is: %d\n", i2c_num - 1);

    if (!parse_num(*(&argv[2]), (unsigned long *)&freq, &argv[2], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }
    if (!parse_num(*(&argv[3]), (unsigned long *)&g_dev_addr_width, &argv[3], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }
    if (!parse_num(*(&argv[4]), (unsigned long *)&g_dev_data_width, &argv[4], ":")) {
        diag_printf("Error: Invalid parameter\n");
        return;
    }

    i2c_init(i2c_base_addr[g_i2c_nr], freq);
    
    diag_printf("initializing i2c:%d, addr-width:%d, data-width:%d\n\n",
                g_i2c_nr, g_dev_addr_width, g_dev_data_width);
}

int enable_3971_ldo5(int enable)
{
    struct mxc_i2c_request rq;
    g_i2c_nr = 0;
    g_dev_addr_width = 1;
    g_dev_data_width = 1;
    i2c_init(i2c_base_addr[g_i2c_nr], 100000);

    if (enable)
    {
        g_dev_value[0] = 0x3e;
    }
    else
    {
        g_dev_value[0] = 0x1e;
    }
    rq.dev_addr = 0x34;
    rq.reg_addr = 0x12;
    rq.reg_addr_sz = 1;
    rq.buffer = g_dev_value;
    rq.buffer_sz = 1;
    if (i2c_xfer(g_i2c_nr, &rq, I2C_WRITE) != 0)
    {
        diag_printf("Could not enable pmic ldo5.\n");
        return -1;
    }
    diag_printf("Enable LDO5 done\n");

    if (enable <= 0)
    {
        return 0;
    }

    rq.dev_addr = 0x34;
    rq.reg_addr = 0x3b;
    rq.reg_addr_sz = 1;
    g_dev_value[0] = 0xe;
    rq.buffer = g_dev_value;
    rq.buffer_sz = 1;
    if (i2c_xfer(g_i2c_nr, &rq, I2C_WRITE) != 0)
    {
        diag_printf("Could not change voltage to 3V.\n");
        return -1;
    }
    diag_printf("Change voltage to 3V done\n");

    rq.dev_addr = 0x34;
    rq.reg_addr = 0x23;
    rq.reg_addr_sz = 1;
    g_dev_value[0] = 0xd;
    rq.buffer = g_dev_value;
    rq.buffer_sz = 1;
    if (i2c_xfer(g_i2c_nr, &rq, I2C_WRITE) != 0)
    {
        diag_printf("Could not change core voltage to 1.4V.\n");
        return -1;
    }
    diag_printf("Change core voltage to 1.4V done\n");

    return 0;

}

int get_voltage(unsigned int dev_addr, unsigned int reg_addr)
{
    struct mxc_i2c_request rq;
    g_i2c_nr = 0;
    g_dev_addr_width = 1;
    g_dev_data_width = 2;
    i2c_init(i2c_base_addr[g_i2c_nr], 100000);

    rq.dev_addr = dev_addr;
    rq.reg_addr = reg_addr;
    rq.reg_addr_sz = g_dev_addr_width;
    rq.buffer = g_dev_value;
    rq.buffer_sz = g_dev_data_width;
    if (i2c_xfer(g_i2c_nr, &rq, I2C_READ) != 0)
    {
        diag_printf("I2C: read from [0x%02x:0x%02x] failed!\n", dev_addr, reg_addr);
        return -1;
    }

    switch (dev_addr)
    {
        case 0x55:
            return (g_dev_value[1] << 8) | g_dev_value[0];
        case 0x54:
            return ((g_dev_value[0] << 4) | (g_dev_value[1] >> 4)) * 4220 / 247;
        default:
            return -1;
    }
}
