//==========================================================================
//
//      soc_misc.c
//
//      HAL misc board support code
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
//========================================================================*/

#include <redboot.h>
#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_misc.h>           // Size constants
#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_arch.h>           // Register state info
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/hal_intr.h>           // Interrupt names
#include <cyg/hal/hal_cache.h>          // Cache control
#include <cyg/hal/hal_soc.h>            // Hardware definitions
#include <cyg/hal/hal_mm.h>             // MMap table definitions

#include <cyg/infra/diag.h>             // diag_printf

// Most initialization has already been done before we get here.
// All we do here is set up the interrupt environment.
// FIXME: some of the stuff in hal_platform_setup could be moved here.

externC void plf_hardware_init(void);

#define IIM_PROD_REV_SH         3
#define IIM_PROD_REV_LEN        5
#define IIM_SREV_REV_SH         4
#define IIM_SREV_REV_LEN        4
#define PROD_SIGNATURE_MX37     0x1

#define PROD_SIGNATURE_SUPPORTED  PROD_SIGNATURE_MX37

#define CHIP_VERSION_NONE           0xFFFFFFFF      // invalid product ID
#define CHIP_VERSION_UNKNOWN        0xDEADBEEF      // invalid chip rev

#define PART_NUMBER_OFFSET          (12)
#define MAJOR_NUMBER_OFFSET         (4)
#define MINOR_NUMBER_OFFSET         (0)

/*
 * System_rev will have the following format
 * 31-12 = part # (0x31, 0x32, 0x27, 0x91131, 0x91321, etc)
 * 11-8 = unused
 * 7-4 = major (1.y)
 * 3-0 = minor (x.0)
 */
unsigned int system_rev = CHIP_REV_1_0;
static int find_correct_chip;
extern char HAL_PLATFORM_EXTRA[20];

/*
 * This functions reads the IIM module and returns the system revision number.
 * It returns the IIM silicon revision reg value if valid product rev is found.
 . Otherwise, it returns -1.
 */
static int read_system_rev(void)
{
    int val;

    val = readl(IIM_BASE_ADDR + IIM_PREV_OFF);

    system_rev = 0x37 << PART_NUMBER_OFFSET; /* For MX37 Platform*/

#if 0
    /* If the IIM doesn't contain valid product signature, return
     * the lowest revision number */
    if (MXC_GET_FIELD(val, IIM_PROD_REV_LEN, IIM_PROD_REV_SH) !=
        PROD_SIGNATURE_SUPPORTED) {
        return CHIP_VERSION_NONE;
    }
#endif

    /* Now trying to retrieve the silicon rev from IIM's SREV register */
    return readl(IIM_BASE_ADDR + IIM_SREV_OFF);
}

extern nfc_setup_func_t *nfc_setup;
unsigned int mxc_nfc_soc_setup(unsigned int pg_sz, unsigned int io_sz,
                                      unsigned int is_mlc);
void hal_hardware_init(void)
{

    volatile unsigned int esdmisc = readl(ESDCTL_BASE + 0x10);
    volatile unsigned int esdctl0 = readl(ESDCTL_BASE);
    int ver = read_system_rev();

    find_correct_chip = ver;

    if (ver != CHIP_VERSION_NONE) {
        /* Valid product revision found. Check actual silicon rev and
         * NOT use the version from the ROM code. */
        if (((ver >> 4) & 0xF) == 0x0) {
            HAL_PLATFORM_EXTRA[5] = '1';
            HAL_PLATFORM_EXTRA[7] = '0';
            system_rev |= 1 << MAJOR_NUMBER_OFFSET; /*Major Number*/
            system_rev |= 0 << MINOR_NUMBER_OFFSET; /*Minor Number*/
        } else {
            HAL_PLATFORM_EXTRA[5] = 'x';
            HAL_PLATFORM_EXTRA[7] = 'x';
            system_rev |= 1 << MAJOR_NUMBER_OFFSET; /*Major Number*/
            system_rev |= 0 << MINOR_NUMBER_OFFSET; /*Minor Number*/
            find_correct_chip = CHIP_VERSION_UNKNOWN;
        }
    }
#if 0
    if ((esdmisc & 0x4) == 0) {
        HAL_PLATFORM_EXTRA[14] = 'S';
    }
    if ((esdctl0 & 0x30000) != 0x20000) {
        HAL_PLATFORM_EXTRA[11] = '1';
        HAL_PLATFORM_EXTRA[12] = '6';
    }

    // Mask all interrupts
    writel(0xFFFFFFFF, AVIC_NIMASK);

    // Make all interrupts do IRQ and not FIQ
    // FIXME: Change this if you use FIQs.
    writel(0, AVIC_INTTYPEH);
    writel(0, AVIC_INTTYPEL);
#endif
    // Enable caches
    HAL_ICACHE_ENABLE();
    HAL_DCACHE_ENABLE();

    // enable EPIT and start it with 32KHz input clock
    writel(0x00010000, EPIT_BASE_ADDR + EPITCR);

    // make sure reset is complete
    while ((readl(EPIT_BASE_ADDR + EPITCR) & 0x10000) != 0) {
    }

    writel(0x030E0002, EPIT_BASE_ADDR + EPITCR);
    writel(0x030E0003, EPIT_BASE_ADDR + EPITCR);

    writel(0, EPIT_BASE_ADDR + EPITCMPR);  // always compare with 0

    if ((readw(WDOG_BASE_ADDR) & 4) != 0) {
        // increase the WDOG timeout value to the max
        writew(readw(WDOG_BASE_ADDR) | 0xFF00, WDOG_BASE_ADDR);
    }

    // Perform any platform specific initializations
    plf_hardware_init();

    // Set up eCos/ROM interfaces
    hal_if_init();

    nfc_setup = (nfc_setup_func_t*)mxc_nfc_soc_setup;
}

// -------------------------------------------------------------------------
void hal_clock_initialize(cyg_uint32 period)
{
}

// This routine is called during a clock interrupt.

// Define this if you want to ensure that the clock is perfect (i.e. does
// not drift).  One reason to leave it turned off is that it costs some
// us per system clock interrupt for this maintenance.
#undef COMPENSATE_FOR_CLOCK_DRIFT

void hal_clock_reset(cyg_uint32 vector, cyg_uint32 period)
{
}

// Read the current value of the clock, returning the number of hardware
// "ticks" that have occurred (i.e. how far away the current value is from
// the start)

// Note: The "contract" for this function is that the value is the number
// of hardware clocks that have happened since the last interrupt (i.e.
// when it was reset).  This value is used to measure interrupt latencies.
// However, since the hardware counter runs freely, this routine computes
// the difference between the current clock period and the number of hardware
// ticks left before the next timer interrupt.
void hal_clock_read(cyg_uint32 *pvalue)
{
}

// This is to cope with the test read used by tm_basic with
// CYGVAR_KERNEL_COUNTERS_CLOCK_LATENCY defined; we read the count ASAP
// in the ISR, *before* resetting the clock.  Which returns 1tick +
// latency if we just use plain hal_clock_read().
void hal_clock_latency(cyg_uint32 *pvalue)
{
}

unsigned int hal_timer_count(void)
{
    return (0xFFFFFFFF - readl(EPIT_BASE_ADDR + EPITCNR));
}

#define WDT_MAGIC_1             0x5555
#define WDT_MAGIC_2             0xAAAA
#define MXC_WDT_WSR             0x2

unsigned int i2c_base_addr[] = {
    I2C_BASE_ADDR,
    I2C2_BASE_ADDR,
    I2C3_BASE_ADDR
};
unsigned int i2c_num = 3;

//
// Delay for some number of micro-seconds
//
void hal_delay_us(unsigned int usecs)
{
    /*
     * This causes overflow.
     * unsigned int delayCount = (usecs * 32768) / 1000000;
     * So use the following one instead
     */
    unsigned int delayCount = (usecs * 512) / 15625;

    if (delayCount == 0) {
        return;
    }

    // issue the service sequence instructions
    if ((readw(WDOG_BASE_ADDR) & 4) != 0) {
        writew(WDT_MAGIC_1, WDOG_BASE_ADDR + MXC_WDT_WSR);
        writew(WDT_MAGIC_2, WDOG_BASE_ADDR + MXC_WDT_WSR);
    }

    writel(0x01, EPIT_BASE_ADDR + EPITSR); // clear the compare status bit

    writel(delayCount, EPIT_BASE_ADDR + EPITLR);

    while ((0x1 & readl(EPIT_BASE_ADDR + EPITSR)) == 0); // return until compare bit is set
}

// -------------------------------------------------------------------------

// This routine is called to respond to a hardware interrupt (IRQ).  It
// should interrogate the hardware and return the IRQ vector number.
int hal_IRQ_handler(void)
{
#ifdef HAL_EXTENDED_IRQ_HANDLER
    cyg_uint32 index;

    // Use platform specific IRQ handler, if defined
    // Note: this macro should do a 'return' with the appropriate
    // interrupt number if such an extended interrupt exists.  The
    // assumption is that the line after the macro starts 'normal' processing.
    HAL_EXTENDED_IRQ_HANDLER(index);
#endif

    return CYGNUM_HAL_INTERRUPT_NONE; // This shouldn't happen!
}

//
// Interrupt control
//

void hal_interrupt_mask(int vector)
{
//    diag_printf("6hal_interrupt_mask(vector=%d) \n", vector);
#ifdef HAL_EXTENDED_INTERRUPT_MASK
    // Use platform specific handling, if defined
    // Note: this macro should do a 'return' for "extended" values of 'vector'
    // Normal vectors are handled by code subsequent to the macro call.
    HAL_EXTENDED_INTERRUPT_MASK(vector);
#endif
}

void hal_interrupt_unmask(int vector)
{
//    diag_printf("7hal_interrupt_unmask(vector=%d) \n", vector);

#ifdef HAL_EXTENDED_INTERRUPT_UNMASK
    // Use platform specific handling, if defined
    // Note: this macro should do a 'return' for "extended" values of 'vector'
    // Normal vectors are handled by code subsequent to the macro call.
    HAL_EXTENDED_INTERRUPT_UNMASK(vector);
#endif
}

void hal_interrupt_acknowledge(int vector)
{

//    diag_printf("8hal_interrupt_acknowledge(vector=%d) \n", vector);
#ifdef HAL_EXTENDED_INTERRUPT_UNMASK
    // Use platform specific handling, if defined
    // Note: this macro should do a 'return' for "extended" values of 'vector'
    // Normal vectors are handled by code subsequent to the macro call.
    HAL_EXTENDED_INTERRUPT_ACKNOWLEDGE(vector);
#endif
}

void hal_interrupt_configure(int vector, int level, int up)
{

#ifdef HAL_EXTENDED_INTERRUPT_CONFIGURE
    // Use platform specific handling, if defined
    // Note: this macro should do a 'return' for "extended" values of 'vector'
    // Normal vectors are handled by code subsequent to the macro call.
    HAL_EXTENDED_INTERRUPT_CONFIGURE(vector, level, up);
#endif
}

void hal_interrupt_set_level(int vector, int level)
{

#ifdef HAL_EXTENDED_INTERRUPT_SET_LEVEL
    // Use platform specific handling, if defined
    // Note: this macro should do a 'return' for "extended" values of 'vector'
    // Normal vectors are handled by code subsequent to the macro call.
    HAL_EXTENDED_INTERRUPT_SET_LEVEL(vector, level);
#endif

    // Interrupt priorities are not configurable.
}

unsigned int mxc_nfc_soc_setup(unsigned int pg_sz, unsigned int io_sz, unsigned int is_mlc)
{
    return MXC_NFC_V2;
}

static void check_correct_chip(void)
{
    if (find_correct_chip == CHIP_VERSION_UNKNOWN) {
        diag_printf("Unrecognized chip version: 0x%x!!!\n", read_system_rev());
        diag_printf("Assuming chip version=0x%x\n", system_rev);
    } else if (find_correct_chip == CHIP_VERSION_NONE) {
        diag_printf("Unrecognized chip: 0x%x!!!\n", readl(IIM_BASE_ADDR + IIM_PREV_OFF));
    }
}

RedBoot_init(check_correct_chip, RedBoot_INIT_LAST);
