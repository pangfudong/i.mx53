/*=============================================================================
//
//      hal_diag.c
//
//      HAL diagnostic output code
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
//===========================================================================*/

#include <pkgconf/hal.h>
#include <pkgconf/system.h>
#include CYGBLD_HAL_PLATFORM_H

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_arch.h>           // basic machine info
#include <cyg/hal/hal_intr.h>           // interrupt macros
#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_if.h>             // Calling interface definitions
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/drv_api.h>            // cyg_drv_interrupt_acknowledge
#include <cyg/hal/hal_misc.h>           // Helper functions
#include <cyg/hal/hal_soc.h>         // Hardware definitions

/*
 * UART Control Register 0 Bit Fields.
 */
#define EUartUCR1_ADEN      (1 << 15)           // Auto dectect interrupt
#define EUartUCR1_ADBR      (1 << 14)           // Auto detect baud rate
#define EUartUCR1_TRDYEN    (1 << 13)           // Transmitter ready interrupt enable
#define EUartUCR1_IDEN      (1 << 12)           // Idle condition interrupt
#define EUartUCR1_RRDYEN    (1 << 9)            // Recv ready interrupt enable
#define EUartUCR1_RDMAEN    (1 << 8)            // Recv ready DMA enable
#define EUartUCR1_IREN      (1 << 7)            // Infrared interface enable
#define EUartUCR1_TXMPTYEN  (1 << 6)            // Transimitter empty interrupt enable
#define EUartUCR1_RTSDEN    (1 << 5)            // RTS delta interrupt enable
#define EUartUCR1_SNDBRK    (1 << 4)            // Send break
#define EUartUCR1_TDMAEN    (1 << 3)            // Transmitter ready DMA enable
#define EUartUCR1_DOZE      (1 << 1)            // Doze
#define EUartUCR1_UARTEN    (1 << 0)            // UART enabled
#define EUartUCR2_ESCI      (1 << 15)           // Escape seq interrupt enable
#define EUartUCR2_IRTS      (1 << 14)           // Ignore RTS pin
#define EUartUCR2_CTSC      (1 << 13)           // CTS pin control
#define EUartUCR2_CTS       (1 << 12)           // Clear to send
#define EUartUCR2_ESCEN     (1 << 11)           // Escape enable
#define EUartUCR2_PREN      (1 << 8)            // Parity enable
#define EUartUCR2_PROE      (1 << 7)            // Parity odd/even
#define EUartUCR2_STPB      (1 << 6)            // Stop
#define EUartUCR2_WS        (1 << 5)            // Word size
#define EUartUCR2_RTSEN     (1 << 4)            // Request to send interrupt enable
#define EUartUCR2_ATEN      (1 << 3)            // Aging timer enable
#define EUartUCR2_TXEN      (1 << 2)            // Transmitter enabled
#define EUartUCR2_RXEN      (1 << 1)            // Receiver enabled
#define EUartUCR2_SRST_     (1 << 0)            // SW reset
#define EUartUCR3_PARERREN  (1 << 12)           // Parity enable
#define EUartUCR3_FRAERREN  (1 << 11)           // Frame error interrupt enable
#define EUartUCR3_ADNIMP    (1 << 7)            // Autobaud detection not improved
#define EUartUCR3_RXDSEN    (1 << 6)            // Receive status interrupt enable
#define EUartUCR3_AIRINTEN  (1 << 5)            // Async IR wake interrupt enable
#define EUartUCR3_AWAKEN    (1 << 4)            // Async wake interrupt enable
#define EUartUCR3_RXDMUXSEL (1 << 2)            // RXD muxed input selected
#define EUartUCR3_INVT      (1 << 1)            // Inverted Infrared transmission
#define EUartUCR3_ACIEN     (1 << 0)            // Autobaud counter interrupt enable
#define EUartUCR4_CTSTL_32  (32 << 10)          // CTS trigger level (32 chars)
#define EUartUCR4_INVR      (1 << 9)            // Inverted infrared reception
#define EUartUCR4_ENIRI     (1 << 8)            // Serial infrared interrupt enable
#define EUartUCR4_WKEN      (1 << 7)            // Wake interrupt enable
#define EUartUCR4_IRSC      (1 << 5)            // IR special case
#define EUartUCR4_LPBYP     (1 << 4)            // Low power bypass
#define EUartUCR4_TCEN      (1 << 3)            // Transmit complete interrupt enable
#define EUartUCR4_BKEN      (1 << 2)            // Break condition interrupt enable
#define EUartUCR4_OREN      (1 << 1)            // Receiver overrun interrupt enable
#define EUartUCR4_DREN      (1 << 0)            // Recv data ready interrupt enable
#define EUartUFCR_RXTL_SHF  0                   // Receiver trigger level shift
#define EUartUFCR_RFDIV_1   (5 << 7)            // Reference freq divider (div 1)
#define EUartUFCR_RFDIV_2   (4 << 7)            // Reference freq divider (div 2)
#define EUartUFCR_RFDIV_3   (3 << 7)            // Reference freq divider (div 3)
#define EUartUFCR_RFDIV_4   (2 << 7)            // Reference freq divider (div 4)
#define EUartUFCR_RFDIV_5   (1 << 7)            // Reference freq divider (div 5)
#define EUartUFCR_RFDIV_6   (0 << 7)            // Reference freq divider (div 6)
#define EUartUFCR_RFDIV_7   (6 << 7)            // Reference freq divider (div 7)
#define EUartUFCR_TXTL_SHF  10                  // Transmitter trigger level shift
#define EUartUSR1_PARITYERR (1 << 15)           // Parity error interrupt flag
#define EUartUSR1_RTSS      (1 << 14)           // RTS pin status
#define EUartUSR1_TRDY      (1 << 13)           // Transmitter ready interrupt/dma flag
#define EUartUSR1_RTSD      (1 << 12)           // RTS delta
#define EUartUSR1_ESCF      (1 << 11)           // Escape seq interrupt flag
#define EUartUSR1_FRAMERR   (1 << 10)           // Frame error interrupt flag
#define EUartUSR1_RRDY      (1 << 9)            // Receiver ready interrupt/dma flag
#define EUartUSR1_AGTIM     (1 << 8)            // Aging timeout interrupt status
#define EUartUSR1_RXDS      (1 << 6)            // Receiver idle interrupt flag
#define EUartUSR1_AIRINT    (1 << 5)            // Async IR wake interrupt flag
#define EUartUSR1_AWAKE     (1 << 4)            // Aysnc wake interrupt flag
#define EUartUSR2_ADET      (1 << 15)           // Auto baud rate detect complete
#define EUartUSR2_TXFE      (1 << 14)           // Transmit buffer FIFO empty
#define EUartUSR2_IDLE      (1 << 12)           // Idle condition
#define EUartUSR2_ACST      (1 << 11)           // Autobaud counter stopped
#define EUartUSR2_IRINT     (1 << 8)            // Serial infrared interrupt flag
#define EUartUSR2_WAKE      (1 << 7)            // Wake
#define EUartUSR2_RTSF      (1 << 4)            // RTS edge interrupt flag
#define EUartUSR2_TXDC      (1 << 3)            // Transmitter complete
#define EUartUSR2_BRCD      (1 << 2)            // Break condition
#define EUartUSR2_ORE       (1 << 1)            // Overrun error
#define EUartUSR2_RDR       (1 << 0)            // Recv data ready
#define EUartUTS_FRCPERR    (1 << 13)           // Force parity error
#define EUartUTS_LOOP       (1 << 12)           // Loop tx and rx
#define EUartUTS_TXEMPTY    (1 << 6)            // TxFIFO empty
#define EUartUTS_RXEMPTY    (1 << 5)            // RxFIFO empty
#define EUartUTS_TXFULL     (1 << 4)            // TxFIFO full
#define EUartUTS_RXFULL     (1 << 3)            // RxFIFO full
#define EUartUTS_SOFTRST    (1 << 0)            // Software reset

#define EUartUFCR_RFDIV                        EUartUFCR_RFDIV_2
//#define EUartUFCR_RFDIV                        EUartUFCR_RFDIV_4
//#define EUartUFCR_RFDIV                        EUartUFCR_RFDIV_7

#if (EUartUFCR_RFDIV==EUartUFCR_RFDIV_2)
#define MXC_UART_REFFREQ                        (get_peri_clock(UART1_BAUD) / 2)
#endif

#if (EUartUFCR_RFDIV==EUartUFCR_RFDIV_4)
#define MXC_UART_REFFREQ                        (get_peri_clock(UART1_BAUD) / 4)
#endif

#if (EUartUFCR_RFDIV==EUartUFCR_RFDIV_7)
#define MXC_UART_REFFREQ                        (get_peri_clock(UART1_BAUD) / 7)
#endif

#if 0
void
cyg_hal_plf_comms_init(void)
{
    static int initialized = 0;

    if (initialized)
        return;

    initialized = 1;

    cyg_hal_plf_serial_init();
}
#endif

//=============================================================================
// MXC Serial Port (UARTx) for Debug
//=============================================================================
#ifdef UART_WIDTH_32
struct mxc_serial {
    volatile cyg_uint32 urxd[16];
    volatile cyg_uint32 utxd[16];
    volatile cyg_uint32 ucr1;
    volatile cyg_uint32 ucr2;
    volatile cyg_uint32 ucr3;
    volatile cyg_uint32 ucr4;
    volatile cyg_uint32 ufcr;
    volatile cyg_uint32 usr1;
    volatile cyg_uint32 usr2;
    volatile cyg_uint32 uesc;
    volatile cyg_uint32 utim;
    volatile cyg_uint32 ubir;
    volatile cyg_uint32 ubmr;
    volatile cyg_uint32 ubrc;
    volatile cyg_uint32 onems;
    volatile cyg_uint32 uts;
};
#else
struct mxc_serial {
    volatile cyg_uint16 urxd[1];
    volatile cyg_uint16 resv0[31];

    volatile cyg_uint16 utxd[1];
    volatile cyg_uint16 resv1[31];
    volatile cyg_uint16 ucr1;
    volatile cyg_uint16 resv2;
    volatile cyg_uint16 ucr2;
    volatile cyg_uint16 resv3;
    volatile cyg_uint16 ucr3;
    volatile cyg_uint16 resv4;
    volatile cyg_uint16 ucr4;
    volatile cyg_uint16 resv5;
    volatile cyg_uint16 ufcr;
    volatile cyg_uint16 resv6;
    volatile cyg_uint16 usr1;
    volatile cyg_uint16 resv7;
    volatile cyg_uint16 usr2;
    volatile cyg_uint16 resv8;
    volatile cyg_uint16 uesc;
    volatile cyg_uint16 resv9;
    volatile cyg_uint16 utim;
    volatile cyg_uint16 resv10;
    volatile cyg_uint16 ubir;
    volatile cyg_uint16 resv11;
    volatile cyg_uint16 ubmr;
    volatile cyg_uint16 resv12;
    volatile cyg_uint16 ubrc;
    volatile cyg_uint16 resv13;
    volatile cyg_uint16 onems;
    volatile cyg_uint16 resv14;
    volatile cyg_uint16 uts;
    volatile cyg_uint16 resv15;
};
#endif

typedef struct {
    volatile struct mxc_serial* base;
    cyg_int32 msec_timeout;
    int isr_vector;
    int baud_rate;
} channel_data_t;

static channel_data_t channels[] = {
#if CYGHWR_HAL_ARM_SOC_UART1 != 0
    {(volatile struct mxc_serial*)UART1_BASE_ADDR, 1000,
      CYGNUM_HAL_INTERRUPT_UART1, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD},
#endif
#if CYGHWR_HAL_ARM_SOC_UART2 != 0
    {(volatile struct mxc_serial*)UART2_BASE_ADDR, 1000,
     CYGNUM_HAL_INTERRUPT_UART2, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD},
#endif
#if CYGHWR_HAL_ARM_SOC_UART3 != 0
    {(volatile struct mxc_serial*)UART3_BASE_ADDR, 1000,
     CYGNUM_HAL_INTERRUPT_UART3, CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD},
#endif
};

/*---------------------------------------------------------------------------*/

static void init_serial_channel(channel_data_t* __ch_data)
{
    volatile struct mxc_serial* base = __ch_data->base;

    /* Wait for UART to finish transmitting */
    while (!(base->uts & EUartUTS_TXEMPTY));

    /* Disable UART */
    base->ucr1 &= ~EUartUCR1_UARTEN;

    /* Set to default POR state */
    base->ucr1 = 0x00000000;
    base->ucr2 = 0x00000000;

    while (!(base->ucr2 & EUartUCR2_SRST_));

    base->ucr3 = 0x00000704;
    base->ucr4 = 0x00008000;
    base->ufcr = 0x00000801;
    base->uesc = 0x0000002B;
    base->utim = 0x00000000;
    base->ubir = 0x00000000;
    base->ubmr = 0x00000000;
    base->onems = 0x00000000;
    base->uts  = 0x00000000;

    /* Configure FIFOs */
    base->ufcr = (1 << EUartUFCR_RXTL_SHF) | EUartUFCR_RFDIV
                 | (2 << EUartUFCR_TXTL_SHF);

    /* Setup One MS timer */
    base->onems  = (MXC_UART_REFFREQ / 1000);

    /* Set to 8N1 */
    base->ucr2 &= ~EUartUCR2_PREN;
    base->ucr2 |= EUartUCR2_WS;
    base->ucr2 &= ~EUartUCR2_STPB;

    /* Ignore RTS */
    base->ucr2 |= EUartUCR2_IRTS;

    /* Enable UART */
    base->ucr1 |= EUartUCR1_UARTEN;

    /* Enable FIFOs */
    base->ucr2 |= EUartUCR2_SRST_ | EUartUCR2_RXEN | EUartUCR2_TXEN;

    /* Clear status flags */
    base->usr2 |= EUartUSR2_ADET  |
                  EUartUSR2_IDLE  |
                  EUartUSR2_IRINT |
                  EUartUSR2_WAKE  |
                  EUartUSR2_RTSF  |
                  EUartUSR2_BRCD  |
                  EUartUSR2_ORE   |
                  EUartUSR2_RDR;

    /* Clear status flags */
    base->usr1 |= EUartUSR1_PARITYERR |
                  EUartUSR1_RTSD      |
                  EUartUSR1_ESCF      |
                  EUartUSR1_FRAMERR   |
                  EUartUSR1_AIRINT    |
                  EUartUSR1_AWAKE;

    /* Set the numerator value minus one of the BRM ratio */
    base->ubir = (__ch_data->baud_rate / 100) - 1;

    /* Set the denominator value minus one of the BRM ratio    */
    base->ubmr = ((MXC_UART_REFFREQ / 1600) - 1);

}

static void stop_serial_channel(channel_data_t* __ch_data)
{
    volatile struct mxc_serial* base = __ch_data->base;

    /* Wait for UART to finish transmitting */
    while (!(base->uts & EUartUTS_TXEMPTY));

    /* Disable UART */
    base->ucr1 &= ~EUartUCR1_UARTEN;
}

//#define debug_uart_log_buf
#ifdef debug_uart_log_buf
#define DIAG_BUFSIZE 2048
static char __log_buf[DIAG_BUFSIZE];
static int diag_bp = 0;
#endif

void cyg_hal_plf_serial_putc(void *__ch_data, char c)
{
    volatile struct mxc_serial* base = ((channel_data_t*)__ch_data)->base;

#ifdef debug_uart_log_buf
    __log_buf[diag_bp++] = c;
    return;
#endif

    CYGARC_HAL_SAVE_GP();

    // Wait for Tx FIFO not full
    while (base->uts & EUartUTS_TXFULL)
        ;
    base->utxd[0] = c;

    CYGARC_HAL_RESTORE_GP();
}

static cyg_bool cyg_hal_plf_serial_getc_nonblock(void* __ch_data,
                                                 cyg_uint8* ch)
{
    volatile struct mxc_serial* base = ((channel_data_t*)__ch_data)->base;

    // If receive fifo is empty, return false
    if (base->uts & EUartUTS_RXEMPTY)
        return false;

    *ch = (char)base->urxd[0];

    return true;
}

cyg_uint8 cyg_hal_plf_serial_getc(void* __ch_data)
{
    cyg_uint8 ch;
    CYGARC_HAL_SAVE_GP();

    while (!cyg_hal_plf_serial_getc_nonblock(__ch_data, &ch));

    CYGARC_HAL_RESTORE_GP();
    return ch;
}

static void cyg_hal_plf_serial_write(void* __ch_data, const cyg_uint8* __buf,
                         cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while(__len-- > 0)
        cyg_hal_plf_serial_putc(__ch_data, *__buf++);

    CYGARC_HAL_RESTORE_GP();
}

static void cyg_hal_plf_serial_read(void* __ch_data, cyg_uint8* __buf,
                                    cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while (__len-- > 0)
        *__buf++ = cyg_hal_plf_serial_getc(__ch_data);

    CYGARC_HAL_RESTORE_GP();
}

cyg_bool cyg_hal_plf_serial_getc_timeout(void* __ch_data,
                                         cyg_uint8* ch)
{
    int delay_count;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    cyg_bool res;
    CYGARC_HAL_SAVE_GP();

    delay_count = chan->msec_timeout * 10; // delay in .1 ms steps

    for(;;) {
        res = cyg_hal_plf_serial_getc_nonblock(__ch_data, ch);
        if (res || 0 == delay_count--)
            break;

        CYGACC_CALL_IF_DELAY_US(100);
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static int cyg_hal_plf_serial_control(void *__ch_data,
                                      __comm_control_cmd_t __func, ...)
{
    static int irq_state = 0;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    int ret = -1;
    va_list ap;

    CYGARC_HAL_SAVE_GP();
    va_start(ap, __func);

    switch (__func) {
    case __COMMCTL_GETBAUD:
        ret = chan->baud_rate;
        break;
    case __COMMCTL_SETBAUD:
        chan->baud_rate = va_arg(ap, cyg_int32);
        // Should we verify this value here?
        init_serial_channel(chan);
        ret = 0;
        break;
    case __COMMCTL_IRQ_ENABLE:
        irq_state = 1;

        chan->base->ucr1 |= EUartUCR1_RRDYEN;

        HAL_INTERRUPT_UNMASK(chan->isr_vector);
        break;
    case __COMMCTL_IRQ_DISABLE:
        ret = irq_state;
        irq_state = 0;

        chan->base->ucr1 &= ~EUartUCR1_RRDYEN;

        HAL_INTERRUPT_MASK(chan->isr_vector);
        break;
    case __COMMCTL_DBG_ISR_VECTOR:
        ret = chan->isr_vector;
        break;
    case __COMMCTL_SET_TIMEOUT:
        ret = chan->msec_timeout;
        chan->msec_timeout = va_arg(ap, cyg_uint32);
        break;
    default:
        break;
    }
    va_end(ap);
    CYGARC_HAL_RESTORE_GP();
    return ret;
}

static int cyg_hal_plf_serial_isr(void *__ch_data, int* __ctrlc,
                       CYG_ADDRWORD __vector, CYG_ADDRWORD __data)
{
    int res = 0;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    char c;

    CYGARC_HAL_SAVE_GP();

    cyg_drv_interrupt_acknowledge(chan->isr_vector);

    *__ctrlc = 0;
    if (!(chan->base->uts & EUartUTS_RXEMPTY)) {
	c = (char)chan->base->urxd[0];

        if (cyg_hal_is_break( &c , 1 ))
            *__ctrlc = 1;

        res = CYG_ISR_HANDLED;
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

void cyg_hal_plf_serial_init(void)
{
    hal_virtual_comm_table_t* comm;
    int cur = CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);
    int i;
    static int jjj = 0;

    // Init channels
#define NUMOF(x) (sizeof(x)/sizeof(x[0]))
    for (i = 0;  i < NUMOF(channels);  i++) {
        init_serial_channel(&channels[i]);
        CYGACC_CALL_IF_SET_CONSOLE_COMM(i+2);
        comm = CYGACC_CALL_IF_CONSOLE_PROCS();
        CYGACC_COMM_IF_CH_DATA_SET(*comm, &channels[i]);
        CYGACC_COMM_IF_WRITE_SET(*comm, cyg_hal_plf_serial_write);
        CYGACC_COMM_IF_READ_SET(*comm, cyg_hal_plf_serial_read);
        CYGACC_COMM_IF_PUTC_SET(*comm, cyg_hal_plf_serial_putc);
        CYGACC_COMM_IF_GETC_SET(*comm, cyg_hal_plf_serial_getc);
        CYGACC_COMM_IF_CONTROL_SET(*comm, cyg_hal_plf_serial_control);
        CYGACC_COMM_IF_DBG_ISR_SET(*comm, cyg_hal_plf_serial_isr);
        CYGACC_COMM_IF_GETC_TIMEOUT_SET(*comm, cyg_hal_plf_serial_getc_timeout);
        if (jjj == 0) {
            cyg_hal_plf_serial_putc(&channels[i], '+');
            jjj++;
        }
        cyg_hal_plf_serial_putc(&channels[i], '+');
    }

    // Restore original console
    CYGACC_CALL_IF_SET_CONSOLE_COMM(cur);
}

void cyg_hal_plf_serial_stop(void)
{
        int i;

        // Init channels
#define NUMOF(x) (sizeof(x)/sizeof(x[0]))
        for (i = 0;  i < NUMOF(channels);  i++) {
                stop_serial_channel(&channels[i]);
        }
}

//=============================================================================
// Compatibility with older stubs
//=============================================================================

#ifndef CYGSEM_HAL_VIRTUAL_VECTOR_DIAG

#include <cyg/hal/hal_stub.h>           // cyg_hal_gdb_interrupt

#if (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL == 2)
#define __BASE ((void*)UART1_BASE_ADDR)
#define CYGHWR_HAL_GDB_PORT_VECTOR CYGNUM_HAL_INTERRUPT_UART1
#elif (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL == 3)
#define __BASE ((void*)UART2_BASE_ADDR)
#define CYGHWR_HAL_GDB_PORT_VECTOR CYGNUM_HAL_INTERRUPT_UART2
#elif (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL == 4)
#define __BASE ((void*)UART3_BASE_ADDR)
#endif

#ifdef __BASE

#ifdef CYGSEM_HAL_ROM_MONITOR
#define CYG_HAL_STARTUP_ROM
#define CYG_HAL_STARTUP_ROMRAM
#undef CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS
#endif

#if (defined(CYG_HAL_STARTUP_ROM) || defined(CYG_HAL_STARTUP_ROMRAM)) && !defined(CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS)
#define HAL_DIAG_USES_HARDWARE
#elif !defined(CYGDBG_HAL_DIAG_TO_DEBUG_CHAN)
#define HAL_DIAG_USES_HARDWARE
#elif CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL != CYGNUM_HAL_VIRTUAL_VECTOR_DEBUG_CHANNEL
#define HAL_DIAG_USES_HARDWARE
#endif

static channel_data_t channel = {
    (volatile struct mxc_serial*)__BASE, 0, CYGHWR_HAL_GDB_PORT_VECTOR
};

#ifdef HAL_DIAG_USES_HARDWARE

void hal_diag_init(void)
{
    static int init = 0;
    char *msg = "\n\rARM eCos\n\r";
    cyg_uint8 lcr;

    if (init++) return;

    init_serial_channel(&channel);

    while (*msg) hal_diag_write_char(*msg++);
}

#ifdef DEBUG_DIAG
#ifndef CYG_HAL_STARTUP_ROM
#define DIAG_BUFSIZE 2048
static char diag_buffer[DIAG_BUFSIZE];
static int diag_bp = 0;
#endif
#endif

void hal_diag_write_char(char c)
{
#ifdef DEBUG_DIAG
#ifndef CYG_HAL_STARTUP_ROM
    diag_buffer[diag_bp++] = c;
    if (diag_bp == sizeof(diag_buffer)) diag_bp = 0;
#endif
#endif
    cyg_hal_plf_serial_putc(&channel, c);
}

void hal_diag_read_char(char *c)
{
    *c = cyg_hal_plf_serial_getc(&channel);
}

#else // not HAL_DIAG_USES_HARDWARE - it uses GDB protocol

void hal_diag_read_char(char *c)
{
    *c = cyg_hal_plf_serial_getc(&channel);
}

void hal_diag_write_char(char c)
{
    static char line[100];
    static int pos = 0;

    // FIXME: Some LED blinking might be nice right here.

    // No need to send CRs
    if( c == '\r' ) return;

    line[pos++] = c;

        if (c == '\n' || pos == sizeof(line)) {
        CYG_INTERRUPT_STATE old;

        // Disable interrupts. This prevents GDB trying to interrupt us
        // while we are in the middle of sending a packet. The serial
        // receive interrupt will be seen when we re-enable interrupts
        // later.

#ifdef CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS
        CYG_HAL_GDB_ENTER_CRITICAL_IO_REGION(old);
#else
        HAL_DISABLE_INTERRUPTS(old);
#endif

        while (1) {
            static char hex[] = "0123456789ABCDEF";
            cyg_uint8 csum = 0;
            int i;
#ifndef CYGDBG_HAL_DEBUG_GDB_CTRLC_SUPPORT
            char c1;
#endif
            cyg_hal_plf_serial_putc(&channel, '$');
            cyg_hal_plf_serial_putc(&channel, 'O');
            csum += 'O';
            for(i = 0; i < pos; i++) {
                char ch = line[i];
                char h = hex[(ch>>4)&0xF];
                char l = hex[ch&0xF];
                cyg_hal_plf_serial_putc(&channel, h);
                cyg_hal_plf_serial_putc(&channel, l);
                csum += h;
                csum += l;
            }
            cyg_hal_plf_serial_putc(&channel, '#');
            cyg_hal_plf_serial_putc(&channel, hex[(csum>>4)&0xF]);
            cyg_hal_plf_serial_putc(&channel, hex[csum&0xF]);

#ifdef CYGDBG_HAL_DEBUG_GDB_CTRLC_SUPPORT

            break; // regardless

#else // not CYGDBG_HAL_DEBUG_GDB_CTRLC_SUPPORT Ie. usually...

            // Wait for the ACK character '+' from GDB here and handle
            // receiving a ^C instead.  This is the reason for this clause
            // being a loop.
            c1 = cyg_hal_plf_serial_getc(&channel);

            if( c1 == '+' )
                break;              // a good acknowledge

#ifdef CYGDBG_HAL_DEBUG_GDB_BREAK_SUPPORT
            cyg_drv_interrupt_acknowledge(CYGHWR_HAL_GDB_PORT_VECTOR);
            if( c1 == 3 ) {
                // Ctrl-C: breakpoint.
                cyg_hal_gdb_interrupt(
                    (target_register_t)__builtin_return_address(0) );
                break;
            }
#endif // CYGDBG_HAL_DEBUG_GDB_BREAK_SUPPORT

#endif // ! CYGDBG_HAL_DEBUG_GDB_CTRLC_SUPPORT
            // otherwise, loop round again
        }

        pos = 0;

        // And re-enable interrupts
#ifdef CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS
        CYG_HAL_GDB_LEAVE_CRITICAL_IO_REGION(old);
#else
        HAL_RESTORE_INTERRUPTS(old);
#endif

    }
}
#endif

#endif // __BASE

#endif // !CYGSEM_HAL_VIRTUAL_VECTOR_DIAG

/*---------------------------------------------------------------------------*/
/* End of hal_diag.c */
