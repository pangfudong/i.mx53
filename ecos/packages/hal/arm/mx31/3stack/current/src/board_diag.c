/*=============================================================================
//
//      board_diag.c
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

#include <cyg/infra/cyg_type.h>         // base types
#include <cyg/infra/cyg_trac.h>         // tracing macros
#include <cyg/infra/cyg_ass.h>          // assertion macros

#include <cyg/hal/hal_arch.h>           // basic machine info
#include <cyg/hal/hal_intr.h>           // interrupt macros
#include <cyg/hal/hal_io.h>             // IO macros
#include <cyg/hal/hal_diag.h>
#include <cyg/hal/hal_if.h>             // Calling-if API
#include <cyg/hal/drv_api.h>            // driver API
#include <cyg/hal/hal_misc.h>           // Helper functions
#include <cyg/hal/hal_soc.h>            // Hardware definitions
#include <cyg/hal/fsl_board.h>          // Platform specifics

static void cyg_hal_plf_duart_init(void);
extern void cyg_hal_plf_serial_init(void);

#define DUART_WORKAROUND_DELAY(a)    hal_delay_us(a);


void cyg_hal_plf_comms_init(void)
{
    static int initialized = 0;

    if (initialized)
        return;

    initialized = 1;

    /* Setup GPIO and enable transceiver for UARTs */
    cyg_hal_plf_duart_init();
    cyg_hal_plf_serial_init();
}

//=============================================================================
// ST16552 DUART driver
//=============================================================================

//-----------------------------------------------------------------------------
// There are two serial ports.
#define CYG_DEV_SERIAL_BASE_A    (BOARD_CS_UART_BASE + 0x0000) // port A
#define CYG_DEV_SERIAL_BASE_B    (BOARD_CS_UART_BASE + 0x8000) // port B

//-----------------------------------------------------------------------------
// Based on 14.7456 MHz xtal
#if CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD==9600
#define CYG_DEV_SERIAL_BAUD_MSB        0x00
#define CYG_DEV_SERIAL_BAUD_LSB        0x60
#endif
#if CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD==19200
#define CYG_DEV_SERIAL_BAUD_MSB        0x00
#define CYG_DEV_SERIAL_BAUD_LSB        0x30
#endif
#if CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD==38400
#define CYG_DEV_SERIAL_BAUD_MSB        0x00
#define CYG_DEV_SERIAL_BAUD_LSB        0x18
#endif
#if CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD==57600
#define CYG_DEV_SERIAL_BAUD_MSB        0x00
#define CYG_DEV_SERIAL_BAUD_LSB        0x10
#endif
#if CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL_BAUD==115200
#define CYG_DEV_SERIAL_BAUD_MSB        0x00
#define CYG_DEV_SERIAL_BAUD_LSB        0x08
#endif

#ifndef CYG_DEV_SERIAL_BAUD_MSB
#error Missing/incorrect serial baud rate defined - CDL error?
#endif

//-----------------------------------------------------------------------------
// Define the serial registers. The board is equipped with a 16552
// serial chip.

#ifdef EXT_UART_x16
#define HAL_WRITE_UINT_UART HAL_WRITE_UINT16
#define HAL_READ_UINT_UART HAL_READ_UINT16
typedef cyg_uint16 uart_width;
#else  //_x8
typedef cyg_uint8 uart_width;
#define HAL_WRITE_UINT_UART HAL_WRITE_UINT8
#define HAL_READ_UINT_UART HAL_READ_UINT8
#endif

#define CYG_DEV_SERIAL_RHR   0x00 // receiver buffer register, read, dlab = 0
#define CYG_DEV_SERIAL_THR   0x00 // transmitter holding register, write, dlab = 0
#define CYG_DEV_SERIAL_DLL   0x00 // divisor latch (LS), read/write, dlab = 1
#define CYG_DEV_SERIAL_IER   0x01 // interrupt enable register, read/write, dlab = 0
#define CYG_DEV_SERIAL_DLM   0x01 // divisor latch (MS), read/write, dlab = 1
#define CYG_DEV_SERIAL_IIR   0x02 // interrupt identification register, read, dlab = 0
#define CYG_DEV_SERIAL_FCR   0x02 // fifo control register, write, dlab = 0
#define CYG_DEV_SERIAL_AFR   0x02 // alternate function register, read/write, dlab = 1
#define CYG_DEV_SERIAL_LCR   0x03 // line control register, read/write
#define CYG_DEV_SERIAL_MCR   0x04
#define CYG_DEV_SERIAL_MCR_A 0x04
#define CYG_DEV_SERIAL_MCR_B 0x04
#define CYG_DEV_SERIAL_LSR   0x05 // line status register, read
#define CYG_DEV_SERIAL_MSR   0x06 // modem status register, read
#define CYG_DEV_SERIAL_SCR   0x07 // scratch pad register

// The interrupt enable register bits.
#define SIO_IER_ERDAI   0x01            // enable received data available irq
#define SIO_IER_ETHREI  0x02            // enable THR empty interrupt
#define SIO_IER_ELSI    0x04            // enable receiver line status irq
#define SIO_IER_EMSI    0x08            // enable modem status interrupt

// The interrupt identification register bits.
#define SIO_IIR_IP      0x01            // 0 if interrupt pending
#define SIO_IIR_ID_MASK 0x0e            // mask for interrupt ID bits
#define ISR_Tx          0x02
#define ISR_Rx          0x04

// The line status register bits.
#define SIO_LSR_DR      0x01            // data ready
#define SIO_LSR_OE      0x02            // overrun error
#define SIO_LSR_PE      0x04            // parity error
#define SIO_LSR_FE      0x08            // framing error
#define SIO_LSR_BI      0x10            // break interrupt
#define SIO_LSR_THRE    0x20            // transmitter holding register empty
#define SIO_LSR_TEMT    0x40            // transmitter register empty
#define SIO_LSR_ERR     0x80            // any error condition

// The modem status register bits.
#define SIO_MSR_DCTS    0x01            // delta clear to send
#define SIO_MSR_DDSR    0x02            // delta data set ready
#define SIO_MSR_TERI    0x04            // trailing edge ring indicator
#define SIO_MSR_DDCD    0x08            // delta data carrier detect
#define SIO_MSR_CTS     0x10            // clear to send
#define SIO_MSR_DSR     0x20            // data set ready
#define SIO_MSR_RI      0x40            // ring indicator
#define SIO_MSR_DCD     0x80            // data carrier detect

// The line control register bits.
#define SIO_LCR_WLS0   0x01             // word length select bit 0
#define SIO_LCR_WLS1   0x02             // word length select bit 1
#define SIO_LCR_STB    0x04             // number of stop bits
#define SIO_LCR_PEN    0x08             // parity enable
#define SIO_LCR_EPS    0x10             // even parity select
#define SIO_LCR_SP     0x20             // stick parity
#define SIO_LCR_SB     0x40             // set break
#define SIO_LCR_DLAB   0x80             // divisor latch access bit

// The FIFO control register
#define SIO_FCR_FCR0   0x01             // enable xmit and rcvr fifos
#define SIO_FCR_FCR1   0x02             // clear RCVR FIFO
#define SIO_FCR_FCR2   0x04             // clear XMIT FIFO

//-----------------------------------------------------------------------------

typedef struct {
    uart_width* base;
    cyg_int32 msec_timeout;
    int isr_vector;
} channel_data_t;

static channel_data_t channels[] = {
#if CYGHWR_HAL_ARM_DUART_UARTA != 0
    {(uart_width*)CYG_DEV_SERIAL_BASE_A, 1000, 0},
#endif
#if CYGHWR_HAL_ARM_DUART_UARTB != 0
    {(uart_width*)CYG_DEV_SERIAL_BASE_B, 1000, 0}
#endif
};

//-----------------------------------------------------------------------------

static void init_duart_channel(channel_data_t* __ch_data)
{
    uart_width* base = __ch_data->base;
    uart_width lcr;

    // 8-1-no parity.
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_LCR,
                        SIO_LCR_WLS0 | SIO_LCR_WLS1);
    DUART_WORKAROUND_DELAY(50);
    HAL_READ_UINT_UART(base+CYG_DEV_SERIAL_LCR, lcr);
    lcr |= SIO_LCR_DLAB;
    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_LCR, lcr);
    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_DLL, CYG_DEV_SERIAL_BAUD_LSB);
    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_DLM, CYG_DEV_SERIAL_BAUD_MSB);
    lcr &= ~SIO_LCR_DLAB;
    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_LCR, lcr);
    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_FCR, 0x07);  // Enable & clear FIFO
}

//#define x_debug_uart_log_buf
#ifdef x_debug_uart_log_buf
#define x_DIAG_BUFSIZE 2048
static char __x_log_buf[x_DIAG_BUFSIZE];
static int x_diag_bp = 0;
#endif

void cyg_hal_plf_duart_putc(void* __ch_data, cyg_uint8 c)
{
    uart_width* base = ((channel_data_t*)__ch_data)->base;
    uart_width lsr;

#ifdef x_debug_uart_log_buf
    __x_log_buf[x_diag_bp++] = c;
#endif
    CYGARC_HAL_SAVE_GP();

    do {
        HAL_READ_UINT_UART(base+CYG_DEV_SERIAL_LSR, lsr);
    } while ((lsr & SIO_LSR_THRE) == 0);

    DUART_WORKAROUND_DELAY(50);
    HAL_WRITE_UINT_UART(base+CYG_DEV_SERIAL_THR, c);

    // Hang around until the character has been safely sent.
    do {
        HAL_READ_UINT_UART(base+CYG_DEV_SERIAL_LSR, lsr);
    } while ((lsr & SIO_LSR_THRE) == 0);

    CYGARC_HAL_RESTORE_GP();
}

static cyg_bool cyg_hal_plf_duart_getc_nonblock(void* __ch_data, cyg_uint8* ch)
{
    uart_width* base = ((channel_data_t*)__ch_data)->base;
    uart_width lsr, ch16;

    HAL_READ_UINT_UART(base+CYG_DEV_SERIAL_LSR, lsr);
    if ((lsr & SIO_LSR_DR) == 0)
        return false;

    HAL_READ_UINT_UART(base+CYG_DEV_SERIAL_RHR, ch16);

    *ch = (cyg_uint8) (ch16 & 0x00FF);

    return true;
}

cyg_uint8 cyg_hal_plf_duart_getc(void* __ch_data)
{
    cyg_uint8 ch;

    CYGARC_HAL_SAVE_GP();

    while (!cyg_hal_plf_duart_getc_nonblock(__ch_data, &ch));

    CYGARC_HAL_RESTORE_GP();
    return ch;
}

static void cyg_hal_plf_duart_write(void* __ch_data, const cyg_uint8* __buf,
                                    cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while (__len-- > 0)
        cyg_hal_plf_duart_putc(__ch_data, *__buf++);

    CYGARC_HAL_RESTORE_GP();
}

static void cyg_hal_plf_duart_read(void* __ch_data, cyg_uint8* __buf,
                                   cyg_uint32 __len)
{
    CYGARC_HAL_SAVE_GP();

    while (__len-- > 0)
        *__buf++ = cyg_hal_plf_duart_getc(__ch_data);

    CYGARC_HAL_RESTORE_GP();
}

cyg_bool cyg_hal_plf_duart_getc_timeout(void* __ch_data, cyg_uint8* ch)
{
    int delay_count;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    cyg_bool res;

    CYGARC_HAL_SAVE_GP();

    delay_count = chan->msec_timeout * 10; // delay in .1 ms steps
    for (;;) {
        res = cyg_hal_plf_duart_getc_nonblock(__ch_data, ch);
        if (res || 0 == delay_count--)
            break;

        CYGACC_CALL_IF_DELAY_US(100);
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static int cyg_hal_plf_duart_control(void *__ch_data,
                                     __comm_control_cmd_t __func, ...)
{
    static int irq_state = 0;
    channel_data_t* chan = (channel_data_t*)__ch_data;
    uart_width ier;
    int ret = 0;

    CYGARC_HAL_SAVE_GP();

    switch (__func) {
    case __COMMCTL_IRQ_ENABLE:
        HAL_INTERRUPT_UNMASK(chan->isr_vector);
        HAL_INTERRUPT_SET_LEVEL(chan->isr_vector, 1);
        HAL_READ_UINT_UART(chan->base+CYG_DEV_SERIAL_IER, ier);
        ier |= SIO_IER_ERDAI;
        HAL_WRITE_UINT_UART(chan->base+CYG_DEV_SERIAL_IER, ier);
        irq_state = 1;
        break;
    case __COMMCTL_IRQ_DISABLE:
        ret = irq_state;
        irq_state = 0;
        HAL_INTERRUPT_MASK(chan->isr_vector);
        HAL_READ_UINT_UART(chan->base+CYG_DEV_SERIAL_IER, ier);
        ier &= ~SIO_IER_ERDAI;
        HAL_WRITE_UINT_UART(chan->base+CYG_DEV_SERIAL_IER, ier);
        break;
    case __COMMCTL_DBG_ISR_VECTOR:
        ret = chan->isr_vector;
        break;
    case __COMMCTL_SET_TIMEOUT:
        {
            va_list ap;

            va_start(ap, __func);

            ret = chan->msec_timeout;
            chan->msec_timeout = va_arg(ap, cyg_uint32);

            va_end(ap);
        }
        break;
    default:
        break;
    }
    CYGARC_HAL_RESTORE_GP();
    return ret;
}

static int cyg_hal_plf_duart_isr(void *__ch_data, int* __ctrlc,
                                 CYG_ADDRWORD __vector, CYG_ADDRWORD __data)
{
    channel_data_t* chan = (channel_data_t*)__ch_data;
    uart_width _iir;
    int res = 0;
    CYGARC_HAL_SAVE_GP();

    HAL_READ_UINT_UART(chan->base+CYG_DEV_SERIAL_IIR, _iir);
    _iir &= SIO_IIR_ID_MASK;

    *__ctrlc = 0;
    if ( ISR_Rx == _iir ) {
        uart_width c, lsr;
        cyg_uint8 c8;
        HAL_READ_UINT_UART(chan->base+CYG_DEV_SERIAL_LSR, lsr);
        if (lsr & SIO_LSR_DR) {

            HAL_READ_UINT_UART(chan->base+CYG_DEV_SERIAL_RHR, c);

            c8 = (cyg_uint8) (c & 0x00FF);

            if (cyg_hal_is_break( &c8 , 1 ))
                *__ctrlc = 1;
        }

        // Acknowledge the interrupt
        HAL_INTERRUPT_ACKNOWLEDGE(chan->isr_vector);
        res = CYG_ISR_HANDLED;
    }

    CYGARC_HAL_RESTORE_GP();
    return res;
}

static void cyg_hal_plf_duart_init(void)
{
    hal_virtual_comm_table_t* comm;
    int cur = CYGACC_CALL_IF_SET_CONSOLE_COMM(CYGNUM_CALL_IF_SET_COMM_ID_QUERY_CURRENT);
    int i;

    // Init channels
#define NUMOF(x) (sizeof(x)/sizeof(x[0]))
    for (i = 0;  i < NUMOF(channels);  i++) {
        HAL_INTERRUPT_MASK(channels[i].isr_vector);
        init_duart_channel(&channels[i]);
        CYGACC_CALL_IF_SET_CONSOLE_COMM(i);
        comm = CYGACC_CALL_IF_CONSOLE_PROCS();
        CYGACC_COMM_IF_CH_DATA_SET(*comm, &channels[i]);
        CYGACC_COMM_IF_WRITE_SET(*comm, cyg_hal_plf_duart_write);
        CYGACC_COMM_IF_READ_SET(*comm, cyg_hal_plf_duart_read);
        CYGACC_COMM_IF_PUTC_SET(*comm, cyg_hal_plf_duart_putc);
        CYGACC_COMM_IF_GETC_SET(*comm, cyg_hal_plf_duart_getc);
        CYGACC_COMM_IF_CONTROL_SET(*comm, cyg_hal_plf_duart_control);
        CYGACC_COMM_IF_DBG_ISR_SET(*comm, cyg_hal_plf_duart_isr);
        CYGACC_COMM_IF_GETC_TIMEOUT_SET(*comm, cyg_hal_plf_duart_getc_timeout);
    }

    // Restore original console
    CYGACC_CALL_IF_SET_CONSOLE_COMM(cur);
}

//=============================================================================
// Compatibility with older stubs
//=============================================================================

//=============================================================================
// Compatibility with older stubs
//=============================================================================

#ifndef CYGSEM_HAL_VIRTUAL_VECTOR_DIAG

#include <cyg/hal/hal_stub.h>           // cyg_hal_gdb_interrupt

#if (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL == 0)
#define __BASE   CMA101_DUARTA
#define _INT     CYGNUM_HAL_INTERRUPT_SERIAL_A
#elif (CYGNUM_HAL_VIRTUAL_VECTOR_CONSOLE_CHANNEL == 1)
#define __BASE   CMA101_DUARTB
#define _INT     CYGNUM_HAL_INTERRUPT_SERIAL_B
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
    (uart_width*) _BASE, 0, 0
};

#ifdef HAL_DIAG_USES_HARDWARE

void hal_diag_init(void)
{
    static int init = 0;
    char *msg = "\n\rARM eCos\n\r";
    uart_width lcr;

    if (init++) return;

    init_duart_channel(&channel);

    while (*msg) hal_diag_write_char(*msg++);
}

#ifdef DEBUG_DIAG
#if defined(CYGDBG_HAL_DEBUG_GDB_INCLUDE_STUBS)
#define DIAG_BUFSIZE 32
#else
#define DIAG_BUFSIZE 2048
#endif
static char diag_buffer[DIAG_BUFSIZE];
static int diag_bp = 0;
#endif

void hal_diag_write_char(char c)
{
    uart_width lsr;

    hal_diag_init();

    cyg_hal_plf_duart_putc(&channel, c)

#ifdef DEBUG_DIAG
    diag_buffer[diag_bp++] = c;
    if (diag_bp == DIAG_BUFSIZE) {
        while (1) ;
        diag_bp = 0;
    }
#endif
}

void hal_diag_read_char(char *c)
{
    *c = cyg_hal_plf_duart_getc(&channel);
}

#else // HAL_DIAG relies on GDB

// Initialize diag port - assume GDB channel is already set up
void hal_diag_init(void)
{
    if (0) init_duart_channel(&channel); // avoid warning
}

// Actually send character down the wire
static void hal_diag_write_char_serial(char c)
{
    cyg_hal_plf_duart_putc(&channel, c);
}

static bool hal_diag_read_serial(char *c)
{
    long timeout = 1000000000;  // A long time...

    while (!cyg_hal_plf_duart_getc_nonblock(&channel, c))
        if (0 == --timeout) return false;

    return true;
}

void hal_diag_read_char(char *c)
{
    while (!hal_diag_read_serial(c)) ;
}

void hal_diag_write_char(char c)
{
    static char line[100];
    static int pos = 0;

    // No need to send CRs
    if (c == '\r') return;

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
            char c1;

            hal_diag_write_char_serial('$');
            hal_diag_write_char_serial('O');
            csum += 'O';
            for (i = 0; i < pos; i++) {
                char ch = line[i];
                char h = hex[(ch>>4)&0xF];
                char l = hex[ch&0xF];
                hal_diag_write_char_serial(h);
                hal_diag_write_char_serial(l);
                csum += h;
                csum += l;
            }
            hal_diag_write_char_serial('#');
            hal_diag_write_char_serial(hex[(csum>>4)&0xF]);
            hal_diag_write_char_serial(hex[csum&0xF]);

            // Wait for the ACK character '+' from GDB here and handle
            // receiving a ^C instead.  This is the reason for this clause
            // being a loop.
            if (!hal_diag_read_serial(&c1))
                continue;   // No response - try sending packet again

            if ( c1 == '+' )
                break;          // a good acknowledge

#ifdef CYGDBG_HAL_DEBUG_GDB_BREAK_SUPPORT
            cyg_drv_interrupt_acknowledge(CYG_DEV_SERIAL_INT);
            if ( c1 == 3 ) {
                // Ctrl-C: breakpoint.
                cyg_hal_gdb_interrupt (__builtin_return_address(0));
                break;
            }
#endif
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

#endif // CYGSEM_HAL_VIRTUAL_VECTOR_DIAG

/*---------------------------------------------------------------------------*/
