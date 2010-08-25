#ifndef CYGONCE_FSL_BOARD_H
#define CYGONCE_FSL_BOARD_H

//=============================================================================
//
//      Platform specific support (register layout, etc)
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

#include <cyg/hal/hal_soc.h>        // Hardware definitions

#define PMIC_SPI_BASE            CSPI2_BASE_ADDR
#define PMIC_SPI_CHIP_SELECT_NO  SPI_CTRL_CS0

#define PBC_BASE                    CS4_BASE_ADDR    /* Peripheral Bus Controller */
#define PBC_VERSION                 0x0
#define PBC_BSTAT2                  0x2
#define PBC_BCTRL1                  0x4
#define PBC_BCTRL1_CLR              0x6
#define PBC_BCTRL2                  0x8
#define PBC_BCTRL2_CLR              0xA
#define PBC_BCTRL3                  0xC
#define PBC_BCTRL3_CLR              0xE
#define PBC_BCTRL4                  0x10
#define PBC_BCTRL4_CLR              0x12
#define PBC_BSTAT1                  0x14
#define BOARD_CS_LAN_BASE           (CS4_BASE_ADDR + 0x00020000 + 0x300)
#define BOARD_CS_UART_BASE          (CS4_BASE_ADDR + 0x00010000)

#define BOARD_FLASH_START	        CS0_BASE_ADDR
#define REDBOOT_IMAGE_SIZE          0x40000

/* MX31 ADS SDRAM is from 0x80000000, 128M */
#define SDRAM_BASE_ADDR             CSD0_BASE_ADDR
#define SDRAM_SIZE                  0x08000000
#define RAM_BANK0_BASE              SDRAM_BASE_ADDR

#define LED_MAX_NUM	2
#define LED_IS_ON(n)    (readw(PBC_BASE+PBC_BCTRL1_CLR) & (1 << (n+6)))
#define TURN_LED_ON(n)  writew((readw(PBC_BASE+PBC_BCTRL1_CLR) | (1 << (n+6))), PBC_BASE+PBC_BCTRL1)
#define TURN_LED_OFF(n) writew((1<<(n+6)), PBC_BASE+PBC_BCTRL1_CLR)


#define BOARD_DEBUG_LED(n) 			\
    CYG_MACRO_START				\
        if (n >= 0 && n < LED_MAX_NUM) { 	\
		if (LED_IS_ON(n)) 		\
			TURN_LED_OFF(n); 	\
		else 				\
			TURN_LED_ON(n);		\
	}					\
    CYG_MACRO_END

#define BOARD_PBC_VERSION       ((*(volatile unsigned short*)(PBC_BASE + PBC_VERSION)) >> 8)

#define DEBUG_SWITCH_1          (1 << 7)
#define DEBUG_SWITCH_2          (1 << 6)
#define DEBUG_SWITCH_3          (1 << 5)
#define DEBUG_SWITCH_4          (1 << 4)
#define DEBUG_SWITCH_5          (1 << 3)
#define DEBUG_SWITCH_6          (1 << 2)
#define DEBUG_SWITCH_7          (1 << 1)
#define DEBUG_SWITCH_8          (1 << 0)
#define CLK_INPUT_27MHZ_SET     DEBUG_SWITCH_4

#define DEBUG_SWITCH_IS_ON(n)   (((*(volatile unsigned short*)(PBC_BASE + PBC_BSTAT2)) & n) == 0)
#if 0
    while (DEBUG_SWITCH_IS_ON(DEBUG_SWITCH_1)) {
        hal_delay_us(100);
    }
#endif
#endif /* CYGONCE_FSL_BOARD_H */
