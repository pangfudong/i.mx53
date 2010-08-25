#ifndef CYGONCE_DEVS_ETH_SMSC_LAN92XX_LAN92XX_H
#define CYGONCE_DEVS_ETH_SMSC_LAN92XX_LAN92XX_H
//==========================================================================
//
//      lan92xx.h
//
//      SMCS LAN9217 (LAN92XX compatible) Ethernet chip
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
// Copyright (C) 2003 Nick Garnett 
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
//####BSDCOPYRIGHTBEGIN####
//
// -------------------------------------------
//
// Portions of this software may have been derived from OpenBSD or other sources,
// and are covered by the appropriate copyright disclaimers included herein.
//
// -------------------------------------------
//
//####BSDCOPYRIGHTEND####

#include <cyg/hal/hal_io.h>
#include <cyg/hal/hal_endian.h>

#define __WANT_CONFIG
#include CYGDAT_DEVS_ETH_SMSC_LAN92XX_CFG
#undef __WANT_CONFIG

typedef struct
{
    unsigned short id;
    unsigned short ver;
    char *id_name;
}smsc_lan92xx_id_t;

// LAN92xx register offset
#define LAN92XX_RX_DATA         0x00
#define LAN92XX_TX_DATA         0x20
#define LAN92XX_RX_STATUS1      0x40
#define LAN92XX_RX_STATUS2      0x44
#define LAN92XX_TX_STATUS1      0x48
#define LAN92XX_TX_STATUS2      0x4C
#define LAN92XX_ID_REV          0x50
#define LAN92XX_IRQ_CFG         0x54
#define LAN92XX_INT_STS         0x58
#define LAN92XX_INT_EN          0x5C
#define LAN92XX_RESERVED1       0x60
#define LAN92XX_BYTE_TEST       0x64
#define LAN92XX_FIFO_INT        0x68
#define LAN92XX_RX_CFG          0x6C
#define LAN92XX_TX_CFG          0x70
#define LAN92XX_HW_CFG          0x74
#define LAN92XX_RX_DP_CTRL      0x78
#define LAN92XX_RX_FIFO_INF     0x7C
#define LAN92XX_TX_FIFO_INF     0x80
#define LAN92XX_PMT_CTRL        0x84
#define LAN92XX_GPIO_CFG        0x88
#define LAN92XX_GPT_CFG         0x8C
#define LAN92XX_GPT_CNT         0x90
#define LAN92XX_RESERVED2       0x94
#define LAN92XX_WORD_SWAP       0x98
#define LAN92XX_FREE_RUN        0x9C
#define LAN92XX_RX_DROP         0xA0
#define LAN92XX_MAC_CMD         0xA4
#define LAN92XX_MAC_DATA        0xA8
#define LAN92XX_AFC_CFG         0xAC
#define LAN92XX_E2P_CMD         0xB0
#define LAN92XX_E2P_DATA        0xB4

// Access these MAC registers indirectly through MAC_CMD and MAC_DATA
// registers. 
#define MAC_MAC_CR 	    1
#define MAC_ADDRH 	    2
#define MAC_ADDRL 	    3
#define MAC_HASHH 	    4
#define MAC_HASHL 	    5
#define MAC_MII_ACC     6
#define MAC_MII_DATA    7
#define MAC_FLOW        8
#define MAC_VLAN1       9
#define MAC_VLAN2       10
#define MAC_WUFF        11
#define MAC_WUCSR       12

// These PHY registers are accessed indirectly through the MAC via the 
// MII interface using the MII_ACC and MII_DATA registers. PHY controls
// the 802.3 physical layer such as 10/100Mbps, full/half mode.
#define PHY_BCR         0
#define PHY_BSR         1
#define PHY_ID1         2
#define PHY_ID2         3
#define PHY_ANAR        4
#define PHY_ANLPAR      5
#define PHY_ANER        6
#define PHY_MCSR        17
#define PHY_SMR         18
#define PHY_SCSI        27
#define PHY_ISR         29
#define PHY_IMR         30
#define PHY_SCSR        31

#define PHY_100TX_FD    0x4000
#define PHY_100TX_HD    0x2000
#define PHY_10T_RD      0x1000
#define PHY_10T_HD      0x0800
#define PHY_LINK_ON     0x0004

#define IS_DUPLEX(x)	((x) & (PHY_100TX_FD | PHY_10T_RD))

#define MAC_TIMEOUT     (1000 * 100)
#define MAC_TICKET      2

#define E2P_CMD_SHIFT	28
#define E2P_CMD_BUSY	0x80000000
#define E2P_CMD_TIMEOUT	0x00000200
#define E2P_CMD_LOADED	0x00000100

enum epc_cmd {
    E2P_CMD_READ    = 0 << E2P_CMD_SHIFT,
    E2P_CMD_EWDS    = 1 << E2P_CMD_SHIFT,
    E2P_CMD_EWEN    = 2 << E2P_CMD_SHIFT,
    E2P_CMD_WRITE   = 3 << E2P_CMD_SHIFT,
    E2P_CMD_WRAL    = 4 << E2P_CMD_SHIFT,
    E2P_CMD_ERASE   = 5 << E2P_CMD_SHIFT,
    E2P_CMD_ERAL    = 6 << E2P_CMD_SHIFT,
    E2P_CMD_Reload  = 7 << E2P_CMD_SHIFT,
};
#define E2P_CMD(cmd, addr) (E2P_CMD_BUSY | (cmd) | (addr))

#define E2P_CONTEXT_ID  0xA5

typedef struct
{
    unsigned int base;
    int status;
    int tx_busy;
    int tx_key;
    unsigned char mac_addr[6];
} smsc_lan92xx_t;

#ifndef LAN92XX_REG_BASE
#define LAN92XX_REG_BASE    PBC_BASE
#endif

#ifndef LAN92XX_REG_READ
#define LAN92XX_REG_READ(reg_offset)  \
    (*(volatile unsigned int *)(LAN92XX_REG_BASE + reg_offset))
#endif    

#ifndef LAN92XX_REG_WRITE
#define LAN92XX_REG_WRITE(reg_offset, val)  \
    (*(volatile unsigned int *)(LAN92XX_REG_BASE + reg_offset) = (val))
#endif

#endif // CYGONCE_DEVS_ETH_SMSC_MAC_MAC_H
