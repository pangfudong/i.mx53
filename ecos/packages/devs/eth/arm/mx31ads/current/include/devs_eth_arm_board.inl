//==========================================================================
//
//      devs_eth_arm_board.inl
//
//      Board ethernet I/O definitions.
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
//===========================================================================

#include <cyg/hal/hal_intr.h>           // CYGNUM_HAL_INTERRUPT_ETHR
#include <cyg/hal/hal_if.h>

#ifdef CYGPKG_REDBOOT
#include <pkgconf/redboot.h>
#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#include <redboot.h>
#include <flash_config.h>
#endif
#endif

#ifdef __WANT_DEVS

#ifdef CYGPKG_DEVS_ETH_ARM_MXCBOARD_ETH0

#if defined(CYGPKG_REDBOOT) && defined(CYGSEM_REDBOOT_FLASH_CONFIG)
RedBoot_config_option("Set " CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME " network hardware address [MAC]",
                      eth0_esa,
                      ALWAYS_ENABLED, true,
                      CONFIG_BOOL, false
                     );
RedBoot_config_option(CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME " network hardware address [MAC]",
                      eth0_esa_data,
                      "eth0_esa", true,
                      CONFIG_ESA, 0
                     );
#endif // CYGPKG_REDBOOT && CYGSEM_REDBOOT_FLASH_CONFIG

#ifdef CYGSEM_HAL_VIRTUAL_VECTOR_SUPPORT
// Note that this section *is* active in an application, outside RedBoot,
// where the above section is not included.

#include <cyg/hal/hal_if.h>

#ifndef CONFIG_ESA
#define CONFIG_ESA (6)
#endif
#ifndef CONFIG_BOOL
#define CONFIG_BOOL (1)
#endif

cyg_bool _board_provide_eth0_esa(struct cs8900a_priv_data* cpd)
{
    cyg_bool set_esa;
    int ok;
    ok = CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                      "eth0_esa", &set_esa, CONFIG_BOOL);
    if (ok && set_esa) {
        ok = CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                          "eth0_esa_data", cpd->esa, CONFIG_ESA);
    }

    return ok && set_esa;
}

#endif // CYGSEM_HAL_VIRTUAL_VECTOR_SUPPORT


// ------------------------------------------------------------------------
// EEPROM access functions
// 
#define PP_ECR                  0x0040
#define PP_EE_READ_CMD          0x0200
#define PP_EE_WRITE_CMD         0x0100
#define PP_EE_EWEN_CMD          0x00F0
#define PP_EE_EWDS_CMD          0x0000
#define PP_EE_ERASE_CMD         0x0300

#define PP_EE_DATA              0x0042
#define PP_EE_ADDR_W0           0x001C
#define PP_EE_ADDR_W1           0x001D
#define PP_EE_ADDR_W2           0x001E

#define EE_TIMEOUT              50000
__inline__ cyg_uint16 read_eeprom(cyg_addrword_t base, cyg_uint16 offset)
{
    unsigned long timeout = EE_TIMEOUT;
    if (get_reg(base, PP_SelfStat) & PP_SelfStat_EEPROM) {
#if 0
        diag_printf("EEPROM PP_SelfStat=0x%x\n", get_reg(base, PP_SelfStat));
#endif
    } else {
        diag_printf("Error: NO EEPROM present\n");
        return 0;
    }

    while ((timeout -- > 0) && (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY))
        ;
    if (timeout == 0) {
        diag_printf("read_eeprom() timeout\n");
        return 0;
    }
    timeout = EE_TIMEOUT;
    put_reg(base, PP_ECR, (offset | PP_EE_READ_CMD));
    while ((timeout -- > 0) && (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY))
        ;
    if (timeout == 0) {
        diag_printf("read_eeprom() timeout\n");
        return 0;
    }
    return get_reg(base, PP_EE_DATA);
}

/*
 * Write a word to an EEPROM location
 * base: package page base (IO base)
 * offset: the EEPROM word offset starting from 0. So for word 1, should pass in 1
 * data: 16 bit data to be written into EEPRM
 */
__inline__ void write_eeprom(cyg_addrword_t base, cyg_uint16 offset, cyg_uint16 data)
{
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
    put_reg(base, PP_ECR, PP_EE_EWEN_CMD);
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
    put_reg(base, PP_ECR, PP_EE_ERASE_CMD|offset);
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
    put_reg(base, PP_EE_DATA, data);
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
    put_reg(base, PP_ECR, (PP_EE_WRITE_CMD|offset));
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
    put_reg(base, PP_ECR, PP_EE_EWDS_CMD);
    while (get_reg(base, PP_SelfStat) & PP_SelfStat_SIBSY)
        ;
}

#define CS8900A_RESET_BYPASS /* define it when reset is done early */

static __inline__ void copy_eeprom(cyg_addrword_t base)
{
    cyg_uint16 esa_word;     
    int i;
    for (i = 0;  i < 6;  i += 2) {
        esa_word = read_eeprom(base, PP_EE_ADDR_W0 + (i/2));
        put_reg(base, (PP_IA+i), esa_word);
//         diag_printf("base=0x%x, copy_eeprom (0x%04x)\n", base, esa_word);
    }
}

#undef  CYGHWR_CL_CS8900A_PLF_RESET
#define CYGHWR_CL_CS8900A_PLF_RESET(base) copy_eeprom(base)

static cs8900a_priv_data_t cs8900a_eth0_priv_data = {
    base : (cyg_addrword_t) BOARD_CS_LAN_BASE,
    interrupt: CYGNUM_HAL_INTERRUPT_ETH,
#ifdef CYGSEM_DEVS_ETH_ARM_MXCBOARD_ETH0_SET_ESA
    esa : CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_ESA,
    hardwired_esa : true,
#else
    hardwired_esa : false,
#endif
#ifdef CYGSEM_HAL_VIRTUAL_VECTOR_SUPPORT
    provide_esa : &_board_provide_eth0_esa,
#else
    provide_esa : NULL,
#endif
};

ETH_DRV_SC(cs8900a_sc,
           &cs8900a_eth0_priv_data, // Driver specific data
           CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME,
           cs8900a_start,
           cs8900a_stop,
           cs8900a_control,
           cs8900a_can_send,
           cs8900a_send,
           cs8900a_recv,
           cs8900a_deliver,     // "pseudoDSR" called from fast net thread
           cs8900a_poll,        // poll function, encapsulates ISR and DSR
           cs8900a_int_vector);

NETDEVTAB_ENTRY(cs8900a_netdev,
                "cs8900a_" CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME,
                cs8900a_init,
                &cs8900a_sc);

#endif // CYGPKG_DEVS_ETH_ARM_MXCBOARD_ETH0

#endif // __WANT_DEVS
