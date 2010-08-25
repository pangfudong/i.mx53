//==========================================================================
//
//      dev/if_fec.c
//
//      Device driver for FEC 
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    Fred Fan
// Contributors: 
// Date:         2006-08-23
// Purpose:      
// Description:  Driver for FEC ethernet controller
//
// Note:         
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <pkgconf/system.h>
#ifdef CYGPKG_KERNEL
#include <cyg/kernel/kapi.h>
#endif
#include <pkgconf/io_eth_drivers.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_endian.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/drv_api.h>
#include <cyg/hal/hal_soc.h>
#undef __ECOS
#define __ECOS
#include <cyg/io/eth/eth_drv.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/io/fec.h>
#include <redboot.h>

#include <cyg/hal/hal_mm.h>
#include <cyg/hal/plf_mmap.h>
#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
#include <flash_config.h>
#endif

/*!
 * Global variable which contains the name of FEC driver and device. 
 */
static char  mxc_fec_name[] = "mxc_fec";

/*!
 * Global variable which defines the private structure of FEC device.
 */
static mxc_fec_priv_t  mxc_fec_private ;

/*!
 *Global variable which defines the buffer descriptions for receiving frame
 * 	comment:: it must aligned by 128-bits.
 */
static mxc_fec_bd_t mxc_fec_rx_bd[FEC_BD_RX_NUM] __attribute__ ( ( aligned(32) ) ) ;

/*!
 *Global variable which defines the buffer descriptions for receiving frame
 * 	comment:: it must aligned by 128-bits.
 */
static mxc_fec_bd_t mxc_fec_tx_bd[FEC_BD_TX_NUM] __attribute__ ( ( aligned(32) ) ) ;

/*!
 * Global variable which contains the frame buffers , 
 */
static unsigned char mxc_fec_rx_buf[FEC_BD_RX_NUM][2048] __attribute__ ( ( aligned(32) ) ) ;

/*!
 * Global variable which contains the frame buffers , 
 */
static unsigned char mxc_fec_tx_buf[FEC_BD_TX_NUM][2048] __attribute__ ( ( aligned(32) ) ) ;


/*!
 * This function get the value of  PHY registers by MII interface
 */
static int 
mxc_fec_mii_read(volatile mxc_fec_reg_t * hw_reg, unsigned char phy_addr, unsigned char reg_addr, unsigned short int * value)
{
	unsigned long waiting = FEC_MII_TIMEOUT;
	
	if(hw_reg->eir & FEC_EVENT_MII ) {
		hw_reg->eir = FEC_EVENT_MII ;
	} 
	hw_reg->mmfr = FEC_MII_READ(phy_addr, reg_addr);/*Write CMD*/
	while(1) {
		if(hw_reg->eir & FEC_EVENT_MII ) {
			hw_reg->eir = FEC_EVENT_MII ; 
			break;
		}
		if ( (--waiting) == 0 ) return -1;
		hal_delay_us(FEC_MII_TICK);	
	}
	*value = FEC_MII_GET_DATA(hw_reg->mmfr);
	return 0;
}

/*!
 * This function set the value of  PHY registers by MII interface
 */
static int 
mxc_fec_mii_write(volatile mxc_fec_reg_t * hw_reg, unsigned char phy_addr, unsigned char reg_addr, unsigned short int value)
{
	unsigned long waiting = FEC_MII_TIMEOUT;
	
	if(hw_reg->eir & FEC_EVENT_MII ) {
		hw_reg->eir = FEC_EVENT_MII ;
	} 
	hw_reg->mmfr = FEC_MII_WRITE(phy_addr, reg_addr, value);/*Write CMD*/
	while(1) {
		if(hw_reg->eir & FEC_EVENT_MII ) {
			hw_reg->eir = FEC_EVENT_MII ; 
			break;
		}
		if ( (--waiting) == 0 ) return -1;
		hal_delay_us(FEC_MII_TICK);	
	}
	return 0;
}

static void
mxc_fec_set_mac_address(volatile mxc_fec_reg_t * dev, unsigned char * enaddr)
{
	unsigned long value;
	
	value = enaddr[0];
	value = (value << 8) + enaddr[1];
	value = (value << 8) + enaddr[2];
	value = (value << 8) + enaddr[3];
	dev->palr = value;
	
	value = enaddr[4];
	value = (value<<8)+enaddr[5];
	dev->paur = (value<<16);
}

/*!
 * This function set the value of  PHY registers by MII interface
 */
static void 
mxc_fec_start(struct eth_drv_sc *sc, unsigned char *enaddr, int flags)
{
	mxc_fec_priv_t * priv = sc?sc->driver_private:NULL;
	volatile mxc_fec_reg_t * chip = priv?priv->hw_reg:NULL;

	if ( !(priv && chip) || enaddr == NULL ) {
		diag_printf("BUG[start]: MAC address or some fields in driver is NULL\n");
		return;
	}
	mxc_fec_set_mac_address(chip, enaddr);

	priv->tx_busy = 0;
	chip->ecr |= FEC_ETHER_EN;
	chip->rdar |= FEC_RX_TX_ACTIVE;
}

/*!
 * This function pauses the FEC controller.
 */
static void 
mxc_fec_stop(struct eth_drv_sc *sc)
{
	mxc_fec_priv_t * priv = sc?sc->driver_private:NULL;
	volatile mxc_fec_reg_t * chip = priv?priv->hw_reg:NULL;
	if ( !(priv && chip)  ) {
		diag_printf("BUG[stop]: some fields in driver is NULL\n");
		return;
	}
	chip->ecr &= ~FEC_ETHER_EN;
}

static int  
mxc_fec_control(struct eth_drv_sc *sc, unsigned long key, void *data, int data_length)
{
	/*TODO:: Add support */
	diag_printf("mxc_fec_control: key=0x%x, data=0x%x, data_len=0x%x\n",
		key, data, data_length);
	return 0;
}

/*!
 * This function checks the status of FEC control.
 */
static int  
mxc_fec_can_send(struct eth_drv_sc *sc)
{
	mxc_fec_priv_t * priv = sc?sc->driver_private:NULL;
	volatile mxc_fec_reg_t * hw_reg = priv?priv->hw_reg:NULL;
	unsigned long value;

	if ( !( priv && hw_reg) ) {
		diag_printf("BUG[can_send]:the private pointer and register pointer in MXC_FEC is NULL\n");
		return 0;
	}
	if ( priv->tx_busy ) {
		diag_printf("WARNING[can_send]: MXC_FEC is busy for transmittinig\n");
		return 0;
	}

	if(!(hw_reg->ecr & FEC_ETHER_EN)) {
		diag_printf("WARNING[can_send]: MXC_FEC is not enabled\n");
		return 0;
	}

	if(hw_reg->tcr & FEC_TCR_RFC_PAUSE) {
		diag_printf("WARNING[can_send]: MXC_FEC is paused\n");
		return 0;
	}

	mxc_fec_mii_read(hw_reg, priv->phy_addr, 1, &value);
	if ( value & PHY_STATUS_LINK_ST) {
                priv->status |= FEC_STATUS_LINK_ON;
        } else {
                priv->status &= ~FEC_STATUS_LINK_ON;
        }
	return (priv->status&FEC_STATUS_LINK_ON);
}

/*!
 * This function transmits a frame.
 */
static void 
mxc_fec_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len, int total, unsigned long key)
{
	mxc_fec_priv_t * dev = sc?sc->driver_private:NULL;
	volatile mxc_fec_reg_t * hw_reg = dev?dev->hw_reg:NULL;
	mxc_fec_bd_t * p;
	int i, off;

	if ( dev == NULL || hw_reg == NULL) {
		diag_printf("BUG[TX]: some fields in driver are NULL\n");
		return;
	}
	if ( total > (FEC_FRAME_LEN-4)) total = FEC_FRAME_LEN-4;
	if ( sg_list == NULL || total <= 14 ) {
		if(sc->funs->eth_drv && sc->funs->eth_drv->tx_done) {
			sc->funs->eth_drv->tx_done(sc, key, -1);
		}
		return;
	}	

	for(i=0, off=0, p = dev->tx_cur; i<sg_len; i++) {
		if(p->status & BD_TX_ST_RDY) {
			diag_printf("BUG[TX]:MXC_FEC's status=%x\n", p->status);
			break;
		}
		if (sg_list[i].buf == NULL ) {
			diag_printf("WARNING[TX]: sg_list->buf is NULL\n");
			break;
		}
		memcpy(hal_ioremap_nocache(p->data)+off, sg_list[i].buf, sg_list[i].len );
		off += sg_list[i].len;
	}
	if ( off < 14 ) {
		diag_printf("WARNING[TX]: data len is %d\n", off);
		return;
	}
	p->length = off; 
	p->status &= ~(BD_TX_ST_LAST|BD_TX_ST_RDY|BD_TX_ST_TC|BD_TX_ST_ABC);
	p->status |= BD_TX_ST_LAST| BD_TX_ST_RDY | BD_TX_ST_TC;
	if(p->status & BD_TX_ST_WRAP ) {
		p = dev->tx_bd;
	} else p++;
	dev->tx_cur = p;
	dev->tx_busy = 1;
	dev->tx_key = key;
	hw_reg->tdar = FEC_RX_TX_ACTIVE;	
}

/*!
 * This function receives ready Frame in DB.
 */
static void 
mxc_fec_recv(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len)
{
        mxc_fec_priv_t * priv = sc?sc->driver_private:NULL;
	mxc_fec_bd_t * p;

	if(sg_list == NULL || priv == NULL || sg_len <= 0) {
		diag_printf("BUG[RX]: driver's private field or argument of this calling is NULL \n");
		return;
	}
	
	/*TODO: I think if buf pointer is NULL, this function 
	 * should not be called
	 */
	if(sg_list->buf == NULL) {
		diag_printf("WARING[RX]: the sg_list is empty\n");
		return;
	}
	p = priv->rx_cur;
	
	if(p->status & BD_RX_ST_EMPTY) {
		diag_printf("BUG[RX]: status =%x\n", p->status);
		return;
	}

	if(!(p->status & BD_RX_ST_LAST)) {
		diag_printf("BUG[RX]: status =%x\n", p->status);
		return;
	} 
	/*TODO::D_CACHE invalid this data buffer*/
	memcpy(sg_list->buf, hal_ioremap_nocache(p->data), p->length -4);
}

static void 
mxc_fec_deliver(struct eth_drv_sc *sc)
{
	/*TODO::When redboot support thread , 
	 *	the polling function will be called at here
	 */
	return;
}

static void 
mxc_fec_check_rx_bd(struct eth_drv_sc * sc)
{
	/* This funtion just called by polling funtion*/
        mxc_fec_priv_t * priv = sc->driver_private;
	mxc_fec_bd_t * p, * prev;
	volatile mxc_fec_reg_t * hw_reg = priv->hw_reg;
	int i;
	
	for(i = 0, p = priv->rx_cur; i< FEC_RX_FRAMES; i++){
		/*TODO::D-CACHE invalid this BD.
	 	*In WRITE_BACK mode: this maybe destroy the next BD 
		*	when the CACHE_LINE write back.
	 	*/
		if(p->status & BD_RX_ST_EMPTY) {
			break;
		}
		if(!(p->status & BD_RX_ST_LAST)) {
			diag_printf("BUG[RX]: status=%x, length=%x\n", p->status, p->length);
			goto skip_next;
		}
		
		if((p->status & BD_RX_ST_ERRS)|| (p->length > FEC_FRAME_LEN)) {
			diag_printf("BUG[RX]: status=%x, length=%x\n", p->status, p->length);
		} else {
			sc->funs->eth_drv->recv(sc, p->length -4);
		}
skip_next:
		p->status = (p->status & BD_RX_ST_WRAP) | BD_RX_ST_EMPTY;
		
		if ( p->status & BD_RX_ST_WRAP) {
			p = priv->rx_bd;
		} else {
			p++;
		} 
		priv->rx_cur = p;	
		hw_reg->ecr |= FEC_ETHER_EN;
		hw_reg->rdar |= FEC_RX_TX_ACTIVE;
	}
}

/*!
 * This function checks the event of FEC controller
 */
static void 
mxc_fec_poll(struct eth_drv_sc * sc)
{
	mxc_fec_priv_t * priv = sc?sc->driver_private:NULL;
        volatile mxc_fec_reg_t * hw_reg = priv?priv->hw_reg:NULL;
        unsigned long value;

	if ( priv == NULL || hw_reg == NULL) {
		diag_printf("BUG[POLL]: some fields in driver are NULL\n");
		return;
	}
	value = hw_reg->eir;
	hw_reg->eir = value&(~FEC_EVENT_MII);
	
	if(value&FEC_EVENT_TX_ERR) {
		diag_printf("WARNING[POLL]: There are error(%x) for transmit\n", value&FEC_EVENT_TX_ERR);
		sc->funs->eth_drv->tx_done(sc, priv->tx_key, -1);
		priv->tx_busy = 0;
	} else {
		if(value&FEC_EVENT_TX) {
			sc->funs->eth_drv->tx_done(sc,  priv->tx_key, 0);
			priv->tx_busy = 0;
		}
	}
	
	if(value&FEC_EVENT_RX) {
		mxc_fec_check_rx_bd(sc);
	}

	if(value & FEC_EVENT_HBERR) {
		diag_printf("WARNGING[POLL]: Hearbeat error!\n");
	}

	if(value & FEC_EVENT_EBERR) {
		diag_printf("WARNING[POLL]: Ethernet Bus Error!\n");
	}
}


static int
mxc_fec_int_vector(struct eth_drv_sc *sc)
{
	/*TODO::
	 *	get FEC interrupt number 	
 	 */
    	return -1;
}

/*!
 * The function initializes the description buffer for receiving or transmitting
 */
static void
mxc_fec_bd_init(mxc_fec_priv_t * dev)
{
	int i;
	mxc_fec_bd_t * p;
	
	p = dev->rx_bd = hal_ioremap_nocache(hal_virt_to_phy((unsigned long)mxc_fec_rx_bd));
	for(i=0; i<FEC_BD_RX_NUM; i++, p++){
		p->status = BD_RX_ST_EMPTY;
		p->length = 0;
		p->data = hal_virt_to_phy((unsigned long)mxc_fec_rx_buf[i]);
	}

	dev->rx_bd[i-1].status |= BD_RX_ST_WRAP;
	dev->rx_cur = dev->rx_bd;

        p = dev->tx_bd = hal_ioremap_nocache(hal_virt_to_phy((unsigned long)mxc_fec_tx_bd));
        for(i=0; i<FEC_BD_TX_NUM; i++, p++){
                p->status = 0;
                p->length = 0;
                p->data = hal_virt_to_phy((unsigned long)mxc_fec_tx_buf[i]);
        }

        dev->tx_bd[i-1].status |= BD_TX_ST_WRAP;
	dev->tx_cur = dev->tx_bd;
	
	/*TODO:: add the sync function for items*/
}

/*!
 *This function initializes FEC controller. 
 */
static void 
mxc_fec_chip_init(mxc_fec_priv_t * dev)
{
	volatile mxc_fec_reg_t * chip = dev->hw_reg;
	unsigned long ipg_clk;

	chip->ecr = FEC_RESET;
	while(chip->ecr & FEC_RESET) {
		hal_delay_us(FEC_COMMON_TICK);
	}

	chip->eimr = 0x00000000;
	chip->eir = 0xFFFFFFFF;
	
	chip->rcr = (chip->rcr&~(0x0000003F))|FEC_RCR_FCE|FEC_RCR_MII_MODE;
	chip->tcr |= FEC_TCR_FDEN;
	chip->mibc |= FEC_MIB_DISABLE;
	
	chip->iaur = 0;
	chip->ialr = 0;
	chip->gaur = 0;
	chip->galr = 0;

	/*TODO:: Use MII_SPEED(IPG_CLK) to get the value*/ 
	ipg_clk = get_main_clock(IPG_CLK);
	
	chip->mscr = (chip->mscr&(~0x7e)) | (((ipg_clk+499999)/2500000/2)<<1);
	
	/*Enable ETHER_EN*/
	chip->emrbr = 2048-16;
	chip->erdsr = hal_virt_to_phy((unsigned long)dev->rx_bd);
	chip->etdsr = hal_virt_to_phy((unsigned long)dev->tx_bd);
}

/*!
 * This function initialize PHY
 */
static bool 
mxc_fec_phy_init(mxc_fec_priv_t * dev)
{
	unsigned long value = 0;
	unsigned long id = 0;

	mxc_fec_mii_read(dev->hw_reg, dev->phy_addr, PHY_IDENTIFY_1, &value);
	id = (value&PHY_ID1_MASK)<<PHY_ID1_SHIFT;
	mxc_fec_mii_read(dev->hw_reg, dev->phy_addr, PHY_IDENTIFY_2, &value);
	id |= (value&PHY_ID2_MASK)<<PHY_ID2_SHIFT;
	switch( id) {
	case 0x00540088:
		break;
	case 0x00007C0C:
		break;
	default:
		diag_printf("[Warning] FEC not connect right PHY: ID=%lx\n", id);
        }
	
	mxc_fec_mii_write(dev->hw_reg, dev->phy_addr, PHY_CTRL_REG, PHY_CTRL_AUTO_NEG|PHY_CTRL_FULL_DPLX);
		
	mxc_fec_mii_read(dev->hw_reg, dev->phy_addr, PHY_MODE_REG, &value);
	value &= ~(PHY_LED_SEL);
	mxc_fec_mii_write(dev->hw_reg, dev->phy_addr, PHY_MODE_REG, value);
	
	mxc_fec_mii_read(dev->hw_reg, dev->phy_addr, PHY_STATUS_REG, &value);
	
	if ( value & PHY_STATUS_LINK_ST) {
		dev->status |= FEC_STATUS_LINK_ON;
	} else {
		dev->status &= ~FEC_STATUS_LINK_ON;
	}
		
	mxc_fec_mii_read(dev->hw_reg, dev->phy_addr, PHY_DIAG_REG, &value);
	if ( value & PHY_DIAG_DPLX) {
		dev->status |= FEC_STATUS_FULL_DPLX;
	} else {
		dev->status &= ~FEC_STATUS_FULL_DPLX;
	}
	if ( value & PHY_DIAG_DPLX) {
                dev->status |= FEC_STATUS_100M;
        } else {
                dev->status &= ~FEC_STATUS_100M;
        }

	diag_printf("FEC: [ %s ] [ %s ] [ %s ]:\n", 
		(dev->status&FEC_STATUS_FULL_DPLX)?"FULL_DUPLEX":"HALF_DUPLEX",
		(dev->status&FEC_STATUS_LINK_ON)?"connected":"disconnected",
		(dev->status&FEC_STATUS_100M)?"100M bps":"10M bps");
	return true;
}

/*! This function initializes the FEC driver. 
 * It is called by net_init in net module of RedBoot during RedBoot init
 */
static bool 
mxc_fec_init(struct cyg_netdevtab_entry *tab)
{
	struct eth_drv_sc * sc = tab?tab->device_instance:NULL;
	mxc_fec_priv_t * private;
    	char eth_add_local[6] = {0x00, 0x00, 0x45, 0x67, 0x89, 0xab};
#ifdef CYGSEM_REDBOOT_FLASH_CONFIG
	cyg_bool set_esa;
    	int ok;

	/* Get MAC address */
    	ok = CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                      "fec_esa", &set_esa, CONFIG_BOOL);
    	if (ok && set_esa) {
        	CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                     "fec_esa_data", eth_add_local, CONFIG_ESA);
	}
#endif

    	if(sc == NULL ){
		diag_printf("FEC:: no driver attached\n");
		return false;
    	} 
	
	private = MXC_FEC_PRIVATE(sc);
	if ( private == NULL ) {
		private = MXC_FEC_PRIVATE(sc) = &mxc_fec_private;
	}

	private->hw_reg = SOC_FEC_BASE;
	private->tx_busy = 0;
	private->status = 0;
	private->phy_addr = PHY_PORT_ADDR;

	mxc_fec_bd_init(private);

	mxc_fec_chip_init(private);

	mxc_fec_phy_init(private);
	
	/*TODO:: initialize System Resource : irq, timer */

	sc->funs->eth_drv->init(sc, eth_add_local);
    
    	return true;
}

/*!
 * Global variable which defines the FEC driver, 
 */
ETH_DRV_SC(mxc_fec_sc,
 	   &mxc_fec_private, // Driver specific data
           mxc_fec_name,
           mxc_fec_start,
           mxc_fec_stop,
           mxc_fec_control,
           mxc_fec_can_send,
           mxc_fec_send,
           mxc_fec_recv,
           mxc_fec_deliver,     // "pseudoDSR" called from fast net thread
           mxc_fec_poll,        // poll function, encapsulates ISR and DSR
           mxc_fec_int_vector);

/*!
 * Global variable which defines the FEC device
 */
NETDEVTAB_ENTRY(mxc_fec_netdev,
                mxc_fec_name,
                mxc_fec_init,
                &mxc_fec_sc);

#if defined(CYGPKG_REDBOOT) && defined(CYGSEM_REDBOOT_FLASH_CONFIG)
extern unsigned int sys_ver;

void _board_provide_fec_esa(void)
{
    cyg_bool set_esa;
    cyg_uint8 addr[6];
    int ok;

    ok = CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                      "fec_esa", &set_esa, CONFIG_BOOL);
    diag_printf("Ethernet FEC MAC address: ");
    if (ok && set_esa) {
        CYGACC_CALL_IF_FLASH_CFG_OP( CYGNUM_CALL_IF_FLASH_CFG_GET,
                                     "fec_esa_data", addr, CONFIG_ESA);
#ifdef CYGPKG_HAL_ARM_MX27
        if(sys_ver == SOC_SILICONID_Rev1_0) {
                writel(addr[5], SOC_FEC_MAC_BASE + 0x0);
                writel(addr[4], SOC_FEC_MAC_BASE + 0x4);
                writel(addr[3], SOC_FEC_MAC_BASE + 0x8);
                writel(addr[2], SOC_FEC_MAC_BASE + 0xC);
                writel(addr[1], SOC_FEC_MAC_BASE + 0x10);
                writel(addr[0], SOC_FEC_MAC_BASE + 0x14);
                addr[5] = readl(SOC_FEC_MAC_BASE + 0x0);
                addr[4] = readl(SOC_FEC_MAC_BASE + 0x4);
                addr[3] = readl(SOC_FEC_MAC_BASE + 0x8);
                addr[2] = readl(SOC_FEC_MAC_BASE + 0xC);
                addr[1] = readl(SOC_FEC_MAC_BASE + 0x10);
                addr[0] = readl(SOC_FEC_MAC_BASE + 0x14);
        } else {
                writel(addr[5], SOC_FEC_MAC_BASE2 + 0x0);
                writel(addr[4], SOC_FEC_MAC_BASE2 + 0x4);
                writel(addr[3], SOC_FEC_MAC_BASE2 + 0x8);
                writel(addr[2], SOC_FEC_MAC_BASE2 + 0xC);
                writel(addr[1], SOC_FEC_MAC_BASE2 + 0x10);
                writel(addr[0], SOC_FEC_MAC_BASE2 + 0x14);
                addr[5] = readl(SOC_FEC_MAC_BASE2 + 0x0);
                addr[4] = readl(SOC_FEC_MAC_BASE2 + 0x4);
                addr[3] = readl(SOC_FEC_MAC_BASE2 + 0x8);
                addr[2] = readl(SOC_FEC_MAC_BASE2 + 0xC);
                addr[1] = readl(SOC_FEC_MAC_BASE2 + 0x10);
                addr[0] = readl(SOC_FEC_MAC_BASE2 + 0x14);
        }
        diag_printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
#endif
    } else {
        diag_printf("is not set\n");
    }
}

RedBoot_init(_board_provide_fec_esa, RedBoot_INIT_LAST);

RedBoot_config_option("Set FEC network hardware address [MAC]",
                      fec_esa,
                      ALWAYS_ENABLED, true,
                      CONFIG_BOOL, false
                     );
RedBoot_config_option("FEC network hardware address [MAC]",
                      fec_esa_data,
                      "fec_esa", true,
                      CONFIG_ESA, 0
                     );
#endif // CYGPKG_REDBOOT && CYGSEM_REDBOOT_FLASH_CONFIG
