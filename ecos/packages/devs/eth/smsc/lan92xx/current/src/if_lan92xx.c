//==========================================================================
//
//      dev/if_lan92xx.c
//
//      Ethernet device driver for SMSC LAN92XX compatible controllers
//
//==========================================================================
//==========================================================================

//#####DESCRIPTIONBEGIN####
//
// Author(s):    Fred Fan
// Contributors:
// Date:         2007-10-16
// Purpose:
// Description:  Driver for SMSC LAN92xx ethernet controller
//
// Note:
//
//####DESCRIPTIONEND####
//
//==========================================================================
#include <pkgconf/system.h>
#include <pkgconf/io_eth_drivers.h>

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_intr.h>
#include <cyg/hal/hal_diag.h>
#include <cyg/infra/cyg_ass.h>
#include <cyg/infra/diag.h>
#include <cyg/hal/drv_api.h>
#include <cyg/io/eth/netdev.h>
#include <cyg/io/eth/eth_drv.h>
#include <cyg/io/smsc_lan92xx.h>


#ifdef CYGPKG_NET
#include <pkgconf/net.h>
#include <cyg/kernel/kapi.h>
#include <net/if.h>  /* Needed for struct ifnet */
#endif

//#define LAN92XX_DEBUG
#ifdef LAN92XX_DEBUG
#define PDEBUG(fmt, args...) diag_printf(fmt, ##args)
#else
#define PDEBUG(fmt, args...)
#endif /*LAN92XX_DEBUG*/

#define __WANT_DEVS
#include CYGDAT_DEVS_ETH_SMSC_LAN92XX_INL
#undef __WANT_DEVS

#define LAN_92XX_DRV_VER    "1.1"

#define MAX_RX_NUM (CYGNUM_IO_ETH_DRIVERS_NUM_PKT - 1)
static smsc_lan92xx_id_t smsc_lan92xx_id_table[] = 
{
    {0x117A, 0x0000, "SMSC LAN9217"},
    {0},
};

static int lan92xx_eeprom_present = 1;

static smsc_lan92xx_t lan92xx_dev;
static inline void
lan92xx_set_mac_addr(struct eth_drv_sc *sc, unsigned char *enaddr);
static void lan92xx_soft_reset(struct eth_drv_sc *sc);
static inline unsigned int
lan92xx_mac_read(struct eth_drv_sc *sc, unsigned char reg);
static inline void
lan92xx_mac_write(struct eth_drv_sc *sc, unsigned char reg, unsigned long val);
static inline unsigned int
lan92xx_mii_read(struct eth_drv_sc *sc, unsigned char addr);
static inline void
lan92xx_mii_write(struct eth_drv_sc *sc, unsigned char addr, unsigned int val);

/*!
 * This function set the value of PHY registers by MII interface
 */
static void
lan92xx_start(struct eth_drv_sc *sc, unsigned char *enaddr, int flags)
{
    unsigned int val;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    lan92xx_set_mac_addr(sc, enaddr);

    pdev->tx_busy = 0;

    val = lan92xx_mac_read(sc, MAC_MAC_CR)& (~0x800);
    val |= 0x0010080C;
    lan92xx_mac_write(sc, MAC_MAC_CR, val);
    val = lan92xx_mac_read(sc, MAC_MAC_CR);
}

/*!
 * This function pauses the FEC controller.
 */
static void
lan92xx_stop(struct eth_drv_sc *sc)
{
    unsigned int val;

    val = lan92xx_mac_read(sc, MAC_MAC_CR);
    val &= ~(0x0000000C);
    lan92xx_mac_write(sc, MAC_MAC_CR, val);
}

static int
lan92xx_control(struct eth_drv_sc *sc, unsigned long key, void *data, int data_length)
{
    /*TODO:: Add support */
    PDEBUG("%s: key=0x%x, data=0x%x, data_len=0x%x\n",
           __FUNCTION__, key, (unsigned long)data, (unsigned long)data_length);
    return 0;
}

/*!
 * This function checks the status of FEC control.
 */
static int
lan92xx_can_send(struct eth_drv_sc *sc)
{
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    if (!(pdev->status & PHY_LINK_ON)) return 0;
    if (pdev->tx_busy) return 0;

    return 1;
}

/*!
 * This function transmits a frame.
 */
static void
lan92xx_send(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len, int total, unsigned long key)
{
    int i, j, len, freespace;
    unsigned int tx_cmd1, tx_cmd2, data, *pdata;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);
    freespace = LAN92XX_REG_READ(LAN92XX_TX_FIFO_INF) & 0xFFFF;

    if (freespace < total + 16 ) {
        sc->funs->eth_drv->tx_done(sc, key, -1);
        return;
    }
    for (i = 0; i < sg_len; i++) {
        len = (sg_list[i].len + 3) >> 2;
        if (i == (sg_len - 1))
            tx_cmd1 = 0x1000;
        else if (i)
            tx_cmd1 = 0x0000;
        else
            tx_cmd1 = 0x2000;

        tx_cmd1 |= sg_list[i].len;
        tx_cmd2 = (total << 16) + total;
        LAN92XX_REG_WRITE(LAN92XX_TX_DATA, tx_cmd1);

        LAN92XX_REG_WRITE(LAN92XX_TX_DATA, tx_cmd2);
        pdata = (unsigned int *)sg_list[i].buf;

        for (j=0; j<len; j++) {
            data = *(pdata++);
            LAN92XX_REG_WRITE(LAN92XX_TX_DATA, data);
            for (data=0; data<2; data++) {
                asm volatile("nop");
                asm volatile("nop");
                asm volatile("nop");
                asm volatile("nop");
            }
        }
    }
    pdev->tx_busy = 1;
    pdev->tx_key = key;
}

static void
lan92xx_drop_packet(struct eth_drv_sc *sc, int count)
{
    unsigned int data;
    if (count >= 4) {
        LAN92XX_REG_WRITE(LAN92XX_RX_DP_CTRL, 0x80000000);
        while (LAN92XX_REG_READ(LAN92XX_RX_DP_CTRL) & 0x80000000) {
        }
    } else {
        while (count--)
            data = LAN92XX_REG_READ(LAN92XX_RX_DATA);
    }
}

/*!
 * This function receives ready Frame in DB.
 */
static void
lan92xx_recv(struct eth_drv_sc *sc, struct eth_drv_sg *sg_list, int sg_len)
{
    unsigned int i, len, rlen, status;
    unsigned int *pdata = (unsigned int *)(sg_list->buf);

    status = LAN92XX_REG_READ(LAN92XX_RX_STATUS1);

    len = (status >> 16) & 0x3FFF;
    rlen = (len + 3) >> 2;
    if (((void *)(sg_list->buf) == NULL) || (sg_list->len == 0)) {
        //diag_printf("WARING[RX]: the sg_list is empty\n");
        goto Drop;
    }

    if (sg_list->len < len) {
        diag_printf("WARING[RX]: packet(%dB)large than buffer (%dB)\n",
                    sg_list->len, len);
        goto Drop;
    }

    for (i = 0; i < rlen; i++) {
        *(pdata++) = LAN92XX_REG_READ(LAN92XX_RX_DATA);
    }
    return; 
Drop:
    lan92xx_drop_packet(sc, rlen);    
}

static void
lan92xx_deliver(struct eth_drv_sc *sc)
{
    /*TODO::When redboot support thread ,
     *      the polling function will be called at here
     */
    return;
}

static void 
lan92xx_link_status(struct eth_drv_sc *sc)
{
    unsigned int val;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);
    val = lan92xx_mii_read(sc, PHY_ISR);
    if (val&0x50) {
        val = lan92xx_mii_read(sc, PHY_BSR);
        if (val != pdev->status) {
            pdev->status = val;
            val = lan92xx_mac_read(sc, MAC_MAC_CR) & (~0x802F0800);
            if ( IS_DUPLEX(pdev->status)) {
                val |= 0x00100000;
            }
            lan92xx_mac_write(sc, MAC_MAC_CR, val);
        }
    }
}
/*!
 * This function checks the event of FEC controller
 */
static void
lan92xx_poll(struct eth_drv_sc *sc)
{
    unsigned int val, reg;
    int rx_num = 0;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    reg = LAN92XX_REG_READ(LAN92XX_INT_STS);
    LAN92XX_REG_WRITE(LAN92XX_INT_STS, reg);

    //diag_printf("INT_STS: %x\n", reg);	
    if (reg & 0x40000) {
        lan92xx_link_status(sc);
    }

    if (reg & 0xE000) {
        diag_printf("%s:: TX or RX error [0x%x]\n", __FUNCTION__, reg);
        lan92xx_soft_reset(sc);
        return;
    }

    while (1) {
        reg = LAN92XX_REG_READ(LAN92XX_RX_FIFO_INF);
        if (!(reg & 0xFF0000))
            break;
        if (!LAN92XX_REG_READ(LAN92XX_RX_STATUS2)) {
            diag_printf("***FIFO 0x%x, wrong status =0x%x: int_sts=0x%x\n",
                        reg, LAN92XX_REG_READ(LAN92XX_RX_STATUS2),
                        LAN92XX_REG_READ(LAN92XX_INT_STS));
            continue;
        }

        if (LAN92XX_REG_READ(LAN92XX_RX_STATUS2) & 0x4000909A) {
            val = (LAN92XX_REG_READ(LAN92XX_RX_STATUS1) >> 16) & 0x3FFF;
            val = (val + 3) >> 2;
            lan92xx_drop_packet(sc, val);
        } else {
            val = (LAN92XX_REG_READ(LAN92XX_RX_STATUS2) >> 16) & 0x3FFF;
            sc->funs->eth_drv->recv(sc, (val + 3) & (~3));
            rx_num++;
        }
        if ( rx_num >= MAX_RX_NUM) break;
    }

    while (1) {
        reg = LAN92XX_REG_READ(LAN92XX_TX_FIFO_INF);
        if (!(reg & 0xFF0000)) break;

        if (!LAN92XX_REG_READ(LAN92XX_TX_STATUS2)) {
            diag_printf("***FIFO %x, wrong status =%x: int_sts=%x\n",
                        reg, LAN92XX_REG_READ(LAN92XX_TX_STATUS2),
                        LAN92XX_REG_READ(LAN92XX_INT_STS));
            continue;
        }
        reg = LAN92XX_REG_READ(LAN92XX_TX_STATUS1);
        if (reg & 0x8000) {
            sc->funs->eth_drv->tx_done(sc, pdev->tx_key, -1);   
        } else {
            sc->funs->eth_drv->tx_done(sc, pdev->tx_key, 0);
        }
        pdev->tx_busy = 0;
    }
}

static int
lan92xx_int_vector(struct eth_drv_sc *sc)
{
    PDEBUG("%s::\n", __FUNCTION__);

    /*TODO::
     *      get FEC interrupt number
     */
    return -1;
}

static smsc_lan92xx_id_t *lan92xx_probe(unsigned long id)
{
    smsc_lan92xx_id_t *p = smsc_lan92xx_id_table;
    while (p->id) {
        if (id == p->id)
            return p;
        p++;
    }
    return NULL;
}

static inline unsigned int
lan92xx_mac_read(struct eth_drv_sc *sc, unsigned char reg)
{
    unsigned int cmd;
    
    if (LAN92XX_REG_READ(LAN92XX_MAC_CMD) & 0x80000000) {
        diag_printf("Error: %d. MAC is busy\n", __LINE__);
        return 0xFFFFFFFF;
    }

    cmd = 0xC0000000 | reg;
    LAN92XX_REG_WRITE(LAN92XX_MAC_CMD, cmd);

    while (LAN92XX_REG_READ(LAN92XX_MAC_CMD) & 0x80000000);

    return LAN92XX_REG_READ(LAN92XX_MAC_DATA);
}

static inline void
lan92xx_mac_write(struct eth_drv_sc *sc, unsigned char reg, unsigned long val)
{
    unsigned int cmd;

    if (LAN92XX_REG_READ(LAN92XX_MAC_CMD) & 0x80000000) {
        diag_printf("Error: %d. MAC is busy\n", __LINE__);
        return;
    }

    LAN92XX_REG_WRITE(LAN92XX_MAC_DATA, val);
    cmd = 0x80000000 | reg;
    LAN92XX_REG_WRITE(LAN92XX_MAC_CMD, cmd);

    while (LAN92XX_REG_READ(LAN92XX_MAC_CMD) & 0x80000000);
}

static inline void 
lan92xx_set_mac_addr(struct eth_drv_sc *sc, unsigned char *enaddr)
{
    unsigned int val;
    val = enaddr[3];
    val = (val << 8) | enaddr[2];
    val = (val << 8) | enaddr[1];
    val = (val << 8) | enaddr[0];
    lan92xx_mac_write(sc, MAC_ADDRL, val);

    val = lan92xx_mac_read(sc, MAC_ADDRH) >> 16;
    val = (val << 8) | enaddr[5];
    val = (val << 8) | enaddr[4];
    lan92xx_mac_write(sc, MAC_ADDRH, val);
}

static inline unsigned int
lan92xx_mii_read(struct eth_drv_sc *sc, unsigned char addr)
{
    unsigned int cmd;

    cmd = (0x1 << 11 ) | (addr << 6) | 1;
    lan92xx_mac_write(sc, MAC_MII_ACC, cmd);
    while (lan92xx_mac_read(sc, MAC_MII_ACC) & 1);

    return lan92xx_mac_read(sc, MAC_MII_DATA)&0xFFFF;
}

static inline void 
lan92xx_mii_write(struct eth_drv_sc *sc, unsigned char addr, unsigned int val)
{
    unsigned int cmd;

    cmd = (0x1 << 11 ) | (addr << 6) | 3;
    lan92xx_mac_write(sc, MAC_MII_DATA, val);
    lan92xx_mac_read(sc, MAC_MII_DATA);
    lan92xx_mac_write(sc, MAC_MII_ACC, cmd);

    while (lan92xx_mac_read(sc, MAC_MII_ACC) & 1);
}

static int lan92xx_phy_init(struct eth_drv_sc *sc)
{
    int val;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    lan92xx_mii_write(sc, PHY_BCR, 0x8000);

    while (lan92xx_mii_read(sc, PHY_BCR) & 0x8000);

    for (val = 0; val < 2500; val++)
        hal_delay_us(4);

    val = lan92xx_mii_read(sc, PHY_ANAR);
    val |= 0x01E1;
    lan92xx_mii_write(sc, PHY_ANAR, val);
    lan92xx_mii_write(sc, PHY_SMR, 0x00E1);
    lan92xx_mii_write(sc, PHY_SCSI, 0x400B);
    lan92xx_mii_write(sc, PHY_IMR, 0x00F0);
    lan92xx_mii_write(sc, PHY_BCR, 0x1200);

    while ((lan92xx_mii_read(sc, PHY_BCR) & 0x200));

    pdev->status = lan92xx_mii_read(sc, PHY_BSR);

    return 0;
}

static int lan92xx_mac_init(struct eth_drv_sc *sc)
{
    static int mac_init = 0;
    unsigned int val;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    val = lan92xx_mac_read(sc, MAC_MAC_CR) & (~0x802F0800);
    if (IS_DUPLEX(pdev->status)) {
        val |= 0x00100000;
    }
    lan92xx_mac_write(sc, MAC_MAC_CR, val);

    lan92xx_mac_write(sc, MAC_HASHH, 0);
    lan92xx_mac_write(sc, MAC_HASHL, 0);

    if (mac_init)
        return 0;

    mac_init = 1;

#if CYGSEM_HAL_VIRTUAL_VECTOR_SUPPORT
    if (!_board_provide_eth0_esa(pdev->mac_addr))
#endif	
    {
        // make sure EPC not busy
        while ((val = LAN92XX_REG_READ(LAN92XX_E2P_CMD)) & E2P_CMD_BUSY);

        if (val & E2P_CMD_TIMEOUT) {
            lan92xx_eeprom_present = 0;
            diag_printf("LAN9217: NO EEPROM\n");
            return -1;
        }
        
        if (!(LAN92XX_REG_READ(LAN92XX_E2P_CMD) & E2P_CMD_LOADED)) {
            diag_printf("LAN9217:EEPROM is empty\n");
        }
        val = lan92xx_mac_read(sc, MAC_ADDRH);
        pdev->mac_addr[5] = (val >> 8) & 0xFF;
        pdev->mac_addr[4] = val&0xFF;
        val = lan92xx_mac_read(sc, MAC_ADDRL);
        pdev->mac_addr[3] = (val >> 24) & 0xFF;
        pdev->mac_addr[2] = (val >> 16) & 0xFF;
        pdev->mac_addr[1] = (val >> 8) & 0xFF;
        pdev->mac_addr[0] = val & 0xFF;
    }
    return 0;
}

/*
 * This function reset LAN9219 .
 */
static void
lan92xx_soft_reset(struct eth_drv_sc *sc)
{
    unsigned int timeout = MAC_TIMEOUT;

    LAN92XX_REG_WRITE(LAN92XX_HW_CFG, 1);
    while ((LAN92XX_REG_READ(LAN92XX_HW_CFG) & 1) && (--timeout)) {
        hal_delay_us(MAC_TICKET);
    }

    if (!timeout) {
        diag_printf("LAN92XX: Reset fail \n");
        return ;
    }

    LAN92XX_REG_WRITE(LAN92XX_INT_EN, 0);
    LAN92XX_REG_WRITE(LAN92XX_HW_CFG, 0x150000);
    LAN92XX_REG_WRITE(LAN92XX_AFC_CFG, 0x6E3740);
    LAN92XX_REG_WRITE(LAN92XX_TX_CFG, 0x2);
    LAN92XX_REG_WRITE(LAN92XX_INT_EN, 0x40000);

    timeout = MAC_TIMEOUT;

    while ((LAN92XX_REG_READ(LAN92XX_E2P_CMD) & 0x80000000) && (--timeout)) {
        hal_delay_us(MAC_TICKET);
    }

    LAN92XX_REG_WRITE(LAN92XX_GPIO_CFG, 0x70070000);
    LAN92XX_REG_WRITE(LAN92XX_INT_STS, 0xFFFFFFFF);
    lan92xx_mac_init(sc);
}

/*!
 * This function initializes the LAN92xx driver.
 * It is called by net_init in net module of RedBoot during RedBoot init
 */
static bool
lan92xx_init(struct cyg_netdevtab_entry *tab)
{
    unsigned int reg, timeout;
    smsc_lan92xx_id_t *id;
    struct eth_drv_sc *sc = tab ? tab->device_instance : NULL;
    smsc_lan92xx_t *pdev = (smsc_lan92xx_t *)(sc->driver_private);

    diag_printf("\nLAN92xx Driver version %s\n", LAN_92XX_DRV_VER);  
    if (!pdev) {
        diag_printf("LAN92xx:: Driver don't attach with device\n");
        return false;
    }
    reg = LAN92XX_REG_READ(LAN92XX_ID_REV);
    id = lan92xx_probe(reg >> 16);
    if (id) {
        diag_printf("%s: ID = 0x%x REV = 0x%x\n", id->id_name, id->id, id->ver);
    } else {
        diag_printf("LAN92XX: unknow chip ID = %x\n", reg);
        return false;
    }

    timeout = MAC_TIMEOUT;
    while ((!(LAN92XX_REG_READ(LAN92XX_PMT_CTRL) & 1)) && (--timeout)) {
        hal_delay_us(MAC_TICKET);
    }
    if (timeout == 0) {
        diag_printf("LAN92XX: is not ready to access\n");
        return false;
    }

    lan92xx_phy_init(sc);

    lan92xx_soft_reset(sc);
    (sc->funs->eth_drv->init)(sc, pdev->mac_addr);
    return true;
}

/*!
 * Global variable which defines the LAN92xx driver,
 */
ETH_DRV_SC(lan92xx_sc,
           &lan92xx_dev, // Driver specific data
           CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME,
           lan92xx_start,
           lan92xx_stop,
           lan92xx_control,
           lan92xx_can_send,
           lan92xx_send,
           lan92xx_recv,
           lan92xx_deliver,     // "pseudoDSR" called from fast net thread
           lan92xx_poll,        // poll function, encapsulates ISR and DSR
           lan92xx_int_vector);

/*!
 * Global variable which defines the FEC device
 */
NETDEVTAB_ENTRY(lan92xx_netdev,
                "lan92xx_" CYGDAT_DEVS_ETH_ARM_MXCBOARD_ETH0_NAME,
                lan92xx_init,
                &lan92xx_sc);

// Low level function to issue a command to the eeprom controller.
// return 0 on success and -1 on failure
static inline int 
_lan92xx_e2p_do_cmd(unsigned int cmd)   
{
    unsigned int v;
    LAN92XX_REG_WRITE(LAN92XX_E2P_CMD, cmd);
    while ((v = LAN92XX_REG_READ(LAN92XX_E2P_CMD)) & E2P_CMD_BUSY);
    if (v & E2P_CMD_TIMEOUT) {
        diag_printf("%s:: EEPROM timeout\n", __FUNCTION__);
        // clear the timeout status bit
        LAN92XX_REG_WRITE(LAN92XX_E2P_CMD, E2P_CMD_TIMEOUT);
        while ((v = LAN92XX_REG_READ(LAN92XX_E2P_CMD)) & E2P_CMD_BUSY);
        return -1;
    }
    return 0;
}

// for all the 7 EEPROM operations
// return 0 on success and -1 on failure
static int
lan92xx_e2p_op(enum epc_cmd cmd, unsigned char addr, unsigned char *data)
{
    switch (cmd) {
    case E2P_CMD_READ:
        if (_lan92xx_e2p_do_cmd(E2P_CMD(cmd, addr)) != 0)
            return -1;
        *data = (unsigned char)LAN92XX_REG_READ(LAN92XX_E2P_DATA);
        return 0;
        break;
    case E2P_CMD_WRAL:
    case E2P_CMD_WRITE:
        LAN92XX_REG_WRITE(LAN92XX_E2P_DATA, *data);
        break;
    default:
        break;
    }

    if (_lan92xx_e2p_do_cmd(E2P_CMD(cmd, addr)) != 0)
        return -1;

    return 0;
}

static void setMac(int argc, char *argv[])
{
    int i;  
    unsigned char data[7];
    unsigned long temp;

    if (!lan92xx_eeprom_present) {
        diag_printf("NO EEPROM present\n\n");
        return;
    }

    if (argc == 1) {
        for (i = 0; i < 7 ; i++) {
            if (lan92xx_e2p_op(E2P_CMD_READ, i, &data[i]) != 0) {
                diag_printf("read MAC %d address fail\n\n", i);
                return;
            }
        }

        if (data[0] != E2P_CONTEXT_ID) {
            diag_printf("Warning: Unprogrammed MAC address: 0x%x\n", data[0]);
            return;
        }

        diag_printf("MAC address: ");
        diag_printf("0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x\n\n",
                    data[1], data[2], data[3],
                    data[4], data[5], data[6]);
        return;
    }

    if (argc != 2) {
        diag_printf("Error: Wrong argument\n");
        return;
    }

    data[0] = E2P_CONTEXT_ID;
    for (i = 1;  i < 7;  i++) {
        if (!parse_num(*(&argv[1]), &temp, &argv[1], ":")) {
            diag_printf("Error: failed to parse command: %d\n", __LINE__);
            return;
        }
        if (temp > 0xFF) {
            diag_printf("Error: invalid valie: 0x%x\n", (unsigned int)temp);
            return;
        }
        data[i] = temp;
    }

    // enable erase/write
    if (lan92xx_e2p_op(E2P_CMD_EWEN, 0, data) != 0) {
        diag_printf("%s:: Enable write/erase fail\n", __FUNCTION__);
        return;
    }
    for (i = 0; i < 7; i++) {
        if (lan92xx_e2p_op(E2P_CMD_ERASE, i, &data[i]) != 0 || 
            lan92xx_e2p_op(E2P_CMD_WRITE, i, &data[i]) != 0) {
            diag_printf("Error: failed to program eeprom at %d\n", i);
            return;
        }
    }

    // disable erase/write
    if (lan92xx_e2p_op(E2P_CMD_EWDS, 0, data) != 0) {
        diag_printf("%s:: Enable write/erase fail\n", __FUNCTION__);
    }
}

RedBoot_cmd("setmac",
            "Set Ethernet MAC address in EEPROM",
            "[0x##:0x##:0x##:0x##:0x##:0x##]",
            setMac
           );
