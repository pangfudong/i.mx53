/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/* Modified from Belcarra Linux USB driver by Yi Li */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned char u8;

#define IO_ADDRESS(x)  (x)
#define cpu_to_le16(x) (x)
#define le16_to_cpu(x) (x)

#if defined (CONFIG_ARCH_MXC91321)
#define MUX_I		0
#define MUX_F		8
#define PAD_I		10
#define _MXC_BUILD_PIN(gp,gi,mi,mf,pi,pf) ((mi << MUX_I) | (mf << MUX_F) | (pi << PAD_I))
#define PIN_USB_VPIN   _MXC_BUILD_PIN(0, 4, 70, 3, 93, 1)
#define PIN_USB_VPOUT  _MXC_BUILD_PIN(0, 5, 70, 2, 93, 0)
#define PIN_USB_VMIN   _MXC_BUILD_PIN(0, 6, 70, 1, 92, 2)
#define PIN_USB_XRXD   _MXC_BUILD_PIN(0, 7, 70, 0, 92, 1)
#define PIN_USB_VMOUT  _MXC_BUILD_PIN(0, 2, 71, 1, 94, 0)
#define PIN_USB_TXENB  _MXC_BUILD_PIN(0, 3, 71, 0, 93, 2)
#define PIN_TO_IOMUX_INDEX(pin) ((pin >> MUX_I) & ((1 << (MUX_F - MUX_I)) - 1))
#define PIN_TO_IOMUX_FIELD(pin) ((pin >> MUX_F) & ((1 << (PAD_I - MUX_F)) - 1))
#define IOMUXC_BASE_ADDR        0x50000000
#define MUX_CTL_BIT_LEN         8
#endif

#if defined (CONFIG_ARCH_MXC91321)
#define OTG_BASE_ADDR           0x50020000
#endif

#define OTG_CORE_BASE     		(OTG_BASE_ADDR+0x000)	
#define OTG_FUNC_BASE     		(OTG_BASE_ADDR+0x040)	
#define OTG_DMA_BASE      		(OTG_BASE_ADDR+0x800)	
#define OTG_EP_BASE       		(OTG_BASE_ADDR+0x400)	
#define OTG_SYS_BASE      		(OTG_BASE_ADDR+0x600)	

#define OTG_DATA_BASE           (OTG_BASE_ADDR+0x1000)	

#define SYS_CTRL_OTG_WU_INT_STAT                (1 << 26)
#define SYS_CTRL_FNT_WU_INT_STAT                (1 << 24)

#define SYS_CTRL_OTG_WU_INT_EN                  (1 << 18)
#define SYS_CTRL_FNT_WU_INT_EN                  (1 << 16)

#define OTG_CORE_HWMODE                         (OTG_CORE_BASE+0x00)

#define XCVR_D_D                                0x00
#define XCVR_SE0_D_NEW                          0x02
#define XCVR_D_SE0_NEW                          0x01
#define XCVR_SE0_SE0                            0x03

#define MODULE_ANASDBEN                         (1 << 14)
#define MODULE_OTGXCVR                          (0x3 << 6)
#define MODULE_HOSTXCVR                         (0x3 << 4)
#define MODULE_CRECFG                           (0x3)
#define MODULE_CRECFG_HHNP                      (0x0)
#define MODULE_CRECFG_HOST                      (0x1)
#define MODULE_CRECFG_FUNC                      (0x2)
#define MODULE_CRECFG_SHNP                      (0x3)

#define OTG_CORE_CINT_STAT                      (OTG_CORE_BASE+0x04)	//  32bit core int status reg

#define MODULE_FCINTDSPEN                       (1 << 6)

#define MODULE_ASHNPINT                         (1 << 5)
#define MODULE_ASFCINT                          (1 << 4)
#define MODULE_ASHCINT                          (1 << 3)
#define MODULE_HNPINT                           (1 << 2)
#define MODULE_FCINT                            (1 << 1)
#define MODULE_HCINT                            (1)

#define OTG_CORE_CINT_STEN                      (OTG_CORE_BASE+0x08)	//  32bit core int enable reg
#define OTG_CORE_CINT_STEN_CLR                  (OTG_CORE_BASE+0x3c)	//  32bit core int enable clear reg

#define MODULE_ASHNPINT_EN                      (1 << 5)
#define MODULE_ASFCINT_EN                       (1 << 4)
#define MODULE_ASHCINT_EN                       (1 << 3)
#define MODULE_HNPINT_EN                        (1 << 2)
#define MODULE_FCINT_EN                         (1 << 1)
#define MODULE_HCINT_EN                         (1)

#define OTG_CORE_CLK_CTRL                       (OTG_CORE_BASE+0x0C)	//  32bit core clock control reg

#define MODULE_FUNC_CLK                         (1 << 2)
#define MODULE_MAIN_CLK                         (1)

#define OTG_CORE_RST_CTRL                       (OTG_CORE_BASE+0x10)	// 32bit core reset control reg

#define MODULE_RSTI2C                           (1 << 15)
#define MODULE_RSTCTRL                          (1 << 5)
#define MODULE_RSTFC                            (1 << 4)
#define MODULE_RSTFSIE                          (1 << 3)
#define MODULE_RSTRH                            (1 << 2)
#define MODULE_RSTHSIE                          (1 << 1)
#define MODULE_RSTHC                            (1)

#define OTG_CORE_FRM_INTVL                      (OTG_CORE_BASE+0x14)	//  32bit core frame interval reg

#define MODULE_RESET_FRAME                      (1 << 15)

#define OTG_CORE_FRM_REMAIN                     (OTG_CORE_BASE+0x18)	//  32bit core frame remaining reg

#define OTG_CORE_HNP_CSTAT                      (OTG_CORE_BASE+0x1C)	//  32bit core HNP current state reg

#define MODULE_HNPDAT                           (1 << 30)
#define MODULE_VBUSBSE                          (1 << 29)
#define MODULE_VBUSABSV                         (1 << 28)
#define MODULE_VBUSGTAVV                        (1 << 27)

#define MODULE_ARMTHNPE                         (1 << 25)
#define MODULE_BHNPEN                           (1 << 24)

#define MODULE_SLAVE                            (1 << 22)
#define MODULE_MASTER                           (1 << 21)
#define MODULE_BGEN                             (1 << 20)
#define MODULE_CMPEN                            (1 << 19)

#define MODULE_SWVBUSPUL                        (1 << 15)

#define MODULE_SWAUTORST                        (1 << 12)
#define MODULE_SWPUDP                           (1 << 11)

#define MODULE_HNPSTATE                         (0x1f << 8)
#define MODULE_ABBUSREQ                         (1 << 1)

#define OTG_CORE_CPUEPSEL_STAT                  (OTG_CORE_BASE+0x34)
#define OTG_CORE_INTERRUPT_STEN                 (OTG_CORE_BASE+0x3c)

#define OTG_FUNC_CMD_STAT                       (OTG_FUNC_BASE+0x00)	//  32bit func command status reg

#define COMMAND_SOFTRESET                       (1 << 7)
#define COMMAND_BADISOAP                        (1 << 3)
#define COMMAND_SUPDET                          (1 << 2)
#define COMMAND_RSMINPROG                       (1 << 1)
#define COMMAND_RESETDET                        (1)

#define OTG_FUNC_DEV_ADDR                       (OTG_FUNC_BASE+0x04)	//  32bit func device address reg
#define OTG_FUNC_SINT_STAT                      (OTG_FUNC_BASE+0x08)	//  32bit func system int status reg

#define SYSTEM_DONEREGINTDS                     (1 << 5)
#define SYSTEM_SOFDETINT                        (1 << 4)
#define SYSTEM_DONEREGINT                       (1 << 3)
#define SYSTEM_SUSPDETINT                       (1 << 2)
#define SYSTEM_RSMFININT                        (1 << 1)
#define SYSTEM_RESETINT                         (1)

#define OTG_FUNC_SINT_STEN                      (OTG_FUNC_BASE+0x0C)	//  32bit func system int enable reg
#define OTG_FUNC_SINT_STEN_CLR                  (OTG_FUNC_BASE+0x10C)	//  32bit func system int enable clear reg

#define SYSTEM_DONEREGINTDS_EN                  (1 << 5)
#define SYSTEM_SOFDETINT_EN                     (1 << 4)
#define SYSTEM_DONEREGINT_EN                    (1 << 3)
#define SYSTEM_SUSPDETINT_EN                    (1 << 2)
#define SYSTEM_RSMFININT_EN                     (1 << 1)
#define SYSTEM_RESETINT_EN                      (1)

#define OTG_FUNC_XINT_STAT                      (OTG_FUNC_BASE+0x10)	//  32bit func X buf int status reg
#define OTG_FUNC_YINT_STAT                      (OTG_FUNC_BASE+0x14)	//  32bit func Y buf int status reg

#define OTG_FUNC_XYINT_STEN                     (OTG_FUNC_BASE+0x18)	//  32bit func XY buf int enable reg
#define OTG_FUNC_XYINT_STEN_CLR                 (OTG_FUNC_BASE+0x118)	//  32bit func XY buf int enable clear reg

#define OTG_FUNC_XFILL_STAT                     (OTG_FUNC_BASE+0x1C)	//  32bit func X filled status reg
#define OTG_FUNC_YFILL_STAT                     (OTG_FUNC_BASE+0x20)	//  32bit func Y filled status reg

#define OTG_FUNC_EP_EN                          (OTG_FUNC_BASE+0x24)	//  32bit func endpoints enable reg
#define OTG_FUNC_EP_EN_CLR                      (OTG_FUNC_BASE+0x124)	//  32bit func endpoints enable clear reg

#define OTG_FUNC_EP_RDY                         (OTG_FUNC_BASE+0x28)	//  32bit func endpoints ready reg
#define OTG_FUNC_EP_RDY_CLR                     (OTG_FUNC_BASE+0x3C)	//  32bit func endpoints ready clear reg

#define OTG_FUNC_IINT                           (OTG_FUNC_BASE+0x2C)	//  32bit func immediate interrupt reg
#define OTG_FUNC_IINT_CLR                       (OTG_FUNC_BASE+0x12C)	//  32bit func immediate interrupt clear reg

#define OTG_FUNC_EP_DSTAT                       (OTG_FUNC_BASE+0x30)	//  32bit func endpoints done status

#define OTG_FUNC_EP_DEN                         (OTG_FUNC_BASE+0x34)	//  32bit func endpoints done enable
#define OTG_FUNC_EP_DEN_CLR                     (OTG_FUNC_BASE+0x134)	//  32bit func endpoints done clear enable

#define OTG_FUNC_EP_TOGGLE                      (OTG_FUNC_BASE+0x38)	//  32bit func endpoints toggle bits
#define OTG_FUNC_FRM_NUM                        (OTG_FUNC_BASE+0x3C)	//  32bit func frame number reg

#define EP0_STALL                               (1 << 31)
#define EP0_SETUP                               (1 << 30)
#define EP0_OVERRUN                             (1 << 29)
#define EP0_AUTOISO                             (1 << 27)

#define EP_FORMAT_CONTROL     0x0
#define EP_FORMAT_ISOC        0x1
#define EP_FORMAT_BULK        0x2
#define EP_FORMAT_INTERRUPT   0x3

#define EP_OUT                                  0x1
#define EP_IN                                   0x2
#define EP_BOTH                                 0x3

#define NUM_ETDS                16
#define DATA_BUFF_SIZE          64
#define DATA_BUFFER_TOTAL       4096
#define NUM_DATA_BUFFS          (4096/DATA_BUFF_SIZE)

#define ep_num_both(n)                          (EP_BOTH << n)
#define ep_num_dir(n, dir)                      ((dir ? EP_IN : EP_OUT) << (n*2))
#define ep_num_out(n)                           ep_num_dir(n, USB_DIR_OUT)
#define ep_num_in(n)                            ep_num_dir(n, USB_DIR_IN)

/* ep descriptor access
 */
static __inline__ u32 ep_word(int n, int dir, int word)
{
	u32 offset = n * 2;
	offset += dir ? 1 : 0;
	offset *= 16;
	offset += word * 4;
	return OTG_EP_BASE + offset;
}

static volatile __inline__ u16 data_x_buf(int n, int dir)
{
	return 0x40 * (n * 4 + 2 * (dir ? 1 : 0));
}
static volatile __inline__ u16 data_y_buf(int n, int dir)
{
	return 0x40 * (n * 4 + 2 * (dir ? 1 : 0) + 1);
}

static volatile __inline__ u8 *data_x_address(int n, int dir)
{
	return (volatile u8 *)IO_ADDRESS(OTG_DATA_BASE + data_x_buf(n, dir));
}
static volatile __inline__ u8 *data_y_address(int n, int dir)
{
	return (volatile u8 *)IO_ADDRESS(OTG_DATA_BASE + data_y_buf(n, dir));
}

#define OTG_DMA_MISC_CTRL     (OTG_DMA_BASE+0x040)	//  32bit dma EP misc control reg
#define OTG_DMA_MISC_ARBMODE  (1 << 1)

#define OTG_DMA_ETD_CH_CLR    (OTG_DMA_BASE+0x048)	//  32bit dma ETD clear channel reg
#define OTG_DMA_EP_CH_CLR     (OTG_DMA_BASE+0x04c)	//  32bit dma EP clear channel reg

#define dma_num_dir(n, dir) (n * 2 + (dir ? 1 : 0))
#define dma_num_out(n) dma_num_dir(n, USB_DIR_OUT)
#define dma_num_in(n) dma_num_dir(n, USB_DIR_IN)

#define OTG_DMA_ETD_MSA(x)    (OTG_DMA_BASE+0x100+x*4)
#define OTG_DMA_EPN_MSA(x)    (OTG_DMA_BASE+0x180+x*4)
#define OTG_DMA_ETDN_BPTR(x)  (OTG_DMA_BASE+0x280+x*4)
#define OTG_DMA_EPN_BPTR(x)   (OTG_DMA_BASE+0x284+x*4)

typedef struct transfer_descriptor_w1 {
	u16 x;
	u16 y;
} volatile transfer_descriptor_w1;

typedef struct control_bulk_transfer_descriptor_w2 {
	u8 rtrydelay;
	u8 reserved;
	u16 flags;
} volatile control_bulk_transfer_descriptor_w2;

typedef struct interrupt_transfer_descriptor_w2 {
	u8 polinterv;
	u8 relpolpos;
	u16 flags;
} volatile interrupt_transfer_descriptor_w2;

typedef struct isoc_transfer_descriptor_w2 {
	u16 startfrm;
	u16 flags;
} volatile isoc_transfer_descriptor_w2;

typedef struct isoc_transfer_descriptor_w3 {
	u16 pkt0;
	u16 pkt1;
} volatile isoc_transfer_descriptor_w3;

typedef struct transfer_descriptor {
	union {
		u32 val;
		transfer_descriptor_w1 bufsrtad;
	} volatile w1;
	union {
		u32 val;
		control_bulk_transfer_descriptor_w2 cb;
		interrupt_transfer_descriptor_w2 intr;
		isoc_transfer_descriptor_w2 isoc;
	} volatile w2;
	union {
		u32 val;
		isoc_transfer_descriptor_w3 isoc;
	} volatile w3;
} __attribute__ ((packed))
volatile transfer_descriptor;

static u8 __inline__ fs_rb(u32 port)
{
	return *(volatile u8 *)(IO_ADDRESS(port));
}

static u32 __inline__ fs_rl(u32 port)
{
	return *(volatile u32 *)(IO_ADDRESS(port));
}

static void __inline__ fs_wb(u32 port, u8 val)
{
	*(volatile u8 *)(IO_ADDRESS(port)) = val;
}

static void __inline__ fs_orb(u32 port, u8 val)
{
	u8 set = fs_rb(port) | val;
	*(volatile u8 *)(IO_ADDRESS(port)) = set;
}

static void __inline__ fs_andb(u32 port, u8 val)
{
	u8 set = fs_rb(port) & val;
	*(volatile u8 *)(IO_ADDRESS(port)) = set;
}

static void __inline__ fs_wl(u32 port, u32 val)
{
	*(volatile u32 *)(IO_ADDRESS(port)) = val;
}

static void __inline__ fs_orl(u32 port, u32 val)
{
	u32 set = fs_rl(port);
	*(volatile u32 *)(IO_ADDRESS(port)) = (set | val);
}

static void __inline__ fs_andl(u32 port, u32 val)
{
	u32 set = fs_rl(port);
	*(volatile u32 *)(IO_ADDRESS(port)) = (set & val);
}

static void inline fs_memcpy32(u32 * dp, u32 * sp, volatile int words)
{
	while (words--)
		*dp++ = *sp++;
}

static void inline fs_memcpy(u8 * dp, u8 * sp, volatile int bytes)
{
	while (bytes--)
		*dp++ = *sp++;
}

static void inline fs_clear_words(volatile u32 * addr, int words)
{
	while (words--)
		*addr++ = 0;
}

#define TIMEOUT_VALUE 1000

void mxc_main_clock_on(void)
{
	u32 timeout = TIMEOUT_VALUE;
	fs_orl(OTG_CORE_CLK_CTRL, MODULE_MAIN_CLK);
	while (!(fs_rl(OTG_CORE_CLK_CTRL) & MODULE_MAIN_CLK)) {
		timeout--;
		if (!timeout)
			break;
	}
}

void mxc_main_clock_off(void)
{
	u32 timeout = TIMEOUT_VALUE;
	fs_wl(OTG_CORE_CLK_CTRL, 0);
	while ((fs_rl(OTG_CORE_CLK_CTRL) & MODULE_MAIN_CLK)) {
		timeout--;
		if (!timeout)
			break;
	}
}

void mxc_func_clock_on(void)
{
	u32 timeout = TIMEOUT_VALUE;
	fs_orl(OTG_CORE_CLK_CTRL, MODULE_FUNC_CLK);
	while (!(fs_rl(OTG_CORE_CLK_CTRL) & MODULE_FUNC_CLK)) {
		timeout--;
		if (!timeout)
			break;
	}
}

void mxc_func_clock_off(void)
{
	u32 timeout = TIMEOUT_VALUE;
	fs_andl(OTG_CORE_CLK_CTRL, 0x0);
	while ((fs_rl(OTG_CORE_CLK_CTRL) & (MODULE_FUNC_CLK | MODULE_MAIN_CLK))) {
		timeout--;
		if (!timeout)
			break;
	}
}
