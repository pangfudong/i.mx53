#ifndef CYGONCE_USBS_MXC_H
#define CYGONCE_USBS_MXC_H

#include <cyg/io/usb/usb.h>

#define BIT0 	0x00000001
#define BIT1 	0x00000002
#define BIT2 	0x00000004
#define BIT3 	0x00000008
#define BIT4	0x00000010
#define BIT5 	0x00000020
#define BIT6	0x00000040
#define BIT7	0x00000080
#define BIT8	0x00000100
#define BIT9 	0x00000200
#define BIT10 	0x00000400
#define BIT11 	0x00000800
#define BIT12 	0x00001000
#define BIT13 	0x00002000
#define BIT14 	0x00004000
#define BIT15 	0x00008000
#define BIT16 	0x00010000
#define BIT17 	0x00020000
#define BIT18 	0x00040000
#define BIT19 	0x00080000
#define BIT20 	0x00100000
#define BIT21 	0x00200000
#define BIT22 	0x00400000
#define BIT23 	0x00800000
#define BIT24 	0x01000000
#define BIT25 	0x02000000
#define BIT26 	0x04000000
#define BIT27 	0x08000000
#define BIT28 	0x10000000
#define BIT29 	0x20000000
#define BIT30 	0x40000000
#define BIT31 	0x80000000

#define  USB_CTRL_REG                         (0x43F88600)

/* USBCTRL */
#define UCTRL_OWIR      (1 << 31)   /* OTG wakeup intr request received */
#define UCTRL_OSIC_MASK     (3 << 29)   /* OTG  Serial Interface Config: */
#define UCTRL_OSIC_DU6      (0 << 29)   /* Differential/unidirectional 6 wire */
#define UCTRL_OSIC_DB4      (1 << 29)   /* Differential/bidirectional  4 wire */
#define UCTRL_OSIC_SU6      (2 << 29)   /* single-ended/unidirectional 6 wire */
#define UCTRL_OSIC_SB3      (3 << 29)   /* single-ended/bidirectional  3 wire */

#define UCTRL_OUIE      (1 << 28)   /* OTG ULPI intr enable */
#define UCTRL_OWIE      (1 << 27)   /* OTG wakeup intr enable */
#define UCTRL_OBPVAL_RXDP   (1 << 26)   /* OTG RxDp status in bypass mode */
#define UCTRL_OBPVAL_RXDM   (1 << 25)   /* OTG RxDm status in bypass mode */
#define UCTRL_OPM       (1 << 24)   /* OTG power mask */
#define UCTRL_H2WIR     (1 << 23)   /* HOST2 wakeup intr request received */
#define UCTRL_H2SIC_MASK    (3 << 21)   /* HOST2 Serial Interface Config: */
#define UCTRL_H2SIC_DU6     (0 << 21)   /* Differential/unidirectional 6 wire */
#define UCTRL_H2SIC_DB4     (1 << 21)   /* Differential/bidirectional  4 wire */
#define UCTRL_H2SIC_SU6     (2 << 21)   /* single-ended/unidirectional 6 wire */
#define UCTRL_H2SIC_SB3     (3 << 21)   /* single-ended/bidirectional  3 wire */

#ifdef CONFIG_ARCH_MX51
#define UCTRL_H2UIE     (1 << 8)    /* HOST2 ULPI intr enable */
#define UCTRL_H2WIE     (1 << 7)    /* HOST2 wakeup intr enable */
#define UCTRL_H2PP      0   /* Power Polarity for uh2 */
#define UCTRL_H2PM      (1 << 4)    /* HOST2 power mask */
#else
#define UCTRL_H2UIE     (1 << 20)   /* HOST2 ULPI intr enable */
#define UCTRL_H2WIE     (1 << 19)   /* HOST2 wakeup intr enable */
#define UCTRL_H2PP      (1 << 18)   /* Power Polarity for uh2 */
#define UCTRL_H2PM      (1 << 16)   /* HOST2 power mask */
#endif

#define UCTRL_H1WIR     (1 << 15)   /* HOST1 wakeup intr request received */
#define UCTRL_H1SIC_MASK    (3 << 13)   /* HOST1 Serial Interface Config: */
#define UCTRL_H1SIC_DU6     (0 << 13)   /* Differential/unidirectional 6 wire */
#define UCTRL_H1SIC_DB4     (1 << 13)   /* Differential/bidirectional  4 wire */
#define UCTRL_H1SIC_SU6     (2 << 13)   /* single-ended/unidirectional 6 wire */
#define UCTRL_H1SIC_SB3     (3 << 13)   /* single-ended/bidirectional  3 wire */
#define UCTRL_OLOCKD        (1 << 13)   /* otg lock disable */
#define UCTRL_H2LOCKD       (1 << 12)   /* HOST2 lock disable */
#define UCTRL_H1UIE     (1 << 12)   /* Host1 ULPI interrupt enable */

#define UCTRL_PP                (1 << 11)       /* power polarity bit */
#define UCTRL_H1WIE     (1 << 11)   /* HOST1 wakeup intr enable */
#define UCTRL_H1BPVAL_RXDP  (1 << 10)   /* HOST1 RxDp status in bypass mode */
#define UCTRL_XCSO              (1 << 10)       /* Xcvr Clock Select for OTG port */
#define UCTRL_H1BPVAL_RXDM  (1 <<  9)   /* HOST1 RxDm status in bypass mode */
#define UCTRL_XCSH2             (1 <<  9)       /* Xcvr Clock Select for Host port */
#define UCTRL_H1PM      (1 <<  8)   /* HOST1 power mask */
#define UCTRL_IP_PULIDP         (1 <<  8)       /* Ipp_Puimpel_Pullup_Dp */

#define UCTRL_IP_PUE_UP         (1 <<  7)       /* ipp_pue_pullup_dp */
#define UCTRL_IP_PUE_DOWN       (1 <<  6)       /* ipp_pue_pulldwn_dpdm */
#define UCTRL_H2DT      (1 <<  5)   /* HOST2 TLL disabled */
#define UCTRL_H1DT      (1 <<  4)   /* HOST1 TLL disabled */
#define UCTRL_USBTE             (1 <<  4)       /* USBT Transceiver enable */
#define UCTRL_OCPOL             (1 <<  3)       /* OverCurrent Polarity */
#define UCTRL_OCE               (1 <<  2)       /* OverCurrent Enable */
#define UCTRL_H2OCPOL       (1 <<  2)       /* OverCurrent Polarity of Host2 */
#define UCTRL_H2OCS             (1 <<  1)       /* Host OverCurrent State */
#define UCTRL_BPE       (1 <<  0)   /* bypass mode enable */
#define UCTRL_OTD       (1 <<  0)   /* OTG TLL Disable */
#define UCTRL_OOCS              (1 <<  0)       /* OTG OverCurrent State */

/* USB INTR Register Bit Masks */
#define  USB_INTR_INT_EN                      (0x00000001)
#define  USB_INTR_ERR_INT_EN                  (0x00000002)
#define  USB_INTR_PTC_DETECT_EN               (0x00000004)
#define  USB_INTR_FRM_LST_ROLL_EN             (0x00000008)
#define  USB_INTR_SYS_ERR_EN                  (0x00000010)
#define  USB_INTR_ASYN_ADV_EN                 (0x00000020)
#define  USB_INTR_RESET_EN                    (0x00000040)
#define  USB_INTR_SOF_EN                      (0x00000080)
#define  USB_INTR_DEVICE_SUSPEND              (0x00000100)

/* USB STS Register Bit Masks */
#define  USB_STS_INT                          (0x00000001)
#define  USB_STS_ERR                          (0x00000002)
#define  USB_STS_PORT_CHANGE                  (0x00000004)
#define  USB_STS_FRM_LST_ROLL                 (0x00000008)
#define  USB_STS_SYS_ERR                      (0x00000010)
#define  USB_STS_IAA                          (0x00000020)
#define  USB_STS_RESET                        (0x00000040)
#define  USB_STS_SOF                          (0x00000080)
#define  USB_STS_SUSPEND                      (0x00000100)
#define  USB_STS_HC_HALTED                    (0x00001000)
#define  USB_STS_RCL                          (0x00002000)
#define  USB_STS_PERIODIC_SCHEDULE            (0x00004000)
#define  USB_STS_ASYNC_SCHEDULE               (0x00008000)

/* Device Address bit masks */
#define  USB_DEVICE_ADDRESS_MASK              (0xFE000000)
#define  USB_DEVICE_ADDRESS_BIT_POS           (25)

/* USB CMD  Register Bit Masks */
#define  USB_CMD_RUN_STOP                     (0x00000001)
#define  USB_CMD_CTRL_RESET                   (0x00000002)
#define  USB_CMD_PERIODIC_SCHEDULE_EN         (0x00000010)
#define  USB_CMD_ASYNC_SCHEDULE_EN            (0x00000020)
#define  USB_CMD_INT_AA_DOORBELL              (0x00000040)
#define  USB_CMD_ASP                          (0x00000300)
#define  USB_CMD_ASYNC_SCH_PARK_EN            (0x00000800)
#define  USB_CMD_SUTW                         (0x00002000)
#define  USB_CMD_ATDTW                        (0x00004000)
#define  USB_CMD_ITC                          (0x00FF0000)

/* USB MODE Register Bit Masks */
#define  USB_MODE_CTRL_MODE_IDLE              (0x00000000)
#define  USB_MODE_CTRL_MODE_DEVICE            (0x00000002)
#define  USB_MODE_CTRL_MODE_HOST              (0x00000003)
#define  USB_MODE_CTRL_MODE_MASK              0x00000003
#define  USB_MODE_CTRL_MODE_RSV               (0x00000001)
#define  USB_MODE_ES                          0x00000004 /* (big) Endian Sel */
#define  USB_MODE_SETUP_LOCK_OFF              (0x00000008)
#define  USB_MODE_STREAM_DISABLE              (0x00000010)

/* endpoint list address bit masks */
#define USB_EP_LIST_ADDRESS_MASK              (0xfffff800)

/* PORTSCX  Register Bit Masks */
#define  PORTSCX_CURRENT_CONNECT_STATUS       (0x00000001)
#define  PORTSCX_CONNECT_STATUS_CHANGE        (0x00000002)
#define  PORTSCX_PORT_ENABLE                  (0x00000004)
#define  PORTSCX_PORT_EN_DIS_CHANGE           (0x00000008)
#define  PORTSCX_OVER_CURRENT_ACT             (0x00000010)
#define  PORTSCX_OVER_CURRENT_CHG             (0x00000020)
#define  PORTSCX_PORT_FORCE_RESUME            (0x00000040)
#define  PORTSCX_PORT_SUSPEND                 (0x00000080)
#define  PORTSCX_PORT_RESET                   (0x00000100)
#define  PORTSCX_LINE_STATUS_BITS             (0x00000C00)
#define  PORTSCX_PORT_POWER                   (0x00001000)
#define  PORTSCX_PORT_INDICTOR_CTRL           (0x0000C000)
#define  PORTSCX_PORT_TEST_CTRL               (0x000F0000)
#define  PORTSCX_WAKE_ON_CONNECT_EN           (0x00100000)
#define  PORTSCX_WAKE_ON_CONNECT_DIS          (0x00200000)
#define  PORTSCX_WAKE_ON_OVER_CURRENT         (0x00400000)
#define  PORTSCX_PHY_LOW_POWER_SPD            (0x00800000)
#define  PORTSCX_PORT_FORCE_FULL_SPEED        (0x01000000)
#define  PORTSCX_PORT_SPEED_MASK              (0x0C000000)
#define  PORTSCX_PORT_WIDTH                   (0x10000000)
#define  PORTSCX_PHY_TYPE_SEL                 (0xC0000000)

/* bit 11-10 are line status */
#define  PORTSCX_LINE_STATUS_SE0              (0x00000000)
#define  PORTSCX_LINE_STATUS_JSTATE           (0x00000400)
#define  PORTSCX_LINE_STATUS_KSTATE           (0x00000800)
#define  PORTSCX_LINE_STATUS_UNDEF            (0x00000C00)
#define  PORTSCX_LINE_STATUS_BIT_POS          (10)

/* bit 15-14 are port indicator control */
#define  PORTSCX_PIC_OFF                      (0x00000000)
#define  PORTSCX_PIC_AMBER                    (0x00004000)
#define  PORTSCX_PIC_GREEN                    (0x00008000)
#define  PORTSCX_PIC_UNDEF                    (0x0000C000)
#define  PORTSCX_PIC_BIT_POS                  (14)

/* bit 19-16 are port test control */
#define  PORTSCX_PTC_DISABLE                  (0x00000000)
#define  PORTSCX_PTC_JSTATE                   (0x00010000)
#define  PORTSCX_PTC_KSTATE                   (0x00020000)
#define  PORTSCX_PTC_SEQNAK                   (0x00030000)
#define  PORTSCX_PTC_PACKET                   (0x00040000)
#define  PORTSCX_PTC_FORCE_EN                 (0x00050000)
#define  PORTSCX_PTC_BIT_POS                  (16)

/* bit 27-26 are port speed */
#define  PORTSCX_PORT_SPEED_FULL              (0x00000000)
#define  PORTSCX_PORT_SPEED_LOW               (0x04000000)
#define  PORTSCX_PORT_SPEED_HIGH              (0x08000000)
#define  PORTSCX_PORT_SPEED_UNDEF             (0x0C000000)
#define  PORTSCX_SPEED_BIT_POS                (26)

/* bit 28 is parallel transceiver width for UTMI interface */
#define  PORTSCX_PTW                          (0x10000000)
#define  PORTSCX_PTW_8BIT                     (0x00000000)
#define  PORTSCX_PTW_16BIT                    (0x10000000)

/* bit 31-30 are port transceiver select */
#define  PORTSCX_PTS_UTMI                     (0x00000000)
#define  PORTSCX_PTS_ULPI                     (0x80000000)
#define  PORTSCX_PTS_FSLS                     (0xC0000000)
#define  PORTSCX_PTS_BIT_POS                  (30)

struct usb_dr_device {
	/* Capability register */
	cyg_uint32 id;
	cyg_uint32 res1[35];
	cyg_uint32 sbuscfg;		/* sbuscfg ahb burst */
	cyg_uint32 res11[27];
	cyg_uint16 caplength;		/* Capability Register Length */
	cyg_uint16 hciversion;		/* Host Controller Interface Version */
	cyg_uint32 hcsparams;		/* Host Controller Structual Parameters */
	cyg_uint32 hccparams;		/* Host Controller Capability Parameters */
	cyg_uint32 res2[5];
	cyg_uint32 dciversion;		/* Device Controller Interface Version */
	cyg_uint32 dccparams;		/* Device Controller Capability Parameters */
	cyg_uint32 res3[6];
	/* Operation register */
	cyg_uint32 usbcmd;		/* USB Command Register */
	cyg_uint32 usbsts;		/* USB Status Register */
	cyg_uint32 usbintr;		/* USB Interrupt Enable Register */
	cyg_uint32 frindex;		/* Frame Index Register */
	cyg_uint32 res4;
	cyg_uint32 deviceaddr;		/* Device Address */
	cyg_uint32 endpointlistaddr;	/* Endpoint List Address Register */
	cyg_uint32 res5;
	cyg_uint32 burstsize;		/* Master Interface Data Burst Size Register */
	cyg_uint32 txttfilltuning;	/* Transmit FIFO Tuning Controls Register */
	cyg_uint32 res6[6];
	cyg_uint32 configflag;		/* Configure Flag Register */
	cyg_uint32 portsc1;		/* Port 1 Status and Control Register */
	cyg_uint32 res7[7];
	cyg_uint32 otgsc;		/* On-The-Go Status and Control */
	cyg_uint32 usbmode;		/* USB Mode Register */
	cyg_uint32 endptsetupstat;	/* Endpoint Setup Status Register */
	cyg_uint32 endpointprime;	/* Endpoint Initialization Register */
	cyg_uint32 endptflush;		/* Endpoint Flush Register */
	cyg_uint32 endptstatus;	/* Endpoint Status Register */
	cyg_uint32 endptcomplete;	/* Endpoint Complete Register */
	cyg_uint32 endptctrl[8 * 2];	/* Endpoint Control Registers */
	cyg_uint32 res8[256];
	cyg_uint32 usbctrl;
};

/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK	0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK		0x80

#define USB_ENDPOINT_XFERTYPE_MASK	0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL	0
#define USB_ENDPOINT_XFER_ISOC		1
#define USB_ENDPOINT_XFER_BULK		2
#define USB_ENDPOINT_XFER_INT		3
#define USB_ENDPOINT_MAX_ADJUSTABLE	0x80

/*!
 * Endpoint Queue Head data struct
 * Rem: all the variables of qh are LittleEndian Mode
 * and NEXT_POINTER_MASK should operate on a LittleEndian, Phy Addr
 */
struct ep_queue_head {
	/*!
	 * Mult(31-30) , Zlt(29) , Max Pkt len  and IOS(15)
	 */
	cyg_uint32 max_pkt_length;

	/*!
	 *  Current dTD Pointer(31-5)
	 */
	cyg_uint32 curr_dtd_ptr;

	/*!
	 *  Next dTD Pointer(31-5), T(0)
	 */
	cyg_uint32 next_dtd_ptr;

	/*!
	 *  Total bytes (30-16), IOC (15), MultO(11-10), STS (7-0)
	 */
	cyg_uint32 size_ioc_int_sts;

	/*!
	 * Buffer pointer Page 0 (31-12)
	 */
	cyg_uint32 buff_ptr0;

	/*!
	 * Buffer pointer Page 1 (31-12)
	 */
	cyg_uint32 buff_ptr1;

	/*!
	 * Buffer pointer Page 2 (31-12)
	 */
	cyg_uint32 buff_ptr2;

	/*!
	 * Buffer pointer Page 3 (31-12)
	 */
	cyg_uint32 buff_ptr3;

	/*!
	 * Buffer pointer Page 4 (31-12)
	 */
	cyg_uint32 buff_ptr4;

	/*!
	 * reserved field 1
	 */
	cyg_uint32 res1;
	/*!
	 * Setup data 8 bytes
	 */
	cyg_uint8 setup_buffer[8];	/* Setup data 8 bytes */

	/*!
	 * reserved field 2,pad out to 64 bytes
	 */
	cyg_uint32 res2[4];
};

struct ep_td_struct {
	/*!
	 *  Next TD pointer(31-5), T(0) set indicate invalid
	 */
	cyg_uint32 next_td_ptr;

	/*!
	 *  Total bytes (30-16), IOC (15),MultO(11-10), STS (7-0)
	 */
	cyg_uint32 size_ioc_sts;

	/*!
	 * Buffer pointer Page 0
	 */
	cyg_uint32 buff_ptr0;

	/*!
	 * Buffer pointer Page 1
	 */
	cyg_uint32 buff_ptr1;

	/*!
	 * Buffer pointer Page 2
	 */
	cyg_uint32 buff_ptr2;

	/*!
	 * Buffer pointer Page 3
	 */
	cyg_uint32 buff_ptr3;

	/*!
	 * Buffer pointer Page 4
	 */
	cyg_uint32 buff_ptr4;

	/*!
	 * Buffer pointer Page 5
	 */
	cyg_uint32 buff_ptr5;

	/*!
	 * Buffer pointer Page 6
	 */
	cyg_uint32 buff_ptr6;

	/*!
	 * make it an even 16 words
	 * */
	cyg_uint32 res[7];
};

/* Endpoint Queue Head Bit Masks */
#define  EP_QUEUE_HEAD_MULT_POS               (30)
#define  EP_QUEUE_HEAD_ZLT_SEL                (0x20000000)
#define  EP_QUEUE_HEAD_MAX_PKT_LEN_POS        (16)
#define  EP_QUEUE_HEAD_MAX_PKT_LEN(ep_info)   (((ep_info)>>16)&0x07ff)
#define  EP_QUEUE_HEAD_IOS                    (0x00008000)
#define  EP_QUEUE_HEAD_NEXT_TERMINATE         (0x00000001)
#define  EP_QUEUE_HEAD_IOC                    (0x00008000)
#define  EP_QUEUE_HEAD_MULTO                  (0x00000C00)
#define  EP_QUEUE_HEAD_STATUS_HALT	      (0x00000040)
#define  EP_QUEUE_HEAD_STATUS_ACTIVE          (0x00000080)
#define  EP_QUEUE_CURRENT_OFFSET_MASK         (0x00000FFF)
#define  EP_QUEUE_HEAD_NEXT_POINTER_MASK      0xFFFFFFE0
#define  EP_QUEUE_FRINDEX_MASK                (0x000007FF)
#define  EP_MAX_LENGTH_TRANSFER               (0x4000)

/* ENDPOINTCTRLx  Register Bit Masks */
#define  EPCTRL_TX_ENABLE                     (0x00800000)
#define  EPCTRL_TX_DATA_TOGGLE_RST            (0x00400000)	/* Not EP0 */
#define  EPCTRL_TX_DATA_TOGGLE_INH            (0x00200000)	/* Not EP0 */
#define  EPCTRL_TX_TYPE                       (0x000C0000)
#define  EPCTRL_TX_DATA_SOURCE                (0x00020000)	/* Not EP0 */
#define  EPCTRL_TX_EP_STALL                   (0x00010000)
#define  EPCTRL_RX_ENABLE                     (0x00000080)
#define  EPCTRL_RX_DATA_TOGGLE_RST            (0x00000040)	/* Not EP0 */
#define  EPCTRL_RX_DATA_TOGGLE_INH            (0x00000020)	/* Not EP0 */
#define  EPCTRL_RX_TYPE                       (0x0000000C)
#define  EPCTRL_RX_DATA_SINK                  (0x00000002)	/* Not EP0 */
#define  EPCTRL_RX_EP_STALL                   (0x00000001)

/* bit 19-18 and 3-2 are endpoint type */
#define  EPCTRL_EP_TYPE_CONTROL               (0)
#define  EPCTRL_EP_TYPE_ISO                   (1)
#define  EPCTRL_EP_TYPE_BULK                  (2)
#define  EPCTRL_EP_TYPE_INTERRUPT             (3)
#define  EPCTRL_TX_EP_TYPE_SHIFT              (18)
#define  EPCTRL_RX_EP_TYPE_SHIFT              (2)

/*
 * ### pipe direction macro from device view
 */
#define USB_RECV	(0)	/* OUT EP */
#define USB_SEND	(1)	/* IN EP */
#define EP_DIR_OUT	(0)
#define EP_DIR_IN	(1)
#define EPOUT_COMPLETE  BIT0
#define EPIN_COMPLETE	BIT16

/* ### define USB registers here
 */
#define USB_MAX_ENDPOINTS		8
#define USB_MAX_PIPES			(USB_MAX_ENDPOINTS*2)
#define USB_MAX_CTRL_PAYLOAD		64
#define	USB_DR_SYS_OFFSET		0x400

/* Endpoint Setup Status bit masks */
#define  EP_SETUP_STATUS_MASK             (0x0000003F)
#define  EP_SETUP_STATUS_EP0		      (0x00000001)

#define USB_REQ_GET_STATUS		0x00
#define USB_REQ_CLEAR_FEATURE		0x01
#define USB_REQ_SET_FEATURE		0x03
#define USB_REQ_SET_ADDRESS		0x05
#define USB_REQ_GET_DESCRIPTOR		0x06
#define USB_REQ_SET_DESCRIPTOR		0x07
#define USB_REQ_GET_CONFIGURATION	0x08
#define USB_REQ_SET_CONFIGURATION	0x09
#define USB_REQ_GET_INTERFACE		0x0A
#define USB_REQ_SET_INTERFACE		0x0B
#define USB_REQ_SYNCH_FRAME		0x0C

/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DT_DEVICE			0x01
#define USB_DT_CONFIG			0x02
#define USB_DT_STRING			0x03
#define USB_DT_INTERFACE		0x04
#define USB_DT_ENDPOINT			0x05
#define USB_DT_DEVICE_QUALIFIER		0x06
#define USB_DT_OTHER_SPEED_CONFIG	0x07
#define USB_DT_INTERFACE_POWER		0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG			0x09
#define USB_DT_DEBUG			0x0a
#define USB_DT_INTERFACE_ASSOCIATION	0x0b
/* these are from the Wireless USB spec */
#define USB_DT_SECURITY			0x0c
#define USB_DT_KEY			0x0d
#define USB_DT_ENCRYPTION_TYPE		0x0e
#define USB_DT_BOS			0x0f
#define USB_DT_DEVICE_CAPABILITY	0x10
#define USB_DT_WIRELESS_ENDPOINT_COMP	0x11
#define USB_DT_WIRE_ADAPTER		0x21
#define USB_DT_RPIPE			0x22
#define USB_DT_CS_RADIO_CONTROL		0x23

/*
 * Device and/or Interface Class codes
 * as found in bDeviceClass or bInterfaceClass
 * and defined by www.usb.org documents
 */
#define USB_CLASS_PER_INTERFACE		0	/* for DeviceClass */
#define USB_CLASS_AUDIO			1
#define USB_CLASS_COMM			2
#define USB_CLASS_HID			3
#define USB_CLASS_PHYSICAL		5
#define USB_CLASS_STILL_IMAGE		6
#define USB_CLASS_PRINTER		7
#define USB_CLASS_MASS_STORAGE		8
#define USB_CLASS_HUB			9
#define USB_CLASS_CDC_DATA		0x0a
#define USB_CLASS_CSCID			0x0b	/* chip+ smart card */
#define USB_CLASS_CONTENT_SEC		0x0d	/* content security */
#define USB_CLASS_VIDEO			0x0e
#define USB_CLASS_WIRELESS_CONTROLLER	0xe0
#define USB_CLASS_MISC			0xef
#define USB_CLASS_APP_SPEC		0xfe
#define USB_CLASS_VENDOR_SPEC		0xff

#define USB_MAX_DEVICE_ADDR			127
#define USB_DEFAULT_ADDR			0x00

struct usb_ctrlrequest {
	cyg_uint8  bRequestType;
	cyg_uint8  bRequest;
	cyg_uint16 wValue;
	cyg_uint16 wIndex;
	cyg_uint16 wLength;
} __attribute__ ((packed));

/* enum for data transfer type on endpoints */
enum
{
    CONTROL,
    ISOCHRONOUS,
    BULK,
    INTERRUPT
};

/* USB Device State which are handled by DCD */
typedef enum
{
    USB_DEV_DUMMY_STATE,
    USB_DEV_DEFAULT_STATE,
    USB_DEV_ADDRESSED_STATE,
    USB_DEV_CONFIGURED_STATE
} usb_state_t;

typedef struct
{
	usb_configuration_descriptor usb_config_desc;
    usb_interface_descriptor  usb_interface_desc;
    usb_endpoint_descriptor  usb_endpoint_desc[2];
} __attribute__((packed)) usb_conf_desc;

typedef void (*RxCompletionFunc)(cyg_uint8*, cyg_uint32);

void usbs_imx_otg_device_init(int serial);
void register_rx_callback(RxCompletionFunc func);
void usbs_imx_otg_download(void);

#endif /* CYGONCE_USBS_MXC_H */
