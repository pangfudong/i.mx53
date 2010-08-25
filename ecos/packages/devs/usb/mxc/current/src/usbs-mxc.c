#include <string.h>
#include <cyg/infra/cyg_type.h>
#include <cyg/infra/diag.h>
#include <cyg/io/usbs-mxc.h>
#include <cyg/hal/hal_soc.h>
#include <cyg/hal/gpio.h>

#define NUM_EP                      3
#define BULK_TD_BUFFER_PAGE_SIZE    0x1000

#define usb_buffer        IRAM_BASE_ADDR
#define ep_dqh_base_addrs usb_buffer
#define ep_dtd_base_addrs (ep_dqh_base_addrs + 64 * 2 * NUM_EP)
#define ep0_buffer_addr   (ep_dtd_base_addrs + 64 * 2 * NUM_EP)
#define bulk_buffer       (IRAM_BASE_ADDR + 0x1000)
#define ep1_bulk_buffer   bulk_buffer
#define ep1_bulk_size     (12 * 1024)
#define ep2_bulk_buffer   (IRAM_BASE_ADDR + 1024)
#define ep2_bulk_size     (3 * 1024)
#define USBS_DEVICE_SET_ADDRESS(addr) dr_regs->deviceaddr = (addr & 0x7F) << 25

volatile struct usb_dr_device* dr_regs =
    (volatile struct usb_dr_device*)OTG_BASE_ADDR;

/* State of USB Device */
static volatile usb_state_t g_usb_dev_state = USB_DEV_DUMMY_STATE;
/* Local setup buffer */
static struct usb_ctrlrequest local_setup_buf;
static RxCompletionFunc rx_callback = NULL;
static cyg_uint8 device_desc[18] =
{
    0x12,       // length
    0x01,       // type
    0x00,       // usb_spec_lo
    0x02,       // usb_spec_hi
    0xFF,       // device_class
    0xFF,       // device_subclass
    0xFF,       // device_protocol
    0x40,       // max_packet_size
    0x59,       // vendor_lo
    0x23,       // vendor_hi
    0x85,       // product_lo
    0x17,       // product_hi
    0x01,       // device_lo
    0x00,       // device_hi
    0x01,       // manufacturer_str
    0x02,       // product_str
    0x03,       // serial_number_str
    0x01        // number_configurations
};

static cyg_uint8 conf_desc[32] =
{
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x04, 0xC0, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0x01, 0x05,
    0x07, 0x05, 0x01, 0x02, 0x00, 0x02, 0x00,
    0x07, 0x05, 0x82, 0x02, 0x00, 0x02, 0x00
};

static cyg_uint8 str0_desc[] =
{
    4, 3, 9, 4
};

/* manufacturer_str */
static cyg_uint8 str1_desc[] =
{
    48,  3,
    'O', 0,
    'n', 0,
    'y', 0,
    'x', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    't', 0,
    'e', 0,
    'r', 0,
    'n', 0,
    'a', 0,
    't', 0,
    'i', 0,
    'o', 0,
    'n', 0,
    'a', 0,
    'l', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    'c', 0,
    '.', 0
};

/* product_str */
static cyg_uint8 str2_desc[] =
{
    36,  3,
    'O', 0,
    'n', 0,
    'y', 0,
    'x', 0,
    ' ', 0,
    'B', 0,
    'O', 0,
    'O', 0,
    'X', 0,
    ' ', 0,
    'e', 0,
    'r', 0,
    'e', 0,
    'a', 0,
    'd', 0,
    'e', 0,
    'r', 0
};

/* serial_number_str */
static cyg_uint8 str3_desc[] =
{
    14,  3,
    '1', 0,
    '2', 0,
    '3', 0,
    '4', 0,
    '5', 0,
    '6', 0
};

static void enable_usbclk(void)
{
    cyg_uint32 reg;
    reg = readl(CCM_BASE_ADDR + CLKCTL_CGR1);
    reg |= 3 << 18;
    writel(reg, CCM_BASE_ADDR + CLKCTL_CGR1);
}

static void gpio_usbotg_hs_active(void)
{
    mxc_request_iomux(MX31_PIN_USBOTG_DATA0, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA1, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA2, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA3, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA4, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA5, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA6, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DATA7, OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_CLK,   OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_DIR,   OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_NXT,   OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);
    mxc_request_iomux(MX31_PIN_USBOTG_STP,   OUTPUTCONFIG_FUNC,
        INPUTCONFIG_FUNC);

    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA0, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA1, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA2, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA3, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA4, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA5, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA6, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA7, (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_CLK,   (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_DIR,   (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_NXT,   (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));
    mxc_iomux_set_pad(MX31_PIN_USBOTG_STP,   (PAD_CTL_DRV_MAX |
        PAD_CTL_SRE_FAST));

    /* reset transceiver */
    mxc_request_iomux(MX31_PIN_USB_PWR, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
    mxc_set_gpio_direction(MX31_PIN_USB_PWR, 0);
    mxc_set_gpio_dataout(MX31_PIN_USB_PWR, 0);
    hal_delay_us(1000);
    mxc_set_gpio_dataout(MX31_PIN_USB_PWR, 1);
}

static void otg_set_ulpi_xcvr(void)
{
    cyg_uint32 v = readl(USB_CTRL_REG);

    v &= ~UCTRL_OSIC_MASK;
    v &= ~UCTRL_BPE;
    v |= UCTRL_OUIE |    /* ULPI intr enable */
        UCTRL_OWIE |    /* OTG wakeup intr enable */
        UCTRL_OPM;        /* power mask */
    writel(v, USB_CTRL_REG);

    /* must set ULPI phy before turning off clock */
    dr_regs->portsc1 &= ~(3 << PORTSCX_PTS_BIT_POS);
    dr_regs->portsc1 |= PORTSCX_PTS_ULPI;

    /* need to reset the controller here so that the ID pin
     * is correctly detected.
     */
    dr_regs->usbcmd |= USB_CMD_CTRL_RESET;

    /* allow controller to reset, and leave time for
     * the ULPI transceiver to reset too.
     */
    hal_delay_us(100000);
}

static void dr_controller_setup(void)
{
    cyg_uint32 tmp = 0;
    cyg_uint32 wait_count = 0;

    /* Stop and reset the usb controller */
    dr_regs->usbcmd &= ~USB_CMD_RUN_STOP;
    dr_regs->usbcmd |= USB_CMD_CTRL_RESET;
    while (dr_regs->usbcmd & USB_CMD_CTRL_RESET)
    {
        if (++wait_count == 1000)
        {
            diag_printf("Reset otg device controller timed out!\n");
            return;
        }

        hal_delay_us(1000);
    }

    /* Set the controller as device mode */
    tmp = dr_regs->usbmode;
    tmp &= ~USB_MODE_CTRL_MODE_MASK;   /* clear mode bits */
    tmp |= USB_MODE_CTRL_MODE_DEVICE;
    tmp |= USB_MODE_SETUP_LOCK_OFF;    /* Disable Setup Lockout */
    dr_regs->usbmode = tmp;

    /* Clear the setup status */
    dr_regs->usbsts = 0;

    /* Set endpoint list address */
    dr_regs->endpointlistaddr = usb_buffer & USB_EP_LIST_ADDRESS_MASK;

    /* Config PHY interface */
    tmp = dr_regs->portsc1;
    tmp &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
    tmp |= PORTSCX_PTS_ULPI;
    dr_regs->portsc1 = tmp;
}

static void dr_controller_run(void)
{
    dr_regs->usbintr = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
        | USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
        | USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

    /* Set controller to Run */
    dr_regs->usbcmd |= USB_CMD_RUN_STOP;
}

void usbs_imx_otg_device_init(int serial)
{
    cyg_uint8 i = 0;
    cyg_uint8 tmp[7];
    diag_sprintf(tmp, "%06d", serial);
    for (i = 0; i < 6; i++)
    {
        str3_desc[2 * (i + 1)] = tmp[i];
    }

    enable_usbclk();

    gpio_usbotg_hs_active();

    otg_set_ulpi_xcvr();

    dr_controller_setup();

    dr_controller_run();
}

static void struct_ep_qh_setup(cyg_uint8  ep_num,
                               cyg_uint8  dir,
                               cyg_uint8  ep_type,
                               cyg_uint32 max_pkt_len,
                               cyg_uint32 zlt,
                               cyg_uint8  mult)
{
    struct ep_queue_head *p_QH = (struct ep_queue_head *)(ep_dqh_base_addrs +
        (2 * ep_num + dir) * sizeof(struct ep_queue_head));
    cyg_uint32 tmp = 0;

    /* set the Endpoint Capabilites in QH */
    switch (ep_type) {
    case USB_ENDPOINT_XFER_CONTROL:
        /* Interrupt On Setup (IOS). for control ep  */
        tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
            | EP_QUEUE_HEAD_IOS;
        break;
    case USB_ENDPOINT_XFER_ISOC:
        tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
            | (mult << EP_QUEUE_HEAD_MULT_POS);
        break;
    case USB_ENDPOINT_XFER_BULK:
    case USB_ENDPOINT_XFER_INT:
        tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
        break;
    default:
        diag_printf("error ep type is %d\n", ep_type);
        return;
    }
    if (zlt)
        tmp |= EP_QUEUE_HEAD_ZLT_SEL;
    p_QH->max_pkt_length = tmp;

    return;
}

static void endpoint_stall(cyg_uint8 endpoint, cyg_uint8 direction)
{
    if (direction == EP_DIR_OUT)
    {
        dr_regs->endptctrl[endpoint] |= EPCTRL_RX_EP_STALL;
    }
    else
    {
        dr_regs->endptctrl[endpoint] |= EPCTRL_TX_EP_STALL;
    }
}

static void dr_ep_setup(cyg_uint8 ep_num, cyg_uint8 dir, cyg_uint8 ep_type)
{
    cyg_uint32 tmp_epctrl = 0;

    tmp_epctrl = dr_regs->endptctrl[ep_num];
    if (dir) {
        if (ep_num)
            tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
        tmp_epctrl |= EPCTRL_TX_ENABLE;
        tmp_epctrl |= ((cyg_uint32)(ep_type)
                << EPCTRL_TX_EP_TYPE_SHIFT);
    } else {
        if (ep_num)
            tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
        tmp_epctrl |= EPCTRL_RX_ENABLE;
        tmp_epctrl |= ((cyg_uint32)(ep_type)
                << EPCTRL_RX_EP_TYPE_SHIFT);
    }

    dr_regs->endptctrl[ep_num] = tmp_epctrl;
}

static void usbs_ep0_init_dqh(void)
{
    memset((cyg_uint8 *)ep_dqh_base_addrs, 0,
        NUM_EP * 2 * sizeof(struct ep_queue_head));

    struct_ep_qh_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
            USB_MAX_CTRL_PAYLOAD, 0, 0);
    struct_ep_qh_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
            USB_MAX_CTRL_PAYLOAD, 0, 0);
    dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
    dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);
}

static void handle_reset(void)
{
    cyg_uint32 temp;

    dr_regs->usbcmd &= ~BIT0; //detach device from bus temprorarily

    /*1. Reading and writing back the ENDPTSETUPSTAT register
      clears the setup token semaphores */
    temp = dr_regs->endptsetupstat;
    dr_regs->endptsetupstat = temp;

    /*2. Reading and writing back the ENDPTCOMPLETE register
      clears the endpoint complete status bits */
    temp = dr_regs->endptcomplete;
    dr_regs->endptcomplete = temp;

    /*3. Cancel all primed status by waiting until all bits in ENDPTPRIME are 0
       and then write 0xFFFFFFFF to ENDPTFLUSH */
    while(dr_regs->endpointprime);
    dr_regs->endptflush = 0xFFFFFFFF;

    /*4. Initialize EP0 Queue Head again*/
    usbs_ep0_init_dqh();

    dr_regs->endpointlistaddr = usb_buffer & USB_EP_LIST_ADDRESS_MASK;

    dr_regs->usbcmd |= BIT0; //re-attach device to the bus

    g_usb_dev_state = USB_DEV_DEFAULT_STATE;
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(cyg_uint8 ep_num, cyg_uint8* buffer_ptr)
{
    struct ep_queue_head* qh = (struct ep_queue_head *)(ep_dqh_base_addrs +
        (2 * ep_num + EP_DIR_OUT) * sizeof(struct ep_queue_head));

    /* Clear bit in ENDPTSETUPSTAT */
    dr_regs->endptsetupstat |= 1 << ep_num;

    /* while a hazard exists when setup package arrives */
    do {
        /* Set Setup Tripwire */
        dr_regs->usbcmd |= USB_CMD_SUTW;

        /* Copy the setup packet to local buffer */
        memcpy(buffer_ptr, (cyg_uint8 *) qh->setup_buffer, 8);
    } while (!(dr_regs->usbcmd & USB_CMD_SUTW));

    /* Clear Setup Tripwire */
    dr_regs->usbcmd &= ~USB_CMD_SUTW;
}

static void ep0_send_data(cyg_uint8* buf, cyg_uint32 len, cyg_uint8 zlt)
{
    cyg_uint32 dqh_address = ep_dqh_base_addrs + 64;
    cyg_uint32 dtd_address = ep_dtd_base_addrs + 64;
    volatile struct ep_td_struct* td = (struct ep_td_struct *)dtd_address;

    td->next_td_ptr = 1;
    td->size_ioc_sts = ((len & 0x7FFF) << 16) | (0 << 15) | BIT7;
    td->buff_ptr0 = (cyg_uint32)buf;

    /* Enable ZLT when data size is in multiple of Maximum Packet Size  */
    if (zlt)
    {
        /* set ZLT enable */
        (*(volatile cyg_uint32*)(dqh_address)) &= ~0x20000000;
    }

    /* 1. write dQH next ptr and dQH terminate bit to 0  */
    *(volatile cyg_uint32*)(dqh_address + 0x8) = dtd_address;

    /* 2. clear active & halt bit in dQH */
    *(volatile cyg_uint32*)(dqh_address + 0xC) &= ~0xFF;

    /* 3. prime endpoint by writing '1' in ENDPTPRIME */
    dr_regs->endpointprime |= BIT16;
    while (dr_regs->endpointprime & BIT16);

    /* 4. wait for complete set and clear */
    while (!(dr_regs->endptcomplete & EPIN_COMPLETE));
    dr_regs->endptcomplete &= EPIN_COMPLETE;

    /* 5. check if transfer is successful */
    if (*(volatile cyg_uint32*)(dqh_address + 0xC) & 0xFF)
    {
        diag_printf("ep0 send data failed.\n");
    }
}

static void ep0_receive_data(cyg_uint8* buf, cyg_uint32 len)
{
    cyg_uint32 dqh_address = ep_dqh_base_addrs;
    cyg_uint32 dtd_address = ep_dtd_base_addrs;
    volatile struct ep_td_struct* td = (struct ep_td_struct *)dtd_address;

    td->next_td_ptr = ((dtd_address + 0x20) & 0xFFFFFFE0) | 1;
    td->size_ioc_sts = ((len & 0x7FFF) << 16) | (0 << 15) | BIT7;
    td->buff_ptr0 = (cyg_uint32)buf;

    /* 1. write dQH next ptr and dQH terminate bit to 0 */
    *(volatile cyg_uint32*)(dqh_address + 0x8) = dtd_address;

    /* 2. clear active & halt bit in dQH */
    *(volatile cyg_uint32*)(dqh_address + 0xC) &= ~0xFF;

    /* 3. prime endpoint by writing '1' in ENDPTPRIME */
    dr_regs->endpointprime |= BIT0;
    while (dr_regs->endpointprime & BIT0);

    /* 4. wait for complete set and clear */
    while (!(dr_regs->endptcomplete & EPOUT_COMPLETE));
    dr_regs->endptcomplete &= EPOUT_COMPLETE;

    /* 5. check if transfer is successful */
    if (*(volatile cyg_uint32*)(dqh_address + 0xC) & 0xFF)
    {
        diag_printf("ep0 receive data failed.\n");
    }
}

static void usbs_status_phase(cyg_uint8 trans_type, cyg_uint8 direction)
{
    if (trans_type == CONTROL)
    {
        if (direction == EP_DIR_OUT)
        {
            /* Receive 0 bytes from host */
            ep0_receive_data((cyg_uint8 *)0xFFFFFFFF, 0);
        }
        else
        {
            /* Send 0 bytes to host */
            ep0_send_data((cyg_uint8 *)0xFFFFFFFF, 0, 0);
        }
    }
}

static void handle_get_device_desc(struct usb_ctrlrequest *setup)
{
    cyg_uint32 bytes_to_send = sizeof(device_desc);

    /* Copy device descriptor to ep0 buffer */
    memcpy((cyg_uint8 *)ep0_buffer_addr, device_desc, bytes_to_send);

    if (bytes_to_send > setup->wLength)
    {
        bytes_to_send = setup->wLength;
    }

    ep0_send_data((cyg_uint8 *)ep0_buffer_addr, bytes_to_send, 0);

    usbs_status_phase(CONTROL, EP_DIR_OUT);
}

static void handle_get_config_desc(struct usb_ctrlrequest *setup)
{
    cyg_uint32 bytes_to_send = sizeof(conf_desc);

    /* Copy device descriptor to ep0 buffer */
    memcpy((cyg_uint8 *)ep0_buffer_addr, conf_desc, bytes_to_send);

    if (bytes_to_send > setup->wLength)
    {
        bytes_to_send = setup->wLength;
    }

    ep0_send_data((cyg_uint8 *)ep0_buffer_addr, bytes_to_send, 0);

    usbs_status_phase(CONTROL, EP_DIR_OUT);
}

static void handle_get_string_desc(struct usb_ctrlrequest *setup)
{
    cyg_uint8* str_buf = NULL;
    cyg_uint8  bytes_to_send = 0;

    switch (setup->wValue & 0xFF)
    {
        case 0:
            str_buf = str0_desc;
            bytes_to_send = sizeof(str0_desc);
            break;
        case 1:
            str_buf = str1_desc;
            bytes_to_send = sizeof(str1_desc);
            break;
        case 2:
            str_buf = str2_desc;
            bytes_to_send = sizeof(str2_desc);
            break;
        case 3:
            str_buf = str3_desc;
            bytes_to_send = sizeof(str3_desc);
            break;
        default:
            diag_printf("Ignore string desc index %d\n", setup->wValue & 0xFF);
            break;
    }

    /* Copy device descriptor to ep0 buffer */
    memcpy((cyg_uint8 *)ep0_buffer_addr, str_buf, bytes_to_send);

    if (bytes_to_send > setup->wLength)
    {
        bytes_to_send = setup->wLength;
    }

    ep0_send_data((cyg_uint8 *)ep0_buffer_addr, bytes_to_send, 0);

    usbs_status_phase(CONTROL, EP_DIR_OUT);
}

static void handle_get_descriptor(struct usb_ctrlrequest *setup)
{
    switch (setup->wValue >> 8)
    {
        case USB_DT_DEVICE:
            diag_printf("USB_DT_DEVICE\n");
            handle_get_device_desc(setup);
            break;
        case USB_DT_DEVICE_QUALIFIER:
            diag_printf("USB_DT_DEVICE_QUALIFIER\n");
            break;
        case USB_DT_OTHER_SPEED_CONFIG:
            diag_printf("USB_DT_OTHER_SPEED_CONFIG\n");
            break;
        case USB_DT_CONFIG:
            diag_printf("USB_DT_CONFIG\n");
            handle_get_config_desc(setup);
            break;
        case USB_DT_STRING:
            diag_printf("USB_DT_STRING\n");
            handle_get_string_desc(setup);
            break;
        default:
            diag_printf("handle_get_descriptor: unhandled wValue: %d\n",
                setup->wValue >> 8);
            break;
    }
}

static void handle_set_address(struct usb_ctrlrequest *setup)
{
    cyg_uint16 device_addrs = setup->wValue;

    if (setup->wIndex == 0 &&
        setup->wLength == 0 &&
        device_addrs <= USB_MAX_DEVICE_ADDR)
    {
        switch (g_usb_dev_state)
        {
            case USB_DEV_DEFAULT_STATE:
                /* send ack to host */
                usbs_status_phase(CONTROL, EP_DIR_IN);
                if (device_addrs != USB_DEFAULT_ADDR)
                {
                    USBS_DEVICE_SET_ADDRESS(device_addrs);
                    g_usb_dev_state = USB_DEV_ADDRESSED_STATE;
                }
                break;
            case USB_DEV_ADDRESSED_STATE:
                /* Send Ack to Host */
                usbs_status_phase(CONTROL, EP_DIR_IN);
                if (device_addrs == USB_DEFAULT_ADDR)
                {
                    /* Set the Device Address */
                    USBS_DEVICE_SET_ADDRESS(USB_DEFAULT_ADDR);
                    /* Change state to ADDRESSED STATE  */
                    g_usb_dev_state = USB_DEV_DEFAULT_STATE;
                }
                else
                {
                    /* Set the Device Address */
                    USBS_DEVICE_SET_ADDRESS(device_addrs);
                }
                break;
            case USB_DEV_CONFIGURED_STATE:
                if (device_addrs == USB_DEFAULT_ADDR)
                {
                    /* Send Ack to Host */
                    usbs_status_phase(CONTROL, EP_DIR_IN);
                    /* Set the Device Address */
                    USBS_DEVICE_SET_ADDRESS(device_addrs);
                    /* Change state to ADDRESSED STATE  */
                    g_usb_dev_state = USB_DEV_DEFAULT_STATE;
                }
                else
                {
                    /* Send STALL Handshake  */
                    endpoint_stall(0, EP_DIR_IN);
                }
                break;
            default:
                break;
        }
    }
}

static void setup_bulk_ep(cyg_uint8 ep_no, cyg_uint8 dir, cyg_uint32 mps)
{
    volatile struct ep_queue_head* qh =
        (struct ep_queue_head *)(ep_dqh_base_addrs + 64 * (2 * ep_no + dir));
    volatile struct ep_td_struct* td =
        (struct ep_td_struct *)(ep_dtd_base_addrs + 64 * (2 * ep_no + dir));

    if (dir == EP_DIR_OUT)
    {
        /* setup queue header */
        qh->max_pkt_length = (1 << 29) | (mps << 16) | (1 << 15);
        qh->curr_dtd_ptr = 0;
        qh->next_dtd_ptr = ((cyg_uint32)td & 0xFFFFFFE0) | 1;
        qh->size_ioc_int_sts = ((ep1_bulk_size & 0x7FFF) << 16) | (1 << 15);

        dr_regs->endptctrl[ep_no] = 0x00080048;
        dr_regs->endptctrl[ep_no] |= BIT7;

        /* setup transfer descriptor */
        td->next_td_ptr = (((cyg_uint32)td + 0x20) & 0xFFFFFFE0) | 1;
        td->size_ioc_sts = ((ep1_bulk_size & 0x7FFF) << 16) | (1 <<15 ) | BIT7;
        td->buff_ptr0 = ep1_bulk_buffer;
        td->buff_ptr1 = ep1_bulk_buffer + BULK_TD_BUFFER_PAGE_SIZE;
        td->buff_ptr2 = ep1_bulk_buffer + BULK_TD_BUFFER_PAGE_SIZE * 2;

        /* 1. write dQH next ptr and dQH terminate bit to 0 */
        *(volatile cyg_uint32*)((cyg_uint32)qh + 0x8) = (cyg_uint32)td;

        /* 2. clear active & halt bit in dQH */
        *(volatile cyg_uint32*)((cyg_uint32)qh + 0xC) &= ~0xFF;

        /* 3. prime endpoint by writing '1' in ENDPTPRIME */
        dr_regs->endpointprime |= (EPOUT_COMPLETE << ep_no);
        while (dr_regs->endpointprime & (EPOUT_COMPLETE << ep_no));
    }
    else
    {
        /* setup queue header */
        qh->max_pkt_length = (1 << 29) | (mps << 16) | (1 << 15);
        qh->curr_dtd_ptr = 0;
        qh->next_dtd_ptr = (cyg_uint32)qh | 1;
        qh->size_ioc_int_sts = ((ep2_bulk_size & 0x7FFF) << 16) | (1 << 15);

        /* 1. Endpoint 2: MPS = 64, IN (Tx endpoint) */
        dr_regs->endptctrl[ep_no] = 0x00480008;

        /* 2. Enable EP2 IN */
        dr_regs->endptctrl[ep_no] |= BIT23;

        /* 3. prime endpoint by writing '1' in ENDPTPRIME */
        dr_regs->endpointprime |= (EPIN_COMPLETE << ep_no);
        while (dr_regs->endpointprime & (EPIN_COMPLETE << ep_no));
    }
}

static void handle_set_configuration(struct usb_ctrlrequest *setup)
{
    cyg_uint8 cfg = setup->wValue & 0xFF;

    if (g_usb_dev_state == USB_DEV_ADDRESSED_STATE)
    {
        if (cfg == 0)
        {
            /* Send Ack to Host*/
            usbs_status_phase(CONTROL, EP_DIR_IN);
        }
        else if (cfg == 1)
        {
            setup_bulk_ep(1, EP_DIR_OUT, 0x200);
            setup_bulk_ep(2, EP_DIR_IN,  0x200);

            /* Send Ack to Host*/
            usbs_status_phase(CONTROL, EP_DIR_IN);

            g_usb_dev_state = USB_DEV_CONFIGURED_STATE;
        }
    }
    else if (g_usb_dev_state == USB_DEV_CONFIGURED_STATE)
    {
        /* Send Ack to Host*/
        usbs_status_phase(CONTROL, EP_DIR_IN);
        if (cfg == 0)
        {
            g_usb_dev_state = USB_DEV_ADDRESSED_STATE;
        }

        diag_printf("Set configuration at configured state!\n");
    }
    else
    {
        /* Send STALL Handshake  */
        endpoint_stall(0, EP_DIR_IN);
        diag_printf("Set configuration at invalid state!\n");
    }
}

static void setup_received_irq(struct usb_ctrlrequest *setup)
{
    switch (setup->bRequest)
    {
        case USB_REQ_GET_STATUS:
            diag_printf("USB_REQ_GET_STATUS\n");
            break;
        case USB_REQ_CLEAR_FEATURE:
            diag_printf("USB_REQ_CLEAR_FEATURE\n");
            break;
        case USB_REQ_SET_FEATURE:
            diag_printf("USB_REQ_SET_FEATURE\n");
            break;
        case USB_REQ_SET_ADDRESS:
            diag_printf("USB_REQ_SET_ADDRESS\n");
            handle_set_address(setup);
            break;
        case USB_REQ_GET_DESCRIPTOR:
            diag_printf("USB_REQ_GET_DESCRIPTOR\n");
            handle_get_descriptor(setup);
            break;
        case USB_REQ_SET_DESCRIPTOR:
            diag_printf("USB_REQ_SET_DESCRIPTOR\n");
            break;
        case USB_REQ_GET_CONFIGURATION:
            diag_printf("USB_REQ_GET_CONFIGURATION\n");
            break;
        case USB_REQ_SET_CONFIGURATION:
            diag_printf("USB_REQ_SET_CONFIGURATION\n");
            handle_set_configuration(setup);
            break;
        case USB_REQ_GET_INTERFACE:
            diag_printf("USB_REQ_GET_INTERFACE\n");
            break;
        case USB_REQ_SET_INTERFACE:
            diag_printf("USB_REQ_SET_INTERFACE\n");
            break;
        case USB_REQ_SYNCH_FRAME:
            diag_printf("USB_REQ_SYNCH_FRAME\n");
            break;
        default:
            break;
    }
}

static void ep1_rx_data(void)
{
    volatile struct ep_queue_head* qh =
        (struct ep_queue_head *)(ep_dqh_base_addrs + 128);
    volatile struct ep_td_struct* td =
        (struct ep_td_struct *)(ep_dtd_base_addrs + 128);

    cyg_uint32 bytes_recv = 0;

    if (g_usb_dev_state != USB_DEV_CONFIGURED_STATE)
    {
        return;
    }

    /*clear the complete status */
	dr_regs->endptcomplete |= (EPOUT_COMPLETE << 1);

    /* calculate the received data length using number of bytes left in TD */
    bytes_recv = ep1_bulk_size - ((td->size_ioc_sts >> 16) & 0x7FFF);
    if (rx_callback)
    {
        (*rx_callback)((cyg_uint8 *)ep1_bulk_buffer, bytes_recv);
    }

    /* Prepare TD for next bulk out transfer */
    td->next_td_ptr = (((cyg_uint32)td + 0x20) & 0xFFFFFFE0) | 1;
    td->size_ioc_sts = ((ep1_bulk_size & 0x7FFF) << 16) | (1 <<15 ) | BIT7;
    td->buff_ptr0 = ep1_bulk_buffer;
    td->buff_ptr1 = ep1_bulk_buffer + BULK_TD_BUFFER_PAGE_SIZE;
    td->buff_ptr2 = ep1_bulk_buffer + BULK_TD_BUFFER_PAGE_SIZE * 2;

    /* 1. write dQH next ptr and dQH terminate bit to 0 */
    *(volatile cyg_uint32*)((cyg_uint32)qh + 0x8) = (cyg_uint32)td;

    /* 2. clear active & halt bit in dQH */
    *(volatile cyg_uint32*)((cyg_uint32)qh + 0xC) &= ~0xFF;

    /* 3. prime endpoint by writing '1' in ENDPTPRIME */
    dr_regs->endpointprime |= (EPOUT_COMPLETE << 1);
    while (dr_regs->endpointprime & (EPOUT_COMPLETE << 1));
}

static void handle_packet(void)
{
    cyg_uint32 tmp;
    if (dr_regs->endptsetupstat & EP_SETUP_STATUS_EP0)
    {
        tripwire_handler(0, (cyg_uint8 *)&local_setup_buf);
        setup_received_irq(&local_setup_buf);
    }

    /* completion of dtd */
    if (dr_regs->endptcomplete)
    {
        tmp = dr_regs->endptcomplete;

        /* clear notification bits */
        dr_regs->endptcomplete = tmp;

        if (tmp & (EPOUT_COMPLETE << 0))
        {
            diag_printf("EPOUT_COMPLETE in ep0\n");
        }
        else if (tmp & (EPOUT_COMPLETE << 1))
        {
            ep1_rx_data();
        }
        else if (tmp & (EPOUT_COMPLETE << 2))
        {
            diag_printf("EPOUT_COMPLETE in ep2\n");
        }
        else if (tmp & (EPIN_COMPLETE << 0))
        {
            diag_printf("EPIN_COMPLETE in ep0\n");
        }
        else if (tmp & (EPIN_COMPLETE << 1))
        {
            diag_printf("EPIN_COMPLETE in ep1\n");
        }
        else if (tmp & (EPIN_COMPLETE << 2))
        {
            diag_printf("EPIN_COMPLETE in ep2\n");
        }
        else
        {
            diag_printf("unknown dtd completion irq.\n");
        }
    }
}

void register_rx_callback(RxCompletionFunc func)
{
    rx_callback = func;
}

void usbs_imx_otg_download(void)
{
    cyg_uint32 tmp;

    while (1)
    {
        tmp = dr_regs->usbsts;

        /* clear notification bits */
        dr_regs->usbsts = tmp;

        if (tmp & USB_STS_INT)
        {
            handle_packet();
        }

        if (tmp & USB_STS_SOF)
        {
            /* do nothing */
        }

        if (tmp & USB_STS_PORT_CHANGE)
        {
            /* do nothing */
            diag_printf("USB_STS_PORT_CHANGE\n");
        }

        if (tmp & USB_STS_RESET)
        {
            diag_printf("USB_STS_RESET\n");
            handle_reset();
        }

        if (tmp & USB_STS_SUSPEND)
        {
            diag_printf("USB_STS_SUSPEND\n");
        }

        if (tmp & (USB_STS_ERR | USB_STS_SYS_ERR))
        {
            diag_printf("USB_STS_ERR | USB_STS_SYS_ERR\n");
        }
    }
}
