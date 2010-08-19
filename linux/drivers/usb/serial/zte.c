/*
  USB Driver for GSM modems

  Copyright (C) 2005  Matthias Urlichs <smurf@smurf.noris.de>

*/

#define DRIVER_VERSION "vautosus_V1.0.0B01"
#define DRIVER_AUTHOR "ZTE Inc.Zhao Ming"
#define DRIVER_DESC "USB Driver for GSM modems---add SS feature"

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

/* Function prototypes */
static int  zte_probe(struct usb_serial *serial,
			const struct usb_device_id *id);
static int  zte_open(struct tty_struct *tty, struct usb_serial_port *port,
							struct file *filp);
static void zte_close(struct tty_struct *tty, struct usb_serial_port *port,
							struct file *filp);
static int  zte_startup(struct usb_serial *serial);
static void zte_shutdown(struct usb_serial *serial);
static int  zte_write_room(struct tty_struct *tty);

static void zte_instat_callback(struct urb *urb);

static int zte_write(struct tty_struct *tty, struct usb_serial_port *port,
			const unsigned char *buf, int count);
static int  zte_chars_in_buffer(struct tty_struct *tty);
static void zte_set_termios(struct tty_struct *tty,
			struct usb_serial_port *port, struct ktermios *old);
static int  zte_tiocmget(struct tty_struct *tty, struct file *file);
static int  zte_tiocmset(struct tty_struct *tty, struct file *file,
				unsigned int set, unsigned int clear);
static int  zte_send_setup(struct tty_struct *tty, struct usb_serial_port *port);
static int  zte_suspend(struct usb_serial *serial, pm_message_t message);
static int  zte_resume(struct usb_serial *serial);

/* Vendor and product IDs */

#define ONDA_VENDOR_ID				0x19d2
#define ONDA_PRODUCT_MSA501HS			0x0001
#define ONDA_PRODUCT_ET502HS			0x0002


/* ZTE PRODUCTS */
#define ZTE_VENDOR_ID				0x19d2
#define ZTE_PRODUCT_MF628			0x0015
#define ZTE_PRODUCT_MF626			0x0031
#define ZTE_PRODUCT_CDMA_TECH			0xfffe



static struct usb_device_id zte_ids[] = {
	{ USB_DEVICE(ONDA_VENDOR_ID, ONDA_PRODUCT_MSA501HS) },
	{ USB_DEVICE(ONDA_VENDOR_ID, ONDA_PRODUCT_ET502HS) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0003) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0004) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0005) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0006) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0007) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0008) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0009) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000a) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000b) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000c) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000d) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000e) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x000f) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0010) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0011) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0012) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0013) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0014) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0015) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0016) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0017) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0018) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0019) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0020) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0021) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0022) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0023) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0024) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0025) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0026) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0027) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0028) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0029) },
	{ USB_DEVICE(ZTE_VENDOR_ID, ZTE_PRODUCT_MF626) },
	{ USB_DEVICE(ZTE_VENDOR_ID, ZTE_PRODUCT_MF628) },
	{ USB_DEVICE(ZTE_VENDOR_ID, ZTE_PRODUCT_CDMA_TECH) },
	{ USB_DEVICE(ONDA_VENDOR_ID, 0x0016) },
	{ } /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, zte_ids);

static struct usb_driver zte_driver = {
	.name       = "zte",
	.probe      = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.suspend    = usb_serial_suspend,
	.resume     = usb_serial_resume,
	.id_table   = zte_ids,
	.no_dynamic_id = 	1,
	.supports_autosuspend =	1,
};

/* The card has three separate interfaces, which the serial driver
 * recognizes separately, thus num_port=1.
 */

static struct usb_serial_driver zte_1port_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"zte1",
	},
	.description       = "GSM modem (1-port)",
	.usb_driver        = &zte_driver,
	.id_table          = zte_ids,
	.num_ports         = 1,
	.probe             = zte_probe,
	.open              = zte_open,
	.close             = zte_close,
	.write             = zte_write,
	.write_room        = zte_write_room,
	.chars_in_buffer   = zte_chars_in_buffer,
	.set_termios       = zte_set_termios,
	.tiocmget          = zte_tiocmget,
	.tiocmset          = zte_tiocmset,
	.attach            = zte_startup,
	.shutdown          = zte_shutdown,
	.read_int_callback = zte_instat_callback,
	.suspend           = zte_suspend,
	.resume            = zte_resume,
};

static int debug;

/* per port private data */

#define N_IN_URB 4
#define N_OUT_URB 4
#define IN_BUFLEN 4096
#define OUT_BUFLEN 4096

struct zte_intf_private {
	spinlock_t susp_lock;
	unsigned int suspended:1;
	int in_flight;
};

struct zte_port_private {
	/* Input endpoints and buffer for this port */
	struct urb *in_urbs[N_IN_URB];
	u8 *in_buffer[N_IN_URB];
	/* Output endpoints and buffer for this port */
	struct urb *out_urbs[N_OUT_URB];
	u8 *out_buffer[N_OUT_URB];
	unsigned long out_busy;		/* Bit vector of URBs in use */
	int opened;
	struct usb_anchor delayed;	

	/* Settings for the port */
	int rts_state;	/* Handshaking pins (outputs) */
	int dtr_state;
	int cts_state;	/* Handshaking pins (inputs) */
	int dsr_state;
	int dcd_state;
	int ri_state;

	unsigned long tx_start_time[N_OUT_URB];
};

/* Functions used by new usb-serial code. */
static int __init zte_init(void)
{
	int retval;
	retval = usb_serial_register(&zte_1port_device);
	if (retval)
		goto failed_1port_device_register;
	retval = usb_register(&zte_driver);
	if (retval)
		goto failed_driver_register;

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");

	return 0;

failed_driver_register:
	usb_serial_deregister(&zte_1port_device);
failed_1port_device_register:
	return retval;
}

static void __exit zte_exit(void)
{
	usb_deregister(&zte_driver);
	usb_serial_deregister(&zte_1port_device);
}

module_init(zte_init);
module_exit(zte_exit);

static int zte_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct zte_intf_private *data;


	data = serial->private = kzalloc(sizeof(struct zte_intf_private), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	spin_lock_init(&data->susp_lock);
	return 0;
}

static void zte_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	dbg("%s", __func__);
	/* Doesn't support zte setting */
	tty_termios_copy_hw(tty->termios, old_termios);
	zte_send_setup(tty, port);
}

static int zte_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct usb_serial_port *port = tty->driver_data;
	unsigned int value;
	struct zte_port_private *portdata;

	portdata = usb_get_serial_port_data(port);

	value = ((portdata->rts_state) ? TIOCM_RTS : 0) |
		((portdata->dtr_state) ? TIOCM_DTR : 0) |
		((portdata->cts_state) ? TIOCM_CTS : 0) |
		((portdata->dsr_state) ? TIOCM_DSR : 0) |
		((portdata->dcd_state) ? TIOCM_CAR : 0) |
		((portdata->ri_state) ? TIOCM_RNG : 0);

	return value;
}

static int zte_tiocmset(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct zte_port_private *portdata;

	portdata = usb_get_serial_port_data(port);

	/* FIXME: what locks portdata fields ? */
	if (set & TIOCM_RTS)
		portdata->rts_state = 1;
	if (set & TIOCM_DTR)
		portdata->dtr_state = 1;

	if (clear & TIOCM_RTS)
		portdata->rts_state = 0;
	if (clear & TIOCM_DTR)
		portdata->dtr_state = 0;
	return zte_send_setup(tty, port);
}

/* Write */
static int zte_write(struct tty_struct *tty, struct usb_serial_port *port,
			const unsigned char *buf, int count)
{
	struct zte_port_private *portdata;
	struct zte_intf_private *intfdata;
	int i;
	int left, todo;
	struct urb *this_urb = NULL; /* spurious */
	int err;
	unsigned long flags;

	portdata = usb_get_serial_port_data(port);
	intfdata = port->serial->private;

	dbg("%s: write (%d chars)", __func__, count);

	i = 0;
	left = count;
	for (i = 0; left > 0 && i < N_OUT_URB; i++) {
		todo = left;
		if (todo > OUT_BUFLEN)
			todo = OUT_BUFLEN;

		this_urb = portdata->out_urbs[i];
		if (test_and_set_bit(i, &portdata->out_busy)) {
			if (time_before(jiffies,
					portdata->tx_start_time[i] + 10 * HZ))
				continue;
			usb_unlink_urb(this_urb);
			continue;
		}
		if (this_urb->status != 0)
			dbg("usb_write %p failed (err=%d)",
				this_urb, this_urb->status);

		dbg("%s: endpoint %d buf %d", __func__,
			usb_pipeendpoint(this_urb->pipe), i);

		err = usb_autopm_get_interface_async(port->serial->interface);
		if (err < 0)
			break;
		/* send the data */
		memcpy(this_urb->transfer_buffer, buf, todo);
		this_urb->transfer_buffer_length = todo;

		this_urb->dev = port->serial->dev;
	
		spin_lock_irqsave(&intfdata->susp_lock, flags);
		if (intfdata->suspended) {
			usb_anchor_urb(this_urb, &portdata->delayed);
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
		} else {
			intfdata->in_flight++;
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
			err = usb_submit_urb(this_urb, GFP_ATOMIC);
			if (err) {
				dbg("usb_submit_urb %p (write bulk) failed "
					"(%d)", this_urb, err);
				clear_bit(i, &portdata->out_busy);
				spin_lock_irqsave(&intfdata->susp_lock, flags);
				intfdata->in_flight--;
				spin_unlock_irqrestore(&intfdata->susp_lock, flags);	
				continue;
			}
		}
		portdata->tx_start_time[i] = jiffies;
		buf += todo;
		left -= todo;
	}

	count -= left;
	dbg("%s: wrote (did %d)", __func__, count);
	return count;
}

static void zte_indat_callback(struct urb *urb)
{
	int err;
	int endpoint;
	struct usb_serial_port *port;
	struct tty_struct *tty;
	unsigned char *data = urb->transfer_buffer;
	int status = urb->status;

	dbg("%s: %p", __func__, urb);

	endpoint = usb_pipeendpoint(urb->pipe);
	port =  urb->context;

	if (status) {
		dbg("%s: nonzero status: %d on endpoint %02x.",
		    __func__, status, endpoint);
	} else {
		tty = tty_port_tty_get(&port->port);
		if (urb->actual_length) {
			tty_buffer_request_room(tty, urb->actual_length);
			tty_insert_flip_string(tty, data, urb->actual_length);
			tty_flip_buffer_push(tty);
		} else 
			dbg("%s: empty read urb received", __func__);
		tty_kref_put(tty);

		/* Resubmit urb so we continue receiving */
		if (port->port.count && status != -ESHUTDOWN) {
			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err)
				printk(KERN_ERR "%s: resubmit read urb failed. "
					"(%d)", __func__, err);
			else
				usb_mark_last_busy(port->serial->dev);
		}
	}
	return;
}

static void zte_outdat_callback(struct urb *urb)
{
	struct usb_serial_port *port;
	struct zte_port_private *portdata;
	struct zte_intf_private *intfdata;
	int i;

	dbg("%s", __func__);

	port =  urb->context;
	intfdata = port->serial->private;

	usb_serial_port_softint(port);
	usb_autopm_put_interface_async(port->serial->interface);
	portdata = usb_get_serial_port_data(port);
	spin_lock(&intfdata->susp_lock);
	intfdata->in_flight--;
	spin_unlock(&intfdata->susp_lock);


	for (i = 0; i < N_OUT_URB; ++i) {
		if (portdata->out_urbs[i] == urb) {
			smp_mb__before_clear_bit();
			clear_bit(i, &portdata->out_busy);
			break;
		}
	}
}

static void zte_instat_callback(struct urb *urb)
{
	int err;
	int status = urb->status;
	struct usb_serial_port *port =  urb->context;
	struct zte_port_private *portdata = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;

	dbg("%s", __func__);
	dbg("%s: urb %p port %p has data %p", __func__, urb, port, portdata);

	if (status == 0) {
		struct usb_ctrlrequest *req_pkt =
				(struct usb_ctrlrequest *)urb->transfer_buffer;

		if (!req_pkt) {
			dbg("%s: NULL req_pkt\n", __func__);
			return;
		}
		if ((req_pkt->bRequestType == 0xA1) &&
				(req_pkt->bRequest == 0x20)) {
			int old_dcd_state;
			unsigned char signals = *((unsigned char *)
					urb->transfer_buffer +
					sizeof(struct usb_ctrlrequest));

			dbg("%s: signal x%x", __func__, signals);

			old_dcd_state = portdata->dcd_state;
			portdata->cts_state = 1;
			portdata->dcd_state = ((signals & 0x01) ? 1 : 0);
			portdata->dsr_state = ((signals & 0x02) ? 1 : 0);
			portdata->ri_state = ((signals & 0x08) ? 1 : 0);

			if (old_dcd_state && !portdata->dcd_state) {
				struct tty_struct *tty =
						tty_port_tty_get(&port->port);
				if (tty && !C_CLOCAL(tty))
					tty_hangup(tty);
				tty_kref_put(tty);
			}
		} else {
			dbg("%s: type %x req %x", __func__,
				req_pkt->bRequestType, req_pkt->bRequest);
		}
	} else
		dbg("%s: error %d", __func__, status);

	/* Resubmit urb so we continue receiving IRQ data */
	if (status != -ESHUTDOWN) {
		urb->dev = serial->dev;
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err)
			dbg("%s: resubmit intr urb failed. (%d)",
				__func__, err);
	}
}

static int zte_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct zte_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;

	portdata = usb_get_serial_port_data(port);


	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		if (this_urb && !test_bit(i, &portdata->out_busy))
			data_len += OUT_BUFLEN;
	}

	dbg("%s: %d", __func__, data_len);
	return data_len;
}

static int zte_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct zte_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;

	portdata = usb_get_serial_port_data(port);

	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		/* FIXME: This locking is insufficient as this_urb may
		   go unused during the test */
		if (this_urb && test_bit(i, &portdata->out_busy))
			data_len += this_urb->transfer_buffer_length;
	}
	dbg("%s: %d", __func__, data_len);
	return data_len;
}

static int zte_open(struct tty_struct *tty,
			struct usb_serial_port *port, struct file *filp)
{
	struct zte_port_private *portdata;
	struct zte_intf_private *intfdata;
	struct usb_serial *serial = port->serial;
	int i, err;
	struct urb *urb;

	portdata = usb_get_serial_port_data(port);
	intfdata = port->serial->private;

	dbg("%s", __func__);

	/* Set some sane defaults */
	portdata->rts_state = 1;
	portdata->dtr_state = 1;

	/* Reset low level data toggle and start reading from endpoints */
	for (i = 0; i < N_IN_URB; i++) {
		urb = portdata->in_urbs[i];
		if (!urb)
			continue;
		if (urb->dev != serial->dev) {
			dbg("%s: dev %p != %p", __func__,
				urb->dev, serial->dev);
			continue;
		}

		/*
		 * make sure endpoint data toggle is synchronized with the
		 * device
		 */
		usb_clear_halt(urb->dev, urb->pipe);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			dbg("%s: submit urb %d failed (%d) %d",
				__func__, i, err,
				urb->transfer_buffer_length);
		}
	}

	/* Reset low level data toggle on out endpoints */
	for (i = 0; i < N_OUT_URB; i++) {
		urb = portdata->out_urbs[i];
		if (!urb)
			continue;
		urb->dev = serial->dev;
		/* usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
				usb_pipeout(urb->pipe), 0); */
	}

	zte_send_setup(tty,port);
	
	serial->interface->needs_remote_wakeup = 1;
	spin_lock_irq(&intfdata->susp_lock);
	portdata->opened = 1;
	spin_unlock_irq(&intfdata->susp_lock);
	usb_autopm_put_interface(serial->interface);

	return 0;

}














static void zte_close(struct tty_struct *tty,
			struct usb_serial_port *port, struct file *filp)
{
	int i;
	struct usb_serial *serial = port->serial;
	struct zte_port_private *portdata;
	struct zte_intf_private *intfdata = port->serial->private;

	dbg("%s", __func__);
	portdata = usb_get_serial_port_data(port);

	portdata->rts_state = 0;
	portdata->dtr_state = 0;

	if (serial->dev) {
		/* Stop reading/writing urbs */
		spin_lock_irq(&intfdata->susp_lock);
		portdata->opened = 0;
		spin_unlock_irq(&intfdata->susp_lock);
		mutex_lock(&serial->disc_mutex);
		if (!serial->disconnected)
			zte_send_setup(tty, port);
		mutex_unlock(&serial->disc_mutex);

		/* Stop reading/writing urbs */
		for (i = 0; i < N_IN_URB; i++)
			usb_kill_urb(portdata->in_urbs[i]);
		for (i = 0; i < N_OUT_URB; i++)
			usb_kill_urb(portdata->out_urbs[i]);
		usb_autopm_get_interface(serial->interface);
		serial->interface->needs_remote_wakeup = 0;
	}
	tty_port_tty_set(&port->port, NULL);
}

/* Helper functions used by zte_setup_urbs */
static struct urb *zte_setup_urb(struct usb_serial *serial, int endpoint,
		int dir, void *ctx, char *buf, int len,
		void (*callback)(struct urb *))
{
	struct urb *urb;

	if (endpoint == -1)
		return NULL;		/* endpoint not needed */

	urb = usb_alloc_urb(0, GFP_KERNEL);		/* No ISO */
	if (urb == NULL) {
		dbg("%s: alloc for endpoint %d failed.", __func__, endpoint);
		return NULL;
	}

		/* Fill URB using supplied data. */
	usb_fill_bulk_urb(urb, serial->dev,
		      usb_sndbulkpipe(serial->dev, endpoint) | dir,
		      buf, len, callback, ctx);

	return urb;
}

/* Setup urbs */
static void zte_setup_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct zte_port_private *portdata;

	dbg("%s", __func__);

	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		/* Do indat endpoints first */
		for (j = 0; j < N_IN_URB; ++j) {
			portdata->in_urbs[j] = zte_setup_urb(serial,
					port->bulk_in_endpointAddress,
					USB_DIR_IN, port,
					portdata->in_buffer[j],
					IN_BUFLEN, zte_indat_callback);
		}

		/* outdat endpoints */
		for (j = 0; j < N_OUT_URB; ++j) {
			portdata->out_urbs[j] = zte_setup_urb(serial,
					port->bulk_out_endpointAddress,
					USB_DIR_OUT, port,
					portdata->out_buffer[j],
					OUT_BUFLEN, zte_outdat_callback);
		}
	}
}


/** send RTS/DTR state to the port.
 *
 * This is exactly the same as SET_CONTROL_LINE_STATE from the PSTN
 * CDC.
*/
static int zte_send_setup(struct tty_struct *tty,
						struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct zte_port_private *portdata;
	int ifNum = serial->interface->cur_altsetting->desc.bInterfaceNumber;
	dbg("%s", __func__);

	portdata = usb_get_serial_port_data(port);

	if (tty) {
		int val = 0;
		if (portdata->dtr_state)
			val |= 0x01;
		if (portdata->rts_state)
			val |= 0x02;

		return usb_control_msg(serial->dev,
			usb_rcvctrlpipe(serial->dev, 0),
			0x22, 0x21, val, ifNum, NULL, 0, USB_CTRL_SET_TIMEOUT);
	}
	return 0;
}

static int zte_startup(struct usb_serial *serial)
{
	int i, j, err;
	struct usb_serial_port *port;
	struct zte_port_private *portdata;
	u8 *buffer;

	dbg("%s", __func__);

	/* Now setup per port private data */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
		if (!portdata) {
			dbg("%s: kmalloc for zte_port_private (%d) failed!.",
					__func__, i);
			return 1;
		}
		init_usb_anchor(&portdata->delayed);

		for (j = 0; j < N_IN_URB; j++) {
			buffer = (u8 *)__get_free_page(GFP_KERNEL);
			if (!buffer)
				goto bail_out_error;
			portdata->in_buffer[j] = buffer;
		}

		for (j = 0; j < N_OUT_URB; j++) {
			buffer = kmalloc(OUT_BUFLEN, GFP_KERNEL);
			if (!buffer)
				goto bail_out_error2;
			portdata->out_buffer[j] = buffer;
		}

		usb_set_serial_port_data(port, portdata);

		if (!port->interrupt_in_urb)
			continue;
		err = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
		if (err)
			dbg("%s: submit irq_in urb failed %d",
				__func__, err);
	}
	zte_setup_urbs(serial);
	return 0;

bail_out_error2:
	for (j = 0; j < N_OUT_URB; j++)
		kfree(portdata->out_buffer[j]);
bail_out_error:
	for (j = 0; j < N_IN_URB; j++)
		if (portdata->in_buffer[j])
			free_page((unsigned long)portdata->in_buffer[j]);
	kfree(portdata);
	return 1;
}

static void stop_read_write_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct zte_port_private *portdata;

	/* Stop reading/writing urbs */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);
		for (j = 0; j < N_IN_URB; j++)
			usb_kill_urb(portdata->in_urbs[j]);
		for (j = 0; j < N_OUT_URB; j++)
			usb_kill_urb(portdata->out_urbs[j]);
	}
}
static void zte_shutdown(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct zte_port_private *portdata;

	dbg("%s", __func__);

	/* Stop reading/writing urbs */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);
		for (j = 0; j < N_IN_URB; j++)
			usb_kill_urb(portdata->in_urbs[j]);
		for (j = 0; j < N_OUT_URB; j++)
			usb_kill_urb(portdata->out_urbs[j]);
	}

	/* Now free them */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		for (j = 0; j < N_IN_URB; j++) {
			if (portdata->in_urbs[j]) {
				usb_free_urb(portdata->in_urbs[j]);
				free_page((unsigned long)
					portdata->in_buffer[j]);
				portdata->in_urbs[j] = NULL;
			}
		}
		for (j = 0; j < N_OUT_URB; j++) {
			if (portdata->out_urbs[j]) {
				usb_free_urb(portdata->out_urbs[j]);
				kfree(portdata->out_buffer[j]);
				portdata->out_urbs[j] = NULL;
			}
		}
	}

	/* Now free per port private data */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		kfree(usb_get_serial_port_data(port));
	}
}

static int zte_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct zte_intf_private *intfdata = serial->private;
	int b;

	dbg("%s entered", __func__);

	if (serial->dev->auto_pm) {
		spin_lock_irq(&intfdata->susp_lock);
		b = intfdata->in_flight;
		spin_unlock_irq(&intfdata->susp_lock);
			
		if (b)
			return -EBUSY;
	}

	spin_lock_irq(&intfdata->susp_lock);
	intfdata->suspended = 1;
	spin_unlock_irq(&intfdata->susp_lock);
	stop_read_write_urbs(serial);

	return 0;
}

static void play_delayed(struct usb_serial_port *port)
{
	struct zte_intf_private *data;
	struct zte_port_private *portdata;
	struct urb *urb;
	int err;

	portdata = usb_get_serial_port_data(port);
	data = port->serial->private;
	while ((urb = usb_get_from_anchor(&portdata->delayed))) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (!err)
			data->in_flight++;
	}
}

static int zte_resume(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct zte_intf_private *intfdata = serial->private;
	struct zte_port_private *portdata;
	struct urb *urb;
	int err = 0;

	dbg("%s entered", __func__);
	/* get the interrupt URBs resubmitted unconditionally */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		if (!port->interrupt_in_urb) {
			dbg("%s: No interrupt URB for port %d\n", __func__, i);
			continue;
		}
		port->interrupt_in_urb->dev = serial->dev;
		err = usb_submit_urb(port->interrupt_in_urb, GFP_NOIO);
		dbg("Submitted interrupt URB for port %d (result %d)", i, err);
		if (err < 0) {
			err("%s: Error %d for interrupt URB of port%d",
				 __func__, err, i);
			goto err_out;
		}
	}

	for (i = 0; i < serial->num_ports; i++) {
		/* walk all ports */
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		/* skip closed ports */
		spin_lock_irq(&intfdata->susp_lock);
		if (!portdata->opened) {
			spin_unlock_irq(&intfdata->susp_lock);
			continue;
		}

		for (j = 0; j < N_IN_URB; j++) {
			urb = portdata->in_urbs[j];
			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err < 0) {
				err("%s: Error %d for bulk URB %d",
					 __func__, err, i);
				spin_unlock_irq(&intfdata->susp_lock);
				goto err_out;
			}
		}
		play_delayed(port);
		spin_unlock_irq(&intfdata->susp_lock);
	}
	spin_lock_irq(&intfdata->susp_lock);
	intfdata->suspended = 0;
	spin_unlock_irq(&intfdata->susp_lock);
err_out:
	return err;
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug messages");
