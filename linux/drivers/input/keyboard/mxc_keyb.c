/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_keyb.c
 *
 * @brief Driver for the Freescale Semiconductor MXC keypad port.
 *
 * The keypad driver is designed as a standard Input driver which interacts
 * with low level keypad port hardware. Upon opening, the Keypad driver
 * initializes the keypad port. When the keypad interrupt happens the driver
 * calles keypad polling timer and scans the keypad matrix for key
 * press/release. If all key press/release happened it comes out of timer and
 * waits for key press interrupt. The scancode for key press and release events
 * are passed to Input subsytem.
 *
 * @ingroup keypad
 */

/*!
 * Comment KPP_DEBUG to disable debug messages
 */
#define KPP_DEBUG        0

#if KPP_DEBUG
#define	DEBUG
#include <linux/kernel.h>
#endif

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/hardware.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/kbd_kern.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/mach/keypad.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <asm/arch/gpio.h>
#include <asm/unaligned.h>
#include <../arch/arm/mach-mx3/iomux.h>
#include <../arch/arm/mach-mx3/mx31_pins.h>

/*
 * Module header file
 */
#include "mxc_keyb.h"

/*!
 * This structure holds the keypad private data structure.
 */
static struct keypad_priv kpp_dev;

/*! Indicates if the key pad device is enabled. */
static unsigned int key_pad_enabled;

/*! Input device structure. */
static struct input_dev *mxckbd_dev = NULL;

/*! KPP clock handle. */
static struct clk *kpp_clk;

/*! This static variable indicates whether a key event is pressed/released. */
static unsigned short KPress;

/*! cur_rcmap and prev_rcmap array is used to detect key press and release. */
static unsigned short *cur_rcmap;	/* max 64 bits (8x8 matrix) */
static unsigned short *prev_rcmap;

/*!
 * Debounce polling period(10ms) in system ticks.
 */
static unsigned short KScanRate = (10 * HZ) / 1000;

static struct keypad_data *keypad;

static int has_leaning_key;
/*!
 * These arrays are used to store press and release scancodes.
 */
static short **press_scancode;
static short **release_scancode;

static const unsigned short *mxckpd_keycodes;
static unsigned short mxckpd_keycodes_size;

#define press_left_code     30
#define press_right_code    29
#define press_up_code       28
#define press_down_code     27

#define rel_left_code       158
#define rel_right_code      157
#define rel_up_code         156
#define rel_down_code       155

/*!
 * These functions are used to configure and the GPIO pins for keypad to
 * activate and deactivate it.
 */
extern void gpio_keypad_active(void);
extern void gpio_keypad_inactive(void);
extern u32 get_gpio_irq(iomux_pin_name_t, unsigned long*);

struct gpio_key_map
{
	iomux_pin_name_t gpio_pin;
	u16				 key_code;
	int				 last_state;
	u32				 irq;
};

static struct gpio_key_map gpio_keys[] =
{
	{ MX31_PIN_GPIO1_1,  KEY_F24 },    /* Wacom touch panel enable/disable */
	{ MX31_PIN_KEY_ROW5, KEY_POWER },  /* Power key interrupt */
	{ MX31_PIN_GPIO1_0,  KEY_F23 },    /* Wacom pen approaching interrupt */
#if defined(CONFIG_ENABLE_JACK_DETECT)
	{ MX31_PIN_DTR_DCE1, KEY_F22 },    /* Jack status */
#endif
	{ MX31_PIN_KEY_ROW4, KEY_F21 },    /* USB connection indicator */
	{ MX31_PIN_SRXD5,    KEY_F20 },    /* 3G power switch */
};

static void (*jack_handler)(int) = NULL;
void register_jack_notify(void (*handler)(int))
{
	jack_handler = handler;
}
EXPORT_SYMBOL(register_jack_notify);

static int lds6107_i2c_read(struct i2c_client* client, u16 reg, u16* rt_value)
{
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
	{
		return -ENODEV;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	data[0] = reg >> 8;
	data[1] = reg & 0xFF;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
	{
		msg->len = 2;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0)
		{
			*rt_value = get_unaligned_be16(data);
			return 0;
		}
	}
	return err;
}

static int lds6107_i2c_write(struct i2c_client* client, u16 reg, u16 data)
{
	struct i2c_msg msg[1];
	u8 buf[4];

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = buf;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = data >> 8;
	buf[3] = data & 0xFF;
	return i2c_transfer(client->adapter, msg, 1);
}

static void deferred_scan(struct work_struct *work);
static DECLARE_DELAYED_WORK(notify_work, deferred_scan);
static struct i2c_client* lds6107_client = NULL;
static void deferred_scan(struct work_struct *work)
{
	u16 val;
	static int left_move = 0;
	static int right_move = 0;

	if (lds6107_i2c_read(lds6107_client, 0x4B, &val) < 0)
	{
		return;
	}

	/* Check bit 7: Position Interrupt Status */
	if (val & 0x80)
	{
		/* Check bit 6: movement status */
		if (val & 0x40)
		{
			/* Get movement direction from bit 5 */
			if (val & 0x0020)
				right_move++;
			else
				left_move++;
		}

		schedule_delayed_work(&notify_work, KScanRate);
	}
	else
	{
		/* The finger has left the sensor */
		if (left_move + right_move > 0)
		{
			unsigned int key_code;
			key_code = left_move > right_move ? KEY_PAGEDOWN : KEY_PAGEUP;
			printk("Sensor key: %d\n", key_code);
			input_event(mxckbd_dev, EV_KEY, key_code, 1);
			input_event(mxckbd_dev, EV_KEY, key_code, 0);
		}

		left_move = right_move = 0;

		/* Disable scan timer and re-enable touch interrupt again */
		enable_irq(IOMUX_TO_IRQ(MX31_PIN_CAPTURE));
	}
}

static irqreturn_t slider_interrupt_handler(int irq, void* data)
{
	disable_irq(irq);

	/* Start timer to scan touch sensor every 10ms */
	schedule_delayed_work(&notify_work, 0);
	return IRQ_RETVAL(1);
}

static irqreturn_t mxc_gpio_interrupt(int irq, void* data)
{
	int i = 0, val;

	// First check if this is a press or release event.
	for (i = 0; i < ARRAY_SIZE(gpio_keys); i++)
	{
		if (gpio_keys[i].irq == irq)
		{
			break;
		}
	}

	val = mxc_get_gpio_datain(gpio_keys[i].gpio_pin);
	if (val != gpio_keys[i].last_state)
	{
		gpio_keys[i].last_state = val;
		if (val == 1)
		{
			// The gpio pin is now in high electrical level
			// A gpio key press event.
			input_event(mxckbd_dev, EV_KEY, gpio_keys[i].key_code, 1);
			set_irq_type(gpio_keys[i].irq, IRQF_TRIGGER_FALLING);
		}
		else
		{
			if (gpio_keys[i].gpio_pin == MX31_PIN_SRXD5)
			{
				mxc_request_iomux(MX31_PIN_STXD5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
				mxc_set_gpio_direction(MX31_PIN_STXD5, 0);
				mxc_set_gpio_dataout(MX31_PIN_STXD5, 0);
				mxc_free_iomux(MX31_PIN_STXD5, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
			}

			input_event(mxckbd_dev, EV_KEY, gpio_keys[i].key_code, 0);
			set_irq_type(gpio_keys[i].irq, IRQF_TRIGGER_RISING);
		}

#if defined(CONFIG_ENABLE_JACK_DETECT)
		if (gpio_keys[i].gpio_pin == MX31_PIN_DTR_DCE1)
		{
			if (jack_handler)
				(*jack_handler)(val);
		}
#endif
	}

	return IRQ_RETVAL(1);
}

/*!
 * Initialize gpio keys
 */
static int init_gpio_keys(void)
{
	int i = 0, retval = 0;
	unsigned int irq = 0;
	unsigned long flag = 0;

	for (i = 0; i < ARRAY_SIZE(gpio_keys); i++)
	{
		irq = get_gpio_irq(gpio_keys[i].gpio_pin, &flag);

		retval = request_irq(irq, mxc_gpio_interrupt, IRQF_DISABLED | flag, MOD_NAME, MOD_NAME);
		if (retval)
		{
			pr_debug("KPP: request_irq(%d) returned error %d\n", irq, retval);
			return retval;
		}

		if (flag == IRQF_TRIGGER_FALLING)
		{
			__set_bit(gpio_keys[i].key_code, mxckbd_dev->key);
			gpio_keys[i].last_state = 1;
		}
		else
		{
			gpio_keys[i].last_state = 0;
		}

		gpio_keys[i].irq = irq;
	}

	return retval;
}

/*!
 * This function is called for generating scancodes for key press and
 * release on keypad for the board.
 *
 *  @param   row        Keypad row pressed on the keypad matrix.
 *  @param   col        Keypad col pressed on the keypad matrix.
 *  @param   press      Indicated key press/release.
 *
 *  @return     Key press/release Scancode.
 */
static signed short mxc_scan_matrix_leaning_key(int row, int col, int press)
{
	static unsigned first_row;
	static unsigned first_set = 0, flag = 0;
	signed short scancode = -1;

	if (press) {
		if ((3 == col) && ((3 == row) ||
				   (4 == row) || (5 == row) || (6 == row))) {
			if (first_set == 0) {
				first_set = 1;
				first_row = row;
			} else {
				first_set = 0;
				if (((first_row == 6) || (first_row == 3))
				    && ((row == 6) || (row == 3)))
					scancode = press_down_code;
				else if (((first_row == 3) || (first_row == 5))
					 && ((row == 3) || (row == 5)))
					scancode = press_left_code;
				else if (((first_row == 6) || (first_row == 4))
					 && ((row == 6) || (row == 4)))
					scancode = press_right_code;
				else if (((first_row == 4) || (first_row == 5))
					 && ((row == 4) || (row == 5)))
					scancode = press_up_code;
				KPress = 1;
				kpp_dev.iKeyState = KStateUp;
				pr_debug("Press (%d, %d) scan=%d Kpress=%d\n",
					 row, col, scancode, KPress);
			}
		} else {
			/*
			 * check for other keys only
			 * if the cursor key presses
			 * are not detected may be
			 * this needs better logic
			 */
			if ((0 == (cur_rcmap[3] & BITSET(0, 3))) &&
			    (0 == (cur_rcmap[4] & BITSET(0, 3))) &&
			    (0 == (cur_rcmap[5] & BITSET(0, 3))) &&
			    (0 == (cur_rcmap[6] & BITSET(0, 3)))) {
				scancode = ((col * kpp_dev.kpp_rows) + row);
				KPress = 1;
				kpp_dev.iKeyState = KStateUp;
				flag = 1;
				pr_debug("Press (%d, %d) scan=%d Kpress=%d\n",
					 row, col, scancode, KPress);
			}
		}
	} else {
		if ((flag == 0) && (3 == col)
		    && ((3 == row) || (4 == row) || (5 == row)
			|| (6 == row))) {
			if (first_set == 0) {
				first_set = 1;
				first_row = row;
			} else {
				first_set = 0;
				if (((first_row == 6) || (first_row == 3))
				    && ((row == 6) || (row == 3)))
					scancode = rel_down_code;
				else if (((first_row == 3) || (first_row == 5))
					 && ((row == 3) || (row == 5)))
					scancode = rel_left_code;
				else if (((first_row == 6) || (first_row == 4))
					 && ((row == 6) || (row == 4)))
					scancode = rel_right_code;
				else if (((first_row == 4) || (first_row == 5))
					 && ((row == 4) || (row == 5)))
					scancode = rel_up_code;
				KPress = 0;
				kpp_dev.iKeyState = KStateDown;
				pr_debug("Release (%d, %d) scan=%d Kpress=%d\n",
					 row, col, scancode, KPress);
			}
		} else {
			/*
			 * check for other keys only
			 * if the cursor key presses
			 * are not detected may be
			 * this needs better logic
			 */
			if ((0 == (prev_rcmap[3] & BITSET(0, 3))) &&
			    (0 == (prev_rcmap[4] & BITSET(0, 3))) &&
			    (0 == (cur_rcmap[5] & BITSET(0, 3))) &&
			    (0 == (cur_rcmap[6] & BITSET(0, 3)))) {
				scancode = ((col * kpp_dev.kpp_rows) + row) +
				    MXC_KEYRELEASE;
				KPress = 0;
				flag = 0;
				kpp_dev.iKeyState = KStateDown;
				pr_debug("Release (%d, %d) scan=%d Kpress=%d\n",
					 row, col, scancode, KPress);
			}
		}
	}
	return scancode;
}

/*!
 * This function is called to scan the keypad matrix to find out the key press
 * and key release events. Make scancode and break scancode are generated for
 * key press and key release events.
 *
 * The following scanning sequence are done for
 * keypad row and column scanning,
 * -# Write 1's to KPDR[15:8], setting column data to 1's
 * -# Configure columns as totem pole outputs(for quick discharging of keypad
 * capacitance)
 * -# Configure columns as open-drain
 * -# Write a single column to 0, others to 1.
 * -# Sample row inputs and save data. Multiple key presses can be detected on
 * a single column.
 * -# Repeat steps the above steps for remaining columns.
 * -# Return all columns to 0 in preparation for standby mode.
 * -# Clear KPKD and KPKR status bit(s) by writing to a 1,
 *    Set the KPKR synchronizer chain by writing "1" to KRSS register,
 *    Clear the KPKD synchronizer chain by writing "1" to KDSC register
 *
 * @result    Number of key pressed/released.
 */
static int mxc_kpp_scan_matrix(void)
{
	unsigned short reg_val;
	int col, row;
	short scancode = 0;
	int keycnt = 0;		/* How many keys are still pressed */

	/*
	 * wmb() linux kernel function which guarantees orderings in write
	 * operations
	 */
	wmb();

	/* save cur keypad matrix to prev */

	memcpy(prev_rcmap, cur_rcmap, kpp_dev.kpp_rows * sizeof(prev_rcmap[0]));
	memset(cur_rcmap, 0, kpp_dev.kpp_rows * sizeof(cur_rcmap[0]));

	for (col = 0; col < kpp_dev.kpp_cols; col++) {	/* Col */
		/* 2. Write 1.s to KPDR[15:8] setting column data to 1.s */
		reg_val = __raw_readw(KPDR);
		reg_val |= 0xff00;
		__raw_writew(reg_val, KPDR);

		/*
		 * 3. Configure columns as totem pole outputs(for quick
		 * discharging of keypad capacitance)
		 */
		reg_val = __raw_readw(KPCR);
		reg_val &= 0x00ff;
		__raw_writew(reg_val, KPCR);

		udelay(2);

		/*
		 * 4. Configure columns as open-drain
		 */
		reg_val = __raw_readw(KPCR);
		reg_val |= ((1 << kpp_dev.kpp_cols) - 1) << 8;
		__raw_writew(reg_val, KPCR);

		/*
		 * 5. Write a single column to 0, others to 1.
		 * 6. Sample row inputs and save data. Multiple key presses
		 * can be detected on a single column.
		 * 7. Repeat steps 2 - 6 for remaining columns.
		 */

		/* Col bit starts at 8th bit in KPDR */
		reg_val = __raw_readw(KPDR);
		reg_val &= ~(1 << (8 + col));
		__raw_writew(reg_val, KPDR);

		/* Delay added to avoid propagating the 0 from column to row
		 * when scanning. */

		udelay(5);

		/* Read row input */
		reg_val = __raw_readw(KPDR);
		for (row = 0; row < kpp_dev.kpp_rows; row++) {	/* sample row */
			if (TEST_BIT(reg_val, row) == 0) {
				cur_rcmap[row] = BITSET(cur_rcmap[row], col);
				keycnt++;
			}
		}
	}

	/*
	 * 8. Return all columns to 0 in preparation for standby mode.
	 * 9. Clear KPKD and KPKR status bit(s) by writing to a .1.,
	 * set the KPKR synchronizer chain by writing "1" to KRSS register,
	 * clear the KPKD synchronizer chain by writing "1" to KDSC register
	 */
	reg_val = 0x00;
	__raw_writew(reg_val, KPDR);
	reg_val = __raw_readw(KPDR);
	reg_val = __raw_readw(KPSR);
	reg_val |= KBD_STAT_KPKD | KBD_STAT_KPKR | KBD_STAT_KRSS |
	    KBD_STAT_KDSC;
	__raw_writew(reg_val, KPSR);

	/* Check key press status change */

	/*
	 * prev_rcmap array will contain the previous status of the keypad
	 * matrix.  cur_rcmap array will contains the present status of the
	 * keypad matrix. If a bit is set in the array, that (row, col) bit is
	 * pressed, else it is not pressed.
	 *
	 * XORing these two variables will give us the change in bit for
	 * particular row and column.  If a bit is set in XOR output, then that
	 * (row, col) has a change of status from the previous state.  From
	 * the diff variable the key press and key release of row and column
	 * are found out.
	 *
	 * If the key press is determined then scancode for key pressed
	 * can be generated using the following statement:
	 *    scancode = ((row * 8) + col);
	 *
	 * If the key release is determined then scancode for key release
	 * can be generated using the following statement:
	 *    scancode = ((row * 8) + col) + MXC_KEYRELEASE;
	 */
	for (row = 0; row < kpp_dev.kpp_rows; row++) {
		unsigned char diff;

		/*
		 * Calculate the change in the keypad row status
		 */
		diff = prev_rcmap[row] ^ cur_rcmap[row];

		for (col = 0; col < kpp_dev.kpp_cols; col++) {
			if ((diff >> col) & 0x1) {
				/* There is a status change on col */
				if ((prev_rcmap[row] & BITSET(0, col)) == 0) {
					/*
					 * Previous state is 0, so now
					 * a key is pressed
					 */
					if (has_leaning_key) {
						scancode =
						    mxc_scan_matrix_leaning_key
						    (row, col, 1);
					} else {
						scancode =
						    ((row * kpp_dev.kpp_cols) +
						     col);
						KPress = 1;
						kpp_dev.iKeyState = KStateUp;
					}
					pr_debug("Press   (%d, %d) scan=%d "
						 "Kpress=%d\n",
						 row, col, scancode, KPress);
					press_scancode[row][col] =
					    (short)scancode;
				} else {
					/*
					 * Previous state is not 0, so
					 * now a key is released
					 */
					if (has_leaning_key) {
						scancode =
						    mxc_scan_matrix_leaning_key
						    (row, col, 0);
					} else {
						scancode =
						    (row * kpp_dev.kpp_cols) +
						    col + MXC_KEYRELEASE;
						KPress = 0;
						kpp_dev.iKeyState = KStateDown;
					}

					pr_debug
					    ("Release (%d, %d) scan=%d Kpress=%d\n",
					     row, col, scancode, KPress);
					release_scancode[row][col] =
					    (short)scancode;
					keycnt++;
				}
			}
		}
	}

	/*
	 * This switch case statement is the
	 * implementation of state machine of debounce
	 * logic for key press/release.
	 * The explaination of state machine is as
	 * follows:
	 *
	 * KStateUp State:
	 * This is in intial state of the state machine
	 * this state it checks for any key presses.
	 * The key press can be checked using the
	 * variable KPress. If KPress is set, then key
	 * press is identified and switches the to
	 * KStateFirstDown state for key press to
	 * debounce.
	 *
	 * KStateFirstDown:
	 * After debounce delay(10ms), if the KPress is
	 * still set then pass scancode generated to
	 * input device and change the state to
	 * KStateDown, else key press debounce is not
	 * satisfied so change the state to KStateUp.
	 *
	 * KStateDown:
	 * In this state it checks for any key release.
	 * If KPress variable is cleared, then key
	 * release is indicated and so, switch the
	 * state to KStateFirstUp else to state
	 * KStateDown.
	 *
	 * KStateFirstUp:
	 * After debounce delay(10ms), if the KPress is
	 * still reset then pass the key release
	 * scancode to input device and change
	 * the state to KStateUp else key release is
	 * not satisfied so change the state to
	 * KStateDown.
	 */
	switch (kpp_dev.iKeyState) {
	case KStateUp:
		if (KPress) {
			/* First Down (must debounce). */
			kpp_dev.iKeyState = KStateFirstDown;
		} else {
			/* Still UP.(NO Changes) */
			kpp_dev.iKeyState = KStateUp;
		}
		break;

	case KStateFirstDown:
		if (KPress) {
			for (row = 0; row < kpp_dev.kpp_rows; row++) {
				for (col = 0; col < kpp_dev.kpp_cols; col++) {
					if ((press_scancode[row][col] != -1)) {
						/* Still Down, so add scancode */
						scancode =
						    press_scancode[row][col];
						input_event(mxckbd_dev, EV_KEY,
							    mxckpd_keycodes
							    [scancode], 1);
						if (mxckpd_keycodes[scancode] ==
						    KEY_LEFTSHIFT) {
							input_event(mxckbd_dev,
								    EV_KEY,
								    KEY_3, 1);
						}
						kpp_dev.iKeyState = KStateDown;
						press_scancode[row][col] = -1;
					}
				}
			}
		} else {
			/* Just a bounce */
			kpp_dev.iKeyState = KStateUp;
		}
		break;

	case KStateDown:
		if (KPress) {
			/* Still down (no change) */
			kpp_dev.iKeyState = KStateDown;
		} else {
			/* First Up. Must debounce */
			kpp_dev.iKeyState = KStateFirstUp;
		}
		break;

	case KStateFirstUp:
		if (KPress) {
			/* Just a bounce */
			kpp_dev.iKeyState = KStateDown;
		} else {
			for (row = 0; row < kpp_dev.kpp_rows; row++) {
				for (col = 0; col < kpp_dev.kpp_cols; col++) {
					if ((release_scancode[row][col] != -1)) {
						scancode =
						    release_scancode[row][col];
						scancode =
						    scancode - MXC_KEYRELEASE;
						input_event(mxckbd_dev, EV_KEY,
							    mxckpd_keycodes
							    [scancode], 0);
						if (mxckpd_keycodes[scancode] ==
						    KEY_LEFTSHIFT) {
							input_event(mxckbd_dev,
								    EV_KEY,
								    KEY_3, 0);
						}
						kpp_dev.iKeyState = KStateUp;
						release_scancode[row][col] = -1;
					}
				}
			}
		}
		break;

	default:
		return -EBADRQC;
		break;
	}

	return keycnt;
}

/*!
 * This function is called to start the timer for scanning the keypad if there
 * is any key press. Currently this interval is  set to 10 ms. When there are
 * no keys pressed on the keypad we return back, waiting for a keypad key
 * press interrupt.
 *
 * @param data  Opaque data passed back by kernel. Not used.
 */
static void mxc_kpp_handle_timer(unsigned long data)
{
	unsigned short reg_val;
	int i;

	if (key_pad_enabled == 0) {
		return;
	}
	if (mxc_kpp_scan_matrix() == 0) {
		/*
		 * Stop scanning and wait for interrupt.
		 * Enable press interrupt and disable release interrupt.
		 */
		__raw_writew(0x00FF, KPDR);
		reg_val = __raw_readw(KPSR);
		reg_val |= (KBD_STAT_KPKR | KBD_STAT_KPKD);
		reg_val |= KBD_STAT_KRSS | KBD_STAT_KDSC;
		__raw_writew(reg_val, KPSR);
		reg_val |= KBD_STAT_KDIE;
		reg_val &= ~KBD_STAT_KRIE;
		__raw_writew(reg_val, KPSR);

		/*
		 * No more keys pressed... make sure unwanted key codes are
		 * not given upstairs
		 */
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			memset(press_scancode[i], -1,
			       sizeof(press_scancode[0][0]) * kpp_dev.kpp_cols);
			memset(release_scancode[i], -1,
			       sizeof(release_scancode[0][0]) *
			       kpp_dev.kpp_cols);
		}
		return;
	}

	/*
	 * There are still some keys pressed, continue to scan.
	 * We shall scan again in 10 ms. This has to be tuned according
	 * to the requirement.
	 */
	kpp_dev.poll_timer.expires = jiffies + KScanRate;
	kpp_dev.poll_timer.function = mxc_kpp_handle_timer;
	add_timer(&kpp_dev.poll_timer);
}

/*!
 * This function is the keypad Interrupt handler.
 * This function checks for keypad status register (KPSR) for key press
 * and interrupt. If key press interrupt has occurred, then the key
 * press interrupt in the KPSR are disabled.
 * It then calls mxc_kpp_scan_matrix to check for any key pressed/released.
 * If any key is found to be pressed, then a timer is set to call
 * mxc_kpp_scan_matrix function for every 10 ms.
 *
 * @param   irq      The Interrupt number
 * @param   dev_id   Driver private data
 *
 * @result    The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *            returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *            \b IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxc_kpp_interrupt(int irq, void *dev_id)
{
	unsigned short reg_val;

	/* Delete the polling timer */
	del_timer(&kpp_dev.poll_timer);
	reg_val = __raw_readw(KPSR);

	/* Check if it is key press interrupt */
	if (reg_val & KBD_STAT_KPKD) {
		/*
		 * Disable key press(KDIE status bit) interrupt
		 */
		reg_val &= ~KBD_STAT_KDIE;
		__raw_writew(reg_val, KPSR);
	} else if (reg_val & KBD_STAT_KPKR) {
		/*
		 * Disable key release(KRIE status bit) interrupt
		 */
		reg_val &= ~KBD_STAT_KRIE;
		__raw_writew(reg_val, KPSR);
	} else {
		/* spurious interrupt */
		return IRQ_RETVAL(0);
	}
	/*
	 * Check if any keys are pressed, if so start polling.
	 */
	mxc_kpp_handle_timer(0);

	return IRQ_RETVAL(1);
}

/*!
 * This function is called when the keypad driver is opened.
 * Since keypad initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always return 0
 */
static int mxc_kpp_open(struct input_dev *dev)
{
	return 0;
}

/*!
 * This function is called close the keypad device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void mxc_kpp_close(struct input_dev *dev)
{
}

#ifdef CONFIG_PM
/*!
 * This function puts the Keypad controller in low-power mode/state.
 * If Keypad is enabled as a wake source(i.e. it can resume the system
 * from suspend mode), the Keypad controller doesn't enter low-power state.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_suspend(struct platform_device *pdev, pm_message_t state)
{
	unsigned short reg_val = __raw_readw(KPSR);

	/* Check if there is a key pressing down */
	if (reg_val & KBD_STAT_KPKD)
	{
		/* Enable key release interrupt */
		reg_val |= KBD_STAT_KRIE;
		__raw_writew(reg_val, KPSR);
	}
	else
	{
		/* Enable key press interrupt */
		reg_val |= KBD_STAT_KDIE;
		__raw_writew(reg_val, KPSR);
	}

	del_timer(&kpp_dev.poll_timer);

	if (device_may_wakeup(&pdev->dev)) {
	} else {
		disable_irq(keypad->irq);
		key_pad_enabled = 0;
		clk_disable(kpp_clk);
		gpio_keypad_inactive();
	}

	return 0;
}

/*!
 * This function brings the Keypad controller back from low-power state.
 * If Keypad is enabled as a wake source(i.e. it can resume the system
 * from suspend mode), the Keypad controller doesn't enter low-power state.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_resume(struct platform_device *pdev)
{
	if (device_may_wakeup(&pdev->dev)) {
	} else {
		gpio_keypad_active();
		clk_enable(kpp_clk);
		key_pad_enabled = 1;
		enable_irq(keypad->irq);
	}

	init_timer(&kpp_dev.poll_timer);

	return 0;
}

#else
#define mxc_kpp_suspend  NULL
#define mxc_kpp_resume   NULL
#endif				/* CONFIG_PM */

/*!
 * This function is called to free the allocated memory for local arrays
 */
static void mxc_kpp_free_allocated(void)
{

	int i;

	if (press_scancode) {
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			if (press_scancode[i])
				kfree(press_scancode[i]);
		}
		kfree(press_scancode);
	}

	if (release_scancode) {
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			if (release_scancode[i])
				kfree(release_scancode[i]);
		}
		kfree(release_scancode);
	}

	if (cur_rcmap)
		kfree(cur_rcmap);

	if (prev_rcmap)
		kfree(prev_rcmap);

	if (mxckbd_dev)
		input_free_device(mxckbd_dev);
}

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions.
 *
 * @return  The function returns 0 on successful registration. Otherwise returns
 *          specific error code.
 */
static int mxc_kpp_probe(struct platform_device *pdev)
{
	int i, irq;
	int retval;
	unsigned int reg_val;

	keypad = (struct keypad_data *)pdev->dev.platform_data;

	kpp_dev.kpp_cols = keypad->colmax;
	kpp_dev.kpp_rows = keypad->rowmax;
	key_pad_enabled = 0;

	/*
	 * Request for IRQ number for keypad port. The Interrupt handler
	 * function (mxc_kpp_interrupt) is called when ever interrupt occurs on
	 * keypad port.
	 */
	irq = platform_get_irq(pdev, 0);
	keypad->irq = irq;
	retval = request_irq(irq, mxc_kpp_interrupt, 0, MOD_NAME, MOD_NAME);
	if (retval) {
		pr_debug("KPP: request_irq(%d) returned error %d\n",
			 MXC_INT_KPP, retval);
		return retval;
	}

	/* Enable keypad clock */
	kpp_clk = clk_get(&pdev->dev, "kpp_clk");
	clk_enable(kpp_clk);

	/* IOMUX configuration for keypad */
	gpio_keypad_active();

	/* Configure keypad */

	/* Enable number of rows in keypad (KPCR[7:0])
	 * Configure keypad columns as open-drain (KPCR[15:8])
	 *
	 * Configure the rows/cols in KPP
	 * LSB nibble in KPP is for 8 rows
	 * MSB nibble in KPP is for 8 cols
	 */
	reg_val = __raw_readw(KPCR);
	reg_val |= (1 << keypad->rowmax) - 1;	/* LSB */
	reg_val |= ((1 << keypad->colmax) - 1) << 8;	/* MSB */
	__raw_writew(reg_val, KPCR);

	/* Write 0's to KPDR[15:8] */
	reg_val = __raw_readw(KPDR);
	reg_val &= 0x00ff;
	__raw_writew(reg_val, KPDR);

	/* Configure columns as output, rows as input (KDDR[15:0]) */
	reg_val = __raw_readw(KDDR);
	reg_val |= 0xff00;
	reg_val &= 0xff00;
	__raw_writew(reg_val, KDDR);

	reg_val = __raw_readw(KPSR);
	reg_val &= ~(KBD_STAT_KPKR | KBD_STAT_KPKD);
	reg_val |= KBD_STAT_KPKD;
	reg_val |= KBD_STAT_KRSS | KBD_STAT_KDSC;
	__raw_writew(reg_val, KPSR);
	reg_val |= KBD_STAT_KDIE;
	reg_val &= ~KBD_STAT_KRIE;
	__raw_writew(reg_val, KPSR);

	has_leaning_key = keypad->learning;
	mxckpd_keycodes = keypad->matrix;
	mxckpd_keycodes_size = keypad->rowmax * keypad->colmax + ARRAY_SIZE(gpio_keys);

	if ((keypad->matrix == (void *)0)
	    || (mxckpd_keycodes_size == 0)) {
		retval = -ENODEV;
		goto err1;
	}

	mxckbd_dev = input_allocate_device();
	if (!mxckbd_dev) {
		printk(KERN_ERR
		       "mxckbd_dev: not enough memory for input device\n");
		retval = -ENOMEM;
		goto err1;
	}

	mxckbd_dev->keycode = (void *)mxckpd_keycodes;
	mxckbd_dev->keycodesize = sizeof(mxckpd_keycodes[0]);
	mxckbd_dev->keycodemax = mxckpd_keycodes_size;
	mxckbd_dev->name = "mxckpd";
	mxckbd_dev->id.bustype = BUS_HOST;
	mxckbd_dev->open = mxc_kpp_open;
	mxckbd_dev->close = mxc_kpp_close;

	retval = input_register_device(mxckbd_dev);
	if (retval < 0) {
		printk(KERN_ERR
		       "mxckbd_dev: failed to register input device\n");
		goto err2;
	}

	// Initialize gpio keys.
	retval = init_gpio_keys();
	if (retval)
	{
		goto err2;
	}

	/* allocate required memory */
	press_scancode = kmalloc(kpp_dev.kpp_rows * sizeof(press_scancode[0]),
				 GFP_KERNEL);
	release_scancode =
	    kmalloc(kpp_dev.kpp_rows * sizeof(release_scancode[0]), GFP_KERNEL);

	if (!press_scancode || !release_scancode) {
		retval = -ENOMEM;
		goto err3;
	}

	for (i = 0; i < kpp_dev.kpp_rows; i++) {
		press_scancode[i] = kmalloc(kpp_dev.kpp_cols
					    * sizeof(press_scancode[0][0]),
					    GFP_KERNEL);
		release_scancode[i] =
		    kmalloc(kpp_dev.kpp_cols * sizeof(release_scancode[0][0]),
			    GFP_KERNEL);

		if (!press_scancode[i] || !release_scancode[i]) {
			retval = -ENOMEM;
			goto err3;
		}
	}

	cur_rcmap =
	    kmalloc(kpp_dev.kpp_rows * sizeof(cur_rcmap[0]), GFP_KERNEL);
	prev_rcmap =
	    kmalloc(kpp_dev.kpp_rows * sizeof(prev_rcmap[0]), GFP_KERNEL);

	if (!cur_rcmap || !prev_rcmap) {
		retval = -ENOMEM;
		goto err3;
	}

	__set_bit(EV_KEY, mxckbd_dev->evbit);

	for (i = 0; i < keypad->rowmax * keypad->colmax; i++)
		__set_bit(mxckpd_keycodes[i], mxckbd_dev->keybit);
	for (i = 0; i < ARRAY_SIZE(gpio_keys); i++)
		__set_bit(gpio_keys[i].key_code, mxckbd_dev->keybit);

	for (i = 0; i < kpp_dev.kpp_rows; i++) {
		memset(press_scancode[i], -1,
		       sizeof(press_scancode[0][0]) * kpp_dev.kpp_cols);
		memset(release_scancode[i], -1,
		       sizeof(release_scancode[0][0]) * kpp_dev.kpp_cols);
	}
	memset(cur_rcmap, 0, kpp_dev.kpp_rows * sizeof(cur_rcmap[0]));
	memset(prev_rcmap, 0, kpp_dev.kpp_rows * sizeof(prev_rcmap[0]));

	key_pad_enabled = 1;
	/* Initialize the polling timer */
	init_timer(&kpp_dev.poll_timer);

	/* By default, devices should wakeup if they can */
	/* So keypad is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	return 0;

      err3:
	mxc_kpp_free_allocated();
      err2:
	input_free_device(mxckbd_dev);
      err1:
	free_irq(irq, MOD_NAME);
	clk_disable(kpp_clk);
	clk_put(kpp_clk);
	return retval;
}

/*!
 * Dissociates the driver from the kpp device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_remove(struct platform_device *pdev)
{
	unsigned short reg_val;

	/*
	 * Clear the KPKD status flag (write 1 to it) and synchronizer chain.
	 * Set KDIE control bit, clear KRIE control bit (avoid false release
	 * events. Disable the keypad GPIO pins.
	 */
	__raw_writew(0x00, KPCR);
	__raw_writew(0x00, KPDR);
	__raw_writew(0x00, KDDR);

	reg_val = __raw_readw(KPSR);
	reg_val |= KBD_STAT_KPKD;
	reg_val &= ~KBD_STAT_KRSS;
	reg_val |= KBD_STAT_KDIE;
	reg_val &= ~KBD_STAT_KRIE;
	__raw_writew(reg_val, KPSR);

	gpio_keypad_inactive();
	clk_disable(kpp_clk);
	clk_put(kpp_clk);

	KPress = 0;

	del_timer(&kpp_dev.poll_timer);

	free_irq(keypad->irq, MOD_NAME);
	input_unregister_device(mxckbd_dev);

	mxc_kpp_free_allocated();

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_kpd_driver = {
	.driver = {
		   .name = "mxc_keypad",
		   .bus = &platform_bus_type,
		   },
	.suspend = mxc_kpp_suspend,
	.resume = mxc_kpp_resume,
	.probe = mxc_kpp_probe,
	.remove = mxc_kpp_remove
};

static int slider_touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int retval = 0;
	u16 mid = 0;
	int irq = 0;
	unsigned long flag = 0;

	/* Detect LDS6107 */
	retval = lds6107_i2c_read(client, 0x1F, &mid);
	if (retval < 0)
	{
		return -ENODEV;
	}

	mxc_request_iomux(MX31_PIN_SVEN0, OUTPUTCONFIG_GPIO, INPUTCONFIG_GPIO);
	mxc_set_gpio_direction(MX31_PIN_SVEN0, 0);
	mxc_set_gpio_dataout(MX31_PIN_SVEN0, 1);

	/* Initialize LDS6107 */
	lds6107_i2c_write(client, 0x0000, 0x0000);  /* COLD RESET */
	lds6107_i2c_write(client, 0x0040, 0x103C);  /* TOUCH DISABLE */
	lds6107_i2c_write(client, 0x000A, 0x0000);  /* DCM CONFIG */
	lds6107_i2c_write(client, 0x0074, 0x851E);  /* SCROLL CONFIG */
	lds6107_i2c_write(client, 0x0041, 0x011E);  /* PSEL */
	lds6107_i2c_write(client, 0x0042, 0x0000);
	lds6107_i2c_write(client, 0x0043, 0x011E);  /* INTERRUPT */
	lds6107_i2c_write(client, 0x0044, 0x0000);
	lds6107_i2c_write(client, 0x004E, 0x5000);  /* Ambient Config */
	lds6107_i2c_write(client, 0x0051, 0x1F1F);
	lds6107_i2c_write(client, 0x0052, 0x07FF);  /* Recalib Config */
	lds6107_i2c_write(client, 0x0053, 0x07FF);  /* Long Touch */
	lds6107_i2c_write(client, 0x005F, 0x0001);  /* C0~C4 Threshold */
	lds6107_i2c_write(client, 0x0061, 0x0028);
	lds6107_i2c_write(client, 0x0062, 0x0028);
	lds6107_i2c_write(client, 0x0063, 0x0046);
	lds6107_i2c_write(client, 0x0064, 0x0023);
	lds6107_i2c_write(client, 0x0068, 0x001E);
	lds6107_i2c_write(client, 0x0040, 0x903C);  /* Touch Enable */
	lds6107_i2c_write(client, 0x0001, 0x0000);  /* Soft RESET */

	irq = get_gpio_irq(MX31_PIN_CAPTURE, &flag);
	retval = request_irq(irq, slider_interrupt_handler, IRQF_DISABLED | flag, MOD_NAME, client);
	if (retval)
	{
		pr_debug("KPP: request_irq(%d) returned error %d\n", irq, retval);
		return retval;
	}

	lds6107_client = client;
	return 0;
}

static int slider_touch_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id slider_touch_id[] = {
	{"lds6107", 0},
	{},
};

static struct i2c_driver slider_touch_driver = {
	.driver = {
		.name = "lds6107",
		.owner = THIS_MODULE,
	},
	.probe = slider_touch_probe,
	.remove = slider_touch_remove,
	.id_table = slider_touch_id,
};

/*!
 * This function is called for module initialization.
 * It registers keypad char driver and requests for KPP irq number. This
 * function does the initialization of the keypad device.
 *
 * The following steps are used for keypad configuration,\n
 * -# Enable number of rows in the keypad control register (KPCR[7:0}).\n
 * -# Write 0's to KPDR[15:8]\n
 * -# Configure keypad columns as open-drain (KPCR[15:8])\n
 * -# Configure columns as output, rows as input (KDDR[15:0])\n
 * -# Clear the KPKD status flag (write 1 to it) and synchronizer chain\n
 * -# Set KDIE control bit, clear KRIE control bit\n
 * In this function the keypad queue initialization is done.
 * The keypad IOMUX configuration are done here.*

 *
 * @return      0 on success and a non-zero value on failure.
 */
static int __init mxc_kpp_init(void)
{
	int err = -ENODEV;
	printk(KERN_INFO "MXC keypad loaded\n");

	err = platform_driver_register(&mxc_kpd_driver);
	if (!err)
	{
		err = i2c_add_driver(&slider_touch_driver);
	}

	return err;
}

/*!
 * This function is called whenever the module is removed from the kernel. It
 * unregisters the keypad driver from kernel and frees the irq number.
 * This function puts the keypad to standby mode. The keypad interrupts are
 * disabled. It calls gpio_keypad_inactive function to switch gpio
 * configuration into default state.
 *
 */
static void __exit mxc_kpp_cleanup(void)
{
	platform_driver_unregister(&mxc_kpd_driver);
}

module_init(mxc_kpp_init);
module_exit(mxc_kpp_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Keypad Controller Driver");
MODULE_LICENSE("GPL");
