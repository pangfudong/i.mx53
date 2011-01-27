/*
 * kernel/power/main.c - PM subsystem core functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/resume-trace.h>
#include <linux/freezer.h>
#include <linux/vmstat.h>
#include <linux/syscalls.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/regulator/regulator.h>
#include <../arch/arm/mach-mx3/mx31_pins.h>

#include "power.h"

#define EPIT_BASE_ADDR          EPIT1_BASE_ADDR
#define EPITCR                  0x00
#define EPITSR                  0x04
#define EPITLR                  0x08
#define EPITCMPR                0x0C
#define EPITCNR                 0x10

static u32 epit_base_reg = IO_ADDRESS(EPIT_BASE_ADDR);

DEFINE_MUTEX(pm_mutex);

unsigned int pm_flags;
EXPORT_SYMBOL(pm_flags);

#ifdef CONFIG_PM_SLEEP

/* Routines for PM-transition notifications */

static BLOCKING_NOTIFIER_HEAD(pm_chain_head);

int register_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_pm_notifier);

int unregister_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_pm_notifier);

int pm_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&pm_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

#ifdef CONFIG_PM_DEBUG
int pm_test_level = TEST_NONE;

static int suspend_test(int level)
{
	if (pm_test_level == level) {
		printk(KERN_INFO "suspend debug: Waiting for 5 seconds.\n");
		mdelay(5000);
		return 1;
	}
	return 0;
}

static const char * const pm_tests[__TEST_AFTER_LAST] = {
	[TEST_NONE] = "none",
	[TEST_CORE] = "core",
	[TEST_CPUS] = "processors",
	[TEST_PLATFORM] = "platform",
	[TEST_DEVICES] = "devices",
	[TEST_FREEZER] = "freezer",
};

static ssize_t pm_test_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char *s = buf;
	int level;

	for (level = TEST_FIRST; level <= TEST_MAX; level++)
		if (pm_tests[level]) {
			if (level == pm_test_level)
				s += sprintf(s, "[%s] ", pm_tests[level]);
			else
				s += sprintf(s, "%s ", pm_tests[level]);
		}

	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t pm_test_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	const char * const *s;
	int level;
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	mutex_lock(&pm_mutex);

	level = TEST_FIRST;
	for (s = &pm_tests[level]; level <= TEST_MAX; s++, level++)
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len)) {
			pm_test_level = level;
			error = 0;
			break;
		}

	mutex_unlock(&pm_mutex);

	return error ? error : n;
}

power_attr(pm_test);
#else /* !CONFIG_PM_DEBUG */
static inline int suspend_test(int level) { return 0; }
#endif /* !CONFIG_PM_DEBUG */

#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_SUSPEND

/* This is just an arbitrary number */
#define FREE_PAGE_NUMBER (100)

static struct platform_suspend_ops *suspend_ops;

/**
 *	suspend_set_ops - Set the global suspend method table.
 *	@ops:	Pointer to ops structure.
 */

void suspend_set_ops(struct platform_suspend_ops *ops)
{
	mutex_lock(&pm_mutex);
	suspend_ops = ops;
	mutex_unlock(&pm_mutex);
}

/**
 * suspend_valid_only_mem - generic memory-only valid callback
 *
 * Platform drivers that implement mem suspend only and only need
 * to check for that in their .valid callback can use this instead
 * of rolling their own .valid callback.
 */
int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/**
 *	suspend_prepare - Do prep work before entering low-power state.
 *
 *	This is common code that is called for each state that we're entering.
 *	Run suspend notifiers, allocate a console and stop all processes.
 */
static int suspend_prepare(void)
{
	int error;
	unsigned int free_pages;

	if (!suspend_ops || !suspend_ops->enter)
		return -EPERM;

	pm_prepare_console();

	error = pm_notifier_call_chain(PM_SUSPEND_PREPARE);
	if (error)
		goto Finish;

	if (suspend_freeze_processes()) {
		error = -EAGAIN;
		goto Thaw;
	}

	free_pages = global_page_state(NR_FREE_PAGES);
	if (free_pages < FREE_PAGE_NUMBER) {
		pr_debug("PM: free some memory\n");
		shrink_all_memory(FREE_PAGE_NUMBER - free_pages);
		if (nr_free_pages() < FREE_PAGE_NUMBER) {
			error = -ENOMEM;
			printk(KERN_ERR "PM: No enough memory\n");
		}
	}
	if (!error)
		return 0;

 Thaw:
	suspend_thaw_processes();
 Finish:
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
	return error;
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

/**
 *	start_epit - Start EPIT (Enhanced Periodic Interrupt Timer)
 *	@timeout:	 Timeout value (in seconds)
 */
static void start_epit(u32 timeout)
{
	/* Enable wake-up by EPIT1 */
	enable_irq_wake(MXC_INT_EPIT1);

	/* Enable EPIT1 clock */
	clk_enable(clk_get(NULL, "epit_clk.0"));

	/* Reset EPIT1 */
	__raw_writel(0x00010000, epit_base_reg + EPITCR);
	while (__raw_readl(epit_base_reg + EPITCR) & 0x00010000);

	/* Setup EPIT1 */
	__raw_writel(timeout * 32768, epit_base_reg + EPITLR);
	__raw_writel(0x033A000E, epit_base_reg + EPITCR);

	/* Enable EPIT1 */
	__raw_writel(0x033A000F, epit_base_reg + EPITCR);
}

/**
 *	stop_epit - Stop EPIT (Enhanced Periodic Interrupt Timer)
 */
static void stop_epit(void)
{
	__raw_writel(0x0, epit_base_reg + EPITCR);
	clk_disable(clk_get(NULL, "epit_clk.0"));

	/* Disable wake-up by EPIT1 */
	disable_irq_wake(MXC_INT_EPIT1);
}

int epit_timeout = 180;
EXPORT_SYMBOL(epit_timeout);

/**
 *	suspend_enter - enter the desired system sleep state.
 *	@state:		state to enter
 *
 *	This function should be called after devices have been suspended.
 */
static int suspend_enter(suspend_state_t state)
{
	int error = 0;

#if defined(CONFIG_MTD_8G_NAND_FLASH)
	/* 180s for idle, 1 hours for standby, for elisa only */
	start_epit(state == PM_SUSPEND_IDLE ? epit_timeout : 1 * 3600);
#else
	/* 180s for idle, 6 hours for standby */
	start_epit(state == PM_SUSPEND_IDLE ? epit_timeout : 6 * 3600);
#endif

	if (state == PM_SUSPEND_IDLE)
		state = PM_SUSPEND_STANDBY;

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	if ((error = device_power_down(PMSG_SUSPEND))) {
		printk(KERN_ERR "PM: Some devices failed to power down\n");
		goto Done;
	}

	if (!suspend_test(TEST_CORE))
		error = suspend_ops->enter(state);

	device_power_up();
 Done:
	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());
	stop_epit();
	return error;
}

/**
 *	suspend_devices_and_enter - suspend devices and enter the desired system
 *				    sleep state.
 *	@state:		  state to enter
 */
int suspend_devices_and_enter(suspend_state_t state)
{
	int error;

	if (!suspend_ops)
		return -ENOSYS;

	if (suspend_ops->begin) {
		error = suspend_ops->begin(state);
		if (error)
			goto Close;
	}
	suspend_console();
	if (state != PM_SUSPEND_IDLE)
	{
		error = device_suspend(PMSG_SUSPEND);
		if (error) {
			printk(KERN_ERR "PM: Some devices failed to suspend\n");
			goto Resume_console;
		}
	}

	if (suspend_test(TEST_DEVICES))
		goto Resume_devices;

	if (suspend_ops->prepare) {
		error = suspend_ops->prepare();
		if (error)
			goto Resume_devices;
	}

	if (suspend_test(TEST_PLATFORM))
		goto Finish;

	error = disable_nonboot_cpus();
	if (!error && !suspend_test(TEST_CPUS))
	{
		struct regulator* reg_ldo1 = regulator_get(NULL, "LDO1");

		enable_irq_wake(MXC_INT_KPP);
		enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_KEY_ROW5));
		if (state == PM_SUSPEND_IDLE)
		{
			/* Enable wakeup by EPIT1/touchscreen/sd/usb/jack */
			enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_GPIO1_0));
			enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_STX0));
			enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_SRXD5));
			enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_KEY_ROW4));
#if defined(CONFIG_ENABLE_JACK_DETECT)
			enable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_DTR_DCE1));
#endif
			/* For issue: if there is a key pressed before entering idle mode,
			   the keyboard will be disfunctional after resuming from idle. */
			suspend_specific_platform_device("mxc_keypad", PMSG_SUSPEND);
		}

		if (reg_ldo1 != NULL && !IS_ERR(reg_ldo1))
			regulator_set_voltage(reg_ldo1, 1800000);

		suspend_enter(state);

		if (reg_ldo1 != NULL && !IS_ERR(reg_ldo1))
		{
			regulator_set_voltage(reg_ldo1, 3300000);
			regulator_put(reg_ldo1, NULL);
		}

		if (state == PM_SUSPEND_IDLE)
		{
			resume_specific_platform_device("mxc_keypad");
			disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_GPIO1_0));
			disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_STX0));
			disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_SRXD5));
			disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_KEY_ROW4));
#if defined(CONFIG_ENABLE_JACK_DETECT)
			disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_DTR_DCE1));
#endif
		}
		disable_irq_wake(IOMUX_TO_IRQ(MX31_PIN_KEY_ROW5));
		disable_irq_wake(MXC_INT_KPP);
	}

	enable_nonboot_cpus();
 Finish:
	if (suspend_ops->finish)
		suspend_ops->finish();
 Resume_devices:
 	if (state != PM_SUSPEND_IDLE)
 	{
		device_resume();
	}
 Resume_console:
	resume_console();
 Close:
	if (suspend_ops->end)
		suspend_ops->end();
	return error;
}

/**
 *	suspend_finish - Do final work before exiting suspend sequence.
 *
 *	Call platform code to clean up, restart processes, and free the
 *	console that we've allocated. This is not called for suspend-to-disk.
 */
static void suspend_finish(void)
{
	suspend_thaw_processes();
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
}




static const char * const pm_states[PM_SUSPEND_MAX] = {
	[PM_SUSPEND_STANDBY]	= "standby",
	[PM_SUSPEND_IDLE]	= "idle",
	[PM_SUSPEND_MEM]	= "mem",
};

static inline int valid_state(suspend_state_t state)
{
	/* All states need lowlevel support and need to be valid
	 * to the lowlevel implementation, no valid callback
	 * implies that none are valid. */
	if (!suspend_ops || !suspend_ops->valid || !suspend_ops->valid(state))
		return 0;
	return 1;
}


/**
 *	enter_state - Do common work of entering low-power state.
 *	@state:		pm_state structure for state we're entering.
 *
 *	Make sure we're the only ones trying to enter a sleep state. Fail
 *	if someone has beat us to it, since we don't want anything weird to
 *	happen when we wake up.
 *	Then, do the setup for suspend, enter the state, and cleaup (after
 *	we've woken up).
 */
static int enter_state(suspend_state_t state)
{
	int error;

	if (!valid_state(state))
		return -ENODEV;

	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

	if (state != PM_SUSPEND_IDLE)
	{
		printk(KERN_INFO "PM: Syncing filesystems ... ");
		sys_sync();
		printk("done.\n");

		pr_debug("PM: Preparing system for %s sleep\n", pm_states[state]);
		error = suspend_prepare();
		if (error)
			goto Unlock;
	}

	if (suspend_test(TEST_FREEZER))
		goto Finish;

	pr_debug("PM: Entering %s sleep\n", pm_states[state]);
	error = suspend_devices_and_enter(state);

 Finish:
 	if (state != PM_SUSPEND_IDLE)
 	{
		pr_debug("PM: Finishing wakeup.\n");
		suspend_finish();
	}
 Unlock:
	mutex_unlock(&pm_mutex);
	return error;
}


/**
 *	pm_suspend - Externally visible function for suspending system.
 *	@state:		Enumerated value of state to enter.
 *
 *	Determine whether or not value is within range, get state
 *	structure, and enter (above).
 */

int pm_suspend(suspend_state_t state)
{
	if (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX)
		return enter_state(state);
	return -EINVAL;
}

EXPORT_SYMBOL(pm_suspend);

#endif /* CONFIG_SUSPEND */

struct kobject *power_kobj;

/**
 *	state - control system power state.
 *
 *	show() returns what states are supported, which is hard-coded to
 *	'standby' (Power-On Suspend), 'mem' (Suspend-to-RAM), and
 *	'disk' (Suspend-to-Disk).
 *
 *	store() accepts one of those strings, translates it into the
 *	proper enumerated value, and initiates a suspend transition.
 */

static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
#ifdef CONFIG_SUSPEND
	int i;

	for (i = 0; i < PM_SUSPEND_MAX; i++) {
		if (pm_states[i] && valid_state(i))
			s += sprintf(s,"%s ", pm_states[i]);
	}
#endif
#ifdef CONFIG_HIBERNATION
	s += sprintf(s, "%s\n", "disk");
#else
	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';
#endif
	return (s - buf);
}

static ssize_t state_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
#ifdef CONFIG_SUSPEND
	suspend_state_t state = PM_SUSPEND_STANDBY;
	const char * const *s;
#endif
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	/* First, check if we are requested to hibernate */
	if (len == 4 && !strncmp(buf, "disk", len)) {
		error = hibernate();
  goto Exit;
	}

#ifdef CONFIG_SUSPEND
	for (s = &pm_states[state]; state < PM_SUSPEND_MAX; s++, state++) {
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len))
			break;
	}
	if (state < PM_SUSPEND_MAX && *s)
		error = enter_state(state);
#endif

 Exit:
	return error ? error : n;
}

power_attr(state);

#ifdef CONFIG_PM_TRACE
int pm_trace_enabled;

static ssize_t pm_trace_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_trace_enabled);
}

static ssize_t
pm_trace_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		pm_trace_enabled = !!val;
		return n;
	}
	return -EINVAL;
}

power_attr(pm_trace);
#endif /* CONFIG_PM_TRACE */

static struct attribute * g[] = {
	&state_attr.attr,
#ifdef CONFIG_PM_TRACE
	&pm_trace_attr.attr,
#endif
#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_PM_DEBUG)
	&pm_test_attr.attr,
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static irqreturn_t mxc_epit_interrupt(int irq, void *dev_id)
{
	printk("Wake up due to EPIT timeout.\n");
	__raw_writel(0x1, epit_base_reg + EPITSR);
	return IRQ_RETVAL(1);
}

static int __init pm_init(void)
{
    int ret;
	power_kobj = kobject_create_and_add("power", NULL);
	if (!power_kobj)
		return -ENOMEM;
	ret = request_irq(MXC_INT_EPIT1, mxc_epit_interrupt, 0, "mxc_pm", NULL);
	if (ret)
    {
        printk("%s: request_irq(%d) failed.\n", __FUNCTION__, MXC_INT_EPIT1);
        return ret;
    }
	return sysfs_create_group(power_kobj, &attr_group);
}

core_initcall(pm_init);
