/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>
#include <linux/module.h>

#include <mach/msm_rpcrouter.h>
#include <mach/pmic.h>

/*
 * Use Qualcomm RPC command 
 */
#define PMIC_VIBRATOR_LEVEL	(3000)

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct hrtimer vibe_timer;

static void set_pmic_vibrator(int on)
{
	int rc = 0;

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc < 0) {
		printk(KERN_ERR "vib set mode rpc failed!\n");
	}

	rc = pmic_vib_mot_set_volt(on?PMIC_VIBRATOR_LEVEL:0);
	if (rc < 0) {
		printk(KERN_ERR "vib set volt rpc failed!\n");
	}
}

static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __init msm_init_pmic_vibrator(void)
{
	int rc = 0;
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
	printk("----------------------vibrator init\n");
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	rc = timed_output_dev_register(&pmic_vibrator);
	if (rc) {
		printk(KERN_ERR "timed_output_dev register failed!\n");
	}
	return 0;
}

module_init(msm_init_pmic_vibrator);

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

