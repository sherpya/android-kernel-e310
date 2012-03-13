/*
 * leds-pmic-nled.c - MSM PMIC NLEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <mach/pmic.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <mach/mpp.h>
#include <linux/msm_rpcrouter.h>
#include <mach/msm_rpcrouter.h> 
#include <linux/err.h>

#define NLED_BLINK_MS_MAX 20000

#define NLED_DEBUG 0

#define NLED_DBG(fmt, args...) \
	if (NLED_DEBUG) printk(KERN_INFO "[PMIC_NLED] %s " fmt,__func__, ##args)

#define NLED_ERR(fmt, args...) \
		printk(KERN_ERR "[PMIC_NLED] %s " fmt,__func__, ##args)

enum {
	NLED_OFF,
	NLED_ON,
	NLED_BLINK,
	NLED_DOUBLE_BLINK,
	NLED_MAX,
};

struct nled_status {
	int on;
	int blink_on_ms;
	int blink_off_ms;
};

struct nled_data {
	struct mutex lock;
	int opened;
	struct nled_status status;
	int old_status;
};

#define CCIPROG 0x30001000
#define CCIVERS 0x00010001
#define ONCRPC_PM_MPP_CONFIG_LED_INDI 31

enum {
	RPC_ON = 0,
	RPC_OFF,
	RPC_BLINK,
	RPC_NO_CHANGE
};

struct cci_rpc_mpp_led_req {
	struct rpc_request_hdr hdr;
	unsigned int index;
	unsigned int  mode;
	unsigned int  on_time;
	unsigned int  off_time_1;
	unsigned int  off_time_2;
	unsigned int  level;
};

int rpc_mpp_config_led_state
	(int index, int mode, int on_time, int off_time_1, int off_time_2, int level)
{
	int rpc_id=0;
	struct msm_rpc_endpoint *ep = NULL;
	struct cci_rpc_mpp_led_req  req;

	req.index = cpu_to_be32(index);
	req.mode = cpu_to_be32(mode);
	req.on_time = cpu_to_be32(on_time);
	req.off_time_1 = cpu_to_be32(off_time_1);
	req.off_time_2 = cpu_to_be32(off_time_2);
	req.level = cpu_to_be32(level);

	NLED_ERR("%d %d %d %d %d %d\n",
		index,
		mode,
		on_time,
		off_time_1,
		off_time_2,
		level);

	ep = msm_rpc_connect(CCIPROG, CCIVERS, 0);
	if (IS_ERR(ep)) {
		NLED_ERR("init rpc failed! rc = %ld\n", PTR_ERR(ep));
		return PTR_ERR(ep);
	}

       rpc_id = msm_rpc_call(ep, ONCRPC_PM_MPP_CONFIG_LED_INDI,
		 &req, sizeof(req),
		 5 * HZ);
	if (rpc_id < 0) {
		NLED_ERR("Can't select MSM device id=%d\n",rpc_id);
		msm_rpc_close(ep);
		return -1;
	}

	// close
	msm_rpc_close(ep);

	return 0;
}

static void nled_update(struct nled_data *nled_data)
{
	NLED_DBG("state: %d->%d\n",nled_data->old_status,nled_data->status.on);

	if (nled_data->status.on == NLED_OFF) {
		rpc_mpp_config_led_state(0, RPC_OFF, 0, 0, 0, 0);

	} else if (nled_data->status.on == NLED_ON) {
		rpc_mpp_config_led_state(0, RPC_ON, 0, 0, 0, 0);

	} else if (nled_data->status.on == NLED_BLINK) {
		rpc_mpp_config_led_state(0, RPC_BLINK,
			nled_data->status.blink_on_ms,
			nled_data->status.blink_off_ms,
			0,0);

	} else if (nled_data->status.on == NLED_DOUBLE_BLINK) {
		rpc_mpp_config_led_state(0, RPC_BLINK,
			nled_data->status.blink_on_ms,
			nled_data->status.blink_on_ms,
			nled_data->status.blink_off_ms,0);

	} else {
		NLED_ERR("ERROR!\n");
	}

	mutex_lock(&nled_data->lock);
	nled_data->old_status = nled_data->status.on;
	mutex_unlock(&nled_data->lock);

	return;
}

static void pmic_nled_set_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	NLED_DBG("NLED %s\n",value?"ON":"OFF");
}

static ssize_t nled_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", nled_data->status.on);
}
	
static ssize_t nled_on_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	int val = -1;

	sscanf(buf, "%d", &val);
	if (val < NLED_OFF || val >= NLED_MAX)
		return -EINVAL;
	mutex_lock(&nled_data->lock);
	nled_data->status.on = val;
	mutex_unlock(&nled_data->lock);

	nled_update(nled_data);

	return count;
}

static ssize_t nled_blink_on_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", nled_data->status.blink_on_ms);
}
	
static ssize_t nled_blink_on_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	int val = -1;

	sscanf(buf, "%d", &val);
	if (val < 0 || val >= NLED_BLINK_MS_MAX)
		return -EINVAL;

	mutex_lock(&nled_data->lock);
	nled_data->status.blink_on_ms = val;
	mutex_unlock(&nled_data->lock);

	return count;
}
static ssize_t nled_blink_off_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", nled_data->status.blink_off_ms);
}
	
static ssize_t nled_blink_off_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct nled_data *nled_data = dev_get_drvdata(dev);

	int val = -1;

	sscanf(buf, "%d", &val);
	if (val < 0 || val >= NLED_BLINK_MS_MAX)
		return -EINVAL;

	mutex_lock(&nled_data->lock);
	nled_data->status.blink_off_ms = val;
	mutex_unlock(&nled_data->lock);

	return count;
}

static DEVICE_ATTR(on, 0644, nled_on_show, nled_on_store);
static DEVICE_ATTR(blink_on, 0644, nled_blink_on_show, nled_blink_on_store);
static DEVICE_ATTR(blink_off, 0644, nled_blink_off_show, nled_blink_off_store);

static struct led_classdev nled_dev = {
	.name			= "nled",
	.brightness_set		= pmic_nled_set_brightness,
	.brightness		= LED_OFF,
};

static int pmic_nled_probe(struct platform_device *pdev)
{
	int rc;
	struct nled_data *nled_data;

	nled_data = kzalloc(sizeof(struct nled_data), GFP_KERNEL);
	if (nled_data == NULL) {
		rc = -ENOMEM;
		return rc;
	}

	mutex_init(&nled_data->lock);
	nled_data->opened = 0;
	nled_data->status.on = NLED_OFF;
	nled_data->status.blink_on_ms = 0;
	nled_data->status.blink_off_ms = 0;
	nled_data->old_status = NLED_OFF;

	rc = led_classdev_register(&pdev->dev, &nled_dev);
	if (rc) {
		NLED_ERR("unable to register led class driver\n");
		goto err_create_led_classdev;
	}

	dev_set_drvdata(nled_dev.dev, nled_data);

	rc = device_create_file(nled_dev.dev, &dev_attr_on);
	if (rc) {
		NLED_ERR("unable to register led class driver\n");
		goto err_create_on;
	}

	rc = device_create_file(nled_dev.dev, &dev_attr_blink_on);
	if (rc) {
		NLED_ERR("unable to register led class driver\n");
		goto err_create_blink_on;
	}

	rc = device_create_file(nled_dev.dev, &dev_attr_blink_off);
	if (rc) {
		NLED_ERR("unable to register led class driver\n");
		goto err_create_blink_off;
	}

	pmic_nled_set_brightness(&nled_dev, LED_OFF);

	return 0;

err_create_blink_off:
	device_remove_file(nled_dev.dev, &dev_attr_blink_on);
err_create_blink_on:
	device_remove_file(nled_dev.dev, &dev_attr_on);
err_create_on:
	led_classdev_unregister(&nled_dev);
err_create_led_classdev:
	kfree(nled_data);

	return rc;
}

static int __devexit pmic_nled_remove(struct platform_device *pdev)
{
	struct nled_data *nled_data = dev_get_drvdata(nled_dev.dev);
	device_remove_file(nled_dev.dev, &dev_attr_on);
	device_remove_file(nled_dev.dev, &dev_attr_blink_on);
	device_remove_file(nled_dev.dev, &dev_attr_blink_off);
	led_classdev_unregister(&nled_dev);
	kfree(nled_data);
	return 0;
}

#ifdef CONFIG_PM
static int pmic_nled_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&nled_dev);

	return 0;
}

static int pmic_nled_resume(struct platform_device *dev)
{
	led_classdev_resume(&nled_dev);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_device pmic_nled_device = {
        .name   = "pmic-nled",
        .id = -1,
};

static struct platform_driver pmic_nled_driver = {
	.probe		= pmic_nled_probe,
	.remove		= __devexit_p(pmic_nled_remove),
	.suspend	= pmic_nled_suspend,
	.resume		= pmic_nled_resume,
	.driver		= {
		.name	= "pmic-nled",
		.owner	= THIS_MODULE,
	},
};

static int __init pmic_nled_init(void)
{
	platform_device_register(&pmic_nled_device);
	return platform_driver_register(&pmic_nled_driver);
}
module_init(pmic_nled_init);

static void __exit pmic_nled_exit(void)
{
	platform_driver_unregister(&pmic_nled_driver);
	platform_device_unregister(&pmic_nled_device);
}
module_exit(pmic_nled_exit);

MODULE_DESCRIPTION("MSM PMIC NLED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-nled");
