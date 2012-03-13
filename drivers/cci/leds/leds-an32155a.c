/*
 * leds-an32155a.c - Panasonic AN32155A LED controller driver.
 *
 * Copyright (c) 2010, Compalcomm Inc All rights reserved.
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <mach/msm_rpcrouter.h>
#include <mach/vreg.h>
#include <mach/clk.h>
#include <linux/ioctl.h>
#include <linux/hrtimer.h>
#include <linux/wait.h>


#define AN32155A_IOCTL_MAGIC 'n'
#define AN32155A_IOCTL  _IO(AN32155A_IOCTL_MAGIC, 0)
#define AN32155A_IOCTL_WRITE_TEST  _IOW(AN32155A_IOCTL_MAGIC, 1, unsigned)
#define AN32155A_IOCTL_READ_TEST  _IOR(AN32155A_IOCTL_MAGIC, 2, unsigned)
#define AN32155A_IOCTL_WRITE_DATA  _IOW(AN32155A_IOCTL_MAGIC, 3, unsigned)
#define AN32155A_IOCTL_SET_INIT  _IOW(AN32155A_IOCTL_MAGIC, 4, unsigned)
#define AN32155A_IOCTL_CANCEL_LED  _IO(AN32155A_IOCTL_MAGIC, 5)
#define AN32155A_IOCTL_TEMPO  _IO(AN32155A_IOCTL_MAGIC, 6)
#define AN32155A_IOCTL_POWER_ON_DONE  _IO(AN32155A_IOCTL_MAGIC, 7)

#define AN32155A_NAME "an32155a"

#define AN_DEBUG 0

#define AN_DBG(fmt, args...) \
	if (AN_DEBUG) printk(KERN_INFO "[LED-AN32155A] %s " fmt,__func__, ##args)

#define AN_ERR(fmt, args...) \
		printk(KERN_ERR "[LED-AN32155A] %s " fmt,__func__, ##args)

struct an32155a_buf {
	unsigned char init_buf[170];
	unsigned char init_len;
	unsigned char buf[500];
	unsigned char buf_len;
	unsigned char rep_time;
};

struct an32155a_data {
	struct work_struct work;
	struct mutex lock;
	struct mutex work_sig_lock;
	wait_queue_head_t wait;
	struct workqueue_struct *wq;
	struct hrtimer timer;
	int opened;
	struct an32155a_buf buf;
	unsigned char buf_index;
	int sleep_done;
	int disable_work;
	int tempo;
	int suspend;
	int working;
};

static struct i2c_client *this_client;
static int initial = 0;

#define AN32155A_ENABLE_PIN 131
static void an_power(int on)
{
	int gpio_value;

	AN_DBG("\n");

	if (!gpio_request(AN32155A_ENABLE_PIN, "led_rst")) {
		gpio_value = gpio_get_value(AN32155A_ENABLE_PIN);
		AN_DBG("gpio_get_value(%d) = %d\n",
			AN32155A_ENABLE_PIN, gpio_get_value(AN32155A_ENABLE_PIN));

		if (gpio_value != on) {
			gpio_set_value(AN32155A_ENABLE_PIN, on);
			if (AN_DEBUG) {
				udelay(1);
			}
			AN_DBG("gpio_get_value(%d) = %d\n",
				AN32155A_ENABLE_PIN, gpio_get_value(AN32155A_ENABLE_PIN));
		}

		gpio_free(AN32155A_ENABLE_PIN);

	} else {
		AN_ERR("failed to request gpio 131\n");
	}

}

static int
an_i2c_write(const char *buf, int len)
{
    return i2c_master_send(this_client, buf, len);
}

//Unused
/*
static int
an_i2c_read(char *buf, int len)
{
    return i2c_master_recv(this_client, buf, len);
}
*/

static enum hrtimer_restart an_led_timer_function(struct hrtimer *timer)
{
	struct an32155a_data *an_data = container_of(timer, struct an32155a_data, timer);

	an_data->sleep_done = 1;

	wake_up(&an_data->wait);
	return HRTIMER_NORESTART;
}

#define SLEEP_CMD 0xC0
#define CMD_MASK 0xF0
#define VAL_MASK 0x0F
#define REPEAT_INFINITE_CMD 0xFF
static void an_work_func(struct work_struct *work)
{
	struct an32155a_data *an_data = container_of(work, struct an32155a_data, work);
	int a, rc, count;
	unsigned int sleep_time = 0;
	unsigned char init_len, buf_len, rep_time;

	mutex_lock(&an_data->work_sig_lock);
	an_data->disable_work=0;
	mutex_unlock(&an_data->work_sig_lock);

	init_len = an_data->buf.init_len;
	buf_len = an_data->buf.buf_len;
	rep_time = an_data->buf.rep_time;

	for (a=0; a < init_len; a++) {
		if(an_data->disable_work==1)
			return;
		if ((an_data->buf.init_buf[a*2] & CMD_MASK) == SLEEP_CMD) {
			sleep_time = (unsigned int)(((an_data->buf.init_buf[a*2] & VAL_MASK) << 8) + an_data->buf.init_buf[a*2+1]);
			sleep_time *= 10;
			hrtimer_start(&an_data->timer, ktime_set(sleep_time/1000, (sleep_time % 1000)* 1000*1000), HRTIMER_MODE_REL);
			an_data->sleep_done = 0;
			rc = wait_event_interruptible(an_data->wait, (an_data->sleep_done == 1));
		} else {
			if (!(an_data->buf.init_buf[a*2] == 0x0 && an_data->buf.init_buf[a*2+1] == 0x1)) {
				an_data->working = 1;
			}
			rc = an_i2c_write(&an_data->buf.init_buf[a*2],2);
			if (an_data->buf.init_buf[a*2] == 0x0 && an_data->buf.init_buf[a*2+1] == 0x1) {
				an_data->working = 0;
			}
		}
	}

	for (count = 0; count < rep_time; count++) {
		if (rep_time == REPEAT_INFINITE_CMD)
			--count;

		for (a = 0; a < buf_len; a++) {
			if(an_data->disable_work==1)
				return;
			if ((an_data->buf.buf[a*2] & CMD_MASK) == SLEEP_CMD) {
				sleep_time = (unsigned int)(((an_data->buf.buf[a*2] & VAL_MASK) << 8) + an_data->buf.buf[a*2+1]);
				sleep_time *= 10;
				hrtimer_start(&an_data->timer, ktime_set(sleep_time/1000, (sleep_time % 1000)* 1000*1000), HRTIMER_MODE_REL);
				an_data->sleep_done = 0;
				rc = wait_event_interruptible(an_data->wait, (an_data->sleep_done == 1));
			} else {
				if (!(an_data->buf.buf[a*2] == 0x0 && an_data->buf.buf[a*2+1] == 0x1)) {
					an_data->working = 1;
				}
				rc = an_i2c_write(&an_data->buf.buf[a*2],2);
				if (an_data->buf.buf[a*2] == 0x0 && an_data->buf.buf[a*2+1] == 0x1) {
					an_data->working = 0;
				}
			}
		}
	}
	if (rep_time > 1 && rep_time < REPEAT_INFINITE_CMD) {
		char reset_buf[] = {0x00,0x01};
		rc = an_i2c_write(reset_buf, 2);
		an_data->working = 0;
	}
}

void an_tempo(short brightness)
{
	if (initial == 1) {
		struct an32155a_data *an_data = i2c_get_clientdata(this_client);
		if (an_data->tempo == 0) {
			return;
		} else {
			unsigned char buf[2];
			unsigned char addr;
			//AN_DBG("TEMPO: %d -> %x\n",brightness,buf[1]);
			buf[1] = brightness;

			for (addr = 0x30; addr <= 0x51; addr+=3) {
				buf[0] = addr;
				an_i2c_write(buf, 2);
			}
		}
	}
}
static void an_power_on(struct an32155a_data *an_data, int on)
{
	if (on) {
		int a;
		unsigned char buf[] =
			{0x00,0x01,0x01,0x03,0x12,0x77,0x13,0x77,
			0x30,0xff,0x33,0xff,0x36,0xff,0x39,0xff,
			0x3c,0xff,0x3f,0xff,0x42,0xff,0x45,0xff,
			0x48,0xff,0x4b,0xff,0x4e,0xff,0x51,0xff,
			0x06,0xc1,
			0x31,0x20,0x34,0x20,0x37,0x20,0x3a,0x20,
			0x3d,0x20,0x40,0x20,0x43,0x20,0x46,0x20,
			0x49,0x20,0x4c,0x20,0x4f,0x20,0x52,0x20,
			0x10,0x22,0x11,0x22};

		mutex_lock(&an_data->work_sig_lock);
		an_data->disable_work=1;
		mutex_unlock(&an_data->work_sig_lock);
		an_data->sleep_done = 1;
		hrtimer_cancel(&an_data->timer);
		wake_up(&an_data->wait);
		cancel_work_sync(&an_data->work);
		for (a=0;a<sizeof(buf);a++){
			an_data->buf.init_buf[a] = buf[a];
		}
		AN_DBG("POWER_ON LED on\n");
		an_data->buf.init_len = sizeof(buf)/2;
		an_data->buf.buf_len = 0;
		an_data->buf.rep_time = 0;
		queue_work(an_data->wq, &an_data->work);
	} else {
		int a;
		unsigned char buf[] =
			{0x31,0xa0,0x34,0xa0,0x37,0xa0,0x3a,0xa0,
			0x3d,0xa0,0x40,0xa0,0x43,0xa0,0x46,0xa0,
			0x49,0xa0,0x4c,0xa0,0x4f,0xa0,0x52,0xa0,
			0x10,0x00,0x11,0x00,0xc7,0xd0,0x0,0x1};

		mutex_lock(&an_data->work_sig_lock);
		an_data->disable_work=1;
		mutex_unlock(&an_data->work_sig_lock);
		an_data->sleep_done = 1;
		hrtimer_cancel(&an_data->timer);
		wake_up(&an_data->wait);
		cancel_work_sync(&an_data->work);
		for (a=0;a<sizeof(buf);a++){
			an_data->buf.init_buf[a] = buf[a];
		}
		AN_DBG("POWER_ON LED off\n");
		an_data->buf.init_len = sizeof(buf)/2;
		an_data->buf.buf_len = 0;
		an_data->buf.rep_time = 0;
		queue_work(an_data->wq, &an_data->work);

	}
}

static long an_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct an32155a_data *an_data = file->private_data;
	void __user *argp = (void __user *)arg;

	mutex_lock(&an_data->lock);
	switch(cmd) {
	case AN32155A_IOCTL_WRITE_TEST:
	case AN32155A_IOCTL_READ_TEST:
	case AN32155A_IOCTL:
		break;
	case AN32155A_IOCTL_POWER_ON_DONE:
		an_power_on(an_data, 0);
		break;
	case AN32155A_IOCTL_TEMPO:
		{
			int a;
			unsigned char tempo_buf[] =
				{0x00,0x01,0x10,0x11,0x11,0x11,0x06,0x01};

			mutex_lock(&an_data->work_sig_lock);
			an_data->disable_work=1;
			mutex_unlock(&an_data->work_sig_lock);
			cancel_work_sync(&an_data->work);
			for (a=0;a<sizeof(tempo_buf);a++){
				an_data->buf.init_buf[a] = tempo_buf[a];
			}
			AN_DBG("IOCTL_TEMPO\n");
			an_data->buf.init_len = sizeof(tempo_buf)/2;
			an_data->buf.buf_len = 0;
			an_data->buf.rep_time = 0;
			queue_work(an_data->wq, &an_data->work);
			an_data->tempo = 1;
		}
		break;

	case AN32155A_IOCTL_CANCEL_LED:
	{
		int a;
		unsigned char cancel_buf[] = {0x00,0x01};

		an_data->tempo = 0;
		mutex_lock(&an_data->work_sig_lock);
		an_data->disable_work=1;
		mutex_unlock(&an_data->work_sig_lock);
		an_data->sleep_done = 1;
		hrtimer_cancel(&an_data->timer);
		wake_up(&an_data->wait);
		cancel_work_sync(&an_data->work);

		for (a=0;a<sizeof(cancel_buf);a++){
			an_data->buf.init_buf[a] = cancel_buf[a];
		}
		AN_DBG("IOCTL_CANCEL\n");
		an_data->buf.init_len = sizeof(cancel_buf)/2;
		an_data->buf.buf_len = 0;
		an_data->buf.rep_time = 0;
		queue_work(an_data->wq, &an_data->work);
	}
		break;

	case AN32155A_IOCTL_WRITE_DATA:
		if (copy_from_user(&an_data->buf, argp, sizeof(an_data->buf)))
			return -EFAULT;
		an_data->tempo = 0;
		mutex_lock(&an_data->work_sig_lock);
		an_data->disable_work=1;
		mutex_unlock(&an_data->work_sig_lock);
		AN_ERR("IOCTL: WRITE_DATA init_len:%d buf_len:%d\n",
			an_data->buf.init_len, an_data->buf.buf_len);
		an_data->sleep_done = 1;
		hrtimer_cancel(&an_data->timer);
		wake_up(&an_data->wait);
		cancel_work_sync(&an_data->work);
		queue_work(an_data->wq, &an_data->work);
		break;
	default:
		AN_ERR("IOCTL: no this command: %d\n",cmd);
		break;
	}

	mutex_unlock(&an_data->lock);

	return 0;
}

static int an_open(struct inode *ip, struct file *file)
{
	int rc = -EBUSY;
 	struct an32155a_data *an_data = i2c_get_clientdata(this_client);
	AN_DBG("open\n");
	if (!an_data->opened) {
		file->private_data = an_data;
		an_data->opened = 1;
		rc = 0;
	}
	return rc;
}

static int an_release(struct inode *ip, struct file *file)
{
	int rc = -EBADF;
	struct an32155a_data *an_data = file->private_data;
	AN_DBG("release\n");
	if (an_data->opened) {
		an_data->opened = 0;
	}
	return rc;
}

static struct file_operations an_fops = {
        .owner 	= THIS_MODULE,
        .open 	= an_open,
        .release = an_release,
        .unlocked_ioctl = an_ioctl,
};

static struct miscdevice an_device = {
        .minor 	= MISC_DYNAMIC_MINOR,
        .name 	= "an32155a",
        .fops 	= &an_fops,
};

static int an_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	struct an32155a_data *an_data;
	AN_DBG("probe\n");

	if(!(an_data = kzalloc( sizeof(struct an32155a_data), GFP_KERNEL))) {
		rc = -ENOMEM;
	}
	mutex_init(&an_data->lock);
	mutex_init(&an_data->work_sig_lock);
	INIT_WORK(&an_data->work, an_work_func);
	an_data->wq = create_singlethread_workqueue("nled_thread");
	init_waitqueue_head(&an_data->wait);
	hrtimer_init(&an_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	an_data->timer.function = an_led_timer_function;
	an_data->suspend = 0;
	an_data->working = 0;
	an_data->opened = 0;
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		goto exit_check_functionality_failed;	

	an_power_on(an_data, 1);

	i2c_set_clientdata(client, an_data);
	this_client = client;

	// Register a misc device 
	rc = misc_register(&an_device);
	if(rc) {
		AN_ERR("misc_register failed \n");
		goto exit_misc_device_register_failed;
	}
	initial = 1;
	return 0;

exit_check_functionality_failed:	
exit_misc_device_register_failed:
	return rc;
}

static int an_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct an32155a_data *an_data = i2c_get_clientdata(client);
	int suspend = 0;

	suspend = an_data->working ? 0 : 1;

	AN_DBG("SUSPEND = %d\n", suspend);

	if (suspend == 1) {
		//mutex_lock(&an_data->work_sig_lock);
		//an_data->disable_work=1;
		//mutex_unlock(&an_data->work_sig_lock);

		//cancel_work_sync(&an_data->work);
		an_power(0);
		an_data->suspend = 1;
	}
	return 0;
}

static int an_resume(struct i2c_client *client)
{
	struct an32155a_data *an_data = i2c_get_clientdata(client);
	AN_DBG("RESUME\n");

	if (an_data->suspend == 1) {
		an_power(1);
		an_data->suspend = 0;
	}
	return 0;
}

static int an_remove(struct i2c_client *client)
{
	struct an32155a_data *an_data = i2c_get_clientdata(client);
	this_client = NULL;
	misc_deregister(&an_device);
	kfree(an_data);
	return 0;
}

static const struct i2c_device_id an32155a_id[] = {
	{ AN32155A_NAME, 0 },
	{ }
};

static struct i2c_driver an32155a_driver = {
	.probe = an_probe,
	.remove = an_remove,
	.suspend = an_suspend,
	.resume = an_resume,
	.id_table = an32155a_id,
	.driver = {
		.name   = AN32155A_NAME,
	},
};

static int an_plat_probe(struct platform_device *pdev __attribute__((unused)))
{
	int rc = -EFAULT;
	AN_DBG(" \n");
	an_power(1);
	rc = i2c_add_driver(&an32155a_driver);
	return rc;
}

static struct platform_driver an32155a_plat_driver = {
        .probe = an_plat_probe,
        .driver = {
                .name = AN32155A_NAME,
                .owner = THIS_MODULE,
        },
};

static struct platform_device an32155a_plat_device = {
        .name   = AN32155A_NAME,
        .id = -1,
};

int driver_get_pj_id(void);
static int __init an_init(void)
{
	AN_ERR(" \n");
	if (driver_get_pj_id() == 0x3 || driver_get_pj_id() == 0x5) { //CAP6 or CAP2
		AN_ERR("CAP6 : load AN32155a driver\n");
		platform_device_register(&an32155a_plat_device);
		return platform_driver_register(&an32155a_plat_driver);
	}
	return 0;
}
module_init(an_init);

static void __exit an_exit(void)
{
	platform_driver_unregister(&an32155a_plat_driver);
	platform_device_unregister(&an32155a_plat_device);
}
module_exit(an_exit);

MODULE_DESCRIPTION("Panasonic AN32155A driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:an32155a");
