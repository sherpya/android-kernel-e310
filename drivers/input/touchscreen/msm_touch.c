/* drivers/input/touchscreen/msm_touch.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
//#include "mach/../../cci_smem.h"
#include "mach/../../smd_private.h"
#include <mach/msm_touch.h>
#include <linux/irq.h>
#include <asm/gpio.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <mach/rpc_server_handset.h>
#include "../../../arch/arm/mach-msm/smd_private.h"
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <linux/msm_rpcrouter.h>
#include <mach/msm_rpcrouter.h> 
#include <linux/proc_fs.h>
//systronix
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include "../../staging/android/logger.h"
#include <linux/time.h>
#include <linux/rtc.h>
static int online_debug = 0;

#define LOW_SENSITIVE		0
#define MEDIUM_SENSITIVE	1
#define HIGH_SENSITIVE		2

#define TS_PENUP_TIMEOUT_MS 500
#define TSSC_DOG_TIMER 20000 //20sec
static int debug_mask = 0;
module_param_named(
	debug_mask, debug_mask,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

 static int touch_irq_state = 0;
module_param_named(
	touch_irq_state, touch_irq_state,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int touch_read_i2c_fail=0;
module_param_named(
	touch_read_i2c_fail, touch_read_i2c_fail,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

struct st1232_ts_data {
	//uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input;	
	struct work_struct work;	
	struct input_dev *input_dev;
	int irq;
	atomic_t irq_disable;
	struct timer_list timer;
	struct timer_list proximity_timer;
	struct work_struct  ts_event_work;
        struct work_struct  ts_dog_work;
//	struct early_suspend early_suspend;
       int prev_touches;
	 //sitronix  
	struct mutex mutex;
	bool isp_enabled;
	bool autotune_result;
	bool always_update;
	char I2C_Offset;
	char I2C_Length;
	char I2C_RepeatTime;
	bool Can_update;
	int  fw_revision;
	int  struct_version;	
	int  data_mode;	
	int  sensitivity_level;
};

//static void __iomem *virt;

#define	MAX_CHANNELS			32
#define CONFIG_PARAM_MAX_CHANNELS	32
#define MUTUAL_CALIBRATION_BASE_SIZE	256

#define	TCXO_SHUTDOWN_WORKAROUND		0
//static DEFINE_SPINLOCK(acct_lock);
//static DEFINE_SPINLOCK(acct_lock1);

struct input_dev *kpdev=NULL;
struct ts *ts_point;
unsigned long timecounter[2],diff_time; 
unsigned long int en_irq=0,dis_irq=0;
static unsigned short report_value = 200;
static bool CanUpdate=0,burnFw=0,release_press_flag=0;
//static int last_key=0;
static int x[3],y[3],index=0,index_jiffies=0;
int tp_version=1;
static int press1=0, press2=0;
static bool bak_p1=0,bak_p2=0,receive_phone=0,phone_ap=0,run_one=1;
static u32 g_rev,g_ver,tpversion;
//static int bak_x=0,bak_y=0;
static u8  keyregister,keypress;
static int interrupt_counter=0;
static bool touchedge=0,keylocation=0,Adjustsensitivity=0,runone=0;
int TOUCH_GPIO = 94;
bool  BKL_EN=1;
int ts_width;
int led_flag=0,press_point=0;
static struct early_suspend ts_early;
struct st1232_ts_data *st1232_ts;	
static int project_id;
	static u32 last_x;
	static u32 last_y;
	static u32 last_x2;
	static u32 last_y2;	
	static u32 last_key_x;
	static u32 last_key_y;		
	//static int set_highest_priority	= 0;
	static int down_edge=0;
	static struct wake_lock tpwakelock,tpwakelock_idle;

unsigned char st1232_suspend = 0;
unsigned long recover_workqueue_cnt = 0;
unsigned char touch_work_state = 0;

extern int vbatt;
extern int batt_capacity;
extern int rpc_mpp_config_led_state(int index, int mode, int on_time, int off_time_1, int off_time_2, int level);
static void st1232_ts_timer(unsigned long arg);
static int sitronix_ts_set_sensitivity(struct i2c_client *client, u32 level);
static irqreturn_t st1232_ts_interrupt(int irq, void *dev_id);
int st1232_i2c_read(struct i2c_client *client, int reg, u8 *buf, int count);
void config_gpio_table(uint32_t *table, int len);
void tp_reset(void);
void tp_reset_and_retry(void);
int tp_reset_and_check(struct i2c_client *client);
static void Touch_debug_save_log_to_ram(unsigned char *str, unsigned char force_write);
static int st1232_proximity_enable(struct file *file, const char *buffer,unsigned long count, void *data);
static int st1232_proximity_control(int en_p_sensor);
static struct workqueue_struct *MT_wq;
static struct workqueue_struct *DOG_wq;

static struct input_dev *fpdev;
static int distance = 0;
static int enablep = 0;
extern unsigned long msm_irq_priority_get(unsigned long int_number);
extern void msm_irq_priority_set(unsigned long int_number,unsigned long level);
extern void cci_show_interrupts(unsigned char to_kernel_log);

void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], 0);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static uint32_t gpio_table[] = {
	GPIO_CFG(94, 0, 0, 0, 0),
        //GPIO_CFG(94, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	};

#define SITRONIX_ISP_ENABLED
#ifdef SITRONIX_ISP_ENABLED

struct config_param_v0{
	u8 	reserve1[3];		
	u8	x_chs;				// Number of X channels.
	u8	y_chs;				// Number of Y channels.								
	u16	x_res;				// X resolution. [6:5]
	u16	y_res;				// Y resolution. [8:7]
	u8	ch_map[MAX_CHANNELS];		// Channle mapping table.

	u8	data_threshold_shift_x;		// Data threshold shift. [41]
	u8	data_threshold_offset_x;	// Data threshold offset.[42]

	u8	pt_threshold_shift;		// Point threshold shift. [43]
	u8	pt_threshold_offset;		// Point threshold offset. [44]
	u8      reserve2[5];
	
	u8 	cnt[MAX_CHANNELS];
	u8	offset[MAX_CHANNELS];
	u16	baseline[MAX_CHANNELS];	
	
	u8	k_chs;
	u8 	reserve4[11];
	
	u8	wake_threshold_shift;		
	u8	data_threshold_shift_y;		// Data threshold shift.  [191]
	u8	data_threshold_offset_y;	// Data threshold offset. [192]
	u8	data_threshold_shift_k;		// Data threshold shift.  [193]
	u8	data_threshold_offset_k;	// Data threshold offset. [194] 
	
	u8	peak_threshold_shift_x;		// Data threshold shift. 	[195]
	u8	peak_threshold_offset_x;	// Data threshold offset.	[196]
	u8	peak_threshold_shift_y;		// Data threshold shift.	[197]
	u8	peak_threshold_offset_y;	// Data threshold offset.	[198]
	
	u8	mutual_threshold_shift;		// Data threshold shift.	[199]
	u8	mutual_threshold_offset;	// Data threshold offset.	[200]
	
	//Filter
	u8	filter_rate;		// [201]
	u16	filter_range_1;	// [202]
	u16	filter_range_2;	// [203]
	
	u8	reserve5[299];
} __attribute__ ((packed)); 


struct config_param_v1{
	u8	reserve1[3];
	u8	x_chs;			           	 //BCB:x_chs, Number of X channels.
	u8	y_chs;			       	//BCB:y_chs, Number of Y channels.
	u8	k_chs;				       //BCB:k_chs, Number of Key channels.
	u16	x_res;				       //BCB:x_res,x report resolution
	u16	y_res;			            	//BCB:y_res,y report resolution
	u8	ch_map[MAX_CHANNELS];		//BCB:ch_map,Channle mapping table.
	
	u8	reserve2[15];
	
	u8	data_threshold_shift_x;		    //BCB:Daimond_data_threshold_shift_X,Data threshold shift.
	u8	data_threshold_offset_x;	    //BCB:Daimond_data_threshold_offset_X,Data threshold offset.
	u8	pt_threshold_shift;		        //BCB:pt_threshold_shift_XY,Point threshold shift.
	u8	pt_threshold_offset;		    //BCB:pt_threshold_offset_XY,Point threshold offset.
	u16	pt_threshold;			        //BCB:pt_threshold,Point threshold.
	u8	reserve3;
	u8	data_threshold_shift_y;		    //BCB:data_threshold_shift_Y,Data threshold shift.
	u8	data_threshold_offset_y;	    //BCB:data_threshold_offset_Y,Data threshold offset.
	u8	data_threshold_shift_k;		    //BCB:pt_threshold_shift_K,Data threshold shift.
	u8	data_threshold_offset_k;	    //BCB:pt_threshold_offset_K,Data threshold offset.
	u8	peak_threshold_shift_x;		    //BCB:Daimond_peak_shift_X,Data threshold shift.
	u8	peak_threshold_offset_x;	    //BCB:Daimond_peak_offset_X,Data threshold offset.
	u8	peak_threshold_shift_y;		    //BCB:Daimond_peak_shift_Y,Data threshold shift.
	u8	peak_threshold_offset_y;	    //BCB:Daimond_peak_offset_Y,Data threshold offset.
	u8	mutual_threshold_shift;		    //BCB:Daimond_mutual_threshold_shift,Data threshold shift.
	u8	mutual_threshold_offset;	//BCB:Daimond_mutual_threshold_offset,Data threshold offset.
	u8	filter_rate;                    	//BCB:filter_rate,
	u16	filter_range_1;                 	//BCB:filter_range1,
	u16	filter_range_2;	              //BCB:filter_range2,
	//
	u8  Bar_X_RAW;                      	//BCB:Bar_X_RAW,
	u8  Bar_X_Raw_2_Peak;             	//BCB:Bar_X_Raw_2_Peak,
	u8  Bar_X_Delta;                    	//BCB:Bar_X_Delta,
	u16 Bar_Y_Delta_2_Peak;           	//BCB:Bar_Y_Delta_2_Peak,
	u8  Border_Offset_X;                	//BCB:Border_Offset_X,
	u8  Border_Offset_Y;                	//BCB:Border_Offset_Y,
	
	u8	reserve[42];
	u8	cnt[CONFIG_PARAM_MAX_CHANNELS];
	u8	offset[CONFIG_PARAM_MAX_CHANNELS];
	u16	baseline[CONFIG_PARAM_MAX_CHANNELS];
	u8	mutual_baseline[MUTUAL_CALIBRATION_BASE_SIZE];		//used to store the mutual calibration baseline in no touch
} __attribute__ ((packed)); 

#define SITRONIX_MT_ENABLED

#define STX_TS_MAX_RES_SHIFT	(11)
#define STX_TS_MAX_VAL		((1 << STX_TS_MAX_RES_SHIFT) - 1)

#define ST1232_FLASH_SIZE	0x3C00
#define ST1232_ISP_PAGE_SIZE	0x200
#define ST1232_ROM_PARAM_ADR 0x3E00

// ISP command.
#define STX_ISP_ERASE		0x80
#define STX_ISP_SEND_DATA	0x81
#define STX_ISP_WRITE_FLASH	0x82
#define STX_ISP_READ_FLASH	0x83
#define STX_ISP_RESET		0x84
#define STX_ISP_READY		0x8F

typedef struct {
	u8	y_h		: 3,
		reserved	: 1,
		x_h		: 3,
		valid		: 1;
	u8	x_l;
	u8	y_l;
} xy_data_t;

typedef struct {
	u8	fingers		: 3,
		gesture		: 5;
	u8	keys;
	xy_data_t	xy_data[2];
} stx_report_data_t;

static u8 isp_page_buf[ST1232_ISP_PAGE_SIZE];

static int sitronix_ts_get_fw_version(struct i2c_client *client, u32 *ver)
{
	char buf[1];
	int ret;

	buf[0] = 0x0;	//Set Reg. address to 0x0 for reading FW version.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	//Read 1 byte FW version from Reg. 0x0 set previously.
	if ((ret = i2c_master_recv(client, buf, 1)) != 1)
		return -EIO;

	*ver = (u32) buf[0];

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;
}

static int sitronix_ts_get_fw_revision(struct i2c_client *client, u32 *rev)
{
	char buf[4];
	int ret;
	printk("[Touch]sitronix_ts_get_fw_revision\r\n");

	buf[0] = 0xC;	//Set Reg. address to 0x0 for reading FW version.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	//Read 1 byte FW version from Reg. 0x0 set previously.
	if ((ret = i2c_master_recv(client, buf, 4)) != 4)
		return -EIO;

	*rev = ((u32)buf[3]);
	*rev |= (((u32)buf[2]) << 8);
	*rev |= (((u32)buf[1]) << 16);
	*rev |= (((u32)buf[0]) << 24);

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;

}

static int sitronix_ts_set_autotune_en(struct i2c_client *client)
{
	char buf[2], device_ctrl;
	int ret; 
	if(CanUpdate) {
	printk("AutoTune enable\n");
	buf[0] = 0x02;	// Set Reg. address to 0x02 for reading Device Control
	if((ret = i2c_master_send(client, buf, 1)) != 1)
	{
		printk("[FW]sitronix_ts_set_autotune_en1\r\n");		
		goto io_err;
	}	
	// Read 1 byte Device Control from Reg. 0x02
	if((ret = i2c_master_recv(client, &device_ctrl, 1)) != 1)
	{
		printk("[FW]sitronix_ts_set_autotune_en2\r\n");		
		goto io_err;
	}	
	device_ctrl |= 0x80;
	
        buf[0] = 0x2;        //Set Reg. address to 0x2 for enable autotune 
	buf[1] = device_ctrl;
	if ((ret = i2c_master_send(client, buf, 2)) != 2)
	{
		printk("[FW]sitronix_ts_set_autotune_en3\r\n");		
		goto io_err;
	}	
	mdelay(100);

        // Check autotune enable bit 
	
	/*
        count = 0;         
        // Read 1 byte Device Control from Reg. 0x02 
        do { 
                msleep(5); 
                if((ret = i2c_master_recv(client, buf, 1)) != 1) 
                        goto io_err; 
                count++; 
        } while((!(buf[0] & 0x80)) && (count <= 50000)); 
        
        if ( count > 50000 ) 
                printk("Enable Autotune Timeout!\n"); 
	*/
	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
	{
		printk("[FW]sitronix_ts_set_autotune_en4\r\n");		
		goto io_err;
	}	
	}

	return 0;
io_err:
	printk("[DEBUG_MSG] TEST!\n"); 
	led_flag=1;
	printk("[FW]sitronix_ts_set_autotune_en\r\n");

	WARN_ON(true);
		return -EIO;
	}

static int sitronix_ts_get_status(struct i2c_client *client)
{
	char buf[2], status;
	int ret;
	buf[0] = 0x01;	// Set Reg. address to 0x01 for reading Device Status
	if((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;
	
	// Read 1 byte Device Control from Reg 0x01
	if((ret = i2c_master_recv(client, &status, 1)) != 1)
		return -EIO;

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	printk(".");
	return status;
	
}
#if 0//brad remove
static int sitronix_ts_get_resolution(struct i2c_client *client, u16 *x_res, u16 *y_res)
{
	char buf[3];
	int ret;
	printk("[Touch]sitronix_ts_get_resolution\r\n");
	buf[0] = 0x4;	//Set Reg. address to 0x4 for reading XY resolution.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	//Read 3 byte XY resolution from Reg. 0x4 set previously.
	if ((ret = i2c_master_recv(client, buf, 3)) != 3)
		return -EIO;

	*x_res = ((buf[0] & 0xF0) << 4) | buf[1];
	*y_res = ((buf[0] & 0x0F) << 8) | buf[2];

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;

}

static int sitronix_ts_set_resolution(struct i2c_client *client, u16 x_res, u16 y_res)
{
	char buf[4];
	int ret = 0;

	buf[0] = 0x4;	//Set Reg. address to 0x4 for reading XY resolution.
	buf[1] = ((u8)((x_res & 0x0700) >> 4)) | ((u8)((y_res & 0x0700) >> 8));
	buf[2] = ((u8)(x_res & 0xFF));
	buf[3] = ((u8)(y_res & 0xFF));
	if ((ret = i2c_master_send(client, buf, 4)) != 4)
		return -EIO;

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;
}

static int sitronix_ts_open(struct input_dev *dev)
{
	struct st1232_ts_data *priv = input_get_drvdata(dev);

	enable_irq(priv->irq);
        ++en_irq;
	return 0;
}

static void sitronix_ts_close(struct input_dev *dev)
{
	struct st1232_ts_data *priv = input_get_drvdata(dev);

	disable_irq(priv->irq);
}
#endif
static int st1232_isp_erase(struct i2c_client *client, u8 page_num)
{
	u8 data[8];

	data[0] = STX_ISP_ERASE;
	data[1] = 0x0;
	data[2] = page_num;

	if (i2c_master_send(client, data, sizeof(data)) != sizeof(data)) {
		dev_err(&client->dev, "%s(%u): ISP erase page(%u) failed!\n",
						__FUNCTION__, __LINE__, (unsigned int)page_num);
		led_flag=1;
		printk("[FW]st1232_isp_erase1\r\n");		
		return -EIO;
	}

	if (i2c_master_recv(client, data, sizeof(data)) != sizeof(data) || data[0] != STX_ISP_READY) {
		dev_err(&client->dev, "%s(%u): ISP read READY failed!\n", __FUNCTION__, __LINE__);
		led_flag=1;
		printk("[FW]st1232_isp_erase2\r\n");		
		return -EIO;
	}

	return 0;
}

static int st1232_isp_reset(struct i2c_client *client)
{
	u8 data[8];

	data[0] = STX_ISP_RESET;

	if (i2c_master_send(client, data, sizeof(data)) != sizeof(data)) {
		dev_err(&client->dev, "%s(%u): ISP reset chip failed!\n", __FUNCTION__, __LINE__);
		return -EIO;
	}

	mdelay(200);
	printk("*************************************** ISP reset ok! ***********************************************\n");
	return 0;
}

static int st1232_jump_to_isp(struct i2c_client *client)
{
	int i;
	u8 signature[] = "STX_FWUP";
	u8 buf[2];

	for (i = 0; i < strlen(signature); i++) {
		buf[0] = 0x0;
		buf[1] = signature[i];
		if (i2c_master_send(client, buf, 2) != 2) {
			dev_err(&client->dev, "%s(%u): Unable to write ISP STX_FWUP!\n", __FUNCTION__, __LINE__);
			led_flag=1;	
			printk("[FW]st1232_jump_to_isp\r\n");		

			return -EIO;
		}
		msleep(100);
	}
	if(!gpio_get_value(94)) {
		printk("*************************************** gpio 93 = 0! ***********************************************\n");
		i2c_master_recv(client, buf, sizeof(buf));
	}
	printk("*************************************** Jump to ISP ok! ***********************************************\n");

	return 0;
}

static int st1232_isp_read_page(struct i2c_client *client, char *page_buf, u8 page_num)
{
	u8 data[8];
	u32 rlen;

	memset(data, 0, sizeof(data));
	memset(page_buf, 0, ST1232_ISP_PAGE_SIZE);
	data[0] = STX_ISP_READ_FLASH;
	data[2] = page_num;
	if (i2c_master_send(client, data, sizeof(data)) != sizeof(data)) {
		dev_err(&client->dev, "%s(%u): ISP read flash failed!\n", __FUNCTION__, __LINE__);
		led_flag=1;		
		printk("[FW]st1232_isp_read_page1\r\n");				
		return -EIO;
	}
	rlen = 0;
	while (rlen < ST1232_ISP_PAGE_SIZE) {
		if (i2c_master_recv(client, (page_buf+rlen), sizeof(data)) != sizeof(data)) {
			dev_err(&client->dev, "%s(%u): ISP read data failed!\n", __FUNCTION__, __LINE__);
			led_flag=1;
			printk("[FW]st1232_isp_read_page2\r\n");		
			
			return -EIO;
		}
		rlen += 8;
	}


	return ST1232_ISP_PAGE_SIZE;
}

static u16 st1232_isp_cksum(char *page_buf)
{
	u16 cksum = 0;
	int i;
	for (i = 0; i < ST1232_ISP_PAGE_SIZE; i++) {
		cksum += (u16) page_buf[i];
	}
	return cksum;
}

static int st1232_isp_write_page(struct i2c_client *client, char *page_buf, u8 page_num)
{
	u8 data[8];
	int wlen;
	u32 len;
	u16 cksum;

	if (st1232_isp_erase(client, page_num) < 0) {
		return -EIO;
	}
	printk("[Touch]st1232_isp_write_page\r\n");

	cksum = st1232_isp_cksum(page_buf);

	data[0] = STX_ISP_WRITE_FLASH;
	data[2] = page_num;
	data[4] = (cksum & 0xFF);
	data[5] = ((cksum & 0xFF) >> 8);

	if (i2c_master_send(client, data, sizeof(data)) != sizeof(data)) {
		dev_err(&client->dev, "%s(%u): ISP write page failed!\n", __FUNCTION__, __LINE__);
		led_flag=1;		
		printk("[FW]st1232_isp_write_page1\r\n");		

		return -EIO;
	}

	data[0] = STX_ISP_SEND_DATA;
	wlen = ST1232_ISP_PAGE_SIZE;
	len = 0;
	while (wlen>0) {
		len = (wlen < 7) ? wlen : 7;
		memcpy(&data[1], page_buf, len);

		if (i2c_master_send(client, data, sizeof(data)) != sizeof(data)) {
			dev_err(&client->dev, "%s(%u): ISP send data failed!\n", __FUNCTION__, __LINE__);
			led_flag=1;	
			printk("[FW]st1232_isp_write_page2\r\n");		
			
			return -EIO;
		}

		wlen -= 7;
		page_buf += 7;
	}

	if (i2c_master_recv(client, data, sizeof(data)) != sizeof(data) || data[0] != STX_ISP_READY) {
		dev_err(&client->dev, "%s(%u): ISP read READY failed!\n", __FUNCTION__, __LINE__);
		led_flag=1;		
		printk("[FW]st1232_isp_write_page3\r\n");		
		
		return -EIO;
	}

	return ST1232_ISP_PAGE_SIZE;
}

static int st1232_isp_read_flash(struct i2c_client *client, char *buf, loff_t off, size_t count)
{
	u32 page_num, page_off;
	u32 len = 0, rlen = 0;

	page_num = off / ST1232_ISP_PAGE_SIZE;
	page_off = off % ST1232_ISP_PAGE_SIZE;
	printk("[Touch]st1232_isp_read_flash\r\n");

	if (page_off) {

		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return len;

		len -= page_off;

		len = (count > len ? len : count);

		memcpy(buf, (isp_page_buf + page_off), len);
		buf += len;
		count -= len;
		rlen += len;
		page_num++;
	}

	while (count) {
		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return len;

		len = (count > len ? len : count);

		memcpy(buf, isp_page_buf, len);

		buf += len;
		count -= len;
		rlen += len;
		page_num++;
	}

	return rlen;
}

static ssize_t st1232_flash_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	int rc;

	dev_dbg(&client->dev, "%s(%u): buf=%p, off=%lli, count=%zi)\n", __FUNCTION__, __LINE__, buf, off, count);
	printk("[Touch]st1232_flash_read\r\n");

	if (off >= ST1232_FLASH_SIZE)
		return 0;

	if (off + count > ST1232_FLASH_SIZE)
		count = ST1232_FLASH_SIZE - off;

	rc = st1232_isp_read_flash(client, buf, off, count);

	if (rc < 0)
		return -EIO;

	return rc;
}

static int st1232_isp_write_flash(struct i2c_client *client, char *buf, loff_t off, size_t count)
{
	u8 page_num, page_off;
	u32 len, wlen = 0;
	int rc;

	page_num = off / ST1232_ISP_PAGE_SIZE;
	page_off = off % ST1232_ISP_PAGE_SIZE;
	printk("[Touch]st1232_isp_write_flash\r\n");

	if (page_off) {

		// Start RMW.
		// Read page.
		if ((rc = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return rc;

		len = ST1232_ISP_PAGE_SIZE - page_off;
		// Modify data.
		memcpy((isp_page_buf+page_off), buf, len);

		// Write back page.
		st1232_isp_write_page(client, isp_page_buf, page_num);

		buf += len;
		count -= len;
		wlen += len;
	}

	while (count) {
		if (count >= ST1232_ISP_PAGE_SIZE) {
			len = ST1232_ISP_PAGE_SIZE;
			memcpy(isp_page_buf, buf, len);

			if ((rc = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return rc;

			buf += len;
			count -= len;
			wlen += len;
		} else {
			// Start RMW.
			// Read page.
			if ((rc = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
				return rc;

			len = count;
			// Modify data.
			memcpy(isp_page_buf, buf, len);

			// Write back page.
			if ((rc = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return rc;

			buf += len;
			count -= len;
			wlen += len;
		}
		page_num++;
	}

	return wlen;
}

static ssize_t st1232_flash_write(struct file *filp,struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int rc;
	//printk("[Touch]st1232_flash_write:priv->Can_update=%d\r\n",priv->Can_update);

	if(priv->Can_update) {

	printk("[Touch]st1232_flash_write\r\n");

	dev_dbg(&client->dev, "%s(%u): buf=%p, off=%lli, count=%zi)\n", __FUNCTION__, __LINE__, buf, off, count);

	if (off >= ST1232_FLASH_SIZE)
		return -ENOSPC;

	if (off + count > ST1232_FLASH_SIZE)
		count = ST1232_FLASH_SIZE - off;

	
	rc = st1232_isp_write_flash(client, buf, off, count);

	if (rc < 0)
		return -EIO;

	return rc;
	} else {
		return count;
	}
}

static struct bin_attribute st1232_flash_bin_attr = {
	.attr = {
		.name = "flash",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = ST1232_FLASH_SIZE,
	.read = st1232_flash_read,
	.write = st1232_flash_write,
};

static ssize_t st1232_panel_config_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	return count;
}


static ssize_t st1232_panel_config_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = kobj_to_i2c_client(kobj);
	int i;
	u8 page_num;
	u32 len;
	if(CanUpdate) {
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	dev_dbg(&client->dev, "%s(%u): buf=%p, off=%lli, count=%zi)\n", __FUNCTION__, __LINE__, buf, off, count);
	
	printk("input panel config\n");
	for(i = 0 ; i < 32 ; i++) {
		printk("%02x %02x %02x %02x %02x %02x %02x %02x ", buf[i*16], buf[i*16+1], buf[i*16+2], buf[i*16+3], buf[i*16+4], buf[i*16+5], buf[i*16+6], buf[i*16+7]);
		printk("%02x %02x %02x %02x %02x %02x %02x %02x\n", buf[i*16+8], buf[i*16+9], buf[i*16+10], buf[i*16+11], buf[i*16+12], buf[i*16+13], buf[i*16+14], buf[i*16+15]);
	}
	
	//Write back
	if ((len = st1232_isp_write_page(client, buf, page_num)) < 0)
	   {
	   			led_flag=1;
				printk("[FW]st1232_panel_config_write\r\n");		
				
				return -EIO;
	    }			
	}
	return count;
}

static struct bin_attribute st1232_panel_bin_attr = {
	.attr = {
		.name = "panel_config",
		.mode = S_IRUGO | S_IWUSR,
	},
	.size = ST1232_ISP_PAGE_SIZE,
	.read = st1232_panel_config_read,
	.write = st1232_panel_config_write,
};

static ssize_t sitronix_ts_isp_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	printk("[Touch]sitronix_ts_isp_ctrl_show\r\n");

	return sprintf(buf, "%d\n", priv->isp_enabled);
}

static ssize_t sitronix_ts_isp_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int enabled = 0;
	if(priv->Can_update) {
	sscanf(buf, "%x", &enabled);
	printk("[Touch]sitronix_ts_isp_ctrl_store:priv->isp_enabled=%d  enabled=%d priv->Can_update=%d\r\n",priv->isp_enabled,enabled,priv->Can_update);
	if (priv->isp_enabled && !enabled) {
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);
	} else if (!priv->isp_enabled && enabled) {
		//Jump to ISP.
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	}
	return count;
}

static DEVICE_ATTR(isp_ctrl, 0644, sitronix_ts_isp_ctrl_show, sitronix_ts_isp_ctrl_store);

static ssize_t sitronix_ts_revision_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u32 rev;
	int err;

	if ((err = sitronix_ts_get_fw_revision(client, &rev))) {
		dev_err(&client->dev, "Unable to get FW revision!\n");
		return 0;
	}
	printk("[Touch]sitronix_ts_revision_show=%d\r\n",rev);

	return sprintf(buf, "%u\n", rev);
}

static DEVICE_ATTR(revision, 0644, sitronix_ts_revision_show, NULL);

static ssize_t sitronix_ts_struct_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 page_num;
	u32 len;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);

	return sprintf(buf, "%u\n", isp_page_buf[1]);
}

static DEVICE_ATTR(struct_version, 0644, sitronix_ts_struct_version_show, NULL);

/*
 * sitronix data threshold show & store
 */
static ssize_t sitronix_ts_data_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 x_shift, y_shift, k_shift;
	u8 x_offset, y_offset, k_offset;
	u8 page_num;
	u32 len;

	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	printk("[Touch] sitronix_ts_data_threshold_show\r\n");

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		x_shift = pROM_v0->data_threshold_shift_x;
		x_offset = pROM_v0->data_threshold_offset_x;
		y_shift = pROM_v0->data_threshold_shift_y;
		y_offset = pROM_v0->data_threshold_offset_y;
		k_shift = pROM_v0->data_threshold_shift_k;
		k_offset = pROM_v0->data_threshold_offset_k;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		x_shift = pROM_v1->data_threshold_shift_x;
		x_offset = pROM_v1->data_threshold_offset_x;
		y_shift = pROM_v1->data_threshold_shift_y;
		y_offset = pROM_v1->data_threshold_offset_y;
		k_shift = pROM_v1->data_threshold_shift_k;
		k_offset = pROM_v1->data_threshold_offset_k;
	}
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	
	return sprintf(buf, "%u %u %u %u %u %u\n", x_shift, x_offset, y_shift, y_offset, k_shift, k_offset);
}

static ssize_t sitronix_ts_data_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 x_shift, y_shift, k_shift;
	u32 x_offset, y_offset, k_offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	printk("[Touch] data_threshold:Adjustsensitivity=%d\r\n",Adjustsensitivity);
	if(Adjustsensitivity==1)
	{
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u %u %u %u %u %u", &x_shift, &x_offset, &y_shift, &y_offset, &k_shift, &k_offset);

	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;
	
	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->data_threshold_shift_x = x_shift&0xFF;
		pROM_v0->data_threshold_offset_x = x_offset&0xFF;
		pROM_v0->data_threshold_shift_y = y_shift&0xFF;
		pROM_v0->data_threshold_offset_y = y_offset&0xFF;
		pROM_v0->data_threshold_shift_k = k_shift&0xFF;
		pROM_v0->data_threshold_offset_k = k_offset&0xFF;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->data_threshold_shift_x = x_shift&0xFF;
		pROM_v1->data_threshold_offset_x = x_offset&0xFF;
		pROM_v1->data_threshold_shift_y = y_shift&0xFF;
		pROM_v1->data_threshold_offset_y = y_offset&0xFF;
		pROM_v1->data_threshold_shift_k = k_shift&0xFF;
		pROM_v1->data_threshold_offset_k = k_offset&0xFF;
	}
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	}
	return count;
}

static DEVICE_ATTR(data_threshold, 0644, sitronix_ts_data_threshold_show, sitronix_ts_data_threshold_store);

/*
 * sitronix point threshold show & store
 */
static ssize_t sitronix_ts_point_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 shift, offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	printk("[Touch] sitronix_ts_point_threshold_show\r\n");

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		shift = pROM_v0->pt_threshold_shift;
		offset = pROM_v0->pt_threshold_offset;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		shift = pROM_v1->pt_threshold_shift;
		offset = pROM_v1->pt_threshold_offset;
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return sprintf(buf, "%u %u\n", shift, offset);
}

static ssize_t sitronix_ts_point_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{


	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 shift, offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;

	timecounter[index_jiffies]=jiffies;
	printk("[Touch]jiffies=%lu HZ=%d\r\n",jiffies,HZ);
	printk("[Touch]point_threshold:Adjustsensitivity=%d \r\n",Adjustsensitivity);

	if(index_jiffies==0)
		Adjustsensitivity=1;
	if(index_jiffies==1)
	{
		diff_time=timecounter[1]-timecounter[0];
		if(diff_time>400)
			Adjustsensitivity=1;
		else
			Adjustsensitivity=0;
		printk("[Touch][%lu %lu],  [%d %d] [%lu %d] Adjustsensitivity=%d\r\n",timecounter[1],timecounter[0],jiffies_to_msecs(timecounter[1]),jiffies_to_msecs(timecounter[0]),diff_time,jiffies_to_msecs(diff_time),Adjustsensitivity);
	}
	++index_jiffies;
	if(index_jiffies==2)
	{
		timecounter[0]=timecounter[1];
		--index_jiffies;
	}
	if(Adjustsensitivity==1)
	{
	atomic_dec(&st1232_ts->irq_disable);
	disable_irq_nosync(st1232_ts->irq);
	dis_irq++;

	
	CanUpdate=1;
	printk("[Touch] point_threshold: Disable_irq=%lu CanUpdate=%d\r\n",dis_irq,CanUpdate);
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u %u", &shift, &offset);
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;
	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->pt_threshold_shift = shift&0xFF;
		pROM_v0->pt_threshold_offset = offset&0xFF;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->pt_threshold_shift = shift&0xFF;
		pROM_v1->pt_threshold_offset = offset&0xFF;
	}
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	}
	return count;
	
}

static DEVICE_ATTR(point_threshold, 0644, sitronix_ts_point_threshold_show, sitronix_ts_point_threshold_store);

/*
 * sitronix peak threshold show & store
 */
static ssize_t sitronix_ts_peak_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 x_shift, y_shift;
	u8 x_offset, y_offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	printk("[Touch] sitronix_ts_peak_threshold_show\r\n");

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00)
	{
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		x_shift = pROM_v0->peak_threshold_shift_x;
		x_offset = pROM_v0->peak_threshold_offset_x;
		y_shift = pROM_v0->peak_threshold_shift_y;
		y_offset = pROM_v0->peak_threshold_offset_y;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		x_shift = pROM_v1->peak_threshold_shift_x;
		x_offset = pROM_v1->peak_threshold_offset_x;
		y_shift = pROM_v1->peak_threshold_shift_y;
		y_offset = pROM_v1->peak_threshold_offset_y;
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);

	return sprintf(buf, "%u %u %u %u\n", x_shift, x_offset, y_shift, y_offset);
}

static ssize_t sitronix_ts_peak_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 x_shift, y_shift;
	u32 x_offset, y_offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	printk("[Touch]peak_threshold: Adjustsensitivity=%d\r\n",Adjustsensitivity);
	if(Adjustsensitivity==1)
	{
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u %u %u %u", &x_shift, &x_offset, &y_shift, &y_offset);
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->peak_threshold_shift_x = x_shift&0xFF;
		pROM_v0->peak_threshold_offset_x = x_offset&0xFF;
		pROM_v0->peak_threshold_shift_y = y_shift&0xFF;
		pROM_v0->peak_threshold_offset_y = y_offset&0xFF;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->peak_threshold_shift_x = x_shift&0xFF;
		pROM_v1->peak_threshold_offset_x = x_offset&0xFF;
		pROM_v1->peak_threshold_shift_y = y_shift&0xFF;
		pROM_v1->peak_threshold_offset_y = y_offset&0xFF;
	}
	
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	CanUpdate=0;

	enable_irq(st1232_ts->irq);
	atomic_inc(&st1232_ts->irq_disable);
	++en_irq;

	printk("[Touch] peak_threshold: Enable_irq=%lu  CanUpdate=%d\r\n",en_irq,CanUpdate);
	
	}
	return count;
}

static DEVICE_ATTR(peak_threshold, 0644, sitronix_ts_peak_threshold_show, sitronix_ts_peak_threshold_store);

/*
 * sitronix mutual threshold show & store
 */
static ssize_t sitronix_ts_mutual_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 shift, offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00)
	{
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		shift = pROM_v0->mutual_threshold_shift;
		offset = pROM_v0->mutual_threshold_offset;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		shift = pROM_v1->mutual_threshold_shift;
		offset = pROM_v1->mutual_threshold_offset;
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	
	return sprintf(buf, "%u %u\n", shift, offset);
}

static ssize_t sitronix_ts_mutual_threshold_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 shift, offset;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u %u", &shift, &offset);
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->mutual_threshold_shift = shift&0xFF;
		pROM_v0->mutual_threshold_offset = offset&0xFF;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->mutual_threshold_shift = shift&0xFF;
		pROM_v1->mutual_threshold_offset = offset&0xFF;
	}
	
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return count;
}

static DEVICE_ATTR(mutual_threshold, 0644, sitronix_ts_mutual_threshold_show, sitronix_ts_mutual_threshold_store);

/*
 * sitronix range filter show & store
 */
static ssize_t sitronix_ts_range_filter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 rate;
	u16 range1,range2;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		range1 = ((pROM_v0->filter_range_1 & 0xFF)<<8) | (pROM_v0->filter_range_1>>8);
		range2 = ((pROM_v0->filter_range_2 & 0xFF)<<8) | (pROM_v0->filter_range_2>>8);
		rate = pROM_v0->filter_rate;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		range1 = ((pROM_v1->filter_range_1 & 0xFF)<<8) | (pROM_v1->filter_range_1>>8);
		range2 = ((pROM_v1->filter_range_2 & 0xFF)<<8) | (pROM_v1->filter_range_2>>8);
		rate = pROM_v1->filter_rate;
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);

	return sprintf(buf, "%u %u %u\n", range1, range2, rate);
}

static ssize_t sitronix_ts_range_filter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 range1,range2,rate;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u %u %u", &range1, &range2, &rate);
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->filter_range_1 = ((range1&0xFF)<<8) | ((range1>>8)&0xFF);
		pROM_v0->filter_range_2 = ((range2&0xFF)<<8) | ((range2>>8)&0xFF);
		pROM_v0->filter_rate = rate&0xFF;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->filter_range_1 = ((range1&0xFF)<<8) | ((range1>>8)&0xFF);
		pROM_v1->filter_range_2 = ((range2&0xFF)<<8) | ((range2>>8)&0xFF);
		pROM_v1->filter_rate = rate&0xFF;
	}
	
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return count;
}

static DEVICE_ATTR(range_filter, 0644 , sitronix_ts_range_filter_show, sitronix_ts_range_filter_store);

/*
 * sitronix barX filter show & store
 */
static ssize_t sitronix_ts_barX_filter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 RAW , RAW_2Peak , Delta;
	u8 page_num;
	int len;
	struct config_param_v1 *pROM_v1;
	if(priv->struct_version != 0x00) {
		page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
		//Jump to ISP.
		if(!priv->isp_enabled) {
			priv->isp_enabled = true;
			st1232_jump_to_isp(client);
		}
	
		//Read data from ROM
		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		RAW = pROM_v1->Bar_X_RAW;
		RAW_2Peak = pROM_v1->Bar_X_Raw_2_Peak;
		Delta = pROM_v1->Bar_X_Delta;
	
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);

		return sprintf(buf, "%u %u %u\n", RAW, RAW_2Peak, Delta);
	} else {
		return sprintf(buf, "Not Support!!\n");
	}
}

static ssize_t sitronix_ts_barX_filter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 RAW , RAW_2Peak , Delta;
	u8 page_num;
	int len;
	struct config_param_v1 *pROM_v1;
	
	if(priv->struct_version != 0x00) {
		page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
		sscanf(buf, "%u %u %u", &RAW, &RAW_2Peak, &Delta);
	
		//Jump to ISP.
		if(!priv->isp_enabled) {
			priv->isp_enabled = true;
			st1232_jump_to_isp(client);
		}
	
		//Read data from ROM
		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->Bar_X_RAW = RAW&0xFF;
		pROM_v1->Bar_X_Raw_2_Peak = RAW_2Peak&0xFF;
		pROM_v1->Bar_X_Delta = Delta&0xFF;
	
	
		//Write back
		if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);
	}		
	return count;
}

static DEVICE_ATTR(barX_filter, 0644 , sitronix_ts_barX_filter_show, sitronix_ts_barX_filter_store);


/*
 * sitronix barY filter show & store
 */
static ssize_t sitronix_ts_barY_filter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u16 RAW_2Peak;
	u8  shift , offset;
	u8 page_num;
	int len;
	struct config_param_v1 *pROM_v1;
	
	if(priv->struct_version != 0x00) {
		page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
		//Jump to ISP.
		if(!priv->isp_enabled) {
			priv->isp_enabled = true;
			st1232_jump_to_isp(client);
		}
	
		//Read data from ROM
		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;

		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		RAW_2Peak =( (pROM_v1->Bar_Y_Delta_2_Peak & 0xFF)<<8) | (pROM_v1->Bar_Y_Delta_2_Peak>>8);
		shift = pROM_v1->peak_threshold_shift_y;
		offset = pROM_v1->peak_threshold_offset_y;
	
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);
	
		return sprintf(buf, "%u %u %u\n", RAW_2Peak, shift, offset);
	} else {
		return sprintf(buf, "Not Support!!\n");
	}
}

static ssize_t sitronix_ts_barY_filter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 RAW_2Peak , shift , offset;
	u8 page_num;
	int len;
	struct config_param_v1 *pROM_v1;
	
	if(priv->struct_version != 0x00) {
		page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
		sscanf(buf, "%u %u %u", &RAW_2Peak, &shift, &offset);
	
		//Jump to ISP.
		if(!priv->isp_enabled) {
			priv->isp_enabled = true;
			st1232_jump_to_isp(client);
		}
	
		//Read data from ROM
		if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;

		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->Bar_Y_Delta_2_Peak = ((RAW_2Peak&0xFF)<<8) | ((RAW_2Peak>>8)&0xFF);
		pROM_v1->peak_threshold_shift_y = shift&0xFF;
		pROM_v1->peak_threshold_offset_y = offset&0xFF;
	
	
		//Write back
		if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
					return -EIO;
	
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);
	}
	return count;
}

static DEVICE_ATTR(barY_filter, 0644 , sitronix_ts_barY_filter_show, sitronix_ts_barY_filter_store);
/*
 * sitronix resolution show & store
 */
static ssize_t sitronix_ts_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u16 x_res, y_res;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	printk("[Touch]sitronix_ts_resolution_show\r\n");

	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		x_res = ((pROM_v0->x_res&0xFF)<<8) | (pROM_v0->x_res>>8);
		y_res = ((pROM_v0->y_res&0xFF)<<8) | (pROM_v0->y_res>>8);
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		x_res = ((pROM_v1->x_res&0xFF)<<8) | (pROM_v1->x_res>>8);
		y_res = ((pROM_v1->y_res&0xFF)<<8) | (pROM_v1->y_res>>8);
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return sprintf(buf, "%u %u\n", x_res, y_res);
}

static ssize_t sitronix_ts_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u32 x_res, y_res;
	u8 page_num;
	int len;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	sscanf(buf, "%u%u", &x_res, &y_res);
	printk("[Touch]sitronix_ts_resolution_store\r\n");

	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;

	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		pROM_v0->x_res = ((x_res&0xFF)<<8) | ((x_res>>8)&0xFF);
		pROM_v0->y_res = ((y_res&0xFF)<<8) | ((y_res>>8)&0xFF);
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		pROM_v1->x_res = ((x_res&0xFF)<<8) | ((x_res>>8)&0xFF);
		pROM_v1->y_res = ((y_res&0xFF)<<8) | ((y_res>>8)&0xFF);
	}
	
	//Write back
	if ((len = st1232_isp_write_page(client, isp_page_buf, page_num)) < 0)
				return -EIO;
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return count;
}

static DEVICE_ATTR(resolution, 0644 , sitronix_ts_resolution_show, sitronix_ts_resolution_store);

static ssize_t sitronix_ts_channel_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 page_num;
	u8 x_chs , y_chs , k_chs;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
		x_chs = pROM_v0->x_chs;
		y_chs = pROM_v0->y_chs;
		k_chs = pROM_v0->k_chs;
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
		x_chs = pROM_v1->x_chs;
		y_chs = pROM_v1->y_chs;
		k_chs = pROM_v1->k_chs;
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return sprintf(buf, "%u %u %u\n", x_chs, y_chs, k_chs);
}
static DEVICE_ATTR(channel_num, 0644, sitronix_ts_channel_num_show, NULL);

static ssize_t sitronix_ts_point_report_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	char Point_buf[8];
	int i ,ret;

	for(i = 0 ; i < 20 ; i++) {
		Point_buf[0] = 0x10;	//Set Reg. address to 0x10 for reading point report.
		if ((ret = i2c_master_send(client, Point_buf, 1)) != 1) {
			printk("I2C Send Data Fail\n");
			return 0;
		}

		//Read 8 byte point data from Reg. 0x10 set previously.
		if ((ret = i2c_master_recv(client, Point_buf, 8)) != 8) {
			printk("I2C Read Data Fail\n");
			return 0;
		}
		
		printk("Buf[0] : 0x%02X\n", Point_buf[0]);
		printk("Buf[1] : 0x%02X\n", Point_buf[1]);
		printk("Buf[2] : 0x%02X\n", Point_buf[2]);
		printk("Buf[3] : 0x%02X\n", Point_buf[3]);
		printk("Buf[4] : 0x%02X\n", Point_buf[4]);
		printk("Buf[5] : 0x%02X\n", Point_buf[5]);
		printk("Buf[6] : 0x%02X\n", Point_buf[6]);
		printk("Buf[7] : 0x%02X\n", Point_buf[7]);
		msleep(100);
		
	}
	return 0;	
}
static DEVICE_ATTR(point_report, 0644, sitronix_ts_point_report_show, NULL);

static ssize_t sitronix_ts_para_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 page_num;
	int len , i;
	
	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
			return -EIO;
	
	for(i = 0 ; i < 32 ; i++) {
		printk("%02x %02x %02x %02x %02x %02x %02x %02x ",isp_page_buf[16*i], isp_page_buf[16*i+1], isp_page_buf[16*i+2], isp_page_buf[16*i+3], isp_page_buf[16*i+4], isp_page_buf[16*i+5], isp_page_buf[16*i+6], isp_page_buf[16*i+7]);
		printk("%02x %02x %02x %02x %02x %02x %02x %02x\n",isp_page_buf[16*i+8], isp_page_buf[16*i+9], isp_page_buf[16*i+10], isp_page_buf[16*i+11], isp_page_buf[16*i+12], isp_page_buf[16*i+13], isp_page_buf[16*i+14], isp_page_buf[16*i+15]);
	}
	
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	return 0;
}
static DEVICE_ATTR(para, 0644 , sitronix_ts_para_show, NULL);


static int sitronix_ts_autotune_result_check(struct device *dev, int count_low, int count_high, int offset_low, int offset_high, int base_low, int base_high)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 page_num;
	int check_num,len, i, baseline;
	struct config_param_v0 *pROM_v0;
	struct config_param_v1 *pROM_v1;
	priv->autotune_result = true;

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;

	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
	{
			led_flag=1;
			printk("[FW]sitronix_ts_autotune_result_check\r\n");		

			return -EIO;
	}		
	if(priv->struct_version == 0x00) {
		pROM_v0 = (struct config_param_v0 *)(&isp_page_buf);
	
		check_num = pROM_v0->x_chs + pROM_v0->y_chs + pROM_v0->k_chs;
	
		for(i = 0 ; i < check_num ; i++) {
			//Check Cnt
			if(i < pROM_v0->x_chs) {
				if(((pROM_v0->cnt[i] & 0x1F) < count_low) || ((pROM_v0->cnt[i] & 0x1F) > count_high)) {
					printk("invalid X[%d]_p1 = 0x%02X\n", i , pROM_v0->cnt[i] & 0x1F);
					priv->autotune_result = false;
				}
			} else if(i < pROM_v0->x_chs + pROM_v0->y_chs) {
				if(((pROM_v0->cnt[i] & 0x1F) < count_low) || ((pROM_v0->cnt[i] & 0x1F) > count_high)) {
					printk("invalid Y[%d]_p1 = 0x%02X\n", i-pROM_v0->x_chs , pROM_v0->cnt[i] & 0x1F);			
					priv->autotune_result = false;
				}
			} else {
				if(((pROM_v0->cnt[i] & 0x1F) < 0x06) || ((pROM_v0->cnt[i] & 0x1F) > 0x1E)) {
					printk("invalid K[%d]_p1 = 0x%02X\n", i-pROM_v0->x_chs-pROM_v0->y_chs , pROM_v0->cnt[i] & 0x1F);
					priv->autotune_result = false;
				}
			}
			//Check Offset
			if((pROM_v0->offset[i] < offset_low) || (pROM_v0->offset[i] > offset_high)) {
				if(i < pROM_v0->x_chs) {
					printk("invalid X[%d]_p2 = 0x%02X\n", i , pROM_v0->offset[i]);
					priv->autotune_result = false;
				} else if(i < pROM_v0->x_chs + pROM_v0->y_chs) {
					printk("invalid Y[%d]_p2 = 0x%02X\n", i-pROM_v0->x_chs , pROM_v0->offset[i]);
					priv->autotune_result = false;
				} else {
					printk("invalid K[%d]_p2 = 0x%02X\n", i-pROM_v0->x_chs-pROM_v0->y_chs , pROM_v0->offset[i]);
					priv->autotune_result = false;
				}
			}
			//Check Baseline 
			baseline = ((pROM_v0->baseline[i] & 0xFF) << 8) | ((pROM_v0->baseline[i] & 0xFF00) >> 8);
			if((baseline < base_low) || (baseline > base_high)) {
				if(i < pROM_v0->x_chs) {
					printk("invalid X[%d]_p3 = 0x%02X\n", i , baseline);
					priv->autotune_result = false;
				} else if(i < pROM_v0->x_chs + pROM_v0->y_chs) {
					printk("invalid Y[%d]_p3 = 0x%02X\n", i-pROM_v0->x_chs , baseline);
					priv->autotune_result = false;
				} else {
					printk("invalid K[%d]_p3 = 0x%02X\n", i-pROM_v0->x_chs-pROM_v0->y_chs , baseline);
					priv->autotune_result = false;
				}
			}
		}
		printk("Channel num : %u\n" , check_num);
	} else {
		pROM_v1 = (struct config_param_v1 *)(&isp_page_buf);
	
		check_num = pROM_v1->x_chs + pROM_v1->y_chs + pROM_v1->k_chs;
	for(i = 0 ; i < check_num ; i++) {
		//Check Cnt
			if(i < pROM_v1->x_chs) {
				if(((pROM_v1->cnt[i] & 0x1F) < count_low) || ((pROM_v1->cnt[i] & 0x1F) > count_high)) {
					printk("invalid X[%d]_p1 = 0x%02X\n", i , pROM_v1->cnt[i] & 0x1F);
				priv->autotune_result = false;
			}
			} else if(i < pROM_v1->x_chs + pROM_v1->y_chs) {
				if(((pROM_v1->cnt[i] & 0x1F) < count_low) || ((pROM_v1->cnt[i] & 0x1F) > count_high)) {
					printk("invalid Y[%d]_p1 = 0x%02X\n", i-pROM_v1->x_chs , pROM_v1->cnt[i] & 0x1F);			
				priv->autotune_result = false;
			}
		} else {
				if(((pROM_v1->cnt[i] & 0x1F) < 0x06) || ((pROM_v1->cnt[i] & 0x1F) > 0x1E)) {
					printk("invalid K[%d]_p1 = 0x%02X\n", i-pROM_v1->x_chs-pROM_v1->y_chs , pROM_v1->cnt[i] & 0x1F);
				priv->autotune_result = false;
			}
		}
		//Check Offset
			if((pROM_v1->offset[i] < offset_low) || (pROM_v1->offset[i] > offset_high)) {
				if(i < pROM_v1->x_chs) {
					printk("invalid X[%d]_p2 = 0x%02X\n", i , pROM_v1->offset[i]);
				priv->autotune_result = false;
				} else if(i < pROM_v1->x_chs + pROM_v1->y_chs) {
					printk("invalid Y[%d]_p2 = 0x%02X\n", i-pROM_v1->x_chs , pROM_v1->offset[i]);
				priv->autotune_result = false;
			} else {
					printk("invalid K[%d]_p2 = 0x%02X\n", i-pROM_v1->x_chs-pROM_v1->y_chs , pROM_v1->offset[i]);
				priv->autotune_result = false;
			}
		}
		//Check Baseline 
			baseline = ((pROM_v1->baseline[i] & 0xFF) << 8) | ((pROM_v1->baseline[i] & 0xFF00) >> 8);
		if((baseline < base_low) || (baseline > base_high)) {
				if(i < pROM_v1->x_chs) {
				printk("invalid X[%d]_p3 = 0x%02X\n", i , baseline);
				priv->autotune_result = false;
				} else if(i < pROM_v1->x_chs + pROM_v1->y_chs) {
					printk("invalid Y[%d]_p3 = 0x%02X\n", i-pROM_v1->x_chs , baseline);
				priv->autotune_result = false;
			} else {
					printk("invalid K[%d]_p3 = 0x%02X\n", i-pROM_v1->x_chs-pROM_v1->y_chs , baseline);
				priv->autotune_result = false;
			}
		}
	}	
		printk("Channel num : %u\n" , check_num);
	}	
	
	if(priv->always_update) {
		return priv->always_update;
	} else {
		return priv->autotune_result;
	}
}

static ssize_t sitronix_ts_autotune_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	char status;
	int count_low, count_high, offset_low, offset_high, base_low, base_high;
	int update = 0;	
	int enabled = 0;
	int wait = 0;
	if(CanUpdate) {


	priv->always_update = false;
	sscanf(buf, "%x %x 0x%02x 0x%02x 0x%02x 0x%02x %u %u",&enabled, &update, &count_low, &count_high, &offset_low, &offset_high, &base_low, &base_high);

	printk("Check parameter : count[%02X:%02X]\toffset[%02X:%02X]\tbaseline[%u:%u]\n", count_low, count_high, offset_low, offset_high, base_low, base_high);
	
	if(enabled) {
		priv->autotune_result = false;
		priv->always_update = update;
		sitronix_ts_set_autotune_en(client);
                mdelay(200);
		status = sitronix_ts_get_status(client);
		while(status && (count < 50000)) {
			status = sitronix_ts_get_status(client);
			wait++;
			if(wait % 16 == 15)
				printk("\n");
                        msleep(100); 
		}
                if(wait <= 50000) { 
			printk("\nAutoTune Done!\n");
			sitronix_ts_autotune_result_check(dev, count_low, count_high, offset_low, offset_high, base_low, base_high);
		} else {
			printk("\nAutoTune Timeout!\n");
		}
	}
	
	}
	return count;
}

static DEVICE_ATTR(autotune_en, 0644, NULL, sitronix_ts_autotune_en_store);

static ssize_t sitronix_ts_autotune_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	
	if(priv->always_update) {
		return sprintf(buf, "%u\n", priv->always_update);
	} else {
		return sprintf(buf, "%u\n", priv->autotune_result);
	}
}

static DEVICE_ATTR(autotune_result, 0644, sitronix_ts_autotune_result_show, NULL);

static ssize_t sitronix_ts_debug_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	char Point_buf[256];
	int i, j, ret;

	for(i = 0 ; i < priv->I2C_RepeatTime ; i++) {
		Point_buf[0] = priv->I2C_Offset;	//Set Reg. address to 0x10 for reading point report.
		if ((ret = i2c_master_send(client, Point_buf, 1)) != 1) {
			printk("I2C Send Data Fail\n");
			return 0;
		}

		//Read 8 byte point data from Reg. 0x10 set previously.
		if ((ret = i2c_master_recv(client, Point_buf, priv->I2C_Length)) != priv->I2C_Length) {
			printk("I2C Read Data Fail\n");
			return 0;
		}
		
		printk("===================debug report===================\n");
		for(j = 0 ; j < priv->I2C_Length ; j++) {
			printk("Buf[%d] : 0x%02X\n",j, Point_buf[j]);
		}		
		msleep(100);
	}
	return 0;	
}

static ssize_t sitronix_ts_debug_check_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int Offset , Length , RepeatTime;

	sscanf(buf, "%u%u%u", &Offset, &Length, &RepeatTime);
	priv->I2C_Offset = Offset&0xFF;
	priv->I2C_Length = Length&0xFF;
	priv->I2C_RepeatTime = RepeatTime&0xFF;

	return count;
}
static DEVICE_ATTR(debug_check, 0644, sitronix_ts_debug_check_show, sitronix_ts_debug_check_store);

static ssize_t sitronix_ts_update_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct st1232_ts_data *priv = i2c_get_clientdata(client);
    u8 buf1[16],buf2[8];	
    int ret=0,rev,err,rc;
    char fwdata[25],fwdata1[1];
    struct file *fp,*fp1;
    mm_segment_t old_fs,old_fs1;

    if ((err = sitronix_ts_get_fw_revision(client, &rev)))
    {
        dev_err(&client->dev, "Unable to get FW revision!\n");		
    }

    buf1[0]=0x0;
    rc=i2c_master_send(st1232_ts->client, buf1,1);
    if(rc!=1)
        printk("[Touch]st1232  i2c_master_send  error\r\n");		
    rc= i2c_master_recv(st1232_ts->client, buf1,16);
    if(rc!=16)
        printk("[Touch]st1232  i2c_master_recv  error\r\n");
		
    printk("[Touch]updatefw :buf[0]=%d\r\n", buf1[0]);
    printk("[Touch]updatefw :buf[1]=%d\r\n", buf1[1]);
    printk("[Touch]updatefw :buf[2]=%d\r\n", buf1[2]);
    printk("[Touch]updatefw :buf[3]=%d\r\n", buf1[3]);
    printk("[Touch]updatefw :buf[4]=%d\r\n", buf1[4]);
    printk("[Touch]updatefw :buf[5]=%d\r\n", buf1[5]);
    printk("[Touch]updatefw :buf[6]=%d\r\n", buf1[6]);
    printk("[Touch]updatefw :buf[7]=%d\r\n", buf1[7]);
    printk("[Touch]updatefw :buf[8]=%d\r\n", buf1[8]);
    printk("[Touch]updatefw :buf[9]=%d\r\n", buf1[9]);
    printk("[Touch]updatefw :buf[10]=%d\r\n", buf1[10]);
    printk("[Touch]updatefw :buf[11]=%d\r\n", buf1[11]);
    printk("[Touch]updatefw :buf[12]=%d\r\n", buf1[12]);
    printk("[Touch]updatefw :buf[13]=%d\r\n", buf1[13]);
    printk("[Touch]updatefw :buf[14]=%d\r\n", buf1[14]);
    printk("[Touch]updatefw :buf[15]=%d\r\n", buf1[15]);
    buf2[0]=0x10;
    rc = i2c_master_send(st1232_ts->client, buf2,1);
    if(rc!=1)
    {
        printk("[Touch]st1232  i2c_master_send  error_3\r\n");
        tp_reset();
    }	
    //if(1)
    //printk("[Touch]rev=%d\n",rev);
    if((rev<1691&&rev>0)||(buf1[0]<6&&buf1[0]>0)||burnFw==1)
    {
        mdelay(5000);
        printk("[Touch]revision=%d vbatt=%d batt_capacity=%d\n",rev,vbatt,batt_capacity);
        //if(1)			
        if(vbatt>3700||batt_capacity>30||burnFw==1)
        {
            printk("[Touch]call nled\n");
            rpc_mpp_config_led_state(0, 2,150,150,0,0);
            printk("[Touch]Start to update touch firmware\n");
            priv->Can_update=1;
            CanUpdate=1;
            wake_lock(&tpwakelock_idle);
            printk("[Touch]wake_lock\n");
            memset(fwdata,0,sizeof(fwdata));
            burnFw=0;
            old_fs = get_fs();
            set_fs(KERNEL_DS);
            fp = filp_open("/data/touch_fw.txt", O_RDWR | O_APPEND | O_CREAT, 0666);
            sprintf(fwdata,"R=%d V=%d C=%d\n",rev,vbatt,batt_capacity);
            if(IS_ERR(fp))
            {
                printk("[Touch]Open file error...\n");
            }
	    else
	    {
            fp->f_op->write(fp,fwdata , 25 , &fp->f_pos);
	    }
            filp_close(fp,NULL);
            set_fs(old_fs);
            ret=1;					 
            old_fs1 = get_fs();
            set_fs(KERNEL_DS);
            fp1 = filp_open("/data/updatefw_status", O_RDWR |  O_CREAT, 0666);
            sprintf(fwdata1,"0");
            if(IS_ERR(fp1))
            {
                printk("[Touch]Open file error...\n");
            }
	    else
	    {
            fp1->f_op->write(fp1,fwdata1 , 1 , &fp1->f_pos);
	    }
            filp_close(fp1,NULL);
            set_fs(old_fs1);					 
        }	
    }
    else
    {
    CanUpdate=0;	
    priv->Can_update=0;
    ret=0;	
    }		
    printk("[Touch]Can_update=%d\n",priv->Can_update);
	
    return sprintf(buf, "%u\n", ret);	
}

static DEVICE_ATTR(update_enable, 0644, sitronix_ts_update_en_show, NULL);

static ssize_t sitronix_ts_update_fw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	unsigned char debug_log[100]={0};
	priv->Can_update=1;
	CanUpdate=1;
	burnFw=1;
	atomic_dec(&st1232_ts->irq_disable);
	disable_irq_nosync(st1232_ts->irq);
	dis_irq++;
	
	sprintf(debug_log,"update fw, dis irq:%lu\n",dis_irq);
	Touch_debug_save_log_to_ram(debug_log,1);
	
	printk("[Touch]Disable touch irq in sitronix_ts_update_fw \r\n");		
	
	printk("[Touch]sitronix_ts_update_fw:Can_update=%d  CanUpdate=%d  burnFw=%d\n",priv->Can_update,CanUpdate,burnFw);
	
	return sprintf(buf, "%u\n", priv->Can_update);	
}

static DEVICE_ATTR(update_fw, 0644, sitronix_ts_update_fw, NULL);


static ssize_t sitronix_ts_reset_fw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret=1 ,err;	 
        char fwdata[2]={0};
	struct file *fp;
	mm_segment_t old_fs;
	unsigned char debug_log[100]={0};

	if(CanUpdate==1)
	{
		wake_unlock(&tpwakelock_idle);
		printk("[Touch]wake_unlock\n");
	}	
	tp_reset();//modify for new firmware upgrade
	rpc_mpp_config_led_state(0, 1, 0, 0, 0, 0);	
	if(led_flag==1)
	{
		printk("[FW]update firmware but meet  i2c error!\n");
		rpc_mpp_config_led_state(0, 0, 0, 0, 0, 0);
	}
	printk("[FW]led_flag=%d!\n",led_flag);

	//finish burn fw
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open("/data/updatefw_status", O_RDWR | O_CREAT, 0666);
	if(led_flag==1)
		sprintf(fwdata,"2");
	else if(CanUpdate==1)
		sprintf(fwdata,"1");		
	if(IS_ERR(fp))
	{
	    printk("[Touch]Open file error...\n");
	}
	else
	{
	 fp->f_op->write(fp,fwdata , 1 , &fp->f_pos);
	}
	 filp_close(fp,NULL);
	 set_fs(old_fs);	
	
	CanUpdate=0;
	burnFw=0;
	enable_irq(st1232_ts->irq);
	atomic_inc(&st1232_ts->irq_disable);
	++en_irq;
	sprintf(debug_log,"ts_reset_fw, en irq:%lu\n",en_irq);
	Touch_debug_save_log_to_ram(debug_log,1);

	if ((err = sitronix_ts_get_fw_version(client, &tpversion))) {
		printk("[Touch]Unable to get FW version...\r\n");
	} else {
		printk("[Touch]FW version=%X... irq_disable : %X\r\n",tpversion , atomic_read(&st1232_ts->irq_disable));

	}
	printk("[Touch]Enable touch irq: sitronix_ts_reset_fw=%d CanUpdate=%d\n",ret,CanUpdate);
	return sprintf(buf, "%u\n", ret);	
}

static DEVICE_ATTR(reset_fw, 0644, sitronix_ts_reset_fw, NULL);


static int sitronix_ts_set_data_mode(struct i2c_client *client, u32 data_mode)
{
	char buf[2];
	int ret;

	//buf[0] = ((data_mode & 0x03) << 4 | 0x80);	//Set Reg. address to 0x2 for setting data mode.
	buf[0] = 0x02;
	buf[1] = ((data_mode & 0x03) << 4 | 0x08);	//Set Reg. address to 0x2 for setting data mode.
	if ((ret = i2c_master_send(client, buf, 2)) != 2)
		return -EIO;

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;

}

static int sitronix_ts_read_raw_data(struct i2c_client *client, u8 *raw_data, u32 len)
{
	char buf[1];
	int ret;

	buf[0] = 0x40;	//Set Reg. address to 0x40 for reading raw data.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	//Read raw data of length len.
	if ((ret = i2c_master_recv(client, raw_data, len)) != len)
		return -EIO;

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;

}

static ssize_t sitronix_ts_raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	char data_buf[32*2];
	int i, ret;
	u32 raw, len;

	printk("st1232 data mode = %d\n" , priv->data_mode);
	if (priv->data_mode == 0)
		return 0;

	if ((ret = sitronix_ts_set_data_mode(client, priv->data_mode)))
		return 0;

	
	sitronix_ts_read_raw_data(client, data_buf, sizeof(data_buf));

	for(i = 0, len = 0; i < sizeof(data_buf); i += 2) {
		raw = ((((u32)data_buf[i]) << 8) | data_buf[i+1]);
		len += sprintf((buf + len), "%u ", raw);
	}

	/*
	if ((ret = sitronix_ts_set_data_mode(client, 0)))
		return 0;
	*/
	return len;
}

static ssize_t sitronix_ts_raw_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);

	sscanf(buf, "%u", &priv->data_mode);

	return count;
}
static DEVICE_ATTR(raw_data, 0664, sitronix_ts_raw_data_show, sitronix_ts_raw_data_store);

static ssize_t sitronix_ts_gpio_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	
	printk("[Touch]sitronix_ts_gpio_reset\r\n");
	
	tp_reset_and_check(st1232_ts->client);

	return 0;
}

static DEVICE_ATTR(gpio_reset, 0644, sitronix_ts_gpio_reset, NULL);


static int sitronix_ts_set_proximity_en(struct i2c_client *client, u32 enable)
{
	char buf[2], data;
	int ret;

	// Get Device Control
	buf[0] = 0x2;
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	if ((ret = i2c_master_recv(client, buf, 1)) != 1)
		return -EIO;

	// Clear Proximity Bit
	data = buf[0];
	data &= 0xFB;
	
	// Write Device Control
	buf[0] = 0x02;
	buf[1] = ((enable & 0x01) << 2 | data);	
	if ((ret = i2c_master_send(client, buf, 2)) != 2)
		return -EIO;

	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
		return -EIO;

	return 0;
}

static ssize_t sitronix_ts_proximity_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int proximity_en;

	sscanf(buf, "%u", &proximity_en);
	
	sitronix_ts_set_proximity_en(client, proximity_en);
	printk("@@@Sitronix_ts_proximity_en_store, proximity_en=%d \r\n",proximity_en);	
	return count;
}
static DEVICE_ATTR(proximity_en, 0664, NULL, sitronix_ts_proximity_en_store);



static int sitronix_ts_set_sensitivity(struct i2c_client *client, u32 level)
{
	char buf[5];
	int ret;
	mm_segment_t old_fs;
	struct file *file;
	char test[1];
	// Write Device Control
	buf[0] = 0x08;
	if(level == LOW_SENSITIVE)
	{
		st1232_ts->sensitivity_level=0;
		buf[1] = 0x00;
		buf[2] = 0x5A;
		buf[3] = 0x20;
		buf[4] = 0x2C;
		printk("[Touch]LOW_SENSITIVE: 0x5A 0x20 0x2C, level=%d \r\n",st1232_ts->sensitivity_level);				
	}
	else if(level == MEDIUM_SENSITIVE)
	{
		st1232_ts->sensitivity_level=1;
		buf[1] = 0x00;
		buf[2] = 0x3C;
		buf[3] = 0x0F;
		buf[4] = 0x08;

		printk("[Touch]MEDIUM_SENSITIVE:0x00 3C 0F 08, level=%d \r\n",st1232_ts->sensitivity_level);						
	}
	else if(level == HIGH_SENSITIVE)
	{
		st1232_ts->sensitivity_level=2;
		buf[1] = 0x00;
		buf[2] = 0x23;
		buf[3] = 0x10;
		buf[4] = 0x2C;
		printk("[Touch]HIGH_SENSITIVE 0x23 0x10 0x2C, level=%d \r\n",st1232_ts->sensitivity_level);						
	}	
	
	if ((ret = i2c_master_send(st1232_ts->client, buf, 5)) != 5)
	{	printk("[Touch]sensitivity adjust error1 \r\n");					
		return -EIO;
	}
	buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(st1232_ts->client, buf, 1)) != 1)
	{	printk("[Touch]sensitivity adjust error2 \r\n");
		return -EIO;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	file = filp_open("/data/touch_sensitive", O_RDWR | O_CREAT, 0666);
	
	if(IS_ERR(file))
	{
	    printk("[Touch]Open file error...\n");
	}
	else
	{
	    sprintf(test,"%d",st1232_ts->sensitivity_level);
	 file->f_op->write(file,test , 1 , &file->f_pos);
	}
	 filp_close(file,NULL);
	 set_fs(old_fs);	

	return 0;
}

static ssize_t sitronix_ts_sensitivity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct st1232_ts_data *priv = i2c_get_clientdata(client);
	char data_buf[5];
	int Touch_threshold , Noise_threshold , Key_threshold;
	int ret;
	
	// Set Reg. address to 0x08 for sensitivity
	data_buf[0] = 0x8;
	if ((ret = i2c_master_send(client, data_buf, 1)) != 1)
		return -EIO;

	// Access sensitivity
	if ((ret = i2c_master_recv(client, data_buf, 4)) != 4)
		return -EIO;

	Touch_threshold = (data_buf[0]<<8) | data_buf[1];
	Noise_threshold = data_buf[2];
	Key_threshold = data_buf[3];
	
	data_buf[0] = 0x10;	//Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, data_buf, 1)) != 1)
		return -EIO;
	printk("[Touch]sitronix_ts_sensitivity_show=%u, %u, %u \r\n",Touch_threshold, Noise_threshold, Key_threshold);	
	return sprintf(buf, "%u %u %u\n", Touch_threshold, Noise_threshold, Key_threshold);
}

static ssize_t sitronix_ts_sensitivity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int sensitivity_level;

	sscanf(buf, "%u", &sensitivity_level);
	
	priv->sensitivity_level = sensitivity_level;
	printk("[Touch]sitronix_ts_sensitivity_store=%d \r\n",sensitivity_level);	
	sitronix_ts_set_sensitivity(client, sensitivity_level);
	return count;
}
static DEVICE_ATTR(sensitivity, 0664, sitronix_ts_sensitivity_show , sitronix_ts_sensitivity_store);

static ssize_t sitronix_ts_tp_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);

	printk("[Touch]tpversion=%x\r\n",tpversion);

	return sprintf(buf, "%d\n", tpversion);
}

static DEVICE_ATTR(tp_version, 0644, sitronix_ts_tp_version, NULL);
#define debug_buffer_size 307200
static unsigned char touch_debug_buffer[debug_buffer_size]={0};
static unsigned long touch_debug_buffer_index = 0;
static unsigned char  touch_debug_buffer_flag = 0;
static unsigned char DebugPageIndex=0;

static ssize_t Touch_debug_set_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk("@@Touch_debug_set_flag start\n");		
	touch_debug_buffer_flag =( buf[0]-0x30);
	printk("@@Touch_debug_set_flag end,buf[0]:%x,touch_debug_buffer_flag:%d\n",buf[0],touch_debug_buffer_flag);
	return 0;
}

static void Touch_debug_save_log_to_ram(unsigned char *str, unsigned char force_write)
{
	unsigned char *buffer=0;
	struct timespec tv;
	unsigned char time_buff[20]={0};

	tv = CURRENT_TIME;
	sprintf(time_buff,"[%lx]",tv.tv_sec);
	
	if((touch_debug_buffer_flag>0) || (force_write==1))
	{
		//printk("@@save TP log,index:%d\n",touch_debug_buffer_index);		
		if((touch_debug_buffer_index+strlen(str)+4+strlen(time_buff)) > debug_buffer_size)
		{
			touch_debug_buffer_index = 0;
			DebugPageIndex++;
			if(DebugPageIndex>9)
				DebugPageIndex=0;
		}
		buffer=touch_debug_buffer;
		buffer+=touch_debug_buffer_index;
		sprintf(buffer,"%s[%d]%s",time_buff,DebugPageIndex,str);
		//printk("@@%s[%d]%s",time_buff,DebugPageIndex,str);
		touch_debug_buffer_index = (touch_debug_buffer_index+strlen(str)+4+strlen(time_buff));
	}
}

static ssize_t Touch_debug_log_store(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct file *fp;
        mm_segment_t old_fs;

        printk("@@Touch_debug_log_store start\n");		
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	 fp = filp_open("/data/touch_log.txt", O_RDWR | O_APPEND | O_CREAT, 0666);
	if(IS_ERR(fp))
	{
	    printk("[Touch]Open file error...\n");
	}
	else
	{
	fp->f_op->write(fp,touch_debug_buffer , debug_buffer_size , &fp->f_pos);
	}
	filp_close(fp,NULL);
	set_fs(old_fs);
	memset(touch_debug_buffer,0,debug_buffer_size);
	touch_debug_buffer_index = 0;
/*
	old_fs1 = get_fs();
	set_fs(KERNEL_DS);
	fp1 = filp_open("/data/touch_raw.txt", O_RDWR |  O_CREAT, 0666);
	if(IS_ERR(fp1))
	    printk("[Touch]Open file error...\n");
	fp1->f_op->write(fp1,touch_debug_RAW_buffer , 1024 , &fp1->f_pos);
	filp_close(fp1,NULL);
	set_fs(old_fs1);			
*/	
	
	printk("@@Touch_debug_log_store end\n");		
	return 0;
}
static DEVICE_ATTR(tssc_debug, 0644, Touch_debug_log_store, Touch_debug_set_flag);

static ssize_t Touch_debug_get_gpio_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 len=0;
	//printk("@@Touch_debug_set_flag start\n");		
	len = sprintf(buf,"TOUCH_GPIO94 = %d\n",gpio_get_value(TOUCH_GPIO) );
	//printk("@@Touch_debug_set_flag end,buf[0]:%x,touch_debug_buffer_flag:%d\n",buf[0],touch_debug_buffer_flag);
	return len;
}

static ssize_t Touch_debug_set_gpio_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 len=0;
	printk("@@Touch_debug_set_gpio_value start\n");	
	if(( buf[0]-0x30)>0)
		gpio_set_value(94, 1);
	else
		gpio_set_value(94, 0);

	//len = sprintf(buf,"Set gpio:%c,TOUCH_GPIO94 = %d\n",buf[0],gpio_get_value(TOUCH_GPIO) );
	printk("@@Touch_debug_set_gpio_value end,buf[0]:0x%x,TOUCH_GPIO94 = %d\n",buf[0],gpio_get_value(TOUCH_GPIO));
	return 0;
}
static DEVICE_ATTR(tssc_gpio_state, 0644, Touch_debug_get_gpio_value, Touch_debug_set_gpio_value);

static ssize_t Touch_debug_flush_work_queue(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk("@@Touch_debug_flush_work_queue start\n");		
	flush_workqueue(MT_wq);
	MT_wq = create_singlethread_workqueue("MT_wq");
         if (!MT_wq)
	 {
        	printk("!MT_wq created failed\r\n");
	 }
	printk("@@Touch_debug_flush_work_queue end\n");
	return 0;
}

static DEVICE_ATTR(tssc_flush_workq, 0644, Touch_debug_flush_work_queue, Touch_debug_flush_work_queue);

static ssize_t sitronix_ts_read_batt_volt(struct device *dev, struct device_attribute *attr, char *buf)
{
	cci_smem_value_t *smem_cci_smem_value = 0;
	smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ) );
	
	printk("[Aaron] Battery voltage = %d\r\n", smem_cci_smem_value->cci_factory_fast_charge_value);

	return sprintf(buf, "%d\n", smem_cci_smem_value->cci_factory_fast_charge_value);
}
static DEVICE_ATTR(read_batt_volt, 0644, sitronix_ts_read_batt_volt, NULL);

static struct attribute *sitronix_ts_attrs_v0[] = {
	&dev_attr_isp_ctrl.attr,
	&dev_attr_revision.attr,
	&dev_attr_struct_version.attr,
	&dev_attr_data_threshold.attr,
	&dev_attr_point_threshold.attr,
	&dev_attr_peak_threshold.attr,
	&dev_attr_mutual_threshold.attr,
	&dev_attr_range_filter.attr,
	&dev_attr_barX_filter.attr,
	&dev_attr_barY_filter.attr,
	&dev_attr_resolution.attr,
	&dev_attr_channel_num.attr,
	&dev_attr_para.attr,
	&dev_attr_autotune_en.attr,
	&dev_attr_autotune_result.attr,
	&dev_attr_point_report.attr,
	&dev_attr_debug_check.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_update_enable.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_reset_fw.attr,		
	&dev_attr_gpio_reset.attr,
	&dev_attr_proximity_en.attr,
	&dev_attr_sensitivity.attr,
	&dev_attr_tp_version.attr,
	&dev_attr_read_batt_volt.attr,
        &dev_attr_tssc_debug.attr,
        &dev_attr_tssc_gpio_state.attr,
        &dev_attr_tssc_flush_workq.attr,
	NULL,
};



static struct attribute_group sitronix_ts_attr_group_v0 = {
	.name = "sitronix_ts_attrs",
	.attrs = sitronix_ts_attrs_v0,
};

static int sitronix_ts_create_sysfs_entry(struct i2c_client *client)
{
	//struct st1232_ts_data *priv = i2c_get_clientdata(client);
	int err;

	err = sysfs_create_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);
	if (err) {
		dev_dbg(&client->dev, "%s(%u): sysfs_create_group() failed!\n", __FUNCTION__, __LINE__);
	}
	
	err = sysfs_create_bin_file(&client->dev.kobj, &st1232_flash_bin_attr);
	if (err) {
		sysfs_remove_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);
		dev_dbg(&client->dev, "%s(%u): sysfs_create_bin_file() failed!\n", __FUNCTION__, __LINE__);
	}
	err = sysfs_create_bin_file(&client->dev.kobj, &st1232_panel_bin_attr);
	if (err) {
		sysfs_remove_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);
		sysfs_remove_bin_file(&(client->dev.kobj), &st1232_flash_bin_attr);
		dev_dbg(&client->dev, "%s(%u): sysfs_create_bin_file() failed!\n", __FUNCTION__, __LINE__);
	}
	return err;
}

static void sitronix_ts_destroy_sysfs_entry(struct i2c_client *client)
{
	//struct st1232_ts_data *priv = i2c_get_clientdata(client);
	sysfs_remove_bin_file(&client->dev.kobj, &st1232_panel_bin_attr);
	sysfs_remove_bin_file(&client->dev.kobj, &st1232_flash_bin_attr);
	sysfs_remove_group(&(client->dev.kobj), &sitronix_ts_attr_group_v0);

	return;
}

static int sitronix_ts_get_struct_version(struct i2c_client *client, u8 *rev)
{
	struct st1232_ts_data *priv = i2c_get_clientdata(client);
	u8 page_num;
	u32 len;
	
			printk("[Touch]sitronix_ts_get_struct_version \r\n");		

	page_num = ST1232_ROM_PARAM_ADR / ST1232_ISP_PAGE_SIZE;
	
	//Jump to ISP.
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	//Read data from ROM
	if ((len = st1232_isp_read_page(client, isp_page_buf, page_num)) < 0)
	{
				printk("[Touch]error \r\n");		
			return -EIO;
	}
	*rev = (u8)isp_page_buf[1];
			printk("[Touch]sitronix_ts_get_struct_version=%X \r\n",*rev);		

	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);

	return 0;
}

#if 0
static int sitronix_ts_internal_update(struct i2c_client *client)
{
	struct sitronix_ts_priv *priv = i2c_get_clientdata(client);
	int err = 1;
	int count, off;
	char buf[512];
	struct file *file = NULL;

	//Jump to ISP.
	printk("Jump to ISP\n");
	if(!priv->isp_enabled) {
		priv->isp_enabled = true;
		st1232_jump_to_isp(client);
	}
	
	// Update FW
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	//file = filp_open("/media/mmcblk0p1/WinTek/touch_panel_T20_m0_QFN48.bin" , O_RDWR , 0644);
	file = filp_open("/system/flex/touch_panel_T20_m3_QFN40.bin" , O_RDWR , 0644);
	if(IS_ERR(file))
	{	
		printk("open file fail(%u)\n",PTR_ERR(file));
		//ISP Reset.
		priv->isp_enabled = false;
		st1232_isp_reset(client);
		return -1;
	}
	
	off = 0;
	
	while((count = file->f_op->read(file, (char *)buf , 512 , &file->f_pos)) > 0) {
		err = st1232_isp_write_flash(client, buf, off, count);
		if (err < 0) {
			printk("update fw fail\n");
			//ISP Reset.
			priv->isp_enabled = false;
			st1232_isp_reset(client);
			return err;
		}
		printk("Update FW : offset %d , length %d\n",off , count);
		off += count;
	}
	filp_close(file, NULL);
	set_fs(old_fs);
	printk("Update FW Finish\n");
	//ISP Reset.
	priv->isp_enabled = false;
	st1232_isp_reset(client);
	printk("ISP Reset\n");
	return err;

}
#endif
#endif //SITRONIX_ISP_ENABLED

extern int cci_in_CTS;  // by cci - for low memory killer



static void st1232_ts_update_pen_state(struct st1232_ts_data *ts, int x, int y, int pressure)
{
	int i=0;
        unsigned char debug_log[200]={0};
	/*
	if (!kpdev)
	{
		kpdev = msm_7k_handset_get_input_dev();
		printk("[Aaron][%s] kpdev=0x%x\r\n", __func__, kpdev);
	}
	*/
	sprintf(debug_log,"[%s] x,y=(%d %d),last_xy=(%d %d)  p=%d  keyregister=%d ,tpversion=%d project_id=%d down_edge=%d #INT=%d\r\n", __func__, x, y, last_x,last_y,pressure,keyregister,tpversion,project_id,down_edge,interrupt_counter);
	Touch_debug_save_log_to_ram(debug_log,0);

	if(debug_mask)
	    printk("[Touch][%s] x,y=(%d %d),last_xy=(%d %d)  p=%d  keyregister=%d ,tpversion=%d project_id=%d down_edge=%d #INT=%d\r\n", __func__, x, y, last_x,last_y,pressure,keyregister,tpversion,project_id,down_edge,interrupt_counter);
	//printk("[Touch]kpdev=%d ,KEY_HOME=%d KEY_SEARCH=%d KEY_BACK=%d KEY_MENU=%d ,pressure=%d\r\n",kpdev,KEY_HOME,KEY_SEARCH,KEY_BACK,KEY_MENU,pressure);
	if(y<=down_edge&&y>=0)
	{	
        	for (i = 0; i < st1232_ts->prev_touches; i++) {
        		input_report_abs(st1232_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        		input_report_abs(st1232_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
        		input_mt_sync(st1232_ts->input_dev);
			printk("[Touch]st1232_ts_update_pen_state,touch release:%d\r\n",st1232_ts->prev_touches);	
        	}
        	st1232_ts->prev_touches = 0;
        	input_sync(st1232_ts->input_dev);
	}
	
 		if(y>down_edge)
 	 	{
			if(x>0&&x<=80)
			{	
				input_report_key(st1232_ts->input_dev, KEY_HOME, pressure);
				//if(interrupt_counter==1||pressure==0)
					//logger_write(0,2,"TP","KEY_HOME, Press=%d #INT=%d\n",pressure,interrupt_counter);				
				if(debug_mask)
				   printk("[Touch]KEY_HOME \r\n");
			}
			else if(x>80&&x<=150)
			{
				input_report_key(st1232_ts->input_dev, KEY_SEARCH, pressure);
				//if(interrupt_counter==1||pressure==0)				
					//logger_write(0,2,"TP","KEY_SEARCH, Press=%d #INT=%d\n",pressure,interrupt_counter);				
				if(debug_mask)				
				   printk("[Touch]KEY_SEARCH \r\n");
			}
			else if(x>150&&x<=260)
			{	
				input_report_key(st1232_ts->input_dev, KEY_BACK, pressure);						
				//if(interrupt_counter==1||pressure==0)
					//logger_write(0,2,"TP","KEY_BACK, Press=%d #INT=%d\n",pressure,interrupt_counter);				
				if(debug_mask)
				   printk("[Touch]KEY_BACK \r\n");
			}
			else if(x>260)
			{
				input_report_key(st1232_ts->input_dev, KEY_MENU, pressure);				
				//if(interrupt_counter==1||pressure==0)				
					//logger_write(0,2,"TP","KEY_MENU, Press=%d #INT=%d\n",pressure,interrupt_counter);				
				if(debug_mask)
				   printk("[Touch]KEY_MENU \r\n");
			}
			last_key_x=x;
			last_key_y=y;
 	 	}
        	input_sync(st1232_ts->input_dev);		
	
	return;	
}
static void st1232_ts_timer(unsigned long arg)
{
	char debug_log[200]={0};	
	
     	 if(DOG_wq)
	{
		printk("st1232_ts_timer trigger\r\n");
		sprintf(debug_log,"st1232_ts_timer trigger\r\n");
		Touch_debug_save_log_to_ram(debug_log,1);
		queue_work(DOG_wq, &(st1232_ts->ts_dog_work));	
	}
	 else
	 {
	 	sprintf(debug_log,"!!error DOG_wq = NULL!!\r\n");
		Touch_debug_save_log_to_ram(debug_log,1);
	 	printk("!!error DOG_wq = NULL!!\r\n");
	 }
	 mod_timer(&st1232_ts->timer,jiffies + msecs_to_jiffies(TSSC_DOG_TIMER));
/*
     if(release_press_flag==0)
     {
	printk("[Touch][%s]test bak_p1,p2=(%d %d)  last_x,y=(%d %d)  last_x2,y2=(%d %d) press_point=%d  p=(%d %d)  #INT=%d\r\n", __func__,bak_p1,bak_p2,last_x,last_y,last_x2,last_y2,press_point,press1,press2,interrupt_counter);     
        release_press_flag=1;
     }
*/	
}
static void proximity_timer(unsigned long arg)
{
	//printk("[Aaron][%s] receive_phone:%d, phone_ap:%d\r\n", __func__, receive_phone, phone_ap);
	
    //input_report_abs(fpdev, ABS_DISTANCE, (receive_phone==1)? 0:1);
    input_report_abs(fpdev, ABS_X, (receive_phone==1)? 0:1);
	input_sync(fpdev);    		    	   
	
	if(enablep)
	{
  		mod_timer(&st1232_ts->proximity_timer,jiffies + msecs_to_jiffies(report_value));   
	}
}

static void release_press(void)
{
	release_press_flag=1;
	//if(debug_mask)
	   printk("[Touch][%s]bak_p1,p2=(%d %d)  last_x,y=(%d %d)  last_x2,y2=(%d %d) press_point=%d  p=(%d %d) keyregister:%d #INT=%d\r\n", __func__,bak_p1,bak_p2,last_x,last_y,last_x2,last_y2,press_point,press1,press2,keyregister,interrupt_counter);
	//logger_write(0,2,"TP","Releaser:p=(%d %d) last_x,y=(%d,%d) last_x2,y2=(%d,%d) #INT=%d \n",bak_p1,bak_p2,last_x,last_y,last_x2,last_y2,interrupt_counter);	
	if(bak_p1==1)	
		st1232_ts_update_pen_state(st1232_ts, last_x, last_y, 0);
	if(bak_p2==1)
		st1232_ts_update_pen_state(st1232_ts, last_x2, last_y2, 0);	
		
	bak_p1=0;
	bak_p2=0;
	touchedge=0;
	keylocation=0;
	keyregister=0;
	interrupt_counter=0;
	index=0;
	press1=0;
	press2=0;
	runone=0;
	press_point=0;
}




static irqreturn_t st1232_ts_interrupt(int irq, void *dev_id)
{
	unsigned char debug_log[100];
	interrupt_counter++;
	touch_irq_state = 1;
        if(debug_mask)
            printk("[Touch]#INT=%d\r\n",interrupt_counter);		
	atomic_dec(&st1232_ts->irq_disable);
	disable_irq_nosync(st1232_ts->irq);
	touch_irq_state = 2;
	++dis_irq;
	sprintf(debug_log,"ts_int,dis irq: %lu\n",dis_irq);
	Touch_debug_save_log_to_ram(debug_log,1);
	
	if(MT_wq)
	{
		touch_irq_state = 3;
		queue_work(MT_wq, &(st1232_ts->ts_event_work));		
		touch_irq_state = 4;
	}
	else
	{
		touch_irq_state = 5;
		printk("!!error MT_wq = NULL!!\r\n");
		touch_irq_state = 6;
	}
	touch_irq_state = 7;
	return IRQ_HANDLED;
}

int st1232_i2c_read(struct i2c_client *client, int reg, u8 *buf, int count)
{
	int rc = 0;
	int ret = 0;

	buf[0] = reg;
	rc = i2c_master_send(client, buf, 1);
	if (rc != 1) {
		dev_err(&client->dev,"%s FAILED: read of register %d\n",__func__, reg);
		ret = -1;
		goto failed;
	}
	//memset(buf,0x00,count);
	rc = i2c_master_recv(client, buf, count);
	if (rc != count) {
		dev_err(&client->dev,"%s FAILED: read %d bytes from reg %d\n", __func__,count, reg);
		ret = -1;
        goto failed;
	}

	return ret;
failed:
    //reset_TP_state();
	return ret;
}

static void report_data(struct st1232_ts_data *ts, u32 x, u32 y, int pressure)
{
//printk("[Touch]report_data x,y=(%d %d)  press=%d\r\n",x,y,pressure);

	//if(interrupt_counter==1)
		//logger_write(0,2,"TP","report_data:x,y=(%d,%d) pressure=%d #INT=%d \n", x,y,pressure,interrupt_counter);	
       if(debug_mask)	
		printk("[Touch]report_data x,y=(%d %d)  press=%d\r\n",x,y,pressure);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, pressure);
//	input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, ts->dd->finger_size);
	input_mt_sync(ts->input_dev);
}

static void st1232_ts_dog_worker(struct work_struct *work)
{
	 char debug_log[200]={0};	
	 u8 buf[8],buf1[1];
	 int rc=0;

	printk("[Touch]Touch  key up/down IRQ priority=%ld %ld %ld\n", msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO)), msm_irq_priority_get(MSM_GPIO_TO_INT(41)),msm_irq_priority_get(MSM_GPIO_TO_INT(42)));
	printk("[Touch]check irq ,en_irq:%d,dis_irq:%d,touch_work_state:%d\n",en_irq,dis_irq,touch_work_state);
	if(msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO))!=0)
	{
		atomic_dec(&st1232_ts->irq_disable);
		disable_irq_nosync(st1232_ts->irq);
		msm_irq_priority_set(MSM_GPIO_TO_INT(TOUCH_GPIO),0);
		enable_irq(st1232_ts->irq);
		atomic_inc(&st1232_ts->irq_disable);
	}

	if( (CanUpdate == 0) &&(st1232_suspend == 0))
	{
	if(touch_read_i2c_fail > 9)
		tp_reset();
#if 0
	 buf1[0]=0x10;
	rc = i2c_master_send(st1232_ts->client, buf1,1);
	if(rc!=1)
	{
		printk("[Touch]DOG i2c send  error\r\n");
		sprintf(debug_log,"DOG i2c send 0x10 error\r\n");
		Touch_debug_save_log_to_ram(debug_log,1);
		return;
	}
	else
	{
		rc = i2c_master_recv(st1232_ts->client, buf, 8);
		if(rc!=8)
		{
			printk("[Touch]DOG  i2c recv error\r\n");
			sprintf(debug_log,"DOG  i2c recv error\r\n");
			Touch_debug_save_log_to_ram(debug_log,1);
			return;
		}
	}
	
	        sprintf(debug_log,"DOG:buf[0~7]=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	        Touch_debug_save_log_to_ram(debug_log,1);
#endif			
		if((en_irq < dis_irq) ||(touch_work_state))
		{
			printk("is workqueue running ?? touch_work_state:%d\n",touch_work_state);
			sprintf(debug_log,"is workqueue running ?? touch_work_state:%d,en_irq:%d,dis_irq:%d\n",touch_work_state,en_irq,dis_irq);
			Touch_debug_save_log_to_ram(debug_log,1);
			recover_workqueue_cnt++;
			printk("re-enable touch irq for touch abnormal issue,en_irq:%d,dis_irq:%d\n",en_irq,dis_irq);
			//enable_irq(st1232_ts->irq);
			//atomic_inc(&st1232_ts->irq_disable);
			//++en_irq;			
			touch_work_state =0;
		}
	}
	
        printk("[Touch] TOUCH_GPIO94 = %d,destroy_workqueue_cnt:%d\n",gpio_get_value(TOUCH_GPIO),recover_workqueue_cnt );
        sprintf(debug_log,"TOUCH_GPIO94 = %d\n",gpio_get_value(TOUCH_GPIO) );
	Touch_debug_save_log_to_ram(debug_log,1);
	cci_show_interrupts(1);
	
}

static void st1232_ts_irq_worker(struct work_struct *work)
{
	char test[80],test1[20];
	struct file *file,*file1;
	mm_segment_t old_fs,old_fs1;
        u8 buf[8],buf1[1];
        u32 x_st1,y_st1,x_st2,y_st2;
        int ret,rc,k;

	int result;
	mm_segment_t old_fs2;
	struct file *file2;
	char sensitive[2];
	int err;
        char debug_log[200];
        unsigned char index4log=0;
		
	touch_work_state = 1;   //work running
	mod_timer(&st1232_ts->timer,jiffies + msecs_to_jiffies(TSSC_DOG_TIMER));
#if 0
 	if (!set_highest_priority)
	{
		//Set to a high priority
		//set_user_nice(current, -10);
		struct sched_param s;
		struct cred *new = prepare_creds();
		cap_raise(new->cap_effective, CAP_SYS_NICE);
		commit_creds(new);
		s.sched_priority = 70;
		if ((sched_setscheduler(current, SCHED_FIFO, &s)) < 0)
		      printk("[CCI] sched_setscheduler failed for Touch Thread !!\n");
		set_highest_priority = 1;
	}
#endif	
	if(CanUpdate==1)
	{	
		printk("[Touch]st1232_ts_irq_worker:  CanUpdate=%d\r\n",CanUpdate);		 
		goto end;
	}	
	
	if(run_one==1)
	{
		//write firmware version
		memset(test1,0,sizeof(test1));
		if ((err = sitronix_ts_get_fw_version(st1232_ts->client, &g_ver))) {
			printk("[Touch]Unable to get FW version...\r\n");
		} else {
			printk("[Touch]FW version=%X\r\n",g_ver);
		}
		
		if ((err = sitronix_ts_get_fw_revision(st1232_ts->client, &g_rev)))
		{
			printk("[Touch]Unable to get FW revision \r\n");		 
		
		}
		old_fs1 = get_fs();
		set_fs(KERNEL_DS);
		file1 = filp_open("/data/touch_ver", O_RDWR | O_CREAT, 0666);
		sprintf(test1,"rev:%d, version:%x",g_rev,g_ver);
	   	if(IS_ERR(file1))
	   	{
		    printk("[Touch]Open file error...\n");
	   	}
		else
		{
		 file1->f_op->write(file1,test1 , 20 , &file1->f_pos);
		}
	  	 filp_close(file1,NULL);
		 set_fs(old_fs1);	
		 run_one=0;
		printk("[Touch]run_one=%d rev=%d tpversion=%d\r\n",run_one,g_rev,g_ver);	


		//read sensitive
		old_fs2 = get_fs();
		set_fs(KERNEL_DS);		
		file2 = filp_open("/data/touch_sensitive", O_RDONLY|O_CREAT, 0777);
		if(IS_ERR(file2)||file2==NULL)
		{
			printk("[Touch]st1232_ts_irq_worker: sensitive=null");				
		 	sensitive[0]=0;		
		}
		else
		{
			file2->f_pos=0x00;
		        result=file2->f_op->read(file2,sensitive,sizeof(sensitive),&file2->f_pos);
			sensitive[1]='\0';
			if(result>=0)
				printk("[Touch]st1232_ts_irq_worker: sensitive=%s",sensitive);
			else
				printk("[Touch]st1232_ts_irq_worker: result=%d",result);
			printk("[Touch]st1232_ts_irq_worker: sensitive=%c,%c,%d,%d",sensitive[0],sensitive[1],sensitive[0],sensitive[1]);
			
		}	
		filp_close(file2,NULL);
		set_fs(old_fs2);			

		if(sensitive[0]==48)
		{
			printk("[Touch]st1232_ts_irq_worker: sensitive=low");
	 	        sitronix_ts_set_sensitivity(st1232_ts->client, 0);
		}		
		else if(sensitive[0]==49)
		{
			printk("[Touch]st1232_ts_irq_worker: sensitive=middle");				
	 	        sitronix_ts_set_sensitivity(st1232_ts->client, 1);
		}		
		else if(sensitive[0]==50)
		{
			printk("[Touch]st1232_ts_irq_worker: sensitive=high");				
	 	        sitronix_ts_set_sensitivity(st1232_ts->client, 2);
		}		
		else
		{
			printk("[Touch]st1232_ts_irq_worker: sensitive=default");						
			sitronix_ts_set_sensitivity(st1232_ts->client, 1);			
		}
		
	 
	} 
	memset(buf,0,sizeof(buf));
	memset(buf1,0,sizeof(buf1));
	if(touch_read_i2c_fail == 0xff)
		goto end;
	if(touch_read_i2c_fail>9)
		tp_reset();
	press1=press2=0;
	buf1[0]=0x10;
	rc = i2c_master_send(st1232_ts->client, buf1,1);
	if(rc!=1)
	{
		touch_read_i2c_fail++;
		printk("[Touch]st1232  i2c_master_send  error4\r\n");
		sprintf(debug_log,"st1232  i2c_master_send 0x10  error4_1\r\n");
		Touch_debug_save_log_to_ram(debug_log,1);
		rc = i2c_master_send(st1232_ts->client, buf1, 1);		
		if(rc!=1)
		{
			touch_read_i2c_fail++;
			printk("[Touch]st1232  i2c_master_send  error4_1\r\n");
			sprintf(debug_log,"st1232  i2c_master_send 0x10 retry error4_1\r\n");
			Touch_debug_save_log_to_ram(debug_log,1);
			goto end;
		}	
	}	
	
	ret = i2c_master_recv(st1232_ts->client, buf, 8);
	if(ret!=8)
	{
		touch_read_i2c_fail++;
		printk("[Touch]st1232  i2c_master_recv  error5\r\n");
		sprintf(debug_log,"st1232  i2c_master_recv coord error5\r\n");
		Touch_debug_save_log_to_ram(debug_log,1);
		ret = i2c_master_recv(st1232_ts->client, buf, 8);
		if(ret!=8)
		{
			touch_read_i2c_fail++;
			printk("[Touch]st1232  i2c_master_recv  error5_1\r\n");
			sprintf(debug_log,"st1232  i2c_master_recv coord retry error5\r\n");
			Touch_debug_save_log_to_ram(debug_log,1);
			goto end;
		}		
	}	
	
	sprintf(debug_log,"buf[0~7]=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	Touch_debug_save_log_to_ram(debug_log,0);      
      
              if( (buf[2]&0x80) && (buf[5]&0x80))
	      {
                    press1= press2=1;
		    runone=1;		
  		    release_press_flag=0;	
              }
              else if( (buf[2]&0x80))
              {     
              	    press1 = 1;
	            release_press_flag=0;
              }		 
              else if( (buf[5]&0x80))
              {
                     press2 = 1;
		     release_press_flag=0;
              }	
	      x_st1 = ((buf[2]&0x70)<<4)|buf[3];
	      y_st1 = (((buf[2]&0x07)<<8)|buf[4]);
	      x_st2 = ((buf[5]&0x70)<<4)|buf[6];
	      y_st2 = (((buf[5]&0x07)<<8)|buf[7]);
             // printk("[Touch]buf[0]>>3=%d  receive_phone=%d  phone_ap=%d \r\n",buf[0]>>3,receive_phone,phone_ap);	

	      if((buf[0]>>3)==0x0F)
	      {
		    receive_phone=1;          	    
		    //printk("[Aaron][%s] P:%d", __func__, receive_phone);	
	      }
	      else if((buf[0]>>3)==0x10)
	      {
	            receive_phone=0;			
	            //printk("[Aaron][%s] P:%d", __func__, receive_phone);
	      }	 
	      if(debug_mask)
 	      {
 	            printk("[Touch]receive_phone=%d  phone_ap=%d dis_irq=%ld en_irq=%ld BKL_EN=%d\r\n",receive_phone,phone_ap,dis_irq,en_irq,BKL_EN);	
	       	    printk("[Touch]buf[0~7]=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
		    printk("[Touch]p1,p2=(%d %d) before_x,y=(%d %d) before_x2,y2=(%d %d)  runone=%d   #INT=%d\r\n",press1,press2,x_st1,y_st1,x_st2,y_st2,runone,interrupt_counter);			
	      }
	
              if(receive_phone==1&&phone_ap==1)
		  goto end;

	if (BKL_EN==1)			  
	//if (BKL_EN==1||(receive_phone==0&&phone_ap==1))
	{  	
	      if(press1==0)
		  x_st1=y_st1=0;
	      if(press2==0)
		  x_st2=y_st2=0;
	      //if( press1 != 0)//fix report ghost point (x,y)=(0,0)
	      {
	      if(index<3)
	      {
		 x[index]=x_st1;
		 y[index]=y_st1;
   		 ++index;
	      }		  
	      else
	      {
		  x[0]=x[1];
		  y[0]=y[1];		
		  x[1]=x[2];
		  y[1]=y[2];				  
		  x[2]=x_st1;
		  y[2]=y_st1;
	      }		
	      }
	       x_st1= x[0];
	       y_st1= y[0];

            if(index==0)
                index4log = index;
            else
                index4log = (index-1);

	      if(debug_mask ||(online_debug>50))
	      {
	      	    online_debug = 0;
		    if(printk_ratelimit())		
		    {
		    	printk("buf[0~7]=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
		    	printk("[Touch] x,y=(%d %d) x2,y2=(%d %d) x,y[%d]=(%d %d) ,Key:0x%x,#INT=%d\r\n",x_st1,y_st1,x_st2,y_st2,index4log,x[index4log], y[index4log],buf[1],interrupt_counter);
	            }	
	      }	
              online_debug++;
	     sprintf(debug_log,"buf[0~7]=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
	     Touch_debug_save_log_to_ram(debug_log,1);
	     sprintf(debug_log," x,y=(%d %d) x2,y2=(%d %d) x,y[%d]=(%d %d),Key:0x%x, #INT=%d,down_edge:%d\r\n",x_st1,y_st1,x_st2,y_st2,index4log,x[index4log], y[index4log],buf[1],interrupt_counter,down_edge);
	     Touch_debug_save_log_to_ram(debug_log,1);

	      if(press1==0&&press2==0)
	      {
		     release_press();
		     goto end;
	      }	 
              else if(press1==0&&press2==1&&runone==1)
	      {
		    if(debug_mask)
  		        printk("[Touch]Release press1: (p1,p2)=(%d %d), last_x,y=(%d %d) #INT=%d\n",press1,press2,last_x,last_y,interrupt_counter);	
			sprintf(debug_log,"Release press1: (p1,p2)=(%d %d), last_x,y=(%d %d) #INT=%d\n",press1,press2,last_x,last_y,interrupt_counter);
		        Touch_debug_save_log_to_ram(debug_log,0);
		    st1232_ts_update_pen_state(st1232_ts, last_x, last_y, 0);
                    x_st1=y_st1=0;
		    index=0;
		    if(press_point==2)
		        st1232_ts_update_pen_state(st1232_ts, last_x2, last_y2, 0);
		    runone=0;
	      }	  

	     	if(x_st1>325||y_st1>600)
	      	{
        	    memset(test,0,sizeof(test));
		    for(k=0;k<8;k++)
			 printk("[Touch]buf[%d]=0x%02X\r\n",k,buf[k]);
		    printk("[Touch]press1=%d  press2=%d x_st1=%d  y_st1=%d  x_st2=%d y_st2=%d #INT=%d\r\n",press1,press2,x_st1,y_st1,x_st2,y_st2,interrupt_counter);
     		    old_fs = get_fs();
		    set_fs(KERNEL_DS);
		    file = filp_open("/data/touch_data.txt", O_RDWR | O_APPEND | O_CREAT, 0666);
		    sprintf(test,"buf=(%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X) x=%d y=%d x2=%d y2=%d INT=%d\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],x_st1,y_st1,x_st2,y_st2,interrupt_counter);
   		    if(IS_ERR(file))
   		    {
		        printk("[Touch]Open file error...\n");
   		    }
		    else
		    {
		    file->f_op->write(file,test , 80 , &file->f_pos);
		    }
  		    filp_close(file,NULL);
		    set_fs(old_fs);
                    tp_reset();
       	      	}
              
             if(press1 && press2) 
	     {
		press_point=2;
		bak_p1=bak_p2=1;
		if(debug_mask)
	  	  printk("[Touch]x,y=(%d %d)  x2,y2=(%d %d)   prev_touches=%d keylocation=%d \r\n",x_st1,y_st1,x_st2,y_st2,st1232_ts->prev_touches,keylocation);				 				 
				
                if(y_st1<=down_edge) 
		{   //finger 1 is in touch area
                     report_data(st1232_ts, x_st1, y_st1, 1 );                     
		     sprintf(debug_log,"1.report_data,x1:%d,y1:%d\r\n",x_st1,y_st1);
		     Touch_debug_save_log_to_ram(debug_log,0);
                     if(y_st2<=down_edge)
		     {
                   	 report_data(st1232_ts, x_st2, y_st2, 1 );
                         st1232_ts->prev_touches = 2;
			 sprintf(debug_log,"2.report_data,x2:%d,y2:%d\r\n",x_st2,y_st2);
		     	 Touch_debug_save_log_to_ram(debug_log,0);
                      }
                     else
                         st1232_ts->prev_touches = 1;
                     input_sync(st1232_ts->input_dev);                     
                 }
                 else { // buttom 4...
                     st1232_ts_update_pen_state(st1232_ts,x_st1, y_st1, 1 );                    
                }
                last_x = x_st1;
                last_y = y_st1; 
                last_x2 = x_st2;
                last_y2 = y_st2;          				                 
            }
            else if(press1)
	    {		
	        press_point=1;
 		bak_p1=1;
		bak_p2=0;

                if(y_st1<=down_edge)
		{ 
		    //finger 1 is in touch area
                    report_data(st1232_ts, x_st1, y_st1, 1 );
		    sprintf(debug_log,"3.report_data,x1:%d,y1:%d\r\n",x_st1,y_st1);
		    Touch_debug_save_log_to_ram(debug_log,0);	
  		    if(debug_mask)
			    printk("[Touch]x,y=(%d %d) prev_touches=%d keylocation=%d touchedge=%d \r\n",x_st1,y_st1,st1232_ts->prev_touches,keylocation,touchedge);				 				 
						
                     if(st1232_ts->prev_touches == 2){					 	
                        input_report_abs(st1232_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
                	input_report_abs(st1232_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
                	input_mt_sync(st1232_ts->input_dev);
                     }
                     st1232_ts->prev_touches = 1;
                    
                     input_sync(st1232_ts->input_dev);
		     if(keylocation==1)
		     {
			st1232_ts_update_pen_state(st1232_ts, last_key_x, last_key_y, 0);			
		      }	
		     keylocation=0;	 			 
		     touchedge=1;
                 }
               	 else
                 {
                	if(debug_mask)
			    printk("[Touch]touchedge=%d y_st1=%d keylocation=%d\r\n",touchedge,y_st1,keylocation);				 				 
                  	if(y_st1>down_edge)
                  	{
     				if(touchedge==1)
				{
	               		    	report_data(st1232_ts, x_st1, y_st1, 0 );		
                                        sprintf(debug_log,"4.report_data,x1:%d,y1:%d\r\n",x_st1,y_st1);
                                        Touch_debug_save_log_to_ram(debug_log,0);
					st1232_ts_update_pen_state(st1232_ts, last_x, last_y, 0);			
					printk("[Touch]trigger edge=%d  (%d %d)\r\n",touchedge,x_st1,y_st1);
					touchedge=0;
				}
				else
				{
					keylocation=1;
	 	             	        st1232_ts_update_pen_state(st1232_ts,x_st1, y_st1, 1 );
                 	        }
                       }
                 }
                 last_x = x_st1;
                 last_y = y_st1;               
                 last_x2 = x_st2;
                 last_y2 = y_st2;           
	     }
            else if(press2)
	    {							
		 press_point=1;
	 	 bak_p1=0;
		 bak_p2=1;

  	        if(debug_mask)
			printk("[Touch]x2,y2=(%d %d)  trigger edge=%d\r\n",x_st2,y_st2,touchedge);				 
				
                if(y_st2<=down_edge)
		{ //finger 1 is in touch area
                    	report_data(st1232_ts, x_st2, y_st2, 1 );
			sprintf(debug_log,"4.report_data,x2:%d,y2:%d\r\n",x_st2,y_st2);
                        Touch_debug_save_log_to_ram(debug_log,0);
                     if(st1232_ts->prev_touches == 2)
		     {
                        input_report_abs(st1232_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
                	input_report_abs(st1232_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
                	input_mt_sync(st1232_ts->input_dev);
                     }
                     st1232_ts->prev_touches = 1;
                     input_sync(st1232_ts->input_dev);
                 }
                 else
                     st1232_ts_update_pen_state(st1232_ts,x_st2, y_st2, 1 );                    
                last_x = x_st1;
                last_y = y_st1;     
                last_x2 = x_st2;
                last_y2 = y_st2;           
				 
            }
	    else
	    {
        	 if(tpversion>=1)
              	 {
    			keypress=buf[1];
			if(keypress!=0)	
				keyregister=keypress;
        		if(keypress!=0)
        		{
		   		if(debug_mask)        		
		    			printk("[Touch]keyregister=%d \r\n",buf[1]);
				x_st1=y_st1=0;
				st1232_ts_update_pen_state(st1232_ts,x_st1,y_st1, 1);		
        		}
                  }
	     }		


	}	
end:	

	/* kick pen up timer - to make sure it expires again(!) */
	enable_irq(st1232_ts->irq);
	atomic_inc(&st1232_ts->irq_disable);
	++en_irq;
	sprintf(debug_log,"irq_worker, en irq:%lu\n",en_irq);
	Touch_debug_save_log_to_ram(debug_log,1);
	touch_work_state = 0;
}
static int call_touch_sensitive_write(struct file *file, const char *buffer,
                                 unsigned long count, void *data)
{
    char *buf;
    int BUFIndex;
            printk("[Touch] call_touch_sensitive_write \n");

    if (count < 1)
        return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
    {
        kfree(buf);
        return -EFAULT;
    }

    BUFIndex = (int)simple_strtol(buf, NULL, 10);


    switch (BUFIndex)
    {
        case 1://Low Level
             printk("[Touch] Sensitive_Low \n");
 	sitronix_ts_set_sensitivity(st1232_ts->client, 0);
	break;

        case 2://Mid Level
            printk("[Touch] Sensitive_Mid \n");
   	    sitronix_ts_set_sensitivity(st1232_ts->client, 1);			
            break;

        case 3://High Level
            printk("[Touch] Sensitive_High \n");
 	    sitronix_ts_set_sensitivity(st1232_ts->client, 2);
            break;

    }
    kfree(buf);
    return count;

}

static ssize_t fakepenable_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	//printk("[Aaron][%s] enablep:%d\r\n", __func__, enablep);
	return sprintf(buf, "%hu\n", enablep);
}

static ssize_t fakepenable_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char * buf, size_t n)
{

	unsigned short value;
	
	printk("[Aaron][%s] kobj:0x%lx\r\n", __func__, (unsigned long)kobj);
	
	if (sscanf(buf, "%hu", &value) != 1 || (value != 0 && value != 1)) 
	{
		printk("[Aaron][%s] Invalid value: %d\r\n", __func__, value);
		return -EINVAL;
	}
	
	enablep = value;
	
	st1232_proximity_control(enablep);
	
	printk("[Aaron][%s] receive_phone:%d, enablep:%d\r\n", __func__, receive_phone, enablep);
	if(enablep)
	{
		mod_timer(&st1232_ts->proximity_timer,jiffies + msecs_to_jiffies(report_value));       
	}
	
	input_report_abs(fpdev, ABS_X, (receive_phone==1)? 0:1);
    input_sync(fpdev);                     
	
	printk("[Aaron][%s] receive_phone:%d, enablep:%d\r\n", __func__, receive_phone, enablep);
	
	return n;
}


static ssize_t fakepdata_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	printk("[Aaron][%s] distance:%d\r\n", __func__, distance);
	return sprintf(buf, "%hu\n", distance);
}

static ssize_t fakepdata_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char * buf, size_t n)
{

	unsigned short value;
	
	printk("[Aaron][%s] kobj:0x%lx\r\n", __func__, (unsigned long)kobj);
	
	if (sscanf(buf, "%hu", &value) != 1 || (value != 0 && value != 1)) 
	{
		printk("[Aaron][%s] Invalid value: %d\r\n", __func__, value);
		return -EINVAL;
	}
	
	distance = value;
	input_report_abs(fpdev, ABS_X, distance);
        input_sync(fpdev);                     
    
	printk("[Aaron][%s] distance:%d\r\n", __func__, distance);
	
	return n;
}

static ssize_t fakepwake_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char * buf, size_t n)
{
	unsigned short value;
	
	printk("[Aaron][%s] kobj:0x%lx\r\n", __func__, (unsigned long)kobj);
	
	if (sscanf(buf, "%hu", &value) != 1 || (value != 0 && value != 1)) 
	{
		printk("[Aaron][%s] Invalid value: %d\r\n", __func__, value);
		return -EINVAL;
	}

	input_report_abs(fpdev, ABS_MISC, 0);
        input_sync(fpdev);                     
    
	printk("[Aaron][%s] distance:%d\r\n", __func__, distance);
	return n;
}

static ssize_t fakeptimer_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char * buf, size_t n)
{

	sscanf(buf, "%hu", &report_value);

    printk("[Aaron][%s] report_value:%d, receive_phone:%d, phone_ap:%d, distance:%d\r\n", __func__, report_value, receive_phone, phone_ap, distance);

	if(enablep)
	{
	  	mod_timer(&st1232_ts->proximity_timer,jiffies + msecs_to_jiffies(report_value));       
	}
     
	return n;
}

static struct kobj_attribute fakepdata_attr =
	__ATTR(data, 0664, fakepdata_show, fakepdata_store);

static struct kobj_attribute fakepenable_attr =
	__ATTR(enable, 0664, fakepenable_show, fakepenable_store);

static struct kobj_attribute fakepwake_attr =
	__ATTR(wake, 0664, NULL, fakepwake_store);

static struct kobj_attribute fakeptimer_attr =
	__ATTR(delay, 0664, NULL, fakeptimer_store);

static int st1232_proximity_enable(struct file *file, const char *buffer,
                                 unsigned long count, void *data)
{
    char *buf;
    int BUFIndex;
    u8 buf1[1],buf2[2],buf3[1];
    int rc;
    
    printk("[Aaron][%s]\r\n", __func__);
    
    if (count < 1)
        return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
    {
        kfree(buf);
        return -EFAULT;
    }

    BUFIndex = (int)simple_strtol(buf, NULL, 10);
    printk("[Aaron][%s] BUFIndex:%d\r\n", __func__, BUFIndex);

    buf1[0]=0x2;
    rc=i2c_master_send(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
 	printk("[Touch]st1232_proximity_enable  i2c_master_send  error1\r\n");		
        tp_reset();
    }
    rc= i2c_master_recv(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
	printk("[Touch]st1232_proximity_enable  i2c_master_recv  error2\r\n");
	tp_reset();
    }			
    printk("[Touch]st1232_proximity_enable: 0x2=%d (before)\r\n", buf1[0]);


    switch (BUFIndex)
    {
        case 0:
             printk("[Touch] st1232_proximity_enable: Disable phone application \n");			 
	     phone_ap=0;
	     buf2[0]=0x2;
	     buf2[1]=0x8;
             rc = i2c_master_send(st1232_ts->client, buf2,2);
	     if(rc!=2)
	     {
		printk("[Touch]st1232_proximity_enable: i2c_master_send 0x2,0x8 error_1\r\n");
		tp_reset();					
	     }
	     wake_unlock(&tpwakelock);
	break;	
        case 1:
             printk("[Touch] st1232_proximity_enable:  Enable phone application  \n");
	     phone_ap=1;
	     buf2[0]=0x2;
	     buf2[1]=0xC;
             rc = i2c_master_send(st1232_ts->client, buf2,2);
	     if(rc!=2)
	     {
		printk("[Touch]st1232_proximity_enable: i2c_master_send 0x2,0xC error_2\r\n");
		tp_reset();					
	     }
	     wake_lock(&tpwakelock);	 
	break;
    }
    //read 0x2 data
    buf1[0]=0x2;
    rc=i2c_master_send(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
 	printk("[Touch]st1232_proximity_enable  i2c_master_send  error1\r\n");		
        tp_reset();
    }
    rc= i2c_master_recv(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
	printk("[Touch]st1232_proximity_enable  i2c_master_recv  error2\r\n");
	tp_reset();
    }			
    printk("[Touch]st1232_proximity_enable: 0x2=%d (after)\r\n", buf1[0]);


    //set original address: 0x10	
    buf3[0]=0x10;
    rc = i2c_master_send(st1232_ts->client, buf3,1);
    if(rc!=1)
    {
	printk("[Touch]set original address error3\r\n");
  	tp_reset();
    }	

	
    kfree(buf);
    return count;

}

static int st1232_proximity_control(int en_p_sensor)
{
    u8 buf1[1],buf2[2],buf3[1];
    int rc=0;
    

    buf1[0]=0x2;
    rc=i2c_master_send(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
 		printk("[Touch]call_receive_phone  i2c_master_send  error1\r\n");		
        tp_reset();
    }
    rc= i2c_master_recv(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
		printk("[Touch]call_receive_phone  i2c_master_recv  error2\r\n");
		tp_reset();
    }			
    printk("[Touch]call_receive_phone: 0x2=%d (before)\r\n", buf1[0]);


    switch (en_p_sensor)
    {
        case 0:
        {
            printk("[Touch] call_receive_phone: Disable phone application \n");
            
	     	phone_ap=0;
	     	buf2[0]=0x2;
	     	buf2[1]=0x8;
            rc = i2c_master_send(st1232_ts->client, buf2,2);
	     	if(rc!=2)
	     	{
				printk("[Touch]call_receive_phone: i2c_master_send 0x2,0x8 error_1\r\n");
				tp_reset();					
	     	}
		printk("[Touch] call_receive_phone2:  wake_lock disable  \n");
		wake_unlock(&tpwakelock);		
	}
	break;	
        case 1:
        {
            printk("[Touch] call_receive_phone:  Enable phone application  \n");
            
	     	phone_ap=1;
	     	buf2[0]=0x2;
	     	buf2[1]=0xC;
            rc = i2c_master_send(st1232_ts->client, buf2,2);
	     	if(rc!=2)
	     	{
				printk("[Touch]call_receive_phone: i2c_master_send 0x2,0xC error_2\r\n");
				tp_reset();					
	     	}
			printk("[Touch] call_receive_phone2:  wake_lock enable  \n");
			wake_lock(&tpwakelock);			
		}
		break;
    }
    
    //read 0x2 data
    buf1[0]=0x2;
    rc=i2c_master_send(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
 		printk("[Touch]call_receive_phone  i2c_master_send  error1\r\n");		
        tp_reset();
    }
    rc= i2c_master_recv(st1232_ts->client, buf1,1);
    if(rc!=1)
    {
		printk("[Touch]call_receive_phone  i2c_master_recv  error2\r\n");
		tp_reset();
    }			
    printk("[Touch]call_receive_phone: 0x2=%d (after)\r\n", buf1[0]);

    //set original address: 0x10	
    buf3[0]=0x10;
    rc = i2c_master_send(st1232_ts->client, buf3,1);
    if(rc!=1)
    {
		printk("[Touch]set original address error3\r\n");
  		tp_reset();
    }	

    return rc;

}

static int __devinit st1232_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{	
	struct input_dev *input_dev;
	int ret = 0,rc;
	//u8 struct_ver;
	int err = -ENOMEM;
//for touch sensitivity
	struct proc_dir_entry *d_entry,*d_entry1;
        u8 buf[8],buf1[16];
	unsigned char debug_log[100]={0};	
	cci_smem_value_t *smem_cci_smem_value = 0;
	smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ) );
	
	//Set Sensitive path " /proc/OJ_Sensitive "
	d_entry = create_proc_entry("touch_sensitive", S_IRWXU | S_IRWXO, NULL);
	if (d_entry)
	{
		d_entry->read_proc = NULL;
		d_entry->write_proc = call_touch_sensitive_write;
		d_entry->data = NULL;
	}
	else
	{
		printk("[Touch] Fail call_touch_sensitive_write  \n");
		remove_proc_entry("touch_sensitive", 0);
		return  -ENOMEM;
	}
	d_entry1 = create_proc_entry("receive_phone", S_IRWXU | S_IRWXO, NULL);
	if (d_entry1)
	{
		d_entry1->read_proc = NULL;
		d_entry1->write_proc = st1232_proximity_enable;
		d_entry1->data = NULL;
	}
	else
	{
		printk("[Touch] Fail st1232_proximity_enable  \n");
		remove_proc_entry("receive_phone", 0);
		return  -ENOMEM;
	}
	printk("[Touch][%s]\r\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[Touch][%s] st1232_ts_probe: need I2C_FUNC_I2C\r\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ret = gpio_request(93,"touch_int");
	if(ret < 0)
		printk("fail to request gpio93 for touch_int! error = %d\n",ret);
	
        printk("[Touch]After %s: TOUCH_GPIO93 = %d\n",__func__,gpio_get_value(93) );
	gpio_set_value(93, 1);
	ret = gpio_request(TOUCH_GPIO,"touch_int");
	if(ret < 0)
		printk("fail to request gpio for touch_int! error = %d\n",ret);
	 printk("[Touch]Before TOUCH_GPIO94 = %d\n",gpio_get_value(TOUCH_GPIO) );
   	// gpio_direction_input(TOUCH_GPIO);
	 // gpio_set_value(TOUCH_GPIO, 1);
 	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));

	 printk("[Touch]After TOUCH_GPIO94 = %d\n",gpio_get_value(TOUCH_GPIO) );
	 
	st1232_ts = kzalloc(sizeof(struct st1232_ts_data), GFP_KERNEL);
	st1232_ts->input_dev = input_allocate_device();

	fpdev = input_allocate_device();

	if (!st1232_ts->input_dev || !st1232_ts)
	{
		ret = -ENOMEM;
		goto err_input;
	}
	
	INIT_WORK(&(st1232_ts->ts_event_work), st1232_ts_irq_worker);
	INIT_WORK(&(st1232_ts->ts_dog_work), st1232_ts_dog_worker);
           //init 
        st1232_ts->prev_touches = 0;

	st1232_ts->client = client;
	st1232_ts->irq = MSM_GPIO_TO_INT(TOUCH_GPIO);
	st1232_ts->isp_enabled = false;
	st1232_ts->Can_update = false;
	st1232_ts->sensitivity_level = MEDIUM_SENSITIVE;
	atomic_set(&st1232_ts->irq_disable, 1);
	i2c_set_clientdata(client, st1232_ts);
	input_dev = st1232_ts->input_dev;
	input_dev->name = "st1232-ts";
	input_dev->phys = "msm_touch/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &st1232_ts->client->dev;
	
	fpdev->name = "proximity";
	fpdev->phys = "msm_touch/input0";
	fpdev->id.bustype = BUS_VIRTUAL;
	fpdev->id.vendor = 0x0001;
	fpdev->id.product = 0x0002;
	fpdev->id.version = 0x0100;
	fpdev->dev.parent = &st1232_ts->client->dev;

	set_bit(EV_ABS, fpdev->evbit);
	input_set_abs_params(fpdev, ABS_X, 0, 1, 0, 0);
	
	if(input_register_device(fpdev))
	{
		printk("[Aaron][%s] register Proximity FAILED\r\n", __func__);
	}
	else
	{
		printk("[Aaron][%s] register Proximity OK\r\n", __func__);
	}
	ret = sysfs_create_file(&fpdev->dev.kobj,&fakepdata_attr.attr);
	ret = sysfs_create_file(&fpdev->dev.kobj,&fakepenable_attr.attr);
	ret = sysfs_create_file(&fpdev->dev.kobj,&fakeptimer_attr.attr);

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(ABS_MISC, input_dev->absbit);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_capability(input_dev,EV_KEY, KEY_HOME);
	input_set_capability(input_dev,EV_KEY, KEY_SEARCH);
	input_set_capability(input_dev,EV_KEY, KEY_BACK);
	input_set_capability(input_dev,EV_KEY, KEY_MENU);	

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,0, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,0, 320, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 320, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 488, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	//printk("[Touch][%s] st1232_ts->input_dev = 0x%x\r\n", __func__, st1232_ts->input_dev);

	MT_wq = create_singlethread_workqueue("MT_wq");
         if (!MT_wq)
	 {
        	printk("!MT_wq created failed\r\n");
	        goto err_input;
	 }

	DOG_wq = create_singlethread_workqueue("DOG_wq");
	if (!DOG_wq)
	 {
        	printk("!DOG_wq created failed\r\n");
	        goto err_input;
	 }	

	down_edge=488;		
	wake_lock_init(&tpwakelock, WAKE_LOCK_SUSPEND, "st1232-ts");
	wake_lock_init(&tpwakelock_idle, WAKE_LOCK_IDLE, "st1232-ts");

	ret = request_irq(st1232_ts->irq, st1232_ts_interrupt, IRQF_TRIGGER_LOW, "touchscreen", NULL);
	if (ret < 0) {
		dev_err(&st1232_ts->client->dev, "Failed to register interrupt\n");
		goto err_irq;
	}

	atomic_dec(&st1232_ts->irq_disable);
	disable_irq_nosync(st1232_ts->irq);
	++dis_irq;
	sprintf(debug_log,"probe, dis irq:%lu\n",dis_irq);
	Touch_debug_save_log_to_ram(debug_log,1);
	printk("[Touch]st1232 probe: Disable touch irq...\r\n");		
	//cancel_delayed_work(&(st1232_ts->ts_event_work)); 
	//cancel_delayed_work(&(st1232_ts->ts_dog_work));
	/*
	if((err = sitronix_ts_get_struct_version(client, &struct_ver))){
		printk("[Touch]Unable to get struct version=%X \r\n",struct_ver);		
		dev_err(&client->dev, "Unable to get struct version!\n");
	}else{
		printk("[Touch]st1232 probe: struct version=%X \r\n",struct_ver);
		dev_dbg(&client->dev, "%s(%u): struct version=%X\n", __FUNCTION__, __LINE__, struct_ver);
		st1232_ts->struct_version = struct_ver;
	}*/
	st1232_ts->struct_version = 1;
	//sitronix_ts_set_sensitivity( client, MEDIUM_SENSITIVE);
	
	ret = input_register_device(st1232_ts->input_dev);
	if (ret)
	{
		printk("[Touch][%s] input_register_device(st1232_ts->input_dev) fail\r\n", __func__);
		goto err_register;		
	}
	input_set_drvdata(st1232_ts->input_dev, st1232_ts);
	setup_timer(&st1232_ts->timer, st1232_ts_timer, (unsigned long)st1232_ts);
	setup_timer(&st1232_ts->proximity_timer, proximity_timer, (unsigned long)st1232_ts);

	if (sitronix_ts_create_sysfs_entry(client) < 0) {
		dev_err(&client->dev, "Failed to create sitronix sysfs entry!\n");
		goto err_sysfs_entry;
	}

	buf1[0]=0x0;
	rc=i2c_master_send(st1232_ts->client, buf1,1);
	if(rc!=1)
	{
		printk("[Touch]st1232  i2c_master_send  error1\r\n");		
		tp_reset();
	}
	rc= i2c_master_recv(st1232_ts->client, buf1,16);
	if(rc!=16)
	{
		printk("[Touch]st1232  i2c_master_recv  error2\r\n");
		tp_reset();
	}			
	printk("[Touch]st1232 probe:st1232_ts->isp_enabled=%d\r\n",st1232_ts->isp_enabled);			
	printk("[Touch]st1232 probe:buf[0]=%d\r\n", buf1[0]);
	printk("[Touch]st1232 probe:buf[1]=%d\r\n", buf1[1]);
	printk("[Touch]st1232 probe:buf[2]=%d\r\n", buf1[2]);
	printk("[Touch]st1232 probe:buf[3]=%d\r\n", buf1[3]);
	printk("[Touch]st1232 probe:buf[4]=%d\r\n", buf1[4]);
	printk("[Touch]st1232 probe:buf[5]=%d\r\n", buf1[5]);
	printk("[Touch]st1232 probe:buf[6]=%d\r\n", buf1[6]);
	printk("[Touch]st1232 probe:buf[7]=%d\r\n", buf1[7]);
	printk("[Touch]st1232 probe:buf[8]=%d\r\n", buf1[8]);
	printk("[Touch]st1232 probe:buf[9]=%d\r\n", buf1[9]);
	printk("[Touch]st1232 probe:buf[10]=%d\r\n", buf1[10]);
	printk("[Touch]st1232 probe:buf[11]=%d\r\n", buf1[11]);
	printk("[Touch]st1232 probe:buf[12]=%d\r\n", buf1[12]);
	printk("[Touch]st1232 probe:buf[13]=%d\r\n", buf1[13]);
	printk("[Touch]st1232 probe:buf[14]=%d\r\n", buf1[14]);
	printk("[Touch]st1232 probe:buf[15]=%d\r\n", buf1[15]);
	tpversion=buf1[0];
	buf[0]=0x10;
	rc = i2c_master_send(st1232_ts->client, buf,1);
	if(rc!=1)
	{
		printk("[Touch]st1232  i2c_master_send  error3\r\n");
		tp_reset();
	}	

	if ((err = sitronix_ts_get_fw_revision(client, &g_rev))) {
		printk("[Touch]sitronix_ts_get_fw_revision error \r\n");
	}
	smem_cci_smem_value->touch_firmware_revision=g_rev;
	smem_cci_smem_value->touch_firmware_version=buf1[0];

	printk("[Touch]Revision=%d  tpversion=%d\r\n",g_rev,tpversion);
	printk("[Touch]touch_firmware_revision=%d  touch_firmware_version=%d\r\n",smem_cci_smem_value->touch_firmware_revision,smem_cci_smem_value->touch_firmware_version);

	printk("[Touch]st1232 probe: irq priority:%ld start\r\n",msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO)));	
	if(msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO))!=0)
	{
		msm_irq_priority_set(MSM_GPIO_TO_INT(TOUCH_GPIO),0);
	}
	printk("[Touch]st1232 probe: irq priority:%ld end\r\n",msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO)));	

	mod_timer(&st1232_ts->timer,jiffies + msecs_to_jiffies(60000));
	return 0;
	
err_sysfs_entry:
err_register:
	input_free_device(st1232_ts->input_dev);
err_irq:
	input_unregister_device(st1232_ts->input_dev);
	st1232_ts->input_dev = NULL;	
err_input:
	return ret;
err_check_functionality_failed:
	return ret;
}

static int st1232_ts_remove(struct i2c_client *client)
{
	printk("[Touch][%s]\r\n", __func__);
	sitronix_ts_destroy_sysfs_entry(client);
	return 0;
}

void st1232_early_suspend(struct early_suspend *h)
{
	printk("[Touch]st1232_early_suspend,en_irq=%ld dis_irq=%ld\r\n",en_irq,dis_irq);

}
void tp_reset(void)
{
	gpio_set_value(93, 0);	
    mdelay(5);
	gpio_set_value(93, 1);
    mdelay(150); 	
	if(sitronix_ts_set_sensitivity( st1232_ts->client, st1232_ts->sensitivity_level)==0)
	{
		printk("[Touch]tp_reset success\r\n");	
		touch_read_i2c_fail=0;//I2c success after TP reset
	}
	else
	{
		printk("[Touch]tp_reset fail\r\n");	
		touch_read_i2c_fail = 0xff;
	}
		
}

void tp_reset_and_retry(void)
{
	//printk("[Touch] tp_reset_and_retry\r\n");
	
	if (tp_reset_and_check(st1232_ts->client) != 0)
	{
		if (tp_reset_and_check(st1232_ts->client) != 0)
		{
			if(tp_reset_and_check(st1232_ts->client)!=0)
				return;
		}
	}
	touch_read_i2c_fail = 0;
}

int tp_reset_and_check(struct i2c_client *client)
{
	char buf[2];
	char data_buf[32*2];
	u32 raw_data, len;
	int ret = 0, i, count = 0;
	
	//printk("[Touch] tp_reset_and_check\r\n");	
	 tp_reset();
    // raw data check
	// set data mode
	buf[0] = 0x02;
	buf[1] = ((1) << 4 | 0x08); //Set Reg. address to 0x2 for setting data mode.
	if ((ret = i2c_master_send(client, buf, 2)) != 2)
	{
		printk("[Touch][%s] Set Reg. address to 0x2 for setting data mode fail.\r\n", __func__);
		goto _gpio_reset_end;
	}

	buf[0] = 0x40; //Set Reg. address to 0x40 for reading raw data.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
	{
		printk("[Touch][%s] Set Reg. address to 0x40 for reading raw data fail.\r\n", __func__);
		goto _gpio_reset_end;
	}

	//Read raw data of length len.
	len = sizeof(data_buf);
	if ((ret = i2c_master_recv(client,data_buf, len)) != len)
	{
		printk("[Touch][%s] Read raw data of length len [part 1] fail.\r\n", __func__);
		goto _gpio_reset_end;
	}
	
	mdelay(15);
	
	if ((ret = i2c_master_recv(client,data_buf, len)) != len)
	{
		printk("[Touch][%s] Read raw data of length len [part 2] fail.\r\n", __func__);
		goto _gpio_reset_end;
	}

	for(i = 0; i < len; i += 2) 
	{
		raw_data= ((((u32)data_buf[i]) << 8) | data_buf[i+1]);
		if (raw_data != 0)
		{
			printk("[Touch][%s] GPIO reset ok!!\r\n", __func__);
			break;
		}
		else
		{
			count++;
		}
		
		if (count > 30)
		{
			printk("[Touch][%s] GPIO reset fail!!\r\n", __func__);
			goto _gpio_reset_end;
		}
	}

	// set data mode
	buf[0] = 0x02;
	buf[1] = 0x08; //Set Reg. address to 0x2 for setting data mode.
	if ((ret = i2c_master_send(client, buf, 2)) != 2)
	{
		printk("[Touch][%s] Set Reg. address to 0x2 for setting data mode fail.\r\n", __func__);
		goto _gpio_reset_end;
	}

	#if 0
	buf[0] = 0x10; //Set Reg. address back to 0x10 for coordinates.
	if ((ret = i2c_master_send(client, buf, 1)) != 1)
	{
		printk("[Touch][%s] Set Reg. address to 0x2 for setting data mode fail.\r\n", __func__);
		goto _gpio_reset_end;
	}
	#endif
	
	return 0;
_gpio_reset_end:
	return -EIO;
}

void st1232_later_resume(struct early_suspend *h)
{
	u8 buf[1];
	int ret;
	receive_phone=0;		
	
	if(CanUpdate==1)
		return;
	else
	{
		//try to receive 1 byte
		ret = i2c_master_recv(st1232_ts->client, buf,1);
		if(ret!=1)
		{
			printk("[Touch]st1232_later_resume,  i2c_master_recv  error_a\r\n");
			tp_reset();
			ret = i2c_master_recv(st1232_ts->client, buf,1);
			if(ret!=1)
			{
				printk("[Touch]st1232_later_resume,  i2c_master_recv  error_b\r\n");
				tp_reset();
			}		
		}
		else
		        printk("[Touch] %s:buf[0]=%d  ,en_irq=%ld ,dis_irq=%ld sensitivity=%d\n",__func__,buf[0],en_irq,dis_irq,st1232_ts->sensitivity_level);

	
	}		
}

int st1232_ts_suspend(struct i2c_client *client, pm_message_t state)
{

	int rc;
	u8 buf[2];
	unsigned char debug_log[100]={0};
	buf[0]=0x2;
	buf[1]=0xA;

	printk("[Touch]st1232_ts_suspend, CanUpdate=%d isp_enabled=%d\r\n",CanUpdate,st1232_ts->isp_enabled);	

	if(CanUpdate==1)
		return 1;
	else
	{
	 	config_gpio_table(gpio_table, ARRAY_SIZE(gpio_table));	
		atomic_dec(&st1232_ts->irq_disable);
		disable_irq(st1232_ts->irq);
		++dis_irq;
		sprintf(debug_log,"suspend, dis irq:%lu\n",dis_irq);
		Touch_debug_save_log_to_ram(debug_log,1);

		printk("[Touch]st1232_ts_suspend: disable_irq\r\n");
		
		if(!st1232_ts->isp_enabled) 
		{
			//power down
			rc = i2c_master_send(st1232_ts->client, buf,2);
			if(rc!=2)
			{
				printk("[Touch]st1232_ts_suspend: i2c_master_send 0x2,0xA error_1\r\n");
				tp_reset();
				rc = i2c_master_send(st1232_ts->client, buf,2);
				if(rc!=2)
				{
					printk("[Touch]st1232_ts_suspend: i2c_master_send 0x2,0xA error_2\r\n");
					tp_reset();					
				}
			}
			else				
				printk("[Touch]st1232_ts_suspend,power down touch chip\r\n");
			st1232_suspend = 1;
		}
		return 0;		
	}
}

int st1232_ts_resume(struct i2c_client *client)
{
	int ret;
	u8 buf[2],buf1[1],buf2[1],power[1];
	unsigned char debug_log[100]={0};
	printk("[Touch]st1232_ts_resume: CanUpdate=%d isp_enabled=%d receive_phone=%d\r\n",CanUpdate,st1232_ts->isp_enabled,receive_phone);

	tp_reset_and_retry();
	
	//set 0x10
	power[0]=0x2;
	ret = i2c_master_send(st1232_ts->client, power,1);
	if(ret != 1)
	{
		printk("[Touch] %s: i2c_master_send 0x10 error_1 and will tp_reset\r\n", __FUNCTION__);
		tp_reset();	
		ret = i2c_master_send(st1232_ts->client, power,1);
		if(ret != 1)
		{
			printk("[Touch] %s: i2c_master_send 0x10 error_2 and will tp_reset\r\n", __FUNCTION__);
			tp_reset();				
		}
	}
	else
		printk("[Touch]st1232_ts_resume,set 0x2\r\n");		

	//get power down bit
	ret = i2c_master_recv(st1232_ts->client, buf2, 1);
	if(ret!=1)
	{
		printk("[Touch]st1232_ts_resume, i2c_master_recv  error_a\r\n");	
	}
	else
	        printk("[Touch] %s:check power down bit:%d \n",__func__,buf2[0]);
	//check  power down
	if((buf2[0] & 0x2) == 0x2)
	{
		//power on
		printk("[Touch]st1232_ts_resume:touch isn't power-on, it will re-power on\r\n");		
		buf[0]=0x2;
		buf[1]=0x8;
		ret = i2c_master_send(st1232_ts->client, buf,2);
		if(ret != 2)
		{
			printk("[Touch] %s: i2c_master_send 0x2,0x8 error_1 and will tp_reset\r\n", __FUNCTION__);
			tp_reset();
			ret = i2c_master_send(st1232_ts->client, buf,2);
			if(ret != 2)
			{
				printk("[Touch] %s: i2c_master_send 0x2,0x8 error_2 and will tp_reset\r\n", __FUNCTION__);
				tp_reset();
			}
		}
		else
			printk("[Touch]st1232_ts_resume, re-power-on touch chip ok\r\n");	
	}
	else
		printk("[Touch]st1232_ts_resume, touch is power-on ok after reset\r\n");	

	//set 0x10
	buf1[0]=0x10;
	ret = i2c_master_send(st1232_ts->client, buf1,1);
	if(ret != 1)
	{
		printk("[Touch] %s: i2c_master_send 0x10 error_3 and will tp_reset\r\n", __FUNCTION__);
		tp_reset();	
		ret = i2c_master_send(st1232_ts->client, buf1,1);
		if(ret != 1)
		{
			printk("[Touch] %s: i2c_master_send 0x10 error_4 and will tp_reset\r\n", __FUNCTION__);
			tp_reset();				
		}
	}
	else
		printk("[Touch]st1232_ts_resume,set 0x10\r\n");		

        if(msm_irq_priority_get(MSM_GPIO_TO_INT(TOUCH_GPIO))!=0)
	{
		printk("set touch irq priority=0\n");
		msm_irq_priority_set(MSM_GPIO_TO_INT(TOUCH_GPIO),0);
	}
	 enable_irq(st1232_ts->irq);
	 atomic_inc(&st1232_ts->irq_disable);	
	 ++en_irq;
	 sprintf(debug_log,"ts_resume, en irq:%lu\n",en_irq);
	Touch_debug_save_log_to_ram(debug_log,1);

	 printk("[Touch] %s: Enable irq, GPIO93 = %d\n",__func__,gpio_get_value(93) );
	st1232_suspend = 0;
	return 0;
}



static const struct i2c_device_id st1232_ts_id[] = {
	{ "st1232-ts", 0 },
	{ }
};

static struct i2c_driver st1232_ts_driver = {
	.probe		= st1232_ts_probe,
	.remove		= st1232_ts_remove,
	.suspend 		 = st1232_ts_suspend,
	.resume		=  st1232_ts_resume,
	.id_table	= st1232_ts_id,
	.driver = {
	.name	= "st1232-ts",
	},
};

static int __init ts_init(void)
{
    u8 buf[8];
    int ret,rc; 
    u32 x_st,y_st;	
    struct vreg *vreg_gp4 = NULL;
    printk("[Touch]set gp4=2.6v\r\n");
    vreg_gp4 = vreg_get(0, "gp4");
    rc = vreg_set_level(vreg_gp4, 2600);
    if (rc) 
    {
        printk("[Touch]%s: vreg set gp4 level failed (%d)\n",	__func__, rc);
    }

    rc = vreg_enable(vreg_gp4);
    if (rc) 
    {
        printk("[Touch]%s: vreg enable gp4 failed (%d)\n", __func__, rc);
    }
    ret = i2c_add_driver(&st1232_ts_driver);
    ts_early.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    ts_early.suspend = st1232_early_suspend;
    ts_early.resume = st1232_later_resume;
    register_early_suspend(&ts_early);	
    printk("[Touch][%s]K4H/CAP6 project, ret = %d\r\n", __func__, ret);

    ret = i2c_master_recv(st1232_ts->client, buf, 8);
    printk("[Touch] %s: ret= %d\n",__func__,ret);
    y_st = ((buf[2]&0x70)<<4)|buf[3];
    x_st = 289 - (((buf[2]&0x07)<<8)|buf[4]);
    printk("[Touch]init x_st=%d, y_st=%d\r\n", x_st, y_st);
    return ret;
}
module_init(ts_init);

static void __exit ts_exit(void)
{
	printk("[Aaron][%s]\r\n", __func__);
}
module_exit(ts_exit);

MODULE_DESCRIPTION("MSM Touch Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm_touchscreen");
