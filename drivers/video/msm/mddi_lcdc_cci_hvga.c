#include "mach/../../cci_smem.h"
#include "mach/../../smd_private.h"
//#include <mach/msm_battery.h>
static cci_smem_value_t *smem_cci_smem_value;
//hw_type
//g_lcd_type

// MDDI interface
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#ifndef CONFIG_FB_MSM_MDDI
#define CONFIG_FB_MSM_MDDI
#endif

#ifdef CONFIG_FB_MSM_MDDI

//#include <asm/gpio.h>
#include <mach/msm_rpcrouter.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define CCI_AB60_HVGA_PRIM 	1
#define LCD_BLK_EN 		32
#define LCD_RST_N 		89
#define LCD_ID0  			20
#define LCD_RST_N_113 113

#define START_DISABLE_HIBERNATION 0
#define STOP_DISABLE_HIBERNATION 1
#define DISABLE_HIBERNATION_THRESHOLD 200

#define BLK_UI_MAX_VAL	255
#define BLK_UI_MIN_VAL	30
#define BLK_LEVEL_NUM	5


typedef enum 
{
   	LCD_TYPE_AUO = 0,
  	LCD_TYPE_TPO,
  	LCD_TYPE_INVALID = 0xFF
} E_LCD_TYPE;

#define ENABLE_DEBUG 		1
#define ENABLE_DEBUG_FUNCTION 	1
#define TPO_DISABLE_HIBERNATION_FUNCTION	1

#if ENABLE_DEBUG
	#define MSG_DBG(format,args...) do { \
				printk(KERN_INFO "[LCD] %s():%d: " format, __func__, __LINE__, ##args); \
			} while(0)
	#define MSG_ERR(format,args...) do { \
				printk(KERN_ERR "[LCD][ERROR] %s():%d: " format, __func__, __LINE__, ##args); \
			} while(0)			
#if ENABLE_DEBUG_FUNCTION
	#define MSG_FUN_ENTER	printk(KERN_INFO "[LCD] %s()+: %d\r\n", __func__, __LINE__)
	#define MSG_FUN_EXIT	printk(KERN_INFO "[LCD] %s()-: %d\r\n", __func__, __LINE__)
#else
	#define MSG_FUN_ENTER	do { } while(0)
	#define MSG_FUN_EXIT	do { } while(0)
#endif

#else
	#define MSG_DBG(format,args...) do { } while(0)
	#define MSG_ERR(format,args...) do { \
				printk(KERN_ERR "[LCD][ERROR] %s():%d: " format, __func__, __LINE__, ##args); \
			} while(0)	
	#define MSG_FUN_ENTER	do { } while(0)
	#define MSG_FUN_EXIT	do { } while(0)		
#endif



static int panel_type_number = -1;
module_param_named( panel_type, panel_type_number, int, S_IRUGO | S_IWUSR | S_IWGRP);

/*******************************************************************************
 * * Local Variable Declaration
 * *******************************************************************************/
static uint32 mddi_cci_ab60_rows_per_second = 13830;	// 5200000/376
static uint32 mddi_cci_ab60_rows_per_refresh = 338;
static uint32 mddi_cci_ab60_usecs_per_refresh = 24440;	// (376+338)/5200000
static boolean mddi_cci_ab60_debug_60hz_refresh = FALSE;
//static int backlight_val = 128;
//E_LCD_TYPE g_lcd_type = LCD_TYPE_INVALID;
//static cci_hw_id_type hw_type = HW_ID_EMU;
#if TPO_DISABLE_HIBERNATION_FUNCTION
static boolean g_disable_hibernation_state = FALSE;
#endif
static boolean blk_on_timer_initialized = FALSE;
boolean LCD_powered = TRUE;
boolean First_bootup = TRUE;
static uint32 blk_value_before_display_on = 0;

static unsigned char blk_ui_range[BLK_LEVEL_NUM];
//static unsigned char other_blk_val_transform_table[BLK_LEVEL_NUM] =  { 30, 55, 90, 165, 255 };
//static unsigned char auo_blk_val_transform_table_dvt2[BLK_LEVEL_NUM] =  { 30, 55, 90, 165, 255 };
//static unsigned char auo_blk_val_transform_table_pvt[BLK_LEVEL_NUM]  =  {  5, 22, 50, 135, 255 };
//static unsigned char tpo_blk_val_transform_table_dvt2[BLK_LEVEL_NUM] =  { 30, 70, 105, 180, 255 };
static unsigned char tpo_blk_val_transform_table_pvt[BLK_LEVEL_NUM]  =  { 5, 30, 65, 130, 255 };
static bool b_blk_transform_init = FALSE;
static unsigned char *p_blk_val_transform;

struct timer_list tpo_update_timer;
struct timer_list blk_on_timer;
static DECLARE_MUTEX(tpo_timer_mutex);
static DECLARE_MUTEX(blk_on_timer_mutex);
static void tpo_lcd_power_off_sequence(void);

/*******************************************************************************
 * * Local Function Declaration
 * *******************************************************************************/
static void mddi_cci_ab60_prim_lcd_init(void);
static void mddi_cci_ab60_lcd_set_backlight(struct msm_fb_data_type *mfd);
static struct msm_panel_common_pdata *mddi_cci_ab60_pdata;
//#define mddih_host_data_packet_size_type int
//#define MDDI_DATA_PACKET_SIZE_4_BYTES 1
boolean write_client_reg_TPO(uint32 reg_addr,uint8 *reg_Value, mddih_host_data_packet_size_type reg_packet_size, boolean wait);

static void mddi_cci_ab60_set_backlight_level(int value);
static void lcd_power_on_init(void);
//static void set_lcd_backlight(struct led_classdev *led_dev, enum led_brightness value);

/*******************************************************************************
 * * External Variable Declaration
 * *******************************************************************************/
extern uint32 mddi_host_core_version;
/*******************************************************************************
 * * External Function Declaration
 * *******************************************************************************/

void blk_value_transform_init(void)
{
	int i, blk_ui_gap;

	blk_ui_range[0] = BLK_UI_MIN_VAL;
	blk_ui_range[BLK_LEVEL_NUM-1] = BLK_UI_MAX_VAL;

	blk_ui_gap = (BLK_UI_MAX_VAL - BLK_UI_MIN_VAL)/(BLK_LEVEL_NUM-1);		
	for(i=1; i<(BLK_LEVEL_NUM-1); i++ )
		blk_ui_range[i] = blk_ui_range[i-1] + blk_ui_gap;
/*
	if( hw_type > HW_ID_DVT2 )
	{
		switch( g_lcd_type )
		{
			case LCD_TYPE_TPO:
				MSG_DBG("[BLK][PVT][TPO]\n");
				p_blk_val_transform = tpo_blk_val_transform_table_pvt;
				break;
			case LCD_TYPE_AUO:
				MSG_DBG("[BLK][PVT][AUO]\n");
				p_blk_val_transform = auo_blk_val_transform_table_pvt;
				break;	
			default:
				MSG_ERR("[BLK][PVT]LCD type: INVALID!\n");
				p_blk_val_transform = other_blk_val_transform_table;
				break;
		}		
	}
	else
	{
		switch( g_lcd_type )
		{
			case LCD_TYPE_TPO:
				MSG_DBG("[BLK][TPO]\n");
				p_blk_val_transform = tpo_blk_val_transform_table_dvt2;
				break;
			case LCD_TYPE_AUO:
				MSG_DBG("[BLK][AUO]\n");
				p_blk_val_transform = auo_blk_val_transform_table_dvt2;
				break;	
			default:
				MSG_ERR("[BLK]LCD type: INVALID!\n");
				p_blk_val_transform = other_blk_val_transform_table;
				break;
		}			
	}			
	*/
	
	MSG_DBG("[BLK][PVT][TPO]\n");
	p_blk_val_transform = tpo_blk_val_transform_table_pvt;

	b_blk_transform_init = TRUE;
}

unsigned char blk_value_transform(unsigned char ui_val)
{
	int i;
	unsigned char transform_val = 0;
	
	if(!b_blk_transform_init)
		blk_value_transform_init();

	if( ui_val <= blk_ui_range[0] )
		transform_val = *p_blk_val_transform;
	else if( ui_val >= blk_ui_range[BLK_LEVEL_NUM-1] )
		transform_val = *(p_blk_val_transform + BLK_LEVEL_NUM-1);
	else
	{
		for(i=0; i<(BLK_LEVEL_NUM-1); i++ )
			if( ui_val > blk_ui_range[i] && ui_val <= blk_ui_range[i+1] )
			{
				transform_val = ( *(p_blk_val_transform +i +1 ) - *(p_blk_val_transform +i) ) * (ui_val-blk_ui_range[i]) / (blk_ui_range[i+1] - blk_ui_range[i]) + *(p_blk_val_transform +i);
				break;
			}
	}
	return transform_val;
}

void backlight_on( boolean bEnable )
{
	if(bEnable)
	{
		//MSG_DBG("John Backlight on!\r\n");
		//gpio_direction_output(LCD_BLK_EN,1);
	}
	else
	{
		//MSG_DBG("John Backlight off!\r\n");
		//gpio_direction_output(LCD_BLK_EN,0);
	}
}

void blk_on_timer_handler(unsigned long data)
{
	MSG_FUN_ENTER;

	blk_on_timer_initialized = FALSE;

	if( LCD_powered )
		backlight_on(TRUE);

	MSG_FUN_EXIT;	
}

void blk_on_proc(void)
{
	down(&blk_on_timer_mutex);

	if( !blk_on_timer_initialized )
	{
		init_timer(&blk_on_timer);
		blk_on_timer.function = blk_on_timer_handler;
		blk_on_timer.expires = jiffies + (HZ/100 * 30);		
		add_timer(&blk_on_timer);	
		blk_on_timer_initialized = TRUE;
	}	
	else
	{
		blk_on_timer.function = blk_on_timer_handler;
		mod_timer(&blk_on_timer, jiffies + (HZ/100 * 30));
	}

	up(&blk_on_timer_mutex);
}

#if TPO_DISABLE_HIBERNATION_FUNCTION
void disable_hibernation_timer_handler(unsigned long data)
{
	boolean bEnable = (boolean)data;
	
	MSG_FUN_ENTER;
	
	if( bEnable )
	{
		if (!g_disable_hibernation_state )
		{
			mddi_host_disable_hibernation(TRUE);
			g_disable_hibernation_state = TRUE;
		}
		tpo_update_timer.data = FALSE;
		mod_timer(&tpo_update_timer, jiffies + (HZ*2));				
	}
	else
	{
		if ( g_disable_hibernation_state )
		{
			mddi_host_disable_hibernation(FALSE);

			g_disable_hibernation_state = FALSE;
		}
	}
	
	MSG_FUN_EXIT;	
}
#endif

void disable_hibernation_proc(boolean bEnable)
{
#if TPO_DISABLE_HIBERNATION_FUNCTION
/*
	static bool timer_init = FALSE;

	if( hw_type >= HW_ID_PVT )
		return;
		
	if( bEnable && backlight_val < DISABLE_HIBERNATION_THRESHOLD  )
		return;
	
	if( !timer_init )
	{
		init_timer(&tpo_update_timer);
		tpo_update_timer.function = disable_hibernation_timer_handler;
		tpo_update_timer.data = (unsigned long)bEnable;
		tpo_update_timer.expires = jiffies + (HZ/1000);
		add_timer(&tpo_update_timer);	
		timer_init = TRUE;
	}
	else
	{
		tpo_update_timer.data = (unsigned long)bEnable;
		mod_timer(&tpo_update_timer, jiffies + (HZ/1000));
	}
*/
#endif
}
void update_backlight_to_battery(int value)
{
/*
    cci_batt_device_status_update(0xE0,0);
    if (value >= 235)
        cci_batt_device_status_update(CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_150,1);
    else if (value >= 145)
        cci_batt_device_status_update(CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_100,1);
    else if (value >= 30)
        cci_batt_device_status_update(CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_50,1);
*/
}
static void mddi_cci_ab60_set_backlight_level(int value)
{
	//static int enable_pwm = 0;
	//int dvt_or_evt = 2;
	int err = 0;
	int value_tmp = 0;
	uint8  RegPrm[5];
	int max;
	int min;

	MSG_FUN_ENTER;
        printk(KERN_INFO "[Wells] want the blk value=%d\r\n", value);
	if ( !LCD_powered )
        {
                blk_value_before_display_on = value;
		return;
        }
/*
	if (smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
	{
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_DVT1) ||
			  (smem_cci_smem_value->cci_hw_id > HW_ID_DVT1))
		{
		  // K4H DVT
		  // Dark 0~ light 255
		  max 				= 255;
		  min 				= 31;
		  dvt_or_evt	= 2;

			value_tmp = value;
			if ( value_tmp > max )
				value_tmp = max;
			else if (value_tmp == 0)
				value_tmp = 0;
			else if ( (value_tmp < min) && (value_tmp != 0) )
				value_tmp = 8;
		}
		else
		{
			// K4H EVT
			// Dark 255~ light 0
			max 				= 224;
			min 				= 0;
			dvt_or_evt	= 1;

			value_tmp = 255 - value;
			if ( (value_tmp > max) && (value_tmp != 255) )
				value_tmp = 247;
			else if (value_tmp == 255)
				value_tmp = 255;
			else if ( value_tmp < min )
				value_tmp = min;
		}
	}
	else if (smem_cci_smem_value->cci_project_id == PROJECT_ID_CAP6)
	{
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_PVT1) ||
			  (smem_cci_smem_value->cci_hw_id > HW_ID_PVT1))
		{
			// CAP6 PVT1
		  // Dark 0~ light 255
		  max 				= 255;
		  min 				= 31;
		  dvt_or_evt	= 2;

			value_tmp = value;
			if ( value_tmp > max )
				value_tmp = max;
			else if (value_tmp == 0)
				value_tmp = 0;
			else if ( (value_tmp < min) && (value_tmp != 0) )
				value_tmp = 8;
		}
		else
		{
			// CAP6 EVT
			// Dark 255~ light 0
			max 				= 217;
			min 				= 0;
		  dvt_or_evt	= 1;

			value_tmp = 255 - value;
			if ( (value_tmp > max) && (value_tmp != 255) )
				value_tmp = max;
			else if (value_tmp == 255)
				value_tmp = 255;
			else if ( value_tmp < min )
				value_tmp = min;
		}
	}
	else if (smem_cci_smem_value->cci_project_id == PROJECT_ID_CAP2)
	{
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_DVT1_2) ||
			  (smem_cci_smem_value->cci_hw_id > HW_ID_DVT1_2))
		{
			// CAP2 DVT1_2
		  // Dark 0~ light 255
		  max 				= 255;
		  min 				= 31;
		  dvt_or_evt	= 2;

			value_tmp = value;
			if ( value_tmp > max )
				value_tmp = max;
			else if (value_tmp == 0)
				value_tmp = 0;
			else if ( (value_tmp < min) && (value_tmp != 0) )
				value_tmp = 8;
		}
		else
		{
			// CAP2 EVT
			// Dark 255~ light 0
			max 				= 217;
			min 				= 0;
		  dvt_or_evt	= 1;

			value_tmp = 255 - value;
			if ( (value_tmp > max) && (value_tmp != 255) )
				value_tmp = max;
			else if (value_tmp == 255)
				value_tmp = 255;
			else if ( value_tmp < min )
				value_tmp = min;
		}
	}
	else
	{
	  // Dark 0~ light 255
	  max 				= 255;
	  min 				= 31;
	  dvt_or_evt	= 2;

		value_tmp = value;
		if ( value_tmp > max )
			value_tmp = max;
		else if (value_tmp == 0)
				value_tmp = 0;
		else if ( (value_tmp < min) && (value_tmp != 0) )
			value_tmp = 8;
	}
	*/
	//MSG_DBG("John value=%d value_tmp=%d\r\n", value, value_tmp);
    
    //C4: infor backlihgt level to battery driver
    update_backlight_to_battery(value);
#if 0
	if (dvt_or_evt == 1)
	{
		if ((enable_pwm == 0) && (value_tmp < 255))
		{
			RegPrm[0]	 = 0x2c;
			RegPrm[1]	 = 0x00;
			RegPrm[2]	 = 0x00;
			RegPrm[3]	 = 0x00;
			err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
            enable_pwm = 1;
		}
		else if ((enable_pwm == 1) && (value_tmp == 255))
		{
			RegPrm[0]	 = 0x00;
			RegPrm[1]	 = 0x00;
			RegPrm[2]	 = 0x00;
			RegPrm[3]	 = 0x00;
			err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
            enable_pwm = 0;
		}
	}
	else
	{
		if ((enable_pwm == 0) && (value_tmp > 0))
		{
			RegPrm[0]	 = 0x2c;
			RegPrm[1]	 = 0x00;
			RegPrm[2]	 = 0x00;
			RegPrm[3]	 = 0x00;
			err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
            enable_pwm = 1;
		}
		else if ((enable_pwm == 1) && (value_tmp == 0))
		{
			RegPrm[0]	 = 0x00;
			RegPrm[1]	 = 0x00;
			RegPrm[2]	 = 0x00;
			RegPrm[3]	 = 0x00;
			err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
            enable_pwm = 0;
		}
	}
#endif
          max = 255;
	  min = 31;
          value_tmp = value;
	  if ( value_tmp > max )
		value_tmp = max;
	  else if (value_tmp == 0)
		value_tmp = 0;
	  else if ( (value_tmp < min) && (value_tmp != 0) )
		value_tmp = 8;

	//if (smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
	{
		//printk("[Aaron][%s] value_tmp=%d\r\n", __func__, value_tmp);
		
		// Reduce the power consumption
		if (value_tmp < 120)
		{
			//printk("[Aaron][%s] < 100\r\n", __func__);
			value_tmp = (value_tmp * 35) / 100;
		}
		else if (value_tmp < 140)
		{
			//printk("[Aaron][%s] < 140\r\n", __func__);
			value_tmp = (value_tmp * 41) / 100;
		}
		else if (value_tmp < 160)
		{
			//printk("[Aaron][%s] < 160\r\n", __func__);
			value_tmp = (value_tmp * 50) / 100;
		}
		else if (value_tmp < 180)
		{
			//printk("[Aaron][%s] < 180\r\n", __func__);
		    value_tmp = (value_tmp * 60) / 100;
	    }
		else if (value_tmp < 200)
		{
			value_tmp = (value_tmp * 70) / 100;
		}
		else if (value_tmp < 220)
		{
			value_tmp = (value_tmp * 80) / 100;
		}
		else if (value_tmp < 240)
		{
			value_tmp = (value_tmp * 90) / 100;
		}
	}

    printk(KERN_INFO "[Wells] the true blk value to mddi value_tmp=%d\r\n", value_tmp);
    RegPrm[0]    = 0x2c;
    RegPrm[1]    = 0x00;
    RegPrm[2]    = 0x00;
    RegPrm[3]    = 0x00;
    err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

    // printk(KERN_INFO "[Wells] the true blk value to mddi value=%d\r\n", value);
        //value_tmp = 200;
	RegPrm[0]	 = value_tmp;
        //RegPrm[0]	 = value;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00;
	err = write_client_reg_TPO(0x003c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	msleep(10);//mdelay(10);

	MSG_FUN_EXIT;
}
/*
static void set_lcd_backlight(struct led_classdev *led_dev, enum led_brightness value)
{
}
*/

boolean write_client_reg_TPO(uint32 reg_addr,uint8 *reg_Value, mddih_host_data_packet_size_type reg_packet_size, boolean wait)
{
	mddih_reg_write_type_uint8   reg_write;
	reg_write.addr  = reg_addr;
	reg_write.pValue = reg_Value;
	reg_write.packet_size = reg_packet_size;
	return(boolean) mddi_queue_multi_register_write_uint8(reg_write, wait, 0);
}

/*
static void auo_lcd_power_on_sequence(void)
{
}
*/

void ILI9486_EnterSleep(void)
{
		uint8 RegPrm[4];
		// Vender: Wait 10ms before write mddi command
		msleep(10);//mdelay(10);	// Delay 10ms

		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 1
		write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		msleep(10);//mdelay(10);	// Delay 10ms

		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 1
		write_client_reg_TPO(0x0010, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		msleep(120);//mdelay(120);	// Delay 10ms
		printk(KERN_INFO "[LCM] +ILI9486_EnterSleep\r\n");

}
void ILI9486_ExitSleep(void)
{
		uint8 RegPrm[4];
		// Vender: Wait 10ms before write mddi command
		msleep(10);//mdelay(10);	// Delay 10ms

		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 1
		write_client_reg_TPO(0x0011, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		msleep(120);//mdelay(120);	// Delay 120ms
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 1
		write_client_reg_TPO(0x0029, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
	printk(KERN_INFO "[LCM] +ILI9486_ExitSleep\r\n");

}

static void tpo_lcd_power_on_sequence(void)
{

	//int rc, err=0, retry=0;
    int err=0, retry=0;
    int line=0;
	MSG_FUN_ENTER;
//ILI9486_ExitSleep();
//#if 0
	// Vender: Wait 1ms after turn on gp5 and rfrx2
    gpio_direction_output(LCD_RST_N, 0);
    gpio_direction_output(LCD_RST_N_113, 0);
	msleep(10);//mdelay(10); // Delay 10ms
    lcd_power_on_init();
    msleep(10);//mdelay(10);
    gpio_direction_output(LCD_RST_N, 1);
    gpio_direction_output(LCD_RST_N_113, 1);
#if 0
    if(smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
    {
        switch(smem_cci_smem_value->cci_hw_id)
        {
            case HW_ID_DVT1:
            case HW_ID_DVT1_2:
            case HW_ID_DVT2:
            case HW_ID_DVT2_2:
            case HW_ID_DVT2_3:
                gpio_direction_output(LCD_RST_N, 1);
                break;
            case HW_ID_PVT:
                gpio_direction_output(LCD_RST_N_113, 1);
                break;
            default:
                gpio_direction_output(LCD_RST_N, 1);
                gpio_direction_output(LCD_RST_N_113, 1);
                break;
        }
    }
#endif
	/*
	if (smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_DVT2) ||
			  (smem_cci_smem_value->cci_hw_id > HW_ID_DVT2))
			  */
			//gpio_direction_output(LCD_RST_N_113, 0);

	/*
	if (smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_DVT2) ||
			  	(smem_cci_smem_value->cci_hw_id > HW_ID_DVT2))
			  	*/
			//gpio_direction_output(LCD_RST_N_113, 1);
	msleep(60);//mdelay(60);	// Delay 60ms
//#endif
restart:
	retry++;
	err = 0;
	{
		uint8 RegPrm[5];

        //printk("[STEVE]The '%d' time in restart.\n", retry);
        //printk("[STEVE]restart from line %d\n", line);
		// Vender: Wait 10ms before write mddi command
		msleep(10);//mdelay(10);	// Delay 10ms

		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 1
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 2
		err = write_client_reg_TPO(0x00f2, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 3
		err = write_client_reg_TPO(0x00e2, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 4
		err = write_client_reg_TPO(0x00e3, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x1c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 5
		err = write_client_reg_TPO(0x00e4, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x1c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 6
		err = write_client_reg_TPO(0x00e5, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 7
		err = write_client_reg_TPO(0x00e6, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x1c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 8
		err = write_client_reg_TPO(0x00e7, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x90;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 9
		err = write_client_reg_TPO(0x00e8, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 10
		err = write_client_reg_TPO(0x0029, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x22;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 11
		err = write_client_reg_TPO(0x0018, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 12
		err = write_client_reg_TPO(0x002a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x13;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 13
		err = write_client_reg_TPO(0x002b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
#if 0 // bpp=18bits
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 14
		err = write_client_reg_TPO(0x0040, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0041, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x39;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0042, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x35;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0043, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x2e;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0044, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3e;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0045, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0046, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x7f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0047, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0048, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x05;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0049, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x06;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x1f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0050, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x11;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0051, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0052, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x06;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0053, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x04;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0054, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3e;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0055, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0056, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x45;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0057, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0058, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x10;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0059, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x19;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x1a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x14;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0xc0;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
#else // bpp=16bits
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 14
		err = write_client_reg_TPO(0x0040, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x2b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0041, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x2f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0042, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x35;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0043, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x36;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0044, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x3f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0045, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x3a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0046, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x7f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0047, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x0d;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0048, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
        RegPrm[0]	 = 0x03;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0049, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x06;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x14;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x1f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0050, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x09;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0051, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x0a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0052, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x10;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0053, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x14;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0054, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x3e;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0055, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0056, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x45;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0057, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0058, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x0b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0059, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x19;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x1c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x12;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0xc0;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
#endif		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 15
		err = write_client_reg_TPO(0x0002, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 16
		err = write_client_reg_TPO(0x0003, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 17
		err = write_client_reg_TPO(0x0004, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x3f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 18
		err = write_client_reg_TPO(0x0005, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 19
		err = write_client_reg_TPO(0x0006, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 20
		err = write_client_reg_TPO(0x0007, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 21
		err = write_client_reg_TPO(0x0008, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0xdf;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 22
		err = write_client_reg_TPO(0x0009, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x92;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 23
		err = write_client_reg_TPO(0x0024, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x73;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 24
		err = write_client_reg_TPO(0x0025, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x04;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 25
		err = write_client_reg_TPO(0x001a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x30;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 26
		err = write_client_reg_TPO(0x001b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x22;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 27
		err = write_client_reg_TPO(0x001d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 28
		err = write_client_reg_TPO(0x0019, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x03;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 29
		err = write_client_reg_TPO(0x001c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x02;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 30
		err = write_client_reg_TPO(0x0001, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x80;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 31
		err = write_client_reg_TPO(0x001f, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(10);//mdelay(10);
		
		RegPrm[0]	 = 0x90;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 32
		err = write_client_reg_TPO(0x001f, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(10);//mdelay(10);
			
		RegPrm[0]	 = 0xd2;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 33
		err = write_client_reg_TPO(0x001f, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(10);//mdelay(10);

		// Clear Gram to 0x0
		RegPrm[0]	 = 0x02;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 34-01
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x03;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 35-02
		err = write_client_reg_TPO(0x0017, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(5);//mdelay(5);

		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 36-03
		err = write_client_reg_TPO(0x0017, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 37-04
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x38;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 38
		err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(40);//mdelay(40);

		RegPrm[0]	 = 0x3c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 39
		err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		msleep(40);//mdelay(40);


		RegPrm[0]	 = 0x0b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 40
		err = write_client_reg_TPO(0x0016, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
#if 0 // bpp=18bits
		RegPrm[0]	 = 0x06;
#else // bpp=16bits
		RegPrm[0]	 = 0x05;
#endif
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 41
		err = write_client_reg_TPO(0x0017, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x08;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 42
		err = write_client_reg_TPO(0x0060, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x1f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 43
		err = write_client_reg_TPO(0x002d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 44
		err = write_client_reg_TPO(0x0026, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 45
		err = write_client_reg_TPO(0x0080, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 46
		err = write_client_reg_TPO(0x0081, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 47
		err = write_client_reg_TPO(0x0082, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	    if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 48
		err = write_client_reg_TPO(0x0083, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
#if 0	
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 49
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 50
		err = write_client_reg_TPO(0x00c5, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 51
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

		RegPrm[0]	 = 0x80;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 52
		err = write_client_reg_TPO(0x003c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

		RegPrm[0]	 = 0x2c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 53
		err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
#endif
	
		// Set the frequence of PWM = 12k
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x00c5, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x00ff, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
        if(err && retry<=3){
			line = __LINE__;
			goto restart;
		}
		printk(KERN_INFO "[John]%s: K4H\r\n", __func__);
			
		
#if 0
		// Backlight control enable
		RegPrm[0]	 = 0x2c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x003d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
#endif
	}
	
	if( (retry <= 3) && ((err == 1)) ) {
		//gpio_direction_output(LCD_RST_N, 0);		
		//gpio_direction_output(LCD_RST_N_113, 0);
		msleep(1);//mdelay(1);	// Delay 1ms
		MSG_ERR("goto restart\r\n");
		goto restart;
	}

	MSG_FUN_EXIT;
}

void tpo_lcd_power_on_sequence_part(void)
{

	//int rc, err=0, retry=0;
        int err=0, retry=0;
	MSG_FUN_ENTER;
       
restart:
        printk(KERN_INFO "[wells] tpo_lcd_power_on_sequence_part\r\n");
	retry++;
	err = 0;
	{
		uint8 RegPrm[5];

		// bpp=16bits
		RegPrm[0]	 = 0x01;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 14
		err = write_client_reg_TPO(0x0040, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x2b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0041, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x2f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0042, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x35;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0043, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x36;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0044, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0045, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0046, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x7f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0047, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0d;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0048, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x03;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0049, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x06;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x14;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x1f;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x004c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0050, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x09;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0051, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0a;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0052, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x10;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0053, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x14;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0054, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x3e;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0055, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0056, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x45;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0057, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x00;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0058, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x0b;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x0059, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x19;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005a, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x1c;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005b, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0x12;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005c, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
		
		RegPrm[0]	 = 0xc0;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00;
		err = write_client_reg_TPO(0x005d, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

		// bpp=16bits
		RegPrm[0]	 = 0x05;
		RegPrm[1]	 = 0x00;
		RegPrm[2]	 = 0x00;
		RegPrm[3]	 = 0x00; // 41
		err = write_client_reg_TPO(0x0017, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	}
	
	if( (retry <= 3) && ((err == 1)) ) {
		//gpio_direction_output(LCD_RST_N, 0);		
		//gpio_direction_output(LCD_RST_N_113, 0);
		msleep(1);//mdelay(1);	// Delay 1ms
		MSG_ERR("goto restart\r\n");
		goto restart;
	}
	
	MSG_FUN_EXIT;
}
/*
static void auo_lcd_power_off_sequence(void)
{
}
*/

static void tpo_lcd_power_off_sequence(void)
{
	int  err=0;
    int rc;
    struct vreg *vreg_rfrx2 = NULL;
    struct vreg *vreg_gp5 = NULL;

	uint8 RegPrm[5];
        printk(KERN_INFO "[Wells] +tpo_lcd_power_off_sequence\r\n");
	MSG_FUN_ENTER;

	RegPrm[0]	 = 0x38;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 1
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	msleep(40);//mdelay(40);

	RegPrm[0]	 = 0x24;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 2
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	msleep(40);//mdelay(40);

	RegPrm[0]	 = 0x04;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 3
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);
	//ILI9486_EnterSleep();
    gpio_direction_output(LCD_RST_N, 0);
    gpio_direction_output(LCD_RST_N_113, 0);
#if 0
    if(smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
    {
        switch(smem_cci_smem_value->cci_hw_id)
        {
            case HW_ID_DVT1:
            case HW_ID_DVT1_2:
            case HW_ID_DVT2:
            case HW_ID_DVT2_2:
            case HW_ID_DVT2_3:
                gpio_direction_output(LCD_RST_N, 0);
                break;
            case HW_ID_PVT:
                gpio_direction_output(LCD_RST_N_113, 0);
                break;
            default:
                gpio_direction_output(LCD_RST_N, 0);
                gpio_direction_output(LCD_RST_N_113, 0);
                break;
        }
    }
#endif
    //msleep(150);//added by steve@cci

    vreg_rfrx2 = vreg_get(0, "rfrx2");
    rc = vreg_disable(vreg_rfrx2);
    if (rc)
    {
        printk("%s: vreg enable rfrx2 failed (%d)\r\n", __func__, rc);
    }

    vreg_gp5 = vreg_get(0, "gp5");
    rc = vreg_disable(vreg_gp5);
    if (rc)
    {
        printk("%s: vreg enable gp5 failed (%d)\r\n", __func__, rc);
    }

/*
	int rc;
	struct vreg *vreg_rfrx2 = NULL;
	struct vreg *vreg_gp5 = NULL;

	int  err=0;
        
	uint8 RegPrm[5];
        printk(KERN_INFO "[Wells] +tpo_lcd_power_off_sequence\r\n");
	MSG_FUN_ENTER;

	RegPrm[0]	 = 0x38;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 1
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

	msleep(40);

	RegPrm[0]	 = 0x24;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 2
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

	msleep(40);

	RegPrm[0]	 = 0x04;
	RegPrm[1]	 = 0x00;
	RegPrm[2]	 = 0x00;
	RegPrm[3]	 = 0x00; // 3
	err = write_client_reg_TPO(0x0028, RegPrm, MDDI_DATA_PACKET_SIZE_4_BYTES, TRUE);

	gpio_direction_output(LCD_RST_N, 0);	 
	gpio_direction_output(LCD_RST_N_113, 0);

	vreg_rfrx2 = vreg_get(0, "rfrx2");	
	rc = vreg_disable(vreg_rfrx2);
	if (rc)
	{
		printk("%s: vreg enable rfrx2 failed (%d)\r\n", __func__, rc);
	}

	vreg_gp5 = vreg_get(0, "gp5");
	rc = vreg_disable(vreg_gp5);
	if (rc)
	{
		printk("%s: vreg enable gp5 failed (%d)\r\n", __func__, rc);
	}

*/
	MSG_FUN_EXIT;
}

static void lcd_power_on_init(void)
{
	int rc;
	struct vreg *vreg_rfrx2 = NULL;
	struct vreg *vreg_gp5 = NULL;

    printk(KERN_INFO "[Wells] lcd_power_on_init\r\n");
	MSG_DBG("John lcd_power_on_init 1\r\n");
	vreg_gp5 = vreg_get(0, "gp5");
    if(smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
    {
        switch(smem_cci_smem_value->cci_hw_id)
        {
            case HW_ID_DVT1:
            case HW_ID_DVT1_2:
            case HW_ID_DVT2:
            case HW_ID_DVT2_2:
            case HW_ID_DVT2_3:
                rc = vreg_set_level(vreg_gp5, 1800);
                break;
            case HW_ID_PVT:
	            rc = vreg_set_level(vreg_gp5, 2600);
                break;
            default:
                rc = vreg_set_level(vreg_gp5, 2600);
                break;
        }
	}
	if (rc) 
	{
		printk(KERN_INFO "%s: vreg set gp5 level failed (%d)\r\n",	__func__, rc);
	}
	rc = vreg_enable(vreg_gp5);
	if (rc) 
	{
		MSG_ERR("%s: vreg enable gp5 failed (%d)\r\n", __func__, rc);
                printk("%s: vreg enable gp5 failed (%d)\r\n", __func__, rc);
	}

	vreg_rfrx2 = vreg_get(0, "rfrx2");

	rc = vreg_set_level(vreg_rfrx2, 2800);
	if (rc) 
	{
		printk(KERN_INFO "%s: vreg set rfrx2 level failed (%d)\r\n", __func__, rc);
	}
	rc = vreg_enable(vreg_rfrx2);
	if (rc) 
	{
                printk("%s: vreg enable gp2 failed (%d)\r\n", __func__, rc);
		MSG_ERR("%s: vreg enable gp2 failed (%d)\r\n", __func__, rc);
	}
	//MSG_DBG("John lcd_power_on_init 2\r\n");

}

static void mddi_cci_ab60_power_on_sequence(void)
{
	MSG_FUN_ENTER;
/*
	switch( g_lcd_type )
	{
		case LCD_TYPE_TPO:
			//MSG_DBG("John mddi_cci_ab60_power_on_sequence LCD_TYPE_TPO\r\n");
			tpo_lcd_power_on_sequence();
			backlight_on(TRUE);
			break;
		case LCD_TYPE_AUO:
			//MSG_DBG("John mddi_cci_ab60_power_on_sequence LCD_TYPE_AUO\r\n");
			auo_lcd_power_on_sequence();
			backlight_on(TRUE);
			break;	
		default:
			MSG_ERR("LCD type: INVALID!\n");
			break;
	}
*/	
	tpo_lcd_power_on_sequence();
	backlight_on(TRUE);

	MSG_FUN_EXIT;
}

#if 1
static void mddi_cci_ab60_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	//unsigned char transform_val;

	MSG_FUN_ENTER;

	//MSG_DBG("########  mfd->bl_level = %d ############## \n", mfd->bl_level);
#if 0
	transform_val = blk_value_transform((unsigned char)mfd->bl_level);
	backlight_val = 0xFF - transform_val;
	//MSG_DBG(" mfd->bl_level=%d, transform_val=%d, backlight_val=%d ############## \n", mfd->bl_level, transform_val, backlight_val);
	mddi_cci_ab60_set_backlight_level(backlight_val);
#else
	mddi_cci_ab60_set_backlight_level(mfd->bl_level);
#endif
/*	
	backlight_val =255 - (mfd->bl_level & 0xFF);
	mddi_cci_ab60_set_backlight_level(backlight_val);
*/	

	MSG_FUN_EXIT;	
}
#else

static DEFINE_MUTEX(key_light_mutex);
static void mddi_cci_ab60_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	int32 level=0x0;
        //CB60_SCR_4644 SOF
        u8 buf[10];
        //unsigned i = 0;

	MSG_FUN_ENTER;

	memset(buf, 0X0, sizeof(buf));
        //CB60_SCR_4644 EOF
	MSG_DBG("######## Config LCM_BL_EN GPIO 91 level = %d ############## \n", mfd->bl_level);

	level = mfd->bl_level;

	//james@cci added for backlight control
	if(level == 7 ) {
		backlight_on(TRUE);
		//MSG_DBG("%s LCD_BLK_EN = %d\n",__func__,gpio_get_value(LCD_BLK_EN));
		MSG_DBG("do backlighton\n");
	}
	//james@cci added for backlight control
	if(level == 0) {//change to 1
		backlight_on(FALSE);
		//MSG_DBG("%s LCD_BLK_EN = %d\n",__func__,gpio_get_value(LCD_BLK_EN));
                //CB60_SCR_4644 SOF
mutex_lock(&key_light_mutex);
#if 0
                //del_timer(&keylight_timer);
		i = 0;
		hrtimer_cancel(&led_timer);
	        do{
                buf[i] = wled_off[i];
		buf[i] = LED_CURRENT_STATE[i];
                i++;
               }while(i<10);
               i2c_master_send(TCA6507_i2c_client, buf, 10);  
		keylight_flag = 0 ;
#endif
mutex_unlock(&key_light_mutex);
                //CB60_SCR_4644 EOF
		MSG_DBG("do backlightoff\n");
	}

	MSG_FUN_EXIT;
}

#endif

static void mddi_cci_ab60_prim_lcd_init(void)
{
	MSG_FUN_ENTER;
         
	//gpio_direction_output(LCD_RST_N, 1);
    //gpio_direction_output(LCD_RST_N_113, 1);
#if 0
    if(smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
    {
        switch(smem_cci_smem_value->cci_hw_id)
        {
            case HW_ID_DVT1:
            case HW_ID_DVT1_2:
            case HW_ID_DVT2:
            case HW_ID_DVT2_2:
            case HW_ID_DVT2_3:
                gpio_direction_output(LCD_RST_N, 0);
                break;
            case HW_ID_PVT:
                gpio_direction_output(LCD_RST_N_113, 0);
                break;
            default:
                gpio_direction_output(LCD_RST_N, 0);
                gpio_direction_output(LCD_RST_N_113, 0);
                break;
        }
    }
#endif
    printk(KERN_INFO "[wells] +mddi_cci_ab60_prim_lcd_init \r\n");
	//lcd_power_on_init();
	if( !LCD_powered )
	{
		MSG_DBG("John mddi_cci_ab60_prim_lcd_init 1\r\n");
		mddi_cci_ab60_power_on_sequence();
		msleep(5);//mdelay(5);
		LCD_powered = TRUE;

                if(blk_value_before_display_on)
                {
                  mddi_cci_ab60_set_backlight_level(blk_value_before_display_on);
                  blk_value_before_display_on = 0;
                }
	}
	else
	{
		MSG_DBG("John mddi_cci_ab60_prim_lcd_init 2\r\n");
		if (First_bootup)
		{
			tpo_lcd_power_on_sequence_part();
		}
		First_bootup = FALSE;
 	}

	/* Set the MDP pixel data attributes for Primary Display */
	mddi_host_write_pix_attr_reg(MDDI_DEFAULT_PRIM_PIX_ATTR);  	

	MSG_FUN_EXIT;
}

static int mddi_cci_ab60_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	MSG_FUN_ENTER;
	//BKL_EN=1;
        printk(KERN_INFO "[Wells] mddi_cci_ab60_lcd_on\r\n");
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
	{
		MSG_ERR("mfd is NULL.\r\n");
		MSG_FUN_EXIT;
		return -ENODEV;
	}

	if (mfd->key != MFD_KEY)
	{
		MSG_ERR("mfd->key != MFD_KEY.\r\n");
		MSG_FUN_EXIT;		
		return -EINVAL;
	}

	if (mfd->panel.id == CCI_AB60_HVGA_PRIM)
        {
                printk(KERN_INFO "[Wells] call mddi_cci_ab60_prim_lcd_init\r\n");
		mddi_cci_ab60_prim_lcd_init();
       
        }
	else
	{
		MSG_ERR(KERN_ALERT "%s:%d --> Unknown LCD Panel id = %d, return -EINVAL\r\n", __func__, __LINE__, mfd->panel.id);
		return -EINVAL;
	}

	MSG_FUN_EXIT;	

	return 0;
}

static int mddi_cci_ab60_lcd_off(struct platform_device *pdev)
{
	MSG_FUN_ENTER;
	LCD_powered = FALSE;
	//BKL_EN=0;
	/*
	switch( g_lcd_type )
	{
		case LCD_TYPE_TPO:
			backlight_on(FALSE);
			tpo_lcd_power_off_sequence();
			break;
		case LCD_TYPE_AUO:
			backlight_on(FALSE);
			auo_lcd_power_off_sequence();
			break;	
		default:
			MSG_ERR("LCD type: INVALID!\r\n");
			break;
	}
	*/
	backlight_on(FALSE);
	tpo_lcd_power_off_sequence();

	//disable_hibernation_proc(FALSE);

	MSG_FUN_EXIT;
	return 0;
}
/*
static E_LCD_TYPE get_lcd_type(void)
{
#if 0
	int lcd_id;
	E_LCD_TYPE lcd_type;
	
	gpio_direction_input(LCD_ID0);
	lcd_id = gpio_get_value(LCD_ID0);
	
	switch(lcd_id)
	{

		case 0:
			lcd_type = LCD_TYPE_AUO;
			break;
		case 1:
			lcd_type = LCD_TYPE_TPO;
			break;				
		default:
			lcd_type = LCD_TYPE_INVALID;
			break;
	}
	return lcd_type;
#else
	return LCD_TYPE_TPO;
#endif
}
*/
static int __devinit mddi_cci_ab60_probe(struct platform_device *pdev)
{
	MSG_FUN_ENTER;

	if (pdev->id == 0) {
		MSG_DBG("%s id == 0\r\n", __func__);
		mddi_cci_ab60_pdata = pdev->dev.platform_data;
		return 0;
	}	

//	g_lcd_type = get_lcd_type();

	//MSG_DBG("%s g_lcd_type=%d\r\n", __func__, g_lcd_type);
	//panel_type_number = (int)g_lcd_type;
	
	msm_fb_add_device(pdev);

	MSG_FUN_EXIT;
	return 0;
}

static int mddi_cci_ab60_suspend(struct platform_device *pdev, pm_message_t state)
{
/*
	int rc;
	struct vreg *vreg_rfrx2 = NULL;
	struct vreg *vreg_gp5 = NULL;
	
	printk("[Wells]+[%s] state.event=%d\r\n", __func__,state.event);
	
	gpio_direction_output(LCD_RST_N, 0);
	
	gpio_direction_output(LCD_RST_N_113, 0);
	//gpio_direction_output(LCD_BLK_EN, 0);

	vreg_rfrx2 = vreg_get(0, "rfrx2");	
	rc = vreg_disable(vreg_rfrx2);
	if (rc)
	{
		printk("%s: vreg enable rfrx2 failed (%d)\r\n", __func__, rc);
	}

	vreg_gp5 = vreg_get(0, "gp5");
	rc = vreg_disable(vreg_gp5);
	if (rc)
	{
		printk("%s: vreg enable gp5 failed (%d)\r\n", __func__, rc);
	}
*/
	return 0;
}
/*
static int mddi_cci_ab60_early_suspend(struct platform_device *pdev)
{
	//printk("[Aaron][%s]\r\n", __func__);

	return 0;
}

static int mddi_cci_ab60_later_resume(struct platform_device *pdev)
{
	//printk("[Aaron][%s]\r\n", __func__);

	return 0;
}
*/
static int mddi_cci_ab60_resume(struct platform_device *pdev)
{
	//printk("[Aaron][%s]\r\n", __func__);

	
	return 0;
}
/*
static struct led_classdev lcd_backlight = {
	.name = "bkl",
	.brightness_set = set_lcd_backlight,
	.default_trigger = "charger",
};
*/

static struct platform_driver this_mddi_driver = {
	.probe  = mddi_cci_ab60_probe,
	.suspend = mddi_cci_ab60_suspend,
	.resume = mddi_cci_ab60_resume,
	.driver = {
		.name   = "mddi_cci_ab60_wvga",
	},
};

static struct msm_fb_panel_data mddi_cci_ab60_panel_data = {
	.on = mddi_cci_ab60_lcd_on,
	.off = mddi_cci_ab60_lcd_off,
	.set_backlight = mddi_cci_ab60_lcd_set_backlight,
	.set_vsync_notifier = NULL,
};

static struct platform_device this_mddi_device = {
	.name   = "mddi_cci_ab60_wvga",
	.id	= CCI_AB60_HVGA_PRIM,
	.dev	= {
		.platform_data = &mddi_cci_ab60_panel_data,
	}
};

static int __init mddi_cci_ab60_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

/*#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	u32 id;
	id = mddi_get_client_id();

	if (((id >> 16) != 0x0) || ((id & 0xffff) != 0x8835))
		return 0;
#endif*/
	MSG_FUN_ENTER;

	if (gpio_request(LCD_RST_N, "lcd_rst_n"))
		MSG_ERR("failed to request gpio lcd_rst_n\n");
    if (gpio_request(LCD_RST_N_113, "lcd_rst_n"))
        MSG_ERR("failed to request gpio lcd_rst_n_113\n");
	//gpio_configure(LCD_RST_N,GPIOF_DRIVE_OUTPUT);
	//MSG_DBG("%s LCD_RST_N = %d\r\n",__func__,gpio_get_value(LCD_RST_N));
 //      gpio_tlmm_config(GPIO_CFG(LCD_RST_N, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),GPIO_CFG_ENABLE);
       //gpio_tlmm_config(GPIO_CFG(LCD_RST_N_113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),GPIO_CFG_ENABLE);
/*	
	if (smem_cci_smem_value->cci_project_id == PROJECT_ID_K4H)
	{
		if ((smem_cci_smem_value->cci_hw_id == HW_ID_DVT2) ||
			  (smem_cci_smem_value->cci_hw_id > HW_ID_DVT2))
		{
				gpio_configure(LCD_RST_N_113,GPIOF_DRIVE_OUTPUT);
				MSG_DBG("%s LCD_RST_N_113 = %d\r\n",__func__,gpio_get_value(LCD_RST_N_113));
		}
	}
	*/
				//gpio_configure(LCD_RST_N_113,GPIOF_DRIVE_OUTPUT);
			//	MSG_DBG("%s LCD_RST_N_113 = %d\r\n",__func__,gpio_get_value(LCD_RST_N_113));
				
	//if (gpio_request(LCD_BLK_EN, "lcd_drv_en"))
	//	MSG_ERR("failed to request gpio lcd_drv_en\n");
	//gpio_configure(LCD_BLK_EN,GPIOF_DRIVE_OUTPUT);
	//  MSG_DBG("%s LCD_BLK_EN = %d\r\n",__func__,gpio_get_value(LCD_BLK_EN));
       // gpio_tlmm_config(GPIO_CFG(32, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (mddi_host_core_version > 8) {
		/* can use faster refresh with newer hw revisions */
		mddi_cci_ab60_debug_60hz_refresh = TRUE;

		/* Timing variables for tracking vsync */
		/* dot_clock = 6.00MHz
		 * horizontal count = 296
		 * vertical count = 338
		 * refresh rate = 6000000/(296+338) = 60Hz
		 */
		mddi_cci_ab60_rows_per_second = 20270;	/* 6000000/296 */
		mddi_cci_ab60_rows_per_refresh = 338;
		mddi_cci_ab60_usecs_per_refresh = 16674;	/* (296+338)/6000000 */
		MSG_DBG("##### %s mddi_host_core_version > 8 #####\n",__func__);
	} else {
		/* Timing variables for tracking vsync */
		/* dot_clock = 5.20MHz
		 * horizontal count = 376
		 * vertical count = 338
		 * refresh rate = 5200000/(376+338) = 41Hz
		 */
		mddi_cci_ab60_rows_per_second = 13830;	/* 5200000/376 */
		mddi_cci_ab60_rows_per_refresh = 338;
		mddi_cci_ab60_usecs_per_refresh = 24440;	/* (376+338)/5200000 */

	}
	ret = platform_driver_register(&this_mddi_driver);
	if (ret)
	{
		MSG_ERR("platform_driver_register return fail.\n");
		MSG_FUN_EXIT;
		return ret;
	}

	pinfo = &mddi_cci_ab60_panel_data.panel_info;
	pinfo->xres = 320;
	pinfo->yres = 480;
	pinfo->type = MDDI_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 16;
	pinfo->fb_num = 2;
	pinfo->clk_rate =    68000000;//81920000;
	pinfo->clk_min =     65000000;//80000000;
	pinfo->clk_max =    70000000;//100000000;
	pinfo->lcd.vsync_enable = TRUE;
	pinfo->lcd.refx100 =
		(mddi_cci_ab60_rows_per_second * 100) / mddi_cci_ab60_rows_per_refresh;
	pinfo->lcd.v_back_porch = 18;
	pinfo->lcd.v_front_porch = 18;
	pinfo->lcd.v_pulse_width = 0;
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = 0;//(1 * HZ);
	pinfo->bl_max = 255;
	pinfo->bl_min = 0;

	ret = platform_device_register(&this_mddi_device);
	if (ret)
		platform_driver_unregister(&this_mddi_driver);

	//led_classdev_register(NULL, &lcd_backlight);
	//mddi_early.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	//mddi_early.suspend = mddi_cci_ab60_early_suspend;
	//mddi_early.resume = mddi_cci_ab60_later_resume;
	//register_early_suspend(&mddi_early);

	MSG_FUN_EXIT;
	return ret;
}
#endif // CONFIG_FB_MSM_MDDI

static int __init mddi_lcdc_cci_init(void)
{
	int ret;
	smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ));
/*
	if(smem_cci_smem_value != 0)
	{
		switch(smem_cci_smem_value->cci_project_id)
		{
			case PROJECT_ID_K4H:
			case PROJECT_ID_CAP6:
			case PROJECT_ID_CAP2:
#ifdef CONFIG_FB_MSM_MDDI
					ret = mddi_cci_ab60_init();
					printk(KERN_INFO "[John] mddi_lcdc_cci_init: MDDI ret=%d\r\n", ret);
#endif // CONFIG_FB_MSM_MDDI
					break;
			case PROJECT_ID_K4:
			case PROJECT_ID_CAP8:
			case PROJECT_ID_K5:
			default:
#ifdef CONFIG_FB_MSM_LCDC
					ret = lcdc_toshiba_panel_init();
					printk(KERN_INFO "[John] mddi_lcdc_cci_init: LCDC ret=%d\r\n", ret);
#endif // CONFIG_FB_MSM_LCDC
					break;
		}
	}
	else
	{
#ifdef CONFIG_FB_MSM_LCDC
			ret = lcdc_toshiba_panel_init();
			printk(KERN_INFO "[John] mddi_lcdc_cci_init: LCDC default.\r\n");
#endif // CONFIG_FB_MSM_LCDC
	}
*/
  ret = mddi_cci_ab60_init();
	printk(KERN_INFO "[John] mddi_lcdc_cci_init: MDDI test1 ret=%d\r\n", ret);
	return ret;
}

module_init(mddi_lcdc_cci_init);
