/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>
#include "../../arch/arm/mach-msm/acpuclock.h"
#include <linux/rtc.h>

#include "../../arch/arm/mach-msm/smd_private.h"
#include <mach/rpc_server_handset.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <mach/mpp.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/delay.h>

#include "cci_battery.h"
#include "linux/read_hwid.h"

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VERS	0x00010003

#define BATTERY_RPC_CB_PROG	0x31000089
#define BATTERY_RPC_CB_VERS	0x00010001

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VERS	    0x00010003

#define BATTERY_REGISTER_PROC                                  2
#define BATTERY_GET_CLIENT_INFO_PROC                   	3
#define BATTERY_MODIFY_CLIENT_PROC                     	4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_SERVICE_TABLES_PROC                    	6
#define BATTERY_IS_SERVICING_TABLES_ENABLED_PROC       	7
#define BATTERY_ENABLE_TABLE_SERVICING_PROC            	8
#define BATTERY_DISABLE_TABLE_SERVICING_PROC           	9
#define BATTERY_READ_PROC                              	10
#define BATTERY_MIMIC_LEGACY_VBATT_READ_PROC           	11
#define BATTERY_ENABLE_DISABLE_FILTER_PROC 		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC 		1
#define BATTERY_CB_ID_ALL_ACTIV       	1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW             3400
#define BATTERY_HIGH            4200

#define TRUE 1
#define FALSE 0 



//RPC index
#define CCIPROG	0x30001000
#define CCIVERS	0x00010001
//end
#define ONCRPC_CHG_IS_CHARGING_PROC 		      2
#define ONCRPC_CHG_IS_CHARGER_VALID_PROC 	3
#define ONCRPC_CHG_IS_BATTERY_VALID_PROC 	4
#define ONCRPC_CHG_UI_EVENT_READ_PROC 		5
#define ONCRPC_CHG_USB_CHARGER_CONNECTED_PROC 7
#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC 	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC 	0xffffffff
#define ONCRPC_CHG_GET_TEMP_PROC            30
#define ONCPRC_CCI_READ_TX_POWER_PROC       34
#define ONCRPC_CCI_GET_CHARGING_UAH         38
#define ONCRPC_CCI_GET_AMSS_CHG_STATE  	    40

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))


#define CHARGER_API_VERSION  			0x00010003
#define DEFAULT_CHARGER_API_VERSION		0x00010001


#define BATT_RPC_TIMEOUT    10000	/* 10 sec */

#define INVALID_BATT_HANDLE    -1


#define SUSPEND_EVENT		(1UL << 0)
#define RESUME_EVENT		(1UL << 1)
#define CLEANUP_EVENT		(1UL << 2)


enum
{
    BATTERY_REGISTRATION_SUCCESSFUL = 0,
    BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
    BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
    BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
    BATTERY_CLIENT_TABLE_FULL = 1,
    BATTERY_REG_PARAMS_WRONG = 2,
    BATTERY_DEREGISTRATION_FAILED = 4,
    BATTERY_MODIFICATION_FAILED = 8,
    BATTERY_INTERROGATION_FAILED = 16,
    /* Client's filter could not be set because perhaps it does not exist */
    BATTERY_SET_FILTER_FAILED         = 32,
    /* Client's could not be found for enabling or disabling the individual client */
    BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
    BATTERY_LAST_ERROR = 128,
};

enum
{
    BATTERY_VOLTAGE_UP = 0,
    BATTERY_VOLTAGE_DOWN,
    BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
    BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
    BATTERY_VOLTAGE_LEVEL,
    BATTERY_ALL_ACTIVITY,
    VBATT_CHG_EVENTS,
    BATTERY_VOLTAGE_UNKNOWN,
};



enum
{
    CHG_UI_EVENT__IDLE,	/* Starting point, no charger.  */
    CHG_UI_EVENT__NO_POWER,	/* No/Weak Battery + Weak Charger. */
    CHG_UI_EVENT__VERY_LOW_POWER,	/* No/Weak Battery + Strong Charger. */
    CHG_UI_EVENT__LOW_POWER,	/* Low Battery + Strog Charger.  */
    CHG_UI_EVENT__NORMAL_POWER, /* Enough Power for most applications. */
    CHG_UI_EVENT__DONE,	/* Done charging, batt full.  */
    CHG_UI_EVENT__BAD_TEMP, /*TEMP is too hot/cold*/
    CHG_UI_EVENT__TIMEOUT,/*state timeout*/
    CHG_UI_EVENT__INVALID,
    CHG_UI_EVENT__MAX32 = 0x7fffffff
};

enum
{
    PM_APP_OTG_A_DEV_TYPE__USB_HOST,       // Standard USB host like a PC
    PM_APP_OTG_A_DEV_TYPE__USB_CARKIT,     // USB carkit as specified in CEA-936-A
    PM_APP_OTG_A_DEV_TYPE__USB_CHARGER,    // USB charger as specified in CEA-936-A
    PM_APP_OTG_A_DEV_TYPE__INVALID
};

enum chg_state_type
{
    CHG_NULL = 0,
    CHG_AC = 1,
    CHG_USB = 2
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type
{
    /* The charger is good      */
    CHARGER_STATUS_GOOD,
    /* The charger is bad       */
    CHARGER_STATUS_BAD,
    /* The charger is weak      */
    CHARGER_STATUS_WEAK,
    /* Invalid charger status.  */
    CHARGER_STATUS_INVALID
};

static char *charger_status[] =
{
    "good", "bad", "weak", "invalid"
};


/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type
{
    /* The charger is removed                 */
    CHARGER_TYPE_NONE,
    /* The charger is a regular wall charger   */
    CHARGER_TYPE_WALL,
    /* The charger is a PC USB                 */
    CHARGER_TYPE_USB_PC,
    /* The charger is a wall USB charger       */
    CHARGER_TYPE_USB_WALL,
    /* The charger is a USB carkit             */
    CHARGER_TYPE_USB_CARKIT,
    /* Invalid charger hardware status.        */
    CHARGER_TYPE_INVALID
};

static char *charger_type[] =
{
    "No charger", "wall", "USB PC", "USB wall", "USB car kit",
    "invalid charger"
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type
{
    /* The battery is good        */
    BATTERY_STATUS_GOOD,
    /* The battery is cold/hot    */
    BATTERY_STATUS_BAD_TEMP,
    /* The battery is bad         */
    BATTERY_STATUS_BAD,
    /* Invalid battery status.    */
    BATTERY_STATUS_INVALID
};

static char *battery_status[] =
{
    "good ", "bad temperature", "bad", "invalid"
};


/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type
{
    /* The battery voltage is dead/very low (less than 3.2V)        */
    BATTERY_LEVEL_DEAD,
    /* The battery voltage is weak/low (between 3.2V and 3.4V)      */
    BATTERY_LEVEL_WEAK,
    /* The battery voltage is good/normal(between 3.4V and 4.2V)  */
    BATTERY_LEVEL_GOOD,
    /* The battery voltage is up to full (close to 4.2V)            */
    BATTERY_LEVEL_FULL,
    /* Invalid battery voltage level.                               */
    BATTERY_LEVEL_INVALID
};

static char *battery_level[] =
{
    "dead", "weak", "good", "full", "invalid"
};


/* Generic type definition used to enable/disable charger functions */
enum
{
    CHG_CMD_DISABLE,
    CHG_CMD_ENABLE,
    CHG_CMD_INVALID,
    CHG_CMD_MAX32 = 0x7fffffff
};

enum CALL_STATE_PROC
{
    PHONE_IDLE,
    PHONE_RINGING,
    PHONE_OFFHOOK,
    ENABLE_BATT_DBG_INFO,    //for debug
    ENABLE_CHARGING_FUNC,
    DISABLE_CHARGING_FUNC,
    BATTERY_TEST_100 = 6,
    BATTERY_TEST_15 = 7,
    BATTERY_TEST_10 = 8,
    BATTERY_TEST_5 = 9,
};

enum CHARGING_STATE_FLAG
{
    CHG_NONE,
    CHG_CHARGING,
    CHG_COMPLETE,
    CHG_OT
};


struct rpc_reply_batt_chg
{
    struct rpc_reply_hdr hdr;
    u32 	more_data;

    u32	charger_status;
    u32	charger_type;
    u32	battery_status;
    u32	battery_level;
    u32 battery_voltage;
    u32	battery_temp;
    u32	battery_id;
};

static struct rpc_reply_batt_chg rep_batt_chg;

struct msm_battery_info
{
    u32 voltage_max_design;
    u32 voltage_min_design;
    u32 chg_api_version;
    u32 batt_technology;

    u32 avail_chg_sources;
    u32 current_chg_source;

    u32 batt_status;
    u32 batt_health;
    u32 voltage_now;
    u32 charger_valid;
    u32 batt_valid;
    s32 batt_capacity;

    s32 charger_status;
    s32 charger_type;
    s32 battery_status;
    s32 battery_level;
    s32 battery_voltage;
    s32 battery_temp;
    s32 batt_temp;

    u32(*calculate_capacity) (u32 voltage);
    s32 batt_handle;
    spinlock_t lock;

    struct power_supply *msm_psy_ac;
    struct power_supply *msm_psy_usb;
    struct power_supply *msm_psy_batt;

    struct msm_rpc_endpoint *batt_ep;
    struct msm_rpc_endpoint *chg_ep;
    struct msm_rpc_endpoint *cci_ep;

    struct workqueue_struct *msm_batt_wq;
    struct delayed_work dwork;

    struct wake_lock wlock;
    struct early_suspend early_suspend;

	atomic_t initflag;
    atomic_t initCharger;

};

enum temperature
{
    TEMPERATURE_0 = 0,
    TEMPERATURE_25,
    TEMPERATURE_45,
};

enum table_mapping
{
    down_table_volt_100 = 0,
    down_table_volt_200,
    down_table_volt_300,
    down_table_volt_400,
    down_table_volt_500,
    down_table_volt_600,
    down_table_volt_700,
    MAX_DISCHARGE_TABLE = 7,    
    up_table_volt_ac_800,                 //Table_ac_charger_cc_mode
    up_table_volt_ac_400,                 //Table_ac_charger_cv_mode
    up_table_volt_usb_new,              //Table_usb_charger_cc_mode
    up_table_volt_usb_new1,            //Table_usb_charger_cv_mode
    MAX_CHARGE_TABLE
};

enum smooth_table
{
    Smooth_Adj_0 = 0,
    Smooth_Adj_1,
    Smooth_Adj_2,
    Smooth_Adj_3,
    Smooth_Adj_4,
    Smooth_Adj_5,
};

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_BATTERY_DEBUG_NORMAL = 1U << 0,
    MSM_BATTERY_DEBUG_INFO = 1U << 1,
};

static int msm_battery_debug_mask = 0x02;
module_param_named(
	debug_mask, msm_battery_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MSM_BATTERY_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & msm_battery_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)


/* Get project id and hardware id */
static int cci_hardware_id;
static int cci_project_id;
const int table_size_i = 3;
const int table_size_j = 12;
const int max_table_items = 41;
const int min_table_items = 20;
static int *project_final_table[3][12];
static int mA_table[41];

// Battery infor
static int MAX_CAP_MA;
static int cci_base_current;
static int BootCount = 10;

static unsigned int cci_batt_device_mask = 0;
static unsigned int cci_cal_current;

/* For others driver current mapping */
static unsigned int cci_batt_device_current[] = {
    400, //CCI_BATT_DEVICE_ON_RF_Max_650
    350, //CCI_BATT_DEVICE_ON_RF_190_550
    300, //CCI_BATT_DEVICE_ON_RF_130_350
    250, //CCI_BATT_DEVICE_ON_RF_90_200
    150, //CCI_BATT_DEVICE_ON_RF_100
    0,   //CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_50
    50,  //CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_100
    100, //CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_150
    100, //CCI_BATT_DEVICE_ON_CAMERA_150
    50,  //CCI_BATT_DEVICE_ON_WIFI_50
    50   //CCI_BATT_DEVICE_ON_VIDEO_50
};

static atomic_t  g_ChgVld = ATOMIC_INIT(NO_CHG);  //Getting charging info from usb
static atomic_t  g_IsSuspend = ATOMIC_INIT(false);
static atomic_t  g_IsOnCall = ATOMIC_INIT(false);


static int *previous_capacity_down_table= NULL;
/* Get datat from share memory */
static int charger_state = 0;
static int charger_flag = CHG_NONE;
static int call_state = 0;
static int Ichg = 0;
static int Volt_modem = 0;
u8 first_boot = false;

#ifdef BATT_FOR_TEST_OT
//[TEST]
static int counter = 0;
//[TEST]
#endif

struct timespec pre_ts;

s64 T_Capacity_uAH = -12345678;   //Theo_capacity (uA)
struct delayed_work g_dwork;
u8   g_Shotdown_Count = 1;
u16  g_Chg_UI_event = CHG_UI_EVENT__INVALID;
s32  g_Resume_Capacity = 0;
u8   g_Suspend_Chg_Source = NO_CHG;        //Get chg_source before suspend
u8   g_Is_Run_suspend = false;
// [BSP] Move to USB at dormancy estate find can't unlock
u8 is_update_v3 = false;

atomic_t is_updating = ATOMIC_INIT(false);

static struct msm_battery_info msm_batt_info =
{
    .batt_handle = INVALID_BATT_HANDLE,
    .charger_status = -1,
    .charger_type = -1,
    .battery_status = -1,
    .battery_level = -1,
    .battery_voltage = -1,
    .battery_temp = -1,
};

static enum power_supply_property msm_power_props[] =
{
    POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] =
{
    "battery",
};

static void msm_batt_update_psy_status_v0(void);
static void msm_batt_update_psy_status_v2(void);
static void msm_batt_update_psy_status_v3(void);
static bool small_charging_current(int current_volt);


static u32 msm_batt_capacity(u32 current_voltage);
static void msm_batt_wait_for_batt_chg_event(struct work_struct *work);
static void msm_batt_early_suspend(struct early_suspend *h);
static int msm_batt_cleanup(void);
static DECLARE_WORK(msm_batt_cb_work, msm_batt_wait_for_batt_chg_event);

extern int smd_is_closed;

/**
 * Check is USB connected to PC.
 */
int IsUSBPlugged(void)
{
       return !!(msm_batt_info.current_chg_source & USB_CHG);
}

EXPORT_SYMBOL(IsUSBPlugged);

void Update_Power_Status(void)
{
    if (msm_batt_info.msm_psy_batt && msm_batt_info.msm_psy_ac && msm_batt_info.msm_psy_usb )
    {
        power_supply_changed(msm_batt_info.msm_psy_batt);
        power_supply_changed(msm_batt_info.msm_psy_ac);
        power_supply_changed(msm_batt_info.msm_psy_usb);
    }
}

void UI_To_UpperLayer(u32 chg_batt_event)
{
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO, "[Battery]: %s()\n",__func__);
    
    switch (chg_batt_event)
    {
        case CHG_UI_EVENT__VERY_LOW_POWER:
        case CHG_UI_EVENT__LOW_POWER:
        case CHG_UI_EVENT__NORMAL_POWER:
        {
            /* Check charging type */
            int type = atomic_read(&g_ChgVld);
            if (type > NO_CHG)
            {
                charger_flag = CHG_CHARGING;
                msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
                msm_batt_info.current_chg_source = type;
            }
            else
            {
                charger_flag = CHG_NONE;
                msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                msm_batt_info.current_chg_source = NO_CHG;
            }
        }
        break;


        case CHG_UI_EVENT__DONE:
            charger_flag = CHG_COMPLETE;
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;

            break;

        case CHG_UI_EVENT__BAD_TEMP:
            charger_flag = CHG_OT;
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            msm_batt_info.current_chg_source = NO_CHG;
            break;

        case CHG_UI_EVENT__TIMEOUT:
        case CHG_UI_EVENT__IDLE:
        case CHG_UI_EVENT__NO_POWER:
        default:
            if (msm_batt_info.charger_valid == FALSE)
            {
                charger_flag = CHG_NONE;
                msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                msm_batt_info.current_chg_source = NO_CHG;
            }
            break;

    }
    /* Update charging type */
    if (g_Chg_UI_event != chg_batt_event)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Change batt event %d ==> %d, batt_status:%d\n",
               g_Chg_UI_event, chg_batt_event, msm_batt_info.batt_status);

        power_supply_changed(msm_batt_info.msm_psy_ac);
        power_supply_changed(msm_batt_info.msm_psy_usb);

        g_Chg_UI_event = chg_batt_event;
    }

}

/**
 * Estimate TX power and others driver current
 */
void Estimate_Current(s32 power_level_dbm)
{
    int i;
    int RF_power = 0;

    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO, "[Battery]: %s()\n",__func__);

    //RF power level
    cci_batt_device_mask &= ~(0x1F);
    if (call_state == PHONE_OFFHOOK)
    {
        if (power_level_dbm >= 24)
        {
            RF_power = 500;
        }
        else if (power_level_dbm >= 23)
        {
            RF_power = 450;
        }

        else if (power_level_dbm >= 21)
        {
            RF_power = 400;
        }
        else if ( power_level_dbm >= 16)
        {
            RF_power = 350;
        }
        else if ( power_level_dbm >= 13)
        {
            RF_power = 300;
        }
        else if ( power_level_dbm >= 10)
        {
            RF_power = 250;
        } 
        else if ( power_level_dbm >= 6)
        {
            RF_power = 200;
        }
        else
        {
            RF_power = 150;
        }
    }

    cci_cal_current = cci_base_current; //reset to base

    /* Estimate total current, inculde Tx power and other drivers */
    for (i = 0; i < ARRAY_SIZE(cci_batt_device_current); i++)
    {
        if (cci_batt_device_mask & (1 << i))
        {
            cci_cal_current += cci_batt_device_current[i];
        }
    }
    cci_cal_current = cci_cal_current + RF_power;

}

/**
 * Export to other drivers to update (Camera, LCM,Video)
 *
 */
void cci_batt_device_status_update(int device, int status)
{
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO, "[Battery]: %s()/n",__func__);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,        
            "[Battery]: device = %d , status = %d\n",device,status);
    
    if (status == 1)
        cci_batt_device_mask |= device;
    else
        cci_batt_device_mask &= ~device;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: %s(), device:0x%x, cci_batt_device_mask:0x%x \n",
            __func__, device, cci_batt_device_mask);

}

/**
 * Called from USB driver to update current charging status
 *
 */
void cci_android_charger_usb_change(int plugin, int type)
{
    int rc;
    struct rpc_req_batt_chg
    {
        struct rpc_request_hdr hdr;
        u32 more_data;
    } req_batt_chg;


    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s, plugin:%d, type:%d \n",__func__,plugin, type);

	if(atomic_read(&msm_batt_info.initflag)!= 1){

        atomic_set(&msm_batt_info.initCharger, type);

	    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s, Driver NOT ready \n",__func__);
		return;
	}

    req_batt_chg.more_data = cpu_to_be32(PM_APP_OTG_A_DEV_TYPE__INVALID);

//Bryan
//    cancel_delayed_work_sync(&msm_batt_info.dwork);
//    schedule_delayed_work(&msm_batt_info.dwork, msecs_to_jiffies(10*1000));

    if (plugin == TRUE)
    {
        atomic_set(&g_ChgVld, type);

        if (type == CHG_AC)
            req_batt_chg.more_data = cpu_to_be32(PM_APP_OTG_A_DEV_TYPE__USB_CHARGER);
        else if(type == CHG_USB)
            req_batt_chg.more_data = cpu_to_be32(PM_APP_OTG_A_DEV_TYPE__USB_HOST);

#ifdef BATT_FOR_TEST_OT
//[TEST]
        counter = 1;
//[TEST]
#endif

        if (msm_batt_info.batt_capacity < 100)
        {
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
        }
        else if (msm_batt_info.batt_capacity == 100)
        {
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
        }

        //charger type 1=AC, 2=USB
        msm_batt_info.current_chg_source = type;
    }
    else
    {
        atomic_set(&g_ChgVld, NO_CHG);

        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        msm_batt_info.current_chg_source = NO_CHG;
    }

    /* Update Charging state to modem */
    rc = msm_rpc_call(msm_batt_info.chg_ep,
            ONCRPC_CHG_USB_CHARGER_CONNECTED_PROC,
            &req_batt_chg, sizeof(req_batt_chg),
            msecs_to_jiffies(BATT_RPC_TIMEOUT));

    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
                "[Battery]: %s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
                __func__,ONCRPC_CHG_USB_CHARGER_CONNECTED_PROC, rc);
        return;
    }


//[BSP] Move to USB at dormancy estate find can't unlock
    cancel_delayed_work_sync(&msm_batt_info.dwork);
    if (atomic_read(&g_IsSuspend) == false)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "%s suspend = false",__func__);

        schedule_delayed_work(&msm_batt_info.dwork, msecs_to_jiffies(10*1000));

        if (type == AC_CHG)
            power_supply_changed(msm_batt_info.msm_psy_ac);
        else if (type == USB_CHG)
            power_supply_changed(msm_batt_info.msm_psy_usb);
        else
            power_supply_changed(msm_batt_info.msm_psy_batt);

        //Update_Power_Status();    //Delay updating power status at late resume
    }
    else
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "%s suspend = true",__func__);

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "%s ",__func__);
	if (atomic_read(&is_updating)==false)
	{        
        msm_batt_update_psy_status_v3();  //Need to run after batt_resume()
			is_update_v3=true;
	}else
			is_update_v3=false;
 
        if (type == AC_CHG)
            power_supply_changed(msm_batt_info.msm_psy_ac);
        else if (type == USB_CHG)
            power_supply_changed(msm_batt_info.msm_psy_usb);
        else
            power_supply_changed(msm_batt_info.msm_psy_batt);

        //is_update_v3 = true;
        schedule_delayed_work(&msm_batt_info.dwork, msecs_to_jiffies(30*1000));
    }
//[BSP] Move to USB at dormancy estate find can't unlock

}


static int msm_power_get_property(struct power_supply *psy,
                                  enum power_supply_property psp,
                                  union power_supply_propval *val)
{
    if (call_state == ENABLE_BATT_DBG_INFO)
        //DBG("[Battery]: %s(), psp:%d\n", __func__,psp);
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: %s(), psp:%d\n", __func__,psp);
    
    switch (psp)
    {
        case POWER_SUPPLY_PROP_ONLINE:

            if (psy->type == POWER_SUPPLY_TYPE_MAINS)
            {
                val->intval = msm_batt_info.current_chg_source & AC_CHG
                              ? 1 : 0;
                if (call_state == ENABLE_BATT_DBG_INFO)
                    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
                        "[Battery]: %s(): power supply = %s online = %d\n"
                            , __func__, psy->name, val->intval);
            }

            if (psy->type == POWER_SUPPLY_TYPE_USB)
            {
                val->intval = msm_batt_info.current_chg_source & USB_CHG
                              ? 1 : 0;
                if (call_state == ENABLE_BATT_DBG_INFO)
                    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
                        "[Battery]: %s(): power supply = %s online = %d\n"
                            , __func__, psy->name, val->intval);
            }

            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static struct power_supply msm_psy_ac =
{
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .supplied_to = msm_power_supplied_to,
    .num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
    .properties = msm_power_props,
    .num_properties = ARRAY_SIZE(msm_power_props),
    .get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb =
{
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
    .supplied_to = msm_power_supplied_to,
    .num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
    .properties = msm_power_props,
    .num_properties = ARRAY_SIZE(msm_power_props),
    .get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] =
{
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
};


static void msm_batt_external_power_changed(struct power_supply *psy)
{
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "%s() : external power supply changed for %s\n",
           __func__, psy->name);
    power_supply_changed(psy);
}

static s32 capacity_algo(s32 capacity)
{
    int rtn = 50;

    if(capacity >= 100 ) //5% range, 100% - 70%
        rtn = 100;
    else if (capacity >= 99)
        rtn = 99; 
    else if (capacity >= 95)
        rtn = 95;
    else if (capacity >= 90)
        rtn = 90;
    else if (capacity >= 85)
        rtn = 85;
    else if (capacity >= 80)
        rtn = 80;
    else if (capacity >= 75)
        rtn = 75;
    else if (capacity >= 70)//10% range, 70% - 20%
        rtn = 70;
    else if (capacity >= 60)
        rtn = 60;
    else if (capacity >= 50)
        rtn = 50;
    else if (capacity >= 40)
        rtn = 40;
    else if (capacity >= 30)
        rtn = 30;
    else if (capacity >= 20)
        rtn = 20;
    else if (capacity >= 15) //5% ragne, 20% - 0%
        rtn = 15;
    else if (capacity >= 10)
        rtn = 10;
    else if (capacity >= 5)
        rtn = 5;
    else if (capacity >= 1)
        rtn = 1;
    else if (capacity == 0)
        rtn = 0;

    return rtn;
}
static int msm_batt_power_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val)
{

    //DBG("[Battery]: %s(), psp:%d\n", __func__,psp);
    switch (psp)
    {
        case POWER_SUPPLY_PROP_STATUS: //0
            val->intval = msm_batt_info.batt_status;
            break;
        case POWER_SUPPLY_PROP_HEALTH: //1
            val->intval = msm_batt_info.batt_health;
            break;
        case POWER_SUPPLY_PROP_PRESENT: //2
            val->intval = msm_batt_info.batt_valid;
            break;
        case POWER_SUPPLY_PROP_TECHNOLOGY: //3
            val->intval = msm_batt_info.batt_technology;
            break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW: //4
         {   
            val->intval = msm_batt_info.voltage_now * 1000;
	    vbatt=msm_batt_info.voltage_now;		
            break;
          }
        case POWER_SUPPLY_PROP_CAPACITY: //5
         {   
            val->intval =  capacity_algo(msm_batt_info.batt_capacity);
	    batt_capacity=capacity_algo(msm_batt_info.batt_capacity);
            break;
          }
        case POWER_SUPPLY_PROP_TEMP: //6
            val->intval = msm_batt_info.batt_temp * 10;
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static struct power_supply msm_psy_batt =
{
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = msm_batt_power_props,
    .num_properties = ARRAY_SIZE(msm_batt_power_props),
    .get_property = msm_batt_power_get_property,
    .external_power_changed = msm_batt_external_power_changed,
};


static int msm_batt_get_batt_chg_status_v1(void)
{
    int rc ;
    struct rpc_req_batt_chg
    {
        struct rpc_request_hdr hdr;
        u32 more_data;
    } req_batt_chg;

    req_batt_chg.more_data = cpu_to_be32(1);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

    rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
                            ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_batt_chg, sizeof(rep_batt_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
        return rc;
    }
    else if (be32_to_cpu(rep_batt_chg.more_data))
    {

        rep_batt_chg.charger_status =
            be32_to_cpu(rep_batt_chg.charger_status);

        rep_batt_chg.charger_type =
            be32_to_cpu(rep_batt_chg.charger_type);

        rep_batt_chg.battery_status =
            be32_to_cpu(rep_batt_chg.battery_status);

        rep_batt_chg.battery_level =
            be32_to_cpu(rep_batt_chg.battery_level);

        rep_batt_chg.battery_voltage =
            be32_to_cpu(rep_batt_chg.battery_voltage);

        rep_batt_chg.battery_temp =
            be32_to_cpu(rep_batt_chg.battery_temp);

        rep_batt_chg.battery_id =
            be32_to_cpu(rep_batt_chg.battery_id);

           
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "charger_status = %s, charger_type = %s,"
               " batt_status = %s, batt_level = %s,"
               " batt_volt = %u, batt_temp = %u, batt_id = %u,\n",
               charger_status[rep_batt_chg.charger_status],
               charger_type[rep_batt_chg.charger_type],
               battery_status[rep_batt_chg.battery_status],
               battery_level[rep_batt_chg.battery_level],
               rep_batt_chg.battery_voltage,
               rep_batt_chg.battery_temp,
               rep_batt_chg.battery_id);

    }
    else
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "%s():No more data in batt_chg rpc reply\n",
               __func__);
        return -EIO;
    }

    return 0;
}

static int Get_charging_uah(s64* val)
{

    int rc;
    s64 value;
    

    struct rpc_request_hdr req_batt_chg;

    struct rpc_reply_chg_reply
    {
        struct rpc_reply_hdr hdr;
        s64 chg_batt_data;
    } rep_chg;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    rc = msm_rpc_call_reply(msm_batt_info.cci_ep,
                            ONCRPC_CCI_GET_CHARGING_UAH,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "[Battery]: %s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
            __func__, ONCRPC_CCI_GET_CHARGING_UAH, rc);
        return rc;
    }

    value = be64_to_cpu(rep_chg.chg_batt_data);
    *val = value;
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: Get_charging_uah, value:0x%x%x\n", (s32)(value >> 32), (s32)(value & 0xFFFFFFFF));

    return 0;


}

static int msm_batt_get_batt_rpc(
    u32 *batt_volt,
    u32 *batt_charging,
    u32 *charger_valid,
    u32 *chg_batt_event,
    s32 *chg_batt_temp,
    s32 *power_level_dbm,
    s32 *amss_chg_state
)
{
    struct rpc_request_hdr req_batt_chg;

    struct power_level_req
    {
        struct rpc_request_hdr hdr;
        u32 option;

    } dBm_req;

    struct rpc_reply_batt_volt
    {
        struct rpc_reply_hdr hdr;
        u32 voltage;
    } rep_volt;

    struct rpc_reply_chg_reply
    {
        struct rpc_reply_hdr hdr;
        s32 chg_batt_data;
    } rep_chg;

    int rc;
    u32 option_dbm = 0;
    cci_smem_value_t *smem_cci_smem_value;

    *batt_charging = 0;
    *chg_batt_event = CHG_UI_EVENT__INVALID;
    *charger_valid = 0;
 
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s()\n", __func__);
   
    // read Ichg(current) form share memory
    smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ) );
    charger_state = smem_cci_smem_value->cci_set_chg_state;
    smem_cci_smem_value->cci_chg_percentage = 10;
    Ichg = (int)smem_cci_smem_value->cci_chg_i;
    Volt_modem = (int)smem_cci_smem_value->cci_factory_fast_charge_value;	

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: From smem: charger_state = %d , Ichg = %d\n",charger_state,Ichg);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: From smem: cci_battery_info  = %d \n",smem_cci_smem_value->cci_battery_info);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: From smem: cci_modem_charging_state  = %d \n",smem_cci_smem_value->cci_modem_charging_state);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: From smem: cci_modem_charging_state  = %d \n",smem_cci_smem_value->cci_chg_percentage);


    dBm_req.option = cpu_to_be32(option_dbm);

//--  RPC for BATTERY_READ_PROC --       //ONCRPC_PM_VBATT_READ_PROC
    rc = msm_rpc_call_reply(msm_batt_info.batt_ep,
                            BATTERY_READ_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_volt, sizeof(rep_volt),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: BATTERY_READ_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, BATTERY_READ_PROC, rc);

        return rc;
    }
    *batt_volt = be32_to_cpu(rep_volt.voltage);
     vbatt= *batt_volt;
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: batt_volt = %d \n", *batt_volt);
//-- END --


//--  RPC for ONCRPC_CHG_IS_CHARGING_PROC --
    rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
                            ONCRPC_CHG_IS_CHARGING_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: ONCRPC_CHG_IS_CHARGING_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_IS_CHARGING_PROC, rc);
        return rc;
    }
    *batt_charging = be32_to_cpu(rep_chg.chg_batt_data);
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: End RPC ONCRPC_CHG_IS_CHARGING_PROC,!!! batt_charging %d\n", *batt_charging);
//-- END --

//--  RPC for ONCRPC_CHG_IS_CHARGER_VALID_PROC --      return chg_is_charger_valid_flag
    rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
                            ONCRPC_CHG_IS_CHARGER_VALID_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s:ONCRPC_CHG_IS_CHARGER_VALID_PROC, msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_IS_CHARGER_VALID_PROC, rc);
        return rc;
    }
    *charger_valid = be32_to_cpu(rep_chg.chg_batt_data);
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: End RPC ONCRPC_CHG_IS_CHARGER_VALID_PROC,!! carger_valid %d\n", *charger_valid);
//-- END --

//--  RPC for ONCRPC_CHG_IS_BATTERY_VALID_PROC --
    rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
                            ONCRPC_CHG_IS_BATTERY_VALID_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {   
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: ONCRPC_CHG_IS_BATTERY_VALID_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_IS_BATTERY_VALID_PROC, rc);
        return rc;
    }
    msm_batt_info.batt_valid = be32_to_cpu(rep_chg.chg_batt_data);
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: End RPC ONCRPC_CHG_IS_BATTERY_VALID_PROC,!!! msm_batt_info.batt_valid = %d\n", msm_batt_info.batt_valid);
//-- END --

//--  RPC for ONCRPC_CHG_UI_EVENT_READ_PROC --  get modem's event
    rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
                            ONCRPC_CHG_UI_EVENT_READ_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: ONCRPC_CHG_UI_EVENT_READ_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_UI_EVENT_READ_PROC, rc);
        return rc;
    }

    //for test mode
    if (call_state == ENABLE_CHARGING_FUNC && !(*batt_charging))
        *chg_batt_event = CHG_UI_EVENT__NORMAL_POWER;
    else if (call_state == DISABLE_CHARGING_FUNC)
        *chg_batt_event = CHG_UI_EVENT__IDLE;
    else
        *chg_batt_event = be32_to_cpu(rep_chg.chg_batt_data);
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: End RPC ONCRPC_CHG_UI_EVENT_READ_PROC,!!! chg_batt_event %d \n",*chg_batt_event);
    
//-- END --

//--  RPC for ONCRPC_CHG_GET_TEMP_PROC -- battery temperature
#if  1
    rc = msm_rpc_call_reply(msm_batt_info.cci_ep,
                            ONCRPC_CHG_GET_TEMP_PROC,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: ONCRPC_CHG_GET_TEMP_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CHG_GET_TEMP_PROC, rc);
        return rc;
    }

    *chg_batt_temp = be32_to_cpu(rep_chg.chg_batt_data);
#else
    *chg_batt_temp = 25;
#endif
    // for modem temperature bug
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: End RPC ONCRPC_CHG_GET_TEMP_PROC,!!! chg_batt_temp %d \n",*chg_batt_temp);
//-- END --

//--  RPC for ONCPRC_CCI_READ_TX_POWER_PROC -- power level  (Change charging table when talking)
#if 1
    rc = msm_rpc_call_reply(msm_batt_info.cci_ep,
                            ONCPRC_CCI_READ_TX_POWER_PROC,
                            &dBm_req, sizeof(dBm_req),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: ONCPRC_CCI_READ_TX_POWER_PROC,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCPRC_CCI_READ_TX_POWER_PROC, rc);
        //return rc;
    }
    *power_level_dbm = be32_to_cpu(rep_chg.chg_batt_data);
#else    
    *power_level_dbm = 100;
#endif     
    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: power_level_dbm = %d\n",*power_level_dbm);
//-- END --

//--  RPC for ONCRPC_CCI_GET_AMSS_CHG_STATE -- not used
#if 0
    rc = msm_rpc_call_reply(msm_batt_info.cci_ep,
                            ONCRPC_CCI_GET_AMSS_CHG_STATE,
                            &req_batt_chg, sizeof(req_batt_chg),
                            &rep_chg, sizeof(rep_chg),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        printk(KERN_ERR "%s: ONCRPC_CCI_GET_AMSS_CHG_STATE,msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, ONCRPC_CCI_GET_AMSS_CHG_STATE, rc);
        return rc;
    }

    *amss_chg_state = be32_to_cpu(rep_chg.chg_batt_data);
#else    
    *amss_chg_state = 0;
#endif    

#ifdef BATT_FOR_TEST_OT
//[TEST]
    if (counter > 0)
    {
        if (counter > 2 && counter < 6)
            *chg_batt_event = CHG_UI_EVENT__BAD_TEMP;
        else
        {
            *chg_batt_event = CHG_UI_EVENT__NORMAL_POWER;
        }

        if (counter > 7)
            counter = 1;
        else
            counter++;
    }
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: counter:%d\n\n\n",counter);
//[TEST]
#endif
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s() Ending !!!!!\n",__func__);
    return 0;
}

//Battery Profile
//#define MAX_CAP_MA     1300
#define CONV_PERCENT_TO_uAH(p)    ((p * MAX_CAP_MA * 1000) / 100)
#define THEO_uAS(mA,sec)  ((mA) * 1000 * (sec))
#define MAX_uAH   CONV_PERCENT_TO_uAH(100)
#define Val_99_uAH    CONV_PERCENT_TO_uAH(99)
/**
 * Smooth Algo.
 *
 */
static void Smooth_Cap(int *ACap, int *CCap, int Chg_Type,bool Is_small_charging_current) 
{
    s64 Local_Adc_uAH = CONV_PERCENT_TO_uAH(*ACap);
    s64 Delta_uAH = Local_Adc_uAH - T_Capacity_uAH;
    s64 THEO_uAH = 0;
    s64 rtn = 0;
    short Local_Ichg = 0;
    short Adj_level = 0;
    short Smooth_Adj = Smooth_Adj_4;

    if (ACap == NULL || CCap == NULL)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "[Battery]: ACap or CCap is NULL\n");
        return;
    }

    //Boundary check for Ichg
    Local_Ichg = Ichg - (cci_cal_current);

    if (Local_Ichg > 1000)
        Local_Ichg = 1000;
    else if (Local_Ichg < 0)
        Local_Ichg = 0;

    if ((Chg_Type > NO_CHG) && (Is_small_charging_current == false)) // charging function
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Charging Function , Local_Ichg:%d\n", Local_Ichg);

        if (charger_flag != CHG_COMPLETE && T_Capacity_uAH >= Val_99_uAH)
        {
            if (msm_batt_info.batt_capacity < 100)    //check prev. capacity was lower than 100
                T_Capacity_uAH = Val_99_uAH;
        }
        else if (charger_state == CCI_CHG_SET_CC_MODE)
        {
            if (Delta_uAH > (-1) * CONV_PERCENT_TO_uAH(20))
            {
                THEO_uAH = THEO_uAS(Local_Ichg, 30);
                do_div(THEO_uAH, 3600);
                T_Capacity_uAH = T_Capacity_uAH + THEO_uAH;
            }
        }
        else
        {
            if (Delta_uAH > (-1) * CONV_PERCENT_TO_uAH(10))
            {
                THEO_uAH = THEO_uAS(Local_Ichg, 30);
                do_div(THEO_uAH, 3600);
                T_Capacity_uAH = T_Capacity_uAH + THEO_uAH;
            }
        }
    }
    else // discharge function
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Discharging Function \n");

        if(Is_small_charging_current)
            Smooth_Adj = Smooth_Adj_3;

        if (Delta_uAH < 0)
        {
            THEO_uAH = THEO_uAS(cci_cal_current, 30);
            do_div(THEO_uAH, 3600);
            T_Capacity_uAH = T_Capacity_uAH - THEO_uAH;
                
            /* For low battery special case */
            if(*ACap < 5)
                Smooth_Adj = Smooth_Adj_1;

            /* For 0c temperature config */
            if(msm_batt_info.batt_temp <= 10)
                Smooth_Adj = Smooth_Adj_2;
           
            Adj_level = (short)(abs(Delta_uAH) / CONV_PERCENT_TO_uAH(Smooth_Adj));
            if (Adj_level > 0 && msm_batt_info.batt_capacity < 100)
            {
                //Adj_level--;
                T_Capacity_uAH = T_Capacity_uAH - THEO_uAH * Adj_level;
            }
        }
    }

    //Boundary check
    if (T_Capacity_uAH < 0)
        T_Capacity_uAH = 0;
    else if (T_Capacity_uAH > MAX_uAH)
        T_Capacity_uAH = MAX_uAH;


    //rounding off
    rtn = (T_Capacity_uAH + 5 * MAX_CAP_MA);    //T_Capacity_uAH + 0.5 * 10 * MAX_CAP_MA
    do_div(rtn, (10 * MAX_CAP_MA));     //(T_Capacity_uAH * 100 / 1000 / MAX_CAP_MA)
    if (rtn > 100)
        rtn = 100;

    *CCap  = (u32)rtn;


}

static void Smooth_Cap_Resume(int *ACap, int *CCap, u32 suspend_time)
{
    s64 chg_uah = 0;
    s64 Local_Adc_uAH = 0;
    s64 rtn = 0;
    s64 THEO_uAH = 0;
    int rc = 0;
    int Local_IsOnCall = atomic_read(&g_IsOnCall);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s()\n", __func__);
    rc = Get_charging_uah(&chg_uah);
    if (rc < 0)    //Can't get rpc from modem
    {
        *CCap = *ACap;
        return;
    }

    Local_Adc_uAH = CONV_PERCENT_TO_uAH(*ACap);

    //if (chg_batt_event == CHG_UI_EVENT__DONE)
    if (charger_flag == CHG_COMPLETE)
    {
        T_Capacity_uAH = MAX_uAH;
    }
    else if (chg_uah > 0 || charger_flag == CHG_OT)
    {
        T_Capacity_uAH += chg_uah;
    }
    else if (g_Suspend_Chg_Source == NO_CHG)
    {
        if (Local_IsOnCall)     //On call after suspend
        {
            if (Local_Adc_uAH <= T_Capacity_uAH)
                T_Capacity_uAH = Local_Adc_uAH;    //using adc's value
        }        
        else if (suspend_time < 19*60)    //Discharged less than 19 min or not a phone call during suspend
        {
            {
                THEO_uAH = THEO_uAS(10, suspend_time);
                do_div(THEO_uAH, 3600);
                T_Capacity_uAH = T_Capacity_uAH - THEO_uAH;
            }
        }
        else
        {
            if (Local_Adc_uAH <= T_Capacity_uAH)
                T_Capacity_uAH = Local_Adc_uAH;    //using adc's value
        }

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Smooth_Cap_Resume, suspend_time:%d, IsOnCall:%d\n",suspend_time, Local_IsOnCall);

    }
    else if (chg_uah == 0 && g_Suspend_Chg_Source == AC_CHG)  //To avoid plugging charger then resume rapidly
    {
        T_Capacity_uAH += chg_uah;
    }
    else
    {
        if (Local_Adc_uAH <= T_Capacity_uAH)
            T_Capacity_uAH = Local_Adc_uAH;    //using adc's value
    }

    //Boundary check
    if (T_Capacity_uAH < 0)
        T_Capacity_uAH = 0;
    else if (T_Capacity_uAH > MAX_uAH)
        T_Capacity_uAH = MAX_uAH;

    //rounding off
    rtn = (T_Capacity_uAH + 5 * MAX_CAP_MA);    //T_Capacity_uAH + 0.5 * 10 * MAX_CAP_MA
    do_div(rtn, (10 * MAX_CAP_MA));     //(T_Capacity_uAH * 100 / 1000 / MAX_CAP_MA)
    if (rtn > 100)
        rtn = 100;

    //Sync with modem's charge state
    if (rtn == 100 && msm_batt_info.current_chg_source > NO_CHG
            && msm_batt_info.batt_capacity != 100 && charger_flag != CHG_COMPLETE )
    {
        rtn = 99;
        T_Capacity_uAH = CONV_PERCENT_TO_uAH(99);
    }

    *CCap  = (u32)rtn;


}

/**
 * Checking current volt > old volt or not
 */
static bool small_charging_current(int current_volt)
{
    static int usb_count = 0;
    static int old_volt = 0;
    int diff_volt = 0;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: %s, old_volt %d current_volt %d diff_volt %d\n",
            __func__ , old_volt, current_volt, diff_volt);

    if (atomic_read(&g_ChgVld) == CHG_NULL)
        return false;

    if (current_volt <= 4000){
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "[Battery]: Switch to CC Mode \n");

        charger_state = CCI_CHG_SET_CC_MODE;
    }
 
    if(usb_count == 0){
        old_volt = current_volt;
        usb_count++;
    }else{
        diff_volt = old_volt - current_volt;

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "[Battery]: old_volt %d current_volt %d diff_volt %d\n", old_volt, current_volt, diff_volt);

        if (diff_volt >= 1){
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: OverCurrent Case, Current_volt < Old_volt \n");

            old_volt = current_volt;
            diff_volt = 0;
            return true;
        }
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Normal Case, Current_vot > Old_volt \n");

    old_volt = current_volt;
    return false;
}
    
/**
 * Update battery status
 */
static void msm_batt_update_psy_status_v0(void)
{
    u32 batt_charging = 0;
    u32 batt_volt = 0;
    u32 chg_batt_event = CHG_UI_EVENT__INVALID;
    s32 chg_batt_temp = 0;//over temp
    s32 power_level_dbm = 0;
    u32 charger_valid = 0;
    s32 localpercent;
    s32 amss_chg_state;
    u32 localvoltage = 0;
    u32 temp_localpercent = 0;
    bool Is_small_charging_current;
    int CCapacity = 0;
    int rc = 0;
    int i = 0;

   atomic_set(&is_updating, true);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    if(BootCount > 0)
    {
        BootCount --;
    }
	MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: BootCount = %d\n",BootCount);
    for(i=0;i<3;i++)
    {

    	rc = msm_batt_get_batt_rpc(&batt_volt,&batt_charging, &charger_valid,
                          &chg_batt_event,&chg_batt_temp,&power_level_dbm,&amss_chg_state);
        if(rc < 0)
    	{
		MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,"[Battery]: Call RPC failed, rc return = %d , retry = %d \n",rc , i);
		msleep(100);
    	}
	else
	{
		MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,"[Battery]: Call RPC success, rc return = %d , retry = %d \n",rc , i);
		break;
	}
    }

    msm_batt_info.charger_valid = charger_valid;

/*	if(batt_charging == 0)
	{
		atomic_set(&g_ChgVld, NO_CHG);
	}
*/
    /* Checking OverCurrent or not */
    Is_small_charging_current = small_charging_current(batt_volt);


    /* Calculate the estimate of current */
    Estimate_Current(power_level_dbm);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: batt_charging = %u  batt_valid = %u \n"
            " batt_volt = %u charger_valid = %u \n"
            " chg_batt_event = %u chg_batt_temp = %d power_level_dbm = %d\n",
            batt_charging, msm_batt_info.batt_valid,
            batt_volt,charger_valid,
            chg_batt_event,chg_batt_temp,power_level_dbm);

    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: cci_cal_current = %d ,cci_batt_device_mask = 0x%x\n",cci_cal_current,cci_batt_device_mask);
    //Bug2433_Hsipo 20100905
    if(batt_capacity == 100 && charger_valid)
        chg_batt_event = CHG_UI_EVENT__DONE;

    UI_To_UpperLayer(chg_batt_event);

    // Is voltage_now initial already ??
    if (msm_batt_info.voltage_now > msm_batt_info.voltage_max_design)
    {
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    else if (msm_batt_info.voltage_now < msm_batt_info.voltage_min_design)
    {
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
    }
    else
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

    /* Get battery temperature */
    msm_batt_info.batt_temp = chg_batt_temp;

    /* Get voltage thorough RPC */
    localvoltage = batt_volt;

    /* IMPORTANT!!! Send local voltage to get battery percentage */
    localpercent = msm_batt_capacity(localvoltage);

    if (T_Capacity_uAH == -12345678)   //For initialize
    {
        T_Capacity_uAH =  CONV_PERCENT_TO_uAH(localpercent);
        msm_batt_info.batt_capacity = localpercent;
    }

    /* Battery is in normal status, Using smooth Algo. */
    //if (chg_batt_event < CHG_UI_EVENT__BAD_TEMP)
    {
        //Convert % unit to uA
        if (T_Capacity_uAH > MAX_uAH || chg_batt_event == CHG_UI_EVENT__DONE)
        {
            T_Capacity_uAH = MAX_uAH;
            msm_batt_info.batt_capacity = 100;
        }
        else if (T_Capacity_uAH > 0)
        {
            Smooth_Cap(&localpercent, &CCapacity,  msm_batt_info.current_chg_source, Is_small_charging_current);

            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "[Battery]: Adc_Capacity:%d, CCapacity:%d\n", localpercent, CCapacity);
            temp_localpercent = localpercent;
            localpercent = CCapacity;
        }
    }


    //Detect battery capacity, if battery capacity = 0%, power off the system
    if (localpercent > 0)
    {       
        if (temp_localpercent == 0 && localpercent <= 1 )
        {
            if (call_state != PHONE_OFFHOOK && localvoltage < 3400)
                localpercent = 0;
            else if (call_state == PHONE_OFFHOOK && localvoltage < 3300)
                localpercent = 0;
        }
        else // Avoid the capacity floating
        {
            if (batt_charging != FALSE){  // Prevent battery don't get USB plug in/out event
                if(Is_small_charging_current == false)   // Checking OverCurrent or not
                {
                    if (atomic_read(&g_ChgVld))
                    {
                        if (localpercent < msm_batt_info.batt_capacity)
                            localpercent = msm_batt_info.batt_capacity;
                    }
                }
                else
                {
                    if (localpercent > msm_batt_info.batt_capacity)
                        localpercent = msm_batt_info.batt_capacity;
                }
            }
            else
            {
                if (localpercent > msm_batt_info.batt_capacity)
                    localpercent = msm_batt_info.batt_capacity;
            }
            g_Shotdown_Count = 1;
        }
        g_Shotdown_Count = 1;
    }
    else if (msm_batt_info.current_chg_source == NO_CHG) //if device is not charging
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: BattLifeNotGood! LifePercent = %d, Count:%d\n", localpercent, g_Shotdown_Count);

        if (g_Shotdown_Count < 1)
        {
            localpercent = 0;
        }
        else
        {
            g_Shotdown_Count--;
            localpercent = 1;
        }
    }

    msm_batt_info.batt_capacity = localpercent;
    msm_batt_info.voltage_now = batt_volt;

    // Send ShutDown evet while in low battery charging modem
    if((Is_small_charging_current) && (msm_batt_info.batt_capacity <= 2)){
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "[Battery]: USB current too small,  LifePercent = %d, We are going to shutdown device \n", localpercent);

        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    }

    g_Resume_Capacity = msm_batt_info.batt_capacity;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: M_Chgr:%d, Flg:%d, Ichg:%d, Call_state:%d, M_Chg:%d, C:%d, msk:0x%x\n",
           charger_state, charger_flag, Ichg, call_state, amss_chg_state, cci_cal_current,cci_batt_device_mask);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: IsChg:%u, b_vld:%u, c_vld:%u, LF:%d, V:%d, T:%d,"
           " M_evt:%u, PWR:%d, g_chgSrc:%d\n",
           batt_charging, msm_batt_info.batt_valid,charger_valid,
           msm_batt_info.batt_capacity, msm_batt_info.voltage_now,msm_batt_info.batt_temp,
           chg_batt_event,power_level_dbm,msm_batt_info.current_chg_source);

    power_supply_changed(&msm_psy_batt);

    atomic_set(&is_updating, false);
	
}

/**
 * Later resume
 */
static void msm_batt_update_psy_status_v2(void)
{
    u32 batt_charging = 0;
    u32 batt_volt = 0;
    u32 chg_batt_event = CHG_UI_EVENT__INVALID;
    s32 chg_batt_temp = 0;//over temp
    s32 power_level_dbm = 0;
    u32 charger_valid = 0;
    s32 localpercent;
    s32 amss_chg_state = 0;

    int rc = 0;
    int i = 0;

    u32 localvoltage = 0;


    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    for(i=0;i<3;i++)
    {
    	rc = msm_batt_get_batt_rpc(&batt_volt,&batt_charging, &charger_valid,
                          &chg_batt_event,&chg_batt_temp,&power_level_dbm,&amss_chg_state);

        if(rc < 0)
    	{
		MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,"[Battery]: Call RPC failed, rc return = %d , retry = %d \n",rc , i);
		msleep(100);
    	}
	else
	{
		MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,"[Battery]: Call RPC success, rc return = %d , retry = %d \n",rc , i);
		break;
	}
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: M_Chgr:%d, Flg:%d, Ichg:%d, Call_state:%d, M_Chg:%d, C:%d, msk:0x%x\n",
           charger_state, charger_flag, Ichg, call_state, amss_chg_state, cci_cal_current,cci_batt_device_mask);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: IsChg:%u, b_vld:%u, c_vld:%u, LF:%d, V:%d, T:%d,"
           " M_evt:%u, PWR:%d, g_chgSrc:%d\n",
           batt_charging, msm_batt_info.batt_valid,charger_valid,
           msm_batt_info.batt_capacity, msm_batt_info.voltage_now,msm_batt_info.batt_temp,
           chg_batt_event,power_level_dbm,msm_batt_info.current_chg_source);

    msm_batt_info.charger_valid = charger_valid;


    Estimate_Current(power_level_dbm);

    UI_To_UpperLayer(chg_batt_event);

    if (msm_batt_info.voltage_now > msm_batt_info.voltage_max_design)
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    else if (msm_batt_info.voltage_now < msm_batt_info.voltage_min_design)
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_DEAD;
    else
        msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

    if (call_state == ENABLE_BATT_DBG_INFO)
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "cci_cal_current = %d ,cci_batt_device_mask = 0x%x\n",cci_cal_current,cci_batt_device_mask);

    //batt_temp
    msm_batt_info.batt_temp = chg_batt_temp;

    localvoltage = batt_volt;
    localpercent = msm_batt_capacity(localvoltage);


    //msm_batt_info.batt_capacity = localpercent;  //Save adc's capacity to batt_capacity
    g_Resume_Capacity = localpercent;  //Save adc's capacity to batt_capacity
    //power_supply_changed(&msm_psy_batt);      //Delay updating power status at late resume

    g_Is_Run_suspend = true;
}

static void msm_batt_update_psy_status_v3(void)
{

    struct timespec end_ts;
    unsigned int elapsed_sec =0;

    int CCapacity = 0;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    //smooth capacity linear due to resume phone
    getnstimeofday(&end_ts);
    //rtc_time_to_tm(end_ts.tv_sec, &end_tm);

    if (end_ts.tv_sec - pre_ts.tv_sec >= 0)
    {
        elapsed_sec = end_ts.tv_sec - pre_ts.tv_sec;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Suspend_sec:%d\n", elapsed_sec);
    }
    else
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: Delta time overflow\n");
    }

    //It will not suspend while usb plugging
    if (g_Is_Run_suspend)
    {
        Smooth_Cap_Resume(&g_Resume_Capacity, &CCapacity, elapsed_sec);
        msm_batt_info.batt_capacity = CCapacity;
    }
    // [BSP] Move to USB at dormancy estate find can't unlock
    getnstimeofday(&pre_ts);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: Late resume capacity:%d, chg_source:%d, g_Suspend_Chg_Source:%d, Is_Sleep:%d \n",
           msm_batt_info.batt_capacity, msm_batt_info.current_chg_source, g_Suspend_Chg_Source, g_Is_Run_suspend);

    //Reset g_Suspend_Chg_Source
    g_Suspend_Chg_Source = msm_batt_info.current_chg_source;

    g_Is_Run_suspend = false;

    power_supply_changed(&msm_psy_batt);


}


static int msm_batt_deregister(u32 handle)
{
    int rc;
    struct batt_client_deregister_req
    {
        struct rpc_request_hdr req;
        s32 handle;
    } batt_deregister_rpc_req;

    struct batt_client_deregister_reply
    {
        struct rpc_reply_hdr hdr;
        u32 batt_error;
    } batt_deregister_rpc_reply;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    batt_deregister_rpc_req.handle = cpu_to_be32(handle);
    batt_deregister_rpc_reply.batt_error = cpu_to_be32(BATTERY_LAST_ERROR);

    rc = msm_rpc_call_reply(msm_batt_info.batt_ep,
                            BATTERY_DEREGISTER_CLIENT_PROC,
                            &batt_deregister_rpc_req,
                            sizeof(batt_deregister_rpc_req),
                            &batt_deregister_rpc_reply,
                            sizeof(batt_deregister_rpc_reply),
                            msecs_to_jiffies(BATT_RPC_TIMEOUT));
    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: msm_rpc_call_reply failed! proc=%d rc=%d\n",
               __func__, BATTERY_DEREGISTER_CLIENT_PROC, rc);
        return rc;
    }

    if (be32_to_cpu(batt_deregister_rpc_reply.batt_error) !=
            BATTERY_DEREGISTRATION_SUCCESSFUL)
    {

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: vBatt deregistration Failed "
               "  proce_num = %d,"
               " batt_clnt_handle = %d\n",
               __func__, BATTERY_DEREGISTER_CLIENT_PROC, handle);
            
        return -EIO;
    }
    return 0;
}


static void msm_batt_wait_for_batt_chg_event(struct work_struct *work)
{
    if (smd_is_closed)
	return;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s: Batt RPC call back thread started, api_ver:0x%X\n",	__func__,msm_batt_info.chg_api_version);
    if (atomic_read(&is_updating)==false)        
    msm_batt_update_psy_status_v0(); //Refresh_timer
    schedule_delayed_work(&msm_batt_info.dwork, msecs_to_jiffies(30*1000));
}


static void msm_batt_start_cb_thread(void)
{
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);
    schedule_delayed_work(&msm_batt_info.dwork, msecs_to_jiffies(30*1000));
}

static int msm_batt_cleanup(void)
{
    int rc = 0;
    int rc_local;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE)
    {

        rc = msm_batt_deregister(msm_batt_info.batt_handle);
        if (rc < 0)
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
                "%s: msm_batt_deregister failed rc=%d\n",
                   __func__, rc);
    }
    msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

    if (msm_batt_info.msm_psy_ac)
        power_supply_unregister(msm_batt_info.msm_psy_ac);

    if (msm_batt_info.msm_psy_usb)
        power_supply_unregister(msm_batt_info.msm_psy_usb);
    if (msm_batt_info.msm_psy_batt)
        power_supply_unregister(msm_batt_info.msm_psy_batt);

    if (msm_batt_info.batt_ep)
    {
        rc_local = msm_rpc_close(msm_batt_info.batt_ep);
        if (rc_local < 0)
        {
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
                "%s: msm_rpc_close failed for batt_ep rc=%d\n",
                   __func__, rc_local);            
            if (!rc)
                rc = rc_local;
        }
    }

    if (msm_batt_info.chg_ep)
    {
        rc_local = msm_rpc_close(msm_batt_info.chg_ep);
        if (rc_local < 0)
        {
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
                "%s: msm_rpc_close failed for chg_ep rc=%d\n",
                   __func__, rc_local);
            if (!rc)
                rc = rc_local;
        }
    }

    if (msm_batt_info.cci_ep)
    {
        rc_local = msm_rpc_close(msm_batt_info.cci_ep);
        if (rc_local < 0)
        {
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
                "%s: msm_rpc_close failed for cci_ep rc=%d\n",
                   __func__, rc_local);
            if (!rc)
                rc = rc_local;
        }
    }


#ifdef CONFIG_HAS_EARLYSUSPEND
    if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
        unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
    return rc;
}


static int Scale_To_Capacity (short current_index, int index, int vbatt_now, int vbatt_scale_min ,int vbatt_scale_max)
{
    int result;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s(), current_index:%d, i:%d, vnow:%d, vsmin:%d, vsmax:%d\n",
           __func__,current_index, index,vbatt_now,vbatt_scale_min,vbatt_scale_max);

    if (current_index == up_table_volt_usb_new)   //USB CC mode
    {
        if ( vbatt_now > 4100)
            return 90;
        else
            return ((vbatt_now - vbatt_scale_min) *10 / (vbatt_scale_max - vbatt_scale_min))  + (index*10);
    }
    else if (current_index == up_table_volt_ac_800)  //AC CC mode
    {
        if ( vbatt_now > 4100)  
            return 85; //CC to CV threshold
        else
            return ((vbatt_now - vbatt_scale_min) *5 / (vbatt_scale_max - vbatt_scale_min))  + (index*5);
    }
    else if (current_index == up_table_volt_ac_400)    //AC CV mode
    {
        if ( vbatt_now <= 4100)
            return 85; //CC to CV threshold
        else
            return ((vbatt_now - vbatt_scale_min) *10 / (vbatt_scale_max - vbatt_scale_min))  + (index*10) + 70;
    }
    else if (current_index == up_table_volt_usb_new1)    //USB CV mode
    {
        if ( vbatt_now <= 4100)
            return 90;
        else
            return ((vbatt_now - vbatt_scale_min) *5 / (vbatt_scale_max - vbatt_scale_min))  + (index*5) + (90);
    }
    else
    {
        if (vbatt_now > vbatt_scale_max)
            vbatt_now = vbatt_scale_max;

        result = ((vbatt_now - vbatt_scale_min) *5 / (vbatt_scale_max - vbatt_scale_min))  + (index*5);
        if ( (((vbatt_now - vbatt_scale_min) *5) -((result-(index*5))*(vbatt_scale_max - vbatt_scale_min)))>=((vbatt_scale_max - vbatt_scale_min)/2))
            result = result +1;
        if (result<1)
            result = result +1;

        return result;
    }

}
/**
 * Select Table
 *
 */
int SelectTable(short current_index, int battery_scale, int average_adc_vbatt_mv, int *battery_capacity_table_volt)
{
    unsigned char    battery_capacity_check_index = 0xFF;
    unsigned char    battery_capacity_check_index_offset = 0;
    int i = 0;
    int adc_battery_capacity = 0;

    battery_capacity_check_index = battery_scale/2 ;
    for ( i = 1; i < battery_scale + 3; i = i * 2 )
    {
        battery_capacity_check_index_offset = ( battery_scale / ( i * 4 ) );

        if ( battery_capacity_check_index_offset == 0 )
            battery_capacity_check_index_offset = 1;

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
                "[Battery] Check_Index:%d, offset:%d\n",battery_capacity_check_index, battery_capacity_check_index_offset);
        /* adc voltage < table voltage*/
        if ( average_adc_vbatt_mv < battery_capacity_table_volt[battery_capacity_check_index] )
        {
            battery_capacity_check_index = battery_capacity_check_index - battery_capacity_check_index_offset;
            if (battery_capacity_check_index ==0)
            {
                if (current_index > MAX_DISCHARGE_TABLE) //Charging mode
                {
                    adc_battery_capacity =
                        Scale_To_Capacity(
                            current_index,
                            battery_capacity_check_index,
                            average_adc_vbatt_mv,
                            battery_capacity_table_volt[battery_capacity_check_index],
                            battery_capacity_table_volt[battery_capacity_check_index+1]
                        );
                }
                else //Discharge mode
                {
                    adc_battery_capacity =
                        Scale_To_Capacity(
                            current_index,
                            battery_capacity_check_index,
                            average_adc_vbatt_mv,
                            battery_capacity_table_volt[battery_capacity_check_index],
                            battery_capacity_table_volt[battery_capacity_check_index+21]
                        );
                }
                break;
            }
        }
        /* adc voltage > table voltage */
        else if ( average_adc_vbatt_mv > battery_capacity_table_volt[battery_capacity_check_index+1] )
        {
            battery_capacity_check_index = battery_capacity_check_index + battery_capacity_check_index_offset;
            if ((battery_capacity_check_index == battery_scale -1) && (battery_scale == 14 || battery_scale == 9))
            {
                adc_battery_capacity =
                    Scale_To_Capacity(
                        current_index,
                        battery_capacity_check_index,
                        average_adc_vbatt_mv,
                        battery_capacity_table_volt[battery_capacity_check_index],
                        battery_capacity_table_volt[battery_capacity_check_index+1]
                    );
                break;
            }
        }
        else
        {
            if (current_index > MAX_DISCHARGE_TABLE) //Charging mode
            {
                adc_battery_capacity =
                    Scale_To_Capacity(
                        current_index,
                        battery_capacity_check_index,
                        average_adc_vbatt_mv,
                        battery_capacity_table_volt[battery_capacity_check_index],
                        battery_capacity_table_volt[battery_capacity_check_index+1]
                    );
            }
            else // Discharge mode
            {
                adc_battery_capacity =
                    Scale_To_Capacity(
                        current_index,
                        battery_capacity_check_index,
                        average_adc_vbatt_mv,
                        battery_capacity_table_volt[battery_capacity_check_index],
                        battery_capacity_table_volt[battery_capacity_check_index+21]
                    );
            }
            break;
        }
    }

    //Boundery_check
    if (adc_battery_capacity > 100)
        adc_battery_capacity = 100;
    else if (adc_battery_capacity < 0)
        adc_battery_capacity = 0;

    return adc_battery_capacity;
}

void get_battery_discharging_table(
        int total_current,
        int average_adc_vbatt_mv,
        int *current_index,
        int temperature_index)
{
    int offset = 0;
    int i;
    u8 half_mA = false;


    if (total_current >= 700 + offset)
    {
        *current_index = down_table_volt_700;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_700)\n",average_adc_vbatt_mv);
    }
    else if (total_current >= 600 + offset)
    {
        *current_index = down_table_volt_600;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_600)\n",average_adc_vbatt_mv);
    }
    else if (total_current >= 550 + offset)
    {
        *current_index = down_table_volt_600;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_550)\n",average_adc_vbatt_mv);
        half_mA = true;

    }
    else if (total_current >= 500 + offset)
    {
        *current_index = down_table_volt_500;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_500)\n",average_adc_vbatt_mv);
    }
    else if (total_current >=450 + offset)
    {
        *current_index = down_table_volt_500;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_450)\n",average_adc_vbatt_mv);
        half_mA = true;

    }
    else if (total_current >= 400 + offset)
    {
        *current_index = down_table_volt_400;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_400)\n",average_adc_vbatt_mv);
    }
    else if (total_current >= 350 + offset)
    {
        *current_index = down_table_volt_400;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_350)\n",average_adc_vbatt_mv);
        half_mA = true;

    }
    else if (total_current >= 300 + offset)
    {
        *current_index = down_table_volt_300;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_300)\n",average_adc_vbatt_mv);
    }
    else if (total_current >=250 + offset)
    {
        *current_index = down_table_volt_300;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_250)\n",average_adc_vbatt_mv);
        half_mA = true;

    }
    else if (total_current >= 200 + offset)
    {
        *current_index = down_table_volt_200;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_200)\n",average_adc_vbatt_mv);
    } 
    else if (total_current >= 150 + offset)
    {
        *current_index = down_table_volt_200;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_150)\n",average_adc_vbatt_mv);
        half_mA = true;

    }
    else
    {
        *current_index = down_table_volt_100;
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: average_adc_vbatt_mv:%d" "(Table_100)\n",average_adc_vbatt_mv);
    }

    // Copy array item
    if(half_mA)
    {
        //Example: 350mA = (400mA + 300mA)/2
        for(i=0; i<41; i++)
            mA_table[i] = (int) (( *(project_final_table[temperature_index][*current_index-1] + i) +
                                   *(project_final_table[temperature_index][*current_index] + i)) / 2);
        half_mA = false;
    }
    else
    {
        for(i=0; i<41; i++)
            mA_table[i] = (int) *(project_final_table[temperature_index][*current_index] + i);
    }

}

/**
 * Find percentage mapping table
 */
void cci_battery_capacity_report(int average_adc_vbatt_mv, int total_current, int *adc_batt_capacity)
{
    int current_index = 0;
    int battery_scale = 0;
    int *battery_capacity_table_volt = NULL;
    int local_temp = msm_batt_info.batt_temp;
    int temperature_index = 0;
    int i = 0;


    if (adc_batt_capacity == NULL)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: adc_batt_capacity is null \n");
        return;
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: charger_flag:%d, charger_state:%d, g_chgVld:%d, chg_state:%d local_temp:%d\n",
    charger_flag,charger_state,atomic_read(&g_ChgVld),charger_state,local_temp);

    /* Determine temperature table */
    if (local_temp > 39)
        temperature_index = TEMPERATURE_45;
    else if (local_temp > 10)
        temperature_index = TEMPERATURE_25;
    else
        temperature_index = TEMPERATURE_0;

    if (first_boot == false){
        if ( (charger_flag == CHG_CHARGING && msm_batt_info.current_chg_source == AC_CHG)
                || (charger_flag == CHG_OT && atomic_read(&g_ChgVld) ==  AC_CHG))     //AC Charging
        {
            if (charger_state == CCI_CHG_SET_CC_MODE)
            {
                current_index = up_table_volt_ac_800;
                battery_scale = 14;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_ac_charger_cc_mode)\n",average_adc_vbatt_mv);
            }
            else if (charger_state == CCI_CHG_SET_CV_MODE)
            {
                current_index = up_table_volt_ac_400;
                battery_scale = 3;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_ac_charger_cv_mode)\n",average_adc_vbatt_mv);
            }
            else
            {
                current_index = up_table_volt_ac_800;
                battery_scale = 14;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_ac_charger_cc_mode)\n",average_adc_vbatt_mv);
            }
            battery_capacity_table_volt = project_final_table[temperature_index][current_index];

        }
        else if ((charger_flag == CHG_CHARGING && msm_batt_info.current_chg_source == USB_CHG)
                || (charger_flag == CHG_OT && atomic_read(&g_ChgVld) == USB_CHG))  //USB Charging
        {
            if (charger_state == CCI_CHG_SET_CC_MODE)
            {
                current_index = up_table_volt_usb_new;
                battery_scale = 9;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_usb_charger_cc_mode)\n",average_adc_vbatt_mv);
            }
            else if (charger_state == CCI_CHG_SET_CV_MODE)
            {
                current_index = up_table_volt_usb_new1;
                battery_scale = 2;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_usb_charger_cv_mode)\n",average_adc_vbatt_mv);
            }
            else
            {
                current_index = up_table_volt_usb_new;
                battery_scale = 9;
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: average_adc_vbatt_mv:%d" "(Table_usb_charger_cc_mode)\n",average_adc_vbatt_mv);
            }

            battery_capacity_table_volt = project_final_table[temperature_index][current_index];   

        }
        else
        {
            battery_scale = 20;
            get_battery_discharging_table(total_current, average_adc_vbatt_mv, &current_index,temperature_index);
            battery_capacity_table_volt = mA_table;
        }
    }
    else
    {
        battery_scale = 20;
        get_battery_discharging_table(total_current, average_adc_vbatt_mv, &current_index,temperature_index);
        battery_capacity_table_volt = mA_table;
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: PID:%d, Temperature index:%d, Current index:%d\n",cci_project_id, temperature_index, current_index);
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: bScale:%d, avg_adc:%d\n",battery_scale, average_adc_vbatt_mv);

    //For array item deubg
    if (0)
    {    
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "Battery scale :%d\n",battery_scale);

        for (i=0;i<20;i++)
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                " battery_capacity_table_volt array list: %d \n", battery_capacity_table_volt[i]);
    }


    /* !!! Select mapping table !!! */
    *adc_batt_capacity = SelectTable(current_index, battery_scale, average_adc_vbatt_mv, battery_capacity_table_volt);

    if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
        previous_capacity_down_table = battery_capacity_table_volt;

}

/**
 * Get battery percentage 
 */
static u32 msm_batt_capacity(u32 current_voltage)
{
    int adc_battery_capacity = 0x0;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s(), current_voltage:%d \n", __func__, current_voltage);

    if (current_voltage < BATTERY_LOW) // Critical low battery 1% or 0%, Battery voltage < 3.4v
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
            "[Battery]: current_voltage = %d\n",current_voltage);
        if (call_state == PHONE_OFFHOOK && current_voltage > 3300)
            return 1;
        else
            return 0;
    }
    else if ((msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL) || charger_flag == CHG_COMPLETE) //Battery full
        return 100;
    else if (current_voltage >= BATTERY_HIGH) // Current voltage > 4.2v
    {
        if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING || charger_flag == CHG_CHARGING)
        {
            if (msm_batt_info.current_chg_source & AC_CHG)
            {
                if (charger_state == CCI_CHG_SET_CC_MODE)
                {
		    if(BootCount > 0)
	            {
			return 95;
		    }
		    else
	            {
                        return 85; //AC Charger, CC to CV threshold 
		    }
                } 
                else
		{
		    return 99;
                }
            }
            else if (msm_batt_info.current_chg_source & USB_CHG)
            {
                if (charger_state == CCI_CHG_SET_CC_MODE)
                {
                    if(BootCount > 0)
		    {
			return 95;
		    }
		    else
		    {
                        return 90; //USB Charger, CC to CV threshold
		    }
                }
                else
		{
		    return 99;
		}
	    }
            else
                return 99;
        }
        else
            return 99;
    }
    else // Normal case
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: cci_cal_current = %d ,cci_batt_device_mask = 0x%x\n",cci_cal_current,cci_batt_device_mask);
        cci_battery_capacity_report(current_voltage, cci_cal_current, &adc_battery_capacity);
        return adc_battery_capacity;
    }

}

void Get_battery_mapping_table(int cci_project_id)
{
    int *TempTable[table_size_i][table_size_j];
    int i,j,k,kitem;
    int *ptrk1;
    int *ptrk2;

    memcpy(TempTable,pTableIndex,sizeof(pTableIndex));

    // Switch current mapping table by project
    switch (cci_project_id){
        case PROJECT_ID_K4:

            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: Project K4 Battery ID = %d\n",rep_batt_chg.battery_id);

            if(rep_batt_chg.battery_id == BATTERY_K4_SMP)
                memcpy(TempTable,pTableIndex,sizeof(pTableIndex));
            else if(rep_batt_chg.battery_id == BATTERY_K4_MEI)
                memcpy(TempTable,pTableIndex_k4_mei,sizeof(pTableIndex_k4_mei));

            MAX_CAP_MA = 1300;
            cci_base_current = 100;

            break;
        case PROJECT_ID_K4H:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: Project C4 Battery ID = %d\n",rep_batt_chg.battery_id);
            if(rep_batt_chg.battery_id == BATTERY_K4_SMP)
                memcpy(TempTable,pTableIndex,sizeof(pTableIndex));
            else if(rep_batt_chg.battery_id == BATTERY_K4_MEI)
                memcpy(TempTable,pTableIndex_k4_mei,sizeof(pTableIndex_k4_mei));

            MAX_CAP_MA = 1300;
            cci_base_current = 50;

            break;
        case PROJECT_ID_K5:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "[Battery]: Project K5 Battery ID = %d\n",rep_batt_chg.battery_id);
            if(rep_batt_chg.battery_id == BATTERY_K4_SMP)
                memcpy(TempTable,pTableIndex,sizeof(pTableIndex));
            else if(rep_batt_chg.battery_id == BATTERY_K4_MEI)
                memcpy(TempTable,pTableIndex_k4_mei,sizeof(pTableIndex_k4_mei));

            MAX_CAP_MA = 1300;
            cci_base_current = 50;
            break;
        case PROJECT_ID_CAP6:
            memcpy(TempTable,pTableIndex,sizeof(pTableIndex));
            break;
        case PROJECT_ID_CAP8:
            memcpy(TempTable,pTableIndex_JHT1000,sizeof(pTableIndex));
            break;
        default:
            memcpy(TempTable,pTableIndex,sizeof(pTableIndex));
            break;
    }
    for(i=0; i<table_size_i; i++){
        for(j=0; j<table_size_j; j++)
        {
            // for usb and ac array
            if (j >= MAX_DISCHARGE_TABLE)
                project_final_table[i][j] = kmalloc(min_table_items*sizeof(int), GFP_KERNEL);
            else
                project_final_table[i][j] = kmalloc(max_table_items*sizeof(int), GFP_KERNEL);
        }
    }

    // Copy data from pTable to projec_final_table
    for(i=0; i<table_size_i; i++)
    {
        for(j=0; j<table_size_j; j++)
        {
            if(TempTable[i][j] == NULL)
                continue;
            ptrk1 = project_final_table[i][j];
            ptrk2 = TempTable[i][j];

            // For usb and ac array
            if(j >= MAX_DISCHARGE_TABLE)
                kitem = min_table_items;
            else
                kitem = max_table_items;
            // copy array itmes
            for(k=0; k<kitem; k++)
                *(ptrk1 + k) = *(ptrk2 + k);
        }
    }

    // For Debug
    if(0){
        ptrk1 = (int *) *( *(project_final_table + 1) + 8);
        for (i=0; i< max_table_items;i++)
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                " %d,", *(ptrk1 + i));
    }
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void msm_batt_early_suspend(struct early_suspend *h)
{
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "%s(): going to early suspend\n", __func__);
    getnstimeofday(&pre_ts);
}

void msm_batt_late_resume(struct early_suspend *h)
{

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s(), charger_flag:%d\n", __func__, charger_flag);

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "[Battery]: %s(), call msm_batt_update_phy_status_v3 \n", __func__);
    
    if (is_update_v3 == false&&atomic_read(&is_updating)==false)
    msm_batt_update_psy_status_v3();
    is_update_v3 = false;

    Update_Power_Status();
    atomic_set(&g_IsSuspend, false);
    atomic_set(&g_IsOnCall, false);

}
#endif

static int batt_suspend(struct platform_device *dev, pm_message_t state)
{

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    g_Suspend_Chg_Source = msm_batt_info.current_chg_source;
    atomic_set(&g_IsSuspend, true);    //false at late resume

    if (call_state == PHONE_OFFHOOK)
        atomic_set(&g_IsOnCall, true);   //false at late resume

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "batt_suspend\n");
    return 0;
}

static int batt_resume(struct platform_device *dev)
{
    
    cci_smem_value_t *smem_cci_smem_value;
    struct input_dev *kpdev;
    smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ) );

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "[Battery]: %s(), charger_flag:%d, smem:%d, ", __func__,charger_flag, smem_cci_smem_value->cci_battery_info);


    /* For Low Battery state, Modem will call battery driver to wake up device */
    if ( smem_cci_smem_value->cci_battery_info == CCI_BATTERY_POWERDOWN)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "Battery Verylow Warning!!!\n");
        kpdev = msm_7k_handset_get_input_dev();
        input_report_key(kpdev, KEY_POWER, 1);
        input_report_key(kpdev, KEY_POWER, 0);
        smem_cci_smem_value->cci_battery_info = 0;

    }
    else if (smem_cci_smem_value->cci_battery_info == CCI_BATTERY_CRITICAL)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "Battery Critical Warning!!!\n");
        kpdev = msm_7k_handset_get_input_dev();
        input_report_key(kpdev, KEY_POWER, 1);
        input_report_key(kpdev, KEY_POWER, 0);
        smem_cci_smem_value->cci_battery_info = 0;
    }
    else if (smem_cci_smem_value->cci_modem_charging_state == CCI_CHG_STATE_COMPLETE)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "Battery Full Charger!!!\n");

        if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_FULL)
        {
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Battery First Full Charger!!!\n");
            kpdev = msm_7k_handset_get_input_dev();
            input_report_key(kpdev, KEY_POWER, 1);
            input_report_key(kpdev, KEY_POWER, 0);
            smem_cci_smem_value->cci_modem_charging_state  = 0;
        }
    }
    else
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "batt_resume\n");
    }
    if (atomic_read(&is_updating)==false)
    msm_batt_update_psy_status_v2();
    return 0;
}


static int __devinit msm_batt_probe(struct platform_device *pdev)
{

    int rc;
    struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    /* Get Hardward and Project ID */
    cci_hardware_id = driver_get_hw_id();
    cci_project_id = driver_get_pj_id();
    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: Hardware ID = %d, Project ID = %d", cci_hardware_id, cci_project_id);

    /* Get K4 Battery ID (SMP or MEI) by RPC */
    msm_batt_get_batt_chg_status_v1();

    /* Get battry mapping table by hardware ID */
    Get_battery_mapping_table(cci_project_id);

    if (pdev->id != -1)
    {
        dev_err(&pdev->dev,
                "%s: MSM chipsets Can only support one"
                " battery ", __func__);
        return -EINVAL;
    }

    if (pdata->avail_chg_sources & AC_CHG)
    {
        rc = power_supply_register(&pdev->dev, &msm_psy_ac);
        if (rc < 0)
        {
            dev_err(&pdev->dev,
                    "%s: power_supply_register failed"
                    " rc = %d\n", __func__, rc);
            msm_batt_cleanup();
            return rc;
        }
        msm_batt_info.msm_psy_ac = &msm_psy_ac;
        msm_batt_info.avail_chg_sources |= AC_CHG;
    }

    if (pdata->avail_chg_sources & USB_CHG)
    {
        rc = power_supply_register(&pdev->dev, &msm_psy_usb);
        if (rc < 0)
        {
            dev_err(&pdev->dev,
                    "%s: power_supply_register failed"
                    " rc = %d\n", __func__, rc);
            msm_batt_cleanup();
            return rc;
        }
        msm_batt_info.msm_psy_usb = &msm_psy_usb;
        msm_batt_info.avail_chg_sources |= USB_CHG;
    }

    if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb)
    {

        dev_err(&pdev->dev,
                "%s: No external Power supply(AC or USB)"
                "is avilable\n", __func__);
        msm_batt_cleanup();
        return -ENODEV;
    }

    msm_batt_info.voltage_max_design = pdata->voltage_max_design;
    msm_batt_info.voltage_min_design = pdata->voltage_min_design;
    msm_batt_info.batt_technology = pdata->batt_technology;

    if (!msm_batt_info.voltage_min_design)
        msm_batt_info.voltage_min_design = BATTERY_LOW;
    if (!msm_batt_info.voltage_max_design)
        msm_batt_info.voltage_max_design = BATTERY_HIGH;

    if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
        msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

//    if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_UNKNOWN)
//        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    if (!msm_batt_info.calculate_capacity)
        msm_batt_info.calculate_capacity = msm_batt_capacity;

    rc = power_supply_register(&pdev->dev, &msm_psy_batt);
    if (rc < 0)
    {
        dev_err(&pdev->dev, "%s: power_supply_register failed"
                " rc=%d\n", __func__, rc);
        msm_batt_cleanup();
        return rc;
    }
    msm_batt_info.msm_psy_batt = &msm_psy_batt;

#ifdef CONFIG_HAS_EARLYSUSPEND
    msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
    msm_batt_info.early_suspend.resume = msm_batt_late_resume;
    register_early_suspend(&msm_batt_info.early_suspend);
#endif

    wake_lock_init(&msm_batt_info.wlock, WAKE_LOCK_SUSPEND, "low_battery_shutdown");

	/* IMPORTANT!!! init the workqueue, this is for battery update */
    INIT_DELAYED_WORK(&msm_batt_info.dwork, msm_batt_wait_for_batt_chg_event);

    /* Start the workqueue after 30s*/
    msm_batt_start_cb_thread();

	atomic_set(&msm_batt_info.initflag, 1);
    
	if (atomic_read(&msm_batt_info.initCharger) != 0){
	    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s, Driver init finish... Update Charger event  \n",__func__ );

        cci_android_charger_usb_change(1, atomic_read(&msm_batt_info.initCharger));
    }

    return 0;
}


static struct platform_driver msm_batt_driver;
static int __devinit msm_batt_init_rpc(void)
{
    int rc;

    spin_lock_init(&msm_batt_info.lock);

    msm_batt_info.msm_batt_wq =
        create_singlethread_workqueue("msm_battery");

    if (!msm_batt_info.msm_batt_wq)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: create workque failed \n", __func__);
        return -ENOMEM;
    }

    msm_batt_info.batt_ep =
        msm_rpc_connect_compatible(BATTERY_RPC_PROG, BATTERY_RPC_VERS, 0);

    if (msm_batt_info.batt_ep == NULL)
    {
        return -ENODEV;
    }
    else if (IS_ERR(msm_batt_info.batt_ep))
    {
        int rc = PTR_ERR(msm_batt_info.batt_ep);
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: rpc connect failed for BATTERY_RPC_PROG."
               " rc = %d\n ", __func__, rc);
        msm_batt_info.batt_ep = NULL;
        return rc;
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_NORMAL, KERN_INFO,
        "CHG_RPC_VERS  0x%8x ", CHG_RPC_VERS);
    msm_batt_info.chg_ep =
        msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VERS, 0);

    if (msm_batt_info.chg_ep == NULL)
    {
        return -ENODEV;
    }
    else if (IS_ERR(msm_batt_info.chg_ep))
    {
        int rc = PTR_ERR(msm_batt_info.chg_ep);
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: rpc connect failed for CHG_RPC_PROG. rc = %d\n",
               __func__, rc);
        msm_batt_info.chg_ep = NULL;
        return rc;
    }

    // CCI RPC
    msm_batt_info.cci_ep = msm_rpc_connect(CCIPROG, CCIVERS, 0);
    if (msm_batt_info.cci_ep == NULL)
    {
        return -ENODEV;
    }
    else if (IS_ERR(msm_batt_info.cci_ep))
    {
        int rc = PTR_ERR(msm_batt_info.cci_ep);
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: rpc connect failed for CCI_RPC_PROG. rc = %d\n",
               __func__, rc);
        msm_batt_info.cci_ep = NULL;
        return rc;
    }
    //end
    
    /* get charger api version */
    msm_batt_info.chg_api_version = DEFAULT_CHARGER_API_VERSION;

    rc = platform_driver_register(&msm_batt_driver);

    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: platform_driver_register failed for "
               "batt driver. rc = %d\n", __func__, rc);
        return rc;
    }

    return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
    int rc;
    wake_lock_destroy(&msm_batt_info.wlock);
    rc = msm_batt_cleanup();

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    if (rc < 0)
    {
        dev_err(&pdev->dev,
                "%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
        return rc;
    }
    return 0;
}



static struct platform_driver msm_batt_driver =
{
    .probe = msm_batt_probe,
    .suspend = batt_suspend,
    .resume	= batt_resume,
    .remove = __devexit_p(msm_batt_remove),
    .driver = {
        .name = "msm-battery",
        .owner = THIS_MODULE,
    },
};

static int call_state_proc_write(struct file *file, const char *buffer,
                                 unsigned long count, void *data)
{
    char *buf;
    int BUFIndex;
    int i;

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

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s(), BUFIndex:%d, buffer:%s \n", __func__, BUFIndex,buffer);

    switch (BUFIndex)
    {
        case PHONE_IDLE:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Phone idle\n");
            call_state = PHONE_IDLE;
            break;

        case PHONE_RINGING:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Phone Ringing\n");
            call_state = PHONE_RINGING;
            break;

        case PHONE_OFFHOOK:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Phone OffHook\n");
            call_state = PHONE_OFFHOOK;
            break;

        case ENABLE_BATT_DBG_INFO:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Enable battery debug info\n");
            call_state = ENABLE_BATT_DBG_INFO;
            break;

        case ENABLE_CHARGING_FUNC:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Enable Charging function\n");
            if (msm_batt_info.current_chg_source & USB_CHG)
            {
                call_state =  ENABLE_CHARGING_FUNC;
                charger_flag = CHG_CHARGING;
                msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
                power_supply_changed(&msm_psy_batt);
            }
            else
                call_state = PHONE_IDLE;
            break;

        case DISABLE_CHARGING_FUNC:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Disable Charging function\n");
            if (msm_batt_info.current_chg_source & USB_CHG)
            {
                call_state = DISABLE_CHARGING_FUNC;
                charger_flag = CHG_NONE;
                msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                power_supply_changed(&msm_psy_batt);
            }
            else
                call_state = PHONE_IDLE;
            break;
        case BATTERY_TEST_100:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Battery test function\n");
            for (i=100; i >= 0; i--){
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "Battery test capacity =%d \n", i);

                msm_batt_info.batt_capacity = i;
                power_supply_changed(&msm_psy_batt);
                mdelay(500);
            }
            break;

        case BATTERY_TEST_15:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Battery test function\n");
            for (i=20; i >= 0; i--){
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "Battery test capacity =%d \n", i);
                msm_batt_info.batt_capacity = i;
                power_supply_changed(&msm_psy_batt);
                mdelay(500);
            }
            break;
        case BATTERY_TEST_10:
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Battery test function\n");
            for (i=13; i >= 0; i--){
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "Battery test capacity =%d \n", i);
                msm_batt_info.batt_capacity = i;
                power_supply_changed(&msm_psy_batt);
                mdelay(500);
            }
            break;
        case BATTERY_TEST_5: 
            MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                "Battery test function\n");
            for (i=8; i >= 0; i--){
                MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
                    "Battery test capacity =%d \n", i);
                msm_batt_info.batt_capacity = i;
                power_supply_changed(&msm_psy_batt);
                mdelay(500);
            }
            break;

    }
    kfree(buf);
    return count;

}

/**
 * Get pure volt from modem side while device initialization
 */

static void Read_init_Volt(struct work_struct *work)
{
    u32 batt_charging = 0;
    u32 batt_volt = 0;
    u32 chg_batt_event = CHG_UI_EVENT__INVALID;
    s32 chg_batt_temp = 0;
    s32 power_level_dbm = 0;
    u32 charger_valid = 0;
    s32 localpercent = 50 ;
    s32 amss_chg_state = 0;
    int rc = -1;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    if(atomic_read(&msm_batt_info.initCharger) == 0)
         msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

    //if (atomic_read(&msm_batt_info.initCharger) == 0){
    
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: NO CHARGER, read Pure Volt from modem \n");

    //return;

    first_boot = true;

    while(rc < 0)
    {
        rc = msm_batt_get_batt_rpc(&batt_volt,&batt_charging, &charger_valid,
                          &chg_batt_event,&chg_batt_temp,&power_level_dbm,&amss_chg_state);

        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO, "[Battery]: msm_batt_get_batt_rpc, rc=%d\n", rc);
        msleep(200);
    }

     //bincent
    msm_batt_info.batt_temp = 25;
    localpercent = msm_batt_capacity(Volt_modem);
    first_boot = false; 
    T_Capacity_uAH = CONV_PERCENT_TO_uAH(localpercent);
    msm_batt_info.batt_capacity = localpercent;
	
    if((batt_charging == 1) && (Volt_modem > 4100))
    {
    	msm_batt_info.batt_capacity = 96;
	 MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: Read_init_Volt : set msm_batt_info.batt_capacity = 96 \n");
    }

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: Read_init_Volt, LF:%d, V:%d, T:%d\n", localpercent, Volt_modem, batt_volt);

}


static int __init msm_batt_init(void)
{
    int rc;
    struct proc_dir_entry *d_entry;

    MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
        "[Battery]: %s()\n", __func__);

    rc = msm_batt_init_rpc();

    if (rc < 0)
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_ERR,
            "%s: msm_batt_init_rpc Failed  rc=%d\n",
               __func__, rc);
        msm_batt_cleanup();
        return rc;
    }

    /* Read pure volt from modem for first percetage value */
    INIT_DELAYED_WORK(&g_dwork, Read_init_Volt);
    schedule_delayed_work(&g_dwork, msecs_to_jiffies(10*1000));

    d_entry = create_proc_entry("call_state",
                                S_IRWXU | S_IRWXO, NULL);
    if (d_entry)
    {
        d_entry->read_proc = NULL;
        d_entry->write_proc = call_state_proc_write;
        d_entry->data = NULL;
    }
    else
    {
        MSM_BATTERY_DPRINTK( MSM_BATTERY_DEBUG_INFO, KERN_INFO,
            "Unable to create /proc/call_state entry\n");
        remove_proc_entry("call_state", 0);
        return  -ENOMEM;
    }

    return 0;
}

static void __exit msm_batt_exit(void)
{
    platform_driver_unregister(&msm_batt_driver);
}

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");

