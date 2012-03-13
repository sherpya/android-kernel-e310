/* arch/arm/mach-msm/board-CCI-keypad.c
 *
 * Copyright (C) 2009 Google, Inc
 * Copyright (C) 2009 CCI Corporation.
 *
 * Author: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/keyreset.h>
#include <linux/platform_device.h>
#include <mach/vreg.h>
#include <mach/board.h>
#include <asm/mach-types.h>

#include "cci_smem.h"
#include "smd_private.h"
*/
#include "board-CCI-keypad.h"
#define GPIO_KEY_VOL_UP      42
#define GPIO_KEY_VOL_DOWN    41
#define GPIO_KEY_PWR_N       19

#define K4_PAD_UP		     36
#define K4_PAD_DOWN	     37
#define K4_PAD_RIGHT	     39
#define K4_PAD_LEFT	     38
#define K4_PAD_SEND	     35
#define K4_PAD_ENTER	     40

/*define by DVT*/
#define GPIO_KEY_PWR_N_DVT   29

/*define by K4H */
#define GPIO_EVT_KEY_CAMERA 122
#define GPIO_DVT_KEY_CAMERA 39


/*For CAP8 EVT*/
static struct gpio_event_direct_entry CCI_CAP8_keypad_key_map[] = {
	{
		.gpio	= GPIO_KEY_VOL_UP,
		.code	= KEY_VOLUMEUP,
	},
	{
		.gpio	= GPIO_KEY_VOL_DOWN,
		.code	= KEY_VOLUMEDOWN,
	},

};
/*For CAP8 DVT*/
static struct gpio_event_direct_entry CCI_CAP8_DVT_keypad_key_map[] = {
	{
		.gpio	= GPIO_KEY_VOL_UP,
		.code	= KEY_VOLUMEUP,
	},
	{
		.gpio	= GPIO_KEY_VOL_DOWN,
		.code	= KEY_VOLUMEDOWN,
	},

};

static struct gpio_event_direct_entry CCI_K4_keypad_key_map[] = {
	{
		.gpio	= GPIO_KEY_VOL_UP,
		.code	= KEY_VOLUMEUP,
	},
	{
		.gpio	= GPIO_KEY_VOL_DOWN,
		.code	= KEY_VOLUMEDOWN,
	},
	{

		.gpio	= K4_PAD_UP,
		.code	= KEY_UP,
	},
	{

		.gpio	=  K4_PAD_DOWN,
		.code	=  KEY_DOWN,
	},
	{

		.gpio	=  K4_PAD_RIGHT,
		.code	=  KEY_RIGHT,
	},
	{

		.gpio	=  K4_PAD_LEFT,
		.code	=  KEY_LEFT,
	},
	{

		.gpio	=  K4_PAD_SEND,
		.code	=  KEY_SEND,
	},
	{

		.gpio	=  K4_PAD_ENTER,
		.code	=  KEY_REPLY,
	}, 
};

//For CAP6 Keypad
static struct gpio_event_direct_entry CCI_CAP6_EVT_keypad_key_map[]={
	{
		.gpio = GPIO_KEY_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = GPIO_KEY_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
};

//For C4 Keypad EVT
// Add Camera Key
static struct gpio_event_direct_entry CCI_K4H_EVT_KeyMap[]={
	{
		.gpio = GPIO_KEY_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = GPIO_KEY_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = GPIO_EVT_KEY_CAMERA,
		.code = KEY_CAMERA,
	},	
};


//For C4 Keypad DVT
// Add Camera Key
static struct gpio_event_direct_entry CCI_K4H_DVT_KeyMap[]={
	{
		.gpio = GPIO_KEY_VOL_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = GPIO_KEY_VOL_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = GPIO_DVT_KEY_CAMERA,
		.code = KEY_CAMERA,
	},	
};



static struct gpio_event_input_info CCI_keypad_key_info = {
	.info.func = gpio_event_input_func,
	//.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS, //0
	.type = EV_KEY,
	.debounce_time.tv = {
		.sec = 0,
		.nsec = 3000000 // Reference: 1000000
	},
	.keymap = CCI_K4H_DVT_KeyMap,
	.keymap_size = ARRAY_SIZE(CCI_K4H_DVT_KeyMap),
};

static struct gpio_event_info *CCI_input_info[] = {
	&CCI_keypad_key_info.info,
};

static struct gpio_event_platform_data CCI_input_data = {
	.name = "CCI_penguin_keypad",
	.info = CCI_input_info,
	.info_count = ARRAY_SIZE(CCI_input_info),
};

static struct platform_device CCI_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &CCI_input_data,
	},
};


static int __init CCI_CAP_init_keypad(void)
{
	int ret;

	cci_smem_value_t *smem_cci_smem_value;
        smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ));

	 
	if(smem_cci_smem_value->cci_project_id == PROJECT_ID_K5)
        {
               printk("[QwertyKey] K5 Project\n");
               ret =  platform_device_register(&keypad_device_surf);
        }
	else
	{
	       printk("[UnQwertyKey] Not K5 Project\n");
		ret = platform_device_register(&CCI_input_device);
	}	
	if (ret) {
		printk(KERN_ERR "%s: register platform device fail (%d)\n",
								__func__, ret);
		return ret;
	}
	return 0;
}

void cci_project_detect(void)
{
	cci_smem_value_t *smem_cci_smem_value;
	smem_cci_smem_value = smem_alloc( SMEM_CCI_SMEM_VALUE, sizeof( cci_smem_value_t ));

	printk(KERN_ERR "cci_project_id = (%d)\n", smem_cci_smem_value->cci_project_id); //Lun
	switch(smem_cci_smem_value->cci_project_id)
	{
		case PROJECT_ID_K4:
			printk(KERN_ERR "[keypad]: K4 project\n");
			CCI_keypad_key_info.keymap = CCI_K4_keypad_key_map;
			CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_K4_keypad_key_map);
			break;
		case PROJECT_ID_CAP8:
			printk(KERN_ERR "[keypad]: CAP8 project\n");
			if(smem_cci_smem_value->cci_hw_id == HW_ID_EVT0)
			{
				CCI_keypad_key_info.keymap = CCI_CAP8_keypad_key_map;
				CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_CAP8_keypad_key_map);
			}else if(smem_cci_smem_value->cci_hw_id >= HW_ID_DVT1)
			{
				CCI_keypad_key_info.keymap = CCI_CAP8_DVT_keypad_key_map;
				CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_CAP8_DVT_keypad_key_map);
			}
			break;
		case PROJECT_ID_CAP6:
		case PROJECT_ID_CAP2: 
			CCI_keypad_key_info.keymap = CCI_CAP6_EVT_keypad_key_map;
			CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_CAP6_EVT_keypad_key_map);
			break;
		case PROJECT_ID_K4H://Add CAMERA Keypad (GPIO : EVT_122, DVT_39)
			if(smem_cci_smem_value->cci_hw_id == HW_ID_EVT0)
			{
				printk("[Keypad] K4H EVT\n");
				CCI_keypad_key_info.keymap = CCI_K4H_EVT_KeyMap;
                        	CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_K4H_EVT_KeyMap);
			}
			else if (smem_cci_smem_value->cci_hw_id >= HW_ID_DVT1)
			{
				printk("[Keypad] K4H DVT\n");
				CCI_keypad_key_info.keymap = CCI_K4H_DVT_KeyMap;
                                CCI_keypad_key_info.keymap_size = ARRAY_SIZE(CCI_K4H_DVT_KeyMap);
			}
			break;
         case PROJECT_ID_K5://Add K5 project
			if(smem_cci_smem_value->cci_hw_id == HW_ID_DVT1)
			{
				printk("[Qwerty Keypad] K5 DVT1\n");
				surf_keypad_matrix_info.keymap = keypad_keymap_surf_DVT1;
			}
			else if (smem_cci_smem_value->cci_hw_id >= HW_ID_DVT2)
			{
				printk("[Qwerty Keypad] K5 DVT2\n");
				surf_keypad_matrix_info.keymap = keypad_keymap_surf_DVT2;
			}
            break;
		default:
			printk(KERN_ERR "[keypad]: doesn't know the project ID\n");
		break;
	}

	
}


device_initcall(CCI_CAP_init_keypad);

