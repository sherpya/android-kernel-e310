#ifndef __BOARD_CCI_KEYPAD_H__
#define __BOARD_CCI_KEYPAD_H__

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


static unsigned int keypad_row_gpios[] = { 35, 34, 33, 32, 31, 0, 96 };

static unsigned int keypad_col_gpios[] = { 42, 41, 40, 39, 38, 37, 36, 3, 124 };



#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

static const unsigned short keypad_keymap_surf_DVT1[ARRAY_SIZE(keypad_col_gpios) *
	ARRAY_SIZE(keypad_row_gpios)] = {
  	[KEYMAP_INDEX(0, 0)] = KEY_A,
//	[KEYMAP_INDEX(0, 1)] = KEY_A,
	[KEYMAP_INDEX(0, 2)] = KEY_SEARCH,  
	[KEYMAP_INDEX(0, 3)] = KEY_P,
	[KEYMAP_INDEX(0, 4)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(0, 5)] = KEY_EMAIL,   
//	[KEYMAP_INDEX(0, 6)] = ,
	[KEYMAP_INDEX(0, 7)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 8)] = KEY_VOLUMEDOWN,


	[KEYMAP_INDEX(1, 0)] = KEY_HOME,
	[KEYMAP_INDEX(1, 1)] = KEY_W,
//	[KEYMAP_INDEX(1, 2)] = ,
//	[KEYMAP_INDEX(1, 3)] = ,          
	[KEYMAP_INDEX(1, 4)] = KEY_BACK,//
	[KEYMAP_INDEX(1, 5)] = KEY_O,
	[KEYMAP_INDEX(1, 6)] = KEY_L,


	[KEYMAP_INDEX(2, 0)] = KEY_SEND,//
	[KEYMAP_INDEX(2, 1)] = KEY_0,    
	[KEYMAP_INDEX(2, 2)] = KEY_S,        
	[KEYMAP_INDEX(2, 3)] = KEY_I,        
	[KEYMAP_INDEX(2, 4)] = KEY_K,   
	[KEYMAP_INDEX(2, 5)] = KEY_M,
	[KEYMAP_INDEX(2, 6)] = KEY_COMMA,

    [KEYMAP_INDEX(3, 0)] = KEY_LEFTALT,
 	[KEYMAP_INDEX(3, 1)] = KEY_MENU,
	[KEYMAP_INDEX(3, 2)] = KEY_E,
	[KEYMAP_INDEX(3, 3)] = KEY_Z,      
	[KEYMAP_INDEX(3, 4)] = KEY_T,
	[KEYMAP_INDEX(3, 5)] = KEY_Y,
	[KEYMAP_INDEX(3, 6)] = KEY_U,


//	[KEYMAP_INDEX(4, 0)] = ,           
	[KEYMAP_INDEX(4, 1)] = KEY_MAIL,//          
	[KEYMAP_INDEX(4, 2)] = KEY_D,
	[KEYMAP_INDEX(4, 3)] = KEY_F,      
	[KEYMAP_INDEX(4, 4)] = KEY_X,
	[KEYMAP_INDEX(4, 5)] = KEY_H,
	[KEYMAP_INDEX(4, 6)] = KEY_J,


	[KEYMAP_INDEX(5, 0)] = KEY_COMPOSE,           
	[KEYMAP_INDEX(5, 1)] = KEY_R,          
	[KEYMAP_INDEX(5, 2)] = KEY_ADDRESSBOOK,
	[KEYMAP_INDEX(5, 3)] = KEY_C,      
	[KEYMAP_INDEX(5, 4)] = KEY_V,
	[KEYMAP_INDEX(5, 5)] = KEY_SPACE,
	[KEYMAP_INDEX(5, 6)] = KEY_N,


	[KEYMAP_INDEX(6, 0)] = KEY_LEFTSHIFT,           
	[KEYMAP_INDEX(6, 1)] = KEY_G,          
	[KEYMAP_INDEX(6, 2)] = KEY_CAMERA,
	[KEYMAP_INDEX(6, 3)] = KEY_B,      
	[KEYMAP_INDEX(6, 4)] = KEY_Q,
	[KEYMAP_INDEX(6, 5)] = KEY_DOT,
	[KEYMAP_INDEX(6, 6)] = KEY_ENTER,
	
};



static const unsigned short keypad_keymap_surf_DVT2[ARRAY_SIZE(keypad_col_gpios) *
	ARRAY_SIZE(keypad_row_gpios)] = {
//  [KEYMAP_INDEX(0, 0)] = KEY_A,
	[KEYMAP_INDEX(0, 1)] = KEY_A,
	[KEYMAP_INDEX(0, 2)] = KEY_SEARCH,  
	[KEYMAP_INDEX(0, 3)] = KEY_P,
	[KEYMAP_INDEX(0, 4)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(0, 5)] = KEY_EMAIL,   
//	[KEYMAP_INDEX(0, 6)] = ,
	[KEYMAP_INDEX(0, 7)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 8)] = KEY_VOLUMEDOWN,


	[KEYMAP_INDEX(1, 0)] = KEY_HOME,
	[KEYMAP_INDEX(1, 1)] = KEY_W,
//	[KEYMAP_INDEX(1, 2)] = ,
//	[KEYMAP_INDEX(1, 3)] = ,          
	[KEYMAP_INDEX(1, 4)] = KEY_BACK,//
	[KEYMAP_INDEX(1, 5)] = KEY_O,
	[KEYMAP_INDEX(1, 6)] = KEY_L,


	[KEYMAP_INDEX(2, 0)] = KEY_SEND,//
	[KEYMAP_INDEX(2, 1)] = KEY_0,    
	[KEYMAP_INDEX(2, 2)] = KEY_S,        
	[KEYMAP_INDEX(2, 3)] = KEY_I,        
	[KEYMAP_INDEX(2, 4)] = KEY_K,   
	[KEYMAP_INDEX(2, 5)] = KEY_M,
	[KEYMAP_INDEX(2, 6)] = KEY_COMMA,

    [KEYMAP_INDEX(3, 0)] = KEY_LEFTALT,
 	[KEYMAP_INDEX(3, 1)] = KEY_MENU,
	[KEYMAP_INDEX(3, 2)] = KEY_E,
	[KEYMAP_INDEX(3, 3)] = KEY_Z,      
	[KEYMAP_INDEX(3, 4)] = KEY_T,
	[KEYMAP_INDEX(3, 5)] = KEY_Y,
	[KEYMAP_INDEX(3, 6)] = KEY_U,


//	[KEYMAP_INDEX(4, 0)] = ,           
	[KEYMAP_INDEX(4, 1)] = KEY_MAIL,//          
	[KEYMAP_INDEX(4, 2)] = KEY_D,
	[KEYMAP_INDEX(4, 3)] = KEY_F,      
	[KEYMAP_INDEX(4, 4)] = KEY_X,
	[KEYMAP_INDEX(4, 5)] = KEY_H,
	[KEYMAP_INDEX(4, 6)] = KEY_J,


	[KEYMAP_INDEX(5, 0)] = KEY_COMPOSE,           
	[KEYMAP_INDEX(5, 1)] = KEY_R,          
	[KEYMAP_INDEX(5, 2)] = KEY_ADDRESSBOOK,
	[KEYMAP_INDEX(5, 3)] = KEY_C,      
	[KEYMAP_INDEX(5, 4)] = KEY_V,
	[KEYMAP_INDEX(5, 5)] = KEY_SPACE,
	[KEYMAP_INDEX(5, 6)] = KEY_N,


	[KEYMAP_INDEX(6, 0)] = KEY_LEFTSHIFT,           
	[KEYMAP_INDEX(6, 1)] = KEY_G,          
	[KEYMAP_INDEX(6, 2)] = KEY_CAMERA,
	[KEYMAP_INDEX(6, 3)] = KEY_B,      
	[KEYMAP_INDEX(6, 4)] = KEY_Q,
	[KEYMAP_INDEX(6, 5)] = KEY_DOT,
	[KEYMAP_INDEX(6, 6)] = KEY_ENTER,
	
};



/* SURF keypad platform device information */
static struct gpio_event_matrix_info surf_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	//.keymap		= keypad_keymap_surf,
	.output_gpios	= keypad_row_gpios,

	.input_gpios	= keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};



static struct gpio_event_info *surf_keypad_info[] = {
	&surf_keypad_matrix_info.info,
};

static struct gpio_event_platform_data surf_keypad_data = {
	.name		= "cci_qwerty",
	.info		= surf_keypad_info,
	.info_count	= ARRAY_SIZE(surf_keypad_info)
};

struct platform_device keypad_device_surf = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data	= &surf_keypad_data,
	},
};

#endif /* __BOARD_CCI_KEYPAD_H__ */
