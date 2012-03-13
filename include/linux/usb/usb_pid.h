/*
 * Copyright (C) 2010 CCI, Inc.
 * Author: Jethro Yeh
 *
 * We will define different VID/PID in different project.
 */
#ifndef	__USB_PID_H
#define	__USB_PID_H

#define USB_VID_PID_TEST	0
#define TEST_PROJECT_ID		PROJECT_ID_K4

#define VENDOR_ID_ACER		0x0502
#define VENDOR_ID_CCI		0x1219

/* CAPx project */
#define CAP_VID			VENDOR_ID_CCI
#define CAP_PID_FASTBOOT_MODE	0x0101
#define CAP_PID_ADB_ENABLE	0x0102
#define CAP_PID_ADB_DISABLE	0x0103
#define CAP_PID_RNDIS_MODE	0x0104
#define CAP_MANUFACTURER_NAME	"CCI Incorporated"		// Max 21 character
#define CAP_PRODUCT_NAME	"Android Device"		// Max 21 character

/* K4 project */
#define K4_VID			VENDOR_ID_ACER
#define K4_PID_FASTBOOT_MODE	0x3301
#define K4_PID_ADB_ENABLE	0x3302
#define K4_PID_ADB_DISABLE	0x3303
#define K4_PID_RNDIS_MODE	0x3304
#define K4_MANUFACTURER_NAME	"Acer Incorporated"
#define K4_PRODUCT_NAME		"Acer HSUSB Device"

/* C4 project */ 
#define C4_VID			VENDOR_ID_ACER
#define C4_PID_FASTBOOT_MODE	0x3306
#define C4_PID_ADB_ENABLE	0x3307
#define C4_PID_ADB_DISABLE	0x3308
#define C4_PID_RNDIS_MODE	0x3309
#define C4_MANUFACTURER_NAME	"Acer Incorporated"
#define C4_PRODUCT_NAME		"Acer HSUSB Device"

/* K5 project */ 
#define K5_VID			VENDOR_ID_ACER
#define K5_PID_FASTBOOT_MODE	0x330B
#define K5_PID_ADB_ENABLE	0x330C
#define K5_PID_ADB_DISABLE	0x330D
#define K5_PID_RNDIS_MODE	0x330E
#define K5_MANUFACTURER_NAME	"Acer Incorporated"
#define K5_PRODUCT_NAME		"Acer HSUSB Device"


#endif	/* __USB_PID_H */
