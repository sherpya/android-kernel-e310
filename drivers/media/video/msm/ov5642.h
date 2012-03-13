/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 *
 */

#ifndef CAMSENSOR_OV5642
#define CAMSENSOR_OV5642

#include <mach/board.h>

/*Register Define*/
/*Register Define*/
#define	PIDH				0x300A
#define	PIDL				0x300B
#define	SCCB_ID				0x300C
#define	CLK               			0x3011
#define	SYS 				0x3012
#define	AUTO_1				0x3013
#define	ALG_1				0x3013
#define	AUTO_2				0x3014
#define	ALG_2				0x3014
#define	AUTO_3				0x3015
#define	ALG_3				0x3015
#define	AUTO_4				0x3016
#define	ALG_4				0x3016
#define	AUTO_5				0x3017
#define	ALG_5				0x3017
#define	WPT_HISH			0x3018
#define	alg_64				0x3018
#define	BPT_HISL			0x3019
#define	alg_72				0x3019
#define	VPT					0x301A
#define	alg_80				0x301A
#define	YAVG				0x301B
#define	AECG_MAX50_ALG	0x301C
#define	AECG_MAX60_ALG	0x301D
#define	HS_8				0x3020
#define	HS_0				0x3021
#define	VS_8				0x3022
#define	VS_0				0x3023
#define	HW_8				0x3024
#define	HW_0				0x3025
#define	VH_8				0x3026
#define	VH_0				0x3027
#define	HTS_8				0x3028
#define	HTS_0				0x3029
#define	VTS_8				0x302A
#define	VTS_0				0x302B
#define	EXHTS_0			0x302C
#define	EXVTS_8			0x302D
#define	EXVTS_0			0x302E
#define	WEIGHT0			0x3030
#define	BD50_0_ALG_104	0x3070
#define	BD50_8_ALG_112	0x3071
#define	BD60_0_ALG_120	0x3072
#define	BD60_8_ALG_128	0x3073
#define	TMC0				0x3076
#define	TMC1				0x3077
#define	TMC4				0x307A
#define	TMC6				0x307C
#define	TMC8				0x307E
#define	TMC_i2c				0x3084
#define	TMC11				0x3087
#define	ISP_XOUT_8			0x3088
#define	ISP_XOUT_0			0x3089
#define	ISP_YOUT_8			0x308A
#define	ISP_YOUT_0			0x308B
#define	TMC12				0x308C
#define	TMC13				0x308D
#define	IO_CTRL0			0x30B0
#define	IO_CTRL1			0x30B1
#define	IO_CTRL2			0x30B2
#define	FMT_CTRL01			0x3401
#define	FMT_CTRL02			0x3402
#define	FMT_CTRL03			0x3403
#define	FMT_CTRL04			0x3404
#define	FMT_CTRL06			0x3406
#define	FMT_CTRL07			0x3407
#define	FMT_CTRL08			0x3408
#define	DITHER_CTRL0		0x3409


#define	SDE_BASE			0x3390
#define   WB_MODE_SW		0x3306


#define   OV5642_SHARPNESS_OFF			0
#define   OV5642_SHARPNESS_POSITIVE_1	1
#define   OV5642_SHARPNESS_POSITIVE_2	2
#define   OV5642_SHARPNESS_POSITIVE_3	3
#define   OV5642_SHARPNESS_POSITIVE_4	4
#define   OV5642_SHARPNESS_POSITIVE_5	5
#define   OV5642_SHARPNESS_MAX			6


typedef  struct
{
   uint16_t	addr;
   uint8_t		val;
}CAM_REG_ADDR_VAL_TYPE;

#endif /* CAMSENSOR_OV5642 */
