/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef __MSM_BATTERY_H__
#define __MSM_BATTERY_H__

#define NO_CHG     0x00000000
#define AC_CHG     0x00000001
#define USB_CHG    0x00000002

#define NOTIFY_BATT__FUNCTION

struct msm_psy_batt_pdata {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 avail_chg_sources;
	u32 batt_technology;
	u32 (*calculate_capacity)(u32 voltage);
};

enum {                                                                                                                           
    CCI_BATT_DEVICE_ON_RF_Max_650 = 1U << 0,
    CCI_BATT_DEVICE_ON_RF_190_550 = 1U << 1,
    CCI_BATT_DEVICE_ON_RF_130_350 = 1U << 2,
    CCI_BATT_DEVICE_ON_RF_90_200 = 1U << 3,
    CCI_BATT_DEVICE_ON_RF_100 = 1U << 4,  
    CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_50 = 1U << 5,
    CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_100 = 1U << 6,
    CCI_BATT_DEVICE_ON_LCD_BACKLIGHT_150 = 1U << 7,
    CCI_BATT_DEVICE_ON_CAMERA_150 = 1U << 8,
    CCI_BATT_DEVICE_ON_WIFI_50 = 1U << 9, 
    CCI_BATT_DEVICE_ON_VIDEO_50 = 1U << 10,
};
extern void cci_batt_device_status_update(int device, int status);

#endif
