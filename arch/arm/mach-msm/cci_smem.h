#ifndef CCI_SMEM_H
#define CCI_SMEM_H

#if 0 //ORIGINAL_VERSION
#else 

//@ Android team add +
//@ Aaron Chen 2010.01.21
#if 0 //bootloader = 1
#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))
#define ALIGN(x,a)		__ALIGN_MASK(x,(unsigned long int)(a)-1)

#define SMEM_DIAG_BUFFER_LEN        0xC8
#define ID_DIAG_ERR_MSG		    			0x06
#endif

/* This macro rounds an address up to be a multiple of 8 */
#define SMEM_ALIGN_SZ(VALUE) ( ( (VALUE) +7 ) & ~0x00000007 )

#define SMEM_NUM_SMD_STREAM_CHANNELS        64
#define SMEM_NUM_SMD_BLOCK_CHANNELS         64
#define SMEM_NUM_SMD_CHANNELS               64

typedef enum
{
  SMEM_MEM_FIRST,
  SMEM_PROC_COMM = SMEM_MEM_FIRST,
  SMEM_FIRST_FIXED_BUFFER = SMEM_PROC_COMM,
	SMEM_HEAP_INFO,
	SMEM_ALLOCATION_TABLE,
	SMEM_VERSION_INFO,
	SMEM_HW_RESET_DETECT,
	SMEM_AARM_WARM_BOOT,
	SMEM_DIAG_ERR_MESSAGE,
	SMEM_SPINLOCK_ARRAY,
	SMEM_MEMORY_BARRIER_LOCATION,
        SMEM_FIXED_ITEM_LAST = SMEM_MEMORY_BARRIER_LOCATION,
  SMEM_LAST_FIXED_BUFFER = SMEM_MEMORY_BARRIER_LOCATION,
	SMEM_AARM_PARTITION_TABLE,
	SMEM_AARM_BAD_BLOCK_TABLE,
	SMEM_RESERVE_BAD_BLOCKS,
	SMEM_WM_UUID,
	SMEM_CHANNEL_ALLOC_TBL,
	SMEM_SMD_BASE_ID,
	SMEM_SMEM_LOG_IDX = SMEM_SMD_BASE_ID + SMEM_NUM_SMD_STREAM_CHANNELS,
	SMEM_SMEM_LOG_EVENTS,
	SMEM_SMEM_STATIC_LOG_IDX,
	SMEM_SMEM_STATIC_LOG_EVENTS,
	SMEM_SMEM_SLOW_CLOCK_SYNC,
	SMEM_SMEM_SLOW_CLOCK_VALUE,
	SMEM_BIO_LED_BUF,
	SMEM_SMSM_SHARED_STATE,
	SMEM_SMSM_INT_INFO,
	SMEM_SMSM_SLEEP_DELAY,
	SMEM_SMSM_LIMIT_SLEEP,
	SMEM_SLEEP_POWER_COLLAPSE_DISABLED,
	SMEM_KEYPAD_KEYS_PRESSED,
	SMEM_KEYPAD_STATE_UPDATED,
	SMEM_KEYPAD_STATE_IDX,
	SMEM_GPIO_INT,
	SMEM_MDDI_LCD_IDX,
	SMEM_MDDI_HOST_DRIVER_STATE,
	SMEM_MDDI_LCD_DISP_STATE,
	SMEM_LCD_CUR_PANEL,
	SMEM_MARM_BOOT_SEGMENT_INFO,
	SMEM_AARM_BOOT_SEGMENT_INFO,
	SMEM_SLEEP_STATIC,
	SMEM_SCORPION_FREQUENCY,
	SMEM_SMD_PROFILES,
	SMEM_TSSC_BUSY,
	SMEM_HS_SUSPEND_FILTER_INFO,
	SMEM_BATT_INFO,
	SMEM_APPS_BOOT_MODE,
	SMEM_VERSION_FIRST,
	SMEM_VERSION_SMD = SMEM_VERSION_FIRST,
  SMEM_VERSION_SMD_BRIDGE,
	SMEM_VERSION_LAST = SMEM_VERSION_FIRST + 24,
	SMEM_OSS_RRCASN1_BUF1,
	SMEM_OSS_RRCASN1_BUF2,
	SMEM_ID_VENDOR0,
	SMEM_ID_VENDOR1,
	SMEM_ID_VENDOR2,
	SMEM_HW_SW_BUILD_ID,
	SMEM_SMD_BLOCK_PORT_BASE_ID,
	SMEM_SMD_BLOCK_PORT_PROC0_HEAP = SMEM_SMD_BLOCK_PORT_BASE_ID +
						SMEM_NUM_SMD_BLOCK_CHANNELS,
	SMEM_SMD_BLOCK_PORT_PROC1_HEAP = SMEM_SMD_BLOCK_PORT_PROC0_HEAP +
						SMEM_NUM_SMD_BLOCK_CHANNELS,
	SMEM_I2C_MUTEX = SMEM_SMD_BLOCK_PORT_PROC1_HEAP +
						SMEM_NUM_SMD_BLOCK_CHANNELS,
	SMEM_SCLK_CONVERSION,
	SMEM_SMD_SMSM_INTR_MUX,
	SMEM_SMSM_CPU_INTR_MASK,
	SMEM_APPS_DEM_SLAVE_DATA,
	SMEM_QDSP6_DEM_SLAVE_DATA,
	SMEM_CLKREGIM_BSP,
	SMEM_CLKREGIM_SOURCES,
	SMEM_SMD_FIFO_BASE_ID,
  SMEM_USABLE_RAM_PARTITION_TABLE = SMEM_SMD_FIFO_BASE_ID + SMEM_NUM_SMD_CHANNELS,
	SMEM_POWER_ON_STATUS_INFO,
	SMEM_DAL_AREA,
	SMEM_SMEM_LOG_POWER_IDX,
	SMEM_SMEM_LOG_POWER_WRAP,
	SMEM_SMEM_LOG_POWER_EVENTS,
	SMEM_ERR_CRASH_LOG,
	SMEM_ERR_F3_TRACE_LOG,
	SMEM_SMD_BRIDGE_ALLOC_TABLE,
	SMEM_SMDLITE_TABLE,
	SMEM_SD_IMG_UPGRADE_STATUS,
	SMEM_SEFS_INFO,
	SMEM_RESET_LOG,
	SMEM_RESET_LOG_SYMBOLS,
	SMEM_CCI_SMEM_VALUE,
	SMEM_MEM_LAST = SMEM_CCI_SMEM_VALUE,
	SMEM_NUM_ITEMS,
	SMEM_FIRST_VALID_TYPE = SMEM_MEM_FIRST,
	SMEM_LAST_VALID_TYPE = SMEM_NUM_ITEMS,
} smem_mem_type;

typedef enum
{
//PVCS ID: Modem_Android_7x27 421
    E_SD_DOWNLOAD_NO_ERROR,
    E_SD_DOWNLOAD_IMG_UPDATE_QCSBL,         /* Start to update QCSBL    */
    E_SD_DOWNLOAD_IMG_UPDATE_OEMSBL,        /* Start to update OEMSBL   */
    E_SD_DOWNLOAD_IMG_UPDATE_AMSS,          /* Start to update AMSS     */
    E_SD_DOWNLOAD_IMG_UPDATE_APPSBL,        /* Start to update APPSBL   */
    E_SD_DOWNLOAD_IMG_UPDATE_APPS,
    E_SD_DOWNLOAD_IMG_UPDATE_FAIL_QCSBL,
    E_SD_DOWNLOAD_IMG_UPDATE_FAIL_OEMSBL,
    E_SD_DOWNLOAD_IMG_UPDATE_FAIL_AMSS,
    E_SD_DOWNLOAD_IMG_UPDATE_FAIL_APPSBL,
    E_SD_DOWNLOAD_IMG_UPDATE_FAIL_APPS,
    E_SD_DOWNLOAD_APP_REQUEST_HW_RESET,     /* HW reset */
    E_SD_DOWNLOAD_INIT,                     /* SD Download init. */
    E_SD_DOWNLOAD_BATTERY_CHECK,            /* start to check battery level */
    E_SD_DOWNLOAD_BATTERY_LEVEL_LOW,        /* battery level is too low */
    E_SD_DOWNLOAD_IMG_CHECK,                /* start to check images that SD Download need */
    E_SD_DOWNLOAD_IMG_NOT_FOUND,            /* Image in SD card not exit or open file fail */
    E_SD_DOWNLOAD_IMG_SIZE_ZERO,            /* Image size that read from SD card is Zero */
    E_SD_DOWNLOAD_IMG_CHECK_FAIL,           /* Image check fail(unknow fail) */
    E_SD_DOWNLOAD_UPDATE_APPS,              /* Start to do Android update */
    E_SD_DOWNLOAD_MODEM_UPGRADE_DONE,       /* modem(Re-partition/QCSBL/OEMSBL/AMSS/APPSBL) update success */
    E_SD_DOWNLOAD_RECOVERY_IMG_WRITTING,    /* Start to write recovery.img to temp partition(FOTA) for Android update */
    E_SD_DOWNLOAD_UPDATE_FAIL,              /* Update QCSBL/OEMSBL/AMSS/APPSBL fail */
    E_SD_DOWNLOAD_REPARTITION,              /* Re-Partition */
    E_SD_DOWNLOAD_REPARTITION_FAIL,         /* Re-Partition fail */
    E_SD_DOWNLOAD_AUTH_FAIL,                /* image authentication fail */
    E_SD_DOWNLOAD_READ_FLH_PARTI_FAIL,      /* Get User partition table fail */
    E_SD_DOWNLOAD_MID_CHECK_FAIL,           /* Model ID in QCSBL/OEMSBL image is not match with target */
    //PVCS ID: Modem_Android_7x27 421, end
    //PVCS ID: Modem_Android_7x27 438
    E_SD_DOWNLOAD_READ_FLH_QCSBLHD_FAIL,    /* Read QCSBLHD of target fail */
    E_SD_DOWNLOAD_READ_FLH_QCSBL_FAIL,      /* Read QCSBL of target fail */
    E_SD_DOWNLOAD_READ_FLH_OEMSBL_FAIL,     /* Read OEMSBL of target fail */
    E_SD_DOWNLOAD_SW_VERSION_IS_TOO_OLD,    /* SD Download and Hynix flash check is supported in QCT 5110 codebase or later */
    E_SD_DOWNLOAD_FLH_DEVICE_NAME_NOT_FOUND,/* Device Name of Flash not found */
    E_SD_DOWNLOAD_GET_SBL_IMAGE_SLV_FAIL,   /* Get security level(image) of QCSBL/OEMSBL fail */
    E_SD_DOWNLOAD_GET_SBL_TARGET_SLV_FAIL,  /* Get security level(target) of QCSBL/OEMSBL fail */
    E_SD_DOWNLOAD_SLV_COMPARE_FAIL,         /* security level between img and target are different */
    E_SD_DOWNLOAD_PARAM_INVALID,            /* parameters of QCSBL/OEMSBL is invalid */
    E_SD_DOWNLOAD_IMG_UPDATE_UNKNOWN,       /* Start to update image (undefine image type)*/
    E_SD_DOWNLOAD_RECOVERY_IMG_WRITE_FAIL,  /* Write recovery.img to temp partition(FOTA) fail*/
    //PVCS ID: Modem_Android_7x27 438, end
    E_SD_DOWNLOAD_ERR_TYPE_INVALID = 0xFFFFFFFF
} E_SD_DOWNLOAD_ERR_TYPE;
//@ Android team add -
//@ Aaron Chen 2010.01.21

typedef enum {

    CCI_CHG_STATE_NONE = 0x00,      /* no charger */
    CCI_CHG_STATE_CHARGING,         /* battery is charging */
    CCI_CHG_STATE_COMPLETE,         /* charging complete */
    CCI_CHG_STATE_ERR_TEMP_STOP_CHGARGING,	/* battery too hot or cold and stop charging */
    CCI_CHG_STATE_ERR_TEMP_STILL_CHGARGING,	/* battery too hot or cold but still charging */
    CCI_CHG_STATE_ERR_CHARGING,     /* charging abnormal, use weak current to charge battery. eg. weak battery, bad charger, weak charger */
    CCI_CHG_STATE_ERR_STOP,         /* charging abnormal, stop charging battery. eg. bad battery */
    CCI_CHG_STATE_INVALID = 0xFFFFFFFF    /* invalid charging state */

}cci_charging_state;


typedef enum{
	CCI_VALID_BATTERY,
	CCI_INVALID_BATTERY,
	CCI_SKIP_BATTERY_CHECK,
	CCI_BATTERY_CRITICAL,
	CCI_BATTERY_POWERDOWN,
	CCI_BATTERY_LOW,
	CCI_BATTERY_INFO_INVALID = 0xFFFFFFFF
}cci_battery_info_type; 


// CCI Modem_L1 Cloud Add_Begin
typedef enum {
	CCI_BATT_TEMP_OK,						// Battery temperature is ok
	CCI_BATT_TEMP_ERROR_LV0,				// Battery temperature is error, stop charging
	CCI_BATT_TEMP_ERROR_LV1,				// Battery temperature is error, set charging current to 500mA
	CCI_BATT_TEMP_ERROR_LV2,				// Battery temperature is error, set charging current to 100mA
	CCI_BATT_TEMP_INVALID = 0xFFFFFFFF	// Invalid value
}cci_battery_temperature_info_type;
// CCI Modem_L1 Cloud Add_End

typedef enum {
    CCI_CHG_SET_IDLE_MODE,
    CCI_CHG_SET_CC_MODE,
    CCI_CHG_SET_CV_MODE,
    CCI_CHG_SET_MODE_INVALID = 0xFFFFFFFF            /* invalid RPC state */
}cci_chg_current_state;

typedef enum{
    CCI_SET_AC,                             //  set AC current, 1A
    CCI_SET_USB,                           //  set USB current, 500mA
    CCI_SET_KITL,					//  set USB current 400mA for KIPL test		//PVCS ID:modem_7k TK671 [1/8]  AV30 V0.0.28  Pierce 090706
    CCI_SET_RE_EMULATION,		//  set USB current 100mA for re-emulation	//PVCS ID:modem_7k TK671
    CCI_SET_INVALID = 0xFFFFFFFF           //  invalid value
}cci_set_power_source_type;


typedef enum{
	CCI_FACTORY_CHARGING_COMPLETE,
	CCI_FACTORY_CHARGING_UN_COMPLETE,
	CCI_FACTORY_CHARGING_INFO_INVALID = 0xFFFFFFFF
}cci_factory_charging_info_type;


/*Bootup Condition*/
typedef enum
{
 	CCI_DOWNLOAD_ANDROID_IMAGE,
	CCI_UN_DOWNLOAD_ANDROID_IMAGE,
	CCI_DOWNLODA_ANDROID_IMAGE_UNKONW = 0xFFFFFFFF
	
} cci_android_image_download_check_config;

 /* Modem Specified Miode */
typedef enum {

	CCI_MODEM_NORMAL_POWER_ON_MODE,
	CCI_MODEM_CHARGING_ONLY_MODE,
	CCI_MODEM_FTM_MODE,
	CCI_MODEM_ARM9_FTM_MODE,
	CCI_MODEM_DOWNLOAD_MODE,
	CCI_MODEM_EFS_FORMAT_MODE,
	CCI_MODEM_FTM_WIFI_MODE,
	CCI_MODEM_DL_BATT_PROFILE_MODE,
	CCI_MODEM_CFC_TEST_MODE,
	CCI_MODEM_ANDROID_DL_MODE,
	CCI_MODEM_SD_DL_MODE,
	CCI_MODEM_ACER_FOTA_OS_UG_MODE,
	CCI_MODEM_FERRARI_CHECK_FAIL,
	CCI_MODEM_SD_DL_MODE_WITH_SKU_CHECK,
	CCI_MODEM_UNKNOW_MODE = 0xFFFFFFFF
	
} cci_modem_boot_mode_config;

 /*Power-On-Down Event*/
//[AB60_TK401] Bryan:Implement new power code for low battery poweroff 
//[AB60_TK348] Bryan:Implement power on/down code
 typedef enum {
	CCI_POWER_ON_DOWN_FROM_POWER_KEY=1,
	CCI_POWER_ON_DOWN_FROM_CHARGER, // 2
	CCI_POWER_ON_DOWN_FROM_POWER_CUT, // 3
	CCI_POWER_ON_DOWN_FROM_CHARGER_REBOOT, // 4
	CCI_POWER_ON_DOWN_FROM_SW_RESET, // 5
	CCI_POWER_ON_DOWN_FROM_HW_RESET, // 6
	CCI_POWER_ON_DOWN_FROM_DOWNLOAD, // 7
	CCI_POWER_ON_DOWN_FROM_FTM, // 8
	CCI_POWER_ON_DOWN_FROM_EFS_FORMAT, // 9
	CCI_POWER_ON_DOWN_FROM_ARM9_FATAL_ERROR, // 10
	CCI_POWER_ON_DOWN_FROM_WDOG, // 11
	CCI_POWER_ON_DOWN_FROM_ARM9_FTM, // 12
	CCI_POWER_ON_DOWN_FROM_CFC_TEST_MODE, // 13
	CCI_POWER_ON_DOWN_FROM_BATT_REMOVE, // 14 //"BATT_REMOVE" and "Non-SW Power Down are the same reason"
	CCI_POWER_ON_DOWN_FROM_PMIC_RTC, // 15
	CCI_POWER_ON_DOWN_FROM_ANDROID_DL, // 16
	CCI_POWER_ON_DOWN_FROM_LOW_BATT, // 17
	CCI_POWER_ON_DOWN_FROM_OTHER, // 18
	CCI_POWER_ON_DOWN_FROM_UNKOWN, // 19
	CCI_POWER_ON_DOWN_FROM_CHARGER_IC_DETECT_REMOVE, // 20	
	CCI_POWER_ON_DOWN_FROM_MAX=21,		
	CCI_POWER_ON_DOWN_FROM_INVALID = 0xFFFFFFFF
} cci_power_on_down_event_config;
//[AB60_TK348] End
//[AB60_TK401] End

 /*ANDROID Specified Mode*/
 typedef enum {
	CCI_ANDROID_UNLOAD_MODE,
	CCI_ANDROID_NORMAL_MODE,
	CCI_ANDROID_CHARGING_ONLY_MODE,
	CCI_ANDROID_FTM_WIFI_MODE,
	CCI_ANDROID_CFC_TEST_MODE,
	CCI_ANDROID_DL_MODE,
	CCI_ANDROID_MENU_MODE,
	CCI_ANDROID_UNKNOW_MODE = 0xFFFFFFFF
} cci_android_boot_mode_config;


typedef enum {
	CCI_HW_RF_BAND_IS_EU 			= 0x1,		/* Bit 0~3 are for RF band */
	CCI_HW_RF_BAND_IS_US 			= 0x2,
	CCI_HW_BOARD_VERSION_BEFORE	= 0x10,	/* Bit 4~7 are for board version */
	CCI_HW_BOARD_VERSION_DVT1_4	= 0x20,
	CCI_HW_BOARD_VERSION_1       = 0x10,                    //HW ID0 GPIO83 detect high (EVT)
  CCI_HW_BOARD_VERSION_2       = 0x20,                    //HW ID0 GPIO83 detect low	(DVT1) and HW ID1 GPIO(N/A) detect high
  CCI_HW_BOARD_VERSION_3       = 0x30,                    //HW ID0 GPIO83 detect low         and HW ID1 GPIO(N/A) detect low  HW ID2 GPIO(N/A) detect high 
  CCI_HW_BOARD_VERSION_4       = 0x40,                    //HW ID0 GPIO83 detect low 				 and HW ID1 GPIO(N/A) detect low  HW ID2 GPIO(N/A) detect low
  CCI_HW_BOARD_VERSION_NONE    = 0xF0,          //no HW ID GPIO detect circuit
  	CCI_HW_LQ86_EVT					= 0x000,		//LQ86 EVT1 TK337 LQ86 090601 V1.02
  	CCI_HW_LQ86_DVT1_1				= 0x110,		//LQ86 DVT1-1 GPIO90 Low TK337 LQ86 090601 V1.02
  	CCI_HW_LQ86_DVT1_2				= 0x210,		//LQ86 DVT1-2 GPIO90 High TK337 LQ86 090601 V1.02
  	CCI_HW_LQ86_DVT2_1				= 0x120,		//LQ86 DVT2-1 TK337 LQ86 090601 V1.02
  	CCI_HW_LQ86_DVT2_2				= 0x220,		//LQ86 DVT2-2 TK337 LQ86 090601 V1.02
	CCI_HW_VERSION_INFO_INVALID = 0xFFFFFFFF
} cci_hw_version_info_type;

// TK ID:
// Project: Penguin Lite
// Author: Arthur Hong
// Description: Add HW ID symbol.
// Date: 2010/07/01
typedef enum {
   	HW_ID_EMU,

  	HW_ID_EVT0,
   	HW_ID_EVT1,
   	HW_ID_EVT2,

   	HW_ID_DVT1,
   	HW_ID_DVT2,
   	HW_ID_DVT3,

   	HW_ID_PVT,

	HW_ID_DVT2_2,			// Arthur Hong, Modem_Android_7x27, TK830, Add K4 DVT2-2 GPIO table, 2010/09/17.
	HW_ID_DVT1_2,			// Arthur Hong, Modem_Android_7x27, TK848, Add CAP6 PVT1, K5 DVT1-2 and CAP2 DVT1-2 GPIO table, 2010/09/29.
	HW_ID_PVT1,			// Arthur Hong, Modem_Android_7x27, TK848, Add CAP6 PVT1, K5 DVT1-2 and CAP2 DVT1-2 GPIO table, 2010/09/29 end.

	HW_ID_2ND_ID_PVT,

	HW_ID_DVT2_3,			// Arthur Hong, Modem_Android_7x27, TK965, Add C4 DVT2-3 GPIO table, 2010/11/18.

	HW_ID_INVALID = 0xFF
} cci_hw_id_type;

typedef enum {
	PROJECT_ID_K4,
	PROJECT_ID_K4H,
	PROJECT_ID_K5,
	PROJECT_ID_CAP6,
	PROJECT_ID_CAP8,
	PROJECT_ID_CAP2,	// Added for dual-SIM.

	PROJECT_ID_INVALID = 0xFF
} cci_project_id_type;
// End of TK ID:

typedef enum {
	WCDMA_US_125=0,
	WCDMA_EU_128=0x10,
	EDGE=0x20,
	BAND_ID_INVALID = 0xFF
} cci_band_id_type;


// 081017 L1 Leon add for download incomplete: START 
typedef enum{
	CCI_DM_SUCEESS,
	CCI_DM_QCSBLHD_FAIL,
	CCI_DM_QCSBL_FAIL,
	CCI_DM_OEMSBL_FAIL,
	CCI_DM_AMSS_FAIL,
	CCI_DM_APPSBL_FAIL,
	CCI_DM_APPS_FAIL,
	CCI_DM_FLASH_BIN_FAIL,
	CCI_DM_FLEX_FAIL,
	CCI_DM_ABOOT_FAIL,
	CCI_DM_ASYSTEM_FAIL,
	CCI_DM_AFLEX_FAIL,
	CCI_DM_ARECOVERY_FAIL,
	CCI_DM_AUSERDATA_FAIL,
	CCI_DM_INVALID = 0xFFFFFFFF
}cci_download_check_condition;
// 081017 L1 Leon add for download incomplete: END

// CCI Modem_L1 Duke 080904 Add Begin
typedef enum {

    CCI_RPC_NOT_READY = 0x00,      	/* ANDROID RPC Not Ready */
	CCI_RPC_READY, 	 				/* ANDROID RPC Ready */		
    CCI_RPC_INVALID = 0xFFFFFFFF    	/* invalid RPC state */

}cci_rpc_state;
// CCI Modem_L1 Duke 080904 Add End


//[AB60_TK170] Add power status of arm11 into CCI_SMEM (Owner/Reviewer:Bryan_Kuo)
/* System States */
typedef enum
{
  SMEM_UNKNOWN                    = (int)0x80000000,

  SMEM_INIT                       = 0x0001,
        /* SMSM shared state data initialized, SMSM ISRs installed.
         */
 
  SMEM_RUN                        = 0x0100,
        /* Either processor will enter this state when
         * SMD and RPC are running on both processors.
         */
 
  SMEM_SLEEP                      = 0x4000,
        /* This state is entered by aARM upon decidign that TCXO
         * shutdown can be performed by the mARM.
         */
 
  SMEM_APPS_REBOOT                = 0x20000,
        /* This state is entered by aARM when it wants to reboot.
         */
 
  SMEM_SYSTEM_DOWNLOAD            = 0x100000,
        /* This state is entered by the aARM when it wants the modem to
         * enter download mode
         */
 
  SMEM_PWRC_SUSPEND               = 0x200000,
        /* aARM power collapse enters this state when entering suspend.
         */
 
  SMEM_APPS_SHUTDOWN              = 0x400000,
        /* This state is entered by mARM upon receiving system reset or
         * powerdown in TMC.
         */
} cci_arm11_power_status_type;
//[AB60_TK170] End

/* This struct will be used by the AMSS to enable reset detection   */
typedef struct
{
  unsigned int magic_1;
  unsigned int magic_2;
} smem_hw_reset_id_type;


// TK ID: Modem_Android_7x27, TK872
// Project: 5 in 1
// Author: Arthur Hong
// Description: Implement PCB version table API
// Date: 2010/10/11
typedef enum
{
	PCB_ID_01,		// CAP2 DVT
	PCB_ID_02,		// K4 EVT
	PCB_ID_03,		// K4 DVT1
	PCB_ID_04,		// K4 DVT2-1
	PCB_ID_05,		// K4 DVT2-2

	PCB_ID_06,		// Not uesd.
	PCB_ID_07,		// Not uesd.
	PCB_ID_08,		// Not uesd.
	PCB_ID_09,		// CAP6 EVT
	PCB_ID_10,		// CAP6 DVT

	PCB_ID_11,		// CAP6 PVT
	PCB_ID_12,		// CAP6 PVT1
	PCB_ID_13,		// Not uesd.
	PCB_ID_14,		// Not uesd.
	PCB_ID_15,		// CAP8 DVT

	PCB_ID_16,		// CAP8 PVT
	PCB_ID_17,		// CAP8 2nd PVT
	PCB_ID_18,		// Not uesd.
	PCB_ID_19,		// Not uesd.
	PCB_ID_20,		// C4 EVT

	PCB_ID_21,		// C4 DVT
	PCB_ID_22,		// Not uesd.
	PCB_ID_23,		// C4 PVT
	PCB_ID_24,		// Not uesd.
	PCB_ID_25,		// Not uesd.

	PCB_ID_26,		// K5 DVT2
	PCB_ID_27,		// Not uesd.
	PCB_ID_28,		// K5 PVT
	PCB_ID_29,		// K5 DVT1
	PCB_ID_30,		// K5 DVT1-2

	PCB_ID_31,		// K5 EVT
	PCB_ID_32,		// CAP2 DVT1-2

	PCB_ID_INVALID = 0x7fffffff
} cci_pcb_id_type;
// End of TK ID: Modem_Android_7x27, TK872


typedef struct 
{
    cci_android_image_download_check_config   cci_android_image_download_check_flag;
    cci_power_on_down_event_config            cci_power_on_event;
    cci_power_on_down_event_config            cci_power_down_event;
    cci_modem_boot_mode_config                cci_modem_boot_mode;
    cci_android_boot_mode_config              cci_android_boot_mode;
    cci_battery_info_type                     cci_battery_info;
    cci_hw_version_info_type                  cci_hw_version_info;
    char                                      modem_version[20]; //[AB60_TK441] Bryan:Enlarge length of software version string
    char                                      os_version[10];
    char                                      language_id[10];
    cci_rpc_state                             cci_android_rpc_state;
    cci_download_check_condition              download_check_flag; // 081017 L1 Leon add for download incomplete
    cci_charging_state                        cci_modem_charging_state;
    cci_factory_charging_info_type            cci_factory_charging_check;
    short                                     cci_factory_fast_charge_value;
    short                                     cci_chg_i;
    cci_chg_current_state                     cci_set_chg_state;
    unsigned char                             cci_chg_percentage;
    unsigned char                            	cci_hw_id;
    unsigned char                          	cci_band_id;
    unsigned char                       	cci_project_id;

    // Arthur Hong, Modem_Android_7x27, TK947, Add cci_pcb_id into the share memory, 2010/11/09.
    cci_pcb_id_type cci_pcb_id;
   // Arthur Hong, Modem_Android_7x27, TK947, Add cci_pcb_id into the share memory, 2010/11/09 end.

    cci_arm11_power_status_type               cci_arm11_power_status; //[AB60_TK170] Add power status of arm11 into CCI_SMEM (Owner/Reviewer:Bryan_Kuo)
    //[AB60_TK195] Bryan_Kuo: Get target serial number from NV and write it into SMEM 
    char                                      cci_serial_number[12];  //[AB60_TK195] Bryan_Kuo: Get target serial number from NV and write it into SMEM
    //[AB60_TK195] End
    //PVCS ID: Modem_Android_7x27 245
    unsigned char                             cci_sd_dload_percentage;
    //PVCS ID: Modem_Android_7x27 245, end
    //[AB60_TK281] Bryan_Kuo: Add customer version into smem
    char                                      cust_modem_version[20]; //[AB60_TK441] Bryan:Enlarge length of software version string
    //[AB60_TK281] End

//Modem_Android_7x27 [TK363] by Jeremy 20100119
//Desc: Hynix patch for solving some issues
    unsigned int                              maker_id;    /**< Maker Identification */ //20100107 distinguish different types of flash
    unsigned int                              device_id;   /**< Device Identification */ //20100107 distinguish different types of flash
    unsigned char *			      cci_boot_fb_addr;			// Add frame buffer address in bootloader
	unsigned char								  cci_msm_hw_id[8];
	char                                                cci_mid[16];
	unsigned int   operator_ID;
	unsigned int   touch_firmware_revision;
	unsigned int   touch_firmware_version;
	char	cci_imei_num[15];
	char	cci_imei_num_ready;
} cci_smem_value_t;

// Arthur Hong, add project IDs, 2010/07/05
cci_project_id_type cci_read_project_id( void );
// Arthur Hong, add project IDs, 2010/07/05 end.

#endif
#endif
