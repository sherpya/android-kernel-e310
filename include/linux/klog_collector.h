//Pochun_TKxxx_20101123_Begin
#include <generated/autoconf.h>

enum cklc_category {
	KLOG_KERNEL = 0,
	KLOG_ANDROID_MAIN,
	KLOG_ANDROID_SYSTEM,
	KLOG_ANDROID_RADIO,
/*	KLOG_ANDROID_EVENTS,*/
	/* Do not touch */
	KLOG_MAX_NUM
};

#define KLOG_IOCTLID    0xD1
#define REQUEST_KLOG_KERNEL		    _IO(KLOG_IOCTLID,     1)
#define REQUEST_KLOG_ANDROID_MAIN  	    _IO(KLOG_IOCTLID,     2)
#define REQUEST_KLOG_ANDROID_RADIO 	    _IO(KLOG_IOCTLID,     3)
#define REQUEST_PANIC		 	    _IO(KLOG_IOCTLID,     4)
#define CHECK_OLD_LOG			    _IO(KLOG_IOCTLID,     5)
#define REQUEST_KLOG_CLEAR_LOG              _IO(KLOG_IOCTLID,     7)
#define REQUEST_KLOG_ANDROID_SYSTEM 	    _IO(KLOG_IOCTLID,     12)

#ifdef CONFIG_CCI_KLOG_COLLECTOR

void cklc_append_kernel_raw_char(unsigned char c);
void cklc_append_char(unsigned int category, unsigned char c);
void cklc_append_str(unsigned int category, unsigned char *str, size_t len);
void cklc_append_newline(unsigned int category);
void cklc_append_separator(unsigned int category);
void cklc_append_time_header(unsigned int category);
void show_android_log_to_console(void);
void cklc_append_android_log(unsigned int category, const unsigned char *priority, const char * const tag, const int tag_bytes, const char * const msg, const int msg_bytes);
void cklc_save_log(void);
void cklc_set_memory_ready(void);

#else

#define cklc_append_kernel_raw_char(c)	do {} while (0)
#define cklc_append_char(category, c)	do {} while (0)
#define cklc_append_str(category, str, len)	do {} while (0)
#define cklc_append_newline(category)	do {} while (0)
#define cklc_append_separator(category)	do {} while (0)
#define cklc_append_time_header(category)	do {} while (0)
#define cklc_save_log				do {} while (0)
#define cklc_set_memory_ready			do {} while (0)

#endif
//Pochun_TKxxx_20101123_End
