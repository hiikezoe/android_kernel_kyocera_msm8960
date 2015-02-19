/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <resetlog.h>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2012 KYOCERA Corporation
(C) 2013 KYOCERA Corporation

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifndef _RESET_LOG_H
#define _RESET_LOG_H

#define HEADER_VERION           "v1.0.2"
#define HEADER_VERION_SIZE      (8)

#define MSM_KCJLOG_BASE         (MSM_UNINIT_RAM_BASE)
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SIZE_RAM_CONSOLE        (CONFIG_ANDROID_RAM_CONSOLE_SIZE)
#else
#define SIZE_RAM_CONSOLE        (0x00080000)
#endif

#define SIZE_SMEM_ALLOC         (512)

#define SIZE_KERNEL_LOG         (SIZE_RAM_CONSOLE - sizeof(ram_console_header_type))
#define SIZE_CONTROL_INFO       (SIZE_SMEM_ALLOC)
#define SIZE_LOGCAT_MAIN        (256 * 1024)
#define SIZE_LOGCAT_SYSTEM      (256 * 1024)
#define SIZE_LOGCAT_EVENTS      (256 * 1024)
#define SIZE_LOGCAT_RADIO       (256 * 1024)
#define SIZE_SMEM_EVENT_LOG     ( 50 * 1024)

#define SIZE_MODEM_F3_LOG       (128 * 1024)
#define SIZE_MODEM_ERR_DATA_LOG ( 16 * 1024)

#define SIZE_EXTENSION_INFO     (sizeof(ram_log_extension_info_type))
#define SIZE_EXTENSION_LOG      (sizeof(ram_log_extension_record_type)*EXTENSION_LOG_API_MAX)

#define SIZE_CPU_CONTEXT_INFO   (  1 * 1024)

#define ADDR_KERNEL_LOG         (unsigned long)(&((ram_console_type *)MSM_KCJLOG_BASE)->msg[0])
#define ADDR_CONTROL_INFO       (MSM_KCJLOG_BASE    + SIZE_RAM_CONSOLE  )
#define ADDR_LOGCAT_MAIN        (ADDR_CONTROL_INFO  + SIZE_CONTROL_INFO )
#define ADDR_LOGCAT_SYSTEM      (ADDR_LOGCAT_MAIN   + SIZE_LOGCAT_MAIN  )
#define ADDR_LOGCAT_EVENTS      (ADDR_LOGCAT_SYSTEM + SIZE_LOGCAT_SYSTEM)
#define ADDR_LOGCAT_RADIO       (ADDR_LOGCAT_EVENTS + SIZE_LOGCAT_EVENTS)
#define ADDR_SMEM_EVENT_LOG     (ADDR_LOGCAT_RADIO  + SIZE_LOGCAT_RADIO )

#define ADDR_MODEM_F3_LOG       (ADDR_SMEM_EVENT_LOG + SIZE_SMEM_EVENT_LOG )
#define ADDR_MODEM_ERR_DATA_LOG (ADDR_MODEM_F3_LOG   + SIZE_MODEM_F3_LOG   )

#define ADDR_EXTENSION_INFO     (ADDR_MODEM_ERR_DATA_LOG + SIZE_MODEM_ERR_DATA_LOG)
#define ADDR_EXTENSION_LOG      (ADDR_EXTENSION_INFO     + SIZE_EXTENSION_INFO    )

#define ADDR_CPU_CONTEXT_INFO   (ADDR_EXTENSION_LOG      + SIZE_EXTENSION_LOG )

enum {
	LOGGER_INFO_MAIN,
	LOGGER_INFO_SYSTEM,
	LOGGER_INFO_EVENTS,
	LOGGER_INFO_RADIO,
	LOGGER_INFO_MAX,
};

#define CRASH_MAGIC_CODE         "KC ERROR"

#define CRASH_SYSTEM_KERNEL      "KERNEL"
#define CRASH_SYSTEM_MODEM       "MODEM"
#define CRASH_SYSTEM_RIVA        "RIVA"
#define CRASH_SYSTEM_LPASS       "LPASS"
#define CRASH_SYSTEM_ANDROID     "ANDROID"
#define CRASH_SYSTEM_UNKNOWN     "UNKNOWN"

#define CRASH_KIND_PANIC         "KERNEL PANIC"
#define CRASH_KIND_FATAL         "ERR FATAL"
#define CRASH_KIND_EXEPTION      "EXEPTION"
#define CRASH_KIND_WDOG_HW       "HW WATCH DOG"
#define CRASH_KIND_WDOG_SW       "SW WATCH DOG"
#define CRASH_KIND_SYS_SERVER    "SYSTEM SERVER CRASH"
#define CRASH_KIND_UNKNOWN       "UNKNOWN"


/* RAM_CONSOLE Contol */
typedef struct {
    unsigned char               magic[4];       // RAM_CONSOLE MAGIC("DEBG")
    unsigned long               start;          // RAM_CONSOLE start
    unsigned long               size;           // RAM_CONSOLE size
} ram_console_header_type;

typedef struct {
    ram_console_header_type     header;         // RAM_CONSOLE header
    unsigned char               msg[1];         // RAM_CONSOLE message
} ram_console_type;

/* Log Control */
struct logger_log_info {
	unsigned long           w_off;
	unsigned long           head;
};

#define CRASH_CODE_SIZE         (24)
#define CRASH_TIME_SIZE         (64)
#define VERSION_SIZE            (64)
#define MODEL_SIZE              (32)
#define CRASH_INFO_DATA_SIZE    (32)
#define PET_TIME_SIZE           (36)

typedef struct {
	unsigned char           magic_code[CRASH_CODE_SIZE];
	unsigned char           crash_system[CRASH_CODE_SIZE];
	unsigned char           crash_kind[CRASH_CODE_SIZE];
	unsigned char           crash_time[CRASH_TIME_SIZE];
	unsigned char           linux_ver[VERSION_SIZE];
	unsigned char           modem_ver[VERSION_SIZE];
	unsigned char           model[MODEL_SIZE];
	struct logger_log_info  info[LOGGER_INFO_MAX];
	unsigned long           pErr_F3_Trace_Buffer;
	unsigned long           Sizeof_Err_F3_Trace_Buffer;
	unsigned long           Err_F3_Trace_Buffer_Head;
	unsigned long           Err_F3_Trace_Wrap_Flag;
	unsigned long           pErr_Data;
	unsigned long           Sizeof_Err_Data;
	unsigned long           pSmem_log_events;
	unsigned long           pSmem_log_write_idx;
	unsigned long           Sizeof_Smem_log;
	unsigned char           crash_info_data[CRASH_INFO_DATA_SIZE];
	unsigned char           panic_info_data[CRASH_INFO_DATA_SIZE];
	unsigned char           pet_time[PET_TIME_SIZE];
	unsigned long           regsave_addr;
	unsigned long           regsave_addr_check;
} ram_log_info_type;

#define EXTENSION_LOG_DATA_SIZE    (32)
#define EXTENSION_LOG_RECORD_MAX   (200)
#define EXTENSION_LOG_API_MAX      (32)

typedef struct {
    unsigned char           magic_code[CRASH_CODE_SIZE];
    unsigned char           linux_ver[VERSION_SIZE];
    unsigned char           modem_ver[VERSION_SIZE];
    unsigned char           model[MODEL_SIZE];
    unsigned char           base_time[CRASH_TIME_SIZE];
    unsigned long           base_sec_uptime;
    unsigned long           base_msec_uptime;
} ram_log_extension_info_type;

typedef struct {
	unsigned long           sec_uptime;
	unsigned long           msec_uptime;
	unsigned char           cpu_id;
	unsigned char           data[EXTENSION_LOG_DATA_SIZE];
} ram_log_extension_data_type;

typedef struct {
    unsigned char                   magic_code[CRASH_CODE_SIZE];
	unsigned int                    index;
	ram_log_extension_data_type     record_data[EXTENSION_LOG_RECORD_MAX];
} ram_log_extension_record_type;



#endif /* _RESET_LOG_H */
