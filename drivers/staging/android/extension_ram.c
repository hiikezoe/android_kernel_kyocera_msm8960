/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <extension_ram.c>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2012 KYOCERA Corporation

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

#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smsm.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/sched.h>

#include "resetlog.h"

enum {
    EXTENSION_DATA_00 = 0,
    EXTENSION_DATA_01,EXTENSION_DATA_02,EXTENSION_DATA_03,EXTENSION_DATA_04,EXTENSION_DATA_05,
    EXTENSION_DATA_06,EXTENSION_DATA_07,EXTENSION_DATA_08,EXTENSION_DATA_09,EXTENSION_DATA_10,
    EXTENSION_DATA_11,EXTENSION_DATA_12,EXTENSION_DATA_13,EXTENSION_DATA_14,EXTENSION_DATA_15,
    EXTENSION_DATA_16,EXTENSION_DATA_17,EXTENSION_DATA_18,EXTENSION_DATA_19,EXTENSION_DATA_20,
    EXTENSION_DATA_21,EXTENSION_DATA_22,EXTENSION_DATA_23,EXTENSION_DATA_24,EXTENSION_DATA_25,
    EXTENSION_DATA_26,EXTENSION_DATA_27,EXTENSION_DATA_28,EXTENSION_DATA_29,EXTENSION_DATA_30,
    EXTENSION_DATA_31
};
#ifdef CONFIG_ANDROID_EXTENSION_RAM

#define EXTENSION_RAM_LOG_SUB(data_no,client_data) \
    ram_log_extension_data_type     extension_data;                                                                             \
    unsigned long long cpu_sec;                                                                                                 \
    unsigned long  cpu_nanosec;                                                                                                 \
    unsigned int   write_index;                                                                                                 \
    /* Client Data check */                                                                                                     \
    if(client_data == NULL)                                                                                                     \
    {                                                                                                                           \
        return;                                                                                                                 \
    }                                                                                                                           \
    memset(&extension_data,0x00,sizeof(extension_data));                                                                        \
    /* Set Header */                                                                                                            \
    extension_data.cpu_id = (unsigned char)smp_processor_id();                                                                  \
    cpu_sec     = cpu_clock(extension_data.cpu_id);                                                                             \
    cpu_nanosec = do_div(cpu_sec, 1000000000);                                                                                  \
    extension_data.sec_uptime  = (unsigned long)cpu_sec;                                                                        \
    extension_data.msec_uptime = (cpu_nanosec / 1000);                                                                          \
    /* Set Client Data */                                                                                                       \
    strncpy(extension_data.data,client_data,EXTENSION_LOG_DATA_SIZE-1);                                                         \
    /* check index */                                                                                                           \
    if (ram_log_extension_record[data_no].index >= EXTENSION_LOG_RECORD_MAX)                                                    \
    {                                                                                                                           \
        ram_log_extension_record[data_no].index = 0;                                                                            \
    }                                                                                                                           \
    write_index = ram_log_extension_record[data_no].index;                                                                      \
    /* Set for Ram */                                                                                                           \
    memcpy(&(ram_log_extension_record[data_no].magic_code[0]),CRASH_MAGIC_CODE,strlen(CRASH_MAGIC_CODE));                       \
    memcpy(&(ram_log_extension_record[data_no].record_data[write_index]),&extension_data,sizeof(ram_log_extension_data_type));  \
    ram_log_extension_record[data_no].index++;

#else
#define EXTENSION_RAM_LOG_SUB(data,client_data)
#endif

ram_log_extension_record_type *ram_log_extension_record = (ram_log_extension_record_type *)ADDR_EXTENSION_LOG;

void extension_ram_log_init(void)
{
#ifdef CONFIG_ANDROID_EXTENSION_RAM
    ram_log_extension_info_type   *ram_log_extension_info   = (ram_log_extension_info_type *)ADDR_EXTENSION_INFO;
    
    memset(ram_log_extension_info,0x00,SIZE_EXTENSION_INFO);
    memset(ram_log_extension_record,0x00,SIZE_EXTENSION_LOG);
    
    memcpy( &ram_log_extension_info->magic_code[0]  , CRASH_MAGIC_CODE , strlen(CRASH_MAGIC_CODE) );
    memcpy( &ram_log_extension_info->linux_ver[0]   , BUILD_DISPLAY_ID   , strlen(BUILD_DISPLAY_ID  ) );
    memcpy( &ram_log_extension_info->model[0]   , PRODUCT_MODEL_NAME   , strlen(PRODUCT_MODEL_NAME  ) );
    
#endif
    
}

extern void get_crash_time(unsigned char *buf, unsigned int bufsize);
static void extension_ram_log_set_crash_time(void)
{
#ifdef CONFIG_ANDROID_EXTENSION_RAM
    unsigned long long cpu_sec;
    unsigned long  cpu_nanosec;
    unsigned char       cpu_id;
    ram_log_extension_info_type   *ram_log_extension_info   = (ram_log_extension_info_type *)ADDR_EXTENSION_INFO;
    
    memset(&ram_log_extension_info->base_time[0],0x00,CRASH_TIME_SIZE);
    get_crash_time( &ram_log_extension_info->base_time[0], CRASH_TIME_SIZE );
    
    cpu_id = (unsigned char)smp_processor_id();
    cpu_sec     = cpu_clock(cpu_id);
    cpu_nanosec = do_div(cpu_sec, 1000000000);
    
    ram_log_extension_info->base_sec_uptime  = (unsigned long)cpu_sec;
    ram_log_extension_info->base_msec_uptime = (cpu_nanosec / 1000);
#endif
    
}

void extension_ram_log_00(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_00,client_data);
    extension_ram_log_set_crash_time();
}

void extension_ram_log_01(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_01,client_data);
}

void extension_ram_log_02(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_02,client_data);
}

void extension_ram_log_03(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_03,client_data);
}

void extension_ram_log_04(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_04,client_data);
}

void extension_ram_log_05(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_05,client_data);
}

void extension_ram_log_06(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_06,client_data);
}

void extension_ram_log_07(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_07,client_data);
}

void extension_ram_log_08(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_08,client_data);
}

void extension_ram_log_09(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_09,client_data);
}

void extension_ram_log_10(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_10,client_data);
}

void extension_ram_log_11(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_11,client_data);
}

void extension_ram_log_12(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_12,client_data);
}

void extension_ram_log_13(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_13,client_data);
}

void extension_ram_log_14(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_14,client_data);
}

void extension_ram_log_15(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_15,client_data);
}

void extension_ram_log_16(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_16,client_data);
}

void extension_ram_log_17(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_17,client_data);
}

void extension_ram_log_18(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_18,client_data);
}

void extension_ram_log_19(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_19,client_data);
}

void extension_ram_log_20(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_20,client_data);
}

void extension_ram_log_21(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_21,client_data);
}

void extension_ram_log_22(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_22,client_data);
}

void extension_ram_log_23(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_23,client_data);
}

void extension_ram_log_24(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_24,client_data);
}

void extension_ram_log_25(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_25,client_data);
}

void extension_ram_log_26(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_26,client_data);
}

void extension_ram_log_27(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_27,client_data);
}

void extension_ram_log_28(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_28,client_data);
}

void extension_ram_log_29(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_29,client_data);
}

void extension_ram_log_30(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_30,client_data);
}

void extension_ram_log_31(char *client_data)
{
    EXTENSION_RAM_LOG_SUB(EXTENSION_DATA_31,client_data);
}

