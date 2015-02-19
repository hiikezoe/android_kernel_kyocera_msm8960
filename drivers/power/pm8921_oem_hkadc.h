#ifndef PM8921_OEM_HKADC_H
#define PM8921_OEM_HKADC_H
/* 
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */

#define OEM_HKADC_MONITOR_TIME_1S		1000

#define HKADC_VBAT_VOLTS_ELEMENT_COUNT		5
#define HKADC_IBAT_CURRENT_ELEMENT_COUNT	5
#define HKADC_PA_THERM_ELEMENT_COUNT		5
#define HKADC_CAMERA_TEMP_ELEMENT_COUNT		5
#define HKADC_BAT_TEMP_ELEMENT_COUNT		5
#define HKADC_SUBSTRATE_THERM_ELEMENT_COUNT	5
#define HKADC_USB_THERM_ELEMENT_COUNT		5

#define HKADC_SUSPEND_MONIT_FREQ		(60 * 10)

#define HKADC_PA_THERM_MONIT_FREQ		5
#define HKADC_CAMERA_TEMP_MONIT_FREQ		5
#define HKADC_SUBSTRATE_THERM_MONIT_FREQ	5
#define HKADC_USB_THERM_MONIT_FREQ			5

DEFINE_SPINLOCK(oem_hkadc_master_lock);
#define MASTERDATA_SPINLOCK_INIT()	spin_lock_init(&oem_hkadc_master_lock);

#define	MASTERDATA_LOCK()		\
	{				\
	unsigned long oem_hkadc_irq_flag;	\
	spin_lock_irqsave(&oem_hkadc_master_lock, oem_hkadc_irq_flag);

#define	MASTERDATA_UNLOCK()						\
	spin_unlock_irqrestore(&oem_hkadc_master_lock, oem_hkadc_irq_flag);	\
	}

typedef enum{
	OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE=0,
	OEM_POWER_SUPPLY_PROP_IDX_BAT_CURRENT,
	OEM_POWER_SUPPLY_PROP_IDX_PA_THERM,
	OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM,
	OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM,
	OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM,
	OEM_POWER_SUPPLY_PROP_IDX_USB_THERM,
	OEM_POWER_SUPPLY_PROP_IDX_NUM
}oem_power_supply_property_index;


#endif

