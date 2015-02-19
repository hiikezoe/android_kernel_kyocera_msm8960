/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/ratelimit.h>
#include <linux/gpio.h>

#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>
#include <mach/gpio.h>

#include <linux/android_alarm.h>
#include <pm8921_oem_hkadc.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <pm8921-charger_oem.h>
#include <mach/oem_fact.h>
#include <linux/reboot.h>

#define CHG_BUCK_CLOCK_CTRL	0x14
#define CHG_BUCK_CLOCK_CTRL_8038	0xD

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6
#define OVP_USB_UVD		0x2B7
#define PM8921_USB_TRIM_SEL	0x339

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200
#define UNPLUG_CHECK_RAMP_MS 25
#define USB_TRIM_ENTRIES 16

enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
	struct	work_struct	work;
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @uvd_voltage_mv:		(PM8917 only) the falling UVD threshold voltage
 * @alarm_low_mv:		the battery alarm voltage low
 * @alarm_high_mv:		the battery alarm voltage high
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @hysteresis_temp_dc:		the hysteresis between temp thresholds in
 *				deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 *
 */
struct pm8921_chg_chip {
	struct device			*dev;
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	unsigned int			uvd_voltage_mv;
	unsigned int			safe_current_ma;
	unsigned int			alarm_low_mv;
	unsigned int			alarm_high_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	int				hysteresis_temp_dc;
	unsigned int			temp_check_period;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	int				resume_charge_percent;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct power_supply		bms_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	int				*usb_trim_table;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
	struct delayed_work		unplug_check_work;
	struct delayed_work		vin_collapse_check_work;
	struct delayed_work		btc_override_work;
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	bool				host_mode;
	bool				has_dc_supply;
	u8				active_path;
	int				recent_reported_soc;
	int				battery_less_hardware;
	int				ibatmax_max_adj_ma;
	int				btc_override;
	int				btc_override_cold_decidegc;
	int				btc_override_hot_decidegc;
	int				btc_delay_ms;
	bool				btc_panic_if_cant_stop_chg;
	int				stop_chg_upon_expiry;
	bool				disable_aicl;
	int				usb_type;
	bool				disable_chg_rmvl_wrkarnd;
};

/* user space parameter to limit usb current */
static unsigned int usb_max_current;
#define OEM_IUSBMAX_NOT_SET 0xFFFF
static int oem_iusbmax_current = OEM_IUSBMAX_NOT_SET;
static int oem_stand_detect_time_counter = 0;
static int oem_get_vbatdet_low(struct pm8921_chg_chip *chip);
struct wake_lock oem_hkadc_wake_lock;
static bool is_hkadc_suspend_monit = 0;
static void oem_uim_smem_init(void);
static void fast_chg_limit_worker(struct work_struct *work);
static void oem_charger_limit_control(struct work_struct *work);
static void oem_hkadc_init(void);
static void oem_chargermonit_init(void);
static void oem_hkadc_exit(void);
static void oem_chargermonit_exit(void);

/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
static int charging_disabled;
static int thermal_mitigation;
static int charge_state;
static bool is_batterydetected;

static struct pm8921_chg_chip *the_chip;
static void check_temp_thresholds(struct pm8921_chg_chip *chip);

#ifdef QUALCOMM_ORIGINAL_FEATURE
#define LPM_ENABLE_BIT	BIT(2)
static int pm8921_chg_set_lpm(struct pm8921_chg_chip *chip, int enable)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}
	reg &= ~LPM_ENABLE_BIT;
	reg |= (enable ? LPM_ENABLE_BIT : 0);

	rc = pm8xxx_writeb(chip->dev->parent, CHG_CNTRL, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}

	return rc;
}
#endif

static int pm_chg_write(struct pm8921_chg_chip *chip, u16 addr, u8 reg)
{
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
	if (rc)
		pr_err("failed: addr=%03X, rc=%d\n", addr, rc);

	return rc;
}

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm_chg_write(chip, addr, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}
#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err = 0, ret = 0;

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;

err_out:
	if (err)
		return err;

	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp, data;
	int err = 0;

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &data);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}

err_out:
	if (err)
		return err;

	/* return the lower 4 bits */
	return data & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

int oem_pm8921_disable_source_current(bool disable);
int oem_pm8921_disable_source_current_flag = 0;
#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
#else
	int ret;

	ret = pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
	if (ret) {
		pr_err("buck converter setting error %d\n", ret);
		return ret;
	}

	if (0 == disable) {
		oem_pm8921_disable_source_current_flag = 0;
	}
	return ret;
#endif
}

static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#define PM8921_CHG_VDDMAX_MIN	3400
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_write(chip, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
	if (is_usb_chg_plugged_in(chip)) {
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}

#define PM8921_CHG_VDDSAFE_MIN	3400
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8917_USB_UVD_MIN_MV	3850
#define PM8917_USB_UVD_MAX_MV	4350
#define PM8917_USB_UVD_STEP_MV	100
#define PM8917_USB_UVD_MASK	0x7
static int pm_chg_uvd_threshold_set(struct pm8921_chg_chip *chip, int thresh_mv)
{
	u8 temp;

	if (thresh_mv < PM8917_USB_UVD_MIN_MV
			|| thresh_mv > PM8917_USB_UVD_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", thresh_mv);
		return -EINVAL;
	}
	temp = (thresh_mv - PM8917_USB_UVD_MIN_MV) / PM8917_USB_UVD_STEP_MV;
	return pm_chg_masked_write(chip, OVP_USB_UVD,
				PM8917_USB_UVD_MASK, temp);
}

#define PM8921_CHG_IBATMAX_MIN	325
#define PM8921_CHG_IBATMAX_MAX	3025
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F
static int pm_chg_ibatmax_get(struct pm8921_chg_chip *chip, int *ibat_ma)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_IBAT_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading ibat max\n", rc);
		*ibat_ma = 0;
		return rc;
	}
	*ibat_ma = (int)(temp & PM8921_CHG_I_MASK) * PM8921_CHG_I_STEP_MA
							+ PM8921_CHG_I_MIN_MA;
	return 0;
}

static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

/* USB Trim tables */
static int usb_trim_pm8921_table_1[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x5,
	0x0,
	-0x7,
	0x0,
	-0x9,
	-0xA,
	0x0,
	0x0,
	-0xE,
	0x0,
	-0xF,
	0x0,
	-0x10,
	0x0
};

static int usb_trim_pm8921_table_2[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x2,
	0x0,
	-0x4,
	0x0,
	-0x4,
	-0x5,
	0x0,
	0x0,
	-0x6,
	0x0,
	-0x6,
	0x0,
	-0x6,
	0x0
};

static int usb_trim_8038_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x9,
	0x0,
	-0xD,
	0x0,
	-0x10,
	-0x11,
	0x0,
	0x0,
	-0x25,
	0x0,
	-0x28,
	0x0,
	-0x32,
	0x0
};

static int usb_trim_8917_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	0xA,
	0xC,
	0x10,
	0x10,
	0x13,
	0x14,
	0x13,
	0x3,
	0x1A,
	0x1D,
	0x1D,
	0x21,
	0x24,
	0x26
};

/* Maximum USB  setting table */
static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define REG_SBI_CONFIG			0x04F
#define PAGE3_ENABLE_MASK		0x6
#define USB_OVP_TRIM_MASK		0x3F
#define USB_OVP_TRIM_PM8917_MASK	0x7F
#define USB_OVP_TRIM_MIN		0x00
#define REG_USB_OVP_TRIM_ORIG_LSB	0x10A
#define REG_USB_OVP_TRIM_ORIG_MSB	0x09C
#define REG_USB_OVP_TRIM_PM8917		0x2B5
#define REG_USB_OVP_TRIM_PM8917_BIT	BIT(0)
#define USB_TRIM_MAX_DATA_PM8917	0x3F
#define USB_TRIM_POLARITY_PM8917_BIT	BIT(6)
static int pm_chg_usb_trim(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, sbi_config, msb, lsb, mask;
	s8 trim;
	int rc = 0;
	static u8 usb_trim_reg_orig = 0xFF;

	/* No trim data for PM8921 */
	if (!chip->usb_trim_table)
		return 0;

	if (usb_trim_reg_orig == 0xFF) {
		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_MSB, &msb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_LSB, &lsb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		msb = msb >> 5;
		lsb = lsb >> 5;
		usb_trim_reg_orig = msb << 3 | lsb;

		if (pm8xxx_get_version(chip->dev->parent)
				== PM8XXX_VERSION_8917) {
			rc = pm8xxx_readb(chip->dev->parent,
					REG_USB_OVP_TRIM_PM8917, &msb);
			if (rc) {
				pr_err("error = %d reading config reg\n", rc);
				return rc;
			}

			msb = msb & REG_USB_OVP_TRIM_PM8917_BIT;
			usb_trim_reg_orig |= msb << 6;
		}
	}

	/* use the original trim value */
	trim = usb_trim_reg_orig;

	trim += chip->usb_trim_table[index];
	if (trim < 0)
		trim = 0;

	pr_debug("trim_orig %d write 0x%x index=%d value 0x%x to USB_OVP_TRIM\n",
		usb_trim_reg_orig, trim, index, chip->usb_trim_table[index]);

	rc = pm8xxx_readb(chip->dev->parent, REG_SBI_CONFIG, &sbi_config);
	if (rc) {
		pr_err("error = %d reading sbi config reg\n", rc);
		return rc;
	}

	temp = sbi_config | PAGE3_ENABLE_MASK;
	rc = pm_chg_write(chip, REG_SBI_CONFIG, temp);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}

	mask = USB_OVP_TRIM_MASK;

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		mask = USB_OVP_TRIM_PM8917_MASK;

	rc = pm_chg_masked_write(chip, USB_OVP_TRIM, mask, trim);
	if (rc) {
		pr_err("error = %d writing USB_OVP_TRIM\n", rc);
		return rc;
	}

	rc = pm_chg_write(chip, REG_SBI_CONFIG, sbi_config);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}
	return rc;
}

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)
static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, fineres, reg_val;
	int rc;

	reg_val = usb_ma_table[index].value >> 1;
	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[index].value;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc) {
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
				return rc;
			}
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}
	}

	rc = pm_chg_usb_trim(chip, index);
	if (rc)
			pr_err("unable to set usb trim rc = %d\n", rc);

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	if (i < 0) {
		pr_err("can't find %d in usb_ma_table. Use min.\n", temp);
		i = 0;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x3F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}


static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
	int64_t rc;

	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);

	if (chip->bms_notify.is_charging ^ new_is_charging) {
		chip->bms_notify.is_charging = new_is_charging;
		schedule_work(&(chip->bms_notify.work));
	}
}

static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

extern bool oem_pm8921_bms_is_cyclecorrection_chargeoffstate(void);

int oem_option_item1_bit0;
int oem_option_item1_bit1;
int oem_option_item1_bit2;
int oem_option_item1_bit3;
int oem_option_item1_bit4;
static void oem_get_fact_option(void)
{
	oem_option_item1_bit0 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 0);
	oem_option_item1_bit1 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 1);
	oem_option_item1_bit2 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 2);
	oem_option_item1_bit3 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 3);
	oem_option_item1_bit4 = oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 4);
}

extern int oem_bms_last_batt_status;
extern int oem_bms_last_charge_type;

int oem_last_batt_status;
int oem_last_charge_type;
int oem_last_dc_present;
int oem_last_usb_present;
int32_t oem_charger_control_flag = 1;
#define USB_WALL_THRESHOLD_MA	500
static uint32_t oem_charge_stand_flag = 0;
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int type;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;

		if ((pm_is_chg_charge_dis(the_chip)) &&
		   (1 != oem_pm8921_disable_source_current_flag)) {
			val->intval = 0;
			return 0;
		}

		if (the_chip->has_dc_supply) {
			val->intval = 1;
			return 0;
		}

		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}

		type = the_chip->usb_type;
		if (type == POWER_SUPPLY_TYPE_USB_DCP ||
			type == POWER_SUPPLY_TYPE_USB_ACA ||
			type == POWER_SUPPLY_TYPE_USB_CDP ||
			type == POWER_SUPPLY_TYPE_MHL ||
			type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = is_usb_chg_plugged_in(the_chip);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int disable_aicl(int disable)
{
	if (disable != POWER_SUPPLY_HEALTH_UNKNOWN
		&& disable != POWER_SUPPLY_HEALTH_GOOD) {
		pr_err("called with invalid param :%d\n", disable);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}

	pr_debug("Disable AICL = %d\n", disable);
	the_chip->disable_aicl = disable;
	return 0;
}

static int switch_usb_to_charge_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (!chip->host_mode)
		return 0;

	/* enable usbin valid comparator and remove force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB2);
	if (rc < 0) {
		pr_err("Failed to write 0xB2 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 0;

	return 0;
}

static int switch_usb_to_host_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->host_mode)
		return 0;

	/* disable usbin valid comparator and force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB3);
	if (rc < 0) {
		pr_err("Failed to write 0xB3 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 1;

	return 0;
}

static int pm_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		if (val->intval == POWER_SUPPLY_SCOPE_SYSTEM)
			return switch_usb_to_host_mode(the_chip);
		if (val->intval == POWER_SUPPLY_SCOPE_DEVICE)
			return switch_usb_to_charge_mode(the_chip);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		return pm8921_set_usb_power_supply_type(val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		return disable_aicl(val->intval);
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;

		if ((pm_is_chg_charge_dis(the_chip)) &&
		    (1 != oem_pm8921_disable_source_current_flag)) {
			return 0;
		}

		if (the_chip->usb_type == POWER_SUPPLY_TYPE_USB)
			val->intval = is_usb_chg_plugged_in(the_chip);

		break;

	case POWER_SUPPLY_PROP_SCOPE:
		if (the_chip->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		val->intval = the_chip->disable_aicl;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_OEM_PA_THERM,
	POWER_SUPPLY_PROP_OEM_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM,
	POWER_SUPPLY_PROP_OEM_USB_THERM,
	POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE,
};

static enum power_supply_property msm_bms_power_props[] = {
	POWER_SUPPLY_PROP_OEM_BMS_CYCLE,
	POWER_SUPPLY_PROP_OEM_BMS_BATT_STATUS,
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return (int)result.physical;
}

static int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
	int current_voltage_uv = get_prop_battery_uvolts(chip);
	int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_uv < 0) {
		pr_err("Error reading current voltage\n");
		return -EIO;
	}

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
}

enum oem_charger_mode {
	OEM_STATUS_NORMAL= 0,
	OEM_STATUS_LIMIT,
	OEM_STATUS_STOP
};
static uint32_t oem_charge_status = OEM_STATUS_NORMAL;
static int oem_vbatt = 0;
static int oem_batt_temp = 0;

enum oem_charge_vbatt_ov_state_ {
	OEM_CHARGE_VBATT_STATE_OFF= 0,
	OEM_CHARGE_VBATT_STATE_ON,
	OEM_CHARGE_VBATT_STATE_OVOFF
};

static struct work_struct	chg_vbatt_ov_work;
int oem_charge_vbatt_ov_state=OEM_CHARGE_VBATT_STATE_OFF;
int oem_charge_vbatt_ov_state_now=OEM_CHARGE_VBATT_STATE_OFF;

static bool is_oem_fast_chg_expired=false;
static int oem_stand_detect  = 0;
static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;

  if ((!pm_chg_get_rt_status(chip, USBIN_UV_IRQ)) && (pm_chg_get_rt_status(chip, USBIN_OV_IRQ))) {
    return POWER_SUPPLY_STATUS_NOT_CHARGING;
  }

  if ((!pm_chg_get_rt_status(chip, DCIN_UV_IRQ)) && (pm_chg_get_rt_status(chip, DCIN_OV_IRQ))){
    return POWER_SUPPLY_STATUS_NOT_CHARGING;
  }

	if (!chip->dc_present && !chip->usb_present && !oem_stand_detect) {
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (oem_charge_vbatt_ov_state_now == OEM_CHARGE_VBATT_STATE_OVOFF){
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if (is_oem_fast_chg_expired) {
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	if (oem_charge_stand_flag) {
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	if (oem_pm8921_bms_is_cyclecorrection_chargeoffstate()) {
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	if (OEM_STATUS_STOP == oem_charge_status)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;

	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return batt_state;
}

static int charge_uim_valid = 1;
static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;

	if (chip->battery_less_hardware)
		return 100;

	if (!get_prop_batt_present(chip))
		percent_soc = voltage_based_capacity(chip);
	else
#ifdef QUALCOMM_ORIGINAL_FEATURE
		percent_soc = pm8921_bms_get_percent_charge();
#else
		percent_soc = pm8921_bms_get_percent_charge(charge_state, charge_uim_valid);
#endif

	if (percent_soc == -ENXIO)
		percent_soc = voltage_based_capacity(chip);

	if (percent_soc < 0) {
		pr_err("Unable to read battery voltage\n");
		goto fail_voltage;
	}

	if (percent_soc <= 10)
		pr_debug("low battery charge = %d%%\n", percent_soc);

fail_voltage:
	chip->recent_reported_soc = percent_soc;
	return percent_soc;
}

static int get_prop_batt_current_max(struct pm8921_chg_chip *chip, int *curr)
{
	*curr = 0;
	*curr = pm8921_bms_get_current_max();
	if (*curr == -EINVAL)
		return -EINVAL;

	return 0;
}

static int get_prop_batt_current(struct pm8921_chg_chip *chip, int *curr)
{
	int rc;

	*curr = 0;
	rc = pm8921_bms_get_battery_current(curr);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(curr);
	}
	if (rc)
		pr_err("unable to get batt current rc = %d\n", rc);

	return rc;
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}

static void chg_vbatt_ov_check_charger(void)
{
	int count=0;

	if (!pm_chg_get_rt_status(the_chip, DCIN_UV_IRQ)){
		count += 1;
	}
	if (!pm_chg_get_rt_status(the_chip, USBIN_UV_IRQ)){
		count += 2;
	}
	if (count == 0){
		oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_OFF;
	}else{
		oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_ON;
	}
	pr_debug("uv state %d\n", count);
	schedule_work(&chg_vbatt_ov_work);
}

struct delayed_work oem_charger_work;
static void chg_vbatt_ov_worker(struct work_struct *work)
{

	bool check = true;
	int ret;

	pr_debug("oem_charge_vbatt_ov_state-1 %d, oem_charge_vbatt_ov_state_now %d\n"
			, oem_charge_vbatt_ov_state, oem_charge_vbatt_ov_state_now);

	switch(oem_charge_vbatt_ov_state){
	case OEM_CHARGE_VBATT_STATE_OFF:

		if (oem_charge_vbatt_ov_state_now == OEM_CHARGE_VBATT_STATE_OVOFF){
			pr_debug("charger removed\n");
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
		}
		oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_OFF;

		break;

	case OEM_CHARGE_VBATT_STATE_ON:
		do {
			if (!is_batterydetected){
				check = false;
				pr_debug("battery removed.\n");
				break;
			}

			if ((oem_param_charger.chg_stop_volt * 1000) > oem_vbatt){
				check = false;
				pr_debug("vbatt normal %d.\n", oem_vbatt);
				break;
			}
		} while(0);

		if (check) {
			pr_err("vbatt ov detected\n");
			if (oem_charge_vbatt_ov_state_now != OEM_CHARGE_VBATT_STATE_OVOFF){
				pr_err("chg off\n");
				if (OEM_STATUS_LIMIT == oem_charge_status) {
					oem_charge_status = OEM_STATUS_NORMAL;
					cancel_delayed_work_sync(&oem_charger_work);
				}
				oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_OVOFF;
				oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_OVOFF;
				ret = oem_pm8921_disable_source_current(true);
				if (ret) {
					pr_err("error buck converter setting value %d\n", ret);
				}
			}
		}else{
			oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_ON;
		}

		break;

	case OEM_CHARGE_VBATT_STATE_OVOFF:

		if (!is_batterydetected){
			pr_debug("battery removed.\n");
			oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_ON;
			oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_ON;
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
			break;
		}

		break;

	default:
		pr_info("oem_charge_vbatt_ov_state Illegal state. %d\n", oem_charge_vbatt_ov_state);
		break;

	}

	pr_debug("oem_charge_vbatt_ov_state-2 %d, oem_charge_vbatt_ov_state_now %d\n", oem_charge_vbatt_ov_state, oem_charge_vbatt_ov_state_now);

}

static int get_prop_batt_charge_now(struct pm8921_chg_chip *chip, int *cc_uah)
{
	int rc;

	*cc_uah = 0;
	rc = pm8921_bms_cc_uah(cc_uah);
	if (rc)
		pr_err("unable to get batt fcc rc = %d\n", rc);

	return rc;
}

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
	int temp;

	if (oem_charge_vbatt_ov_state_now == OEM_CHARGE_VBATT_STATE_OVOFF)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	if (!is_batterydetected)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_COLD;

	if (OEM_STATUS_STOP == oem_charge_status)
		return POWER_SUPPLY_HEALTH_DEAD;
	
	if ((!pm_chg_get_rt_status(chip, USBIN_UV_IRQ)) && (pm_chg_get_rt_status(chip, USBIN_OV_IRQ))) {
	  pr_err("USB OVP detect\n");
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}
	if ((!pm_chg_get_rt_status(chip, DCIN_UV_IRQ)) && (pm_chg_get_rt_status(chip, DCIN_OV_IRQ))){
	  pr_err("DCIN OVP detect");
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (oem_charge_stand_flag) {
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	}

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (is_ext_trickle_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (is_ext_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int get_prop_batt_temp(struct pm8921_chg_chip *chip, int *temp)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	if (chip->battery_less_hardware) {
		*temp = 300;
		return 0;
	}

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

	*temp = (int)result.physical;

	return rc;
}

static int oem_charger_pm_batt_power_get_property(enum power_supply_property psp, int *intval);
static int oem_hkadc_pm_batt_power_get_property(enum power_supply_property psp, int *intval);
static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int result;
	int rc = 0;
	int value;
	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);
	result = oem_charger_pm_batt_power_get_property(psp, &(val->intval));
	if (result!=-EINVAL){
		return result;
	}

	result = oem_hkadc_pm_batt_power_get_property(psp, &(val->intval));
	if (result!=-EINVAL){
		return result;
	}

	switch (psp) {
#ifdef QUALCOMM_ORIGINAL_FEATURE
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
#else
	case POWER_SUPPLY_PROP_STATUS:
		if ((OEM_STATUS_LIMIT == oem_charge_status) && (1 == oem_charger_control_flag)) {
			val->intval = oem_last_batt_status;
		} else if (oem_pm8921_bms_is_cyclecorrection_chargeoffstate()){
			val->intval = oem_bms_last_batt_status;
		} else {
		val->intval = get_prop_batt_status(chip);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if ((OEM_STATUS_LIMIT == oem_charge_status) && (1 == oem_charger_control_flag)) {
			val->intval = oem_last_charge_type;
		} else if (oem_pm8921_bms_is_cyclecorrection_chargeoffstate()){
			val->intval = oem_bms_last_charge_type;
		} else {
		val->intval = get_prop_charge_type(chip);
		}
		break;
#endif
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = get_prop_batt_present(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = get_prop_battery_uvolts(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = get_prop_batt_capacity(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = get_prop_batt_current(chip, &value);
		if (!rc)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = get_prop_batt_current_max(chip, &value);
		if (!rc)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = get_prop_batt_temp(chip, &value);
		if (!rc)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		rc = get_prop_batt_fcc(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		rc = get_prop_batt_charge_now(chip, &value);
		if (!rc) {
			val->intval = value;
			rc = 0;
		}
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

static int pm_bms_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int result = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_BMS_CYCLE:
		val->intval = oem_pm8921_bms_get_chargecycles();
		pr_debug("BMS_CYCLE:%d\n", val->intval);
		break;

	case POWER_SUPPLY_PROP_OEM_BMS_BATT_STATUS:
		val->intval = oem_pm8921_bms_get_deteriorationstatus();
		pr_debug("BMS_BATT_STATUS:%d\n", val->intval);
		break;

	default:
		result = -EINVAL;
	}
	return result;
}

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

static void __pm8921_charger_vbus_draw(unsigned int mA)
{
	int i, rc;
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_debug("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}

	if (mA <= 2) {
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       mA);
			i = 0;
		}

		/* Check if IUSB_FINE_RES is available */
		while ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		if (rc)
			pr_err("unable to set iusb to %d rc = %d\n", i, rc);
	}
}

/* USB calls these to tell us how much max usb current the system can draw */
void pm8921_charger_vbus_draw(unsigned int mA)
{
	int set_usb_now_ma;

	pr_debug("Enter charge=%d\n", mA);

	oem_iusbmax_current = mA;

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (mA == 0 && the_chip && !get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		if (!the_chip->has_dc_supply) {
			pr_err("rejected: no other power source mA = %d\n", mA);
			return;
		}
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	if (usb_target_ma)
		usb_target_ma = mA;


	if (mA > USB_WALL_THRESHOLD_MA)
		set_usb_now_ma = USB_WALL_THRESHOLD_MA;
	else
		set_usb_now_ma = mA;

	if (the_chip && the_chip->disable_aicl)
		set_usb_now_ma = mA;

	if (the_chip)
		__pm8921_charger_vbus_draw(set_usb_now_ma);
	else
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		usb_chg_current = set_usb_now_ma;
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);

int pm8921_get_chg_vddmax(int *voltage)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
  return pm_chg_vddmax_get(the_chip, voltage);
}
EXPORT_SYMBOL(pm8921_get_chg_vddmax);

/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm_chg_write(the_chip, CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8917_set_under_voltage_detection_threshold(int mv)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_uvd_threshold_set(the_chip, mv);
}
EXPORT_SYMBOL(pm8917_set_under_voltage_detection_threshold);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");

	pm_chg_usb_suspend_enable(the_chip, disable);

	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB && type > POWER_SUPPLY_TYPE_BATTERY)
		return -EINVAL;

	the_chip->usb_type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	int temp = 0, rc = 0;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = get_prop_batt_temp(the_chip, &temp);
	if (rc) {
		pr_err("Unable to read temperature");
		return rc;
	}
	return temp;
}

struct wake_lock oem_charge_wake_lock;
static struct work_struct	fast_chg_count_stop_work;
static struct work_struct	fast_chg_count_stop_work_usb;
static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present;

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;
		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
		pm8921_bms_calibrate_hkadc();
	}
	if (usb_present) {
		pm_chg_auto_enable(chip, 1);
		pr_debug("handle_usb_insertion_removal() usb valid\n");
		wake_lock(&oem_charge_wake_lock);
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	} else {
		/* USB unplugged reset target current */
		pr_debug("handle_usb_insertion_removal() usb invalid\n");
		schedule_work(&fast_chg_count_stop_work_usb);
		wake_unlock(&oem_charge_wake_lock);
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	}
	bms_notify_check(chip);
}

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}

	/* Force BATFET=ON */
	pm8921_disable_source_current(true);

	schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/*
	 * since we wont get a fastchg irq from external charger
	 * use eoc worker to detect end of charging
	 */
	schedule_delayed_work(&chip->eoc_work, delay);
	wake_lock(&chip->eoc_wake_lock);
	if (chip->btc_override)
		schedule_delayed_work(&chip->btc_override_work,
				round_jiffies_relative(msecs_to_jiffies
					(chip->btc_delay_ms)));
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void turn_off_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to ovptestreg rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from ovptestreg rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x ovptestreg rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

static int param_open_ovp_counter = 10;
module_param(param_open_ovp_counter, int, 0644);

#define USB_ACTIVE_BIT BIT(5)
#define DC_ACTIVE_BIT BIT(6)
static int is_active_chg_plugged_in(struct pm8921_chg_chip *chip,
						u8 active_chg_mask)
{
	if (active_chg_mask & USB_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	else if (active_chg_mask & DC_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	else
		return 0;
}

#define WRITE_BANK_4		0xC0
#define OVP_DEBOUNCE_TIME 0x06
static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg_gone = 0, active_chg_plugged_in = 0;
	int count = 0;
	u8 active_mask = 0;
	u16 ovpreg, ovptestreg;

	if (is_usb_chg_plugged_in(chip) &&
		(chip->active_path & USB_ACTIVE_BIT)) {
		ovpreg = USB_OVP_CONTROL;
		ovptestreg = USB_OVP_TEST;
		active_mask = USB_ACTIVE_BIT;
	} else if (is_dc_chg_plugged_in(chip) &&
		(chip->active_path & DC_ACTIVE_BIT)) {
		ovpreg = DC_OVP_CONTROL;
		ovptestreg = DC_OVP_TEST;
		active_mask = DC_ACTIVE_BIT;
	} else {
		return;
	}

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		active_chg_plugged_in
			= is_active_chg_plugged_in(chip, active_mask);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_debug("OVP FET count = %d chg_gone=%d, active_valid = %d\n",
					count, chg_gone, active_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && active_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_ovp_fet(chip, ovptestreg);

			msleep(20);

			turn_on_ovp_fet(chip, ovptestreg);
		} else {
			break;
		}
	}
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x6);
	else
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x2);

	pr_debug("Exit count=%d chg_gone=%d, active_valid=%d\n",
		count, chg_gone, active_chg_plugged_in);
	return;
}

static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		while (!the_chip->iusb_fine_res && i > 0
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES))
			i--;

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       *value);
			i = 0;
		}

		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/*
	 * AICL only for wall-chargers. If the charger appears to be plugged
	 * back in now, the corresponding unplug must have been because of we
	 * were trying to draw more current than the charger can support. In
	 * such a case reset usb current to 500mA and decrease the target.
	 * The AICL algorithm will step up the current from 500mA to target
	 */
	if (is_usb_chg_plugged_in(chip)
		&& usb_target_ma > USB_WALL_THRESHOLD_MA
		&& !chip->disable_aicl) {
		pm_chg_auto_enable(chip, 1);
		wake_lock(&oem_charge_wake_lock);
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
		if (!delayed_work_pending(&chip->unplug_check_work))
			schedule_delayed_work(&chip->unplug_check_work,
				      msecs_to_jiffies
						(UNPLUG_CHECK_WAIT_PERIOD_MS));
	} else {
		handle_usb_insertion_removal(chip);
	}
}

static void oem_uim_check(unsigned int* max_voltage_normal);
enum voltage_delta {
	VOLTAGE_DELTA_INIT = 0,
	VOLTAGE_DELTA_RESUME
};
static void oem_voltage_setting(enum voltage_delta delta_id)
{
	int rc;
	int set_voltage_mv;

	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	oem_uim_check(&the_chip->max_voltage_mv);

	if (VOLTAGE_DELTA_INIT == delta_id) {
		the_chip->resume_voltage_delta = oem_param_charger.initial_delta_volt;
	} else if (VOLTAGE_DELTA_RESUME == delta_id) {
		the_chip->resume_voltage_delta = oem_param_charger.rechg_delta_volt;
	}

	if (the_chip->is_bat_warm) {
		set_voltage_mv = the_chip->warm_bat_voltage;
	} else if (the_chip->is_bat_cool) {
		set_voltage_mv = the_chip->cool_bat_voltage;
	} else {
		set_voltage_mv = the_chip->max_voltage_mv;
	}

	rc = pm_chg_vbatdet_set(the_chip,
			set_voltage_mv - the_chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			set_voltage_mv - the_chip->resume_voltage_delta, rc);
	}

	rc = pm_chg_vddmax_set(the_chip, set_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						set_voltage_mv, rc);
	}

}

#define VIN_MIN_COLLAPSE_CHECK_MS	50
static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{

	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
	    handle_usb_insertion_removal(data);

	chg_vbatt_ov_check_charger();
	return IRQ_HANDLED;
}

static irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	pr_debug("USB OverVoltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	pr_debug("USB UnderVoltage\n");
	return IRQ_HANDLED;
}

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
	pr_debug("battery present=%d", status);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);

	if (high_transition) {
		/* enable auto charging */
		pm_chg_auto_enable(chip, !charging_disabled);
		pr_info("batt fell below resume voltage %s\n",
			charging_disabled ? "" : "charger enabled");
	}
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

	if (!chip->stop_chg_upon_expiry) {
		ret = pm_chg_failed_clear(chip, 1);
		if (ret)
			pr_err("Failed to write CHG_FAILED_CLEAR bit\n");
	}

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
	return IRQ_HANDLED;
}

static struct delayed_work	fast_chg_limit_work;
static struct work_struct	fast_chg_count_start_work;
static bool is_oem_fast_chg_counting=false;
static int oem_hkadc_master_data_read(enum power_supply_property psp, int *intval);
static void fast_chg_count_start_worker(struct work_struct *work)
{

	int ret = 0;
	int battemp;

	if (!is_oem_fast_chg_counting){

		ret = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_TEMP, &battemp);
		if (ret) {
			pr_err("Failed to reading batt temp, ret = %d\n", ret);
			return;
		}

		if (battemp > (oem_param_charger.chg_cool_tmp * 10)){
			schedule_delayed_work(&fast_chg_limit_work,
					round_jiffies_relative(msecs_to_jiffies
						(oem_param_charger.time_chg * 60 * 1000)));
		}else{
			schedule_delayed_work(&fast_chg_limit_work,
					round_jiffies_relative(msecs_to_jiffies
						(oem_param_charger.time_cool_chg * 60 * 1000)));
		}
		is_oem_fast_chg_counting = true;
		is_oem_fast_chg_expired=false;
	}

}

static void fast_chg_count_stop_worker(struct work_struct *work)
{
	pr_debug("fast_chg_count_stop_worker() called\n");
	pr_debug("oem_charge_status=%d\n", oem_charge_status);
	if ((is_oem_fast_chg_counting) &&
		(OEM_STATUS_LIMIT != oem_charge_status) &&
		(!oem_pm8921_bms_is_cyclecorrection_chargeoffstate())){
		pr_debug("fast_chg_count_stop_worker() canceled\n");
		cancel_work_sync(&fast_chg_count_start_work);
		cancel_delayed_work_sync(&fast_chg_limit_work);
		is_oem_fast_chg_counting = false;
	}
}

static void fast_chg_count_stop_worker_usb(struct work_struct *work)
{
	pr_debug("fast_chg_count_stop_worker_usb() called\n");
	if (is_oem_fast_chg_counting){
		pr_debug("fast_chg_count_stop_worker() canceled\n");
		cancel_work_sync(&fast_chg_count_start_work);
		cancel_delayed_work_sync(&fast_chg_limit_work);
		is_oem_fast_chg_counting = false;
	}
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);


	bms_notify_check(chip);
#else
	struct pm8921_chg_chip *chip = data;
	int fsm_state = pm_chg_get_fsm_state(chip);

	if (FSM_STATE_EOC_10 == fsm_state) {

	} else if (FSM_STATE_ON_CHG_HIGHI_1 == fsm_state) {

	} else {
		oem_voltage_setting(VOLTAGE_DELTA_INIT);
	}
#endif

	return IRQ_HANDLED;
}

enum {
	PON_TIME_25NS	= 0x04,
	PON_TIME_50NS	= 0x08,
	PON_TIME_100NS	= 0x0C,
};

static void set_min_pon_time(struct pm8921_chg_chip *chip, int pon_time_ns)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x40);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* clear the min pon time select bit */
	temp &= 0xF3;
	/* set the pon time */
	temp |= (u8)pon_time_ns;
	/* write enable bank 4 */
	temp |= 0x80;
	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip)
{
	pr_debug("Start\n");
	set_min_pon_time(chip, PON_TIME_100NS);
	pm_chg_vinmin_set(chip, chip->vin_min + 200);
	msleep(250);
	pm_chg_vinmin_set(chip, chip->vin_min);
	set_min_pon_time(chip, PON_TIME_25NS);
	pr_debug("End\n");
}

#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US	200
#define VIN_MIN_INCREASE_MV	100
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop = 0, active_path;
	int rc, ibat, active_chg_plugged_in, usb_ma;
	int chg_gone = 0;
	bool ramp = false;

	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS1, &active_path);
	if (rc) {
		pr_err("Failed to read PBL_ACCESS1 rc=%d\n", rc);
		return;
	}

	chip->active_path = active_path;
	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg_plugged_in = %d\n",
			active_path, active_chg_plugged_in);
	if (active_path & USB_ACTIVE_BIT) {
		pr_debug("USB charger active\n");

		pm_chg_iusbmax_get(chip, &usb_ma);

		if (usb_ma <= 100) {
			pr_debug(
				"Unenumerated or suspended usb_ma = %d skip\n",
				usb_ma);
			goto check_again_later;
		}
	} else if (active_path & DC_ACTIVE_BIT) {
		pr_debug("DC charger active\n");
	} else {
		/* No charger active */
		if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			get_prop_batt_current(chip, &ibat);
			pr_debug(
			"Stop: chg removed reg_loop = %d, fsm = %d ibat = %d\n",
				pm_chg_get_regulation_loop(chip),
				pm_chg_get_fsm_state(chip), ibat);
			return;
		} else {
			goto check_again_later;
		}
	}

	/* AICL only for usb wall charger */
	if ((active_path & USB_ACTIVE_BIT) && usb_target_ma > 0 &&
		!chip->disable_aicl) {
		reg_loop = pm_chg_get_regulation_loop(chip);
		pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);
		if ((reg_loop & VIN_ACTIVE_BIT) &&
			(usb_ma > USB_WALL_THRESHOLD_MA)
			&& !charging_disabled) {
			decrease_usb_ma_value(&usb_ma);
			usb_target_ma = usb_ma;
			/* end AICL here */
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
				usb_ma, usb_target_ma);
		}
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	rc = get_prop_batt_current(chip, &ibat);
	if ((reg_loop & VIN_ACTIVE_BIT) && !chip->disable_chg_rmvl_wrkarnd) {
		if (ibat > 0 && !rc) {
			pr_debug("revboost ibat = %d fsm = %d loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
			attempt_reverse_boost_fix(chip);
			/* after reverse boost fix check if the active
			 * charger was detected as removed */
			active_chg_plugged_in
				= is_active_chg_plugged_in(chip,
					active_path);
			pr_debug("revboost post: active_chg_plugged_in = %d\n",
					active_chg_plugged_in);
		}
	}

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg = %d\n",
			active_path, active_chg_plugged_in);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && active_chg_plugged_in == 1 &&
					!chip->disable_chg_rmvl_wrkarnd) {
		pr_debug("chg_gone=%d, active_chg_plugged_in = %d\n",
					chg_gone, active_chg_plugged_in);
		unplug_ovp_fet_open(chip);
	}

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg_plugged_in = %d\n",
			active_path, active_chg_plugged_in);
	if (active_path & USB_ACTIVE_BIT) {
		pr_debug("USB charger active\n");

		pm_chg_iusbmax_get(chip, &usb_ma);
	} else if (active_path & DC_ACTIVE_BIT) {
		pr_debug("DC charger active\n");
	} else {
		/* No charger active */
		if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			get_prop_batt_current(chip, &ibat);
			pr_debug(
			"Stop: chg removed reg_loop = %d, fsm = %d ibat = %d\n",
				pm_chg_get_regulation_loop(chip),
				pm_chg_get_fsm_state(chip),
				ibat
				);
			return;
		} else {
			goto check_again_later;
		}
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	/* AICL only for usb wall charger */
	if (!(reg_loop & VIN_ACTIVE_BIT) && (active_path & USB_ACTIVE_BIT)
		&& usb_target_ma > 0
		&& !charging_disabled
		&& !chip->disable_aicl) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			if (usb_ma > usb_target_ma)
				usb_ma = usb_target_ma;
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
			ramp = true;
		} else {
			usb_target_ma = usb_ma;
		}
	}
check_again_later:
	pr_debug("ramp: %d\n", ramp);
	/* schedule to check again later */
	if (ramp)
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
	else
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
}

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	schedule_work(&chip->unplug_check_work.work);
	return IRQ_HANDLED;
}

struct ibatmax_max_adj_entry {
	int ibat_max_ma;
	int max_adj_ma;
};

static struct ibatmax_max_adj_entry ibatmax_adj_table[] = {
	{975, 300},
	{1475, 150},
	{1975, 200},
	{2475, 250},
};

static int find_ibat_max_adj_ma(int ibat_target_ma)
{
	int i = 0;

	for (i = ARRAY_SIZE(ibatmax_adj_table); i > 0; i--) {
		if (ibat_target_ma >= ibatmax_adj_table[i - 1].ibat_max_ma)
			break;
	}

	if (i > 0)
		i--;

	return ibatmax_adj_table[i].max_adj_ma;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;


	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}

	pr_debug("fastchg_irq_handler() FASTCHG_IRQ %d\n", high_transition);
	if (high_transition){
		if (!is_oem_fast_chg_expired){
			schedule_work(&fast_chg_count_start_work);
			pr_debug("fastchg_irq_handler() -schedule_work(&fast_chg_count_start_work)\n");
		}
	}else{
		schedule_work(&fast_chg_count_stop_work);
	}

	if (high_transition
		&& chip->btc_override
		&& !delayed_work_pending(&chip->btc_override_work)) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
	}
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_debug("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_debug("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

	if (bat_temp_ok)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

	pm_chg_failed_clear(chip, 1);
	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);

	if (chip->dc_present ^ dc_present)
		pm8921_bms_calibrate_hkadc();

	if (dc_present)
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	else
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);

	chip->dc_present = dc_present;

	if (chip->ext_psy) {
		if (dc_present)
			handle_start_ext_chg(chip);
		else
			handle_stop_ext_chg(chip);
	} else {
		if (dc_present)
			schedule_delayed_work(&chip->unplug_check_work,
				msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
		power_supply_changed(&chip->dc_psy);
	}

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);

	if (oem_stand_detect_time_counter >= 1) {
		oem_pm8921_disable_source_current(false);
		oem_stand_detect_time_counter = 0;
	}

	return IRQ_HANDLED;
}

static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	if (!the_chip)
		return;

	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#define LOW_SOC_HEARTBEAT_MS	20000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);
	bool chg_present = chip->usb_present || chip->dc_present;

	/* for battery health when charger is not connected */
	if (chip->btc_override && !chg_present)
		schedule_delayed_work(&chip->btc_override_work,
			round_jiffies_relative(msecs_to_jiffies
					(chip->btc_delay_ms)));

	/*
	 * check temp thresholds when charger is present and
	 * and battery is FULL. The temperature here can impact
	 * the charging restart conditions.
	 */
	if (chip->btc_override && chg_present &&
				!wake_lock_active(&chip->eoc_wake_lock))
		check_temp_thresholds(chip);

	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);

#define MIN_DELTA_MV_TO_INCREASE_VDD_MAX	13
#define PM8921_CHG_VDDMAX_RES_MV	10
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip,
						int vbat_batt_terminal_uv)
{
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
		pr_debug("Exiting as rconn_mohm is 0\n");
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	if (delta_mv > 0) /* meaning we want to increase the vddmax */ {
		if (delta_mv < MIN_DELTA_MV_TO_INCREASE_VDD_MAX) {
			pr_debug("vterm = %d is not low enough to inc vdd\n",
							vbat_batt_terminal_mv);
			return;
		}
	}

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	adj_vdd_max_mv = (adj_vdd_max_mv / PM8921_CHG_VDDMAX_RES_MV)
						* PM8921_CHG_VDDMAX_RES_MV;

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;
	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

static void set_appropriate_vbatdet(struct pm8921_chg_chip *chip)
{
	if (chip->is_bat_cool)
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	else if (chip->is_bat_warm)
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	else
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
}

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);

	pm_chg_ibatmax_set(the_chip, chg_current);
}

#define TEMP_HYSTERISIS_DECIDEGC 20
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);

	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void check_temp_thresholds(struct pm8921_chg_chip *chip)
{
	int temp = 0, rc;

	rc = get_prop_batt_temp(chip, &temp);
	pr_debug("temp = %d, warm_thr_temp = %d, cool_thr_temp = %d\n",
			temp, chip->warm_temp_dc,
			chip->cool_temp_dc);

	if (chip->warm_temp_dc != INT_MIN) {
		if (chip->is_bat_warm
			&& temp < chip->warm_temp_dc - chip->hysteresis_temp_dc)
			battery_warm(false);
		else if (!chip->is_bat_warm && temp >= chip->warm_temp_dc)
			battery_warm(true);
	}

	if (chip->cool_temp_dc != INT_MIN) {
		if (chip->is_bat_cool
			&& temp > chip->cool_temp_dc + chip->hysteresis_temp_dc)
			battery_cool(false);
		else if (!chip->is_bat_cool && temp <= chip->cool_temp_dc)
			battery_cool(true);
	}
}

enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};

#define VBAT_TOLERANCE_MV	70
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip,
			int vbat_batt_terminal_uv, int ichg_meas_ma)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int vbat_programmed, iterm_programmed, vbat_intended;
#else
	int vbat_programmed, iterm_programmed;
#endif
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;
	int vbatdet_low;

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		vbatdet_low = oem_get_vbatdet_low(chip);
		pr_debug("vbatdet_low = %d\n", vbatdet_low);
		if (vbatdet_low == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_batt_terminal_uv=%d\n",
			 vbat_programmed, vbat_batt_terminal_uv);

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

#ifdef QUALCOMM_ORIGINAL_FEATURE
		if (chip->is_bat_cool)
			vbat_intended = chip->cool_bat_voltage;
		else if (chip->is_bat_warm)
			vbat_intended = chip->warm_bat_voltage;
		else
			vbat_intended = chip->max_voltage_mv;

		if (vbat_batt_terminal_uv / 1000
			< vbat_intended - MIN_DELTA_MV_TO_INCREASE_VDD_MAX) {
			pr_debug("terminal_uv:%d < vbat_intended:%d-hyst:%d\n",
							vbat_batt_terminal_uv,
							vbat_intended,
							vbat_intended);
			return CHG_IN_PROGRESS;
		}
#endif

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

	return CHG_FINISHED;
}

#define COMP_OVERRIDE_HOT_BANK	6
#define COMP_OVERRIDE_COLD_BANK	7
#define COMP_OVERRIDE_BIT  BIT(1)
static int pm_chg_override_cold(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_COLD_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc cold = %d val = 0x%x\n", flag, val);
	return rc;
}

static int pm_chg_override_hot(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_HOT_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc hot = %d val = 0x%x\n", flag, val);
	return rc;
}

static void __devinit pm8921_chg_btc_override_init(struct pm8921_chg_chip *chip)
{
	int rc = 0;
	u8 reg;
	u8 val;

	val = COMP_OVERRIDE_HOT_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		goto cold_init;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_HOT_BANK, rc);
		goto cold_init;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not hot */
		rc = pm_chg_override_hot(chip, 0);
		if (rc < 0)
			pr_err("Could not override hot rc = %d\n", rc);
	}

cold_init:
	val = COMP_OVERRIDE_COLD_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_COLD_BANK, rc);
		return;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not cold */
		rc = pm_chg_override_cold(chip, 0);
		if (rc < 0)
			pr_err("Could not override cold rc = %d\n", rc);
	}
}

static void btc_override_worker(struct work_struct *work)
{
	int decidegc;
	int temp;
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, btc_override_work);

	if (!chip->btc_override) {
		pr_err("called when not enabled\n");
		return;
	}

	rc = get_prop_batt_temp(chip, &decidegc);
	if (rc) {
		pr_info("Failed to read temperature\n");
		goto fail_btc_temp;
	}

	pr_debug("temp=%d\n", decidegc);

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp) {
		if (decidegc < chip->btc_override_hot_decidegc -
				chip->hysteresis_temp_dc)
			/* stop forcing batt hot */
			rc = pm_chg_override_hot(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to hot comp\n");
	} else {
		if (decidegc >= chip->btc_override_hot_decidegc)
			/* start forcing batt hot */
			rc = pm_chg_override_hot(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp) {
		if (decidegc > chip->btc_override_cold_decidegc +
				chip->hysteresis_temp_dc)
			/* stop forcing batt cold */
			rc = pm_chg_override_cold(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to cold comp\n");
	} else {
		if (decidegc <= chip->btc_override_cold_decidegc)
			/* start forcing batt cold */
			rc = pm_chg_override_cold(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	if ((is_dc_chg_plugged_in(the_chip) || is_usb_chg_plugged_in(the_chip))
		&& get_prop_batt_status(chip) != POWER_SUPPLY_STATUS_FULL) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
		return;
	}

fail_btc_temp:
	rc = pm_chg_override_hot(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to hot comp\n");
	rc = pm_chg_override_cold(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to cold comp\n");
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;
	int vbat_meas_uv, vbat_meas_mv;
	int ichg_meas_ua, ichg_meas_ma;
	int vbat_batt_terminal_uv;

	pm8921_bms_get_simultaneous_battery_voltage_and_current(
					&ichg_meas_ua,	&vbat_meas_uv);
	vbat_meas_mv = vbat_meas_uv / 1000;
	/* rconn_mohm is in milliOhms */
	ichg_meas_ma = ichg_meas_ua / 1000;
	vbat_batt_terminal_uv = vbat_meas_uv
					+ ichg_meas_ma
					* the_chip->rconn_mohm;

	end = is_charging_finished(chip, vbat_batt_terminal_uv, ichg_meas_ma);

	if (end == CHG_NOT_IN_PROGRESS && (!chip->btc_override ||
		!(chip->usb_present || chip->dc_present))) {
		count = 0;
		goto eoc_worker_stop;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");

		oem_voltage_setting(VOLTAGE_DELTA_RESUME);

		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;
		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
	} else {
		check_temp_thresholds(chip);
		if (end != CHG_NOT_IN_PROGRESS)
			adjust_vdd_max_for_fastchg(chip, vbat_batt_terminal_uv);
		pr_debug("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
		return;
	}

eoc_worker_stop:
	/* set the vbatdet back, in case it was changed to trigger charging */
	set_appropriate_vbatdet(chip);
	wake_unlock(&chip->eoc_wake_lock);
}

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}

	set_appropriate_battery_current(chip);
	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
			pm8921_charger_vbus_draw(usb_max_current);
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

#define PM8921_USB_TRIM_SEL_BIT		BIT(6)
/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	int fsm_state;
	int is_fast_chg;
	int rc = 0;
	u8 trim_sel_reg = 0, regsbi;

	chip->dc_present = !!is_dc_chg_plugged_in(chip);
	chip->usb_present = !!is_usb_chg_plugged_in(chip);

	notify_usb_of_the_plugin_event(chip->usb_present);
	if (chip->usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);

		if (chip->btc_override)
			schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
	}

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

	pm8921_chg_enable_irq(chip, TRKLCHG_IRQ);
	pm8921_chg_enable_irq(chip, CHGSTATE_IRQ);

	if (get_prop_batt_present(the_chip) || is_dc_chg_plugged_in(the_chip))
		if (usb_chg_current)
			/*
			 * Reissue a vbus draw call only if a battery
			 * or DC is present. We don't want to brown out the
			 * device if usb is its only source
			 */
			__pm8921_charger_vbus_draw(usb_chg_current);
	usb_chg_current = 0;

	/*
	 * The bootloader could have started charging, a fastchg interrupt
	 * might not happen. Check the real time status and if it is fast
	 * charging invoke the handler so that the eoc worker could be
	 * started
	 */
	is_fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (is_fast_chg)
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);

	fsm_state = pm_chg_get_fsm_state(chip);
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
		pm8921_bms_charging_began();
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);

	/* Determine which USB trim column to use */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917) {
		chip->usb_trim_table = usb_trim_8917_table;
	} else if (pm8xxx_get_version(chip->dev->parent) ==
						PM8XXX_VERSION_8038) {
		chip->usb_trim_table = usb_trim_8038_table;
	} else if (pm8xxx_get_version(chip->dev->parent) ==
						PM8XXX_VERSION_8921) {
		rc = pm8xxx_readb(chip->dev->parent, REG_SBI_CONFIG, &regsbi);
		rc |= pm8xxx_writeb(chip->dev->parent, REG_SBI_CONFIG, 0x5E);
		rc |= pm8xxx_readb(chip->dev->parent, PM8921_USB_TRIM_SEL,
								&trim_sel_reg);
		rc |= pm8xxx_writeb(chip->dev->parent, REG_SBI_CONFIG, regsbi);
		if (rc)
			pr_err("Failed to read trim sel register rc=%d\n", rc);

		if (trim_sel_reg & PM8921_USB_TRIM_SEL_BIT)
			chip->usb_trim_table = usb_trim_pm8921_table_1;
		else
			chip->usb_trim_table = usb_trim_pm8921_table_2;
	}
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
	CHG_IRQ(USBIN_OV_IRQ, IRQF_TRIGGER_RISING, usbin_ov_irq_handler),
	CHG_IRQ(USBIN_UV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							usbin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

static void pm8921_chg_force_19p2mhz_clk(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD5;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	udelay(183);

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
	udelay(32);

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

static void pm8921_chg_set_hw_clk_switching(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define SAFE_CURRENT_MA		1500
#define VREF_BATT_THERM_FORCE_ON_2	BIT(4)
#define PM_SUB_REV		0x001
#define MIN_CHARGE_CURRENT_MA	350
#define DEFAULT_SAFETY_MINUTES	500
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	u8 subrev;
	int rc, vdd_safe, fcc_uah, safety_time = DEFAULT_SAFETY_MINUTES;

	/* forcing 19p2mhz before accessing any charger registers */
	pm8921_chg_force_19p2mhz_clk(chip);

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_BIT, BOOT_DONE_BIT);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}

	if (chip->safe_current_ma == 0)
		chip->safe_current_ma = SAFE_CURRENT_MA;

	rc = pm_chg_ibatsafe_set(chip, chip->safe_current_ma);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}

	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

	fcc_uah = pm8921_bms_get_fcc();
	if (fcc_uah > 0) {
		safety_time = div_s64((s64)fcc_uah * 60,
						1000 * MIN_CHARGE_CURRENT_MA);
		/* add 20 minutes of buffer time */
		safety_time += 20;

		/* make sure we do not exceed the maximum programmable time */
		if (safety_time > PM8921_CHG_TCHG_MAX)
			safety_time = PM8921_CHG_TCHG_MAX;
	}

	rc = pm_chg_tchg_max_set(chip, safety_time);
	if (rc) {
		pr_err("Failed to set max time to %d minutes rc=%d\n",
						safety_time, rc);
		return rc;
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->ttrkl_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	if (pm8xxx_get_revision(chip->dev->parent) >= PM8XXX_REVISION_8038_1p0)
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL_8038, 0x15);
	else
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL, 0x15);

	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0
	&& pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8921) {
		rc = pm8xxx_readb(chip->dev->parent, PM_SUB_REV, &subrev);
		if (rc) {
			pr_err("read failed: addr=%03X, rc=%d\n",
				PM_SUB_REV, rc);
			return rc;
		}
		/* Check if die 3.0.1 is present */
		if (subrev & 0x1)
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xA4);
		else
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xAC);
	}

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917) {
		/* Set PM8917 USB_OVP debounce time to 15 ms */
		rc = pm_chg_masked_write(chip, USB_OVP_CONTROL,
			OVP_DEBOUNCE_TIME, 0x6);
		if (rc) {
			pr_err("Failed to set USB OVP db rc=%d\n", rc);
			return rc;
		}

		/* Enable isub_fine resolution AICL for PM8917 */
		chip->iusb_fine_res = true;
		if (chip->uvd_voltage_mv) {
			rc = pm_chg_uvd_threshold_set(chip,
					chip->uvd_voltage_mv);
			if (rc) {
				pr_err("Failed to set UVD threshold %drc=%d\n",
						chip->uvd_voltage_mv, rc);
				return rc;
			}
		}
	}

	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);

	rc = pm_chg_charge_dis(chip, charging_disabled);
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

	return 0;
}

enum pm8921_dc_ov_threshold {
	PM_DC_OV_8P5V,
	PM_DC_OV_9V,
	PM_DC_OV_9P5V,
	PM_DC_OV_10V,
};

#define DC_OV_THRESHOLD_MASK 0x60
#define DC_OV_THRESHOLD_SHIFT 5
int pm8921_dc_ovp_set_threshold(enum pm8921_dc_ov_threshold ov) {
	u8 temp;

  if (!the_chip)
	{
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_DC_OV_10V)
	{
		pr_err("limiting to over voltage threshold to 7volts\n");
	        ov = PM_DC_OV_10V;
	}

	temp = DC_OV_THRESHOLD_MASK & (ov << DC_OV_THRESHOLD_SHIFT);
	temp |= 0x80;
	return pm_chg_masked_write(the_chip, DC_OVP_CONTROL, DC_OV_THRESHOLD_MASK, temp);
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm_chg_write(the_chip, addr, temp);
	if (ret) {
		pr_err("pm_chg_write to %x value =%d errored = %d\n",
			addr, temp, ret);
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");



static int reg_loop;
#define MAX_REG_LOOP_CHAR	10
static int get_reg_loop_param(char *buf, struct kernel_param *kp)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	return snprintf(buf, MAX_REG_LOOP_CHAR, "%d", temp);
}
module_param_call(reg_loop, NULL, get_reg_loop_param,
					&reg_loop, 0644);

static int max_chg_ma;
#define MAX_MA_CHAR	10
static int get_max_chg_ma_param(char *buf, struct kernel_param *kp)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return snprintf(buf, MAX_MA_CHAR, "%d", the_chip->max_bat_chg_current);
}
module_param_call(max_chg_ma, NULL, get_max_chg_ma_param,
					&max_chg_ma, 0644);
static int ibatmax_ma;
static int set_ibat_max(const char *val, struct kernel_param *kp)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("error setting value %d\n", rc);
		return rc;
	}

	if (abs(ibatmax_ma - the_chip->max_bat_chg_current)
				<= the_chip->ibatmax_max_adj_ma) {
		rc = pm_chg_ibatmax_set(the_chip, ibatmax_ma);
		if (rc) {
			pr_err("Failed to set ibatmax rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}
static int get_ibat_max(char *buf, struct kernel_param *kp)
{
	int ibat_ma;
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = pm_chg_ibatmax_get(the_chip, &ibat_ma);
	if (rc) {
		pr_err("ibatmax_get error = %d\n", rc);
		return rc;
	}

	return snprintf(buf, MAX_MA_CHAR, "%d", ibat_ma);
}
module_param_call(ibatmax_ma, set_ibat_max, get_ibat_max,
					&ibatmax_ma, 0644);
enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

static int pm8921_charger_suspend_noirq(struct device *dev)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int rc;
#endif
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef QUALCOMM_ORIGINAL_FEATURE
	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);

	rc = pm8921_chg_set_lpm(chip, 1);
	if (rc)
		pr_err("Failed to set lpm rc=%d\n", rc);
#endif

	pm8921_chg_set_hw_clk_switching(chip);

	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
#ifdef QUALCOMM_ORIGINAL_FEATURE
	int rc;
#endif
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

#ifdef QUALCOMM_ORIGINAL_FEATURE
	rc = pm8921_chg_set_lpm(chip, 0);
	if (rc)
		pr_err("Failed to set lpm rc=%d\n", rc);
#endif

	pm8921_chg_force_19p2mhz_clk(chip);

#ifdef QUALCOMM_ORIGINAL_FEATURE
	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
#endif
	return 0;
}

static int pm8921_charger_resume(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}

	if (chip->btc_override && (is_dc_chg_plugged_in(the_chip) ||
					is_usb_chg_plugged_in(the_chip)))
		schedule_delayed_work(&chip->btc_override_work, 0);

	schedule_delayed_work(&chip->update_heartbeat_work, 0);

	wake_lock(&oem_hkadc_wake_lock);
	is_hkadc_suspend_monit = 1;

	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->update_heartbeat_work);

	if (chip->btc_override)
		cancel_delayed_work_sync(&chip->btc_override_work);

	if (is_usb_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

	return 0;
}

static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;
#ifdef CONFIG_CHARGE_STAND_SUPPORT
	int gpio_trigger_tbatt2 = 0;
	int gpio_trigger_det_n = 0;
	bool boot_stand_flag = true;
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	oem_chg_param_init();
	oem_get_fact_option();
	oem_uim_smem_init();

	chip->dev = &pdev->dev;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->ttrkl_time = pdata->ttrkl_time;
#else
	chip->ttrkl_time = oem_param_charger.time_trkl_pm;
#endif
	chip->update_time = pdata->update_time;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->max_voltage_mv = pdata->max_voltage;
#else
	oem_uim_check(&chip->max_voltage_mv);
#endif
	chip->alarm_low_mv = pdata->alarm_low_mv;
	chip->alarm_high_mv = pdata->alarm_high_mv;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->safe_current_ma = pdata->safe_current_ma;
	chip->uvd_voltage_mv = pdata->uvd_thresh_voltage;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
#else
	chip->resume_voltage_delta = oem_param_charger.initial_delta_volt;
#endif
	chip->resume_charge_percent = pdata->resume_charge_percent;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->term_current = pdata->term_current;
#else
	chip->term_current = oem_param_charger.i_chg_finish;
#endif
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;
	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;
#else
	if (oem_param_charger.chg_cool_tmp != INT_MIN)
		chip->cool_temp_dc = oem_param_charger.chg_cool_tmp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (oem_param_charger.chg_warm_tmp != INT_MIN)
		chip->warm_temp_dc = oem_param_charger.chg_warm_tmp * 10;
	else
		chip->warm_temp_dc = INT_MIN;
#endif
	if (pdata->hysteresis_temp)
		chip->hysteresis_temp_dc = pdata->hysteresis_temp * 10;
	else
		chip->hysteresis_temp_dc = TEMP_HYSTERISIS_DECIDEGC;

	chip->temp_check_period = pdata->temp_check_period;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
#else
	chip->max_bat_chg_current = oem_param_charger.i_chg_norm;
#endif
	/* Assign to corresponding module parameter */
	usb_max_current = pdata->usb_max_current;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
#else
	chip->cool_bat_chg_current = oem_param_charger.i_chg_cool;
	chip->warm_bat_chg_current = oem_param_charger.i_chg_warm;
	chip->cool_bat_voltage = oem_param_charger.cool_chg;
	chip->warm_bat_voltage = oem_param_charger.warm_chg;
#endif
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->vin_min = pdata->vin_min;
#else
	chip->vin_min = oem_param_charger.maint_chg_vin;
#endif
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;
	chip->disable_chg_rmvl_wrkarnd = pdata->disable_chg_rmvl_wrkarnd;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
#ifdef QUALCOMM_ORIGINAL_FEATURE
	chip->rconn_mohm = pdata->rconn_mohm;
#else
	chip->rconn_mohm = oem_param_charger.rconn_mohm;
#endif
	chip->led_src_config = pdata->led_src_config;
	chip->has_dc_supply = pdata->has_dc_supply;
	chip->battery_less_hardware = pdata->battery_less_hardware;
	chip->btc_override = pdata->btc_override;
	if (chip->btc_override) {
		chip->btc_delay_ms = pdata->btc_delay_ms;
		chip->btc_override_cold_decidegc
			= pdata->btc_override_cold_degc * 10;
		chip->btc_override_hot_decidegc
			= pdata->btc_override_hot_degc * 10;
		chip->btc_panic_if_cant_stop_chg
			= pdata->btc_panic_if_cant_stop_chg;
	}

	if (chip->battery_less_hardware)
		charging_disabled = 1;

	if (oem_param_charger.chg_disable_test == 0x01)
		charging_disabled = 1;

	chip->ibatmax_max_adj_ma = find_ibat_max_adj_ma(
					chip->max_bat_chg_current);

	vdd_max_increase_mv = oem_param_charger.vdd_max_inc_mv;

	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

	if (chip->btc_override)
		pm8921_chg_btc_override_init(chip);

	chip->stop_chg_upon_expiry = pdata->stop_chg_upon_expiry;
	chip->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;

	chip->usb_psy.name = "usb";
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	chip->usb_psy.supplied_to = pm_power_supplied_to;
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->usb_psy.properties = pm_power_props_usb;
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb);
	chip->usb_psy.get_property = pm_power_get_property_usb;
	chip->usb_psy.set_property = pm_power_set_property_usb;
	chip->usb_psy.property_is_writeable = usb_property_is_writeable;

	chip->dc_psy.name = "pm8921-dc";
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->dc_psy.supplied_to = pm_power_supplied_to;
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->dc_psy.properties = pm_power_props_mains;
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
	chip->dc_psy.get_property = pm_power_get_property_mains;

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = msm_batt_power_props;
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props);
	chip->batt_psy.get_property = pm_batt_power_get_property;
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed;
	
	chip->bms_psy.name = "bms";
	chip->bms_psy.type = POWER_SUPPLY_TYPE_BMS;
	chip->bms_psy.properties = msm_bms_power_props;
	chip->bms_psy.num_properties = ARRAY_SIZE(msm_bms_power_props);
	chip->bms_psy.get_property = pm_bms_power_get_property;
	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}

	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto unregister_usb;
	}

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}

	rc = power_supply_register(chip->dev, &chip->bms_psy);
	if (rc < 0) {
		pr_err("power_supply_register bms failed rc = %d\n", rc);
		goto unregister_batt;
	}

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);

	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&chip->btc_override_work, btc_override_worker);

	wake_lock_init(&oem_charge_wake_lock, WAKE_LOCK_SUSPEND, "oem_charge");
	INIT_WORK(&fast_chg_count_start_work, fast_chg_count_start_worker);
	INIT_WORK(&fast_chg_count_stop_work, fast_chg_count_stop_worker);
	INIT_WORK(&fast_chg_count_stop_work_usb, fast_chg_count_stop_worker_usb);
	INIT_WORK(&chg_vbatt_ov_work, chg_vbatt_ov_worker);
	INIT_DELAYED_WORK(&fast_chg_limit_work, fast_chg_limit_worker);

	rc = pm8921_usb_ovp_set_threshold(PM_USB_OV_6V);
	if (rc) {
		pr_err("couldn't perform pm8921_usb_ovp_set_threshold() rc=%d\n", rc);
		goto unregister_batt;
	}

	rc = pm8921_dc_ovp_set_threshold(PM_DC_OV_8P5V);
	if (rc) {
		pr_err("couldn't perform pm8921_dc_ovp_set_threshold() rc=%d\n", rc);
		goto unregister_batt;
	}

	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_bms;
	}

#ifdef QUALCOMM_ORIGINAL_FEATURE
	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);
#else
	rc = enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	rc |= enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	rc |= enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	rc |= enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);

	rc |= enable_irq_wake(chip->pmic_chg_irq[USBIN_OV_IRQ]);
	rc |= enable_irq_wake(chip->pmic_chg_irq[USBIN_UV_IRQ]);

	if (rc) {
		pr_err("couldn't enable wake irqs rc=%d\n", rc);
		goto free_irq;
	}
#endif

	create_debugfs_entries(chip);

	/* determine what state the charger is in */
	determine_initial_state(chip);

	if (chip->update_time)
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));

	INIT_DELAYED_WORK(&oem_charger_work, oem_charger_limit_control);
	oem_hkadc_init();
	oem_chargermonit_init();

#ifdef CONFIG_CHARGE_STAND_SUPPORT
	wake_lock_init(&charging_stand_wake_lock, WAKE_LOCK_SUSPEND, "charging_stand");
	INIT_DELAYED_WORK(&oem_chg_tbatt2_work, oem_chg_tbatt2_control);
	INIT_DELAYED_WORK(&oem_chg_det_n_work, oem_chg_det_n_control);
	INIT_DELAYED_WORK(&oem_bat_ovp_n_work, oem_bat_ovp_n_control);

	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_TBATT2, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_DET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_BAT_OVP_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		gpio_trigger_tbatt2 = IRQF_TRIGGER_HIGH;
	} else {
		gpio_trigger_tbatt2 = IRQF_TRIGGER_LOW;
		boot_stand_flag = false;
	}
	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		gpio_trigger_det_n = IRQF_TRIGGER_HIGH;
	} else {
		gpio_trigger_det_n = IRQF_TRIGGER_LOW;
		boot_stand_flag = false;
	}

	if (boot_stand_flag) {
		oem_tbatt2_detect = DETECT_LO_FIX;
		oem_det_n_detect = DETECT_LO_FIX;
		oem_stand_detect = DETECT_STAND;
		wake_lock(&charging_stand_wake_lock);
		oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
	}

	rc = request_irq(gpio_to_irq(GPIO_CHG_TBATT2), oem_chg_tbatt2_isr, gpio_trigger_tbatt2, "CHG_TBATT2", 0);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_bms;
	}
	rc = request_irq(gpio_to_irq(GPIO_CHG_DET_N), oem_chg_det_n_isr, gpio_trigger_det_n, "CHG_DET_N", 0);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_bms;
	}
	rc = request_irq(gpio_to_irq(GPIO_BAT_OVP_N), oem_bat_ovp_n_isr, IRQF_TRIGGER_LOW, "BAT_OVP_N", 0);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_bms;
	}
	enable_irq_wake(gpio_to_irq(GPIO_CHG_TBATT2));
	enable_irq_wake(gpio_to_irq(GPIO_CHG_DET_N));
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

	chg_vbatt_ov_check_charger();

	if (pm_chg_get_rt_status(the_chip, USBIN_VALID_IRQ)) {
		wake_lock(&oem_charge_wake_lock);
	}

	pr_info("OK\n");

	return 0;

free_irq:
	free_irqs(chip);
unregister_bms:
	power_supply_unregister(&chip->bms_psy);
unregister_batt:
	wake_lock_destroy(&chip->eoc_wake_lock);
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

	oem_hkadc_exit();
	oem_chargermonit_exit();

	free_irqs(chip);
	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
			.pm	= &pm8921_pm_ops,
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);

int oem_pm8921_disable_source_current(bool disable)
{
	int ret;
	int usb_ma;

	if (disable) {
		pm_chg_iusbmax_get(the_chip, &usb_ma);
		if (usb_ma > 1000)
			__pm8921_charger_vbus_draw(1000);
		mdelay(1);
		if (usb_ma > 500)
			__pm8921_charger_vbus_draw(500);
		mdelay(1);
}

	ret = pm8921_disable_source_current(disable);
	if (ret) {
		pr_err("buck converter setting error %d\n", ret);
		return ret;
	}

	if (disable) {
		oem_pm8921_disable_source_current_flag = 1;
	}
	else {
		if(oem_iusbmax_current == OEM_IUSBMAX_NOT_SET) {
			pr_err("oem_iusbmax_current is not set\n");
		}
		else {
			usb_target_ma = oem_iusbmax_current;
			pm8921_charger_vbus_draw(oem_iusbmax_current);
		}
	}

	return ret;
}

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static int oem_chg_stand_off(void);
#endif /* CONFIG_CHARGE_STAND_SUPPORT */
static void fast_chg_limit_worker(struct work_struct *work)
{
	int ret;

	pr_debug("fast_chg_limit_worker() called.\n");

	is_oem_fast_chg_counting=false;

	if (1 == oem_charge_stand_flag) {
#ifdef CONFIG_CHARGE_STAND_SUPPORT
		oem_chg_stand_off();
#endif /* CONFIG_CHARGE_STAND_SUPPORT */
		schedule_work(&fast_chg_count_start_work);
	}else{
		is_oem_fast_chg_expired=true;

		if (OEM_STATUS_LIMIT == oem_charge_status) {
			oem_charge_status = OEM_STATUS_NORMAL;
			cancel_delayed_work_sync(&oem_charger_work);
		}

		ret = oem_pm8921_disable_source_current(false);
		if (ret) {
			pr_err("error buck converter setting value %d\n", ret);
		}

		ret = pm_chg_auto_enable(the_chip, false);
		if (ret) {
			pr_err("error bat_fet setting value %d\n", ret);
		}
	}

}

#define EOC_CHECK_VOLTAGE_DELTA	100
static int oem_get_vbatdet_low(struct pm8921_chg_chip *chip)
{
	int battery_voltage;
	int set_voltage_mv;
	int ret;

	ret = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_VOLTAGE_NOW, &battery_voltage);
	if (ret) {
		pr_err("Failed to reading battery voltage, ret = %d\n", ret);
		return 1;
	}

	if (chip->is_bat_warm) {
		set_voltage_mv = chip->warm_bat_voltage;
	} else if (chip->is_bat_cool) {
		set_voltage_mv = chip->cool_bat_voltage;
	} else {
		set_voltage_mv = chip->max_voltage_mv;
	}

	if (battery_voltage >= ((set_voltage_mv - EOC_CHECK_VOLTAGE_DELTA) * 1000)) {
		return 0;
	} else {
		return 1;
	}
}

static uint32_t *uim_status_smem_ptr = NULL;
static void oem_uim_smem_init(void)
{
	uim_status_smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_UICC_INFO, 1);
	if (NULL == uim_status_smem_ptr) {
		pr_err("oem_uim_smem_init() smem uicc read error.\n");
		return;
	}
}

static bool is_oem_uim_status(void)
{
	if (NULL == uim_status_smem_ptr)
	{
		pr_err("is_oem_uim_status() smem uicc read error.\n");
		return false;
	}

	if ((0 == *uim_status_smem_ptr) || (1 == *uim_status_smem_ptr))
	{
		return true;
	}
	return false;
}

static void oem_uim_check(unsigned int* max_voltage_normal)
{
	if (oem_option_item1_bit4) {
		charge_uim_valid = 1;
		*max_voltage_normal = oem_param_charger.normal_chg;
		return;
	}

	if (!is_oem_uim_status()) {
		charge_uim_valid = 0;
		*max_voltage_normal = oem_param_charger.uim_undete_chg;
	} else {
		charge_uim_valid = 1;
		*max_voltage_normal = oem_param_charger.normal_chg;
		}
	return;
}

static int oem_call_state = 0;
module_param(oem_call_state, int, 0644);

#define OEM_CHARGE_FACT_LIMIT_TEMP 90
#define OEM_CHARGE_FACT_STOP_TEMP  95

#define OEM_CORRECTION_VALUE \
	(oem_charge_stand_flag ? (oem_param_charger.chg_adp_tmp_delta) : 0)

#define OEM_INTE_CAM_ON_TELE_ON   (30)
#define OEM_INTE_CAM_OFF_TELE_ON  (27)

static uint32_t camera_therm_status = OEM_STATUS_NORMAL;
static void oem_camera_therm_set_status(int temp)
{
	int8_t correction_value = OEM_CORRECTION_VALUE;
	int32_t chg_cam_tmp_off_work;
	int32_t chg_inte_cam_on_work;
	int32_t chg_inte_cam_off_work;

	if (oem_option_item1_bit3) {
		chg_inte_cam_on_work = OEM_CHARGE_FACT_LIMIT_TEMP;
		chg_inte_cam_off_work = oem_param_charger.chg_inte_cam_off + correction_value;
		chg_cam_tmp_off_work = OEM_CHARGE_FACT_STOP_TEMP;
	} else  {
		if (oem_call_state == 1) {
			chg_inte_cam_on_work = OEM_INTE_CAM_ON_TELE_ON;
			chg_inte_cam_off_work = OEM_INTE_CAM_OFF_TELE_ON;
		} else {
			chg_inte_cam_on_work = oem_param_charger.chg_inte_cam_on + correction_value;
			chg_inte_cam_off_work = oem_param_charger.chg_inte_cam_off + correction_value;
		}
		chg_cam_tmp_off_work = oem_param_charger.chg_cam_tmp_off + correction_value;
	}

	switch (camera_therm_status) {

	case OEM_STATUS_NORMAL:
		if (chg_cam_tmp_off_work <= temp) {
			camera_therm_status = OEM_STATUS_STOP;
		} else if (chg_inte_cam_on_work <= temp) {
			camera_therm_status = OEM_STATUS_LIMIT;
		} else {
			camera_therm_status = OEM_STATUS_NORMAL;
	}
		break;

	case OEM_STATUS_LIMIT:
		if (chg_inte_cam_off_work >= temp) {
			camera_therm_status = OEM_STATUS_NORMAL;
		} else if (chg_cam_tmp_off_work <= temp) {
			camera_therm_status = OEM_STATUS_STOP;
		} else {
			camera_therm_status = OEM_STATUS_LIMIT;
		}
		break;

	case OEM_STATUS_STOP:
		if (chg_inte_cam_off_work >= temp) {
			camera_therm_status = OEM_STATUS_NORMAL;
		} else if (oem_param_charger.chg_cam_tmp_on + correction_value >= temp) {
			camera_therm_status = OEM_STATUS_LIMIT;
		} else {
			camera_therm_status = OEM_STATUS_STOP;
		}
		break;

	default:
		break;
	}

	pr_debug("camera_temp = %d, camera_therm_status = %d\n", temp, camera_therm_status);
	}

#define OEM_INTE_PHONE_ON_TELE_ON   (37)
#define OEM_INTE_PHONE_OFF_TELE_ON  (34)

static uint32_t substrate_therm_status = OEM_STATUS_NORMAL;
static void oem_substrate_therm_set_status(int temp)
{
	int8_t correction_value = OEM_CORRECTION_VALUE;
	int32_t chg_phone_tmp_off_work;
	int32_t chg_inte_phone_on_work;
	int32_t chg_inte_phone_off_work;

	if (oem_option_item1_bit3) {
		chg_inte_phone_on_work = OEM_CHARGE_FACT_LIMIT_TEMP;
		chg_inte_phone_off_work = oem_param_charger.chg_inte_phone_off + correction_value;
		chg_phone_tmp_off_work = OEM_CHARGE_FACT_STOP_TEMP;
	} else  {
		if (oem_call_state == 1) {
			chg_inte_phone_on_work = OEM_INTE_PHONE_ON_TELE_ON;
			chg_inte_phone_off_work = OEM_INTE_PHONE_OFF_TELE_ON;
		} else {
			chg_inte_phone_on_work = oem_param_charger.chg_inte_phone_on + correction_value;
			chg_inte_phone_off_work = oem_param_charger.chg_inte_phone_off + correction_value;
		}
		chg_phone_tmp_off_work = oem_param_charger.chg_phone_tmp_off + correction_value;
}

	switch (substrate_therm_status) {

	case OEM_STATUS_NORMAL:
		if (chg_phone_tmp_off_work <= temp) {
			substrate_therm_status = OEM_STATUS_STOP;
		} else if (chg_inte_phone_on_work <= temp) {
			substrate_therm_status = OEM_STATUS_LIMIT;
		} else {
			substrate_therm_status = OEM_STATUS_NORMAL;
}
		break;

	case OEM_STATUS_LIMIT:
		if (chg_inte_phone_off_work >= temp) {
			substrate_therm_status = OEM_STATUS_NORMAL;
		} else if (chg_phone_tmp_off_work <= temp) {
			substrate_therm_status = OEM_STATUS_STOP;
		} else {
			substrate_therm_status = OEM_STATUS_LIMIT;
		}
		break;

	case OEM_STATUS_STOP:
		if (chg_inte_phone_off_work >= temp) {
			substrate_therm_status = OEM_STATUS_NORMAL;
		} else if (oem_param_charger.chg_phone_tmp_on + correction_value >= temp) {
			substrate_therm_status = OEM_STATUS_LIMIT;
		} else {
			substrate_therm_status = OEM_STATUS_STOP;
		}
		break;

	default:
		break;
	}

	pr_debug("substrate_temp = %d, substrate_therm_status = %d\n", temp, substrate_therm_status);
}

#define OEM_USB_THERM_NORM_TO_STOP_TEMP  90
#define OEM_USB_THERM_STOP_TO_NORM_TEMP  50
static uint32_t usb_therm_status = OEM_STATUS_NORMAL;
static void oem_usb_therm_set_status(int temp)
{
	switch (usb_therm_status) {

	case OEM_STATUS_NORMAL:
		if (OEM_USB_THERM_NORM_TO_STOP_TEMP <= temp) {
			usb_therm_status = OEM_STATUS_STOP;
		} else {
			usb_therm_status = OEM_STATUS_NORMAL;
		}
		break;

	case OEM_STATUS_LIMIT:
		break;

	case OEM_STATUS_STOP:
		if (OEM_USB_THERM_STOP_TO_NORM_TEMP >= temp) {
			usb_therm_status = OEM_STATUS_NORMAL;
		} else {
			usb_therm_status = OEM_STATUS_STOP;
		}
		break;

	default:
		break;
	}

	pr_debug("usb_temp = %d, usb_therm_status = %d\n", temp, usb_therm_status);
}

static uint32_t battery_temp_status = OEM_STATUS_NORMAL;
static void oem_battery_temp_set_status(int temp)
{
	int16_t wait_chg_on_work;

	if (oem_option_item1_bit3) {
		wait_chg_on_work = OEM_CHARGE_FACT_STOP_TEMP * 10;
	} else  {
		wait_chg_on_work = oem_param_charger.wait_chg_on * 10;
	}

	switch (battery_temp_status) {

	case OEM_STATUS_NORMAL:
		if (wait_chg_on_work <= temp) {
			battery_temp_status = OEM_STATUS_LIMIT;
		} else {
			battery_temp_status = OEM_STATUS_NORMAL;
		}
		break;

	case OEM_STATUS_LIMIT:
		if ((oem_param_charger.wait_chg_off * 10) >= temp) {
			battery_temp_status = OEM_STATUS_NORMAL;
		} else {
			battery_temp_status = OEM_STATUS_LIMIT;
		}
		break;

	case OEM_STATUS_STOP:
		break;

	default:
		break;
	}

	pr_debug("battery_temp = %d, battery_temp_status = %d\n", temp, battery_temp_status);
}

#define CHARGE_STAND_LIMIT_VOLTAGE  4100
#define OEM_BATTERY_VOLTS_CORRECTION_VALUE \
	(oem_charge_stand_flag ? CHARGE_STAND_LIMIT_VOLTAGE : (oem_param_charger.maint_wait_chg_on_volt))
static uint32_t vbatt_status = OEM_STATUS_NORMAL;
static void oem_battery_volts_set_status(int vbat)
{
	switch (vbatt_status) {

	case OEM_STATUS_NORMAL:

		if ((OEM_BATTERY_VOLTS_CORRECTION_VALUE * 1000) <= vbat) {
			vbatt_status = OEM_STATUS_LIMIT;
		} else {
			vbatt_status = OEM_STATUS_NORMAL;
		}
		break;

	case OEM_STATUS_LIMIT:
		if ((oem_param_charger.maint_wait_chg_off_volt * 1000) >= vbat) {
			vbatt_status = OEM_STATUS_NORMAL;
		} else {
			vbatt_status = OEM_STATUS_LIMIT;
		}
		break;

	case OEM_STATUS_STOP:
		break;

	default:
		break;
	}

	pr_debug("battery_vol = %d, vbatt_status = %d\n", vbat, vbatt_status);
}

static void oem_charger_limit_control(struct work_struct *work)
{
	unsigned int charger_control_timer;

	if ((!pm_chg_get_rt_status(the_chip, FASTCHG_IRQ)) &&
	    (1 != oem_pm8921_disable_source_current_flag)) {
		oem_charge_status = OEM_STATUS_NORMAL;
		return;
	}

	if (0 != oem_charger_control_flag) {
		oem_pm8921_disable_source_current(false);
		oem_charger_control_flag = 0;
		charger_control_timer = oem_param_charger.maint_wait_chg_on_time * 1000;
	} else {
		oem_last_batt_status = get_prop_batt_status(the_chip);
		oem_last_charge_type = get_prop_charge_type(the_chip);
		oem_last_dc_present = the_chip->dc_present;
		oem_last_usb_present = the_chip->usb_present;
		oem_pm8921_disable_source_current(true);
		oem_charger_control_flag = 1;
		charger_control_timer = oem_param_charger.maint_wait_chg_off_time * 1000;
	}

	schedule_delayed_work(&oem_charger_work,
			round_jiffies_relative(msecs_to_jiffies(charger_control_timer)));
}

static int oem_charger_limit_control_check(int enable)
{
	if(1 == enable) {
		if (OEM_STATUS_LIMIT != oem_charge_status) {
			oem_charger_control_flag = 1;
			schedule_delayed_work(&oem_charger_work,
					round_jiffies_relative(msecs_to_jiffies(0)));
		}
	} else if (0 == enable) {
		if (OEM_STATUS_LIMIT == oem_charge_status) {
			cancel_delayed_work_sync(&oem_charger_work);
			oem_pm8921_disable_source_current(false);
		}
	} else {
		pr_err("Out of range : enable = %d\n", enable);
		return -EINVAL;

	}
	return 0;
}

static int oem_batt_health = POWER_SUPPLY_HEALTH_GOOD;
static void oem_charger_monitor(void)
{
	if (((the_chip->usb_present || the_chip->dc_present)) &&
	    ((POWER_SUPPLY_STATUS_CHARGING == get_prop_batt_status(the_chip)) ||
	    (OEM_STATUS_NORMAL != oem_charge_status))) {

		if ((OEM_STATUS_STOP == camera_therm_status) &&
		    (OEM_STATUS_STOP == substrate_therm_status)) {
			if (OEM_STATUS_STOP != oem_charge_status) {
				cancel_delayed_work_sync(&oem_charger_work);
				oem_pm8921_disable_source_current(true);
				oem_charge_status = OEM_STATUS_STOP;
			}

		}else if (OEM_STATUS_STOP == usb_therm_status) {
			if (OEM_STATUS_STOP != oem_charge_status) {
				cancel_delayed_work_sync(&oem_charger_work);
				oem_pm8921_disable_source_current(true);
				oem_charge_status = OEM_STATUS_STOP;
			}

		}else if ((POWER_SUPPLY_HEALTH_OVERHEAT == oem_batt_health) ||
		          (POWER_SUPPLY_HEALTH_COLD == oem_batt_health)) {
				if (OEM_STATUS_STOP != oem_charge_status) {
					cancel_delayed_work_sync(&oem_charger_work);
					oem_charge_status = OEM_STATUS_STOP;
				}

		} else {
			if (OEM_STATUS_STOP == oem_charge_status) {
				oem_charge_status = OEM_STATUS_NORMAL;
				oem_pm8921_disable_source_current(false);
			}

			if ((((OEM_STATUS_LIMIT <= camera_therm_status) &&
			      (OEM_STATUS_LIMIT <= substrate_therm_status)) ||
			      (OEM_STATUS_LIMIT == battery_temp_status)) &&
			      (OEM_STATUS_LIMIT == vbatt_status)) {
				oem_charger_limit_control_check(1);
				oem_charge_status = OEM_STATUS_LIMIT;

			} else {
				oem_charger_limit_control_check(0);
				oem_charge_status = OEM_STATUS_NORMAL;
			}
		}

	} else if (OEM_STATUS_NORMAL != oem_charge_status) {
		if (OEM_STATUS_LIMIT == oem_charge_status) {
			cancel_delayed_work_sync(&oem_charger_work);
		}
		oem_pm8921_disable_source_current(false);
		oem_charge_status = OEM_STATUS_NORMAL;
	}

	pr_debug("oem_charge_status = %d\n", oem_charge_status);

	return;
}

static int oem_adc_read;
static int oem_adc_read_cmd(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8xxx_adc_chan_result result;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	ret = pm8xxx_adc_read(oem_adc_read, &result);

	pr_info("channel = %d physical = %lld raw = %d",
			oem_adc_read, result.physical, result.adc_code);

	return 0;
}
module_param_call(oem_adc_read, oem_adc_read_cmd,
					param_get_uint, &oem_adc_read, 0644);

static int oem_ibatt_set_param;
static int oem_ibatt_get_param;
static int oem_ibatt_set_read_cmd(const char *val, struct kernel_param *kp)
{
	int result_ua = 0;
	int rc = 0;

	rc = pm8921_bms_get_battery_current(&result_ua);
	if (rc) {
		oem_ibatt_set_param = -EINVAL;
	} else {
		oem_ibatt_set_param = result_ua;
	}

	return 0;
}
static int oem_ibatt_get_read_cmd(char *buffer, struct kernel_param *kp)
{

	int res = 0;

	res = snprintf(buffer, 16, "%d", oem_ibatt_set_param);

	return res;
}
module_param_call(oem_ibatt_get_param, oem_ibatt_set_read_cmd,
					oem_ibatt_get_read_cmd, &oem_ibatt_get_param, 0644);

static int oem_charge_back_ctrl;
static int oem_charge_back_ctrl_cmd(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if ((0 == oem_charge_back_ctrl) || (1 == oem_charge_back_ctrl)) {
		ret = pm8921_disable_source_current(oem_charge_back_ctrl);
	} else {
		pr_err("Out of range %d \n", oem_charge_back_ctrl);
		ret = -EINVAL;
	}
	return ret;
}
module_param_call(oem_charge_back_ctrl, oem_charge_back_ctrl_cmd,
					param_get_uint, &oem_charge_back_ctrl, 0644);

static int oem_auto_charger_ctrl;
static int oem_auto_charger_ctrl_cmd(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if ((0 == oem_auto_charger_ctrl) || (1 == oem_auto_charger_ctrl)) {
		ret = pm_chg_auto_enable(the_chip, !oem_auto_charger_ctrl);
	} else {
		pr_err("Out of range %d \n", oem_auto_charger_ctrl);
		ret = -EINVAL;
	}
	return ret;
}
module_param_call(oem_auto_charger_ctrl, oem_auto_charger_ctrl_cmd,
					param_get_uint, &oem_auto_charger_ctrl, 0644);

static struct delayed_work	oem_hkadc_work;
static int oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_NUM];
static int oem_charger_connect_state = POWER_SUPPLY_OEM_CONNECT_NONE;

static int vbat_work[HKADC_VBAT_VOLTS_ELEMENT_COUNT]={0};
static int bat_temp_work[HKADC_BAT_TEMP_ELEMENT_COUNT]={0};
static int ibat_work[HKADC_IBAT_CURRENT_ELEMENT_COUNT]={0};
static int pa_therm_work[HKADC_PA_THERM_ELEMENT_COUNT]={0};
static int camera_therm_work[HKADC_CAMERA_TEMP_ELEMENT_COUNT]={0};
static int substrate_therm_work[HKADC_SUBSTRATE_THERM_ELEMENT_COUNT]={0};
static int usb_therm_work[HKADC_USB_THERM_ELEMENT_COUNT]={0};

static struct alarm androidalarm;
static struct timespec androidalarm_interval_timespec;
struct early_suspend hkadc_early_suspend;
struct wake_lock oem_hkadc_wake_lock;

static int pa_therm_monit_freq = 0;
static int camera_temp_monit_freq = 0;
static int substrate_therm_monit_freq = 0;
static int usb_therm_monit_freq = 0;

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static int oem_chg_stand_charge_timer = 0;
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

static atomic_t is_hkadc_initialized=ATOMIC_INIT(0);
static atomic_t is_chargermonit_initialized=ATOMIC_INIT(0);

static void oem_hkadc_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp){
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_CURRENT];
		break;

	case POWER_SUPPLY_PROP_TEMP:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM];
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*masterdata = val;
		MASTERDATA_UNLOCK();
	}

}
static void oem_charger_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata
		 = &oem_charger_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*masterdata = val;
		MASTERDATA_UNLOCK();
	}

}

static int oem_hkadc_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_hkadc_initialized);

	if (!initialized) {
		pr_err("called before init\n");
		MASTERDATA_LOCK();
		*intval = 0;
		MASTERDATA_UNLOCK();
		return -EAGAIN;
	}

	switch(psp){
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE];
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_CURRENT];
		break;

	case POWER_SUPPLY_PROP_TEMP:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM];
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*intval = *masterdata;
		MASTERDATA_UNLOCK();
	}else{
		pr_debug("Out of range psp %d \n", psp);
		ret = -EINVAL;
	}

	return ret;

}

static int oem_charger_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_chargermonit_initialized);

	if (!initialized) {
		pr_err("called before init\n");
		*intval = 0;
		return -EAGAIN;
	}

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata = &oem_charger_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*intval = *masterdata;
		MASTERDATA_UNLOCK();
	}else{
		pr_debug("Out of range psp %d \n", psp);
		ret = -EINVAL;
	}

	return ret;

}

#define HKADC_BATT_DEFAULT_VOLTAGE	3600000
static int oem_hkadc_init_battery_uvolts(void)
{
	int rc;
	int work=0;
	int cnt;
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->vbat_channel, rc);
		work = HKADC_BATT_DEFAULT_VOLTAGE;
	}else{
		work = (int)result.physical;
	}
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		vbat_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE] = work;

	pr_debug("init hkadc uvolts = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE]);

	return 0;
}

static int oem_hkadc_get_battery_uvolts(int *vbat)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->vbat_channel, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat);
		pr_debug("hkadc get volt vbat do not updated. %d %d\n", rc, *vbat);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT-1;cnt++) {
		vbat_work[cnt] = vbat_work[cnt+1];
		total = total + vbat_work[cnt];
	}
	vbat_work[cnt] = work;
	total = total + vbat_work[cnt];

	max = vbat_work[0];
	min = vbat_work[0];
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		if (max < vbat_work[cnt]) {
			max = vbat_work[cnt];
		} else if (min > vbat_work[cnt]) {
			min = vbat_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_VBAT_VOLTS_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, average);
	*vbat = average;
	pr_debug("hkadc volt  total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc uvolts = %d   %d %d %d %d %d \n", 
		average,
		vbat_work[0],
		vbat_work[1],
		vbat_work[2],
		vbat_work[3],
		vbat_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_battery_uvolts(int *vbat)
{
	int rc;
	int cnt;
	int ngcnt;
	int vbat_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int vbat_work_tmp[HKADC_VBAT_VOLTS_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
						the_chip->vbat_channel, rc);
			ngcnt++;
		}else{
			vbat_new = (int)result.physical;
			vbat_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			vbat_work[cnt]=vbat_work_tmp[cnt];
			total = total + vbat_work_tmp[cnt];
		}
		max = vbat_work[0];
		min = vbat_work[0];
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			if (max < vbat_work[cnt]) {
				max = vbat_work[cnt];
			} else if (min > vbat_work[cnt]) {
				min = vbat_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_VBAT_VOLTS_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, average);
		*vbat = average;
		pr_debug("hkadc wakeup volt  total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup uvolts = %d   %d %d %d %d %d \n", 
			average,
			vbat_work[0],
			vbat_work[1],
			vbat_work[2],
			vbat_work[3],
			vbat_work[4]);
	}else if (ngcnt < HKADC_VBAT_VOLTS_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			vbat_work[cnt] = vbat_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat_new);
		*vbat = vbat_new;
		pr_debug("hkadc wakeup volt vbat_new = %d\n", vbat_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat);
		pr_debug("hkadc wakeup volt vbat do not updated. %d %d\n", rc, *vbat);
	}

	return 0;
}

static void oem_charger_perform_battery_detection_state(void)
{
	int ret;

	if (oem_option_item1_bit0){
		if (is_batterydetected){
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
		}else{
			ret = oem_pm8921_disable_source_current(true);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
		}
	}else{
		if (!is_batterydetected){
			kernel_power_off();
		}
	}
}

#define CHG_BATT_DETECTION_TEMP		-330
#define CHG_BATT_DETECTION_INTERVAL	10
#define CHG_BATT_DETECTION_NUM		5
static void oem_charger_init_batterydetectionstate(int battemp)
{
	if ((battemp) > CHG_BATT_DETECTION_TEMP) {
		is_batterydetected = true;
	
	}else{
		is_batterydetected = false;
	}
}

static void oem_charger_check_battery_detection_state(void)
{
	int i;
	int temp;
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->batt_temp_channel, rc);
		temp = CHG_BATT_DETECTION_TEMP;
	}else{
		temp = (int)result.physical;
	}
	if ((temp) > CHG_BATT_DETECTION_TEMP) {
		if (is_batterydetected == false){
			for (i=0 ; i<CHG_BATT_DETECTION_NUM ; i++){
				mdelay(CHG_BATT_DETECTION_INTERVAL);
				rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
				if (rc) {
					pr_err("error reading adc channel = %d, rc = %d\n",
								the_chip->batt_temp_channel, rc);
					break;
				}
				temp = (int)result.physical;
				if ((temp) <= CHG_BATT_DETECTION_TEMP){
					pr_debug("check batterydetectionstate inserted fail:%d\n", i);
					break;
				}
				pr_debug("check batterydetectionstate inserted count %d\n", i);
			}

			if (i >= CHG_BATT_DETECTION_NUM){
				is_batterydetected = true;
				oem_charger_perform_battery_detection_state();
				pr_debug("batterydetectionstate inserted.\n");
			}
		}
	}else{
		if (is_batterydetected == true){
			for (i=0 ; i<CHG_BATT_DETECTION_NUM ; i++){
				mdelay(CHG_BATT_DETECTION_INTERVAL);
				rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
				if (rc) {
					pr_err("error reading adc channel = %d, rc = %d\n",
								the_chip->batt_temp_channel, rc);
					break;
				}
				temp = (int)result.physical;
				if ((temp) > CHG_BATT_DETECTION_TEMP){
					pr_debug("check batterydetectionstate removed fail:%d\n", i);
					break;
				}
				pr_debug("check batterydetectionstate removed count %d\n", i);
			}

			if (i >= CHG_BATT_DETECTION_NUM){
				is_batterydetected = false;
				oem_charger_perform_battery_detection_state();
				pr_debug("batterydetectionstate removed.\n");
			}
		}
	}
}

#define HKADC_BATT_DEFAULT_TEMP		250
static int oem_hkadc_init_battery_temp(void)
{
	int rc;
	int work=0;
	int cnt;
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->batt_temp_channel, rc);
		work = HKADC_BATT_DEFAULT_TEMP;
	}else{
		work = (int)result.physical;
	}
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		bat_temp_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM] = work;

	pr_debug("init hkadc battery temp = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM]);

	oem_charger_init_batterydetectionstate(work);

	return 0;
}

static int oem_hkadc_get_battery_temp(int *temp)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->batt_temp_channel, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_TEMP, temp);
		pr_debug("hkadc get battery temp do not updated. %d %d\n", rc, *temp);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT-1;cnt++) {
		bat_temp_work[cnt] = bat_temp_work[cnt+1];
		total = total + bat_temp_work[cnt];
	}
	bat_temp_work[cnt] = work;
	total = total + bat_temp_work[cnt];

	max = bat_temp_work[0];
	min = bat_temp_work[0];
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		if (max < bat_temp_work[cnt]) {
			max = bat_temp_work[cnt];
		} else if (min > bat_temp_work[cnt]) {
			min = bat_temp_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_BAT_TEMP_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, average);
	*temp = average;
	pr_debug("hkadc battery temp total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc battery temps = %d   %d %d %d %d %d \n", 
		average,
		bat_temp_work[0],
		bat_temp_work[1],
		bat_temp_work[2],
		bat_temp_work[3],
		bat_temp_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_battery_temp(int *temp)
{
	int rc;
	int cnt;
	int ngcnt;
	int battemp_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int bat_temp_work_tmp[HKADC_BAT_TEMP_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(the_chip->batt_temp_channel, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
						the_chip->batt_temp_channel, rc);
			ngcnt++;
		}else{
			battemp_new = (int)result.physical;
			bat_temp_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			bat_temp_work[cnt]=bat_temp_work_tmp[cnt];
			total = total + bat_temp_work_tmp[cnt];
		}
		max = bat_temp_work[0];
		min = bat_temp_work[0];
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			if (max < bat_temp_work[cnt]) {
				max = bat_temp_work[cnt];
			} else if (min > bat_temp_work[cnt]) {
				min = bat_temp_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_BAT_TEMP_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, average);
		*temp = average;
		pr_debug("hkadc wakeup battery temp total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup battery temp = %d   %d %d %d %d %d \n", 
			average,
			bat_temp_work[0],
			bat_temp_work[1],
			bat_temp_work[2],
			bat_temp_work[3],
			bat_temp_work[4]);
	}else if (ngcnt < HKADC_BAT_TEMP_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			bat_temp_work[cnt] = battemp_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, battemp_new);
		*temp = battemp_new;
		pr_debug("hkadc wakeup battery temp battemp_new = %d\n", battemp_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_TEMP, temp);
		pr_debug("hkadc wakeup battery temp do not updated. %d %d\n", rc, *temp);
	}

	return 0;
}

static int oem_hkadc_init_battery_current(void)
{
	int work=0;
	int cnt;
	int rc;
	struct pm8xxx_adc_chan_result result;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(CHANNEL_IBAT, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				CHANNEL_IBAT, rc);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_IBAT_CURRENT_ELEMENT_COUNT;cnt++) {
		ibat_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_CURRENT] = work;

	pr_debug("init hkadc battery current = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_CURRENT]);

	return 0;
}

static int oem_hkadc_get_battery_current(int* ibat_uv)
{
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;
	int rc;
	struct pm8xxx_adc_chan_result result;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm8xxx_adc_read(CHANNEL_IBAT, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				CHANNEL_IBAT, rc);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_IBAT_CURRENT_ELEMENT_COUNT-1;cnt++) {
		ibat_work[cnt] = ibat_work[cnt+1];
		total = total + ibat_work[cnt];
	}
	ibat_work[cnt] = work;
	total = total + ibat_work[cnt];

	max = ibat_work[0];
	min = ibat_work[0];
	for (cnt=0;cnt<HKADC_IBAT_CURRENT_ELEMENT_COUNT;cnt++) {
		if (max < ibat_work[cnt]) {
			max = ibat_work[cnt];
		} else if (min > ibat_work[cnt]) {
			min = ibat_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_IBAT_CURRENT_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_CURRENT_NOW, average);
	*ibat_uv = average;
	pr_debug("hkadc battery current total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc battery current = %d   %d %d %d %d %d \n", 
		average,
		ibat_work[0],
		ibat_work[1],
		ibat_work[2],
		ibat_work[3],
		ibat_work[4]);

	return 0;
}

static int oem_hkadc_init_pa_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX3, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX3, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		pa_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM] = work;

	pr_debug("init hkadc pa therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM]);

	return 0;
}

static int oem_hkadc_get_pa_therm(void)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX3, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX3, rc);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT-1;cnt++) {
		pa_therm_work[cnt] = pa_therm_work[cnt+1];
		total = total + pa_therm_work[cnt];
	}
	pa_therm_work[cnt] = (int)result.physical;
	total = total + pa_therm_work[cnt];

	max = pa_therm_work[0];
	min = pa_therm_work[0];
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		if (max < pa_therm_work[cnt]) {
			max = pa_therm_work[cnt];
		} else if (min > pa_therm_work[cnt]) {
			min = pa_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_PA_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, average);

	pr_debug("hkadc pa_therm total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc pa_therm = %d   %d %d %d %d %d \n", 
		average,
		pa_therm_work[0],
		pa_therm_work[1],
		pa_therm_work[2],
		pa_therm_work[3],
		pa_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_pa_therm(void)
{
	int rc;
	int cnt;
	int ngcnt;
	int pa_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int pa_therm_work_tmp[HKADC_PA_THERM_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(ADC_MPP_1_AMUX3, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
					ADC_MPP_1_AMUX3, rc);
			ngcnt++;
		}else{
			pa_therm_new = (int)result.physical;
			pa_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			pa_therm_work[cnt]=pa_therm_work_tmp[cnt];
			total = total + pa_therm_work_tmp[cnt];
		}
		max = pa_therm_work[0];
		min = pa_therm_work[0];
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			if (max < pa_therm_work[cnt]) {
				max = pa_therm_work[cnt];
			} else if (min > pa_therm_work[cnt]) {
				min = pa_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_PA_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, average);
		pr_debug("hkadc wakeup pa_therm total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup pa_therm = %d   %d %d %d %d %d \n", 
			average,
			pa_therm_work[0],
			pa_therm_work[1],
			pa_therm_work[2],
			pa_therm_work[3],
			pa_therm_work[4]);
	}else if (ngcnt < HKADC_PA_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			pa_therm_work[cnt] = pa_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, pa_therm_new);
		pr_debug("hkadc wakeup pa_therm_new = %d\n", pa_therm_new);
	}else{
		pr_debug("hkadc wakeup pa_therm do not updated.\n");
	}

	return 0;
}

static int oem_hkadc_init_camera_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX7, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX7, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		camera_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM] = work;

	pr_debug("init hkadc camera therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM]);

	return 0;
}

static int oem_hkadc_get_camera_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX7, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX7, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, therm);
		pr_debug("hkadc get camera_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT-1;cnt++) {
		camera_therm_work[cnt] = camera_therm_work[cnt+1];
		total = total + camera_therm_work[cnt];
	}
	camera_therm_work[cnt] = work;
	total = total + camera_therm_work[cnt];

	max = camera_therm_work[0];
	min = camera_therm_work[0];
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		if (max < camera_therm_work[cnt]) {
			max = camera_therm_work[cnt];
		} else if (min > camera_therm_work[cnt]) {
			min = camera_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_CAMERA_TEMP_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, average);
	*therm = average;
	pr_debug("hkadc camera therm total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc camera therm = %d   %d %d %d %d %d \n", 
		average, camera_therm_work[0],
		camera_therm_work[1],
		camera_therm_work[2],
		camera_therm_work[3],
		camera_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_camera_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int camera_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int camera_therm_work_tmp[HKADC_CAMERA_TEMP_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(ADC_MPP_1_AMUX7, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
					ADC_MPP_1_AMUX7, rc);
			ngcnt++;
		}else{
			camera_therm_new = (int)result.physical;
			camera_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			camera_therm_work[cnt]=camera_therm_work_tmp[cnt];
			total = total + camera_therm_work_tmp[cnt];
		}
		max = camera_therm_work[0];
		min = camera_therm_work[0];
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			if (max < camera_therm_work[cnt]) {
				max = camera_therm_work[cnt];
			} else if (min > camera_therm_work[cnt]) {
				min = camera_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_CAMERA_TEMP_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, average);
		*therm = average;
		pr_debug("hkadc wakeup camera_therm total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup camera_therm = %d   %d %d %d %d %d \n", 
			average,
			camera_therm_work[0],
			camera_therm_work[1],
			camera_therm_work[2],
			camera_therm_work[3],
			camera_therm_work[4]);
	}else if (ngcnt < HKADC_CAMERA_TEMP_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			camera_therm_work[cnt] = camera_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, camera_therm_new);
		*therm = camera_therm_new;
		pr_debug("hkadc wakeup camera_therm_new = %d\n", camera_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, therm);
		pr_debug("hkadc wakeup camera_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

static int oem_hkadc_init_substrate_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX4, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX4, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		substrate_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM] = work;

	pr_debug("init hkadc substrate therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM]);

	return 0;
}

static int oem_hkadc_get_substrate_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX4, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX4, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, therm);
		pr_debug("hkadc get substrate_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-1;cnt++) {
		substrate_therm_work[cnt] = substrate_therm_work[cnt+1];
		total = total + substrate_therm_work[cnt];
	}
	substrate_therm_work[cnt] = work;
	total = total + substrate_therm_work[cnt];

	max = substrate_therm_work[0];
	min = substrate_therm_work[0];
	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		if (max < substrate_therm_work[cnt]) {
			max = substrate_therm_work[cnt];
		} else if (min > substrate_therm_work[cnt]) {
			min = substrate_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, average);
	*therm = average;
	pr_debug("hkadc substrate therm total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc substrate therm = %d   %d %d %d %d %d \n", 
		average,
		substrate_therm_work[0],
		substrate_therm_work[1],
		substrate_therm_work[2],
		substrate_therm_work[3],
		substrate_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_substrate_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int substrate_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int substrate_therm_work_tmp[HKADC_SUBSTRATE_THERM_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(ADC_MPP_1_AMUX4, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
					ADC_MPP_1_AMUX4, rc);
			ngcnt++;
		}else{
			substrate_therm_new = (int)result.physical;
			substrate_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			substrate_therm_work[cnt]=substrate_therm_work_tmp[cnt];
			total = total + substrate_therm_work_tmp[cnt];
		}
		max = substrate_therm_work[0];
		min = substrate_therm_work[0];
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			if (max < substrate_therm_work[cnt]) {
				max = substrate_therm_work[cnt];
			} else if (min > substrate_therm_work[cnt]) {
				min = substrate_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, average);
		*therm = average;
		pr_debug("hkadc wakeup substrate_therm total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup substrate_therm = %d   %d %d %d %d %d \n", 
			average,
			substrate_therm_work[0],
			substrate_therm_work[1],
			substrate_therm_work[2],
			substrate_therm_work[3],
			substrate_therm_work[4]);
	}else if (ngcnt < HKADC_SUBSTRATE_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			substrate_therm_work[cnt] = substrate_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, substrate_therm_new);
		*therm = substrate_therm_new;
		pr_debug("hkadc wakeup substrate_therm_new = %d\n", substrate_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, therm);
		pr_debug("hkadc wakeup substrate_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

static int oem_hkadc_init_usb_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX8, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX8, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		usb_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM] = work;

	pr_debug("init hkadc usb therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM]);

	return 0;
}

static int oem_hkadc_get_usb_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(ADC_MPP_1_AMUX8, &result);
	if (rc) {
		pr_debug("error reading adc channel = %d, rc = %d\n",
				ADC_MPP_1_AMUX8, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_USB_THERM, therm);
		pr_debug("hkadc get usb_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT-1;cnt++) {
		usb_therm_work[cnt] = usb_therm_work[cnt+1];
		total = total + usb_therm_work[cnt];
	}
	usb_therm_work[cnt] = work;
	total = total + usb_therm_work[cnt];

	max = usb_therm_work[0];
	min = usb_therm_work[0];
	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		if (max < usb_therm_work[cnt]) {
			max = usb_therm_work[cnt];
		} else if (min > usb_therm_work[cnt]) {
			min = usb_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_USB_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, average);
	*therm = average;
	pr_debug("hkadc usb therm total = %d max = %d min = %d \n", total, max, min);
	pr_debug("hkadc usb therm = %d   %d %d %d %d %d \n", 
		average,
		usb_therm_work[0],
		usb_therm_work[1],
		usb_therm_work[2],
		usb_therm_work[3],
		usb_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_usb_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int usb_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int usb_therm_work_tmp[HKADC_USB_THERM_ELEMENT_COUNT];
	struct pm8xxx_adc_chan_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		rc = pm8xxx_adc_read(ADC_MPP_1_AMUX8, &result);
		if (rc) {
			pr_err("error reading adc channel = %d, rc = %d\n",
					ADC_MPP_1_AMUX8, rc);
			ngcnt++;
		}else{
			usb_therm_new = (int)result.physical;
			usb_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			usb_therm_work[cnt]=usb_therm_work_tmp[cnt];
			total = total + usb_therm_work_tmp[cnt];
		}
		max = usb_therm_work[0];
		min = usb_therm_work[0];
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			if (max < usb_therm_work[cnt]) {
				max = usb_therm_work[cnt];
			} else if (min > usb_therm_work[cnt]) {
				min = usb_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_USB_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, average);
		*therm = average;
		pr_debug("hkadc wakeup usb_therm total = %d max = %d min = %d \n", total, max, min);
		pr_debug("hkadc wakeup usb_therm = %d   %d %d %d %d %d \n", 
			average,
			usb_therm_work[0],
			usb_therm_work[1],
			usb_therm_work[2],
			usb_therm_work[3],
			usb_therm_work[4]);
	}else if (ngcnt < HKADC_USB_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			usb_therm_work[cnt] = usb_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, usb_therm_new);
		*therm = usb_therm_new;
		pr_debug("hkadc wakeup usb_therm_new = %d\n", usb_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_USB_THERM, therm);
		pr_debug("hkadc wakeup usb_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

static int oem_hkadc_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_OEM_PA_THERM:
	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		ret = oem_hkadc_master_data_read(psp, intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if ((POWER_SUPPLY_PROP_TEMP == psp) &&
		    (oem_param_share.factory_mode_1)) {
			*intval = 250;
			ret = 0;
		}else {
			ret = oem_hkadc_master_data_read(psp, intval);
		}
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static int oem_charger_determine_connect_state(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (oem_stand_detect){
		pr_debug("charger connect %d \n", oem_charge_stand_flag);
		if (oem_charge_stand_flag){
			return POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_1;
		}else{
			return POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_2;
		}
		
	}else{
		pr_debug("charger disconnect %d \n", the_chip->usb_present);
		if (the_chip->usb_present){
			return POWER_SUPPLY_OEM_CONNECT_USB;
		}else{
			return POWER_SUPPLY_OEM_CONNECT_NONE;
		}
	}
}

static int oem_charger_init_connect_state(void)
{
	int state;

	state = oem_charger_determine_connect_state();
	pr_debug("init charger connect state = %d \n", state);

	if (-1 < state) {
		oem_charger_connect_state = state;
	} else {
		return state;
	}

	return 0;
}

static int oem_charger_get_connect_state(void)
{
	int state;

	state = oem_charger_determine_connect_state();
	pr_debug("charger connect state = %d \n", state);

	if (-1 < state) {
		oem_charger_master_data_write(POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE, state);
	} else {
		return state;
	}

	return 0;
}

static uint32_t oem_charge_fail_status = 0;
static void oem_judge_charge_state(void)
{
	do {
		if (!the_chip->dc_present && !the_chip->usb_present && !oem_stand_detect) {
			charge_state = CHG_STATE_NONCONNECTED;
			break;
		}

		if (oem_charge_stand_flag) {
			charge_state = CHG_STATE_CHG_STAND;
			break;
		}

		if (is_oem_fast_chg_expired) {
			charge_state = CHG_STATE_CHG_TIMEOUT;
			break;
		}

		if (pm_chg_get_rt_status(the_chip, TRKLCHG_IRQ)) {
			charge_state = CHG_STATE_TRICKLE;
			break;
		}

		if (POWER_SUPPLY_STATUS_FULL == get_prop_batt_status(the_chip)) {
			charge_state = CHG_STATE_CHG_COMP;
			break;
		}

		if (OEM_STATUS_LIMIT == oem_charge_status) {
			if (the_chip->is_bat_warm) {
				charge_state = CHG_STATE_INTE_WARM;
			} else if (the_chip->is_bat_cool) {
				charge_state = CHG_STATE_INTE_COOL;
			} else {
				charge_state = CHG_STATE_INTE_NORMAL;
			}
			break;
		}

		if ((pm_chg_get_rt_status(the_chip, FASTCHG_IRQ)) ||
		    (oem_pm8921_bms_is_cyclecorrection_chargeoffstate())
		   ) {
			if (the_chip->is_bat_warm) {
				charge_state = CHG_STATE_FAST_WARM;
			} else if (the_chip->is_bat_cool) {
				charge_state = CHG_STATE_FAST_COOL;
			} else {
				charge_state = CHG_STATE_FAST_NORMAL;
			}
			break;
		}

		if (oem_batt_health == POWER_SUPPLY_HEALTH_COLD) {
			charge_state = CHG_STATE_BATT_TEMP_COLD;
			break;
		}

		if (oem_batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
			charge_state = CHG_STATE_BATT_TEMP_HOT;
			break;
		}

		if (OEM_STATUS_STOP == oem_charge_status) {
			charge_state = CHG_STATE_WAIT_TEMP;
			break;
		}

		if (pm_chg_get_rt_status(the_chip, BATT_REMOVED_IRQ)) {
			charge_state = CHG_STATE_BATT_ID_ERROR;
			break;
		}

		if (oem_charge_fail_status) {
			charge_state = CHG_STATE_CHG_ERROR;
			break;
		}

		charge_state = CHG_STATE_IDLE;

	} while(0);
	pr_debug("charge state = %d\n", charge_state);
	return;
}

static void oem_chg_check_charger_removal_after_timer_expired(void)
{
	int ret;
	if (is_oem_fast_chg_expired == true){
		if (!the_chip->dc_present && !the_chip->usb_present && !oem_stand_detect) {
			is_oem_fast_chg_expired = false;
			pr_debug("oem_chg_check_charger_removal_after_timer_expired()\n");
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
			ret = pm_chg_auto_enable(the_chip, true);
			if (ret) {
				pr_err("error bat_fet setting value %d\n", ret);
			}
		}
	}
}

static void oem_chg_recharge_check_after_timer_expired(void)
{
	int ret;
	if (is_oem_fast_chg_expired == true){
		if (oem_vbatt <= ((the_chip->max_voltage_mv - the_chip->resume_voltage_delta)*1000)){
			is_oem_fast_chg_expired = false;
			pr_debug("oem_chg_recharge_check_after_timer_expired()\n");
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
			ret = pm_chg_auto_enable(the_chip, true);
			if (ret) {
				pr_err("error bat_fet setting value %d\n", ret);
			}
#ifdef CONFIG_CHARGE_STAND_SUPPORT
			oem_chg_stand_off();
#endif /* CONFIG_CHARGE_STAND_SUPPORT */
			schedule_work(&fast_chg_count_start_work);
		}
	}
}

static void oem_chg_vbat_ov_check_after_timer_expired(void)
{
	int ret;
	if (is_oem_fast_chg_expired == true){
		if ((oem_param_charger.chg_stop_volt * 1000) <= oem_vbatt){
			is_oem_fast_chg_expired = false;
			pr_debug("oem_chg_vbat_ov_check_after_timer_expired()\n");
			ret = oem_pm8921_disable_source_current(true);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
			ret = pm_chg_auto_enable(the_chip, true);
			if (ret) {
				pr_err("error bat_fet setting value %d\n", ret);
			}
		}
	}
}

static void oem_chg_check_battery_detection_after_timer_expired(void)
{
	int ret;
	if (is_oem_fast_chg_expired == true){
		if (oem_option_item1_bit0){
			if (!is_batterydetected){
				is_oem_fast_chg_expired = false;
				pr_debug("oem_chg_check_battery_detection_after_timer_expired()\n");
				ret = oem_pm8921_disable_source_current(false);
				if (ret) {
					pr_err("error buck converter setting value %d\n", ret);
				}
				ret = pm_chg_auto_enable(the_chip, false);
				if (ret) {
					pr_err("error bat_fet setting value %d\n", ret);
				}
			}
		}
	}
}

static void oem_chg_check_adapter_overvoltage_after_timer_expired(void)
{
	int ret;
	if (is_oem_fast_chg_expired == true){
		if (pm_chg_get_rt_status(the_chip, DCIN_OV_IRQ)){
			is_oem_fast_chg_expired = false;
			pr_debug("oem_chg_check_adapter_overvoltage_after_timer_expired()\n");
			ret = oem_pm8921_disable_source_current(false);
			if (ret) {
				pr_err("error buck converter setting value %d\n", ret);
			}
			ret = pm_chg_auto_enable(the_chip, true);
			if (ret) {
				pr_err("error bat_fet setting value %d\n", ret);
			}
		}
	}
}

static void oem_chg_check_vbatt_ov(void){

	pr_debug("oem_chg_check_vbatt_ov. %d %d\n", oem_vbatt, is_batterydetected);

	if (((oem_param_charger.chg_stop_volt * 1000) <= oem_vbatt) || (!is_batterydetected)){
		pr_debug("chg_vbatt_ov_worker performed.\n");
		schedule_work(&chg_vbatt_ov_work);
	}
}

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static void oem_stand_check(void);
#endif /* CONFIG_CHARGE_STAND_SUPPORT */
extern void oem_pm8921_bms_calculate_cyclecorrection(void);

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static void oem_chg_stand_check_control(void);
#endif /* CONFIG_CHARGE_STAND_SUPPORT */
static void oem_hkadc_monitor(struct work_struct *work)
{
	int vbat  = 0;
	int temp  = 0;
	int ibat  = 0;
	int camera_therm  = 0;
	int substrate_therm = 0;
	int usb_therm = 0;
	int rc = 0;

	oem_hkadc_get_battery_current(&ibat);

	if (is_hkadc_suspend_monit){
		oem_hkadc_get_wakeup_battery_uvolts(&vbat);
		oem_hkadc_get_wakeup_battery_temp(&temp);
		oem_hkadc_get_wakeup_pa_therm();
		oem_hkadc_get_wakeup_camera_therm(&camera_therm);
		oem_hkadc_get_wakeup_substrate_therm(&substrate_therm);
		oem_hkadc_get_wakeup_usb_therm(&usb_therm);

		oem_battery_volts_set_status(vbat);
		oem_battery_temp_set_status(temp);
		oem_camera_therm_set_status(camera_therm);
		oem_substrate_therm_set_status(substrate_therm);
		oem_usb_therm_set_status(usb_therm);

		oem_pm8921_bms_low_vol_detect_standby(temp);

		pa_therm_monit_freq = 0;
		camera_temp_monit_freq = 0;
		substrate_therm_monit_freq = 0;
		usb_therm_monit_freq = 0;
	}else{
		rc = oem_hkadc_get_battery_uvolts(&vbat);
		rc |= oem_hkadc_get_battery_temp(&temp);
		oem_battery_volts_set_status(vbat);
		oem_battery_temp_set_status(temp);

		pa_therm_monit_freq++;
		camera_temp_monit_freq++;
		substrate_therm_monit_freq++;
		usb_therm_monit_freq++;

		if (pa_therm_monit_freq >= HKADC_PA_THERM_MONIT_FREQ){
			pa_therm_monit_freq=0;
			oem_hkadc_get_pa_therm();
		}
		if (camera_temp_monit_freq >= HKADC_CAMERA_TEMP_MONIT_FREQ){
			camera_temp_monit_freq=0;
			oem_hkadc_get_camera_therm(&camera_therm);
			oem_camera_therm_set_status(camera_therm);
		}
		if (substrate_therm_monit_freq >= HKADC_SUBSTRATE_THERM_MONIT_FREQ){
			substrate_therm_monit_freq=0;
			oem_hkadc_get_substrate_therm(&substrate_therm);
			oem_substrate_therm_set_status(substrate_therm);
		}
		if (usb_therm_monit_freq >= HKADC_USB_THERM_MONIT_FREQ){
			usb_therm_monit_freq=0;
			oem_hkadc_get_usb_therm(&usb_therm);
			oem_usb_therm_set_status(usb_therm);
		}

		oem_pm8921_bms_low_vol_detect_active(vbat, temp, rc);
	}

	oem_vbatt = vbat;
	oem_batt_temp = temp;

	if (the_chip->dc_present || the_chip->usb_present || oem_stand_detect) {
		oem_charger_check_battery_detection_state();
	}

	oem_batt_health = get_prop_batt_health(the_chip);

#ifdef CONFIG_CHARGE_STAND_SUPPORT
	oem_stand_check();
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

	oem_charger_monitor();

	oem_charger_get_connect_state();

	oem_chg_check_charger_removal_after_timer_expired();

	oem_chg_recharge_check_after_timer_expired();

	oem_chg_check_adapter_overvoltage_after_timer_expired();

	oem_chg_vbat_ov_check_after_timer_expired();

	oem_chg_check_battery_detection_after_timer_expired();

#ifdef CONFIG_CHARGE_STAND_SUPPORT
	oem_chg_stand_check_control();
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

	oem_judge_charge_state();

	oem_pm8921_bms_calculate_cyclecorrection();

	oem_chg_check_vbatt_ov();

	if (is_hkadc_suspend_monit){
		wake_unlock(&oem_hkadc_wake_lock);
		is_hkadc_suspend_monit = 0;
	}
	power_supply_changed(&the_chip->batt_psy);
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	power_supply_changed(&the_chip->bms_psy);

	alarm_cancel(&androidalarm);
	androidalarm_interval_timespec = ktime_to_timespec(alarm_get_elapsed_realtime());
	androidalarm_interval_timespec.tv_sec += HKADC_SUSPEND_MONIT_FREQ;
	androidalarm_interval_timespec.tv_nsec = 0;
	alarm_start_range(&androidalarm, timespec_to_ktime(androidalarm_interval_timespec),
		timespec_to_ktime(androidalarm_interval_timespec));

	schedule_delayed_work(&oem_hkadc_work,
			round_jiffies_relative(msecs_to_jiffies(OEM_HKADC_MONITOR_TIME_1S)));
}

static void oem_hkadc_alarm_handler(struct alarm *alarm)
{
	return;
}

static void oem_hkadc_early_suspend(struct early_suspend *h)
{
	return;
}
static void oem_hkadc_early_resume(struct early_suspend *h)
{
	return;
}

static void oem_hkadc_init(void)
{
	memset(&oem_hkadc_master_data, 0x0, sizeof(oem_hkadc_master_data));

	oem_hkadc_init_battery_uvolts();
	oem_hkadc_init_battery_temp();
	oem_hkadc_init_camera_therm();
	oem_hkadc_init_substrate_therm();
	oem_hkadc_init_pa_therm();
	oem_hkadc_init_battery_current();
	oem_hkadc_init_usb_therm();

	atomic_set(&is_hkadc_initialized, 1);

	camera_temp_monit_freq = 0;
	substrate_therm_monit_freq = 0;

	hkadc_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	hkadc_early_suspend.suspend = oem_hkadc_early_suspend;
	hkadc_early_suspend.resume = oem_hkadc_early_resume;
	register_early_suspend(&hkadc_early_suspend);

	MASTERDATA_SPINLOCK_INIT();
	wake_lock_init(&oem_hkadc_wake_lock, WAKE_LOCK_SUSPEND, "oem_hkadc");

	INIT_DELAYED_WORK(&oem_hkadc_work, oem_hkadc_monitor);
	schedule_delayed_work(&oem_hkadc_work,
			round_jiffies_relative(msecs_to_jiffies(0)));

	alarm_init(&androidalarm,
		ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
		oem_hkadc_alarm_handler);
		}

static void oem_hkadc_exit(void)
{
	int rc;

	rc = alarm_cancel(&androidalarm);
	pr_debug("alarm_cancel result=%d\n", rc);

	atomic_set(&is_hkadc_initialized, 0);
	}

int32_t kcbms_get_substrate_therm(int32_t *temp){

	int ret;

	ret = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, temp);
	pr_debug("ret = %d substrate_therm = %d\n", ret, *temp);

	return ret;
}

static int oem_charger_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		ret = oem_charger_master_data_read(psp, intval);
		pr_debug("charger connect state = %d\n", *intval);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
	}

static void oem_chargermonit_init(void)
{
	oem_charger_init_connect_state();
	atomic_set(&is_chargermonit_initialized, 1);
	pr_debug("oem_chargermonit_init()\n");

	}

static void oem_chargermonit_exit(void)
{
	atomic_set(&is_chargermonit_initialized, 0);
	}

void oem_hkadc_vrefbat_on(void)
{
	int rc;
	if (the_chip){

		rc = pm_chg_masked_write(the_chip, CHG_CNTRL_2, VREF_BATT_THERM_FORCE_ON_2,
						VREF_BATT_THERM_FORCE_ON_2);
		if (rc)
			pr_err("Failed to Force Vref therm(2) on rc=%d\n", rc);
	}
}

void oem_hkadc_vrefbat_off(void)
{
		return;
	}

unsigned int oem_chg_is_dc_present(void)
{
	return the_chip->dc_present;
}

unsigned int oem_chg_is_usb_present(void)
{
	return the_chip->usb_present;
	}

int oem_chg_get_oem_stand_detect(void)
{
	return oem_stand_detect;
	}

int oem_chg_get_charge_state(void)
{
	return charge_state;
	}

int oem_chg_get_prop_batt_status(void)
{
	return get_prop_batt_status(the_chip);
	}

int oem_chg_get_prop_charge_type(void)
{
	return get_prop_charge_type(the_chip);
}

#define THRESHOLD_VBATT_UV_LO        3800000
#define THRESHOLD_VBATT_UV_HI        4200000
#define INTERVAL_STAND_CHECK_FIRST   5*60
#define INTERVAL_STAND_CHG_START     30
#define IBATMAX_500                  500
#define IBATMAX_1000                 1000
#define GPIO_BAT_OVP_RST_N           8
#define GPIO_CHG_TBATT2_ON           57
#define GPIO_CHG_TBATT2              58
#define GPIO_CHG_DET_N               69
#define GPIO_BAT_OVP_N               70
#define GPIO_INPUT_LO                0
#define GPIO_INPUT_HI                1
#define NOT_DETECT                   0
#define DETECT_LO_PRE_FIX            1
#define DETECT_LO_FIX                2
#define DETECT_STAND                 1
#define THRESHOLD_IBAT_STAND_CHARGE_STOP_LO (-1500000)
#define THRESHOLD_IBAT_STAND_CHARGE_STOP_HI (-3000000)
#define THRESHOLD_DCIN               6000000

#define THRESHOLD_DCIN_HI (4400 * 1000)
#define THRESHOLD_DCIN_LO (1000 * 1000)

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static void oem_stand_check(void)
{
	struct pm8xxx_adc_chan_result result;
	int ret;

	if (((DETECT_STAND == oem_stand_detect) && (1 != oem_charge_stand_flag)) &&
		(0 == oem_stand_detect_time_counter)) {
		ret = pm8xxx_adc_read(CHANNEL_DCIN, &result);
		if (ret) {
			pr_err("Failed to reading dcin, rc = %d\n", ret);
		return;
	}

		if (((THRESHOLD_DCIN_HI) >= result.physical) &&
		    ((THRESHOLD_DCIN_LO) <= result.physical)) {
			oem_pm8921_disable_source_current(true);
			oem_stand_detect_time_counter = 1;
	}
}

	if (1 <= oem_stand_detect_time_counter) {
		oem_stand_detect_time_counter ++;
		if (4 == oem_stand_detect_time_counter) {
			oem_pm8921_disable_source_current(false);
			oem_stand_detect_time_counter = 0;
		}
	}
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static int oem_chg_stand_on(void)
{
	gpio_set_value(GPIO_BAT_OVP_RST_N, 1);

	mdelay(100);

	gpio_set_value(GPIO_BAT_OVP_RST_N, 0);

	gpio_set_value(GPIO_CHG_TBATT2_ON, 1);

	mdelay(100);

	oem_charge_stand_flag = 1;

	enable_irq(gpio_to_irq(GPIO_BAT_OVP_N));

	schedule_work(&fast_chg_count_start_work);

	return 0;
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static int oem_chg_stand_off(void)
{
	mdelay(100);

	gpio_set_value(GPIO_CHG_TBATT2_ON, 0);

	oem_charge_stand_flag = 0;

	oem_chg_stand_charge_timer = INTERVAL_STAND_CHECK_FIRST;

	return 0;
	}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static bool oem_chg_ibatt_check(void)
{
	bool ibatt = true;
	int get_ibatt = 0;
	int rc;

	rc = get_prop_batt_current(the_chip, &get_ibatt);
	if ((oem_param_charger.i_chg_adp_chk * -1000) < get_ibatt) {
		pr_err("ibatt error %duA\n", get_ibatt);
		ibatt = false;
	}

	return ibatt;
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static bool oem_chg_dcin_check(void)
{
	struct pm8xxx_adc_chan_result result;
	int rc = 0;
	bool dcin = true;

	rc = pm8xxx_adc_read(CHANNEL_DCIN, &result);
	if (rc) {
		pr_err("Failed to reading dcin, rc = %d\n", rc);
	}
	pr_debug("dcin(6x) : %lld\n", result.physical);
	if (THRESHOLD_DCIN < result.physical) {
		dcin = false;
	}

	return dcin;
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static bool oem_chg_dcr_check(void)
{
	int rc = 0;
	int vbatt_1 = 0, vbatt_2 = 0;
	int ibatt_1 = 0, ibatt_2 = 0;
	int calc_dcr = 0;
	bool dcr = true;
	struct pm8xxx_adc_chan_result result;

	rc = pm_chg_ibatmax_set(the_chip, IBATMAX_500);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d [500mA]\n", rc);
	}
	mdelay(100);
	rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->vbat_channel, rc);
		return false;
	}
	vbatt_1 = (int)result.physical;
	rc = get_prop_batt_current(the_chip, &ibatt_1) * (-1);
	if (rc) {
		pr_err("error reading batt current1 rc = %d\n", rc);
	}

	rc = pm_chg_ibatmax_set(the_chip, IBATMAX_1000);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d [1000mA]\n", rc);
	}
	mdelay(100);
	rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->vbat_channel, rc);
		return false;
	}
	vbatt_2 = (int)result.physical;
	rc = get_prop_batt_current(the_chip, &ibatt_2) * (-1);
	if (rc) {
		pr_err("error reading batt current2 rc = %d\n", rc);
	}

	if (ibatt_1 != ibatt_2) {
		calc_dcr = ((vbatt_2 - vbatt_1) * 1000) / (ibatt_2 - ibatt_1);
	} else {
		calc_dcr = 0;
	}

	if (oem_param_charger.z_chg_adp_chk < calc_dcr) {
		dcr = false;
	}

	rc = pm_chg_ibatmax_set(the_chip, the_chip->max_bat_chg_current);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
	}

	return dcr;
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static int oem_tbatt2_detect = 0;
static int oem_det_n_detect  = 0;
struct delayed_work oem_chg_tbatt2_work;
struct delayed_work oem_chg_det_n_work;
struct delayed_work oem_bat_ovp_n_work;
struct wake_lock charging_stand_wake_lock;
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static void oem_chg_stand_check_control(void)
{
	bool check = true;
	int get_ibatt = 0;
	int fsm_state;
	int rc;

	fsm_state = pm_chg_get_fsm_state(the_chip);
	if((the_chip->dc_present || oem_stand_detect) &&
		 (fsm_state == FSM_STATE_FAST_CHG_7) &&
		 (pm_chg_get_rt_status(the_chip, FASTCHG_IRQ)) &&
		 !delayed_work_pending(&the_chip->eoc_work)) {
		wake_lock(&the_chip->eoc_wake_lock);
		schedule_delayed_work(&the_chip->eoc_work,
			round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS)));
	}

	if (oem_stand_detect != DETECT_STAND){
		pr_debug("oem_stand_detect not DETECT_STAND:%d\n", oem_stand_detect);
		return;
	} else {
		if (GPIO_INPUT_HI == gpio_get_value(GPIO_CHG_DET_N)) {
			oem_det_n_detect = NOT_DETECT;
			schedule_delayed_work(&oem_chg_det_n_work,
					round_jiffies_relative(msecs_to_jiffies(100)));
		}
		if (GPIO_INPUT_HI == gpio_get_value(GPIO_CHG_TBATT2)) {
			oem_tbatt2_detect = NOT_DETECT;
			schedule_delayed_work(&oem_chg_tbatt2_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		}
	}

	if (oem_chg_stand_charge_timer != 0){
		oem_chg_stand_charge_timer--;
		pr_debug("oem_chg_stand_charge_timer count. remain:%d\n", oem_chg_stand_charge_timer);
		return;
	}

	if (0 == oem_charge_stand_flag) {
		do {
			if ((!pm_chg_get_rt_status(the_chip, FASTCHG_IRQ)) &&
			    (!oem_pm8921_bms_is_cyclecorrection_chargeoffstate())) {
				pr_debug("Not Fast Charge\n");
				check = false;
				break;
			}
			if (((oem_param_charger.chg_adp_tmp1 * 10) > oem_batt_temp) ||
				((oem_param_charger.chg_adp_tmp3 * 10) < oem_batt_temp)) {
				pr_debug("Batt Temp NG [%d]\n", oem_batt_temp);
				check = false;
				break;
			}
			if ((THRESHOLD_VBATT_UV_LO > oem_vbatt) ||
				(THRESHOLD_VBATT_UV_HI < oem_vbatt)) {
				pr_debug("VBATT NG [%d]\n", oem_vbatt);
				check = false;
				break;
			}
			if (!oem_chg_ibatt_check()) {
				pr_debug("IBATT NG\n");
				check = false;
				break;
			}
			if (!oem_chg_dcin_check() || pm_chg_get_rt_status(the_chip, DCIN_OV_IRQ)){
				pr_debug("Stand Charge do not start -dcin voltage NG detected.\n");
				check = false;
				break;
			}
			if (!oem_chg_dcr_check()) {
				pr_debug("DCR NG\n");
				check = false;
				break;
			}
		} while(0);

		if (check) {
			oem_chg_stand_on();
			pr_debug("stand charge start. \n");
		}

	} else {
		do {
			if (((oem_param_charger.chg_adp_tmp1 * 10) > oem_batt_temp) ||
				((oem_param_charger.chg_adp_tmp2 * 10) < oem_batt_temp)) {
				pr_debug("Stand Charge Stop batt_temp=%d\n", oem_batt_temp);
				check = false;
				break;
			}
			if (!is_batterydetected){
				pr_debug("Stand Charge Stop -battery removed.\n");
				check = false;
				break;
			}
			if (pm_chg_get_rt_status(the_chip, DCIN_OV_IRQ)){
				pr_debug("Stand Charge Stop -adapter over voltage detected.\n");
				check = false;
				break;
			}
			if ((oem_param_charger.chg_stop_volt * 1000) <= oem_vbatt){
				pr_debug("Stand Charge Stop -battery voltage NG detected.\n");
				check = false;
				break;
			}
			if (!oem_chg_dcin_check()){
				pr_debug("Stand Charge Stop -dcin voltage NG detected.\n");
				check = false;
				break;
			}
			if (OEM_STATUS_NORMAL != oem_charge_status) {
				pr_debug("Stand Charge Stop -function limitation oem_charge_status=%d.\n",
				        oem_charge_status);
				check = false;
				break;
	}

			rc = get_prop_batt_current(the_chip, &get_ibatt);
			if ((THRESHOLD_IBAT_STAND_CHARGE_STOP_LO <= (get_ibatt))||
			    (THRESHOLD_IBAT_STAND_CHARGE_STOP_HI > (get_ibatt))){
				pr_debug("Stand Charge Stop -ibat NG detected.%d\n", get_ibatt);
				check = false;
				break;
	}

		} while(0);

		if (!check) {
			oem_chg_stand_off();
			pr_debug("stand charge end. \n");
		}
	}
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
irqreturn_t oem_chg_tbatt2_isr(int irq, void *ptr);
irqreturn_t oem_chg_det_n_isr(int irq, void *ptr);

static bool from_oem_chg_tbatt2_isr = false;
static bool oem_repeat_tbatt2 = false;

static void oem_chg_tbatt2_control(struct work_struct *work)
{
	int rc = 0;
	int gpio_trigger = 0;

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		if (NOT_DETECT == oem_tbatt2_detect) {
			oem_tbatt2_detect = DETECT_LO_PRE_FIX;
			oem_repeat_tbatt2 = true;
		} else if (DETECT_LO_PRE_FIX == oem_tbatt2_detect) {
			oem_tbatt2_detect = DETECT_LO_FIX;
			gpio_trigger = IRQF_TRIGGER_HIGH;
		}
		
		if ((DETECT_LO_FIX == oem_tbatt2_detect) &&
		    (DETECT_LO_FIX == oem_det_n_detect)) {
			oem_stand_detect = DETECT_STAND;
			the_chip->dc_present = 1;
			oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
		}
	} else {
		if (NOT_DETECT != oem_tbatt2_detect) {
			oem_tbatt2_detect = NOT_DETECT;
			oem_repeat_tbatt2 = true;
		} else {
			if (NOT_DETECT != oem_stand_detect) {
				if (1 == oem_charge_stand_flag) {
					oem_chg_stand_off();
	}
				oem_stand_detect = NOT_DETECT;
				cancel_delayed_work_sync(&fast_chg_limit_work);
				is_oem_fast_chg_counting=false;
				the_chip->dc_present = 0;
				pr_debug("oem_chg_tbatt2_control stand removed.\n");
			}
			wake_unlock(&charging_stand_wake_lock);
			gpio_trigger = IRQF_TRIGGER_LOW;
		}
	}

	if (oem_repeat_tbatt2) {
		oem_repeat_tbatt2 = false;
		schedule_delayed_work(&oem_chg_tbatt2_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		return;
	}

	if (from_oem_chg_tbatt2_isr) {
		from_oem_chg_tbatt2_isr = false;
		free_irq(gpio_to_irq(GPIO_CHG_TBATT2), 0);
		rc = request_irq(gpio_to_irq(GPIO_CHG_TBATT2), oem_chg_tbatt2_isr, gpio_trigger, "CHG_TBATT2", 0);
	if (rc) {
			pr_err("couldn't register interrupts rc=%d\n", rc);
		}
		return;
	}

	}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static bool from_oem_chg_det_n_isr = false;
static bool oem_repeat_det_n = false;

static void oem_chg_det_n_control(struct work_struct *work)
{
	int rc = 0;
	int gpio_trigger = 0;

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		if (NOT_DETECT == oem_det_n_detect) {
			oem_det_n_detect = DETECT_LO_PRE_FIX;
			oem_repeat_det_n = true;
		} else if (DETECT_LO_PRE_FIX == oem_det_n_detect) {
			oem_det_n_detect = DETECT_LO_FIX;
			gpio_trigger = IRQF_TRIGGER_HIGH;
		}
		
		if ((DETECT_LO_FIX == oem_det_n_detect) &&
		    (DETECT_LO_FIX == oem_tbatt2_detect)) {
			oem_stand_detect = DETECT_STAND;
			the_chip->dc_present = 1;
			oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
		}
	} else {
		if (NOT_DETECT != oem_det_n_detect) {
			oem_det_n_detect = NOT_DETECT;
			oem_repeat_det_n = true;
		} else {
			if (NOT_DETECT != oem_stand_detect) {
				if (1 == oem_charge_stand_flag) {
					oem_chg_stand_off();
	}
				oem_stand_detect = NOT_DETECT;
				cancel_delayed_work_sync(&fast_chg_limit_work);
				is_oem_fast_chg_counting=false;
				the_chip->dc_present = 0;
				pr_debug("oem_chg_det_n_control stand removed.\n");
		}
			wake_unlock(&charging_stand_wake_lock);
			gpio_trigger = IRQF_TRIGGER_LOW;
	}
		}

	if (oem_repeat_det_n) {
		oem_repeat_det_n = false;
		schedule_delayed_work(&oem_chg_det_n_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		return;
	}

	if (from_oem_chg_det_n_isr) {
		from_oem_chg_det_n_isr = false;
		free_irq(gpio_to_irq(GPIO_CHG_DET_N), 0);
		rc = request_irq(gpio_to_irq(GPIO_CHG_DET_N), oem_chg_det_n_isr, gpio_trigger, "CHG_DET_N", 0);
		if (rc) {
			pr_err("couldn't register interrupts rc=%d\n", rc);
		}
		return;
	}

	}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
static void oem_bat_ovp_n_control(struct work_struct *work)
{
	oem_chg_stand_off();
	}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
irqreturn_t oem_chg_tbatt2_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_CHG_TBATT2));

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		oem_tbatt2_detect = DETECT_LO_PRE_FIX;
		wake_lock(&charging_stand_wake_lock);
	} else {
		oem_tbatt2_detect = NOT_DETECT;
	}

	from_oem_chg_tbatt2_isr = true;

	schedule_delayed_work(&oem_chg_tbatt2_work,
			round_jiffies_relative(msecs_to_jiffies(100)));

	return IRQ_HANDLED;
		}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
irqreturn_t oem_chg_det_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_CHG_DET_N));

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		oem_det_n_detect = DETECT_LO_PRE_FIX;
		wake_lock(&charging_stand_wake_lock);
	} else {
		oem_det_n_detect = NOT_DETECT;
			}

	from_oem_chg_det_n_isr = true;

	schedule_delayed_work(&oem_chg_det_n_work,
			round_jiffies_relative(msecs_to_jiffies(100)));

	return IRQ_HANDLED;
		}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

#ifdef CONFIG_CHARGE_STAND_SUPPORT
irqreturn_t oem_bat_ovp_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_BAT_OVP_N));

	if ((GPIO_INPUT_LO == gpio_get_value(GPIO_BAT_OVP_N)) &&
		(1 == oem_charge_stand_flag)) {
		schedule_delayed_work(&oem_bat_ovp_n_work,
				round_jiffies_relative(msecs_to_jiffies(0)));
	}

	return IRQ_HANDLED;
}
#endif /* CONFIG_CHARGE_STAND_SUPPORT */

