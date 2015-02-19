/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
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
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/mm_types.h>
/*----------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include "board-8960.h"
/*----------------------------------------------------------------------------*/
#include <mach/rpm-regulator.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/rtc.h>
/*----------------------------------------------------------------------------*/
#include <linux/mfd/pm8xxx/pm8921-charger.h>
/*----------------------------------------------------------------------------*/
#include <linux/dnand_cdev_driver.h>
#include <linux/dnand_clid.h>
/*----------------------------------------------------------------------------*/
typedef void (*err_test_func_type)(void);
/*----------------------------------------------------------------------------*/
#define PMIC_TEST_LOG_01 KERN_ERR
#define PMIC_TEST_LOG_02 KERN_ERR
#define PMIC_TEST_LOG_03 KERN_ERR
#define PMIC_TEST_LOG_04 KERN_DEBUG
#define PMIC_TEST_LOG_05 KERN_DEBUG
#define PMIC_TEST_LOG_06 KERN_DEBUG

#define PM_REGU_LDO_NUM		29
#define PM_REGU_SMPS_NUM	8
#define PM_REGU_LVS_NUM		7
#define PM_REGU_MAX_NUM		(PM_REGU_LDO_NUM+PM_REGU_SMPS_NUM+PM_REGU_LVS_NUM)
#define PM_REGU_LVS_NUM_START	(PM_REGU_MAX_NUM-PM_REGU_LVS_NUM-1)
#define PM_REGU_LVS_NUM_END		(PM_REGU_MAX_NUM-1)

#define PMIC_TEST_V_PER_BIT_MUL_FACTOR	97656
#define PMIC_TEST_V_PER_BIT_DIV_FACTOR	1000
#define PMIC_TEST_XOADC_INTRINSIC_OFFSET	0x6000

enum {
	PMIC_TEST_RTC		=	0x0000,
	PMIC_TEST_CHG		=	0x0001,
	PMIC_TEST_USB		=	0x0002,
	PMIC_TEST_AMUX		=	0x0003,
	PMIC_TEST_VREG		=	0x0004,
	PMIC_TEST_INT		=	0x0005,
	PMIC_TEST_UI		=	0x0006,
	PMIC_TEST_DVT		=	0x0007,
	PMIC_TEST_SPKR		=	0x0008,
	PMIC_TEST_VID		=	0x0009,
	PMIC_TEST_MIC		=	0x000A,
	PMIC_TEST_RESET		=	0x000B,
	PMIC_TEST_MPP		=	0x000C,
	PMIC_TEST_GEN		=	0x000D,
	PMIC_FUSE_CMDS		=	0x000E,
	PMIC_TEST_SMPL		=	0x0100,
	PMIC_TEST_TCXO		=	0x0101,
	PMIC_TEST_VIB		=	0x0102,
	PMIC_TEST_MBG		=	0x0105,
	PMIC_TEST_XTAL		=	0x0106,
	PMIC_TEST_XO		=	0x0107,
	PMIC_TEST_XOADC		=	0x0108,
	PMIC_TEST_GPIO		=	0x0109,
	PMIC_TEST_HSED		=	0x010A,
	PMIC_TEST_PAON		=	0x010B,
	PMIC_TEST_PWM		=	0x010C,
	PMIC_TEST_SEC_INT	=	0x010D,
	PMIC_TEST_PWRON		=	0x010E,
	PMIC_TEST_KCGEN		=	0x010F,
	PMIC_TEST_KCCHG		=	0x0110,
};

enum {
	PMIC_TEST_GPIO_SET,
	PMIC_TEST_GPIO_GET,
	PMIC_TEST_GPIO_CONFIG,
	PMIC_TEST_GPIO_UART_CTRL,
};
enum {
	PMIC_TEST_MPP_SET,
	PMIC_TEST_MPP_GET,
	PMIC_TEST_MPP_CONFIG,
};
enum {
	PMIC_TEST_VREG_ON,
	PMIC_TEST_VREG_OFF,
	PMIC_TEST_VREG_SET_VOLT,
	PMIC_TEST_VREG_SET_MODE,
	PMIC_TEST_VREG_SET_FREQ,
};
enum {
	PMIC_TEST_AMUX_GET,
};
enum {
	PMIC_TEST_RTC_GET_TIME,
	PMIC_TEST_RTC_SET_TIME,
	PMIC_TEST_RTC_GET_ALARM,
	PMIC_TEST_RTC_SET_ALARM,
};
enum {
	PMIC_TEST_CHG_SET_IVAT_MAX,
	PMIC_TEST_CHG_SET_IUSB_MAX,
};
enum {
	PMIC_TEST_PWRON_GET_STATUS,
	PMIC_TEST_PWRON_SMPL_CTRL,
	PMIC_TEST_PWRON_SMPL_SET_DELAY,
	PMIC_TEST_PWRON_HARD_RESET_CONFIG,
};
/*----------------------------------------------------------------------------*/
typedef struct {
	const char *id;
	int min_uV;
	int max_uV;
} ldo_setting_data;

typedef struct {
	int data1;
	int data2;
	int data3;
	int data4;
	int data5;
} str_pmic_res;

typedef struct {
	int cmd;
	int port_no;
	int arg1;
	int arg2;
	int arg3;
} str_gpio_req;

typedef struct {
	int cmd;
	int port_no;
	int arg1;
	int arg2;
	int arg3;
} str_mpp_req;

typedef struct {
	int cmd;
	int vreg_no;
	int arg1;
	int arg2;
	int arg3;
} str_vreg_req;

typedef struct {
	int cmd;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
} str_amux_req;

typedef struct {
	int cmd;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
} str_rtc_req;

typedef struct {
	int cmd;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
} str_chg_req;

typedef struct {
	int cmd;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
} str_pwron_req;

/*----------------------------------------------------------------------------*/
static struct regulator *reg_store[PM_REGU_MAX_NUM];
static int reg_volt_set_check[PM_REGU_MAX_NUM];
/*----------------------------------------------------------------------------*/
ldo_setting_data		dev_ldo_data[PM_REGU_MAX_NUM] = {
	/*	id				min_uV		max_uV			name	*/
	{	"8921_l1",		1050000,	1050000	},	/*	RPM_LDO_L1	*/
	{	"8921_l2",		1200000,	1200000	},	/*	RPM_LDO_L2	*/
	{	"8921_l3",		3075000,	3075000	},	/*	RPM_LDO_L3	*/
	{	"8921_l4",		1800000,	1800000	},	/*	RPM_LDO_L4	*/
	{	"8921_l5",		2750000,	3000000	},	/*	RPM_LDO_L5	*/
	{	"8921_l6",		2750000,	3000000	},	/*	RPM_LDO_L6	*/
	{	"8921_l7",		3000000,	3000000	},	/*	RPM_LDO_L7	*/
	{	"8921_l8",		3000000,	3000000	},	/*	RPM_LDO_L8	*/
	{	"8921_l9",		2800000,	3000000	},	/*	RPM_LDO_L9	*/
	{	"8921_l10",		3000000,	3000000	},	/*	RPM_LDO_L10	*/
	{	"8921_l11",		2700000,	3300000	},	/*	RPM_LDO_L11	*/
	{	"8921_l12",		1200000,	1450000	},	/*	RPM_LDO_L12	*/
	{	"8921_l13",		0,	0	},	/*	RPM_LDO_L13	*/
	{	"8921_l14",		1700000,	1900000	},	/*	RPM_LDO_L14	*/
	{	"8921_l15",		2850000,	3300000	},	/*	RPM_LDO_L15	*/
	{	"8921_l16",		2700000,	3300000	},	/*	RPM_LDO_L16	*/
	{	"8921_l17",		3000000,	3000000	},	/*	RPM_LDO_L17	*/
	{	"8921_l18",		1200000,	1500000	},	/*	RPM_LDO_L18	*/
	{	"8921_l19",		0,	0	},	/*	RPM_LDO_L19	*/
	{	"8921_l20",		0,	0	},	/*	RPM_LDO_L20	*/
	{	"8921_l21",		1900000,	1900000	},	/*	RPM_LDO_L21	*/
	{	"8921_l22",		2850000,	2850000	},	/*	RPM_LDO_L22	*/
	{	"8921_l23",		1800000,	1800000	},	/*	RPM_LDO_L23	*/
	{	"8921_l24",		750000,	1150000	},	/*	RPM_LDO_L24	*/
	{	"8921_l25",		1225000,	1225000	},	/*	RPM_LDO_L25	*/
	{	"8921_l26",		1050000,	1050000	},	/*	RPM_LDO_L26	*/
	{	"8921_l27",		1050000,	1050000	},	/*	RPM_LDO_L27	*/
	{	"8921_l28",		1050000,	1050000	},	/*	RPM_LDO_L28	*/
	{	"8921_l29",		1800000,	2200000	},	/*	RPM_LDO_L29	*/
	{	"8921_s1",		1225000,	1225000	},	/*	RPM_SMPS_1	*/
	{	"8921_s2",		1300000,	1300000	},	/*	RPM_SMPS_2	*/
	{	"8921_s3",		500000,	1150000	},	/*	RPM_SMPS_3	*/
	{	"8921_s4",		1800000,	1800000	},	/*	RPM_SMPS_4	*/
	{	"8921_s5",		850000,	1300000	},	/*	RPM_SMPS_5	*/
	{	"8921_s6",		850000,	1300000	},	/*	RPM_SMPS_6	*/
	{	"8921_s7",		1150000,	1150000	},	/*	RPM_SMPS_7	*/
	{	"8921_s8",		2100000,	2100000	},	/*	RPM_SMPS_8	*/
	{	"8921_lvs1",		1800000,	1800000	},	/*	RPM_VS_LVS1(constant voltage)	*/
	{	"8921_lvs2",		1200000,	1200000	},	/*	RPM_VS_LVS2(constant voltage)	*/
	{	"8921_lvs3",		1800000,	1800000	},	/*	RPM_VS_LVS3(constant voltage)	*/
	{	"8921_lvs4",		1800000,	1800000	},	/*	RPM_VS_LVS4(constant voltage)	*/
	{	"8921_lvs5",		1800000,	1800000	},	/*	RPM_VS_LVS5(constant voltage)	*/
	{	"8921_lvs6",		1800000,	1800000	},	/*	RPM_VS_LVS6(constant voltage)	*/
	{	"8921_lvs7",		1800000,	1800000	},	/*	RPM_VS_LVS7(constant voltage)	*/
};
/*----------------------------------------------------------------------------*/
/*                            GPIO                                            */
/*----------------------------------------------------------------------------*/
static int gpio_test_set(int port_no, int arg1)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(port_no), "label");
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() gpio_request ERR\n", __LINE__, __func__);
		if(!(rc == -EBUSY))
			return -EFAULT;
	}
	gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(port_no), arg1);
	if(!rc)
		gpio_free(PM8921_GPIO_PM_TO_SYS(port_no));
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
static int gpio_test_get(int port_no)
{
	int arg1 = -1;
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rc = gpio_request(PM8921_GPIO_PM_TO_SYS(port_no), "label");
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() gpio_request ERR\n", __LINE__, __func__);
		if(!(rc == -EBUSY))
			return -EFAULT;
	}
	arg1 = gpio_get_value_cansleep(PM8921_GPIO_PM_TO_SYS(port_no));
	if(!rc)
		gpio_free(PM8921_GPIO_PM_TO_SYS(port_no));
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  arg1 = %d\n", __LINE__, __func__, arg1);
	return arg1;
}
static int gpio_test_config(int port_no, int arg1, int arg2)
{
	int rc = 0;
	struct pm_gpio test_gpio_config;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	/* kernel\include\linux\mfd\pm8xxx\gpio.h */
	test_gpio_config.direction      = 0xFF & arg1;
	test_gpio_config.pull           = 0xFF & (arg1 >> 0x08);
	test_gpio_config.out_strength   = 0xFF & (arg1 >> 0x10);
	test_gpio_config.function       = 0xFF & (arg1 >> 0x18);
	test_gpio_config.inv_int_pol    = 0xFF & arg2;
	test_gpio_config.vin_sel        = 0xFF & (arg2 >> 0x08);
	test_gpio_config.output_buffer  = 0xFF & (arg2 >> 0x10);
	test_gpio_config.output_value   = 0xFF & (arg2 >> 0x18);

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() port_no = %d, direction = %d, pull = %d, out_strength = %d\n \
	function = %d, inv_int_pol = %d, vin_sel = %d, output_buffer = %d, output_value = %d\n"
	, __LINE__, __func__, port_no,
	test_gpio_config.direction,
	test_gpio_config.pull,
	test_gpio_config.out_strength,
	test_gpio_config.function,
	test_gpio_config.inv_int_pol,
	test_gpio_config.vin_sel,
	test_gpio_config.output_buffer,
	test_gpio_config.output_value);
	rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(port_no), &test_gpio_config);

	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
static int gpio_test_uart_ctrl(int uart_path_sel)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() uart_path_sel = %d\n", __LINE__, __func__, uart_path_sel);
	rc = pm8xxx_uart_gpio_mux_ctrl((enum pm8xxx_uart_path_sel)uart_path_sel);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
static int gpio_test(unsigned long arg)
{
	int rc = 0;
	str_gpio_req req;
	str_pmic_res res;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X  port_no = %d\n", __LINE__, __func__, req.cmd, req.port_no);
	memset(&res, 0x00, sizeof(res));
	switch(req.cmd)
	{
	case PMIC_TEST_GPIO_SET:
		rc = gpio_test_set(req.port_no, req.arg1);
		break;

	case PMIC_TEST_GPIO_GET:
		res.data1 = gpio_test_get(req.port_no);
		if(res.data1 < 0) {
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() res.data1 < 0\n", __LINE__, __func__);
			return -EINVAL;
		}
		break;

	case PMIC_TEST_GPIO_CONFIG:
		rc = gpio_test_config(req.port_no, req.arg1, req.arg2);
		break;

	case PMIC_TEST_GPIO_UART_CTRL:
		rc = gpio_test_uart_ctrl(req.port_no);
		break;

	default:
		return -EINVAL;
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}

/*----------------------------------------------------------------------------*/
/*                            MPP                                             */
/*----------------------------------------------------------------------------*/
static int mpp_test_set(int port_no, int arg1)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rc = gpio_request(PM8921_MPP_PM_TO_SYS(port_no), "label");
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() gpio_request ERR\n", __LINE__, __func__);
		if(!(rc == -EBUSY))
			return -EFAULT;
	}
	gpio_set_value_cansleep(PM8921_MPP_PM_TO_SYS(port_no), arg1);
	if(!rc)
		gpio_free(PM8921_MPP_PM_TO_SYS(port_no));
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
static int mpp_test_get(int port_no)
{
	int arg1 = -1;
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rc = gpio_request(PM8921_MPP_PM_TO_SYS(port_no), "label");
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() gpio_request ERR\n", __LINE__, __func__);
		if(!(rc == -EBUSY))
			return -EFAULT;
	}
	arg1 = gpio_get_value_cansleep(PM8921_MPP_PM_TO_SYS(port_no));
	if(!rc)
		gpio_free(PM8921_MPP_PM_TO_SYS(port_no));
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  arg1 = %d\n", __LINE__, __func__, arg1);
	return arg1;
}
static int mpp_test_config(int port_no, int arg1)
{
	int rc = 0;
	struct pm8xxx_mpp_config_data test_mpp_config;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	/* kernel\include\linux\mfd\pm8xxx\mpp.h */
	test_mpp_config.type    = 0xFF & arg1;
	test_mpp_config.level   = 0xFF & (arg1 >> 0x08);
	test_mpp_config.control = 0xFF & (arg1 >> 0x10);

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() port_no = %d, type = %d, level = %d, control = %d\n"
	, __LINE__, __func__, port_no,
	test_mpp_config.type,
	test_mpp_config.level,
	test_mpp_config.control);
	rc = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(port_no), &test_mpp_config);

	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
static int mpp_test(unsigned long arg)
{
	int rc = 0;
	str_mpp_req req;
	str_pmic_res res;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X  port_no = %d\n", __LINE__, __func__, req.cmd, req.port_no);
	memset(&res, 0x00, sizeof(res));
	switch(req.cmd)
	{
	case PMIC_TEST_MPP_SET:
		rc = mpp_test_set(req.port_no, req.arg1);
		break;

	case PMIC_TEST_MPP_GET:
		res.data1 = mpp_test_get(req.port_no);
		if(res.data1 < 0) {
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() res.data1 < 0\n", __LINE__, __func__);
			return -EINVAL;
		}
		break;

	case PMIC_TEST_MPP_CONFIG:
		rc = mpp_test_config(req.port_no, req.arg1);
		break;

	default:
		return -EINVAL;
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
/*                            VREG                                            */
/*----------------------------------------------------------------------------*/
static int vreg_test_enable(int vreg_no)
{
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  vreg_no = %d\n", __LINE__, __func__, vreg_no);
	vreg_no -= 1;
	if(!reg_store[vreg_no]){
		reg_store[vreg_no] = regulator_get(NULL, dev_ldo_data[vreg_no].id);
		if(IS_ERR(reg_store[vreg_no])){
			reg_store[vreg_no] = NULL;
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
			return -ENODEV;
		}
	}
	if(!(PM_REGU_LVS_NUM_START <= vreg_no && vreg_no <= PM_REGU_LVS_NUM_END))
	{
		if(!reg_volt_set_check[vreg_no]) {
			if(regulator_set_voltage(reg_store[vreg_no], dev_ldo_data[vreg_no].min_uV,
				dev_ldo_data[vreg_no].max_uV)) {
				printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
				goto reg_ldo_put;
			}
			reg_volt_set_check[vreg_no] = 1;
		}
	}
	if(regulator_enable(reg_store[vreg_no]))
		goto reg_ldo_disable;
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;

reg_ldo_disable:
	if(!(PM_REGU_LVS_NUM_START <= vreg_no && vreg_no <= PM_REGU_LVS_NUM_END))
	{
		regulator_set_voltage(reg_store[vreg_no], 0, dev_ldo_data[vreg_no].max_uV);
		regulator_force_disable(reg_store[vreg_no]);
	}
	else
		regulator_disable(reg_store[vreg_no]);
reg_ldo_put:
	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	reg_volt_set_check[vreg_no] = 0;
	printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
	return -ENODEV;
}
static int vreg_test_disable(int vreg_no)
{
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  vreg_no = %d\n", __LINE__, __func__, vreg_no);
	vreg_no -= 1;
	if(!reg_store[vreg_no]){
		reg_store[vreg_no] = regulator_get(NULL, dev_ldo_data[vreg_no].id);
		if(IS_ERR(reg_store[vreg_no])){
			reg_store[vreg_no] = NULL;
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
			return -ENODEV;
		}
	}
	if(!(PM_REGU_LVS_NUM_START <= vreg_no && vreg_no <= PM_REGU_LVS_NUM_END))
	{
		regulator_set_voltage(reg_store[vreg_no], 0, dev_ldo_data[vreg_no].max_uV);
		regulator_force_disable(reg_store[vreg_no]);
	}
	else
		regulator_disable(reg_store[vreg_no]);
	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	reg_volt_set_check[vreg_no] = 0;
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
static int vreg_test_set_volt(int vreg_no, int min_uV, int max_uV)
{
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  vreg_no = %d, min_uV = %d, max_uV = %d\n", __LINE__, __func__, vreg_no, min_uV, max_uV);
	vreg_no -= 1;
	if(PM_REGU_LVS_NUM_START <= vreg_no && vreg_no <= PM_REGU_LVS_NUM_END)
		return 0;
	if(!reg_store[vreg_no]){
		reg_store[vreg_no] = regulator_get(NULL, dev_ldo_data[vreg_no].id);
		if(IS_ERR(reg_store[vreg_no])){
			reg_store[vreg_no] = NULL;
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
			return -ENODEV;
		}
	}
	regulator_set_voltage(reg_store[vreg_no], min_uV, max_uV);
	reg_volt_set_check[vreg_no] = 1;
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
static int vreg_test_set_mode(int vreg_no, int mode)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  vreg_no = %d, mode = %d\n", __LINE__, __func__, vreg_no, mode);
	vreg_no -= 1;
	if(!reg_store[vreg_no]){
		reg_store[vreg_no] = regulator_get(NULL, dev_ldo_data[vreg_no].id);
		if(IS_ERR(reg_store[vreg_no])){
			reg_store[vreg_no] = NULL;
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
			return -ENODEV;
		}
	}
	rc = regulator_set_mode(reg_store[vreg_no], mode);
	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}
static int vreg_test_set_freq(int vreg_no, int freq)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  vreg_no = %d, freq = %d\n", __LINE__, __func__, vreg_no, freq);
	rc = rpm_vreg_set_frequency((enum rpm_vreg_id_8960)vreg_no, (enum rpm_vreg_freq)freq);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
static int vreg_test(unsigned long arg)
{
	int rc = 0;
	str_vreg_req req;
	str_pmic_res res;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X  vreg_no = %d\n", __LINE__, __func__, req.cmd, req.vreg_no);
	memset(&res, 0x00, sizeof(res));
	switch(req.cmd)
	{
	case PMIC_TEST_VREG_ON:
		rc = vreg_test_enable(req.vreg_no);
		break;

	case PMIC_TEST_VREG_OFF:
		rc = vreg_test_disable(req.vreg_no);
		break;

	case PMIC_TEST_VREG_SET_VOLT:
		rc = vreg_test_set_volt(req.vreg_no, req.arg1, req.arg2);
		break;

	case PMIC_TEST_VREG_SET_MODE:
		rc = vreg_test_set_mode(req.vreg_no, req.arg1);
		break;

	case PMIC_TEST_VREG_SET_FREQ:
		rc = vreg_test_set_freq(req.vreg_no, req.arg1);
		break;

	default:
		return -EINVAL;
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
/*                            AMUX                                            */
/*----------------------------------------------------------------------------*/
static int amux_test_get(int channel, struct pm8xxx_adc_chan_result *result)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.:result = 0x%08X\n", __LINE__, __func__, (unsigned int)result);
	rc = pm8xxx_adc_read((enum pm8xxx_adc_channels)channel, result);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
static int pmic_test_xoadc_reading_to_microvolt(unsigned int a)
{
	if (a <= PMIC_TEST_XOADC_INTRINSIC_OFFSET)
		return 0;

	return (a - PMIC_TEST_XOADC_INTRINSIC_OFFSET)
			* PMIC_TEST_V_PER_BIT_MUL_FACTOR / PMIC_TEST_V_PER_BIT_DIV_FACTOR;
}
/*----------------------------------------------------------------------------*/
int64_t pmic_test_div_64bit(int64_t numerator, unsigned int denominator)
{
	union {
	int64_t longlong;
	unsigned short ushort[4];
	} x, y;
	unsigned int div, mod;

	x.longlong = numerator;

	div = x.ushort[3] / denominator;
	mod = x.ushort[3] % denominator;
	y.ushort[3] = div;

	div = (mod * 65536 + x.ushort[2]) / denominator;
	mod = (mod * 65536 + x.ushort[2]) % denominator;
	y.ushort[2] = div;

	div = (mod * 65536 + x.ushort[1]) / denominator;
	mod = (mod * 65536 + x.ushort[1]) % denominator;
	y.ushort[1] = div;

	div = (mod * 65536 + x.ushort[0]) / denominator;
	mod = (mod * 65536 + x.ushort[0]) % denominator;
	y.ushort[0] = div;

	return y.longlong;
}
/*----------------------------------------------------------------------------*/
static int amux_test(unsigned long arg)
{
	int rc = 0;
	str_amux_req req;
	str_pmic_res res;
	struct pm8xxx_adc_chan_result result;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	if(		req.arg1 != CHANNEL_VBAT
		&&	req.arg1 != CHANNEL_BATT_THERM
		&&	req.arg1 != CHANNEL_BATT_ID
		&&	req.arg1 != CHANNEL_DIE_TEMP
		&&	req.arg1 != CHANNEL_625MV
		&&	req.arg1 != CHANNEL_125V ) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() BAD arg\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X  port_no = %d\n", __LINE__, __func__, req.cmd, req.arg1);
	memset(&res, 0x00, sizeof(res));
	switch(req.cmd)
	{
	case PMIC_TEST_AMUX_GET:
		res.data1 = amux_test_get(req.arg1, &result);
		if(res.data1 < 0) {
			printk(PMIC_TEST_LOG_01 "[%04d]:%s() res.data1 < 0\n", __LINE__, __func__);
			return -EINVAL;
		}
		if(req.arg1 == CHANNEL_625MV || req.arg1 == CHANNEL_125V) {
			res.data1 = pmic_test_xoadc_reading_to_microvolt(result.adc_code);
			break;
		}
		else if(req.arg1 == CHANNEL_BATT_THERM) {
			if(result.physical > 0)
				result.physical = pmic_test_div_64bit(result.physical, 10);
		}
		else if(req.arg1 == CHANNEL_DIE_TEMP) {
			if(result.physical > 0)
				result.physical = pmic_test_div_64bit(result.physical, 1000);
		}
		res.data1 = 0xFFFFFFFF & result.physical;
		res.data2 = 0xFFFFFFFF & (result.physical >> 0x20);
		break;

	default:
		return -EINVAL;
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
/*                            RTC                                             */
/*----------------------------------------------------------------------------*/
static int rtc_test_get_time(struct rtc_time *tm)
{
	int rc = 0;
	struct rtc_device *rtc;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	rtc = rtc_class_open("rtc0");
	if (rtc == NULL) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		return -EFAULT;
	}
	rc = rtc_read_time(rtc, tm);
	if (rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		rtc_class_close(rtc);
		return -EFAULT;
	}
	rtc_class_close(rtc);

	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}
static int rtc_test_set_time(struct rtc_time tm)
{
	int rc = 0;
	struct rtc_device *rtc;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rtc = rtc_class_open("rtc0");
	if (rtc == NULL) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		return -EFAULT;
	}
	rc = rtc_set_time(rtc, &tm);
	if (rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		rtc_class_close(rtc);
		return -EFAULT;
	}
	rtc_class_close(rtc);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}
static int rtc_test_get_alarm(struct rtc_time *tm, int *info)
{
	int rc = 0;
	struct rtc_wkalrm alarm;
	struct rtc_device *rtc;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	rtc = rtc_class_open("rtc0");
	if (rtc == NULL) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		return -EFAULT;
	}
	rc = rtc_read_alarm(rtc, &alarm);
	if (rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		rtc_class_close(rtc);
		return -EFAULT;
	}
	rtc_class_close(rtc);

	tm = &alarm.time;

	*info =		(alarm.enabled << 0x10 & 0xFF0000);
	*info |=	alarm.pending;

	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}
static int rtc_test_set_alarm(struct rtc_time tm, int info)
{
	int rc = 0;
	struct rtc_wkalrm alarm;
	struct rtc_device *rtc;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	memcpy(&alarm.time, &tm, sizeof(alarm.time));
	alarm.enabled = (char)((info >> 0x10) & 0xFF);
	alarm.pending = (char)(info & 0xFF);

	rtc = rtc_class_open("rtc0");
	if (rtc == NULL) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		return -EFAULT;
	}
	rc = rtc_set_alarm(rtc, &alarm);
	if (rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
		rtc_class_close(rtc);
		return -EFAULT;
	}
	rtc_class_close(rtc);

	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}
/*----------------------------------------------------------------------------*/
static int rtc_test(unsigned long arg)
{
	int rc = 0;
	str_rtc_req req;
	str_pmic_res res;
	struct rtc_time tm;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X\n", __LINE__, __func__, req.cmd);
	memset(&res, 0x00, sizeof(res));

	if(req.cmd == PMIC_TEST_RTC_SET_TIME || req.cmd == PMIC_TEST_RTC_SET_ALARM)
	{
		tm.tm_sec = (int)(req.arg1 & 0xFF);							/* SECOND (0~59) */
		tm.tm_min = (int)((req.arg1 >> 0x08) & 0xFF);				/* MINUTE (0~59) */
		tm.tm_hour = (int)((req.arg1 >> 0x10) & 0xFF);				/* HOUR   (0~23) */
		tm.tm_mday = (int)((req.arg1 >> 0x18) & 0xFF);				/* DATE   (1~31) */
		tm.tm_mon = (int)((req.arg1 & 0xFF) -1);					/* MONTH  (0~11) */
		tm.tm_year = (int)(((req.arg1 >> 0x08) & 0xFFFF) -1900);	/* YEAR   (YEAR - 1900) */
		tm.tm_wday = (int)((req.arg1 >> 0x18) & 0xFF);				/* WEEK   (0~6 : 0=sunday) */
	}
	switch(req.cmd)
	{
	case PMIC_TEST_RTC_GET_TIME:
		rc = rtc_test_get_time(&tm);
		break;

	case PMIC_TEST_RTC_SET_TIME:
		rc = rtc_test_set_time(tm);
		break;

	case PMIC_TEST_RTC_GET_ALARM:
		rc = rtc_test_get_alarm(&tm, &res.data3);
		break;

	case PMIC_TEST_RTC_SET_ALARM:
		rc = rtc_test_set_alarm(tm, req.arg3);
		break;

	default:
		return -EINVAL;
	}
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() rc = %d\n", __LINE__, __func__, rc);
		return -EINVAL;
	}
	if(req.cmd == PMIC_TEST_RTC_GET_TIME || req.cmd == PMIC_TEST_RTC_GET_ALARM)
	{
		res.data1 |= tm.tm_sec & 0xFF;
		res.data1 |= ((tm.tm_min << 0x08) & 0xFF00);
		res.data1 |= ((tm.tm_hour << 0x10) & 0xFF0000);
		res.data1 |= ((tm.tm_mday << 0x18) & 0xFF000000);
		res.data2 |= (tm.tm_mon +1) & 0xFF;
		res.data2 |= (((tm.tm_year + 1900) << 0x08) & 0xFFFF00);
		res.data2 |= ((tm.tm_wday << 0x18) & 0xFF000000);
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
/*                            CHG                                             */
/*----------------------------------------------------------------------------*/
static int chg_test_set_ivat_max(int ma)
{
	int rc = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  ma = %d\n", __LINE__, __func__, ma);
	rc = pm8921_set_max_battery_charge_current(ma);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
static int chg_test_set_iusb_max(int ma)
{
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  ma = %d\n", __LINE__, __func__, ma);
	pm8921_charger_vbus_draw(ma);
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int chg_test(unsigned long arg)
{
	int rc = 0;
	str_chg_req req;
	str_pmic_res res;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if(copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X\n", __LINE__, __func__, req.cmd);
	memset(&res, 0x00, sizeof(res));
	switch(req.cmd)
	{
	case PMIC_TEST_CHG_SET_IVAT_MAX:
		rc = chg_test_set_ivat_max(req.arg1);
		break;

	case PMIC_TEST_CHG_SET_IUSB_MAX:
		rc = chg_test_set_iusb_max(req.arg1);
		break;

	default:
		return -EINVAL;
	}
	if(rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() rc = %d\n", __LINE__, __func__, rc);
		return -EINVAL;
	}
	if(copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n", __LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
/*                            PWRON                                           */
/*----------------------------------------------------------------------------*/
static int pwron_test(unsigned long arg)
{
	int rc = 0;
	str_pwron_req req;
	str_pmic_res res;
	struct kc_pm8xxx_hard_reset_config config;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if (copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_from_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() cmd = 0x%08X\n", __LINE__,
							__func__, req.cmd);

	memset(&res, 0x00, sizeof(res));
	switch (req.cmd) {
	case PMIC_TEST_PWRON_HARD_RESET_CONFIG:
		config.enable   = (req.arg1 & 0xFF);
		config.delay    = ((req.arg1 >> 8) & 0xFF);
		config.debounce = ((req.arg1 >> 16) & 0xFF);
		rc = kc_pm8xxx_hard_reset_config(&config);
		break;

	default:
		return -EINVAL;
	}

	if (rc) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() rc = %d\n", __LINE__,
								__func__, rc);
		return -EINVAL;
	}
	if (copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(PMIC_TEST_LOG_01 "[%04d]:%s() copy_to_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() end.  rc = %d\n", __LINE__,
								__func__, rc);
	return rc;
}

/*----------------------------------------------------------------------------*/
/*                            not use                                         */
/*----------------------------------------------------------------------------*/
static ssize_t pmic_test_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
/*                            write                                           */
/*----------------------------------------------------------------------------*/
static ssize_t pmic_test_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	uint8_t pbuf = 0;
	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	kdnand_id_write(DNAND_ID_KERNEL_25, 0, &pbuf, sizeof(uint8_t));
	pbuf = 1;
	kdnand_id_read(DNAND_ID_KERNEL_25, 0, &pbuf, sizeof(uint8_t));
	printk(PMIC_TEST_LOG_04 "[%04d]:%s() end.  kdnand = %d\n", __LINE__, __func__, pbuf);
	return count;
}
/*----------------------------------------------------------------------------*/
/*                            open                                            */
/*----------------------------------------------------------------------------*/
static int pmic_test_open(struct inode *ip, struct file *fp)
{
	int	ret;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	ret = nonseekable_open(ip, fp);
	printk(PMIC_TEST_LOG_04 "[%04d]:%s() end.  ret = %d\n", __LINE__, __func__, ret);
	if (ret)
	{
		return ret;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
/*                            release                                         */
/*----------------------------------------------------------------------------*/
static int pmic_test_release(struct inode *ip, struct file *fp)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
/*                            ioctl                                           */
/*----------------------------------------------------------------------------*/
static long pmic_test_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	printk(PMIC_TEST_LOG_06 "[%04d]:%s() start.  cmd = 0x%08X\n", __LINE__, __func__, cmd);

	switch(cmd)
	{
	case PMIC_TEST_GPIO:
		rc = gpio_test(arg);
		break;

	case PMIC_TEST_MPP:
		rc = mpp_test(arg);
		break;

	case PMIC_TEST_VREG:
		rc = vreg_test(arg);
		break;

	case PMIC_TEST_AMUX:
		rc = amux_test(arg);
		break;

	case PMIC_TEST_RTC:
		rc = rtc_test(arg);
		break;

	case PMIC_TEST_CHG:
		rc = chg_test(arg);
		break;

	case PMIC_TEST_PWRON:
		rc = pwron_test(arg);
		break;

	default:
		return -EINVAL;
	}
	printk(PMIC_TEST_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__, __func__, rc);
	return rc;
}
/*----------------------------------------------------------------------------*/
static const struct file_operations pmic_test_fops = {
	.owner			= THIS_MODULE,
	.read			= pmic_test_read,
	.write			= pmic_test_write,
	.open			= pmic_test_open,
	.release		= pmic_test_release,
	.unlocked_ioctl	= pmic_test_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice pmic_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kc_pmic_test",
	.fops = &pmic_test_fops,
};
/*----------------------------------------------------------------------------*/
static int __init pmic_test_init(void)
{
	return misc_register(&pmic_test_dev);
}
/*----------------------------------------------------------------------------*/
module_init(pmic_test_init);

MODULE_DESCRIPTION("PMIC Test");
MODULE_LICENSE("GPL v2");
