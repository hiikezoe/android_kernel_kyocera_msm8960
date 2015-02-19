/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/export.h>

#include <linux/dtvtuner_pm.h>

/* delay time (msec) */
#define D_DTVTUNER_DEVICE_RST_WAITTIME		(100)

struct dtvtuner_pm_drvdata {
	struct platform_device *pdev;
};

/**
 * dtvtuner_pm_dev_init() - Device initialization.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 *   -EIO - Fail of initialization
 */
static int dtvtuner_pm_dev_init(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	struct dtvtuner_pm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;
	int ret;

	ret = gpio_request(pfdata->gpio_rst, "dtvtuner HW reset");
	if (ret) {
		dev_err(&drvdata->pdev->dev, "RST request %d\n", ret);
		goto err_request_gpio_rst_req;
	}

	ret = gpio_direction_output(pfdata->gpio_rst, 0);
	if (ret) {
		dev_err(&drvdata->pdev->dev, "RST status %d\n", ret);
		goto err_request_gpio_rst;
	}

	return ret;

err_request_gpio_rst:
	gpio_free(pfdata->gpio_rst);
err_request_gpio_rst_req:

	return -EIO;
}

/**
 * dtvtuner_pm_dev_finalize() - Device finalization.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_finalize(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	struct dtvtuner_pm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	gpio_free(pfdata->gpio_rst);

	return 0;
}

/**
 * dtvtuner_pm_dev_gpio_put() - put value to GPIO.
 * @gpio:	[IN]    number of gpio
 * @data:	[IN]    put value
 */
static void dtvtuner_pm_dev_gpio_put(
	unsigned gpio,
	int      data
)
{
	gpio_set_value(gpio, data);
}

/**
 * dtvtuner_pm_dev_tuner_power_on() - power on dtvtuner pm device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static struct regulator *reg_cfg;
static int dtvtuner_pm_dev_tuner_power_on(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	int ret = 0;
	struct dtvtuner_pm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	// power on
	reg_cfg = regulator_get(NULL, pfdata->regulator_id);
	if (IS_ERR(reg_cfg)) {
		ret = PTR_ERR(reg_cfg);
		dev_err(&drvdata->pdev->dev, "regulator_get error[%d]\n", ret);
		goto out;
	}

	ret = regulator_set_voltage(reg_cfg, 1200000, 1400000);
	if (ret) {
		dev_err(&drvdata->pdev->dev, "regulator_set_voltage error ret[%d]\n", ret);
		goto regulator_free;
	}

	ret = regulator_set_optimum_mode(reg_cfg, 150000);
	if (ret < 0) {
		dev_err(&drvdata->pdev->dev, "regulator_set_optimum_mode error ret[%d]\n", ret);
		goto regulator_free;
	}

	ret = regulator_enable(reg_cfg);
	if (ret) {
		dev_err(&drvdata->pdev->dev, "regulator_enable error ret[%d]\n", ret);
		goto regulator_free;
	}
	dev_info(&drvdata->pdev->dev, "PowerOn success\n");

	// reset HW(L)
	dtvtuner_pm_dev_gpio_put(pfdata->gpio_rst, 0);
	dev_info(&drvdata->pdev->dev, "HW reset(L)\n");

	msleep(D_DTVTUNER_DEVICE_RST_WAITTIME);


return 0;

regulator_free:
	regulator_put(reg_cfg);
	reg_cfg=NULL;
out:
	return ret;
}

/**
 * dtvtuner_pm_dev_tuner_HW_reset() - dtvtuner HW reset.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_tuner_HW_reset(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	struct dtvtuner_pm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	// reset HW(H)
	dtvtuner_pm_dev_gpio_put(pfdata->gpio_rst, 1);
	dev_info(&drvdata->pdev->dev, "HW reset(H)\n");
	msleep(D_DTVTUNER_DEVICE_RST_WAITTIME);

	return 0;
}

/**
 * dtvtuner_pm_dev_tuner_power_off() - power off dtvtuner pm device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_tuner_power_off(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	int ret = 0;
	struct dtvtuner_pm_platform_data *pfdata =
				drvdata->pdev->dev.platform_data;

	msleep(D_DTVTUNER_DEVICE_RST_WAITTIME);
	dtvtuner_pm_dev_gpio_put(pfdata->gpio_rst, 0);
	msleep(D_DTVTUNER_DEVICE_RST_WAITTIME);

	if (!IS_ERR_OR_NULL(reg_cfg)) {
		dev_info(&drvdata->pdev->dev, "regulator_disable call\n");
		ret = regulator_disable(reg_cfg);
		if (ret) {
			dev_err(&drvdata->pdev->dev, "regulator_disable error ret[%d]\n", ret);
		}
		ret = regulator_set_optimum_mode(reg_cfg, 10000);
		if (ret < 0) {
			dev_err(&drvdata->pdev->dev, "regulator_set_optimum_mode error ret[%d]\n", ret);
		}
		regulator_put(reg_cfg);
		reg_cfg = NULL;
	}

	dev_info(&drvdata->pdev->dev, "PowerOff\n");

	return 0;
}

/**
 * dtvtuner_pm_driver_powerctrl_store() - The ioctl handler of power off
 * @dev:	[IN]	device data
 * @attr:	[IN]	device attribute data
 * @buf:	[IN]	data from user side.
 * @count:	[IN]	amount of data from user side.
 *
 * Return codes
 *   0 - Success
 *   -EINVAL - Failure
 */
static ssize_t dtvtuner_pm_driver_powerctrl_store(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count
)
{
	struct dtvtuner_pm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long value;

	if (strict_strtoul(buf, 0, &value)) {
		dev_err(&drvdata->pdev->dev, "Invalid value for power_ctrl\n");
		goto err_strict_strtoul;
	}

	if (value == D_DTVTUNER_POWER_ON)
		dtvtuner_pm_dev_tuner_power_on(drvdata);
	else if (value == D_DTVTUNER_POWER_OFF)
		dtvtuner_pm_dev_tuner_power_off(drvdata);
	else if (value == D_DTVTUNER_HW_RESET)
		dtvtuner_pm_dev_tuner_HW_reset(drvdata);

	return count;

err_strict_strtoul:
	return 0;
}

static DEVICE_ATTR(power_ctrl, S_IWUSR | S_IRUSR,
			NULL, dtvtuner_pm_driver_powerctrl_store);

/**
 * dtvtuner_pm_probe() - Prove dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 *
 * Return codes
 *   0 - Success
 *   -ENODEV - fail to probe
 *   -EINVAL - no platform data
 *   -ENOMEM - no memory for driver data
 */
static int dtvtuner_pm_probe(struct platform_device *pdev)
{
	int	ret = -ENODEV;
	struct dtvtuner_pm_platform_data *pfdata;
	struct dtvtuner_pm_drvdata *drvdata;

	pfdata = pdev->dev.platform_data;

	if (!pfdata) {
		dev_err(&pdev->dev, "No platform data.\n");
		ret = -EINVAL;
		goto err_get_platform_data;
	}

	drvdata = kzalloc(sizeof(struct dtvtuner_pm_drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "No enough memory for dtvtuner_pm\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	ret = dtvtuner_pm_dev_init(drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Fail to initialize\n");
		goto err_gpio_init;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_power_ctrl);
	if (ret) {
		dev_err(&pdev->dev, "Fail to initialize\n");
		goto err_device_create_file;
	}

	return 0;

err_device_create_file:
	dtvtuner_pm_dev_finalize(drvdata);
err_gpio_init:
	kfree(drvdata);
err_alloc_data:
err_get_platform_data:

	return ret;
}

/**
 * dtvtuner_pm_remove() - Remove dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 */
static int __devexit dtvtuner_pm_remove(struct platform_device *pdev)
{
	struct dtvtuner_pm_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	device_remove_file(&pdev->dev, &dev_attr_power_ctrl);
	dtvtuner_pm_dev_finalize(drvdata);
	kfree(drvdata);

	return 0;
}

#ifdef CONFIG_SUSPEND
/**
 * dtvtuner_pm_suspend() - Suspend dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 */
static int dtvtuner_pm_suspend(struct device *dev)
{
	return 0;
}

/**
 * dtvtuner_pm_resume() - Resume dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 */
static int dtvtuner_pm_resume(struct device *dev)
{
	return 0;
}
#else
#define dtvtuner_pm_suspend	NULL
#define dtvtuner_pm_resume	NULL
#endif

static const struct dev_pm_ops dtvtuner_pm_ops = {
	.suspend	= dtvtuner_pm_suspend,
	.resume		= dtvtuner_pm_resume,
};

/**
 * brief Platform driver data structure of dtvtuner driver
 */
static struct platform_driver dtvtuner_pm_driver = {
	.probe		= dtvtuner_pm_probe,
	.remove		= __exit_p(dtvtuner_pm_remove),
	.driver		= {
		.name		= D_DTVTUNER_PM_DRIVER_NAME,
		.owner		= THIS_MODULE,
		.pm = &dtvtuner_pm_ops,
	},
};

/**
 * dtvtuner_pm_driver_init() - The module init handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static int __init dtvtuner_pm_driver_init(void)
{
	return platform_driver_register(&dtvtuner_pm_driver);
}

/**
 * dtvtuner_pm_driver_exit() - The module exit handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static void __exit dtvtuner_pm_driver_exit(void)
{
	platform_driver_unregister(&dtvtuner_pm_driver);
}

module_init(dtvtuner_pm_driver_init);
module_exit(dtvtuner_pm_driver_exit);

MODULE_DESCRIPTION("dtvtuner_pm driver");
MODULE_LICENSE("GPL v2");
