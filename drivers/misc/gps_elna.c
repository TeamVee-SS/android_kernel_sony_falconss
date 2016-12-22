/*
 * Copyright (C) 2013 Arima Communications.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gps_elna.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define DRIVER_AUTHOR "Murphy Ou"
#define DRIVER_NAME GPS_ELNA_DEVICE_NAME

// 1: enable, 0: disable
#define GPS_ELNA_REGULATOR_L16_ENABLE 0 // PDP1, PDP2 uses L16
#define GPS_ELNA_REGULATOR_L19_ENABLE 1 // DP uses L19

/* LDO16 voltage @3.0 V */
#define MXT_ELNA_L16_VTG_MIN_UV 3000000
#define MXT_ELNA_L16_VTG_MAX_UV 3000000
#define MXT_ELNA_L16_ACTIVE_LOAD_UA 600000 // 600 mA
#define MXT_ELNA_L16_LPM_LOAD_UA 10

/* LDO19 voltage @2.85 V */
#define MXT_ELNA_L19_VTG_MIN_UV 2850000
#define MXT_ELNA_L19_VTG_MAX_UV 2850000
#define MXT_ELNA_L19_ACTIVE_LOAD_UA 600000 // 600 mA
#define MXT_ELNA_L19_LPM_LOAD_UA 10

#define MAX_BUFFER_SIZE 512

typedef struct GPS_ELNA_CTRL_BLOCK_Tag {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct miscdevice gps_elna_device;
	spinlock_t irq_enabled_lock;
	struct device *dev;
	unsigned int gpio_enable;
	enum of_gpio_flags gpio_enable_flag;
	struct regulator *vcc_elna_l16;
	struct regulator *vcc_elna_l19;

} GPS_ELNA_CTRL_BLOCK_T;

static int gps_elna_power_on_l16(GPS_ELNA_CTRL_BLOCK_T *data, bool on);
static int gps_elna_power_on_l19(GPS_ELNA_CTRL_BLOCK_T *data, bool on);

static ssize_t gps_elna_dev_read(struct file *filp, char __user *buf,
				 size_t count, loff_t *offset)
{
	struct miscdevice *m = filp->private_data;
	GPS_ELNA_CTRL_BLOCK_T *gps_elna_ctrl_block =
	    container_of(m, GPS_ELNA_CTRL_BLOCK_T, gps_elna_device);
	char tmp[MAX_BUFFER_SIZE];
	int ret, i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s: reading %zu bytes.\n", __func__, count);

	mutex_lock(&gps_elna_ctrl_block->read_mutex);
	ret = count;
	mutex_unlock(&gps_elna_ctrl_block->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__,
		       ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s: IFD->PC:", __func__);
	for (i = 0; i < ret; i++) {
		pr_debug(" %02X", tmp[i]);
	}
	pr_debug("\n");

	return ret;
}

static ssize_t gps_elna_dev_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *offset)
{
	char tmp[MAX_BUFFER_SIZE];
	int ret, i;

	if (count > MAX_BUFFER_SIZE) {
		count = MAX_BUFFER_SIZE;
	}
	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
	ret = count;
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	pr_debug("%s: IFD->PC:", __func__);
	for (i = 0; i < count; i++) {
		pr_debug(" %02X", tmp[i]);
	}
	pr_debug("\n");

	return ret;
}

static int gps_elna_dev_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static long gps_elna_dev_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	struct miscdevice *m = filp->private_data;
	GPS_ELNA_CTRL_BLOCK_T *gps_elna_ctrl_block =
	    container_of(m, GPS_ELNA_CTRL_BLOCK_T, gps_elna_device);

	switch (cmd) {
	case GPS_ELNA_SET_PWR: {
		if (arg == 1) {
			/* power on */
			pr_debug("[GPS_DBG] %s: power on\n", __func__);

			if (GPS_ELNA_REGULATOR_L16_ENABLE)
				gps_elna_power_on_l16(gps_elna_ctrl_block,
						      true);

			if (GPS_ELNA_REGULATOR_L19_ENABLE)
				gps_elna_power_on_l19(gps_elna_ctrl_block,
						      true);
		} else if (arg == 0) {
			/* power off */
			pr_debug("[GPS_DBG] %s: power off\n", __func__);

			if (GPS_ELNA_REGULATOR_L16_ENABLE)
				gps_elna_power_on_l16(gps_elna_ctrl_block,
						      false);

			if (GPS_ELNA_REGULATOR_L19_ENABLE)
				gps_elna_power_on_l19(gps_elna_ctrl_block,
						      false);
		} else {
			pr_err("%s: bad arg %u\n", __func__, (unsigned int)arg);
			return -EINVAL;
		}

		break;
	}
	default: {
		pr_err("%s: bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}
	}

	return 0;
}

static const struct file_operations gps_elna_dev_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .read = gps_elna_dev_read,
    .write = gps_elna_dev_write,
    .open = gps_elna_dev_open,
    .unlocked_ioctl = gps_elna_dev_ioctl,
};

static int gps_elna_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0)
		   ? regulator_set_optimum_mode(reg, load_uA)
		   : 0;
}

static int gps_elna_power_on_l16(GPS_ELNA_CTRL_BLOCK_T *data, bool on)
{
	int rc;

	if (on == false) {
		goto power_off;
	}

	// L16
	rc = gps_elna_set_optimum_mode_check(data->vcc_elna_l16,
					     MXT_ELNA_L16_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(data->dev,
			"%s: Regulator vcc_elna_l16 set_opt failed rc=%d\n", __func__,
			rc);

		pr_debug("[GPS_DBG] %s: gps_elna_set_optimum_mode_check(), "
			 "vcc_elna_l16 - fail.\n",
			 __func__);

		return rc;
	}

	rc = regulator_enable(data->vcc_elna_l16);
	if (rc) {
		dev_err(data->dev,
			"%s: Regulator vcc_elna_l16 enable failed rc=%d\n",
			__func__, rc);

		pr_debug(
		    "[GPS_DBG] %s: regulator_enable(), vcc_elna_l16 - fail.\n",
		    __func__);

		goto error_reg_en_vcc_elna_l16;
	}

	return 0;

error_reg_en_vcc_elna_l16:
	gps_elna_set_optimum_mode_check(data->vcc_elna_l16, 0);

	return rc;

power_off:
	gps_elna_set_optimum_mode_check(data->vcc_elna_l16, 0);
	regulator_disable(data->vcc_elna_l16);

	return 0;
}

static int gps_elna_power_on_l19(GPS_ELNA_CTRL_BLOCK_T *data, bool on)
{
	int rc;

	if (on == false) {
		goto power_off;
	}

	// L19
	rc = gps_elna_set_optimum_mode_check(data->vcc_elna_l19,
					     MXT_ELNA_L19_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(data->dev,
			"%s: Regulator vcc_elna_l19 set_opt failed rc=%d\n",
			__func__, rc);

		pr_debug("[GPS_DBG] %s: gps_elna_set_optimum_mode_check(), "
			 "vcc_elna_l19 - fail.\n",
			 __func__);

		return rc;
	}

	rc = regulator_enable(data->vcc_elna_l19);
	if (rc) {
		dev_err(data->dev,
			"%s: Regulator vcc_elna_l19 enable failed rc=%d\n",
			__func__, rc);

		pr_debug(
		    "[GPS_DBG] %s: regulator_enable(), vcc_elna_l19 - fail.\n",
		    __func__);

		goto error_reg_en_vcc_elna_l19;
	}

	return 0;

error_reg_en_vcc_elna_l19:
	gps_elna_set_optimum_mode_check(data->vcc_elna_l19, 0);

	return rc;

power_off:
	gps_elna_set_optimum_mode_check(data->vcc_elna_l19, 0);
	regulator_disable(data->vcc_elna_l19);

	return 0;
}

static int gps_elna_regulator_configure_l16(GPS_ELNA_CTRL_BLOCK_T *data,
					    bool on)
{
	int rc = 0;

	if (on == false)
		goto hw_shutdown;

	// L16
	data->vcc_elna_l16 = regulator_get(data->dev, "vcc_elna_l16");
	if (IS_ERR(data->vcc_elna_l16)) {
		rc = PTR_ERR(data->vcc_elna_l16);
		dev_err(data->dev,
			"%s: Regulator get failed vcc_elna_l16 rc=%d\n",
			__func__, rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_elna_l16) > 0) {
		rc = regulator_set_voltage(data->vcc_elna_l16,
					   MXT_ELNA_L16_VTG_MIN_UV,
					   MXT_ELNA_L16_VTG_MAX_UV);
		if (rc) {
			dev_err(
			    data->dev,
			    "%s: regulator set_vtg vcc_elna_l16 failed rc=%d\n",
			    __func__, rc);
			goto error_set_vtg_vcc_elna_l16;
		}
	}

	return 0;

error_set_vtg_vcc_elna_l16:
	regulator_put(data->vcc_elna_l16);

	return rc;

hw_shutdown:
	if (regulator_count_voltages(data->vcc_elna_l16) > 0)
		regulator_set_voltage(data->vcc_elna_l16, 0,
				      MXT_ELNA_L16_VTG_MAX_UV);

	regulator_put(data->vcc_elna_l16);

	return 0;
}

static int gps_elna_regulator_configure_l19(GPS_ELNA_CTRL_BLOCK_T *data,
					    bool on)
{
	int rc = 0;

	if (on == false)
		goto hw_shutdown;

	// L19
	data->vcc_elna_l19 = regulator_get(data->dev, "vcc_elna_l19");
	if (IS_ERR(data->vcc_elna_l19)) {
		rc = PTR_ERR(data->vcc_elna_l19);
		dev_err(data->dev,
			"%s: Regulator get failed vcc_elna_l19 rc=%d\n",
			__func__, rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_elna_l19) > 0) {
		rc = regulator_set_voltage(data->vcc_elna_l19,
					   MXT_ELNA_L19_VTG_MIN_UV,
					   MXT_ELNA_L19_VTG_MAX_UV);
		if (rc) {
			dev_err(
			    data->dev,
			    "%s: regulator set_vtg vcc_elna_l19 failed rc=%d\n",
			    __func__, rc);
			goto error_set_vtg_vcc_elna_l19;
		}
	}

	return 0;

error_set_vtg_vcc_elna_l19:
	regulator_put(data->vcc_elna_l19);

	return rc;

hw_shutdown:
	if (regulator_count_voltages(data->vcc_elna_l19) > 0)
		regulator_set_voltage(data->vcc_elna_l19, 0,
				      MXT_ELNA_L19_VTG_MAX_UV);

	regulator_put(data->vcc_elna_l19);

	return 0;
}

static int gps_elna_dts_parsing(GPS_ELNA_CTRL_BLOCK_T *data)
{
	struct device_node *np = NULL;

	if (!data)
		return -1;
	if (!data->dev)
		return -1;
	if (!(np = data->dev->of_node))
		return -1;

	data->gpio_enable = of_get_named_gpio_flags(np, "elna,enable-gpio", 0,
						    &(data->gpio_enable_flag));

	pr_debug("[GPS_DBG] %s: gpio_enable = %d, gpio_enable_flag = %d\n",
		 __func__, data->gpio_enable, (int)(data->gpio_enable_flag));

	if (GPS_ELNA_REGULATOR_L16_ENABLE)
		gps_elna_regulator_configure_l16(data, true);

	if (GPS_ELNA_REGULATOR_L19_ENABLE)
		gps_elna_regulator_configure_l19(data, true);

	return 0;
}

/*
 * module load/unload record keeping
 */
static int __devinit gps_elna_probe(struct platform_device *pdev)
{
	int ret = 0;
	GPS_ELNA_CTRL_BLOCK_T *gps_elna_ctrl_block = NULL;

	gps_elna_ctrl_block =
	    kzalloc(sizeof(GPS_ELNA_CTRL_BLOCK_T), GFP_KERNEL);
	if (gps_elna_ctrl_block == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}

	memset(gps_elna_ctrl_block, 0x00, sizeof(GPS_ELNA_CTRL_BLOCK_T));

	gps_elna_ctrl_block->dev = &(pdev->dev);
	platform_set_drvdata(pdev, gps_elna_ctrl_block);

	if (gps_elna_dts_parsing(gps_elna_ctrl_block) != 0) {
		ret = -EINVAL;
		goto err_dts_parsing;
	}

	/* init mutex and queues */
	init_waitqueue_head(&gps_elna_ctrl_block->read_wq);
	mutex_init(&gps_elna_ctrl_block->read_mutex);
	spin_lock_init(&gps_elna_ctrl_block->irq_enabled_lock);

	gps_elna_ctrl_block->gps_elna_device.minor = MISC_DYNAMIC_MINOR;
	gps_elna_ctrl_block->gps_elna_device.name = GPS_ELNA_DEVICE_NAME;
	gps_elna_ctrl_block->gps_elna_device.fops = &gps_elna_dev_fops;

	ret = misc_register(&gps_elna_ctrl_block->gps_elna_device);
	if (ret) {
		pr_err("[GPS_DBG] %s: misc_register failed\n", __func__);
		ret = -EINVAL;
		goto err_misc_register;
	}

	pr_info("[GPS_DBG] %s: successfully probed\n", __func__);

	return 0;

err_misc_register:
	mutex_destroy(&gps_elna_ctrl_block->read_mutex);
err_dts_parsing:
	kfree(gps_elna_ctrl_block);
err_exit:
	pr_err("[GPS_DBG] %s: error\n", __func__);

	return ret;
}

static const struct of_device_id gps_elna_ids[] = {
    {.compatible = "arima,gps_elna"}, {}};

static struct platform_driver gps_elna_platform_driver = {
    .driver =
	{
	    .name = DRIVER_NAME,
	    .owner = THIS_MODULE,
	    .of_match_table = gps_elna_ids,
	},
    .probe = gps_elna_probe,
};

static int __init gps_elna_dev_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&gps_elna_platform_driver);
	if (ret) {
		pr_err("[GPS_DBG] %s: failed to load\n", __func__);
	}

	return ret;
}

static void __exit gps_elna_dev_exit(void)
{
	pr_info("[GPS_DBG] %s: Unloading GPS eLNA driver\n", __func__);
	platform_driver_unregister(&gps_elna_platform_driver);
}

module_init(gps_elna_dev_init);
module_exit(gps_elna_dev_exit);

MODULE_AUTHOR("Murphy Ou");
MODULE_DESCRIPTION("GPS eLNA driver");
MODULE_LICENSE("GPL");
