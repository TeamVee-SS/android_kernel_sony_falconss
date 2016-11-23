/* NM326 host Interface
 *
 * Copyright (C) 2012 NMI Inc
 * Copyright (C) 2016 Caio Oliveira <caiooliveirafarias0@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <mach/board.h>

#include "nmi326.h"

static struct class *isdbt_class;

static wait_queue_head_t isdbt_irq_waitqueue;
static char g_bCatchIrq = 0;
static int g_irq_status = DTV_IRQ_DEINIT;

static int nmi_iron_irq = 0;
static int chip_open = 0;

static int NMI_POWER_PIN = 0;
static int NMI_RESET_PIN = 0;
static int NMI_IRQN_PIN = 0;
static int HW_DP_DET_PIN = 35;

#define NM32X_IRQ_NAME "NM32X_IRQ"

#define SPI_RW_BUF (188 * 50 * 2)

static int dtv_status = 0;

void isdbt_gpio_init(void)
{
	int ret = 0;

	gpio_request(HW_DP_DET_PIN, "hw_det_pin");

	if (!gpio_get_value(HW_DP_DET_PIN)) {
		pr_debug("%s: HW PDP2\n", __func__);
		NMI_IRQN_PIN = 90;
		NMI_POWER_PIN = 92;
		NMI_RESET_PIN = 91;
	} else {
		pr_debug("%s: HW DP\n", __func__);
		NMI_IRQN_PIN = 101;
		NMI_POWER_PIN = 79;
		NMI_RESET_PIN = 91;
	}

	// interrupt
	ret = gpio_request_one(NMI_IRQN_PIN, GPIOF_IN, "nmi_int");
	if (ret)
		return;

	// PWR Enable
	ret = gpio_request_one(NMI_POWER_PIN, GPIOF_OUT_INIT_LOW, "nmi_pwr");
	if (ret)
		goto err_pwren;

	// n_Reset
	ret = gpio_request_one(NMI_RESET_PIN, GPIOF_OUT_INIT_HIGH, "nmi_rst");
	if (ret)
		goto err_reset;

	nmi_iron_irq = gpio_to_irq(NMI_IRQN_PIN);

err_reset:
	gpio_free(NMI_POWER_PIN);
err_pwren:
	gpio_free(NMI_IRQN_PIN);
}

void isdbt_gpio_power_on(void) { gpio_set_value(NMI_POWER_PIN, 1); }
void isdbt_gpio_power_off(void) { gpio_set_value(NMI_POWER_PIN, 0); }
void isdbt_gpio_reset_up(void) { gpio_set_value(NMI_RESET_PIN, 1); }
void isdbt_gpio_reset_down(void) { gpio_set_value(NMI_RESET_PIN, 0); }

static irqreturn_t isdbt_irq_handler(int irq, void *dev_id)
{
	ISDBT_OPEN_INFO_T *isdbt_dev = (ISDBT_OPEN_INFO_T *)(dev_id);
	unsigned long flags;

	spin_lock_irqsave(&isdbt_dev->isr_lock, flags);
	disable_irq_nosync(nmi_iron_irq);
	g_irq_status = DTV_IRQ_INIT;
	g_bCatchIrq = 1;
	wake_up(&isdbt_irq_waitqueue);
	spin_unlock_irqrestore(&isdbt_dev->isr_lock, flags);

	return IRQ_HANDLED;
}

void isdbt_gpio_interrupt_register(ISDBT_OPEN_INFO_T *pdev)
{
	int ret = 1;

	// irq register
	ret = request_irq(nmi_iron_irq, isdbt_irq_handler, IRQF_TRIGGER_LOW,
			  NM32X_IRQ_NAME, pdev);
	if (ret) {
		pr_err("%s: request_irq failed [%d]\n", __func__, ret);
	}
}

void isdbt_gpio_interrupt_unregister(ISDBT_OPEN_INFO_T *pdev)
{
	free_irq(nmi_iron_irq, pdev);
}

void isdbt_gpio_interrupt_enable(void) { enable_irq(nmi_iron_irq); }
void isdbt_gpio_interrupt_disable(void) { disable_irq_nosync(nmi_iron_irq); }

static void isdbt_read_gpio(void)
{
	int val = -1;
	val = gpio_get_value(NMI_IRQN_PIN);
}

static unsigned int isdbt_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &isdbt_irq_waitqueue, wait);

	if (g_irq_status == DTV_IRQ_DEINIT) {
		isdbt_read_gpio();
		return mask;
	}

	if (g_bCatchIrq == 1) {
		mask |= (POLLIN | POLLRDNORM);
		g_bCatchIrq = 0;
	}

	return mask;
}

static int isdbt_open(struct inode *inode, struct file *filp)
{
	ISDBT_OPEN_INFO_T *pdev = NULL;

	pdev =
	    (ISDBT_OPEN_INFO_T *)kmalloc(sizeof(ISDBT_OPEN_INFO_T), GFP_KERNEL);
	if (pdev == NULL) {
		pr_err("%s: kmalloc failed\n", __func__);
		return -1;
	}

	memset(pdev, 0x00, sizeof(ISDBT_OPEN_INFO_T));
	g_bCatchIrq = 0;

	filp->private_data = pdev;

	pdev->rwBuf = kmalloc(SPI_RW_BUF, GFP_KERNEL);
	if (pdev->rwBuf == NULL) {
		pr_err("%s: kmalloc failed\n", __func__);
		return -1;
	}

	spin_lock_init(&pdev->isr_lock);
	init_waitqueue_head(&isdbt_irq_waitqueue);

	chip_open = 1;

	return 0;
}

static int isdbt_release(struct inode *inode, struct file *filp)
{
	ISDBT_OPEN_INFO_T *pdev = (ISDBT_OPEN_INFO_T *)(filp->private_data);

	chip_open = 0;

	if (g_irq_status != DTV_IRQ_DEINIT) {
		g_irq_status = DTV_IRQ_DEINIT;
		isdbt_gpio_interrupt_unregister(pdev);
	}

	dtv_status = 0;
	g_bCatchIrq = 0;

	kfree(pdev->rwBuf);
	kfree((void *)pdev);

	return 0;
}

ssize_t isdbt_read(struct file *filp, char __user *buf, size_t count,
		   loff_t *f_pos)
{
	int rv1 = 0;
	int rv2 = 0;
	int blk_cnt = count / 1024;
	int remain = count % 1024;
	ISDBT_OPEN_INFO_T *pdev = (ISDBT_OPEN_INFO_T *)(filp->private_data);

	if (blk_cnt) {
		rv1 = nmi326_spi_read(pdev->rwBuf, blk_cnt * 1024);
		if (rv1) {
			pr_err("%s: nmi326_spi_read failed [%d]\n", __func__,
			       rv1);
			return rv1;
		}
	}

	if (remain) {
		rv2 = nmi326_spi_read(&pdev->rwBuf[rv1], remain);
		if (rv2) {
			pr_err("%s: nmi326_spi_read failed [%d]\n", __func__,
			       rv2);
			return rv1 + rv2;
		}
	}

	if (copy_to_user(buf, pdev->rwBuf, count))
		return -1;

	return rv1 + rv2;
}

static ssize_t isdbt_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *f_pos)
{
	int rv = 0;
	ISDBT_OPEN_INFO_T *pdev = (ISDBT_OPEN_INFO_T *)(filp->private_data);

	/* move data from user area to kernel area */
	if (copy_from_user(pdev->rwBuf, buf, count))
		return -1;

	/* write data to SPI Controller */
	rv = nmi326_spi_write(pdev->rwBuf, count);
	if (rv) {
		pr_err("%s: nmi326_spi_write failed [%d]\n", __func__, rv);
		return rv;
	}

	return rv;
}

static long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long res = 1;
	ISDBT_OPEN_INFO_T *pdev = (ISDBT_OPEN_INFO_T *)(filp->private_data);

	int err = 0, size = 0;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	size = _IOC_SIZE(cmd);

	if (size) {
		if (_IOC_DIR(cmd) & _IOC_READ)
			err = access_ok(VERIFY_WRITE, (void *)arg, size);
		else if (_IOC_DIR(cmd) & _IOC_WRITE)
			err = access_ok(VERIFY_READ, (void *)arg, size);
		if (!err) {
			pr_err("%s: Wrong argument! cmd(0x%08x) _IOC_NR(%d) "
			       "_IOC_TYPE(0x%x) _IOC_SIZE(%d) "
			       "_IOC_DIR(0x%x)\n",
			       __func__, cmd, _IOC_NR(cmd), _IOC_TYPE(cmd),
			       _IOC_SIZE(cmd), _IOC_DIR(cmd));
			return err;
		}
	}

	switch (cmd) {
	case IOCTL_ISDBT_POWER_ON:
		isdbt_gpio_power_on();
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_gpio_power_off();
		break;
	case IOCTL_ISDBT_RST_DN:
		isdbt_gpio_reset_down();
		break;
	case IOCTL_ISDBT_RST_UP:
		isdbt_gpio_reset_up();
		break;
	case IOCTL_ISDBT_INTERRUPT_REGISTER: {
		unsigned long flags;

		spin_lock_irqsave(&pdev->isr_lock, flags);

		if (g_irq_status == DTV_IRQ_DEINIT && dtv_status == 0) {
			g_bCatchIrq = 0;
			isdbt_gpio_interrupt_register(pdev);
			g_irq_status = DTV_IRQ_INIT;
			dtv_status = 1;
		}
		spin_unlock_irqrestore(&pdev->isr_lock, flags);
		break;
	}
	case IOCTL_ISDBT_INTERRUPT_UNREGISTER: {
		unsigned long flags;

		spin_lock_irqsave(&pdev->isr_lock, flags);

		if (dtv_status == 1) {
			isdbt_gpio_interrupt_unregister(pdev);
			dtv_status = 0;
		}
		spin_unlock_irqrestore(&pdev->isr_lock, flags);
		break;
	}
	case IOCTL_ISDBT_INTERRUPT_ENABLE: {
		unsigned long flags;

		spin_lock_irqsave(&pdev->isr_lock, flags);

		if (g_irq_status == DTV_IRQ_INIT) {
			isdbt_gpio_interrupt_enable();
			g_irq_status = DTV_IRQ_SET;
		}
		spin_unlock_irqrestore(&pdev->isr_lock, flags);
		break;
	}
	case IOCTL_ISDBT_INTERRUPT_DISABLE: {
		unsigned long flags;

		spin_lock_irqsave(&pdev->isr_lock, flags);

		if (g_irq_status == DTV_IRQ_SET) {
			g_bCatchIrq = 0;
			isdbt_gpio_interrupt_disable();
			g_irq_status = DTV_IRQ_INIT;
		}
		spin_unlock_irqrestore(&pdev->isr_lock, flags);
		break;
	}
	case IOCTL_ISDBT_INTERRUPT_DONE: {
		unsigned long flags;

		spin_lock_irqsave(&pdev->isr_lock, flags);

		if (g_irq_status == DTV_IRQ_INIT) {
			isdbt_gpio_interrupt_enable();
			g_irq_status = DTV_IRQ_SET;
		}
		spin_unlock_irqrestore(&pdev->isr_lock, flags);
		break;
	}
	default:
		res = 1;
		break;
	}

	return res;
}

static const struct file_operations isdbt_fops = {
    .owner = THIS_MODULE,
    .open = isdbt_open,
    .release = isdbt_release,
    .read = isdbt_read,
    .write = isdbt_write,
    .unlocked_ioctl = isdbt_ioctl,
    .poll = isdbt_poll,
};

static ssize_t dtv_chip_id_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	isdbt_gpio_power_on();
	msleep(30);
	isdbt_gpio_power_off();

	return 0;
}
static DEVICE_ATTR(chip_id, 0644, dtv_chip_id_read, NULL);

static struct attribute *dtv_attrs[] = {
    &dev_attr_chip_id.attr, NULL,
};
static struct attribute_group dtv_attr_group = {
    .attrs = dtv_attrs,
};

static int __devinit isdbt_probe(struct platform_device *pdev)
{
	int ret, rc = 0;
	struct device *isdbt_dev;

	// 1. register character device
	ret = register_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME, &isdbt_fops);
	if (ret)
		pr_err("%s: register_chrdev(ISDBT_DEV) failed\n", __func__);

	// 2. class create
	isdbt_class = class_create(THIS_MODULE, ISDBT_DEV_NAME);
	if (IS_ERR(isdbt_class)) {
		unregister_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME);
		class_destroy(isdbt_class);
		pr_err("%s: class create failed\n", __func__);

		return -EFAULT;
	}

	// 3. device create
	isdbt_dev = device_create(isdbt_class, NULL,
				  MKDEV(ISDBT_DEV_MAJOR, ISDBT_DEV_MINOR), NULL,
				  ISDBT_DEV_NAME);

	rc = sysfs_create_group(&isdbt_dev->kobj, &dtv_attr_group);
	if (IS_ERR(isdbt_dev)) {
		unregister_chrdev(ISDBT_DEV_MAJOR, ISDBT_DEV_NAME);
		class_destroy(isdbt_class);
		pr_err("%s: device create failed\n", __func__);

		return -EFAULT;
	}

	isdbt_gpio_init();

	return 0;
}

static int __devexit isdbt_remove(struct platform_device *pdev)
{
	gpio_free(NMI_RESET_PIN);
	gpio_free(NMI_POWER_PIN);
	gpio_free(NMI_IRQN_PIN);
	return 0;
}

static struct of_device_id nmi325_isdbt_of_match[] = {
    {
	.compatible = "nmi,nmi325_isdbt",
    },
    {},
};
MODULE_DEVICE_TABLE(of, nmi325_isdbt_of_match);

static struct platform_driver isdbt_driver = {
    .probe = isdbt_probe,
    .remove = __devexit_p(isdbt_remove),
    .driver = {
	.name = "isdbt",
	.owner = THIS_MODULE,
	.of_match_table = nmi325_isdbt_of_match,
    }};

static int __init isdbt_init(void)
{
	int result = 0;

	result = platform_driver_register(&isdbt_driver);
	if (result)
		return result;

	return 0;
}

static void __exit isdbt_exit(void)
{
	unregister_chrdev(ISDBT_DEV_MAJOR, "isdbt");
	device_destroy(isdbt_class, MKDEV(ISDBT_DEV_MAJOR, ISDBT_DEV_MINOR));
	class_destroy(isdbt_class);

	platform_driver_unregister(&isdbt_driver);
}

module_init(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");
