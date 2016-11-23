/* NM326 SPI Interface
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

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/map.h>
#include <asm/page.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wait.h>

#include "nmi326.h"

static struct spi_device *nmi326_spi;

unsigned long check_dtv_chip_id(void) { return 0; }

int nmi326_spi_read(u8 *buf, size_t len)
{

	struct spi_message msg;
	struct spi_transfer transfer[2];
	unsigned char status = 0;
	int r_len;

	memset(&msg, 0, sizeof(msg));
	memset(transfer, 0, sizeof(transfer));

	spi_message_init(&msg);
	msg.spi = nmi326_spi;

	transfer[0].tx_buf = (unsigned char *)NULL;
	transfer[0].rx_buf = (unsigned char *)buf;
	transfer[0].len = len;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;

	spi_message_add_tail(&(transfer[0]), &msg);

	status = spi_sync(nmi326_spi, &msg);
	if (status == 0)
		r_len = len;
	else
		r_len = status;

	return r_len;
}

int nmi326_spi_write(u8 *buf, size_t len)
{

	struct spi_message msg;
	struct spi_transfer transfer[2];
	unsigned char status = 0;
	int w_len;

	memset(&msg, 0, sizeof(msg));
	memset(transfer, 0, sizeof(transfer));

	spi_message_init(&msg);
	msg.spi = nmi326_spi;

	transfer[0].tx_buf = (unsigned char *)buf;
	transfer[0].rx_buf = (unsigned char *)NULL;
	transfer[0].len = len;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;

	spi_message_add_tail(&(transfer[0]), &msg);

	status = spi_sync(nmi326_spi, &msg);
	if (status == 0)
		w_len = len;
	else
		w_len = status;

	return w_len;
}

static int __devinit nmi326_spi_probe(struct spi_device *spi)
{
	int retval = 0;

	nmi326_spi = spi;
	nmi326_spi->mode = (SPI_MODE_0);
	nmi326_spi->bits_per_word = 8;

	retval = spi_setup(nmi326_spi);
	if (retval) {
		pr_err("%s: spi_setup exited with [%d]\n", __func__, retval);
	}

	return 0;
}

static int __devexit nmi326_spi_remove(struct spi_device *spi) { return 0; }

static const struct of_device_id nmi_nmispi_dt_match[] = {
    {.compatible = "nmi,nmispi"}, {}};
MODULE_DEVICE_TABLE(of, nmi_nmispi_dt_match);

static struct spi_driver nmi326_spi_driver = {
    .driver =
	{
	    .name = "nmispi",
	    .bus = &spi_bus_type,
	    .owner = THIS_MODULE,
	    .of_match_table = nmi_nmispi_dt_match,
	},
    .probe = nmi326_spi_probe,
    .remove = __devexit_p(nmi326_spi_remove),
};

static int __init nmi326_spi_init(void)
{
	int retval = 0;

	retval = spi_register_driver(&nmi326_spi_driver);
	if (retval) {
		pr_err("%s: spi_register_driver exited with [%d]\n", __func__,
		       retval);
		return retval;
	}

	return 0;
}

static void __exit nmi326_spi_exit(void)
{
	spi_unregister_driver(&nmi326_spi_driver);
}

module_init(nmi326_spi_init);
module_exit(nmi326_spi_exit);
