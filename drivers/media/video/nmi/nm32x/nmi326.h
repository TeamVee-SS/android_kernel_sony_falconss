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

#ifndef __NMI_HW_H__
#define __NMI_HW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/sched.h>

#define ISDBT_DEV_NAME "isdbt"
#define ISDBT_DEV_MAJOR 227
#define ISDBT_DEV_MINOR 0

#define DTV_IRQ_DEINIT 0
#define DTV_IRQ_INIT 1
#define DTV_IRQ_SET 2

typedef struct {
	long index;
	void **hInit;
	void *hI2C;
	spinlock_t isr_lock;
	struct fasync_struct *async_queue;
	unsigned char *rwBuf;
} ISDBT_OPEN_INFO_T;

#define MAX_OPEN_NUM 8

#define IOCTL_MAGIC 't'

#define IOCTL_MAXNR 9

#define IOCTL_ISDBT_POWER_ON _IO(IOCTL_MAGIC, 0)
#define IOCTL_ISDBT_POWER_OFF _IO(IOCTL_MAGIC, 1)
#define IOCTL_ISDBT_RST_DN _IO(IOCTL_MAGIC, 2)
#define IOCTL_ISDBT_RST_UP _IO(IOCTL_MAGIC, 3)

#define IOCTL_ISDBT_INTERRUPT_REGISTER _IO(IOCTL_MAGIC, 4)
#define IOCTL_ISDBT_INTERRUPT_UNREGISTER _IO(IOCTL_MAGIC, 5)
#define IOCTL_ISDBT_INTERRUPT_ENABLE _IO(IOCTL_MAGIC, 6)
#define IOCTL_ISDBT_INTERRUPT_DISABLE _IO(IOCTL_MAGIC, 7)
#define IOCTL_ISDBT_INTERRUPT_DONE _IO(IOCTL_MAGIC, 8)

unsigned long nmi326_spi_read_chip_id(void);
int nmi326_spi_read(u8 *buf, size_t len);
int nmi326_spi_write(u8 *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif // __NMI_HW_H__
