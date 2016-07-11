/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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

//#define ELAN_BUFFER_MODE
//#define ELAN_PROTOCOL		/* This is to support normal packet format */
//#define ELAN_BUTTON
//#define RE_CALIBRATION	/* This was designed for ektf3k serial. */
//#define ELAN_2WIREICE
//#define ELAN_POWER_SOURCE

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/input/ektf2k.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define ELAN_VTG_MIN_UV		2850000
#define ELAN_VTG_MAX_UV		3300000
#define ELAN_ACTIVE_LOAD_UA	15000
#define ELAN_LPM_LOAD_UA	10
#define ELAN_VTG_DIG_MIN_UV	1800000
#define ELAN_VTG_DIG_MAX_UV	1800000
#define ELAN_ACTIVE_LOAD_DIG_UA	10000
#define ELAN_LPM_LOAD_DIG_UA	10
#define ELAN_I2C_VTG_MIN_UV	1800000
#define ELAN_I2C_VTG_MAX_UV	1800000
#define ELAN_I2C_LOAD_UA	10000
#define ELAN_I2C_LPM_LOAD_UA	10
#define PACKET_SIZE		8	/* support 2 fingers packet */
//#define PACKET_SIZE		24	/* support 5 fingers packet */
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL	1
#define PWR_STATE_MASK		BIT(3)
#define CMD_S_PKT		0x52
#define CMD_R_PKT		0x53
#define CMD_W_PKT		0x54
#define HELLO_PKT		0x55
#define TWO_FINGERS_PKT		0x5A
#define FIVE_FINGERS_PKT	0x5D
#define MTK_FINGERS_PKT		0x6D
#define TEN_FINGERS_PKT		0x62
#define BUFFER_PKT		0x63
#define BUFFER55_PKT		0x66
#define RESET_PKT		0x77
#define CALIB_PKT		0x66
#define SYSTEM_RESET_PIN_SR	0
#define PAGERETRY		30
#define IAPRESTART		5
#define ELAN_IOCTLID		0xD0
#define CUSTOMER_IOCTLID	0xA0

#define IOCTL_I2C_SLAVE		_IOW(ELAN_IOCTLID, 1, int)
#define IOCTL_MAJOR_FW_VER	_IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER	_IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET		_IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK	_IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE	_IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER		_IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION	_IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION	_IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID		_IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE	_IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK	_IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT		_IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME		_IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK	_IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK	_IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE		_IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER		_IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE		_IOR(ELAN_IOCTLID, 19, int)
#define IOCTL_CIRCUIT_CHECK	_IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE	_IOR(CUSTOMER_IOCTLID, 2, int)

uint8_t RECOVERY = 0x00;
int FW_VERSION = 0x00;
int X_RESOLUTION = 576;
int Y_RESOLUTION = 960;

uint8_t chip_type = 0x00;

int LCM_X_RESOLUTION = 480;
int LCM_Y_RESOLUTION = 800;

int FW_ID = 0x00;
int work_lock = 0x00;
int power_lock = 0x00;
int circuit_ver = 0x01;
int file_fops_addr = 0x15;
int button_state = 0;

static int touch_panel_type;
static unsigned short chip_reset_flag = 0;

uint8_t ic_status = 0x00;	// 0:OK, 1:master fail, 2:slave fail
int update_progree = 0;
uint8_t I2C_DATA[3] = {0x15, 0x20, 0x21};	/* I2C devices address */
int is_OldBootCode = 0;		// 0:new, 1:old

static unsigned long chip_mode_set = 0;
static unsigned long talking_mode_set = 0;

/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data_Truly_2127[] = {
#include "Truly_E1_V5507_BC2104_20140121.i"
};

static uint8_t file_fw_data_Eely_2127[] = {
#include "Eely_E1_V5507_BC2104_20140121.i"
};

static uint8_t file_fw_data_Truly[] = {
#include "Truly_IN1_V17_BC5568_20131119_RAM.i"
};

static uint8_t file_fw_data_Eely[] = {
#include "Eely_IN1_V17_BC5568_20131119_RAM.i"
};

static uint8_t *file_fw_data = NULL;
static uint8_t tp_sleep_status = 0;

enum {
	PageSize	= 132,
	PageNum		= 249,
	ACK_Fail	= 0x00,
	ACK_OK		= 0xAA,
	ACK_REWRITE	= 0x55,
};

enum {
	E_FD		= -1,
};

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct delayed_work check_work;
	struct early_suspend early_suspend;
	struct elan_ktf2k_i2c_platform_data *pdata;
	int intr_gpio;
	int reset_gpio;
	struct regulator *vcc_ana;
	struct regulator *vcc_i2c;
	struct mutex lock;
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	struct miscdevice firmware;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
};

static struct elan_ktf2k_ts_data *private_ts;
static int elan_ktf2k_ts_parse_dt(struct device*, struct elan_ktf2k_i2c_platform_data*);
static int elan_ktf2k_ts_set_mode_state(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_get_mode_state(struct i2c_client *client);
static int elan_ktf2k_ts_set_talking_state(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_get_talking_state(struct i2c_client *client);
static int elan_ktf2k_set_scan_mode(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_setup(struct i2c_client *client);
static int elan_ktf2k_ts_hw_reset(struct i2c_client *client);
static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct device *dev);
static int elan_ktf2k_ts_suspend(struct device *dev);
static int __elan_ktf2k_ts_poll(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int __hello_packet_handler(struct i2c_client *client);
static void elan_ktf2k_clear_ram(struct i2c_client *client);
int Update_FW_One(struct i2c_client *client, int recovery);

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE(struct i2c_client *client);
#endif

int elan_iap_open(struct inode *inode, struct file *filp)
{
	if (private_ts == NULL)
		pr_info("[ELAN|%s] private_ts is NULL", __func__);
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count,
		loff_t *offp)
{
	int ret;
	char *tmp;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	ret = i2c_master_send(private_ts->client, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);

	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(private_ts->client, tmp, count);

	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;
}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int __user *ip = (int __user *)arg;

	pr_info("[ELAN|%s] cmd value [%x]\n", __func__, cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:
		private_ts->client->addr = (int __user)arg;
		break;
	case IOCTL_MAJOR_FW_VER:
	case IOCTL_MINOR_FW_VER:
		break;
	case IOCTL_RESET:
		gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		msleep(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		msleep(5);
		break;
	case IOCTL_IAP_MODE_LOCK:
	cancel_delayed_work_sync(&private_ts->check_work);
		if (work_lock == 0) {
			work_lock = 1;
			disable_irq(private_ts->client->irq);
			flush_work(&private_ts->work);
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (work_lock == 1) {
			work_lock = 0;
			enable_irq(private_ts->client->irq);
		}
		schedule_delayed_work(&private_ts->check_work,
				msecs_to_jiffies(2500));
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return RECOVERY;
		break;
	case IOCTL_FW_VER:
		__fw_packet_handler(private_ts->client);
		return FW_VERSION;
		break;
	case IOCTL_X_RESOLUTION:
		__fw_packet_handler(private_ts->client);
		return X_RESOLUTION;
		break;
	case IOCTL_Y_RESOLUTION:
		__fw_packet_handler(private_ts->client);
		return Y_RESOLUTION;
		break;
	case IOCTL_FW_ID:
		__fw_packet_handler(private_ts->client);
		return FW_ID;
		break;
	case IOCTL_ROUGH_CALIBRATE:
		return elan_ktf2k_ts_rough_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->intr_gpio), ip);
		break;
	case IOCTL_RESUME:
		elan_ktf2k_ts_resume(&(private_ts->client->dev));
		break;
	case IOCTL_POWER_LOCK:
		power_lock = 1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock = 0;
		break;
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree = (int __user)arg;
		break;
	case IOCTL_FW_UPDATE:
		Update_FW_One(private_ts->client, 0);
		break;
#ifdef ELAN_2WIREICE
	case IOCTL_2WIREICE:
		elan_TWO_WIRE_ICE(private_ts->client);
		break;
#endif
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;
		break;
	default:
		pr_info("[ELAN|%s] Un-known IOCTL Command [%d]\n", __func__,
				cmd);
		break;
	}
	return 0;
}

struct file_operations elan_touch_fops = {
	.open =		elan_iap_open,
	.write =	elan_iap_write,
	.read =		elan_iap_read,
	.release =	elan_iap_release,
	.unlocked_ioctl =	elan_iap_ioctl,
};

int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;

	len = i2c_master_send(private_ts->client, isp_cmd, sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		pr_err("[ELAN|%s] Fail! [%d]\n", __func__, len);
		return -1;
	} else
		pr_info("[ELAN|%s] IAPMode write data successfully! "
				"[%2x, %2x, %2x, %2x]\n", __func__, isp_cmd[0],
				isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
	if (len != byte) {
		pr_err("[ELAN|%s] read page error [%d]\n", __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t * szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		pr_err("[ELAN|%s] write page error [%d]\n", __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	char buff[2] = {0};

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		pr_err("[ELAN|%s] read data error, write 50 times error [%d]\n",
				__func__, len);
		return -1;
	}

	pr_info("[ELAN|%s] %x, %x", __func__, buff[0], buff[1]);
	if (buff[0] == 0xaa)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent,page_tatol,percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i+1] = '\0';
	}

	page_tatol = page + 249 * (ic_num-j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;

	pr_info("[ELAN|%s] Progress [%s] | %d\n", __func__, str, percent);

	if (page == (249))
		pr_info("\n");
}
/*
* Reset (and Send normal_command ?)
* Get Hello Packet
*/

int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0,ic_num = 1;
	int iPage = 0, rewriteCnt = 0;
	int i = 0;
	uint8_t data;
	int restartCnt = 0;

	uint8_t recovery_buffer[8] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd_2227[] = {0x54, 0x00, 0x12, 0x34};
	uint8_t isp_cmd_2127[] = {0x45, 0x49, 0x41, 0x50};

	pr_debug("[ELAN|%s] [%d]\n", __func__, ic_num);

IAP_RESTART:
	data = I2C_DATA[0];
	pr_debug("[ELAN|%s] address [0x%x]\n", __func__, data);

	if (recovery != 0x80) {
		pr_info("[ELAN|%s] Firmware upgrade normal mode!\n", __func__);
		gpio_set_value(SYSTEM_RESET_PIN_SR,0);
		mdelay(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR,1);
		mdelay(5);
		if (chip_type == 0x22) {
			if (elan_ktf2k_ts_poll(private_ts->client) < 0)
				goto IAP_RESTART;

			res = i2c_master_recv(private_ts->client,
					recovery_buffer, 8);
			res = EnterISPMode(private_ts->client, isp_cmd_2227);
		} else
			res = EnterISPMode(private_ts->client, isp_cmd_2127);
	} else
		pr_info("[ELAN|%s] Firmware upgrade recovery mode!\n",
				__func__);

	res = i2c_master_recv(private_ts->client, recovery_buffer, 4);
	pr_info("[ELAN|%s] recovery byte data [%x, %x, %x, %x]\n", __func__,
			recovery_buffer[0], recovery_buffer[1],
			recovery_buffer[2], recovery_buffer[3]);

	if (recovery_buffer[1] != 0xaa && restartCnt < 5) {
		restartCnt++;
		goto IAP_RESTART;
	} else if (restartCnt >= 5) {
		pr_err("[ELAN|%s] IAP fail!\n", __func__);
		return 0;
	}

	// Send Dummy Byte
	pr_info("[ELAN] send one byte data [%x, %x]", private_ts->client->addr,
			data);

	res = i2c_master_send(private_ts->client, &data, sizeof(data));
	if (res != sizeof(data))
		pr_err("[ELAN] dummy error code [%d]\n", res);

	msleep(10);

	// Start IAP
	for (iPage = 1; iPage <= PageNum; iPage++) {
PAGE_REWRITE:
		for (byte_count = 1; byte_count <= 17; byte_count++) {
			if (byte_count != 17) {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 8;
				res = WritePage(szBuff, 8);
			} else {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		}

		if (iPage == 249 || iPage == 1)
			msleep(600);
		else
			msleep(50);

		res = GetAckData(private_ts->client);

		if (ACK_OK != res) {
			msleep(50);
			pr_err("[ELAN|%s] fail! [%d]\n", __func__, res);
			if (res == ACK_REWRITE) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY) {
					pr_err("[ELAN] ID 0x%02x %dth page "
							"ReWrite %d times "
							"fails!\n", data,
							iPage, PAGERETRY);
					return E_FD;
				} else {
					pr_info("[ELAN] %d page ReWrite %d "
							"times!\n", iPage,
							rewriteCnt);
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5) {
					pr_err("[ELAN] ID 0x%02x ReStart %d "
							"times fails!\n", data,
							IAPRESTART);
					return E_FD;
				} else {
					pr_info("[ELAN] %d page ReStart %d "
							"times!\n", iPage,
							restartCnt);
					goto IAP_RESTART;
				}
			}
		} else {
			pr_info("data : 0x%02x", data);
			rewriteCnt = 0;
			print_progress(iPage,ic_num,i);
		}

		msleep(10);
	}

	pr_info("[ELAN] Read Hello Packet data!\n");
	res = __hello_packet_handler(client);
	if (res > 0)
		pr_info("[ELAN] Update ALL Firmware successfully!\n");

	return 0;
}

#ifdef ELAN_2WIREICE
static uint8_t file_bin_data[] = {
#include "2wireice.i"
};

int write_ice_status = 0;
int shift_out_16(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbf,
			0xff};
	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int tms_reset(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0xee, 0xee, 0xea, 0xe0};
	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int mode_gen(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0xee, 0xee, 0xee, 0x20, 0xa6, 0xa6, 0x6a, 0xa6, 0x6a,
			0x6a, 0xa6, 0x6a, 0xe2, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
			0xaa, 0xaa, 0xaa, 0xe0};
	uint8_t buff_1[] = {0x2a, 0x6a, 0xa6, 0xa6, 0x6e};
	char mode_buff[2] = {0};
	res = i2c_master_send(client, buff, sizeof(buff));
	res = i2c_master_recv(client, mode_buff, sizeof(mode_buff));
	pr_info("[ELAN] mode_gen read [%x, %x]\n", mode_buff[0], mode_buff[1]);

	res = i2c_master_send(client, buff_1, sizeof(buff_1));
	return res;
}

int word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26,
			0x66};
	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int long_word_scan_out(struct i2c_client *client)
{
	int res;
	uint8_t buff[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
			0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26, 0x66};
	res = i2c_master_send(client, buff, sizeof(buff));
	return res;
}

int bit_manipulation(int TDI, int TMS, int TCK, int TDO,int TDI_1, int TMS_1,
		int TCK_1, int TDO_1)
{
	int res;
	res = ((TDI << 3 | TMS << 2 | TCK |TDO) << 4) | (TDI_1 << 3 |TMS_1 << 2
			| TCK_1 | TDO_1);
	return res;
}

int ins_write(struct i2c_client *client, uint8_t buf)
{
	int res = 0;
	int length = 13;
	uint8_t write_buf[7] = {0};
	int TDI_bit[13] = {0};
	int TMS_bit[13] = {0};
	int i = 0;
	uint8_t buf_rev = 0;
	int TDI = 0, TMS = 0, TCK = 0, TDO = 0;
	int bit_tdi, bit_tms;
	int len;

	for (i = 0; i < 8; i++)
		buf_rev = buf_rev | (((buf >> i) & 0x01) << (7-i));

	TDI = (0x7<<10) | buf_rev <<2 |0x00;
	TMS = 0x1007;
	TCK = 0x2;
	TDO = 0;

	for (len = 0; len <= length-1; len++) {
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[length-1-len] =bit_tdi;
		TMS_bit[length-1-len] = bit_tms;
		TDI = TDI >>1;
		TMS = TMS >>1;
	}

	for (len = 0; len <= length - 1; len = len+2) {
		if (len == length-1 && len%2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, 0, 0, 0, 0);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, TDI_bit[len+1], TMS_bit[len+1],
					TCK, TDO);

		write_buf[len/2] = res;
	}
	res = i2c_master_send(client, write_buf, sizeof(write_buf));
	return res;
}

int word_scan_in(struct i2c_client *client, uint16_t buf)
{
	int res = 0;
	uint8_t write_buf[10] = {0};
	int TDI_bit[20] = {0};
	int TMS_bit[20] = {0};
	int TDI = buf << 2 |0x00;
	int TMS = 0x7;
	int TCK = 0x2;
	int TDO = 0;

	int bit_tdi, bit_tms;
	int len;
	for (len = 0; len <= 19; len++) {
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;
		TDI_bit[19 - len] = bit_tdi;
		TMS_bit[19 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for (len = 0; len <= 19; len = len + 2) {
		if (len == 19 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, 0, 0, 0, 0);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, TDI_bit[len + 1], TMS_bit[len + 1],
					TCK, TDO);
		write_buf[len/2] = res;
	}

	res = i2c_master_send(client, write_buf, sizeof(write_buf));

	return res;
}

int long_word_scan_in(struct i2c_client *client, int buf_1, int buf_2)
{
	uint8_t write_buf[18] = {0};
	int TDI_bit[36] = {0};
	int TMS_bit[36] = {0};
	int TDI = buf_1 << 18 | buf_2 << 2 | 0x00;
	int TMS = 0x7;
	int TCK = 0x2;
	int TDO = 0;

	int bit_tdi, bit_tms;
	int len;
	int res = 0;

	for (len = 0; len <= 35; len++) {
		bit_tdi = TDI & 0x1;
		bit_tms = TMS & 0x1;

		TDI_bit[35 - len] = bit_tdi;
		TMS_bit[35 - len] = bit_tms;
		TDI = TDI >> 1;
		TMS = TMS >> 1;
	}

	for (len = 0; len <= 35; len = len + 2) {
		if (len == 35 && len % 2 == 0)
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, 0,0,0,0);
		else
			res = bit_manipulation(TDI_bit[len], TMS_bit[len], TCK,
					TDO, TDI_bit[len+1], TMS_bit[len+1],
					TCK, TDO);
		write_buf[len/2] = res;
	}
	res = i2c_master_send(client, write_buf, sizeof(write_buf));

	return res;
}

uint16_t trimtable[8] = {0};

int Read_SFR(struct i2c_client *client, int open)
{
	uint8_t voltage_recv[2] = {0};
	int count, ret;

	// 0
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0000);
	shift_out_16(client);

	mdelay(10);
	count = 0;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 1
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0001);
	shift_out_16(client);

	mdelay(10);
	count = 1;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 2
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0002);
	shift_out_16(client);

	mdelay(10);
	count = 2;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 3
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0003);
	shift_out_16(client);

	mdelay(10);
	count = 3;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 4
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0004);
	shift_out_16(client);

	mdelay(10);
	count = 4;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 5
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0005);
	shift_out_16(client);

	mdelay(10);
	count = 5;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 6
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0006);
	shift_out_16(client);

	mdelay(10);
	count=6;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	trimtable[count] = voltage_recv[0] << 8 | voltage_recv[1];

	// 7
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0020);
	long_word_scan_in(client, 0x007f, 0x4000);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9002);
	ins_write(client, 0x68);
	word_scan_in(client, 0x0007);
	shift_out_16(client);

	mdelay(10);
	count = 7;
	ret = i2c_master_recv(client, voltage_recv, sizeof(voltage_recv));
	pr_info("open= %d\n", open);
	if (open == 1)
		trimtable[count] = voltage_recv[0] << 8 | (voltage_recv[1] &
				0xbf);
	else
		trimtable[count]=voltage_recv[0]<<8 | (voltage_recv[1] | 0x40);

	pr_info("[ELAN] Open_High_Voltage recv -1 1word =%x %x, "
			"trimtable[%d]=%x\n", voltage_recv[0],voltage_recv[1],
			count, trimtable[count]);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

	return 0;
}

int Write_SFR(struct i2c_client *client)
{
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9001);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0000, trimtable[0]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0001, trimtable[1]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0002, trimtable[2]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0003, trimtable[3]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0004, trimtable[4]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0005, trimtable[5]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0006, trimtable[6]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0007, trimtable[7]);
	ins_write(client, 0xfd);
	word_scan_in(client, 0x7f);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7f, 0x8000);

	return 0;
}

int Enter_Mode(struct i2c_client *client)
{
	mode_gen(client);
	tms_reset(client);
	ins_write(client, 0xfc);	//system reset
	tms_reset(client);
	return 0;
}

int Open_High_Voltage(struct i2c_client *client, int open)
{
	Read_SFR(client, open);
	Write_SFR(client);
	Read_SFR(client, open);
	return 0;
}

int Mass_Erase(struct i2c_client *client)
{
	char mass_buff[4] = {0};
	char mass_buff_1[2] = {0};
	int ret, finish = 0, i = 0;

	ins_write(client, 0x01);	//id code read
	mdelay(2);
	long_word_scan_out(client);

	ret = i2c_master_recv(client, mass_buff, sizeof(mass_buff));
	pr_info("[ELAN|%s] mass_buff [%x %x %x %x / c0 08 01 00]\n", __func__,
			mass_buff[0],mass_buff[1],mass_buff[2],mass_buff[3]);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x8000);
	long_word_scan_in(client, 0x007f, 0x9040);
	ins_write(client, 0x66);
	long_word_scan_in(client, 0x0008, 0x8765);
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0xf3);

	while (finish == 0) {
		word_scan_out(client);
		ret = i2c_master_recv(client, mass_buff_1, sizeof(mass_buff_1));
		finish = (mass_buff_1[1] >> 4) & 0x01;

		// [80 10]: OK, [80 00]: fail
		pr_info("[ELAN|%s] mass_buff_1[0] = [%x], mass_buff_1[1] = "
				"[%x / 80 10]! Finish = [%d]\n", __func__,
				mass_buff_1[0], mass_buff_1[1], finish);

		if (mass_buff_1[1] != 0x10 && finish != 1 && i < 100) {
			mdelay(100);
			i++;
			if (i == 50)
				pr_err("[ELAN|%s] fail!\n", __func__);
		}
	}

	return 0;
}

int Reset_ICE(struct i2c_client *client)
{
	int res;

	ins_write(client, 0x94);
	ins_write(client, 0xd4);
	ins_write(client, 0x20);
	client->addr = 0x10;

	pr_info("[ELAN|%s] Modify address = %x\n", __func__, client->addr);
	elan_ktf2k_ts_hw_reset(private_ts->client);
	mdelay(250);

	res = __hello_packet_handler(client);

	return 0;
}

int normal_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint16_t szbuff = 0, szbuff_1 = 0
	uint16_t sendbuff = 0;
	int write_byte, iw;

	ins_write(client, 0xfd);
	word_scan_in(client, j * 64);
	ins_write(client, 0x65);

	write_byte = 64;

	for (iw = 0; iw < write_byte; iw++) {
		szbuff = *szBuff;
		szbuff_1 = *(szBuff + 1);
		sendbuff = szbuff_1 << 8 | szbuff;
		pr_info("[ELAN] Write Page: sendbuff [0x%04x]\n", sendbuff);
		word_scan_in(client, sendbuff);
		szBuff += 2;
	}

	return 0;
}

int fastmode_write_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfwbuff = 0, szfwbuff_1 = 0;
	uint8_t sendfwbuff[130] = {0};
	uint16_t tmpbuff;
	int i = 0, len = 0;
	private_ts->client->addr = 0x76;

	sendfwbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfwbuff[1] = tmpbuff;

	for (i = 2; i < 129; i = i + 2) {
		szfwbuff = *szBuff;
		szfwbuff_1 = *(szBuff + 1);
		sendfwbuff[i] = szfwbuff_1;
		sendfwbuff[i + 1] = szfwbuff;
		szBuff += 2;
	}

	len = i2c_master_send(private_ts->client, sendfwbuff,
			sizeof(sendfwbuff));

	private_ts->client->addr = 0x77;

	return 0;
}

int ektSize;
int lastpage_byte;
int lastpage_flag = 0;

int Write_Page(struct i2c_client *client, int j, uint8_t *szBuff)
{
	int len, finish = 0;
	char buff_read_data[2];
	int i = 0;

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x9400);

	ins_write(client, 0x66);

	long_word_scan_in(client, j * 64, 0x5a5a);
	fastmode_write_func(client, j, szBuff);

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x9000);

	long_word_scan_in(client, 0x007f, 0x8000);

	ins_write(client, 0xf3);

	while (finish == 0) {
		word_scan_out(client);
		len = i2c_master_recv(client, buff_read_data,
				sizeof(buff_read_data));
		finish = (buff_read_data[1] >> 4) & 0x01;
		if (finish != 1) {
			mdelay(10);
			pr_info("[ELAN] Write_Page finish !=1\n");
			i++;
			if (i == 50) {
				write_ice_status = 1;
				return -1;
			}
		}
	}

	return 0;
}

int fastmode_read_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	uint8_t szfrbuff = 0, szfrbuff_1 = 0;
	uint8_t sendfrbuff[2] = {0};
	uint8_t recvfrbuff[130] = {0};
	uint16_t tmpbuff;
	int i = 0, len = 0;

	ins_write(client, 0x67);

	private_ts->client->addr = 0x76;

	sendfrbuff[0] = (j * 64) >> 8;
	tmpbuff = ((j * 64) << 8) >> 8;
	sendfrbuff[1] = tmpbuff;
	len = i2c_master_send(private_ts->client, sendfrbuff,
			sizeof(sendfrbuff));

	len = i2c_master_recv(private_ts->client, recvfrbuff,
			sizeof(recvfrbuff));

	for (i = 2; i < 129; i = i + 2) {
		szfrbuff = *szBuff;
		szfrbuff_1 = *(szBuff + 1);
		szBuff += 2;
		if (recvfrbuff[i] != szfrbuff_1 ||
				recvfrbuff[i + 1] != szfrbuff) {
			pr_err("[ELAN] Read Page Compare Fail: recvfrbuff[%d] ="
					" [%x], recvfrbuff[i+1] = [%x], "
					"szfrbuff_1 = [%x], szfrbuff = [%x], "
					"j = [%d]\n", i,recvfrbuff[i],
					recvfrbuff[i+1], szfrbuff_1,
					szfrbuff, j);
			write_ice_status = 1;
		}
		break;
	}

	private_ts->client->addr = 0x77;

	return 0;
}

int normal_read_func(struct i2c_client *client, int j, uint8_t *szBuff)
{
	char read_buff[2];
	int m, len, read_byte;
	uint16_t szbuff = 0, szbuff_1 = 0;

	ins_write(client, 0xfd);
	word_scan_in(client, j * 64);
	ins_write(client, 0x67);
	word_scan_out(client);
	read_byte = 64;

	for (m = 0; m < read_byte; m++) {
		word_scan_out(client);
		len=i2c_master_recv(client, read_buff, sizeof(read_buff));

		szbuff = *szBuff;
		szbuff_1 = *(szBuff+1);
		szBuff += 2;
		pr_info("[ELAN] Read Page: byte [%x%x], szbuff [%x%x]\n",
				read_buff[0], read_buff[1],szbuff, szbuff_1);

		if (read_buff[0] != szbuff_1 || read_buff[1] != szbuff) {
			pr_err("[ELAN] Read Page Compare Fail: j = [%d], m = "
					"[%d]\n", j, m);
			write_ice_status=1;
		}
	}
	return 0;
}

int Read_Page(struct i2c_client *client, int j, uint8_t *szBuff)
{
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007e, 0x0023);
	long_word_scan_in(client, 0x007f, 0x9000);

	fastmode_read_func(client, j, szBuff);

	//Clear Flashce
	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x007f, 0x8000);
	return 0;
}

int TWO_WIRE_ICE(struct i2c_client *client)
{
	int i;

	uint8_t *szBuff = NULL;
	int curIndex = 0;
	int PageSize = 128;
	int res;

	write_ice_status = 0;
	ektSize = sizeof(file_bin_data) / PageSize;

	client->addr = 0x77;

	pr_info("[ELAN] ektSize [%d], modify address [%x]\n ", ektSize,
			client->addr);

	i = Enter_Mode(client);
	i = Open_High_Voltage(client, 1);
	if (i == -1) {
		pr_err("[ELAN] Open High Voltage fail\n");
		return -1;
	}

	i = Mass_Erase(client);
	if (i == -1) {
		pr_err("[ELAN] Mass Erase fail\n");
		return -1;
	}

	ins_write(client, 0x6f);
	long_word_scan_in(client, 0x7e, 0x36);
	long_word_scan_in(client, 0x7f, 0x8000);
	long_word_scan_in(client, 0x7e, 0x37);
	long_word_scan_in(client, 0x7f, 0x76);

	pr_info("[ELAN] client->addr [%2x]\n", client->addr);
	for (i = 0; i<ektSize; i++) {
		szBuff = file_bin_data + curIndex;
		curIndex = curIndex + PageSize;

		res = Write_Page(client, i, szBuff);
		if (res == -1) {
			pr_err("[ELAN] Write_Page [%d] fail\n ", i);
			break;
		}
		mdelay(1);
		Read_Page(client,i, szBuff);
	}
	pr_info("[ELAN] client->addr = %2x\n", client->addr);

	if (write_ice_status == 0)
		pr_info("[ELAN] Update_FW_Boot Finish!\n");
	else
		pr_err("[ELAN] Update_FW_Boot Fail!\n");

	pr_info("[ELAN] close High Voltage\n");
	i = Open_High_Voltage(client, 0);
	if (i == -1)
		return -1;

	Reset_ICE(client);

	return 0;
}

int elan_TWO_WIRE_ICE(struct i2c_client *client)
{
	work_lock = 1;
	disable_irq(private_ts->client->irq);
	TWO_WIRE_ICE(client);
	work_lock = 0;
	enable_irq(private_ts->client->irq);
	return 0;
}
#endif

static ssize_t elan_ktf2k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	pr_debug("[ELAN] GPIO_TP_INT_N [%d]\n", ts->intr_gpio);
	sprintf(buf, "[ELAN] GPIO_TP_INT_N [%d]\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t elan_ktf2k_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (strict_strtoul(buf, 10, &chip_mode_set))
		return -EINVAL;

	mutex_lock(&private_ts->lock);

	if (tp_sleep_status == 0) {
		mutex_unlock(&private_ts->lock);
		return count;
	}

	disable_irq(private_ts->client->irq);
		flush_work(&private_ts->work);
	cancel_delayed_work_sync(&private_ts->check_work);

	if (chip_type == 0x22) {
		elan_ktf2k_set_scan_mode(private_ts->client,0);
		elan_ktf2k_ts_set_mode_state(
				private_ts->client,chip_mode_set);
		if (elan_ktf2k_ts_get_mode_state(
				private_ts->client)!=chip_mode_set)
			elan_ktf2k_ts_set_mode_state(
					private_ts->client,chip_mode_set);
		msleep(10);
		elan_ktf2k_set_scan_mode(private_ts->client,1);
	} else {
		elan_ktf2k_ts_set_mode_state(
				private_ts->client,chip_mode_set);
		if (elan_ktf2k_ts_get_mode_state(
				private_ts->client)!=chip_mode_set)
			elan_ktf2k_ts_set_mode_state(
					private_ts->client,chip_mode_set);
	}

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);
	mutex_unlock(&private_ts->lock);

	return count;
}

static ssize_t elan_ktf2k_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	static unsigned long chip_mode_get;
	ssize_t ret = 0;

	chip_mode_get = elan_ktf2k_ts_get_mode_state(private_ts->client);

	ret = snprintf(buf, PAGE_SIZE, "%lu\n", chip_mode_get);

	return ret;
}

static ssize_t elan_ktf2k_talking_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (strict_strtoul(buf, 10, &talking_mode_set))
		return -EINVAL;

	mutex_lock(&private_ts->lock);
	if (tp_sleep_status == 0) {
		mutex_unlock(&private_ts->lock);
		return count;
	}

	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);
	cancel_delayed_work_sync(&private_ts->check_work);

	if (chip_type == 0x22) {
		elan_ktf2k_set_scan_mode(private_ts->client,0);
		elan_ktf2k_ts_set_talking_state(
				private_ts->client,talking_mode_set);
		if (elan_ktf2k_ts_get_talking_state(
				private_ts->client)!=talking_mode_set)
			elan_ktf2k_ts_set_talking_state(
					private_ts->client,talking_mode_set);
		msleep(10);
		elan_ktf2k_set_scan_mode(private_ts->client,1);
	} else {
		elan_ktf2k_ts_set_talking_state(
				private_ts->client,talking_mode_set);
		if (elan_ktf2k_ts_get_talking_state(
				private_ts->client)!=talking_mode_set) {
			pr_emerg("[ELAN|%s] We got the different type\n",
					__func__);
			elan_ktf2k_ts_set_talking_state(
					private_ts->client,talking_mode_set);
		} else
			pr_emerg("[ELAN|%s] We got the same type\n", __func__);
	}
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);
	mutex_unlock(&private_ts->lock);

	return count;
}

static ssize_t elan_ktf2k_talking_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	static unsigned long talking_mode_get;
	ssize_t ret = 0;

	talking_mode_get = elan_ktf2k_ts_get_talking_state(private_ts->client);

	ret = snprintf(buf, PAGE_SIZE, "%lu\n",talking_mode_get);

	return ret;
}

static ssize_t elan_ktf2k_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	static unsigned long chip_reset;

	if (strict_strtoul(buf, 10, &chip_reset))
		return -EINVAL;

	if (chip_reset > 0) {
		if (work_lock == 0) {
			work_lock = 1;
			disable_irq(private_ts->client->irq);
			cancel_work_sync(&private_ts->work);
		}
		elan_ktf2k_ts_hw_reset(private_ts->client);
		if (elan_ktf2k_ts_setup(private_ts->client) < 0)
			pr_info("[ELAN] No Elan chip inside\n");

		if (work_lock == 1) {
			work_lock = 0;
			enable_irq(private_ts->client->irq);
		}
	}

	return count;
}

static ssize_t elan_ktf2k_irq_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	static unsigned long chip_irq_set;

	if (strict_strtoul(buf, 10, &chip_irq_set))
		return -EINVAL;

	if (chip_irq_set > 0)
		enable_irq(private_ts->client->irq);
	else
		disable_irq(private_ts->client->irq);

	return count;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);
static DEVICE_ATTR(mode, 0644, elan_ktf2k_mode_show, elan_ktf2k_mode_set);
static DEVICE_ATTR(hw_reset, 0644, NULL, elan_ktf2k_reset);
static DEVICE_ATTR(talking_set, 0644, elan_ktf2k_talking_show,
		elan_ktf2k_talking_set);
static DEVICE_ATTR(set_irq, 0644, NULL, elan_ktf2k_irq_set);

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("[ELAN|%s] subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_group failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_mode.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_group failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_hw_reset.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_group failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_talking_set.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_group failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_set_irq.attr);
	if (ret) {
		pr_err("[ELAN|%s] sysfs_create_group failed\n", __func__);
		return ret;
	}

	return 0;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 20;

	do {
		status = gpio_get_value(ts->intr_gpio);
		retry--;
		if (status==1)
			msleep(25);
	} while (status == 1 && retry > 0);

	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);

	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
				buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

#ifdef RE_CALIBRATION
	mdelay(200);
	rc = i2c_master_recv(client, buf_recv, 8);
	pr_info("[ELAN|%s] Re-Calibration Packet [%2x:%2x:%2x:%2x]\n", __func__,
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	if (buf_recv[0] != 0x66) {
		mdelay(200);
		rc = i2c_master_recv(client, buf_recv, 8);
		pr_info("[ELAN|%s] Re-Calibration Packet, re-try again "
				"[%2x:%2x:%2x:%2x]\n", __func__, buf_recv[0],
				buf_recv[1], buf_recv[2], buf_recv[3]);
	}
#endif

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		pr_err("[ELAN|%s] Int poll failed!\n", __func__);
		RECOVERY = 0x80;
		return 0x88;
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	if (rc < 0)
		rc = i2c_master_recv(client, buf_recv, 8);
	pr_debug("[ELAN|%s] hello packet [%2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x]\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3], buf_recv[4], buf_recv[5], buf_recv[6],
			buf_recv[7]);

	if (buf_recv[0] == 0x55 && buf_recv[1] == 0x55 && buf_recv[2] == 0x80 &&
			buf_recv[3] == 0x80) {
		if (buf_recv[6] == 0x04 && buf_recv[7] == 0x21)
			chip_type = 0x21;
		RECOVERY = 0x80;
		return RECOVERY;
	}
	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00};
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00};
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01};
		uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};
	uint8_t buf_recv[4] = {0};

	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

	if (major == 0x55)
		chip_type = 0x21;
	else
		chip_type = 0x22;

	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor;
	X_RESOLUTION = ts->x_resolution;

	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;

	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor;
	Y_RESOLUTION = ts->y_resolution;

	pr_info("[ELAN|%s] Firmware version: 0x%4.4x\n", __func__, ts->fw_ver);
	pr_info("[ELAN|%s] Firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
	pr_info("[ELAN|%s] Bootcode Version: 0x%4.4x\n", __func__, ts->bc_ver);
	pr_info("[ELAN|%s] X Resolution: %d, Y Resolution: %d\n", __func__,
			X_RESOLUTION, Y_RESOLUTION);

	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data, uint16_t *x,
		uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc = 0;
	int touch_retry = 0;

	touch_retry = 3;

	do {
		elan_ktf2k_ts_hw_reset(client);
		rc = __hello_packet_handler(client);
		touch_retry--;
	} while (rc == 0x88 && touch_retry > 0);
	pr_info("[ELAN] first hellopacket's rc = %d\n",rc);

	mdelay(10);

	if (rc != 0x80 && rc != 0x88) {
		rc = __fw_packet_handler(client);
		if (rc < 0)
			pr_err("[ELAN|%s] fw_packet_handler fail, [%d]",
					__func__, rc);

		pr_debug("[ELAN|%s] firmware checking done\n", __func__);
		if (FW_VERSION == 0x00) {
			rc = 0x80;
			pr_err("[ELAN] FW_VERSION [%d], last FW update fail\n",
					FW_VERSION);
		}
	}
	return -rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	pr_debug("[ELAN] dump cmd [%02x, %02x, %02x, %02x]\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_set_mode_state(struct i2c_client *client, int mode)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x56, 0x01, 0x01};

	if (chip_type == 0x21)
		cmd[1] = 0x5C;

	cmd[2] = mode;

	pr_debug("[ELAN|%s] dump cmd [%02x, %02x, %02x, %02x]\n", __func__,
			cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	msleep(1);
	return 0;
}

/* 53 56 00 01*/
static int elan_ktf2k_ts_get_mode_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x56, 0x00, 0x01};
	uint8_t buf[4] = {0}, mode_state = 0;

	if (chip_type == 0x21)
		cmd[1] = 0x5C;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	mode_state = buf[2];
	pr_debug("[ELAN] dump response [%0x]\n", mode_state);

	return mode_state;
}

static int elan_ktf2k_ts_set_talking_state(struct i2c_client *client, int mode)
{
	uint8_t cmd[] = {0x54, 0x57, 0x01, 0x01};

	if (chip_type == 0x21)
		cmd[1] = 0x5D;

	cmd[2] = mode;

	pr_debug("[ELAN] dump cmd [%02x, %02x, %02x, %02x]\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	msleep(1);
	return 0;
}

static int elan_ktf2k_ts_get_talking_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {0x53, 0x57, 0x00, 0x01};
	uint8_t buf[4] = {0}, mode_state = 0;

	if (chip_type == 0x21)
		cmd[1] = 0x5D;

	pr_debug("[ELAN|%s] dump cmd [%02x, %02x, %02x, %02x]\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	mode_state = buf[2];
	pr_debug("[ELAN] dump response [%0x]\n", mode_state);

	return mode_state;
}

static void elan_ktf2k_clear_ram(struct i2c_client *client)
{
	uint8_t clear_cmd[] = {0x53, 0x0A, 0x00, 0x01};

	if ((i2c_master_send(client, clear_cmd, sizeof(clear_cmd))) !=
			sizeof(clear_cmd)) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return;
	}
	__elan_ktf2k_ts_poll(private_ts->client);
	i2c_master_recv(private_ts->client, clear_cmd, 4);
	pr_emerg("[ELAN|%s] %2x, %2x, %2x, %2x\n", __func__, clear_cmd[0],
			clear_cmd[1], clear_cmd[2], clear_cmd[3]);
}

static int elan_ktf2k_set_scan_mode(struct i2c_client *client, int mode)
{
	uint8_t stop_cmd[] = {0x54, 0x9F, 0x01, 0x00, 0x00, 0x01};
	uint8_t start_cmd[] = {0x54, 0x9F, 0x00, 0x00, 0x00, 0x01};

	if (mode) {
		if ((i2c_master_send(client, start_cmd, sizeof(start_cmd))) !=
				sizeof(start_cmd)) {
			pr_err("[ELAN|%s] i2c_master_send failed, mode [%d]\n",
					__func__, mode);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(client, stop_cmd, sizeof(stop_cmd))) !=
				sizeof(stop_cmd)) {
			pr_err("[ELAN|%s] i2c_master_send failed, mode:%d\n",
					__func__, mode);
			return -EINVAL;
		}
		msleep(1);
	}
	return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	cmd[1] |= (state << 3);

	pr_debug("[ELAN] dump cmd [%02x, %02x, %02x, %02x]\n", cmd[0], cmd[1],
			cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		pr_err("[ELAN|%s] i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_hw_reset(struct i2c_client *client)
{
	gpio_direction_output(SYSTEM_RESET_PIN_SR, 0);
	msleep(20);
	gpio_direction_output(SYSTEM_RESET_PIN_SR, 1);
	msleep(130);
	return 0;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status = 0;

void touch_callback(unsigned cable_status)
{
		now_usb_cable_status = cable_status;
}
#endif

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf,
		int bytes_to_recv)
{
	int rc;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

#ifdef ELAN_PROTOCOL
	rc = i2c_master_recv(client, buf, bytes_to_recv);
	pr_info("[ELAN] Elan protocol [%d]\n", rc);
	if (rc != bytes_to_recv) {
		pr_err("[ELAN|%s] i2c_master_recv error?!\n", __func__);
		return -1;
	}
#else
	rc = i2c_master_recv(client, buf, 8);
	if (rc != 8) {
		pr_err("[ELAN] Read the first package error\n");
		mdelay(30);
		return -1;
	}

	mdelay(1);
#endif
	return rc;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits = 0;
	uint8_t i, num, reported = 0;
	uint8_t idx, btn_idx;
	int finger_num;

	if (buf[0] == TEN_FINGERS_PKT) {
		/* for 10 fingers */
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		btn_idx = 33;
	} else if ((buf[0] == MTK_FINGERS_PKT) ||
			(buf[0] == FIVE_FINGERS_PKT)) {
		/* for 5 fingers */
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >>3;
		idx = 2;
		btn_idx = 17;
	} else {
		/* for 2 fingers */
		finger_num = 2;
		num = buf[7] & 0x03;
		fbits = buf[7] & 0x03;
		idx = 1;
		btn_idx = 7;
	}

	switch (buf[0]) {
	case 0x78:
		break;
	case 0x55:
		if (chip_type == 0x22) {
			if (buf[0] == 0x55 && buf[1] == 0x55 && buf[2] == 0x55
					&& buf[3] == 0x55) {
				mutex_lock(&private_ts->lock);
				pr_info("[ELAN|%s] get tp chip int gpio status"
						" [%d]\n", __func__,
						gpio_get_value(
						private_ts->intr_gpio));
				if (chip_type == 0x22) {
					elan_ktf2k_set_scan_mode(private_ts->client, 0);
					elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
					if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
						elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
					elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
					if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
						elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
					msleep(10);
					elan_ktf2k_set_scan_mode(private_ts->client, 1);
				} else {
					if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
						elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
					elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
					if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
						elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
					msleep(10);
				}
				mutex_unlock(&private_ts->lock);
			}
		}
		break;
	case 0x66:
		if (buf[0] == 0x66 && buf[1] == 0x66 && buf[2] == 0x66 &&
				buf[3] == 0x66)
			pr_debug("[ELAN] calibration packet\n");
		else
			pr_debug("[ELAN] unknow packet type\n");
		break;
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:
		input_report_key(idev, BTN_TOUCH, 1);
		if (num == 0) {
#ifdef ELAN_BUTTON
			if (buf[btn_idx] == 0x21) {
				button_state = 0x21;
				input_report_key(idev, KEY_BACK, 1);
				input_report_key(idev, KEY_BACK, 0);
				pr_info("[ELAN] button %x\n", buf[btn_idx]);
			} else if (buf[btn_idx] == 0x41) {
				button_state = 0x41;
				input_report_key(idev, KEY_HOME, 1);
			} else if (buf[btn_idx] == 0x81) {
				button_state = 0x81;
				input_report_key(idev, KEY_MENU, 1);
			} else if (button_state == 0x21) {
				button_state = 0;
				input_report_key(idev, KEY_BACK, 0);
			} else if (button_state == 0x41) {
				button_state = 0;
				input_report_key(idev, KEY_HOME, 0);
			} else if (button_state == 0x81) {
				button_state = 0;
				input_report_key(idev, KEY_MENU, 0);
			} else {
				pr_emerg("[ELAN] No Press\n");
				input_mt_sync(idev);
			}
#endif
		} else {
			pr_debug("[ELAN] %d fingers\n", num);
			input_report_key(idev, BTN_TOUCH, 1);
			for (i = 0; i < finger_num; i++) {
				if ((fbits & 0x01)) {
					elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
					x = x * (LCM_X_RESOLUTION)/X_RESOLUTION;
					y = y * (LCM_Y_RESOLUTION)/Y_RESOLUTION;

					if (!((x <= 0) || (y <= 0) || (x >= LCM_X_RESOLUTION) || (y>=LCM_Y_RESOLUTION))) {
						input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 8);
						input_report_abs(idev, ABS_MT_POSITION_X, x);
						input_report_abs(idev, ABS_MT_POSITION_Y, y);
						input_mt_sync(idev);
						reported++;
					}
				}
				fbits = fbits >> 1;
				idx += 3;
			}
		}
		if (reported)
			input_sync(idev);
		else {
			input_mt_sync(idev);
			input_sync(idev);
		}
		break;
	default:
		pr_err("[ELAN|%s] unknown packet type [%0x]\n", __func__,
				buf[0]);
		break;
	}

	return;
}

static void elan_ktf2k_ts_check_work_func(struct work_struct *work)
{
	int do_tp_reset = 0;
	int touch_retry = 0;

	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);

	if (chip_reset_flag == 0) {
		chip_reset_flag = 1;
		schedule_delayed_work(&private_ts->check_work,
				msecs_to_jiffies(2500));
		enable_irq(private_ts->client->irq);
		return;
	}

	touch_retry = 3;
	do {
		elan_ktf2k_ts_hw_reset(private_ts->client);
		do_tp_reset = __hello_packet_handler(private_ts->client);
		touch_retry--;
	} while (do_tp_reset != 0 && touch_retry > 0);

	if (do_tp_reset != 0)
		pr_err("[ELAN] Receive hello package fail\n");
	else {
		mutex_lock(&private_ts->lock);
		if (chip_type == 0x22) {
			elan_ktf2k_set_scan_mode(private_ts->client, 0);
			elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
			if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
				elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

			elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
			if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
				elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);

			msleep(10);
			elan_ktf2k_set_scan_mode(private_ts->client, 1);
		} else {
			elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
			if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
				elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
			elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
			if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
				elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
		}
		mutex_unlock(&private_ts->lock);
	}
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[4+PACKET_SIZE] = { 0 };
#ifdef ELAN_BUFFER_MODE
	uint8_t buf1[PACKET_SIZE] = { 0 };
#endif

	chip_reset_flag = 0;

	if (gpio_get_value(ts->intr_gpio)) {
		pr_info("[ELAN] Detected the jitter on INT pin\n");
		enable_irq(ts->client->irq);
		return;
	}

	rc = elan_ktf2k_ts_recv_data(ts->client, buf,4+PACKET_SIZE);

	if (rc < 0) {
		pr_info("[ELAN] Received the packet Error.\n");
		enable_irq(ts->client->irq);
		return;
	}

#ifndef ELAN_BUFFER_MODE
	elan_ktf2k_ts_report_data(ts->client, buf);
#else
	elan_ktf2k_ts_report_data(ts->client, buf+4);
	// Second package
	if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
		rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
		if (rc < 0) {
			enable_irq(ts->client->irq);
			return;
		}
		elan_ktf2k_ts_report_data(ts->client, buf1);
		// Final package
		if (buf[1] == 3) {
			rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
			if (rc < 0) {
				enable_irq(ts->client->irq);
				return;
			}
			elan_ktf2k_ts_report_data(ts->client, buf1);
		}
	}
#endif
	enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		pr_err("[ELAN|%s] request_irq [%d] failed\n", __func__,
				client->irq);

	return err;
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int elan_ktf2k_ts_power_on(struct elan_ktf2k_ts_data *data, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(data->vcc_ana, ELAN_ACTIVE_LOAD_UA);
	if (rc < 0) {
		pr_err("[ELAN] Regulator vcc_ana set_opt failed [%d]\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_ana);
	if (rc) {
		pr_err("Regulator vcc_ana enable failed [%d]\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (data->pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(data->vcc_i2c,
				ELAN_I2C_LOAD_UA);
		if (rc < 0) {
			pr_err("Regulator vcc_i2c set_opt failed [%d]\n", rc);
			goto error_reg_opt_vcc;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			pr_err("[ELAN] Regulator vcc_i2c enable failed [%d]\n",
					rc);
			goto error_reg_en_vcc_i2c;
		}
	} else
		pr_err("[ELAN] we do not have i2c_pull_up\n");

	return 0;

	error_reg_en_vcc_i2c:
	if (data->pdata->i2c_pull_up)
		reg_set_optimum_mode_check(data->vcc_i2c, 0);
	error_reg_opt_vcc:
	regulator_disable(data->vcc_ana);
	error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(data->vcc_ana, 0);

	power_off:
	reg_set_optimum_mode_check(data->vcc_ana, 0);
	regulator_disable(data->vcc_ana);
	if (data->pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(data->vcc_i2c, 0);
		regulator_disable(data->vcc_i2c);
	}
	msleep(50);
	return 0;
}

static int elan_ktf2k_ts_regulator_configure(struct elan_ktf2k_ts_data *data,
		bool on)
{
	int rc = 0;

	if (on == false)
		goto hw_shutdown;

	data->vcc_ana = regulator_get(&data->client->dev, "vdd_ana");
	if (IS_ERR(data->vcc_ana)) {
		rc = PTR_ERR(data->vcc_ana);
		pr_err("[ELAN] Regulator get failed vcc_ana [%d]\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_ana) > 0) {
		rc = regulator_set_voltage(data->vcc_ana, ELAN_VTG_MIN_UV,
				ELAN_VTG_MAX_UV);
		if (rc) {
			pr_err("[ELAN] regulator set_vtg failed [%d]\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	if (data->pdata->i2c_pull_up) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			pr_err("[ELAN] Regulator get failed [%d]\n", rc);
			goto error_set_vtg_vcc_ana;
		}
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c,
					ELAN_I2C_VTG_MIN_UV,
					ELAN_I2C_VTG_MAX_UV);
			if (rc) {
				pr_err("[ELAN] regulator set_vtg failed [%d]\n",
						rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

	error_set_vtg_i2c:
	regulator_put(data->vcc_i2c);

	error_set_vtg_vcc_ana:
	regulator_put(data->vcc_ana);
	return rc;

	hw_shutdown:
	if (regulator_count_voltages(data->vcc_ana) > 0)
		regulator_set_voltage(data->vcc_ana, 0, ELAN_VTG_MAX_UV);
	regulator_put(data->vcc_ana);

	if (data->pdata->i2c_pull_up) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0,
					ELAN_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
	}
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct elan_ktf2k_ts_data *elan_dev_data =
		container_of(self, struct elan_ktf2k_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			elan_dev_data && elan_dev_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			elan_ktf2k_ts_resume(&elan_dev_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			elan_ktf2k_ts_suspend(&elan_dev_data->client->dev);
	}

	return 0;
}
#endif

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;
	int New_FW_ID;
	int New_FW_VER;

	pr_debug("[ELAN|%s] client->addr [0x%x], name [%s]\n", __func__,
			client->addr, client->name);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct elan_ktf2k_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("[ELAN] Failed to allocate memory\n");
			return -ENOMEM;
		}
		err = elan_ktf2k_ts_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else
		pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[ELAN|%s] i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("[ELAN|%s] allocate elan_ktf2k_ts_data failed\n",
				__func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		pr_err("[ELAN|%s] create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);

	INIT_DELAYED_WORK(&ts->check_work, elan_ktf2k_ts_check_work_func);
	mutex_init(&ts->lock);

	ts->client = client;
	ts->pdata = pdata;

	i2c_set_clientdata(client, ts);

	elan_ktf2k_ts_regulator_configure(ts, true);
	elan_ktf2k_ts_power_on(ts,true);
	pr_emerg("[ELAN] pdata->intr_gpio [%d], pdata->reset_gpio [%d]\n", 
			pdata->intr_gpio, pdata->reset_gpio);

	if (gpio_is_valid(pdata->intr_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->intr_gpio, "elan_intr_gpio");
		if (err)
			pr_err("[ELAN] unable to request gpio [%d]\n",
					pdata->intr_gpio);
		err = gpio_direction_input(pdata->intr_gpio);
		client->irq = gpio_to_irq(pdata->intr_gpio);
		pr_emerg("[ELAN] elan_intr_gpio [%d], request gpio [%d]\n",
				pdata->intr_gpio, client->irq);
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->reset_gpio, "elan_reset_gpio");
		if (err)
			pr_err("[ELAN] unable to request gpio [%d]\n",
					pdata->reset_gpio);
		gpio_direction_output(pdata->reset_gpio, 1);
	}

	if (gpio_is_valid(pdata->hw_det_gpio)) {
		touch_panel_type = gpio_get_value(pdata->hw_det_gpio);
		pr_info("[ELAN] touch_panel_type [%d]\n", touch_panel_type);
	} else
		pr_emerg("[ELAN] Touch hw_det_gpio is WRONG\n");

	if (likely(pdata != NULL))
		ts->intr_gpio = pdata->intr_gpio;

	fw_err = elan_ktf2k_ts_setup(client);
	if (fw_err == -136) {
		pr_err("[ELAN] No Elan chip inside [%d]\n", fw_err);
		err = -ENODEV;
		goto err_create_wq_failed;

	}
	else if (fw_err < 0)
		pr_err("[ELAN] No Elan chip inside?? [%d]\n", fw_err);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		pr_err("[ELAN] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	if (!touch_panel_type && chip_type == 0x22) {
		file_fw_data = file_fw_data_Truly;
		pr_emerg("[ELAN] Touch is [2227] truly panel\n");
	} else if (!touch_panel_type && chip_type == 0x21) {
		file_fw_data = file_fw_data_Truly_2127;
		pr_emerg("[ELAN] Touch is [2127e] truly panel\n");
	} else if (touch_panel_type && chip_type == 0x22) {
		file_fw_data = file_fw_data_Eely;
		pr_emerg("[ELAN] Touch is [2227] eely panel\n");
	} else {
		file_fw_data = file_fw_data_Eely_2127;
		pr_emerg("[ELAN] Touch is [2127] eely panel\n");
	}

	private_ts = ts;

	ts->input_dev->name = ELAN_KTF2K_NAME;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = touch_panel_type;
	ts->input_dev->id.version = FW_VERSION;
	ts->input_dev->dev.parent = &client->dev;
	ts->input_dev->open = NULL;
	ts->input_dev->close = NULL;

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef ELAN_BUTTON
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif

	/* For single touch */
	input_set_abs_params(ts->input_dev, ABS_X, 0, LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, LCM_Y_RESOLUTION, 0, 0);

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
			LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			LCM_Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	// Start Firmware auto Update
	if (1) {
		work_lock=1;
		power_lock = 1;
		if (chip_type == 0x22) {
			pr_info("[ELAN] [7E65] = 0x%02x, "
					"[7E64] = 0x%02x, "
					"[0x7E67] = 0x%02x, "
					"[0x7E66] = 0x%02x\n",
					file_fw_data[0x7E65],
					file_fw_data[0x7E64],
					file_fw_data[0x7E67],
					file_fw_data[0x7E66]);
			New_FW_ID = file_fw_data[0x7E67] << 8 |
					file_fw_data[0x7E66];
			New_FW_VER = file_fw_data[0x7E65] << 8 |
					file_fw_data[0x7E64];
		} else {
			pr_info("[ELAN] [7D97] = 0x%02x, "
					"[7D96] = 0x%02x, "
					"[0x7D99] = 0x%02x, "
					"[0x7D98] = 0x%02x\n",
					file_fw_data[0x7D97],
					file_fw_data[0x7D96],
					file_fw_data[0x7D99],
					file_fw_data[0x7D98]);
			New_FW_ID = file_fw_data[0x7D67] << 8 |
					file_fw_data[0x7D66];
			New_FW_VER = file_fw_data[0x7D65] << 8 |
					file_fw_data[0x7D64];
		}

		pr_info("[ELAN] FW_ID [0x%x], New_FW_ID [0x%x]\n", FW_ID,
				New_FW_ID);
		pr_info("[ELAN] FW_VERSION [0x%x], New_FW_VER [0x%x], chip_type"
				" [0x%x]\n", FW_VERSION, New_FW_VER,chip_type);

		if (New_FW_ID == FW_ID) {
			if (New_FW_VER > (FW_VERSION)) {
				Update_FW_One(client, RECOVERY);
				FW_VERSION = New_FW_VER;
				ts->input_dev->id.version = FW_VERSION;
			}
		} else
			pr_info("[ELAN] FW_ID is different!");

		if (FW_ID == 0) {
			RECOVERY = 0x80;
			Update_FW_One(client, RECOVERY);
			FW_VERSION = New_FW_VER;
			ts->input_dev->id.version = FW_VERSION;
		}

		power_lock = 0;
		work_lock = 0;
	}

	err = input_register_device(ts->input_dev);
	if (err) {
		pr_err("[ELAN|%s] unable to register [%s] input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_touch_sysfs_init();

	pr_info("[ELAN] Start touchscreen [%s] in interrupt mode\n",
		ts->input_dev->name);

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);
	if (err)
		pr_err("Unable to register fb_notifier [%d]\n", err);
#endif
	// Firmware Update++
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		pr_err("[ELAN] misc_register failed!!");
	else
		pr_info("[ELAN] misc_register finished!!");

	// Firmware Update--
	if (chip_type == 0x22)
		elan_ktf2k_clear_ram(private_ts->client);

	tp_sleep_status = 1;
	elan_ktf2k_ts_register_interrupt(ts->client);
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	pr_emerg("[ELAN|%s] Finish\n", __func__);

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
err_create_wq_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_parse_dt(struct device *dev, struct elan_ktf2k_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "elan,reset-gpio",
				0, &pdata->reset_gpio_flags);
	pdata->intr_gpio = of_get_named_gpio_flags(np, "elan,irq-gpio",
				0, &pdata->intr_gpio_flags);
	pdata->hw_det_gpio = of_get_named_gpio_flags(np, "elan,hw-det-gpio",
				0, &pdata->hw_det_gpio_flags);
	pdata->i2c_pull_up = of_property_read_bool(np, "elan,i2c-pull-up");

	return 0;
}

static int elan_ktf2k_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	mutex_lock(&private_ts->lock);
	if (tp_sleep_status == 1) {
		if (power_lock == 0) {
			disable_irq(client->irq);
			flush_work(&ts->work);
			cancel_delayed_work_sync(&ts->check_work);
			elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
		}
		tp_sleep_status = 0;
	}
	mutex_unlock(&private_ts->lock);
	return 0;
}

static int elan_ktf2k_ts_resume(struct device *dev)
{
#ifdef RE_CALIBRATION
	uint8_t buf_recv[4] = { 0 };
#endif

	struct i2c_client *client = to_i2c_client(dev);

	mutex_lock(&private_ts->lock);
	if (tp_sleep_status == 0) {
		if (power_lock == 0) {
			gpio_direction_output(SYSTEM_RESET_PIN_SR, 0);
			msleep(5);
			gpio_direction_output(SYSTEM_RESET_PIN_SR, 1);
			msleep(150);

			if (__hello_packet_handler(private_ts->client) < 0)
				pr_err("[ELAN|%s] Hellopacket's receive fail\n",
						__func__);

			if (chip_type == 0x22) {
				elan_ktf2k_set_scan_mode(private_ts->client, 0);
				elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

				if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
					elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

				elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);

				if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
					elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);

				msleep(10);
				elan_ktf2k_set_scan_mode(private_ts->client, 1);
			} else {
				elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

				if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
					elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

				elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);

				if (elan_ktf2k_ts_get_talking_state(private_ts->client) != talking_mode_set)
					elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
			}

			schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
			enable_irq(client->irq);
		}
		tp_sleep_status = 1;
	}
	mutex_unlock(&private_ts->lock);	// release lock

	return 0;
}

static const struct dev_pm_ops ktf2k_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend	= elan_ktf2k_ts_suspend,
	.resume	= elan_ktf2k_ts_resume,
#endif
};

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
	{ ELAN_KTF2K_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, elan_ktf2k_ts_id);

static struct of_device_id elan_ktf2k_ts_match_table[] = {
	{ .compatible = "elan,ktf2k_ts",},
	{ },
};

static struct i2c_driver ktf2k_driver = {
	.driver = {
		.name	= "elan_ktf2k_ts",
		.owner	= THIS_MODULE,
		.of_match_table = elan_ktf2k_ts_match_table,
		.pm	= &ktf2k_pm_ops,
	},
	.probe		= elan_ktf2k_ts_probe,
	.remove		= __devexit_p(elan_ktf2k_ts_remove),
	.id_table	= elan_ktf2k_ts_id,
};
module_i2c_driver(ktf2k_driver);

MODULE_DESCRIPTION("ELAN EKTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
