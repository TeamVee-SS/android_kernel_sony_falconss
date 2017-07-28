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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 * 2012/12/1:  Release version 0x0005: support up to 10 fingers but no
 *             buffer mode.
 *
 */

#include <asm/ioctl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/ektf2k.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>

#if defined(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

/*[Arima Edison] add ++*/
// Analog voltage @2.7 V
#define ELAN_VTG_MIN_UV 2850000
#define ELAN_VTG_MAX_UV 3300000 // 33
#define ELAN_ACTIVE_LOAD_UA 15000
#define ELAN_LPM_LOAD_UA 10

#define ELAN_I2C_VTG_MIN_UV 1800000
#define ELAN_I2C_VTG_MAX_UV 1800000
#define ELAN_I2C_LOAD_UA 10000
#define ELAN_I2C_LPM_LOAD_UA 10
/*[Arima Edison] add--*/

#define PACKET_SIZE 8 /* support 2 fingers packet  */

#define PWR_STATE_DEEP_SLEEP 0
#define PWR_STATE_NORMAL 1
#define PWR_STATE_MASK BIT(3)

#define CMD_S_PKT 0x52
#define CMD_R_PKT 0x53
#define CMD_W_PKT 0x54
#define HELLO_PKT 0x55
#define TWO_FINGERS_PKT 0x5A
#define FIVE_FINGERS_PKT 0x5D
#define MTK_FINGERS_PKT 0x6D
#define TEN_FINGERS_PKT 0x62
#define BUFFER_PKT 0x63
#define BUFFER55_PKT 0x66
#define RESET_PKT 0x77
#define CALIB_PKT 0x66

// modify
#define SYSTEM_RESET_PIN_SR 0 // nexus7 TEGRA_GPIO_PH6	62

// Add these Define
#define PAGERETRY 30
#define IAPRESTART 5

// For Firmware Update
#define ELAN_IOCTLID 0xD0
#define IOCTL_I2C_SLAVE _IOW(ELAN_IOCTLID, 1, int)
#define IOCTL_MAJOR_FW_VER _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID 0xA0
#define IOCTL_CIRCUIT_CHECK _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE _IOR(CUSTOMER_IOCTLID, 2, int)

uint8_t RECOVERY = 0x00;
int FW_VERSION = 0x00;
int X_RESOLUTION = 576; // nexus7 1280
int Y_RESOLUTION = 960; // nexus7 2112

//[Arima Edison]++
uint8_t chip_type = 0x00;
//[Arima Edison]--

int LCM_X_RESOLUTION = 480;
int LCM_Y_RESOLUTION = 800;

int FW_ID = 0x00;
int work_lock = 0x00;
int power_lock = 0x00;
int circuit_ver = 0x01;
/*++++i2c transfer start+++++++*/
int file_fops_addr = 0x15;
/*++++i2c transfer end+++++++*/

static int touch_panel_type;
static int chip_reset_flag = 0;
uint8_t ic_status = 0x00; // 0:OK 1:master fail 2:slave fail
int update_progree = 0;
uint8_t I2C_DATA[3] = {0x15, 0x20, 0x21}; /*I2C devices address*/ // 1218 modify
int is_OldBootCode = 0;						  // 0:new 1:old

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

enum { PageSize = 132,
       PageNum = 249,
       ACK_Fail = 0x00,
       ACK_OK = 0xAA,
       ACK_REWRITE = 0x55,
};

enum { E_FD = -1,
};

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct delayed_work check_work;
	struct elan_ktf2k_i2c_platform_data *pdata;
	int intr_gpio;
	int reset_gpio; // Arima Edison add
			/*Arima Edison add++*/
	struct regulator *vcc_ana;
	struct regulator *vcc_i2c;
	/*Arima Edison add--*/
	// Firmware Information
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	// For Firmare Update
	struct miscdevice firmware;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
};

//[Arima Edison] ++
static int elan_ktf2k_ts_parse_dt(struct device *,
				  struct elan_ktf2k_i2c_platform_data *);
//[Arima Edison]--

static struct elan_ktf2k_ts_data *private_ts;
static int elan_ktf2k_ts_hw_reset(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int
elan_ktf2k_ts_rough_calibrate(struct i2c_client *client); // Arima Edison mask
static int elan_ktf2k_ts_resume(struct device *dev);
static int elan_ktf2k_ts_suspend(struct device *dev);

int Update_FW_One(struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);

// For Firmware Update
int elan_iap_open(struct inode *inode, struct file *filp)
{
	printk("[ELAN]into elan_iap_open\n");
	if (private_ts == NULL)
		printk("private_ts is NULL~~~");
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp) { return 0; }

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count,
			      loff_t *offp)
{
	int ret;
	char *tmp;

	printk("[ELAN]into elan_iap_write\n");

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count)) {
		return -EFAULT;
	}

	ret = i2c_master_send(private_ts->client, tmp, count);

	kfree(tmp);

	return (ret) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;

	printk("[ELAN]into elan_iap_read\n");

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(private_ts->client, tmp, count);
	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret) ? count : ret;
}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{

	int __user *ip = (int __user *)arg;
	printk("[ELAN]into elan_iap_ioctl\n");
	printk("cmd value %x\n", cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:
		private_ts->client->addr = (int __user)arg; // 1218
		printk(KERN_EMERG "IOCTL_I2C_SLAVE =0x%x \n", (int __user)arg);
		break;
	case IOCTL_MAJOR_FW_VER:
		break;
	case IOCTL_MINOR_FW_VER:
		break;
	case IOCTL_RESET:
		// modify
		gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		msleep(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		msleep(20);
		printk(KERN_EMERG "elan_iap_ioctl : IOCTL_RESET\n ");
		break;
	case IOCTL_IAP_MODE_LOCK:
		cancel_delayed_work_sync(&private_ts->check_work); // 1218
		if (!work_lock) {
			work_lock = 1;
			disable_irq(private_ts->client->irq);
			flush_work(&private_ts->work);
			printk(KERN_EMERG "IOCTL_IAP_MODE_LOCK\n");
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (work_lock) {
			work_lock = 0;
			enable_irq(private_ts->client->irq);
			printk(KERN_EMERG "IOCTL_IAP_MODE_UNLOCK\n");
		}
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
	case IOCTL_CIRCUIT_CHECK:
		return circuit_ver;
		break;
	default:
		printk("[elan] Un-known IOCTL Command %d\n", cmd);
		break;
	}
	return 0;
}

struct file_operations elan_touch_fops = {
    .open = elan_iap_open,
    .write = elan_iap_write,
    .read = elan_iap_read,
    .release = elan_iap_release,
    .unlocked_ioctl = elan_iap_ioctl,
};

int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;

	len = i2c_master_send(private_ts->client, isp_cmd, sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
		return -1;
	} else
		printk("[ELAN] IAPMode write data successfully! cmd = [%2x, "
		       "%2x, %2x, %2x]\n",
		       isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
	return 0;
}

int ExtractPage(struct file *filp, uint8_t *szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage, byte, &filp->f_pos);
	if (len != byte) {
		printk("[ELAN] ExtractPage ERROR: read page error, read error. "
		       "len=%d\r\n",
		       len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t *szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		printk(
		    "[ELAN] ERROR: write page error, write error. len=%d\r\n",
		    len);
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
		printk("[ELAN] ERROR: read data error, write 50 times error. "
		       "len=%d\r\n",
		       len);
		return -1;
	}

	pr_info("[ELAN] GetAckData:%x,%x", buff[0], buff[1]);
	if (buff[0] == 0xaa /* && buff[1] == 0xaa*/)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	page_tatol = page + 249 * (ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;

	printk("\rprogress %s| %d%%", str, percent);

	if (page == (249))
		printk("\n");
}
/*
* Restet and (Send normal_command ?)
* Get Hello Packet
*/

int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0; // rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0; // For IAP_RESTART

	uint8_t recovery_buffer[8] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd_2227[] = {0x54, 0x00, 0x12, 0x34}; // for 2227e
	uint8_t isp_cmd_2127[] = {0x45, 0x49, 0x41, 0x50}; // for 2127e

	dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:
	// reset
	// modify

	data = I2C_DATA[0]; // Master
	dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__,
		data);

	if (recovery != 0x80) {
		printk("[ELAN] Firmware upgrade normal mode !\n");
		gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		mdelay(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		mdelay(5);
		if (chip_type == 0x22) {
			res = i2c_master_recv(private_ts->client,
					      recovery_buffer, 8);
			res =
			    EnterISPMode(private_ts->client,
					 isp_cmd_2227); // enter 2227e IAP mode
		} else
			res =
			    EnterISPMode(private_ts->client,
					 isp_cmd_2127); // enter 2127e IAP mode
	} else {
		printk("[ELAN] Firmware upgrade recovery mode !\n");
	}
	res = i2c_master_recv(private_ts->client, recovery_buffer,
			      4); // 55 aa 33 cc
	printk("[ELAN]recovery byte data:%x,%x,%x,%x \n", recovery_buffer[0],
	       recovery_buffer[1], recovery_buffer[2], recovery_buffer[3]);

	if (recovery_buffer[1] != 0xaa &&
	    restartCnt < 5) // if buffer[0]!=aa, means we do not enter IAP mode
	{
		restartCnt++;
		goto IAP_RESTART;
	} else if (restartCnt >= 5) {
		printk(KERN_EMERG "[Elan] IAP fail !!! \n");
		return 0;
	}

	// Send Dummy Byte
	printk("[ELAN] send one byte data:%x,%x", private_ts->client->addr,
	       data);
	res = i2c_master_send(private_ts->client, &data, sizeof(data));
	if (res != sizeof(data)) {
		printk("[ELAN] dummy error code = %d\n", res);
	}
	msleep(10);

	// Start IAP
	for (iPage = 1; iPage <= PageNum; iPage++) {
	PAGE_REWRITE:
		// 8 bytes mode
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
		} // end of for(byte_count=1;byte_count<=17;byte_count++)

		if (iPage == 249 || iPage) {
			msleep(600);
		} else {
			msleep(50);
		}

		res = GetAckData(private_ts->client);
		if (ACK_OK != res) {
			msleep(50);
			printk("[ELAN] ERROR: GetAckData fail! res=%d\r\n",
			       res);
			if (res == ACK_REWRITE) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY) {
					printk("[ELAN] ID 0x%02x %dth page "
					       "ReWrite %d times fails!\n",
					       data, iPage, PAGERETRY);
					return E_FD;
				} else {
					printk("[ELAN] ---%d--- page ReWrite "
					       "%d times!\n",
					       iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5) {
					printk("[ELAN] ID 0x%02x ReStart %d "
					       "times fails!\n",
					       data, IAPRESTART);
					return E_FD;
				} else {
					printk("[ELAN] ===%d=== page ReStart "
					       "%d times!\n",
					       iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		} else {
			printk("  data : 0x%02x ", data);
			rewriteCnt = 0;
			print_progress(iPage, ic_num, i);
		}

		msleep(10);
	} // end of for(iPage = 1; iPage <= PageNum; iPage++)

	printk("[ELAN] read Hello packet data!\n");
	res = __hello_packet_handler(client);
	if (res > 0)
		printk("[ELAN] Update ALL Firmware successfully!\n");

	return 0; // Arima edison add
}
// End Firmware Update

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
				  uint8_t *buf, size_t size)
{
	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev, "[elan]%s: i2c_master_send failed\n",
			__func__);
		return -EINVAL;
	}

	if (i2c_master_recv(client, buf, size) != size || buf[0] != CMD_S_PKT)
		return -EINVAL;

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = {0};

	rc = i2c_master_recv(client, buf_recv, 8);
	//[Arima Edison] do the receive it again++
	if (rc < 0)
		rc = i2c_master_recv(client, buf_recv, 8);
	//[Arima Edison] do the receive it again--

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
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01}; /* Get Firmware Version*/
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00};    /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00};    /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01};   /*Get firmware ID*/
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00,
			    0x01}; /* Get BootCode Version*/
	uint8_t buf_recv[4] = {0};

	// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

	//[Arima Edison] add to set chip type++
	if (major == 0x55)
		chip_type = 0x21;
	else
		chip_type = 0x22;
	//[Arima Edison] add to set chip type++

	// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	// Bootcode version
	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

	// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor;
	X_RESOLUTION = ts->x_resolution;

	// Y Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor;
	Y_RESOLUTION = ts->y_resolution;

	printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n", __func__,
	       ts->fw_ver);
	printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n", __func__,
	       ts->fw_id);
	printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n", __func__,
	       ts->bc_ver);
	printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
	       __func__, X_RESOLUTION, Y_RESOLUTION);

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

	elan_ktf2k_ts_hw_reset(client);

	rc = __hello_packet_handler(client);
	if (rc != 0x80) {
		rc = __fw_packet_handler(client);
		if (rc < 0)
			printk("[elan] %s, fw_packet_handler fail, rc = %d",
			       __func__, rc);
		dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n",
			__func__);
		// Check for FW_VERSION, if 0x0000 means FW update fail!
		if (FW_VERSION == 0x00) {
			rc = 0x80;
			printk("[elan] FW_VERSION = %d, last FW update fail\n",
			       FW_VERSION);
		}
	}
	return -rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	printk("[elan] %s: enter\n", __func__);
	dev_info(&client->dev, "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		 cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

//[Arima Edison] add ram clear command++
static void elan_ktf2k_clear_ram(struct i2c_client *client)
{
	uint8_t clear_cmd[] = {0x53, 0x0A, 0x00, 0x01};

	if ((i2c_master_send(client, clear_cmd, sizeof(clear_cmd))) !=
	    sizeof(clear_cmd)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n",
			__func__);
		return;
	}

	i2c_master_recv(private_ts->client, clear_cmd, 4);
	printk(KERN_EMERG "%s, %2x,%2x,%2x,%2x \n", __func__, clear_cmd[0],
	       clear_cmd[1], clear_cmd[2], clear_cmd[3]);
}
//[Arima Edison] add ram clear command--

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev, "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n",
			__func__);
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

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf,
				   int bytes_to_recv)
{

	int rc;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	rc = i2c_master_recv(client, buf, 8);
	if (rc != 8) {
		printk("[elan] Read the first package error.\n");
		mdelay(30);
		return -1;
	}

	return rc;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	uint16_t x, y;
	uint16_t fbits = 0;
	uint8_t i, num, reported = 0;
	uint8_t idx;
	int finger_num;

	/* for 10 fingers	*/
	if (buf[0] == TEN_FINGERS_PKT) {
		finger_num = 10;
		num = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
	}
	/* for 5 fingers	*/
	else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)) {
		finger_num = 5;
		num = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
	}
	/* for 2 fingers */
	else {
		finger_num = 2;
		num = buf[7] &
		      0x03; // for elan old 5A protocol the finger ID is 0x06
		fbits = buf[7] & 0x03;
		idx = 1;
	}

	switch (buf[0]) {
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:
		dev_dbg(&client->dev, "[elan] %d fingers\n", num);
		for (i = 0; i < finger_num; i++) {
			if ((fbits & 0x01)) {
				elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);

				x = x * (LCM_X_RESOLUTION) / X_RESOLUTION;
				y = y * (LCM_Y_RESOLUTION) / Y_RESOLUTION;

				if (!((x <= 0) || (y <= 0) ||
				      (x >= LCM_X_RESOLUTION) ||
				      (y >= LCM_Y_RESOLUTION))) {
					input_report_abs(ts->input_dev,
							 ABS_MT_TOUCH_MAJOR, 8);
					input_report_abs(ts->input_dev,
							 ABS_MT_POSITION_X, x);
					input_report_abs(ts->input_dev,
							 ABS_MT_POSITION_Y, y);
					input_mt_sync(ts->input_dev);
					reported++;
				} // end if border
			}	 // end if finger status
			fbits = fbits >> 1;
			idx += 3;
		} // end for

		if (!reported)
			input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);

		break;

	default:
		dev_err(&client->dev, "[elan] %s: unknown packet type: %0x\n",
			__func__, buf[0]);
		break;
	} // end switch

	return;
}

static void elan_ktf2k_ts_check_work_func(struct work_struct *work)
{

	int rc = 0;

	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);

	if (!chip_reset_flag) {
		chip_reset_flag = 1;
		enable_irq(private_ts->client->irq);
		return;
	}

	elan_ktf2k_ts_hw_reset(private_ts->client);

	rc = __hello_packet_handler(private_ts->client);
	if (rc != 0) {
		printk(KERN_INFO "Receive hello package fail\n");
	}

	enable_irq(private_ts->client->irq);
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	    container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[4 + PACKET_SIZE] = {0};

	chip_reset_flag = 0;

	rc = elan_ktf2k_ts_recv_data(ts->client, buf, 4 + PACKET_SIZE);
	if (rc < 0) {
		printk("[elan] Received the packet Error.\n");
		enable_irq(ts->client->irq);
		return;
	}

	if (buf[0] != 0x52 && buf[0] != 0x55 && buf[0] != 0x66 &&
	    buf[0] != 0x78)
		elan_ktf2k_ts_report_data(ts->client, buf);

	enable_irq(ts->client->irq);

	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "[elan] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	printk(KERN_EMERG "%s \n", __func__);
	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			  IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
			__func__, client->irq);

	return err;
}

/*Arima Edison add to configure and enable the vreg++*/

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0)
		   ? regulator_set_optimum_mode(reg, load_uA)
		   : 0;
}

static int elan_ktf2k_ts_power_on(struct elan_ktf2k_ts_data *data, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(data->vcc_ana, ELAN_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_ana);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (data->pdata->i2c_pull_up) {
		rc =
		    reg_set_optimum_mode_check(data->vcc_i2c, ELAN_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_vcc;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	} else {
		printk(KERN_EMERG "we do not have i2c_pull_up\n");
	}

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
		dev_err(&data->client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_ana) > 0) {
		rc = regulator_set_voltage(data->vcc_ana, ELAN_VTG_MIN_UV,
					   ELAN_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	if (data->pdata->i2c_pull_up) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"Regulator get failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c,
						   ELAN_I2C_VTG_MIN_UV,
						   ELAN_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
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
/*Arima Edison add to enable the vreg--*/

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

	printk(KERN_EMERG "%s * client->addr = 0x%x  name=%s\n", __func__,
	       client->addr, client->name);

	//[Arima Edison] get data from device tree ++
	if (client->dev.of_node) {
		printk(KERN_EMERG "%s  of_node\n", __func__);
		pdata = devm_kzalloc(
		    &client->dev, sizeof(struct elan_ktf2k_i2c_platform_data),
		    GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		err = elan_ktf2k_ts_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else
		pdata = client->dev.platform_data;
	//[Arima Edison] get data from device tree --

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[elan] %s: i2c check functionality error\n",
		       __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR
		       "[elan] %s: allocate elan_ktf2k_ts_data failed\n",
		       __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		printk(KERN_ERR "[elan] %s: create workqueue failed\n",
		       __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);

	//[Arima Edison] add ++
	INIT_DELAYED_WORK(&ts->check_work,
			  elan_ktf2k_ts_check_work_func); // reset if check hang
	//[Arima Edison] add --

	ts->client = client;
	ts->pdata = pdata;

	i2c_set_clientdata(client, ts);

	//[Arima Ediosn] get/set  power/gpio  ++
	elan_ktf2k_ts_regulator_configure(ts, true);
	elan_ktf2k_ts_power_on(ts, true);
	printk(KERN_EMERG "!!!!pdata->intr_gpio=%d, pdata->reset_gpio=%d \n",
	       pdata->intr_gpio, pdata->reset_gpio);

	if (gpio_is_valid(pdata->intr_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->intr_gpio, "elan_intr_gpio");
		if (err)
			printk(KERN_EMERG "unable to request gpio [%d]\n",
			       pdata->intr_gpio);
		err = gpio_direction_input(pdata->intr_gpio);
		client->irq = gpio_to_irq(pdata->intr_gpio);
		printk(KERN_EMERG "elan_intr_gpio=%d, request gpio=%d \n",
		       pdata->intr_gpio, client->irq);
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->reset_gpio, "elan_reset_gpio");
		if (err)
			printk(KERN_EMERG "unable to request gpio [%d]\n",
			       pdata->reset_gpio);
		gpio_direction_output(pdata->reset_gpio, 1);
	}
	//[Arima Ediosn] --
	if (gpio_is_valid(pdata->hw_det_gpio)) {
		touch_panel_type = gpio_get_value(pdata->hw_det_gpio);
		printk(KERN_EMERG "!!!!! touch_panel_type = %d \n",
		       touch_panel_type);
	} else {
		printk(KERN_EMERG "ELAN TOUCH HW DET GPIO IS WRONG \n");
	}

	if (likely(pdata != NULL)) {
		ts->intr_gpio = pdata->intr_gpio;
	}

	fw_err = elan_ktf2k_ts_setup(client);
	if (fw_err == -136) // means we can not poll the int pin at the init
			    // sate, maybe there is no panel
	{
		printk(KERN_INFO "No Elan chip inside, fw_err=%d \n", fw_err);
		err = -ENODEV;
		goto err_create_wq_failed;

	} else if (fw_err < 0) {
		printk(KERN_INFO "No Elan chip inside??, fw_err=%d \n", fw_err);
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"[elan] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	//[Arima Edison] take chip_type and panel_type as condition to set fw++
	if (!touch_panel_type && chip_type == 0x22) {
		file_fw_data = file_fw_data_Truly;
		printk(KERN_EMERG "!!!!! touch : 2227 truly panel \n");
	} else if (!touch_panel_type && chip_type == 0x21) {
		file_fw_data = file_fw_data_Truly_2127;
		printk(KERN_EMERG "!!!!! touch : 2127e truly panel \n");
	} else if (touch_panel_type && chip_type == 0x22) {
		file_fw_data = file_fw_data_Eely;
		printk(KERN_EMERG "!!!!! touch : 2227 eely panel \n");
	} else {
		file_fw_data = file_fw_data_Eely_2127;
		printk(KERN_EMERG "!!!!! touch : 2127 eely panel \n");
	}
	//[Arima Edison] take chip_type and panel_type as condition to set fw--

	private_ts = ts;

	ts->input_dev->name = ELAN_KTF2K_NAME;
	ts->input_dev->id.bustype = BUS_I2C;
	//[Arima Edison] add vendor name and version 20130801++
	ts->input_dev->id.vendor = touch_panel_type;
	ts->input_dev->id.version = FW_VERSION;
	//[Arima Edison] add vendor name and version 20130801--
	ts->input_dev->dev.parent = &client->dev;
	ts->input_dev->open = NULL;  // mxt_input_open;
	ts->input_dev->close = NULL; // mxt_input_close;

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
			     LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			     LCM_Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	// Start Firmware auto Update
	work_lock = 1;
	power_lock = 1;

	/* FW ID & FW VER*/
	if (chip_type == 0x22) // 2227e
	{
		printk("[ELAN]  [7E65]=0x%02x,  [7E64]=0x%02x, "
		       "[0x7E67]=0x%02x, [0x7E66]=0x%02x\n",
		       file_fw_data[0x7E65], file_fw_data[0x7E64],
		       file_fw_data[0x7E67], file_fw_data[0x7E66]);
		New_FW_ID = file_fw_data[0x7E67] << 8 | file_fw_data[0x7E66];
		New_FW_VER = file_fw_data[0x7E65] << 8 | file_fw_data[0x7E64];
	} else // 2127e
	{
		printk("[ELAN]  [7D97]=0x%02x,  [7D96]=0x%02x, "
		       "[0x7D99]=0x%02x, [0x7D98]=0x%02x\n",
		       file_fw_data[0x7D97], file_fw_data[0x7D96],
		       file_fw_data[0x7D99], file_fw_data[0x7D98]);
		New_FW_ID = file_fw_data[0x7D67] << 8 | file_fw_data[0x7D66];
		New_FW_VER = file_fw_data[0x7D65] << 8 | file_fw_data[0x7D64];
	}
	printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n", FW_ID, New_FW_ID);
	printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x,  chip_type=0x%x \n",
	       FW_VERSION, New_FW_VER, chip_type);

	// for firmware auto-upgrade
	if (New_FW_ID == FW_ID) {
		if (New_FW_VER > (FW_VERSION)) {
			Update_FW_One(client, RECOVERY);
			FW_VERSION = New_FW_VER;
			ts->input_dev->id.version = FW_VERSION;
		} else
			printk("We do not NEED update TOUCH FW !! \n");
	} else {
		printk("FW_ID is different!");
	}

	if (!FW_ID) {
		RECOVERY = 0x80;
		Update_FW_One(client, RECOVERY);
		FW_VERSION = New_FW_VER;
		ts->input_dev->id.version = FW_VERSION;
	}

	power_lock = 0;
	work_lock = 0;
	// End Firmware auto Update

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"[elan]%s: unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	dev_info(&client->dev,
		 "[elan] Start touchscreen %s in interrupt mode\n",
		 ts->input_dev->name);

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);
	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#endif

	// Firmware Update++
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG | S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		printk("[ELAN]misc_register failed!!");
	else
		printk("[ELAN]misc_register finished!!");
	// Firmware Update--

	if (chip_type == 0x22)
		elan_ktf2k_clear_ram(private_ts->client);

	tp_sleep_status = 1;

	elan_ktf2k_ts_register_interrupt(ts->client);

	printk(KERN_EMERG "%s finish \n", __func__);

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

	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_parse_dt(struct device *dev,
				  struct elan_ktf2k_i2c_platform_data *pdata)
{

	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "elan,reset-gpio", 0,
						    &pdata->reset_gpio_flags);
	pdata->intr_gpio = of_get_named_gpio_flags(np, "elan,irq-gpio", 0,
						   &pdata->intr_gpio_flags);
	pdata->hw_det_gpio = of_get_named_gpio_flags(np, "elan,hw-det-gpio", 0,
						     &pdata->hw_det_gpio_flags);
	pdata->i2c_pull_up = of_property_read_bool(np, "elan,i2c-pull-up");

	return 0;
}

// static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t
// mesg)
static int elan_ktf2k_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	// if chip in the sleep mode, we do not need to do it
	if (!tp_sleep_status)
		return 0;

	// The power_lock can be removed when firmware upgrade procedure will
	// not be enter into suspend mode.
	if (power_lock)
		return 0;

	disable_irq(client->irq);

	// Sync remaining touch events
	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);

	flush_work(&ts->work);
	cancel_delayed_work_sync(&ts->check_work);
	elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);

	tp_sleep_status = 0;

	return 0;
}

static int elan_ktf2k_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	// if chip in the resume mode, we do not need to do it
	if (tp_sleep_status)
		return 0;

	// The power_lock can be removed when firmware upgrade procedure will
	// not be enter into resume mode.
	if (power_lock)
		return 0;

	elan_ktf2k_ts_hw_reset(private_ts->client);

	rc = __hello_packet_handler(private_ts->client);
	if (rc < 0)
		printk("[elan] %s: hellopacket's receive fail \n", __func__);

	enable_irq(client->irq);

	tp_sleep_status = 1;

	return 0;
}

static const struct dev_pm_ops ktf2k_pm_ops = {
#if (!defined(CONFIG_FB))
    .suspend = elan_ktf2k_ts_suspend, .resume = elan_ktf2k_ts_resume,
#endif
};

/**********Arima Edison modify for device tree ++***********/
static const struct i2c_device_id elan_ktf2k_ts_id[] = {{ELAN_KTF2K_NAME, 0},
							{}};

MODULE_DEVICE_TABLE(i2c, elan_ktf2k_ts_id);

static struct of_device_id elan_ktf2k_ts_match_table[] = {
    {
	.compatible = "elan,ktf2k_ts",
    },
    {},
};

static struct i2c_driver ktf2k_driver = {
    .driver =
	{
	    .name = "elan_ktf2k_ts",
	    .owner = THIS_MODULE,
	    .of_match_table = elan_ktf2k_ts_match_table,
	    .pm = &ktf2k_pm_ops,
	},
    .probe = elan_ktf2k_ts_probe,
    .remove = __devexit_p(elan_ktf2k_ts_remove),
    .id_table = elan_ktf2k_ts_id,
};

module_i2c_driver(ktf2k_driver);
/**********Arima Edison modify for device tree  --***********/

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
