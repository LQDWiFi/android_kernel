/* drivers/input/touchscreen/himax8250.c
 *
 * Copyright (C) 2013 MM Solutions.
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

#include <linux/himax8526a.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/atomic.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#define FIRMWARE_MAX_SIZE (32*1000)
#define FIRMWARE_MIN_SIZE 1
#define HIMAX_I2C_RETRY_TIMES 10

#define MXT_VTG_MIN_UV		2700000
#define MXT_VTG_MAX_UV		3300000

/* Himax TP COMMANDS -> Do not modify the below definition */
#define HX_CMD_NOP			0x00   /* no operation */
#define HX_CMD_DEVICE_ID		0x31   /* get device id */
#define HX_CMD_VERSION_ID		0x32   /* get version id */
#define HX_CMD_SETMICROOFF		0x35   /* set micro on */
#define HX_CMD_SETROMRDY		0x36   /* set flash ready */
#define HX_CMD_TSSLPIN			0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT			0x81   /* set sleep out */
#define HX_CMD_TSSOFF			0x82   /* sense off */
#define HX_CMD_TSSON			0x83   /* sense on */
#define HX_CMD_ROE			0x85   /* read one event */
#define HX_CMD_RAE			0x86   /* read all events */
#define HX_CMD_RLE			0x87   /* read latest event */
#define HX_CMD_CLRES			0x88   /* clear event stack */
#define HX_CMD_TSSWRESET		0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB		0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN		0xDD   /* set cache function */
#define HX_CMD_SETIDLE			0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY		0xF3   /* set idle delay */
#define HX_CMD_MANUALMODE		0x42
#define HX_CMD_FLASH_ENABLE		0x43
#define HX_CMD_FLASH_SET_ADDRESS	0x44
#define HX_CMD_FLASH_WRITE_REGISTER	0x45
#define HX_CMD_FLASH_SET_COMMAND	0x47
#define HX_CMD_FLASH_WRITE_BUFFER	0x48
#define HX_CMD_FLASH_PAGE_ERASE		0x4D
#define HX_CMD_FLASH_SECTOR_ERASE	0x4E
#define HX_CMD_CB			0xCB
#define HX_CMD_EA			0xEA
#define HX_CMD_4A			0x4A
#define HX_CMD_4F			0x4F
#define HX_CMD_B9			0xB9
#define HX_CMD_76			0x76

struct himax_data {
	struct workqueue_struct *himax_wq;
	struct input_dev *input_dev;
	struct mutex mutex;
	struct wake_lock wakelock;
	struct hrtimer timer;
	struct work_struct work;
	struct i2c_client *client;
	int use_irq;
	int fw_ver;
	uint8_t debug_log_level;
	int (*power)(int on);
//	struct early_suspend early_suspend;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t diag_command;
	int16_t diag_data[200];
	uint8_t finger_pressed;
	uint8_t first_pressed;
	int pre_finger_data[2];
	uint8_t suspend_mode;
	uint8_t reset_activate;
	uint8_t himax_command;
	struct himax_i2c_platform_data *pdata;
	struct regulator **vdd;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

static int himax_i2c_read(struct i2c_client *client, uint8_t command,
				uint8_t *data, uint8_t length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = 1,
			 .buf = &command,
		},
		{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = length,
			 .buf = data,
		}
	};

	for (retry = 0; retry < HIMAX_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(20);
	}
	if (retry == HIMAX_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			HIMAX_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

static int himax_i2c_write(struct i2c_client *client, uint8_t command,
			   uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = length + 1,
			 .buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < HIMAX_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(20);
	}

	if (retry == HIMAX_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			HIMAX_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

static int himax_i2c_write_command(struct i2c_client *client, uint8_t command)
{
	return himax_i2c_write(client, command, NULL, 0);
}

static int himax_i2c_master_write(struct i2c_client *client, uint8_t *data,
				  uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length];

	struct i2c_msg msg[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = length,
			 .buf = buf,
		}
	};

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i] = data[loop_i];

	for (retry = 0; retry < HIMAX_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(20);
	}

	if (retry == HIMAX_I2C_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			HIMAX_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int himax_read_dev_err(struct himax_data *ts)
{
	int ret;
	uint8_t data[4] = { 0 };

	ret = himax_i2c_read(ts->client, HX_CMD_DEVICE_ID, data, 3);
	if (ret)
		goto error;
	ret = himax_i2c_read(ts->client, HX_CMD_VERSION_ID, &data[3], 1);
	if (ret)
		goto error;

	dev_dbg(&ts->client->dev,
		"DeviceID: 0x%2.2X 0x%2.2X 0x%2.2X FW ver:0x%2.2X\n",
		data[0], data[1], data[2], data[3]);
	ts->fw_ver = data[3];
	return 0;
error:
	dev_err(&ts->client->dev,
		"failed communication with device during configuration\n");
	return ret;
}

static int himax_load_sensor_config(struct himax_data *ts)
{
	struct himax_ts_config_data *cdata = ts->pdata->cdata;
	int ret;
	char cmd[3] = {0};
static struct himax_ts_config_data himax_config_data = {
	.c1 = { 0x37, 0xFF, 0x08, 0xFF, 0x08 },
	.c2 = { 0x3F, 0x00 },
	.c3 = { 0x62, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c4 = { 0x63, 0x10, 0x00, 0x10, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c5 = { 0x64, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c6 = { 0x65, 0x10, 0x00, 0x10, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c7 = { 0x66, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c8 = { 0x67, 0x10, 0x00, 0x10, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c9 = { 0x68, 0x01, 0x00, 0x01, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 },
	.c10 = { 0x69, 0x10, 0x00, 0x10, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c11 = { 0x6A, 0x01, 0x00, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c12 = { 0x6B, 0x10, 0x00, 0x10, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c13 = { 0x6C, 0x01, 0x00, 0x01, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c14 = { 0x6D, 0x10, 0x00, 0x10, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c15 = { 0x6E, 0x01, 0x00, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c16 = { 0x6F, 0x10, 0x00, 0x10, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c17 = { 0x70, 0x01, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00,
		 0x00 },
	.c18 = { 0x7B, 0x03 },
	.c19 = { 0x7C, 0x00, 0xD8, 0x8C },
	.c20 = { 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00 },
	.c21 = { 0xA4, 0x94, 0x62, 0x94, 0x86 },
	.c22 = { 0xB4, 0x04, 0x01, 0x01, 0x01, 0x01, 0x03, 0x0F, 0x04, 0x07,
		 0x04, 0x07, 0x04, 0x07, 0x00 },
	.c23 = { 0xB9, 0x01, 0x36 },
	.c24 = { 0xBA, 0x00 },
	.c25 = { 0xBB, 0x00 },
	.c26 = { 0xBC, 0x00, 0x00, 0x00, 0x00 },
	.c27 = { 0xBD, 0x04, 0x0C },
	.c28 = { 0xC2, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	.c29 = { 0xC5, 0x0A, 0x1D, 0x00, 0x10, 0x1A, 0x1E, 0x0B, 0x1D, 0x08,
		 0x16 },
	.c30 = { 0xC6, 0x1A, 0x10, 0x1F },
	.c31 = { 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x15, 0x17, 0x17,
		 0x19, 0x19, 0x1F, 0x1F, 0x1B, 0x1B, 0x1D, 0x1D, 0x21, 0x21,
		 0x23, 0x23, 0x25, 0x25, 0x27, 0x27, 0x29, 0x29, 0x2B, 0x2B,
		 0x2D, 0x2D, 0x2F, 0x2F, 0x16, 0x16, 0x18, 0x18, 0x1A, 0x1A,
		 0x20, 0x20, 0x1C, 0x1C, 0x1E, 0x1E, 0x22, 0x22, 0x24, 0x24,
		 0x26, 0x26, 0x28, 0x28, 0x2A, 0x2A, 0x2C, 0x2C, 0x2E, 0x2E,
		 0x30, 0x30, 0x00, 0x00, 0x00 },
	.c32 = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x9F,
		 0x00, 0x00, 0x00 },
	.c33 = { 0xD0, 0x06, 0x01 },
	.c34 = { 0xD3, 0x06, 0x01 },
	.c35 = { 0xD5, 0xA5, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	.c36 = { 0x40, 0x01, 0x5A, 0x5F, 0x00, 0xF0, 0x10, 0x00, 0x00, 0x64,
		 0x0E, 0x0A, 0x10, 0x06, 0x0C, 0x0C, 0x0F, 0x0F, 0x0F, 0x52,
		 0x34, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 0x00, 0x64, 0x08, 0x80, 0x82, 0x85, 0x00, 0x35, 0x25, 0x0F,
		 0x0F, 0x83, 0x3C, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x0F,
		 0x0F, 0x00, 0x12, 0x00, 0x00, 0x10, 0x02, 0x10, 0x64, 0x00,
		 0x00, 0x40, 0x3F, 0x3F, 0x01, 0x14, 0x00, 0x00, 0x00, 0x04,
		 0x03, 0x12, 0x06, 0x06, 0x00, 0x00, 0x00, 0x18, 0x18, 0x05,
		 0x00, 0x00, 0xD8, 0x8C, 0x00, 0x00, 0x42, 0x03, 0x00, 0x00,
		 0x00, 0x00, 0x00, 0x10, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00,
		 0x0C, 0x10, 0x12, 0x20, 0x32, 0x01, 0x04, 0x07, 0x09, 0xB4,
		 0x6E, 0x32, 0x00, 0x0F, 0x1C, 0xA0, 0x25, 0x00, 0x00, 0x03,
		 0x20, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x39, 0x25, 0x00,
		 0x00, 0x00, 0x00, 0x00, 0x11, 0xC7, 0x13 },
};

cdata=&himax_config_data;

	cmd[0] = 0xE3;
	cmd[1] = 0x00;	/* reload disable */
	ret = himax_i2c_master_write(ts->client, cmd, 2);
	if (ret)
		goto error;

	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c1, sizeof((*cdata).c1));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c2, sizeof((*cdata).c2));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c3, sizeof((*cdata).c3));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c4, sizeof((*cdata).c4));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c5, sizeof((*cdata).c5));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c6, sizeof((*cdata).c6));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c7, sizeof((*cdata).c7));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c8, sizeof((*cdata).c8));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c9, sizeof((*cdata).c9));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c10, sizeof((*cdata).c10));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c11, sizeof((*cdata).c11));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c12, sizeof((*cdata).c12));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c13, sizeof((*cdata).c13));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c14, sizeof((*cdata).c14));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c15, sizeof((*cdata).c15));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c16, sizeof((*cdata).c16));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c17, sizeof((*cdata).c17));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c18, sizeof((*cdata).c18));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c19, sizeof((*cdata).c19));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c20, sizeof((*cdata).c20));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c21, sizeof((*cdata).c21));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c22, sizeof((*cdata).c22));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c23, sizeof((*cdata).c23));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c24, sizeof((*cdata).c24));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c25, sizeof((*cdata).c25));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c26, sizeof((*cdata).c26));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c27, sizeof((*cdata).c27));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c28, sizeof((*cdata).c28));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c29, sizeof((*cdata).c29));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c30, sizeof((*cdata).c30));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c31, sizeof((*cdata).c31));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c32, sizeof((*cdata).c32));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c33, sizeof((*cdata).c33));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c34, sizeof((*cdata).c34));
msleep(50);
	if (ret)
		goto error;
	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c35, sizeof((*cdata).c35));
msleep(50);
	if (ret)
		goto error;

	cmd[0] = 0xE1;
	cmd[1] = 0x15;
	ret = himax_i2c_master_write(ts->client, cmd, 2);
	if (ret)
		goto error;

	cmd[0] = 0xD8;
	cmd[1] = 0x00;
	cmd[2] = 0x00;	/* Start addr */
	ret = himax_i2c_master_write(ts->client, cmd, sizeof(cmd));
	if (ret)
		goto error;

	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c36, sizeof((*cdata).c36));
	if (ret)
		goto error;

	cmd[0] = 0xE1;
	cmd[1] = 0x00;
	ret = himax_i2c_master_write(ts->client, cmd, 2);
	if (ret)
		goto error;

	cmd[0] = 0xE1;
	cmd[1] = 0x15;
	ret = himax_i2c_master_write(ts->client, cmd, 2);
	if (ret)
		goto error;

	cmd[0] = 0xD8;
	cmd[1] = 0x00;
	cmd[2] = 0x88;	/* Start addr */
	ret = himax_i2c_master_write(ts->client, cmd, sizeof(cmd));
	if (ret)
		goto error;

	ret = himax_i2c_master_write(ts->client,
				     (*cdata).c36, sizeof((*cdata).c36));
	if (ret)
		goto error;

	cmd[0] = 0xE1;
	cmd[1] = 0x00;
	ret = himax_i2c_master_write(ts->client, cmd, 2);
	if (ret)
		goto error;

	return ret;
error:
	dev_err(&ts->client->dev,
		"failed communication with device during configuration\n");
	return ret;
}

static int himax_poweron(struct himax_data *ts)
{
	int result;
	char cmd[3] = {0};

	result = himax_i2c_write_command(ts->client, HX_CMD_TSSLPOUT);
	if (result)
		goto error;
	msleep(120);

	cmd[0] = HX_CMD_MANUALMODE;
	cmd[1] = 0x02;
	result = himax_i2c_master_write(ts->client, cmd , 2);
	if (result)
		goto error;

	cmd[0] = HX_CMD_SETMICROOFF;
	cmd[1] = 0x02;
	result = himax_i2c_master_write(ts->client, cmd , 2);
	if (result)
		goto error;

	cmd[0] = HX_CMD_SETROMRDY;
	cmd[1] = 0x0F;
	cmd[2] = 0x53;
	result = himax_i2c_master_write(ts->client, cmd , sizeof(cmd));
	if (result)
		goto error;

	cmd[0] = HX_CMD_SET_CACHE_FUN;
	cmd[1] = 0x04;
	cmd[2] = 0x02;
	result = himax_i2c_master_write(ts->client, cmd , sizeof(cmd));
	if (result)
		goto error;

	cmd[0] = HX_CMD_76;
	cmd[1] = 0x01;
	cmd[2] = 0x2D;
	result = himax_i2c_master_write(ts->client, cmd , sizeof(cmd));
	if (result)
		goto error;

	result = himax_load_sensor_config(ts);
	if (result) {
		dev_err(&ts->client->dev,
			"failed sending of configuration to the device\n");
		goto error;
	}
	msleep(100);

	result = himax_i2c_write_command(ts->client, HX_CMD_TSSON);
	if (result)
		goto error;

	msleep(50);

	return 0;

error:
	dev_err(&ts->client->dev, "failed communication with device\n");
	return result;
}

static int himax_enter_manual_mode(struct himax_data *ts)
{
	int result;
	uint8_t cmd = 0x01;

	result = himax_i2c_write(ts->client, HX_CMD_MANUALMODE, &cmd, 1);
	if (result)
		goto fail;

	return 0;
fail:
	dev_err(&ts->client->dev, "failed enter in manual mode\n");
	return result;
}

static int himax_exit_manual_mode(struct himax_data *ts)
{
	int result;
	uint8_t cmd = 0x00;

	result = himax_i2c_write(ts->client, HX_CMD_MANUALMODE, &cmd, 1);
	if (result)
		goto fail;

	return 0;
fail:
	dev_err(&ts->client->dev, "failed exit from manual mode\n");
	return result;
}

static int himax_enter_flash_mode(struct himax_data *ts)
{
	int result;
	uint8_t cmd = 0x01;

	result = himax_i2c_write(ts->client, HX_CMD_FLASH_ENABLE, &cmd, 1);
	if (result)
		goto fail;

	return 0;
fail:
	dev_err(&ts->client->dev, "failed enter in flash mode\n");
	return result;
}

static int himax_exit_flash_mode(struct himax_data *ts)
{
	int result;
	uint8_t cmd = 0x00;

	result = himax_i2c_write(ts->client, HX_CMD_FLASH_ENABLE, &cmd, 1);
	if (result)
		goto fail;

	return 0;
fail:
	dev_err(&ts->client->dev, "failed exit from flash mode\n");
	return result;
}

static int himax_lock_flash(struct himax_data *ts)
{
	int result;
	uint8_t cmd[5];

	/* lock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_ENABLE, cmd, 3);
	if (result)
		goto fail;

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_SET_ADDRESS, cmd, 3);
	if (result)
		goto fail;

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x7D;
	cmd[3] = 0x03;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_WRITE_REGISTER,
				 cmd, 4);
	if (result)
		goto fail;

	result = himax_i2c_write_command(ts->client, HX_CMD_4A);
	if (result)
		goto fail;
	msleep(50);

	/* lock sequence end */
	return 0;
fail:
	dev_err(&ts->client->dev, "failed locking of flash!\n");
	return result;
}

static int himax_unlock_flash(struct himax_data *ts)
{
	int result;
	uint8_t cmd[5];

	/* unlock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_ENABLE, cmd, 3);
	if (result)
		goto fail;

	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_SET_ADDRESS, cmd, 3);
	if (result)
		goto fail;

	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x3D;
	cmd[3] = 0x03;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_WRITE_REGISTER,
				 cmd, 4);
	if (result)
		goto fail;

	result = himax_i2c_write_command(ts->client, HX_CMD_4A);
	if (result)
		goto fail;
	msleep(50);

	/* unlock sequence end */
	return 0;
fail:
	dev_err(&ts->client->dev, "failed unlocking of flash!\n");
	return result;
}

static u8 himax_calc_sw_checksum(struct himax_data *ts, const u8 *data,
				 int data_len)
{
	int result = 0;
	u16 checksum = 0;
	uint8_t cmd[5], last_byte;
	int file_len, last_length, i, k;

	himax_enter_flash_mode(ts);

	file_len = data_len - 2;
	file_len = (file_len + 3) / 4;
	for (i = 0; i < file_len; i++) {
		last_byte = 0;

		cmd[0] = i & 0x1F;
		if (cmd[0] == 0x1F || i == file_len - 1)
			last_byte = 1;

		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		result = himax_i2c_write(ts->client, HX_CMD_FLASH_SET_ADDRESS,
					 cmd, 3);
		if (result)
			goto err;

		result = himax_i2c_write_command(ts->client, 0x46);
		if (result)
			goto err;

		result = himax_i2c_read(ts->client, 0x59, cmd, 4);
		if (result)
			goto err;

		if (i < (file_len - 1)) {
			checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
			if (i == 0)
				dev_err(&ts->client->dev,
					 "himax_marked cmd 0 to 3 "
					 "(first 4 bytes): %d, %d, %d, %d\n",
					 cmd[0], cmd[1], cmd[2], cmd[3]);
		} else {
			dev_err(&ts->client->dev,
					"himax_marked cmd 0 to 3 "
					"(last 4 bytes): %d, %d, %d, %d\n",
					cmd[0], cmd[1], cmd[2], cmd[3]);
			dev_err(&ts->client->dev,
				 "himax_marked, checksum (not last): %d\n",
				 checksum);

			last_length = (((data_len - 2) % 4) > 0) ?
						((data_len - 2) % 4) : 4;

			for (k = 0; k < last_length; k++)
				checksum += cmd[k];

			dev_err(&ts->client->dev,
				 "himax_marked, checksum (final): %d\n",
				 checksum);

			/* Check Success */
			if (data[data_len-1] == (u8)(0xFF & (checksum >> 8)) &&
				data[data_len - 2] == (u8)(0xFF & checksum))
				result = 1;
			else
				result = 0;
		}
	}

	himax_exit_flash_mode(ts);

	return result;
err:
	dev_err(&ts->client->dev, "failed calc of sw checksum!\n");
	return result;
}

static void himax_upgrade_start(struct himax_data *ts, const u8 *data,
				int data_len)
{
	int result;
	int file_len = data_len - 2;
	uint8_t cmd[5], last_byte, pre_page;
	uint8_t checksum_result = 0;
	int i;

	if (file_len < FIRMWARE_MIN_SIZE) {
		dev_err(&ts->client->dev, "too short firmware file!\n");
		goto fail;
	}

	if (file_len > FIRMWARE_MAX_SIZE) {
		dev_err(&ts->client->dev, "too big firmware file!\n");
		goto fail;
	}

	if (ts->pdata->reset) {
		if (ts->debug_log_level)
			dev_err(&ts->client->dev, "reseting the device ...\n");

		ts->pdata->reset();
	}

	result = himax_i2c_write_command(ts->client, HX_CMD_TSSLPOUT);
	if (result) {
		dev_err(&ts->client->dev,
			 "failed communication with device\n");
		goto fail;
	}
	msleep(120);

	result = himax_unlock_flash(ts);
	if (result)
		goto fail;

	cmd[0] = 0x05;
	cmd[1] = 0x00;
	cmd[2] = 0x02;
	result = himax_i2c_write(ts->client, HX_CMD_FLASH_ENABLE, cmd, 3);
	if (result)
		goto fail;

	result = himax_i2c_write(ts->client, HX_CMD_4F, cmd, 0);
	if (result)
		goto fail;
	msleep(50);

	result = himax_enter_manual_mode(ts);
	if (result)
		goto fail;
	result = himax_enter_flash_mode(ts);
	if (result)
		goto fail;

	file_len = (file_len + 3) / 4;
	for (i = 0, pre_page = 0; i < file_len; i++) {
		last_byte = 0;
		cmd[0] = i & 0x1F;
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;

		if ((cmd[0] == 0x1F) || (i == file_len - 1))
			last_byte = 1;

		result = himax_i2c_write(ts->client,
					 HX_CMD_FLASH_SET_ADDRESS, cmd, 3);
		if (result)
			goto fail;

		if (pre_page != cmd[1] || i == 0) {
			pre_page = cmd[1];
			cmd[0] = 0x01;
			cmd[1] = 0x09;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			cmd[0] = 0x01;
			cmd[1] = 0x0D;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			cmd[0] = 0x01;
			cmd[1] = 0x09;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;
		}

		memcpy(cmd, data+(4*i), 4);
		result = himax_i2c_write(ts->client,
					 HX_CMD_FLASH_WRITE_REGISTER, cmd, 4);
		if (result)
			goto fail;

		cmd[0] = 0x01;
		cmd[1] = 0x0D;
		/* cmd[2] = 0x02; */
		result = himax_i2c_write(ts->client,
					 HX_CMD_FLASH_ENABLE, cmd, 2);
		if (result)
			goto fail;

		cmd[0] = 0x01;
		cmd[1] = 0x09;
		/* cmd[2] = 0x02; */
		result = himax_i2c_write(ts->client,
					 HX_CMD_FLASH_ENABLE, cmd, 2);
		if (result)
			goto fail;

		if (last_byte == 1) {
			cmd[0] = 0x01;
			cmd[1] = 0x01;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			cmd[0] = 0x01;
			cmd[1] = 0x05;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			cmd[0] = 0x01;
			cmd[1] = 0x01;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			cmd[0] = 0x01;
			cmd[1] = 0x00;
			/* cmd[2] = 0x02; */
			result = himax_i2c_write(ts->client,
						 HX_CMD_FLASH_ENABLE, cmd, 2);
			if (result)
				goto fail;

			msleep(20);

			if (i == (file_len - 1)) {
				result = himax_exit_flash_mode(ts);
				if (result)
					goto fail;
				result = himax_exit_manual_mode(ts);
				if (result)
					goto fail;

				checksum_result =
					himax_calc_sw_checksum(ts,
							       data, data_len);
				if (checksum_result < 0)
					goto fail;

				result = himax_lock_flash(ts);
				if (result)
					goto fail;
			}
		}
	}

	if (checksum_result)
		dev_err(&ts->client->dev, "Successful upgrading of firmware\n");
	else
		dev_err(&ts->client->dev,
			"Not successful upgrading of firmware\n");
	return;
fail:
	dev_err(&ts->client->dev, "failed upgrading of firmware\n");
	return;
}
static void himax_work_func(struct work_struct *work)
{
	struct himax_data *ts = container_of(work, struct himax_data, work);
	uint8_t buf[128], loop_i, finger_num, finger_pressed;
	u32 checksum;

	memset(buf, 0x00, sizeof(buf));

	mutex_lock(&ts->mutex);
	if (himax_i2c_read(ts->client, HX_CMD_RAE, buf,
			   ts->diag_command ? 128 : 24)) {
		dev_err(&ts->client->dev, "can't read data from chip!\n");
		goto err_workqueue_out;
	} else {
		for (loop_i = 0, checksum = 0;
				loop_i < (ts->diag_command ? 128 : 24);
				loop_i++)
			checksum += buf[loop_i];

		if (checksum == 0 && !ts->reset_activate) {
			msleep(20);
			dev_err(&ts->client->dev,
				 "ESD reset detected, load sensor config.\n");
			(void)himax_poweron(ts);
			ts->reset_activate = 1;
			goto err_workqueue_out;
		} else if (checksum == 0 && ts->reset_activate) {
			msleep(20);
			dev_err(&ts->client->dev,
				 "back from ESD reset, "
				 "but reset by ESD again.\n");
			(void)himax_poweron(ts);
			goto err_workqueue_out;
		} else if (checksum == 0xFF && ts->reset_activate &&
				buf[0] == 0xFF) {
			ts->reset_activate = 0;
			dev_err(&ts->client->dev,
				 "back from ESD reset, ready to serve.\n");
			goto err_workqueue_out;
		}
	}

	if (ts->debug_log_level & 0x1) {
		dev_err(&ts->client->dev, "raw data:\n");
		for (loop_i = 0; loop_i < 24; loop_i++) {
			dev_err(&ts->client->dev, "0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				dev_err(&ts->client->dev, "\n");
		}
	}

	if (ts->diag_command >= 1 && ts->diag_command <= 6) {
		int index = 0;
		/* Header: %x, %x, %x, %x\n",
		 * buf[24], buf[25], buf[26], buf[27] */
		if (buf[24] == buf[25] &&
				buf[24] == buf[26] && buf[24] == buf[27] &&
				buf[24] > 0 && buf[24] < 5) {
			index = (buf[24] - 1) * 50;

			for (loop_i = 0; loop_i < 50; loop_i++) {
				if ((buf[loop_i * 2 + 28] & 0x80) == 0x80) {
					ts->diag_data[index + loop_i] = 0 -
						((buf[loop_i * 2 + 28] << 8 |
						  buf[loop_i * 2 + 29]) &
						  0x4FFF);
				} else
					ts->diag_data[index + loop_i] =
						buf[loop_i * 2 + 28] << 8 |
						buf[loop_i * 2 + 29];
				/* printk("Header: %d, data: %5d\n", buf[24],
					ts->diag_data[loop_i]); */
			}
		}
	}

	if (buf[20] == 0xFF && buf[21] == 0xFF) {
		/* finger leave */
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);

		if (ts->first_pressed == 1) {
			ts->first_pressed = 0;
			if (ts->debug_log_level & 0x2)
				dev_err(&ts->client->dev, "End @ %d, %d\n",
					 ts->pre_finger_data[0],
					 ts->pre_finger_data[1]);
		}

		if (ts->debug_log_level & 0x2)
			dev_err(&ts->client->dev, "Finger leave\n");
	} else {
		finger_num = buf[20] & 0x0F;
		finger_pressed = buf[21];
		for (loop_i = 0; loop_i < 4; loop_i++) {
			if (((finger_pressed >> loop_i) & 1) == 1) {
				int base = loop_i * 4;
				int x = ((buf[base] << 8) | buf[base + 1]);
				int y = ((buf[base + 2] << 8) | buf[base + 3]);
				int w = buf[16 + loop_i];

				x = ts->pdata->inv_x ?
						ts->pdata->abs_x_max - x : x;
				y = ts->pdata->inv_y ?
						ts->pdata->abs_y_max - y : y;

				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_abs(ts->input_dev,
						 ABS_MT_TRACKING_ID, loop_i);
				input_report_abs(ts->input_dev,
						 ABS_MT_TOOL_TYPE,
						 MT_TOOL_FINGER);
				input_report_abs(ts->input_dev,
						 ABS_MT_TOUCH_MAJOR, w);
				input_report_abs(ts->input_dev,
						 ABS_MT_WIDTH_MAJOR, w);
				input_report_abs(ts->input_dev,
						 ABS_MT_PRESSURE, w);
				input_report_abs(ts->input_dev,
						 ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev,
						 ABS_MT_POSITION_Y, y);
				input_mt_sync(ts->input_dev);

				if (!ts->first_pressed) {
					ts->first_pressed = 1;
					if (ts->debug_log_level & 0x2)
						dev_err(&ts->client->dev,
							 "Start @ "
							 "%d , %d\n", x, y);
				}
				if (ts->first_pressed == 1) {
					ts->pre_finger_data[0] = x;
					ts->pre_finger_data[1] = y;
				}

				if (ts->debug_log_level & 0x2)
					dev_err(&ts->client->dev,
						 "Finger move %d => "
						 "X:%d, Y:%d w:%d, z:%d\n",
						 loop_i + 1, x, y, w, w);
			}
		}
	}

	input_sync(ts->input_dev);

err_workqueue_out:
	if (ts->use_irq)
		enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	mutex_unlock(&ts->mutex);
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_data *ts;

	ts = container_of(timer, struct himax_data, timer);
//	queue_work(ts->himax_wq, &ts->work);
	return HRTIMER_NORESTART;
}
static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	struct himax_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->himax_wq, &ts->work);
	return IRQ_HANDLED;
}

static ssize_t himax_register_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	uint8_t data[64] = { 0 }, loop_i;

	mutex_lock(&ts->mutex);
	dev_err(&ts->client->dev, "%x\n", ts->himax_command);
	if (himax_i2c_read(ts->client, ts->himax_command, data, 64) < 0) {
		dev_warn(&ts->client->dev, "%s: read fail\n", __func__);
		mutex_unlock(&ts->mutex);
		return ret;
	}

	ret += snprintf(buf, 20, "command: %x\n", ts->himax_command);
	mutex_unlock(&ts->mutex);
	for (loop_i = 0; loop_i < 64; loop_i++) {
		ret += snprintf(buf + ret, 10, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += snprintf(buf + ret, 2, "\n");
	}
	ret += snprintf(buf + ret, 2, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;
	uint8_t loop_i, base = 5;

	mutex_lock(&ts->mutex);
	if ((buf[0] != 'r' && buf[0] != 'w') ||
			(buf[1] != ':' && buf[2] != 'x')) {
		dev_err(&ts->client->dev, "illegal data format\n");
		goto out;
	}

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memcpy(buf_tmp, buf + 3, 2);

	if (!kstrtoul(buf_tmp, 16, &result))
		ts->himax_command = result;

	for (loop_i = 0; loop_i < 100; loop_i++) {
		if (buf[base] == '\n') {
			if (buf[0] == 'w')
				himax_i2c_write(ts->client, ts->himax_command,
						&write_da[0], length);
			dev_err(&ts->client->dev, "CMD: %x, %x, %d\n",
				 ts->himax_command, write_da[0], length);

			for (veriLen = 0; veriLen < length; veriLen++)
				dev_err(&ts->client->dev, "%x ",
					 *((&write_da[0]) + veriLen));
			dev_err(&ts->client->dev, "\n");
			goto out;
		}
		if (buf[base + 1] == 'x') {
			buf_tmp[4] = '\n';
			buf_tmp[5] = '\0';
			memcpy(buf_tmp, buf + base + 2, 2);
			if (!kstrtoul(buf_tmp, 16, &result))
				write_da[loop_i] = result;
			length++;
		}
		base += 4;
	}

out:
	mutex_unlock(&ts->mutex);
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
		   himax_register_show, himax_register_store);


static ssize_t himax_vendor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	ssize_t ret = 0;

	mutex_lock(&ts->mutex);
	snprintf(buf, 20, "%s_%#x\n", HIMAX8526A_NAME, ts->fw_ver);
	mutex_unlock(&ts->mutex);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, himax_vendor_show, NULL);


static ssize_t himax_debug_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	size_t count = 0;

	mutex_lock(&ts->mutex);
	count += snprintf(buf, 10, "%d\n", ts->debug_log_level);
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);

	mutex_lock(&ts->mutex);
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - '0';
	mutex_unlock(&ts->mutex);

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
		   himax_debug_level_show, himax_debug_level_dump);

static ssize_t himax_diag_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	size_t count = 0;
	uint8_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutex_lock(&ts->mutex);
	if (ts->diag_command == 0) {
		count += snprintf(buf + count, 40,
				  "Command = 0. Noting to show ...\n");
		goto end;
	}

	if (ts->diag_command < 1 || ts->diag_command > 6) {
		count += snprintf(buf + count, 40,
				  "Illegal diag command. Noting to show ...\n");
		goto end;
	}

	mutual_num = ts->x_channel * ts->y_channel;
	self_num = ts->x_channel + ts->y_channel;
	width = ts->x_channel;
	count += snprintf(buf + count, 40, "Channel: %4d, %4d\n\n",
			 ts->x_channel, ts->y_channel);

	if (ts->diag_command < 3) {
		for (loop_i = 0; loop_i < 150 && loop_i < mutual_num;
				loop_i++) {
			count += snprintf(buf + count, 6, "%4d",
					 ts->diag_data[loop_i]);
			if ((loop_i % 15) == 14)
				count += snprintf(buf + count, 2, "\n");
		}
		count += snprintf(buf + count, 2, "\n");
		for (loop_i = 150; loop_i < 200 && loop_i < (self_num + 150);
				loop_i++) {
			count += snprintf(buf + count, 6, "%4d",
					 ts->diag_data[loop_i]);
			if ((loop_i % 15) == 14)
				count += snprintf(buf + count, 2, "\n");
		}
	} else if (ts->diag_command > 4) {
		for (loop_i = 150; loop_i < 200 && loop_i < (self_num + 150);
				loop_i++) {
			count += snprintf(buf + count, 6, "%4d",
					 ts->diag_data[loop_i]);
			if ((loop_i % 15) == 14)
				count += snprintf(buf + count, 2, "\n");
		}
	} else {
		for (loop_i = 0; loop_i < 150 && loop_i < mutual_num;
				loop_i++) {
			count += snprintf(buf + count, 6, "%4d",
					 ts->diag_data[loop_i]);
			if ((loop_i % 15) == 14)
				count += snprintf(buf + count, 2, "\n");
		}
	}

end:
	mutex_unlock(&ts->mutex);
	return count;
}

static ssize_t himax_diag_dump(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0xFC;
	const uint8_t command_ec_128_raw_baseline_flag = 0x02 |
			command_ec_128_raw_flag;
	uint8_t new_command[2] = {0x91, 0x00};

	mutex_lock(&ts->mutex);
	dev_dbg(&ts->client->dev, "entered, buf[0]=%c.\n", buf[0]);
	if (buf[0] == '1' || buf[0] == '3' || buf[0] == '5') {
		new_command[1] = command_ec_128_raw_baseline_flag;
		himax_i2c_master_write(ts->client, new_command,
				       sizeof(new_command));
		ts->diag_command = buf[0] - '0';
	} else if (buf[0] == '2' || buf[0] == '4' || buf[0] == '6') {
		new_command[1] = command_ec_128_raw_flag;
		himax_i2c_master_write(ts->client, new_command,
				       sizeof(new_command));
		ts->diag_command = buf[0] - '0';
	} else {
		new_command[1] = command_ec_24_normal_flag;
		himax_i2c_master_write(ts->client, new_command,
				       sizeof(new_command));
		ts->diag_command = 0;
	}

	mutex_unlock(&ts->mutex);
	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
		   himax_diag_show, himax_diag_dump);

static ssize_t himax_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	size_t count = 0;

	mutex_lock(&ts->mutex);
	if (ts->reset_activate)
		count += snprintf(buf, 40,
				  "Resetting touch chip in progress.\n");
	else
		count += snprintf(buf, 40,
				  "Reset complete or not trigger yet.\n");
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t himax_reset_set(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&ts->mutex);
	if (buf[0] == '1' && ts->pdata->reset) {
		if (ts->use_irq)
			disable_irq_nosync(ts->client->irq);
		else
			hrtimer_cancel(&ts->timer);

		ret = cancel_work_sync(&ts->work);
		if (ret && ts->use_irq)
			enable_irq(ts->client->irq);

		dev_err(&ts->client->dev, "Now reset the Touch chip.\n");

		if (ts->pdata->reset)
			ts->pdata->reset();

		if (ts->use_irq)
			enable_irq(ts->client->irq);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0),
				      HRTIMER_MODE_REL);
	}
	mutex_unlock(&ts->mutex);
	return count;
}

static DEVICE_ATTR(reset, (S_IWUSR|S_IRUGO),
		   himax_reset_show, himax_reset_set);


static ssize_t himax_firmware_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	size_t count = 0;

	mutex_lock(&ts->mutex);
	(void)himax_read_dev_err(ts);
	count += snprintf(buf, 15, "FW ver:0x%2.2X\n", ts->fw_ver);
	mutex_unlock(&ts->mutex);

	return count;
}

static ssize_t himax_firmware_set(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct himax_data *ts = i2c_get_clientdata(client);
	const struct firmware *upgrade_fw;
	char filename[128];
	int result = 0;

	if (count <= 2) {
		dev_err(&client->dev, "filename too short!\n");
		goto out;
	}

	memset(filename, 0, sizeof(filename));
	strlcpy(filename, buf, (count-1)%sizeof(filename));

	wake_lock(&ts->wakelock);
	mutex_lock(&ts->mutex);

	dev_err(&client->dev, "started upgrade from file(%s)!\n", filename);
	result = request_firmware(&upgrade_fw, filename, &client->dev);
	if (result < 0) {
		dev_err(&client->dev, "%s firmware request failed(%d)\n",
			filename, result);
	} else {
		/* check and start upgrade */
		if (ts->use_irq)
			disable_irq(ts->client->irq);
		else
			hrtimer_cancel(&ts->timer);

		himax_upgrade_start(ts, upgrade_fw->data, upgrade_fw->size);
		release_firmware(upgrade_fw);

		if (ts->pdata->reset)
			ts->pdata->reset();

		if (ts->use_irq)
			enable_irq(ts->client->irq);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0),
				      HRTIMER_MODE_REL);
	}

out:
	mutex_unlock(&ts->mutex);
	wake_unlock(&ts->wakelock);
	return count;
}

static DEVICE_ATTR(ts_fw, (S_IWUSR|S_IRUGO),
		   himax_firmware_show, himax_firmware_set);

static int himax_sysfs_init(struct himax_data *ts)
{
	int ret;

	ret = device_create_file(&ts->client->dev, &dev_attr_debug_level);
	if (ret) {
        printk("Pawan sysfs entry for debug level failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for debug level failed\n");
		goto error_dev_file_debug_level;
	}

	ret = device_create_file(&ts->client->dev, &dev_attr_vendor);
	if (ret) {
        printk("Pawan sysfs entry for vendor information failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for vendor information failed\n");
		goto error_rm_dev_file_debug_level;
	}

	ret = device_create_file(&ts->client->dev, &dev_attr_diag);
	if (ret) {
        printk("Pawan sysfs entry for diag failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for diag failed\n");
		goto error_rm_dev_file_vendor;
	}

	ret = device_create_file(&ts->client->dev, &dev_attr_register);
	if (ret) {
        printk("Pawan sysfs entry for register information failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for register information failed\n");
		goto error_rm_dev_file_diag;
	}

	ret = device_create_file(&ts->client->dev, &dev_attr_reset);
	if (ret) {
        printk("Pawan sysfs entry for reset failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for reset failed\n");
		goto error_rm_dev_file_register;
	}

	ret = device_create_file(&ts->client->dev, &dev_attr_ts_fw);
	if (ret) {
        printk("Pawan sysfs entry for firmware failed\n");
		dev_err(&ts->client->dev,
			"sysfs entry for firmware failed\n");
		goto error_rm_dev_file_firmware;
	}

	return 0;
error_rm_dev_file_firmware:
	device_remove_file(&ts->client->dev, &dev_attr_reset);
error_rm_dev_file_register:
	device_remove_file(&ts->client->dev, &dev_attr_register);
error_rm_dev_file_diag:
	device_remove_file(&ts->client->dev, &dev_attr_diag);
error_rm_dev_file_vendor:
	device_remove_file(&ts->client->dev, &dev_attr_vendor);
error_rm_dev_file_debug_level:
	device_remove_file(&ts->client->dev, &dev_attr_debug_level);
error_dev_file_debug_level:
	return ret;
}

static void himax_sysfs_deinit(struct himax_data *ts)
{
	device_remove_file(&ts->client->dev, &dev_attr_ts_fw);
	device_remove_file(&ts->client->dev, &dev_attr_reset);
	device_remove_file(&ts->client->dev, &dev_attr_register);
	device_remove_file(&ts->client->dev, &dev_attr_diag);
	device_remove_file(&ts->client->dev, &dev_attr_vendor);
	device_remove_file(&ts->client->dev, &dev_attr_debug_level);
}

static int himax_power_device(struct himax_data *ts, bool on)
{
	int rc = 0, i;
	const struct himax_regulator *reg_info =
			ts->pdata->regulator_info;
	u8 num_reg = ts->pdata->num_regulators;

	if (!reg_info) {
        printk("Pawan regulator pdata not specified\n");
		dev_err(&ts->client->dev, "regulator pdata not specified\n");
		return -EINVAL;
	}

	if (on == false) /* Turn off the regulators */
		goto ts_reg_disable;

	ts->vdd = kzalloc(num_reg * sizeof(struct regulator *), GFP_KERNEL);
	if (!ts->vdd) {
        printk("Pawan unable to allocate memory\n");
		dev_err(&ts->client->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < num_reg; i++) {
		ts->vdd[i] = regulator_get(&ts->client->dev, reg_info[i].name);
		if (IS_ERR(ts->vdd[i])) {
			rc = PTR_ERR(ts->vdd[i]);
            printk("Pawan regulator_get [%s] failed rc=%d\n",reg_info[i].name, rc);
			dev_err(&ts->client->dev,
				"regulator_get [%s] failed rc=%d\n",
				reg_info[i].name, rc);
			goto error_vdd;
		}

		if (regulator_count_voltages(ts->vdd[i]) > 0) {
			rc = regulator_set_voltage(ts->vdd[i],
						   reg_info[i].min_uV,
						   reg_info[i].max_uV);
			if (rc) {
                printk("Pawan regulator_set_voltage [%s] failed rc =%d\n",reg_info[i].name, rc);
				dev_err(&ts->client->dev,
					"regulator_set_voltage [%s] "
					"failed rc =%d\n",
					reg_info[i].name, rc);
				regulator_put(ts->vdd[i]);
				goto error_vdd;
			}

			rc = regulator_set_optimum_mode(ts->vdd[i],
						reg_info[i].hpm_load_uA);
			if (rc < 0) {
                printk("Pawan regulator_set_optimum_mode [%s] failed rc=%d\n", reg_info[i].name, rc);
				dev_err(&ts->client->dev,
					"regulator_set_optimum_mode [%s] failed"
					" rc=%d\n", reg_info[i].name, rc);

				regulator_set_voltage(ts->vdd[i], 0,
						      reg_info[i].max_uV);
				regulator_put(ts->vdd[i]);
				goto error_vdd;
			}
		}

		rc = regulator_enable(ts->vdd[i]);
		if (rc) {
            printk("Pawan regulator_enable [%s] failed rc =%d\n",reg_info[i].name, rc);
			dev_err(&ts->client->dev,
				"regulator_enable [%s] failed rc =%d\n",
				reg_info[i].name, rc);
			if (regulator_count_voltages(ts->vdd[i]) > 0) {
				regulator_set_optimum_mode(ts->vdd[i], 0);
				regulator_set_voltage(ts->vdd[i], 0,
						      reg_info[i].max_uV);
			}
			regulator_put(ts->vdd[i]);
			goto error_vdd;
		}
	}

	return rc;

ts_reg_disable:
	i = ts->pdata->num_regulators;
error_vdd:
	while (--i >= 0) {
		if (regulator_count_voltages(ts->vdd[i]) > 0) {
			regulator_set_voltage(ts->vdd[i], 0,
					      reg_info[i].max_uV);
			regulator_set_optimum_mode(ts->vdd[i], 0);
		}
		regulator_disable(ts->vdd[i]);
		regulator_put(ts->vdd[i]);
	}
	kfree(ts->vdd);
	return rc;
}

static int himax_parse_dt(struct device *dev, struct himax_i2c_platform_data *pdata)
{
	int rc;
	struct device_node  *np = dev->of_node;



        rc = of_property_read_u32(np, "msm,abs_x_min", &pdata->abs_x_min);
        if (rc) {
                dev_err(dev, "Failed to read abs_x_min x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,abs_x_max", &pdata->abs_x_max);
        if (rc) {
                dev_err(dev, "Failed to read abs_x_max x\n");
                return -EINVAL;
        }

        rc = of_property_read_u32(np, "msm,abs_y_min", &pdata->abs_y_min);
        if (rc) {
                dev_err(dev, "Failed to read abs_y_min x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,abs_y_max", &pdata->abs_y_max);
        if (rc) {
                dev_err(dev, "Failed to read abs_y_max x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,abs_pressure_min", &pdata->abs_pressure_min);
        if (rc) {
                dev_err(dev, "Failed to read abs_pressure_min x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,abs_pressure_max", &pdata->abs_pressure_max);
        if (rc) {
                dev_err(dev, "Failed to read abs_pressure_max x\n");
                return -EINVAL;
        }

        rc = of_property_read_u32(np, "msm,abs_width_min", &pdata->abs_width_min);
        if (rc) {
                dev_err(dev, "Failed to read abs_width_min x\n");
                return -EINVAL;
        }

        rc = of_property_read_u32(np, "msm,abs_width_max", &pdata->abs_width_max);
        if (rc) {
                dev_err(dev, "Failed to read abs_width_max x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,inv_x", &pdata->inv_x);
        if (rc) {
                dev_err(dev, "Failed to read inv_x x\n");
                return -EINVAL;
        }
        rc = of_property_read_u32(np, "msm,inv_y", &pdata->inv_y);
        if (rc) {
                dev_err(dev, "Failed to read inv_y x\n");
                return -EINVAL;
        }


	pdata->reset_gpio = of_get_named_gpio_flags(np, "msm,reset-gpio",
				0, &pdata->reset_gpio_flags);

return 0;

}



static int himax_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret = 0, err = 0, error;
	struct himax_data *ts;
	struct himax_i2c_platform_data *pdata;
	uint8_t data[5] = { 0 };



	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct himax_i2c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		error = himax_parse_dt(&client->dev, pdata);

	} else

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "platform data is null.\n");

		goto err_platform_data_null;
	}

	ts = kzalloc(sizeof(struct himax_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev, "allocate himax_data failed\n");

		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->power = pdata->power;
	ts->pdata = pdata;
	ts->reset_activate = 0;



	gpio_direction_output(pdata->reset_gpio, 0);
	msleep(30);
	gpio_direction_output(pdata->reset_gpio, 1);


	ret = himax_poweron(ts);
	if (ret) {
		dev_err(&client->dev, "power on procedure failed\n");
		goto err_power_failed;
	}


	ts->fw_ver = data[3];
	himax_i2c_read(ts->client, HX_CMD_EA, &data[0], 2);
	ts->x_channel = data[0];
	ts->y_channel = data[1];

	ts->himax_wq = create_singlethread_workqueue("himax_wq");
	if (!ts->himax_wq) {
		dev_err(&client->dev, "Unable to create workqueue\n");
		goto err_create_wq_failed;
	}
	INIT_WORK(&ts->work, himax_work_func);

	mutex_init(&ts->mutex);
	wake_lock_init(&ts->wakelock, WAKE_LOCK_SUSPEND, "himax_wakelock");

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");

		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "himax-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	dev_err(&client->dev,
		 "input_set_abs_params: "
		 "mix_x %d, max_x %d, min_y %d, max_y %d\n",
		 pdata->abs_x_min, pdata->abs_x_max,
		 pdata->abs_y_min, pdata->abs_y_max);


	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		     pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		     pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		     pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 10, 0 , 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		     pdata->abs_width_min, pdata->abs_width_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 4, 0, 0);

	dev_err(&client->dev, "Register input device\n");
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register input device\n");
		goto err_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	ret = himax_sysfs_init(ts);
	if (ret) {
		dev_err(&client->dev, "Unable to create sysfs entries\n");

		goto err_sysfs_entries_failed;
	}

	if (client->irq) {
		ts->use_irq = 1;
		ret = request_irq(client->irq, himax_ts_irq_handler,
				  IRQF_TRIGGER_LOW, client->name, ts);
		if (ret == 0)
			dev_err(&client->dev, "irq enabled at qpio: %d\n",
				 client->irq);
		else {
			ts->use_irq = 0;
			dev_err(&client->dev, "request_irq failed\n");
		}
	} else {
		dev_err(&client->dev,
			 "client->irq is empty, use polling mode.\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		dev_err(&client->dev, "polling mode enabled\n");
	}

	dev_err(&client->dev, "Successful registration\n");

	return 0;

err_sysfs_entries_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	wake_lock_destroy(&ts->wakelock);
	mutex_destroy(&ts->mutex);
	destroy_workqueue(ts->himax_wq);

err_create_wq_failed:
//err_reading_fw:
err_power_failed:
	if (ts->pdata->regulator_info)
		himax_power_device(ts, false);

//err_regulators_failed:
	i2c_set_clientdata(client, NULL);

err_platform_data_null:
	kfree(ts);

err_alloc_data_failed:
//err_check_functionality_failed:
	return ret;

}

static int himax_remove(struct i2c_client *client)
{
	struct himax_data *ts = i2c_get_clientdata(client);

	himax_sysfs_deinit(ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	destroy_workqueue(ts->himax_wq);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);

	if (ts->pdata->regulator_info)
		himax_power_device(ts, false);

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	i2c_set_clientdata(client, NULL);
	wake_lock_destroy(&ts->wakelock);
	mutex_destroy(&ts->mutex);
	kfree(ts);

	return 0;
}

static int himax_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	uint8_t data = 0x01;
	struct himax_data *ts = i2c_get_clientdata(client);
	uint8_t new_command[2] = {0x91, 0x00};

	if (ts->suspend_mode)
		return 0;

	himax_i2c_master_write(ts->client, new_command, sizeof(new_command));

	dev_dbg(&client->dev, "diag_command = %d\n", ts->diag_command);

	if (ts->use_irq)
		disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	if (ret) {
		if (ts->use_irq)
			enable_irq(client->irq);
	}

	himax_i2c_write_command(ts->client, HX_CMD_TSSOFF);
	msleep(120);
	himax_i2c_write_command(ts->client, HX_CMD_TSSLPIN);
	msleep(120);
	himax_i2c_write(ts->client, HX_CMD_SETDEEPSTB, &data, 1);

	ts->first_pressed = 0;
	ts->suspend_mode = 1;
	if (ts->debug_log_level)
		dev_err(&client->dev, "device suspended\n");

	return 0;
}

static int himax_resume(struct i2c_client *client)
{
	uint8_t data[2] = { 0 };
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_128_raw_baseline_flag = 0x02 |
			command_ec_128_raw_flag;
	uint8_t new_command[2] = {0x91, 0x00};

	struct himax_data *ts = i2c_get_clientdata(client);

	if (!ts->suspend_mode)
		return 0;

	data[0] = 0x00;
	himax_i2c_write(ts->client, HX_CMD_SETDEEPSTB, &data[0], 1);
	udelay(100);

	data[0] = HX_CMD_MANUALMODE;
	data[1] = 0x02;
	himax_i2c_master_write(ts->client, data, sizeof(data));

	himax_i2c_write_command(ts->client, HX_CMD_TSSLPOUT);
	msleep(50);

	data[0] = 0x02;
	himax_i2c_write(ts->client, HX_CMD_SETMICROOFF, &data[0], 1);

	data[0] = 0x0F;
	data[1] = 0x53;
	himax_i2c_write(ts->client, HX_CMD_SETROMRDY, &data[0], 2);

	data[0] = 0x04;
	data[1] = 0x02;
	himax_i2c_write(ts->client, HX_CMD_SET_CACHE_FUN, &data[0], 2);

	himax_i2c_write_command(ts->client, HX_CMD_TSSON);
	dev_dbg(&client->dev, "diag_command = %d\n", ts->diag_command);

	msleep(20);
	if (ts->diag_command == 1 || ts->diag_command == 3 ||
			ts->diag_command == 5) {
		new_command[1] = command_ec_128_raw_baseline_flag;
		himax_i2c_master_write(ts->client, new_command,
				       sizeof(new_command));
	} else if (ts->diag_command == 2 || ts->diag_command == 4 ||
			ts->diag_command == 6) {
		new_command[1] = command_ec_128_raw_flag;
		himax_i2c_master_write(ts->client, new_command,
				       sizeof(new_command));
	}

	ts->suspend_mode = 0;

	if (ts->use_irq)
		enable_irq(client->irq);
	if (ts->debug_log_level)
		dev_err(&client->dev, "device resumed\n");
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_data *ts;
	ts = container_of(h, struct himax_data, early_suspend);
	himax_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_data *ts;
	ts = container_of(h, struct himax_data, early_suspend);
	himax_resume(ts->client);

}
#endif

static const struct i2c_device_id himax_ts_id[] = {
	{ HIMAX8526A_NAME, 0 },
	{}
};

static struct of_device_id himax_match_table[] = {
	{ .compatible = "msm,himax",},
	{ },
};

static struct i2c_driver himax_driver = {
	.id_table	= himax_ts_id,
	.probe		= himax_probe,
	.remove		= himax_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= himax_suspend,
	.resume		= himax_resume,
#endif
	.driver		= {
		.name = HIMAX8526A_NAME,
		.owner = THIS_MODULE,
		.of_match_table = himax_match_table,
	},
};

static int __init himax_init(void)
{
	return i2c_add_driver(&himax_driver);
}

static void __exit himax_exit(void)
{
	i2c_del_driver(&himax_driver);
}

module_init(himax_init);
module_exit(himax_exit);

MODULE_DESCRIPTION("Himax8526a driver");
MODULE_LICENSE("GPL");
