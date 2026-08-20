// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 *
 * lt7911exc type-c/DP to MIPI CSI-2 bridge driver.
 *
 * Author: Jianwei Fan <jianwei.fan@rock-chips.com>
 *
 * LT7911EXC variant based on Rockchip's LT7911UXC V4L2 bridge. The firmware
 * update node follows Lontium's LT2408 Linux I2C Driver v2.0 CRC flow.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/compat.h>
#include <media/v4l2-controls_rockchip.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

#define I2C_MAX_XFER_SIZE	128
#define POLL_INTERVAL_MS	1000

#define LT7911EXC_FW_SIZE		(64 * 1024)
#define LT7911EXC_FW_CRC_ADDR		(LT7911EXC_FW_SIZE - 4)
#define LT7911EXC_FW_PAGE_SIZE		32
#define LT7911EXC_FW_FILE \
	"violoop/lt7911exc/LT7911EXC_U2Q02CEM_FCS01x870BF9(CSx600856_CRCx74101F2A).bin"

#define LT7911EXC_LINK_FREQ_1250M	1250000000
#define LT7911EXC_LINK_FREQ_860M	860000000
#define LT7911EXC_LINK_FREQ_700M	700000000
#define LT7911EXC_LINK_FREQ_400M	400000000
#define LT7911EXC_LINK_FREQ_300M	300000000
#define LT7911EXC_LINK_FREQ_200M	200000000
#define LT7911EXC_LINK_FREQ_100M	100000000

#define LT7911EXC_PIXEL_RATE		800000000

#define LT7911EXC_CHIPID	0x2408
/* LT7911EXC register guide: E1:00 is high, E1:01 is low. */
#define CHIPID_REGH		0xe100
#define CHIPID_REGL		0xe101
#define I2C_EN_REG		0xe0ee
#define I2C_ENABLE		0x1
#define I2C_DISABLE		0x0

#define LT7911EXC_VIDEO_STATUS		0xe084
#define LT7911EXC_VIDEO_OFF		0x00
#define LT7911EXC_VIDEO_READY		0x01
#define LT7911EXC_VIDEO_LOST		0x02
#define LT7911EXC_AUDIO_READY		0x03

#define HTOTAL_H		0xe088
#define HTOTAL_L		0xe089
#define HACT_H			0xe08c
#define HACT_L			0xe08d

#define VTOTAL_H		0xe08a
#define VTOTAL_L		0xe08b
#define VACT_H			0xe08e
#define VACT_L			0xe08f

#define PCLK_H			0xe085
#define PCLK_M			0xe086
#define PCLK_L			0xe087

#define BYTE_PCLK_H		0xe0a0
#define BYTE_PCLK_M		0xe0a1
#define BYTE_PCLK_L		0xe0a2
#define LT7911EXC_MIPI_PORT_LANE		0xe0a3
#define LT7911EXC_MIPI_FORMAT		0xe0a4
#define LT7911EXC_MIPI_FORMAT_YUV422_8BIT	0x00

#define HFP_H			0xe090
#define HFP_L			0xe091
#define HSW_H			0xe092
#define HSW_L			0xe093
#define HBP_H			0xe094
#define HBP_L			0xe095
#define VFP_H			0xe096
#define VFP_L			0xe097
#define VSW_H			0xe098
#define VSW_L			0xe099
#define VBP_H			0xe09a
#define VBP_L			0xe09b

//CPHY timing
#define CLK_ZERO_REG		0xf9a7
#define CLK_PRE_REG		0xf9a8
#define CLK_POST_REG		0xf9a9
#define HS_LPX_REG		0xf9a4
#define HS_PREP_REG		0xf9a5
#define HS_TRAIL		0xf9a6
#define HS_RQST_PRE_REG		0xf98a

#ifdef LT7911EXC_OUT_RGB
#define LT7911EXC_MEDIA_BUS_FMT		MEDIA_BUS_FMT_BGR888_1X24
#else
#define LT7911EXC_MEDIA_BUS_FMT		MEDIA_BUS_FMT_UYVY8_2X8
#endif

#define LT7911EXC_NAME			"LT7911EXC"

static const s64 link_freq_menu_items[] = {
	LT7911EXC_LINK_FREQ_1250M,
	LT7911EXC_LINK_FREQ_860M,
	LT7911EXC_LINK_FREQ_700M,
	LT7911EXC_LINK_FREQ_400M,
	LT7911EXC_LINK_FREQ_300M,
	LT7911EXC_LINK_FREQ_200M,
	LT7911EXC_LINK_FREQ_100M,
};

struct lt7911exc {
	struct v4l2_mbus_config_mipi_csi2 bus;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	struct i2c_client *i2c_client;
	struct mutex confctl_mutex;
	struct mutex fw_lock;
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct delayed_work delayed_work_hotplug;
	struct delayed_work delayed_work_res_change;
	struct v4l2_dv_timings timings;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *plugin_det_gpio;
	struct gpio_desc *power_gpio;
	struct work_struct work_i2c_poll;
	struct timer_list timer;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	const struct lt7911exc_mode *cur_mode;
	const struct lt7911exc_mode *support_modes;
	u32 cfg_num;
	struct v4l2_fwnode_endpoint bus_cfg;
	bool nosignal;
	bool enable_hdcp;
	bool is_audio_present;
	bool power_on;
	bool initialized;
	bool runtime_registered;
	bool fw_update_active;
	int plugin_irq;
	u32 irq_count;
	u32 mipi_freq_idx;
	u32 mbus_fmt_code;
	u32 module_index;
	u32 audio_sampling_rate;
};

static const struct v4l2_dv_timings_cap lt7911exc_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 800000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

struct lt7911exc_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
};

static struct rkmodule_csi_dphy_param rk3588_dcphy_param = {
	.vendor = PHY_VENDOR_SAMSUNG,
	.lp_vol_ref = 3,
	.lp_hys_sw = {3, 0, 3, 0},
	.lp_escclk_pol_sel = {1, 1, 0, 0},
	.skew_data_cal_clk = {0, 0, 0, 0},
	.clk_hs_term_sel = 0,
	.data_hs_term_sel = {0, 0, 0, 0},
	.reserved = {0},
};

static const struct lt7911exc_mode supported_modes_dphy[] = {
	{
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 4400,
		.vts_def = 2250,
		.mipi_freq_idx = 0,
	}, {
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 2200,
		.vts_def = 1125,
		.mipi_freq_idx = 4,
	}, {
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 2160,
		.vts_def = 1250,
		.mipi_freq_idx = 4,
	}, {
		.width = 1280,
		.height = 960,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1712,
		.vts_def = 994,
		.mipi_freq_idx = 5,
	}, {
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1650,
		.vts_def = 750,
		.mipi_freq_idx = 5,
	}, {
		.width = 800,
		.height = 600,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1056,
		.vts_def = 628,
		.mipi_freq_idx = 6,
	}, {
		.width = 720,
		.height = 576,
		.max_fps = {
			.numerator = 10000,
			.denominator = 500000,
		},
		.hts_def = 864,
		.vts_def = 625,
		.mipi_freq_idx = 6,
	}, {
		.width = 720,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 858,
		.vts_def = 525,
		.mipi_freq_idx = 6,
	},
};

static const struct lt7911exc_mode supported_modes_cphy[] = {
	{
		.width = 5120,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 5500,
		.vts_def = 2250,
		.mipi_freq_idx = 1,
	}, {
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 4400,
		.vts_def = 2250,
		.mipi_freq_idx = 2,
	}, {
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 2200,
		.vts_def = 1125,
		.mipi_freq_idx = 5,
	}, {
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1650,
		.vts_def = 750,
		.mipi_freq_idx = 6,
	}, {
		.width = 720,
		.height = 576,
		.max_fps = {
			.numerator = 10000,
			.denominator = 500000,
		},
		.hts_def = 864,
		.vts_def = 625,
		.mipi_freq_idx = 6,
	}, {
		.width = 720,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 858,
		.vts_def = 525,
		.mipi_freq_idx = 6,
	},
};

static void lt7911exc_format_change(struct v4l2_subdev *sd);
static int lt7911exc_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd);
static int lt7911exc_s_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings);

static inline struct lt7911exc *to_lt7911exc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lt7911exc, sd);
}

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct i2c_client *client = lt7911exc->i2c_client;
	int err;
	u8 buf[2] = { 0xFF, reg >> 8};
	u8 reg_addr = reg & 0xFF;
	struct i2c_msg msgs[3];

	if (READ_ONCE(lt7911exc->fw_update_active)) {
		memset(values, 0, n);
		return;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = 0;
	msgs[1].len = 1;
	msgs[1].buf = &reg_addr;

	msgs[2].addr = client->addr;
	msgs[2].flags = I2C_M_RD;
	msgs[2].len = n;
	msgs[2].buf = values;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed\n",
				__func__, reg, client->addr);
	}

	if (!debug)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x\n",
			reg, values[0]);
		break;
	case 2:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x%02x\n",
			reg, values[1], values[0]);
		break;
	case 4:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x%02x%02x%02x\n",
			reg, values[3], values[2], values[1], values[0]);
		break;
	default:
		v4l2_info(sd, "I2C read %d bytes from address 0x%04x\n",
			n, reg);
	}
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct i2c_client *client = lt7911exc->i2c_client;
	int err, i;
	struct i2c_msg msgs[2];
	u8 data[I2C_MAX_XFER_SIZE];
	u8 buf[2] = { 0xFF, reg >> 8};

	if (READ_ONCE(lt7911exc->fw_update_active))
		return;

	if ((1 + n) > I2C_MAX_XFER_SIZE) {
		n = I2C_MAX_XFER_SIZE - 1;
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 1 + n);
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = 0;
	msgs[1].len = 1 + n;
	msgs[1].buf = data;

	data[0] = reg & 0xff;
	for (i = 0; i < n; i++)
		data[1 + i] = values[i];

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed\n",
				__func__, reg, client->addr);
		return;
	}

	if (!debug)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x\n",
				reg, data[1]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x\n",
				reg, data[2], data[1]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x%02x%02x\n",
				reg, data[4], data[3], data[2], data[1]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04x\n",
				n, reg);
	}
}

static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u32 val = 0;

	i2c_rd(sd, reg, (u8 __force *)&val, 1);
	return val;
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wr(sd, reg, &val, 1);
}

static void lt7911exc_i2c_enable(struct v4l2_subdev *sd)
{
	i2c_wr8(sd, I2C_EN_REG, I2C_ENABLE);
}

static void lt7911exc_i2c_disable(struct v4l2_subdev *sd)
{
	i2c_wr8(sd, I2C_EN_REG, I2C_DISABLE);
}

/*
 * LT7911EXC firmware is written directly through its normal I2C address.
 * This is deliberately an explicit, root-only maintenance operation: probe
 * never erases or rewrites flash.  The vendor image contains the LT7911EXC
 * MCU program; it is not a host-side updater executable.
 */
static int lt7911exc_fw_write(struct lt7911exc *lt7911exc, u8 reg,
			      const u8 *values, size_t count)
{
	struct i2c_client *client = lt7911exc->i2c_client;
	u8 buffer[LT7911EXC_FW_PAGE_SIZE + 1];
	struct i2c_msg message = {
		.addr = client->addr,
		.flags = 0,
		.buf = buffer,
	};
	int ret;

	if (count > LT7911EXC_FW_PAGE_SIZE)
		return -EINVAL;

	buffer[0] = reg;
	memcpy(buffer + 1, values, count);
	message.len = count + 1;
	ret = i2c_transfer(client->adapter, &message, 1);
	if (ret < 0)
		return ret;
	if (ret != 1)
		return -EIO;

	return 0;
}

static int lt7911exc_fw_write8(struct lt7911exc *lt7911exc, u8 reg, u8 value)
{
	return lt7911exc_fw_write(lt7911exc, reg, &value, 1);
}

static int lt7911exc_fw_read(struct lt7911exc *lt7911exc, u8 reg,
			     u8 *values, size_t count)
{
	struct i2c_client *client = lt7911exc->i2c_client;
	struct i2c_msg messages[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = values,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, messages, ARRAY_SIZE(messages));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(messages))
		return -EIO;

	return 0;
}

static int lt7911exc_fw_select_flash(struct lt7911exc *lt7911exc)
{
	int ret;

	ret = lt7911exc_fw_write8(lt7911exc, 0xff, 0xe0);
	if (ret)
		return ret;

	return lt7911exc_fw_write8(lt7911exc, 0xee, 0x01);
}

static void lt7911exc_fw_release_flash(struct lt7911exc *lt7911exc)
{
	if (!lt7911exc_fw_write8(lt7911exc, 0xff, 0xe0))
		lt7911exc_fw_write8(lt7911exc, 0xee, 0x00);
}

static u32 lt7911exc_fw_crc32(const u8 *data, size_t length)
{
	u32 crc = 0xffffffff;
	size_t offset;
	int bit;

	for (offset = 0; offset < length; offset += 4) {
		u32 word = (u32)data[offset] |
			   ((u32)data[offset + 1] << 8) |
			   ((u32)data[offset + 2] << 16) |
			   ((u32)data[offset + 3] << 24);

		crc ^= word;
		for (bit = 0; bit < 32; bit++)
			crc = (crc & BIT(31)) ?
				(crc << 1) ^ 0x04c11db7 : crc << 1;
	}

	return crc;
}

static int lt7911exc_fw_load(struct lt7911exc *lt7911exc,
			     const struct firmware **firmware, u32 *crc)
{
	struct device *dev = &lt7911exc->i2c_client->dev;
	u8 *image;
	int ret;

	ret = request_firmware(firmware, LT7911EXC_FW_FILE, dev);
	if (ret)
		return ret;
	if (!(*firmware)->size || (*firmware)->size > LT7911EXC_FW_CRC_ADDR) {
		ret = -EFBIG;
		goto release;
	}

	image = kvmalloc(LT7911EXC_FW_SIZE, GFP_KERNEL);
	if (!image) {
		ret = -ENOMEM;
		goto release;
	}

	memset(image, 0xff, LT7911EXC_FW_SIZE);
	memcpy(image, (*firmware)->data, (*firmware)->size);
	*crc = lt7911exc_fw_crc32(image, LT7911EXC_FW_CRC_ADDR);
	kvfree(image);
	return 0;

release:
	release_firmware(*firmware);
	*firmware = NULL;
	return ret;
}

static int lt7911exc_fw_reset_fifo(struct lt7911exc *lt7911exc)
{
	int ret;

	ret = lt7911exc_fw_select_flash(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x08);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x00);
	if (!ret)
		msleep(100);
	return ret;
}

static int lt7911exc_fw_read_hw_crc(struct lt7911exc *lt7911exc, u32 *crc)
{
	u8 value[4];
	int ret;

	ret = lt7911exc_fw_select_flash(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x7b, 0x60);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x7b, 0x40);
	if (ret)
		return ret;
	msleep(150);
	ret = lt7911exc_fw_read(lt7911exc, 0x22, value, sizeof(value));
	if (ret)
		return ret;

	*crc = ((u32)value[0] << 24) | ((u32)value[1] << 16) |
	       ((u32)value[2] << 8) | value[3];
	return 0;
}

static int lt7911exc_fw_read_flash_crc(struct lt7911exc *lt7911exc, u32 *crc)
{
	u8 value[4];
	int ret;

	ret = lt7911exc_fw_reset_fifo(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x54, 0x45);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x55, 0x03);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x56, 0x04);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x57, 0x00);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x58, 0x00);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5a, 0x00);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5b, 0xff);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5c, 0xfc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x51, 0x01);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x51, 0x00);
	if (ret)
		return ret;
	ret = lt7911exc_fw_read(lt7911exc, 0x5e, value, sizeof(value));
	if (ret)
		return ret;

	*crc = ((u32)value[3] << 24) | ((u32)value[2] << 16) |
	       ((u32)value[1] << 8) | value[0];
	return 0;
}

static int lt7911exc_fw_erase(struct lt7911exc *lt7911exc)
{
	static const struct {
		u8 reg;
		u8 value;
	} sequence[] = {
		{ 0x54, 0x01 }, { 0x55, 0x06 }, { 0x51, 0x01 },
		{ 0x51, 0x00 }, { 0x54, 0x05 }, { 0x55, 0xd8 },
		{ 0x5a, 0x00 }, { 0x5b, 0x00 }, { 0x5c, 0x00 },
		{ 0x51, 0x01 }, { 0x51, 0x00 },
	};
	unsigned int index;
	int ret;

	ret = lt7911exc_fw_select_flash(lt7911exc);
	if (ret)
		return ret;
	for (index = 0; index < ARRAY_SIZE(sequence); index++) {
		ret = lt7911exc_fw_write8(lt7911exc, sequence[index].reg,
					  sequence[index].value);
		if (ret)
			return ret;
	}
	msleep(200);
	return 0;
}

static int lt7911exc_fw_set_address(struct lt7911exc *lt7911exc, u32 address)
{
	int ret;

	ret = lt7911exc_fw_write8(lt7911exc, 0x5a, address >> 16);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5b, address >> 8);
	if (ret)
		return ret;
	return lt7911exc_fw_write8(lt7911exc, 0x5c, address);
}

static int lt7911exc_fw_flush_partial(struct lt7911exc *lt7911exc,
				      u32 address)
{
	int ret;

	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x00);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x01);
	if (ret)
		return ret;
	ret = lt7911exc_fw_set_address(lt7911exc, address);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x05);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x01);
	if (ret)
		return ret;
	usleep_range(1000, 1500);
	return lt7911exc_fw_write8(lt7911exc, 0x5f, 0x00);
}

static int lt7911exc_fw_program_data(struct lt7911exc *lt7911exc,
				     u32 address, const u8 *data,
				     size_t length)
{
	size_t offset = 0;
	int ret;

	ret = lt7911exc_fw_select_flash(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x01);
	if (ret)
		return ret;
	ret = lt7911exc_fw_set_address(lt7911exc, address);
	if (ret)
		return ret;

	while (offset < length) {
		size_t count = min_t(size_t, LT7911EXC_FW_PAGE_SIZE,
					 length - offset);

		ret = lt7911exc_fw_write(lt7911exc, 0x5d,
					 data + offset, count);
		if (ret)
			return ret;
		if (count < LT7911EXC_FW_PAGE_SIZE) {
			ret = lt7911exc_fw_flush_partial(lt7911exc,
							 address + offset);
			if (ret)
				return ret;
		}
		offset += count;
	}

	return lt7911exc_fw_write8(lt7911exc, 0x5f, 0x00);
}

static int lt7911exc_fw_write_crc(struct lt7911exc *lt7911exc, u32 crc)
{
	u8 value[4] = { crc, crc >> 8, crc >> 16, crc >> 24 };
	int ret;

	ret = lt7911exc_fw_select_flash(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write8(lt7911exc, 0x5f, 0x01);
	if (ret)
		return ret;
	ret = lt7911exc_fw_set_address(lt7911exc, LT7911EXC_FW_CRC_ADDR);
	if (ret)
		return ret;
	ret = lt7911exc_fw_write(lt7911exc, 0x5d, value, sizeof(value));
	if (ret)
		return ret;
	return lt7911exc_fw_flush_partial(lt7911exc,
					    LT7911EXC_FW_CRC_ADDR);
}

static void lt7911exc_fw_pause_runtime(struct lt7911exc *lt7911exc)
{
	if (!lt7911exc->runtime_registered)
		return;

	if (lt7911exc->i2c_client->irq)
		disable_irq(lt7911exc->i2c_client->irq);
	else {
		del_timer_sync(&lt7911exc->timer);
		cancel_work_sync(&lt7911exc->work_i2c_poll);
	}
	cancel_delayed_work_sync(&lt7911exc->delayed_work_hotplug);
	cancel_delayed_work_sync(&lt7911exc->delayed_work_res_change);
}

static void lt7911exc_fw_resume_runtime(struct lt7911exc *lt7911exc)
{
	if (!lt7911exc->runtime_registered)
		return;

	if (lt7911exc->i2c_client->irq)
		enable_irq(lt7911exc->i2c_client->irq);
	else
		mod_timer(&lt7911exc->timer,
			  jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
	schedule_delayed_work(&lt7911exc->delayed_work_res_change,
			      msecs_to_jiffies(POLL_INTERVAL_MS));
}

static int lt7911exc_fw_begin(struct lt7911exc *lt7911exc)
{
	mutex_lock(&lt7911exc->fw_lock);
	if (lt7911exc->fw_update_active) {
		mutex_unlock(&lt7911exc->fw_lock);
		return -EBUSY;
	}
	WRITE_ONCE(lt7911exc->fw_update_active, true);
	lt7911exc_fw_pause_runtime(lt7911exc);
	usleep_range(20000, 25000);
	return 0;
}

static void lt7911exc_fw_end(struct lt7911exc *lt7911exc)
{
	lt7911exc_fw_release_flash(lt7911exc);
	WRITE_ONCE(lt7911exc->fw_update_active, false);
	lt7911exc_fw_resume_runtime(lt7911exc);
	mutex_unlock(&lt7911exc->fw_lock);
}

static int lt7911exc_fw_update(struct lt7911exc *lt7911exc, bool force)
{
	const struct firmware *firmware = NULL;
	u32 expected_crc, flash_crc, hardware_crc;
	int ret;

	ret = lt7911exc_fw_load(lt7911exc, &firmware, &expected_crc);
	if (ret)
		return ret;

	ret = lt7911exc_fw_read_hw_crc(lt7911exc, &hardware_crc);
	if (!ret && !force) {
		if (hardware_crc == expected_crc) {
			dev_info(&lt7911exc->i2c_client->dev,
				 "firmware CRC matches 0x%08x; no upgrade needed\n",
				 expected_crc);
			goto out;
		}
	}

	if (force)
		dev_info(&lt7911exc->i2c_client->dev,
			 "forced firmware update; programming %s (%zu bytes, crc 0x%08x)\n",
			 LT7911EXC_FW_FILE, firmware->size, expected_crc);
	else
		dev_info(&lt7911exc->i2c_client->dev,
			 "firmware CRC mismatch; programming %s (%zu bytes, crc 0x%08x)\n",
			 LT7911EXC_FW_FILE, firmware->size, expected_crc);
	ret = lt7911exc_fw_erase(lt7911exc);
	if (ret)
		goto out;
	ret = lt7911exc_fw_program_data(lt7911exc, 0, firmware->data,
					 firmware->size);
	if (ret)
		goto out;
	ret = lt7911exc_fw_write_crc(lt7911exc, expected_crc);
	if (ret)
		goto out;
	ret = lt7911exc_fw_read_flash_crc(lt7911exc, &flash_crc);
	if (ret)
		goto out;
	ret = lt7911exc_fw_read_hw_crc(lt7911exc, &hardware_crc);
	if (ret)
		goto out;
	if (flash_crc != expected_crc || hardware_crc != expected_crc) {
		dev_err(&lt7911exc->i2c_client->dev,
			"firmware verify failed: file=0x%08x flash=0x%08x hardware=0x%08x\n",
			expected_crc, flash_crc, hardware_crc);
		ret = -EIO;
		goto out;
	}
	dev_info(&lt7911exc->i2c_client->dev,
		 "LT7911EXC firmware programmed and verified\n");

out:
	release_firmware(firmware);
	return ret;
}

static ssize_t firmware_update_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	bool force;
	int ret;

	if (sysfs_streq(buf, "update"))
		force = false;
	else if (sysfs_streq(buf, "force"))
		force = true;
	else
		return -EINVAL;

	ret = lt7911exc_fw_begin(lt7911exc);
	if (ret)
		return ret;
	ret = lt7911exc_fw_update(lt7911exc, force);
	lt7911exc_fw_end(lt7911exc);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_WO(firmware_update);

static struct attribute *lt7911exc_fw_attrs[] = {
	&dev_attr_firmware_update.attr,
	NULL,
};

static const struct attribute_group lt7911exc_fw_attr_group = {
	.attrs = lt7911exc_fw_attrs,
};

static inline bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	bool ret;
	int val, i, cnt;
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	/* if not use plugin det gpio */
	if (!lt7911exc->plugin_det_gpio)
		return true;

	cnt = 0;
	for (i = 0; i < 5; i++) {
		val = gpiod_get_value(lt7911exc->plugin_det_gpio);
		if (val > 0)
			cnt++;
		usleep_range(500, 600);
	}

	ret = (cnt >= 3) ? true : false;
	v4l2_dbg(1, debug, sd, "%s: %d\n", __func__, ret);

	return ret;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	v4l2_dbg(1, debug, sd, "%s no signal:%d\n", __func__,
			lt7911exc->nosignal);

	return lt7911exc->nosignal;
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	return lt7911exc->is_audio_present;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	static const int code_to_rate[] = {
		44100, 0, 48000, 32000, 22050, 384000, 24000, 352800,
		88200, 768000, 96000, 705600, 176400, 0, 192000, 0
	};

	if (no_signal(sd))
		return 0;

	return code_to_rate[2];
}

static inline unsigned int fps_calc(const struct v4l2_bt_timings *t)
{
	if (!V4L2_DV_BT_FRAME_HEIGHT(t) || !V4L2_DV_BT_FRAME_WIDTH(t))
		return 0;

	return DIV_ROUND_CLOSEST((unsigned int)t->pixelclock,
			V4L2_DV_BT_FRAME_HEIGHT(t) * V4L2_DV_BT_FRAME_WIDTH(t));
}

static bool lt7911exc_rcv_supported_res(struct v4l2_subdev *sd, u32 width,
		u32 height)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	u32 i;

	for (i = 0; i < lt7911exc->cfg_num; i++) {
		if ((lt7911exc->support_modes[i].width == width) &&
		    (lt7911exc->support_modes[i].height == height)) {
			break;
		}
	}

	if (i == lt7911exc->cfg_num) {
		v4l2_err(sd, "%s do not support res wxh: %dx%d\n", __func__,
				width, height);
		return false;
	} else {
		return true;
	}
}

static u32 lt7911exc_find_link_freq_index(u64 mipi_clk)
{
	u64 best_delta = ~0ULL;
	u32 best_index = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); i++) {
		u64 candidate = link_freq_menu_items[i];
		u64 delta = mipi_clk > candidate ?
			    mipi_clk - candidate : candidate - mipi_clk;

		if (delta < best_delta) {
			best_delta = delta;
			best_index = i;
		}
	}

	return best_index;
}

static int lt7911exc_get_detected_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 hact, vact, htotal, vtotal;
	u32 hfp, hsw, hbp, vfp, vsw, vbp;
	u32 pixel_clock, fps, halt_pix_clk;
	u8 clk_h, clk_m, clk_l;
	u8 val_h, val_l;
	u8 video_status, mipi_format, mipi_port_lane;
	u64 byte_clk, mipi_clk, mipi_data_rate;
	int ret = 0;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	/*
	 * E0:84 is an event code, not a persistent video-valid bit.  The
	 * 20 ms VIDEO_READY event can already have been replaced by
	 * AUDIO_READY when Linux handles the IRQ or probes an already-running
	 * bridge.  Firmware keeps the video timing snapshot latched in both
	 * cases, so accept AUDIO_READY and validate the timing registers below.
	 *
	 * The LT7911EXC register guide explicitly says that signal-information
	 * registers do not require the E0:EE I2C-control enable sequence.
	 */
	video_status = i2c_rd8(sd, LT7911EXC_VIDEO_STATUS);
	if (video_status != LT7911EXC_VIDEO_READY &&
	    video_status != LT7911EXC_AUDIO_READY) {
		lt7911exc->nosignal = true;
		lt7911exc->is_audio_present = false;
		v4l2_dbg(1, debug, sd, "video is not ready, status:0x%02x\n",
			 video_status);
		ret = -ENOLINK;
		goto out;
	}

	mipi_format = i2c_rd8(sd, LT7911EXC_MIPI_FORMAT);
	if (mipi_format != LT7911EXC_MIPI_FORMAT_YUV422_8BIT) {
		lt7911exc->nosignal = true;
		v4l2_err(sd,
			 "unsupported MIPI format 0x%02x, configure LT7911EXC firmware for YUV422 8-bit\n",
			 mipi_format);
		ret = -EINVAL;
		goto out;
	}

	mipi_port_lane = i2c_rd8(sd, LT7911EXC_MIPI_PORT_LANE);
	if ((mipi_port_lane & 0x0f) != 4)
		v4l2_warn(sd,
			  "firmware reports %u MIPI lanes; DTS is configured for 4\n",
			  mipi_port_lane & 0x0f);

	clk_h = i2c_rd8(sd, PCLK_H);
	clk_m = i2c_rd8(sd, PCLK_M);
	clk_l = i2c_rd8(sd, PCLK_L);
	halt_pix_clk = ((clk_h << 16) | (clk_m << 8) | clk_l);
	pixel_clock = halt_pix_clk * 1000;

	clk_h = i2c_rd8(sd, BYTE_PCLK_H);
	clk_m = i2c_rd8(sd, BYTE_PCLK_M);
	clk_l = i2c_rd8(sd, BYTE_PCLK_L);
	byte_clk = ((clk_h << 16) | (clk_m << 8) | clk_l) * 1000;
	mipi_clk = byte_clk * 4;
	mipi_data_rate = byte_clk * 8;

	val_h = i2c_rd8(sd, HTOTAL_H);
	val_l = i2c_rd8(sd, HTOTAL_L);
	htotal = ((val_h << 8) | val_l);

	val_h = i2c_rd8(sd, VTOTAL_H);
	val_l = i2c_rd8(sd, VTOTAL_L);
	vtotal = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, HACT_H);
	val_l = i2c_rd8(sd, HACT_L);
	hact = ((val_h << 8) | val_l);

	val_h = i2c_rd8(sd, VACT_H);
	val_l = i2c_rd8(sd, VACT_L);
	vact = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, HFP_H);
	val_l = i2c_rd8(sd, HFP_L);
	hfp = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, HSW_H);
	val_l = i2c_rd8(sd, HSW_L);
	hsw = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, HBP_H);
	val_l = i2c_rd8(sd, HBP_L);
	hbp = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, VFP_H);
	val_l = i2c_rd8(sd, VFP_L);
	vfp = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, VSW_H);
	val_l = i2c_rd8(sd, VSW_L);
	vsw = (val_h << 8) | val_l;

	val_h = i2c_rd8(sd, VBP_H);
	val_l = i2c_rd8(sd, VBP_L);
	vbp = (val_h << 8) | val_l;

	if (!pixel_clock || !htotal || !vtotal || !hact || !vact) {
		lt7911exc->nosignal = true;
		v4l2_err(sd,
			 "invalid timing: act:%ux%u total:%ux%u pixclk:%u\n",
			 hact, vact, htotal, vtotal, pixel_clock);
		ret = -EINVAL;
		goto out;
	}

	if (hact + hfp + hsw + hbp != htotal ||
	    vact + vfp + vsw + vbp != vtotal)
		v4l2_warn(sd,
			  "timing sum mismatch: act:%ux%u blank h:%u/%u/%u v:%u/%u/%u total:%ux%u\n",
			  hact, vact, hfp, hsw, hbp, vfp, vsw, vbp,
			  htotal, vtotal);

	lt7911exc->nosignal = false;
	lt7911exc->is_audio_present =
		video_status == LT7911EXC_AUDIO_READY;
	timings->type = V4L2_DV_BT_656_1120;
	bt->interlaced = V4L2_DV_PROGRESSIVE;
	bt->width = hact;
	bt->height = vact;
	bt->pixelclock = pixel_clock;
	bt->hfrontporch = hfp;
	bt->hsync = hsw;
	bt->hbackporch = hbp;
	bt->vfrontporch = vfp;
	bt->vsync = vsw;
	bt->vbackporch = vbp;
	fps = pixel_clock / (htotal * vtotal);

	if (!lt7911exc_rcv_supported_res(sd, hact, vact)) {
		lt7911exc->nosignal = true;
		v4l2_err(sd, "%s: rcv err res, return no signal!\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	lt7911exc->mipi_freq_idx =
		lt7911exc_find_link_freq_index(mipi_clk);
	if (lt7911exc->link_freq)
		__v4l2_ctrl_s_ctrl(lt7911exc->link_freq,
				   lt7911exc->mipi_freq_idx);

	v4l2_info(sd, "act:%dx%d, total:%dx%d, pixclk:%d, fps:%d\n",
			hact, vact, htotal, vtotal, pixel_clock, fps);
	v4l2_info(sd,
		  "byte_clk:%llu, mipi_clk:%llu, mipi_data_rate:%llu, link_freq_idx:%u\n",
		  byte_clk, mipi_clk, mipi_data_rate,
		  lt7911exc->mipi_freq_idx);
	v4l2_info(sd, "mipi ports:%u, lanes:%u, format:0x%02x\n",
			(mipi_port_lane >> 4) & 0x0f, mipi_port_lane & 0x0f,
			mipi_format);
	v4l2_info(sd, "inerlaced:%d\n", bt->interlaced);

out:
	return ret;
}

static void lt7911exc_delayed_work_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct lt7911exc *lt7911exc = container_of(dwork,
			struct lt7911exc, delayed_work_hotplug);
	struct v4l2_subdev *sd = &lt7911exc->sd;

	lt7911exc_s_ctrl_detect_tx_5v(sd);
}

static void lt7911exc_s_ctrl_detect_event(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	u8 val;

	val = i2c_rd8(sd, LT7911EXC_VIDEO_STATUS);
	if (val == LT7911EXC_VIDEO_READY ||
	    val == LT7911EXC_AUDIO_READY)
		v4l2_ctrl_s_ctrl(lt7911exc->detect_tx_5v_ctrl, 1);
	else
		v4l2_ctrl_s_ctrl(lt7911exc->detect_tx_5v_ctrl, 0);
}

static void lt7911exc_delayed_work_res_change(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct lt7911exc *lt7911exc = container_of(dwork,
			struct lt7911exc, delayed_work_res_change);
	struct v4l2_subdev *sd = &lt7911exc->sd;

	/*
	 * The interrupt is requested early in probe.  A bridge that is already
	 * running may pulse INT before controls and the subdevice are ready.
	 */
	if (!READ_ONCE(lt7911exc->initialized)) {
		mod_delayed_work(system_wq,
				 &lt7911exc->delayed_work_res_change,
				 msecs_to_jiffies(100));
		return;
	}

	lt7911exc_s_ctrl_detect_event(sd);
	lt7911exc_format_change(sd);

	/*
	 * A bridge that is already connected can still need several seconds to
	 * lock the DP input after the Linux driver probes.  Keep retrying until
	 * a valid timing is available instead of leaving the cached VGA default
	 * forever when the first check runs too early.
	 */
	if (lt7911exc->nosignal)
		schedule_delayed_work(&lt7911exc->delayed_work_res_change,
				      msecs_to_jiffies(POLL_INTERVAL_MS));
}

static int lt7911exc_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	return v4l2_ctrl_s_ctrl(lt7911exc->detect_tx_5v_ctrl,
			tx_5v_power_present(sd));
}

static int lt7911exc_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	return v4l2_ctrl_s_ctrl(lt7911exc->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));
}

static int lt7911exc_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	return v4l2_ctrl_s_ctrl(lt7911exc->audio_present_ctrl,
			audio_present(sd));
}

static int lt7911exc_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	ret |= lt7911exc_s_ctrl_detect_tx_5v(sd);
	ret |= lt7911exc_s_ctrl_audio_sampling_rate(sd);
	ret |= lt7911exc_s_ctrl_audio_present(sd);

	return ret;
}

static void lt7911exc_cphy_timing_config(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	if (lt7911exc->bus_cfg.bus_type == V4L2_MBUS_CSI2_CPHY) {
		while (i2c_rd8(sd, HS_RQST_PRE_REG) != 0x3c) {
			i2c_wr8(sd, HS_RQST_PRE_REG, 0x3c);
			usleep_range(500, 600);
		}
		// i2c_wr8(sd, HS_TRAIL, 0x0b);
	}

	v4l2_dbg(1, debug, sd, "%s config timing succeed\n", __func__);
}

static bool lt7911exc_match_timings(const struct v4l2_dv_timings *t1,
					const struct v4l2_dv_timings *t2)
{
	u64 pclk_delta;

	if (t1->type != t2->type || t1->type != V4L2_DV_BT_656_1120)
		return false;

	pclk_delta = t1->bt.pixelclock > t2->bt.pixelclock ?
		     t1->bt.pixelclock - t2->bt.pixelclock :
		     t2->bt.pixelclock - t1->bt.pixelclock;

	if (t1->bt.width == t2->bt.width &&
		t1->bt.height == t2->bt.height &&
		t1->bt.interlaced == t2->bt.interlaced &&
		t1->bt.hfrontporch == t2->bt.hfrontporch &&
		t1->bt.hsync == t2->bt.hsync &&
		t1->bt.hbackporch == t2->bt.hbackporch &&
		t1->bt.vfrontporch == t2->bt.vfrontporch &&
		t1->bt.vsync == t2->bt.vsync &&
		t1->bt.vbackporch == t2->bt.vbackporch &&
		pclk_delta <= 1000000)
		return true;

	return false;
}

static inline void enable_stream(struct v4l2_subdev *sd, bool enable)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	lt7911exc_cphy_timing_config(&lt7911exc->sd);
	/*
	 * B0 stream control is firmware-dependent on LT7911EXC.  The bridge
	 * normally starts/stops its CSI output autonomously, so do not write B0
	 * unless the exact firmware contract is known.
	 */

	v4l2_dbg(2, debug, sd, "%s: %sable\n",
			__func__, enable ? "en" : "dis");
}

static int lt7911exc_get_reso_dist(const struct lt7911exc_mode *mode,
				struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 cur_fps, dist_fps;

	cur_fps = fps_calc(bt);
	dist_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);

	return abs(mode->width - bt->width) +
		abs(mode->height - bt->height) + abs(dist_fps - cur_fps);
}

static const struct lt7911exc_mode *
lt7911exc_find_best_fit(struct lt7911exc *lt7911exc)
{
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < lt7911exc->cfg_num; i++) {
		dist = lt7911exc_get_reso_dist(&lt7911exc->support_modes[i], &lt7911exc->timings);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	dev_dbg(&lt7911exc->i2c_client->dev,
		"find current mode: support_mode[%d], %dx%d@%dfps\n",
		cur_best_fit, lt7911exc->support_modes[cur_best_fit].width,
		lt7911exc->support_modes[cur_best_fit].height,
		DIV_ROUND_CLOSEST(lt7911exc->support_modes[cur_best_fit].max_fps.denominator,
		lt7911exc->support_modes[cur_best_fit].max_fps.numerator));

	return &lt7911exc->support_modes[cur_best_fit];
}

static void lt7911exc_print_dv_timings(struct v4l2_subdev *sd, const char *prefix)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct device *dev = &lt7911exc->i2c_client->dev;
	const struct v4l2_bt_timings *bt = &lt7911exc->timings.bt;
	const struct lt7911exc_mode *mode;
	u32 htot, vtot;
	u32 fps;

	mode = lt7911exc_find_best_fit(lt7911exc);
	lt7911exc->cur_mode = mode;
	htot = lt7911exc->cur_mode->hts_def;
	vtot = lt7911exc->cur_mode->vts_def;
	if (bt->interlaced)
		vtot /= 2;

	fps = (htot * vtot) > 0 ? div_u64((100 * (u64)bt->pixelclock),
				(htot * vtot)) : 0;

	if (prefix == NULL)
		prefix = "";

	dev_info(dev, "%s: %s%ux%u%s%u.%02u (%ux%u)\n", sd->name, prefix,
		bt->width, bt->height, bt->interlaced ? "i" : "p",
		fps / 100, fps % 100, htot, vtot);
}

static void lt7911exc_format_change(struct v4l2_subdev *sd)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct v4l2_dv_timings timings;
	const struct v4l2_event lt7911exc_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (lt7911exc_get_detected_timings(sd, &timings)) {
		enable_stream(sd, false);
		v4l2_dbg(1, debug, sd, "%s: No signal\n", __func__);
		return;
	}

	if (!lt7911exc_match_timings(&lt7911exc->timings, &timings)) {
		enable_stream(sd, false);
		/* automatically set timing rather than set by user */
		lt7911exc_s_dv_timings(sd, &timings);
		lt7911exc_print_dv_timings(sd,
				"Format_change: New format: ");
	}
	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &lt7911exc_ev_fmt);
}

static const char *lt7911exc_event_name(u8 event)
{
	switch (event) {
	case LT7911EXC_VIDEO_OFF:
		return "video-off";
	case LT7911EXC_VIDEO_READY:
		return "video-ready";
	case LT7911EXC_VIDEO_LOST:
		return "video-lost";
	case LT7911EXC_AUDIO_READY:
		return "audio-ready";
	default:
		return "unknown";
	}
}

static int lt7911exc_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct device *dev = &lt7911exc->i2c_client->dev;
	u8 event;

	/*
	 * INT is an active-low pulse lasting only about 20 ms.  Do not defer
	 * register sampling by the old 50 ms delay, otherwise E0:84 may already
	 * have advanced from VIDEO_READY to AUDIO_READY.
	 */
	event = i2c_rd8(sd, LT7911EXC_VIDEO_STATUS);
	lt7911exc->irq_count++;
	dev_info(dev, "INT triggered: irq=%d count=%u E0:84=0x%02x (%s)\n",
		 lt7911exc->i2c_client->irq, lt7911exc->irq_count,
		 event, lt7911exc_event_name(event));
	mod_delayed_work(system_wq, &lt7911exc->delayed_work_res_change, 0);
	*handled = true;

	return 0;
}

static irqreturn_t lt7911exc_res_change_irq_handler(int irq, void *dev_id)
{
	struct lt7911exc *lt7911exc = dev_id;
	bool handled;

	lt7911exc_isr(&lt7911exc->sd, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t plugin_detect_irq_handler(int irq, void *dev_id)
{
	struct lt7911exc *lt7911exc = dev_id;

	/* control hpd output level after 25ms */
	schedule_delayed_work(&lt7911exc->delayed_work_hotplug,
			HZ / 40);

	return IRQ_HANDLED;
}

static void lt7911exc_irq_poll_timer(struct timer_list *t)
{
	struct lt7911exc *lt7911exc = from_timer(lt7911exc, t, timer);

	schedule_work(&lt7911exc->work_i2c_poll);
	mod_timer(&lt7911exc->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

static void lt7911exc_work_i2c_poll(struct work_struct *work)
{
	struct lt7911exc *lt7911exc = container_of(work,
			struct lt7911exc, work_i2c_poll);
	struct v4l2_subdev *sd = &lt7911exc->sd;

	lt7911exc_format_change(sd);
}

static int lt7911exc_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static int lt7911exc_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int lt7911exc_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	if (!timings)
		return -EINVAL;

	if (debug)
		v4l2_print_dv_timings(sd->name, "s_dv_timings: ",
				timings, false);

	if (lt7911exc_match_timings(&lt7911exc->timings, timings)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	lt7911exc->timings = *timings;

	enable_stream(sd, false);

	return 0;
}

static int lt7911exc_g_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	*timings = lt7911exc->timings;

	return 0;
}

static int lt7911exc_enum_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&lt7911exc_timings_cap, NULL, NULL);
}

static int lt7911exc_query_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	*timings = lt7911exc->timings;
	if (debug)
		v4l2_print_dv_timings(sd->name,
				"query_dv_timings: ", timings, false);

	if (!v4l2_valid_dv_timings(timings, &lt7911exc_timings_cap, NULL,
				NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n",
				__func__);

		return -ERANGE;
	}

	return 0;
}

static int lt7911exc_dv_timings_cap(struct v4l2_subdev *sd,
				struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;

	*cap = lt7911exc_timings_cap;

	return 0;
}

static int lt7911exc_g_mbus_config(struct v4l2_subdev *sd,
			unsigned int pad, struct v4l2_mbus_config *cfg)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	cfg->type = lt7911exc->bus_cfg.bus_type;
	cfg->bus.mipi_csi2 = lt7911exc->bus_cfg.bus.mipi_csi2;

	return 0;
}

static int lt7911exc_s_stream(struct v4l2_subdev *sd, int on)
{
	enable_stream(sd, on);

	return 0;
}

static int lt7911exc_enum_mbus_code(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->index) {
	case 0:
		code->code = LT7911EXC_MEDIA_BUS_FMT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int lt7911exc_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	if (fse->index >= lt7911exc->cfg_num)
		return -EINVAL;

	if (fse->code != LT7911EXC_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = lt7911exc->support_modes[fse->index].width;
	fse->max_width  = lt7911exc->support_modes[fse->index].width;
	fse->max_height = lt7911exc->support_modes[fse->index].height;
	fse->min_height = lt7911exc->support_modes[fse->index].height;

	return 0;
}

static int lt7911exc_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	const struct lt7911exc_mode *mode;
	u32 mipi_freq_idx;

	mutex_lock(&lt7911exc->confctl_mutex);
	format->format.code = lt7911exc->mbus_fmt_code;
	format->format.width = lt7911exc->timings.bt.width;
	format->format.height = lt7911exc->timings.bt.height;
	format->format.field =
		lt7911exc->timings.bt.interlaced ?
		V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;
	mutex_unlock(&lt7911exc->confctl_mutex);

	mode = lt7911exc_find_best_fit(lt7911exc);
	lt7911exc->cur_mode = mode;
	mipi_freq_idx = lt7911exc->nosignal ?
			mode->mipi_freq_idx : lt7911exc->mipi_freq_idx;

	__v4l2_ctrl_s_ctrl_int64(lt7911exc->pixel_rate,
				LT7911EXC_PIXEL_RATE);
	__v4l2_ctrl_s_ctrl(lt7911exc->link_freq,
				mipi_freq_idx);

	v4l2_dbg(1, debug, sd, "%s: mipi_freq_idx(%u)",
		 __func__, mipi_freq_idx);

	v4l2_dbg(1, debug, sd, "%s: fmt code:%d, w:%d, h:%d, field code:%d\n",
			__func__, format->format.code, format->format.width,
			format->format.height, format->format.field);

	return 0;
}

static int lt7911exc_enum_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	if (fie->index >= lt7911exc->cfg_num)
		return -EINVAL;

	fie->code = LT7911EXC_MEDIA_BUS_FMT;

	fie->width = lt7911exc->support_modes[fie->index].width;
	fie->height = lt7911exc->support_modes[fie->index].height;
	fie->interval = lt7911exc->support_modes[fie->index].max_fps;

	return 0;
}

static int lt7911exc_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	const struct lt7911exc_mode *mode;

	/* is overwritten by get_fmt */
	u32 code = format->format.code;
	int ret = lt7911exc_get_fmt(sd, sd_state, format);

	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
	case LT7911EXC_MEDIA_BUS_FMT:
		break;

	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	lt7911exc->mbus_fmt_code = format->format.code;
	mode = lt7911exc_find_best_fit(lt7911exc);
	lt7911exc->cur_mode = mode;

	enable_stream(sd, false);

	return 0;
}

static int lt7911exc_g_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *fi)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	const struct lt7911exc_mode *mode = lt7911exc->cur_mode;

	mutex_lock(&lt7911exc->confctl_mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&lt7911exc->confctl_mutex);

	return 0;
}

static void lt7911exc_get_module_inf(struct lt7911exc *lt7911exc,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, LT7911EXC_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, lt7911exc->module_name, sizeof(inf->base.module));
	strscpy(inf->base.lens, lt7911exc->len_name, sizeof(inf->base.lens));
}

static long lt7911exc_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	long ret = 0;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		lt7911exc_get_module_inf(lt7911exc, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDMI_MODE:
		*(int *)arg = RKMODULE_HDMIIN_MODE;
		break;
	case RKMODULE_SET_CSI_DPHY_PARAM:
		dphy_param = (struct rkmodule_csi_dphy_param *)arg;
		if (dphy_param->vendor == PHY_VENDOR_SAMSUNG)
			rk3588_dcphy_param = *dphy_param;
		dev_dbg(&lt7911exc->i2c_client->dev,
			"sensor set dphy param\n");
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = (struct rkmodule_csi_dphy_param *)arg;
		*dphy_param = rk3588_dcphy_param;
		dev_dbg(&lt7911exc->i2c_client->dev,
			"sensor get dphy param\n");
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int lt7911exc_s_power(struct v4l2_subdev *sd, int on)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	int ret = 0;

	mutex_lock(&lt7911exc->confctl_mutex);

	if (lt7911exc->power_on == !!on)
		goto unlock_and_return;

	if (on)
		lt7911exc->power_on = true;
	else
		lt7911exc->power_on = false;

unlock_and_return:
	mutex_unlock(&lt7911exc->confctl_mutex);

	return ret;
}

#ifdef CONFIG_COMPAT
static long lt7911exc_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;
	int *seq;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = lt7911exc_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDMI_MODE:
		seq = kzalloc(sizeof(*seq), GFP_KERNEL);
		if (!seq) {
			ret = -ENOMEM;
			return ret;
		}

		ret = lt7911exc_ioctl(sd, cmd, seq);
		if (!ret) {
			ret = copy_to_user(up, seq, sizeof(*seq));
			if (ret)
				ret = -EFAULT;
		}
		kfree(seq);
		break;
	case RKMODULE_SET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(dphy_param, up, sizeof(*dphy_param));
		if (!ret)
			ret = lt7911exc_ioctl(sd, cmd, dphy_param);
		else
			ret = -EFAULT;
		kfree(dphy_param);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = lt7911exc_ioctl(sd, cmd, dphy_param);
		if (!ret) {
			ret = copy_to_user(up, dphy_param, sizeof(*dphy_param));
			if (ret)
				ret = -EFAULT;
		}
		kfree(dphy_param);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int lt7911exc_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct lt7911exc_mode *def_mode = &lt7911exc->support_modes[0];

	mutex_lock(&lt7911exc->confctl_mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = LT7911EXC_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&lt7911exc->confctl_mutex);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops lt7911exc_internal_ops = {
	.open = lt7911exc_open,
};
#endif

static const struct v4l2_subdev_core_ops lt7911exc_core_ops = {
	.s_power = lt7911exc_s_power,
	.interrupt_service_routine = lt7911exc_isr,
	.subscribe_event = lt7911exc_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = lt7911exc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = lt7911exc_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops lt7911exc_video_ops = {
	.g_input_status = lt7911exc_g_input_status,
	.s_dv_timings = lt7911exc_s_dv_timings,
	.g_dv_timings = lt7911exc_g_dv_timings,
	.query_dv_timings = lt7911exc_query_dv_timings,
	.s_stream = lt7911exc_s_stream,
	.g_frame_interval = lt7911exc_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops lt7911exc_pad_ops = {
	.enum_mbus_code = lt7911exc_enum_mbus_code,
	.enum_frame_size = lt7911exc_enum_frame_sizes,
	.enum_frame_interval = lt7911exc_enum_frame_interval,
	.set_fmt = lt7911exc_set_fmt,
	.get_fmt = lt7911exc_get_fmt,
	.enum_dv_timings = lt7911exc_enum_dv_timings,
	.dv_timings_cap = lt7911exc_dv_timings_cap,
	.get_mbus_config = lt7911exc_g_mbus_config,
};

static const struct v4l2_subdev_ops lt7911exc_ops = {
	.core = &lt7911exc_core_ops,
	.video = &lt7911exc_video_ops,
	.pad = &lt7911exc_pad_ops,
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_sampling_rate = {
	.id = RK_V4L2_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config lt7911exc_ctrl_audio_present = {
	.id = RK_V4L2_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static int lt7911exc_init_v4l2_ctrls(struct lt7911exc *lt7911exc)
{
	const struct lt7911exc_mode *mode;
	struct v4l2_subdev *sd;
	int ret;

	mode = lt7911exc->cur_mode;
	sd = &lt7911exc->sd;
	ret = v4l2_ctrl_handler_init(&lt7911exc->hdl, 5);
	if (ret)
		return ret;

	lt7911exc->link_freq = v4l2_ctrl_new_int_menu(&lt7911exc->hdl, NULL,
			V4L2_CID_LINK_FREQ,
			ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			link_freq_menu_items);
	lt7911exc->pixel_rate = v4l2_ctrl_new_std(&lt7911exc->hdl, NULL,
			V4L2_CID_PIXEL_RATE,
			0, LT7911EXC_PIXEL_RATE, 1, LT7911EXC_PIXEL_RATE);

	lt7911exc->detect_tx_5v_ctrl = v4l2_ctrl_new_std(&lt7911exc->hdl,
			NULL, V4L2_CID_DV_RX_POWER_PRESENT,
			0, 1, 0, 0);

	lt7911exc->audio_sampling_rate_ctrl =
		v4l2_ctrl_new_custom(&lt7911exc->hdl,
				&lt7911exc_ctrl_audio_sampling_rate, NULL);
	lt7911exc->audio_present_ctrl = v4l2_ctrl_new_custom(&lt7911exc->hdl,
			&lt7911exc_ctrl_audio_present, NULL);

	sd->ctrl_handler = &lt7911exc->hdl;
	if (lt7911exc->hdl.error) {
		ret = lt7911exc->hdl.error;
		v4l2_err(sd, "cfg v4l2 ctrls failed! ret:%d\n", ret);
		return ret;
	}

	__v4l2_ctrl_s_ctrl(lt7911exc->link_freq, mode->mipi_freq_idx);
	__v4l2_ctrl_s_ctrl_int64(lt7911exc->pixel_rate, LT7911EXC_PIXEL_RATE);

	if (lt7911exc_update_controls(sd)) {
		ret = -ENODEV;
		v4l2_err(sd, "update v4l2 ctrls failed! ret:%d\n", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static int lt7911exc_probe_of(struct lt7911exc *lt7911exc)
{
	struct device *dev = &lt7911exc->i2c_client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *ep;
	int ret;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
			&lt7911exc->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
			&lt7911exc->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
			&lt7911exc->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
			&lt7911exc->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	lt7911exc->power_gpio = devm_gpiod_get_optional(dev, "power",
			GPIOD_OUT_LOW);
	if (IS_ERR(lt7911exc->power_gpio)) {
		dev_err(dev, "failed to get power gpio\n");
		ret = PTR_ERR(lt7911exc->power_gpio);
		return ret;
	}

	lt7911exc->reset_gpio = devm_gpiod_get_optional(dev, "reset",
			GPIOD_OUT_HIGH);
	if (IS_ERR(lt7911exc->reset_gpio)) {
		dev_err(dev, "failed to get reset gpio\n");
		ret = PTR_ERR(lt7911exc->reset_gpio);
		return ret;
	}

	lt7911exc->plugin_det_gpio = devm_gpiod_get_optional(dev, "plugin-det",
			GPIOD_IN);
	if (IS_ERR(lt7911exc->plugin_det_gpio)) {
		dev_err(dev, "failed to get plugin det gpio\n");
		ret = PTR_ERR(lt7911exc->plugin_det_gpio);
		return ret;
	}

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep),
					&lt7911exc->bus_cfg);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		goto put_node;
	}

	if (lt7911exc->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY) {
		lt7911exc->support_modes = supported_modes_dphy;
		lt7911exc->cfg_num = ARRAY_SIZE(supported_modes_dphy);
	} else {
		lt7911exc->support_modes = supported_modes_cphy;
		lt7911exc->cfg_num = ARRAY_SIZE(supported_modes_cphy);
	}

	/* LT7911EXC reference design uses its own crystal; xvclk is optional. */
	lt7911exc->xvclk = devm_clk_get_optional(dev, "xvclk");
	if (IS_ERR(lt7911exc->xvclk)) {
		dev_err(dev, "failed to get xvclk\n");
		ret = PTR_ERR(lt7911exc->xvclk);
		goto put_node;
	}

	if (lt7911exc->xvclk) {
		ret = clk_prepare_enable(lt7911exc->xvclk);
		if (ret) {
			dev_err(dev, "failed to enable xvclk\n");
			goto put_node;
		}
	}

	lt7911exc->enable_hdcp = false;

	ret = 0;

put_node:
	of_node_put(ep);
	return ret;
}
#else
static inline int lt7911exc_probe_of(struct lt7911exc *state)
{
	return -ENODEV;
}
#endif

static int __lt7911exc_power_on(struct lt7911exc *lt7911exc)
{
	struct device *dev = &lt7911exc->i2c_client->dev;

	dev_info(dev, "lt7911exc power on\n");
	if (lt7911exc->reset_gpio)
		gpiod_set_value(lt7911exc->reset_gpio, 1);
	usleep_range(20000, 25000);
	if (lt7911exc->power_gpio)
		gpiod_set_value(lt7911exc->power_gpio, 1);
	//delay 20ms before reset
	usleep_range(25000, 30000);
	if (lt7911exc->reset_gpio)
		gpiod_set_value(lt7911exc->reset_gpio, 0);
	usleep_range(25000, 30000);

	return 0;
}

static void __lt7911exc_power_off(struct lt7911exc *lt7911exc)
{
	struct device *dev = &lt7911exc->i2c_client->dev;

	dev_info(dev, "lt7911exc power off\n");

	if (lt7911exc->reset_gpio && !IS_ERR(lt7911exc->reset_gpio))
		gpiod_set_value(lt7911exc->reset_gpio, 1);

	if (lt7911exc->power_gpio && !IS_ERR(lt7911exc->power_gpio))
		gpiod_set_value(lt7911exc->power_gpio, 0);
}

static int lt7911exc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	return __lt7911exc_power_on(lt7911exc);
}

static int lt7911exc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	__lt7911exc_power_off(lt7911exc);

	return 0;
}

static const struct dev_pm_ops lt7911exc_pm_ops = {
	.suspend = lt7911exc_suspend,
	.resume = lt7911exc_resume,
};

static int lt7911exc_check_chip_id(struct lt7911exc *lt7911exc)
{
	struct device *dev = &lt7911exc->i2c_client->dev;
	struct v4l2_subdev *sd = &lt7911exc->sd;
	u8 id_h, id_l;
	u32 chipid;
	int ret = 0;

	lt7911exc_i2c_enable(sd);
	id_l  = i2c_rd8(sd, CHIPID_REGL);
	id_h  = i2c_rd8(sd, CHIPID_REGH);
	lt7911exc_i2c_disable(sd);

	chipid = (id_h << 8) | id_l;
	if (chipid != LT7911EXC_CHIPID) {
		dev_err(dev, "chipid err, read:%#x, expect:%#x\n",
				chipid, LT7911EXC_CHIPID);
		return -EINVAL;
	}
	dev_info(dev, "check chipid ok, id:%#x", chipid);

	return ret;
}

static int lt7911exc_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct v4l2_dv_timings default_timing =
				V4L2_DV_BT_CEA_640X480P59_94;
	struct lt7911exc *lt7911exc;
	struct v4l2_subdev *sd;
	struct device *dev = &client->dev;
	char facing[2];
	int err;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	lt7911exc = devm_kzalloc(dev, sizeof(struct lt7911exc), GFP_KERNEL);
	if (!lt7911exc)
		return -ENOMEM;

	sd = &lt7911exc->sd;
	lt7911exc->i2c_client = client;
	lt7911exc->mbus_fmt_code = LT7911EXC_MEDIA_BUS_FMT;
	i2c_set_clientdata(client, sd);
	mutex_init(&lt7911exc->fw_lock);

	err = lt7911exc_probe_of(lt7911exc);
	if (err) {
		v4l2_err(sd, "lt7911exc_parse_of failed! err:%d\n", err);
		return err;
	}

	lt7911exc->timings = default_timing;
	lt7911exc->cur_mode = &lt7911exc->support_modes[0];
	lt7911exc->nosignal = true;

	__lt7911exc_power_on(lt7911exc);
	err = devm_device_add_group(dev, &lt7911exc_fw_attr_group);
	if (err) {
		dev_err(dev, "failed to create firmware maintenance interface: %d\n",
			err);
		return err;
	}
	err = lt7911exc_check_chip_id(lt7911exc);
	if (err < 0) {
		dev_warn(dev,
			 "runtime firmware not detected; I2C firmware updater remains available\n");
		return 0;
	}

	INIT_DELAYED_WORK(&lt7911exc->delayed_work_hotplug,
			lt7911exc_delayed_work_hotplug);
	INIT_DELAYED_WORK(&lt7911exc->delayed_work_res_change,
			lt7911exc_delayed_work_res_change);

	if (lt7911exc->i2c_client->irq) {
		dev_info(dev, "using active-low falling-edge IRQ %d\n",
			 lt7911exc->i2c_client->irq);
		err = devm_request_threaded_irq(dev,
				lt7911exc->i2c_client->irq,
				NULL, lt7911exc_res_change_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"lt7911exc", lt7911exc);
		if (err) {
			v4l2_err(sd, "request irq failed! err:%d\n", err);
			goto err_work_queues;
		}
	} else {
		v4l2_dbg(1, debug, sd, "no irq, cfg poll!\n");
		INIT_WORK(&lt7911exc->work_i2c_poll, lt7911exc_work_i2c_poll);
		timer_setup(&lt7911exc->timer, lt7911exc_irq_poll_timer, 0);
		lt7911exc->timer.expires = jiffies +
				       msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&lt7911exc->timer);
	}

	if (lt7911exc->plugin_det_gpio) {
		lt7911exc->plugin_irq = gpiod_to_irq(lt7911exc->plugin_det_gpio);
		if (lt7911exc->plugin_irq < 0) {
			dev_warn(dev, "failed to get plugin det irq (%d)\n",
				 lt7911exc->plugin_irq);
		} else {
			err = devm_request_threaded_irq(dev,
					lt7911exc->plugin_irq, NULL,
					plugin_detect_irq_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"lt7911exc", lt7911exc);
			if (err)
				dev_warn(dev,
					 "failed to register plugin det irq (%d)\n",
					 err);
		}
	}

	mutex_init(&lt7911exc->confctl_mutex);
	err = lt7911exc_init_v4l2_ctrls(lt7911exc);
	if (err)
		goto err_free_hdl;

	client->flags |= I2C_CLIENT_SCCB;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	v4l2_i2c_subdev_init(sd, client, &lt7911exc_ops);
	sd->internal_ops = &lt7911exc_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	lt7911exc->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	err = media_entity_pads_init(&sd->entity, 1, &lt7911exc->pad);
	if (err < 0) {
		v4l2_err(sd, "media entity init failed! err:%d\n", err);
		goto err_free_hdl;
	}
#endif
	memset(facing, 0, sizeof(facing));
	if (strcmp(lt7911exc->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 lt7911exc->module_index, facing,
		 LT7911EXC_NAME, dev_name(sd->dev));
	err = v4l2_async_register_subdev_sensor(sd);
	if (err < 0) {
		v4l2_err(sd, "v4l2 register subdev failed! err:%d\n", err);
		goto err_clean_entity;
	}

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err) {
		v4l2_err(sd, "v4l2 ctrl handler setup failed! err:%d\n", err);
		goto err_clean_entity;
	}

	WRITE_ONCE(lt7911exc->runtime_registered, true);
	WRITE_ONCE(lt7911exc->initialized, true);
	schedule_delayed_work(&lt7911exc->delayed_work_res_change,
			      msecs_to_jiffies(POLL_INTERVAL_MS));

	enable_stream(sd, false);
	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
			client->addr << 1, client->adapter->name);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_free_hdl:
	v4l2_ctrl_handler_free(&lt7911exc->hdl);
	mutex_destroy(&lt7911exc->confctl_mutex);
err_work_queues:
	if (!lt7911exc->i2c_client->irq)
		flush_work(&lt7911exc->work_i2c_poll);
	cancel_delayed_work_sync(&lt7911exc->delayed_work_hotplug);
	cancel_delayed_work_sync(&lt7911exc->delayed_work_res_change);

	return err;
}

static void lt7911exc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lt7911exc *lt7911exc = to_lt7911exc(sd);

	if (!lt7911exc->runtime_registered) {
		if (lt7911exc->xvclk)
			clk_disable_unprepare(lt7911exc->xvclk);
		mutex_destroy(&lt7911exc->fw_lock);
		return;
	}

	WRITE_ONCE(lt7911exc->initialized, false);
	WRITE_ONCE(lt7911exc->runtime_registered, false);
	if (!lt7911exc->i2c_client->irq) {
		del_timer_sync(&lt7911exc->timer);
		flush_work(&lt7911exc->work_i2c_poll);
	}
	cancel_delayed_work_sync(&lt7911exc->delayed_work_hotplug);
	cancel_delayed_work_sync(&lt7911exc->delayed_work_res_change);
	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&lt7911exc->hdl);
	mutex_destroy(&lt7911exc->confctl_mutex);
	mutex_destroy(&lt7911exc->fw_lock);
	if (lt7911exc->xvclk)
		clk_disable_unprepare(lt7911exc->xvclk);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id lt7911exc_of_match[] = {
	{ .compatible = "lontium,lt7911exc" },
	{},
};
MODULE_DEVICE_TABLE(of, lt7911exc_of_match);
#endif

static struct i2c_driver lt7911exc_driver = {
	.driver = {
		.name = LT7911EXC_NAME,
		.pm = &lt7911exc_pm_ops,
		.of_match_table = of_match_ptr(lt7911exc_of_match),
	},
	.probe = lt7911exc_probe,
	.remove = lt7911exc_remove,
};

static int __init lt7911exc_driver_init(void)
{
	return i2c_add_driver(&lt7911exc_driver);
}

static void __exit lt7911exc_driver_exit(void)
{
	i2c_del_driver(&lt7911exc_driver);
}

device_initcall_sync(lt7911exc_driver_init);
module_exit(lt7911exc_driver_exit);

MODULE_DESCRIPTION("Lontium lt7911exc DP/type-c to CSI-2 bridge driver");
MODULE_AUTHOR("Jianwei Fan <jianwei.fan@rock-chips.com>");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(LT7911EXC_FW_FILE);
