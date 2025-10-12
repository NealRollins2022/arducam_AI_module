/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd. <www.arducam.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arducam_mega

#include <drivers/video/arducam_mega.h>

#include <zephyr/device.h>
#include <drivers/video.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mega_camera);

#define ARDUCHIP_FIFO   0x04 /* FIFO and I2C control */
#define ARDUCHIP_FIFO_2 0x07
#define FIFO_CLEAR_ID_MASK 0x01
#define FIFO_START_MASK    0x02

#define ARDUCHIP_TRIG 0x44
#define VSYNC_MASK    0x01
#define SHUTTER_MASK  0x02
#define CAP_DONE_MASK 0x04

#define FIFO_SIZE1 0x45
#define FIFO_SIZE2 0x46
#define FIFO_SIZE3 0x47

#define BURST_FIFO_READ  0x3C
#define SINGLE_FIFO_READ 0x3D

/* DSP register bank FF=0x00 */
#define CAM_REG_POWER_CONTROL                 0X02
#define CAM_REG_SENSOR_RESET                  0X07
#define CAM_REG_FORMAT                        0X20
#define CAM_REG_CAPTURE_RESOLUTION            0X21
#define CAM_REG_BRIGHTNESS_CONTROL            0X22
#define CAM_REG_CONTRAST_CONTROL              0X23
#define CAM_REG_SATURATION_CONTROL            0X24
#define CAM_REG_EV_CONTROL                    0X25
#define CAM_REG_WHITEBALANCE_CONTROL          0X26
#define CAM_REG_COLOR_EFFECT_CONTROL          0X27
#define CAM_REG_SHARPNESS_CONTROL             0X28
#define CAM_REG_AUTO_FOCUS_CONTROL            0X29
#define CAM_REG_IMAGE_QUALITY                 0x2A
#define CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE 0X30
#define CAM_REG_MANUAL_GAIN_BIT_9_8           0X31
#define CAM_REG_MANUAL_GAIN_BIT_7_0           0X32
#define CAM_REG_MANUAL_EXPOSURE_BIT_19_16     0X33
#define CAM_REG_MANUAL_EXPOSURE_BIT_15_8      0X34
#define CAM_REG_MANUAL_EXPOSURE_BIT_7_0       0X35
#define CAM_REG_BURST_FIFO_READ_OPERATION     0X3C
#define CAM_REG_SINGLE_FIFO_READ_OPERATION    0X3D
#define CAM_REG_SENSOR_ID                     0x40
#define CAM_REG_YEAR_SDK                      0x41
#define CAM_REG_MONTH_SDK                     0x42
#define CAM_REG_DAY_SDK                       0x43
#define CAM_REG_SENSOR_STATE                  0x44
#define CAM_REG_FPGA_VERSION_NUMBER           0x49
#define CAM_REG_DEBUG_DEVICE_ADDRESS          0X0A
#define CAM_REG_DEBUG_REGISTER_HIGH           0X0B
#define CAM_REG_DEBUG_REGISTER_LOW            0X0C
#define CAM_REG_DEBUG_REGISTER_VALUE          0X0D

#define SENSOR_STATE_IDLE   (1 << 1)
#define SENSOR_RESET_ENABLE (1 << 6)

#define CTR_WHITEBALANCE 0X02
#define CTR_EXPOSURE     0X01
#define CTR_GAIN         0X00

#define AC_STACK_SIZE 4096
#define AC_PRIORITY 5

K_THREAD_STACK_DEFINE(ac_stack_area, AC_STACK_SIZE);

struct k_work_q ac_work_q;

struct mega_sdk_data {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t version;
};

struct arducam_mega_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec cs_gpio;
};

struct arducam_mega_data {
	const struct device *dev;
	struct video_format fmt;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct k_work buf_work;
	struct k_timer stream_schedule_timer;
	struct k_poll_signal *signal;
	struct arducam_mega_info *info;
	struct mega_sdk_data ver;
	uint8_t fifo_first_read;
	uint32_t fifo_length;
	uint8_t stream_on;
};

static struct arducam_mega_info mega_infos[] = {{
	.support_resolution = 7894,
	.support_special_effects = 63,
	.exposure_value_max = 30000,
	.exposure_value_min = 1,
	.gain_value_max = 1023,
	.gain_value_min = 1,
	.enable_focus = 1,
	.enable_sharpness = 0,
	.device_address = 0x78,
}, {
	.support_resolution = 7638,
	.support_special_effects = 319,
	.exposure_value_max = 30000,
	.exposure_value_min = 1,
	.gain_value_max = 1023,
	.gain_value_min = 1,
	.enable_focus = 0,
	.enable_sharpness = 1,
	.device_address = 0x78,
}};

#define ARDUCAM_MEGA_VIDEO_FORMAT_CAP(width, height, format) \
	{ .pixelformat = (format), .width_min = (width), .width_max = (width), \
	  .height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0 }

static struct video_format_cap fmts[] = {
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_RGB565),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_RGB565),
	{0},
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_JPEG),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_JPEG),
	{0},
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(96, 96, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(128, 128, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 240, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(320, 320, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(640, 480, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1280, 720, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1600, 1200, VIDEO_PIX_FMT_YUYV),
	ARDUCAM_MEGA_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_YUYV),
	{0},
	{0},
};

#define SUPPORT_RESOLUTION_NUM 9

static inline void cs_select(const struct arducam_mega_config *cfg)
{
	gpio_pin_set_dt(&cfg->cs_gpio, 0);
}

static inline void cs_deselect(const struct arducam_mega_config *cfg)
{
	gpio_pin_set_dt(&cfg->cs_gpio, 1);
}

static uint8_t support_resolution[SUPPORT_RESOLUTION_NUM] = {
	MEGA_RESOLUTION_96X96, MEGA_RESOLUTION_128X128, MEGA_RESOLUTION_QVGA,
	MEGA_RESOLUTION_320X320, MEGA_RESOLUTION_VGA, MEGA_RESOLUTION_HD,
	MEGA_RESOLUTION_UXGA, MEGA_RESOLUTION_FHD, MEGA_RESOLUTION_NONE,
};

/* Write SPI register */
static int arducam_mega_write_reg(const struct arducam_mega_config *cfg,
                                  uint8_t reg_addr, uint8_t value)
{
	uint8_t tries = 3;
	reg_addr |= 0x80;

	struct spi_buf tx_buf[2] = {
		{ .buf = &reg_addr, .len = 1 },
		{ .buf = &value, .len = 1 },
	};
	struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 2 };

	while (tries--) {
		cs_select(cfg);
		int ret = spi_write_dt(&cfg->bus, &tx_bufs);
		cs_deselect(cfg);

		if (ret == 0)
			return 0;
		k_msleep(1);
	}
	LOG_ERR("Write reg 0x%x fail", reg_addr);
	return -EIO;
}

/* Read SPI register */
static int arducam_mega_read_reg(const struct arducam_mega_config *cfg,
                                 uint8_t reg_addr, uint8_t *val)
{
	uint8_t tries = 3;
	struct spi_buf tx_buf[2] = {
		{ .buf = &reg_addr, .len = 1 },
		{ .buf = val, .len = 1 },
	};
	struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 2 };

	while (tries--) {
		cs_select(cfg);
		int ret = spi_transceive_dt(&cfg->bus, &tx_bufs, NULL);
		cs_deselect(cfg);

		if (ret == 0)
			return 0;
		k_msleep(1);
	}
	LOG_ERR("Read reg 0x%x fail", reg_addr);
	return -EIO;
}

/* Burst read function holds CS low across full transfer */
static int arducam_mega_burst_read(const struct arducam_mega_config *cfg,
                                   uint8_t *buf, size_t len)
{
	uint8_t cmd = BURST_FIFO_READ;

	struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	struct spi_buf rx_buf = { .buf = buf, .len = len };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };

	cs_select(cfg);
	int ret = spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs);
	cs_deselect(cfg);

	if (ret < 0) {
		LOG_ERR("Burst read fail");
		return ret;
	}

	return 0;
}

/* FIFO clear */
static int arducam_mega_fifo_clear(const struct arducam_mega_config *cfg)
{
	return arducam_mega_write_reg(cfg, ARDUCHIP_FIFO, FIFO_CLEAR_ID_MASK);
}

/* Start capture */
static int arducam_mega_start_capture(const struct arducam_mega_config *cfg)
{
	return arducam_mega_write_reg(cfg, ARDUCHIP_FIFO, FIFO_START_MASK);
}

/* Get FIFO length */
static int arducam_mega_get_fifo_length(const struct arducam_mega_config *cfg,
                                        uint32_t *length)
{
	uint8_t buf[3] = {0};
	int ret;

	ret = arducam_mega_read_reg(cfg, FIFO_SIZE1, &buf[0]);
	if (ret)
		return ret;
	ret = arducam_mega_read_reg(cfg, FIFO_SIZE2, &buf[1]);
	if (ret)
		return ret;
	ret = arducam_mega_read_reg(cfg, FIFO_SIZE3, &buf[2]);
	if (ret)
		return ret;

	*length = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
	return 0;
}

/* Initialize device */
static int arducam_mega_init(const struct device *dev)
{
	const struct arducam_mega_config *cfg = dev->config;
	int ret;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_HIGH);
	if (ret) {
		LOG_ERR("CS GPIO config fail");
		return ret;
	}

	cs_deselect(cfg);
	k_work_queue_start(&ac_work_q, ac_stack_area,
	                   K_THREAD_STACK_SIZEOF(ac_stack_area),
	                   AC_PRIORITY, NULL);

	return 0;
}

#define ARDUCAM_MEGA_DEVICE(inst)                                     \
	static const struct arducam_mega_config arducam_mega_cfg_##inst = { \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER, 0), \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(inst, cs_gpios),             \
	};                                                              \
	DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, NULL,      \
			      &arducam_mega_cfg_##inst, POST_KERNEL,    \
			      CONFIG_VIDEO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_DEVICE)
