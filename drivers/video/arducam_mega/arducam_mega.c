/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd. <www.arducam.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arducam_mega

#include <zephyr/drivers/video/arducam_mega.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mega_camera);

#define ARDUCHIP_FIFO   0x04 /* FIFO and I2C control */
#define ARDUCHIP_FIFO_2 0x07 /* FIFO and I2C control */

#define FIFO_CLEAR_ID_MASK 0x01
#define FIFO_START_MASK    0x02

#define ARDUCHIP_TRIG 0x44 /* Trigger source */
#define VSYNC_MASK    0x01
#define SHUTTER_MASK  0x02
#define CAP_DONE_MASK 0x04

#define FIFO_SIZE1 0x45 /* Camera write FIFO size[7:0] for burst to read */
#define FIFO_SIZE2 0x46 /* Camera write FIFO size[15:8] */
#define FIFO_SIZE3 0x47 /* Camera write FIFO size[18:16] */

#define BURST_FIFO_READ  0x3C /* Burst FIFO read operation */
#define SINGLE_FIFO_READ 0x3D /* Single FIFO read operation */

/* DSP register bank FF=0x00 */
#define CAM_REG_POWER_CONTROL                 0x02
#define CAM_REG_SENSOR_RESET                  0x07
#define CAM_REG_FORMAT                        0x20
#define CAM_REG_CAPTURE_RESOLUTION            0x21
#define CAM_REG_BRIGHTNESS_CONTROL            0x22
#define CAM_REG_CONTRAST_CONTROL              0x23
#define CAM_REG_SATURATION_CONTROL            0x24
#define CAM_REG_EV_CONTROL                    0x25
#define CAM_REG_WHITEBALANCE_CONTROL          0x26
#define CAM_REG_COLOR_EFFECT_CONTROL          0x27
#define CAM_REG_SHARPNESS_CONTROL             0x28
#define CAM_REG_AUTO_FOCUS_CONTROL            0x29
#define CAM_REG_IMAGE_QUALITY                 0x2A
#define CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE 0x30
#define CAM_REG_MANUAL_GAIN_BIT_9_8           0x31
#define CAM_REG_MANUAL_GAIN_BIT_7_0           0x32
#define CAM_REG_MANUAL_EXPOSURE_BIT_19_16     0x33
#define CAM_REG_MANUAL_EXPOSURE_BIT_15_8      0x34
#define CAM_REG_MANUAL_EXPOSURE_BIT_7_0       0x35
#define CAM_REG_BURST_FIFO_READ_OPERATION     0x3C
#define CAM_REG_SINGLE_FIFO_READ_OPERATION    0x3D
#define CAM_REG_SENSOR_ID                     0x40
#define CAM_REG_YEAR_SDK                      0x41
#define CAM_REG_MONTH_SDK                     0x42
#define CAM_REG_DAY_SDK                       0x43
#define CAM_REG_SENSOR_STATE                  0x44
#define CAM_REG_FPGA_VERSION_NUMBER           0x49
#define CAM_REG_DEBUG_DEVICE_ADDRESS          0x0A
#define CAM_REG_DEBUG_REGISTER_HIGH           0x0B
#define CAM_REG_DEBUG_REGISTER_LOW            0x0C
#define CAM_REG_DEBUG_REGISTER_VALUE          0x0D

#define SENSOR_STATE_IDLE   (1 << 1)
#define SENSOR_RESET_ENABLE (1 << 6)

#define CTR_WHITEBALANCE 0x02
#define CTR_EXPOSURE     0x01
#define CTR_GAIN         0x00

#define AC_STACK_SIZE 4096
#define AC_PRIORITY 5

K_THREAD_STACK_DEFINE(ac_stack_area, AC_STACK_SIZE);

struct k_work_q ac_work_q;

/**
 * @struct mega_sdk_data
 * @brief Basic information of the camera firmware
 */
struct mega_sdk_data {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t version;
};

struct arducam_mega_config {
    struct spi_dt_spec bus;
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

static struct arducam_mega_info mega_infos[] = {
    {
        .support_resolution = 7894,
        .support_special_effects = 63,
        .exposure_value_max = 30000,
        .exposure_value_min = 1,
        .gain_value_max = 1023,
        .gain_value_min = 1,
        .enable_focus = 1,
        .enable_sharpness = 0,
        .device_address = 0x78,
    },
    {
        .support_resolution = 7638,
        .support_special_effects = 319,
        .exposure_value_max = 30000,
        .exposure_value_min = 1,
        .gain_value_max = 1023,
        .gain_value_min = 1,
        .enable_focus = 0,
        .enable_sharpness = 1,
        .device_address = 0x78,
    }
};

#define ARDUCAM_MEGA_VIDEO_FORMAT_CAP(width, height, format) \
    { \
        .pixelformat = (format), \
        .width_min = (width), \
        .width_max = (width), \
        .height_min = (height), \
        .height_max = (height), \
        .width_step = 0, \
        .height_step = 0 \
    }

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
};

static struct video_caps caps = {
    .type = VIDEO_BUF_TYPE_OUTPUT,
    .format_caps = fmts,
    .min_vbuf_count = 1,
    .min_line_count = LINE_COUNT_HEIGHT,
    .max_line_count = LINE_COUNT_HEIGHT,
};

/* Forward declarations */
static int arducam_mega_get_caps(const struct device *dev, struct video_caps *caps);
void on_stream_schedule_timer_func(struct k_timer *timer);

static uint8_t arducam_mega_read_reg(const struct spi_dt_spec *bus, uint8_t reg_addr)
{
    uint8_t tx_buf[2] = {reg_addr, 0x00};
    uint8_t rx_buf[2] = {0x00, 0x00};
    const struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = 2,
    };
    const struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = 2,
    };
    const struct spi_buf_set tx = {&tx_spi_buf, 1};
    const struct spi_buf_set rx = {&rx_spi_buf, 1};

    if (spi_transceive_dt(bus, &tx, &rx)) {
        return -EIO;
    }

    return rx_buf[1];
}

static int arducam_mega_write_reg(const struct spi_dt_spec *bus, uint8_t reg_addr, uint8_t value)
{
    uint8_t tx_buf[2] = {reg_addr | 0x80, value};
    const struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = 2,
    };
    const struct spi_buf_set tx = {&tx_spi_buf, 1};

    if (spi_write_dt(bus, &tx)) {
        return -EIO;
    }

    return 0;
}

static int arducam_mega_check_connection(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    struct arducam_mega_data *drv_data = dev->data;
    uint8_t sensor_id = arducam_mega_read_reg(&cfg->bus, CAM_REG_SENSOR_ID);

    switch (sensor_id) {
    case ARDUCAM_SENSOR_3MP_1:
    case ARDUCAM_SENSOR_3MP_2:
        drv_data->info = &mega_infos[0];
        break;
    case ARDUCAM_SENSOR_5MP_1:
    case ARDUCAM_SENSOR_5MP_2:
        drv_data->info = &mega_infos[1];
        break;
    default:
        return -ENODEV;
    }

    drv_data->info->camera_id = sensor_id;

    return 0;
}

static int arducam_mega_soft_reset(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    uint8_t val = arducam_mega_read_reg(&cfg->bus, CAM_REG_SENSOR_RESET);

    val |= SENSOR_RESET_ENABLE;
    arducam_mega_write_reg(&cfg->bus, CAM_REG_SENSOR_RESET, val);
    k_msleep(1000);

    return 0;
}

static int arducam_mega_set_brightness(const struct device *dev, enum MEGA_BRIGHTNESS_LEVEL level)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_BRIGHTNESS_CONTROL, level);
}

static int arducam_mega_set_contrast(const struct device *dev, enum MEGA_CONTRAST_LEVEL level)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_CONTRAST_CONTROL, level);
}

static int arducam_mega_set_saturation(const struct device *dev, enum MEGA_SATURATION_LEVEL level)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_SATURATION_CONTROL, level);
}

static int arducam_mega_set_EV(const struct device *dev, enum MEGA_EV_LEVEL level)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_EV_CONTROL, level);
}

static int arducam_mega_set_white_bal(const struct device *dev, enum MEGA_WHITE_BALANCE mode)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_WHITEBALANCE_CONTROL, mode);
}

static int arducam_mega_set_special_effects(const struct device *dev, enum MEGA_COLOR_FX effect)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_COLOR_EFFECT_CONTROL, effect);
}

static int arducam_mega_set_sharpness(const struct device *dev, enum MEGA_SHARPNESS_LEVEL level)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_SHARPNESS_CONTROL, level);
}

static int arducam_mega_set_JPEG_quality(const struct device *dev, enum MEGA_IMAGE_QUALITY quality)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_IMAGE_QUALITY, quality);
}

static int arducam_mega_set_lowpower_enable(const struct device *dev, uint8_t enable)
{
    const struct arducam_mega_config *cfg = dev->config;

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_POWER_CONTROL, enable);
}

static int arducam_mega_set_exposure_enable(const struct device *dev, uint8_t enable)
{
    const struct arducam_mega_config *cfg = dev->config;
    uint8_t val = arducam_mega_read_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE);

    if (enable) {
        val &= ~CTR_EXPOSURE;
    } else {
        val |= CTR_EXPOSURE;
    }

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, val);
}

static int arducam_mega_set_gain_enable(const struct device *dev, uint8_t enable)
{
    const struct arducam_mega_config *cfg = dev->config;
    uint8_t val = arducam_mega_read_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE);

    if (enable) {
        val &= ~CTR_GAIN;
    } else {
        val |= CTR_GAIN;
    }

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, val);
}

static int arducam_mega_set_white_bal_enable(const struct device *dev, uint8_t enable)
{
    const struct arducam_mega_config *cfg = dev->config;
    uint8_t val = arducam_mega_read_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE);

    if (enable) {
        val &= ~CTR_WHITEBALANCE;
    } else {
        val |= CTR_WHITEBALANCE;
    }

    return arducam_mega_write_reg(&cfg->bus, CAM_REG_EXPOSURE_GAIN_WHITEBAL_ENABLE, val);
}

static int arducam_mega_set_exposure(const struct device *dev, uint32_t exposure_value)
{
    const struct arducam_mega_config *cfg = dev->config;
    int ret = 0;

    ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_19_16, (exposure_value >> 16) & 0x0f);
    ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_15_8, (exposure_value >> 8) & 0xff);
    ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_EXPOSURE_BIT_7_0, exposure_value & 0xff);

    return ret;
}

static int arducam_mega_set_gain(const struct device *dev, uint16_t gain_value)
{
    const struct arducam_mega_config *cfg = dev->config;
    int ret = 0;

    ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_GAIN_BIT_9_8, (gain_value >> 8) & 0x03);
    ret |= arducam_mega_write_reg(&cfg->bus, CAM_REG_MANUAL_GAIN_BIT_7_0, gain_value & 0xff);

    return ret;
}

static int arducam_mega_set_fmt(const struct device *dev, struct video_format *fmt)
{
    struct arducam_mega_data *drv_data = dev->data;
    const struct arducam_mega_config *cfg = dev->config;
    int ret = 0;
    uint8_t val = 0;
    enum MEGA_RESOLUTION resolution = MEGA_RESOLUTION_NONE;
    enum MEGA_PIXELFORMAT pixelformat = MEGA_PIXELFORMAT_JPG;
    unsigned int bpp = video_bits_per_pixel(fmt->pixelformat);

    if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
        return -ENOTSUP;
    }

    if (bpp == 0 && fmt->pixelformat != VIDEO_PIX_FMT_JPEG) {
        return -ENOTSUP;
    }

    switch (fmt->pixelformat) {
    case VIDEO_PIX_FMT_RGB565:
        pixelformat = MEGA_PIXELFORMAT_RGB565;
        fmt->pitch = fmt->width * 2;
        break;
    case VIDEO_PIX_FMT_YUYV:
        pixelformat = MEGA_PIXELFORMAT_YUV;
        fmt->pitch = fmt->width * 2;
        break;
    case VIDEO_PIX_FMT_JPEG:
        pixelformat = MEGA_PIXELFORMAT_JPG;
        fmt->pitch = fmt->width * 3; // Approximate for max
        break;
    default:
        return -ENOTSUP;
    }

    for (enum MEGA_RESOLUTION i = MEGA_RESOLUTION_QQVGA; i < MEGA_RESOLUTION_NONE; i++) {
        if (resolution_table[i][0] == fmt->width && resolution_table[i][1] == fmt->height) {
            resolution = i;
            break;
        }
    }

    if (resolution == MEGA_RESOLUTION_NONE) {
        return -ENOTSUP;
    }

    val = (pixelformat << 4) | resolution;

    ret = arducam_mega_write_reg(&cfg->bus, CAM_REG_CAPTURE_RESOLUTION, val);

    if (ret == 0) {
        drv_data->fmt = *fmt;
    }

    return ret;
}

static int arducam_mega_get_fmt(const struct device *dev, struct video_format *fmt)
{
    struct arducam_mega_data *drv_data = dev->data;

    *fmt = drv_data->fmt;

    return 0;
}

static int arducam_mega_stream_start(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    struct arducam_mega_data *drv_data = dev->data;
    uint8_t val = arducam_mega_read_reg(&cfg->bus, ARDUCHIP_FIFO);

    val |= FIFO_CLEAR_ID_MASK;
    arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO, val);

    val = arducam_mega_read_reg(&cfg->bus, ARDUCHIP_FIFO_2);
    val |= FIFO_START_MASK;
    arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO_2, val);

    drv_data->stream_on = 1;

    return 0;
}

static int arducam_mega_stream_stop(const struct device *dev)
{
    struct arducam_mega_data *drv_data = dev->data;

    drv_data->stream_on = 0;

    return 0;
}

static int arducam_mega_get_caps(const struct device *dev, struct video_caps *caps)
{
    *caps = caps;

    return 0;
}

static int arducam_mega_flush(const struct device *dev, bool cancel)
{
    struct arducam_mega_data *drv_data = dev->data;
    struct video_buffer *buf;

    while ((buf = k_fifo_get(&drv_data->fifo_in, K_NO_WAIT)) != NULL) {
        if (cancel) {
            buf->bytesused = 0;
        }
        k_fifo_put(&drv_data->fifo_out, buf);
    }

    return 0;
}

static void __buffer_work(struct k_work *work)
{
    struct arducam_mega_data *drv_data = CONTAINER_OF(work, struct arducam_mega_data, buf_work);
    const struct arducam_mega_config *cfg = drv_data->dev->config;
    struct video_buffer *buf = k_fifo_get(&drv_data->fifo_in, K_NO_WAIT);
    uint8_t val;

    if (buf == NULL) {
        return;
    }

    buf->line_offset = 0;

    if (drv_data->fifo_first_read) {
        drv_data->fifo_first_read = 0;
        drv_data->fifo_length = (arducam_mega_read_reg(&cfg->bus, FIFO_SIZE1) & 0xff) |
                                ((arducam_mega_read_reg(&cfg->bus, FIFO_SIZE2) & 0xff) << 8) |
                                ((arducam_mega_read_reg(&cfg->bus, FIFO_SIZE3) & 0x07) << 16);
    }

    if (drv_data->fifo_length > buf->size) {
        buf->bytesused = 0;
        k_fifo_put(&drv_data->fifo_out, buf);
        if (drv_data->signal) {
            k_poll_signal_raise(drv_data->signal, VIDEO_BUF_ERROR);
        }
        return;
    }

    // Read all data at once since full frame
    for (uint32_t i = 0; i < drv_data->fifo_length; i++) {
        buf->buffer[i] = arducam_mega_read_reg(&cfg->bus, BURST_FIFO_READ);
    }
    buf->bytesused = drv_data->fifo_length;

    drv_data->fifo_length = 0;
    drv_data->fifo_first_read = 1;

    k_fifo_put(&drv_data->fifo_out, buf);

    if (drv_data->signal) {
        k_poll_signal_raise(drv_data->signal, VIDEO_BUF_DONE);
    }

    // Start next capture if streaming
    if (drv_data->stream_on) {
        val = arducam_mega_read_reg(&cfg->bus, ARDUCHIP_FIFO);
        val |= FIFO_CLEAR_ID_MASK;
        arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO, val);

        val = arducam_mega_read_reg(&cfg->bus, ARDUCHIP_FIFO_2);
        val |= FIFO_START_MASK;
        arducam_mega_write_reg(&cfg->bus, ARDUCHIP_FIFO_2, val);
    }
}

static int arducam_mega_enqueue(const struct device *dev, struct video_buffer *buf)
{
    struct arducam_mega_data *drv_data = dev->data;

    if (buf->type != VIDEO_BUF_TYPE_OUTPUT) {
        return -ENOTSUP;
    }

    k_fifo_put(&drv_data->fifo_in, buf);

    if (drv_data->stream_on) {
        k_work_submit_to_queue(&ac_work_q, &drv_data->buf_work);
    }

    return 0;
}

static int arducam_mega_dequeue(const struct device *dev, struct video_buffer **buf, k_timeout_t timeout)
{
    struct arducam_mega_data *drv_data = dev->data;

    *buf = k_fifo_get(&drv_data->fifo_out, timeout);

    return (*buf == NULL) ? -EAGAIN : 0;
}

static int arducam_mega_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
    int ret = 0;
    struct video_control *ctrl = (struct video_control *)value;

    switch (cid) {
    case VIDEO_CID_EXPOSURE_AUTO:
        {
            enum video_exposure_type type = (enum video_exposure_type)ctrl->val;
            uint8_t enable = (type == VIDEO_EXPOSURE_AUTO) ? 1 : 0;
            ret |= arducam_mega_set_exposure_enable(dev, enable);
        }
        break;
    case VIDEO_CID_EXPOSURE_ABSOLUTE:
        ret |= arducam_mega_set_exposure(dev, (uint32_t)ctrl->val);
        break;
    case VIDEO_CID_AUTOGAIN:
        ret |= arducam_mega_set_gain_enable(dev, (uint8_t)ctrl->val);
        break;
    case VIDEO_CID_GAIN:
        ret |= arducam_mega_set_gain(dev, (uint16_t)ctrl->val);
        break;
    case VIDEO_CID_BRIGHTNESS:
        ret |= arducam_mega_set_brightness(dev, (enum MEGA_BRIGHTNESS_LEVEL)ctrl->val);
        break;
    case VIDEO_CID_SATURATION:
        ret |= arducam_mega_set_saturation(dev, (enum MEGA_SATURATION_LEVEL)ctrl->val);
        break;
    case VIDEO_CID_AUTO_WHITE_BALANCE:
        ret |= arducam_mega_set_white_bal_enable(dev, (uint8_t)ctrl->val);
        break;
    case VIDEO_CID_WHITE_BALANCE_TEMPERATURE:
        ret |= arducam_mega_set_white_bal(dev, (enum MEGA_WHITE_BALANCE)ctrl->val);
        break;
    case VIDEO_CID_CONTRAST:
        ret |= arducam_mega_set_contrast(dev, (enum MEGA_CONTRAST_LEVEL)ctrl->val);
        break;
    case VIDEO_CID_JPEG_COMPRESSION_QUALITY:
        ret |= arducam_mega_set_JPEG_quality(dev, (enum MEGA_IMAGE_QUALITY)ctrl->val);
        break;
    case VIDEO_CID_ARDUCAM_EV:
        ret |= arducam_mega_set_EV(dev, (enum MEGA_EV_LEVEL)ctrl->val);
        break;
    case VIDEO_CID_SHARPNESS:
        ret |= arducam_mega_set_sharpness(dev, (enum MEGA_SHARPNESS_LEVEL)ctrl->val);
        break;
    case VIDEO_CID_COLORFX:
        ret |= arducam_mega_set_special_effects(dev, (enum MEGA_COLOR_FX)ctrl->val);
        break;
    case VIDEO_CID_ARDUCAM_RESET:
        ret |= arducam_mega_soft_reset(dev);
        ret |= arducam_mega_check_connection(dev);
        break;
    case VIDEO_CID_ARDUCAM_LOWPOWER:
        ret |= arducam_mega_set_lowpower_enable(dev, (uint8_t)ctrl->val);
        break;
    default:
        return -ENOTSUP;
    }

    return ret;
}

static int arducam_mega_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
    int ret = 0;
    struct video_control *ctrl = (struct video_control *)value;

    switch (cid) {
    case VIDEO_CID_ARDUCAM_INFO:
        ret |= arducam_mega_get_info(dev, (struct arducam_mega_info *)ctrl->val);
        break;
    default:
        return -ENOTSUP;
    }

    return ret;
}

static int arducam_mega_set_signal(const struct device *dev, struct k_poll_signal *signal)
{
    struct arducam_mega_data *drv_data = dev->data;

    drv_data->signal = signal;

    return 0;
}

static const struct video_driver_api arducam_mega_driver_api = {
    .set_format = arducam_mega_set_fmt,
    .get_format = arducam_mega_get_fmt,
    .stream_start = arducam_mega_stream_start,
    .stream_stop = arducam_mega_stream_stop,
    .get_caps = arducam_mega_get_caps,
    .flush = arducam_mega_flush,
    .set_ctrl = arducam_mega_set_ctrl,
    .get_ctrl = arducam_mega_get_ctrl,
    .enqueue = arducam_mega_enqueue,
    .dequeue = arducam_mega_dequeue,
    .set_signal = arducam_mega_set_signal,
};

static int arducam_mega_init(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    struct arducam_mega_data *drv_data = dev->data;

    struct video_format fmt;
    int ret = 0;

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("%s: device is not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    drv_data->dev = dev;
    k_fifo_init(&drv_data->fifo_in);
    k_fifo_init(&drv_data->fifo_out);
    k_work_queue_init(&ac_work_q);
    k_work_queue_start(&ac_work_q, ac_stack_area, K_THREAD_STACK_SIZEOF(ac_stack_area),
        AC_PRIORITY, NULL);

    k_timer_init(&drv_data->stream_schedule_timer, on_stream_schedule_timer_func, NULL);
    drv_data->stream_schedule_timer.user_data = (void *)drv_data;

    k_work_init(&drv_data->buf_work, __buffer_work);

    arducam_mega_soft_reset(dev);
    ret = arducam_mega_check_connection(dev);

    if (ret) {
        LOG_ERR("arducam mega camera not connected.\n");
        return ret;
    }

    drv_data->ver.year = arducam_mega_read_reg(&cfg->bus, CAM_REG_YEAR_SDK) & 0x3F;
    drv_data->ver.month = arducam_mega_read_reg(&cfg->bus, CAM_REG_MONTH_SDK) & 0x0F;
    drv_data->ver.day = arducam_mega_read_reg(&cfg->bus, CAM_REG_DAY_SDK) & 0x1F;
    drv_data->ver.version = arducam_mega_read_reg(&cfg->bus, CAM_REG_FPGA_VERSION_NUMBER) & 0xFF;

    LOG_INF("arducam mega ver: %d-%d-%d \t %x", drv_data->ver.year, drv_data->ver.month,
        drv_data->ver.day, drv_data->ver.version);

    /* set default/init format 96x96 RGB565 */
    fmt.type = VIDEO_BUF_TYPE_OUTPUT;
    fmt.pixelformat = VIDEO_PIX_FMT_RGB565;
    fmt.width = 96;
    fmt.height = 96;
    fmt.pitch = 96 * 2;
    ret = arducam_mega_set_fmt(dev, &fmt);
    if (ret) {
        LOG_ERR("Unable to configure default format");
        return -EIO;
    }

    return ret;
}

#define ARDUCAM_MEGA_INIT(inst) \
    static const struct arducam_mega_config arducam_mega_cfg_##inst = { \
        .bus = SPI_DT_SPEC_INST_GET(inst, \
                                    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | \
                                    SPI_CS_ACTIVE_HIGH | SPI_LINES_SINGLE | \
                                    SPI_LOCK_ON, \
                                    0), \
    }; \
    static struct arducam_mega_data arducam_mega_data_##inst; \
    DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, &arducam_mega_data_##inst, \
                          &arducam_mega_cfg_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, \
                          &arducam_mega_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_INIT)