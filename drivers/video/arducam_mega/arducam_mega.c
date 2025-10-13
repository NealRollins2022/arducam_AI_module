/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arducam_mega

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(mega_camera);

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
    uint8_t fifo_first_read;
    uint32_t fifo_length;
    uint8_t stream_on;
};

/* --- SPI helper functions with manual CS --- */

static int arducam_mega_write_reg(const struct arducam_mega_config *cfg, uint8_t reg_addr, uint8_t value)
{
    uint8_t tries = 3;
    reg_addr |= 0x80;

    struct spi_buf tx_buf[2] = {
        {.buf = &reg_addr, .len = 1},
        {.buf = &value, .len = 1}
    };
    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 2};

    while (tries--) {
        gpio_pin_set_dt(&cfg->cs_gpio, 0);
        if (!spi_write_dt(&cfg->bus, &tx_bufs)) {
            gpio_pin_set_dt(&cfg->cs_gpio, 1);
            return 0;
        }
        gpio_pin_set_dt(&cfg->cs_gpio, 1);
        k_msleep(5);
    }
    LOG_ERR("Failed to write 0x%x to 0x%x", value, reg_addr);
    return -1;
}

static int arducam_mega_read_reg(const struct arducam_mega_config *cfg, uint8_t reg_addr)
{
    uint8_t tries = 3;
    uint8_t value;
    reg_addr &= 0x7F;

    struct spi_buf tx_buf[] = {{.buf = &reg_addr, .len = 1}};
    struct spi_buf rx_buf[] = {{.buf = &value, .len = 1}};
    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 1};

    while (tries--) {
        gpio_pin_set_dt(&cfg->cs_gpio, 0);
        if (!spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs)) {
            gpio_pin_set_dt(&cfg->cs_gpio, 1);
            return value;
        }
        gpio_pin_set_dt(&cfg->cs_gpio, 1);
        k_msleep(5);
    }
    LOG_ERR("Failed to read 0x%x register", reg_addr);
    return -1;
}

static int arducam_mega_read_block(const struct arducam_mega_config *cfg,
                                   uint8_t *img_buff,
                                   uint32_t img_len,
                                   uint8_t first)
{
    uint8_t cmd_fifo_read[] = {0x3C, 0x00};
    uint8_t buf_len = first ? 2 : 1;

    struct spi_buf tx_buf[] = {{.buf = cmd_fifo_read, .len = buf_len}};
    struct spi_buf rx_buf[2] = {
        {.buf = cmd_fifo_read, .len = buf_len},
        {.buf = img_buff, .len = img_len}
    };
    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 2};

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs);
    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    return ret;
}

/* --- Initialization --- */

static int arducam_mega_init(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    struct arducam_mega_data *data = dev->data;

    data->dev = dev;
    data->fifo_first_read = 1;
    data->fifo_length = 0;
    data->stream_on = 0;

    if (cfg->cs_gpio.port) {
        gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_HIGH);
    }

    return 0;
}

/* --- Device instance macro --- */

#define ARDUCAM_MEGA_INIT(inst) \
static const struct arducam_mega_config arducam_mega_cfg_##inst = { \
    .bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE, 0), \
    .cs_gpio = { \
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), \
        .pin = 25, \
        .dt_flags = GPIO_ACTIVE_LOW, \
    }, \
}; \
static struct arducam_mega_data arducam_mega_data_##inst; \
DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, &arducam_mega_data_##inst, &arducam_mega_cfg_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_INIT)
