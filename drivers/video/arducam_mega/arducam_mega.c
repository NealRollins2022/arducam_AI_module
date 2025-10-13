/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <drivers/video/arducam_mega.h>

LOG_MODULE_REGISTER(mega_camera, LOG_LEVEL_INF);

struct arducam_mega_data {
    const struct device *dev;
    uint8_t fifo_first_read;
    uint32_t fifo_length;
    uint8_t stream_on;
};

struct arducam_mega_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
};

/* --- Helper SPI functions --- */
static int arducam_mega_write_reg(const struct arducam_mega_config *cfg,
                                  uint8_t reg_addr, uint8_t value)
{
    reg_addr |= 0x80;
    struct spi_buf tx_buf[2] = {
        { .buf = &reg_addr, .len = 1 },
        { .buf = &value, .len = 1 }
    };
    struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 2 };

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_write_dt(&cfg->bus, &tx_bufs);
    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    LOG_INF("Write reg 0x%x = 0x%x, ret=%d", reg_addr, value, ret);
    return ret;
}

static int arducam_mega_read_reg(const struct arducam_mega_config *cfg, uint8_t reg_addr)
{
    reg_addr &= 0x7F;
    uint8_t value = 0x00;

    struct spi_buf tx_buf[] = { { .buf = &reg_addr, .len = 1 } };
    struct spi_buf rx_buf[] = { { .buf = &value, .len = 1 } };
    struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 1 };
    struct spi_buf_set rx_bufs = { .buffers = rx_buf, .count = 1 };

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs);
    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    LOG_INF("Read reg 0x%x -> 0x%x, ret=%d", reg_addr, value, ret);
    return value;
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

    LOG_INF("Initializing Arducam Mega");

    if (cfg->cs_gpio.port) {
        int ret = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_HIGH);
        LOG_INF("CS GPIO configure ret=%d", ret);
    }

    LOG_INF("SPI bus ready: %p", &cfg->bus);

    return 0;
}

/* --- Device macro --- */
#define ARDUCAM_MEGA_INIT(inst) \
static const struct arducam_mega_config arducam_mega_cfg_##inst = { \
    .bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE, 0), \
    .cs_gpio = { .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), .pin = 25, .dt_flags = GPIO_ACTIVE_LOW } \
}; \
static struct arducam_mega_data arducam_mega_data_##inst; \
DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, &arducam_mega_data_##inst, &arducam_mega_cfg_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_INIT)
