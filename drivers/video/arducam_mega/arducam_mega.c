/**
 * SPDX-License-Identifier: Apache-2.0
 * Arducam Mega driver for Zephyr (custom module)
 */

#define DT_DRV_COMPAT arducam_mega

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/video.h>           /* Your custom video.h */
#include <drivers/video/arducam_mega.h>    /* Your custom arducam_mega.h */
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mega_camera);

struct arducam_mega_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
};

struct arducam_mega_data {
    const struct device *dev;
    struct video_format fmt;
    uint8_t fifo_first_read;
    uint32_t fifo_length;
    uint8_t stream_on;
};

/* --- SPI helper functions --- */

static int arducam_mega_write_reg(const struct arducam_mega_config *cfg,
                                  uint8_t reg_addr, uint8_t value)
{
    uint8_t tx[2] = { reg_addr | 0x80, value };

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_write_dt(&cfg->bus, &(struct spi_buf_set){
        .buffers = (struct spi_buf[]){{.buf = tx, .len = 2}},
        .count = 1
    });
    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    return ret;
}

static int arducam_mega_read_reg(const struct arducam_mega_config *cfg,
                                 uint8_t reg_addr)
{
    uint8_t tx = reg_addr & 0x7F;
    uint8_t rx = 0;

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_transceive_dt(&cfg->bus,
        &(struct spi_buf_set){ .buffers = (struct spi_buf[]){{.buf = &tx, .len = 1}}, .count = 1 },
        &(struct spi_buf_set){ .buffers = (struct spi_buf[]){{.buf = &rx, .len = 1}}, .count = 1 }
    );
    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    return ret ? ret : rx;
}

static int arducam_mega_read_block(const struct arducam_mega_config *cfg,
                                   uint8_t *img_buff, uint32_t img_len, uint8_t first)
{
    uint8_t cmd_fifo_read[2] = { 0x3C, 0x00 }; /* BURST_FIFO_READ */
    uint8_t tx_len = first ? 2 : 1;

    struct spi_buf tx_buf = { .buf = cmd_fifo_read, .len = tx_len };
    struct spi_buf rx_bufs[2] = {
        { .buf = cmd_fifo_read, .len = tx_len },
        { .buf = img_buff, .len = img_len }
    };

    gpio_pin_set_dt(&cfg->cs_gpio, 0);
    int ret = spi_transceive_dt(&cfg->bus,
        &(struct spi_buf_set){ .buffers = &tx_buf, .count = 1 },
        &(struct spi_buf_set){ .buffers = rx_bufs, .count = 2 }
    );
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
