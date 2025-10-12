/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT arducam_mega

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <drivers/video.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(mega_camera);

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

/* Driver config holds SPI bus and manual CS */
struct arducam_mega_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
};

/* Driver runtime data */
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

/* --- Helper SPI functions with manual CS --- */

/* helper: set CS if available */
static inline void arducam_cs_set(const struct gpio_dt_spec *cs, int val)
{
    if (cs == NULL) {
        return;
    }
    if (cs->port == NULL) {
        return;
    }
    /* device_is_ready check is optional if you configured during init,
     * but safe to do here anyway.
     */
    if (!device_is_ready(cs->port)) {
        LOG_WRN("CS gpio device not ready");
        return;
    }
    gpio_pin_set_dt(cs, val);
}

static int arducam_mega_write_reg(const struct arducam_mega_config *cfg,
                                  uint8_t reg_addr, uint8_t value)
{
    uint8_t tries = 3;
    reg_addr |= 0x80;

    struct spi_buf tx_buf[2] = {
        {.buf = &reg_addr, .len = 1},
        {.buf = &value, .len = 1}
    };
    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 2};

    while (tries--) {
        arducam_cs_set(&cfg->cs_gpio, 0);
        if (!spi_write_dt(&cfg->bus, &tx_bufs)) {
            arducam_cs_set(&cfg->cs_gpio, 1);
            return 0;
        }
        arducam_cs_set(&cfg->cs_gpio, 1);
        k_msleep(5);
    }
    LOG_ERR("failed to write 0x%x to 0x%x", value, reg_addr);
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
        arducam_cs_set(&cfg->cs_gpio, 0);
        if (!spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs)) {
            arducam_cs_set(&cfg->cs_gpio, 1);
            return value;
        }
        arducam_cs_set(&cfg->cs_gpio, 1);
        k_msleep(5);
    }
    LOG_ERR("failed to read 0x%x register", reg_addr);
    return -1;
}

static int arducam_mega_read_block(const struct arducam_mega_config *cfg,
                                   uint8_t *img_buff,
                                   uint32_t img_len,
                                   uint8_t first)
{
    uint8_t cmd_fifo_read[] = {0x3C, 0x00}; /* BURST_FIFO_READ */
    uint8_t buf_len = first ? 2 : 1;

    struct spi_buf tx_buf[] = {{.buf = cmd_fifo_read, .len = buf_len}};
    struct spi_buf rx_buf[2] = {
        {.buf = cmd_fifo_read, .len = buf_len},
        {.buf = img_buff, .len = img_len}
    };

    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 1};
    struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 2};

    arducam_cs_set(&cfg->cs_gpio, 0);
    int ret = spi_transceive_dt(&cfg->bus, &tx_bufs, &rx_bufs);
    arducam_cs_set(&cfg->cs_gpio, 1);

    return ret;
}

/* --- Initialization --- */

static int arducam_mega_init(const struct device *dev)
{
    const struct arducam_mega_config *cfg = dev->config;
    struct arducam_mega_data *data = dev->data;
    int rc = 0;

    data->dev = dev;
    data->fifo_first_read = 1;
    data->fifo_length = 0;
    data->stream_on = 0;

    /* configure CS pin high manually if provided */
    if (cfg->cs_gpio.port) {
        if (!device_is_ready(cfg->cs_gpio.port)) {
            LOG_ERR("CS gpio device not ready for %s", dev->name);
        } else {
            rc = gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_HIGH);
            if (rc) {
                LOG_ERR("Failed to configure CS pin (%d)", rc);
            } else {
                /* ensure it starts high */
                gpio_pin_set_dt(&cfg->cs_gpio, 1);
                LOG_DBG("CS gpio configured on port %p pin %u", cfg->cs_gpio.port,
                        cfg->cs_gpio.pin);
            }
        }
    } else {
        LOG_DBG("No CS gpio provided (manual CS disabled)");
    }

    return 0;
}
/* --- Device instance macro --- */
#define ARDUCAM_MEGA_INIT(inst) \
static const struct arducam_mega_config arducam_mega_cfg_##inst = { \
    .bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE, 0), \
    .cs_gpio = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(cs_arducam##inst), gpios, {0}), \
}; \
static struct arducam_mega_data arducam_mega_data_##inst; \
DEVICE_DT_INST_DEFINE(inst, &arducam_mega_init, NULL, &arducam_mega_data_##inst, &arducam_mega_cfg_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, NULL);


DT_INST_FOREACH_STATUS_OKAY(ARDUCAM_MEGA_INIT)

