#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec arducam_cs =
    GPIO_DT_SPEC_GET(DT_NODELABEL(arducam0), cs_gpios);

static struct spi_config spi_cfg = {
    .frequency = 8000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .cs = NULL, /* manual CS */
};

/* --- ArduCAM register helpers --- */
static int arducam_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), val };
    struct spi_buf tx_buf = { .buf = tx, .len = 2 };
    struct spi_buf_set txs = { &tx_buf, 1 };

    gpio_pin_set_dt(&arducam_cs, 0);
    int ret = spi_write(spi_dev, &spi_cfg, &txs);
    gpio_pin_set_dt(&arducam_cs, 1);
    return ret;
}

static int arducam_read_reg(uint8_t addr, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = {0};
    struct spi_buf tx_buf = { .buf = tx, .len = 2 };
    struct spi_buf rx_buf = { .buf = rx, .len = 2 };
    struct spi_buf_set txs = { &tx_buf, 1 };
    struct spi_buf_set rxs = { &rx_buf, 1 };

    gpio_pin_set_dt(&arducam_cs, 0);
    int ret = spi_transceive(spi_dev, &spi_cfg, &txs, &rxs);
    gpio_pin_set_dt(&arducam_cs, 1);

    *val = rx[1];
    return ret;
}

/* --- ArduCAM commands --- */
#define ARDUCHIP_FIFO        0x04
#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02
#define FIFO_RDPTR_RST_MASK  0x10
#define ARDUCHIP_TRIG        0x41

void arducam_start_capture(void)
{
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

bool arducam_capture_done(void)
{
    uint8_t val = 0;
    arducam_read_reg(ARDUCHIP_TRIG, &val);
    return (val & 0x08);
}

uint32_t arducam_read_fifo_length(void)
{
    uint8_t len1, len2, len3;
    arducam_read_reg(0x42, &len1);
    arducam_read_reg(0x43, &len2);
    arducam_read_reg(0x44, &len3);
    return ((uint32_t)len3 << 16) | ((uint32_t)len2 << 8) | len1;
}

void main(void)
{
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI1 device not ready");
        return;
    }
    if (!device_is_ready(arducam_cs.port)) {
        LOG_ERR("CS GPIO not ready");
        return;
    }
    gpio_pin_configure_dt(&arducam_cs, GPIO_OUTPUT_INACTIVE);

    LOG_INF("ArduCAM minimal capture test starting");

    while (1) {
        LOG_INF("Starting capture...");
        arducam_start_capture();

        /* Poll until capture done */
        int tries = 1000;
        while (!arducam_capture_done() && tries-- > 0) {
            k_sleep(K_MSEC(10));
        }

        if (tries <= 0) {
            LOG_ERR("Capture timeout!");
            continue;
        }

        uint32_t len = arducam_read_fifo_length();
        LOG_INF("Capture complete, FIFO length = %u bytes", len);

        /* Just print first few bytes of FIFO */
        gpio_pin_set_dt(&arducam_cs, 0);
        uint8_t cmd = 0x3C; /* burst read */
        struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
        struct spi_buf_set txs = { &tx_buf, 1 };
        spi_write(spi_dev, &spi_cfg, &txs);

        uint8_t buf[16] = {0};
        struct spi_buf rx_buf = { .buf = buf, .len = sizeof(buf) };
        struct spi_buf_set rxs = { &rx_buf, 1 };
        spi_read(spi_dev, &spi_cfg, &rxs);
        gpio_pin_set_dt(&arducam_cs, 1);

        LOG_HEXDUMP_INF(buf, sizeof(buf), "FIFO[0:16]");

        k_sleep(K_SECONDS(5));
    }
}
