#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <drivers/video.h>
#include <drivers/video/arducam_mega.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define MSG_SIZE 12
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *console;
static const struct device *video;
static struct video_buffer *vbuf;
static volatile uint8_t preview_on = 0;

static uint8_t uart_header[] = {0xFF, 0xAA};
static uint8_t uart_footer[] = {0xFF, 0xBB};

static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

static void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) return;

    while (uart_fifo_read(dev, &c, 1) == 1) {
        if (c == 0xAA && rx_buf_pos > 0) {
            k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
            rx_buf_pos = 0;
        } else if (c == 0x55) {
            rx_buf_pos = 0;
        } else {
            rx_buf[rx_buf_pos++] = c;
            if (rx_buf_pos >= MSG_SIZE) rx_buf_pos = 0;
        }
    }
}

int main(void)
{
    console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(console)) {
        return -1;
    }
    uart_irq_callback_user_data_set(console, serial_cb, NULL);
    uart_irq_rx_enable(console);
    LOG_INF("UART initialized");

    video = DEVICE_DT_GET(DT_NODELABEL(arducam0));
    if (!device_is_ready(video)) {
        LOG_ERR("Video device not ready");
        return -1;
    }
    LOG_INF("Video device ready: %p", video);

    struct video_buffer *buffers[3];
    for (int i = 0; i < 3; i++) {
        buffers[i] = video_buffer_alloc(1024);
        if (!buffers[i]) {
            LOG_ERR("Failed to allocate video buffer %d", i);
            return -1;
        }
        video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
        LOG_INF("Enqueued buffer %d", i);
    }

    LOG_INF("Mega Camera started");

    uint8_t recv_buffer[MSG_SIZE];
    while (1) {
        if (k_msgq_get(&uart_msgq, recv_buffer, K_NO_WAIT) == 0) {
            LOG_INF("Received UART command 0x%x", recv_buffer[0]);
        }
        k_msleep(1);
    }
}
