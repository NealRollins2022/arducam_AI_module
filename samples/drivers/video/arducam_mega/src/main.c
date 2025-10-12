/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/video.h>
#include <drivers/video/arducam_mega.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(main);

/* UART message queue */
#define MSG_SIZE 12
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* UART header/footer */
static uint8_t uart_header[] = {0xFF, 0xAA};
static uint8_t uart_footer[] = {0xFF, 0xBB};

/* Devices */
static const struct device *console;
static const struct device *video;
static struct video_buffer *vbuf;

/* Flags */
static volatile uint8_t preview_on;
static volatile uint8_t capture_flag;

/* Resolution tables */
static const uint32_t pixel_format_table[] = {
    VIDEO_PIX_FMT_JPEG,
    VIDEO_PIX_FMT_RGB565,
    VIDEO_PIX_FMT_YUYV,
};

static const uint16_t resolution_table[][2] = {
    {160, 120}, {320, 240}, {640, 480}, {800, 600}, {1280, 720},
    {1280, 960}, {1600, 1200}, {1920, 1080}, {2048, 1536}, {2592, 1944},
    {96, 96}, {128, 128}, {320, 320},
};

static const uint8_t resolution_num = sizeof(resolution_table)/sizeof(resolution_table[0]);

/* Current state */
static uint8_t current_resolution;
static uint8_t take_picture_fmt = 0x1A;

/* --- Helper functions --- */
static void uart_buffer_send(const struct device *dev, uint8_t *buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        uart_poll_out(dev, buffer[i]);
    }
}

static void uart_packet_send(uint8_t type, uint8_t *buffer, uint32_t length)
{
    uart_header[2] = type;
    uart_buffer_send(console, uart_header, 2);
    uart_buffer_send(console, (uint8_t *)&length, sizeof(length));
    uart_buffer_send(console, buffer, length);
    uart_buffer_send(console, uart_footer, sizeof(uart_footer));
}

/* Set video resolution */
static int set_mega_resolution(uint8_t sfmt)
{
    uint8_t resolution = sfmt & 0x0F;
    uint8_t pixelformat = (sfmt & 0x70) >> 4;

    if (resolution >= resolution_num || pixelformat == 0 || pixelformat > 3) {
        return -1;
    }

    struct video_format fmt = {
        .width = resolution_table[resolution][0],
        .height = resolution_table[resolution][1],
        .pixelformat = pixel_format_table[pixelformat - 1]
    };
    current_resolution = resolution;

    return video_set_format(video, VIDEO_EP_OUT, &fmt);
}

/* Take a single picture */
static int take_picture(void)
{
    int err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
    if (err) {
        LOG_ERR("Unable to dequeue video buffer");
        return -1;
    }

    uart_packet_send(0x01, vbuf->buffer, vbuf->bytesused);
    video_enqueue(video, VIDEO_EP_OUT, vbuf);
    return 0;
}

/* Video preview streaming */
static void video_preview(void)
{
    if (!preview_on) return;

    if (video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER)) return;

    uart_buffer_send(console, vbuf->buffer, vbuf->bytesused);

    video_enqueue(video, VIDEO_EP_OUT, vbuf);
}

/* UART receive callback */
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

/* --- Process UART commands --- */
static void recv_process(uint8_t *buff)
{
    switch (buff[0]) {
    case 0x01: /* SET_PICTURE_RESOLUTION */
        if (set_mega_resolution(buff[1]) == 0) take_picture_fmt = buff[1];
        break;
    case 0x02: /* SET_VIDEO_RESOLUTION */
        if (!preview_on) {
            set_mega_resolution(buff[1] | 0x10);
            video_stream_start(video);
            capture_flag = 1;
        }
        preview_on = 1;
        break;
    case 0x10: /* TAKE_PICTURE */
        video_stream_start(video);
        take_picture();
        video_stream_stop(video);
        break;
    case 0x21: /* STOP_STREAM */
        if (preview_on) {
            video_stream_stop(video);
            set_mega_resolution(take_picture_fmt);
        }
        preview_on = 0;
        break;
    default:
        break;
    }
}

/* --- Check UART queue --- */
static uint8_t uart_available(uint8_t *p)
{
    return k_msgq_get(&uart_msgq, p, K_NO_WAIT) == 0;
}

/* --- Main --- */
int main(void)
{
    console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(console)) return -1;

    uart_irq_callback_user_data_set(console, serial_cb, NULL);
    uart_irq_rx_enable(console);

    video = DEVICE_DT_GET(DT_NODELABEL(arducam0));
    if (!device_is_ready(video)) return -1;

    struct video_buffer *buffers[3];
    for (int i = 0; i < 3; i++) {
        buffers[i] = video_buffer_alloc(1024);
        if (!buffers[i]) return -1;
        video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
    }

    LOG_INF("Mega Camera started");

    uint8_t recv_buffer[MSG_SIZE];
    while (1) {
        if (uart_available(recv_buffer)) {
            recv_process(recv_buffer);
        }
        video_preview();
        k_msleep(1);
    }
}

