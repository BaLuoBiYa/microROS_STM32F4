#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "cmsis_os.h"
#include "main.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "usbd_cdc_if.h"
#include "usb_device.h"

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- USB CDC Handles (defined by CubeMX in usb_device.c / usbd_cdc_if.c) ---
extern USBD_HandleTypeDef hUsbDeviceFS;

    // --- micro-ROS Transports ---
    #define USB_CDC_BUFFER_SIZE 2048

// 缓冲区放在 CCMRAM（64KB @ 0x10000000），不占用 SRAM
static uint8_t cdc_buffer[USB_CDC_BUFFER_SIZE] __attribute__((section(".ccmram")));
static volatile size_t cdc_head  = 0;
static volatile size_t cdc_tail  = 0;
static volatile bool cdc_tx_done = true;

// ===================================================================
// CDC 回调（在 USB ISR 上下文中执行）
// ===================================================================

static int8_t cdc_receive_cb(uint8_t *Buf, uint32_t *Len)
{
    // 单字节逐入环形缓冲区
    for (uint32_t i = 0; i < *Len; i++) {
        size_t next = (cdc_tail + 1) % USB_CDC_BUFFER_SIZE;
        if (next == cdc_head) {
            // 缓冲区满，丢弃最旧字节
            cdc_head = (cdc_head + 1) % USB_CDC_BUFFER_SIZE;
        }
        cdc_buffer[cdc_tail] = Buf[i];
        cdc_tail             = next;
    }

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    return USBD_OK;
}

static int8_t cdc_transmit_cplt_cb(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void) Buf;
    (void) Len;
    (void) epnum;
    cdc_tx_done = true;
    return USBD_OK;
}

// ===================================================================
// micro-ROS transport API
// ===================================================================

bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
    (void) transport;

    // 只覆盖 Receive 和 TransmitCplt，Control 沿用 CubeMX 默认实现
    USBD_Interface_fops_FS.Receive      = cdc_receive_cb;
    USBD_Interface_fops_FS.TransmitCplt = cdc_transmit_cplt_cb;

    // 等待 USB 枚举完成（最长等 3 秒）
    uint32_t wait_ms = 0;
    while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        osDelay(10);
        wait_ms += 10;
        if (wait_ms >= 3000) {
            return false;  // USB 未连接
        }
    }

    // 重置环形缓冲区指针，避免 session 重连时残留旧数据
    cdc_head    = 0;
    cdc_tail    = 0;
    cdc_tx_done = true;

    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
    (void) transport;
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              uint8_t *buf, size_t len, uint8_t *err)
{
    (void) transport;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return 0;
    }

    // 等待上一次发送完成
    uint32_t wait_ms = 0;
    while (!cdc_tx_done) {
        osDelay(1);
        if (++wait_ms >= 100) {
            return 0;
        }
    }

    cdc_tx_done = false;
    uint8_t ret = CDC_Transmit_FS(buf, (uint16_t) len);
    if (ret != USBD_OK) {
        cdc_tx_done = true;
        return 0;
    }

    // 等待本次发送完成
    wait_ms = 0;
    while (!cdc_tx_done) {
        osDelay(1);
        if (++wait_ms >= 100) {
            cdc_tx_done = true;
            return 0;
        }
    }

    return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport *transport,
                             uint8_t *buf, size_t len,
                             int timeout, uint8_t *err)
{
    (void) transport;

    int ms_used = 0;
    do {
        size_t head, tail;
        __disable_irq();
        head = cdc_head;
        tail = cdc_tail;
        __enable_irq();

        if (head != tail) {
            size_t wrote = 0;
            while (head != tail && wrote < len) {
                buf[wrote] = cdc_buffer[head];
                head       = (head + 1) % USB_CDC_BUFFER_SIZE;
                wrote++;
            }
            __disable_irq();
            cdc_head = head;
            __enable_irq();
            return wrote;
        }

        ms_used++;
        osDelay(1);
    } while (ms_used < timeout);

    return 0;
}

#endif  // RMW_UXRCE_TRANSPORT_CUSTOM