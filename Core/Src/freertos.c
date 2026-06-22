/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmsis_os.h>

#include "ros.h"
#include "lcd.h"
#include "can.h"
#include "gpio.h"

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_srvs/srv/set_bool.h>
#include <rosidl_runtime_c/string_functions.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;
extern CAN_HandleTypeDef hcan1;

extern osMessageQueueId_t canTxHandle;
extern osMessageQueueId_t canRxHandle;
extern osMessageQueueId_t dispNumHandle;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

GPIO pump, led_red, led_blue;
CAN can1;

// ── pump_ctlk service callback ──
void pump_ctl_callback(const void *req, void *res)
{
    std_srvs__srv__SetBool_Response *res_ = (std_srvs__srv__SetBool_Response *) res;
    std_srvs__srv__SetBool_Request *req_  = (std_srvs__srv__SetBool_Request *) req;

    res_->success = true;
    if (req_->data == true) {
        led_red.on(&led_red);
        pump.on(&pump);
        rosidl_runtime_c__String__assign(&res_->message, "on");

    } else {
        led_red.off(&led_red);
        pump.off(&pump);
        rosidl_runtime_c__String__assign(&res_->message, "off");
    }
}

// ── alive callback ──
void alive_callback(const void *msg)
{
    (void) msg;
    HAL_IWDG_Refresh(&hiwdg);
}

// ── num display callback ──
uint8_t disp_num = 0;
void dispNum_callback(const void *msg)
{
    std_msgs__msg__UInt8 *msg_ = (std_msgs__msg__UInt8 *) msg;
    disp_num                   = msg_->data % 4;
    osMessageQueuePut(dispNumHandle, &disp_num, 0, 0);
}

// ── can_tx callback ──
void canTx_callback(const void *msg)
{
    std_msgs__msg__UInt8MultiArray *msg_ = (std_msgs__msg__UInt8MultiArray *) msg;
    if (msg_->data.size != 13) {
        return;
    }
    CANFrame_t frame;
    memcpy(frame.raw, msg_->data.data, 13);
    osMessageQueuePut(canTxHandle, &frame, 0, 0);
}

// ── entity setup ──
static std_msgs__msg__UInt8 disp_msg;
static std_msgs__msg__Empty alive_msg;
static std_msgs__msg__UInt8MultiArray can_tx_msg;
static std_msgs__msg__UInt8MultiArray can_rx_msg;
static uint8_t can_tx_data[13]; /* 预分配 UInt8MultiArray 序列缓冲 */

static std_srvs__srv__SetBool_Request svc_req;
static std_srvs__srv__SetBool_Response svc_res;

bool setupEntities(Node *node)
{
    // 重连时重置 response 字符串，避免复用已释放的内存
    rosidl_runtime_c__String__fini(&svc_res.message);
    rosidl_runtime_c__String__init(&svc_res.message);

    /* 预分配 UInt8MultiArray 序列内存，确保 deserialize 不失败 */
    can_tx_msg.data.data     = can_tx_data;
    can_tx_msg.data.capacity = 13;
    can_tx_msg.data.size     = 0;

    bool ok;
    ok = initSubscriber(node, 0,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                        "STM32/disp_num",
                        &disp_msg, dispNum_callback);


    ok = ok && initSubscriber(node, 1,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
                              "STM32/alive",
                              &alive_msg, alive_callback);

    ok = ok && initSubscriber(node, 2,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                              "STM32/can_tx",
                              &can_tx_msg, canTx_callback);

    ok = ok && initPublisher(node, 0,
                             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
                             "STM32/can_rx");

    ok = ok && initService(node, 0,
                           ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                           "STM32/pump_ctrl",
                           &svc_req, &svc_res,
                           pump_ctl_callback);
    return ok;
}

// ── microROS thread ──────────────────────────────────────────────
void StartMicroROS(void *argument)
{
    GPIOInit(&pump, PUMP_GPIO_Port, PUMP_Pin);
    GPIOInit(&led_red, LED_RED_GPIO_Port, LED_RED_Pin);
    GPIOInit(&led_blue, LED_BLUE_GPIO_Port, LED_BLUE_Pin);

    // custom allocator
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate        = microros_allocate;
    freeRTOS_allocator.deallocate      = microros_deallocate;
    freeRTOS_allocator.reallocate      = microros_reallocate;
    freeRTOS_allocator.zero_allocate   = microros_zero_allocate;
    (void) rcutils_set_default_allocator(&freeRTOS_allocator);

    // custom transport
    rmw_uros_set_custom_transport(
        true, (void *) &huart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    // ── node ─────────────────────────────────────────────────────
    static Node node;
    initNode(&node);
    node.setup = setupEntities;

    // ── main loop ────────────────────────────────────────────────
    for (;;) {
        if (!node.inited) {
            // keep watchdog alive while waiting for agent
            HAL_IWDG_Refresh(&hiwdg);

            // probe agent with quick ping — avoid 10s blocking in createNode
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (node.create(&node) && node.setup(&node)) {
                    node.inited      = true;
                    node.error_count = 0;
                } else {
                    // partial init — clean up
                    if (node.inited) {
                        node.destroy(&node);
                    }
                }
            }
            osDelay(500);
            continue;
        }

        node.spin(&node);

        CANFrame_t rx_frame;
        bool has_frame = false;
        while (osMessageQueueGet(canRxHandle, &rx_frame, NULL, 0) == osOK) {
            has_frame = true;
        }
        if (has_frame) {
            can_rx_msg.data.data     = rx_frame.raw;
            can_rx_msg.data.size     = 13;
            can_rx_msg.data.capacity = 13;
            node.publish(&node, 0, (void *) &can_rx_msg);
            led_blue.toggle(&led_blue);
        }

        osDelay(10);
    }
}

void StartCAN(void *argument)
{
    CANInit(&can1, &hcan1);
    CANFrame_t tx_frame;
    for (;;) {
        /* drain all RX frames from ISR ring buffer */
        CANFrame_t *rx_frame;
        while ((rx_frame = can1.getRxFrame(&can1)) != NULL) {
            osMessageQueuePut(canRxHandle, rx_frame, 0, 0);
        }

        /* non-blocking TX */
        if (osMessageQueueGet(canTxHandle, &tx_frame, NULL, 0) == osOK) {
            can1.send(&can1, &tx_frame);
        }

        osDelay(1); /* yield CPU to microROS thread */
    }
}

/* USER CODE END Application */
