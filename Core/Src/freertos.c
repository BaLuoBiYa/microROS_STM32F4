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

#include <std_msgs/msg/int32.h>
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

// ── SetBool service callback ─────────────────────────────────────
void setBool_callback(const void *req, void *res)
{
    std_srvs__srv__SetBool_Response *res_ = (std_srvs__srv__SetBool_Response *) res;
    std_srvs__srv__SetBool_Request *req_  = (std_srvs__srv__SetBool_Request *) req;

    res_->success = true;
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, req_->data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, req_->data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    bool pump_state = HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin) ? true : false;

    rosidl_runtime_c__String__assign(&res_->message, pump_state ? "on" : "off");
}

// ── entity setup ─────────────────────────────────────────────────
static std_msgs__msg__Int32 msg;
static std_srvs__srv__SetBool_Request svc_req;
static std_srvs__srv__SetBool_Response svc_res;

bool setupEntities(Node *node)
{
    // 重连时重置 response 字符串，避免复用已释放的内存
    rosidl_runtime_c__String__fini(&svc_res.message);
    rosidl_runtime_c__String__init(&svc_res.message);

    bool ok;
    ok = initPublisher(node, 0,
                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                       "STM32_HeartBeat");

    ok = ok && initService(node, 0,
                           ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                           "pump_ctrl",
                           &svc_req, &svc_res,
                           setBool_callback);
    return ok;
}

// ── microROS thread ──────────────────────────────────────────────
void StartMicroROS(void *argument)
{
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

    msg.data = 0;

    // ── main loop ────────────────────────────────────────────────
    for (;;) {
        if (!node.inited) {
            // probe agent with quick ping — avoid 10s blocking in createNode
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (node.create(&node) && node.setup(&node)) {
                    node.inited      = true;
                    node.error_count = 0;
                    msg.data         = 0;
                } else {
                    // partial init — clean up
                    if (node.inited)
                        node.destroy(&node);
                }
            }
            osDelay(500);
            continue;
        }

        node.spin(&node);

        if (node.publish(&node, 0, &msg)) {
            msg.data++;
            node.error_count = 0;
        } else {
            node.error_count++;
            if (node.error_count > 50) {  // ~500ms continuous failure
                node.destroy(&node);
                // loop will retry on next iteration
            }
        }

        osDelay(10);
    }
}
/* USER CODE END Application */
