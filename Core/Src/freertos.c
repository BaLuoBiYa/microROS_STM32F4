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
#include "arm.h"

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_srvs/srv/set_bool.h>

#include <rosidl_runtime_c/string_functions.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 机械臂目标命令: micro-ROS subscriber → CAN 控制线程 */
typedef struct {
    float height_mm;
    float base_angle_rad;
} ArmTargetCmd;

/* 机械臂当前状态: CAN 控制线程 → micro-ROS publisher */
typedef struct {
    float base_angle_rad;
    float height_mm;
} ArmStateMsg;

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
extern osMessageQueueId_t armTargetHandle;
extern osMessageQueueId_t armStateHandle;

// ── entity setup ──
static std_msgs__msg__UInt8 disp_msg;
static std_msgs__msg__Empty alive_msg;
static std_msgs__msg__Float32MultiArray target_msg;
static float target_data_buf[2];

static std_srvs__srv__SetBool_Request svc_req;
static std_srvs__srv__SetBool_Response svc_res;

static Node node __attribute__((section(".ccmram")));
static Arm_Control arm_control __attribute__((section(".ccmram")));
static GPIO pump, led_red, led_blue __attribute__((section(".ccmram")));

extern CAN can1;

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

// ── pump_ctlk service callback ──
void pump_ctl_callback(const void *req, void *res)
{
    std_srvs__srv__SetBool_Response *res_ = (std_srvs__srv__SetBool_Response *) res;
    std_srvs__srv__SetBool_Request *req_  = (std_srvs__srv__SetBool_Request *) req;

    res_->success = true;
    if (req_->data == true) {
        GPIO_Off(&led_red);
        GPIO_Off(&pump);
        rosidl_runtime_c__String__assign(&res_->message, "on");

    } else {
        GPIO_On(&led_red);
        GPIO_On(&pump);
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

// ── arm_target callback (subscriber) ──
void arm_target_callback(const void *msg)
{
    const std_msgs__msg__Float32MultiArray *m =
        (const std_msgs__msg__Float32MultiArray *) msg;
    if (m->data.size >= 2) {
        ArmTargetCmd cmd = {
            .height_mm      = m->data.data[0],
            .base_angle_rad = m->data.data[1],
        };
        osMessageQueuePut(armTargetHandle, &cmd, 0, 0);
    }
}

// ── arm_state publisher (FreeRTOS timer callback, 100ms) ──
void StartPubArmState(void *argument)
{
    (void) argument;

    ArmStateMsg s;
    bool has_data = false;
    while (osMessageQueueGet(armStateHandle, &s, NULL, 0) == osOK) {
        has_data = true;
    }
    if (!has_data) {
        return;
    }

    static float state_buf[2];
    static std_msgs__msg__Float32MultiArray state_msg;
    state_msg.data.capacity = 2;
    state_msg.data.size     = 2;
    state_msg.data.data     = state_buf;
    state_buf[0]            = s.base_angle_rad;
    state_buf[1]            = s.height_mm;

    if (node.inited) {
        Node_Publish(&node, 0, &state_msg);
    }
}

bool setupEntities(Node *n)
{
    (void) n;
    // 重连时重置 response 字符串，避免复用已释放的内存
    rosidl_runtime_c__String__fini(&svc_res.message);
    rosidl_runtime_c__String__init(&svc_res.message);

    /* 预分配 arm_target subscriber 序列内存 */
    target_msg.data.data     = target_data_buf;
    target_msg.data.capacity = 2;
    target_msg.data.size     = 0;

    bool ok;
    ok = Node_InitSubscriber(&node, 0,
                             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                             "STM32/disp_num",
                             &disp_msg, dispNum_callback);

    ok = ok && Node_InitSubscriber(&node, 1,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
                                   "STM32/alive",
                                   &alive_msg, alive_callback);

    ok = ok && Node_InitSubscriber(&node, 2,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                   "/arm_target",
                                   &target_msg, arm_target_callback);

    ok = ok && Node_InitPublisher(&node, 0,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                  "/arm_state");

    ok = ok && Node_InitService(&node, 0,
                                ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                "STM32/pump_ctrl",
                                &svc_req, &svc_res,
                                pump_ctl_callback);
    return ok;
}

// ── microROS thread ──────────────────────────────────────────────
void StartMicroROS(void *argument)
{
    GPIO_Init(&pump, PUMP_GPIO_Port, PUMP_Pin);
    GPIO_Init(&led_red, LED_RED_GPIO_Port, LED_RED_Pin);
    GPIO_Init(&led_blue, LED_BLUE_GPIO_Port, LED_BLUE_Pin);

    GPIO_On(&pump);
    GPIO_On(&led_red);

    // custom allocator
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate        = microros_allocate;
    freeRTOS_allocator.deallocate      = microros_deallocate;
    freeRTOS_allocator.reallocate      = microros_reallocate;
    freeRTOS_allocator.zero_allocate   = microros_zero_allocate;
    (void) rcutils_set_default_allocator(&freeRTOS_allocator);

    // custom transport (USB CDC)
    rmw_uros_set_custom_transport(
        true, NULL,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    // ── node ─────────────────────────────────────────────────────
    Node_Init(&node);
    node.setup = setupEntities;

    // ── main loop ────────────────────────────────────────────────
    for (;;) {
        if (!node.inited) {
            HAL_IWDG_Refresh(&hiwdg);

            if (rmw_uros_ping_agent(0, 1) == RMW_RET_OK) {
                if (Node_Create(&node) && node.setup(&node)) {
                    node.inited      = true;
                    node.error_count = 0;
                } else {
                    if (node.inited) {
                        Node_Destroy(&node);
                    }
                }
            }
            osDelay(500);
            continue;
        }

        Node_Spin(&node);
        osDelay(1);
    }
}

void StartCAN(void *argument)
{
    (void) argument;
    CAN_Init(&can1, &hcan1);

    for (;;) {
        /* 1. ISR 环缓冲 → canRxHandle 队列 */
        CANFrame_t *rx_frame;
        while ((rx_frame = CAN_GetRxFrame(&can1)) != NULL) {
            osMessageQueuePut(canRxHandle, rx_frame, 0, 0);
        }

        /* 2. canTxHandle 队列 → CAN 总线发送 (清空所有待发帧) */
        CANFrame_t tx_frame;
        while (osMessageQueueGet(canTxHandle, &tx_frame, NULL, 0) == osOK) {
            if (CAN_Send(&can1, &tx_frame)) {
                GPIO_Toggle(&led_blue);
            }
        }

        osDelay(1);
    }
}

void StartArm(void *argument)
{
    (void) argument;

    /* 初始化 ARM 控制实例, 绑定 CAN 收发队列 */
    Arm_Init(&arm_control, NULL);
    arm_control.canTxQueue = canTxHandle;
    arm_control.canRxQueue = canRxHandle;

    /* 归零: DM 回零 + C620 升降堵转归零 (阻塞, 完成后处于零位) */
    Arm_Homing(&arm_control);

    const TickType_t period_ticks =
        (TickType_t) (arm_control.cfg.loop_period_s * 1000.0f);
    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        /* 1. 从 canRxHandle 取帧喂入 arm 解码 */
        CANFrame_t rx_frame;
        while (osMessageQueueGet(canRxHandle, &rx_frame, NULL, 0) == osOK) {
            Arm_FeedRxFrame(&arm_control, &rx_frame);
        }

        /* 2. [TEST] C620 升降测试: 0 → 150mm → 0, 每 5s 切换 (1s 后首次) */
        {
            static uint32_t tick = 0;
            tick++;
            if (tick >= 5000) {
                static bool to_150 = false; /* 首次翻转为 true → 150mm */
                to_150             = !to_150;
                Arm_SetTarget(&arm_control, 0.0f, to_150 ? 150.0f : 0.0f);
                tick = 0;
            }
        }

        /* 2b. 消费 arm_target 命令 (测试期间可屏蔽) */
        // ArmTargetCmd cmd;
        // while (osMessageQueueGet(armTargetHandle, &cmd, NULL, 0) == osOK) {
        //     Arm_SetTarget(&arm_control, cmd.base_angle_rad, cmd.height_mm);
        // }

        /* 3. 单步控制: 解码 → PID → 组帧 → 推入 canTxHandle */
        Arm_Update(&arm_control);

        /* 4. [DEBUG] 每 200ms 输出斜率/高度/target */
        {
            static uint32_t dbg_tick = 0;
            dbg_tick++;
            if (dbg_tick >= 200) {
                dbg_tick = 0;
                char buf[80];
                int sv  = (int) (arm_control.height_slope.out * 10.0f + 0.5f);
                int hv  = (int) (Arm_GetHeightMm(&arm_control) * 10.0f + 0.5f);
                int tv  = (int) (arm_control.height_slope.target * 10.0f + 0.5f);
                int av  = (int) (arm_control.c620.target_angle * 10.0f + 0.5f);
                int len = snprintf(buf, sizeof(buf),
                                   "S_%04d H_%04d T_%04d A_%04d\r\n", sv, hv, tv, av);
                if (len > 0 && len < (int) sizeof(buf)) {
                    HAL_UART_Transmit(&huart1, (uint8_t *) buf, (uint16_t) len, 10);
                }
            }
        }

        /* 5. 生产状态消息供 micro-ROS 发布 */
        ArmStateMsg state = {
            .base_angle_rad = Arm_GetBaseAngle(&arm_control),
            .height_mm      = Arm_GetHeightMm(&arm_control),
        };
        osMessageQueuePut(armStateHandle, &state, 0, 0);
        last_wake += (period_ticks > 0 ? period_ticks : 1);
        osDelayUntil(last_wake);
    }
}

/* USER CODE END Application */
