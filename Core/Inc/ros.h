#ifndef ROS_H
#define ROS_H

#include <assert.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/wait.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#define DOMAIN_ID      10
#define NODE_NAME      "stm"
#define NODE_NAMESPACE ""

#define SUBSCRIBER_NUM 4
#define PUBLISHER_NUM  1
#define SERVICE_NUM    0
#define CLIENT_NUM     0
#define TIMER_NUM      0

typedef struct Node Node;

typedef struct Node {
    rcl_node_t node;
    rclc_support_t support;
    rcl_wait_set_t wait_set;
    uint8_t error_count;
    bool inited;

#if PUBLISHER_NUM > 0
    rcl_publisher_t publisher[PUBLISHER_NUM];
#endif
#if SUBSCRIBER_NUM > 0
    rcl_subscription_t subscriber[SUBSCRIBER_NUM];
    void *sub_msg[SUBSCRIBER_NUM];
    rclc_subscription_callback_t sub_cb[SUBSCRIBER_NUM];
#endif
#if SERVICE_NUM > 0
    rcl_service_t service[SERVICE_NUM];
    void *svc_req[SERVICE_NUM];
    void *svc_res[SERVICE_NUM];
    rclc_service_callback_t svc_cb[SERVICE_NUM];
#endif
#if CLIENT_NUM > 0
    rcl_client_t client[CLIENT_NUM];
#endif
#if TIMER_NUM > 0
    rcl_timer_t timer[TIMER_NUM];
    rcl_timer_callback_t timer_cb[TIMER_NUM];
#endif
    bool (*setup)(Node *node);
} Node;

void Node_Init(Node *node);
bool Node_Create(Node *node);
bool Node_Destroy(Node *node);
void Node_Spin(Node *node);

#if PUBLISHER_NUM > 0
bool Node_InitPublisher(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic);
bool Node_FiniPublisher(Node *node, int idx);
bool Node_Publish(Node *node, int idx, void *msg);
#endif

#if SUBSCRIBER_NUM > 0
bool Node_InitSubscriber(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic, void *msg, rclc_subscription_callback_t cb);
bool Node_FiniSubscriber(Node *node, int idx);
#endif

#if SERVICE_NUM > 0
bool Node_InitService(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc, void *req, void *res, rclc_service_callback_t cb);
bool Node_FiniService(Node *node, int idx);
#endif

#if CLIENT_NUM > 0
bool Node_InitClient(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc);
bool Node_FiniClient(Node *node, int idx);
bool Node_ClientSendRequest(Node *node, int idx, void *req, void *res, int timeout_ms);
#endif

#if TIMER_NUM > 0
bool Node_InitTimer(Node *node, int idx, int64_t period_ms, rcl_timer_callback_t cb);
bool Node_FiniTimer(Node *node, int idx);
#endif

#endif  // !ROS_H