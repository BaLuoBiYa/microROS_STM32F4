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

#define SUBSCRIBER_NUM 1
#define PUBLISHER_NUM  1
#define SERVICE_NUM    2
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
    bool (*publish)(Node *node, int idx, void *msg);
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
    rclc_timer_callback_t timer_cb[TIMER_NUM];
#endif
    bool (*create)(Node *node);
    bool (*destroy)(Node *node);
    void (*spin)(Node *node);
    bool (*setup)(Node *node);
} Node;

void initNode(Node *node);
bool createNode(Node *node);
bool destroyNode(Node *node);
void spinNode(Node *node);

#if PUBLISHER_NUM > 0
bool initPublisher(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic);
bool finiPublisher(Node *node, int idx);
bool publishMsg(Node *node, int idx, void *msg);
#endif

#if SUBSCRIBER_NUM > 0
bool initSubscriber(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic, void *msg, rclc_subscription_callback_t cb);
bool finiSubscriber(Node *node, int idx);
#endif

#if SERVICE_NUM > 0
bool initService(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc, void *req, void *res, rclc_service_callback_t cb);
bool finiService(Node *node, int idx);
#endif

#if CLIENT_NUM > 0
bool initClient(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc);
bool finiClient(Node *node, int idx);
#endif

#if TIMER_NUM > 0
bool initTimer(Node *node, int idx, int64_t period_ms, rclc_timer_callback_t cb);
bool finiTimer(Node *node, int idx);
#endif

#endif  // !ROS_H