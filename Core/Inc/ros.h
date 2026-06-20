#ifndef ROS_H
#define ROS_H

#include "rclc/executor_handle.h"
#include <assert.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#define DOMAIN_ID      10
#define NODE_NAME      "stm"
#define NODE_NAMESPACE ""

#define HANDLER_NUM    2
#define SUBSCRIBER_NUM 0
#define PUBLISHER_NUM  1
#define TIMER_NUM      0
#define SERVICE_NUM    2
#define CLIENT_NUM     0

static_assert((SUBSCRIBER_NUM + TIMER_NUM + SERVICE_NUM + CLIENT_NUM) <= HANDLER_NUM,
              "Executor handler quota exceeded: (subscriber+timer+service+client) > HANDLER_NUM");

typedef struct Node Node;

typedef enum {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} AgentStates;

typedef struct Node {
    rcl_node_t node;
    rclc_executor_t executor;
    rclc_support_t support;

#if PUBLISHER_NUM > 0
    rcl_publisher_t publisher[PUBLISHER_NUM];
#endif
#if SUBSCRIBER_NUM > 0
    rcl_subscription_t subscriber[SUBSCRIBER_NUM];
#endif
#if SERVICE_NUM > 0
    rcl_service_t service[SERVICE_NUM];
#endif
#if CLIENT_NUM > 0
    rcl_client_t client[CLIENT_NUM];
#endif
#if TIMER_NUM > 0
    rcl_timer_t timer[TIMER_NUM];
#endif
    AgentStates state;

    bool (*create)(Node *node);
    bool (*destroy)(Node *node);
    void (*spin)(Node *node);
    bool (*setup)(Node *node);

    bool (*publish)(Node *node, int idx, void *msg);
} Node;

void initNode(Node *node);
bool createNode(Node *node);
bool destroyNode(Node *node);
void spinNode(Node *node);

// ── Publisher ────────────────────────────────────────────────────
#if PUBLISHER_NUM > 0
bool initPublisher(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic);
bool finiPublisher(Node *node, int idx);
bool publishMsg(Node *node, int idx, void *msg);
#endif

// ── Subscriber ───────────────────────────────────────────────────
#if SUBSCRIBER_NUM > 0
bool initSubscriber(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic, void *msg, rclc_subscription_callback_t cb, rclc_executor_handle_invocation_t inv);
bool finiSubscriber(Node *node, int idx);
#endif

// ── Service ──────────────────────────────────────────────────────
#if SERVICE_NUM > 0
bool initService(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc, void *req, void *res, rclc_service_callback_t cb);
bool finiService(Node *node, int idx);
#endif

// ── Client ───────────────────────────────────────────────────────
#if CLIENT_NUM > 0
bool initClient(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc);
bool finiClient(Node *node, int idx);
#endif

// ── Timer ────────────────────────────────────────────────────────
#if TIMER_NUM > 0
bool initTimer(Node *node, int idx, int64_t period_ms, rclc_timer_callback_t cb);
bool finiTimer(Node *node, int idx);
#endif

#endif  // !ROS_H