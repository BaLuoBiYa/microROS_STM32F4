#include "ros.h"

#include "cmsis_os.h"

// ── initNode ─────────────────────────────────────────────────────
void initNode(Node *node)
{
    node->state   = WAITING_AGENT;
    node->create  = createNode;
    node->destroy = destroyNode;
    node->spin    = spinNode;

    node->publish = publishMsg;
    node->setup   = NULL;
}

// ── createNode ───────────────────────────────────────────────────
bool createNode(Node *node)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        return false;
    }

    if (rcl_init_options_set_domain_id(&init_options, DOMAIN_ID) != RCL_RET_OK) {
        return false;
    }

    if (rclc_support_init_with_options(&node->support, 0, NULL,
                                       &init_options, &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(&node->node, NODE_NAME, NODE_NAMESPACE,
                               &node->support) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_init(&node->executor,
                           &node->support.context,
                           HANDLER_NUM,
                           &allocator) != RCL_RET_OK) {
        return false;
    }

    return true;
}

// ── destroyNode ──────────────────────────────────────────────────
bool destroyNode(Node *node)
{
    rcl_ret_t ret = RCL_RET_OK;

    ret |= rclc_executor_fini(&node->executor);
    ret |= rcl_node_fini(&node->node);

    return (ret == RCL_RET_OK);
}

// ── spinNode ─────────────────────────────────────────────────────
void spinNode(Node *node)
{
    switch (node->state) {
        case WAITING_AGENT:
            osDelay(500);
            node->state =
                (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
            break;
        case AGENT_AVAILABLE:
            osDelay(500);
            node->state = (true == node->create(node)) ? AGENT_CONNECTED : WAITING_AGENT;
            if (node->state == AGENT_CONNECTED && node->setup != NULL) {
                node->setup(node);
            }
            if (node->state == WAITING_AGENT) {
                node->destroy(node);
            };
            break;
        case AGENT_CONNECTED:
            osDelay(200);
            node->state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (node->state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&node->executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            node->destroy(node);
            node->state = WAITING_AGENT;
            break;
        default:
            break;
    }
}

// ── Publisher ────────────────────────────────────────────────────
#if PUBLISHER_NUM > 0
bool initPublisher(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rclc_publisher_init_default(&node->publisher[idx],
                                       &node->node,
                                       type,
                                       topic) == RCL_RET_OK;
}

bool finiPublisher(Node *node, int idx)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rcl_publisher_fini(&node->publisher[idx], &node->node) == RCL_RET_OK;
}

bool publishMsg(Node *node, int idx, void *msg)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rcl_publish(&node->publisher[idx], msg, NULL) == RCL_RET_OK;
}
#endif

// ── Subscriber ───────────────────────────────────────────────────
#if SUBSCRIBER_NUM > 0
bool initSubscriber(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic, void *msg, rclc_subscription_callback_t cb, rclc_executor_handle_invocation_t inv)
{
    assert(idx >= 0 && idx < SUBSCRIBER_NUM);

    if (rclc_subscription_init_default(&node->subscriber[idx],
                                       &node->node,
                                       type,
                                       topic) != RCL_RET_OK) {
        return false;
    }

    return rclc_executor_add_subscription(&node->executor,
                                          &node->subscriber[idx],
                                          msg,
                                          cb,
                                          inv) == RCL_RET_OK;
}

bool finiSubscriber(Node *node, int idx)
{
    assert(idx >= 0 && idx < SUBSCRIBER_NUM);
    return rcl_subscription_fini(&node->subscriber[idx], &node->node) == RCL_RET_OK;
}
#endif

// ── Service ──────────────────────────────────────────────────────
#if SERVICE_NUM > 0
bool initService(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc, void *req, void *res, rclc_service_callback_t cb)
{
    assert(idx >= 0 && idx < SERVICE_NUM);

    if (rclc_service_init_default(&node->service[idx],
                                  &node->node,
                                  type,
                                  svc) != RCL_RET_OK) {
        return false;
    }

    return rclc_executor_add_service(&node->executor,
                                     &node->service[idx],
                                     req,
                                     res,
                                     cb) == RCL_RET_OK;
}

bool finiService(Node *node, int idx)
{
    assert(idx >= 0 && idx < SERVICE_NUM);
    return rcl_service_fini(&node->service[idx], &node->node) == RCL_RET_OK;
}
#endif

// ── Client ───────────────────────────────────────────────────────
#if CLIENT_NUM > 0
bool initClient(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc)
{
    assert(idx >= 0 && idx < CLIENT_NUM);
    return rclc_client_init_default(&node->client[idx],
                                    &node->node,
                                    type,
                                    svc) == RCL_RET_OK;
}

bool finiClient(Node *node, int idx)
{
    assert(idx >= 0 && idx < CLIENT_NUM);
    return rcl_client_fini(&node->client[idx], &node->node) == RCL_RET_OK;
}
#endif

// ── Timer ────────────────────────────────────────────────────────
#if TIMER_NUM > 0
bool initTimer(Node *node, int idx, int64_t period_ms, rclc_timer_callback_t cb)
{
    assert(idx >= 0 && idx < TIMER_NUM);

    if (rclc_timer_init_default(&node->timer[idx],
                                &node->support,
                                RCL_MS_TO_NS((uint64_t) period_ms),
                                cb) != RCL_RET_OK) {
        return false;
    }

    return rclc_executor_add_timer(&node->executor,
                                   &node->timer[idx]) == RCL_RET_OK;
}

bool finiTimer(Node *node, int idx)
{
    assert(idx >= 0 && idx < TIMER_NUM);
    return rcl_timer_fini(&node->timer[idx]) == RCL_RET_OK;
}
#endif
