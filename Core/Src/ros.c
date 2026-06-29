#include "ros.h"
#include "cmsis_os.h"
#include "rclc/subscription.h"
#include <string.h>

// ── initNode ─────────────────────────────────────────────────────
void Node_Init(Node *node)
{
    memset(node, 0, sizeof(Node));
    node->inited = false;
    node->setup  = NULL;
}

// ── createNode ───────────────────────────────────────────────────
bool Node_Create(Node *node)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        return false;
    }

    if (rcl_init_options_set_domain_id(&init_options, DOMAIN_ID) != RCL_RET_OK) {
        rcl_init_options_fini(&init_options);
        return false;
    }

    rcl_ret_t ret = rclc_support_init_with_options(&node->support, 0, NULL,
                                                   &init_options, &allocator);
    rcl_init_options_fini(&init_options);  // CRITICAL: avoid dangling stack ptr
    if (ret != RCL_RET_OK) {
        // try to clean up partially-initialized support, then zero the struct
        rclc_support_fini(&node->support);
        memset(&node->support, 0, sizeof(node->support));
        return false;
    }

    if (rclc_node_init_default(&node->node, NODE_NAME, NODE_NAMESPACE,
                               &node->support) != RCL_RET_OK) {
        rcl_node_fini(&node->node);
        rclc_support_fini(&node->support);
        return false;
    }

    // shorten entity creation/destroy session timeouts (default 10s → 1s)
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&node->support.context);
    rmw_uros_set_context_entity_creation_session_timeout(rmw_ctx, 1000);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 1000);

    // wait_set replaces executor — sized for all entities (except timer/publisher)
    node->wait_set = rcl_get_zero_initialized_wait_set();
    if (rcl_wait_set_init(&node->wait_set,
                          SUBSCRIBER_NUM, 0, 0,
                          CLIENT_NUM, SERVICE_NUM, 0,
                          &node->support.context, allocator) != RCL_RET_OK) {
        rcl_wait_set_fini(&node->wait_set);
        rcl_node_fini(&node->node);
        rclc_support_fini(&node->support);
        return false;
    }

    node->inited = true;
    return true;
}

// ── destroyNode ──────────────────────────────────────────────────
bool Node_Destroy(Node *node)
{
    rcl_ret_t ret          = RCL_RET_OK;
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&node->support.context);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 500);

    // 1. fini entities
#if SUBSCRIBER_NUM > 0
    for (int i = 0; i < SUBSCRIBER_NUM; i++) {
        ret |= rcl_subscription_fini(&node->subscriber[i], &node->node);
    }
#endif
#if CLIENT_NUM > 0
    for (int i = 0; i < CLIENT_NUM; i++) {
        ret |= rcl_client_fini(&node->client[i], &node->node);
    }
#endif
#if SERVICE_NUM > 0
    for (int i = 0; i < SERVICE_NUM; i++) {
        ret |= rcl_service_fini(&node->service[i], &node->node);
    }
#endif
#if TIMER_NUM > 0
    for (int i = 0; i < TIMER_NUM; i++) {
        ret |= rcl_timer_fini(&node->timer[i]);
    }
#endif
#if PUBLISHER_NUM > 0
    for (int i = 0; i < PUBLISHER_NUM; i++) {
        ret |= rcl_publisher_fini(&node->publisher[i], &node->node);
    }
#endif

    // 2. fini wait_set
    ret |= rcl_wait_set_fini(&node->wait_set);

    // 3. fini node
    ret |= rcl_node_fini(&node->node);

    // 4. fini support (closes session + transport)
    ret |= rclc_support_fini(&node->support);

    node->setup  = NULL;
    node->inited = false;
    return (ret == RCL_RET_OK);
}

// ── spinNode ─────────────────────────────────────────────────────
void Node_Spin(Node *node)
{
    if (!node->inited) {
        return;
    }

#if SUBSCRIBER_NUM > 0 || SERVICE_NUM > 0 || CLIENT_NUM > 0
    rcl_wait_set_clear(&node->wait_set);

    #if SUBSCRIBER_NUM > 0
    for (int i = 0; i < SUBSCRIBER_NUM; i++) {
        rcl_wait_set_add_subscription(&node->wait_set, &node->subscriber[i], NULL);
    }
    #endif
    #if SERVICE_NUM > 0
    for (int i = 0; i < SERVICE_NUM; i++) {
        rcl_wait_set_add_service(&node->wait_set, &node->service[i], NULL);
    }
    #endif
    #if CLIENT_NUM > 0
    for (int i = 0; i < CLIENT_NUM; i++) {
        rcl_wait_set_add_client(&node->wait_set, &node->client[i], NULL);
    }
    #endif

    rcl_ret_t rc = rcl_wait(&node->wait_set, 0);
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        node->error_count++;
        return;
    }

    #if SUBSCRIBER_NUM > 0
    for (int i = 0; i < SUBSCRIBER_NUM; i++) {
        if (node->wait_set.subscriptions[i]) {
            if (rcl_take(&node->subscriber[i], node->sub_msg[i], NULL, NULL) == RCL_RET_OK) {
                if (node->sub_cb[i]) {
                    node->sub_cb[i](node->sub_msg[i]);
                }
            }
        }
    }
    #endif

    #if SERVICE_NUM > 0
    for (int i = 0; i < SERVICE_NUM; i++) {
        if (node->wait_set.services[i]) {
            rmw_request_id_t req_id;
            if (rcl_take_request(&node->service[i], &req_id,
                                 node->svc_req[i]) == RCL_RET_OK) {
                node->svc_cb[i](node->svc_req[i], node->svc_res[i]);
                rcl_send_response(&node->service[i], &req_id,
                                  node->svc_res[i]);
            }
        }
    }
    #endif
#endif

#if TIMER_NUM > 0
    for (int i = 0; i < TIMER_NUM; i++) {
        bool is_ready = false;
        if (rcl_timer_is_ready(&node->timer[i], &is_ready) == RCL_RET_OK && is_ready) {
            rcl_timer_call(&node->timer[i]);
        }
    }
#endif
}

// ── Publisher ────────────────────────────────────────────────────
#if PUBLISHER_NUM > 0
bool Node_InitPublisher(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rclc_publisher_init_default(&node->publisher[idx],
                                       &node->node, type, topic) == RCL_RET_OK;
}

bool Node_FiniPublisher(Node *node, int idx)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rcl_publisher_fini(&node->publisher[idx], &node->node) == RCL_RET_OK;
}

bool Node_Publish(Node *node, int idx, void *msg)
{
    assert(idx >= 0 && idx < PUBLISHER_NUM);
    return rcl_publish(&node->publisher[idx], msg, NULL) == RCL_RET_OK;
}
#endif

// ── Service ──────────────────────────────────────────────────────
#if SERVICE_NUM > 0
bool Node_InitService(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc, void *req, void *res, rclc_service_callback_t cb)
{
    assert(idx >= 0 && idx < SERVICE_NUM);
    node->svc_cb[idx]  = cb;
    node->svc_req[idx] = req;
    node->svc_res[idx] = res;
    return rclc_service_init_default(&node->service[idx],
                                     &node->node, type, svc) == RCL_RET_OK;
}

bool Node_FiniService(Node *node, int idx)
{
    assert(idx >= 0 && idx < SERVICE_NUM);
    return rcl_service_fini(&node->service[idx], &node->node) == RCL_RET_OK;
}
#endif

// ── Subscriber ───────────────────────────────────────────────────
#if SUBSCRIBER_NUM > 0
bool Node_InitSubscriber(Node *node, int idx, const rosidl_message_type_support_t *type, const char *topic, void *msg, rclc_subscription_callback_t cb)
{
    assert(idx >= 0 && idx < SUBSCRIBER_NUM);
    node->sub_cb[idx]  = cb;
    node->sub_msg[idx] = msg;
    return rclc_subscription_init_best_effort(&node->subscriber[idx],
                                              &node->node, type, topic) == RCL_RET_OK;
}

bool Node_FiniSubscriber(Node *node, int idx)
{
    assert(idx >= 0 && idx < SUBSCRIBER_NUM);
    return rcl_subscription_fini(&node->subscriber[idx], &node->node) == RCL_RET_OK;
}
#endif

// ── Client ───────────────────────────────────────────────────────
#if CLIENT_NUM > 0
bool Node_InitClient(Node *node, int idx, const rosidl_service_type_support_t *type, const char *svc)
{
    assert(idx >= 0 && idx < CLIENT_NUM);
    return rclc_client_init_default(&node->client[idx],
                                    &node->node, type, svc) == RCL_RET_OK;
}

bool Node_FiniClient(Node *node, int idx)
{
    assert(idx >= 0 && idx < CLIENT_NUM);
    return rcl_client_fini(&node->client[idx], &node->node) == RCL_RET_OK;
}

bool Node_ClientSendRequest(Node *node, int idx, void *req, void *res, int timeout_ms)
{
    assert(idx >= 0 && idx < CLIENT_NUM && node->inited);

    // non-blocking send
    int64_t seq  = -1;
    rcl_ret_t rc = rcl_send_request(&node->client[idx], req, &seq);
    if (rc != RCL_RET_OK)
        return false;

    // blocking wait for response
    int elapsed       = 0;
    const int step_ms = 10;
    while (elapsed < timeout_ms) {
        rcl_wait_set_clear(&node->wait_set);
        rcl_wait_set_add_client(&node->wait_set, &node->client[idx], NULL);
        rcl_ret_t wait_rc = rcl_wait(&node->wait_set, RCL_MS_TO_NS(step_ms));
        if (wait_rc == RCL_RET_OK) {
            rmw_request_id_t req_id;
            if (rcl_take_response(&node->client[idx], &req_id, res) == RCL_RET_OK) {
                return true;
            }
        }
        elapsed += step_ms;
    }
    return false;  // timeout
}
#endif

// ── Timer ────────────────────────────────────────────────────────
#if TIMER_NUM > 0
bool Node_InitTimer(Node *node, int idx, int64_t period_ms, rcl_timer_callback_t cb)
{
    assert(idx >= 0 && idx < TIMER_NUM);
    node->timer_cb[idx] = cb;
    return rclc_timer_init_default2(&node->timer[idx],
                                    &node->support,
                                    RCL_MS_TO_NS((uint64_t) period_ms),
                                    cb,
                                    true) == RCL_RET_OK;
}

bool Node_FiniTimer(Node *node, int idx)
{
    assert(idx >= 0 && idx < TIMER_NUM);
    return rcl_timer_fini(&node->timer[idx]) == RCL_RET_OK;
}
#endif
