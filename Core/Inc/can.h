#ifndef CAN_H
#define CAN_H

#include "main.h"
#include <stdbool.h>

#define CAN_BUF_DEPTH 16

typedef struct CAN CAN;

typedef union {
    struct __attribute__((packed)) {
        uint32_t id;     /* 标准 ID (11bit) 或扩展 ID (29bit) */
        uint8_t dlc;     /* 数据长度码 (0~8) */
        uint8_t data[8]; /* 数据场 (最多8字节) */
    } CANFrame;
    uint8_t raw[13];
} CANFrame_t;

typedef struct CAN {
    CAN_HandleTypeDef *hcan;
    CANFrame_t rx_buf[CAN_BUF_DEPTH];
    volatile uint8_t rx_head; /* ISR 写索引 */
    volatile uint8_t rx_tail; /* 用户读索引 */
} CAN;

void CAN_Init(CAN *can, CAN_HandleTypeDef *hcan);
bool CAN_Send(CAN *can, CANFrame_t *frame);
void CAN_Receive(CAN *can);
void CAN_ReceiveISR(CAN *can);
CANFrame_t *CAN_GetRxFrame(CAN *can);

#endif /* CAN_H */