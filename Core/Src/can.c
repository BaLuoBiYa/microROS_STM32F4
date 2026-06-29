#include "can.h"

void CANInit(CAN *can, CAN_HandleTypeDef *hcan)
{
    can->hcan       = hcan;
    can->rx_head    = 0;
    can->rx_tail    = 0;

    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;                      // 过滤器组0
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;  // 掩码模式
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;  // 32位宽
    filter.FilterIdHigh         = 0x0000;                 // 标识符高16位（标准帧：11位ID）
    filter.FilterIdLow          = 0x0000;                 // 标识符低16位
    filter.FilterMaskIdHigh     = 0x0000;                 // 掩码高16位（0=不关心）
    filter.FilterMaskIdLow      = 0x0000;                 // 掩码低16位
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;           // 接收FIFO0
    filter.FilterActivation     = ENABLE;                 // 使能过滤器
    filter.SlaveStartFilterBank = 14;                     // 单CAN控制器，无需从机
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(hcan) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

bool CANSend(CAN *can, CANFrame_t *frame)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) == 0) {
        return false;
    }

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = frame->CANFrame.id & 0x7FF;
    txHeader.ExtId = 0;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.IDE   = CAN_ID_STD;
    txHeader.DLC   = frame->CANFrame.dlc;
    uint32_t tx_mailbox;

    return HAL_CAN_AddTxMessage(can->hcan, &txHeader,
                                frame->CANFrame.data, &tx_mailbox) == HAL_OK;
}

void CANReceive(CAN *can)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
            uint8_t next = (can->rx_head + 1) % CAN_BUF_DEPTH;
            if (next != can->rx_tail) {
                if (rxHeader.IDE == CAN_ID_STD) {
                    can->rx_buf[can->rx_head].CANFrame.id = rxHeader.StdId;
                } else {
                    can->rx_buf[can->rx_head].CANFrame.id = rxHeader.ExtId;
                }

                can->rx_buf[can->rx_head].CANFrame.dlc = rxHeader.DLC;
                for (uint8_t i = 0; i < rxHeader.DLC && i < 8; i++) {
                    can->rx_buf[can->rx_head].CANFrame.data[i] = rxData[i];
                }
                can->rx_head = next;
            }
        }
    }
}

void CANReceiveISR(CAN *can)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    while (HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
        uint8_t next = (can->rx_head + 1) % CAN_BUF_DEPTH;
        if (next == can->rx_tail) {
            /* 环缓冲满，丢弃最旧帧 */
            can->rx_tail = (can->rx_tail + 1) % CAN_BUF_DEPTH;
        }
        if (rxHeader.IDE == CAN_ID_STD) {
            can->rx_buf[can->rx_head].CANFrame.id = rxHeader.StdId;
        } else {
            can->rx_buf[can->rx_head].CANFrame.id = rxHeader.ExtId;
        }
        can->rx_buf[can->rx_head].CANFrame.dlc = rxHeader.DLC;
        for (uint8_t i = 0; i < rxHeader.DLC && i < 8; i++) {
            can->rx_buf[can->rx_head].CANFrame.data[i] = rxData[i];
        }
        can->rx_head = next;
    }
}

CANFrame_t *CANGetRxFrame(CAN *can)
{
    if (can->rx_tail == can->rx_head) {
        return NULL;
    }

    CANFrame_t *frame = &can->rx_buf[can->rx_tail];
    can->rx_tail      = (can->rx_tail + 1) % CAN_BUF_DEPTH;
    return frame;
}