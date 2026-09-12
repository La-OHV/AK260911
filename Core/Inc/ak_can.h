#ifndef AK_CAN_H
#define AK_CAN_H

#include "fdcan.h"

/* 配置标准帧和扩展帧滤波器，然后启动 FDCAN。 */
HAL_StatusTypeDef AK_CAN_Start(FDCAN_HandleTypeDef *hfdcan);

/* 发送一帧经典 CAN 数据，id_type 使用 FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID。 */
HAL_StatusTypeDef AK_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                              uint32_t id_type, const uint8_t *data,
                              uint8_t length);

/* 从 RX FIFO0 读取一帧；读取成功返回 1，无数据或读取失败返回 0。 */
uint8_t AK_CAN_Receive(FDCAN_HandleTypeDef *hfdcan,
                       FDCAN_RxHeaderTypeDef *header, uint8_t data[8]);

#endif
