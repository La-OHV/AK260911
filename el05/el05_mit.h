#ifndef EL05_MIT_H
#define EL05_MIT_H

#include "../ak/ak_can.h"

/* EL05 固定使用 CAN ID 0x03。 */
#define EL05_MIT_ID  0x03U

typedef struct
{
    float position_rad;      /* 位置，rad */
    float velocity_rad_s;    /* 速度，rad/s */
    float torque_nm;         /* 扭矩，N·m */
    float temperature_c;     /* 绕组温度，℃ */
    uint8_t mode;            /* 0复位，1标定，2运行 */
    uint8_t fault;           /* 1表示有故障 */
    uint8_t warning;         /* 1表示有预警 */
    uint32_t last_rx_tick;   /* 最近反馈时间，ms */
} EL05_MIT_State;

extern EL05_MIT_State el05_mit_state;

HAL_StatusTypeDef EL05_MIT_Enable(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef EL05_MIT_Disable(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef EL05_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan);

/* MIT控制：目标位置、目标速度、Kp、Kd、前馈扭矩。 */
HAL_StatusTypeDef EL05_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   float position_rad, float velocity_rad_s,
                                   float kp, float kd, float torque_nm);

uint8_t EL05_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header,
                               const uint8_t data[8]);
uint8_t EL05_MIT_IsOnline(uint32_t timeout_ms);

#endif
