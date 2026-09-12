#ifndef EL05_MIT_H
#define EL05_MIT_H

#include "../ak/ak_can.h"

/* EL05 使用扩展帧；电机 ID 和参考工程保持一致。 */
#define EL05_MIT_ID          0x03U
#define EL05_MASTER_CAN_ID   0x11U

/* RobStride 通信类型。 */
#define EL05_COMM_MOTION     0x01U  /* 运动控制 */
#define EL05_COMM_FEEDBACK   0x02U  /* 电机反馈 */
#define EL05_COMM_ENABLE     0x03U  /* 使能 */
#define EL05_COMM_STOP       0x04U  /* 停止 */
#define EL05_COMM_SET_ZERO   0x06U  /* 设置当前位置为零点 */

typedef struct
{
    float position_rad;      /* 位置，rad */
    float velocity_rad_s;    /* 速度，rad/s */
    float torque_nm;         /* 扭矩，N*m */
    float temperature_c;     /* 温度，摄氏度 */
    uint8_t mode;            /* 反馈中的运行模式 */
    uint8_t fault;           /* 1 表示有错误 */
    uint8_t warning;         /* 当前协议未单独提供预警位 */
    uint8_t error_code;      /* 反馈中的错误码 */
    uint32_t last_rx_tick;   /* 最近反馈时间，ms */
} EL05_MIT_State;

extern EL05_MIT_State el05_mit_state;

HAL_StatusTypeDef EL05_MIT_Enable(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef EL05_MIT_Disable(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef EL05_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan);

/* 运动控制：目标位置、速度、Kp、Kd、前馈扭矩。 */
HAL_StatusTypeDef EL05_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   float position_rad, float velocity_rad_s,
                                   float kp, float kd, float torque_nm);

/* 解析 EL05 扩展反馈帧；成功返回 1。 */
uint8_t EL05_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header,
                               const uint8_t data[8]);
uint8_t EL05_MIT_IsOnline(uint32_t timeout_ms);

#endif
