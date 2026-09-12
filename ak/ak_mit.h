#ifndef AK_MIT_H
#define AK_MIT_H

#include "ak_can.h"

/* AK80-9 MIT 参数范围，取自参考代码；更换型号时需按电机手册修改。 */
#define AK_MIT_P_MIN   (-12.5f)
#define AK_MIT_P_MAX   ( 12.5f)
#define AK_MIT_V_MIN   (-65.0f)
#define AK_MIT_V_MAX   ( 65.0f)
#define AK_MIT_T_MIN   (-18.0f)
#define AK_MIT_T_MAX   ( 18.0f)
#define AK_MIT_KP_MIN  (  0.0f)
#define AK_MIT_KP_MAX  (500.0f)
#define AK_MIT_KD_MIN  (  0.0f)
#define AK_MIT_KD_MAX  (  5.0f)

typedef struct
{
    uint8_t motor_id;        /* 电机 CAN ID */
    float position_rad;      /* 位置，单位：rad */
    float velocity_rad_s;    /* 速度，单位：rad/s */
    float torque_nm;         /* 扭矩，单位：N·m */
    int16_t temperature_c;   /* 温度，单位：℃ */
    uint8_t error;           /* 电机故障码 */
    uint32_t last_rx_tick;   /* 最近一次反馈的系统时刻，单位：ms */
} AK_MIT_State;

/* 双电机反馈：ak_mit_state[0] 对应 CAN ID 1，ak_mit_state[1] 对应 CAN ID 2。 */
extern AK_MIT_State ak_mit_state[2];

/* MIT 模式管理指令：使能、失能和将当前位置设为零点。 */
HAL_StatusTypeDef AK_MIT_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
HAL_StatusTypeDef AK_MIT_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
HAL_StatusTypeDef AK_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);

/* MIT 运控指令：目标位置、目标速度、Kp、Kd、前馈扭矩。 */
HAL_StatusTypeDef AK_MIT_Control(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id,
                                 float position_rad, float velocity_rad_s,
                                 float kp, float kd, float torque_nm);

/* 解析一帧 MIT 反馈；成功返回 1，并更新 ak_mit_state。 */
uint8_t AK_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8]);

/* 指定电机在 timeout_ms 内收到过反馈则返回 1。 */
uint8_t AK_MIT_IsOnline(uint8_t motor_id, uint32_t timeout_ms);

#endif
