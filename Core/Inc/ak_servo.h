#ifndef AK_SERVO_H
#define AK_SERVO_H

#include "ak_can.h"

/* 伺服模式反馈数据：保存最近一次收到的电机状态。 */
typedef struct
{
    uint8_t motor_id;       /* 电机 CAN ID */
    float position_deg;     /* 位置，单位：度 */
    float speed_erpm;       /* 电角速度，单位：ERPM */
    float current_a;        /* 电流，单位：A */
    int8_t temperature_c;   /* 温度，单位：℃ */
    uint8_t error;          /* 电机故障码 */
    uint32_t last_rx_tick;  /* 最近一次反馈的系统时刻，单位：ms */
} AK_ServoState;

extern AK_ServoState ak_servo_state;

/* 伺服模式控制接口，返回 HAL_OK 表示 CAN 帧成功加入发送队列。 */
HAL_StatusTypeDef AK_Servo_SetDuty(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float duty);
HAL_StatusTypeDef AK_Servo_SetCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a);
HAL_StatusTypeDef AK_Servo_SetBrakeCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a);
HAL_StatusTypeDef AK_Servo_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed_erpm);
HAL_StatusTypeDef AK_Servo_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float position_deg);
HAL_StatusTypeDef AK_Servo_SetOrigin(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t origin_mode);

/* 位置速度复合控制：位置(度)、速度(ERPM)、加速度(ERPM/s)。 */
HAL_StatusTypeDef AK_Servo_SetPositionSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                            float position_deg, float speed_erpm,
                                            float acceleration_erpm_s);

/* 解析一帧伺服模式反馈；成功返回 1，并更新 ak_servo_state。 */
uint8_t AK_Servo_ParseFeedback(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8]);

/* 在 timeout_ms 内收到过反馈则返回 1。 */
uint8_t AK_Servo_IsOnline(uint32_t timeout_ms);

#endif
