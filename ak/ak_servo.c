#include "ak_servo.h"

/* 伺服模式命令号，放在扩展 CAN ID 的第 8 位以上。 */
enum
{
    AK_SERVO_SET_DUTY = 0,
    AK_SERVO_SET_CURRENT,
    AK_SERVO_SET_BRAKE_CURRENT,
    AK_SERVO_SET_SPEED,
    AK_SERVO_SET_POSITION,
    AK_SERVO_SET_ORIGIN,
    AK_SERVO_SET_POSITION_SPEED
};

/* 最近一次伺服模式反馈。 */
AK_ServoState ak_servo_state = {0};

/* 协议使用大端字节序：高字节先发送。 */
static void put_i32(uint8_t *data, int32_t value)
{
    data[0] = (uint8_t)((uint32_t)value >> 24);
    data[1] = (uint8_t)((uint32_t)value >> 16);
    data[2] = (uint8_t)((uint32_t)value >> 8);
    data[3] = (uint8_t)value;
}

static void put_i16(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8);
    data[1] = (uint8_t)value;
}

static HAL_StatusTypeDef send(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                              uint8_t packet_id, const uint8_t *data, uint32_t length)
{
    /* 扩展帧 ID = 电机 ID | (命令号 << 8)。 */
    uint32_t id = motor_id | ((uint32_t)packet_id << 8);
    return AK_CAN_Send(hfdcan, id, FDCAN_EXTENDED_ID, data, (uint8_t)length);
}

static HAL_StatusTypeDef send_i32(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                  uint8_t packet_id, int32_t value)
{
    uint8_t data[4];
    put_i32(data, value);
    return send(hfdcan, motor_id, packet_id, data, FDCAN_DLC_BYTES_4);
}

HAL_StatusTypeDef AK_Servo_SetDuty(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float duty)
{
    /* 占空比有效范围为 -1.0 ~ 1.0。 */
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    return send_i32(hfdcan, motor_id, AK_SERVO_SET_DUTY, (int32_t)(duty * 100000.0f));
}

HAL_StatusTypeDef AK_Servo_SetCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a)
{
    return send_i32(hfdcan, motor_id, AK_SERVO_SET_CURRENT, (int32_t)(current_a * 1000.0f));
}

HAL_StatusTypeDef AK_Servo_SetBrakeCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a)
{
    return send_i32(hfdcan, motor_id, AK_SERVO_SET_BRAKE_CURRENT, (int32_t)(current_a * 1000.0f));
}

HAL_StatusTypeDef AK_Servo_SetSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float speed_erpm)
{
    return send_i32(hfdcan, motor_id, AK_SERVO_SET_SPEED, (int32_t)speed_erpm);
}

HAL_StatusTypeDef AK_Servo_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float position_deg)
{
    return send_i32(hfdcan, motor_id, AK_SERVO_SET_POSITION, (int32_t)(position_deg * 10000.0f));
}

HAL_StatusTypeDef AK_Servo_SetOrigin(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t origin_mode)
{
    return send(hfdcan, motor_id, AK_SERVO_SET_ORIGIN, &origin_mode, FDCAN_DLC_BYTES_1);
}

HAL_StatusTypeDef AK_Servo_SetPositionSpeed(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                                            float position_deg, float speed_erpm,
                                            float acceleration_erpm_s)
{
    /* 协议中速度和加速度的每个整数单位代表 10 ERPM。 */
    float speed = speed_erpm * 0.1f;
    float acceleration = acceleration_erpm_s * 0.1f;
    uint8_t data[8];

    if (speed > 32767.0f) speed = 32767.0f;
    if (speed < -32768.0f) speed = -32768.0f;
    if (acceleration > 32767.0f) acceleration = 32767.0f;
    if (acceleration < -32768.0f) acceleration = -32768.0f;

    /* 8 字节：位置 int32 + 速度 int16 + 加速度 int16，均为大端。 */
    put_i32(data, (int32_t)(position_deg * 10000.0f));
    put_i16(&data[4], (int16_t)speed);
    put_i16(&data[6], (int16_t)acceleration);
    return send(hfdcan, motor_id, AK_SERVO_SET_POSITION_SPEED, data, FDCAN_DLC_BYTES_8);
}

uint8_t AK_Servo_ParseFeedback(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    /* 伺服反馈必须是 8 字节扩展数据帧。 */
    if ((header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_EXTENDED_ID) ||
        (header->DataLength < FDCAN_DLC_BYTES_8)) {
        return 0;
    }

    /* 反馈格式：位置、速度、电流各 2 字节，温度和故障码各 1 字节。 */
    ak_servo_state.motor_id = (uint8_t)(header->Identifier & 0xFFU);
    ak_servo_state.position_deg = (float)(int16_t)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
    ak_servo_state.speed_erpm = (float)(int16_t)(((uint16_t)data[2] << 8) | data[3]) * 10.0f;
    ak_servo_state.current_a = (float)(int16_t)(((uint16_t)data[4] << 8) | data[5]) * 0.01f;
    ak_servo_state.temperature_c = (int8_t)data[6];
    ak_servo_state.error = data[7];
    ak_servo_state.last_rx_tick = HAL_GetTick();
    return 1;
}

uint8_t AK_Servo_IsOnline(uint32_t timeout_ms)
{
    return (ak_servo_state.last_rx_tick != 0U) &&
           ((HAL_GetTick() - ak_servo_state.last_rx_tick) <= timeout_ms);
}
