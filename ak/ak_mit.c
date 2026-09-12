#include "ak_mit.h"

/* 最近一次 MIT 模式反馈。 */
AK_MIT_State ak_mit_state[2] = {0};

/* 将浮点物理量限制在有效范围内，再量化为无符号整数。 */
static uint32_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    if (value < min) value = min;
    if (value > max) value = max;
    return (uint32_t)((value - min) * (float)((1UL << bits) - 1UL) / (max - min));
}

/* 将协议中的无符号整数还原为浮点物理量。 */
static float uint_to_float(uint32_t value, float min, float max, uint8_t bits)
{
    return (float)value * (max - min) / (float)((1UL << bits) - 1UL) + min;
}

static HAL_StatusTypeDef send(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint8_t data[8])
{
    /* MIT 模式使用 11 位标准 CAN ID。 */
    return AK_CAN_Send(hfdcan, motor_id, FDCAN_STANDARD_ID, data, 8);
}

static HAL_StatusTypeDef send_mode(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint8_t command)
{
    /* 模式指令为 7 个 0xFF 加 1 个命令字节。 */
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
    return send(hfdcan, motor_id, data);
}

HAL_StatusTypeDef AK_MIT_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    return send_mode(hfdcan, motor_id, 0xFC);
}

HAL_StatusTypeDef AK_MIT_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    return send_mode(hfdcan, motor_id, 0xFD);
}

HAL_StatusTypeDef AK_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    return send_mode(hfdcan, motor_id, 0xFE);
}

HAL_StatusTypeDef AK_MIT_Control(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id,
                                 float position_rad, float velocity_rad_s,
                                 float kp, float kd, float torque_nm)
{
    /* 位置为 16 位，其余控制量为 12 位。超出范围时自动限幅。 */
    uint32_t p = float_to_uint(position_rad, AK_MIT_P_MIN, AK_MIT_P_MAX, 16);
    uint32_t v = float_to_uint(velocity_rad_s, AK_MIT_V_MIN, AK_MIT_V_MAX, 12);
    uint32_t kp_u = float_to_uint(kp, AK_MIT_KP_MIN, AK_MIT_KP_MAX, 12);
    uint32_t kd_u = float_to_uint(kd, AK_MIT_KD_MIN, AK_MIT_KD_MAX, 12);
    uint32_t t = float_to_uint(torque_nm, AK_MIT_T_MIN, AK_MIT_T_MAX, 12);
    uint8_t data[8];

    /* MIT 固定 8 字节打包格式：P16、V12、Kp12、Kd12、T12。 */
    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (kp_u >> 8));
    data[4] = (uint8_t)kp_u;
    data[5] = (uint8_t)(kd_u >> 4);
    data[6] = (uint8_t)((kd_u << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return send(hfdcan, motor_id, data);
}

uint8_t AK_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    uint32_t p, v, t;
    uint8_t motor_id, index;
    /* MIT 反馈是标准数据帧，前 6 字节包含 ID、位置、速度和扭矩。 */
    if ((header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_STANDARD_ID) ||
        (header->DataLength < FDCAN_DLC_BYTES_6)) {
        return 0;
    }

    /* 只保存两个电机的反馈：CAN ID 1、2 分别放入下标 0、1。 */
    motor_id = data[0];
    if ((motor_id < 1U) || (motor_id > 2U)) {
        return 0;
    }
    index = motor_id - 1U;

    /* 解包 P16、V12 和 T12，再还原为物理量。 */
    p = ((uint32_t)data[1] << 8) | data[2];
    v = ((uint32_t)data[3] << 4) | (data[4] >> 4);
    t = ((uint32_t)(data[4] & 0x0FU) << 8) | data[5];
    ak_mit_state[index].motor_id = motor_id;
    ak_mit_state[index].position_rad = uint_to_float(p, AK_MIT_P_MIN, AK_MIT_P_MAX, 16);
    ak_mit_state[index].velocity_rad_s = uint_to_float(v, AK_MIT_V_MIN, AK_MIT_V_MAX, 12);
    ak_mit_state[index].torque_nm = uint_to_float(t, AK_MIT_T_MIN, AK_MIT_T_MAX, 12);
    if (header->DataLength >= FDCAN_DLC_BYTES_8) {
        /* 新版 8 字节反馈还包含温度和故障码。 */
        ak_mit_state[index].temperature_c = (int16_t)data[6] - 40;
        ak_mit_state[index].error = data[7];
    }
    ak_mit_state[index].last_rx_tick = HAL_GetTick();
    return 1;
}

uint8_t AK_MIT_IsOnline(uint8_t motor_id, uint32_t timeout_ms)
{
    uint8_t index;

    if ((motor_id < 1U) || (motor_id > 2U)) {
        return 0;
    }
    index = motor_id - 1U;
    return (ak_mit_state[index].last_rx_tick != 0U) &&
           ((HAL_GetTick() - ak_mit_state[index].last_rx_tick) <= timeout_ms);
}
