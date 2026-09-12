#include "el05_mit.h"

/* EL05 运动控制模式的参数范围。 */
#define P_MIN   (-12.57f)
#define P_MAX   ( 12.57f)
#define V_MIN   (-50.0f)
#define V_MAX   ( 50.0f)
#define KP_MIN  (  0.0f)
#define KP_MAX  (500.0f)
#define KD_MIN  (  0.0f)
#define KD_MAX  (  5.0f)
#define T_MIN   ( -5.5f)
#define T_MAX   (  5.5f)

EL05_MIT_State el05_mit_state = {0};

static uint16_t float_to_u16(float value, float min, float max)
{
    if (value < min) value = min;
    if (value > max) value = max;
    return (uint16_t)((value - min) * 65535.0f / (max - min));
}

static float u16_to_float(uint16_t value, float min, float max)
{
    return (float)value * (max - min) / 65535.0f + min;
}

/* 普通管理帧：通信类型 + 主机 ID + 电机 ID。 */
static uint32_t make_command_id(uint8_t communication_type)
{
    return ((uint32_t)communication_type << 24) |
           ((uint32_t)EL05_MASTER_CAN_ID << 8) |
           EL05_MIT_ID;
}

static HAL_StatusTypeDef send_command(FDCAN_HandleTypeDef *hfdcan,
                                       uint8_t communication_type,
                                       const uint8_t data[8])
{
    return AK_CAN_Send(hfdcan, make_command_id(communication_type),
                       FDCAN_EXTENDED_ID, data, 8);
}

HAL_StatusTypeDef EL05_MIT_Enable(FDCAN_HandleTypeDef *hfdcan)
{
    const uint8_t data[8] = {0};
    return send_command(hfdcan, EL05_COMM_ENABLE, data);
}

HAL_StatusTypeDef EL05_MIT_Disable(FDCAN_HandleTypeDef *hfdcan)
{
    const uint8_t data[8] = {0};
    return send_command(hfdcan, EL05_COMM_STOP, data);
}

HAL_StatusTypeDef EL05_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan)
{
    const uint8_t data[8] = {0};
    return send_command(hfdcan, EL05_COMM_SET_ZERO, data);
}

HAL_StatusTypeDef EL05_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   float position_rad, float velocity_rad_s,
                                   float kp, float kd, float torque_nm)
{
    uint16_t p = float_to_u16(position_rad, P_MIN, P_MAX);
    uint16_t v = float_to_u16(velocity_rad_s, V_MIN, V_MAX);
    uint16_t kp_u = float_to_u16(kp, KP_MIN, KP_MAX);
    uint16_t kd_u = float_to_u16(kd, KD_MIN, KD_MAX);
    uint16_t torque_u = float_to_u16(torque_nm, T_MIN, T_MAX);
    uint8_t data[8] = {
        (uint8_t)(p >> 8), (uint8_t)p,
        (uint8_t)(v >> 8), (uint8_t)v,
        (uint8_t)(kp_u >> 8), (uint8_t)kp_u,
        (uint8_t)(kd_u >> 8), (uint8_t)kd_u
    };
    uint32_t identifier = ((uint32_t)EL05_COMM_MOTION << 24) |
                          ((uint32_t)torque_u << 8) | EL05_MIT_ID;

    /* 运动控制帧：扭矩在扩展 ID，位置/速度/Kp/Kd 在数据区。 */
    return AK_CAN_Send(hfdcan, identifier, FDCAN_EXTENDED_ID, data, 8);
}

uint8_t EL05_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header,
                               const uint8_t data[8])
{
    uint32_t identifier;
    uint16_t p, v, torque, temperature;

    if ((header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_EXTENDED_ID) ||
        (header->DataLength < FDCAN_DLC_BYTES_8)) {
        return 0;
    }

    identifier = header->Identifier;
    if (((identifier >> 24) & 0x3FU) != EL05_COMM_FEEDBACK ||
        (((identifier >> 8) & 0xFFU) != EL05_MIT_ID)) {
        return 0;
    }

    p = ((uint16_t)data[0] << 8) | data[1];
    v = ((uint16_t)data[2] << 8) | data[3];
    torque = ((uint16_t)data[4] << 8) | data[5];
    temperature = ((uint16_t)data[6] << 8) | data[7];

    el05_mit_state.position_rad = u16_to_float(p, P_MIN, P_MAX);
    el05_mit_state.velocity_rad_s = u16_to_float(v, V_MIN, V_MAX);
    el05_mit_state.torque_nm = u16_to_float(torque, T_MIN, T_MAX);
    el05_mit_state.temperature_c = (float)temperature * 0.1f;
    el05_mit_state.mode = (uint8_t)((identifier >> 22) & 0x03U);
    el05_mit_state.error_code = (uint8_t)((identifier >> 16) & 0x3FU);
    el05_mit_state.fault = (el05_mit_state.error_code != 0U);
    el05_mit_state.warning = 0U;
    el05_mit_state.last_rx_tick = HAL_GetTick();
    return 1;
}

uint8_t EL05_MIT_IsOnline(uint32_t timeout_ms)
{
    return (el05_mit_state.last_rx_tick != 0U) &&
           ((HAL_GetTick() - el05_mit_state.last_rx_tick) <= timeout_ms);
}
