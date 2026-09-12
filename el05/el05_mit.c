#include "el05_mit.h"

/* EL05说明书规定的MIT参数范围。 */
#define P_MIN   (-12.57f)
#define P_MAX   ( 12.57f)
#define V_MIN   (-50.0f)
#define V_MAX   ( 50.0f)
#define KP_MIN  (  0.0f)
#define KP_MAX  (500.0f)
#define KD_MIN  (  0.0f)
#define KD_MAX  (  5.0f)
#define T_MIN   ( -6.0f)
#define T_MAX   (  6.0f)

EL05_MIT_State el05_mit_state = {0};

static uint32_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    if (value < min) value = min;
    if (value > max) value = max;
    return (uint32_t)((value - min) * (float)((1UL << bits) - 1UL) / (max - min));
}

static float uint_to_float(uint32_t value, float min, float max, uint8_t bits)
{
    return (float)value * (max - min) / (float)((1UL << bits) - 1UL) + min;
}

static HAL_StatusTypeDef send_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t command)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
    return AK_CAN_Send(hfdcan, EL05_MIT_ID, FDCAN_STANDARD_ID, data, 8);
}

HAL_StatusTypeDef EL05_MIT_Enable(FDCAN_HandleTypeDef *hfdcan)
{
    return send_mode(hfdcan, 0xFC);
}

HAL_StatusTypeDef EL05_MIT_Disable(FDCAN_HandleTypeDef *hfdcan)
{
    return send_mode(hfdcan, 0xFD);
}

HAL_StatusTypeDef EL05_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan)
{
    return send_mode(hfdcan, 0xFE);
}

HAL_StatusTypeDef EL05_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   float position_rad, float velocity_rad_s,
                                   float kp, float kd, float torque_nm)
{
    uint32_t p = float_to_uint(position_rad, P_MIN, P_MAX, 16);
    uint32_t v = float_to_uint(velocity_rad_s, V_MIN, V_MAX, 12);
    uint32_t kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t t = float_to_uint(torque_nm, T_MIN, T_MAX, 12);
    uint8_t data[8];

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (kp_u >> 8));
    data[4] = (uint8_t)kp_u;
    data[5] = (uint8_t)(kd_u >> 4);
    data[6] = (uint8_t)((kd_u << 4) | (t >> 8));
    data[7] = (uint8_t)t;

    return AK_CAN_Send(hfdcan, EL05_MIT_ID, FDCAN_STANDARD_ID, data, 8);
}

uint8_t EL05_MIT_ParseFeedback(const FDCAN_RxHeaderTypeDef *header,
                               const uint8_t data[8])
{
    uint32_t p, v, t;
    uint16_t temperature;

    if ((header == NULL) || (data == NULL) ||
        (header->IdType != FDCAN_STANDARD_ID) ||
        (header->DataLength < FDCAN_DLC_BYTES_8) ||
        (data[0] != EL05_MIT_ID)) {
        return 0;
    }

    p = ((uint32_t)data[1] << 8) | data[2];
    v = ((uint32_t)data[3] << 4) | (data[4] >> 4);
    t = ((uint32_t)(data[4] & 0x0FU) << 8) | data[5];
    temperature = ((uint16_t)(data[6] & 0x0FU) << 8) | data[7];

    el05_mit_state.position_rad = uint_to_float(p, P_MIN, P_MAX, 16);
    el05_mit_state.velocity_rad_s = uint_to_float(v, V_MIN, V_MAX, 12);
    el05_mit_state.torque_nm = uint_to_float(t, T_MIN, T_MAX, 12);
    el05_mit_state.temperature_c = (float)temperature * 0.1f;
    el05_mit_state.mode = (data[6] >> 6) & 0x03U;
    el05_mit_state.fault = (data[6] >> 5) & 0x01U;
    el05_mit_state.warning = (data[6] >> 4) & 0x01U;
    el05_mit_state.last_rx_tick = HAL_GetTick();
    return 1;
}

uint8_t EL05_MIT_IsOnline(uint32_t timeout_ms)
{
    return (el05_mit_state.last_rx_tick != 0U) &&
           ((HAL_GetTick() - el05_mit_state.last_rx_tick) <= timeout_ms);
}
