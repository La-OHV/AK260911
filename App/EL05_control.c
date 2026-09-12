//
// Created by 23906 on 2026/9/12.
//

#include "EL05_control.h"

#include "cmsis_os2.h"
#include "../el05/el05_mit.h"

/* 可在调试器中直接修改这5个MIT控制量。 */
float el05_position = 0.0f;
float el05_velocity = 0.0f;
float el05_kp = 0.0f;
float el05_kd = 0.0f;
float el05_torque = 0.0f;
volatile HAL_StatusTypeDef el05_tx_status;

void EL05_control(void *argument)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    el05_tx_status = EL05_MIT_Enable(&hfdcan1);
    osDelay(10);

    for (;;)
    {
        /* 读取并解析 EL05 的反馈帧。 */
        while (AK_CAN_Receive(&hfdcan1, &rx_header, rx_data))
            EL05_MIT_ParseFeedback(&rx_header, rx_data);

        el05_tx_status = EL05_MIT_Control(&hfdcan1,
                                          el05_position, el05_velocity,
                                          el05_kp, el05_kd, el05_torque);
        osDelay(2);
    }
}
