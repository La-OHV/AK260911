//
// Created by 23906 on 2026/9/11.
//

#include "AK_control.h"

#include "cmsis_os2.h"
#include "../ak/ak_can.h"
#include "../ak/ak_mit.h"
#include "../el05/el05_mit.h"

float kp = 0.0f;
float kd = 0.0f;
float target_position[2] = {0.0f, 0.0f}; /* ID1、ID2 的目标位置 */
volatile HAL_StatusTypeDef tx_status[2];

void AK_control(void *argument)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t id;

    /* 使能 CAN ID 为 1、2 的两个电机。 */
    AK_MIT_Enable(&hfdcan1, 1);
    osDelay(10);
    AK_MIT_Enable(&hfdcan1, 2);
    osDelay(10);

    for (;;)
    {
        /* FDCAN1统一接收，再按电机ID保存反馈。 */
        while (AK_CAN_Receive(&hfdcan1, &rx_header, rx_data))
        {
            AK_MIT_ParseFeedback(&rx_header, rx_data);
            EL05_MIT_ParseFeedback(&rx_header, rx_data);
        }

        /* 依次控制 ID1 和 ID2。 */
        for (id = 1; id <= 2; id++)
        {
            tx_status[id - 1] = AK_MIT_Control(&hfdcan1, id,
                                               target_position[id - 1], 0.0f,
                                               kp, kd, 0.0f);
        }

        osDelay(2);
    }
}
