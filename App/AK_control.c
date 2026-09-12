//
// Created by 23906 on 2026/9/11.
//

#include "AK_control.h"

#include "cmsis_os2.h"
#include "../Core/Inc/ak_can.h"
#include "../Core/Inc/ak_mit.h"
float kp=0.0;
float kd=0.0;
volatile HAL_StatusTypeDef tx_status;
void AK_control(void *argument) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    AK_MIT_Enable(&hfdcan1, 1);  /* 电机ID=1 */
    osDelay(10);
    for (;;) {
        /* 接收反馈 */
        while (AK_CAN_Receive(&hfdcan1, &rx_header, rx_data))
        {
            AK_MIT_ParseFeedback(&rx_header, rx_data);
        }

        /* 位置控制：目标0 rad，Kp=5，Kd=0.2 */
        tx_status=AK_MIT_Control(&hfdcan1, 1,
                       0.0f, 0.0f,
                       kp, kd,
                       0.0f);

        osDelay(2);

    }
}