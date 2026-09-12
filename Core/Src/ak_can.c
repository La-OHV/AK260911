#include "ak_can.h"

HAL_StatusTypeDef AK_CAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL) return HAL_ERROR;

    /* ID 和掩码均为 0，表示接收全部标准帧。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;

    /* 同一个 FIFO0 也接收伺服模式使用的扩展帧。 */
    filter.IdType = FDCAN_EXTENDED_ID;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) return HAL_ERROR;

    /* 未通过滤波器的帧和远程帧直接丢弃。 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef AK_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
                              uint32_t id_type, const uint8_t *data,
                              uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((hfdcan == NULL) || (data == NULL) || (length > 8U) ||
        ((id_type != FDCAN_STANDARD_ID) && (id_type != FDCAN_EXTENDED_ID)) ||
        ((id_type == FDCAN_STANDARD_ID) && (id > 0x7FFU)) ||
        ((id_type == FDCAN_EXTENDED_ID) && (id > 0x1FFFFFFFU))) {
        return HAL_ERROR;
    }

    header.Identifier = id;
    header.IdType = id_type;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = length; /* 当前 HAL 中 0~8 与对应 DLC 值相同。 */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
}

uint8_t AK_CAN_Receive(FDCAN_HandleTypeDef *hfdcan,
                       FDCAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    if ((hfdcan == NULL) || (header == NULL) || (data == NULL) ||
        (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0U)) {
        return 0;
    }

    return HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, header, data) == HAL_OK;
}
