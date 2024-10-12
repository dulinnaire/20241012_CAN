#include "can.h"
#include "gpio.h"
#include "stm32f4xx.h"
#include "tim.h"

uint8_t can_rx_buff[8];
CAN_RxHeaderTypeDef can_rx_header;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        // 接收到报文，进行处理
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_buff) == HAL_OK) {
            if (can_rx_buff[0] == 'A') {
                HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
            }
        }
    }
}
extern CAN_TxHeaderTypeDef can_tx_header;





