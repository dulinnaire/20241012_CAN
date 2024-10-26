#include "can.h"
#include "motor.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "tim.h"

uint8_t can_rx_buff[8];
CAN_RxHeaderTypeDef can_rx_header;

PID motor2006_pid(20, 2, 0.5, 2500, 1000);
Motor motor2006(motor2006_pid);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        // 接收到报文，进行处理
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_buff) == HAL_OK) {
            motor2006.data_process(can_rx_buff);
        }
    }
}

uint16_t ref = 150;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim1) {
        motor2006.set_speed(ref);
    }
}

