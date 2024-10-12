#include "can.h"
#include "stm32f4xx.h"

uint8_t can_rx_buff[8];
CAN_RxHeaderTypeDef can_rx_header;

float ratio = 3591 / 187; // 电机减速比

float ecd_angle; // 当前电机编码器角度 range:[0,8191]
float last_ecd_angle; // 上次电机编码器角度 range:[0,8191]
float delta_ecd_angle; // 编码器端新转动的角度
float total_ecd_angle; // 编码器转过的总角度
int32_t round_cnt; // 转过的总圈数

float rotate_speed; // dps 反馈转子转速
float current; // A 反馈转矩电流
float temp; // °C 反馈电机温度

static float linear_mapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return out_min + 1.0 * (in - in_min) / (in_max - in_min) * (out_max - out_min);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        // 接收到报文，进行处理
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, can_rx_buff) == HAL_OK) {
            // rotate_speed =
            //  linear_mapping((can_rx_buff[2] << 8) | can_rx_buff[3], -32768, 32767, -500, 500);
            rotate_speed = (int16_t)((can_rx_buff[2] << 8) | can_rx_buff[3]);
            current = linear_mapping(
                (int16_t)((can_rx_buff[4] << 8) | can_rx_buff[5]),
                -32768,
                32767,
                -20,
                20
            );
            temp = linear_mapping((uint8_t)can_rx_buff[6], 0, 255, 0, 125);

            last_ecd_angle = ecd_angle;
            ecd_angle =
                linear_mapping((uint16_t)((can_rx_buff[0] << 8) | can_rx_buff[1]), 0, 8191, 0, 360);
            delta_ecd_angle = ecd_angle - last_ecd_angle;

            // if (rotate_speed > 0) {
            //     if (delta_ecd_angle < 0) {
            //         round_cnt++;
            //     }
            // } else if (rotate_speed < 0) {
            //     if (delta_ecd_angle > 0) {
            //         round_cnt--;
            //     }
            // }
            if (delta_ecd_angle > 180) {
                round_cnt--;
            } else if (delta_ecd_angle < -180) {
                round_cnt++;
            }

            total_ecd_angle = 360 * round_cnt + ecd_angle;

        }
    }
}