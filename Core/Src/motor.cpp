//
// Created by 81301 on 2024/10/26.
//
#include "motor.h"

Motor::Motor() {
    ratio = 3591 / 187; // 电机减速比

    motor_pid = PID(0, 0, 0, 0, 0);

    ecd_angle = 0; // 当前电机编码器角度 range:[0,8191]
    last_ecd_angle = 0; // 上次电机编码器角度 range:[0,8191]
    delta_ecd_angle = 0; // 编码器端新转动的角度
    total_ecd_angle = 0; // 编码器转过的总角度
    round_cnt = 0; // 转过的总圈数

    rotate_speed = 0; // rpm 反馈转子转速
    current = 0; // A 反馈转矩电流
    temp = 0; // °C 反馈电机温度
}

Motor::Motor(PID pid) {
    ratio = 3591 / 187; // 电机减速比

    motor_pid = pid;

    ecd_angle = 0; // 当前电机编码器角度 range:[0,8191]
    last_ecd_angle = 0; // 上次电机编码器角度 range:[0,8191]
    delta_ecd_angle = 0; // 编码器端新转动的角度
    total_ecd_angle = 0; // 编码器转过的总角度
    round_cnt = 0; // 转过的总圈数

    rotate_speed = 0; // rpm 反馈转子转速
    current = 0; // A 反馈转矩电流
    temp = 0; // °C 反馈电机温度
}

float Motor::linear_mapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return out_min + 1.0 * (in - in_min) / (in_max - in_min) * (out_max - out_min);
}

void Motor::data_process(uint8_t data[8]) {
    rotate_speed = (int16_t)((data[2] << 8) | data[3]);
    current = linear_mapping((int16_t)((data[4] << 8) | data[5]), -32768, 32767, -20, 20);
    // temp = linear_mapping((uint8_t)data[6], 0, 255, 0, 125);

    last_ecd_angle = ecd_angle;
    ecd_angle = linear_mapping((uint16_t)((data[0] << 8) | data[1]), 0, 8191, 0, 360);
    delta_ecd_angle = ecd_angle - last_ecd_angle;

    if (delta_ecd_angle > 180) {
        round_cnt--;
    } else if (delta_ecd_angle < -180) {
        round_cnt++;
    }

    total_ecd_angle = 360 * round_cnt + ecd_angle;
}

extern CAN_TxHeaderTypeDef can_tx_header;
void Motor::set_speed(uint16_t spd) { // 单位 rpm
    uint32_t tx_mailbox;
    uint8_t can_tx_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint16_t ctrl = 0;

    ctrl = motor_pid.calc(spd, rotate_speed);

    can_tx_data[0] = ctrl >> 8;
    can_tx_data[1] = ctrl & 0xFF;
    HAL_CAN_AddTxMessage(&hcan1, &can_tx_header, can_tx_data, &tx_mailbox);
}
