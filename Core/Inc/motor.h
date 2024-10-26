//
// Created by 81301 on 2024/10/26.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "can.h"
#include "pid.h"
#include "stm32f4xx.h"

class Motor {
public:
    Motor();
    Motor(PID speed_pid, CascadePID angle_pid);

    void data_process(uint8_t data[8]);

    void set_speed(uint16_t spd);

    void set_angle(float angle);

private:
    float linear_mapping(int in, int in_min, int in_max, float out_min, float out_max);

    PID motor_speed_pid;

    CascadePID motor_angle_pid;

    float ratio; // 电机减速比

    float ecd_angle; // 当前电机编码器角度 range:[0,8191]
    float last_ecd_angle; // 上次电机编码器角度 range:[0,8191]
    float delta_ecd_angle; // 编码器端新转动的角度
    float total_ecd_angle; // 编码器转过的总角度
    int32_t round_cnt; // 转过的总圈数

    float rotate_speed; // rpm 反馈转子转速
    float current; // A 反馈转矩电流
    float temp; // °C 反馈电机温度
};

#endif //MOTOR_H
