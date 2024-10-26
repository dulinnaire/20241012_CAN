//
// Created by 81301 on 2024/10/26.
//
#include "pid.h"

float PID::in_range(float val, float min, float max) {
    return val < min ? min : (val > max ? max : val);
}

PID::PID():
    error_sum_(0),
    ref_(0),
    fdb_(0),
    kp_(0),
    ki_(0),
    kd_(0),
    p_out_(0),
    i_out_(0),
    d_out_(0),
    i_max_(0),
    out_max_(0),
    output_(0) {
    error_[0] = 0;
    error_[1] = 0;
}

PID::PID(float kp, float ki, float kd, float out_max, float i_max):
    error_sum_(0),
    ref_(0),
    fdb_(0),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    p_out_(0),
    i_out_(0),
    d_out_(0),
    i_max_(i_max),
    out_max_(out_max),
    output_(0) {
    error_[0] = 0;
    error_[1] = 0;
}

float PID::calc(float ref, float fdb) {
    // update last error
    error_[1] = error_[0];
    // update current error
    ref_ = ref;
    fdb_ = fdb;
    error_[0] = ref_ - fdb_;

    // calculate error sum
    error_sum_ += error_[0];
    // limit error sum
    error_sum_ = in_range(error_sum_, -i_max_, i_max_);

    // calculate p_out
    p_out_ = kp_ * error_[0];
    // calculate i_out
    i_out_ = ki_ * error_sum_;
    // calculate d_out
    d_out_ = kd_ * (error_[0] - error_[1]);

    // calculate output
    output_ = p_out_ + i_out_ + d_out_;
    // limit output
    return in_range(output_, -out_max_, out_max_);
}

float AnglePID::calc(float ref, float fdb) {
    if (ref - fdb < -180) {
        fdb -= 360;
    } else if (ref - fdb > 180) {
        fdb += 360;
    }
    return PID::calc(ref, fdb);
}

AnglePID::AnglePID() {}

AnglePID::AnglePID(float kp, float ki, float kd, float out_max, float i_max):
    PID(kp, ki, kd, out_max, i_max) {}

CascadePID::CascadePID() {
    inner_pid_ = PID(0, 0, 0, 0, 0);
    outer_pid_ = AnglePID(0, 0, 0, 0, 0);
}

CascadePID::CascadePID(PID inner_pid, AnglePID outer_pid) {
    inner_pid_ = inner_pid;
    outer_pid_ = outer_pid;
}

float CascadePID::calc(float outer_ref, float outer_fdb, float inner_fdb) {
    float inner_ref = outer_pid_.calc(outer_ref, outer_fdb);
    return inner_pid_.calc(inner_ref, inner_fdb);
}
