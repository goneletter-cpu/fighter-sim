#include "pid.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd),
      output_limit_(output_limit),
      integral_(0.0f), prev_error_(0.0f), first_run_(true)
{}

float PID::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // 微分项（第一帧跳过，避免导数突变）
    float derivative = 0.0f;
    if (!first_run_) {
        derivative = (error - prev_error_) / dt;
    }
    first_run_ = false;

    // 比例项输出（先算，用于anti-windup判断）
    float p_out = kp_ * error;
    float d_out = kd_ * derivative;

    // 积分项 + anti-windup：只有在输出未饱和时才累积积分
    float pre_integral_out = p_out + ki_ * integral_ + d_out;
    bool saturated = (pre_integral_out > output_limit_) ||
                     (pre_integral_out < -output_limit_);

    // 条件积分：饱和时且误差和积分同号则不再累积
    bool same_sign = (error > 0 && integral_ > 0) ||
                     (error < 0 && integral_ < 0);
    if (!(saturated && same_sign)) {
        integral_ += error * dt;
    }

    float output = p_out + ki_ * integral_ + d_out;
    output = std::clamp(output, -output_limit_, output_limit_);

    prev_error_ = error;
    return output;
}

void PID::reset() {
    integral_  = 0.0f;
    prev_error_ = 0.0f;
    first_run_ = true;
}

void PID::set_gains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
    reset();
}
