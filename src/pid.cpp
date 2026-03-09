#include "pid.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd),
      output_limit_(output_limit),
      integral_(0.0f), prev_error_(0.0f), first_run_(true)
{}

float PID::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    //  translated comment
    float derivative = 0.0f;
    if (!first_run_) {
        derivative = (error - prev_error_) / dt;
    }
    first_run_ = false;

    //  translated comment
    float p_out = kp_ * error;
    float d_out = kd_ * derivative;

    //  translated comment
    float pre_integral_out = p_out + ki_ * integral_ + d_out;
    bool saturated = (pre_integral_out > output_limit_) ||
                     (pre_integral_out < -output_limit_);

    //  translated comment
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
