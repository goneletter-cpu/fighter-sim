#pragma once

// 单轴PID控制器
// 用于控制俯仰/滚转/偏航
class PID {
public:
    PID(float kp, float ki, float kd, float output_limit = 1.0f);

    // 输入：目标值、当前值、时间步长
    // 输出：控制量（已限幅）
    float update(float setpoint, float measured, float dt);

    void reset();
    void set_gains(float kp, float ki, float kd);

private:
    float kp_, ki_, kd_;
    float output_limit_;   // 输出限幅，防止舵面过饱和

    float integral_;       // 积分项（带anti-windup）
    float prev_error_;     // 上一帧误差（用于微分）
    bool  first_run_;
};
