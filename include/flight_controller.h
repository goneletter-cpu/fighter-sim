#pragma once
#include "pid.h"
#include "physics.h"

// 目标姿态指令（来自飞行员输入或自动驾驶）
struct AttitudeCommand {
    float pitch_rad = 0.0f;
    float roll_rad  = 0.0f;
    float yaw_rate_rad_s = 0.0f;  // 偏航通常控制角速率而非绝对角
    float throttle  = 0.5f;
};

// 飞控系统：三轴PID回路
// 输入目标姿态 → 输出舵面偏转
class FlightController {
public:
    FlightController();

    ControlInput update(const AircraftState& state,
                        const AttitudeCommand& cmd,
                        float dt);

    void reset();

    // 暴露PID参数方便后续调参
    PID pitch_pid;
    PID roll_pid;
    PID yaw_pid;
};
