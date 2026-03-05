#include "flight_controller.h"
#include <glm/gtc/quaternion.hpp>

// PID参数初始值（之后需要根据飞行表现调整）
FlightController::FlightController()
    : pitch_pid(1.1f, 0.05f, 0.25f, 0.45f)
    , roll_pid (1.3f, 0.05f, 0.22f, 0.45f)
    , yaw_pid  (0.7f, 0.03f, 0.18f, 0.35f)
{}

ControlInput FlightController::update(const AircraftState& state,
                                       const AttitudeCommand& cmd,
                                       float dt)
{
    glm::vec3 euler = state.euler_angles();
    // GLM eulerAngles 返回顺序: pitch=x, yaw=y, roll=z
    float pitch = euler.x;
    float roll  = euler.z;

    ControlInput ctrl;
    ctrl.elevator = pitch_pid.update(cmd.pitch_rad, pitch, dt);
    ctrl.aileron  = roll_pid .update(cmd.roll_rad,  roll,  dt);
    // 偏航：控制角速率而非绝对角
    ctrl.rudder   = yaw_pid  .update(cmd.yaw_rate_rad_s, state.angular_vel.y, dt);
    ctrl.throttle = cmd.throttle;

    return ctrl;
}

void FlightController::reset() {
    pitch_pid.reset();
    roll_pid .reset();
    yaw_pid  .reset();
}
