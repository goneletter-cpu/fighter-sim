#include "flight_controller.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/common.hpp>
#include <cmath>

FlightController::FlightController()
    : pitch_pid(1.1f, 0.05f, 0.25f, 0.45f)
    , roll_pid (1.3f, 0.05f, 0.22f, 0.45f)
    , yaw_pid  (0.7f, 0.03f, 0.18f, 0.35f)
{}

ControlInput FlightController::update(const AircraftState& state,
                                       const AttitudeCommand& cmd,
                                       float dt)
{
    glm::mat3 world_from_body = glm::mat3_cast(state.orientation);
    glm::vec3 forward = glm::normalize(world_from_body * glm::vec3(0, 0, -1));
    glm::vec3 up      = glm::normalize(world_from_body * glm::vec3(0, 1, 0));
    const glm::vec3 world_up(0.0f, 1.0f, 0.0f);

    //  translated comment
    glm::vec3 right_ref = glm::cross(world_up, forward);
    if (glm::length(right_ref) < 1e-4f) {
        right_ref = glm::cross(glm::vec3(1, 0, 0), forward);
    }
    right_ref = glm::normalize(right_ref);
    glm::vec3 up_ref = glm::normalize(glm::cross(forward, right_ref));
    float roll = std::atan2(glm::dot(glm::cross(up_ref, up), forward),
                            glm::dot(up_ref, up));
    float pitch = std::asin(glm::clamp(forward.y, -1.0f, 1.0f));

    ControlInput ctrl;
    ctrl.elevator = pitch_pid.update(cmd.pitch_rad, pitch, dt);
    ctrl.aileron  = roll_pid .update(cmd.roll_rad,  roll,  dt);
    ctrl.rudder   = yaw_pid  .update(cmd.yaw_rate_rad_s, state.angular_vel.y, dt);
    ctrl.throttle = cmd.throttle;

    //  translated comment
    if (up.y < 0.0f) {
        float recover = glm::clamp(-up.y, 0.0f, 1.0f);
        ctrl.aileron += glm::clamp(-roll * 0.55f * recover, -0.35f, 0.35f);
    }

    ctrl.elevator = glm::clamp(ctrl.elevator, -1.0f, 1.0f);
    ctrl.aileron  = glm::clamp(ctrl.aileron,  -1.0f, 1.0f);
    ctrl.rudder   = glm::clamp(ctrl.rudder,   -1.0f, 1.0f);

    return ctrl;
}

void FlightController::reset() {
    pitch_pid.reset();
    roll_pid .reset();
    yaw_pid  .reset();
}
