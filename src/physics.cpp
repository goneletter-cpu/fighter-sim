#define GLM_ENABLE_EXPERIMENTAL
#include "physics.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <cmath>

// ── AircraftState ─────────────────────────────────────────────────────────────

glm::vec3 AircraftState::euler_angles() const {
    //  translated comment
    return glm::eulerAngles(orientation);  //  translated comment
}

// ── FlightDynamics ────────────────────────────────────────────────────────────

FlightDynamics::FlightDynamics() {}

//  translated comment
glm::vec3 FlightDynamics::aero_force(const AircraftState& s,
                                      const ControlInput& ctrl) const {
    //  translated comment
    glm::mat3 body_from_world = glm::mat3_cast(glm::inverse(s.orientation));
    glm::vec3 v_body = body_from_world * s.velocity;

    float airspeed = glm::length(s.velocity);
    float q = 0.5f * 1.225f * airspeed * airspeed;  //  translated comment

    //  translated comment
    //  translated comment
    float alpha = 0.0f;
    if (airspeed > 1.0f) {
        float forward_speed = -v_body.z;
        if (forward_speed < 0.1f) forward_speed = 0.1f;
        alpha = std::atan2(-v_body.y, forward_speed);
    }
    //  translated comment
    alpha = glm::clamp(alpha, glm::radians(-25.0f), glm::radians(25.0f));
    float CL = 0.1f + 2.0f * alpha + 0.5f * ctrl.elevator;
    float CD = 0.02f + 0.1f * CL * CL;

    float lift = q * wing_area * CL;
    float drag = q * wing_area * CD;

    //  translated comment
    glm::vec3 lift_force  = glm::vec3(0, lift, 0);
    glm::vec3 drag_force  = glm::vec3(0, 0, drag);   //  translated comment

    return lift_force - drag_force;
}

//  translated comment
glm::vec3 FlightDynamics::aero_torque(const AircraftState& s,
                                       const ControlInput& ctrl) const {
    float airspeed = glm::length(s.velocity);
    float q = 0.5f * 1.225f * airspeed * airspeed;
    float qS = q * wing_area;

    //  translated comment
    float Cm = -2.0f * ctrl.elevator - 0.5f * s.angular_vel.x;  //  translated comment
    float Cl = -1.5f * ctrl.aileron  - 0.3f * s.angular_vel.z;  //  translated comment
    float Cn =  1.0f * ctrl.rudder   - 0.5f * s.angular_vel.y;  //  translated comment

    float mean_chord = 3.0f;  //  translated comment
    float span       = 9.5f;  //  translated comment

    return glm::vec3(
        qS * mean_chord * Cm,   //  translated comment
        qS * span       * Cn,   //  translated comment
        qS * span       * Cl    //  translated comment
    );
}

FlightDynamics::Derivative FlightDynamics::compute_derivative(
    const AircraftState& s, const ControlInput& ctrl) const
{
    Derivative d;

    //  translated comment
    d.dpos = s.velocity;

    //  translated comment
    glm::mat3 world_from_body = glm::mat3_cast(s.orientation);

    glm::vec3 thrust_body  = glm::vec3(0, 0, -max_thrust * ctrl.throttle); //  translated comment
    glm::vec3 gravity_world = glm::vec3(0, -9.81f * mass, 0);

    glm::vec3 aero_body  = aero_force(s, ctrl);
    glm::vec3 total_force = world_from_body * (thrust_body + aero_body)
                          + gravity_world;

    d.dvel = total_force / mass;

    //  translated comment
    glm::quat omega_quat(0, s.angular_vel.x, s.angular_vel.y, s.angular_vel.z);
    d.dorientation = 0.5f * s.orientation * omega_quat;

    //  translated comment
    glm::vec3 torque = aero_torque(s, ctrl);
    glm::vec3 w = s.angular_vel;
    d.dangular_vel = glm::vec3(
        (torque.x - (inertia_.z - inertia_.y) * w.y * w.z) / inertia_.x,
        (torque.y - (inertia_.x - inertia_.z) * w.z * w.x) / inertia_.y,
        (torque.z - (inertia_.y - inertia_.x) * w.x * w.y) / inertia_.z
    );

    return d;
}

void FlightDynamics::step(AircraftState& s, const ControlInput& ctrl, float dt) {
    // RK4
    auto k1 = compute_derivative(s, ctrl);

    AircraftState s2;
    s2.position    = s.position    + k1.dpos         * (dt * 0.5f);
    s2.velocity    = s.velocity    + k1.dvel         * (dt * 0.5f);
    s2.orientation = glm::normalize(s.orientation    + k1.dorientation * (dt * 0.5f));
    s2.angular_vel = s.angular_vel + k1.dangular_vel * (dt * 0.5f);
    auto k2 = compute_derivative(s2, ctrl);

    AircraftState s3;
    s3.position    = s.position    + k2.dpos         * (dt * 0.5f);
    s3.velocity    = s.velocity    + k2.dvel         * (dt * 0.5f);
    s3.orientation = glm::normalize(s.orientation    + k2.dorientation * (dt * 0.5f));
    s3.angular_vel = s.angular_vel + k2.dangular_vel * (dt * 0.5f);
    auto k3 = compute_derivative(s3, ctrl);

    AircraftState s4;
    s4.position    = s.position    + k3.dpos         * dt;
    s4.velocity    = s.velocity    + k3.dvel         * dt;
    s4.orientation = glm::normalize(s.orientation    + k3.dorientation * dt);
    s4.angular_vel = s.angular_vel + k3.dangular_vel * dt;
    auto k4 = compute_derivative(s4, ctrl);

    s.position    += (k1.dpos         + 2.0f*k2.dpos         + 2.0f*k3.dpos         + k4.dpos        ) * (dt / 6.0f);
    s.velocity    += (k1.dvel         + 2.0f*k2.dvel         + 2.0f*k3.dvel         + k4.dvel        ) * (dt / 6.0f);
    s.orientation  = glm::normalize(s.orientation +
                    (k1.dorientation  + 2.0f*k2.dorientation + 2.0f*k3.dorientation + k4.dorientation) * (dt / 6.0f));
    s.angular_vel += (k1.dangular_vel + 2.0f*k2.dangular_vel + 2.0f*k3.dangular_vel + k4.dangular_vel) * (dt / 6.0f);
}
