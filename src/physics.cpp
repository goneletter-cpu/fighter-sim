#define GLM_ENABLE_EXPERIMENTAL
#include "physics.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/common.hpp>
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

    float airspeed = glm::length(v_body);
    float safe_speed = glm::max(airspeed, 0.5f);
    float q = 0.5f * air_density_ * safe_speed * safe_speed;  //  translated comment

    float forward_speed = glm::max(-v_body.z, 0.1f);
    float alpha = std::atan2(v_body.y, forward_speed);
    float beta  = std::atan2(v_body.x, forward_speed);

    float stall_alpha = alpha_stall_rad_;
    float stall_factor = 1.0f - glm::smoothstep(stall_alpha, alpha_max_rad_, std::abs(alpha));

    float q_hat = (safe_speed > 1.0f) ? (s.angular_vel.x * mean_chord / (2.0f * safe_speed)) : 0.0f;

    float CL = CL0_ + CL_alpha_ * alpha + CL_elevator_ * ctrl.elevator + CL_q_ * q_hat;
    CL *= stall_factor;

    float CD = CD0_ + k_induced_ * CL * CL + CD_beta_ * beta * beta;
    CD += CD_stall_ * (1.0f - stall_factor);

    float CY = CY_beta_ * beta + CY_rudder_ * ctrl.rudder;
    CY *= glm::mix(1.0f, 0.35f, 1.0f - stall_factor);

    float lift = q * wing_area * CL;
    float drag = q * wing_area * CD;
    float side = q * wing_area * CY;

    glm::vec3 v_hat = (airspeed > 1e-3f) ? (v_body / airspeed) : glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 right(1.0f, 0.0f, 0.0f);
    glm::vec3 lift_dir = glm::normalize(glm::cross(right, v_hat));
    if (glm::length(lift_dir) < 1e-4f) {
        lift_dir = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    glm::vec3 side_dir = glm::normalize(glm::cross(v_hat, lift_dir));

    glm::vec3 lift_force = lift_dir * lift;
    glm::vec3 drag_force = -v_hat * drag;
    glm::vec3 side_force = side_dir * side;

    return lift_force + drag_force + side_force;
}

//  translated comment
glm::vec3 FlightDynamics::aero_torque(const AircraftState& s,
                                       const ControlInput& ctrl) const {
    glm::mat3 body_from_world = glm::mat3_cast(glm::inverse(s.orientation));
    glm::vec3 v_body = body_from_world * s.velocity;
    float airspeed = glm::length(v_body);
    float safe_speed = glm::max(airspeed, 0.5f);
    float q = 0.5f * air_density_ * safe_speed * safe_speed;
    float qS = q * wing_area;

    float forward_speed = glm::max(-v_body.z, 0.1f);
    float alpha = std::atan2(v_body.y, forward_speed);
    float beta  = std::atan2(v_body.x, forward_speed);

    float stall_factor = 1.0f - glm::smoothstep(alpha_stall_rad_, alpha_max_rad_, std::abs(alpha));
    float p_hat = (safe_speed > 1.0f) ? (s.angular_vel.z * wing_span / (2.0f * safe_speed)) : 0.0f;
    float q_hat = (safe_speed > 1.0f) ? (s.angular_vel.x * mean_chord / (2.0f * safe_speed)) : 0.0f;
    float r_hat = (safe_speed > 1.0f) ? (s.angular_vel.y * wing_span / (2.0f * safe_speed)) : 0.0f;

    float Cm = Cm0_ + Cm_alpha_ * alpha + Cm_q_ * q_hat + Cm_elevator_ * ctrl.elevator;
    float Cl = Cl_beta_ * beta + Cl_p_ * p_hat + Cl_aileron_ * ctrl.aileron;
    float Cn = Cn_beta_ * beta + Cn_r_ * r_hat + Cn_rudder_ * ctrl.rudder;

    float eff = glm::mix(0.35f, 1.0f, stall_factor);
    Cm *= eff;
    Cl *= eff;
    Cn *= eff;

    return glm::vec3(
        qS * mean_chord * Cm,   //  translated comment
        qS * wing_span  * Cn,   //  translated comment
        qS * wing_span  * Cl    //  translated comment
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
