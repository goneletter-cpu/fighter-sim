#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

//  translated comment
struct ControlInput {
    float elevator;   //  translated comment
    float aileron;    //  translated comment
    float rudder;     //  translated comment
    float throttle;   //  translated comment
};

//  translated comment
struct AircraftState {
    glm::vec3 position;       //  translated comment
    glm::vec3 velocity;       //  translated comment
    glm::quat orientation;    //  translated comment
    glm::vec3 angular_vel;    //  translated comment

    //  translated comment
    glm::vec3 euler_angles() const;  //  translated comment
};

//  translated comment
//  translated comment
class FlightDynamics {
public:
    FlightDynamics();

    // RK4 translated comment
    void step(AircraftState& state, const ControlInput& ctrl, float dt);

    //  translated comment
    float mass         = 9000.0f;   // kg， translated comment
    float max_thrust   = 76000.0f;  // N
    float wing_area    = 28.0f;     // m²
    float wing_span    = 9.5f;      // m
    float mean_chord   = 3.0f;      // m

private:
    //  translated comment
    struct Derivative {
        glm::vec3 dpos;
        glm::vec3 dvel;
        glm::quat dorientation;
        glm::vec3 dangular_vel;
    };

    Derivative compute_derivative(const AircraftState& s,
                                  const ControlInput& ctrl) const;

    //  translated comment
    glm::vec3 aero_force(const AircraftState& s, const ControlInput& ctrl) const;
    glm::vec3 aero_torque(const AircraftState& s, const ControlInput& ctrl) const;

    //  translated comment
    glm::vec3 inertia_ = {12875.0f, 75674.0f, 85552.0f}; // Ixx Iyy Izz (kg·m²)

    //  translated comment
    float air_density_ = 1.225f;   // kg/m^3 sea-level ISA
    float alpha_stall_rad_ = 0.30f; // ~17 deg
    float alpha_max_rad_   = 0.55f; // ~31 deg

    //  translated comment
    float CL0_ = 0.25f;
    float CL_alpha_ = 4.6f;
    float CL_elevator_ = 0.6f;
    float CL_q_ = 7.5f;

    float CD0_ = 0.022f;
    float k_induced_ = 0.075f;
    float CD_beta_ = 0.22f;
    float CD_stall_ = 0.65f;

    float CY_beta_ = -0.85f;
    float CY_rudder_ = 0.18f;

    float Cm0_ = 0.04f;
    float Cm_alpha_ = -1.15f;
    float Cm_q_ = -9.5f;
    float Cm_elevator_ = -1.05f;

    float Cl_beta_ = -0.12f;
    float Cl_p_ = -0.55f;
    float Cl_aileron_ = 0.28f;

    float Cn_beta_ = 0.22f;
    float Cn_r_ = -0.35f;
    float Cn_rudder_ = -0.12f;
};
