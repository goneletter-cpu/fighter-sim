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
};
