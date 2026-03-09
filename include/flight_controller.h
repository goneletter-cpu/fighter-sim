#pragma once
#include "pid.h"
#include "physics.h"

//  translated comment
struct AttitudeCommand {
    float pitch_rad = 0.0f;
    float roll_rad  = 0.0f;
    float yaw_rate_rad_s = 0.0f;  //  translated comment
    float throttle  = 0.5f;
};

//  translated comment
//  translated comment
class FlightController {
public:
    FlightController();

    ControlInput update(const AircraftState& state,
                        const AttitudeCommand& cmd,
                        float dt);

    void reset();

    //  translated comment
    PID pitch_pid;
    PID roll_pid;
    PID yaw_pid;
};
