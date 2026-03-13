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
    PID pitch_rate_pid;
    PID roll_rate_pid;
    PID yaw_rate_pid;

    float pitch_att_gain = 3.2f;
    float roll_att_gain  = 4.0f;
    float yaw_rate_limit = 1.1f;
    float pitch_rate_limit = 1.5f;
    float roll_rate_limit  = 2.4f;
};
