#pragma once

//  translated comment
//  translated comment
class PID {
public:
    PID(float kp, float ki, float kd, float output_limit = 1.0f);

    //  translated comment
    //  translated comment
    float update(float setpoint, float measured, float dt);

    void reset();
    void set_gains(float kp, float ki, float kd);

private:
    float kp_, ki_, kd_;
    float output_limit_;   //  translated comment

    float integral_;       //  translated comment
    float prev_error_;     //  translated comment
    bool  first_run_;
};
