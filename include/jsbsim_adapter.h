#pragma once

#include "physics.h"
#include <string>

namespace JSBSim {
class FGFDMExec;
}

class JSBSimAdapter {
public:
    JSBSimAdapter();
    ~JSBSimAdapter();

    bool init(const std::string& model,
              const AircraftState& initial_state,
              float dt);

    void set_controls(const ControlInput& ctrl);
    void set_aux_controls(float flap_norm, float gear_norm);
    void enable_autopilot();
    bool step(float dt);
    void sync_state(AircraftState& state) const;

private:
    JSBSim::FGFDMExec* fdm_;
    bool initialized_;
};
