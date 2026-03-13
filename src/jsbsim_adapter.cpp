#include "jsbsim_adapter.h"
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace {
constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 3.28083989501312;
}

JSBSimAdapter::JSBSimAdapter()
    : fdm_(nullptr)
    , initialized_(false)
{}

JSBSimAdapter::~JSBSimAdapter() {
    delete fdm_;
}

bool JSBSimAdapter::init(const std::string& model,
                         const AircraftState& initial_state,
                         float dt) {
    if (!fdm_) fdm_ = new JSBSim::FGFDMExec();

    std::string model_name = model;
    std::string root = std::string(FIGHTER_SIM_ROOT) + "/third_party/jsbsim";
    std::string aircraft_path = "aircraft";
    std::string engine_path = "engine";
    std::string systems_path = "systems";

    auto apply_fg_paths = [&](const std::string& name) {
        const size_t slash = name.find('/');
        if (slash == std::string::npos) return;
        const std::string aircraft_dir = name.substr(0, slash);
        const std::string base_dir = aircraft_path + "/" + aircraft_dir;
        const std::string engines_dir = base_dir + "/Engines";
        const std::string systems_dir = base_dir + "/Systems";
        // Prefer subfolders when present (FGData/FGAddon layout).
        engine_path = std::filesystem::exists(root + "/" + engines_dir) ? engines_dir : base_dir;
        systems_path = std::filesystem::exists(root + "/" + systems_dir) ? systems_dir : base_dir;
    };

    if (model.rfind("fgaddon:", 0) == 0) {
        model_name = model.substr(8);
        root = std::string(FIGHTER_SIM_ROOT) + "/third_party/fgaddon";
        aircraft_path = "Aircraft";
        apply_fg_paths(model_name);
    } else if (model.rfind("fgdata:", 0) == 0) {
        model_name = model.substr(7);
        root = std::string(FIGHTER_SIM_ROOT) + "/third_party/fgdata";
        aircraft_path = "Aircraft";
        apply_fg_paths(model_name);
    }

    fdm_->SetRootDir(SGPath(root));
    fdm_->SetAircraftPath(SGPath(aircraft_path));
    fdm_->SetEnginePath(SGPath(engine_path));
    fdm_->SetSystemsPath(SGPath(systems_path));

    const bool add_model_to_path = (model_name.find('/') == std::string::npos);
    if (!fdm_->LoadModel(model_name, add_model_to_path)) {
        return false;
    }

    fdm_->Setdt(dt);

    const glm::vec3 pos_m = initial_state.position;
    const glm::vec3 vel_mps = initial_state.velocity;
    const glm::vec3 euler = initial_state.euler_angles();

    const double vn_fps = -vel_mps.z * kMToFt;
    const double ve_fps =  vel_mps.x * kMToFt;
    const double vd_fps = -vel_mps.y * kMToFt;

    auto ic = fdm_->GetIC();
    ic->SetLatitudeDegIC(0.0);
    ic->SetLongitudeDegIC(0.0);
    ic->SetAltitudeASLFtIC(pos_m.y * kMToFt);
    ic->SetVNorthFpsIC(vn_fps);
    ic->SetVEastFpsIC(ve_fps);
    ic->SetVDownFpsIC(vd_fps);
    ic->SetPhiRadIC(euler.z);
    ic->SetThetaRadIC(euler.x);
    ic->SetPsiRadIC(euler.y);
    ic->SetPRadpsIC(0.0);
    ic->SetQRadpsIC(0.0);
    ic->SetRRadpsIC(0.0);
    ic->SetWindNEDFpsIC(0.0, 0.0, 0.0);
    ic->SetTerrainElevationFtIC(0.0);

    fdm_->SetPropertyValue("propulsion/cutoff_cmd", 0.0);
    fdm_->SetPropertyValue("propulsion/starter_cmd", 1.0);
    fdm_->SetPropertyValue("propulsion/engine[0]/set-running", 1.0);
    fdm_->SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
    fdm_->SetPropertyValue("fcs/mixture-cmd-norm[0]", 1.0);
    fdm_->SetPropertyValue("fcs/left-brake-cmd-norm", 0.0);
    fdm_->SetPropertyValue("fcs/right-brake-cmd-norm", 0.0);
    fdm_->SetPropertyValue("fcs/parking-brake-cmd-norm", 0.0);
    fdm_->SetPropertyValue("forces/hold-down", 0.0);
    fdm_->SetPropertyValue("fcs/flap-cmd-norm", 0.0);
    fdm_->SetPropertyValue("fcs/roll-trim-cmd-norm", 0.0);
    fdm_->SetPropertyValue("fcs/pitch-trim-cmd-norm", 0.0);
    fdm_->SetPropertyValue("fcs/yaw-trim-cmd-norm", 0.0);
    // FGData c172p expects this to exist for bushkit logic.
    fdm_->SetPropertyValue("bushkit", 0.0);
    fdm_->SetPropertyValue("fdm/jsbsim/bushkit", 0.0);
    fdm_->SetPropertyValue("controls/engines/active-engine", 0.0);
    fdm_->SetPropertyValue("controls/engines/engine/use-primer", 0.0);
    fdm_->SetPropertyValue("controls/gear/brake-parking", 0.0);
    fdm_->SetPropertyValue("controls/gear/water-rudder", 0.0);
    fdm_->SetPropertyValue("sim/model/c172p/securing/chock-visible", 0.0);
    fdm_->SetPropertyValue("fdm/jsbsim/damage/repairing", 0.0);
    fdm_->SetPropertyValue("sim/model/c172p/hydraulics/hydraulic-pump", 1.0);
    fdm_->SetPropertyValue("engines/active-engine/killed", 0.0);
    fdm_->SetPropertyValue("engines/active-engine/oil-lacking", 0.0);
    fdm_->SetPropertyValue("fcs/stick-force-per-g", 0.0);

    initialized_ = fdm_->RunIC();
    return initialized_;
}

void JSBSimAdapter::set_controls(const ControlInput& ctrl) {
    if (!initialized_) return;
    const double aileron = std::clamp((double)ctrl.aileron, -1.0, 1.0);
    const double elevator = std::clamp((double)ctrl.elevator, -1.0, 1.0);
    const double rudder = std::clamp((double)ctrl.rudder, -1.0, 1.0);
    const double throttle = std::clamp((double)ctrl.throttle, 0.0, 1.0);

    fdm_->SetPropertyValue("fcs/aileron-cmd-norm", aileron);
    fdm_->SetPropertyValue("fcs/elevator-cmd-norm", elevator);
    fdm_->SetPropertyValue("fcs/rudder-cmd-norm", rudder);
    fdm_->SetPropertyValue("fcs/throttle-cmd-norm", throttle);
    fdm_->SetPropertyValue("fcs/throttle-cmd-norm[0]", throttle);
    fdm_->SetPropertyValue("fcs/mixture-cmd-norm", 1.0);
    fdm_->SetPropertyValue("fcs/mixture-cmd-norm[0]", 1.0);
}

void JSBSimAdapter::set_aux_controls(float flap_norm, float gear_norm) {
    if (!initialized_) return;
    const double flap = std::clamp((double)flap_norm, 0.0, 1.0);
    const double gear = std::clamp((double)gear_norm, 0.0, 1.0);
    fdm_->SetPropertyValue("fcs/flap-cmd-norm", flap);
    fdm_->SetPropertyValue("fcs/gear-cmd-norm", gear);
}

void JSBSimAdapter::enable_autopilot() {
    if (!initialized_) return;
    const double heading_rad = fdm_->GetPropertyValue("attitude/heading-true-rad");
    const double heading_deg = heading_rad * 57.29577951308232;
    double alt_ft = fdm_->GetPropertyValue("position/h-agl-ft");
    if (alt_ft < 1.0) {
        alt_ft = fdm_->GetPropertyValue("position/h-sl-ft");
    }

    fdm_->SetPropertyValue("ap/attitude_hold", 1.0);
    fdm_->SetPropertyValue("ap/heading_hold", 1.0);
    fdm_->SetPropertyValue("ap/altitude_hold", 1.0);
    fdm_->SetPropertyValue("ap/heading_setpoint", heading_deg);
    fdm_->SetPropertyValue("ap/altitude_setpoint", alt_ft);
}

bool JSBSimAdapter::step(float dt) {
    if (!initialized_) return false;
    fdm_->Setdt(dt);
    return fdm_->Run();
}

void JSBSimAdapter::sync_state(AircraftState& state) const {
    if (!initialized_) return;

    const double n_ft = fdm_->GetPropertyValue("position/from-start-neu-n-ft");
    const double e_ft = fdm_->GetPropertyValue("position/from-start-neu-e-ft");
    const double u_ft = fdm_->GetPropertyValue("position/from-start-neu-u-ft");
    state.position.x = (float)(e_ft * kFtToM);
    state.position.y = (float)(u_ft * kFtToM);
    state.position.z = (float)(-n_ft * kFtToM);

    const double vn_fps = fdm_->GetPropertyValue("velocities/v-north-fps");
    const double ve_fps = fdm_->GetPropertyValue("velocities/v-east-fps");
    const double vd_fps = fdm_->GetPropertyValue("velocities/v-down-fps");
    state.velocity.x = (float)(ve_fps * kFtToM);
    state.velocity.y = (float)(-vd_fps * kFtToM);
    state.velocity.z = (float)(-vn_fps * kFtToM);

    const double phi = fdm_->GetPropertyValue("attitude/phi-rad");
    const double theta = fdm_->GetPropertyValue("attitude/theta-rad");
    const double psi = fdm_->GetPropertyValue("attitude/psi-rad");
    state.orientation = glm::normalize(glm::quat(glm::vec3((float)theta, (float)psi, (float)phi)));

    const double p = fdm_->GetPropertyValue("velocities/p-rad_sec");
    const double q = fdm_->GetPropertyValue("velocities/q-rad_sec");
    const double r = fdm_->GetPropertyValue("velocities/r-rad_sec");
    state.angular_vel.x = (float)q;
    state.angular_vel.y = (float)r;
    state.angular_vel.z = (float)p;
}
