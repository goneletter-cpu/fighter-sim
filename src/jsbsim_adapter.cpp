#include "jsbsim_adapter.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 3.28083989501312;

}

/*========================================================*/

JSBSimAdapter::JSBSimAdapter()
    : fdm_(nullptr),
      initialized_(false)
{}

JSBSimAdapter::~JSBSimAdapter()
{
    delete fdm_;
}

/*========================================================*/
/*                 DEBUG PROPERTY DUMP                    */
/*========================================================*/

void JSBSimAdapter::dump_engine_state()
{
    double rpm =
        fdm_->GetPropertyValue("propulsion/engine[0]/engine-rpm");

    double throttle =
        fdm_->GetPropertyValue("fcs/throttle-cmd-norm");

    double mixture =
        fdm_->GetPropertyValue("fcs/mixture-cmd-norm");

    double mags =
        fdm_->GetPropertyValue("controls/switches/magnetos");

    double fuel_flow =
        fdm_->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");

    double running =
        fdm_->GetPropertyValue("propulsion/engine[0]/running");

    double thrust =
        fdm_->GetPropertyValue("propulsion/engine[0]/thrust-lbs");

    double fuel =
        fdm_->GetPropertyValue("propulsion/tank[0]/contents-lbs");

    std::cout
    << std::fixed << std::setprecision(2)
    << "[ENGINE] "
    << "RPM=" << rpm
    << "  Thr=" << throttle
    << "  Mix=" << mixture
    << "  Mag=" << mags
    << "  FuelFlow=" << fuel_flow
    << "  Run=" << running
    << "  Thrust=" << thrust
    << "  Tank=" << fuel
    << std::endl;
}

/*========================================================*/
/*                       INIT                             */
/*========================================================*/

bool JSBSimAdapter::init(
    const std::string&,
    const AircraftState& initial_state,
    float dt)
{
    std::cout << "\n================ JSBSim INIT ================\n";

    if (!fdm_)
    {
        std::cout << "[JSBSim] Creating FGFDMExec\n";
        fdm_ = new JSBSim::FGFDMExec();
    }

    std::string root =
    std::string(FIGHTER_SIM_ROOT) 
    +"/jsbsim";

    std::cout << "[JSBSim] RootDir = " << root << std::endl;

    fdm_->SetRootDir(SGPath(root));
    fdm_->SetAircraftPath(SGPath("aircraft"));

    std::cout << "[JSBSim] AircraftPath = aircraft\n";
    std::cout << "[JSBSim] Loading model\n";

    if (!fdm_->LoadModel("ball", true))
    //选择模型并处理错误
    {
        std::cout << "[JSBSim] ERROR: LoadModel failed\n";
        return false;
    }

    std::cout << "[JSBSim] Model loaded\n";

    fdm_->Setdt(dt);

    const glm::vec3 pos_m = initial_state.position;
    const glm::vec3 vel_mps = initial_state.velocity;
    const glm::vec3 euler = initial_state.euler_angles();

    auto ic = fdm_->GetIC();

    std::cout << "[JSBSim] Setting initial conditions\n";

    ic->SetLatitudeDegIC(0.0);
    ic->SetLongitudeDegIC(0.0);

    ic->SetAltitudeASLFtIC(pos_m.y * kMToFt);

    ic->SetVNorthFpsIC(-vel_mps.z * kMToFt);
    ic->SetVEastFpsIC( vel_mps.x * kMToFt);
    ic->SetVDownFpsIC(-vel_mps.y * kMToFt);

    ic->SetPhiRadIC(euler.z);
    ic->SetThetaRadIC(euler.x);
    ic->SetPsiRadIC(euler.y);

    ic->SetPRadpsIC(0.0);
    ic->SetQRadpsIC(0.0);
    ic->SetRRadpsIC(0.0);

    ic->SetWindNEDFpsIC(0,0,0);
    ic->SetTerrainElevationFtIC(0);

    std::cout << "[JSBSim] Running IC...\n";

    initialized_ = fdm_->RunIC();

    std::cout << "[JSBSim] RunIC result = "
              << initialized_
              << std::endl;

    if (!initialized_)
        return false;

    /*--------------------------------------------------*/
    /*               ENGINE START SEQUENCE              */
    /*--------------------------------------------------*/

    std::cout << "[JSBSim] Starting engine sequence\n";

    fdm_->SetPropertyValue(
        "controls/switches/magnetos", 3);

    fdm_->SetPropertyValue(
        "fcs/mixture-cmd-norm", 1);

    fdm_->SetPropertyValue(
        "fcs/throttle-cmd-norm", 0.2);

    fdm_->SetPropertyValue(
        "propulsion/engine[0]/set-running", 1);

    /*--------------------------------------------------*/
    /*                     WARMUP                       */
    /*--------------------------------------------------*/

    std::cout << "\n[JSBSim] Warmup phase\n";

    for (int i = 0; i < 120; ++i)
    {
        fdm_->Run();

        if (i % 10 == 0)
        {
            std::cout << "[JSBSim] step " << i << " ";
            dump_engine_state();
        }
    }

    std::cout << "\n[JSBSim] Final engine state\n";
    dump_engine_state();

    std::cout << "=============================================\n\n";

    return true;
}

/*========================================================*/
/*                     CONTROLS                           */
/*========================================================*/

void JSBSimAdapter::set_controls(const ControlInput& ctrl)
{
    if (!initialized_) return;

    fdm_->SetPropertyValue(
        "fcs/aileron-cmd-norm",
        std::clamp((double)ctrl.aileron,-1.0,1.0));

    fdm_->SetPropertyValue(
        "fcs/elevator-cmd-norm",
        std::clamp((double)ctrl.elevator,-1.0,1.0));

    fdm_->SetPropertyValue(
        "fcs/rudder-cmd-norm",
        std::clamp((double)ctrl.rudder,-1.0,1.0));

    double thr =
        std::clamp((double)ctrl.throttle,0.0,1.0);

    fdm_->SetPropertyValue("fcs/throttle-cmd-norm",thr);
}

/*========================================================*/
/*                        STEP                            */
/*========================================================*/

bool JSBSimAdapter::step(float dt)
{
    if (!initialized_)
        return false;

    fdm_->Setdt(dt);

    bool ok = fdm_->Run();

    dump_engine_state();

    return ok;
}

/*========================================================*/
/*                    SYNC STATE                          */
/*========================================================*/

void JSBSimAdapter::sync_state(AircraftState& state) const
{
    if (!initialized_) return;

    const double n_ft =
        fdm_->GetPropertyValue("position/from-start-neu-n-ft");

    const double e_ft =
        fdm_->GetPropertyValue("position/from-start-neu-e-ft");

    const double u_ft =
        fdm_->GetPropertyValue("position/from-start-neu-u-ft");

    state.position.x = (float)( e_ft * kFtToM);
    state.position.y = (float)( u_ft * kFtToM);
    state.position.z = (float)(-n_ft * kFtToM);
}

/*========================================================*/

double JSBSimAdapter::get_property(
    const std::string& name) const
{
    if (!initialized_) return 0.0;

    return fdm_->GetPropertyValue(name);
}
void JSBSimAdapter::set_aux_controls(float flap_norm, float gear_norm)
{
    if (!initialized_) return;

    fdm_->SetPropertyValue("fcs/flap-cmd-norm", flap_norm);
    fdm_->SetPropertyValue("gear/gear-cmd-norm", gear_norm);
}