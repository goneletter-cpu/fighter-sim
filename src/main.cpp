#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iomanip>
#include <vector>

#include "physics.h"
#include "pid.h"
#include "flight_controller.h"
#include "jsbsim_adapter.h"
#include "renderer.h"

// ──  translated comment

#ifndef ENABLE_CSV_LOG
#define ENABLE_CSV_LOG 0
#endif

static const int   WIDTH  = 1280;
static const int   HEIGHT = 720;
static const float SIM_DT = 1.0f / 120.0f;   //  translated comment
static const glm::vec3 START_POS(0.0f, 1000.0f, 0.0f);
[[maybe_unused]] static const glm::vec3 FIXED_WAYPOINT(6000.0f, 1800.0f, -8000.0f); // x/y/z translated comment
static const bool kEnableCombat = false;
static const bool kUseAutopilot = false;
static const char* kScriptPath = "config/flight_script.txt";

// ──  translated comment

struct KeyState {
    bool pitch_up   = false;
    bool pitch_down = false;
    bool roll_left  = false;
    bool roll_right = false;
    bool yaw_left   = false;
    bool yaw_right  = false;
    bool speed_up   = false;
    bool speed_down = false;
    bool trim_up    = false;
    bool trim_down  = false;
    bool trim_left  = false;
    bool trim_right = false;
    bool fire_held = false;
    bool fire_released = false;
    double fire_press_time = 0.0;
    double fire_release_duration = 0.0;
};

static KeyState g_keys;
static float g_game_speed_scale = 1.0f;
static const float LOW_ALT_MIN_Y = 55.0f;
static const float LOW_ALT_MAX_Y = 820.0f;
static const float TRIM_WHEEL_MAX_V = 120.0f;
static glm::vec3 g_trim_origin = START_POS;
static glm::vec2 g_trim_wheel_vel(0.0f, 0.0f);
static float g_throttle_cmd = 0.65f;

static float slew_to(float current, float target, float max_step) {
    float delta = glm::clamp(target - current, -max_step, max_step);
    return current + delta;
}

static ControlInput build_direct_surface_controls(float dt, float throttle_cmd) {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    if (g_keys.pitch_up)   pitch += 1.0f;
    if (g_keys.pitch_down) pitch -= 1.0f;
    if (g_keys.roll_right) roll += 1.0f;
    if (g_keys.roll_left)  roll -= 1.0f;
    if (g_keys.yaw_right)  yaw += 1.0f;
    if (g_keys.yaw_left)   yaw -= 1.0f;

    static ControlInput smooth = {};
    const float rate = 2.8f; // per second
    float step = rate * dt;

    float target_elev = glm::clamp(pitch * 0.7f, -1.0f, 1.0f);
    float target_ail  = glm::clamp(roll  * 0.8f, -1.0f, 1.0f);
    float target_rud  = glm::clamp(yaw   * 0.6f, -1.0f, 1.0f);

    smooth.elevator = slew_to(smooth.elevator, target_elev, step);
    smooth.aileron  = slew_to(smooth.aileron,  target_ail,  step);
    smooth.rudder   = slew_to(smooth.rudder,   target_rud,  step);
    smooth.throttle = glm::clamp(throttle_cmd, 0.0f, 1.0f);
    return smooth;
}

static ControlInput build_autopilot_controls(float throttle_cmd) {
    ControlInput ctrl{};
    ctrl.elevator = 0.0f;
    ctrl.aileron = 0.0f;
    ctrl.rudder = 0.0f;
    ctrl.throttle = glm::clamp(throttle_cmd, 0.0f, 1.0f);
    return ctrl;
}

static float smooth_step(float e0, float e1, float x) {
    float t = glm::clamp((x - e0) / glm::max(e1 - e0, 1e-6f), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

[[maybe_unused]] static void update_throttle_cmd(float dt) {
    const float rate = 0.35f;
    float dir = 0.0f;
    if (g_keys.speed_up) dir += 1.0f;
    if (g_keys.speed_down) dir -= 1.0f;
    g_throttle_cmd = glm::clamp(g_throttle_cmd + dir * rate * dt, 0.0f, 1.0f);
}

struct Phase {
    std::string name;
    float duration_s = 5.0f;
    float throttle = 0.6f;
    float elevator = 0.0f;
    float aileron = 0.0f;
    float rudder = 0.0f;
    float flap = 0.0f;
    float gear = 1.0f;
    float target_speed_mps = 60.0f;
    float target_climb_mps = 0.0f;
    float target_heading_deg = 0.0f;
    bool has_targets = false;
};

struct ScriptConfig {
    std::string run_mode = "single"; // single | batch
    //机型
    std::string model = "ball";
    std::vector<std::string> models;
    float initial_alt_m = 1000.0f;
    float initial_speed_mps = 55.0f;
    std::vector<Phase> phases;
    bool use_pid = false;
};

static std::string trim_copy(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        std::string t = trim_copy(item);
        if (!t.empty()) out.push_back(t);
    }
    return out;
}

static std::vector<std::string> split_ws(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (ss >> item) {
        out.push_back(item);
    }
    return out;
}

static std::string lower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

static float wrap_angle_deg(float deg) {
    float r = std::fmod(deg, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}

static float shortest_angle_deg(float target_deg, float current_deg) {
    float t = wrap_angle_deg(target_deg);
    float c = wrap_angle_deg(current_deg);
    float d = t - c;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

static ScriptConfig load_script(const std::string& path) {
    ScriptConfig cfg;
    std::ifstream in(path);
    if (!in.is_open()) {
        return cfg;
    }

    std::string line;
    std::vector<std::string> columns;
    bool have_columns = false;

    while (std::getline(in, line)) {
        std::string t = trim_copy(line);
        if (t.empty() || t[0] == '#') continue;

        auto parse_kv = [&](const std::string& key, const std::string& val) {
            std::string k = lower_copy(key);
            if (k == "models") {
                cfg.models = split_csv(val);
            } else if (k == "run_mode") {
                cfg.run_mode = val;
            } else if (k == "model") {
                cfg.model = val;
            } else if (k == "initial_alt_m") {
                cfg.initial_alt_m = std::stof(val);
            } else if (k == "initial_speed_mps") {
                cfg.initial_speed_mps = std::stof(val);
            }
        };

        size_t colon = t.find(':');
        if (colon != std::string::npos) {
            std::string key = trim_copy(t.substr(0, colon));
            std::string val = trim_copy(t.substr(colon + 1));
            if (lower_copy(key) == "columns") {
                columns = split_csv(val);
                if (columns.empty()) columns = split_ws(val);
                for (auto& c : columns) c = lower_copy(c);
                have_columns = !columns.empty();
            } else {
                parse_kv(key, val);
            }
            continue;
        }

        // Support "key value" directives (non-YAML).
        {
            auto parts = split_ws(t);
            if (parts.size() >= 2) {
                parse_kv(parts[0], parts[1]);
                continue;
            }
        }

        // Data lines: CSV or whitespace values.
        std::vector<std::string> vals = split_csv(t);
        if (vals.size() <= 1) vals = split_ws(t);
        if (vals.empty()) continue;

        Phase p{};
        if (!have_columns) {
            // Fixed order: duration_s, throttle, elevator, aileron, rudder, flap, gear
            if (vals.size() >= 1) p.duration_s = std::stof(vals[0]);
            if (vals.size() >= 2) p.throttle = std::stof(vals[1]);
            if (vals.size() >= 3) p.elevator = std::stof(vals[2]);
            if (vals.size() >= 4) p.aileron = std::stof(vals[3]);
            if (vals.size() >= 5) p.rudder = std::stof(vals[4]);
            if (vals.size() >= 6) p.flap = std::stof(vals[5]);
            if (vals.size() >= 7) p.gear = std::stof(vals[6]);
            cfg.phases.push_back(p);
            continue;
        }

        for (size_t i = 0; i < vals.size() && i < columns.size(); ++i) {
            const std::string& c = columns[i];
            const std::string& v = vals[i];
            if (c == "duration_s") p.duration_s = std::stof(v);
            else if (c == "fcs/throttle-cmd-norm") p.throttle = std::stof(v);
            else if (c == "fcs/elevator-cmd-norm") p.elevator = std::stof(v);
            else if (c == "fcs/aileron-cmd-norm") p.aileron = std::stof(v);
            else if (c == "fcs/rudder-cmd-norm") p.rudder = std::stof(v);
            else if (c == "fcs/flap-cmd-norm") p.flap = std::stof(v);
            else if (c == "fcs/gear-cmd-norm") p.gear = std::stof(v);
            else if (c == "target_speed_mps") { p.target_speed_mps = std::stof(v); p.has_targets = true; }
            else if (c == "target_climb_mps") { p.target_climb_mps = std::stof(v); p.has_targets = true; }
            else if (c == "target_heading_deg") { p.target_heading_deg = std::stof(v); p.has_targets = true; }
        }
        if (p.has_targets) cfg.use_pid = true;
        cfg.phases.push_back(p);
    }
    return cfg;
}

static void write_default_script(const std::string& path) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return;
    out << "run_mode single\n";
    out << "model fgdata:c172p\n";
    out << "models fgdata:c172p\n";
    out << "initial_alt_m 0\n";
    out << "initial_speed_mps 0\n";
    out << "columns: duration_s,target_speed_mps,target_climb_mps,target_heading_deg,fcs/flap-cmd-norm,fcs/gear-cmd-norm\n";
    out << "12,30,0,0,0.2,1.0\n";
    out << "18,45,2.0,0,0.0,1.0\n";
    out << "60,60,3.0,0,0.0,1.0\n";
    out << "120,70,0.5,0,0.0,1.0\n";
    out << "80,55,-1.0,0,0.1,1.0\n";
    out << "60,50,-0.8,0,0.3,1.0\n";
    out << "20,40,-0.5,0,0.4,1.0\n";
}

static WireMesh make_quadrant_overlay_mesh(float z_plane, float ring_scale, float ring_pulse) {
    WireMesh m;
    auto add_seg = [&](const glm::vec3& a, const glm::vec3& b) {
        unsigned int i = (unsigned int)m.vertices.size();
        m.vertices.push_back(a);
        m.vertices.push_back(b);
        m.line_indices.push_back(i);
        m.line_indices.push_back(i + 1);
    };
    // Mouse-aim style ring (War Thunder-like): outer + inner circle + top notch.
    const int seg = 40;
    const float r0 = (36.0f * ring_scale) * (1.0f + ring_pulse);
    const float r1 = (30.0f * ring_scale) * (1.0f + ring_pulse * 0.85f);
    for (int i = 0; i < seg; ++i) {
        float a0 = (2.0f * glm::pi<float>() * i) / seg;
        float a1 = (2.0f * glm::pi<float>() * (i + 1)) / seg;
        add_seg({std::cos(a0) * r0, std::sin(a0) * r0, z_plane},
                {std::cos(a1) * r0, std::sin(a1) * r0, z_plane});
        add_seg({std::cos(a0) * r1, std::sin(a0) * r1, z_plane},
                {std::cos(a1) * r1, std::sin(a1) * r1, z_plane});
    }
    // Top notch.
    add_seg({-4.0f, r0 + 6.0f, z_plane}, {4.0f, r0 + 6.0f, z_plane});
    // Small center dot.
    add_seg({-3.0f, 0.0f, z_plane}, {3.0f, 0.0f, z_plane});
    add_seg({0.0f, -3.0f, z_plane}, {0.0f, 3.0f, z_plane});
    return m;
}

static void framebuffer_size_callback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

static void key_callback(GLFWwindow* win, int key, int, int action, int) {
    bool pressed = (action != GLFW_RELEASE);
    switch (key) {
        case GLFW_KEY_W:         g_keys.pitch_up      = pressed; break;
        case GLFW_KEY_S:         g_keys.pitch_down    = pressed; break;
        case GLFW_KEY_A:         g_keys.roll_left     = pressed; break;
        case GLFW_KEY_D:         g_keys.roll_right    = pressed; break;
        case GLFW_KEY_Q:         g_keys.yaw_left      = pressed; break;
        case GLFW_KEY_E:         g_keys.yaw_right     = pressed; break;
        case GLFW_KEY_LEFT_SHIFT:
        case GLFW_KEY_RIGHT_SHIFT:   g_keys.speed_up   = pressed; break;
        case GLFW_KEY_LEFT_CONTROL:
        case GLFW_KEY_RIGHT_CONTROL: g_keys.speed_down = pressed; break;
        case GLFW_KEY_UP:
        case GLFW_KEY_I:             g_keys.trim_up    = pressed; break;
        case GLFW_KEY_DOWN:
        case GLFW_KEY_K:             g_keys.trim_down  = pressed; break;
        case GLFW_KEY_LEFT:
        case GLFW_KEY_J:             g_keys.trim_left  = pressed; break;
        case GLFW_KEY_RIGHT:
        case GLFW_KEY_L:             g_keys.trim_right = pressed; break;
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS) {
                g_keys.fire_held = true;
                g_keys.fire_press_time = glfwGetTime();
            } else if (action == GLFW_RELEASE) {
                g_keys.fire_held = false;
                g_keys.fire_released = true;
                g_keys.fire_release_duration = glfwGetTime() - g_keys.fire_press_time;
            }
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(win, true); break;
        default: break;
    }
}

//  translated comment
// base_cmd  translated comment
[[maybe_unused]] static void apply_player_intervention(AttitudeCommand& cmd,
                                      const AttitudeCommand& base_cmd,
                                      float dt) {
    const float PITCH_DELTA = glm::radians(6.0f);   // rad/s
    const float ROLL_DELTA  = glm::radians(10.0f);
    const float YAW_DELTA   = glm::radians(5.0f);

    if (g_keys.pitch_up)    cmd.pitch_rad += PITCH_DELTA * dt;
    if (g_keys.pitch_down)  cmd.pitch_rad -= PITCH_DELTA * dt;
    if (g_keys.roll_left)   cmd.roll_rad  += ROLL_DELTA  * dt;
    if (g_keys.roll_right)  cmd.roll_rad  -= ROLL_DELTA  * dt;
    if (g_keys.yaw_left)    cmd.yaw_rate_rad_s += YAW_DELTA * dt;
    if (g_keys.yaw_right)   cmd.yaw_rate_rad_s -= YAW_DELTA * dt;

    //  translated comment
    if (!g_keys.pitch_up && !g_keys.pitch_down) {
        cmd.pitch_rad = slew_to(cmd.pitch_rad, base_cmd.pitch_rad, glm::radians(18.0f) * dt);
    }
    if (!g_keys.roll_left && !g_keys.roll_right) {
        cmd.roll_rad = slew_to(cmd.roll_rad, base_cmd.roll_rad, glm::radians(30.0f) * dt);
    }
    if (!g_keys.yaw_left && !g_keys.yaw_right) {
        cmd.yaw_rate_rad_s = slew_to(cmd.yaw_rate_rad_s, base_cmd.yaw_rate_rad_s, glm::radians(22.0f) * dt);
    }

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.yaw_rate_rad_s = glm::clamp(cmd.yaw_rate_rad_s, glm::radians(-16.0f), glm::radians(16.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
}

[[maybe_unused]] static void update_game_speed_scale(float dt) {
    const float kMinScale = 0.70f;
    const float kMaxScale = 1.60f;
    const float kAdjustPerSec = 0.45f;
    float dir = 0.0f;
    if (g_keys.speed_up) dir += 1.0f;
    if (g_keys.speed_down) dir -= 1.0f;
    g_game_speed_scale = glm::clamp(g_game_speed_scale + dir * kAdjustPerSec * dt, kMinScale, kMaxScale);
}

[[maybe_unused]] static void update_trim_origin(float dt, const AircraftState& state) {
    // Auto-moving trim target in world space.
    const float auto_forward = 85.0f * g_game_speed_scale;
    const float auto_sway = 40.0f;
    const float auto_heave = 28.0f;
    g_trim_origin.z -= auto_forward * dt;
    g_trim_origin.x = std::sin((float)glfwGetTime() * 0.35f) * auto_sway;
    g_trim_origin.y = glm::clamp(
        state.position.y + std::sin((float)glfwGetTime() * 0.42f) * auto_heave,
        LOW_ALT_MIN_Y + 120.0f, LOW_ALT_MAX_Y - 120.0f
    );
    g_trim_origin.y = glm::clamp(g_trim_origin.y, LOW_ALT_MIN_Y + 120.0f, LOW_ALT_MAX_Y - 120.0f);
}

//  translated comment
// Real physics helper (currently disabled in side-scroller mode).
// Kept intentionally for future full-physics expansion.
[[maybe_unused]] static void apply_direct_player_attitude_control(AircraftState& state, float dt) {
    float pitch_input = 0.0f;
    float roll_input = 0.0f;
    float yaw_input = 0.0f;

    if (g_keys.pitch_up)   pitch_input += 1.0f;
    if (g_keys.pitch_down) pitch_input -= 1.0f;
    if (g_keys.roll_left)  roll_input  += 1.0f;
    if (g_keys.roll_right) roll_input  -= 1.0f;
    if (g_keys.yaw_left)   yaw_input   += 1.0f;
    if (g_keys.yaw_right)  yaw_input   -= 1.0f;

    //  translated comment
    glm::vec3 input_accel(
        pitch_input * glm::radians(165.0f), // x: pitch accel
        yaw_input   * glm::radians(130.0f), // y: yaw accel
        roll_input  * glm::radians(240.0f)  // z: roll accel
    );

    //  translated comment
    glm::vec3 coupled_accel(
        -0.18f * state.angular_vel.y * std::abs(state.angular_vel.z),
        -0.28f * state.angular_vel.z * (0.7f + 0.3f * std::abs(roll_input)),
         0.10f * state.angular_vel.x * state.angular_vel.y
    );

    state.angular_vel += (input_accel + coupled_accel) * dt;

    //  translated comment
    glm::vec3 damping(1.8f, 2.2f, 2.0f);
    state.angular_vel.x -= state.angular_vel.x * damping.x * dt;
    state.angular_vel.y -= state.angular_vel.y * damping.y * dt;
    state.angular_vel.z -= state.angular_vel.z * damping.z * dt;

    //  translated comment
    state.angular_vel.x = glm::clamp(state.angular_vel.x, glm::radians(-130.0f), glm::radians(130.0f));
    state.angular_vel.y = glm::clamp(state.angular_vel.y, glm::radians(-120.0f), glm::radians(120.0f));
    state.angular_vel.z = glm::clamp(state.angular_vel.z, glm::radians(-170.0f), glm::radians(170.0f));
}

struct Projectile {
    glm::vec3 position;
    glm::vec3 velocity;
    float ttl;
};

struct EnemyAircraft {
    glm::vec3 position;
    glm::vec3 velocity;
    glm::quat orientation;
};

struct Cloud {
    glm::vec3 position;
};

struct AABattery {
    glm::vec3 position;
    float cooldown_s;
};

struct AAShell {
    glm::vec3 position;
    glm::vec3 velocity;
    float ttl_s;
};

struct Explosion {
    glm::vec3 position;
    float ttl_s;
    float max_ttl_s;
};

static WireMesh make_catapult_marks_mesh() {
    WireMesh m;
    //  translated comment
    m.vertices = {
        { 0.0f,  8.02f,  86.0f}, { 0.0f,  8.02f, -86.0f},   // center line
        {-9.0f,  8.02f,  76.0f}, {-9.0f,  8.02f, -72.0f},   // catapult L
        { 9.0f,  8.02f,  76.0f}, { 9.0f,  8.02f, -72.0f},   // catapult R
        {-18.0f, 8.02f, -72.0f}, {18.0f,  8.02f, -72.0f},   // launch bar
        {-14.0f, 8.02f,  28.0f}, {14.0f,  8.02f,  28.0f},   // landing mark
    };
    m.line_indices = {0,1, 2,3, 4,5, 6,7, 8,9};
    return m;
}

static WireMesh make_bomb_mesh() {
    WireMesh m;
    m.vertices = {
        { 0.0f,  0.8f,  0.0f},  // 0 top
        { 0.0f, -0.8f,  0.0f},  // 1 bottom
        { 0.6f,  0.0f,  0.0f},  // 2 right
        {-0.6f,  0.0f,  0.0f},  // 3 left
        { 0.0f,  0.0f,  0.6f},  // 4 back
        { 0.0f,  0.0f, -0.6f},  // 5 front
    };
    m.line_indices = {
        0,2, 0,3, 0,4, 0,5,
        1,2, 1,3, 1,4, 1,5,
        2,4, 4,3, 3,5, 5,2
    };
    return m;
}

static WireMesh make_bullet_mesh() {
    WireMesh m;
    m.vertices = {
        { 0.0f,  0.0f, -0.35f},
        { 0.0f,  0.0f,  0.35f},
        { 0.12f, 0.0f,  0.15f},
        {-0.12f, 0.0f,  0.15f},
        { 0.0f,  0.12f, 0.15f},
        { 0.0f, -0.12f, 0.15f},
    };
    m.line_indices = {
        0,1, 2,3, 4,5, 1,2, 1,3, 1,4, 1,5
    };
    return m;
}

static WireMesh make_cloud_mesh() {
    WireMesh m;
    const int seg = 14;
    const float r1 = 18.0f;
    const float r2 = 13.0f;
    const float r3 = 10.0f;

    int base1 = 0;
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        m.vertices.push_back({std::cos(t) * r1, std::sin(t) * r2, 0.0f});
    }
    int base2 = (int)m.vertices.size();
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        m.vertices.push_back({std::cos(t) * r2, 0.0f, std::sin(t) * r1});
    }
    int base3 = (int)m.vertices.size();
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        m.vertices.push_back({0.0f, std::cos(t) * r3, std::sin(t) * r2});
    }
    for (int i = 0; i < seg; ++i) {
        int j = (i + 1) % seg;
        m.line_indices.insert(m.line_indices.end(), {
            (unsigned int)(base1 + i), (unsigned int)(base1 + j),
            (unsigned int)(base2 + i), (unsigned int)(base2 + j),
            (unsigned int)(base3 + i), (unsigned int)(base3 + j)
        });
    }
    return m;
}

static WireMesh make_explosion_mesh() {
    WireMesh m;
    const int seg = 18;
    const float rxy = 5.8f;
    const float rz = 4.8f;
    int b1 = 0;
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        float wobble = 0.82f + 0.24f * std::sin(3.0f * t);
        m.vertices.push_back({std::cos(t) * rxy * wobble, std::sin(t) * rxy * wobble, 0.0f});
    }
    int b2 = (int)m.vertices.size();
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        float wobble = 0.78f + 0.26f * std::cos(2.0f * t + 0.7f);
        m.vertices.push_back({std::cos(t) * rxy * wobble, 0.0f, std::sin(t) * rz * wobble});
    }
    int b3 = (int)m.vertices.size();
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        float wobble = 0.80f + 0.22f * std::sin(4.0f * t + 0.5f);
        m.vertices.push_back({0.0f, std::cos(t) * rxy * 0.85f * wobble, std::sin(t) * rz * wobble});
    }
    for (int i = 0; i < seg; ++i) {
        int j = (i + 1) % seg;
        m.line_indices.insert(m.line_indices.end(), {
            (unsigned int)(b1 + i), (unsigned int)(b1 + j),
            (unsigned int)(b2 + i), (unsigned int)(b2 + j),
            (unsigned int)(b3 + i), (unsigned int)(b3 + j),
            (unsigned int)(b1 + i), (unsigned int)(b2 + i)
        });
    }
    return m;
}

static WireMesh make_roundel_mesh() {
    WireMesh m;
    const int seg = 28;
    const float r = 1.0f;
    for (int i = 0; i < seg; ++i) {
        float t = (2.0f * glm::pi<float>() * i) / seg;
        m.vertices.push_back({std::cos(t) * r, 0.0f, std::sin(t) * r});
    }
    for (int i = 0; i < seg; ++i) {
        int j = (i + 1) % seg;
        m.line_indices.push_back((unsigned int)i);
        m.line_indices.push_back((unsigned int)j);
    }
    return m;
}

static glm::quat orientation_from_forward(const glm::vec3& forward_world) {
    glm::vec3 f = glm::normalize(forward_world);
    glm::vec3 up_ref(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::cross(up_ref, -f);
    if (glm::length(right) < 1e-4f) right = glm::vec3(1.0f, 0.0f, 0.0f);
    right = glm::normalize(right);
    glm::vec3 up = glm::normalize(glm::cross(-f, right));
    glm::mat3 basis(right, up, -f); // local -Z  translated comment
    return glm::normalize(glm::quat_cast(basis));
}

static void respawn_enemy(EnemyAircraft& enemy,
                          const AircraftState& player,
                          int idx,
                          float t_now,
                          float game_speed_scale) {
    float seed = (float)idx * 1.618f + t_now * 0.21f;
    float az = glm::two_pi<float>() * (0.5f + 0.5f * std::sin(seed * 2.1f));
    float radius = 1500.0f + 1400.0f * (0.5f + 0.5f * std::cos(seed * 1.37f));
    float h = player.position.y + (-120.0f + 240.0f * (0.5f + 0.5f * std::sin(seed * 0.87f)));

    enemy.position = player.position + glm::vec3(std::cos(az) * radius, 0.0f, std::sin(az) * radius);
    enemy.position.y = glm::clamp(h, LOW_ALT_MIN_Y + 40.0f, LOW_ALT_MAX_Y - 25.0f);

    glm::vec3 to_player = player.position - enemy.position;
    glm::vec3 tangent(-to_player.z, 0.0f, to_player.x);
    if (glm::length(tangent) > 1e-4f) tangent = glm::normalize(tangent);
    float curve = std::sin(seed * 1.7f) * 0.28f;
    glm::vec3 dir = glm::normalize(to_player + tangent * curve * glm::length(to_player));
    enemy.velocity = dir * (58.0f + 4.0f * (idx % 3)) * game_speed_scale;
    enemy.orientation = orientation_from_forward(enemy.velocity);
}

// Real physics helper (currently disabled in side-scroller mode).
// Kept intentionally for future full-physics expansion.
[[maybe_unused]] static void keep_forward_flight(AircraftState& state, float dt) {
    glm::mat3 world_from_body = glm::mat3_cast(state.orientation);
    glm::vec3 fwd = glm::normalize(world_from_body * glm::vec3(0, 0, -1));
    glm::vec3 up = glm::normalize(world_from_body * glm::vec3(0, 1, 0));
    glm::vec3 right = glm::normalize(world_from_body * glm::vec3(1, 0, 0));
    glm::mat3 body_from_world = glm::transpose(world_from_body);
    glm::vec3 v_body = body_from_world * state.velocity;

    float forward_speed = -v_body.z; // local -Z  translated comment
    float target_speed = 185.0f;
    float thrust_accel = (target_speed - forward_speed) * 1.6f;
    v_body.z -= thrust_accel * dt;

    //  translated comment
    v_body.x += (-2.4f * v_body.x) * dt;
    v_body.y += (-1.7f * v_body.y) * dt;

    //  translated comment
    glm::vec3 horiz_lift(up.x, 0.0f, up.z);
    float hl = glm::length(horiz_lift);
    if (hl > 1e-4f) {
        horiz_lift /= hl;
        float speed_factor = glm::clamp(forward_speed / 190.0f, 0.35f, 1.25f);
        float turn_accel = 20.0f * hl * speed_factor;
        state.velocity += horiz_lift * turn_accel * dt;

        //  translated comment
        float bank_sign = glm::dot(right, glm::vec3(0, 1, 0));
        state.angular_vel.y += -bank_sign * glm::radians(22.0f) * speed_factor * dt;
    }

    //  translated comment
    float pitch_lift = glm::clamp(fwd.y, -0.6f, 0.7f);
    state.velocity.y += pitch_lift * 14.0f * dt;

    //  translated comment
    glm::vec3 world_from_body_v = world_from_body * v_body;
    state.velocity.x = 0.65f * state.velocity.x + 0.35f * world_from_body_v.x;
    state.velocity.y = 0.65f * state.velocity.y + 0.35f * world_from_body_v.y;
    state.velocity.z = 0.65f * state.velocity.z + 0.35f * world_from_body_v.z;
}

//  translated comment
[[maybe_unused]] static void apply_constrained_3d_motion(AircraftState& state, float dt) {
    // Restore 2.5D feel: motion constrained in a trim-centered XY window.
    float axis_x = 0.0f;
    float axis_y = 0.0f;
    float ad = 0.0f;
    if (g_keys.yaw_left)   axis_x -= 1.0f;  // Q
    if (g_keys.yaw_right)  axis_x += 1.0f;  // E
    if (g_keys.pitch_up)   axis_y += 1.0f;  // W
    if (g_keys.pitch_down) axis_y -= 1.0f;  // S
    if (g_keys.roll_left)  ad -= 1.0f;      // A
    if (g_keys.roll_right) ad += 1.0f;      // D

    float input_x = glm::clamp(axis_x + ad * 0.85f, -1.0f, 1.0f);
    float input_y = glm::clamp(axis_y + ad * 0.30f, -1.0f, 1.0f);
    float scroll_speed = 138.0f * g_game_speed_scale;
    float side_speed = 120.0f * g_game_speed_scale;
    float vertical_speed = 100.0f * g_game_speed_scale;

    glm::vec3 to_origin = g_trim_origin - state.position;
    float chase_vx = glm::clamp(to_origin.x * 0.35f, -side_speed, side_speed);
    float chase_vy = glm::clamp(to_origin.y * 0.32f, -vertical_speed, vertical_speed);
    float target_vx = chase_vx + input_x * side_speed * 0.85f;
    float target_vy = chase_vy + input_y * vertical_speed * 0.85f;
    target_vx = glm::clamp(target_vx, -side_speed, side_speed);
    target_vy = glm::clamp(target_vy, -vertical_speed, vertical_speed);
    float blend = 1.0f - std::exp(-12.0f * dt);
    state.velocity.x = glm::mix(state.velocity.x, target_vx, blend);
    state.velocity.y = glm::mix(state.velocity.y, target_vy, blend);
    state.velocity.z = -scroll_speed;
    state.position += state.velocity * dt;

    // No quadrant boundary anymore; keep only low-altitude envelope.
    state.position.y = glm::clamp(state.position.y, LOW_ALT_MIN_Y, LOW_ALT_MAX_Y);

    // Aircraft points back toward trim origin on XY plane.
    float target_yaw = std::atan2(to_origin.x, -state.velocity.z);
    float target_pitch = glm::clamp(std::atan2(to_origin.y, glm::max(std::abs(state.velocity.z), 1.0f)), glm::radians(-20.0f), glm::radians(20.0f));
    float target_roll = glm::radians(-32.0f) * ad;
    glm::quat target_q = glm::quat(glm::vec3(target_pitch, target_yaw, target_roll));
    float s = 1.0f - std::exp(-8.0f * dt);
    state.orientation = glm::normalize(glm::slerp(state.orientation, target_q, s));
    state.angular_vel *= std::exp(-4.0f * dt);

}

static void update_enemies(std::vector<EnemyAircraft>& enemies,
                           const AircraftState& player,
                           float dt,
                           float t_now,
                           float game_speed_scale) {
    for (size_t i = 0; i < enemies.size(); ++i) {
        EnemyAircraft& e = enemies[i];
        float desired_speed = (58.0f + 4.0f * (i % 3)) * game_speed_scale;
        float v_len = glm::length(e.velocity);
        if (v_len > 1e-4f) {
            e.velocity = (e.velocity / v_len) * desired_speed;
        } else {
            glm::vec3 to_player = player.position - e.position;
            float d = glm::length(to_player);
            e.velocity = (d > 1e-4f) ? (to_player / d) * desired_speed : glm::vec3(0.0f, 0.0f, -desired_speed);
        }
        e.position += e.velocity * dt;
        e.position.y = glm::clamp(e.position.y, LOW_ALT_MIN_Y + 25.0f, LOW_ALT_MAX_Y - 15.0f);
        e.orientation = orientation_from_forward(e.velocity);

        glm::vec3 rel = e.position - player.position;
        if (glm::length(rel) > 6200.0f ||
            e.position.y < LOW_ALT_MIN_Y + 20.0f ||
            e.position.y > LOW_ALT_MAX_Y - 10.0f) {
            respawn_enemy(e, player, (int)i, t_now, game_speed_scale);
        }
    }
}

static void update_clouds(std::vector<Cloud>& clouds,
                          const AircraftState& player,
                          float t_now) {
    glm::vec3 player_fwd = glm::normalize(glm::mat3_cast(player.orientation) * glm::vec3(0, 0, -1));
    glm::vec3 player_right = glm::normalize(glm::mat3_cast(player.orientation) * glm::vec3(1, 0, 0));
    for (size_t i = 0; i < clouds.size(); ++i) {
        glm::vec3 rel = clouds[i].position - player.position;
        if (glm::dot(rel, player_fwd) < -350.0f) {
            float ahead = 1800.0f + 220.0f * (i % 6);
            float side = ((int)(i % 7) - 3) * 220.0f + 90.0f * std::sin(t_now * 0.4f + (float)i);
            float h = 900.0f + 220.0f * (i % 5);
            clouds[i].position = player.position + player_fwd * ahead + player_right * side + glm::vec3(0, h, 0);
        }
    }
}

static void update_ground_aa(std::vector<AABattery>& batteries,
                             std::vector<AAShell>& shells,
                             const AircraftState& player,
                             float dt) {
    for (size_t i = 0; i < batteries.size(); ++i) {
        AABattery& b = batteries[i];
        b.cooldown_s -= dt;
        if (b.cooldown_s <= 0.0f) {
            glm::vec3 target = player.position + player.velocity * 1.8f;
            glm::vec3 dir = target - b.position;
            if (dir.y < 80.0f) dir.y = 80.0f;
            dir = glm::normalize(dir);

            AAShell s;
            s.position = b.position + glm::vec3(0, 2.0f, 0);
            s.velocity = dir * 220.0f;
            s.ttl_s = 8.0f;
            shells.push_back(s);

            b.cooldown_s = 0.7f + 0.35f * (float)((i % 4));
        }
    }

    for (auto& s : shells) {
        s.velocity += glm::vec3(0, -9.81f, 0) * dt;
        s.position += s.velocity * dt;
        s.ttl_s -= dt;
    }
    shells.erase(std::remove_if(shells.begin(), shells.end(),
                 [](const AAShell& s) { return s.ttl_s <= 0.0f || s.position.y < 0.0f; }),
                 shells.end());
}

static void spawn_bomb(const AircraftState& aircraft, std::vector<Projectile>& bombs) {
    glm::mat3 world_from_body = glm::mat3_cast(aircraft.orientation);
    glm::vec3 forward = world_from_body * glm::vec3(0, 0, -1);
    glm::vec3 right   = world_from_body * glm::vec3(1, 0, 0);

    Projectile b;
    b.position = aircraft.position + forward * 4.8f + right * 0.2f;
    b.velocity = aircraft.velocity + forward * 165.0f;
    b.ttl = 7.5f;
    bombs.push_back(b);
}

static void spawn_bullet(const AircraftState& aircraft,
                         std::vector<Projectile>& bullets,
                         int muzzle_index) {
    glm::mat3 world_from_body = glm::mat3_cast(aircraft.orientation);
    glm::vec3 forward = world_from_body * glm::vec3(0, 0, -1);
    glm::vec3 right   = world_from_body * glm::vec3(1, 0, 0);
    glm::vec3 up      = world_from_body * glm::vec3(0, 1, 0);
    float side = (muzzle_index % 2 == 0) ? -1.0f : 1.0f;

    Projectile p;
    p.position = aircraft.position + forward * 5.8f + right * (side * 1.3f) - up * 0.2f;
    p.velocity = aircraft.velocity + forward * 420.0f;
    p.ttl = 2.0f;
    bullets.push_back(p);
}

static void update_projectiles(std::vector<Projectile>& bullets,
                               std::vector<Projectile>& bombs,
                               float dt) {
    for (auto& b : bombs) {
        //  translated comment
        b.velocity += glm::vec3(0.0f, -1.3f, 0.0f) * dt;
        b.position += b.velocity * dt;
        b.ttl -= dt;
    }

    for (auto& p : bullets) {
        p.position += p.velocity * dt;
        p.ttl -= dt;
    }

    bullets.erase(std::remove_if(bullets.begin(), bullets.end(),
                  [](const Projectile& p) { return p.ttl <= 0.0f || p.position.y < 0.0f; }),
                  bullets.end());
    bombs.erase(std::remove_if(bombs.begin(), bombs.end(),
                [](const Projectile& b) { return b.ttl <= 0.0f || b.position.y < 0.0f; }),
                bombs.end());
}

static void update_explosions(std::vector<Explosion>& explosions, float dt) {
    for (auto& e : explosions) e.ttl_s -= dt;
    explosions.erase(std::remove_if(explosions.begin(), explosions.end(),
                     [](const Explosion& e) { return e.ttl_s <= 0.0f; }),
                     explosions.end());
}

static float wrap_pi(float a) {
    while (a > glm::pi<float>()) a -= glm::two_pi<float>();
    while (a < -glm::pi<float>()) a += glm::two_pi<float>();
    return a;
}

[[maybe_unused]] static AttitudeCommand build_combat_assist_command(const AircraftState& state,
                                                   const std::vector<EnemyAircraft>& enemies,
                                                   float dt,
                                                   const AttitudeCommand& prev_cmd) {
    AttitudeCommand cmd = prev_cmd;

    glm::vec3 forward = glm::normalize(glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1));
    float current_heading = std::atan2(forward.x, -forward.z);

    const EnemyAircraft* target = nullptr;
    float best_score = 1e9f;
    for (const auto& e : enemies) {
        glm::vec3 rel = e.position - state.position;
        float dist = glm::length(rel);
        if (dist < 1.0f) continue;
        float ahead = glm::dot(rel, forward);
        if (ahead < -300.0f) continue;
        float score = dist - 0.35f * ahead; //  translated comment
        if (score < best_score) {
            best_score = score;
            target = &e;
        }
    }

    float desired_pitch = 0.0f;
    float desired_roll = 0.0f;
    float desired_yaw_rate = 0.0f;
    float desired_throttle = 0.82f;

    if (target) {
        glm::vec3 to_target = target->position - state.position;
        float horiz = glm::length(glm::vec2(to_target.x, to_target.z));
        float desired_heading = std::atan2(to_target.x, -to_target.z);
        float heading_error = wrap_pi(desired_heading - current_heading);
        float pitch_to_target = std::atan2(to_target.y, glm::max(horiz, 1.0f));

        desired_pitch = glm::clamp(pitch_to_target * 0.85f - 0.08f * state.angular_vel.x,
                                   glm::radians(-16.0f), glm::radians(16.0f));
        desired_roll = glm::clamp(heading_error * 1.5f - 0.10f * state.angular_vel.z,
                                  glm::radians(-38.0f), glm::radians(38.0f));
        desired_yaw_rate = glm::clamp(heading_error * 0.75f - 0.18f * state.angular_vel.y,
                                      glm::radians(-16.0f), glm::radians(16.0f));
        float dist = glm::length(to_target);
        desired_throttle = glm::clamp(0.78f + 0.00018f * dist, 0.72f, 0.95f);
    }

    cmd.pitch_rad = slew_to(cmd.pitch_rad, desired_pitch, glm::radians(24.0f) * dt);
    cmd.roll_rad = slew_to(cmd.roll_rad, desired_roll, glm::radians(42.0f) * dt);
    cmd.yaw_rate_rad_s = slew_to(cmd.yaw_rate_rad_s, desired_yaw_rate, glm::radians(46.0f) * dt);
    cmd.throttle = slew_to(cmd.throttle, desired_throttle, 0.35f * dt);

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.yaw_rate_rad_s = glm::clamp(cmd.yaw_rate_rad_s, glm::radians(-16.0f), glm::radians(16.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
    return cmd;
}

[[maybe_unused]] static float update_autopilot_command(const AircraftState& state,
                                                       const glm::vec3& waypoint,
                                                       float dt,
                                                       AttitudeCommand& cmd) {
    glm::vec3 to_target = waypoint - state.position;
    float distance = glm::length(to_target);

    //  translated comment
    glm::vec3 forward = glm::normalize(glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1));
    float speed = glm::length(state.velocity);

    float current_heading = std::atan2(forward.x, -forward.z);
    float desired_heading = std::atan2(to_target.x, -to_target.z);
    float heading_error = wrap_pi(desired_heading - current_heading);
    if (std::abs(heading_error) < glm::radians(1.2f)) {
        heading_error = 0.0f;  //  translated comment
    }

    float horiz_dist = glm::length(glm::vec2(to_target.x, to_target.z));
    float desired_pitch = std::atan2(to_target.y, glm::max(horiz_dist, 1.0f));
    desired_pitch = glm::clamp(desired_pitch, glm::radians(-12.0f), glm::radians(12.0f));

    float desired_roll = glm::clamp(heading_error * 1.05f,
                                    glm::radians(-28.0f), glm::radians(28.0f));
    //  translated comment
    desired_pitch += 0.10f * std::abs(desired_roll);
    //  translated comment
    desired_pitch -= 0.22f * state.angular_vel.x;  // pitch rate q
    desired_roll  -= 0.28f * state.angular_vel.z;  // roll rate p

    //  translated comment
    float align = glm::clamp(1.0f - std::abs(heading_error) / glm::radians(16.0f), 0.0f, 1.0f);
    desired_roll = glm::mix(desired_roll, 0.0f, 0.78f * align);

    desired_pitch = glm::clamp(desired_pitch, glm::radians(-14.0f), glm::radians(14.0f));
    desired_roll  = glm::clamp(desired_roll,  glm::radians(-28.0f), glm::radians(28.0f));

    float desired_yaw_rate = glm::clamp(heading_error * 0.34f,
                                        glm::radians(-8.0f), glm::radians(8.0f));
    desired_yaw_rate -= 0.20f * state.angular_vel.y; // yaw damping
    desired_yaw_rate = glm::clamp(desired_yaw_rate, glm::radians(-8.0f), glm::radians(8.0f));

    float target_speed = (distance > 1200.0f) ? 195.0f : 165.0f;
    float target_throttle = 0.58f + 0.005f * (target_speed - speed);
    target_throttle = glm::clamp(target_throttle, 0.40f, 0.92f);

    if (distance < 120.0f) {
        desired_pitch = 0.0f;
        desired_roll = 0.0f;
        desired_yaw_rate = 0.0f;
        target_throttle = 0.52f;
    }

    cmd.pitch_rad = slew_to(cmd.pitch_rad, desired_pitch, glm::radians(18.0f) * dt);
    cmd.roll_rad = slew_to(cmd.roll_rad, desired_roll, glm::radians(28.0f) * dt);
    cmd.yaw_rate_rad_s = slew_to(cmd.yaw_rate_rad_s, desired_yaw_rate, glm::radians(35.0f) * dt);
    cmd.throttle = slew_to(cmd.throttle, target_throttle, 0.25f * dt);

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
    return distance;
}

//  translated comment
[[maybe_unused]] static void apply_sink_guard(const AircraftState& state, float dt, AttitudeCommand& cmd) {
    float alt = state.position.y;
    float sink_rate = -state.velocity.y; // >0  translated comment

    //  translated comment
    float alt_factor = glm::clamp((650.0f - alt) / 450.0f, 0.0f, 1.0f);
    float sink_factor = glm::clamp((sink_rate - 10.0f) / 22.0f, 0.0f, 1.0f);
    float g = glm::max(alt_factor, sink_factor);

    if (g > 0.0f) {
        float min_pitch = glm::mix(glm::radians(4.0f), glm::radians(16.0f), g);
        float min_thr = glm::mix(0.70f, 1.00f, g);

        cmd.pitch_rad = glm::max(cmd.pitch_rad, min_pitch);
        cmd.throttle = glm::max(cmd.throttle, min_thr);
        cmd.roll_rad = slew_to(cmd.roll_rad, 0.0f, glm::radians(55.0f) * dt * (0.35f + g));
        cmd.yaw_rate_rad_s = slew_to(cmd.yaw_rate_rad_s, 0.0f, glm::radians(40.0f) * dt * (0.35f + g));
    }

    //  translated comment
    if (alt < 180.0f || sink_rate > 42.0f) {
        cmd.pitch_rad = glm::radians(18.0f);
        cmd.roll_rad = slew_to(cmd.roll_rad, 0.0f, glm::radians(90.0f) * dt);
        cmd.yaw_rate_rad_s = slew_to(cmd.yaw_rate_rad_s, 0.0f, glm::radians(70.0f) * dt);
        cmd.throttle = 1.0f;
    }

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.yaw_rate_rad_s = glm::clamp(cmd.yaw_rate_rad_s, glm::radians(-16.0f), glm::radians(16.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
}

//  translated comment
[[maybe_unused]] static void apply_hard_safety_floor(AircraftState& state, float dt) {
    float alt = state.position.y;

    if (alt < 520.0f) {
        float g = glm::clamp((520.0f - alt) / 420.0f, 0.0f, 1.0f);

        //  translated comment
        float min_vy = glm::mix(-18.0f, -1.8f, g);
        if (state.velocity.y < min_vy) {
            state.velocity.y = slew_to(state.velocity.y, min_vy, 62.0f * dt);
        }

        //  translated comment
        float damp = std::exp(-3.6f * g * dt);
        state.angular_vel.x *= damp;
        state.angular_vel.z *= damp;
    }

    //  translated comment
    if (alt < 140.0f && state.velocity.y < 0.0f) {
        state.velocity.y = slew_to(state.velocity.y, 0.0f, 95.0f * dt);
    }

    //  translated comment
    const float HARD_ALT_FLOOR = 45.0f;
    if (state.position.y < HARD_ALT_FLOOR) {
        state.position.y = HARD_ALT_FLOOR;
        if (state.velocity.y < 0.0f) state.velocity.y = 0.0f;
    }
}

// ──  translated comment

static void update_chase_camera(const AircraftState& state,
                                float dt,
                                glm::vec3& eye,
                                glm::vec3& target) {
    // WT-like chase: stabilized horizon, behind velocity vector with lag.
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    glm::vec3 vel = state.velocity;
    if (glm::length(vel) < 1e-3f) vel = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 fwd = glm::normalize(vel);
    glm::vec3 right = glm::normalize(glm::cross(up, -fwd));
    if (glm::length(right) < 1e-4f) right = glm::vec3(1.0f, 0.0f, 0.0f);

    float speed = glm::length(state.velocity);
    float back = glm::mix(52.0f, 86.0f, glm::clamp(speed / 240.0f, 0.0f, 1.0f));
    float height = glm::mix(18.0f, 28.0f, glm::clamp(speed / 240.0f, 0.0f, 1.0f));
    float side = 3.0f;

    glm::vec3 desired_eye = state.position - fwd * back + up * height + right * side;
    glm::vec3 desired_target = state.position + fwd * 120.0f + up * 6.0f;

    static bool initialized = false;
    static glm::vec3 smoothed_eye;
    static glm::vec3 smoothed_target;
    if (!initialized) {
        smoothed_eye = desired_eye;
        smoothed_target = desired_target;
        initialized = true;
    }

    float smooth = 1.0f - std::exp(-6.0f * dt);
    smoothed_eye = glm::mix(smoothed_eye, desired_eye, smooth);
    smoothed_target = glm::mix(smoothed_target, desired_target, smooth);

    eye = smoothed_eye;
    target = smoothed_target;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    // GLFW translated comment
    if (!glfwInit()) {
        printf("GLFW init failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // macOS  translated comment

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Fighter Sim", nullptr, nullptr);
    if (!window) {
        printf("Window creation failed\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);   // vsync
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    int fb_w = 0, fb_h = 0;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    glViewport(0, 0, fb_w, fb_h);

    
    //  translated comment
    Renderer renderer(WIDTH, HEIGHT);
    if (!renderer.init()) {
        printf("Renderer init failed\n");
        return 1;
    }


    //  translated comment
    WireMesh fighter_mesh = make_fighter_mesh();
    WireMesh carrier_mesh = make_carrier_mesh();
    WireMesh carrier_marks_mesh = make_catapult_marks_mesh();
    WireMesh bomb_mesh = make_bomb_mesh();
    WireMesh bullet_mesh = make_bullet_mesh();
    WireMesh cloud_mesh = make_cloud_mesh();
    WireMesh explosion_mesh = make_explosion_mesh();
    const float player_model_scale = 1.42f;
    const float enemy_model_scale = 1.42f;
    const glm::vec3 player_usn_base(0.18f, 0.28f, 0.56f);   // WWII USN night sea blue (boosted visibility)
    const glm::vec3 player_usn_roundel_blue(0.12f, 0.24f, 0.50f);
    const glm::vec3 player_usn_roundel_white(0.94f, 0.94f, 0.92f);

    //  translated comment
    AircraftState state;
    state.position    = START_POS;                  // 1000m translated comment
    state.velocity    = glm::vec3(0, 0, -60.0f);   //  translated comment
    state.orientation = glm::quat(1, 0, 0, 0);      //  translated comment
    state.angular_vel = glm::vec3(0, 0, 0);

    // --- JSBSim Flight Dynamics ---
    JSBSimAdapter jsbsim;
    bool jsbsim_ready = false;
    ScriptConfig script;
    int model_index = 0;
    int phase_index = 0;
    float phase_time_s = 0.0f;
    std::vector<Projectile> bullets;
    std::vector<Projectile> bombs;
    std::vector<EnemyAircraft> enemies(kEnableCombat ? 6 : 0);
    std::vector<Cloud> clouds;
    std::vector<AABattery> aa_batteries;
    std::vector<AAShell> aa_shells;
    std::vector<Explosion> explosions;
    int player_hp = 100;
    int score = 0;
    int muzzle_index = 0;
    const double hold_fire_threshold_s = 0.22;
    const double gun_interval_s = 1.0 / 18.0;
    double gun_timer_s = 0.0;

    double last_time = glfwGetTime();
    double accumulator = 0.0;
    bool crashed = false;
    bool intro_active = false;
    double intro_time_s = 0.0;
    const double intro_duration_s = 8.5;
    [[maybe_unused]] const glm::vec3 carrier_pos(0.0f, START_POS.y - 70.0f, 220.0f);
    std::ofstream csv_log;
    std::ofstream track_log;
    double sim_time_s = 0.0;
    double next_track_log_s = 0.0;
    PID speed_pid(0.06f, 0.005f, 0.02f, 0.6f);
    PID climb_pid(0.20f, 0.02f, 0.06f, 0.4f);
    PID heading_pid(0.02f, 0.000f, 0.01f, 0.35f);

    printf("Fighter Sim started\n");
    printf("OpenGL: %s\n", glGetString(GL_VERSION));
    if (kEnableCombat) {
        for (size_t i = 0; i < enemies.size(); ++i) {
            respawn_enemy(enemies[i], state, (int)i, 0.0f, g_game_speed_scale);
        }
    }
    // clouds disabled
    if (kEnableCombat) {
        for (int i = 0; i < 10; ++i) {
            AABattery b;
            b.position = glm::vec3(-1800.0f + i * 380.0f, 0.0f, -2200.0f - i * 520.0f);
            b.cooldown_s = 0.35f * i;
            aa_batteries.push_back(b);
        }
    }

    printf("Mode: JSBSim scripted experiment\n");
    printf("Intro: disabled\n");
    printf("Controls: ESC=quit\n\n");

    std::string script_path = std::string(FIGHTER_SIM_ROOT) + "/" + kScriptPath;
    if (!std::filesystem::exists(script_path)) {
        std::filesystem::create_directories(std::string(FIGHTER_SIM_ROOT) + "/config");
        write_default_script(script_path);
        printf("Created default script: %s\n", script_path.c_str());
    }
    script = load_script(script_path);
    if (script.run_mode == "single") {
        script.models.clear();
        if (!script.model.empty()) {
            script.models.push_back(script.model);
        }
    }
    if (script.models.empty()) {
        printf("No models found in %s. Exiting.\n", script_path.c_str());
        return 1;
    }
    //没有飞行脚本的情况下足够长的时间自由落体
    if (script.phases.empty()) {
    printf("No phases found in %s. Using free flight mode (zero controls).\n", script_path.c_str());
    Phase default_phase;
    default_phase.duration_s = 1e9f;          // 足够长，相当于无限
    default_phase.throttle = 0.0f;
    default_phase.elevator = 0.0f;
    default_phase.aileron = 0.0f;
    default_phase.rudder = 0.0f;
    default_phase.flap = 0.0f;
    default_phase.gear = 1.0f;                 // 起落架默认收起（可根据需要调整）
    default_phase.has_targets = false;         // 不使用 PID
    script.phases.push_back(default_phase);
    }

    std::filesystem::create_directories(std::string(FIGHTER_SIM_ROOT) + "/build/logs");

    state.position = glm::vec3(0.0f, script.initial_alt_m, 0.0f);
    state.velocity = glm::vec3(0.0f, 0.0f, -script.initial_speed_mps);
    state.orientation = glm::quat(1, 0, 0, 0);
    state.angular_vel = glm::vec3(0.0f);

    jsbsim_ready = jsbsim.init(script.models[model_index], state, SIM_DT);
    if (!jsbsim_ready) {
        printf("JSBSim init failed (model %s). Exiting.\n", script.models[model_index].c_str());
        return 1;
    }
    jsbsim.sync_state(state);
    if (kUseAutopilot) {
        jsbsim.enable_autopilot();
    }

    std::string log_path = std::string(FIGHTER_SIM_ROOT) + "/build/logs/" + script.models[model_index] + ".csv";
    std::string track_path = std::string(FIGHTER_SIM_ROOT) + "/build/logs/last_flight.txt";
#if ENABLE_CSV_LOG
    csv_log.open(log_path, std::ios::out | std::ios::trunc);
    if (csv_log.is_open()) {
        csv_log << "t_s,phase,px_m,py_m,pz_m,vx_mps,vy_mps,vz_mps,roll_rad,pitch_rad,yaw_rad,p_rad_s,q_rad_s,r_rad_s,aoa_rad,airspeed_mps,throttle\n";
    } else {
        printf("Warning: could not open %s for writing.\n", log_path.c_str());
    }
#endif
    track_log.open(track_path, std::ios::out | std::ios::trunc);
    if (track_log.is_open()) {
        track_log << "t_s,alt_m,px_m,py_m,pz_m,speed_mps,vy_mps,aileron,elevator,rudder,thr_cmd,thr_actual,eng_rpm\n";
    } else {
        printf("Warning: could not open %s for writing.\n", track_path.c_str());
    }

    // ──  translated comment

    while (!glfwWindowShouldClose(window)) {
        double now = glfwGetTime();
        double frame_time = now - last_time;
        last_time = now;
        if (frame_time > 0.1) frame_time = 0.1;   //  translated comment
        accumulator += frame_time;

        glfwPollEvents();

        if (intro_active) {
            intro_time_s += frame_time;
            float nt = glm::clamp((float)(intro_time_s / intro_duration_s), 0.0f, 1.0f);
            float run_t = smooth_step(0.02f, 0.58f, nt);
            float climb_t = smooth_step(0.46f, 1.0f, nt);

            float pitch = glm::mix(0.0f, glm::radians(18.0f), climb_t);
            state.orientation = glm::quat(glm::vec3(pitch, 0.0f, 0.0f));
            glm::vec3 forward = glm::normalize(glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1));

            state.position = carrier_pos + glm::vec3(
                0.0f,
                14.0f + 90.0f * climb_t * climb_t,
                106.0f - 560.0f * run_t
            );
            float launch_speed = 35.0f + 235.0f * run_t;
            state.velocity = forward * launch_speed;
            state.angular_vel = glm::vec3(0.0f);

            glm::vec3 cam_pos = state.position + glm::vec3(76.0f, 30.0f, 90.0f);
            glm::vec3 cam_target = state.position + forward * 70.0f + glm::vec3(0.0f, 7.0f, 0.0f);
            renderer.set_camera(cam_pos, cam_target);

            //  translated comment
            float takeoff_start = (float)(intro_duration_s * 0.58);
            float gear_deploy = 1.0f - smooth_step(takeoff_start, takeoff_start + 2.0f, (float)intro_time_s);
            float flap_deploy = 1.0f - smooth_step(takeoff_start + 0.35f, takeoff_start + 2.35f, (float)intro_time_s);
            WireMesh intro_fighter_mesh = make_fighter_mesh_variant(gear_deploy, flap_deploy);

            renderer.begin_frame();
            renderer.draw_ground_grid(3200.0f, 120.0f, carrier_pos.y - 8.0f);
            renderer.draw_mesh(carrier_mesh, carrier_pos, glm::quat(1, 0, 0, 0), {0.12f, 0.13f, 0.15f});
            renderer.draw_mesh(carrier_marks_mesh, carrier_pos, glm::quat(1, 0, 0, 0), {0.92f, 0.92f, 0.92f});
            renderer.draw_mesh_scaled(intro_fighter_mesh, state.position, state.orientation,
                                      player_model_scale, player_usn_base);
            renderer.end_frame();
            glfwSwapBuffers(window);

            char intro_title[160];
            std::snprintf(intro_title, sizeof(intro_title),
                          "Fighter Sim | ENTERPRISE LAUNCH | %.0f%%",
                          nt * 100.0f);
            glfwSetWindowTitle(window, intro_title);

            if (intro_time_s >= intro_duration_s) {
                intro_active = false;
                //  translated comment
                state.velocity = glm::vec3(0.0f, 0.0f, -55.0f);
                state.orientation = glm::quat(1, 0, 0, 0);
                g_trim_origin = state.position;
            }
            continue;
        }

        // Game speed locked when using JSBSim direct control.
        g_trim_origin = state.position;

        if (kEnableCombat) {
            double hold_duration = g_keys.fire_held ? (now - g_keys.fire_press_time) : 0.0;
            bool machine_gun_mode = g_keys.fire_held && hold_duration >= hold_fire_threshold_s;
            if (g_keys.fire_released) {
                if (g_keys.fire_release_duration < hold_fire_threshold_s) {
                    spawn_bomb(state, bombs);
                }
                g_keys.fire_released = false;
            }
            if (machine_gun_mode) {
                gun_timer_s -= frame_time;
                while (gun_timer_s <= 0.0) {
                    spawn_bullet(state, bullets, muzzle_index++);
                    gun_timer_s += gun_interval_s;
                }
            } else {
                gun_timer_s = 0.0;
            }
        } else {
            g_keys.fire_released = false;
            gun_timer_s = 0.0;
        }

        //  translated comment
        while (accumulator >= SIM_DT) {
            float sim_dt = SIM_DT * g_game_speed_scale;
            const Phase& phase = script.phases[phase_index];
            ControlInput ctrl{};
            if (kUseAutopilot) {
                ctrl = build_autopilot_controls(g_throttle_cmd);
            } else {
                if (script.use_pid || phase.has_targets) {
                    float airspeed = glm::length(state.velocity);
                    float throttle_bias = 0.45f;
                    float throttle_out = speed_pid.update(phase.target_speed_mps, airspeed, sim_dt);
                    ctrl.throttle = glm::clamp(throttle_bias + throttle_out, 0.0f, 1.0f);

                    float climb_out = climb_pid.update(phase.target_climb_mps, state.velocity.y, sim_dt);
                    ctrl.elevator = glm::clamp(climb_out, -0.6f, 0.6f);

                    glm::vec3 euler = state.euler_angles();
                    float heading_deg = wrap_angle_deg(glm::degrees(euler.y));
                    float heading_err = shortest_angle_deg(phase.target_heading_deg, heading_deg);
                    float rudder_out = heading_pid.update(0.0f, -heading_err, sim_dt);
                    ctrl.rudder = glm::clamp(rudder_out, -0.6f, 0.6f);

                    ctrl.aileron = 0.0f;
                } else {
                    ctrl.elevator = glm::clamp(phase.elevator, -1.0f, 1.0f);
                    ctrl.aileron  = glm::clamp(phase.aileron,  -1.0f, 1.0f);
                    ctrl.rudder   = glm::clamp(phase.rudder,   -1.0f, 1.0f);
                    ctrl.throttle = glm::clamp(phase.throttle, 0.0f, 1.0f);
                }
            }
            g_throttle_cmd = ctrl.throttle;
            jsbsim.set_controls(ctrl);
            jsbsim.set_aux_controls(phase.flap, phase.gear);
            jsbsim.step(sim_dt);
            jsbsim.sync_state(state);
            sim_time_s += sim_dt;
            if (ENABLE_CSV_LOG && csv_log.is_open()) {
                glm::vec3 euler = state.euler_angles();
                glm::vec3 body_vel = glm::mat3_cast(glm::inverse(state.orientation)) * state.velocity;
                float forward_speed = -body_vel.z;
                if (forward_speed < 0.1f) forward_speed = 0.1f;
                float aoa_rad = std::atan2(body_vel.y, forward_speed);
                float airspeed = glm::length(state.velocity);
                csv_log << sim_time_s << ","
                        << "\"" << phase.name << "\"" << ","
                        << state.position.x << "," << state.position.y << "," << state.position.z << ","
                        << state.velocity.x << "," << state.velocity.y << "," << state.velocity.z << ","
                        << euler.z << "," << euler.x << "," << euler.y << ","
                        << state.angular_vel.z << "," << state.angular_vel.x << "," << state.angular_vel.y << ","
                        << aoa_rad << "," << airspeed << ","
                        << g_throttle_cmd << "\n";
            }
            if (track_log.is_open()) {
                if (sim_time_s + 1e-6 >= next_track_log_s) {
                    float speed = glm::length(state.velocity);
                    double thr_actual = jsbsim.get_property("controls/engines/engine[0]/throttle");
                    // FGPiston exposes RPM as "propulsion/engine[N]/engine-rpm" (not just "/rpm")
                    double eng_rpm = jsbsim.get_property("propulsion/engine[0]/engine-rpm");
                    if (eng_rpm < 1.0) eng_rpm = jsbsim.get_property("propulsion/engine[0]/rpm");
                    if (eng_rpm < 1.0) eng_rpm = jsbsim.get_property("engines/active-engine/rpm");
                    track_log.setf(std::ios::fixed);
                    track_log << std::setprecision(2)
                              << sim_time_s << ","
                              << state.position.y << ","
                              << state.position.x << "," << state.position.y << "," << state.position.z << ","
                              << speed << ","
                              << state.velocity.y << ","
                              << ctrl.aileron << "," << ctrl.elevator << "," << ctrl.rudder << ","
                              << g_throttle_cmd << "," << thr_actual << "," << eng_rpm << "\n";
                    next_track_log_s = std::floor(sim_time_s + 1.0);
                }
            }
            phase_time_s += sim_dt;
            if (phase_time_s >= phase.duration_s) {
                phase_time_s = 0.0f;
                phase_index++;
                speed_pid.reset();
                climb_pid.reset();
                heading_pid.reset();
                if (phase_index >= (int)script.phases.size()) {
                    phase_index = 0;
                    model_index++;
#if ENABLE_CSV_LOG
                    if (csv_log.is_open()) {
                        csv_log.close();
                    }
#endif
                    if (track_log.is_open()) {
                        track_log.close();
                    }
                    if (model_index >= (int)script.models.size()) {
                        glfwSetWindowShouldClose(window, true);
                        break;
                    }

                    state.position = glm::vec3(0.0f, script.initial_alt_m, 0.0f);
                    state.velocity = glm::vec3(0.0f, 0.0f, -script.initial_speed_mps);
                    state.orientation = glm::quat(1, 0, 0, 0);
                    state.angular_vel = glm::vec3(0.0f);
                    speed_pid.reset();
                    climb_pid.reset();
                    heading_pid.reset();

                    jsbsim_ready = jsbsim.init(script.models[model_index], state, SIM_DT);
                    if (!jsbsim_ready) {
                        printf("JSBSim init failed (model %s). Exiting.\n", script.models[model_index].c_str());
                        glfwSetWindowShouldClose(window, true);
                        break;
                    }
                    jsbsim.sync_state(state);
                    if (kUseAutopilot) {
                        jsbsim.enable_autopilot();
                    }
                    sim_time_s = 0.0;

                    std::string log_path = std::string(FIGHTER_SIM_ROOT) + "/build/logs/" + script.models[model_index] + ".csv";
                    std::string track_path = std::string(FIGHTER_SIM_ROOT) + "/build/logs/last_flight.txt";
#if ENABLE_CSV_LOG
                    csv_log.open(log_path, std::ios::out | std::ios::trunc);
                    if (csv_log.is_open()) {
                        csv_log << "t_s,phase,px_m,py_m,pz_m,vx_mps,vy_mps,vz_mps,roll_rad,pitch_rad,yaw_rad,p_rad_s,q_rad_s,r_rad_s,aoa_rad,airspeed_mps,throttle\n";
                    } else {
                        printf("Warning: could not open %s for writing.\n", log_path.c_str());
                    }
#endif
                    track_log.open(track_path, std::ios::out | std::ios::trunc);
                    if (track_log.is_open()) {
                        track_log << "t_s,alt_m,px_m,py_m,pz_m,speed_mps,vy_mps,aileron,elevator,rudder,thr_cmd,thr_actual,eng_rpm\n";
                    } else {
                        printf("Warning: could not open %s for writing.\n", track_path.c_str());
                    }
                    next_track_log_s = 0.0;
                }
            }
            if (kEnableCombat) {
                update_projectiles(bullets, bombs, sim_dt);
                update_ground_aa(aa_batteries, aa_shells, state, sim_dt);
                update_explosions(explosions, sim_dt);
            }
            accumulator -= SIM_DT;
        }

        if (kEnableCombat) {
            update_enemies(enemies, state, (float)frame_time, (float)now, g_game_speed_scale);
        }
        // clouds disabled

        //  translated comment
        if (kEnableCombat) {
            for (auto it_b = bullets.begin(); it_b != bullets.end();) {
                bool hit = false;
                for (size_t i = 0; i < enemies.size(); ++i) {
                    if (glm::distance(it_b->position, enemies[i].position) < 26.0f) {
                        explosions.push_back(Explosion{enemies[i].position, 0.45f, 0.45f});
                        respawn_enemy(enemies[i], state, (int)i, (float)now, g_game_speed_scale);
                        score += 10;
                        hit = true;
                        break;
                    }
                }
                if (hit) it_b = bullets.erase(it_b);
                else ++it_b;
            }
        }
        //  translated comment
        if (kEnableCombat) {
            for (auto it_m = bombs.begin(); it_m != bombs.end();) {
                bool exploded = false;
                for (size_t i = 0; i < enemies.size(); ++i) {
                    if (glm::distance(it_m->position, enemies[i].position) < 58.0f) {
                        explosions.push_back(Explosion{enemies[i].position, 0.52f, 0.52f});
                        respawn_enemy(enemies[i], state, (int)i, (float)now, g_game_speed_scale);
                        score += 20;
                        exploded = true;
                        break;
                    }
                }
                if (exploded) it_m = bombs.erase(it_m);
                else ++it_m;
            }
            for (auto it_s = aa_shells.begin(); it_s != aa_shells.end();) {
                if (glm::distance(it_s->position, state.position) < 24.0f) {
                    player_hp -= 8;
                    it_s = aa_shells.erase(it_s);
                } else {
                    ++it_s;
                }
            }
        }
        if (player_hp <= 0) {
            crashed = true;
            printf("\n\n[DOWNED] Hit by ground anti-air fire.\n");
            glfwSetWindowTitle(window, "Fighter Sim | DOWNED");
            glfwSetWindowShouldClose(window, true);
            continue;
        }

        if (state.position.y < 0.0f) {
            crashed = true;
            float speed = glm::length(state.velocity);
            printf("\n\n[CRASH] Aircraft impacted ground. Alt: %.2f m, Speed: %.2f m/s\n",
                   state.position.y, speed);
            glfwSetWindowTitle(window, "Fighter Sim | CRASHED");
            glfwSetWindowShouldClose(window, true);
            continue;
        }

        //  translated comment
        glm::vec3 cam_pos, cam_target;
        update_chase_camera(state, (float)frame_time, cam_pos, cam_target);
        renderer.set_camera(cam_pos, cam_target);

        //  translated comment
        renderer.begin_frame();
        renderer.draw_ground_grid(3200.0f, 120.0f, 0.0f);
        renderer.draw_axes(80.0f);
        float qz = state.position.z - 220.0f;
        float ang_mag = glm::length(state.angular_vel);
        float speed_mag = glm::max(glm::length(state.velocity), 1.0f);
        float trim_mag = glm::length(g_trim_wheel_vel);
        glm::vec3 forward = glm::normalize(glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1));
        glm::vec3 vel_dir = glm::normalize(state.velocity);
        float alignment = glm::clamp(glm::dot(forward, vel_dir), -1.0f, 1.0f);
        float slip = std::acos(alignment); // 0 = on-velocity, higher = more "off"
        float yaw_rate = std::abs(state.angular_vel.y);
        float turn_factor = glm::clamp((yaw_rate * speed_mag) / 260.0f, 0.0f, 1.0f);
        float g_load = glm::length(glm::cross(state.velocity, state.angular_vel)) / 9.81f;
        g_load = glm::clamp(g_load, 0.0f, 6.0f);
        float g_threshold = 2.0f;
        float g_boost = (g_load > g_threshold) ? (g_load - g_threshold) * 0.14f : 0.0f;
        float ring_target = 0.80f
                          + glm::clamp(ang_mag / glm::radians(55.0f), 0.0f, 1.0f) * 0.22f
                          + glm::clamp(slip / glm::radians(22.0f), 0.0f, 1.0f) * 0.55f
                          + turn_factor * 0.20f
                          + glm::clamp(trim_mag / TRIM_WHEEL_MAX_V, 0.0f, 1.0f) * 0.10f
                          + g_boost;
        ring_target = glm::clamp(ring_target, 0.75f, 1.6f);
        static float ring_scale = 1.0f;
        float rise_rate = 0.7f;   // expand slower
        float fall_rate = 2.8f;   // shrink faster
        float rate = (ring_target > ring_scale) ? rise_rate : fall_rate;
        ring_scale = ring_scale + (ring_target - ring_scale) * (1.0f - std::exp(-rate * (float)frame_time));
        float ring_pulse = 0.05f * std::sin((float)now * 3.2f);
        // HUD ring/quadrant disabled
        // clouds disabled
        glLineWidth(3.0f);
        renderer.draw_mesh_scaled(fighter_mesh, state.position, state.orientation,
                                  player_model_scale, player_usn_base);   //  translated comment
        if (kEnableCombat) {
            for (const auto& e : enemies) {
                // IJN late-1930s/WWII style: aged ame-iro base + red roundels.
                renderer.draw_mesh_scaled(fighter_mesh, e.position, e.orientation,
                                          enemy_model_scale, {0.52f, 0.50f, 0.38f});

            }
        }
        glLineWidth(1.5f);
        const glm::quat identity_q(1.0f, 0.0f, 0.0f, 0.0f);
        if (kEnableCombat) {
            for (const auto& b : bombs) {
                renderer.draw_mesh(bomb_mesh, b.position, identity_q, {0.78f, 0.10f, 0.08f});
            }
            for (const auto& p : bullets) {
                renderer.draw_mesh(bullet_mesh, p.position, identity_q, {0.92f, 0.12f, 0.10f});
            }
            for (const auto& s : aa_shells) {
                renderer.draw_mesh(bullet_mesh, s.position, identity_q, {0.78f, 0.18f, 0.10f});
            }
            for (const auto& ex : explosions) {
                float t = 1.0f - ex.ttl_s / ex.max_ttl_s;
                //  translated comment
                float outer_scale = 1.0f + 0.32f * t;
                float inner_scale = 0.72f + 0.18f * t;
                glm::vec3 outer_color = glm::mix(glm::vec3(1.00f, 0.78f, 0.18f),
                                                 glm::vec3(0.90f, 0.16f, 0.06f), t);
                glm::vec3 inner_color = glm::mix(glm::vec3(1.00f, 0.95f, 0.45f),
                                                 glm::vec3(1.00f, 0.56f, 0.12f), t);
                renderer.draw_mesh_scaled(explosion_mesh, ex.position, identity_q, outer_scale, outer_color);
                renderer.draw_mesh_scaled(explosion_mesh, ex.position, identity_q, inner_scale, inner_color);
            }
        }
        if (kEnableCombat) {
            std::vector<glm::vec3> enemy_positions;
            std::vector<glm::vec3> enemy_velocities;
            enemy_positions.reserve(enemies.size());
            enemy_velocities.reserve(enemies.size());
            for (const auto& e : enemies) {
                enemy_positions.push_back(e.position);
                enemy_velocities.push_back(e.velocity);
            }
            renderer.draw_radar(state.position, enemy_positions, enemy_velocities, state.velocity, 720.0f);
            renderer.draw_world_map(state.position, enemy_positions, state.velocity, 4200.0f);
        }
        // attitude gauge disabled
        glm::vec3 euler_rad = state.euler_angles();
        glm::vec3 euler_deg = glm::degrees(euler_rad);
        glm::vec3 body_vel = glm::mat3_cast(glm::inverse(state.orientation)) * state.velocity;
        float forward_speed = -body_vel.z;
        if (forward_speed < 0.1f) forward_speed = 0.1f;
        float aoa_deg = glm::degrees(std::atan2(body_vel.y, forward_speed));
        float beta_deg = glm::degrees(std::atan2(body_vel.x, forward_speed));
        glm::vec3 angular_vel_deg_s = glm::degrees(state.angular_vel);
        // JSBSim telemetry (top-left)
        renderer.draw_hud(euler_deg,
                          state.position,
                          state.velocity,
                          angular_vel_deg_s,
                          g_throttle_cmd,
                          g_game_speed_scale,
                          aoa_deg,
                          beta_deg);
        renderer.end_frame();

        glfwSwapBuffers(window);

        char title[256];
        if (kEnableCombat) {
            std::snprintf(title, sizeof(title),
                          "Fighter Sim | GAME | HP %d | Score %d | GS %.0f%% | PosXY(%.0f,%.0f) | Alt %.0f m | V %.0f m/s | Vy %.1f m/s | AoA %.1f deg | Thr %.0f%% | Bullets %zu | Bombs %zu | AA %zu",
                          player_hp, score,
                          g_game_speed_scale * 100.0f, g_trim_origin.x, g_trim_origin.y,
                          state.position.y, glm::length(state.velocity), state.velocity.y,
                          aoa_deg, g_throttle_cmd * 100.0f,
                          bullets.size(), bombs.size(), aa_shells.size());
        } else {
            std::snprintf(title, sizeof(title),
                          "Fighter Sim | JSBSim Minimal | Alt %.0f m | V %.0f m/s | Vy %.1f m/s | AoA %.1f deg | Thr %.0f%%",
                          state.position.y, glm::length(state.velocity), state.velocity.y,
                          aoa_deg, g_throttle_cmd * 100.0f);
        }
        glfwSetWindowTitle(window, title);
    }

    if (crashed) {
        printf("Flight ended: crashed.\n");
    } else {
        printf("\nBye.\n");
    }
#if ENABLE_CSV_LOG
    if (csv_log.is_open()) {
        csv_log.close();
        printf("JSBSim logs written to build/logs/*.csv\n");
    }
#endif
    if (track_log.is_open()) {
        track_log.close();
        printf("Flight track written to build/logs/last_flight.txt\n");
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
