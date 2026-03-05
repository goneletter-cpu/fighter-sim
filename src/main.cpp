#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <cmath>

#include "physics.h"
#include "pid.h"
#include "flight_controller.h"
#include "renderer.h"

// ── 窗口参数 ──────────────────────────────────────────────────────────────────

static const int   WIDTH  = 1280;
static const int   HEIGHT = 720;
static const float SIM_DT = 1.0f / 120.0f;   // 物理步长120Hz，渲染按实际帧率
static const glm::vec3 START_POS(0.0f, 1000.0f, 0.0f);
static const glm::vec3 FIXED_WAYPOINT(6000.0f, 1800.0f, -8000.0f); // x/y/z均与起点不同

// ── 键盘状态 ──────────────────────────────────────────────────────────────────

struct KeyState {
    bool pitch_up   = false;
    bool pitch_down = false;
    bool roll_left  = false;
    bool roll_right = false;
    bool yaw_left   = false;
    bool yaw_right  = false;
    bool throttle_up   = false;
    bool throttle_down = false;
};

static KeyState g_keys;

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
        case GLFW_KEY_LEFT_SHIFT:  g_keys.throttle_up   = pressed; break;
        case GLFW_KEY_LEFT_CONTROL:g_keys.throttle_down = pressed; break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(win, true); break;
        default: break;
    }
}

// 玩家输入只作为“干预量”，叠加在自动驾驶指令上
static void apply_player_intervention(AttitudeCommand& cmd, float dt) {
    const float PITCH_DELTA = glm::radians(6.0f);   // rad/s
    const float ROLL_DELTA  = glm::radians(10.0f);
    const float YAW_DELTA   = glm::radians(5.0f);
    const float THR_DELTA   = 0.06f;

    if (g_keys.pitch_up)    cmd.pitch_rad += PITCH_DELTA * dt;
    if (g_keys.pitch_down)  cmd.pitch_rad -= PITCH_DELTA * dt;
    if (g_keys.roll_left)   cmd.roll_rad  -= ROLL_DELTA  * dt;
    if (g_keys.roll_right)  cmd.roll_rad  += ROLL_DELTA  * dt;
    if (g_keys.yaw_left)    cmd.yaw_rate_rad_s -= YAW_DELTA;
    if (g_keys.yaw_right)   cmd.yaw_rate_rad_s += YAW_DELTA;
    if (g_keys.throttle_up)   cmd.throttle += THR_DELTA * dt;
    if (g_keys.throttle_down) cmd.throttle -= THR_DELTA * dt;

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.yaw_rate_rad_s = glm::clamp(cmd.yaw_rate_rad_s, glm::radians(-16.0f), glm::radians(16.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
}

static float wrap_pi(float a) {
    while (a > glm::pi<float>()) a -= glm::two_pi<float>();
    while (a < -glm::pi<float>()) a += glm::two_pi<float>();
    return a;
}

static float slew_to(float current, float target, float max_step) {
    float delta = glm::clamp(target - current, -max_step, max_step);
    return current + delta;
}

static float update_autopilot_command(const AircraftState& state,
                                      const glm::vec3& waypoint,
                                      float dt,
                                      AttitudeCommand& cmd) {
    glm::vec3 to_target = waypoint - state.position;
    float distance = glm::length(to_target);
    float horiz_dist = glm::length(glm::vec2(to_target.x, to_target.z));

    glm::vec3 forward = glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1);
    float speed = glm::length(state.velocity);
    if (speed > 20.0f) forward = glm::normalize(state.velocity);
    else forward = glm::normalize(forward);

    float current_heading = std::atan2(forward.x, -forward.z);
    float desired_heading = std::atan2(to_target.x, -to_target.z);
    float heading_error = wrap_pi(desired_heading - current_heading);

    float desired_pitch = std::atan2(to_target.y, glm::max(horiz_dist, 1.0f));
    desired_pitch = glm::clamp(desired_pitch, glm::radians(-12.0f), glm::radians(12.0f));

    float desired_roll = glm::clamp(heading_error * 1.6f,
                                    glm::radians(-35.0f), glm::radians(35.0f));
    float desired_yaw_rate = glm::clamp(heading_error * 1.2f,
                                        glm::radians(-14.0f), glm::radians(14.0f));

    float target_speed = (distance > 1200.0f) ? 190.0f : 155.0f;
    float target_throttle = 0.56f + 0.005f * (target_speed - speed);
    target_throttle = glm::clamp(target_throttle, 0.35f, 0.90f);

    if (distance < 120.0f) {
        desired_pitch = 0.0f;
        desired_roll = 0.0f;
        desired_yaw_rate = 0.0f;
        target_throttle = 0.45f;
    }

    cmd.pitch_rad = slew_to(cmd.pitch_rad, desired_pitch, glm::radians(18.0f) * dt);
    cmd.roll_rad = slew_to(cmd.roll_rad, desired_roll, glm::radians(28.0f) * dt);
    cmd.yaw_rate_rad_s = desired_yaw_rate;
    cmd.throttle = slew_to(cmd.throttle, target_throttle, 0.25f * dt);

    cmd.pitch_rad = glm::clamp(cmd.pitch_rad, glm::radians(-30.0f), glm::radians(30.0f));
    cmd.roll_rad  = glm::clamp(cmd.roll_rad,  glm::radians(-50.0f), glm::radians(50.0f));
    cmd.throttle  = glm::clamp(cmd.throttle, 0.0f, 1.0f);
    return distance;
}

// ── 带惯性的追踪相机（不完全绑定机体，避免“原地打转”观感）─────────────────────

static void update_chase_camera(const AircraftState& state,
                                float dt,
                                glm::vec3& eye,
                                glm::vec3& target) {
    glm::vec3 forward = glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1);
    float speed = glm::length(state.velocity);
    if (speed > 5.0f) {
        forward = glm::normalize(state.velocity);
    } else {
        forward = glm::normalize(forward);
    }

    glm::vec3 desired_eye = state.position - forward * 60.0f + glm::vec3(0, 18.0f, 0);
    glm::vec3 desired_target = state.position + forward * 120.0f + glm::vec3(0, -10.0f, 0);

    static bool initialized = false;
    static glm::vec3 smoothed_eye;
    static glm::vec3 smoothed_target;
    if (!initialized) {
        smoothed_eye = desired_eye;
        smoothed_target = desired_target;
        initialized = true;
    }

    float smooth = 1.0f - std::exp(-4.0f * dt);
    smoothed_eye = glm::mix(smoothed_eye, desired_eye, smooth);
    smoothed_target = glm::mix(smoothed_target, desired_target, smooth);

    eye = smoothed_eye;
    target = smoothed_target;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    // GLFW初始化
    if (!glfwInit()) {
        printf("GLFW init failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // macOS 必须

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

    // 渲染器初始化
    Renderer renderer(WIDTH, HEIGHT);
    if (!renderer.init()) {
        printf("Renderer init failed\n");
        return 1;
    }

    // 飞机模型
    WireMesh fighter_mesh = make_fighter_mesh();

    // 飞行状态初始化
    AircraftState state;
    state.position    = START_POS;                  // 1000m高度
    state.velocity    = glm::vec3(0, 0, -150.0f);   // 初速150m/s向前
    state.orientation = glm::quat(1, 0, 0, 0);      // 水平飞行
    state.angular_vel = glm::vec3(0, 0, 0);

    FlightDynamics  dynamics;
    FlightController controller;
    AttitudeCommand command;
    command.throttle = 0.6f;
    glm::vec3 waypoint = FIXED_WAYPOINT;
    float waypoint_distance = 0.0f;

    double last_time = glfwGetTime();
    double accumulator = 0.0;
    bool crashed = false;

    printf("Fighter Sim 启动\n");
    printf("OpenGL: %s\n", glGetString(GL_VERSION));
    printf("模式: 自动飞行(玩家干预)\n");
    printf("操控: W/S=俯仰干预 A/D=滚转干预 Q/E=偏航干预 Shift/Ctrl=油门干预 ESC=退出\n");
    printf("固定目标点: (%.1f, %.1f, %.1f)\n\n", waypoint.x, waypoint.y, waypoint.z);

    // ── 主循环 ────────────────────────────────────────────────────────────────

    while (!glfwWindowShouldClose(window)) {
        double now = glfwGetTime();
        double frame_time = now - last_time;
        last_time = now;
        if (frame_time > 0.1) frame_time = 0.1;   // 防止死机时间爆炸
        accumulator += frame_time;

        glfwPollEvents();
        waypoint_distance = update_autopilot_command(state, waypoint, (float)frame_time, command);
        apply_player_intervention(command, (float)frame_time);

        // 固定步长物理更新
        while (accumulator >= SIM_DT) {
            ControlInput ctrl = controller.update(state, command, SIM_DT);
            dynamics.step(state, ctrl, SIM_DT);
            accumulator -= SIM_DT;
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

        // 追踪相机
        glm::vec3 cam_pos, cam_target;
        update_chase_camera(state, (float)frame_time, cam_pos, cam_target);
        renderer.set_camera(cam_pos, cam_target);

        // 渲染
        renderer.begin_frame();
        renderer.draw_ground_grid(8000.0f, 200.0f, 0.0f);
        renderer.draw_axes(80.0f);
        renderer.draw_mesh(fighter_mesh, state.position, state.orientation,
                           {0.08f, 0.10f, 0.14f});   // 白底下清晰的深色线框
        renderer.end_frame();

        glfwSwapBuffers(window);

        // 终端HUD
        glm::vec3 euler_rad = state.euler_angles();
        glm::vec3 euler_deg = glm::degrees(euler_rad);
        glm::vec3 body_vel = glm::mat3_cast(glm::inverse(state.orientation)) * state.velocity;
        float forward_speed = -body_vel.z;
        if (forward_speed < 0.1f) forward_speed = 0.1f;
        float aoa_deg = glm::degrees(std::atan2(-body_vel.y, forward_speed));
        float beta_deg = glm::degrees(std::atan2(body_vel.x, forward_speed));
        glm::vec3 angular_vel_deg_s = glm::degrees(state.angular_vel);
        renderer.draw_hud(euler_deg, state.position, state.velocity, angular_vel_deg_s,
                          command.throttle, aoa_deg, beta_deg);

        char title[256];
        std::snprintf(title, sizeof(title),
                      "Fighter Sim | AUTO+INPUT | Dist %.0f m | Alt %.0f m | V %.0f m/s | Vy %.1f m/s | AoA %.1f deg | Thr %.0f%%",
                      waypoint_distance,
                      state.position.y, glm::length(state.velocity), state.velocity.y,
                      aoa_deg, command.throttle * 100.0f);
        glfwSetWindowTitle(window, title);
    }

    if (crashed) {
        printf("Flight ended: crashed.\n");
    } else {
        printf("\n再见\n");
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
