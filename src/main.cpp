#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>

#include "physics.h"
#include "pid.h"
#include "flight_controller.h"
#include "renderer.h"

// ── 窗口参数 ──────────────────────────────────────────────────────────────────

static const int   WIDTH  = 1280;
static const int   HEIGHT = 720;
static const float SIM_DT = 1.0f / 120.0f;   // 物理步长120Hz，渲染按实际帧率
static const glm::vec3 START_POS(0.0f, 1000.0f, 0.0f);
[[maybe_unused]] static const glm::vec3 FIXED_WAYPOINT(6000.0f, 1800.0f, -8000.0f); // x/y/z均与起点不同

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
    bool fire_held = false;
    bool fire_released = false;
    double fire_press_time = 0.0;
    double fire_release_duration = 0.0;
};

static KeyState g_keys;
static int g_motion_quadrant = 1;

static float slew_to(float current, float target, float max_step) {
    float delta = glm::clamp(target - current, -max_step, max_step);
    return current + delta;
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
        case GLFW_KEY_LEFT_SHIFT:  g_keys.throttle_up   = pressed; break;
        case GLFW_KEY_LEFT_CONTROL:g_keys.throttle_down = pressed; break;
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

// 玩家输入只作为“干预量”，叠加在自动驾驶指令上
// base_cmd 是自动驾驶给出的基线指令，松杆时回到基线而不是回0，避免与自动驾驶打架
static void apply_player_intervention(AttitudeCommand& cmd,
                                      const AttitudeCommand& base_cmd,
                                      float dt) {
    const float PITCH_DELTA = glm::radians(6.0f);   // rad/s
    const float ROLL_DELTA  = glm::radians(10.0f);
    const float YAW_DELTA   = glm::radians(5.0f);
    const float THR_DELTA   = 0.06f;

    if (g_keys.pitch_up)    cmd.pitch_rad += PITCH_DELTA * dt;
    if (g_keys.pitch_down)  cmd.pitch_rad -= PITCH_DELTA * dt;
    if (g_keys.roll_left)   cmd.roll_rad  += ROLL_DELTA  * dt;
    if (g_keys.roll_right)  cmd.roll_rad  -= ROLL_DELTA  * dt;
    if (g_keys.yaw_left)    cmd.yaw_rate_rad_s += YAW_DELTA * dt;
    if (g_keys.yaw_right)   cmd.yaw_rate_rad_s -= YAW_DELTA * dt;
    if (g_keys.throttle_up)   cmd.throttle += THR_DELTA * dt;
    if (g_keys.throttle_down) cmd.throttle -= THR_DELTA * dt;

    // 稳定增强：松杆后回到自动驾驶基线，避免持续偏置
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

// 玩家输入产生角加速度，结合轴间耦合形成“有惯性”的姿态反馈
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

    // 输入映射为角加速度（不是直接把角速度拉到目标）
    glm::vec3 input_accel(
        pitch_input * glm::radians(165.0f), // x: pitch accel
        yaw_input   * glm::radians(130.0f), // y: yaw accel
        roll_input  * glm::radians(240.0f)  // z: roll accel
    );

    // 轴间耦合：滚转会诱导偏航，偏航会诱导少量俯仰
    glm::vec3 coupled_accel(
        -0.18f * state.angular_vel.y * std::abs(state.angular_vel.z),
        -0.28f * state.angular_vel.z * (0.7f + 0.3f * std::abs(roll_input)),
         0.10f * state.angular_vel.x * state.angular_vel.y
    );

    state.angular_vel += (input_accel + coupled_accel) * dt;

    // 速率阻尼（有输入也存在），避免原地高频抽搐
    glm::vec3 damping(1.8f, 2.2f, 2.0f);
    state.angular_vel.x -= state.angular_vel.x * damping.x * dt;
    state.angular_vel.y -= state.angular_vel.y * damping.y * dt;
    state.angular_vel.z -= state.angular_vel.z * damping.z * dt;

    // 软限幅，防止异常值
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

static glm::quat orientation_from_forward(const glm::vec3& forward_world) {
    glm::vec3 f = glm::normalize(forward_world);
    glm::vec3 up_ref(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::cross(up_ref, -f);
    if (glm::length(right) < 1e-4f) right = glm::vec3(1.0f, 0.0f, 0.0f);
    right = glm::normalize(right);
    glm::vec3 up = glm::normalize(glm::cross(-f, right));
    glm::mat3 basis(right, up, -f); // local -Z 对齐到 forward_world
    return glm::normalize(glm::quat_cast(basis));
}

static void respawn_enemy(EnemyAircraft& enemy,
                          const AircraftState& player,
                          int idx,
                          float t_now,
                          int quadrant) {
    glm::mat3 world_from_body = glm::mat3_cast(player.orientation);
    glm::vec3 fwd = glm::normalize(world_from_body * glm::vec3(0, 0, -1));
    glm::vec3 right = glm::normalize(world_from_body * glm::vec3(1, 0, 0));
    int sx = (quadrant == 1 || quadrant == 4) ? 1 : -1;
    int sy = (quadrant == 1 || quadrant == 2) ? 1 : -1;

    float lateral_mag = 170.0f + 95.0f * (idx % 4) + 45.0f * std::sin(0.7f * t_now + idx);
    float vertical_mag = 90.0f + 48.0f * (idx % 4) + 40.0f * std::sin(0.9f * t_now + 0.7f * idx);
    float lateral = sx * lateral_mag;
    float vertical = sy * vertical_mag;
    float ahead = 2200.0f + idx * 280.0f;

    float spawn_alt = glm::clamp(player.position.y + vertical, 260.0f, 2200.0f);
    enemy.position = player.position + fwd * ahead + right * lateral;
    enemy.position.y = spawn_alt;
    glm::vec3 dir_to_player = glm::normalize(player.position - enemy.position);
    enemy.velocity = dir_to_player * (190.0f + 12.0f * (idx % 3));
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

    float forward_speed = -v_body.z; // local -Z 为机头方向
    float target_speed = 185.0f;
    float thrust_accel = (target_speed - forward_speed) * 1.6f;
    v_body.z -= thrust_accel * dt;

    // 侧滑阻尼和垂向阻尼：防止“抽搐式横摆”
    v_body.x += (-2.4f * v_body.x) * dt;
    v_body.y += (-1.7f * v_body.y) * dt;

    // 姿态到动力转换：机体上方向与世界重力的夹角决定升力方向
    glm::vec3 horiz_lift(up.x, 0.0f, up.z);
    float hl = glm::length(horiz_lift);
    if (hl > 1e-4f) {
        horiz_lift /= hl;
        float speed_factor = glm::clamp(forward_speed / 190.0f, 0.35f, 1.25f);
        float turn_accel = 20.0f * hl * speed_factor;
        state.velocity += horiz_lift * turn_accel * dt;

        // 协调转弯耦合：滚转引入偏航角速度变化
        float bank_sign = glm::dot(right, glm::vec3(0, 1, 0));
        state.angular_vel.y += -bank_sign * glm::radians(22.0f) * speed_factor * dt;
    }

    // 抬头时提供附加升力，低头时减弱，减少“原地颤振式升降”
    float pitch_lift = glm::clamp(fwd.y, -0.6f, 0.7f);
    state.velocity.y += pitch_lift * 14.0f * dt;

    // 回写速度（保留上面的世界系修正）
    glm::vec3 world_from_body_v = world_from_body * v_body;
    state.velocity.x = 0.65f * state.velocity.x + 0.35f * world_from_body_v.x;
    state.velocity.y = 0.65f * state.velocity.y + 0.35f * world_from_body_v.y;
    state.velocity.z = 0.65f * state.velocity.z + 0.35f * world_from_body_v.z;
}

// 横版射击运动：始终向前卷轴，玩家只控制上下/左右机动
static void apply_side_scroller_motion(AircraftState& state, float dt) {
    // 坐标轴定义：WS=Y轴，QE=X轴；AD依据象限变成斜向轴
    float axis_x = 0.0f;
    float axis_y = 0.0f;
    float ad = 0.0f;
    if (g_keys.yaw_left)   axis_x -= 1.0f;  // Q
    if (g_keys.yaw_right)  axis_x += 1.0f;  // E
    if (g_keys.pitch_up)   axis_y += 1.0f;  // W
    if (g_keys.pitch_down) axis_y -= 1.0f;  // S
    if (g_keys.roll_left)  ad -= 1.0f;      // A
    if (g_keys.roll_right) ad += 1.0f;      // D

    float pitch_hint = (glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1)).y;
    int y_sign = 0;
    if (axis_y > 0.1f) y_sign = 1;
    else if (axis_y < -0.1f) y_sign = -1;
    else if (std::abs(pitch_hint) > 0.04f) y_sign = (pitch_hint > 0.0f) ? 1 : -1;

    float input_x = axis_x + ad;
    float input_y = axis_y + ad * (float)y_sign * 0.9f;
    input_x = glm::clamp(input_x, -1.0f, 1.0f);
    input_y = glm::clamp(input_y, -1.0f, 1.0f);

    // 当前象限（用于限制运动与敌机出现）
    float qx = (std::abs(axis_x) > 0.1f) ? axis_x : input_x;
    float qy = (std::abs(axis_y) > 0.1f) ? axis_y : (float)y_sign;
    if (qx >= 0.0f && qy >= 0.0f) g_motion_quadrant = 1;
    else if (qx < 0.0f && qy >= 0.0f) g_motion_quadrant = 2;
    else if (qx < 0.0f && qy < 0.0f) g_motion_quadrant = 3;
    else g_motion_quadrant = 4;

    const float scroll_speed = 185.0f;
    const float side_speed = 95.0f;
    const float vertical_speed = 80.0f;

    float target_vx = input_x * side_speed;
    float target_vy = input_y * vertical_speed;
    float blend = 1.0f - std::exp(-8.0f * dt);

    state.velocity.x = glm::mix(state.velocity.x, target_vx, blend);
    state.velocity.y = glm::mix(state.velocity.y, target_vy, blend);
    state.velocity.z = -scroll_speed;
    state.position += state.velocity * dt;

    // 边界限制（横版战场窗口）
    state.position.x = glm::clamp(state.position.x, -680.0f, 680.0f);
    state.position.y = glm::clamp(state.position.y, 220.0f, 1900.0f);

    // 姿态视觉反馈：
    // W/S -> 俯仰，Q/E -> 机头绕机位偏转（偏航），A/D -> 翻滚
    float target_pitch = glm::radians(10.0f) * axis_y;
    float target_yaw   = glm::radians(16.0f) * axis_x;
    float target_roll  = glm::radians(-24.0f) * ad;
    glm::quat target_q = glm::quat(glm::vec3(target_pitch, target_yaw, target_roll));
    float s = 1.0f - std::exp(-9.0f * dt);
    state.orientation = glm::normalize(glm::slerp(state.orientation, target_q, s));

    state.angular_vel *= std::exp(-5.5f * dt);
}

static void update_enemies(std::vector<EnemyAircraft>& enemies,
                           const AircraftState& player,
                           float dt,
                           float t_now,
                           int quadrant) {
    glm::vec3 player_fwd = glm::normalize(glm::mat3_cast(player.orientation) * glm::vec3(0, 0, -1));
    for (size_t i = 0; i < enemies.size(); ++i) {
        EnemyAircraft& e = enemies[i];
        e.position += e.velocity * dt;
        e.orientation = orientation_from_forward(e.velocity);

        glm::vec3 rel = e.position - player.position;
        float ahead = glm::dot(rel, player_fwd);
        if (ahead < -500.0f ||
            glm::length(rel) > 4800.0f ||
            e.position.y < 140.0f ||
            e.position.y > player.position.y + 900.0f) {
            respawn_enemy(e, player, (int)i, t_now, quadrant);
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
    glm::vec3 up      = world_from_body * glm::vec3(0, 1, 0);

    Projectile b;
    b.position = aircraft.position + forward * 1.8f - up * 1.4f;
    b.velocity = aircraft.velocity + forward * 12.0f - up * 5.0f;
    b.ttl = 18.0f;
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
        b.velocity += glm::vec3(0.0f, -9.81f, 0.0f) * dt;
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

static float wrap_pi(float a) {
    while (a > glm::pi<float>()) a -= glm::two_pi<float>();
    while (a < -glm::pi<float>()) a += glm::two_pi<float>();
    return a;
}

static AttitudeCommand build_combat_assist_command(const AircraftState& state,
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
        float score = dist - 0.35f * ahead; // 偏好更近且在前方的目标
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

    // 航向控制基于机头方向，避免速度向量侧滑时引入抖动
    glm::vec3 forward = glm::normalize(glm::mat3_cast(state.orientation) * glm::vec3(0, 0, -1));
    float speed = glm::length(state.velocity);

    float current_heading = std::atan2(forward.x, -forward.z);
    float desired_heading = std::atan2(to_target.x, -to_target.z);
    float heading_error = wrap_pi(desired_heading - current_heading);
    if (std::abs(heading_error) < glm::radians(1.2f)) {
        heading_error = 0.0f;  // 死区，防止小角度来回抖动
    }

    float horiz_dist = glm::length(glm::vec2(to_target.x, to_target.z));
    float desired_pitch = std::atan2(to_target.y, glm::max(horiz_dist, 1.0f));
    desired_pitch = glm::clamp(desired_pitch, glm::radians(-12.0f), glm::radians(12.0f));

    float desired_roll = glm::clamp(heading_error * 1.05f,
                                    glm::radians(-28.0f), glm::radians(28.0f));
    // 协调转弯补偿：有滚转时给少量抬头，减小转弯掉高
    desired_pitch += 0.10f * std::abs(desired_roll);
    // 角速度阻尼，减小高频抽搐
    desired_pitch -= 0.22f * state.angular_vel.x;  // pitch rate q
    desired_roll  -= 0.28f * state.angular_vel.z;  // roll rate p

    // 非机动阶段倾向于摆正姿态
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

// 硬性保底防下坠：高度过低或下降过快时，强制抬头+加油门+改平
static void apply_sink_guard(const AircraftState& state, float dt, AttitudeCommand& cmd) {
    float alt = state.position.y;
    float sink_rate = -state.velocity.y; // >0 表示正在下坠

    // 渐进触发：越低、下坠越快，保护越强
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

    // 硬触发：接近地面时直接进入最大恢复
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

// 状态级硬保护：即便控制器瞬时失效，也尽量防止继续下坠到撞地
static void apply_hard_safety_floor(AircraftState& state, float dt) {
    float alt = state.position.y;

    if (alt < 520.0f) {
        float g = glm::clamp((520.0f - alt) / 420.0f, 0.0f, 1.0f);

        // 低空逐步限制最大下沉率
        float min_vy = glm::mix(-18.0f, -1.8f, g);
        if (state.velocity.y < min_vy) {
            state.velocity.y = slew_to(state.velocity.y, min_vy, 62.0f * dt);
        }

        // 低空抑制俯仰/滚转角速度，减少抽搐和翻滚带来的失控下坠
        float damp = std::exp(-3.6f * g * dt);
        state.angular_vel.x *= damp;
        state.angular_vel.z *= damp;
    }

    // 临近地面时，强制把垂向速度拉回到非负
    if (alt < 140.0f && state.velocity.y < 0.0f) {
        state.velocity.y = slew_to(state.velocity.y, 0.0f, 95.0f * dt);
    }

    // 最终硬地板：保证不会撞地（可按需调整高度）
    const float HARD_ALT_FLOOR = 45.0f;
    if (state.position.y < HARD_ALT_FLOOR) {
        state.position.y = HARD_ALT_FLOOR;
        if (state.velocity.y < 0.0f) state.velocity.y = 0.0f;
    }
}

// ── 带惯性的追踪相机（不完全绑定机体，避免“原地打转”观感）─────────────────────

static void update_chase_camera(const AircraftState& state,
                                float dt,
                                glm::vec3& eye,
                                glm::vec3& target) {
    glm::mat3 world_from_body = glm::mat3_cast(state.orientation);
    glm::vec3 forward = glm::normalize(world_from_body * glm::vec3(0, 0, -1));
    glm::vec3 up = glm::normalize(world_from_body * glm::vec3(0, 1, 0));
    glm::vec3 right = glm::normalize(world_from_body * glm::vec3(1, 0, 0));

    // 尾翼锚点（机体局部坐标，接近垂尾根部）
    glm::vec3 tail_anchor_local(0.0f, 0.70f, 3.20f);
    glm::vec3 tail_anchor = state.position + world_from_body * tail_anchor_local;

    // 隐形自拍杆：从尾翼锚点向后并略高，轻微右偏避免模型遮挡
    glm::vec3 desired_eye = tail_anchor - forward * 44.0f + up * 9.0f + right * 1.5f;
    glm::vec3 desired_target = tail_anchor + forward * 65.0f + up * 1.5f;

    static bool initialized = false;
    static glm::vec3 smoothed_eye;
    static glm::vec3 smoothed_target;
    if (!initialized) {
        smoothed_eye = desired_eye;
        smoothed_target = desired_target;
        initialized = true;
    }

    float smooth = 1.0f - std::exp(-7.0f * dt);
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
    WireMesh bomb_mesh = make_bomb_mesh();
    WireMesh bullet_mesh = make_bullet_mesh();
    WireMesh cloud_mesh = make_cloud_mesh();

    // 飞行状态初始化
    AircraftState state;
    state.position    = START_POS;                  // 1000m高度
    state.velocity    = glm::vec3(0, 0, -150.0f);   // 初速150m/s向前
    state.orientation = glm::quat(1, 0, 0, 0);      // 水平飞行
    state.angular_vel = glm::vec3(0, 0, 0);

    AttitudeCommand command;
    // --- Real Physics Path (reserved for future expansion) ---
    // FlightDynamics  dynamics;
    // FlightController controller;
    command.throttle = 0.6f;
    std::vector<Projectile> bullets;
    std::vector<Projectile> bombs;
    std::vector<EnemyAircraft> enemies(6);
    std::vector<Cloud> clouds(14);
    std::vector<AABattery> aa_batteries;
    std::vector<AAShell> aa_shells;
    int player_hp = 100;
    int score = 0;
    int muzzle_index = 0;
    const double hold_fire_threshold_s = 0.22;
    const double gun_interval_s = 1.0 / 18.0;
    double gun_timer_s = 0.0;

    double last_time = glfwGetTime();
    double accumulator = 0.0;
    bool crashed = false;

    printf("Fighter Sim 启动\n");
    printf("OpenGL: %s\n", glGetString(GL_VERSION));
    for (size_t i = 0; i < enemies.size(); ++i) {
        respawn_enemy(enemies[i], state, (int)i, 0.0f, g_motion_quadrant);
    }
    for (size_t i = 0; i < clouds.size(); ++i) {
        clouds[i].position = state.position + glm::vec3(
            ((int)(i % 7) - 3) * 220.0f,
            920.0f + 200.0f * (i % 5),
            -1200.0f - 240.0f * (i / 2)
        );
    }
    for (int i = 0; i < 10; ++i) {
        AABattery b;
        b.position = glm::vec3(-1800.0f + i * 380.0f, 0.0f, -2200.0f - i * 520.0f);
        b.cooldown_s = 0.35f * i;
        aa_batteries.push_back(b);
    }

    printf("模式: 横版射击(3D表现)\n");
    printf("操控: W/S=上下移动 A/D=左右移动 Q/E=轻微偏航姿态 Shift/Ctrl=油门 Space短按=炸弹 长按=机枪 ESC=退出\n\n");

    // ── 主循环 ────────────────────────────────────────────────────────────────

    while (!glfwWindowShouldClose(window)) {
        double now = glfwGetTime();
        double frame_time = now - last_time;
        last_time = now;
        if (frame_time > 0.1) frame_time = 0.1;   // 防止死机时间爆炸
        accumulator += frame_time;

        glfwPollEvents();

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

        AttitudeCommand base_command =
            build_combat_assist_command(state, enemies, (float)frame_time, command);
        command = base_command;
        apply_player_intervention(command, base_command, (float)frame_time);
        apply_sink_guard(state, (float)frame_time, command);

        // 固定步长物理更新
        while (accumulator >= SIM_DT) {
            // --- Real Physics Path (reserved for future expansion) ---
            // apply_direct_player_attitude_control(state, SIM_DT);
            // ControlInput ctrl = controller.update(state, command, SIM_DT);
            // dynamics.step(state, ctrl, SIM_DT);
            // keep_forward_flight(state, SIM_DT);

            apply_side_scroller_motion(state, SIM_DT);
            apply_hard_safety_floor(state, SIM_DT);
            update_projectiles(bullets, bombs, SIM_DT);
            update_ground_aa(aa_batteries, aa_shells, state, SIM_DT);
            accumulator -= SIM_DT;
        }

        update_enemies(enemies, state, (float)frame_time, (float)now, g_motion_quadrant);
        update_clouds(clouds, state, (float)now);

        // 命中检测：机枪打敌机 + 地面防空打玩家
        for (auto it_b = bullets.begin(); it_b != bullets.end();) {
            bool hit = false;
            for (size_t i = 0; i < enemies.size(); ++i) {
                if (glm::distance(it_b->position, enemies[i].position) < 26.0f) {
                    respawn_enemy(enemies[i], state, (int)i, (float)now, g_motion_quadrant);
                    score += 10;
                    hit = true;
                    break;
                }
            }
            if (hit) it_b = bullets.erase(it_b);
            else ++it_b;
        }
        for (auto it_s = aa_shells.begin(); it_s != aa_shells.end();) {
            if (glm::distance(it_s->position, state.position) < 24.0f) {
                player_hp -= 8;
                it_s = aa_shells.erase(it_s);
            } else {
                ++it_s;
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

        // 追踪相机
        glm::vec3 cam_pos, cam_target;
        update_chase_camera(state, (float)frame_time, cam_pos, cam_target);
        renderer.set_camera(cam_pos, cam_target);

        // 渲染
        renderer.begin_frame();
        renderer.draw_ground_grid(8000.0f, 200.0f, 0.0f);
        renderer.draw_axes(80.0f);
        for (const auto& c : clouds) {
            renderer.draw_mesh(cloud_mesh, c.position, glm::quat(1, 0, 0, 0), {0.86f, 0.88f, 0.91f});
        }
        renderer.draw_mesh(fighter_mesh, state.position, state.orientation,
                           {0.08f, 0.10f, 0.14f});   // 白底下清晰的深色线框
        for (const auto& e : enemies) {
            renderer.draw_mesh(fighter_mesh, e.position, e.orientation, {0.72f, 0.12f, 0.10f});
        }
        const glm::quat identity_q(1.0f, 0.0f, 0.0f, 0.0f);
        for (const auto& b : bombs) {
            renderer.draw_mesh(bomb_mesh, b.position, identity_q, {0.78f, 0.10f, 0.08f});
        }
        for (const auto& p : bullets) {
            renderer.draw_mesh(bullet_mesh, p.position, identity_q, {0.08f, 0.08f, 0.08f});
        }
        for (const auto& s : aa_shells) {
            renderer.draw_mesh(bullet_mesh, s.position, identity_q, {0.78f, 0.18f, 0.10f});
        }
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
                      "Fighter Sim | GAME | Q%d | HP %d | Score %d | Alt %.0f m | V %.0f m/s | Vy %.1f m/s | AoA %.1f deg | Thr %.0f%% | Bullets %zu | Bombs %zu | AA %zu",
                      g_motion_quadrant, player_hp, score,
                      state.position.y, glm::length(state.velocity), state.velocity.y,
                      aoa_deg, command.throttle * 100.0f,
                      bullets.size(), bombs.size(), aa_shells.size());
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
