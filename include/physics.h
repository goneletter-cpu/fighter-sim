#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// 飞机控制输入（归一化到 -1 ~ 1）
struct ControlInput {
    float elevator;   // 升降舵 → 俯仰
    float aileron;    // 副翼   → 滚转
    float rudder;     // 方向舵 → 偏航
    float throttle;   // 油门   → 推力 (0 ~ 1)
};

// 飞机完整状态
struct AircraftState {
    glm::vec3 position;       // 世界坐标位置 (m)
    glm::vec3 velocity;       // 世界坐标速度 (m/s)
    glm::quat orientation;    // 姿态四元数（避免万向节死锁）
    glm::vec3 angular_vel;    // 机体坐标系角速率 (rad/s)

    // 从四元数提取欧拉角（仅用于PID误差计算和显示）
    glm::vec3 euler_angles() const;  // 返回 (pitch, roll, yaw) rad
};

// 简化的六自由度动力学
// 不需要真实气动数据，用近似参数
class FlightDynamics {
public:
    FlightDynamics();

    // RK4积分一步
    void step(AircraftState& state, const ControlInput& ctrl, float dt);

    // 飞机参数（可以后续调整）
    float mass         = 9000.0f;   // kg，约F-16空重
    float max_thrust   = 76000.0f;  // N
    float wing_area    = 28.0f;     // m²

private:
    // 计算某状态下的导数（力和力矩）
    struct Derivative {
        glm::vec3 dpos;
        glm::vec3 dvel;
        glm::quat dorientation;
        glm::vec3 dangular_vel;
    };

    Derivative compute_derivative(const AircraftState& s,
                                  const ControlInput& ctrl) const;

    // 简化气动力（线性近似）
    glm::vec3 aero_force(const AircraftState& s, const ControlInput& ctrl) const;
    glm::vec3 aero_torque(const AircraftState& s, const ControlInput& ctrl) const;

    // 转动惯量（对角张量近似）
    glm::vec3 inertia_ = {12875.0f, 75674.0f, 85552.0f}; // Ixx Iyy Izz (kg·m²)
};
