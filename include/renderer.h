#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// 线框网格：顶点对列表
struct WireMesh {
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> line_indices;  // 每两个索引构成一条线段
};

// 简单的OpenGL线框渲染器
class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();

    bool init();
    void begin_frame();
    void end_frame();

    // 用姿态四元数和位置渲染一个线框模型
    void draw_mesh(const WireMesh& mesh,
                   const glm::vec3& position,
                   const glm::quat& orientation,
                   const glm::vec3& color = {0.0f, 1.0f, 0.0f});
    void draw_mesh_scaled(const WireMesh& mesh,
                          const glm::vec3& position,
                          const glm::quat& orientation,
                          float uniform_scale,
                          const glm::vec3& color = {0.0f, 1.0f, 0.0f});

    // 画世界坐标轴（调试用）
    void draw_axes(float length = 5.0f);
    void draw_ground_grid(float extent = 5000.0f,
                          float step = 200.0f,
                          float y = 0.0f);

    // 调试HUD：在左上角打印姿态数值
    // （用简单的OpenGL线条字体，不引入额外库）
    void draw_hud(const glm::vec3& euler_deg,
                  const glm::vec3& position,
                  const glm::vec3& velocity,
                  const glm::vec3& angular_vel_deg_s,
                  float throttle,
                  float aoa_deg,
                  float beta_deg);
    void draw_radar(const glm::vec3& player_pos,
                    const std::vector<glm::vec3>& enemy_positions,
                    const std::vector<glm::vec3>& enemy_velocities,
                    const glm::vec3& player_velocity,
                    float radar_range_world = 1200.0f);
    void draw_attitude_gauge(const glm::vec3& euler_deg);

    // 相机控制
    void set_camera(const glm::vec3& eye,
                    const glm::vec3& target,
                    const glm::vec3& up = {0, 1, 0});

    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_, height_;

    unsigned int shader_program_;
    unsigned int vao_, vbo_, ebo_;

    glm::mat4 view_;
    glm::mat4 projection_;

    bool compile_shaders();
};

// 工厂函数：生成简化战斗机线框模型（约40条线）
WireMesh make_fighter_mesh();
WireMesh make_fighter_mesh_variant(float gear_deploy, float flap_deploy);
WireMesh make_carrier_mesh();
