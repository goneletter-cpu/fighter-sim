#include "renderer.h"
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB   // 让 GLFW 包含 Core Profile 头文件（有 VAO 等）
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <vector>

// ── Shader源码（内嵌，不需要外部文件）────────────────────────────────────────

static const char* VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

static const char* FRAG_SRC = R"(
#version 330 core
uniform vec3 color;
out vec4 FragColor;

void main() {
    FragColor = vec4(color, 1.0);
}
)";

// ── 着色器编译工具 ─────────────────────────────────────────────────────────────

static unsigned int compile_shader(unsigned int type, const char* src) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    int ok;
    glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(id, 512, nullptr, log);
        printf("[Shader Error] %s\n", log);
    }
    return id;
}

// ── Renderer ──────────────────────────────────────────────────────────────────

Renderer::Renderer(int width, int height)
    : width_(width), height_(height),
      shader_program_(0), vao_(0), vbo_(0), ebo_(0)
{}

Renderer::~Renderer() {
    glDeleteProgram(shader_program_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
}

bool Renderer::init() {
    if (!compile_shaders()) return false;

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    projection_ = glm::perspective(glm::radians(60.0f),
                                   (float)width_ / height_,
                                   0.1f, 10000.0f);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.5f);
    return true;
}

bool Renderer::compile_shaders() {
    unsigned int vs = compile_shader(GL_VERTEX_SHADER,   VERT_SRC);
    unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, FRAG_SRC);

    shader_program_ = glCreateProgram();
    glAttachShader(shader_program_, vs);
    glAttachShader(shader_program_, fs);
    glLinkProgram(shader_program_);
    glDeleteShader(vs);
    glDeleteShader(fs);

    int ok;
    glGetProgramiv(shader_program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(shader_program_, 512, nullptr, log);
        printf("[Program Error] %s\n", log);
        return false;
    }
    return true;
}

void Renderer::set_camera(const glm::vec3& eye,
                           const glm::vec3& target,
                           const glm::vec3& up) {
    view_ = glm::lookAt(eye, target, up);
}

void Renderer::begin_frame() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(shader_program_);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"),       1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
}

void Renderer::end_frame() {
    // 交换缓冲由main负责（glfwSwapBuffers）
}

void Renderer::draw_mesh(const WireMesh& mesh,
                          const glm::vec3& position,
                          const glm::quat& orientation,
                          const glm::vec3& color)
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                    * glm::mat4_cast(orientation);

    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(color));

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 mesh.vertices.size() * sizeof(glm::vec3),
                 mesh.vertices.data(),
                 GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 mesh.line_indices.size() * sizeof(unsigned int),
                 mesh.line_indices.data(),
                 GL_DYNAMIC_DRAW);

    glDrawElements(GL_LINES, (int)mesh.line_indices.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Renderer::draw_axes(float length) {
    // X轴红、Y轴绿、Z轴蓝
    struct AxisLine { glm::vec3 a, b; glm::vec3 color; };
    AxisLine axes[] = {
        {{0,0,0},{length,0,0},{1,0,0}},
        {{0,0,0},{0,length,0},{0,1,0}},
        {{0,0,0},{0,0,length},{0,0,1}},
    };

    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));

    for (auto& ax : axes) {
        glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(ax.color));
        glm::vec3 verts[] = {ax.a, ax.b};
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, 2);
        glBindVertexArray(0);
    }
}

void Renderer::draw_ground_grid(float extent, float step, float y) {
    if (step <= 1.0f || extent <= step) return;

    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));
    glUniform3f(glGetUniformLocation(shader_program_, "color"), 0.80f, 0.83f, 0.88f);

    std::vector<glm::vec3> lines;
    int line_count = (int)(extent / step);
    lines.reserve((line_count * 4 + 4) * 2);

    for (int i = -line_count; i <= line_count; ++i) {
        float c = i * step;
        lines.push_back(glm::vec3(-extent, y, c));
        lines.push_back(glm::vec3( extent, y, c));
        lines.push_back(glm::vec3(c, y, -extent));
        lines.push_back(glm::vec3(c, y,  extent));
    }

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 lines.size() * sizeof(glm::vec3),
                 lines.data(),
                 GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, (int)lines.size());
    glBindVertexArray(0);
}

void Renderer::draw_hud(const glm::vec3& euler_deg,
                         const glm::vec3& position,
                         const glm::vec3& velocity,
                         const glm::vec3& angular_vel_deg_s,
                         float throttle,
                         float aoa_deg,
                         float beta_deg)
{
    float airspeed = glm::length(velocity);
    printf("\rP:%6.1f R:%6.1f Y:%6.1f deg | Alt:%8.1fm V:%6.1fm/s Vy:%6.1fm/s | AoA:%6.1f Beta:%6.1f deg | p:%6.1f q:%6.1f r:%6.1f deg/s | Thr:%5.1f%% | Pos:(%7.1f,%7.1f,%7.1f)   ",
           euler_deg.x, euler_deg.z, euler_deg.y,
           position.y, airspeed, velocity.y,
           aoa_deg, beta_deg,
           angular_vel_deg_s.x, angular_vel_deg_s.y, angular_vel_deg_s.z,
           throttle * 100.0f,
           position.x, position.y, position.z);
    fflush(stdout);
}

// ── 战斗机线框模型 ────────────────────────────────────────────────────────────
// 坐标约定：机头朝-Z，机翼沿X，机腹朝-Y

WireMesh make_fighter_mesh() {
    WireMesh m;
    // 简约客机风格：圆润机身 + 基本翼面，减少线条但保留体积感
    const std::vector<float> zs = {-6.8f, -4.2f, -1.2f, 2.0f, 5.2f};
    const std::vector<float> rx = {0.12f, 0.90f, 1.05f, 0.85f, 0.45f};
    const std::vector<float> ry = {0.12f, 0.95f, 1.08f, 0.90f, 0.50f};
    const int ring_pts = 8;

    // 机头与机尾尖点
    m.vertices.push_back({0.0f, 0.0f, -7.5f}); // 0 nose
    m.vertices.push_back({0.0f, 0.0f,  6.2f}); // 1 tail

    // 椭圆截面环
    const int ring_start = (int)m.vertices.size();
    for (size_t r = 0; r < zs.size(); ++r) {
        for (int i = 0; i < ring_pts; ++i) {
            float t = (2.0f * 3.1415926f * i) / ring_pts;
            m.vertices.push_back({
                rx[r] * std::cos(t),
                ry[r] * std::sin(t),
                zs[r]
            });
        }
    }

    auto ring_idx = [&](int r, int i) { return ring_start + r * ring_pts + (i % ring_pts); };

    // 每个截面闭环
    for (int r = 0; r < (int)zs.size(); ++r) {
        for (int i = 0; i < ring_pts; ++i) {
            m.line_indices.push_back(ring_idx(r, i));
            m.line_indices.push_back(ring_idx(r, i + 1));
        }
    }

    // 环与环之间连接，形成圆润“球体感”骨架
    for (int r = 0; r < (int)zs.size() - 1; ++r) {
        for (int i = 0; i < ring_pts; ++i) {
            m.line_indices.push_back(ring_idx(r, i));
            m.line_indices.push_back(ring_idx(r + 1, i));
        }
    }

    // 机头/机尾与最近截面连接
    for (int i = 0; i < ring_pts; ++i) {
        m.line_indices.push_back(0);
        m.line_indices.push_back(ring_idx(0, i));
        m.line_indices.push_back(1);
        m.line_indices.push_back(ring_idx((int)zs.size() - 1, i));
    }

    // 简化机翼
    const unsigned int wing_root_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.9f, -0.05f, -0.6f});
    const unsigned int wing_root_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.9f, -0.05f, -0.6f});
    const unsigned int wing_tip_l  = (unsigned int)m.vertices.size(); m.vertices.push_back({-4.2f, -0.12f,  0.9f});
    const unsigned int wing_tip_r  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 4.2f, -0.12f,  0.9f});
    const unsigned int wing_back_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.4f, -0.08f,  1.5f});
    const unsigned int wing_back_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.4f, -0.08f,  1.5f});

    m.line_indices.insert(m.line_indices.end(), {
        wing_root_l, wing_tip_l, wing_tip_l, wing_back_l, wing_back_l, wing_root_l,
        wing_root_r, wing_tip_r, wing_tip_r, wing_back_r, wing_back_r, wing_root_r,
        wing_root_l, wing_root_r
    });

    // 简化平尾 + 垂尾
    const unsigned int tail_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.9f, 0.06f, 5.0f});
    const unsigned int tail_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.9f, 0.06f, 5.0f});
    const unsigned int tail_c = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.0f, 0.08f, 4.6f});
    const unsigned int fin_t  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.0f, 1.45f, 5.05f});

    m.line_indices.insert(m.line_indices.end(), {
        tail_c, tail_l, tail_c, tail_r, tail_l, tail_r,
        tail_c, fin_t
    });

    return m;
}
