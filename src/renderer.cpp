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

    // F-16风格修长线框（近似比例）
    m.vertices = {
        // 机身中心线
        { 0.00f,  0.00f, -7.20f}, // 0 机头
        { 0.00f,  0.35f, -5.60f}, // 1 风挡前
        { 0.00f,  0.62f, -4.20f}, // 2 座舱中
        { 0.00f,  0.68f, -2.00f}, // 3 背脊前
        { 0.00f,  0.42f,  1.60f}, // 4 背脊后
        { 0.00f,  0.15f,  4.80f}, // 5 喷口上缘
        { 0.00f,  0.00f,  6.00f}, // 6 尾端

        // 机身两侧与进气道
        {-0.35f,  0.05f, -6.00f}, // 7 左机鼻棱线
        { 0.35f,  0.05f, -6.00f}, // 8 右机鼻棱线
        {-0.95f, -0.15f, -4.40f}, // 9 左进气口前缘
        { 0.95f, -0.15f, -4.40f}, // 10 右进气口前缘
        {-0.85f, -0.22f, -2.80f}, // 11 左进气口后缘
        { 0.85f, -0.22f, -2.80f}, // 12 右进气口后缘
        {-0.75f, -0.10f, -0.60f}, // 13 左机身中段
        { 0.75f, -0.10f, -0.60f}, // 14 右机身中段
        {-0.58f, -0.05f,  2.50f}, // 15 左尾段
        { 0.58f, -0.05f,  2.50f}, // 16 右尾段
        { 0.00f, -0.48f, -4.80f}, // 17 机腹前
        { 0.00f, -0.62f, -1.20f}, // 18 机腹中
        { 0.00f, -0.46f,  3.80f}, // 19 机腹后

        // LERX 与主翼
        {-1.55f,  0.02f, -3.20f}, // 20 左LERX
        { 1.55f,  0.02f, -3.20f}, // 21 右LERX
        {-1.05f, -0.03f, -2.20f}, // 22 左翼前缘根
        { 1.05f, -0.03f, -2.20f}, // 23 右翼前缘根
        {-4.45f, -0.18f,  0.60f}, // 24 左翼尖
        { 4.45f, -0.18f,  0.60f}, // 25 右翼尖
        {-1.55f, -0.08f,  1.20f}, // 26 左翼后缘根
        { 1.55f, -0.08f,  1.20f}, // 27 右翼后缘根

        // 全动平尾
        {-0.72f,  0.05f,  4.20f}, // 28 左平尾根
        { 0.72f,  0.05f,  4.20f}, // 29 右平尾根
        {-2.25f,  0.08f,  5.25f}, // 30 左平尾尖
        { 2.25f,  0.08f,  5.25f}, // 31 右平尾尖

        // 垂尾
        { 0.00f,  0.70f,  3.20f}, // 32 垂尾前缘根
        { 0.00f,  2.15f,  4.35f}, // 33 垂尾顶
        { 0.00f,  0.55f,  5.55f}, // 34 垂尾后缘根

        // 座舱细节
        {-0.42f,  0.38f, -4.90f}, // 35 左座舱边
        { 0.42f,  0.38f, -4.90f}, // 36 右座舱边
        {-0.34f,  0.52f, -3.30f}, // 37 左座舱后
        { 0.34f,  0.52f, -3.30f}, // 38 右座舱后

        // 腹鳍与喷口
        {-0.48f, -0.62f,  4.95f}, // 39 左腹鳍
        { 0.48f, -0.62f,  4.95f}, // 40 右腹鳍
        {-0.42f,  0.00f,  5.35f}, // 41 喷口左
        { 0.42f,  0.00f,  5.35f}, // 42 喷口右
        { 0.00f, -0.32f,  5.35f}, // 43 喷口下
    };

    // 线段索引（每两个构成一条线）
    m.line_indices = {
        // 机身中心线
        0,1, 1,2, 2,3, 3,4, 4,5, 5,6,

        // 机鼻/进气道/机身侧轮廓
        0,7, 0,8, 7,8,
        7,9, 8,10, 9,10,
        9,11, 10,12, 11,12,
        11,13, 12,14, 13,14,
        13,15, 14,16, 15,16,

        // 机腹体积线
        0,17, 17,18, 18,19, 19,6,
        17,9, 17,10, 18,13, 18,14, 19,15, 19,16,

        // 座舱框架
        1,35, 1,36, 35,37, 36,38, 37,38, 37,2, 38,2,

        // LERX 和主翼
        20,21,
        20,22, 21,23,
        20,24, 21,25,
        22,24, 23,25,
        22,26, 23,27,
        26,24, 27,25,
        24,25, 26,27,
        26,15, 27,16,

        // 平尾
        28,29, 28,30, 29,31, 30,31, 28,4, 29,4,

        // 垂尾
        32,33, 33,34, 34,32, 32,4, 34,6,

        // 喷口/尾段
        5,41, 5,42, 41,42, 41,43, 42,43, 19,43,

        // 腹鳍
        39,19, 40,19, 39,6, 40,6, 39,40,

        // 纵向加强线（提高立体感）
        3,13, 3,14, 4,15, 4,16,
    };

    return m;
}
