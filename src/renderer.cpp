#include "renderer.h"
#define GL_SILENCE_DEPRECATION
#define GLFW_INCLUDE_GLCOREARB   //  translated comment
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>

// ── Shader translated comment

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

// ──  translated comment

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

    projection_ = glm::perspective(glm::radians(44.0f),
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
    //  translated comment
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

void Renderer::draw_mesh_scaled(const WireMesh& mesh,
                                const glm::vec3& position,
                                const glm::quat& orientation,
                                float uniform_scale,
                                const glm::vec3& color)
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                    * glm::mat4_cast(orientation)
                    * glm::scale(glm::mat4(1.0f), glm::vec3(uniform_scale));

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
    // X translated comment
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
                         float game_speed_scale,
                         float aoa_deg,
                         float beta_deg)
{
    float airspeed = glm::length(velocity);

    glm::mat4 saved_view = view_;
    glm::mat4 saved_proj = projection_;

    view_ = glm::mat4(1.0f);
    projection_ = glm::ortho(0.0f, (float)width_, 0.0f, (float)height_, -1.0f, 1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));

    auto draw_lines = [&](const std::vector<glm::vec3>& lines, const glm::vec3& color) {
        if (lines.empty()) return;
        glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(color));
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     lines.size() * sizeof(glm::vec3),
                     lines.data(),
                     GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, (int)lines.size());
        glBindVertexArray(0);
    };

    auto add_seg = [](float x0, float y0, float x1, float y1, std::vector<glm::vec3>& out) {
        out.push_back({x0, y0, 0.0f});
        out.push_back({x1, y1, 0.0f});
    };

    auto add_char = [&](char c, float x, float y, float s, std::vector<glm::vec3>& out) {
        float w = s;
        float h = s * 1.6f;
        float x0 = x;
        float x1 = x + w;
        float y0 = y;
        float y1 = y + h;
        float ym = y + h * 0.5f;

        auto seg = [&](char id) {
            switch (id) {
                case 'A': add_seg(x0, y1, x1, y1, out); break;
                case 'B': add_seg(x1, y1, x1, ym, out); break;
                case 'C': add_seg(x1, ym, x1, y0, out); break;
                case 'D': add_seg(x0, y0, x1, y0, out); break;
                case 'E': add_seg(x0, ym, x0, y0, out); break;
                case 'F': add_seg(x0, y1, x0, ym, out); break;
                case 'G': add_seg(x0, ym, x1, ym, out); break;
                default: break;
            }
        };

        switch (c) {
            case '0': seg('A'); seg('B'); seg('C'); seg('D'); seg('E'); seg('F'); break;
            case '1': seg('B'); seg('C'); break;
            case '2': seg('A'); seg('B'); seg('G'); seg('E'); seg('D'); break;
            case '3': seg('A'); seg('B'); seg('G'); seg('C'); seg('D'); break;
            case '4': seg('F'); seg('G'); seg('B'); seg('C'); break;
            case '5': seg('A'); seg('F'); seg('G'); seg('C'); seg('D'); break;
            case '6': seg('A'); seg('F'); seg('G'); seg('E'); seg('C'); seg('D'); break;
            case '7': seg('A'); seg('B'); seg('C'); break;
            case '8': seg('A'); seg('B'); seg('C'); seg('D'); seg('E'); seg('F'); seg('G'); break;
            case '9': seg('A'); seg('B'); seg('C'); seg('D'); seg('F'); seg('G'); break;
            case 'A': seg('A'); seg('B'); seg('C'); seg('E'); seg('F'); seg('G'); break;
            case 'B': seg('F'); seg('E'); seg('G'); seg('A'); seg('D'); seg('B'); seg('C'); break;
            case 'D': seg('A'); seg('B'); seg('C'); seg('D'); seg('E'); seg('F'); break;
            case 'E': seg('A'); seg('F'); seg('G'); seg('E'); seg('D'); break;
            case 'H': seg('F'); seg('E'); seg('G'); seg('B'); seg('C'); break;
            case 'L': seg('F'); seg('E'); seg('D'); break;
            case 'O': seg('A'); seg('B'); seg('C'); seg('D'); seg('E'); seg('F'); break;
            case 'P': seg('A'); seg('B'); seg('F'); seg('G'); seg('E'); break;
            case 'R': seg('A'); seg('B'); seg('F'); seg('G'); seg('E'); add_seg(x0, ym, x1, y0, out); break;
            case 'S': seg('A'); seg('F'); seg('G'); seg('C'); seg('D'); break;
            case 'T': seg('A'); add_seg(x0 + w * 0.5f, y1, x0 + w * 0.5f, y0, out); break;
            case 'V': add_seg(x0, y1, x0 + w * 0.5f, y0, out); add_seg(x1, y1, x0 + w * 0.5f, y0, out); break;
            case 'Y': add_seg(x0, y1, x0 + w * 0.5f, ym, out); add_seg(x1, y1, x0 + w * 0.5f, ym, out); add_seg(x0 + w * 0.5f, ym, x0 + w * 0.5f, y0, out); break;
            case '%': add_seg(x0, y1, x1, y0, out); add_seg(x0 + w * 0.15f, y1, x0 + w * 0.35f, y1, out); add_seg(x1 - w * 0.35f, y0, x1 - w * 0.15f, y0, out); break;
            case '-': add_seg(x0, ym, x1, ym, out); break;
            case '.': add_seg(x1 - w * 0.1f, y0, x1, y0, out); break;
            case ':': add_seg(x0 + w * 0.5f, y0 + h * 0.3f, x0 + w * 0.5f, y0 + h * 0.3f, out); add_seg(x0 + w * 0.5f, y0 + h * 0.7f, x0 + w * 0.5f, y0 + h * 0.7f, out); break;
            default: break;
        }
    };

    auto draw_text = [&](const std::string& text, float x, float y, float size, const glm::vec3& color) {
        std::vector<glm::vec3> lines;
        float cursor = x;
        for (char c : text) {
            if (c == ' ') {
                cursor += size * 0.8f;
                continue;
            }
            add_char((char)std::toupper((unsigned char)c), cursor, y, size, lines);
            cursor += size * 1.1f;
        }
        draw_lines(lines, color);
    };

    char line1[128];
    char line2[128];
    char line3[128];
    std::snprintf(line1, sizeof(line1), "ALT %.0f  SPD %.1f  VY %.1f", position.y, airspeed, velocity.y);
    std::snprintf(line2, sizeof(line2), "AOA %.1f  BETA %.1f  P %.1f Q %.1f R %.1f",
                  aoa_deg, beta_deg, angular_vel_deg_s.x, angular_vel_deg_s.y, angular_vel_deg_s.z);
    std::snprintf(line3, sizeof(line3), "THR %.0f%%  GS %.0f%%", throttle * 100.0f, game_speed_scale * 100.0f);

    const float size = 8.0f;
    const float x = 12.0f;
    float y = height_ - 20.0f;
    draw_text(line1, x, y, size, {0.05f, 0.35f, 0.12f});
    y -= size * 2.2f;
    draw_text(line2, x, y, size, {0.05f, 0.35f, 0.12f});
    y -= size * 2.2f;
    draw_text(line3, x, y, size, {0.05f, 0.35f, 0.12f});

    view_ = saved_view;
    projection_ = saved_proj;
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
}

void Renderer::draw_radar(const glm::vec3& player_pos,
                          const std::vector<glm::vec3>& enemy_positions,
                          const std::vector<glm::vec3>& enemy_velocities,
                          const glm::vec3& player_velocity,
                          float radar_range_world) {
    if (radar_range_world <= 1.0f) return;

    glm::mat4 saved_view = view_;
    glm::mat4 saved_proj = projection_;

    view_ = glm::mat4(1.0f);
    projection_ = glm::ortho(0.0f, (float)width_, 0.0f, (float)height_, -1.0f, 1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));

    auto draw_lines = [&](const std::vector<glm::vec3>& lines, const glm::vec3& color) {
        if (lines.empty()) return;
        glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(color));
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(glm::vec3), lines.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, (int)lines.size());
        glBindVertexArray(0);
    };

    const glm::vec2 center(112.0f, 112.0f);
    const float r_px = 82.0f;
    const int seg = 56;
    const float sector_half = glm::radians(60.0f); //  translated comment
    const float arc_start = glm::half_pi<float>() - sector_half;
    const float arc_end = glm::half_pi<float>() + sector_half;

    glm::vec2 fwd2(player_velocity.x, -player_velocity.z);
    if (glm::length(fwd2) < 1e-3f) fwd2 = glm::vec2(0.0f, 1.0f);
    else fwd2 = glm::normalize(fwd2);
    float fwd_ang = std::atan2(fwd2.y, fwd2.x);
    float rot = glm::half_pi<float>() - fwd_ang; //  translated comment

    std::vector<glm::vec3> ring;
    ring.reserve(seg * 2 + 12);
    for (int i = 0; i < seg; ++i) {
        float u0 = (float)i / seg;
        float u1 = (float)(i + 1) / seg;
        float t0 = glm::mix(arc_start, arc_end, u0);
        float t1 = glm::mix(arc_start, arc_end, u1);
        ring.push_back(glm::vec3(center.x + std::cos(t0) * r_px, center.y + std::sin(t0) * r_px, 0.0f));
        ring.push_back(glm::vec3(center.x + std::cos(t1) * r_px, center.y + std::sin(t1) * r_px, 0.0f));
    }
    ring.push_back(glm::vec3(center.x, center.y, 0.0f));
    ring.push_back(glm::vec3(center.x + std::cos(arc_start) * r_px, center.y + std::sin(arc_start) * r_px, 0.0f));
    ring.push_back(glm::vec3(center.x, center.y, 0.0f));
    ring.push_back(glm::vec3(center.x + std::cos(arc_end) * r_px, center.y + std::sin(arc_end) * r_px, 0.0f));
    draw_lines(ring, glm::vec3(0.11f, 0.28f, 0.15f));

    //  translated comment
    for (int k = 1; k <= 3; ++k) {
        float rr = r_px * (k / 4.0f);
        std::vector<glm::vec3> rr_lines;
        rr_lines.reserve(seg * 2);
        for (int i = 0; i < seg; ++i) {
            float u0 = (float)i / seg;
            float u1 = (float)(i + 1) / seg;
            float t0 = glm::mix(arc_start, arc_end, u0);
            float t1 = glm::mix(arc_start, arc_end, u1);
            rr_lines.push_back(glm::vec3(center.x + std::cos(t0) * rr, center.y + std::sin(t0) * rr, 0.0f));
            rr_lines.push_back(glm::vec3(center.x + std::cos(t1) * rr, center.y + std::sin(t1) * rr, 0.0f));
        }
        draw_lines(rr_lines, glm::vec3(0.09f, 0.21f, 0.12f));
    }

    std::vector<glm::vec3> ticks;
    for (int deg = -60; deg <= 60; deg += 15) {
        float a = glm::half_pi<float>() + glm::radians((float)deg);
        float r0 = r_px * 0.92f;
        float r1 = r_px;
        ticks.push_back(glm::vec3(center.x + std::cos(a) * r0, center.y + std::sin(a) * r0, 0.0f));
        ticks.push_back(glm::vec3(center.x + std::cos(a) * r1, center.y + std::sin(a) * r1, 0.0f));
    }
    draw_lines(ticks, glm::vec3(0.12f, 0.32f, 0.18f));

    std::vector<glm::vec3> spokes;
    for (int deg = -60; deg <= 60; deg += 30) {
        float a = glm::half_pi<float>() + glm::radians((float)deg);
        spokes.push_back(glm::vec3(center.x, center.y, 0.0f));
        spokes.push_back(glm::vec3(center.x + std::cos(a) * r_px * 0.86f, center.y + std::sin(a) * r_px * 0.86f, 0.0f));
    }
    draw_lines(spokes, glm::vec3(0.08f, 0.18f, 0.10f));

    // N/E/S/W  translated comment
    auto add_letter_n = [&](float cx, float cy, float s, std::vector<glm::vec3>& out) {
        out.insert(out.end(), {{cx - s, cy - s, 0}, {cx - s, cy + s, 0},
                               {cx - s, cy + s, 0}, {cx + s, cy - s, 0},
                               {cx + s, cy - s, 0}, {cx + s, cy + s, 0}});
    };
    auto add_letter_e = [&](float cx, float cy, float s, std::vector<glm::vec3>& out) {
        out.insert(out.end(), {{cx - s, cy - s, 0}, {cx - s, cy + s, 0},
                               {cx - s, cy + s, 0}, {cx + s, cy + s, 0},
                               {cx - s, cy, 0}, {cx + s * 0.7f, cy, 0},
                               {cx - s, cy - s, 0}, {cx + s, cy - s, 0}});
    };
    auto add_letter_w = [&](float cx, float cy, float s, std::vector<glm::vec3>& out) {
        out.insert(out.end(), {{cx - s, cy + s, 0}, {cx - s * 0.5f, cy - s, 0},
                               {cx - s * 0.5f, cy - s, 0}, {cx, cy + s * 0.3f, 0},
                               {cx, cy + s * 0.3f, 0}, {cx + s * 0.5f, cy - s, 0},
                               {cx + s * 0.5f, cy - s, 0}, {cx + s, cy + s, 0}});
    };
    std::vector<glm::vec3> labels;
    float ls = 3.6f;
    add_letter_n(center.x, center.y + r_px + 11.0f, ls, labels);
    add_letter_e(center.x + std::cos(arc_start) * (r_px + 13.0f), center.y + std::sin(arc_start) * (r_px + 13.0f), ls, labels);
    add_letter_w(center.x + std::cos(arc_end) * (r_px + 13.0f), center.y + std::sin(arc_end) * (r_px + 13.0f), ls, labels);
    draw_lines(labels, glm::vec3(0.34f, 0.78f, 0.36f));

    //  translated comment
    float phase = std::fmod((float)glfwGetTime() * 0.9f, 1.0f);
    float tri = 1.0f - std::abs(phase * 2.0f - 1.0f);
    float scan_angle = glm::mix(arc_start, arc_end, tri);
    //  translated comment
    for (int i = 0; i < 18; ++i) {
        float a0 = scan_angle - i * 0.05f;
        float a1 = scan_angle - (i + 1) * 0.05f;
        if (a0 < arc_start || a1 < arc_start || a0 > arc_end || a1 > arc_end) continue;
        float k = 1.0f - 0.055f * i;
        if (k < 0.1f) k = 0.1f;
        std::vector<glm::vec3> arc = {
            {center.x + std::cos(a0) * r_px, center.y + std::sin(a0) * r_px, 0.0f},
            {center.x + std::cos(a1) * r_px, center.y + std::sin(a1) * r_px, 0.0f}
        };
        draw_lines(arc, glm::vec3(0.05f, 0.36f * k, 0.15f * k));
    }
    for (int i = 0; i < 9; ++i) {
        float a = scan_angle - i * 0.13f;
        if (a < arc_start || a > arc_end) continue;
        float x = center.x + std::cos(a) * r_px;
        float y = center.y + std::sin(a) * r_px;
        std::vector<glm::vec3> line = {
            {center.x, center.y, 0.0f},
            {x, y, 0.0f}
        };
        float k = 1.0f - 0.10f * i;
        if (k < 0.15f) k = 0.15f;
        draw_lines(line, glm::vec3(0.08f, 0.75f * k, 0.26f * k));
    }

    auto wrap_pi = [](float a) {
        while (a > glm::pi<float>()) a -= glm::two_pi<float>();
        while (a < -glm::pi<float>()) a += glm::two_pi<float>();
        return a;
    };

    std::vector<glm::vec3> blips_dim;
    std::vector<glm::vec3> blips_hot;
    std::vector<glm::vec3> tracks;
    blips_dim.reserve(enemy_positions.size() * 4);
    blips_hot.reserve(enemy_positions.size() * 4);
    tracks.reserve(enemy_positions.size() * 2);
    bool has_nearest = false;
    glm::vec2 nearest_p(0.0f);
    float nearest_d = 1e9f;
    for (size_t i = 0; i < enemy_positions.size(); ++i) {
        const auto& ep = enemy_positions[i];
        glm::vec2 rel(ep.x - player_pos.x, -(ep.z - player_pos.z));
        float d = glm::length(rel);
        if (d > radar_range_world) continue;
        float lx = rel.x * std::cos(rot) - rel.y * std::sin(rot);
        float ly = rel.x * std::sin(rot) + rel.y * std::cos(rot);
        float off = std::atan2(lx, ly);
        if (std::abs(off) > sector_half) continue;
        glm::vec2 p = center + glm::vec2(lx, ly) / radar_range_world * r_px;
        float blip_ang = std::atan2(p.y - center.y, p.x - center.x);
        bool highlighted = std::abs(wrap_pi(blip_ang - scan_angle)) < 0.20f;
        float s = 2.7f;
        auto& out = highlighted ? blips_hot : blips_dim;
        out.push_back(glm::vec3(p.x - s, p.y - s, 0.0f));
        out.push_back(glm::vec3(p.x + s, p.y + s, 0.0f));
        out.push_back(glm::vec3(p.x - s, p.y + s, 0.0f));
        out.push_back(glm::vec3(p.x + s, p.y - s, 0.0f));

        //  translated comment
        glm::vec2 vr(0.0f);
        if (i < enemy_velocities.size()) {
            glm::vec3 vrel3 = enemy_velocities[i] - player_velocity;
            float rvx = vrel3.x * std::cos(rot) + vrel3.z * std::sin(rot);
            float rvy = vrel3.x * std::sin(rot) - vrel3.z * std::cos(rot);
            vr = glm::vec2(rvx, rvy);
        }
        float vlen = glm::length(vr);
        if (vlen > 1e-3f) {
            glm::vec2 dir = vr / vlen;
            float tail = glm::clamp(vlen * 0.04f, 4.0f, 12.0f);
            tracks.push_back(glm::vec3(p.x, p.y, 0.0f));
            tracks.push_back(glm::vec3(p.x + dir.x * tail, p.y + dir.y * tail, 0.0f));
        }

        if (d < nearest_d) {
            nearest_d = d;
            nearest_p = p;
            has_nearest = true;
        }
    }
    if (!blips_dim.empty()) draw_lines(blips_dim, glm::vec3(0.22f, 0.54f, 0.24f));
    if (!tracks.empty()) draw_lines(tracks, glm::vec3(0.35f, 0.70f, 0.36f));
    if (!blips_hot.empty()) {
        float pulse = 0.65f + 0.35f * std::sin((float)glfwGetTime() * 18.0f);
        draw_lines(blips_hot, glm::vec3(0.82f + 0.18f * pulse, 0.86f + 0.14f * pulse, 0.22f + 0.22f * pulse));
    }

    //  translated comment
    if (has_nearest) {
        float b = 8.5f;
        float c = 3.8f;
        std::vector<glm::vec3> lock = {
            {nearest_p.x - b, nearest_p.y + b, 0}, {nearest_p.x - b + c, nearest_p.y + b, 0},
            {nearest_p.x - b, nearest_p.y + b, 0}, {nearest_p.x - b, nearest_p.y + b - c, 0},
            {nearest_p.x + b, nearest_p.y + b, 0}, {nearest_p.x + b - c, nearest_p.y + b, 0},
            {nearest_p.x + b, nearest_p.y + b, 0}, {nearest_p.x + b, nearest_p.y + b - c, 0},
            {nearest_p.x - b, nearest_p.y - b, 0}, {nearest_p.x - b + c, nearest_p.y - b, 0},
            {nearest_p.x - b, nearest_p.y - b, 0}, {nearest_p.x - b, nearest_p.y - b + c, 0},
            {nearest_p.x + b, nearest_p.y - b, 0}, {nearest_p.x + b - c, nearest_p.y - b, 0},
            {nearest_p.x + b, nearest_p.y - b, 0}, {nearest_p.x + b, nearest_p.y - b + c, 0},
        };
        draw_lines(lock, glm::vec3(0.78f, 0.92f, 0.34f));
    }

    std::vector<glm::vec3> player_marker = {
        {center.x, center.y + 4.0f, 0.0f},
        {center.x - 3.5f, center.y - 3.0f, 0.0f},
        {center.x, center.y + 4.0f, 0.0f},
        {center.x + 3.5f, center.y - 3.0f, 0.0f},
        {center.x - 3.5f, center.y - 3.0f, 0.0f},
        {center.x + 3.5f, center.y - 3.0f, 0.0f},
    };
    draw_lines(player_marker, glm::vec3(0.08f, 0.42f, 0.12f));

    //  translated comment
    std::vector<glm::vec3> heading = {
        {center.x, center.y + r_px + 4.0f, 0.0f},
        {center.x - 4.5f, center.y + r_px - 2.0f, 0.0f},
        {center.x, center.y + r_px + 4.0f, 0.0f},
        {center.x + 4.5f, center.y + r_px - 2.0f, 0.0f}
    };
    draw_lines(heading, glm::vec3(0.40f, 0.85f, 0.42f));

    view_ = saved_view;
    projection_ = saved_proj;
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
}

void Renderer::draw_world_map(const glm::vec3& player_pos,
                              const std::vector<glm::vec3>& enemy_positions,
                              const glm::vec3& player_velocity,
                              float map_range_world) {
    if (map_range_world <= 1.0f) return;

    glm::mat4 saved_view = view_;
    glm::mat4 saved_proj = projection_;
    view_ = glm::mat4(1.0f);
    projection_ = glm::ortho(0.0f, (float)width_, 0.0f, (float)height_, -1.0f, 1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));

    auto draw_lines = [&](const std::vector<glm::vec3>& lines, const glm::vec3& color) {
        if (lines.empty()) return;
        glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(color));
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(glm::vec3), lines.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, (int)lines.size());
        glBindVertexArray(0);
    };

    const glm::vec2 center(250.0f, 112.0f);
    const float half = 52.0f;

    std::vector<glm::vec3> frame = {
        {center.x - half, center.y - half, 0}, {center.x + half, center.y - half, 0},
        {center.x + half, center.y - half, 0}, {center.x + half, center.y + half, 0},
        {center.x + half, center.y + half, 0}, {center.x - half, center.y + half, 0},
        {center.x - half, center.y + half, 0}, {center.x - half, center.y - half, 0},
        {center.x - half, center.y, 0}, {center.x + half, center.y, 0},
        {center.x, center.y - half, 0}, {center.x, center.y + half, 0}
    };
    draw_lines(frame, glm::vec3(0.20f, 0.32f, 0.36f));

    std::vector<glm::vec3> enemy_pts;
    enemy_pts.reserve(enemy_positions.size() * 4);
    for (const auto& ep : enemy_positions) {
        glm::vec2 rel(ep.x - player_pos.x, -(ep.z - player_pos.z));
        rel /= map_range_world;
        rel = glm::clamp(rel, glm::vec2(-1.0f), glm::vec2(1.0f));
        glm::vec2 p = center + rel * half;
        float s = 2.2f;
        enemy_pts.push_back({p.x - s, p.y, 0});
        enemy_pts.push_back({p.x + s, p.y, 0});
        enemy_pts.push_back({p.x, p.y - s, 0});
        enemy_pts.push_back({p.x, p.y + s, 0});
    }
    draw_lines(enemy_pts, glm::vec3(0.84f, 0.16f, 0.14f));

    glm::vec2 fwd(player_velocity.x, -player_velocity.z);
    if (glm::length(fwd) < 1e-3f) fwd = glm::vec2(0.0f, 1.0f);
    else fwd = glm::normalize(fwd);
    glm::vec2 right(fwd.y, -fwd.x);
    std::vector<glm::vec3> player = {
        {center.x + fwd.x * 7.0f, center.y + fwd.y * 7.0f, 0.0f},
        {center.x - fwd.x * 4.5f + right.x * 3.5f, center.y - fwd.y * 4.5f + right.y * 3.5f, 0.0f},
        {center.x + fwd.x * 7.0f, center.y + fwd.y * 7.0f, 0.0f},
        {center.x - fwd.x * 4.5f - right.x * 3.5f, center.y - fwd.y * 4.5f - right.y * 3.5f, 0.0f},
        {center.x - fwd.x * 4.5f + right.x * 3.5f, center.y - fwd.y * 4.5f + right.y * 3.5f, 0.0f},
        {center.x - fwd.x * 4.5f - right.x * 3.5f, center.y - fwd.y * 4.5f - right.y * 3.5f, 0.0f}
    };
    draw_lines(player, glm::vec3(0.18f, 0.52f, 0.90f));

    view_ = saved_view;
    projection_ = saved_proj;
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
}

void Renderer::draw_attitude_gauge(const glm::vec3& euler_deg,
                                   float airspeed_mps,
                                   float game_speed_scale) {
    glm::mat4 saved_view = view_;
    glm::mat4 saved_proj = projection_;

    view_ = glm::mat4(1.0f);
    projection_ = glm::ortho(0.0f, (float)width_, 0.0f, (float)height_, -1.0f, 1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
    glm::mat4 identity(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "model"), 1, GL_FALSE, glm::value_ptr(identity));

    auto draw_lines = [&](const std::vector<glm::vec3>& lines, const glm::vec3& color) {
        if (lines.empty()) return;
        glUniform3fv(glGetUniformLocation(shader_program_, "color"), 1, glm::value_ptr(color));
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(glm::vec3), lines.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, (int)lines.size());
        glBindVertexArray(0);
    };

    const glm::vec2 c((float)width_ - 116.0f, 112.0f);
    const float r = 82.0f;
    const int seg = 64;
    const float pitch = euler_deg.x; // deg
    const float roll = euler_deg.z;  // deg
    const float roll_rad = glm::radians(roll);

    //  translated comment
    std::vector<glm::vec3> rim;
    rim.reserve(seg * 2);
    for (int i = 0; i < seg; ++i) {
        float t0 = (2.0f * glm::pi<float>() * i) / seg;
        float t1 = (2.0f * glm::pi<float>() * (i + 1)) / seg;
        rim.push_back({c.x + std::cos(t0) * r, c.y + std::sin(t0) * r, 0.0f});
        rim.push_back({c.x + std::cos(t1) * r, c.y + std::sin(t1) * r, 0.0f});
    }
    draw_lines(rim, {0.14f, 0.22f, 0.15f});

    //  translated comment
    std::vector<glm::vec3> aircraft = {
        {c.x - 24.0f, c.y, 0.0f}, {c.x - 7.0f, c.y, 0.0f},
        {c.x + 24.0f, c.y, 0.0f}, {c.x + 7.0f, c.y, 0.0f},
        {c.x - 7.0f, c.y, 0.0f}, {c.x + 7.0f, c.y, 0.0f},
        {c.x, c.y, 0.0f}, {c.x, c.y - 8.0f, 0.0f}
    };
    draw_lines(aircraft, {0.92f, 0.92f, 0.80f});

    //  translated comment
    float pitch_px = glm::clamp(pitch, -25.0f, 25.0f) * 1.8f;
    glm::vec2 n(std::cos(roll_rad), std::sin(roll_rad));
    glm::vec2 t(-n.y, n.x);
    glm::vec2 horizon_center = c + n * pitch_px;
    float half = r * 0.95f;

    std::vector<glm::vec3> horizon = {
        {horizon_center.x - t.x * half, horizon_center.y - t.y * half, 0.0f},
        {horizon_center.x + t.x * half, horizon_center.y + t.y * half, 0.0f}
    };
    draw_lines(horizon, {0.30f, 0.82f, 0.32f});

    //  translated comment
    std::vector<glm::vec3> pitch_marks;
    for (int deg = -20; deg <= 20; deg += 5) {
        if (deg == 0) continue;
        float off = (glm::clamp(pitch, -30.0f, 30.0f) - (float)deg) * 1.8f;
        glm::vec2 pc = c + n * off;
        float len = (deg % 10 == 0) ? 18.0f : 10.0f;
        pitch_marks.push_back({pc.x - t.x * len * 0.5f, pc.y - t.y * len * 0.5f, 0.0f});
        pitch_marks.push_back({pc.x + t.x * len * 0.5f, pc.y + t.y * len * 0.5f, 0.0f});
    }
    draw_lines(pitch_marks, {0.24f, 0.56f, 0.24f});

    //  translated comment
    std::vector<glm::vec3> pointer = {
        {c.x, c.y + r - 3.0f, 0.0f},
        {c.x - 4.0f, c.y + r - 10.0f, 0.0f},
        {c.x, c.y + r - 3.0f, 0.0f},
        {c.x + 4.0f, c.y + r - 10.0f, 0.0f}
    };
    draw_lines(pointer, {0.88f, 0.88f, 0.30f});

    // Vertical airspeed tape next to the attitude gauge.
    const float speed_min = 0.0f;
    const float speed_max = 260.0f;
    const float tape_h = 136.0f;
    const float tape_half_h = tape_h * 0.5f;
    const float tape_x = c.x - r - 34.0f;
    const float tape_w = 14.0f;
    const float y0 = c.y - tape_half_h;
    const float y1 = c.y + tape_half_h;

    std::vector<glm::vec3> tape_frame = {
        {tape_x, y0, 0.0f}, {tape_x + tape_w, y0, 0.0f},
        {tape_x + tape_w, y0, 0.0f}, {tape_x + tape_w, y1, 0.0f},
        {tape_x + tape_w, y1, 0.0f}, {tape_x, y1, 0.0f},
        {tape_x, y1, 0.0f}, {tape_x, y0, 0.0f}
    };
    draw_lines(tape_frame, {0.14f, 0.22f, 0.15f});

    std::vector<glm::vec3> speed_ticks;
    for (int s = 0; s <= (int)speed_max; s += 20) {
        float t = ((float)s - speed_min) / (speed_max - speed_min);
        float y = y0 + t * tape_h;
        float tick_len = (s % 40 == 0) ? 9.0f : 5.0f;
        speed_ticks.push_back({tape_x - tick_len, y, 0.0f});
        speed_ticks.push_back({tape_x, y, 0.0f});
    }
    draw_lines(speed_ticks, {0.24f, 0.56f, 0.24f});

    float sp = glm::clamp(airspeed_mps, speed_min, speed_max);
    float sp_t = (sp - speed_min) / (speed_max - speed_min);
    float sp_y = y0 + sp_t * tape_h;
    std::vector<glm::vec3> speed_marker = {
        {tape_x - 2.0f, sp_y, 0.0f}, {tape_x - 14.0f, sp_y, 0.0f},
        {tape_x - 14.0f, sp_y, 0.0f}, {tape_x - 10.0f, sp_y + 4.0f, 0.0f},
        {tape_x - 14.0f, sp_y, 0.0f}, {tape_x - 10.0f, sp_y - 4.0f, 0.0f}
    };
    draw_lines(speed_marker, {0.96f, 0.42f, 0.10f});

    // Small GS bar under the speed tape (70%..160%).
    const float gs_min = 0.70f;
    const float gs_max = 1.60f;
    const float gs_y = y0 - 18.0f;
    std::vector<glm::vec3> gs_base = {
        {tape_x, gs_y, 0.0f}, {tape_x + tape_w, gs_y, 0.0f}
    };
    draw_lines(gs_base, {0.14f, 0.22f, 0.15f});
    float gs_t = (glm::clamp(game_speed_scale, gs_min, gs_max) - gs_min) / (gs_max - gs_min);
    float gs_x = tape_x + gs_t * tape_w;
    std::vector<glm::vec3> gs_marker = {
        {gs_x, gs_y - 4.0f, 0.0f}, {gs_x, gs_y + 4.0f, 0.0f}
    };
    draw_lines(gs_marker, {0.88f, 0.88f, 0.30f});

    view_ = saved_view;
    projection_ = saved_proj;
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, glm::value_ptr(view_));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "projection"), 1, GL_FALSE, glm::value_ptr(projection_));
}

// ──  translated comment
//  translated comment

WireMesh make_fighter_mesh_variant(float gear_deploy, float flap_deploy) {
    WireMesh m;
    gear_deploy = glm::clamp(gear_deploy, 0.0f, 1.0f);
    flap_deploy = glm::clamp(flap_deploy, 0.0f, 1.0f);
    // More detailed C172-style wireframe
    const std::vector<float> zs = {
        -7.4f, -6.3f, -4.8f, -3.1f, -1.2f, 0.8f, 2.6f, 4.3f, 6.0f, 7.4f
    };
    const std::vector<float> rx = {
        0.06f, 0.28f, 0.55f, 0.78f, 0.88f, 0.82f, 0.70f, 0.54f, 0.32f, 0.12f
    };
    const std::vector<float> ry = {
        0.06f, 0.32f, 0.65f, 0.92f, 0.98f, 0.92f, 0.80f, 0.62f, 0.36f, 0.14f
    };
    const int ring_pts = 14;

    m.vertices.push_back({0.0f, 0.02f, -7.9f}); // 0 nose
    m.vertices.push_back({0.0f, 0.08f,  8.2f}); // 1 tail cone

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

    // Hide fuselage ring circles around the wing region to avoid circular artifacts on the wing.
    const float ring_hide_min_z = -0.2f;
    const float ring_hide_max_z = 2.8f;
    for (int r = 0; r < (int)zs.size(); ++r) {
        if (zs[r] >= ring_hide_min_z && zs[r] <= ring_hide_max_z) {
            continue;
        }
        for (int i = 0; i < ring_pts; ++i) {
            m.line_indices.push_back(ring_idx(r, i));
            m.line_indices.push_back(ring_idx(r, i + 1));
        }
    }
    for (int r = 0; r < (int)zs.size() - 1; ++r) {
        for (int i = 0; i < ring_pts; ++i) {
            m.line_indices.push_back(ring_idx(r, i));
            m.line_indices.push_back(ring_idx(r + 1, i));
        }
    }
    for (int i = 0; i < ring_pts; ++i) {
        m.line_indices.push_back(0);
        m.line_indices.push_back(ring_idx(0, i));
        m.line_indices.push_back(1);
        m.line_indices.push_back(ring_idx((int)zs.size() - 1, i));
    }

    // High-wing layout (C172-like) with dihedral and struts
    const float wing_y = 0.78f;
    const float wing_z = 0.4f;
    const unsigned int wing_root_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.95f, wing_y, wing_z});
    const unsigned int wing_root_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.95f, wing_y, wing_z});
    const unsigned int wing_tip_l  = (unsigned int)m.vertices.size(); m.vertices.push_back({-5.2f, wing_y + 0.22f,  1.4f});
    const unsigned int wing_tip_r  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 5.2f, wing_y + 0.22f,  1.4f});
    const unsigned int wing_back_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.6f, wing_y - 0.05f,  2.6f});
    const unsigned int wing_back_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.6f, wing_y - 0.05f,  2.6f});
    const unsigned int wing_mid_l  = (unsigned int)m.vertices.size(); m.vertices.push_back({-3.1f, wing_y + 0.14f,  1.2f});
    const unsigned int wing_mid_r  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 3.1f, wing_y + 0.14f,  1.2f});

    m.line_indices.insert(m.line_indices.end(), {
        wing_root_l, wing_mid_l, wing_mid_l, wing_tip_l, wing_tip_l, wing_back_l, wing_back_l, wing_root_l,
        wing_root_r, wing_mid_r, wing_mid_r, wing_tip_r, wing_tip_r, wing_back_r, wing_back_r, wing_root_r,
        wing_root_l, wing_root_r, wing_mid_l, wing_mid_r
    });

    // Wing struts
    const unsigned int strut_l_fuse = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.65f, 0.18f, 1.2f});
    const unsigned int strut_r_fuse = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.65f, 0.18f, 1.2f});
    const unsigned int strut_l_wing = (unsigned int)m.vertices.size(); m.vertices.push_back({-3.0f, wing_y - 0.10f, 1.3f});
    const unsigned int strut_r_wing = (unsigned int)m.vertices.size(); m.vertices.push_back({ 3.0f, wing_y - 0.10f, 1.3f});
    m.line_indices.insert(m.line_indices.end(), {
        strut_l_fuse, strut_l_wing,
        strut_r_fuse, strut_r_wing
    });

    // Cabin windows (simple outline)
    const unsigned int win_l_fwd = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.55f, 0.45f, -1.3f});
    const unsigned int win_l_aft = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.85f, 0.45f,  0.6f});
    const unsigned int win_l_top = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.75f, 0.70f, -0.2f});
    const unsigned int win_r_fwd = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.55f, 0.45f, -1.3f});
    const unsigned int win_r_aft = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.85f, 0.45f,  0.6f});
    const unsigned int win_r_top = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.75f, 0.70f, -0.2f});
    m.line_indices.insert(m.line_indices.end(), {
        win_l_fwd, win_l_top, win_l_top, win_l_aft, win_l_aft, win_l_fwd,
        win_r_fwd, win_r_top, win_r_top, win_r_aft, win_r_aft, win_r_fwd
    });

    // Engine cowl ring (nose)
    const int cowl_start = (int)m.vertices.size();
    const int cowl_pts = 12;
    const float cowl_z = -6.8f;
    const float cowl_rx = 0.45f;
    const float cowl_ry = 0.48f;
    for (int i = 0; i < cowl_pts; ++i) {
        float t = (2.0f * 3.1415926f * i) / cowl_pts;
        m.vertices.push_back({cowl_rx * std::cos(t), cowl_ry * std::sin(t) + 0.05f, cowl_z});
    }
    for (int i = 0; i < cowl_pts; ++i) {
        m.line_indices.push_back(cowl_start + i);
        m.line_indices.push_back(cowl_start + ((i + 1) % cowl_pts));
    }

    // Tailplane + fin (smaller than fighter)
    const unsigned int tail_l = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.9f, 0.28f, 6.1f});
    const unsigned int tail_r = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.9f, 0.28f, 6.1f});
    const unsigned int tail_c = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.0f, 0.30f, 5.6f});
    const unsigned int fin_t  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.0f, 1.55f, 6.4f});
    const unsigned int fin_b  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.0f, 0.35f, 5.7f});

    m.line_indices.insert(m.line_indices.end(), {
        tail_c, tail_l, tail_c, tail_r, tail_l, tail_r,
        fin_b, fin_t, tail_c, fin_b
    });

    // Simple fixed gear (C172 is fixed gear)
    if (gear_deploy > 0.001f) {
        float gy = -0.35f - 0.25f * gear_deploy;
        const unsigned int ng_top = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.00f, -0.08f, -4.9f});
        const unsigned int ng_bot = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.00f, gy,   -4.5f});
        const unsigned int ng_wl  = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.22f * gear_deploy, gy - 0.04f, -4.45f});
        const unsigned int ng_wr  = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.22f * gear_deploy, gy - 0.04f, -4.45f});

        const unsigned int lg_l_top = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.05f, -0.02f,  1.6f});
        const unsigned int lg_l_bot = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.25f, gy, 1.9f});
        const unsigned int lg_r_top = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.05f, -0.02f,  1.6f});
        const unsigned int lg_r_bot = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.25f, gy, 1.9f});
        const unsigned int lg_l_brace = (unsigned int)m.vertices.size(); m.vertices.push_back({-0.85f, -0.12f,  1.4f});
        const unsigned int lg_r_brace = (unsigned int)m.vertices.size(); m.vertices.push_back({ 0.85f, -0.12f,  1.4f});

        m.line_indices.insert(m.line_indices.end(), {
            ng_top, ng_bot, ng_bot, ng_wl, ng_bot, ng_wr, ng_wl, ng_wr,
            lg_l_top, lg_l_bot, lg_r_top, lg_r_bot,
            lg_l_top, lg_l_brace, lg_l_brace, lg_l_bot,
            lg_r_top, lg_r_brace, lg_r_brace, lg_r_bot
        });
    }

    // Flaps (subtle)
    {
        float fdy = wing_y - 0.10f - 0.18f * flap_deploy;
        float fdz = 0.05f + 0.18f * flap_deploy;
        const unsigned int flap_l_i0 = (unsigned int)m.vertices.size(); m.vertices.push_back({-1.15f, fdy, 1.45f + fdz});
        const unsigned int flap_l_o0 = (unsigned int)m.vertices.size(); m.vertices.push_back({-3.40f, fdy - 0.03f, 1.05f + fdz});
        const unsigned int flap_r_i0 = (unsigned int)m.vertices.size(); m.vertices.push_back({ 1.15f, fdy, 1.45f + fdz});
        const unsigned int flap_r_o0 = (unsigned int)m.vertices.size(); m.vertices.push_back({ 3.40f, fdy - 0.03f, 1.05f + fdz});

        m.line_indices.insert(m.line_indices.end(), {
            flap_l_i0, flap_l_o0,
            flap_r_i0, flap_r_o0
        });
    }

    return m;
}

WireMesh make_fighter_mesh() {
    return make_fighter_mesh_variant(0.0f, 0.0f); //  translated comment
}

WireMesh make_carrier_mesh() {
    WireMesh m;
    //  translated comment
    m.vertices = {
        {-38.0f, 0.0f,  96.0f}, { 38.0f, 0.0f,  96.0f}, // 0 1  translated comment
        {-46.0f, 0.0f, -98.0f}, { 46.0f, 0.0f, -98.0f}, // 2 3  translated comment
        {-34.0f, 8.0f,  92.0f}, { 34.0f, 8.0f,  92.0f}, // 4 5  translated comment
        {-42.0f, 8.0f, -90.0f}, { 42.0f, 8.0f, -90.0f}, // 6 7  translated comment
        { 18.0f, 8.0f,  28.0f}, { 28.0f, 8.0f,  28.0f}, // 8 9  translated comment
        { 18.0f, 8.0f,   6.0f}, { 28.0f, 8.0f,   6.0f}, // 10 11
        { 20.0f, 24.0f, 22.0f}, { 28.0f, 24.0f, 22.0f}, // 12 13  translated comment
        { 20.0f, 24.0f, 10.0f}, { 28.0f, 24.0f, 10.0f}, // 14 15
        { 24.0f, 32.0f, 16.0f}, // 16  translated comment
        {-42.0f, 8.0f, -90.0f}, { 0.0f, 14.0f, -110.0f}, {42.0f, 8.0f, -90.0f}, // 17 18 19  translated comment
    };
    m.line_indices = {
        0,1, 1,3, 3,2, 2,0, //  translated comment
        4,5, 5,7, 7,6, 6,4, //  translated comment
        0,4, 1,5, 2,6, 3,7, //  translated comment
        4,6, 5,7, //  translated comment

        8,9, 9,11, 11,10, 10,8, //  translated comment
        12,13, 13,15, 15,14, 14,12, //  translated comment
        8,12, 9,13, 10,14, 11,15, //  translated comment
        12,16, 13,16, 14,16, 15,16, //  translated comment

        17,18, 18,19 //  translated comment
    };
    return m;
}
