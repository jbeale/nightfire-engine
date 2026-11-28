#include "driving/snow_tracks.hpp"

#include <algorithm>
#include <cstddef>

#include "render/gl.hpp"

namespace nf::driving {
using namespace nf::gl;
namespace {

const char* kVertex = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_track;
uniform mat4 u_vp;
out vec3 v_track;
out float v_depth;
void main() {
    gl_Position = u_vp * vec4(a_pos, 1.0);
    v_track = a_track;
    v_depth = gl_Position.w;
}
)";

const char* kFragment = R"(#version 330 core
in vec3 v_track;
in float v_depth;
out vec4 o_color;
void main() {
    float x = abs(v_track.x);
    float rut = 1.0 - smoothstep(0.21, 0.37, x);
    float ski = 1.0 - smoothstep(0.025, 0.10, abs(x - 0.81));
    float edge = 1.0 - smoothstep(0.02, 0.09, abs(x - 0.35));
    float phase = fract(v_track.y * 4.2 + x * 1.5);
    float tread = smoothstep(0.12, 0.28, phase) * (1.0 - smoothstep(0.51, 0.68, phase));
    // Fade the fine tread as it approaches one pixel, before it can shimmer.
    tread = mix(tread, 0.35, smoothstep(0.14, 0.5, fwidth(v_track.y) * 4.2));
    float alpha = rut * (0.26 + 0.23 * tread) + ski * 0.32 + edge * 0.08;
    float fade = (1.0 - smoothstep(38.0, 100.0, v_depth)) * v_track.z;
    o_color = vec4(0.24, 0.32, 0.42, alpha * fade);
}
)";

}  // namespace

SnowTrackRenderer::SnowTrackRenderer() {
    program_ = compile_program(kVertex, kFragment);
    u_vp_ = glGetUniformLocation(program_, "u_vp");
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, side)));
}

SnowTrackRenderer::~SnowTrackRenderer() {
    glDeleteBuffers(1, &vbo_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteProgram(program_);
}

void SnowTrackRenderer::draw(const SnowTracks& tracks, const Mat4& vp) {
    if (tracks.segments().empty()) return;
    vertices_.clear();
    vertices_.reserve(tracks.segments().size() * 12);
    for (const auto& segment : tracks.segments()) {
        const float opacity = std::clamp((SnowTracks::kLifetime - tracks.age(segment)) / 9.0f, 0.0f, 1.0f);
        const auto& a = segment.start;
        const auto& b = segment.end;
        const Vertex left{a.left, -1, a.distance, opacity}, center{a.center, 0, a.distance, opacity},
                     right{a.right, 1, a.distance, opacity};
        const Vertex next_left{b.left, -1, b.distance, opacity}, next_center{b.center, 0, b.distance, opacity},
                     next_right{b.right, 1, b.distance, opacity};
        vertices_.insert(vertices_.end(), {left, center, next_left, next_left, center, next_center,
                                           center, right, next_center, next_center, right, next_right});
    }
    glUseProgram(program_);
    glUniformMatrix4fv(u_vp_, 1, GL_FALSE, vp.data());
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertices_.size() * sizeof(Vertex)), vertices_.data(), GL_STREAM_DRAW);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(vertices_.size()));
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

}  // namespace nf::driving
