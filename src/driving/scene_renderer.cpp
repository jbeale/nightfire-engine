#include "driving/scene_renderer.hpp"

#include <algorithm>
#include <cstddef>

namespace nf::driving {

Mat4 drive_view_matrix(const Vec3& eye, const Vec3& target, const Vec3& up) {
    const Vec3 delta = target - eye;
    const float distance = length(delta);
    const Vec3 f = distance > 1e-6f ? delta * (1.0f / distance) : Vec3{0, 0, 1};
    const Vec3 lateral = cross(up, f);
    const float lateral_length = length(lateral);
    const Vec3 r = lateral_length > 1e-6f ? lateral * (1.0f / lateral_length) : Vec3{1, 0, 0};
    const Vec3 u = cross(f, r);
    return {r[0], u[0], -f[0], 0, r[1], u[1], -f[1], 0, r[2], u[2], -f[2], 0,
            -dot(r, eye), -dot(u, eye), dot(f, eye), 1};
}

using namespace nf::gl;

namespace {

const char* kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec2 a_uv;
layout(location = 2) in vec4 a_rgba;
uniform mat4 u_mvp;
uniform mat4 u_model;
uniform float u_underlay_near;
out vec2 v_uv;
out vec4 v_color;
out float v_depth;
out vec3 v_world;
void main() {
    v_uv = a_uv;
    v_color = a_rgba;
    v_world = (u_model * vec4(a_pos, 1.0)).xyz;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
    v_depth = gl_Position.w;
    // Collision fill should sit behind the authored surface even where the
    // collision plane pokes through a cliff. Shift depth by about half a
    // metre along the view ray without moving the silhouette.
    if (u_underlay_near > 0.0 && v_depth > 0.0)
        gl_Position.z += u_underlay_near * smoothstep(1.0, 8.0, v_depth) / (v_depth + 0.5);
}
)";

const char* kFragmentShader = R"(#version 330 core
in vec2 v_uv;
in vec4 v_color;
in float v_depth;
in vec3 v_world;
uniform sampler2D u_tex;
uniform vec3 u_fog_color;
uniform vec4 u_fog_range;
uniform vec3 u_ambient;
uniform float u_alpha_ref;
uniform float u_fogged;
uniform float u_min_view_depth;
uniform bool u_light_enabled;
uniform vec3 u_eye, u_key_dir, u_key_colour;
uniform int u_lamp_count;
uniform vec3 u_lamps[12];
uniform vec3 u_sled, u_sled_dir;
uniform bool u_lamp_glow;
out vec4 o_color;
void main() {
    if (u_lamp_glow) {
        vec2 p = v_uv * 2.0 - 1.0;
        float r2 = dot(p, p);
        float halo = exp(-4.8 * r2) * (1.0 - smoothstep(0.6, 1.0, r2));
        float core = exp(-75.0 * r2);
        float visibility = 1.0 - smoothstep(45.0, 125.0, v_depth);
        o_color = vec4(mix(vec3(1.0, 0.70, 0.34), vec3(1.0, 0.94, 0.78), core),
                       max(core, 0.28 * halo) * visibility);
        return;
    }
    if (v_depth < u_min_view_depth) discard;
    vec4 t = texture(u_tex, v_uv) * v_color;
    if (t.a < u_alpha_ref) discard;
    vec3 lighting = u_ambient;
    if (u_light_enabled) {
        vec3 normal = normalize(cross(dFdx(v_world), dFdy(v_world)));
        if (dot(normal, u_eye - v_world) < 0.0) normal = -normal;
        lighting = lighting * 0.87 + u_key_colour * max(dot(normal, u_key_dir), 0.0);
        for (int i = 0; i < u_lamp_count; ++i) {
            vec3 offset = u_lamps[i] - v_world;
            float d = length(offset);
            float falloff = 1.0 - smoothstep(2.0, 17.0, d);
            lighting += vec3(0.70, 0.49, 0.25) * falloff *
                        (0.18 + 0.82 * max(dot(normal, offset / max(d, 0.01)), 0.0));
        }
        vec3 headlamp = u_sled + 1.45 * u_sled_dir + vec3(0.0, 0.8, 0.0);
        vec3 beam = v_world - headlamp;
        float distance_to_sled = length(beam);
        vec3 to_surface = beam / max(distance_to_sled, 0.01);
        vec3 lamp_dir = normalize(u_sled_dir - vec3(0.0, 0.20, 0.0));
        float cone = smoothstep(0.89, 0.98, dot(to_surface, lamp_dir));
        float throw_light = max(1.0 - distance_to_sled / 32.0, 0.0);
        lighting += vec3(0.40, 0.49, 0.66) * cone * throw_light *
                    (0.46 + 0.54 * max(dot(normal, -to_surface), 0.0));
        vec3 offset = v_world - u_sled;
        vec3 sled_right = cross(vec3(0.0, 1.0, 0.0), u_sled_dir);
        vec2 footprint = vec2(dot(offset, sled_right) / 1.4, dot(offset, u_sled_dir) / 2.5);
        float shadow = (1.0 - smoothstep(0.20, 1.0, dot(footprint, footprint))) *
                       (1.0 - smoothstep(0.18, 0.85, abs(offset.y + 0.35)));
        lighting *= 1.0 - 0.40 * shadow * max(normal.y, 0.0);
    }
    vec3 lit = t.rgb * lighting;
    if (u_light_enabled) lit /= 1.0 + max(lit - 0.82, 0.0) * 0.8;
    float f = mix(1.0, clamp((u_fog_range.y - v_depth) / (u_fog_range.y - u_fog_range.x), 0.0, 1.0), u_fogged);
    o_color = vec4(mix(u_fog_color, lit, f), t.a);
}
)";

}  // namespace

SceneRenderer::SceneRenderer(const std::vector<SshFile>& shapes) : level_shapes_(shapes) {
    program_ = compile_program(kVertexShader, kFragmentShader);
    u_mvp_ = glGetUniformLocation(program_, "u_mvp");
    u_fog_color_ = glGetUniformLocation(program_, "u_fog_color");
    u_fog_range_ = glGetUniformLocation(program_, "u_fog_range");
    u_ambient_ = glGetUniformLocation(program_, "u_ambient");
    u_fogged_ = glGetUniformLocation(program_, "u_fogged");
    u_min_view_depth_ = glGetUniformLocation(program_, "u_min_view_depth");
    u_underlay_near_ = glGetUniformLocation(program_, "u_underlay_near");
    u_model_ = glGetUniformLocation(program_, "u_model");
    u_light_enabled_ = glGetUniformLocation(program_, "u_light_enabled");
    u_eye_ = glGetUniformLocation(program_, "u_eye");
    u_key_dir_ = glGetUniformLocation(program_, "u_key_dir");
    u_key_colour_ = glGetUniformLocation(program_, "u_key_colour");
    u_lamp_count_ = glGetUniformLocation(program_, "u_lamp_count");
    u_lamp_glow_ = glGetUniformLocation(program_, "u_lamp_glow");
    u_sled_ = glGetUniformLocation(program_, "u_sled");
    u_sled_dir_ = glGetUniformLocation(program_, "u_sled_dir");
    for (std::size_t i = 0; i < u_lamps_.size(); ++i)
        u_lamps_[i] = glGetUniformLocation(program_, ("u_lamps[" + std::to_string(i) + "]").c_str());
    glUseProgram(program_);
    glUniform1i(glGetUniformLocation(program_, "u_tex"), 0);
    glUniform3f(u_fog_color_, 0.6f, 0.65f, 0.7f);
    glUniform4f(u_fog_range_, 400.0f, 1500.0f, 0, 0);
    glUniform3f(u_ambient_, 1.0f, 1.0f, 1.0f);
    const std::uint32_t white = 0xFFFFFFFF;
    glGenTextures(1, &white_);
    glBindTexture(GL_TEXTURE_2D, white_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, &white);
}

SceneRenderer::~SceneRenderer() {
    for (const GpuMesh& m : meshes_)
        for (const GpuBatch& b : m.batches) {
            glDeleteVertexArrays(1, &b.vao);
            GLuint bufs[2] = {b.vbo, b.ebo};
            glDeleteBuffers(2, bufs);
        }
    for (const auto& [_, texture] : textures_) glDeleteTextures(1, &texture);
    glDeleteTextures(1, &white_);
    glDeleteProgram(program_);
}

void SceneRenderer::set_fog(const Vec3& color, float start, float end) {
    glUseProgram(program_);
    glUniform3f(u_fog_color_, color[0], color[1], color[2]);
    glUniform4f(u_fog_range_, start, end, 0, 0);
}

void SceneRenderer::set_ambient(const Vec3& color) {
    glUseProgram(program_);
    glUniform3f(u_ambient_, color[0], color[1], color[2]);
}

void SceneRenderer::set_key_light(const Vec3& direction, const Vec3& colour) {
    if (!lighting_) {
        SceneMesh mesh;
        auto& quad = mesh.batches.emplace_back();
        quad.vertices = {{{-1, -1, 0}, {0, 0}, 0xFFFFFFFF}, {{1, -1, 0}, {1, 0}, 0xFFFFFFFF},
                         {{-1, 1, 0}, {0, 1}, 0xFFFFFFFF}, {{1, 1, 0}, {1, 1}, 0xFFFFFFFF}};
        quad.indices = {0, 1, 2, 2, 1, 3};
        lamp_glow_ = upload(mesh);
    }
    lighting_ = true;
    glUseProgram(program_);
    glUniform3f(u_key_dir_, direction[0], direction[1], direction[2]);
    glUniform3f(u_key_colour_, colour[0], colour[1], colour[2]);
}

void SceneRenderer::set_view_lights(const Vec3& eye, std::span<const Vec3> lamps, const Vec3& sled,
                                    const Vec3& sled_forward) {
    const auto selected = std::partial_sort_copy(lamps.begin(), lamps.end(), closest_lamps_.begin(), closest_lamps_.end(),
        [&](const Vec3& a, const Vec3& b) { return dot(a - eye, a - eye) < dot(b - eye, b - eye); });
    lamp_count_ = std::size_t(selected - closest_lamps_.begin());
    glUseProgram(program_);
    glUniform3f(u_eye_, eye[0], eye[1], eye[2]);
    glUniform3f(u_sled_, sled[0], sled[1], sled[2]);
    glUniform3f(u_sled_dir_, sled_forward[0], sled_forward[1], sled_forward[2]);
    glUniform1i(u_lamp_count_, int(lamp_count_));
    for (std::size_t i = 0; i < lamp_count_; ++i) {
        const auto& lamp = closest_lamps_[i];
        glUniform3f(u_lamps_[i], lamp[0], lamp[1], lamp[2]);
    }
}

GLuint SceneRenderer::texture_for(const std::string& shape, const std::vector<const SshFile*>& pools) {
    if (shape.empty()) return white_;
    std::vector<const SshFile*> all = pools;
    for (const SshFile& f : level_shapes_) all.push_back(&f);
    const auto ref = find_shape(all, shape);
    if (!ref) return white_;
    const SshFile* file = all[ref->file];
    const auto key = std::make_pair(file, ref->image);
    if (auto it = textures_.find(key); it != textures_.end()) return it->second;
    const SshImage& img = file->images[ref->image];
    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, GLsizei(img.width), GLsizei(img.height), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 img.rgba.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    textures_[key] = id;
    return id;
}

SceneRenderer::Handle SceneRenderer::upload(const SceneMesh& mesh, const std::vector<const SshFile*>& extra_shapes) {
    GpuMesh gm;
    for (const MeshBatch& b : mesh.batches) {
        if (b.indices.empty()) continue;
        GpuBatch g{};
        glGenVertexArrays(1, &g.vao);
        glBindVertexArray(g.vao);
        glGenBuffers(1, &g.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(b.vertices.size() * sizeof(MeshVertex)), b.vertices.data(), GL_STATIC_DRAW);
        glGenBuffers(1, &g.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(b.indices.size() * 4), b.indices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, pos)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, uv)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(MeshVertex), reinterpret_cast<void*>(offsetof(MeshVertex, rgba)));
        g.count = GLsizei(b.indices.size());
        g.texture = texture_for(b.shape, extra_shapes);
        g.alpha_test = b.alpha_test;
        g.translucent = b.translucent;
        g.fogged = b.fogged;
        gm.batches.push_back(g);
    }
    meshes_.push_back(std::move(gm));
    return {meshes_.size() - 1};
}
void SceneRenderer::draw_sky(Handle h, const Mat4& view_projection, const Vec3& eye) const {
    Mat4 model = identity();
    model[12] = eye[0];
    model[13] = eye[1];
    model[14] = eye[2];
    draw(h, view_projection, model, false);
    glClear(GL_DEPTH_BUFFER_BIT);
}

void SceneRenderer::draw_lamp_glows(const Mat4& view_projection, const Vec3& eye, const Vec3& target,
                                    const Vec3& up) const {
    if (!lighting_ || lamp_count_ == 0) return;
    const Mat4 view = drive_view_matrix(eye, target, up);
    const Vec3 right{view[0], view[4], view[8]};
    const Vec3 vertical{view[1], view[5], view[9]};
    const auto& quad = meshes_[lamp_glow_.id].batches.front();
    glUseProgram(program_);
    glUniform1i(u_lamp_glow_, 1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    glBindVertexArray(quad.vao);
    for (std::size_t i = 0; i < lamp_count_; ++i) {
        const auto& lamp = closest_lamps_[i];
        const float distance = length(lamp - eye);
        if (distance >= 125.0f || distance < 0.4f) continue;
        const Vec3 center = lamp + (eye - lamp) * (0.13f / distance);
        constexpr float radius = 1.1f;
        const Vec3 x = right * radius, y = vertical * radius;
        const Mat4 panel{x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, 0, 0, 1, 0,
                         center[0], center[1], center[2], 1};
        const Mat4 mvp = mul(view_projection, panel);
        glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp.data());
        glDrawElements(GL_TRIANGLES, quad.count, GL_UNSIGNED_INT, nullptr);
    }
    glUniform1i(u_lamp_glow_, 0);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneRenderer::draw(Handle h, const Mat4& view_projection, const Mat4& model, bool world_lighting,
                         float min_view_depth, float underlay_near) const {
    const Mat4 mvp = mul(view_projection, model);
    glUseProgram(program_);
    glUniformMatrix4fv(u_mvp_, 1, GL_FALSE, mvp.data());
    glUniformMatrix4fv(u_model_, 1, GL_FALSE, model.data());
    glUniform1i(u_light_enabled_, world_lighting && lighting_);
    glUniform1f(u_min_view_depth_, min_view_depth);
    glUniform1f(u_underlay_near_, underlay_near);
    const GLint alpha_ref = glGetUniformLocation(program_, "u_alpha_ref");
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_DEPTH_TEST);
    // Opaque (and alpha-tested cutout) first with depth writes on, then blended
    // translucent (water, glass, particle sheets) with depth writes off, preserving
    // upload order within each pass. Previously every batch drew opaque with a single
    // 0.4 cutout, so translucent water/glass occluded like rock and cutout foliage
    // either leaked or punched holes depending on its texel alpha.
    const GpuMesh& mesh = meshes_[h.id];
    for (int pass = 0; pass < 2; ++pass) {
        for (const GpuBatch& b : mesh.batches) {
            if ((b.translucent ? 1 : 0) != pass) continue;
            glBindTexture(GL_TEXTURE_2D, b.texture);
            glBindVertexArray(b.vao);
            glUniform1f(u_fogged_, b.fogged ? 1.0f : 0.0f);
            if (b.translucent) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                // Tinted glass/water blends whole; only opaque cutout foliage discards.
                glUniform1f(alpha_ref, 0.02f);
            } else {
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
                glUniform1f(alpha_ref, b.alpha_test ? 0.5f : 0.0f);
            }
            glDrawElements(GL_TRIANGLES, b.count, GL_UNSIGNED_INT, nullptr);
        }
    }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

}  // namespace nf::driving
