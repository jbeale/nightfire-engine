#pragma once

#include <map>
#include <span>
#include <string>
#include <vector>

#include "assets/ssh_texture.hpp"
#include "core/math.hpp"
#include "driving/track_model.hpp"
#include "render/gl.hpp"

namespace nf::driving {

// Driving world coordinates are +X right, +Y up, +Z forward. Keep that screen
// orientation while projecting points in front of the camera to negative view Z.
Mat4 drive_view_matrix(const Vec3& eye, const Vec3& target, const Vec3& up);

// Draws SceneMesh batches with textures taken from a shape pool (`.ssh` files). Meshes are uploaded once;
// each draw call supplies its own model-view-projection matrix.
class SceneRenderer {
public:
    explicit SceneRenderer(const std::vector<SshFile>& shapes);
    ~SceneRenderer();
    SceneRenderer(const SceneRenderer&) = delete;
    SceneRenderer& operator=(const SceneRenderer&) = delete;

    struct Handle { std::size_t id; };
    // Additional shape files (e.g. a vehicle's own `.ssh`) searched before the level's.
    Handle upload(const SceneMesh& mesh, const std::vector<const SshFile*>& extra_shapes = {});
    void draw(Handle h, const Mat4& view_projection, const Mat4& model = identity(), bool world_lighting = true,
              float min_view_depth = 0.0f, float underlay_near = 0.0f) const;
    void draw_sky(Handle h, const Mat4& view_projection, const Vec3& eye) const;
    void draw_lamp_glows(const Mat4& view_projection, const Vec3& eye, const Vec3& target,
                         const Vec3& up) const;
    void set_fog(const Vec3& color, float start, float end);
    // Global diffuse tint (AmbientSky{World} of the level's Lighting tuning).
    void set_ambient(const Vec3& color);
    void set_key_light(const Vec3& direction, const Vec3& colour);
    void set_view_lights(const Vec3& eye, std::span<const Vec3> lamps, const Vec3& sled,
                         const Vec3& sled_forward);

private:
    struct GpuBatch { GLuint vao, vbo, ebo; GLsizei count; GLuint texture; bool alpha_test, translucent, fogged; };
    struct GpuMesh { std::vector<GpuBatch> batches; };

    GLuint texture_for(const std::string& shape, const std::vector<const SshFile*>& pools);

    const std::vector<SshFile>& level_shapes_;
    std::vector<GpuMesh> meshes_;
    std::map<std::pair<const SshFile*, std::size_t>, GLuint> textures_;
    GLuint program_ = 0, white_ = 0;
    bool lighting_ = false;
    Handle lamp_glow_{};
    GLint u_mvp_ = -1, u_fog_color_ = -1, u_fog_range_ = -1, u_ambient_ = -1, u_fogged_ = -1;
    GLint u_min_view_depth_ = -1, u_underlay_near_ = -1;
    GLint u_model_ = -1, u_light_enabled_ = -1, u_eye_ = -1, u_key_dir_ = -1, u_key_colour_ = -1;
    GLint u_lamp_count_ = -1, u_sled_ = -1, u_sled_dir_ = -1;
    GLint u_lamp_glow_ = -1;
    std::array<GLint, 12> u_lamps_{};
    std::array<Vec3, 12> closest_lamps_{};
    std::size_t lamp_count_ = 0;
};

}  // namespace nf::driving
