#pragma once

#include <array>
#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "core/math.hpp"

namespace nf::driving {

class CollisionWorld;
struct SceneMesh;

// Ground-projected tread and ski marks. One short history per sled, one
// bounded pool of segments shared by the chase.
class SnowTracks {
public:
    struct Edge {
        Vec3 left, center, right;
        float distance = 0;
    };
    struct Segment {
        Edge start, end;
        float born;
    };
    static constexpr float kLifetime = 24.0f;

    SnowTracks(const SceneMesh& scene, std::size_t sleds);
    void tick(float dt);
    void sample(std::size_t sled, std::optional<Vec3> position, const CollisionWorld& collision);
    const std::deque<Segment>& segments() const { return segments_; }
    float age(const Segment& segment) const { return time_ - segment.born; }

private:
    using Triangle = std::array<Vec3, 3>;
    struct Trail {
        std::optional<Vec3> point;
        std::optional<Edge> edge;
    };

    std::optional<Vec3> project(Vec3 p, const CollisionWorld& collision) const;

    std::map<std::array<int, 2>, std::vector<std::size_t>> cells_;
    std::vector<Triangle> surface_;
    std::deque<Segment> segments_;
    std::vector<Trail> trails_;
    float time_ = 0;
};

class SnowTrackRenderer {
public:
    SnowTrackRenderer();
    ~SnowTrackRenderer();
    SnowTrackRenderer(const SnowTrackRenderer&) = delete;
    SnowTrackRenderer& operator=(const SnowTrackRenderer&) = delete;

    void draw(const SnowTracks& tracks, const Mat4& vp);

private:
    struct Vertex {
        Vec3 position;
        float side, distance, opacity;
    };
    std::vector<Vertex> vertices_;
    unsigned program_ = 0, vao_ = 0, vbo_ = 0;
    int u_vp_ = -1;
};

}  // namespace nf::driving
