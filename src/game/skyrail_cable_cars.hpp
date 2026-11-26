#pragma once

#include <vector>

#include "game/world.hpp"

namespace nf {

// Skyrail's looping entity script drives both the visible cars and their triangle collision.
class SkyrailCableCars final : public System {
public:
    explicit SkyrailCableCars(World& world);
    const std::vector<Placement>& poses() const { return poses_; }
    const std::vector<std::size_t>& static_markers() const { return markers_; }
    void before_player_update(World& world, FrameTiming timing) override;
    void tick(World&, FrameTiming) override {}

private:
    struct Track {
        std::size_t begin, end;
        int parent;
    };

    void sample(float frame);
    std::vector<float> times_;
    std::vector<Vec3> positions_;
    std::vector<std::array<float, 4>> rotations_;
    std::vector<Track> tracks_;
    std::vector<Placement> poses_;
    std::vector<std::size_t> markers_;
    Mat4 anchor_ = identity();
    CollisionWorld* collision_ = nullptr;
    std::size_t first_handle_ = 0;
    float period_ = 0;
    float clock_ = 0;
};

}  // namespace nf
