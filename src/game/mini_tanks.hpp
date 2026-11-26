#pragma once

#include <optional>
#include <vector>

#include "game/weapons.hpp"

namespace nf {

// Multiplayer Mini Vehicles, Tanks mode. The player remains in the world while driving remotely.
class MiniTanks final : public System {
public:
    struct View {
        Vec3 eye;
        Vec3 direction;
        int rockets;
        int heat;
    };

    MiniTanks(World& world, WeaponSystem& weapons, bool enabled);
    void bind_player(int slot, Player& player);
    std::optional<View> view(int slot) const;
    std::vector<Placement> draw_placements(int viewer) const;
    const std::vector<std::size_t>& static_markers() const { return markers_; }
    void tick(World& world, FrameTiming timing) override;

private:
    struct Tank final : DamageTarget {
        Placement hull;
        Vec3 home{}, position{}, up{0, 1, 0};
        float home_yaw = 0, yaw = 0, turret_yaw = 0, turret_pitch = 0;
        float speed = 0, steering = 0, fall_speed = 0;
        float respawn = 0, damage = 0, heat = 0, primary_delay = 0, secondary_delay = 0;
        int rockets = 0, owner = -1, target_id = -1, attacker = -1;
        std::uint64_t activated_frame = 0;
        bool overheated = false;

        Vec3 center() const override { return position + Vec3{0, .22f, 0}; }
        float radius() const override { return 0.55f; }
        float half_height() const override { return 0.10f; }
        bool alive() const override { return respawn <= 0.0f; }
        bool autoaim_target() const override { return false; }
        void hurt(const HitInfo& hit) override {
            if (hit.damage <= 0.0f) return;
            damage += hit.damage;
            attacker = hit.attacker;
        }
    };

    bool use(int slot, const Player::ActivationProbe& probe);
    void detonate(Tank& tank, int attacker);
    void drive(Tank& tank, const ActionInput* input, FrameTiming timing) const;
    void fire(Tank& tank, const ActionInput& input, FrameTiming timing);
    void reset(Tank& tank) const;
    Mat4 hull_matrix(const Tank& tank) const;
    Mat4 turret_matrix(const Tank& tank) const;
    Mat4 barrel_matrix(const Tank& tank) const;

    World& world_;
    WeaponSystem& weapons_;
    std::vector<Tank> tanks_;
    std::vector<std::size_t> markers_;
    Placement turret_{}, barrel_{};
};

}  // namespace nf
