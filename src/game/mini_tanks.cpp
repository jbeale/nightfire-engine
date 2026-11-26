#include "game/mini_tanks.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "assets/anim.hpp"
#include "game/world.hpp"

namespace nf {
namespace {

constexpr std::uint32_t kRcClass = 0xDA;
constexpr std::uint32_t kHull = 0x020003D6;
constexpr std::uint32_t kTurret = 0x02000502;
constexpr std::uint32_t kBarrel = 0x02000505;
constexpr float kHullBottom = 0.145472f;
constexpr float kTurretReach = 0.252657f;
constexpr float kBarrelRise = 0.162f;
constexpr float kCameraRise = 0.07085f;
constexpr int kActPrimary = 9, kActSecondary = 12;
constexpr int kBullet = 96, kRocket = 42;

Vec3 axis(const Mat4& m, std::size_t i) { return {m[i], m[i + 1], m[i + 2]}; }

}  // namespace

MiniTanks::MiniTanks(World& world, WeaponSystem& weapons, bool enabled) : world_(world), weapons_(weapons) {
    Level& level = world_.level();
    if (!level.map()) return;
    const auto& placements = level.placements();
    for (std::size_t i = 0; i < placements.size(); ++i) {
        const auto& p = placements[i];
        if (level.map()->chunk.statics[p.instance].object_class() == kRcClass) markers_.push_back(i);
    }
    if (!enabled || markers_.empty()) return;
    const auto find_part = [&](std::uint32_t hash) -> std::optional<Placement> {
        for (std::size_t c = 0; c < level.chunks().size(); ++c) {
            const auto& models = level.chunks()[c].chunk.models;
            for (std::size_t m = 0; m < models.size(); ++m)
                if (models[m].hash == std::int32_t(hash)) return Placement{c, m, 0, identity()};
        }
        return std::nullopt;
    };

    const auto turret = find_part(kTurret), barrel = find_part(kBarrel);
    if (!turret || !barrel) return;
    turret_ = *turret;
    barrel_ = *barrel;
    for (const auto i : markers_) {
        const auto& p = placements[i];
        if (level.chunks()[p.chunk].chunk.models[p.model].hash != std::int32_t(kHull)) continue;
        Tank& tank = tanks_.emplace_back();
        tank.hull = p;
        tank.home = tank.position = axis(p.transform, 12);
        // Car_Create's ground setup uses the opposite yaw sign from the static model placement.
        tank.home_yaw = tank.yaw = std::atan2(-p.transform[8], p.transform[10]);
    }
    for (auto& tank : tanks_) tank.target_id = weapons_.register_target(&tank);
}

void MiniTanks::bind_player(int slot, Player& player) {
    if (tanks_.empty()) return;
    player.set_use_handler([this, slot](const Player::ActivationProbe& probe) { return use(slot, probe); });
}

bool MiniTanks::use(int slot, const Player::ActivationProbe& probe) {
    Player& player = *world_.player(slot);
    if (player.in_vehicle()) return player.vehicle().kind == VehicleKind::Car;
    if (!player.alive() || (player.substate != SubState::Walk && player.substate != SubState::Crouch)) return false;
    Tank* nearest = nullptr;
    float best = 1e8f;
    for (auto& tank : tanks_) {
        if (!tank.alive() || tank.owner >= 0) continue;
        const Vec3 delta = tank.center() - probe.center;
        const float distance = dot(delta, delta);
        const float reach = probe.radius + tank.radius();
        if (distance >= best || distance > reach * reach ||
            !world_.collision().line_of_sight(player.eye(), tank.center(), pick::kIgnoreGhost)) continue;
        best = distance;
        nearest = &tank;
    }
    if (!nearest) return false;
    nearest->owner = slot;
    nearest->activated_frame = world_.frame();
    nearest->rockets = 10;
    nearest->heat = nearest->primary_delay = nearest->secondary_delay = 0.0f;
    nearest->overheated = false;
    player.board_vehicle(VehicleKind::Car, std::uint32_t(nearest->target_id));
    return true;
}

void MiniTanks::reset(Tank& tank) const {
    tank.position = tank.home;
    tank.yaw = tank.home_yaw;
    tank.up = {0, 1, 0};
    tank.turret_yaw = tank.turret_pitch = 0.0f;
    tank.speed = tank.steering = tank.fall_speed = 0.0f;
    tank.damage = 0.0f;
    tank.attacker = -1;
}

void MiniTanks::detonate(Tank& tank, int attacker) {
    if (tank.owner >= 0) {
        if (Player* player = world_.player(tank.owner); player && player->vehicle().kind == VehicleKind::Car &&
            player->vehicle().object == std::uint32_t(tank.target_id)) player->leave_vehicle();
    }
    const Vec3 blast_pos = tank.position;
    tank.owner = -1;
    tank.respawn = 1800.0f;
    reset(tank);
    WeaponDef blast{};
    blast.blast_radius = 10.0f;
    blast.damage = 50.0f;
    weapons_.explode_at(blast_pos, blast, attacker, 1.0f, 0x06000052);
}

Mat4 MiniTanks::hull_matrix(const Tank& tank) const {
    const Vec3 heading{std::sin(tank.yaw), 0, std::cos(tank.yaw)};
    const Vec3 right = normalised(cross(tank.up, heading));
    const Vec3 forward = cross(right, tank.up);
    return {right[0], right[1], right[2], 0, tank.up[0], tank.up[1], tank.up[2], 0,
            forward[0], forward[1], forward[2], 0, tank.position[0], tank.position[1], tank.position[2], 1};
}

Mat4 MiniTanks::turret_matrix(const Tank& tank) const {
    const float a = tank.turret_yaw * 0.5f;
    return mul(hull_matrix(tank), quat_trans_to_mat({0, std::sin(a), 0, std::cos(a)}, {}));
}

Mat4 MiniTanks::barrel_matrix(const Tank& tank) const {
    const float a = tank.turret_pitch * 0.5f;
    return mul(turret_matrix(tank),
               quat_trans_to_mat({std::sin(a), 0, 0, std::cos(a)}, {0, kBarrelRise, kTurretReach}));
}

std::optional<MiniTanks::View> MiniTanks::view(int slot) const {
    for (const Tank& tank : tanks_) {
        if (tank.owner != slot) continue;
        const Mat4 barrel = barrel_matrix(tank);
        return View{axis(barrel, 12) + tank.up * kCameraRise, axis(barrel, 8), tank.rockets, int(tank.heat)};
    }
    return std::nullopt;
}

std::vector<Placement> MiniTanks::draw_placements(int viewer) const {
    std::vector<Placement> out;
    for (const Tank& tank : tanks_) {
        if (!tank.alive() || (tank.owner >= 0 && tank.owner == viewer)) continue;
        Placement hull = tank.hull;
        hull.transform = hull_matrix(tank);
        Placement turret = turret_;
        turret.transform = turret_matrix(tank);
        Placement barrel = barrel_;
        barrel.transform = barrel_matrix(tank);
        out.insert(out.end(), {hull, turret, barrel});
    }
    return out;
}

void MiniTanks::drive(Tank& tank, const ActionInput* input, FrameTiming timing) const {
    const float dt = timing.rec();
    const float forward = input ? input->actionf(kActForward) : 0.0f;
    const float turn = input ? input->actionf(kActStrafe) : 0.0f;
    const Vec3 old_forward = axis(hull_matrix(tank), 8);
    float step = tank.speed * dt;
    step = std::clamp(step + (forward * timing.mul() * 2.0f - step) * 0.15f, -5.0f * dt, 5.0f * dt);
    tank.speed = forward == 0.0f && std::fabs(step) < 0.01f ? 0.0f : step / dt;
    float yaw_step = tank.steering * dt;
    yaw_step += (turn * timing.mul() * 0.05f - yaw_step) * 0.5f;
    tank.steering = turn == 0.0f && std::fabs(yaw_step) < 0.01f
        ? 0.0f : std::clamp(yaw_step / dt, -std::numbers::pi_v<float>, std::numbers::pi_v<float>);
    tank.yaw += tank.steering * dt;
    tank.fall_speed = std::max(-45.0f, tank.fall_speed + world_.params().gravity[1] * dt);
    tank.position += old_forward * (tank.speed * dt) + Vec3{0, tank.fall_speed * dt, 0};

    const Vec3 center = tank.position + Vec3{0, tank.radius() - kHullBottom, 0};
    const auto contacts = world_.collision().cylinder({center, center, tank.radius(), pick::kIgnoreGhost});
    tank.position += contacts.push_out;
    for (const CollisionHit& hit : contacts.hits) {
        if (hit.normal[1] <= 0.5f) continue;
        tank.fall_speed = 0.0f;
        if (hit.normal[1] > 0.64f) tank.up = normalised(tank.up + (hit.normal - tank.up) * (dt * 3.0f));
        break;
    }
}

void MiniTanks::fire(Tank& tank, const ActionInput& input, FrameTiming timing) {
    const float mul = timing.mul();
    tank.turret_yaw = std::remainder(tank.turret_yaw - input.actionf(kActTurn) * mul * 0.05f,
                                    2.0f * std::numbers::pi_v<float>);
    const float max_pitch = (1.0f - std::fabs(tank.turret_yaw * 1.2732395f)) * 0.129f + 0.08f;
    tank.turret_pitch = std::clamp(tank.turret_pitch - input.actionf(kActLookPitch) * mul * 0.05f, -0.705f, max_pitch);
    tank.heat = std::max(0.0f, tank.heat - mul);
    if (tank.heat == 0.0f) tank.overheated = false;
    tank.primary_delay -= mul;
    tank.secondary_delay -= mul;
    const Mat4 barrel = barrel_matrix(tank);
    const Vec3 direction = axis(barrel, 8);
    WeaponSystem::Shooter shot{tank.owner, axis(barrel, 12) + direction * 0.61648f, direction};
    shot.ignored_target = tank.target_id;
    if (input.held(kActPrimary) && tank.primary_delay <= 0.0f && !tank.overheated) {
        weapons_.fire(shot, kBullet, 0x10a);
        tank.primary_delay = 6.0f;
        tank.heat += 10.0f;
        tank.overheated = tank.heat > 100.0f;
    }
    if (input.held(kActSecondary) && tank.secondary_delay <= 0.0f && tank.rockets > 0) {
        weapons_.fire(shot, kRocket, 0x40c);
        tank.secondary_delay = 120.0f;
        --tank.rockets;
    }
}

void MiniTanks::tick(World& world, FrameTiming timing) {
    for (auto& tank : tanks_) {
        if (!tank.alive()) {
            tank.respawn -= timing.mul();
            continue;
        }
        if (tank.damage > 200.0f || tank.position[1] < tank.home[1] - 80.0f) {
            detonate(tank, tank.attacker);
            continue;
        }
        const ActionInput* input = nullptr;
        if (tank.owner >= 0) {
            const Player* player = world.player(tank.owner);
            if (!player || !player->alive() || player->vehicle().kind != VehicleKind::Car ||
                player->vehicle().object != std::uint32_t(tank.target_id)) {
                detonate(tank, tank.owner);
                continue;
            }
            if (tank.activated_frame != world.frame()) {
                input = &world.input(tank.owner);
                if (input->pressed(kActUse)) {
                    detonate(tank, tank.owner);
                    continue;
                }
            }
        }
        drive(tank, input, timing);
        if (input) fire(tank, *input, timing);
    }
}

}  // namespace nf
