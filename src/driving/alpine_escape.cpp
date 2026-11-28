#include "driving/alpine_escape.hpp"
#include "driving/car_weapons.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace nf::driving {
namespace {

constexpr float kStep = 1.0f / kTickHz;
constexpr Vec3 kUp{0, 1, 0};
constexpr Vec3 kGarageStart{223.631f, 10.998f, -411.058f};
constexpr Vec3 kGarageDoor{219.885f, 12.234f, -413.024f};
constexpr Vec3 kBombPosition{225.626f, -0.154f, -413.033f};
constexpr Vec3 kGarageBlast{224.3f, 13.0f, -410.36f};
constexpr float kBarrelRadius = 0.61f;
constexpr Vec3 kExitDoor{228.831f, 12.234f, -407.854f};
constexpr Vec3 kDoorOut{-0.866f, 0, -0.5f};
constexpr Vec3 kDoorRight{0.5f, 0, -0.866f};
constexpr float kAimRange = 120.0f;
constexpr float kAcquireDot = 0.993f;  // about seven degrees
constexpr float kReleaseDot = 0.978f;
constexpr float kHeadOnSeconds = 3.4f;
constexpr float kGateCutSeconds = 3.8f;
constexpr float kSceneFps = 30.0f;
constexpr float kChaletInteriorFps = 15.0f;
constexpr float kChaletExitCameraFps = 50.0f;
constexpr float kChaletExitFrame = 72.0f;
constexpr float kChaletReverseSeconds = 3.8f;
constexpr float kBridgeFireFrame = 188.0f;
constexpr float kBridgeBlastFrame = 214.0f;
constexpr float kStrafeTravel = 0.25f;
constexpr Vec3 kRavineArmor{1733.407f, -203.659f, 166.816f};

Vec3 unit(const Vec3& v) {
    const float n = length(v);
    return n > 0.0001f ? v * (1.0f / n) : Vec3{};
}

Vec3 direction(float yaw, float pitch) {
    const float cp = std::cos(pitch);
    return {std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp};
}

float yaw_to(const Vec3& v) { return std::atan2(v[0], v[2]); }
float wrap(float a) { return std::remainder(a, 2.0f * std::numbers::pi_v<float>); }

void turn_sled(float target, float& heading, float& rate) {
    const float error = wrap(target - heading);
    const float target_rate = std::clamp(error * 5.0f, -1.85f, 1.85f);
    rate += (target_rate - rate) * (kStep / 0.15f);
    heading = wrap(heading + rate * kStep);
}

Vec3 path_ahead(const std::vector<Vec3>& path, std::size_t waypoint, Vec3 position, float distance) {
    for (; waypoint < path.size(); ++waypoint) {
        const Vec3 delta{path[waypoint][0] - position[0], 0, path[waypoint][2] - position[2]};
        const float span = length(delta);
        if (distance < span) return position + delta * (distance / span);
        position += delta;
        distance -= span;
    }
    return position;
}

Vec3 aim_point(const AlpineEscape::Guard& g) { return g.position + Vec3{0, g.sled ? 0.95f : 1.25f, 0}; }

std::vector<Vec3> path_positions(const CarpFile& carp, std::string_view name) {
    for (const CarpEntry* entry : carp.find("pt")) {
        const Bytes bytes = carp.payload(*entry);
        if (bytes.size() < 0x70 || load_cstr(bytes, 0x18) != name) continue;
        // The second path channel contains float4 position keys. The header
        // gives the relative key offset and count from the start of the record.
        const auto offset = load<std::uint32_t>(bytes, 0x5c);
        const auto count = load<std::uint32_t>(bytes, 0x60);
        if (offset > bytes.size() || count > (bytes.size() - offset) / 16) return {};
        std::vector<Vec3> path;
        for (std::uint32_t i = 0; i < count; ++i) {
            const std::size_t p = offset + i * 16;
            const Vec3 point{load<float>(bytes, p), load<float>(bytes, p + 4), load<float>(bytes, p + 8)};
            if (path.empty() || length(point - path.back()) > 0.05f) path.push_back(point);
        }
        return path;
    }
    return {};
}

// Key times in the path records are authored 30 Hz frames. Preserve the
// independent position and rotation channels, which have different key spacing.
template<class Key, class Blend>
auto sample_keys(const std::vector<Key>& keys, float frame, Blend blend) {
    const auto next = std::ranges::upper_bound(keys, frame, {}, &Key::frame);
    if (next == keys.begin()) return next->value;
    const auto& previous = *(next - 1);
    if (next == keys.end()) return previous.value;
    return blend(previous.value, next->value, (frame - previous.frame) / (next->frame - previous.frame));
}

}  // namespace

AlpineEscape::ScenePath AlpineEscape::ScenePath::load(const CarpFile& carp, std::string_view name) {
    for (const CarpEntry* entry : carp.find("pt")) {
        const Bytes bytes = carp.payload(*entry);
        if (bytes.size() < 0x70 || load_cstr(bytes, 0x18) != name) continue;
        ScenePath path;
        auto read = [&](auto& keys, std::size_t time_field, std::size_t value_field, auto decode) {
            const auto time = nf::load<std::uint32_t>(bytes, time_field);
            const auto value = nf::load<std::uint32_t>(bytes, value_field);
            const auto count = nf::load<std::uint32_t>(bytes, value_field + 4);
            if (!count || time > bytes.size() || value > bytes.size() ||
                count > (bytes.size() - time) / 4 || count > (bytes.size() - value) / 16) return false;
            keys.reserve(count);
            for (std::uint32_t i = 0; i < count; ++i)
                keys.push_back({nf::load<float>(bytes, time + 4 * i), decode(value + 16 * i)});
            return std::ranges::all_of(keys, [](const auto& key) { return std::isfinite(key.frame); }) &&
                   std::ranges::is_sorted(keys, {}, [](const auto& key) { return key.frame; });
        };
        const bool positions = read(path.positions, 0x58, 0x5c, [&](std::size_t i) {
            return Vec3{nf::load<float>(bytes, i), nf::load<float>(bytes, i + 4), nf::load<float>(bytes, i + 8)};
        });
        const bool rotations = read(path.rotations, 0x44, 0x48, [&](std::size_t i) {
            return nf::Quat{nf::load<float>(bytes, i), nf::load<float>(bytes, i + 4),
                        nf::load<float>(bytes, i + 8), nf::load<float>(bytes, i + 12)};
        });
        if (positions && rotations) return path;
        break;
    }
    throw FormatError("MIS3 scene path is missing or invalid: " + std::string(name));
}

Mat4 AlpineEscape::ScenePath::pose(float frame) const {
    const Vec3 position = sample_keys(positions, frame, [](Vec3 a, Vec3 b, float t) { return a + (b - a) * t; });
    return quat_trans_to_mat(sample_keys(rotations, frame, slerp), position);
}

AlpineEscape::AlpineEscape(DriveSession& session, const CarpFile& carp, const MissionData& data,
                           const TrackRenderExtras& extras, AlpineCheckpoint checkpoint)
    : course_y_(kGarageStart[1] - Snowmobile::kRideHeight) {
    if (data.route.size() < 143) throw FormatError("MIS3 opening route is incomplete");
    session.sled().set_scripted_pose(kGarageStart, heading_, 0);
    // rn 128 onward runs out through the garage and across the compound. The
    // stock road cut takes over at the first collision path, before rn 143.
    escape_path_.assign(data.route.begin() + 128, data.route.begin() + 142);
    if (data.route.size() >= 148) {
        // rn 145..147 form the branch next to the authored 007 pickup.
        pickup_path_.assign(data.route.begin() + 128, data.route.begin() + 135);
        pickup_path_.insert(pickup_path_.end(), data.route.begin() + 145, data.route.begin() + 148);
        pickup_path_.insert(pickup_path_.end(), data.route.begin() + 136, data.route.begin() + 142);
    }
    chase_path_ = path_positions(carp, "path_CollisionSnowMobile");
    if (chase_path_.size() < 2) throw FormatError("MIS3 snowmobile chase path is missing");
    chase_path_.insert(chase_path_.end(), data.route.begin(), data.route.begin() + 11);
    opening_end_ = chase_path_.size();
    chase_path_.insert(chase_path_.end(), data.route.begin() + 11, data.route.begin() + 26);
    // rn 26..38 is the parallel line through the forest. The main course
    // joins the culvert and head-on scene at rn 39.
    chase_path_.insert(chase_path_.end(), data.route.begin() + 39, data.route.begin() + 60);
    forest_checkpoint_ = chase_path_.size() - 1;  // rn 59
    chase_path_.insert(chase_path_.end(), data.route.begin() + 60, data.route.begin() + 84);
    gate_stop_ = chase_path_.size();
    chase_path_.push_back({1629.70f, -172.83f, 278.43f});
    chase_path_.insert(chase_path_.end(), data.route.begin() + 84, data.route.begin() + 85);
    bridge_camera_ = ScenePath::load(carp, "path_BridgeExpCamera");
    bridge_sled_ = ScenePath::load(carp, "path_SnowMobileBridgePath");
    bridge_heli_ = ScenePath::load(carp, "path_HeliPath");
    ravine_heli_ = ScenePath::load(carp, "path_PostBridgeHeli");
    const auto& takeoff = bridge_sled_.positions.front().value;
    chase_path_.push_back({takeoff[0], takeoff[1] - 1.05f, takeoff[2]});
    bridge_entry_ = chase_path_.size();
    // The map's rn route skips the gully. Trace the stock gunner run to the
    // chalet entrance, accounting for the camera offset.
    chase_path_.insert(chase_path_.end(), {
        {1717.0f, -197.0f, 200.6f}, {1725.6f, -200.5f, 180.5f},
        {1734.2f, -203.9f, 166.4f}, {1746.2f, -207.0f, 155.2f},
        {1759.0f, -209.6f, 146.5f}, {1773.0f, -212.8f, 139.1f},
        {1787.6f, -215.4f, 133.6f}, {1802.7f, -217.2f, 128.6f},
        {1813.6f, -217.8f, 120.3f}, {1814.9f, -217.7f, 109.6f},
        {1814.8f, -219.3f, 98.0f}, {1819.4f, -222.9f, 89.5f},
        {1822.4f, -229.4f, 78.2f}, {1827.4f, -236.2f, 64.6f},
    });
    chalet_interior_ = ScenePath::load(carp, "path_chalet");
    chalet_exit_sled_ = ScenePath::load(carp, "path_SnowMobileChalet");
    chalet_exit_camera_ = ScenePath::load(carp, "path_ChaletExitCamera");
    chalet_reverse_camera_ = ScenePath::load(carp, "path_ChaletExitCamera2");
    chalet_pursuer_path_ = ScenePath::load(carp, "path_EnemyPathChalet");
    chalet_heli_ = ScenePath::load(carp, "path_HeliChalet");
    post_chalet_heli_ = ScenePath::load(carp, "path_PostChaletHeli");
    const Vec3 entry = chalet_interior_.positions.front().value;
    chase_path_.push_back(entry - kUp * (0.48f + Snowmobile::kRideHeight));
    chalet_entry_ = chase_path_.size();

    // The cut leaves the sled on the first bend beyond the lodge. Keep the
    // preceding route points so the pursuing sled can follow the same trail.
    chase_path_.insert(chase_path_.end(), data.route.begin() + 92, data.route.begin() + 93);
    chase_path_.insert(chase_path_.end(), data.route.begin() + 94, data.route.begin() + 98);
    chase_path_.push_back({1928.2f, -251.0f, -39.5f});
    chalet_rejoin_ = chase_path_.size() - 1;
    chase_path_.insert(chase_path_.end(), data.route.begin() + 98, data.route.begin() + 109);
    // The stock first-person chase keeps going past the authored main road
    // nodes. These last points follow the stock replay to the lower forest bend.
    chase_path_.insert(chase_path_.end(), {
        {1992.0f, -274.4f, -215.2f}, {2001.7f, -277.2f, -229.2f},
        {2013.4f, -279.4f, -241.9f}, {2025.6f, -281.1f, -253.4f},
        {2039.2f, -284.6f, -261.7f}, {2059.9f, -288.8f, -267.6f},
    });
    chase_distances_.push_back(0.0f);
    for (std::size_t i = 1; i < chase_path_.size(); ++i) {
        const Vec3 delta = chase_path_[i] - chase_path_[i - 1];
        chase_distances_.push_back(chase_distances_.back() + std::hypot(delta[0], delta[2]));
    }
    head_on_camera_ = path_positions(carp, "path_ChickenCamera");
    gate_camera_ = path_positions(carp, "path_GateCamera");
    bomb_camera_ = path_positions(carp, "path_BombCamera");
    barrels_.reserve(extras.barrels.size());
    for (const auto& barrel : extras.barrels)
        barrels_.push_back({barrel.position});

    // Staging positions and timings approximate the visible waves at the garage
    // and the start of the ridge. The snowmobile's motion uses the disc course.
    guards_ = {
        {.position={212.3f,10.61f,-419.3f}, .destination={217.5f,10.61f,-415.8f}, .appears_at=3.0f},
        {.position={209.4f,10.61f,-416.6f}, .destination={215.9f,10.61f,-414.0f}, .appears_at=4.2f},
        {.position={210.0f,10.61f,-422.1f}, .destination={216.7f,10.61f,-417.3f}, .appears_at=5.0f},
        {.position={250.0f,10.61f,-398.3f}, .destination={249.0f,10.61f,-398.3f}, .appears_at=10.0f},
        {.position={273.0f,10.61f,-378.0f}, .destination={268.0f,10.61f,-381.0f}, .appears_at=12.0f},
        {.position={290.0f,10.61f,-374.0f}, .destination={286.0f,10.61f,-374.0f}, .appears_at=13.0f},
        {.position={565.0f,-41.0f,-33.0f}, .destination={565.0f,-41.0f,-33.0f}, .appears_at=16.0f},
        {.position={606.0f,-46.0f,1.0f}, .destination={606.0f,-46.0f,1.0f}, .appears_at=18.0f},
        {.position={1632.0f,-172.8f,285.2f}, .destination={1630.0f,-172.8f,283.4f}, .health=30},
        {.position={1631.3f,-172.8f,272.8f}, .destination={1633.0f,-172.8f,274.1f}, .health=30},
        {.position={1637.0f,-164.4f,283.7f}, .destination={1637.0f,-164.4f,281.7f}, .health=36},
    };
    // The pursuit sleds enter from different sides of the trail and trade
    // positions as the player reaches the culvert.
    guards_.push_back({.health=60, .sled=true, .lane=-3.1f, .lead=12.0f});
    guards_.push_back({.health=60, .sled=true, .lane=3.9f, .lead=27.0f});
    guards_.push_back({.health=60, .sled=true, .lane=1.3f, .lead=-14.0f});
    oncoming_rider_ = guards_.size();
    guards_.push_back({.health=60, .sled=true, .lane=-0.8f});
    guards_.push_back({.health=60, .sled=true, .lane=-4.4f, .lead=14.0f, .forest=true});
    guards_.push_back({.health=48, .sled=true, .lane=4.1f, .lead=-4.0f, .forest=true});
    chalet_pursuer_ = guards_.size();
    guards_.push_back({.health=60, .sled=true, .lane=0.45f, .lead=-11.5f});
    // The stock gunner camera starts looking out the rear shutter while
    // the sled points the other way, toward the escape route.
    yaw_ = std::numbers::pi_v<float>;
    if (checkpoint != AlpineCheckpoint::Garage) start_at(session, checkpoint);
    update_camera(session);
}

void AlpineEscape::start_at(DriveSession& session, AlpineCheckpoint checkpoint) {
    // Seed the completed scenery and pickups on a fresh mission, without
    // running the skipped encounters or queuing their audio and effects.
    clock_ = 30.0f;
    departure_ = kDepartureCutEnd;
    garage_detonated_at_ = clock_ - 10.0f;
    door_lift_ = 2.42f;
    exit_door_lift_ = 3.2f;
    for (Barrel& barrel : barrels_)
        barrel.destroyed = length(barrel.center - kGarageBlast) < 12.0f;
    grate_broken_ = passed_opening_ = armor_ = true;
    armor_points_ = 50.0f;

    collected_grenades_ = checkpoint != AlpineCheckpoint::Forest;
    grenade_ammo_ = collected_grenades_ ? 21 : 0;
    gate_destroyed_ = collected_grenades_ && checkpoint != AlpineCheckpoint::Gate;
    gate_closure_ = collected_grenades_ ? 1.0f : 0.0f;
    if (gate_destroyed_) grenade_ammo_ = 20;

    std::size_t point = forest_checkpoint_;
    stage_ = Stage::ForestRun;
    stage_time_ = 0;
    yaw_ = pitch_ = 0;
    speed_ = 19.0f;
    if (checkpoint == AlpineCheckpoint::Gate) {
        point = gate_stop_;
        stage_ = Stage::GateHold;
        speed_ = 0.0f;
        grenade_selected_ = true;
    } else if (checkpoint == AlpineCheckpoint::Bridge) {
        point = gate_stop_ + 1;
        stage_ = Stage::GateClear;
        speed_ = 8.0f;
    } else if (checkpoint == AlpineCheckpoint::Chalet) {
        point = chalet_entry_ - 2;
        stage_ = Stage::Ravine;
        speed_ = 4.0f;
        stage_time_ = (chase_distances_[point] - chase_distances_[bridge_entry_]) / 23.0f;
        helicopter_attack_at_ = clock_ + 1.2f;
    } else if (checkpoint == AlpineCheckpoint::Pursuit || checkpoint == AlpineCheckpoint::LowerForest) {
        point = checkpoint == AlpineCheckpoint::Pursuit ? chalet_rejoin_ : chase_path_.size() - 2;
        begin_chalet_chase(point);
        stage_time_ = (chase_distances_[point] - chase_distances_[chalet_rejoin_]) / 23.0f;
        chalet_door_at_ = chalet_window_at_ = clock_ - 10.0f;
    }
    if (checkpoint == AlpineCheckpoint::Chalet || chalet_chase()) {
        bridge_destroyed_at_ = clock_ - 10.0f;
        ravine_armor_collected_ = true;
    }

    waypoint_ = point + 1;
    chase_distance_ = chase_distances_[point];
    heading_ = yaw_to(chase_path_[waypoint_] - chase_path_[point]);
    turn_rate_ = 0;
    course_y_ = chase_path_[point][1];
    follow_ground(session, chase_path_[point] + kUp * Snowmobile::kRideHeight);
    update_helicopter();
}

int AlpineEscape::timer_digit() const {
    return std::clamp(int(std::ceil((kTimerZero - departure_) / 0.5f)), 0, 3);
}

const char* AlpineEscape::objective() const {
    if (gate_waiting()) return "Blast the gate with the JL-7C";
    if (pursuing_helicopter() || in_chalet() || chalet_chase() || trail_end()) return "Evade Drake's pursuing forces";
    return stage_ == Stage::Garage || stage_ == Stage::Escape
        ? "Escape from Drake's castle compound" : "Cover Zoe as she drives";
}

void AlpineEscape::advance_driver(DriveSession& session) {
    if (stage_ == Stage::Garage) {
        door_lift_ = 2.42f * std::clamp((stage_time_ - 0.6f) / 2.2f, 0.0f, 1.0f);
        const bool cleared = std::none_of(guards_.begin(), guards_.begin() + 3,
                                          [](const Guard& guard) { return guard.health > 0; });
        if (stage_time_ < 5.5f || (!cleared && stage_time_ < 11.0f)) return;
        stage_ = Stage::Escape;
        stage_time_ = 0;
        yaw_ = pitch_ = 0;
        lock_.reset();
    }
    if (stage_ == Stage::Escape && exit_door_lift_ < 3.2f) {
        exit_door_lift_ = 3.2f * std::clamp(stage_time_ / 1.4f, 0.0f, 1.0f);
        if (exit_door_lift_ < 3.2f) return;
    }
    if (stage_ == Stage::RoadCut) {
        if (stage_time_ >= 2.5f) {
            stage_ = Stage::Chase;
            stage_time_ = 0;
            yaw_ = pitch_ = 0;
        }
    }
    if (stage_ == Stage::TrailEnd) return;
    if (stage_ == Stage::BridgeCut) {
        advance_bridge(session);
        return;
    }
    if (stage_ == Stage::ChaletInterior || stage_ == Stage::ChaletExit) {
        advance_chalet(session);
        return;
    }
    if (stage_ == Stage::GateHold) {
        if (!gate_destroyed_) return;
        stage_ = Stage::GateClear;
        stage_time_ = 0;
    }
    if (stage_ == Stage::GateCut) {
        gate_closure_ = std::clamp(stage_time_ / 1.8f, 0.0f, 1.0f);
        if (stage_time_ >= kGateCutSeconds) {
            stage_ = Stage::GateApproach;
            stage_time_ = 0;
            yaw_ = pitch_ = 0;
        }
    }
    if (stage_ == Stage::HeadOn && stage_time_ >= kHeadOnSeconds) {
        stage_ = Stage::ForestRun;
        stage_time_ = 0;
        yaw_ = pitch_ = 0;
    }
    if (stage_ == Stage::Escape && token_revealed_ && !pickup_path_.empty() &&
        waypoint_ <= 7 && session.player_position()[2] < -296.0f) {
        escape_path_ = std::move(pickup_path_);
        pickup_path_.clear();
    }
    const auto& path = stage_ == Stage::Escape ? escape_path_ : chase_path_;
    Vec3 p = session.player_position();
    const bool cut = stage_ == Stage::Escape && waypoint_ + 2 >= path.size();
    const bool approach = stage_ == Stage::GateApproach || stage_ == Stage::GateCut;
    const float target_speed = (stage_ == Stage::Ravine || chalet_chase()) ? 23.0f
        : stage_ == Stage::HeadOn || approach ? 8.0f
        : stage_ == Stage::Escape && !cut ? 15.0f : 19.0f;
    const bool stopping = (approach && waypoint_ == gate_stop_) || (chalet_chase() && waypoint_ + 1 == path.size());
    const float brake_speed = stopping ? std::max(2.0f, std::sqrt(15.0f * length(path[waypoint_] - p))) : target_speed;
    const float desired_speed = std::min(target_speed, brake_speed);
    speed_ += std::clamp(desired_speed - speed_, -9.0f * kStep, 8.0f * kStep);
    const Vec3 ahead = path_ahead(path, waypoint_, p, 3.0f + speed_ * 0.32f) - p;
    if (length(ahead) > 0.05f) turn_sled(yaw_to(ahead), heading_, turn_rate_);
    float budget = speed_ * kStep;
    while (waypoint_ < path.size() && budget > 0) {
        Vec3 target = path[waypoint_];
        Vec3 delta{target[0] - p[0], 0, target[2] - p[2]};
        const float distance = length(delta);
        const float travel = std::min(distance, budget);
        if (distance > 0.0001f) {
            const float fraction = travel / distance;
            p += delta * fraction;
            course_y_ += (target[1] - course_y_) * fraction;
            budget -= travel;
            if (stage_ != Stage::Escape) chase_distance_ += travel;
        }
        if (travel >= distance) {
            ++waypoint_;
            if ((stage_ == Stage::GateClear && waypoint_ == bridge_entry_) ||
                (stage_ == Stage::Ravine && waypoint_ == chalet_entry_)) break;
            if (approach && waypoint_ == gate_stop_ + 1) {
                stage_ = Stage::GateHold;
                stage_time_ = 0;
                speed_ = 0;
                break;
            }
        }
    }
    follow_ground(session, p);
    if (stage_ == Stage::GateClear && waypoint_ == bridge_entry_) {
        stage_ = Stage::BridgeCut;
        stage_time_ = 0;
        lock_.reset();
        return;
    }
    if (stage_ == Stage::Ravine && waypoint_ == chalet_entry_) {
        stage_ = Stage::ChaletInterior;
        stage_time_ = 0;
        yaw_ = pitch_ = 0;
        lock_.reset();
        return;
    }
    if (!armor_ && stage_ == Stage::Chase && waypoint_ + 15 >= opening_end_) {
        armor_ = true;
        armor_points_ = 50;
    }
    if (stage_ == Stage::Chase && waypoint_ >= opening_end_) {
        stage_ = Stage::Pursuit;
        stage_time_ = 0;
        passed_opening_ = true;
    }
    // The sled clears the grate before its camera reaches the drain mouth.
    if (stage_ == Stage::Pursuit && p[0] >= kGratePosition[0] - 1.8f) grate_broken_ = true;
    if (stage_ == Stage::Pursuit && p[0] >= 963.5f) {
        stage_ = Stage::HeadOn;
        stage_time_ = 0;
        lock_.reset();
    }
    if (stage_ == Stage::ForestRun && p[0] >= 1508.5f) {
        stage_ = Stage::GateCut;
        stage_time_ = 0;
        lock_.reset();
    }
    if (waypoint_ < path.size()) return;
    if (stage_ == Stage::Escape) {
        stage_ = Stage::RoadCut;
        stage_time_ = 0;
        // Start the scripted roadside track where it meets the escape route.
        waypoint_ = 1;
        const auto& first = chase_path_[0];
        heading_ = yaw_to(chase_path_[1] - first);
        turn_rate_ = 0;
        course_y_ = first[1];
        heave_speed_ = 0;
        sled_up_ = kUp;
        follow_ground(session, first + kUp * Snowmobile::kRideHeight);
        lock_.reset();
    } else {
        stage_ = Stage::TrailEnd;
        speed_ = 0;
        session.sled().set_scripted_pose(session.player_position(), heading_, 0, sled_up_);
    }
}

void AlpineEscape::advance_bridge(DriveSession& session) {
    const float frame = stage_time_ * kSceneFps;
    follow_scene_sled(session, bridge_sled_.pose(frame), 0.70f);
    if (frame < bridge_sled_.end_frame()) return;
    const Vec3 position = session.player_position();
    GroundHit ground;
    course_y_ = position[1] - Snowmobile::kRideHeight;
    if (session.collision().ground_below(position + kUp, ground) &&
        std::abs(ground.point[1] - course_y_) < 2.0f) course_y_ = ground.point[1];
    heave_speed_ = 0;
    speed_ = 22.0f;
    waypoint_ = bridge_entry_;
    stage_ = Stage::Ravine;
    stage_time_ = 0;
    yaw_ = pitch_ = 0;
    helicopter_attack_at_ = clock_ + 0.75f;
}

void AlpineEscape::follow_scene_sled(DriveSession& session, const Mat4& pose, float height_offset) {
    const Vec3 position{pose[12], pose[13] - height_offset, pose[14]};
    speed_ = std::min(34.0f, length(position - session.player_position()) / kStep);
    heading_ = yaw_to({pose[0], pose[1], pose[2]});
    turn_rate_ = 0;
    sled_up_ = {pose[4], pose[5], pose[6]};
    session.sled().set_scripted_pose(position, heading_, speed_, sled_up_);
}

void AlpineEscape::advance_chalet(DriveSession& session) {
    if (in_chalet()) {
        const float frame = stage_time_ * kChaletInteriorFps;
        follow_scene_sled(session, chalet_interior_.pose(frame), 0.48f);
        if (frame >= kChaletExitFrame) {
            stage_ = Stage::ChaletExit;
            stage_time_ = 0;
            lock_.reset();
        }
        return;
    }

    const float reverse_shot = chalet_exit_camera_.end_frame() / kChaletExitCameraFps;
    const float frame = stage_time_ * kChaletExitCameraFps;
    if (stage_time_ <= reverse_shot) {
        follow_scene_sled(session, chalet_exit_sled_.pose(frame), 0.48f);
        return;
    }

    // The second camera watches the pursuer cross the snow. Under that cut
    // Zoe is already driving toward the point where Bond gets control back.
    const Vec3 start = chalet_exit_sled_.positions.back().value - kUp * 0.48f;
    const float elapsed = stage_time_ - reverse_shot;
    const float t = std::clamp(elapsed / kChaletReverseSeconds, 0.0f, 1.0f);
    const Vec3 end = chase_path_[chalet_rejoin_] + kUp * Snowmobile::kRideHeight;
    const Vec3 position = start + (end - start) * t;
    heading_ = yaw_to(end - start);
    turn_rate_ = 0;
    course_y_ = position[1] - Snowmobile::kRideHeight;
    follow_ground(session, position);
    if (t < 1.0f) return;

    begin_chalet_chase(chalet_rejoin_);
}

void AlpineEscape::begin_chalet_chase(std::size_t route_point) {
    stage_ = Stage::ChaletChase;
    stage_time_ = 0;
    waypoint_ = route_point + 1;
    chase_distance_ = chase_distances_[route_point];
    speed_ = 19.0f;
    yaw_ = std::numbers::pi_v<float>;
    pitch_ = 0;
    helicopter_attack_at_ = clock_ + 1.2f;
}

void AlpineEscape::update_helicopter() {
    if (stage_ == Stage::BridgeCut) {
        const float frame = stage_time_ * kSceneFps;
        helicopter_ = frame <= bridge_heli_.end_frame() ? bridge_heli_.pose(frame)
                                                       : ravine_heli_.pose(frame - bridge_heli_.end_frame());
    } else if (stage_ == Stage::Ravine) {
        helicopter_ = ravine_heli_.pose(bridge_sled_.end_frame() - bridge_heli_.end_frame() + stage_time_ * kSceneFps);
    } else if (in_chalet() || stage_ == Stage::ChaletExit) {
        helicopter_ = chalet_heli_.pose(stage_ == Stage::ChaletExit ? stage_time_ * kSceneFps : 0.0f);
    } else if (chalet_chase()) {
        helicopter_ = post_chalet_heli_.pose(stage_time_ * kSceneFps);
    } else if (!trail_end()) {
        helicopter_.reset();
    }
}

std::optional<Vec3> AlpineEscape::bridge_rocket() const {
    const float frame = stage_time_ * kSceneFps;
    if (stage_ != Stage::BridgeCut || frame < kBridgeFireFrame || frame >= kBridgeBlastFrame) return std::nullopt;
    const Mat4 firing_pose = bridge_heli_.pose(kBridgeFireFrame);
    const Vec3 muzzle{firing_pose[12], firing_pose[13] - 0.45f, firing_pose[14]};
    return muzzle + (kBridgeCenter - muzzle) * ((frame - kBridgeFireFrame) / (kBridgeBlastFrame - kBridgeFireFrame));
}

std::optional<Vec3> AlpineEscape::helicopter_round() const {
    if (!strafe_) return std::nullopt;
    const float progress = std::clamp((clock_ - strafe_->starts_at) / kStrafeTravel, 0.0f, 1.0f);
    return strafe_->muzzle + (strafe_->impact - strafe_->muzzle) * progress;
}

float AlpineEscape::tick_helicopter_attack(const DriveSession& session, std::vector<SoundEvent>& sounds) {
    if ((!pursuing_helicopter() && !chalet_chase()) || !helicopter_) {
        strafe_.reset();
        return 0.0f;
    }
    const Vec3 sled = session.player_position();
    if (strafe_ && clock_ >= strafe_->starts_at + kStrafeTravel) {
        const Vec3 impact = strafe_->impact;
        strafe_.reset();
        blasts_.push_back({impact, 0.23f});
        return length(impact - sled) < 3.2f ? 4.0f : 0.0f;
    }
    if (clock_ < helicopter_attack_at_ || strafe_) return 0.0f;

    const Mat4& aircraft = *helicopter_;
    const Vec3 muzzle{aircraft[12], aircraft[13] - 0.55f, aircraft[14]};
    if (length(muzzle - sled) > 78.0f) return 0.0f;
    const Vec3 forward = session.player_forward();
    Vec3 impact = sled + forward * (session.player_speed() * kStrafeTravel) - kUp * 0.25f;
    SegmentHit hit;
    if (session.collision().segment_hit(muzzle, impact, hit)) impact = hit.point;
    strafe_ = Strafe{muzzle, impact, clock_};
    helicopter_attack_at_ = clock_ + 2.25f;
    sounds.push_back({"SFX_sno_HmntmachFire", muzzle, 0.82f});
    return 0.0f;
}

void AlpineEscape::follow_ground(DriveSession& session, Vec3 position) {
    const Vec3 forward = direction(heading_, 0);
    const Vec3 right = cross(kUp, forward);
    // Sample beneath the two front skis and the rear track. Use the course
    // elevation to reject overhead decks and ground on another elevation.
    const bool ravine = pursuing_helicopter() || chalet_chase();
    const float drop_limit = ravine ? 6.0f : 1.65f;
    const auto contact = [&](const Vec3& offset, GroundHit& hit) {
        return session.collision().ground_below({position[0] + offset[0], course_y_ + 1.5f,
                                                  position[2] + offset[2]}, hit) &&
               course_y_ - hit.point[1] < drop_limit && hit.normal[1] > 0.52f;
    };
    GroundHit left, right_ski, rear, center;
    float height = course_y_;
    Vec3 normal = kUp;
    if (contact(forward * 1.05f - right * 0.8f, left) &&
        contact(forward * 1.05f + right * 0.8f, right_ski) && contact(forward * -1.25f, rear)) {
        const Vec3 axle = (left.point + right_ski.point) * 0.5f;
        normal = unit(cross(axle - rear.point, right_ski.point - left.point));
        height = (axle[1] * 1.25f + rear.point[1] * 1.05f) / 2.3f;
    } else if (contact({}, center)) {
        height = center.point[1];
        normal = center.normal;
    }
    sled_up_ = unit(sled_up_ * 0.80f + normal * 0.20f);
    // A damped vertical response filters individual terrain triangles while
    // keeping the skis from sinking through a rising surface.
    const float target = std::min(height + Snowmobile::kRideHeight, position[1] + 0.16f);
    const float suspension = ravine ? 345.0f : 260.0f;
    heave_speed_ += ((target - position[1]) * suspension - 26.0f * heave_speed_) * kStep;
    heave_speed_ = std::clamp(heave_speed_, ravine ? -17.0f : -9.0f, 9.0f);
    position[1] += heave_speed_ * kStep;
    if (position[1] < target - 0.045f) {
        position[1] = target - 0.045f;
        heave_speed_ = std::max(0.0f, heave_speed_);
    }
    session.sled().set_scripted_pose(position, heading_, speed_, sled_up_);
}

void AlpineEscape::update_camera(const DriveSession& session) {
    const Vec3 pos = session.player_position();
    camera_.up = kUp;
    camera_.fovy = 0.88f;
    camera_.znear = 0.06f;
    if (timer_cut()) {
        const float t = std::clamp((departure_ - kTimerCutStart) / (kTimerZero - kTimerCutStart), 0.0f, 1.0f);
        camera_.eye = bomb_camera_.empty() ? kBombPosition + Vec3{0.5f, 0.12f, 0.8f}
            : bomb_camera_.front() + (bomb_camera_.back() - bomb_camera_.front()) * (t * 0.42f);
        camera_.target = kBombPosition + Vec3{0, 0.015f, 0};
        camera_.fovy = 0.46f;
        camera_.znear = 0.025f;
        return;
    }
    if (garage_blast_cut()) {
        const Vec3 forward = session.player_forward();
        camera_.eye = pos + Vec3{0, 4.4f, 0} + forward * 3.3f;
        camera_.target = pos * 0.16f + kGarageBlast * 0.84f;
        return;
    }
    if (stage_ == Stage::ChaletExit) {
        const float frame = stage_time_ * kChaletExitCameraFps;
        const Mat4 pose = frame <= chalet_exit_camera_.end_frame() ? chalet_exit_camera_.pose(frame)
            : chalet_reverse_camera_.pose(frame - chalet_exit_camera_.end_frame());
        camera_.eye = {pose[12], pose[13], pose[14]};
        camera_.target = camera_.eye + Vec3{pose[0], pose[1], pose[2]};
        camera_.up = {pose[4], pose[5], pose[6]};
        return;
    }
    if (stage_ == Stage::BridgeCut) {
        const float frame = stage_time_ * kSceneFps;
        if (frame <= bridge_camera_.end_frame()) {
            const Mat4 pose = bridge_camera_.pose(frame);
            camera_.eye = {pose[12], pose[13], pose[14]};
            camera_.target = camera_.eye + Vec3{pose[0], pose[1], pose[2]};
            camera_.up = {pose[4], pose[5], pose[6]};
        } else {
            // After the exterior cut the camera comes back down with the sled.
            const Vec3 forward = session.player_forward();
            camera_.eye = pos + kUp * 2.05f - forward * 4.75f;
            camera_.target = pos + forward * 1.65f + kUp * 0.45f;
            camera_.up = unit(kUp + sled_up_);
        }
        return;
    }
    if (exterior()) {
        const Vec3 rt = cross(kUp, session.player_forward());
        camera_.eye = pos + rt * 7.5f + kUp * 4.0f + session.player_forward() * 3.0f;
        camera_.target = pos;
        if (stage_ == Stage::GateCut && !gate_camera_.empty()) {
            const float t = std::clamp(stage_time_ / kGateCutSeconds, 0.0f, 1.0f);
            camera_.eye = gate_camera_.front() + (gate_camera_.back() - gate_camera_.front()) * t;
            camera_.target = kGateCenter - Vec3{3, 0, 0};
        } else if (stage_ == Stage::HeadOn) {
            const Vec3 forward = session.player_forward();
            camera_.eye = pos - rt * 4.4f + kUp * 1.9f - forward * 3.0f;
            const Guard& opponent = guards_[oncoming_rider_];
            if (stage_time_ >= 1.45f && opponent.active) {
                camera_.eye = opponent.position - rt * 3.1f + kUp * 1.1f - forward * 3.8f;
                camera_.target = aim_point(opponent);
            } else if (!head_on_camera_.empty()) {
                camera_.target = pos + kUp * 0.65f + unit(head_on_camera_.back() - head_on_camera_.front()) * 1.2f;
            }
        }
        return;
    }
    camera_.up = unit(kUp * 0.70f + sled_up_ * 0.30f);
    camera_.eye = pos + camera_.up * 1.53f - session.player_forward() * 1.093f;
    camera_.target = camera_.eye + direction(heading_ + yaw_, pitch_);
}

bool AlpineEscape::visible(const DriveSession& session, const Vec3& target) const {
    const float from_door = dot(camera_.eye - kGarageDoor, kDoorOut);
    const float past_door = dot(target - kGarageDoor, kDoorOut);
    if (stage_ == Stage::Garage && from_door < 0 && past_door > 0) {
        // The track collision does not include the animated leaf or both
        // stone jambs. Limit garage shots to the visible door opening.
        const Vec3 at_door = camera_.eye + (target - camera_.eye) * (-from_door / (past_door - from_door));
        if (std::abs(dot(at_door - kGarageDoor, kDoorRight)) > 2.0f ||
            at_door[1] > 10.65f + door_lift_) return false;
    }
    SegmentHit hit;
    return !session.collision().segment_hit(camera_.eye, target, hit) || hit.t > 0.985f;
}

void AlpineEscape::place_sled(Guard& guard, const DriveSession& session, float distance, float lane,
                              bool oncoming, bool was_active) {
    const auto sample = [&](float d) {
        d = std::clamp(d, 0.0f, chase_distances_.back());
        const auto it = std::ranges::upper_bound(chase_distances_, d);
        const std::size_t i = std::clamp<std::size_t>(it - chase_distances_.begin(), 1, chase_path_.size() - 1);
        const float span = std::max(0.01f, chase_distances_[i] - chase_distances_[i - 1]);
        return chase_path_[i - 1] + (chase_path_[i] - chase_path_[i - 1]) * ((d - chase_distances_[i - 1]) / span);
    };
    distance = std::clamp(distance, 0.0f, chase_distances_.back());
    const Vec3 delta = sample(distance + 4.0f) - sample(distance - 4.0f);
    const Vec3 forward = unit(Vec3{delta[0], 0, delta[2]});
    guard.position = sample(distance) + cross(kUp, forward) * lane;
    GroundHit ground;
    if (session.collision().ground_below(guard.position + kUp * 1.6f, ground) &&
        std::abs(ground.point[1] - guard.position[1]) <= 2.4f)
        guard.position[1] = ground.point[1];
    const float target = yaw_to(forward * (oncoming ? -1.0f : 1.0f));
    if (was_active) turn_sled(target, guard.yaw, guard.turn_rate);
    else {
        guard.yaw = target;
        guard.turn_rate = 0;
    }
}

void AlpineEscape::damage_guard(Guard& guard, float damage, const Vec3& impulse,
                               std::vector<SoundEvent>& sounds) {
    if (guard.health <= 0) return;
    guard.health = std::max(0.0f, guard.health - damage);
    if (guard.health > 0) return;

    guard.fall_velocity = impulse;
    const Vec3 forward = direction(guard.yaw, 0);
    if (guard.sled) {
        guard.sled_wreck = guard.position;
        guard.sled_drift = 10.0f;
        guard.fall_velocity += forward * 6.0f;
    }
    guard.position += kUp * (guard.sled ? 1.20f : 0.88f);
    guard.fall_spin = std::copysign(1.0f, dot(impulse, forward)) *
                      (guard.sled ? 5.2f : 4.6f);
    if (lock_ && &guards_[*lock_] == &guard) lock_.reset();
    sounds.push_back({guard.sled ? "SFX_sno_Med1" : "SFX_HDeath1", guard.position, 0.85f});
}

void AlpineEscape::advance_fall(Guard& guard, const DriveSession& session) {
    if (guard.sled && guard.sled_drift > 0.0f) {
        const Vec3 advance = direction(guard.yaw, 0) * (guard.sled_drift * kStep);
        const Vec3 next = guard.sled_wreck + advance;
        GroundHit ground;
        if (session.collision().ground_below(next + kUp * 0.65f, ground) &&
            std::abs(ground.point[1] - guard.sled_wreck[1]) < 0.68f)
            guard.sled_wreck = ground.point;
        guard.sled_drift = std::max(0.0f, guard.sled_drift - 12.0f * kStep);
        guard.sled_roll = 0.68f * (1.0f - guard.sled_drift / 10.0f);
    }
    if (guard.landed) return;
    guard.fall_velocity[1] -= 9.8f * kStep;
    guard.fall_angle += guard.fall_spin * kStep;
    const float laid_out = std::copysign(1.48f, guard.fall_spin);
    const Vec3 next = guard.position + guard.fall_velocity * kStep;
    SegmentHit wall;
    if (session.collision().segment_hit(guard.position, next, wall)) {
        guard.position = wall.point + wall.normal * 0.09f;
        guard.fall_velocity = (guard.fall_velocity - wall.normal * dot(guard.fall_velocity, wall.normal)) * 0.40f;
    } else {
        guard.position = next;
    }
    const float floor_offset = 0.21f + 0.59f * std::max(0.0f, std::cos(guard.fall_angle));
    GroundHit ground;
    if (guard.fall_velocity[1] <= 0 &&
        session.collision().ground_below(guard.position + kUp * 0.35f, ground) &&
        guard.position[1] <= ground.point[1] + floor_offset) {
        guard.position[1] = ground.point[1] + floor_offset;
        guard.fall_velocity = {guard.fall_velocity[0] * 0.93f, 0, guard.fall_velocity[2] * 0.93f};
        guard.fall_angle = laid_out + (guard.fall_angle - laid_out) * 0.91f;
        guard.fall_spin *= 0.77f;
        guard.landed = length(guard.fall_velocity) < 0.10f && std::abs(guard.fall_angle - laid_out) < 0.06f;
    }
}

AlpineEscape::Barrel* AlpineEscape::barrel_hit(const Vec3& origin, const Vec3& direction, float& nearest) {
    Barrel* target = nullptr;
    for (auto& barrel : barrels_) {
        if (barrel.destroyed) continue;
        const Vec3 offset = barrel.center - origin;
        const float along = dot(offset, direction);
        const Vec3 lateral = offset - direction * along;
        const float radius_squared = kBarrelRadius * kBarrelRadius - dot(lateral, lateral);
        if (along <= 0 || radius_squared <= 0) continue;
        const float distance = std::max(0.0f, along - std::sqrt(radius_squared));
        if (distance < nearest) {
            nearest = distance;
            target = &barrel;
        }
    }
    return target;
}

void AlpineEscape::explode(const Vec3& center, float scale, float radius, std::string_view sound,
                           const DriveSession& session, std::vector<SoundEvent>& sounds) {
    blasts_.push_back({center, scale});
    sounds.push_back({std::string(sound), center, 1.0f});
    for (Guard& guard : guards_) {
        const Vec3 offset = aim_point(guard) - center;
        const float distance = length(offset);
        if (!guard.active || guard.health <= 0 || distance >= radius) continue;
        SegmentHit obstruction;
        if (session.collision().segment_hit(center, aim_point(guard), obstruction) && obstruction.t < 0.98f) continue;
        const float force = 1.0f - distance / radius;
        damage_guard(guard, 84.0f * force, unit(offset) * (2.5f + 7.0f * force) + kUp * 2.0f, sounds);
    }
    for (auto& barrel : barrels_) {
        if (barrel.destroyed || length(center - barrel.center) > std::min(3.0f * scale, radius)) continue;
        SegmentHit obstruction;
        if (!session.collision().segment_hit(center, barrel.center, obstruction) || obstruction.t > 0.98f)
            detonate(barrel, session, sounds);
    }
}

void AlpineEscape::detonate(Barrel& barrel, const DriveSession& session, std::vector<SoundEvent>& sounds) {
    barrel.destroyed = true;
    if (length(kHillToken - barrel.center) < 6.8f) token_revealed_ = true;
    explode(barrel.center, 0.72f, 6.6f, "SFX_sno_MedExp", session, sounds);
}

float AlpineEscape::tick_guards(const DriveSession& session, std::vector<SoundEvent>& sounds) {
    float damage = 0;
    for (std::size_t i = 0; i < guards_.size(); ++i) {
        Guard& g = guards_[i];
        if (g.health <= 0) {
            advance_fall(g, session);
            continue;
        }
        if (clock_ < g.appears_at) continue;
        if (g.sled) {
            const bool was_active = g.active;
            if (i == chalet_pursuer_) {
                g.active = stage_ == Stage::ChaletExit || chalet_chase();
                if (stage_ == Stage::ChaletExit) {
                    const Mat4 pose = chalet_pursuer_path_.pose(stage_time_ * kSceneFps);
                    g.position = {pose[12], pose[13] - 0.7f, pose[14]};
                    g.yaw = yaw_to({pose[0], pose[1], pose[2]});
                    g.turn_rate = 0;
                } else if (chalet_chase()) {
                    const float lane = g.lane + 0.65f * std::cos(stage_time_ * 1.3f);
                    place_sled(g, session, chase_distance_ + g.lead, lane, false, was_active);
                    if (clock_ >= g.fire_at && visible(session, aim_point(g))) {
                        g.fire_at = clock_ + 0.95f;
                        sounds.push_back({"SFX_sno_HmntmachFire", g.position, 0.56f});
                        damage += 1.1f;
                    }
                }
                continue;
            }
            const bool oncoming = i == oncoming_rider_;
            g.active = oncoming ? stage_ == Stage::HeadOn
                : g.forest ? stage_ == Stage::ForestRun && session.player_position()[0] > 1215.0f
                : stage_ == Stage::Pursuit && !grate_broken_;
            if (!g.active) continue;
            if (oncoming) {
                // A separate opposing line keeps the two sleds clear when they pass.
                const float gap = 36.0f - 16.0f * stage_time_;
                place_sled(g, session, chase_distance_ + gap,
                           g.lane - 2.8f * std::clamp(stage_time_ - 0.9f, 0.0f, 1.0f), true, was_active);
            } else {
                const float wave = stage_time_ * 0.7f + float(i) * 1.9f;
                const float lead = g.lead + std::sin(wave) * (g.forest ? 9.0f : 13.0f);
                place_sled(g, session, chase_distance_ + lead, g.lane + std::cos(wave) * 1.3f, false, was_active);
                const float distance = length(session.player_position() - g.position);
                if (clock_ >= g.fire_at && distance < 38.0f && visible(session, aim_point(g))) {
                    g.fire_at = clock_ + 1.45f;
                    sounds.push_back({"SFX_par_hmachgun1", g.position, 0.43f});
                    damage += g.forest ? 0.55f : distance < 16.0f ? 1.6f : 0.6f;
                }
            }
            continue;
        }
        const Vec3 to_bond = session.player_position() - g.position;
        const float distance = length(to_bond);
        g.active = distance < 95.0f;
        if (!g.active) continue;
        if (distance > 0.0001f) g.yaw = yaw_to(to_bond);
        Vec3 travel = g.destination - g.position;
        travel[1] = 0;
        const float remaining = length(travel);
        g.step_distance = std::min(remaining, 2.4f * kStep);
        if (g.step_distance > 0.0001f) {
            g.position += unit(travel) * g.step_distance;
            g.yaw = yaw_to(travel);
        }
        GroundHit ground;
        if (session.collision().ground_below(g.position + kUp * 0.8f, ground) &&
            std::abs(ground.point[1] - g.position[1]) < 1.8f) g.position[1] = ground.point[1];
        if (clock_ >= g.fire_at && distance < 48.0f && !exterior() && visible(session, aim_point(g))) {
            g.fire_at = clock_ + 1.2f;
            sounds.push_back({"SFX_par_hmachgun1", g.position, 0.37f});
            damage += gate_waiting() ? 0.95f : distance < 24.0f ? 2.2f : 0.75f;
        }
    }
    return damage;
}

void AlpineEscape::tick_gun(const PadState& pad, const DriveSession& session, std::vector<SoundEvent>& sounds) {
    const bool manual = pad.held(kPadL1);
    const float aim_rate = manual ? 0.65f : 1.0f;
    const float stick_yaw = pad_stick_axis(pad.lx) + pad_stick_axis(pad.rx);
    const float stick_pitch = pad_stick_axis(pad.ly) + pad_stick_axis(pad.ry);
    const float delta_yaw = (std::clamp(stick_yaw, -1.0f, 1.0f) * 1.8f * kStep - pad.mouse.yaw) * aim_rate;
    const float delta_pitch = (-std::clamp(stick_pitch, -1.0f, 1.0f) * 1.55f * kStep + pad.mouse.pitch) * aim_rate;
    yaw_ = wrap(yaw_ + delta_yaw);
    pitch_ = std::clamp(pitch_ + delta_pitch, -0.63f, 0.72f);
    const Vec3 raw_aim = direction(heading_ + yaw_, pitch_);
    auto score = [&](const Guard& g) {
        const Vec3 delta = aim_point(g) - camera_.eye;
        const float distance = length(delta);
        if (!g.active || g.health <= 0 || distance < 0.01f || distance > kAimRange) return -1.0f;
        return dot(delta * (1.0f / distance), raw_aim);
    };
    if (grenade_selected_ || manual || (lock_ && (score(guards_[*lock_]) < kReleaseDot ||
                             !visible(session, aim_point(guards_[*lock_]))))) lock_.reset();
    if (!grenade_selected_ && !manual && !lock_) {
        float best = kAcquireDot;
        for (std::size_t i = 0; i < guards_.size(); ++i) {
            const float alignment = score(guards_[i]);
            if (alignment <= best || !visible(session, aim_point(guards_[i]))) continue;
            best = alignment;
            lock_ = i;
        }
    }

    update_camera(session);
    if (manual) camera_.fovy *= 0.76f;
    Vec3 firing = raw_aim;
    if (lock_) {
        firing = unit(aim_point(guards_[*lock_]) - camera_.eye);
        // Apply the magnetism to the view only. Keeping the raw angles independent
        // lets continued stick or mouse movement break the lock cleanly.
        camera_.target = camera_.eye + unit(raw_aim * 0.22f + firing * 0.78f);
    }
    if (!pad.held(kPadR1) || clock_ < next_shot_) return;
    if (grenade_selected_) {
        if (grenade_.life > 0) return;
        next_shot_ = clock_ + 0.55f;
        if (grenade_ammo_ == 0) {
            sounds.push_back({"SFX_DryFire", camera_.eye, 0.5f});
            return;
        }
        --grenade_ammo_;
        grenade_ = {.pos=camera_.eye + firing * 1.25f, .vel=firing * 36.0f + kUp * 1.7f, .life=2.0f};
        sounds.push_back({"SFX_grenadefire", camera_.eye, 0.85f});
        return;
    }
    next_shot_ = clock_ + 0.083f;
    if (ammo_ == 0) {
        sounds.push_back({"SFX_DryFire", camera_.eye, 0.5f});
        return;
    }
    --ammo_;
    shot_seconds_ = 0.055f;
    sounds.push_back({"SFX_sno_snomachFire", camera_.eye, 0.85f});
    shot_end_ = camera_.eye + firing * kAimRange;
    SegmentHit hit;
    if (session.collision().segment_hit(camera_.eye, shot_end_, hit)) shot_end_ = hit.point;
    if (stage_ == Stage::Garage && door_lift_ < 2.0f) return;
    // Resolve direct hits in manual aim too. Select the closest visible hit
    // along the ray before the background impact.
    float nearest = length(shot_end_ - camera_.eye) + 0.10f;
    Guard* victim = nullptr;
    for (Guard& g : guards_) {
        const Vec3 offset = aim_point(g) - camera_.eye;
        const float along = dot(offset, firing);
        if (g.active && g.health > 0 && along > 0 && along < nearest &&
            length(offset - firing * along) < (g.sled ? 0.75f : 0.46f) && visible(session, aim_point(g))) {
            nearest = along;
            victim = &g;
        }
    }
    const float guard_distance = nearest;
    if (Barrel* barrel = barrel_hit(camera_.eye, firing, nearest);
        barrel && visible(session, camera_.eye + firing * nearest)) {
        shot_end_ = camera_.eye + firing * nearest;
        detonate(*barrel, session, sounds);
    } else if (victim) {
        shot_end_ = camera_.eye + firing * guard_distance;
        damage_guard(*victim, 12.0f, firing * 2.5f + kUp * 0.85f, sounds);
    }
}

void AlpineEscape::update_grenade(const DriveSession& session, std::vector<SoundEvent>& sounds) {
    if (grenade_.life <= 0) return;
    grenade_.life -= kStep;
    grenade_.vel[1] -= 9.8f * kStep;
    const Vec3 previous = grenade_.pos;
    const Vec3 destination = previous + grenade_.vel * kStep;
    SegmentHit hit;
    const bool impact = session.collision().segment_hit(previous, destination, hit);
    const Vec3 travel = destination - previous;
    float distance = length(impact ? hit.point - previous : travel);
    if (Barrel* barrel = barrel_hit(previous, unit(travel), distance)) {
        grenade_.pos = previous + unit(travel) * distance;
        grenade_.life = 0;
        detonate(*barrel, session, sounds);
        return;
    }
    grenade_.pos = impact ? hit.point : destination;
    const float gate_plane = kGateCenter[0] - 0.7f;
    if (stage_ == Stage::GateHold && !gate_destroyed_ && previous[0] < gate_plane && destination[0] >= gate_plane &&
        std::abs(grenade_.pos[2] - kGateCenter[2]) < 5.7f && std::abs(grenade_.pos[1] - kGateCenter[1]) < 2.7f) {
        gate_destroyed_ = true;
        grenade_.life = 0;
        explode(kGateCenter, 1.0f, 6.6f, "SFX_sno_LrgExp", session, sounds);
    } else if (impact || grenade_.life <= 0) {
        grenade_.life = 0;
        explode(grenade_.pos + kUp * 0.1f, 0.6f, 4.5f, "SFX_sno_SmallExp", session, sounds);
    }
}

float AlpineEscape::tick(const PadState& pad, DriveSession& session, std::vector<SoundEvent>& sounds) {
    const bool starting = clock_ == 0;
    const Stage previous_stage = stage_;
    clock_ += kStep;
    stage_time_ += kStep;
    shot_seconds_ = std::max(0.0f, shot_seconds_ - kStep);
    const bool door_was_closed = door_lift_ == 0;
    const bool grate_was_intact = !grate_broken_;
    blasts_.clear();
    token_notice_ = std::max(0.0f, token_notice_ - kStep);
    advance_driver(session);
    update_helicopter();
    if (in_chalet() && !chalet_door_broken()) {
        chalet_door_at_ = clock_;
        sounds.push_back({"SFX_WoodStruct2", kChaletDoor, 1.0f});
    }
    if (stage_ == Stage::ChaletExit && !chalet_window_broken() && session.player_position()[0] > kChaletWindow[0]) {
        chalet_window_at_ = clock_;
        sounds.push_back({"SFX_GlassShatter2", kChaletWindow, 1.0f});
    }
    if (stage_ == Stage::BridgeCut && stage_time_ * kSceneFps >= kBridgeFireFrame &&
        (stage_time_ - kStep) * kSceneFps < kBridgeFireFrame) {
        if (const auto rocket = bridge_rocket()) sounds.push_back({"SFX_RocketFire", *rocket, 1.0f});
    }
    if (stage_ == Stage::BridgeCut && !bridge_destroyed() && stage_time_ * kSceneFps >= kBridgeBlastFrame) {
        bridge_destroyed_at_ = clock_;
        explode(kBridgeCenter, 4.1f, 13.0f, "SFX_sno_HugeExp", session, sounds);
    }
    if (stage_ == Stage::Escape && exit_door_lift_ >= 3.2f && departure_ < kDepartureCutEnd) {
        const bool had_timer_cut = timer_cut();
        const int digit = timer_digit();
        departure_ += kStep;
        if (timer_cut() && (!had_timer_cut || digit != timer_digit()))
            sounds.push_back({"SFX_sno_timerbeep", kBombPosition, 0.92f});
        if (garage_blast_cut() && !garage_destroyed()) {
            garage_detonated_at_ = clock_;
            explode(kGarageBlast, 4.6f, 12.0f, "SFX_sno_HugeExp", session, sounds);
        }
    }
    const Vec3 token_delta = session.player_position() - kHillToken;
    if (token_visible() && std::abs(token_delta[1]) < 3.0f && std::hypot(token_delta[0], token_delta[2]) < 3.4f) {
        token_collected_ = true;
        token_notice_ = 2.4f;
        sounds.push_back({std::string(kSpeech[5]), {}, 1.0f, true});
    }
    if (stage_ == Stage::ForestRun && !collected_grenades_ && session.player_position()[0] >= 1430.0f) {
        collected_grenades_ = true;
        grenade_ammo_ = 21;
        sounds.push_back({"SFX_PowerupWeap", session.player_position(), 0.85f});
    }
    if (starting)
        sounds.push_back({std::string(kSpeech[0]), {}, 1.0f, true});
    if (door_was_closed && door_lift_ > 0)
        sounds.push_back({"SFX_sno_garagedoor", kGarageDoor, 0.85f});
    if (grate_was_intact && grate_broken_)
        sounds.push_back({"SFX_MetalGate1", kGratePosition, 1.0f});
    if (previous_stage != stage_) {
        if (stage_ == Stage::Escape) {
            sounds.push_back({"SFX_sno_garagedoor", kExitDoor, 0.85f});
            sounds.push_back({std::string(kSpeech[1]), {}, 1.0f, true});
        } else if (stage_ == Stage::Chase)
            sounds.push_back({std::string(kSpeech[2]), {}, 1.0f, true});
        else if (stage_ == Stage::HeadOn)
            sounds.push_back({std::string(kSpeech[3]), {}, 0.88f, true});
        else if (stage_ == Stage::GateCut)
            sounds.push_back({std::string(kSpeech[4]), {}, 0.93f, true});
        else if (stage_ == Stage::BridgeCut)
            sounds.push_back({std::string(kSpeech[6]), {}, 0.93f, true});
        else if (stage_ == Stage::ChaletExit)
            sounds.push_back({std::string(kSpeech[7]), {}, 0.93f, true});
    }
    update_camera(session);
    if (pursuing_helicopter() && !ravine_armor_collected_ &&
        length(session.player_position() - kRavineArmor) < 2.2f) {
        ravine_armor_collected_ = true;
        armor_points_ = 50.0f;
        sounds.push_back({"SFX_pickuparmor", kRavineArmor, 0.85f});
    }
    const float guard_damage = tick_guards(session, sounds);
    const float damage = guard_damage + tick_helicopter_attack(session, sounds);
    if (exterior()) update_camera(session);
    took_damage_ = damage > 0;
    const float absorbed = std::min(armor_points_, damage);
    armor_points_ -= absorbed;
    const bool switch_down = pad.held(kPadR2) || pad.held(kPadTriangle);
    if (!exterior()) {
        if (switch_down && !switch_was_down_ && (grenade_ammo_ > 0 || grenade_selected_))
            grenade_selected_ = !grenade_selected_;
        tick_gun(pad, session, sounds);
    }
    switch_was_down_ = switch_down;
    update_grenade(session, sounds);
    if (gate_waiting() && grenade_ammo_ == 0 && grenade_.life <= 0 && !gate_destroyed_) {
        grenade_ammo_ = 3;
        sounds.push_back({"SFX_PowerupWeap", session.player_position(), 0.85f});
    }
    return damage - absorbed;
}

}  // namespace nf::driving
