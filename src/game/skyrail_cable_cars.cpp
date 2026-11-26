#include "game/skyrail_cable_cars.hpp"

#include <algorithm>

#include "assets/anim.hpp"
#include "assets/cutscene.hpp"

namespace nf {

namespace {

constexpr std::uint32_t kScript = 0x06000058;
constexpr std::uint32_t kMap = 0x010000FA;

}  // namespace

SkyrailCableCars::SkyrailCableCars(World& world) {
    Level& level = world.level();
    if (!level.map() || level.map()->entry.hash != kMap) return;
    const auto& placements = level.placements();
    const auto& statics = level.map()->chunk.statics;
    const auto anchor = std::find_if(placements.begin(), placements.end(), [&](const Placement& p) {
        const auto& instance = statics[p.instance];
        return instance.object_class() == 0xD9 && instance.param(0) == kScript && instance.param(2) == 8;
    });
    const Bytes script = level.script(kScript);
    if (anchor == placements.end() || script.empty()) return;
    const CutsceneBin bin = parse_cutscene_bin(script);
    // Time 7590 is the last frame. Script_StreamEnd resets the stream for the next frame.
    period_ = float(bin.d) + 1.0f;
    anchor_ = anchor->transform;
    for (std::size_t i = 0; i < bin.e; ++i) {
        const Bytes key = bin.keys.subspan(i * 48, 48);
        times_.push_back(load<float>(key, 44));
        positions_.push_back({load<float>(key, 0), load<float>(key, 4), load<float>(key, 8)});
        rotations_.push_back({load<float>(key, 16), load<float>(key, 20), load<float>(key, 24), load<float>(key, 28)});
    }

    std::vector<int> pose_for_stream(bin.scripts.size(), -1);
    for (std::size_t i = 0; i < bin.scripts.size(); ++i) {
        const auto& stream = bin.scripts[i];
        const auto commands = decode_stream(stream);
        const auto start = std::find_if(commands.begin(), commands.end(), [](const ScriptCommand& command) {
            return command.op == std::uint8_t(CutsceneOp::EntityStart);
        });
        if (start == commands.end()) continue;  // The unused stream still occupies a directory slot.
        if (stream.id > stream.flags || stream.flags >= times_.size() || start->payload.size() < 7 ||
            start->payload[6] != 0)
            throw FormatError("invalid Skyrail cable-car linear track");
        ScriptCursor args(start->payload);
        const std::uint32_t hash = args.u32();
        const bool linked = args.u8() != 0;
        const std::uint8_t link = args.u8();
        const auto model = std::find_if(placements.begin(), placements.end(), [&](const Placement& placed) {
            return level.chunks()[placed.chunk].chunk.models[placed.model].hash == std::int32_t(hash);
        });
        if (model == placements.end()) throw FormatError("missing Skyrail cable-car model");
        int parent = -1;
        if (linked && link != 0xFF) {
            if (link >= i || pose_for_stream[link] < 0) throw FormatError("invalid Skyrail cable-car parent");
            parent = pose_for_stream[link];
        }
        pose_for_stream[i] = int(poses_.size());
        tracks_.push_back({stream.id, stream.flags, parent});
        poses_.push_back(*model);
        const auto marker = std::size_t(model - placements.begin());
        if (std::find(markers_.begin(), markers_.end(), marker) == markers_.end()) markers_.push_back(marker);
    }
    if (poses_.empty()) return;
    first_handle_ = placements.size();
    collision_ = &world.objects().enable_moving_meshes();
    sample(0.0f);
    for (std::size_t i = 0; i < poses_.size(); ++i) {
        const auto& p = poses_[i];
        collision_->add_model(level, first_handle_ + i, p.chunk, p.model, p.transform);
    }
}

void SkyrailCableCars::sample(float frame) {
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
        const auto& track = tracks_[i];
        const auto seg = spline_segment(times_, frame, track.begin, track.end);
        const auto a = std::size_t(seg.i1);
        const auto b = std::size_t(seg.i2);
        const auto q = slerp_acc(rotations_[a], rotations_[b], seg.f);
        const Vec3 p = positions_[a] * (1.0f - seg.f) + positions_[b] * seg.f;
        const Mat4 local = quat_trans_to_mat({q[0], q[1], q[2], q[3]}, p);
        const Mat4& parent = track.parent < 0 ? anchor_ : poses_[std::size_t(track.parent)].transform;
        poses_[i].transform = mul(parent, local);
    }
}

void SkyrailCableCars::before_player_update(World& world, FrameTiming timing) {
    if (!collision_) return;
    sample(clock_);
    clock_ += timing.mul();
    if (clock_ >= period_) clock_ = 0;
    for (int slot = 0; slot < World::kMaxPlayers; ++slot) {
        Player* player = world.player(slot);
        if (!player || !player->alive() || (player->body_flags & body::kOnGround) == 0 ||
            (player->substate != SubState::Walk && player->substate != SubState::Crouch)) continue;
        const Vec3 sole = player->pos - Vec3{0, player->stand_height, 0};
        const Vec3 from = sole + Vec3{0, 0.2f, 0};
        const Vec3 to = sole - Vec3{0, 0.3f, 0};
        const auto support = collision_->ray(from, to, pick::kIgnoreGhost);
        if (!support || support->normal[1] < 0.5f) continue;
        const auto floor = world.collision().ray(from, to, pick::kIgnoreGhost);
        if (floor && floor->dist < support->dist - 0.02f) continue;
        const auto& next = poses_[support->placement - first_handle_].transform;
        const Vec3 carried = collision_->carry_point(support->placement, next, player->pos);
        player->settled_pos += carried - player->pos;
        player->pos = carried;
    }
    for (std::size_t i = 0; i < poses_.size(); ++i)
        collision_->move_model(first_handle_ + i, poses_[i].transform);
}

}  // namespace nf
