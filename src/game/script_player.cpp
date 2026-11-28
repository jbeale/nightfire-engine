#include "game/script_player.hpp"
#include "assets/character.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace nf {

namespace {

std::uint32_t payload_u32(const std::vector<std::uint8_t>& p, std::size_t o) {
    return std::uint32_t(p[o]) | (std::uint32_t(p[o + 1]) << 8) | (std::uint32_t(p[o + 2]) << 16) |
           (std::uint32_t(p[o + 3]) << 24);
}
std::uint16_t payload_u16(const std::vector<std::uint8_t>& p, std::size_t o) {
    return std::uint16_t(p[o] | (p[o + 1] << 8));
}
float payload_f32(const std::vector<std::uint8_t>& p, std::size_t o) {
    const std::uint32_t v = payload_u32(p, o);
    float f = 0;
    std::memcpy(&f, &v, 4);
    return f;
}

}  // namespace

CutscenePlayer::CutscenePlayer(const CutsceneBin* bin, std::uint32_t id, Host* host, const CharacterBank* bank)
    : bin_(bin), id_(id), host_(host), bank_(bank), end_time_(float(bin->d)) {
    for (const CutsceneScript& s : bin->scripts) {
        Stream st;
        st.cmds = decode_stream(s);
        st.entity_first = s.id;
        st.entity_last = s.flags;
        streams_.push_back(std::move(st));
    }
}

void CutscenePlayer::play(bool fast) {
    (void)fast;  // `Script_FFwd` timescale: NIS skip is owned by the mission system (flag 0x80 paths)
    for (Stream& s : streams_) {
        s.cursor = 0;
        s.time = 0;
        s.next = 0;
        s.done = false;
        s.has_entity = false;
        s.detached_entity = false;
        s.actor = {};
        s.sprite = {};
        s.subscript.reset();
        s.subscript_bin.reset();
        s.sound_id = 0;
        s.sound_source = -1;
        s.has_camera = false;
        s.camera_live = false;
    }
    time_ = 0;
    playing_ = true;
    staged_ = false;
    detach_entities_ = false;
    locks_player_ = false;
    locks_drones_ = false;
    handoff_.reset();
}

void CutscenePlayer::stage() {
    play(false);
    tick(0);
    staged_ = true;
}

void CutscenePlayer::animation_sounds(Stream& stream, float frame) {
    if (bank_ && stream.actor.skin != 0) {
        if (const auto* script = bank_->script(stream.actor.script)) {
            for (const auto& command : script->cmds) {
                if (command.op != kScriptSound || command.words.size() < 2 ||
                    command.words[0] <= stream.last_actor_frame || command.words[0] > frame) continue;
                Sound cue;
                cue.id = command.words[1];
                cue.positional = true;
                cue.actor = actor_pose(stream);
                cue.actor.frame = float(command.words[0]);
                cue.animation = true;
                sounds_.push_back(cue);
            }
        }
    }
    stream.last_actor_frame = frame;
}

void CutscenePlayer::stop() {
    for (Stream& stream : streams_) cue_sound(stream, SoundAction::End);
    playing_ = false;
    if (depth_ == 0) host_->set_scriptcam(0);
}

void CutscenePlayer::cue_sound(Stream& stream, SoundAction action) {
    if (stream.sound_id == 0) return;
    Sound cue;
    cue.id = stream.sound_id;
    cue.stream = std::uint16_t(&stream - streams_.data());
    cue.action = action;
    if (action != SoundAction::End && stream.sound_source >= 0) {
        const Stream& source = streams_[std::size_t(stream.sound_source)];
        cue.positional = true;
        cue.pos = source.epos;
        cue.actor = actor_pose(source);
        if (cue.actor.skin != 0) cue.actor.frame += source.time - source.entity_start;
    }
    sounds_.push_back(cue);
    if (action == SoundAction::End) stream.sound_id = 0;
}

std::size_t CutscenePlayer::key_count() const { return bin_->keys.size() / 48; }

std::array<float, 3> CutscenePlayer::key_pos(std::size_t i) const {
    std::array<float, 3> out{};
    const std::size_t o = i * 48;
    for (int k = 0; k < 3; ++k) {
        std::uint32_t v = load<std::uint32_t>(bin_->keys, o + std::size_t(4 * k));
        std::memcpy(&out[std::size_t(k)], &v, 4);
    }
    return out;
}

std::array<float, 4> CutscenePlayer::key_quat(std::size_t i) const {
    std::array<float, 4> out{0, 0, 0, 1};
    const std::size_t o = i * 48 + 16;
    for (int k = 0; k < 4; ++k) {
        std::uint32_t v = load<std::uint32_t>(bin_->keys, o + std::size_t(4 * k));
        std::memcpy(&out[std::size_t(k)], &v, 4);
    }
    return out;
}

float CutscenePlayer::key_time(std::size_t i) const {
    std::uint32_t v = load<std::uint32_t>(bin_->keys, i * 48 + 44);
    float f = 0;
    std::memcpy(&f, &v, 4);
    return f;
}

float CutscenePlayer::key_aux(std::size_t i) const {
    std::uint32_t v = load<std::uint32_t>(bin_->keys, i * 48 + 32);
    float f = 0;
    std::memcpy(&f, &v, 4);
    return f;
}

void CutscenePlayer::interpolate(Stream& s) {
    // `Script_GetInterp`: sample the key track at the stream time (spline or linear blend,
    // slerped rotation, +32 fov). Segment search, weights and blends mirror the originals.
    const std::size_t n = key_count();
    if (n == 0) return;
    std::size_t begin = 0, end = n - 1;
    if (s.has_camera && s.key_begin <= s.key_end && s.key_end < n) {
        begin = s.key_begin;
        end = s.key_end;
    } else if (s.has_entity && s.entity_first <= s.entity_last && s.entity_last < n) {
        begin = s.entity_first;
        end = s.entity_last;
    }
    if (begin >= end) {
        if (s.has_camera) {
            const auto p = key_pos(begin);
            const auto q = key_quat(begin);
            s.eye = p;
            s.forward = {2 * (q[0] * q[2] + q[3] * q[1]), 2 * (q[1] * q[2] - q[3] * q[0]),
                         1 - 2 * (q[0] * q[0] + q[1] * q[1])};
            s.fov = std::clamp(key_aux(begin) * (4.0f / 3.0f), 0.1f, 2.5f);
        }
        if (s.has_entity) {
            s.epos = key_pos(begin);
            s.equat = key_quat(begin);
        }
        return;
    }
    std::vector<float> times;
    times.reserve(n);
    for (std::size_t k = 0; k < n; ++k) times.push_back(key_time(k));
    const SplineSegment seg = spline_segment(times, s.time, begin, end);
    const auto pa = key_pos(std::size_t(seg.i1)), pb = key_pos(std::size_t(seg.i2));
    const auto qa = key_quat(std::size_t(seg.i1)), qb = key_quat(std::size_t(seg.i2));
    const auto q = slerp_acc(qa, qb, seg.f);
    std::array<float, 3> p;
    float fov = 0.7853982f;
    if (s.spline) {
        const auto w = spline_weights(seg.f);
        const auto p0 = key_pos(std::size_t(seg.i0)), p3 = key_pos(std::size_t(seg.i3));
        p = spline_eval3d(p0, pa, pb, p3, seg.f);
        fov = key_aux(std::size_t(seg.i0)) * w[0] + key_aux(std::size_t(seg.i1)) * w[1] +
              key_aux(std::size_t(seg.i2)) * w[2] + key_aux(std::size_t(seg.i3)) * w[3];
    } else {
        const float w0 = 1.0f - seg.f, w1 = seg.f;
        for (int k = 0; k < 3; ++k) p[std::size_t(k)] = pa[std::size_t(k)] * w0 + pb[std::size_t(k)] * w1;
        fov = key_aux(std::size_t(seg.i1)) * w0 + key_aux(std::size_t(seg.i2)) * w1;
    }
    if (s.has_camera) {
        s.eye = p;
        s.forward = {2 * (q[0] * q[2] + q[3] * q[1]), 2 * (q[1] * q[2] - q[3] * q[0]),
                     1 - 2 * (q[0] * q[0] + q[1] * q[1])};
        s.fov = std::clamp(fov * (4.0f / 3.0f), 0.1f, 2.5f);
    }
    if (s.has_entity) {
        s.epos = p;
        s.equat = q;
    }
}

void CutscenePlayer::fire(const ScriptCommand& cmd, Stream& s) {
    const auto& p = cmd.payload;
    switch (CutsceneOp(cmd.op)) {
    case CutsceneOp::EntityStart: {
        // dword entity hash + parent flag/index + spline flag + kind + extra-dword count.
        s.has_entity = true;
        s.detached_entity = false;
        s.entity_hash = payload_u32(p, 0);
        s.entity_kind = p[7];
        s.entity_params.fill(0);
        for (std::size_t i = 0; i < std::min<std::size_t>(p[8], s.entity_params.size()); ++i)
            s.entity_params[i] = payload_u32(p, 9 + 4 * i);
        s.obj_ref = p[4] != 0 ? int(p[5]) : -1;
        s.spline = p[6] != 0;
        s.entity_start = s.time;
        interpolate(s);
        break;
    }
    case CutsceneOp::EntityEnd:
        if (detach_entities_ && s.has_entity) {
            interpolate(s);
            s.detached_entity = true;
            break;
        }
        s.has_entity = false;
        s.obj_ref = -1;
        break;
    case CutsceneOp::AnimStart:
        s.actor = {payload_u32(p, 4), payload_u32(p, 0), float(payload_u32(p, 8))};
        s.last_actor_frame = s.actor.frame - 1;
        s.entity_start = s.time;
        break;
    case CutsceneOp::AnimEnd:
        animation_sounds(s, s.actor.frame + cmd.time - s.entity_start);
        s.actor = {};
        break;
    case CutsceneOp::CameraStart: {
        // u8 mode + u16 key start + u16 key end (`ScriptCam` = this script while live).
        s.has_camera = true;
        s.camera_live = true;
        s.key_begin = payload_u16(p, 1);
        s.spline = p[0] != 0;
        const auto end = payload_u16(p, 3);
        s.key_end = end > 0 ? end - 1 : 0;
        host_->set_scriptcam(id_);
        host_->music(11, std::int32_t(id_));  // kNisStarted
        break;
    }
    case CutsceneOp::CameraEnd:
        s.camera_live = false;
        host_->set_scriptcam(0);
        host_->music(12, std::int32_t(id_));  // kNisEnded
        break;
    case CutsceneOp::TextStart:
        host_->text(payload_u32(p, 0), payload_u16(p, 4));
        break;
    case CutsceneOp::SoundStart: {
        cue_sound(s, SoundAction::End);
        s.sound_id = payload_u32(p, 0) & 0xFFFFF;
        // Script_SoundStart addresses a stream by its directory index. With no explicit
        // index it uses the script's last object, and an absent object falls back to 2D.
        const std::size_t stream = std::size_t(&s - streams_.data());
        const std::size_t source = p[5] != 0 ? p[6] : (stream != 0 ? stream - 1 : 0);
        s.sound_source = -1;
        if (p[4] == 0 && source < streams_.size() &&
            (streams_[source].has_entity || streams_[source].actor.skin != 0))
            s.sound_source = int(source);
        cue_sound(s, SoundAction::Start);
        break;
    }
    case CutsceneOp::SoundEnd:
        cue_sound(s, SoundAction::End);
        break;
    case CutsceneOp::FadeStart: {
        float seconds = payload_f32(p, 0);
        host_->fade(seconds);
        break;
    }
    case CutsceneOp::Event: {
        const std::uint8_t ev = p[1];
        const std::size_t nargs = p[0];
        auto arg = [&](std::size_t k) { return nargs > k ? payload_u32(p, 2 + 4 * k) : 0; };
        // The event addresses the object in the previous stream.
        const Stream* marker = &s != streams_.data() ? &s - 1 : nullptr;
        const Vec3* at = marker && marker->has_entity ? &marker->epos : nullptr;
        switch (ev) {
        case 3:  // loop/restart bookkeeping: nothing to present
            break;
        case 4:
            locks_player_ = true;
            break;
        case 5:
            locks_drones_ = true;
            break;
        case 6: {  // Script_EventHandler uses the previous stream's object matrix.
            if (at) {
                const auto& q = marker->equat;
                const float yaw = std::atan2(2 * (q[0] * q[2] + q[3] * q[1]),
                                             1 - 2 * (q[0] * q[0] + q[1] * q[1]));
                handoff_ = PlayerHandoff{*at, yaw, std::uint8_t(arg(0))};
            }
            break;
        }
        case 7:  // flag 0x80 (NIS skip arm): nothing to present
            break;
        case 8: {  // `Drone_CoderCreate` at the object: spawn a drone with 4 args
            const std::uint32_t args[4] = {arg(0), arg(1), arg(2), arg(3)};
            if (at) host_->spawn_drone(*at, args);
            break;
        }
        case 9:
            host_->camera_mode(arg(0));  // `Player_Cam2Mode`
            break;
        case 10:
            if (at) host_->break_near(*at);  // `Break_DoBreak`
            break;
        case 11:
            if (at) host_->set_link_byte(*at, std::uint8_t(arg(0) & 0xFF));
            break;
        case 12:  // Script_KillStream leaves the object in the world at its last pose.
            detach_entities_ = true;
            break;
        case 14:  // stream flag: nothing to present
        case 16:  // flag 0x20: nothing to present
            break;
        case 13:
            if (arg(0) != 0) host_->set_channel(std::uint16_t(arg(0) & 0xFFFF), std::uint8_t(arg(1) & 0xFF));
            break;
        case 15:
            host_->message_callback(std::uint16_t(arg(0) & 0xFFFF), arg(1));
            break;
        case 17: {
            const std::uint32_t ch = arg(0);
            const std::uint32_t src = host_->channel(std::uint16_t(ch & 0xFFFF)) ? arg(1) : arg(2);
            if (src != 0) host_->set_channel(std::uint16_t(src & 0xFFFF), std::uint8_t(arg(3) & 0xFF));
            break;
        }
        case 18:
            host_->ram_save();
            host_->load_level(arg(0));  // `ResetMap_LevelToLoad(arg0, 1, 0)`
            break;
        case 19:
            host_->set_scriptcam(id_);
            break;
        default:
            break;
        }
        break;
    }
    case CutsceneOp::LightStart: {
        // `DynamicLights::create` at the entity position (Characters-2's map).
        Light l;
        l.intensity = p[1];
        l.type = p[2] == 1;
        if (key_count() > 0) {
            const float span = std::max(1.0f, end_time_ - s.entity_start);
            const float f = std::clamp((s.time - s.entity_start) / span, 0.0f, 1.0f);
            l.pos = key_pos(std::min(key_count() - 1, std::size_t(f * float(key_count() - 1))));
        }
        lights_.push_back(l);
        break;
    }
    case CutsceneOp::LightEnd:
        break;
    case CutsceneOp::SpriteStart:
        s.sprite = {payload_u32(p, 10), payload_u16(p, 0), payload_u16(p, 2),
                    payload_u16(p, 4), payload_u16(p, 6), payload_u16(p, 8)};
        break;
    case CutsceneOp::SpriteEnd:
        s.sprite = {};
        break;
    case CutsceneOp::SubScriptStart: {
        if (!bank_ || depth_ >= 8) break;
        const auto hash = payload_u32(p, 0);
        const auto found = bank_->cutscenes().find(hash);
        if (found == bank_->cutscenes().end()) break;
        s.subscript.reset();
        try {
            s.subscript_bin = std::make_unique<CutsceneBin>(parse_cutscene_bin(found->second));
            s.subscript = std::make_unique<CutscenePlayer>(s.subscript_bin.get(), hash, host_, bank_);
        } catch (const FormatError&) {
            break;
        }
        s.subscript->depth_ = depth_ + 1;
        s.has_entity = true; // its stream supplies a placement track for the child
        s.entity_hash = 0;
        s.obj_ref = p[4] != 0 ? int(p[5]) : -1;
        s.subscript->play(false);
        s.subscript->tick(0);
        break;
    }
    case CutsceneOp::SubScriptEnd:
        s.has_entity = false;
        s.subscript.reset();
        s.subscript_bin.reset();
        break;
    case CutsceneOp::StreamEnd:
    case CutsceneOp::Wait:
    case CutsceneOp::CondSkip:
        break;  // timing commands are handled by the playback loop
    }
}

void CutscenePlayer::tick(float frames) {
    if (!playing_ || staged_) return;
    time_ += frames;
    for (Stream& s : streams_) {
        if (s.done) continue;
        s.time += frames;
        // Fire every due command (`Script_Run`: while time >= next).
        while (s.cursor < s.cmds.size()) {
            const ScriptCommand& cmd = s.cmds[s.cursor];
            if (cmd.op == 4) {  // StreamEnd: wait for its word, then the pass ends
                const float word = float(cmd.payload[0] | (cmd.payload[1] << 8));
                s.next = word;
                if (s.time > word) {
                    s.done = true;  // no loop-back in the travelled data; hold the last frame
                    cue_sound(s, SoundAction::End);
                }
                break;
            }
            if (cmd.time > s.time) {
                s.next = cmd.time;
                break;
            }
            if (cmd.op == 5) {  // decode_stream already assigned absolute times to the commands.
                ++s.cursor;
                continue;
            }
            fire(cmd, s);
            ++s.cursor;
        }
        if (!s.done && !s.detached_entity) interpolate(s);
        if (s.subscript) {
            s.subscript->set_origin(mul(origin_, entity_transform(std::size_t(&s - streams_.data()))));
            s.subscript->tick(frames);
            if (!s.subscript->playing()) {
                const float carry = std::max(0.0f, s.subscript->time_ - s.subscript->end_time_);
                s.subscript->play(false);
                s.subscript->tick(carry);
            }
        }
        if (s.actor.skin != 0) animation_sounds(s, s.actor.frame + s.time - s.entity_start);
    }
    for (Stream& s : streams_) {
        if (!s.done && s.sound_source >= 0) cue_sound(s, SoundAction::Move);
    }
    // Child scripts repeat while the parent stream is active. Restart at the authored
    // duration so EntityEnd doesn't leave looping props hidden during the top-level grace period.
    if (time_ >= end_time_ && (depth_ != 0 || locks_player_ || locks_drones_ || time_ >= end_time_ + 60)) stop();
}

void CutscenePlayer::finish(float frames) {
    if (staged_ || !playing_ || !(frames > 0)) return;
    // Replay command order at the ordinary step size. Large jumps would run an entire
    // stream before its neighbors and can change switch reads and handoff markers.
    const auto keep_cleanup = [&] {
        std::erase_if(sounds_, [](const Sound& sound) { return sound.action != SoundAction::End; });
        lights_.clear();
    };
    while (playing_) {
        tick(frames);
        keep_cleanup();
    }
}

std::optional<CutscenePlayer::Camera> CutscenePlayer::camera() const {
    if (!playing_) return std::nullopt;
    for (const Stream& s : streams_) {
        if (s.has_camera && s.camera_live && !s.done) {
            Camera c;
            c.eye = s.eye;
            c.forward = s.forward;
            c.fov = s.fov;
            return c;
        }
    }
    return std::nullopt;
}

std::vector<CutscenePlayer::EntityPose> CutscenePlayer::entities() const {
    std::vector<EntityPose> out;
    for (const Stream& s : streams_) {
        if (playing_ && s.subscript) {
            auto children = s.subscript->entities();
            out.insert(out.end(), children.begin(), children.end());
        }
        if (!s.has_entity || (!playing_ && !s.detached_entity)) continue;
        if (key_count() == 0 && s.done) continue;
        // Interpolated pose (`Script_GetInterp` sampling, stored per tick); holds the last
        // pose when the stream is done, and the rest pose before the first tick. Keyless effect
        // scripts (explosion NIS) report the rest pose; the presenter adds the blast base.
        EntityPose e;
        e.hash = s.entity_hash;
        e.pos = s.epos;
        e.quat = s.equat;
        e.kind = s.entity_kind;
        e.stream = std::size_t(&s - streams_.data());
        e.params = s.entity_params;
        e.transform = mul(origin_, entity_transform(std::size_t(&s - streams_.data())));
        out.push_back(e);
    }
    return out;
}

Mat4 CutscenePlayer::entity_transform(std::size_t stream, std::size_t depth) const {
    const auto& s = streams_[stream];
    const auto& q = s.equat;
    const Mat4 local = quat_trans_to_mat({q[0], q[1], q[2], q[3]}, s.epos);
    if (depth >= streams_.size() || s.obj_ref < 0 || std::size_t(s.obj_ref) >= streams_.size() ||
        !streams_[std::size_t(s.obj_ref)].has_entity) return local;
    return mul(entity_transform(std::size_t(s.obj_ref), depth + 1), local);
}

std::vector<CutscenePlayer::Sprite> CutscenePlayer::sprites() const {
    std::vector<Sprite> out;
    if (playing_) for (const auto& stream : streams_)
        if (stream.sprite.hash != 0) out.push_back(stream.sprite);
    return out;
}

std::vector<CutscenePlayer::ActorPose> CutscenePlayer::actors() const {
    std::vector<ActorPose> out;
    if (!playing_) return out;
    for (const Stream& s : streams_) {
        if (!s.actor.skin || s.done) continue;
        ActorPose pose = actor_pose(s);
        pose.frame = std::max(1.0f, pose.frame + s.time - s.entity_start);
        out.push_back(pose);
    }
    return out;
}

CutscenePlayer::ActorPose CutscenePlayer::actor_pose(const Stream& stream) const {
    ActorPose pose = stream.actor;
    if (id_ == 0x06000078 && pose.skin == 0x05000026 && streams_.size() > 2 && streams_[2].subscript) {
        // Retain the seated pose and put its root on the helicopter stream's cockpit seat.
        const Mat4 seat{0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, -.02f, -.50f, 5.52f, 1};
        pose.transform = mul(mul(origin_, entity_transform(2)), seat);
    }
    return pose;
}

std::optional<CutscenePlayer::PlayerHandoff> CutscenePlayer::take_handoff() {
    if (playing_) return std::nullopt;
    auto out = handoff_;
    handoff_.reset();
    return out;
}

std::vector<CutscenePlayer::Sound> CutscenePlayer::take_sounds() {
    std::vector<Sound> out;
    out.swap(sounds_);
    return out;
}

std::vector<CutscenePlayer::Light> CutscenePlayer::take_lights() {
    std::vector<Light> out;
    out.swap(lights_);
    return out;
}

}  // namespace nf
