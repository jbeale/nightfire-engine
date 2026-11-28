#include "game/objects.hpp"

#include <algorithm>
#include <cmath>

#include "assets/nav_data.hpp"
#include "assets/collision.hpp"  // parse_collision (script-driven mover bounds)
#include "assets/cutscene.hpp"  // spline_eval3d / slerp_acc (framelist eval)
#include "game/damage.hpp"
#include "game/player_basis.hpp"
#include "game/sp_common.hpp"  // SwitchChannels
#include "game/weapons.hpp"

namespace nf {

namespace {

// Param keys: Create functions read `level_tag+0x2c+4*key`, i.e. StaticInstance::param(key).
std::uint32_t uparam(const StaticInstance& s, std::int32_t key) { return s.param(key); }
 std::uint16_t uparam16(const StaticInstance& s, std::int32_t key) {
     return std::uint16_t(s.param(key) & 0xFFFF);
 }
 // `Mine_Create` reads damage/radius as ints: exact float conversion, except negative odds
 // round up (`(v & 1 | v >> 1) * 2`, arithmetic shift). Params are non-negative on the disc.
 float mine_param_float(std::int32_t v) {
     if (v >= 0) return float(v);
     const std::int32_t h = (v & 1) | (v >> 1);
     return float(h) + float(h);
 }

float dist2(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
}

// Volume classes for the touch tests (model collision at the static transform; exits
// (232) and movies (244) carry authored box volumes like the other triggers).
constexpr std::uint32_t kVolumeClasses[] = {32, 40, 41, 46, 48, 49, 51, 219, 220, 225,
                                            226, 228, 232, 234, 235, 240, 244, 249, 251, 254};
bool is_volume_class(std::uint32_t cls) {
    for (std::uint32_t c : kVolumeClasses)
        if (c == cls) return true;
    return false;
}

}  // namespace

SpObjects::SpObjects(Level& level, std::uint32_t level_id, sp::SwitchChannels& channels, int difficulty)
    : level_(level), level_id_(level_id), difficulty_(difficulty), channels_(channels), trigger_volumes_(make_volumes(level)) {
    // Level-load draws (e.g. searchlight sweep phases) go to the process-global stream.
}

CollisionWorld SpObjects::make_volumes(Level& level) {
    return CollisionWorld::of_objects(level, std::span<const std::uint32_t>(kVolumeClasses, std::size(kVolumeClasses)));
}

const StaticInstance* SpObjects::statics(std::size_t placement) const {
    const MapChunk* map = level_.map() ? &level_.map()->chunk : nullptr;
    if (!map) return nullptr;
    const Placement& p = level_.placements()[placement];
    return p.instance < map->statics.size() ? &map->statics[p.instance] : nullptr;
}

std::array<float, 3> SpObjects::placement_pos(std::size_t placement) const {
    const Placement& p = level_.placements()[placement];
    return {p.transform[12], p.transform[13], p.transform[14]};
}

void SpObjects::build() {
    const MapChunk* map = level_.map() ? &level_.map()->chunk : nullptr;
    if (!map) return;
    for (std::size_t i = 0; i < level_.placements().size(); ++i) {
        const Placement& p = level_.placements()[i];
        if (p.instance >= map->statics.size()) continue;
        const StaticInstance& s = map->statics[p.instance];
        if ((s.flags & 0x8000) != 0) continue;  // world cel, not an object
        const std::uint32_t cls = s.object_class();
        switch (cls) {
        case 0x3b: {  // DynamicObject_Create / Update
            const auto show = uparam16(s, 0);
            dynamic_blockers_.push_back({i, show, uparam16(s, 1), uparam(s, 3) != 0, show == 0});
            break;
        }
        case 219: {  // Door_Create: p0 flags, p1 group, p2 unlock, p3 lock, p5/6/7 sounds
            DoorObject d;
            d.placement = i;
            d.flags = uparam16(s, 0);
            d.group = uparam16(s, 1);
            d.unlock_channel = uparam16(s, 2);
            d.lock_channel = uparam16(s, 3);
            d.open_sound = uparam16(s, 5);
            d.close_sound = uparam16(s, 6);
            d.locked_sound = uparam16(s, 7);
            if (d.open_sound == 0xFFFF) d.open_sound = 0;  // -1 = silent
            if (d.close_sound == 0xFFFF) d.close_sound = 0;
            if (d.locked_sound == 0xFFFF) d.locked_sound = 0;
            d.rate = 1.0f / 60.0f;  // full swing in ~1 s at 30 Hz (see `Door_Interp` advance)
            d.spline = uparam(s, 4) != 0;  // `+112 & 1`: spline vs keyframe framelist eval
            d.auto_door = (d.flags & 2) != 0;  // `obj+244 & 2`: proximity doors (data: p0 = 2)
            const auto state_flags = uparam(s, 8) | uparam(s, 9);
            d.opening = (state_flags & 4) != 0;
            d.progress = d.opening ? 1.0f : 0.0f;
            d.hold_open = (state_flags & 0x20) != 0;
            if (d.opening && d.unlock_channel != 0) channels_.set(d.unlock_channel, true);
            doors_.push_back(d);
            break;
        }
        case 220:
        case 234:
        case 235: {  // Trigger_Create / TouchOnce(→1) / Touch(→2)
            TriggerObject t;
            t.placement = i;
            t.type = cls == 234 ? 1 : 2;
            if (cls == 220) t.type = uparam16(s, 0);
            // Base triggers read out at key1; the Touch wrappers shuffle editor key0 there.
            t.out_channel = cls == 220 ? uparam(s, 1) : uparam(s, 0);
            for (int k = 0; k < 8; ++k) t.inputs[std::size_t(k)] = uparam16(s, 2 + k);
            t.gate_channel = uparam16(s, 10);
            t.extra = uparam16(s, 11);
            if (cls != 220) {
                // The touch wrappers move the editor class mask and shutoff into Trigger_Create's fields.
                t.touch_mask = uparam16(s, 1);
                t.disable_channel = uparam16(s, cls == 234 ? 7 : 6);
                t.gate_channel = 0;
            }
            triggers_.push_back(t);
            break;
        }
        case 232: {  // Trigger_LoadLevelCreate (→ type 10)
            TriggerObject t;
            t.placement = i;
            t.type = 10;
            t.level_id = uparam(s, 0);  // destination bin hash (verified: all placed exits)
            t.out_channel = uparam(s, 1);
            for (int k = 0; k < 8; ++k) t.inputs[std::size_t(k)] = uparam16(s, 2 + k);
            t.gate_channel = uparam16(s, 10);
            triggers_.push_back(t);
            break;
        }
        case 244: {  // Trigger_MoviePlayer (→ type 13)
            TriggerObject t;
            t.placement = i;
            t.type = 13;
            t.script_hash = uparam(s, 0);  // type-7 script hash (verified: resolves on-disc)
            t.gate_channel = uparam16(s, 10);
            triggers_.push_back(t);
            break;
        }
        case 236:
        case 237:
        case 238:
        case 239: {  // Multiplex variants: out = param 0, inputs = params 1..
            TriggerObject t;
            t.placement = i;
            t.type = cls == 236 ? 3 : cls == 237 ? 4 : cls == 238 ? 5 : 12;
            t.out_channel = uparam(s, 0);
            for (int k = 0; k < 8; ++k) t.inputs[std::size_t(k)] = uparam16(s, 1 + k);
            multiplex_.push_back(t);
            break;
        }
        case 41: {  // Switch_Create: init channels[channel] = param 3
            Switch sw;
            sw.placement = i;
            sw.channel = uparam16(s, 0);
            sw.gate = uparam16(s, 5);
            if (sw.channel != 0) channels_.set(int(sw.channel), uparam16(s, 3) != 0);
            switches_.push_back(sw);
            break;
        }
        case 226: {  // SS_Create: mode = p0, touch mask = p2, value channel = p3
            if (uparam(s, 0) == 0x40 && uparam(s, 3) != 0 && (uparam(s, 2) & 0x1a) == 0) {
                DoorObject motion;
                motion.placement = i;
                motion.signal_driven = true;
                motion.open_signal = uparam16(s, 3);
                motion.open_sound = uparam16(s, 4);
                motion.spline = uparam(s, 6) != 0;
                doors_.push_back(motion);
                break;
            }
            TriggerObject t;
            t.placement = i;
            t.type = 2;  // touch-driven (SS_Operate: touch bits & mask, or value channel)
            t.out_channel = uparam(s, 0);
            t.extra = uparam16(s, 1);  // class mask [INFERENCE: players always pass]
            t.value = uparam(s, 3);
            t.rate = float(uparam(s, 4));  // sound id (reuses the spline-rate slot: SS never roves)
            triggers_.push_back(t);
            break;
        }
        case 32: {  // Break_Create: hp = param 1, sound = param 2
            Breakable b;
            b.placement = i;
            b.hp = float(std::max<std::int32_t>(1, std::int32_t(uparam(s, 1))));
            b.sound = uparam(s, 2);
            b.center = placement_pos(i);
            breakables_.push_back(b);
            break;
        }
        case 225: {  // Destroy_Create: gate = param 1, out = param 0
            Breakable b;
            b.placement = i;
            b.hp = 1;  // smashes on the gate channel, not on damage
            b.gate = uparam16(s, 1);
            b.sound = uparam(s, 4);
            b.center = placement_pos(i);
            const auto& models = level_.chunks()[p.chunk].chunk.models;
            const auto replacement = models[p.model].linked_lod;
            if (replacement >= 0 && std::size_t(replacement) < models.size() && std::size_t(replacement) != p.model) {
                b.damaged_model = std::size_t(replacement);
                b.precise_hits = !models[p.model].collision.empty();
            }
            breakables_.push_back(b);
            break;
        }
        case 40: {  // Sensor_Create: alarm = param 3, gate = param 4
            Sensor se;
            se.placement = i;
            se.alarm_channel = uparam16(s, 3);
            se.gate_channel = uparam16(s, 4);
            se.range = float(std::max<std::int32_t>(1, std::int32_t(uparam(s, 2))));
            se.half_sin = std::sin(float(uparam(s, 1)) * 0.0087266f);
            sensors_.push_back(se);
            break;
        }
        case 222: {  // Searchlight_Create: head (1), impact pool (2), and beam (3).
            if (uparam(s, 0) != 1) break;
            Searchlight light;
            light.head = i;
            light.on = uparam16(s, 1);
            light.off = uparam16(s, 2);
            light.alarm = uparam16(s, 3);
            const float lo = float(std::min(uparam(s, 4), uparam(s, 5))) * 0.017453294f;
            const float hi = float(std::max(uparam(s, 4), uparam(s, 5))) * 0.017453294f;
            light.base_yaw = std::atan2(p.transform[8], p.transform[10]);
            light.base_pitch = -(lo + hi) * 0.5f;
            light.amp = (hi - lo) * 0.5f;
            light.yaw = light.base_yaw;
            light.pitch = light.base_pitch;
            light.phase = float(game_rng().rand_int(480));
            searchlights_.push_back(light);
            break;
        }
        case 224: {  // Copter / GunImp / Shooter / Creature: scripted shooters
            Turret t;
            t.placement = i;
            t.kind = cls;
            t.range = cls == 210 ? 40.0f : 30.0f;
            t.damage = cls == 47 ? 15.0f : 10.0f;
            t.period = 60.0f;
            t.timer = t.period;
            turrets_.push_back(t);
            break;
        }
        case 228: {  // Hurt volumes: damage = p0 per tick while inside (kill-planes: p0 = 6)
            Volume v;
            v.placement = i;
            v.kind = cls;
            v.damage = float(std::int32_t(uparam(s, 0)));
            if (!(v.damage > 0)) v.damage = 6.0f;
            v.radius = 2.0f;
            volumes_.push_back(v);
            break;
        }
        case 254: {  // Mine_Create: damage = float(p0), blast radius = float(p1); p5 is the
            // explosion script (0x06000052 on the disc: the grenade blast NIS, whose
            // visual/sound the shared explosion event covers like every weapon blast)
            Volume v;
            v.placement = i;
            v.kind = cls;
            v.damage = mine_param_float(std::int32_t(uparam(s, 0)));
            v.radius = mine_param_float(std::int32_t(uparam(s, 1)));
            v.script = uparam(s, 5);
            volumes_.push_back(v);
            break;
        }
        case 46:  // Lock_Create: p0 = lock type, p1 = key, p5 = unlocked channel
            if (uparam(s, 0) == 1) {
                DoorObject bolt;
                bolt.placement = i;
                bolt.signal_driven = true;
                bolt.laser_lock = true;
                bolt.cut_signal = uparam16(s, 5);
                bolt.spline = true;
                doors_.push_back(bolt);
                break;
            }
            [[fallthrough]];
        case 48: {  // Monitor_Create: use -> channel p1 [INFERENCE]
            Switch sw;
            sw.placement = i;
            sw.gate = uparam16(s, 0);
            sw.out = uparam16(s, 1);
            switches_.push_back(sw);
            break;
        }
        case 49: {  // FuseBox_Create: spark script = p0, use powers p1 (no gate) [INFERENCE]
            Switch sw;
            sw.placement = i;
            sw.out = uparam16(s, 1);
            switches_.push_back(sw);
            break;
        }
        case 51: {  // Hint_Create: label = p0, sfx = p1, difficulty = p2, channels = p3/p4
            const auto restriction = uparam(s, 2);
            if ((restriction >= 5 && restriction <= 7 && int(restriction) - 4 != difficulty_) ||
                (restriction != 0 && restriction < 5 && int(restriction) < difficulty_)) break;
            Switch sw;
            sw.placement = i;
            sw.label = uparam(s, 0);
            sw.sound = uparam16(s, 1);
            sw.gate = uparam16(s, 3);
            sw.disable = uparam16(s, 4);
            switches_.push_back(sw);
            break;
        }
        case 249: {  // SoundTrigger_Create: chA = p0, chB = p4, sfx = p2 (one-shot)
            Switch sw;
            sw.placement = i;
            sw.channel = uparam16(s, 0);
            sw.gate = uparam16(s, 4);
            sw.sound = uparam16(s, 2);
            switches_.push_back(sw);
            break;
        }
        case 251: {  // MusicTrigger_Create: gate = p1 (alt p0), event = p2
            Switch sw;
            sw.placement = i;
            sw.channel = uparam16(s, 1);
            sw.gate = uparam16(s, 0);
            sw.event = uparam(s, 2);
            switches_.push_back(sw);
            break;
        }
        case 231: {  // ThirdCam_Create: camera anchor, id = p0
            thirdcams_.push_back({i, uparam(s, 0)});
            break;
        }
        case 253: {  // CamSubject_Create: look-at anchor (recorded for the camera)
            thirdcams_.push_back({i, 0xFFFFFFFF});
            break;
        }
        case 217: {  // SP_CreateScriptPlayer: hash = p0, auto = (p2 == 1), trigger = p6
            ScriptPlayerAnchor sp;
            sp.placement = i;
            sp.hash = uparam(s, 0);
            sp.auto_play = uparam(s, 2) == 1;
            sp.preload = uparam(s, 2) == 0x40;
            sp.trigger_channel = uparam16(s, 6);
            script_players_.push_back(sp);
            break;
        }
        case 113: {  // rotor_init: spinning rotor
            rotors_.push_back({i, 0});
            break;
        }
        case 218: {  // Car_Create: parked car, solid with zero displacement
            DoorObject d;
            d.placement = i;
            d.rate = 0;  // never moves (scripted cars are a Driving-slice concern)
            d.swing = true;
            doors_.push_back(d);
            break;
        }
        case 240: {  // Pickup_Create: kind = p0, id = p1, amount = p2, channel = p3, sound = p4
            PickupObj pk;
            pk.placement = i;
            pk.kind = uparam16(s, 0);
            pk.arg0 = uparam(s, 1);
            pk.arg1 = uparam(s, 2);
            pk.channel = uparam16(s, 3);
            pk.sound = uparam(s, 4);
            if (pk.sound == 0xFFFF) pk.sound = 245;
            pk.respawn = uparam(s, 6);
            pickups_.push_back(pk);
            break;
        }
        default:
            statics_.push_back({i, cls});
            break;
        }
    }
    // Breakable bounds for the damage tests.
    // Door spline tracks (`static_path_refs` by static index; swing otherwise).
    const std::vector<PathTrack> tracks = parse_path_data(*map);
    for (const StaticPathRef& ref : static_path_refs(*map)) {
        if (ref.count == 0 || ref.path_index >= tracks.size()) continue;
        for (DoorObject& d : doors_) {
            const Placement& dp = level_.placements()[d.placement];
            if (dp.instance == ref.static_index && tracks[ref.path_index].keys.size() >= 2) {
                d.path = tracks[ref.path_index].keys;
                d.has_path = true;
                d.swing = false;
                if (d.signal_driven) d.rate = 1.0f / (ref.aux == 0 ? 75.0f : float(ref.aux));
            }
        }
    }
    for (Searchlight& light : searchlights_) {
        float beam_distance = 9.0f, pool_distance = 9.0f;
        for (std::size_t i = 0; i < level_.placements().size(); ++i) {
            const StaticInstance* part = statics(i);
            if (!part || part->object_class() != 222) continue;
            const Vec3 delta = placement_pos(i) - placement_pos(light.head);
            const float distance = dot(delta, delta);
            if (uparam(*part, 0) == 3 && distance < beam_distance) {
                light.beam = i;
                beam_distance = distance;
            } else if (uparam(*part, 0) == 2 && distance < pool_distance) {
                light.pool = i;
                pool_distance = distance;
            }
        }
    }
    trigger_meshes_.resize(level_.placements().size());
    // Cache world-space bounds per placement with collision (touch tests + movers run per tick).
    for (std::size_t i = 0; i < trigger_volumes_.solid_count(); ++i) {
        const auto solid = trigger_volumes_.solid(i);
        if (solid.collision.tris.empty()) continue;
        trigger_meshes_[solid.placement] = true;
        const bool fresh = bounds_.count(solid.placement) == 0;
        auto& slot = bounds_[solid.placement];
        bool init = fresh;
        for (const CollisionTri& tri : solid.collision.tris) {
            for (const Vec3& v : tri.v) {
                const Vec3 w = transform_point(solid.transform, v);
                if (init) {
                    slot.first = w;
                    slot.second = w;
                    init = false;
                } else {
                    for (int k = 0; k < 3; ++k) {
                        slot.first[std::size_t(k)] = std::min(slot.first[std::size_t(k)], w[std::size_t(k)]);
                        slot.second[std::size_t(k)] = std::max(slot.second[std::size_t(k)], w[std::size_t(k)]);
                    }
                }
            }
        }
    }
    // Fallback for volume classes whose model carries no collision (`Control_BuildWorldSph`
    // builds the original's volume from the render bounds): mesh bbox to world. Only when the
    // mesh itself is missing does the touch test fall back to its 2 m box (`object_bounds`).
    for (std::size_t i = 0; i < level_.placements().size(); ++i) {
        if (bounds_.count(i) != 0) continue;
        const Placement& pl = level_.placements()[i];
        const StaticInstance* st = statics(i);
        if (!st || !is_volume_class(st->object_class())) continue;
        try {
            const GfxMesh& mesh = level_.mesh(pl.chunk, pl.model);
            const Placement& plc = level_.placements()[i];
            auto& slot = bounds_[i];
            bool init = true;
            for (int corner = 0; corner < 8; ++corner) {
                const Vec3 q = {corner & 1 ? mesh.bbox_max[0] : mesh.bbox_min[0],
                                corner & 2 ? mesh.bbox_max[1] : mesh.bbox_min[1],
                                corner & 4 ? mesh.bbox_max[2] : mesh.bbox_min[2]};
                // Placement transform is column-major model -> world (`instance_transform`).
                const std::array<float, 16>& m = plc.transform;
                const Vec3 w = {m[0] * q[0] + m[4] * q[1] + m[8] * q[2] + m[12],
                                m[1] * q[0] + m[5] * q[1] + m[9] * q[2] + m[13],
                                m[2] * q[0] + m[6] * q[1] + m[10] * q[2] + m[14]};
                if (init) {
                    slot.first = w;
                    slot.second = w;
                    init = false;
                } else {
                    for (int k = 0; k < 3; ++k) {
                        slot.first[std::size_t(k)] = std::min(slot.first[std::size_t(k)], w[std::size_t(k)]);
                        slot.second[std::size_t(k)] = std::max(slot.second[std::size_t(k)], w[std::size_t(k)]);
                    }
                }
            }
        } catch (const std::exception&) {
        }
    }
    for (Breakable& b : breakables_) {
        std::array<float, 3> mn{}, mx{};
        object_bounds(b.placement, mn, mx);
        b.center = {(mn[0] + mx[0]) * 0.5f, (mn[1] + mx[1]) * 0.5f, (mn[2] + mx[2]) * 0.5f};
        const float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
        b.radius = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    // Model-space door bounds (rotated by the live pose each tick for correct swing AABBs).
    for (std::size_t i = 0; i < trigger_volumes_.solid_count(); ++i) {
        const auto solid = trigger_volumes_.solid(i);
        if (solid.collision.tris.empty()) continue;
        for (DoorObject& d : doors_) {
            if (d.placement != solid.placement) continue;
            for (const CollisionTri& tri : solid.collision.tris) {
                for (const Vec3& v : tri.v) {
                    if (!d.has_local) {
                        d.local_mn = v;
                        d.local_mx = v;
                        d.has_local = true;
                    } else {
                        for (int k = 0; k < 3; ++k) {
                            d.local_mn[std::size_t(k)] = std::min(d.local_mn[std::size_t(k)], v[std::size_t(k)]);
                            d.local_mx[std::size_t(k)] = std::max(d.local_mx[std::size_t(k)], v[std::size_t(k)]);
                        }
                    }
                }
            }
        }
    }
}
void SpObjects::object_bounds(std::size_t placement, std::array<float, 3>& mn,
                              std::array<float, 3>& mx) const {
    const auto it = bounds_.find(placement);
    if (it != bounds_.end()) {
        mn = it->second.first;
        mx = it->second.second;
        return;
    }
    mn = placement_pos(placement);  // no model at all: a 2 m box at the marker
    mx = mn;
    for (int k = 0; k < 3; ++k) {
        mn[std::size_t(k)] -= 1.0f;
        mx[std::size_t(k)] += 1.0f;
    }
}

bool SpObjects::touch_test(std::size_t placement, const Toucher& t) const {
    std::array<float, 3> mn{}, mx{};
    object_bounds(placement, mn, mx);
    // Horizontal closeness + vertical overlap (the capsule vs the volume's world AABB).
    const float cx = std::clamp(t.pos[0], mn[0], mx[0]);
    const float cz = std::clamp(t.pos[2], mn[2], mx[2]);
    const float dx = t.pos[0] - cx, dz = t.pos[2] - cz;
    if (dx * dx + dz * dz > (t.radius + 0.3f) * (t.radius + 0.3f)) return false;
    const float top = t.pos[1] + t.height, bottom = t.pos[1] - 0.5f;
    // Volumes are often flat floor plates: step tolerance in Y.
    return top + 0.3f >= mn[1] && bottom - 0.6f <= mx[1];
}

void SpObjects::door_pose(DoorObject& d, std::array<float, 16>& out) const {
    const StaticInstance* s = statics(d.placement);
    const Placement& p = level_.placements()[d.placement];
    if (!d.has_path || d.path.size() < 2) {
        // No framelist: procedural yaw about the model's hinge edge (`Door_SetupSwing`
        // doors). The slab spans x wider than z, so the hinge is a vertical (y) edge and
        // the opening side picks it; the swing angle scale is authored per door.
        const float ang = d.progress * 1.75f * d.direction;
        const float c = std::cos(ang), sn = std::sin(ang);
        float px = 0;
        if (d.has_local) px = d.direction > 0 ? d.local_mn[0] : d.local_mx[0];
        Mat4 rot = identity();
        rot[0] = c;
        rot[2] = -sn;
        rot[8] = sn;
        rot[10] = c;
        rot[12] = px - c * px;
        rot[14] = sn * px;
        std::array<float, 16> base{};
        if (s) base = instance_transform(*s);
        else base = p.transform;
        out = mul(base, rot);
        return;
    }
    // Framelist doors (`Door_Interp`): linear (`KeyFrame_Interp`) or spline (`Spline_Interp`)
    // over the path keys by open progress; rotation slerps (`Quat_Slerp_Acc`).
    const std::size_t n = d.path.size();
    const float fi = std::clamp(d.progress, 0.0f, 1.0f) * float(n - 1);
    const std::size_t i = std::min(n - 2, std::size_t(fi));
    const float f = fi - float(i);
    const PathKey& a = d.path[i];
    const PathKey& b = d.path[i + 1];
    std::array<float, 3> pos;
    std::array<float, 4> q;
    if (d.spline && i > 0 && i + 2 < n) {
        const PathKey& p0 = d.path[i - 1];
        const PathKey& p3 = d.path[i + 2];
        pos = spline_eval3d(p0.pos, a.pos, b.pos, p3.pos, f);
        q = slerp_acc(a.quat, b.quat, f);
    } else {
        const float w0 = 1.0f - f;
        for (int k = 0; k < 3; ++k) pos[std::size_t(k)] = a.pos[std::size_t(k)] * w0 + b.pos[std::size_t(k)] * f;
        q = slerp_acc(a.quat, b.quat, f);
    }
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    Mat4 m = identity();
    m[0] = 1 - 2 * (y * y + z * z);
    m[1] = 2 * (x * y + z * w);
    m[2] = 2 * (x * z - y * w);
    m[4] = 2 * (x * y - z * w);
    m[5] = 1 - 2 * (x * x + z * z);
    m[6] = 2 * (y * z + x * w);
    m[8] = 2 * (x * z + y * w);
    m[9] = 2 * (y * z - x * w);
    m[10] = 1 - 2 * (x * x + y * y);
    m[12] = pos[0];
    m[13] = pos[1];
    m[14] = pos[2];
    out = m;
}

void SpObjects::update_script_entities(const std::vector<ScriptEntity>& ents) {
    // Script-driven solids (`SP_SetPosRot`): an entity pose whose model hash matches a
    // placement drives that placement's collision like a mover (`Door_Interp` poses do for
    // doors). Key poses are offsets composed with the placement transform.
    if (ents.empty()) return;
    const MapChunk* map = level_.map() ? &level_.map()->chunk : nullptr;
    if (!map) return;
    std::vector<std::size_t> published;  // one draw override per placement (streams may share a hash);
    // movers are per solid leaf so stacked shaft segments ride/displace independently
    for (const ScriptEntity& e : ents) {
        for (std::size_t i = 0; i < level_.placements().size(); ++i) {
            const Placement& p = level_.placements()[i];
            if (p.instance >= map->statics.size() || p.chunk >= level_.chunks().size()) continue;
            const auto& models = level_.chunks()[p.chunk].chunk.models;
            if (p.model >= models.size()) continue;
            const Model& md = models[p.model];
            if (md.hash < 0 || std::uint32_t(md.hash) != e.hash || md.collision.empty()) continue;
            bool seen = false;
            for (std::size_t q : published)
                if (q == i) {
                    seen = true;
                    break;
                }
            if (seen) continue;
            published.push_back(i);
            // Model-space solid leaves, parsed once (BVH leaves with non-pass-through
            // tris; `Collide_Filter` skips 0xC0 material the same way).
            auto sit = script_solids_.find(i);
            if (sit == script_solids_.end()) {
                Collision coll;
                try {
                    coll = parse_collision(md.collision);
                } catch (...) {
                    continue;
                }
                ScriptSolid solid;
                for (const CollisionBox& box : coll.boxes) {
                    if (!box.leaf) continue;
                    bool init = true;
                    std::array<float, 3> mn{}, mx{};
                    for (std::uint32_t ti = box.first_tri; ti < box.end_tri && ti < coll.tris.size(); ++ti) {
                        const CollisionTri& tri = coll.tris[ti];
                        if (tri.material & 0xC0) continue;
                        for (const Vec3& v : tri.v) {
                            if (init) {
                                mn = v;
                                mx = v;
                                init = false;
                            } else {
                                for (int k = 0; k < 3; ++k) {
                                    mn[std::size_t(k)] = std::min(mn[std::size_t(k)], v[std::size_t(k)]);
                                    mx[std::size_t(k)] = std::max(mx[std::size_t(k)], v[std::size_t(k)]);
                                }
                            }
                        }
                    }
                    if (init) continue;
                    solid.leaves.emplace_back(mn, mx);
                }
                if (solid.leaves.empty()) continue;
                script_solids_[i] = std::move(solid);
                sit = script_solids_.find(i);
            }
            // World matrix = placement transform composed with the key pose (same quat
            // convention as the door poses).
            const float x = e.quat[0], y = e.quat[1], z = e.quat[2], w = e.quat[3];
            Mat4 local = identity();
            local[0] = 1 - 2 * (y * y + z * z);
            local[1] = 2 * (x * y + z * w);
            local[2] = 2 * (x * z - y * w);
            local[4] = 2 * (x * y - z * w);
            local[5] = 1 - 2 * (x * x + z * z);
            local[6] = 2 * (y * z + x * w);
            local[8] = 2 * (x * z + y * w);
            local[9] = 2 * (y * z - x * w);
            local[10] = 1 - 2 * (x * x + y * y);
            local[12] = e.pos[0];
            local[13] = e.pos[1];
            local[14] = e.pos[2];
            const StaticInstance* st = statics(i);
            std::array<float, 16> base = st ? instance_transform(*st) : p.transform;
            const Mat4 world = mul(base, local);
            ScriptSolid& solid = sit->second;
            // World-space box per leaf, then back together: adjacent leaves of one deck
            // (tops within 0.25, XZ touching) merge into their union. The BVH split leaves
            // phantom internal edges that shove the capsule sideways (seen: the 0700004a
            // car deck grazed Bond off at its z = -4.69 leaf seam); the original tests
            // triangles, where coplanar neighbours form one continuous floor. Union is a
            // superset of the solids, so this only removes false edges; the top rounds up
            // by at most the bucket epsilon (inside the ride window).
            std::vector<std::pair<Vec3, Vec3>> boxes;
            boxes.reserve(solid.leaves.size());
            for (const auto& leaf : solid.leaves) {
                Vec3 mn{}, mx{};
                bool init = true;
                for (int cx = 0; cx < 8; ++cx) {
                    const Vec3 corner{cx & 1 ? leaf.second[0] : leaf.first[0],
                                      cx & 2 ? leaf.second[1] : leaf.first[1],
                                      cx & 4 ? leaf.second[2] : leaf.first[2]};
                    const Vec3 v = transform_point(world, corner);
                    if (init) {
                        mn = v;
                        mx = v;
                        init = false;
                    } else {
                        for (int k = 0; k < 3; ++k) {
                            mn[std::size_t(k)] = std::min(mn[std::size_t(k)], v[std::size_t(k)]);
                            mx[std::size_t(k)] = std::max(mx[std::size_t(k)], v[std::size_t(k)]);
                        }
                    }
                }
                // Greedy first-fit over leaf order (deterministic, rigid-invariant grouping).
                bool placed = false;
                for (auto& g : boxes) {
                    if (std::abs(g.second[1] - mx[1]) > 0.25f) continue;
                    const float xgap = std::max(g.first[0] - mx[0], mn[0] - g.second[0]);
                    const float zgap = std::max(g.first[2] - mx[2], mn[2] - g.second[2]);
                    if (xgap > 0.6f || zgap > 0.6f) continue;
                    for (int k = 0; k < 3; ++k) {
                        g.first[std::size_t(k)] = std::min(g.first[std::size_t(k)], mn[std::size_t(k)]);
                        g.second[std::size_t(k)] = std::max(g.second[std::size_t(k)], mx[std::size_t(k)]);
                    }
                    placed = true;
                    break;
                }
                if (!placed) boxes.emplace_back(mn, mx);
            }
            const bool first = solid.last_max.size() != boxes.size();
            if (first) solid.last_max.assign(boxes.size(), Vec3{});
            for (std::size_t bi = 0; bi < boxes.size(); ++bi) {
                Mover m;
                m.min = boxes[bi].first;
                m.max = boxes[bi].second;
                m.id = std::uint32_t(i);
                if (first) {
                    solid.last_max[bi] = m.max;
                } else {
                    m.displacement = {m.max[0] - solid.last_max[bi][0], m.max[1] - solid.last_max[bi][1],
                                      m.max[2] - solid.last_max[bi][2]};
                    solid.last_max[bi] = m.max;
                }
                movers_.push_back(m);
            }
            DrawOverride dr;
            dr.placement = i;
            dr.transform = world;
            draws_.push_back(dr);
            hides_.push_back(i);
        }
    }
}

bool SpObjects::use_door(DoorObject& d) {
    if (d.signal_driven) return false;
    const auto center = placement_pos(d.placement);
    if (d.lock_channel != 0 && channels_.on(int(d.lock_channel))) {
        texts_.push_back({0x02000003, 180, 1});
        if (d.locked_sound != 0) sounds_.push_back({d.locked_sound, center, true});
        d.locked_shown = true;
        return true;
    }
    d.locked_shown = false;
    d.opening = !d.opening;  // `Door_Activate` toggles
    return true;
}

bool SpObjects::use_switch(Switch& sw) {
    const StaticInstance* s = statics(sw.placement);
    const std::uint32_t cls = s ? s->object_class() : 0;
    if (cls == 41) {  // `Switch_Activate`: use toggles the channel (gated)
        if (sw.gate != 0 && channels_.on(int(sw.gate))) return false;
        channels_.set(int(sw.channel), !channels_.on(int(sw.channel)));
        return true;
    }
    if (cls == 46 || cls == 48 || cls == 49) {  // Lock/Monitor/FuseBox: use sets out
        if (sw.gate != 0 && !channels_.on(int(sw.gate))) return false;
        if (sw.out != 0) channels_.set(int(sw.out), true);
        return true;
    }
    return false;
}

bool SpObjects::activate_at(const Vec3& center, float radius) {
    // The probe tests against object volumes (`Collide_SphereIntersect` radius 1.0), not centers.
    const std::array<float, 3> c{center[0], center[1], center[2]};
    auto dist_to = [&](std::size_t placement) {
        std::array<float, 3> mn{}, mx{};
        object_bounds(placement, mn, mx);
        float d2 = 0;
        for (int k = 0; k < 3; ++k) {
            const float v = c[std::size_t(k)];
            const float lo = mn[std::size_t(k)] - radius, hi = mx[std::size_t(k)] + radius;
            if (v < lo) d2 += (lo - v) * (lo - v);
            else if (v > hi) d2 += (v - hi) * (v - hi);
        }
        return d2;
    };
    DoorObject* best_door = nullptr;
    float best_d = radius * radius;
    for (DoorObject& d : doors_) {
        if (d.signal_driven) continue;
        const float dd = dist_to(d.placement);
        if (dd < best_d) {
            best_d = dd;
            best_door = &d;
        }
    }
    if (best_door) return use_door(*best_door);
    Switch* best_sw = nullptr;
    best_d = radius * radius;
    for (Switch& sw : switches_) {
        const StaticInstance* s = statics(sw.placement);
        const std::uint32_t cls = s ? s->object_class() : 0;
        if (cls != 41 && cls != 46 && cls != 48 && cls != 49) continue;
        const float dd = dist_to(sw.placement);
        if (dd < best_d) {
            best_d = dd;
            best_sw = &sw;
        }
    }
    if (best_sw) return use_switch(*best_sw);
    return false;
}
std::size_t SpObjects::door_count() const { return doors_.size(); }

bool SpObjects::any_door_open() const {
    for (const DoorObject& d : doors_)
        if (d.progress > 0.05f) return true;
    return false;
}

void SpObjects::break_near(const std::array<float, 3>& pos) {
    for (Breakable& b : breakables_) {
        if (b.broken) continue;
        if (dist2(b.center, pos) < (b.radius + 1.0f) * (b.radius + 1.0f)) {
            b.broken = true;
            if (b.sound != 0) sounds_.push_back({b.sound, b.center, true});
        }
    }
}

void SpObjects::set_link_byte(const std::array<float, 3>& pos, std::uint8_t value) {
    // Event 11 writes the linked door's lock byte: lock/unlock the nearest door [INFERENCE].
    DoorObject* best = nullptr;
    float best_d = 0;
    for (DoorObject& d : doors_) {
        const auto pp = placement_pos(d.placement);
        const float dd = dist2(pp, pos);
        if (!best || dd < best_d) {
            best = &d;
            best_d = dd;
        }
    }
    if (!best) return;
    if (value != 0) {
        best->locked_shown = true;  // freeze shut like `Door_Activate`'s denied path
        best->opening = false;
    } else {
        best->locked_shown = false;
    }
}

std::uint32_t SpObjects::take_load_level() {
    const std::uint32_t out = load_level_;
    load_level_ = 0;
    return out;
}

std::uint32_t SpObjects::take_movie() {
    const std::uint32_t out = movie_;
    movie_ = 0;
    return out;
}

std::vector<SoundRequest> SpObjects::take_sounds() {
    std::vector<SoundRequest> out;
    out.swap(sounds_);
    return out;
}

std::vector<std::uint32_t> SpObjects::take_music() {
    std::vector<std::uint32_t> out;
    out.swap(music_);
    return out;
}

std::vector<SpObjects::Text> SpObjects::take_texts() {
    std::vector<Text> out;
    out.swap(texts_);
    return out;
}

SpObjects::Census SpObjects::census() const {
    Census c;
    const MapChunk* map = level_.map() ? &level_.map()->chunk : nullptr;
    if (!map) {
        c.issues.push_back("no Map entry");
        return c;
    }
    for (std::size_t i = 0; i < level_.placements().size(); ++i) {
        const Placement& p = level_.placements()[i];
        if (p.instance >= map->statics.size()) continue;
        const StaticInstance& s = map->statics[p.instance];
        if ((s.flags & 0x8000) != 0) continue;
        const std::uint32_t cls = s.object_class();
        if (cls < 256) c.per_class[cls]++;
    }
    return c;
}


void SpObjects::tick(const std::vector<Toucher>& touchers, const std::vector<bool>& use_pressed,
                     const WeaponEvents& weapon_events, float dt_frames, WeaponSystem* weapons,
                     const HurtSink& hurt) {
    movers_.clear();
    draws_.clear();
    hides_.clear();
    ++tick_;
    auto rising = [&](int ch) { return ch > 0 && ch < 256 && channels_.on(ch) && !prev_[std::size_t(ch)]; };
    auto touching = [&](std::size_t placement, bool players_only) {
        for (const Toucher& t : touchers) {
            if (players_only && !t.is_player) continue;
            if (touch_test(placement, t)) return true;
        }
        return false;
    };
    std::vector<bool> touched_meshes(trigger_meshes_.size());
    bool has_capsule = false;
    for (const Toucher& t : touchers) {
        if (!t.is_player || !t.capsule) continue;
        has_capsule = true;
        CylinderQuery capsule = *t.capsule;
        capsule.pick = 0;
        capsule.hit = hitflag::kNoPushOut;
        for (const auto& hit : trigger_volumes_.cylinder(capsule).hits)
            touched_meshes[hit.placement] = true;
    }
    auto use_near = [&](std::size_t placement, float radius) {
        const auto pp = placement_pos(placement);
        for (const Toucher& t : touchers) {
            if (!t.is_player || t.slot < 0 || std::size_t(t.slot) >= use_pressed.size()) continue;
            if (!use_pressed[std::size_t(t.slot)]) continue;
            if (dist2(pp, t.pos) < radius * radius) return true;
        }
        return false;
    };
    // ---- doors (`Door_Update`) ----
    for (DoorObject& d : doors_) {
        const auto center = placement_pos(d.placement);
        // Proximity scan (`Door_Update` sphere test): anyone within ~3 m of the panel.
        bool near = false;
        for (const Toucher& t : touchers) {
            if (dist2(center, t.pos) < (3.0f + t.radius) * (3.0f + t.radius)) {
                near = true;
                break;
            }
        }
        if (d.signal_driven) {
            if (!d.opening && d.laser_lock) {
                bool heated = false;
                for (const ImpactEvent& impact : weapon_events.impacts) {
                    if (impact.on_body || (impact.weapon != 0x4e && impact.weapon != 0x4f)) continue;
                    if (impact.placement != d.placement) {
                        // At the seam the frame or door can win the ray by a few centimeters.
                        // Only accept a contact immediately beside this hinge's authored bounds.
                        const auto bounds = bounds_.find(d.placement);
                        if (bounds == bounds_.end()) continue;
                        const auto& [lo, hi] = bounds->second;
                        Vec3 nearest;
                        for (std::size_t axis = 0; axis < 3; ++axis)
                            nearest[axis] = std::clamp(impact.point[axis], lo[axis], hi[axis]);
                        if (dist2(impact.point, nearest) > .07f * .07f) continue;
                    }
                    d.lock_heat += (impact.weapon == 0x4e ? 3.0f : 10.0f) * dt_frames;
                    d.cooling_delay = 2;
                    heated = true;
                }
                if (!heated && d.cooling_delay <= 0) d.lock_heat = std::max(0.0f, d.lock_heat - dt_frames);
                if (!heated) d.cooling_delay -= dt_frames;
                if (d.lock_heat >= 252) {
                    d.opening = true;
                    if (d.cut_signal != 0) channels_.set(d.cut_signal, true);
                    sounds_.push_back({0x249, center, true});
                }
            }
            if (!d.opening && channels_.on(d.open_signal)) {
                d.opening = true;
                if (d.open_sound != 0 && d.open_sound != 0xffff) sounds_.push_back({d.open_sound, center, true});
            }
            if (d.opening) {
                d.progress = std::min(1.0f, d.progress + d.rate * dt_frames);
                d.lock_heat = std::max(0.0f, d.lock_heat - 2.0f * dt_frames);
            }
        } else if (d.rate > 0) {
            if (d.locked_shown) {  // frozen by the denied path; the unlock channel releases it
                if (d.unlock_channel != 0 && channels_.on(int(d.unlock_channel))) {
                    d.locked_shown = false;
                    d.opening = true;
                }
            } else if (d.lock_channel != 0 && rising(int(d.lock_channel))) {
                // Lock edge: "locked" message + denied freeze (`Door_Update` LABEL_34).
                texts_.push_back({0x02000003, 180, 1});
                if (d.locked_sound != 0) sounds_.push_back({d.locked_sound, center, true});
                d.locked_shown = true;
                d.opening = false;
            } else {
                if (d.unlock_channel != 0 && channels_.on(int(d.unlock_channel))) {
                    d.locked_shown = false;
                    d.opening = true;
                } else if (d.unlock_channel != 0 && d.progress >= 1.0f && !near) {
                    // A script may clear an already-open door's channel to close it once clear.
                    d.opening = false;
                } else if (d.auto_door && near) {
                    d.locked_shown = false;
                    d.opening = true;
                }
                if (use_near(d.placement, 2.5f)) use_door(d);
            }
            const bool was_open = d.progress > 0.02f;
            d.progress = std::clamp(d.progress + (d.opening ? d.rate * dt_frames : -d.rate * dt_frames), 0.0f, 1.0f);
            const bool is_open = d.progress > 0.02f;
            if (is_open && !was_open && d.open_sound != 0) sounds_.push_back({d.open_sound, center, true});
            if (!is_open && was_open && d.close_sound != 0) sounds_.push_back({d.close_sound, center, true});
            if (is_open && !near) {
                d.idle_frames += 1.0f;  // auto-close after ~6 s clear (`Door_Update` 6*FRAME_RATE_INT)
                if (d.auto_door && !d.hold_open && d.idle_frames > 180.0f) d.opening = false;
            } else {
                d.idle_frames = 0;
            }
            // `Door_SetState` reports a change. Reasserting on every tick masks scripted clears.
            if (d.unlock_channel != 0 && was_open != is_open)
                channels_.set(d.unlock_channel, is_open);
        }
        // Publish the solid: the model-space box posed by the live matrix (correct for both
        // sliders and swung panels); falls back to the shifted closed box without local bounds.
        std::array<float, 16> pose{};
        door_pose(d, pose);
        Mat4 pm;
        for (int k = 0; k < 16; ++k) pm[std::size_t(k)] = pose[std::size_t(k)];
        Mover m;
        if (d.has_local) {
            bool init = false;
            for (int cx = 0; cx < 8; ++cx) {
                const Vec3 corner{(cx & 1) != 0 ? d.local_mx[0] : d.local_mn[0],
                                  (cx & 2) != 0 ? d.local_mx[1] : d.local_mn[1],
                                  (cx & 4) != 0 ? d.local_mx[2] : d.local_mn[2]};
                const Vec3 w = transform_point(pm, corner);
                if (!init) {
                    m.min = w;
                    m.max = w;
                    init = true;
                } else {
                    for (int k = 0; k < 3; ++k) {
                        m.min[std::size_t(k)] = std::min(m.min[std::size_t(k)], w[std::size_t(k)]);
                        m.max[std::size_t(k)] = std::max(m.max[std::size_t(k)], w[std::size_t(k)]);
                    }
                }
            }
        } else {
            std::array<float, 3> mn{}, mx{};
            object_bounds(d.placement, mn, mx);
            m.min = mn;
            m.max = mx;
        }
        m.id = std::uint32_t(d.placement);
        if (d.has_last) m.displacement = {m.max[0] - d.last_max[0], m.max[1] - d.last_max[1], m.max[2] - d.last_max[2]};
        d.last_max = m.max;
        d.has_last = true;
        movers_.push_back(m);
        DrawOverride dr;
        dr.placement = d.placement;
        dr.transform = pose;
        const float heat = d.lock_heat / 255.0f;
        dr.tint = {1.0f + heat, 1.0f - heat, 1.0f - heat};
        draws_.push_back(dr);
        hides_.push_back(d.placement);
    }
    // ---- touch triggers ----
    for (TriggerObject& t : triggers_) {
        if (t.gate_channel != 0 && !channels_.on(int(t.gate_channel))) continue;
        if (channels_.on(t.disable_channel)) continue;
        const bool mesh_trigger = (t.type == 1 || t.type == 2) && trigger_meshes_[t.placement];
        bool player_touch = (t.touch_mask & 8) != 0 &&
            (mesh_trigger && has_capsule ? touched_meshes[t.placement] : touching(t.placement, true));
        if (!player_touch && (t.touch_mask & 8) != 0 && t.type == 2 && mesh_trigger) {
            // A capsule wholly inside a touch volume may miss every boundary triangle.
            const auto& [lo, hi] = bounds_.at(t.placement);
            player_touch = std::any_of(touchers.begin(), touchers.end(), [&](const Toucher& actor) {
                const auto& p = actor.pos;
                return actor.is_player && p[0] >= lo[0] && p[0] <= hi[0] &&
                       p[1] >= lo[1] && p[1] <= hi[1] && p[2] >= lo[2] && p[2] <= hi[2];
            });
        }
        const bool drone_touch = (t.touch_mask & 0x10) != 0 &&
            std::any_of(touchers.begin(), touchers.end(), [&](const Toucher& actor) {
                if (actor.is_player || !touch_test(t.placement, actor)) return false;
                if (!mesh_trigger || !actor.capsule) return true;
                auto capsule = *actor.capsule;
                capsule.hit = hitflag::kNoPushOut;
                const auto hits = trigger_volumes_.cylinder(capsule).hits;
                return std::any_of(hits.begin(), hits.end(),
                    [&](const CollisionHit& hit) { return hit.placement == t.placement; });
            });
        const bool touch = player_touch || drone_touch;
        const bool was = t.touched;
        t.touched = touch;
        switch (t.type) {
        case 1:  // TouchOnce: latch once on the rising edge
            if (touch && !t.once_fired) {
                t.once_fired = true;
                if (t.out_channel != 0) channels_.set(int(t.out_channel), true);
            }
            break;
        case 2: {  // Touch / SS: latch while touched (SS also ORs its value channel)
            const bool ss = t.value != 0 || t.rate != 0;
            bool out = touch;
            if (ss && t.value != 0 && channels_.on(int(t.value))) out = true;
            if (out && t.out_channel != 0) channels_.set(int(t.out_channel), true);
            if (ss && out && !was && t.rate != 0) {
                const auto pp = placement_pos(t.placement);
                sounds_.push_back({std::uint32_t(t.rate), pp, true});
            }
            break;
        }
        case 10: {  // LoadLevel (`Trigger_Activate`): A = p1 required, B = p2 picks D = p4 else C = p3
            if (t.cooldown > 0) {
                t.cooldown--;
                break;
            }
            if (!touch) break;
            const std::uint16_t need = t.out_channel;  // editor param 1 (blocker A)
            if (need != 0 && !channels_.on(int(need))) {
                texts_.push_back({0x0200021, 180, 1});  // "you can't leave yet"
                t.cooldown = 90;
                break;
            }
            const std::uint16_t cond = t.inputs[0];  // editor param 2 (branch B)
            if (cond != 0 && channels_.on(int(cond))) {
                const std::uint16_t done = t.inputs[2];  // editor param 4 (D)
                exit_channel_ = done;
                if (done != 0) channels_.set(int(done), true);
            } else {
                const std::uint16_t out = t.inputs[1];  // editor param 3 (C)
                exit_channel_ = out;
                if (out != 0) channels_.set(int(out), true);
            }
            load_level_ = t.level_id;
            break;
        }
        case 13:  // MoviePlayer: touch or gate edge plays the script
            if ((touch && !was) || rising(int(t.gate_channel))) {
                if (t.script_hash != 0) movie_ = t.script_hash;
            }
            break;
        default:
            break;
        }
    }
    // ---- multiplex logic (no volumes) ----
    for (TriggerObject& m : multiplex_) {
        switch (m.type) {
        case 3: {  // AND
            bool any = false, all = true;
            for (std::uint16_t ch : m.inputs) {
                if (ch == 0) continue;
                any = true;
                if (!channels_.on(int(ch))) all = false;
            }
            if (any && m.out_channel != 0) channels_.set(int(m.out_channel), all);
            break;
        }
        case 12: {  // OR
            bool any = false;
            for (std::uint16_t ch : m.inputs) {
                if (ch != 0 && channels_.on(int(ch))) any = true;
            }
            if (m.out_channel != 0) channels_.set(int(m.out_channel), any);
            break;
        }
        case 5:  // fan-out: inputs follow the output
            for (std::uint16_t ch : m.inputs) {
                if (ch == 0) continue;
                channels_.set(int(ch), m.out_channel != 0 && channels_.on(int(m.out_channel)));
            }
            break;
        case 4: {  // sequence: inputs in order inside a 4-tick window
            for (int k = 0; k < 8; ++k) {
                const std::uint16_t ch = m.inputs[std::size_t(k)];
                if (ch == 0) continue;
                if (rising(int(ch))) {
                    if (k == m.seq_pos && float(tick_) - m.seq_tick <= 4.0f) {
                        m.seq_pos++;
                        m.seq_tick = float(tick_);
                    } else if (k == 0) {
                        m.seq_pos = 1;
                        m.seq_tick = float(tick_);
                    }
                }
            }
            int need = 0;
            for (std::uint16_t ch : m.inputs)
                if (ch != 0) need++;
            if (need > 0 && m.seq_pos >= need && m.out_channel != 0) channels_.set(int(m.out_channel), true);
            break;
        }
        default:
            break;
        }
    }
    tick_switches(touchers, use_pressed);
    for (auto& blocker : dynamic_blockers_) {
        if (!blocker.removed && (blocker.show_channel == 0 || channels_.on(blocker.show_channel))) {
            const bool hide = channels_.on(blocker.hide_channel);
            blocker.removed = hide && !blocker.reversible;
            blocker.visible = !hide;
        }
        if (!blocker.visible) hides_.push_back(blocker.placement);
    }
    tick_damage(touchers, weapon_events, dt_frames, weapons, hurt);
}
void SpObjects::tick_switches(const std::vector<Toucher>& touchers, const std::vector<bool>& use_pressed) {
    auto use_near = [&](std::size_t placement, float radius) {
        const auto pp = placement_pos(placement);
        for (const Toucher& t : touchers) {
            if (!t.is_player || t.slot < 0 || std::size_t(t.slot) >= use_pressed.size()) continue;
            if (!use_pressed[std::size_t(t.slot)]) continue;
            if (dist2(pp, t.pos) < radius * radius) return true;
        }
        return false;
    };
    for (Switch& sw : switches_) {
        const StaticInstance* s = statics(sw.placement);
        const std::uint32_t cls = s ? s->object_class() : 0;
        if (cls == 41) {  // Switch_Activate: use toggles the channel (gated)
            if (!use_near(sw.placement, 2.0f)) continue;
            use_switch(sw);
            continue;
        }
        if (cls == 46 || cls == 48 || cls == 49) {  // Lock/Monitor/FuseBox: use sets out
            if (!use_near(sw.placement, 2.0f)) continue;
            use_switch(sw);
            continue;
        }
        if (cls == 51) {  // Hint: a player touching the volume sees the text once
            if (channels_.on(sw.disable)) sw.fired = true;
            if (!sw.fired && (sw.gate == 0 || channels_.on(sw.gate)) &&
                std::any_of(touchers.begin(), touchers.end(), [&](const Toucher& t) {
                    return t.is_player && touch_test(sw.placement, t);
                })) {
                sw.fired = true;
                texts_.push_back({sw.label, 180, 1});
                if (sw.sound != 0 && sw.sound != 0xFFFF) {
                    const auto pp = placement_pos(sw.placement);
                    sounds_.push_back({sw.sound, pp, false});
                }
            }
            continue;
        }
        if (cls == 249) {  // SoundTrigger: rising edge on either channel plays once
            const bool a = sw.channel != 0 && channels_.on(int(sw.channel)) && !prev_[sw.channel];
            const bool b = sw.gate != 0 && channels_.on(int(sw.gate)) && !prev_[sw.gate];
            if ((a || b) && !sw.fired && sw.sound != 0 && sw.sound != 0xFFFF) {
                sw.fired = true;
                const auto pp = placement_pos(sw.placement);
                sounds_.push_back({sw.sound, pp, true});
            }
            if (!channels_.on(int(sw.channel)) && !channels_.on(int(sw.gate))) sw.fired = false;
            continue;
        }
        if (cls == 251) {  // MusicTrigger: held gate holds Music_Event(event, 2)
            const bool active = (sw.channel != 0 && channels_.on(int(sw.channel))) ||
                                (sw.gate != 0 && channels_.on(int(sw.gate)));
            if (active && sw.event != 0 && sw.event != 0xFFFF) music_.push_back(sw.event);
            continue;
        }
    }
}

std::optional<Vec3> SpObjects::update_searchlights(const std::vector<Toucher>& touchers,
                                                  const CollisionWorld& world, float frames,
                                                  const CollisionWorld* moving) {
    std::optional<Vec3> detected;
    const auto orient = [](const Vec3& forward, const Vec3& up, const Vec3& pos, float reach = 1.0f) {
        Vec3 lateral = cross(up, forward);
        if (length(lateral) < 0.001f) lateral = cross(Vec3{0, 0, 1}, forward);
        const Vec3 right = normalised(lateral);
        const Vec3 vertical = cross(forward, right);
        const Vec3 span = forward * reach;
        return Mat4{right[0], right[1], right[2], 0, vertical[0], vertical[1], vertical[2], 0,
                    span[0], span[1], span[2], 0, pos[0], pos[1], pos[2], 1};
    };
    const auto viewer = std::find_if(touchers.begin(), touchers.end(), [](const Toucher& p) { return p.is_player; });
    for (Searchlight& light : searchlights_) {
        if (light.beam != SIZE_MAX) hides_.push_back(light.beam);
        if (light.pool != SIZE_MAX) hides_.push_back(light.pool);
        if ((light.on != 0 && !channels_.on(light.on)) || (light.off != 0 && channels_.on(light.off))) continue;
        const Vec3 origin = placement_pos(light.head);
        if (light.hold <= 0) {
            const float phase = light.phase * 0.013089971f;
            light.yaw = light.base_yaw + std::cos(phase) * light.amp;
            light.pitch = light.base_pitch - std::sin(phase) * light.amp;
            light.phase = std::fmod(light.phase + frames, 480.0f);
        }
        const Vec3 dir{std::sin(light.yaw) * std::cos(light.pitch), std::sin(light.pitch),
                       std::cos(light.yaw) * std::cos(light.pitch)};
        const Vec3 beam_start = origin + dir * 0.15f, beam_end = origin + dir * 100.0f;
        auto impact = world.ray(beam_start, beam_end, pick::kIgnoreGhost);
        if (moving) {
            const auto hit = moving->ray(beam_start, beam_end, pick::kIgnoreGhost);
            if (hit && (!impact || hit->dist < impact->dist)) impact = hit;
        }
        bool was_lit = false;
        if (impact) {
            for (const Toucher& p : touchers) {
                if (!p.is_player || length(p.pos - impact->point) > 2.0f) continue;
                if (!world.line_of_sight(beam_start, p.pos, pick::kIgnoreGhost) ||
                    (moving && !moving->line_of_sight(beam_start, p.pos, pick::kIgnoreGhost))) continue;
                detected = p.pos;
                was_lit = true;
                const Vec3 to = p.pos - Vec3{0, 1, 0} - origin;
                const float target_yaw = light.base_yaw + std::clamp(std::remainder(
                    std::atan2(to[0], to[2]) - light.base_yaw, 6.2831853f), -light.amp, light.amp);
                const float target_pitch = std::clamp(std::atan2(to[1], std::hypot(to[0], to[2])),
                                                      light.base_pitch - light.amp, light.base_pitch + light.amp);
                light.yaw += (target_yaw - light.yaw) * 0.15f * frames;
                light.pitch += (target_pitch - light.pitch) * 0.15f * frames;
                break;
            }
        }
        light.hold = was_lit ? 900.0f : std::max(0.0f, light.hold - frames);
        if (light.hold > 0 && light.alarm != 0) channels_.set(light.alarm, true);
        if (light.beam != SIZE_MAX) {
            const auto& p = level_.placements()[light.beam];
            const auto& mesh = level_.mesh(p.chunk, p.model);
            const Vec3 end = impact ? impact->point : origin + dir * 100.0f;
            const float beam_length = std::max(1.0f, mesh.bbox_max[2] - mesh.bbox_min[2]);
            const Vec3 up = viewer == touchers.end() ? Vec3{0, 1, 0} : viewer->pos - origin;
            draws_.push_back({light.beam, orient(dir, up, origin, length(end - origin) / beam_length)});
        }
        if (light.pool != SIZE_MAX && impact) {
            const Vec3 normal = impact->normal;
            const Vec3 axis = std::fabs(normal[2]) > 0.9f ? Vec3{1, 0, 0} : Vec3{0, 0, 1};
            const Vec3 tangent = normalised(axis - normal * dot(axis, normal));
            draws_.push_back({light.pool, orient(tangent, normal, impact->point + normal * 0.015f)});
        }
    }
    return detected;
}

void SpObjects::tick_damage(const std::vector<Toucher>& touchers, const WeaponEvents& weapon_events,
                            float dt_frames, WeaponSystem* weapons, const HurtSink& hurt) {
    // Breakables from bullets and blasts (`Break_Kill` / `Destroy_Smash` essence).
    for (Breakable& b : breakables_) {
        if (b.broken) continue;
        if (b.gate != 0) {  // Destroy: smash when the gate channel is set
            if (channels_.on(int(b.gate))) {
                b.broken = true;
                if (b.sound != 0 && b.sound != 0xFFFF) sounds_.push_back({b.sound, b.center, true});
            }
            continue;
        }
        for (const ImpactEvent& im : weapon_events.impacts) {
            if (im.on_body || im.weapon == 0x4e || im.weapon == 0x4f) continue;
            const bool hit = b.precise_hits
                ? im.placement == b.placement
                : dist2(im.point, b.center) < (b.radius + .5f) * (b.radius + .5f);
            if (hit) b.hp -= 25.0f;
        }
        for (const ExplosionEvent& ex : weapon_events.explosions) {
            if (dist2(ex.position, b.center) < (b.radius + ex.radius) * (b.radius + ex.radius)) b.hp -= 100.0f;
        }
        if (b.hp <= 0) {
            b.broken = true;
            if (b.sound != 0 && b.sound != 0xFFFF) sounds_.push_back({b.sound, b.center, true});
        }
    }
    for (const Breakable& b : breakables_) {
        if (!b.broken) continue;
        hides_.push_back(b.placement);
        if (b.damaged_model != SIZE_MAX)
            draws_.push_back({b.placement, level_.placements()[b.placement].transform, false, b.damaged_model});
    }
    // Fixed sensors: cone trip raises the alarm channel.
    for (Sensor& se : sensors_) {
        if (se.gate_channel != 0 && !channels_.on(int(se.gate_channel))) {
            if (se.tripped && se.alarm_channel != 0) channels_.set(int(se.alarm_channel), false);
            se.tripped = false;
            continue;
        }
        const Placement& p = level_.placements()[se.placement];
        std::array<float, 3> dir{p.transform[8], p.transform[9], p.transform[10]};
        const float dl = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
        if (dl > 1e-6f) {
            dir[0] /= dl;
            dir[1] /= dl;
            dir[2] /= dl;
        }
        const auto origin = placement_pos(se.placement);
        bool trip = false;
        for (const Toucher& t : touchers) {
            if (!t.is_player) continue;
            std::array<float, 3> to{t.pos[0] - origin[0], t.pos[1] + 0.5f - origin[1], t.pos[2] - origin[2]};
            const float dist = std::sqrt(to[0] * to[0] + to[1] * to[1] + to[2] * to[2]);
            if (dist > se.range || dist < 1e-6f) continue;
            const float dot = (to[0] * dir[0] + to[1] * dir[1] + to[2] * dir[2]) / dist;
            if (dot >= std::sqrt(std::max(0.0f, 1.0f - se.half_sin * se.half_sin))) {
                trip = true;
                break;
            }
        }
        if (trip && !se.tripped) sounds_.push_back({1194, origin, true});  // trip beep
        if (!trip && se.tripped) sounds_.push_back({1245, origin, true});  // clear
        se.tripped = trip;
        if (se.alarm_channel != 0) channels_.set(int(se.alarm_channel), trip);
    }
    // Scripted shooters: range check, then hurt the player on a timer (no LOS yet [INFERENCE]).
    for (Turret& t : turrets_) {
        t.timer -= 1.0f;
        if (t.timer > 0) continue;
        const auto pp = placement_pos(t.placement);
        const Toucher* best = nullptr;
        float best_d = t.range * t.range;
        for (const Toucher& c : touchers) {
            if (!c.is_player || c.slot < 0) continue;
            const float dd = dist2(pp, c.pos);
            if (dd < best_d) {
                best_d = dd;
                best = &c;
            }
        }
        if (best && hurt) {
            hurt(best->slot, t.damage, DamageType::Bullet, pp);
            t.timer = t.period;
        } else {
            t.timer = 15.0f;  // re-scan soon when nobody is near
        }
    }
    // Hurt volumes and mines.
    for (Volume& v : volumes_) {
        if (v.kind == 254 && v.spent) continue;
        if (v.gate_channel != 0 && !channels_.on(int(v.gate_channel))) continue;
        const auto pp = placement_pos(v.placement);
        for (const Toucher& t : touchers) {
            if (!t.is_player || t.slot < 0) continue;
            if (dist2(pp, t.pos) > v.radius * v.radius) continue;
            if (v.kind == 254) {  // mine: one blast through the shared path, then gone
                // `Mine_Update` ~ `Explode_Create`: authored radius/damage with falloff,
                // shake, boom and chain-detonation; Remote Mine row for the sound flag and
                // kill credit, world attacker (an environment kill in MP).
                v.spent = true;
                hides_.push_back(v.placement);
                if (weapons) {
                    WeaponDef def = weapons->table().weapon(55);
                    def.blast_radius = v.radius;
                    def.damage = v.damage;
                    weapons->explode_at(pp, def, -1, 1.0f, v.script);
                } else if (hurt) {
                    hurt(t.slot, v.damage, DamageType::Bullet, pp);
                }
                break;  // one blast per mine (`Mine_Update` dies with obj+254 |= 1)
            } else {  // hurt volume: damage per tick while inside
                if (!hurt) continue;
                hurt(t.slot, v.damage * 0.5f * dt_frames, DamageType::HurtVolume, pp);
            }
        }
    }
    // Pickups: walk-over grants with respawn.
    for (PickupObj& pk : pickups_) {
        if (pk.kind == 4 && pk.channel != 0 && channels_.on(pk.channel)) pk.taken = true;
        if (pk.taken) {
            if (pk.respawn != 0) {
                pk.respawn_in -= 1.0f;
                if (pk.respawn_in <= 0) pk.taken = false;
                else hides_.push_back(pk.placement);
            } else {
                hides_.push_back(pk.placement);
            }
            continue;
        }
        if (!weapons) continue;
        const auto pp = placement_pos(pk.placement);
        bool granted = false;
        for (const Toucher& t : touchers) {
            if (!t.is_player || t.slot < 0) continue;
            if (dist2(pp, t.pos) > 1.5f * 1.5f) continue;
            const int id = int(pk.arg0), rounds = int(pk.arg1);
            if (pk.kind == 0) granted = weapons->give_weapon(t.slot, weapons->table().upgrade(id), rounds);
            else if (pk.kind == 1) granted = weapons->give_ammo(t.slot, id, rounds);
            else if (pk.kind == 3) granted = weapons->give_armour(t.slot, 50.0f);
            else if (pk.kind == 4) granted = true;
            else if (pk.kind == 6) {
                granted = true;
                ++bond_pickups_;
                texts_.push_back({0x0200004a, 180, 1});
            }
            else granted = weapons->give_weapon(t.slot, id, rounds) || weapons->give_ammo(t.slot, id, rounds);
            if (granted) {
                if (pk.channel != 0) channels_.set(int(pk.channel), true);
                const auto sound = pk.kind == 6 ? 0x14cu : pk.sound;
                if (sound != 0 && sound != 0xFFFF) sounds_.push_back({sound, pp, false});
                pk.taken = true;
                if (pk.kind == 6) pk.respawn = 0;
                pk.respawn_in = float(pk.respawn);
            }
            break;
        }
        if (pk.kind == 6) {
            hides_.push_back(pk.placement);
            if (!pk.taken) {
                const float angle = float(tick_ % 180) * 0.03490659f;
                const float c = std::cos(angle), s = std::sin(angle);
                const Mat4 spin{c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1};
                draws_.push_back({pk.placement, mul(level_.placements()[pk.placement].transform, spin)});
            }
        }
    }
    // Rotors spin for the renderer.
    for (Rotor& r : rotors_) {
        r.angle += 0.1f * dt_frames;
        const Placement& p = level_.placements()[r.placement];
        const float c = std::cos(r.angle), s = std::sin(r.angle);
        Mat4 rot = identity();
        rot[0] = c;
        rot[2] = -s;
        rot[8] = s;
        rot[10] = c;
        DrawOverride dr;
        dr.placement = r.placement;
        dr.transform = mul(p.transform, rot);
        draws_.push_back(dr);
        hides_.push_back(r.placement);
    }
    // Snapshot channels for next tick's rising edges.
    for (int i = 0; i < 256; ++i) prev_[std::size_t(i)] = channels_.on(i) ? 1 : 0;
}

}  // namespace nf
