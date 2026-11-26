#include "game/mission.hpp"

#include <algorithm>
#include <cmath>

#include "assets/game_files.hpp"
#include "assets/character.hpp"
#include "game/drone.hpp"
#include "game/drone_system.hpp"
#include "game/player.hpp"
#include "game/sp_common.hpp"
#include "game/weapons.hpp"

namespace nf {

MissionSystem::MissionSystem(Level& level, std::uint32_t level_id, const MissionEntry& entry, WeaponSystem& weapons,
                             sp::SpSystem* sp)
    : level_(level), level_id_(level_id), entry_(entry), weapons_(weapons), sp_(sp), objects_(std::make_unique<SpObjects>(level, level_id, channels_)) {
    // `Mission_Init`: objectives start announced or waiting; channels preset from +13.
    for (const MissionObjective& o : entry_.objectives) {
        Objective obj;
        obj.def = o;
        // `Mission_MonitorObjectives` treats both 0 and 255 as "no second channel".
        obj.state = (o.channel2 != 0 && o.channel2 != 255) ? ObjectiveState::WaitSecond : ObjectiveState::Announce;
        objectives_.push_back(obj);
        if (o.channel != 0) channels_.set(int(o.channel), o.init != 0);
    }
    objects_->build();
}

MissionSystem::~MissionSystem() = default;

void MissionSystem::add_scripts(std::vector<std::pair<std::uint32_t, CutsceneBin>> scripts) {
    // Append: frontends add entries one at a time (`session_sp`) or all at once (`nfgame`).
    scripts_.insert(scripts_.end(), std::make_move_iterator(scripts.begin()), std::make_move_iterator(scripts.end()));
}

void MissionSystem::apply_loadout(int slot) {
    // `Player_InitWeapon` fresh-start grants (docs/spec-weapons.md §10), by level low byte.
    const std::uint32_t lv = level_id_ & 0xFF;
    auto give = [&](int id, int rounds) { weapons_.give_weapon(slot, id, rounds); };
    auto upgrade = [&](int base, int rounds) { give(weapons_.table().upgrade(base), rounds); };
    const bool ram = false;  // RamLoad continuations restore on transitions (ram_load)
    (void)ram;
    if (lv == 0x01) {
        upgrade(6, 48);
        upgrade(74, 999);
        give(91, 999);  // v38 "Shaver" gadget [INFERENCE width: id 91]
        weapons_.select_weapon(slot, weapons_.table().upgrade(6));
    } else if (lv == 0x05 || lv == 0x06) {
        upgrade(6, 48);
        upgrade(74, 999);
        give(80, 0);
        give(84, 0);
        weapons_.select_weapon(slot, weapons_.table().upgrade(6));
    } else if (lv == 0x07 || lv == 0x08) {
        upgrade(6, 48);
        upgrade(74, 999);
        give(80, 0);
        give(84, 0);
        weapons_.select_weapon(slot, 1);  // fists
    } else if (lv == 0x09 || lv == 0x0A || lv == 0x0B) {
        upgrade(6, 48);
        upgrade(74, 999);
        upgrade(86, 0);
        give(88, 0);
        give(80, 0);
        upgrade(67, 999);
        give(91, 999);
        weapons_.select_weapon(slot, weapons_.table().upgrade(67));
    } else if (lv == 0x0C || lv == 0x0D) {
        upgrade(6, 48);
        upgrade(30, 999);
        upgrade(74, 999);
        give(80, 0);
        give(84, 0);
        give(88, 0);
        weapons_.select_weapon(slot, weapons_.table().upgrade(30));
    } else if (lv == 0x11 || lv == 0x12 || lv == 0x13 || lv == 0x4A) {
        give(16, 7);
        give(80, 0);
        weapons_.select_weapon(slot, 16);
    } else if (lv >= 0x14 && lv <= 0x17) {
        upgrade(6, 48);
        give(17, 12);
        give(52, 5);
        give(55, 5);
        upgrade(74, 999);
        give(80, 0);
        give(84, 0);
        upgrade(86, 0);
        weapons_.select_weapon(slot, 17);
    } else if (lv >= 0x18 && lv <= 0x1B) {
        give(51, 999);
        weapons_.give_armour(slot, 50.0f);
        weapons_.select_weapon(slot, 51);
    } else {
        upgrade(6, 48);  // debug/unknown levels: pistol + taser like 01
        upgrade(74, 999);
        weapons_.select_weapon(slot, weapons_.table().upgrade(6));
    }
}

MissionSystem::Inventory MissionSystem::ram_save(int slot) const {
    Inventory inv;
    for (int id = 1; id < WeaponTable::kWeaponCount; ++id) {
        if (!weapons_.owns(slot, id)) continue;
        inv.weapons.emplace_back(id, weapons_.clip(slot, id));
    }
    for (int a = 1; a < WeaponTable::kAmmoCount; ++a) {
        const int pool = weapons_.ammo_pool(slot, a);
        if (pool > 0) inv.ammo.emplace_back(a, pool);
    }
    inv.armour = weapons_.armour(slot);
    inv.selected = weapons_.selected_weapon(slot);
    inv.valid = true;
    return inv;
}

void MissionSystem::ram_load(World& world, int slot, const Inventory& inv) {
    if (!inv.valid) return;
    for (const auto& [id, rounds] : inv.weapons) weapons_.give_weapon(slot, id, rounds);
    if (auto* state = weapons_.state(slot)) {
        state->pool.fill(0);
        for (const auto& [type, rounds] : inv.ammo)
            state->pool.at(std::size_t(type)) = std::uint16_t(rounds);
        for (const auto& [id, rounds] : inv.weapons)
            weapons_.restore_clip(slot, id, rounds);
    }
    if (auto* player = world.player(slot)) player->set_armor(inv.armour);
    weapons_.select_weapon(slot, inv.selected);
}

void MissionSystem::pre_tick(World& world, const std::vector<bool>& use, FrameTiming timing,
                             const WeaponEvents* previous_weapon_events) {
     // Mines detonate through `WeaponSystem::explode_at` below, which needs the world for its
     // victim gather and shake; the weapon tick (which binds it) runs after this.
     weapons_.attach(world);
     // NOTE: the drone-channel import lives after `objects_->tick` below: the multiplex
     // re-eval unconditionally stores its outputs (e.g. OR-34/164), so importing before the
     // tick would let it stomp drone one-shot writes (deaths, spawner completion) on shared
     // numbers. Import-after + export-merged keeps drone 1-bits sticky in both stores.
    std::vector<SpObjects::Toucher> touchers;
    for (int i = 0; i < World::kMaxPlayers; ++i) {
        Player* p = world.player(i);
        if (!p || !p->alive() || !p->enabled()) continue;
        SpObjects::Toucher t;
        t.pos = p->pos;
        t.radius = 0.55f;
        t.height = 1.0f;
        t.is_player = true;
        t.slot = i;
        const Vec3 base = p->pos - Vec3{0, p->stand_height - p->capsule_radius, 0};
        t.capsule = CylinderQuery{base, p->pos + Vec3{0, 0.5f, 0}, p->capsule_radius};
        touchers.push_back(t);
    }
    if (sp_) {
        for (drone::Drone* d : sp_->sp_drones()) {
            if (!d) continue;
            SpObjects::Toucher t;
            t.pos = d->pos;
            t.radius = 0.5f;
            t.height = 1.0f;
            t.is_player = false;
            t.slot = -1;
            touchers.push_back(t);
        }
    }
    const float dt = 2.0f;  // pre_tick runs once per 30 Hz tick like the object updates
    objects_->tick(touchers, use, previous_weapon_events ? *previous_weapon_events : weapons_.events(), dt, &weapons_,
                   [&](int slot, float dmg, DamageType type, const std::array<float, 3>&) {
                       weapons_.hurt_player(slot, dmg, type, -1);
                   });
    const auto illuminated = objects_->update_searchlights(touchers, world.collision(), timing.mul());
    if (level_id_ == 0x07000005 && illuminated) {
        if (sp_ && !spotlight_contact_) sp_->alert_near_spotlight(*illuminated);
        music(2, spotlight_contact_ ? 5 : 3);
    }
    spotlight_contact_ = illuminated.has_value();
    // Script-driven solids (`SP_SetPosRot`): entity poses from the playing scripts become
    // movers before the players collide (one tick behind the script clock: scripts advance
    // in System::tick, after collision).
    std::vector<SpObjects::ScriptEntity> script_ents;
    for (const auto& p : players_) {
        for (const auto& e : p->entities()) {
            SpObjects::ScriptEntity se;
            se.hash = e.hash;
            se.pos = e.pos;
            se.quat = e.quat;
            script_ents.push_back(se);
        }
    }
    objects_->update_script_entities(script_ents);
    update_moving_geometry(world);
    auto boxes = objects_->movers();
    std::erase_if(boxes, [&](const Mover& m) {
        return std::find(door_handles_.begin(), door_handles_.end(), m.id) != door_handles_.end();
    });
    world.objects().set_movers(std::move(boxes));
    // Import drone one-shot writes AFTER the object tick (multiplex re-eval above stores its
    // outputs unconditionally), then export the merged state so drone 1-bits stay sticky.
    if (sp_) {
        for (int i = 0; i < 256; ++i)
            if (sp_->channels.on(i)) channels_.set(i, true);
    }
    for (int i = 0; i < 256; ++i) {
        const bool on = channels_.on(i);
        world.objects().set_channel(unsigned(i), on);  // ladders/icons read these
        if (sp_) sp_->channels.set(i, on);
    }
}
void MissionSystem::update_moving_geometry(World& world) {
    if (!moving_geometry_) {
        moving_geometry_ = &world.objects().enable_moving_meshes();
        for (const auto& door : objects_->doors()) {
            const auto& p = level_.placements()[door.placement];
            if (level_.chunks()[p.chunk].chunk.models[p.model].collision.empty()) continue;
            moving_geometry_->add_model(level_, door.placement, p.chunk, p.model, p.transform);
            door_handles_.push_back(door.placement);
        }
    }
    for (const auto& draw : objects_->draws()) {
        if (std::find(door_handles_.begin(), door_handles_.end(), draw.placement) != door_handles_.end())
            moving_geometry_->move_model(draw.placement, draw.transform);
    }
    if (level_id_ != 0x07000005 || !character_bank_) return;
    // The truck's rigid chassis is driven by bone 4 in its authored cutscene skin.
    constexpr std::uint32_t truck_skin = 0x05000005, chassis_hash = 0x020003BE;
    const auto actors = script_actors();
    const auto pose = std::find_if(actors.begin(), actors.end(),
                                   [](const auto& actor) { return actor.skin == truck_skin; });
    if (pose == actors.end()) return;
    parked_truck_pose_ = *pose;
    if (!truck_) {
        const auto* skin = character_bank_->skin(truck_skin);
        if (!skin) return;
        auto actor = std::make_unique<CharacterInstance>(*character_bank_, *skin);
        actor->extract_root_motion(false);
        if (!actor->play(pose->script, false)) return;
        truck_ = std::move(actor);
    }
    truck_->set_frame(pose->frame);
    const Mat4 next = truck_->bone_world(4);
    if (!truck_handle_) {
        for (std::size_t c = 0; c < level_.chunks().size(); ++c) {
            const auto& models = level_.chunks()[c].chunk.models;
            for (std::size_t m = 0; m < models.size(); ++m) {
                if (std::uint32_t(models[m].hash) != chassis_hash || models[m].collision.empty()) continue;
                truck_handle_ = level_.placements().size();
                moving_geometry_->add_model(level_, *truck_handle_, c, m, next);
                return;
            }
        }
        return;
    }
    if (Player* player = world.player(0); player && player->enabled() && player->jump_state == 0 &&
        (player->substate == SubState::Walk || player->substate == SubState::Crouch)) {
        const Vec3 sole = player->pos - Vec3{0, player->stand_height, 0};
        const Vec3 from = sole + Vec3{0, 0.2f, 0}, to = sole - Vec3{0, 0.3f, 0};
        const auto support = moving_geometry_->ray(from, to, pick::kIgnoreGhost);
        if (support && support->placement == *truck_handle_ && support->normal[1] > 0.5f) {
            const auto ground = world.collision().ray(from, to, pick::kIgnoreGhost);
            if (!ground || ground->dist >= support->dist - 0.02f) {
                const Vec3 carried = moving_geometry_->carry_point(*truck_handle_, next, player->pos);
                player->settled_pos += carried - player->pos;
                player->pos = carried;
            }
        }
    }
    moving_geometry_->move_model(*truck_handle_, next);
}

void MissionSystem::play_script(std::uint32_t hash) {
    for (const auto& [h, bin] : scripts_) {
        if (h != hash) continue;
        auto player = std::make_unique<CutscenePlayer>(&bin, hash, this);
        player->play(false);
        players_.push_back(std::move(player));
        std::printf("mission nis: %08x (%zu streams)\n", hash, bin.scripts.size());
        return;
    }
    std::printf("mission nis: %08x missing!\n", hash);
}

void MissionSystem::tick(World& world, FrameTiming timing) {
    ++frame_;
    stats_.frames++;
    // Import drone-side writes (spawner completion, deaths, mission states) before evaluating.
    if (sp_) {
        for (int i = 0; i < 256; ++i)
            if (sp_->channels.on(i)) channels_.set(i, true);
    }
    // Script-player anchors: auto-play intros, channel edges for the rest.
    for (SpObjects::ScriptPlayerAnchor& a : objects_->script_players()) {
        if (a.started) continue;
        if (a.auto_play || (a.trigger_channel != 0 && channels_.on(int(a.trigger_channel)))) {
            a.started = true;
            play_script(a.hash);
        }
    }
    if (std::uint32_t movie = objects_->take_movie()) play_script(movie);
    if (std::uint32_t dest = objects_->take_load_level()) {
        if (state_ == State::Failed) {
            // Fail bytes 26FCF0..3: the original takes no exit while failing.
        } else {
            pending_level_ = dest;
            music(3, 1);  // kLevelExit (`Trigger_Activate`)
            ram_save();  // `Player_RamSave`: snapshot slot 0 for the next level
        }
    }
    const float frames = timing.mul();  // 60 Hz frames this tick (2.0 at 30 Hz)
    for (auto& p : players_) {
        if (p->playing()) p->tick(frames);
        if (auto handoff = p->take_handoff(); handoff && player_disabled_)
            player_handoff_ = handoff;
    }
    // Route object + player queues into the hooks/queues.
    for (const SoundRequest& s : objects_->take_sounds()) {
        Vec3 pos{s.pos[0], s.pos[1], s.pos[2]};
        sounds_.push_back({{s.id, pos, s.positional}});
        if (hooks_.sound) hooks_.sound(s.id, pos, s.positional);
    }
    for (const SpObjects::Text& t : objects_->take_texts()) {
        texts_.push_back({t.label, t.frames, t.type});
        if (hooks_.message) hooks_.message(t.label, t.frames, t.type);
    }
    for (std::uint32_t ev : objects_->take_music()) {
        music_.push_back({int(ev), 2});
        if (hooks_.music) hooks_.music(int(ev), 2);
    }
    for (std::size_t i = 0; i < players_.size(); ++i) {
        auto& p = players_[i];
        for (const auto& s : p->take_sounds()) {
            sounds_.push_back({s, i + 1});
            if (hooks_.sound && s.action == CutscenePlayer::SoundAction::Start)
                hooks_.sound(s.id, s.pos, s.positional);
        }
        for (const auto& l : p->take_lights()) lights_.push_back({l.pos, l.intensity, l.type});
    }
    // Background script players may stay active after the cutscene hands control back.
    const bool lock_player = std::any_of(players_.begin(), players_.end(),
                                         [](const auto& script) { return script->locks_player(); });
    if (Player* player = world.player(0); player && lock_player != player_disabled_) {
        if (lock_player) player->disable();
        else {
            if (player->alive()) {
                if (player_handoff_) {
                    player->enable_at(player_handoff_->position, player_handoff_->yaw);
                    if (player_handoff_->mode != 0) player->attach_near_script_wall(world.collision());
                } else {
                    player->enable();
                }
            }
            player_handoff_.reset();
        }
        player_disabled_ = lock_player;
    }
    const bool lock_drones = std::any_of(players_.begin(), players_.end(),
                                         [](const auto& script) { return script->locks_drones(); });
    if (lock_drones != drones_disabled_) {
        enable_drones(!lock_drones);
        drones_disabled_ = lock_drones;
    }
    if (world.player(0) && !world.player(0)->alive() && state_ == State::Playing) {
        state_ = State::Failed;  // `byte_26FCF0` (`Player_CheckForDeath`): fail label 0x04000034
        state_timer_ = 240;
        fail_label_ = 0x04000034;
        music(6, 1);
    }
    if (state_ == State::Playing) monitor_objectives();
    update_state(timing);
    // Poll-based channel watch for headless driver tracing.
    for (int i = 0; i < 256; ++i) {
        const bool on = channels_.on(i);
        if (!watch_init_ || on != (channel_watch_[std::size_t(i)] != 0)) {
            if (watch_init_) channel_log_.push_back({frame_, i, on});
            channel_watch_[std::size_t(i)] = on ? 1 : 0;
        }
    }
    watch_init_ = true;
    // Fade easing toward its target.
    if (fade_ != fade_target_) {
        const float step = fade_rate_ * frames / 60.0f;
        fade_ = (fade_ < fade_target_) ? std::min(fade_target_, fade_ + step) : std::max(fade_target_, fade_ - step);
        if (hooks_.fade) hooks_.fade(fade_);
    }
}

void MissionSystem::monitor_objectives() {
    // `Mission_MonitorObjectives`: every objective's channel feeds the all-set AND; state
    // transitions announce/complete/fail with the original's labels and timings.
    bool all = !entry_.has_later_objectives;
    for (Objective& o : objectives_) {
        const bool held = o.def.channel != 0 && channels_.on(int(o.def.channel));
        const bool v19 = o.def.inverted() ? !held : held;
        all = all && v19;
        switch (o.state) {
        case ObjectiveState::Announce:
            texts_.push_back({o.def.label, 300, 2});
            if (hooks_.message) hooks_.message(o.def.label, 300, 2);
            o.state = o.def.inverted() ? ObjectiveState::Monitored : ObjectiveState::DoneShown;
            break;
        case ObjectiveState::WaitSecond:
            if (o.def.channel2 != 0 && o.def.channel2 != 255 && channels_.on(int(o.def.channel2))) {
                texts_.push_back({o.def.label, 300, 2});
                if (hooks_.message) hooks_.message(o.def.label, 300, 2);
                o.state = o.def.inverted() ? ObjectiveState::Monitored : ObjectiveState::DoneShown;
            }
            break;
        case ObjectiveState::Monitored:
            // Inverted-flag objectives hold their channel: clearing it fails the mission
            // ("Mission Failed: <label>"); holding it just feeds the all-set AND below.
            if (!v19) {
                texts_.push_back({o.def.label, 180, 2});
                if (hooks_.message) hooks_.message(o.def.label, 180, 2);
                o.state = ObjectiveState::Failed;
                state_ = State::Failed;
                state_timer_ = 240;
                fail_label_ = o.def.fail_label;
                music(7, 1);
            }
            break;
        case ObjectiveState::DoneShown:
            if (v19) o.state = ObjectiveState::Done;
            break;
        case ObjectiveState::Failed:
            if (o.def.fail_label != 0xFFFFFFFF) fail_label_ = o.def.fail_label;
            break;
        case ObjectiveState::Done:
            break;
        }
    }
    if (state_ == State::Failed && fail_label_ == 0) fail_label_ = 0x04000034;
    if (all && state_ == State::Playing) {
        music(8, 1);  // every objective channel holds: mission complete
        state_ = State::Succeeded;
        state_timer_ = 240;  // 4 s at 60 Hz like the original's TimeOut
        texts_.push_back({0x02000006, 240, 3});
        if (hooks_.message) hooks_.message(0x02000006, 240, 3);
    }
}

void MissionSystem::update_state(FrameTiming timing) {
    if (state_ == State::Succeeded || state_ == State::Failed) {
        state_timer_ -= timing.mul();
        if (state_timer_ <= 0) state_ = State::Done;
    }
    if (state_ == State::Failed && state_timer_ <= 0) {
        pending_level_ = mission_fail_destination(level_id_);
    }
}

std::optional<CutscenePlayer::Camera> MissionSystem::script_camera() const {
    for (const auto& p : players_) {
        if (auto cam = p->camera()) return cam;
    }
    return std::nullopt;
}

const std::vector<Mover>& MissionSystem::movers() const { return objects_->movers(); }
const std::vector<SpObjects::DrawOverride>& MissionSystem::draws() const { return objects_->draws(); }
const std::vector<std::size_t>& MissionSystem::hides() const { return objects_->hides(); }

std::vector<CutscenePlayer::ActorPose> MissionSystem::script_actors() const {
    std::vector<CutscenePlayer::ActorPose> actors;
    for (const auto& script : players_) {
        const auto poses = script->actors();
        actors.insert(actors.end(), poses.begin(), poses.end());
    }
    if (parked_truck_pose_ && std::none_of(actors.begin(), actors.end(), [&](const auto& actor) {
            return actor.skin == parked_truck_pose_->skin;
        })) actors.push_back(*parked_truck_pose_);
    return actors;
}

std::vector<MissionSystem::Sound> MissionSystem::take_sounds() {
    std::vector<Sound> out;
    out.swap(sounds_);
    return out;
}

std::vector<MissionSystem::Text> MissionSystem::take_texts() {
    std::vector<Text> out;
    out.swap(texts_);
    return out;
}

std::vector<MissionSystem::Music> MissionSystem::take_music() {
    std::vector<Music> out;
    out.swap(music_);
    return out;
}

std::vector<MissionSystem::Spawn> MissionSystem::take_spawns() {
    std::vector<Spawn> out;
    out.swap(spawns_);
    return out;
}

std::vector<MissionSystem::Light> MissionSystem::take_lights() {
    std::vector<Light> out;
    out.swap(lights_);
    return out;
}

std::vector<MissionSystem::ChannelEvent> MissionSystem::take_channel_log() {
    std::vector<ChannelEvent> out;
    out.swap(channel_log_);
    return out;
}

std::vector<MissionSystem::ObjectiveInfo> MissionSystem::objectives() const {
    std::vector<ObjectiveInfo> out;
    for (const Objective& o : objectives_) out.push_back({o.def, o.state});
    return out;
}

std::uint64_t MissionSystem::kills() const { return sp_ ? sp_->stats.deaths : 0; }

bool MissionSystem::channel(std::uint16_t ch) const { return channels_.on(int(ch)); }

void MissionSystem::set_channel(std::uint16_t ch, std::uint8_t value) { channels_.set(int(ch), value != 0); }

void MissionSystem::spawn_drone(const std::array<float, 3>& pos, const std::uint32_t args[4]) {
    Spawn s;
    s.pos = pos;
    s.args[0] = args[0];
    s.args[1] = args[1];
    s.args[2] = args[2];
    s.args[3] = args[3];
    spawns_.push_back(s);
    if (hooks_.spawn_drone) hooks_.spawn_drone(pos, args);
}

void MissionSystem::enable_drones(bool enable) {
    if (!sp_) return;  // `Drone_EnableAll`: without Bots there is nothing to freeze
    for (drone::Drone* d : sp_->sp_drones()) {
        if (!d) continue;
        if (enable) {
            if (std::find(suspended_drones_.begin(), suspended_drones_.end(), d->id) != suspended_drones_.end())
                sp::enable_drone(*d, true);
        } else if (!d->ghost && !d->hidden) {
            suspended_drones_.push_back(d->id);
            sp::enable_drone(*d, false);
        }
    }
    if (enable) suspended_drones_.clear();
}

void MissionSystem::break_near(const std::array<float, 3>& pos) { objects_->break_near(pos); }

void MissionSystem::set_link_byte(const std::array<float, 3>& pos, std::uint8_t value) {
    objects_->set_link_byte(pos, value);
}

void MissionSystem::load_level(std::uint32_t id) { pending_level_ = id; }

void MissionSystem::set_scriptcam(std::uint32_t id) { scriptcam_ = id; }

void MissionSystem::camera_mode(std::uint32_t mode) { (void)mode; }  // `Player_Cam2Mode`: HUD-side

void MissionSystem::fail_mission(std::uint32_t label) {
    if (state_ != State::Playing) return;
    state_ = State::Failed;
    state_timer_ = 240;
    fail_label_ = label;
    music(6, 1);
}

void MissionSystem::ram_save() {
    // `Player_RamSave`: snapshot slot 0 for the next level (carried by the frontend).
    carried_inventory_ = ram_save(0);
}

void MissionSystem::text(std::uint32_t label, std::uint16_t frames) {
    texts_.push_back({label, int(frames), 4});
    if (hooks_.message) hooks_.message(label, int(frames), 4);
}


void MissionSystem::fade(float seconds) {
    // `Camera_SetFade(4, 255, 0, -t)`: positive data fades out, negative fades in [INFERENCE].
    if (seconds > 0) {
        fade_target_ = 1.0f;
        fade_rate_ = 1.0f / seconds;
    } else if (seconds < 0) {
        fade_target_ = 0.0f;
        fade_rate_ = 1.0f / (-seconds);
    }
    if (hooks_.fade) hooks_.fade(fade_target_);
}

void MissionSystem::music(std::uint32_t id, std::int32_t value) {
    music_.push_back({int(id), int(value)});
    if (hooks_.music) hooks_.music(int(id), int(value));
}

void MissionSystem::message_callback(std::uint16_t arg, std::uint32_t data) {
    (void)arg;
    (void)data;  // level-specific C callbacks (event 15): no-ops without the EE
}

}  // namespace nf
