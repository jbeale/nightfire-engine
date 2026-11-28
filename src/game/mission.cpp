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
    : level_(level), level_id_(level_id), entry_(entry), weapons_(weapons), sp_(sp),
      objects_(std::make_unique<SpObjects>(level, level_id, channels_, sp ? sp->config().difficulty : 2)) {
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
    if (level_id_ == 0x07000008 && level_.map()) {
        const auto& map = level_.map()->chunk;
        for (const auto& ref : static_path_refs(map)) {
            const auto& obj = map.statics[ref.static_index];
            if (obj.object_class() != 0x2f || obj.param(0) != 0 || ref.aux == 0) continue;
            const auto& placements = level_.placements();
            const auto placed = std::find_if(placements.begin(), placements.end(), [&](const Placement& p) {
                return p.instance == ref.static_index;
            });
            auto tracks = parse_path_data(map);
            if (placed == placements.end() || ref.path_index >= tracks.size() ||
                tracks[ref.path_index].keys.size() < 2) continue;
            EscapeCopter copter{std::size_t(placed - placements.begin()), std::uint16_t(obj.param(6)),
                std::uint16_t(obj.param(7)), std::uint16_t(obj.param(3)), ref.aux,
                obj.param(4), obj.param(5), std::move(tracks[ref.path_index])};
            const int difficulty = sp_ ? sp_->config().difficulty : 2;
            copter.health = copter.max_health = 299.0f * (difficulty == 2 ? 1.5f : difficulty == 3 ? 2.0f : 1.0f);
            copter.pose = placed->transform;
            escape_copter_ = std::move(copter);
            break;
        }
    }
    if (sp_) {
        sp_->on_mission_fail = [this](drone::Drone&, int reason, std::uint32_t label) {
            if (state_ != State::Playing) return;
            // DoorGuardHandler reports a blown cover failure through switch 99.
            if (reason == 0xc) {
                set_channel(99, 1);
                label = 0x04000033;
            }
            if (label != 0) texts_.push_back({label, 300, 3});
            fail_mission(label);
        };
    }
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
    upgrade(0x4e, 999);  // Player_InitWeapon equips the watch laser after the level-specific loadout
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
    if (!moving_geometry_ && level_id_ == 0x07000008) {
        for (const auto* room : {"Room_Stairwell_T_1", "Room_Stairwell_T_2", "Room_Stairwell_T_3", "Room_Guardhouse_F2"})
            world.enable_portal_geometry(room);
        world.hide_room_overlap("Room_Guardhouse_F2", "Room_Outside_3");
    }
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
            if (!d || !d->alive() || d->hidden || d->ghost) continue;
            SpObjects::Toucher t;
            t.pos = d->pos;
            t.radius = 0.5f;
            t.height = 1.0f;
            t.is_player = false;
            t.slot = -1;
            t.capsule = CylinderQuery{d->feet() + Vec3{0, t.radius, 0}, d->pos, t.radius};
            touchers.push_back(t);
        }
    }
    const float dt = timing.mul();
    objects_->tick(touchers, use, previous_weapon_events ? *previous_weapon_events : weapons_.events(), dt, &weapons_,
                   [&](int slot, float dmg, DamageType type, const std::array<float, 3>&) {
                       weapons_.hurt_player(slot, dmg, type, -1);
                   });
    // Touch and switch triggers can start a cutscene before the same frame's AI update.
    // Apply its time-zero locks before guards react to Bond entering the trigger.
    start_ready_scripts(world);
    apply_script_locks(world);
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
    const auto& weapon_events = previous_weapon_events ? *previous_weapon_events : weapons_.events();
    update_escape_copter(world, weapon_events, timing);
    update_moving_geometry(world);
    update_script_models(world, weapon_events);
    const auto illuminated = objects_->update_searchlights(touchers, world.collision(), timing.mul(), moving_geometry_);
    if (level_id_ == 0x07000005 && illuminated) {
        if (sp_ && !spotlight_contact_) sp_->alert_near_spotlight(*illuminated);
        music(2, spotlight_contact_ ? 5 : 3);
    }
    spotlight_contact_ = illuminated.has_value();
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
        for (const auto& blocker : objects_->dynamic_blockers()) {
            const auto& p = level_.placements()[blocker.placement];
            moving_geometry_->add_model(level_, blocker.placement, p.chunk, p.model, p.transform);
        }
    }
    for (const auto& blocker : objects_->dynamic_blockers())
        moving_geometry_->set_model_enabled(blocker.placement, blocker.visible);
    for (const auto& prop : objects_->breakables()) {
        if (prop.damaged_model == SIZE_MAX) continue;
        const auto& p = level_.placements()[prop.placement];
        const auto model = prop.broken ? prop.damaged_model : p.model;
        auto [where, added] = breakable_models_.try_emplace(prop.placement, model);
        if (added || where->second != model) {
            moving_geometry_->set_object_model(level_, prop.placement, model);
            where->second = model;
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

void MissionSystem::update_escape_copter(World& world, const WeaponEvents& events, FrameTiming timing) {
    if (!escape_copter_ || !character_bank_) return;
    auto& copter = *escape_copter_;
    objects_->hide_placement(copter.placement);
    if (copter.destroyed || !channels_.on(copter.start)) return;
    if (!copter.flight) copter.flight = play_script(copter.flight_script, true);
    if (!copter.flight) return;

    // These events were traced against the previous tick's aircraft. Use that same
    // hull collider and transform before moving the flight script to its next pose.
    if (copter.hull) {
        const auto& hull = *copter.hull;
        const auto& mesh = level_.mesh(hull.model.chunk, hull.model.model);
        const auto& transform = hull.model.transform;
        const Vec3 origin{transform[12], transform[13], transform[14]};
        const auto hull_impact = [&](const ImpactEvent& impact) {
            return !impact.on_body && impact.placement == hull.handle && impact.weapon >= 0 &&
                   impact.weapon < WeaponTable::kWeaponCount;
        };
        for (const auto& impact : events.impacts) {
            if (hull_impact(impact))
                copter.health -= weapons_.table().weapon(impact.weapon).damage;
        }
        for (const auto& explosion : events.explosions) {
            if (explosion.weapon < 0 || explosion.weapon >= WeaponTable::kWeaponCount || explosion.radius <= 0)
                continue;
            // Projectile impacts and the explosions they create share the contact point.
            if (std::any_of(events.impacts.begin(), events.impacts.end(), [&](const auto& impact) {
                    return hull_impact(impact) && impact.weapon == explosion.weapon && impact.point == explosion.position;
                })) continue;
            const Vec3 delta = explosion.position - origin;
            const Vec3 local{dot(delta, Vec3{transform[0], transform[1], transform[2]}),
                             dot(delta, Vec3{transform[4], transform[5], transform[6]}),
                             dot(delta, Vec3{transform[8], transform[9], transform[10]})};
            Vec3 nearest{};
            for (int axis = 0; axis < 3; ++axis)
                nearest[axis] = std::clamp(local[axis], mesh.bbox_min[axis], mesh.bbox_max[axis]);
            const float distance = length(local - nearest);
            if (distance < explosion.radius)
                copter.health -= weapons_.table().weapon(explosion.weapon).damage * (1 - distance / explosion.radius);
        }
    }
    if (copter.health <= 0) {
        copter.destroyed = true;
        copter.flight->stop();
        if (auto* crash = play_script(copter.crash_script)) crash->set_origin(copter.pose);
        set_channel(copter.done, 1);
        return;
    }

    // Copter_Create loads the authored framelist and a separate script for the airframe and rotors.
    const auto& keys = copter.path.keys;
    const auto last = keys.size() - 1;
    const auto a = std::min(std::size_t(copter.clock), last - 1);
    const float fraction = copter.clock - float(a);
    const auto q = slerp_acc(keys[a].quat, keys[a + 1].quat, fraction);
    const Vec3 pos = keys[a].pos * (1 - fraction) + keys[a + 1].pos * fraction;
    copter.pose = quat_trans_to_mat({q[0], q[1], q[2], q[3]}, pos);
    copter.flight->set_origin(copter.pose);
    copter.clock += timing.mul() * float(last) / float(copter.path_duration);
    if (copter.clock > float(last)) copter.clock = float(std::min(std::size_t(copter.restart_key), last));

    // Copter_Update starts trailing smoke once the aircraft has lost half its health.
    if (copter.health < copter.max_health * .5f) {
        copter.smoke_wait -= timing.mul();
        if (copter.smoke_wait <= 0) {
            copter_smoke_ = transform_point(copter.pose, {0, 2.0f, -3.3f});
            copter.smoke_wait += 6;
        }
    }

    const auto* bond = world.player(0);
    if (!bond || !bond->alive() || !bond->enabled()) return;
    const auto& b = copter.pose;
    const Vec3 aim = bond->eye();
    const Vec3 forward{b[8], b[9], b[10]};
    const Vec3 muzzle = transform_point(b, {0.044f, -2.194f, 9.0f});
    const Vec3 direction = normalised(aim - muzzle);
    const bool in_sight = dot(direction, forward) > 0.5f &&
                          world.collision().line_of_sight(muzzle, aim);
    copter.attack_wait -= timing.mul();
    if (!in_sight) copter.burst_remaining = 0;
    if (in_sight && copter.attack_wait <= 0 && copter.burst_remaining <= 0) {
        copter.attack_wait = 540;
        copter.burst_remaining = 300;
        copter.shot_wait = 0;
        music(2, 1);
    }
    if (copter.burst_remaining > 0) {
        copter.burst_remaining -= timing.mul();
        copter.shot_wait -= timing.mul();
        if (copter.shot_wait <= 0) {
            weapons_.fire({-1, muzzle, direction}, 0x61, 0x5df);
            copter.shot_wait += 6;
        }
    }
}

std::optional<Vec3> MissionSystem::take_copter_smoke() { return std::exchange(copter_smoke_, std::nullopt); }

bool MissionSystem::hit_copter_hull(const ImpactEvent& impact) const {
    return escape_copter_ && escape_copter_->hull && !impact.on_body &&
           impact.placement == escape_copter_->hull->handle;
}

void MissionSystem::update_script_models(World& world, const WeaponEvents& events) {
    script_models_.clear();
    if (escape_copter_) escape_copter_->hull.reset();
    if (!character_bank_) return;
    for (auto& [hash, collider] : script_colliders_) collider.active = false;

    Player* player = world.player(0);
    std::optional<std::size_t> support;
    if (player && player->enabled() && player->jump_state == 0 &&
        (player->substate == SubState::Walk || player->substate == SubState::Crouch)) {
        const Vec3 feet = player->pos - Vec3{0, player->stand_height, 0};
        const auto hit = moving_geometry_->ray(feet + Vec3{0, .2f, 0}, feet - Vec3{0, .3f, 0}, pick::kIgnoreGhost);
        if (hit && hit->normal[1] > .5f) support = hit->placement;
    }

    const auto move_collider = [&](std::size_t handle, const Mat4& transform) {
        if (support == handle) {
            const Vec3 carried = moving_geometry_->carry_point(handle, transform, player->pos);
            player->settled_pos += carried - player->pos;
            player->pos = carried;
        }
        moving_geometry_->move_model(handle, transform);
    };
    bool gondola_present = false;
    const auto& chunks = level_.chunks();
    for (std::size_t i = 0; i < players_.size(); ++i) {
        const auto& script = players_[i];
        const bool gondola = script->id() == 0x0600004c;
        for (const auto& entity : script->entities()) {
            if (gondola && entity.stream == 20 &&
                (entity.hash == 0x0200037a || entity.hash == 0x02000431)) {
                // The intact and damaged shells share the same deck, but swap collision models
                // during the stop. Keep one support attached to the cabin through that swap.
                constexpr float deck_y = -5.58f;
                gondola_present = true;
                if (!gondola_floor_handle_) {
                    gondola_floor_handle_ = level_.placements().size() + 1;
                    moving_geometry_->add_floor(*gondola_floor_handle_, entity.transform,
                                                {-1.49f, deck_y, -2.06f}, {1.49f, deck_y, 2.06f});
                } else {
                    move_collider(*gondola_floor_handle_, entity.transform);
                }
                if (sp_) {
                    for (drone::Drone* d : sp_->sp_drones()) {
                        if (!d->alive() || d->skin_hash != 0x0500003b) continue; // Zoe
                        d->riding_scripted_platform = true;
                        d->pos = transform_point(entity.transform, {-.86f, deck_y, .7f}) +
                                 Vec3{0, d->stand_height, 0};
                        d->velocity = d->fall_velocity = {};
                        d->on_ground = true;
                        if (!script->staged() && !channel(155)) set_channel(155, 1);
                        break;
                    }
                }
            }
            const bool pickup = entity.kind == 0xf0;
            const bool destructible = gondola && entity.kind == 0xe1;
            const bool glass = gondola && entity.kind == 0x20;
            if (entity.hash == 0 || (entity.kind >= 0xe0 && !pickup && !destructible)) continue;
            const auto ref = character_bank_->find_model(entity.hash);
            if (!ref || ref->chunk >= chunks.size() || ref->model >= chunks[ref->chunk].chunk.models.size() ||
                chunks[ref->chunk].entry.hash != character_bank_->chunks()[ref->chunk].entry.hash) continue;
            const auto& model = chunks[ref->chunk].chunk.models[ref->model];
            if (!pickup && entity.kind != 0x2f && std::any_of(level_.placements().begin(), level_.placements().end(), [&](const Placement& p) {
                    return p.chunk == ref->chunk && p.model == ref->model;
                })) continue; // SpObjects owns the override for placed models
            if (pickup) {
                const auto key = std::tuple{i, entity.stream, entity.hash};
                const auto& args = entity.params;
                if (collected_script_pickups_.contains(key) || (args[0] == 4 && args[3] && channel(args[3]))) continue;
                const auto& mesh = level_.mesh(ref->chunk, ref->model);
                const Vec3 spot = transform_point(entity.transform, (mesh.bbox_min + mesh.bbox_max) * .5f);
                if (player && player->enabled() && player->alive() && length(player->pos - spot) < 1.5f &&
                    world.collision().line_of_sight(player->eye(), spot, pick::kIgnoreGhost) &&
                    moving_geometry_->line_of_sight(player->eye(), spot, pick::kIgnoreGhost)) {
                    const auto id = int(args[1]);
                    const auto amount = int(args[2]);
                    bool taken = false;
                    if (args[0] == 0 && id > 0 && id < WeaponTable::kWeaponCount)
                        taken = weapons_.give_weapon(0, weapons_.table().upgrade(id), amount);
                    if (args[0] == 1) taken = weapons_.give_ammo(0, id, amount);
                    if (args[0] == 4) taken = true;
                    if (taken) {
                        collected_script_pickups_.insert(key);
                        if (args[3] != 0) set_channel(std::uint16_t(args[3]), 1);
                        sounds_.push_back({{args[4] == 0xffff || args[4] == 0 ? 245u : args[4], spot, false}});
                        continue;
                    }
                }
            }
            std::size_t rendered_model = ref->model;
            std::optional<float> health;
            if (glass || destructible) {
                health = float(entity.params[glass ? 1 : 3]);
                const auto original = script_colliders_.find({i, entity.stream, ref->chunk, ref->model});
                if (original != script_colliders_.end() && original->second.health) {
                    float& hp = *original->second.health;
                    for (const ImpactEvent& impact : events.impacts) {
                        if (impact.on_body || impact.placement != original->second.handle ||
                            impact.weapon < 0 || impact.weapon >= WeaponTable::kWeaponCount) continue;
                        const auto& weapon = weapons_.table().weapon(impact.weapon);
                        if ((weapon.class_flags & 0xf8) != 0) hp -= weapon.damage;
                    }
                    if (hp < 0 || (destructible && entity.params[1] != 0 && channel(std::uint16_t(entity.params[1])))) {
                        hp = -1;
                        if (model.linked_lod < 0) continue;
                        rendered_model = std::size_t(model.linked_lod);
                        if (rendered_model >= chunks[ref->chunk].chunk.models.size()) continue;
                        health.reset();
                    }
                }
            }
            script_models_.push_back({ref->chunk, rendered_model, 0, entity.transform});
            if (chunks[ref->chunk].chunk.models[rendered_model].collision.empty() ||
                (entity.kind != 0x31 && entity.kind != 0x20 && entity.kind != 0x2f && !destructible)) continue;

            const auto handle = level_.placements().size() + 2 + script_colliders_.size();
            auto [it, added] = script_colliders_.try_emplace({i, entity.stream, ref->chunk, rendered_model},
                                                            ScriptCollider{handle, false, health});
            auto& collider = it->second;
            if (collider.active) continue;
            collider.active = true;
            if (added) moving_geometry_->add_model(level_, collider.handle, ref->chunk, rendered_model, entity.transform);
            else move_collider(collider.handle, entity.transform);
            moving_geometry_->set_model_enabled(collider.handle, true);
            if (escape_copter_ && script.get() == escape_copter_->flight && entity.hash == 0x0200035d)
                escape_copter_->hull = EscapeCopter::Hull{collider.handle, script_models_.back()};
        }
    }
    if (gondola_floor_handle_) {
        moving_geometry_->set_model_enabled(*gondola_floor_handle_, gondola_present);
        if (!gondola_present && sp_)
            for (drone::Drone* d : sp_->sp_drones()) d->riding_scripted_platform = false;
    }
    for (const auto& [hash, collider] : script_colliders_)
        if (!collider.active) moving_geometry_->set_model_enabled(collider.handle, false);
}

CutscenePlayer* MissionSystem::play_script(std::uint32_t hash, bool stage) {
    for (const auto& [h, bin] : scripts_) {
        if (h != hash) continue;
        auto player = std::make_unique<CutscenePlayer>(&bin, hash, this, character_bank_);
        if (stage) player->stage();
        else {
            player->play(false);
            player->tick(0);
        }
        players_.push_back(std::move(player));
        if (!stage) std::printf("mission nis: %08x (%zu streams)\n", hash, bin.scripts.size());
        return players_.back().get();
    }
    std::printf("mission nis: %08x missing!\n", hash);
    return nullptr;
}

void MissionSystem::start_ready_scripts(World& world) {
    // Script-player anchors: auto-play intros, channel edges for the rest.
    const Player* bond = world.player(0);
    const bool can_trigger = state_ == State::Playing && bond && bond->alive();
    for (SpObjects::ScriptPlayerAnchor& a : objects_->script_players()) {
        if (a.started) continue;
        if (a.auto_play || (can_trigger && channels_.on(a.trigger_channel))) {
            a.started = true;
            if (a.staged) {
                a.staged->resume();
                std::printf("mission nis: %08x (resumed)\n", a.hash);
            } else play_script(a.hash);
        } else if (a.preload && a.trigger_channel != 0 && !a.staged && can_trigger) {
            a.staged = play_script(a.hash, true);
            if (!a.staged) a.preload = false;
        }
    }
}

void MissionSystem::apply_script_locks(World& world) {
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
}

void MissionSystem::tick(World& world, FrameTiming timing) {
    ++frame_;
    stats_.frames++;
    // Import drone-side writes (spawner completion, deaths, mission states) before evaluating.
    if (sp_) {
        for (int i = 0; i < 256; ++i)
            if (sp_->channels.on(i)) channels_.set(i, true);
    }
    start_ready_scripts(world);
    if (std::uint32_t movie = objects_->take_movie()) play_script(movie);
    if (std::uint32_t dest = objects_->take_load_level()) {
        if (state_ != State::Failed && state_ != State::Done) {
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
            sounds_.push_back({s, s.animation ? 0 : i + 1});
            if (hooks_.sound && s.action == CutscenePlayer::SoundAction::Start)
                hooks_.sound(s.id, s.pos, s.positional);
        }
        for (const auto& l : p->take_lights()) lights_.push_back({l.pos, l.intensity, l.type});
    }
    apply_script_locks(world);
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

const CutscenePlayer* MissionSystem::skippable_cutscene() const {
    for (const auto& scene : players_) {
        if (scene->camera()) return scene->locks_player() ? scene.get() : nullptr;
    }
    return nullptr;
}

void MissionSystem::finish_cutscene(World& world, CutscenePlayer& scene, FrameTiming timing) {
    fast_forwarding_ = true;
    scene.finish(timing.mul());
    fast_forwarding_ = false;
    if (auto handoff = scene.take_handoff(); handoff && player_disabled_) player_handoff_ = handoff;
    fade_ = fade_target_;
    apply_script_locks(world);
}

void MissionSystem::skip_cutscene(World& world, FrameTiming timing) {
    const auto* active = skippable_cutscene();
    if (!active) return;
    for (const auto& scene : players_) {
        if (scene.get() != active) continue;
        std::printf("mission skip: %08x\n", scene->id());
        finish_cutscene(world, *scene, timing);
        return;
    }
}

void MissionSystem::finish_checkpoint_scene(World& world, std::uint32_t hash, FrameTiming timing) {
    for (auto& anchor : objects_->script_players())
        if (anchor.hash == hash) anchor.started = true;
    if (auto* scene = play_script(hash)) {
        apply_script_locks(world);
        finish_cutscene(world, *scene, timing);
    }
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

std::vector<CutscenePlayer::Sprite> MissionSystem::script_sprites() const {
    std::vector<CutscenePlayer::Sprite> sprites;
    for (const auto& script : players_) {
        const auto active = script->sprites();
        sprites.insert(sprites.end(), active.begin(), active.end());
    }
    return sprites;
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

void MissionSystem::set_channel(std::uint16_t ch, std::uint8_t value) {
    channels_.set(ch, value != 0);
    if (sp_) sp_->channels.set(ch, value != 0);
}

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
    if (fast_forwarding_) return;
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
