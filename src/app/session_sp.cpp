// Single-player ACTION session: build, tick, draw, pause and end-mission.
#include "app/session_sp.hpp"

#include "app/movie.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

#include "assets/bin_archive.hpp"
#include "assets/character.hpp"
#include "assets/cutscene.hpp"
#include "assets/level.hpp"
#include "assets/menu_validate.hpp"
#include "assets/mission_data.hpp"
#include "assets/strings.hpp"
#include "audio/audio.hpp"
#include "audio/music_director.hpp"
#include "game/arena_view.hpp"   // kViewFovY (Camera_CalcViewAngles 1.0471976)
#include "game/drone_render.hpp"
#include "game/drone_system.hpp"
#include "game/local_pad.hpp"
#include "game/mission.hpp"
#include "game/nav.hpp"
#include "game/nfgame_effects.hpp"
#include "game/nfgame_weapon_view.hpp"
#include "game/plr_stats.hpp"
#include "game/sp_common.hpp"
#include "game/sp_placement.hpp"
#include "game/sp_tables.hpp"
#include "game/weapons.hpp"
#include "game/world.hpp"
#include "render/character_renderer.hpp"
#include "render/gl.hpp"
#include "render/level_renderer.hpp"
#include "render/weather_renderer.hpp"
#include "ui/prompts.hpp"

namespace nf::app {

namespace {

constexpr float kTau = 6.2831853071795864769f;

Camera interpolate_camera(const Vec3& previous_eye, float previous_yaw, float previous_pitch,
                          const Player& current, float alpha) {
    const Vec3 eye = previous_eye + (current.shaken_eye() - previous_eye) * alpha;
    const float yaw = previous_yaw + std::remainder(current.yaw - previous_yaw, kTau) * alpha;
    const float pitch = previous_pitch + (current.view_pitch() - previous_pitch) * alpha;
    return camera_for_eye_yaw_pitch(eye, yaw, pitch);
}
// P_ENDMISSION lives in the level-bin menu script (frontend_level.cpp); P_NFRESULTS in the frontend script.
constexpr std::uint32_t kPageEndMission = 0x40000042, kPageResults = 0x40000036;

// Menu pad from the keyboard plus the first gamepad's buttons (the shared menu bindings).
PadState menu_pad(SDL_Gamepad* gamepad) { return input_bindings().sample(InputContext::Menu, gamepad); }

}  // namespace

struct SpSession::Impl {
    Impl(AppContext& c, Window& w, ui::Renderer& u, ui::TextRenderer& t, const SpLaunch& l, AppConfig& cfg)
        : ctx(c), window(w), ui(u), text(t), launch(l), config(cfg), timing(float(cfg.logic_hz)), user_config(cfg) {}

    AppContext& ctx;
    Window& window;
    ui::Renderer& ui;
    ui::TextRenderer& text;
    SpLaunch launch;
    AppConfig config;
    FrameTiming timing;
    AppConfig& user_config;   // controller style / Y-inversion edited in the pause menu persist here

    // Declared in dependency order (Level outlives World, bank outlives renderers).
    std::vector<std::uint8_t> bin_bytes;  // raw level .bin (cutscenes, menu script)
    std::unique_ptr<Level> level;
    std::uint32_t level_id = 0;
    std::unique_ptr<World> world;
    std::unique_ptr<CharacterBank> bank;
    WeaponSystem* weapons = nullptr;  // owned by World
    drone::DroneSystem* drones = nullptr;
    sp::SpSystem* spsys = nullptr;
    MissionSystem* mission_sys = nullptr;  // owned by World (absent when the level has no MissionData entry)
    std::unique_ptr<NavNetwork> nav;
    SpriteLibrary fx_sprites;  // level chunk sprites for WeaponEffects (must outlive `effects`)
    std::unique_ptr<WeaponEffects> effects;
    std::unique_ptr<Hud> hud;
    std::unique_ptr<LevelRenderer> renderer;
    std::unique_ptr<WeatherRenderer> weather;
    std::unique_ptr<CharacterRenderer> chars;
    std::unique_ptr<WeaponView> weapon_view;
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::unique_ptr<CharacterInstance>> script_characters;
    std::unique_ptr<drone::DroneRenderer> drone_renderer;
    ViewModel previous_viewmodel{};
    MenuFile level_menu;
    bool has_level_menu = false;

    std::unique_ptr<SoundArchive> archive;
    std::unique_ptr<audio::AudioSystem> audio;
    std::unique_ptr<audio::MusicDirector> music;
    struct ScriptSound {
        audio::SfxHandle handle = 0;
        std::int32_t tag = 0;
    };
    std::map<std::pair<std::size_t, std::uint16_t>, ScriptSound> script_sounds;
    std::int32_t next_script_sound_tag = 1;
    float step_distance = 0.0f;
    SDL_Gamepad* gamepad = nullptr;
    LocalPad pad{true, nullptr};

    int death_frames = 0;  // ticks since the end began (Done, death or fail signal)
    bool end_shown = false;
    bool mission_won = false;  // the state machine reported Succeeded before Done
    PlrStats plr_;              // mission counters feeding the results score
    PlrScoreTables score_tables_;
    bool score_tables_ok_ = false;
    bool give_given = false, give_done = false;  // --give ID debug equip progress
    int prev_muzzle_ = 0;              // muzzle_frames edge = one trigger pull
    std::uint64_t prev_deaths_ = 0;    // SpSystem::stats.deaths edge = kills
    std::uint64_t prev_mframes_ = 0;   // mission frames edge = elapsed time
    bool prev_use_ = false;            // Cross edge for pre_tick (held would flap switches)

    bool build() {
        std::string bin = launch.bin;
        bin_bytes = read_level_bin(ctx.files, bin);
        if (bin_bytes.empty()) {
            std::fprintf(stderr, "nightfire: no such level .bin: %s\n", launch.bin.c_str());
            return false;
        }
        level = std::make_unique<Level>(bin_bytes);  // copy: entries below still need the raw .bin
        if (!level->map()) {
            std::fprintf(stderr, "nightfire: %s has no Map entry\n", launch.bin.c_str());
            return false;
        }
        level_id = level_id_from_name(launch.bin);
        const std::vector<SpawnPoint> spawns = find_spawn_points(*level);
        if (spawns.empty()) {
            std::fprintf(stderr, "nightfire: %s has no player start markers\n", launch.bin.c_str());
            return false;
        }
        std::string tuning_text;
        PlayerParams params;
        if (const GameFile* tuning = ctx.files.find("TuningVars.txt")) {
            const auto raw = ctx.files.read(*tuning);
            tuning_text.assign(reinterpret_cast<const char*>(raw.data()), raw.size());
            params = player_params_from_tuning(tuning_text, "");
        }
        params.health.damage.mode = GameMode::SinglePlayer;

        world = std::make_unique<World>(*level, InputTables::from_elf(ctx.action_elf), params);
        auto start = std::find_if(spawns.begin(), spawns.end(), [&](const SpawnPoint& marker) {
            return launch.entry_channel != 0 && marker.entry_channel == launch.entry_channel;
        });
        world->spawn_player(0, start == spawns.end() ? spawns.front() : *start);
        // The profile's controller style and Y-axis inversion (PlayerSetting+0xE / +0).
        world->settings(0).controller_style = std::clamp(config.controller_style, 0, 7);
        world->settings(0).invert_look = config.invert_y;

        bank = open_character_bank(ctx.files, launch.bin);
        if (const SkinDef* bond = bank->skin(0x05000007))
            world->player(0)->set_rope_animator(make_rope_animator(*bank, *bond));
        auto weapons_owned =
            std::make_unique<WeaponSystem>(WeaponTable::from_elf(ctx.action_elf), params.health.damage);
        weapons_owned->set_autoaim(AutoaimTuning::load(tuning_text, ""));
        weapons_owned->set_bank(bank.get());
        weapons = weapons_owned.get();
        world->add_system(std::move(weapons_owned));

        // NPCs: nav + drones + the single-player layer (mirrors DroneCli::setup --sp).
        drone::DroneConfig dcfg;
        dcfg.elf = &ctx.action_elf;
        dcfg.level_id = level_id;
        dcfg.difficulty = launch.difficulty;
        dcfg.tuning = drone::DroneTuning::load(tuning_text, level_id);
        nav = std::make_unique<NavNetwork>(*level, world->collision(), NavLimits::for_level(level_id));
        auto sys = std::make_unique<drone::DroneSystem>(*world, *bank, dcfg);
        drones = sys.get();
        drones->set_nav(nav->empty() ? nullptr : nav.get());
        drones->set_weapons(weapons);
        weapons->set_drone_system(drones);   // idle-fidget threat gate (any_visible_threat)
        drones->callbacks().player_alive = [this](int slot) { return weapons->alive(slot); };
        sp::SpConfig spcfg;
        spcfg.level_id = level_id;
        spcfg.difficulty = launch.difficulty;
        auto sp = std::make_unique<sp::SpSystem>(*drones, sp::parse_sp_level(*level, level_id),
                                                 sp::SpTables::load(ctx.action_elf), spcfg);
        spsys = sp.get();
        world->add_system(std::move(sp));
        world->add_system(std::move(sys));
        const std::size_t placed = spsys->spawn_placed();

        // Mission flow (Scripting): objectives, movers, cutscenes and success/failure.
        for (const MissionEntry& entry : load_mission_data(ctx.action_elf)) {
            if (entry.level != level_id) continue;
            auto mission = std::make_unique<MissionSystem>(*level, level_id, entry, *weapons, spsys);
            for (const auto& [hash, data] : bank->cutscenes()) {
                try {
                    mission->add_scripts({{hash, parse_cutscene_bin(data)}});
                } catch (const std::exception& ex) {
                    std::fprintf(stderr, "nightfire: %s: cutscene %08x skipped: %s\n", launch.bin.c_str(), hash,
                                 ex.what());
                }
            }
            mission_sys = mission.get();
            mission_sys->set_character_bank(bank.get());
            world->add_system(std::move(mission));
            weapons->attach(*world);
            SpawnLoadout loadout;
            loadout.start_weapon = 0;
            loadout.sleeve = 1;
            weapons->spawn_player(0, loadout);
            // Player_InitWeapon resets weapons and armour for the party disguise.
            // The stock party entry equips fists and keeps the pistol in inventory.
            if (launch.inventory.valid && level_id != 0x07000007)
                mission_sys->ram_load(*world, 0, launch.inventory);
            else
                mission_sys->apply_loadout(0);
            if (launch.entry_channel != 0) mission_sys->set_channel(launch.entry_channel, 1);
            for (const auto& [ch, val] : launch.channels) {
                mission_sys->set_channel(std::uint16_t(ch), std::uint8_t(val));
                std::printf("mission: debug preset channel %d = %d\n", ch, val);
            }
            if (launch.give >= 0)  // --give ID: applied in tick_world once the slot state exists
                std::printf("mission: debug give weapon %d (armed, applies when idle)\n", launch.give);
            break;
        }

        if (level->map()) fx_sprites.add(level->map()->chunk);
        effects = std::make_unique<WeaponEffects>(weapons->table(), *bank, &fx_sprites);
        effects->set_map_lights(bank->lights());
        effects->set_level(level.get());
        for (const BinEntry& e : parse_bin_archive(Bytes(bin_bytes))) {
            if (e.type != EntryType::Script || (e.hash != 0x06000052 && e.hash != 0x060007C4)) continue;
            try {
                effects->set_explosion_script(std::uint32_t(e.hash), e.data);
            } catch (const std::exception& ex) {
                std::fprintf(stderr, "nightfire: %s: effect script %08x skipped: %s\n", launch.bin.c_str(),
                             e.hash, ex.what());
            }
        }

        add_level_sprites(ctx.assets.sprites, Bytes(bin_bytes));
        HudConfig hcfg;
        hcfg.frame_rate = timing.FRAME_RATE;
        hud = std::make_unique<Hud>(ctx.assets, ctx.hud_data, hcfg);

        renderer = std::make_unique<LevelRenderer>(*level);
        renderer->set_level(level_id);
        weather = std::make_unique<WeatherRenderer>(*level);
        weather->set_level(level_id);
        chars = std::make_unique<CharacterRenderer>(*bank);
        weapon_view = std::make_unique<WeaponView>(*bank, *chars);
        drone_renderer = std::make_unique<drone::DroneRenderer>(*bank);

        level_menu = load_menu_from_bin(Bytes(bin_bytes));
        has_level_menu = true;
        // Audio: level bank, volumes from the persisted settings, music script.
        archive = std::make_unique<SoundArchive>(std::filesystem::path(ctx.gamedir));
        audio = std::make_unique<audio::AudioSystem>(*archive);
        audio->enter_level(level_id);
        std::vector<MapSound> map_sounds;
        for (const auto& chunk : level->chunks()) {
            for (const auto& block : chunk.chunk.blocks) {
                if (block.id != std::uint8_t(BlockId::MapSounds)) continue;
                auto sounds = parse_map_sounds(block.data);
                map_sounds.insert(map_sounds.end(), sounds.begin(), sounds.end());
            }
        }
        audio->load_map_sounds(std::move(map_sounds));
        audio->set_sfx_volume(config.sfx_volume);
        audio->set_music_volume(config.music_volume);
        audio->set_subtitle_callback([this](const audio::SubtitleEvent& subtitle) {
            hud->add_message(HudMessage{HudMsgType::Subtitle, subtitle.label, {},
                                         int(std::ceil(subtitle.duration_sec * timing.FRAME_RATE))});
        });
        music = std::make_unique<audio::MusicDirector>(*audio, ctx.music);
        music->start_level(level_id);

        // NPC -> app hooks: text, music, speech/SFX and mission fail.
        spsys->on_text = [this](std::uint32_t label) { hud->add_message(HudMessage{HudMsgType::Info, label, {}, 180}); };
        spsys->on_music_event = [this](int event, int arg) { music->event(std::uint32_t(event), arg); };
        spsys->on_combat_music = [this] { music->event(2, 5); };
        spsys->on_sfx = [this](drone::Drone& drone, int sfx_id) {
            return audio->play_sfx(std::uint32_t(sfx_id), {.position = drone.pos});
        };
        spsys->sfx_finished = [this](int handle) { return !audio->is_playing(handle); };
        drones->callbacks().on_footstep = [this](drone::Drone& drone) { play_footstep(drone.pos); };
        drones->callbacks().on_anim_sound = [this](drone::Drone& drone, int sfx_id) {
            audio->play_sfx(std::uint32_t(sfx_id), {.position = drone.pos});
        };
        spsys->on_mission_fail = [this](drone::Drone&, int, std::uint32_t label) {
            if (label != 0) hud->add_message(HudMessage{HudMsgType::Mission, label, {}, 300});
            if (mission_sys) mission_sys->fail_mission(label);  // drone reason fails the mission, not just the message
        };
        // Scoring tables + per-mission counter reset (results score source).
        try {
            score_tables_ = load_plr_score_tables(ctx.action_elf);
            score_tables_ok_ = true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "nightfire: score tables unavailable: %s\n", e.what());
        }
        plr_.reset_for_mission();
        const Player& p = *world->player(0);
        std::printf("%s: %zu placements, %zu npcs, spawn at %.2f,%.2f,%.2f\n", launch.bin.c_str(),
                    level->placements().size(), placed, p.pos[0], p.pos[1], p.pos[2]);
        return true;
    }

    void update_listener() {
        const Player& p = *world->player(0);
        audio::Listener listener;
        if (auto camera = mission_sys ? mission_sys->script_camera() : std::nullopt) {
            listener.position = camera->eye;
            listener.dir = camera->forward;
            listener.norm = normalised(cross(Vec3{0, 1, 0}, camera->forward));
        } else {
            const float c = std::cos(p.view_pitch());
            listener.position = p.eye();
            listener.dir = {std::sin(p.yaw) * c, std::sin(p.view_pitch()), std::cos(p.yaw) * c};
            listener.norm = {std::cos(p.yaw), 0.0f, -std::sin(p.yaw)};
        }
        audio->set_listener(listener);
    }

    void play_footstep(const Vec3& pos) {
        const Vec3 to = pos - Vec3{0, 2.0f, 0};
        auto surface = world->collision().ray(pos, to, pick::kIgnoreGhost);
        if (const auto* moving = world->objects().moving_meshes()) {
            if (auto hit = moving->ray(pos, to, pick::kIgnoreGhost);
                hit && (!surface || hit->dist < surface->dist)) surface = hit;
        }
        if (!surface || surface->normal[1] <= 0.0f) return;
        const auto id = weapons->table().surface(surface->material & 0x3F).footstep_sound;
        if (id != 0 && id != 0xFFFF) audio->play_sfx(id, {.position = surface->point});
    }

    void audio_frame(const Vec3& previous_pos) {
        update_listener();
        for (const SoundEvent& e : weapons->events().sounds) {
            if (e.positional) {
                if (const auto* def = archive->sfx_defaults(std::uint32_t(e.id)); def && def->alertness > 0.0f)
                    drones->emit_noise(e.position, def->alertness, e.exclude);
            }
            if (e.exclude == 0 || (e.listener >= 0 && e.listener != 0)) continue;
            audio::PlayOptions o;
            if (e.positional) o.position = e.position;
            audio->play_sfx(std::uint32_t(e.id), o);
        }
        weapons->events().sounds.clear();
        for (const SoundEvent& e : effects->take_blast_sounds()) {
            audio::PlayOptions o;
            if (e.positional) o.position = e.position;
            audio->play_sfx(std::uint32_t(e.id), o);
        }
        const Player& p = *world->player(0);
        if (p.enabled() && p.alive() && p.substate == SubState::Walk && p.ground_history != 0 && p.jump_state == 0) {
            const float moved = std::hypot(p.pos[0] - previous_pos[0], p.pos[2] - previous_pos[2]);
            step_distance += std::min(moved, std::hypot(p.velocity[0], p.velocity[2]));
            if (step_distance >= 1.65f) {
                step_distance -= 1.65f;
                play_footstep(p.pos);
            }
        } else {
            step_distance = 0.0f;
        }
        const int updates = static_cast<int>(timing.FRAME_RATE_MUL);
        for (int i = 0; i < updates; ++i) {
            music->update();
            audio->update();
        }
        if (!audio->device_open()) {
            std::array<std::int16_t, 3200> pcm{};
            audio->render(pcm.data(), std::min(pcm.size() / 2,
                std::size_t(audio->output_rate() / timing.FRAME_RATE_INT)));
        }
    }

    // --give ID debug equip: slot state is spawned lazily on the first tick and select
    // only adopts from Idle, so give once the state exists, then select once idle.
    void maybe_give() {
        if (launch.give < 0 || give_done) return;
        if (weapons->state(0) == nullptr) return;
        if (!give_given) {
            if (!weapons->give_weapon(0, launch.give, 999)) return;
            give_given = true;
        }
        if (weapons->anim_state(0) != WeaponAnim::Idle) return;
        weapons->select_weapon(0, launch.give);
        give_done = true;
        std::printf("mission: debug give weapon %d equipped\n", launch.give);
    }

    void tick_world(const PadState& pad) {
        PadInputs pads{};
        pads[0] = pad;
        // Movers publish before the players collide (MissionSystem::pre_tick). Cross is an
        // edge here: the switch use-headers toggle on press, and held-Cross would flap them.
        const bool use = pad.held(kPadCross) && !prev_use_;
        prev_use_ = pad.held(kPadCross);
        update_listener();
        const auto previous_weapon_events = std::exchange(weapons->events(), {});
        if (mission_sys) mission_sys->pre_tick(*world, {use, false, false, false}, timing, &previous_weapon_events);
        if (drone_renderer && drones) drone_renderer->capture_previous(*drones);
        if (weapons) previous_viewmodel = weapons->viewmodel(0);
        const Vec3 previous_pos = world->player(0)->pos;
        world->tick(pads, timing);
        if (mission_sys) {
            const auto collected = mission_sys->objects().take_bond_pickups();
            for (unsigned i = 0; i < collected; ++i) plr_.log_bond();
        }
        if (weather)
            weather->update(world->player(0)->eye(), timing.FRAME_RATE_MUL, timing.REC_FRAME_RATE,
                            [this](int ch) { return spsys->channels.on(ch); });
        feed_stats();
        poll_mission();
        effects->consume(weapons->events());
        effects->tick(timing.FRAME_RATE_MUL, weapons->projectiles());
        audio_frame(previous_pos);
        maybe_give();
        // End of mission: the state machine settles on Done (success shows results, failure the
        // end-mission page); without a mission entry the legacy death/fail signals end it.
        if (mission_sys) {
            if (mission_sys->state() == MissionSystem::State::Succeeded) mission_won = true;
            if (mission_sys->state() == MissionSystem::State::Done) ++death_frames;
        } else if (!world->player(0)->alive() || spsys->mission_fail_reason != 0) {
            ++death_frames;
        }
    }

    // PlrStat counter feed (Log* mirrors): trigger pulls from the muzzle edge, hits and
    // scenery from the local player's impacts, kills from drone deaths, elapsed from frames.
    // Detections, disabled/surrender detail, health and bond ids have no observable source
    // yet and stay 0 (documented in plr_stats.hpp).
    void feed_stats() {
        if (const PlayerWeapons* p = weapons->state(0)) {
            if (p->muzzle_frames > prev_muzzle_) plr_.log_shot_fired();
            prev_muzzle_ = p->muzzle_frames;
        }
        for (const ImpactEvent& e : weapons->events().impacts) {
            if (e.shooter != 0) continue;
            if (e.on_body) plr_.log_shot_hit_enemy();
            else plr_.log_shot_hit_scenery();
        }
        const std::uint64_t deaths = spsys->stats.deaths;
        while (prev_deaths_ < deaths) {
            plr_.log_kill();
            ++prev_deaths_;
        }
        if (mission_sys) plr_.set_elapsed_100ths(std::uint32_t(mission_sys->stats().frames * 100u / 30u));
    }
    CharacterInstance* sample_actor(const CutscenePlayer::ActorPose& pose) {
        auto& actor = script_characters[{pose.skin, pose.script}];
        if (!actor) {
            const SkinDef* skin = bank->skin(pose.skin);
            if (!skin) return nullptr;
            auto model = std::make_unique<CharacterInstance>(*bank, *skin);
            if (!model->play(pose.script, false)) return nullptr;
            model->extract_root_motion(false);
            actor = std::move(model);
        }
        actor->set_frame(pose.frame);
        return actor.get();
    }

    CharacterLighting character_lighting(const Vec3& position) const {
        auto light = effects->lighting_at(position, 3.0f);
        if (const auto ambient = bank->ambient_at(position))
            light.tint = {float((*ambient)[0]) / 255.0f, float((*ambient)[1]) / 255.0f,
                          float((*ambient)[2]) / 255.0f};
        return light;
    }

    void play_mission_sound(const MissionSystem::Sound& s) {
        audio::PlayOptions options;
        if (s.positional) {
            options.position = s.pos;
            if (s.actor.skin != 0) {
                if (auto* actor = sample_actor(s.actor)) {
                    const Mat4 root = actor->bone_world(0);
                    options.position = Vec3{root[12], root[13], root[14]};
                }
            }
        }
        if (s.script_instance == 0) {
            audio->play_sfx(s.id, options);
            return;
        }
        using Action = CutscenePlayer::SoundAction;
        const auto key = std::pair{s.script_instance, s.stream};
        const auto* def = archive->sfx_defaults(s.id);
        const bool looping = def && def->looping;
        if (s.action == Action::End) {
            if (auto it = script_sounds.find(key); it != script_sounds.end()) {
                if (looping) audio->stop_sfx(it->second.handle);
                script_sounds.erase(it);
            }
            return;
        }
        if (s.action == Action::Start) {
            options.tag = next_script_sound_tag++;
            script_sounds[key] = {audio->play_sfx(s.id, options), options.tag};
            return;
        }
        if (auto it = script_sounds.find(key); it != script_sounds.end() && options.position) {
            auto& sound = it->second;
            if (sound.handle != 0) {
                audio->set_position(*options.position, {}, sound.tag, s.id);
            } else if (looping) {
                options.tag = sound.tag;
                sound.handle = audio->play_sfx(s.id, options);
            }
        }
    }

    void poll_mission() {
        if (!mission_sys) return;
        for (const auto& spawn : mission_sys->take_spawns()) spsys->spawn_scripted(spawn.pos, spawn.args);
        for (const MissionSystem::Sound& s : mission_sys->take_sounds()) play_mission_sound(s);
        for (const MissionSystem::Text& t : mission_sys->take_texts()) {
            HudMsgType type = HudMsgType::Info;
            if (t.type == 2) type = HudMsgType::Objective;
            else if (t.type == 3) type = HudMsgType::Mission;
            else if (t.type == 4) type = HudMsgType::Subtitle;
            hud->add_message(HudMessage{type, t.label, {}, t.frames});
        }
        for (const MissionSystem::Music& m : mission_sys->take_music()) music->event(std::uint32_t(m.event), m.value);
        // Coder spawns (Bots): zeroed-DIVars default drones at the event feet (DMODE 0, Idle).
        // Log lines match DroneCli::drain_coder_spawns for cross-tool greppability.
        for (const MissionSystem::Spawn& s : mission_sys->take_spawns()) {
            std::printf("mission spawn: %.1f,%.1f,%.1f\n", s.pos[0], s.pos[1], s.pos[2]);
            if (drone::Drone* d = spsys->spawn_scripted({s.pos[0], s.pos[1], s.pos[2]}, s.args))
                std::printf("sp coder spawn: drone %d\n", d->id);
        }
    }

    void draw_frame(const Camera& cam, float interpolation) {
        int width = 0, height = 0;
        window.begin_frame(width, height);
        // The selected game viewport is full-window Hor+ unless 4:3 pillarboxing is enabled;
        // all 3D (including the viewmodel) uses this aspect.
        const GameView gv = game_view(config, width, height);
        const int gl_y = height - gv.y - gv.h;
        const float aspect = gv.aspect;
        Camera wc = cam;
        wc.fovy = kViewFovY / std::max(1.0f, weapons->zoom(0));
        const Player& player = *world->player(0);
        const auto cutscene = mission_sys ? mission_sys->script_camera() : std::nullopt;
        const bool wall_hug = !cutscene && player.substate == SubState::Creep;
        if (cutscene) {
            const Vec3& f = cutscene->forward;
            wc = camera_for_eye_yaw_pitch(cutscene->eye, std::atan2(f[0], f[2]),
                                          std::asin(std::clamp(f[1], -1.0f, 1.0f)));
            wc.fovy = cutscene->fov;
        } else if (wall_hug) {
            const Vec3 target = player.pos + Vec3{0, 0.25f, 0};
            const Vec3 facing{std::sin(player.yaw), 0, std::cos(player.yaw)};
            Vec3 eye = target + facing * 5.6f + Vec3{facing[2] * 2.2f, -3.25f, -facing[0] * 2.2f};
            if (const auto hit = world->collision().ray(target, eye, pick::kIgnoreGhost))
                eye = target + (hit->point - target) * 0.9f;
            const Vec3 direction = target - eye;
            wc = camera_for_eye_yaw_pitch(eye, std::atan2(direction[0], direction[2]),
                                          std::atan2(direction[1], std::hypot(direction[0], direction[2])));
            wc.fovy = kViewFovY;
        }
        glDisable(GL_SCISSOR_TEST);
        glViewport(0, 0, width, height);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);
        glViewport(gv.x, gl_y, gv.w, gv.h);
        glScissor(gv.x, gl_y, gv.w, gv.h);
        glClearColor(0.25f, 0.3f, 0.4f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (mission_sys) renderer->set_hidden_placements(mission_sys->hides());
        renderer->draw(wc, aspect, false);
        if (weather && weather->active())
            weather->draw(wc, renderer->view_projection(wc, aspect));
        if (drone_renderer && drones)
            drone_renderer->draw(wc, aspect, *drones, interpolation,
                                 [this](const Vec3& root) { return character_lighting(root); });
        auto object_draws = effects->take_blast_draws(interpolation);
        if (mission_sys) {
            for (const auto& draw : mission_sys->draws()) {
                const auto& placement = level->placements()[draw.placement];
                object_draws.push_back({placement.chunk, placement.model, draw.transform});
            }
            for (const auto& pose : mission_sys->script_actors()) {
                auto* actor = sample_actor(pose);
                if (!actor) continue;
                const Mat4 root = actor->bone_world(0);
                const Vec3 position{root[12], root[13], root[14]};
                chars->draw(wc, aspect, actor->skin(), actor->palette(), identity(), 0, actor->facial(),
                            character_lighting(position));
            }
        }
        renderer->draw_objects(wc, aspect, object_draws);
        if (wall_hug) {
            if (const auto* actor = player.rope_character()) {
                const float s = std::sin(player.yaw), c = std::cos(player.yaw);
                const Mat4 transform{c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0,
                                     player.pos[0], player.pos[1], player.pos[2], 1};
                chars->draw(wc, aspect, actor->skin(), actor->palette(), transform, 0, actor->facial());
            }
        }
        effects->draw(wc, aspect, *chars, weapons->projectiles(), interpolation);
        glClear(GL_DEPTH_BUFFER_BIT);
        ViewModel vm = weapons->viewmodel(0);
        if (vm.visible && previous_viewmodel.visible && vm.weapon == previous_viewmodel.weapon) {
            vm.offset = previous_viewmodel.offset + (vm.offset - previous_viewmodel.offset) * interpolation;
            vm.zoom = previous_viewmodel.zoom + (vm.zoom - previous_viewmodel.zoom) * interpolation;
            vm.muzzle_flash =
                previous_viewmodel.muzzle_flash + (vm.muzzle_flash - previous_viewmodel.muzzle_flash) * interpolation;
        }
        if (!cutscene && !wall_hug && vm.visible && vm.skin && vm.anim) {
            const auto& def = weapons->table().weapon(vm.weapon);
            const Vec3 muzzle = weapon_view->draw(wc, aspect, vm, def, effects->lighting_at(wc.eye, 2.0f),
                                                  &world->collision());
            if (vm.muzzle_flash > 0.0f && muzzle != Vec3{}) effects->muzzle_flash(muzzle, def, 0);
        }
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        // HUD overlay (the ui canvas letterboxes over the 3D frame; no clear).
        HudState hs;
        weapons->fill_hud(0, hs);
        hs.context_icon = world->player(0)->icon_context();   // HUD_UpdateHealthPane reads BLData+0x95F
        hs.controller_style = world->settings(0).controller_style;
        if (wall_hug) hs.cam_mode = 1;
        if (hud->enabled(HudPane::Ammo) == wall_hug) hud->enable(HudPane::Ammo, !wall_hug);
        hud->update(hs);
        ui.begin(width, height, false);
        ui::select_prompts(0, InputContext::OnFoot);
        if (cutscene) {
            const float edge = (ui.canvas_width() - ui::kScreenW) * -0.5f;
            constexpr float bar = 64.0f;
            ui.fill({edge, 0, ui.canvas_width(), bar}, {0, 0, 0, 255});
            ui.fill({edge, ui::kScreenH - bar, ui.canvas_width(), bar}, {0, 0, 0, 255});
        } else {
            hud->draw(ui, text);
        }
        // Cutscene fade overlay (MissionSystem::fade 0..1 black).
        if (mission_sys && mission_sys->fade() > 0.001f) {
            const auto a = std::uint8_t(std::clamp(mission_sys->fade(), 0.0f, 1.0f) * 128.0f);
            ui.fill({0, 0, ui::kScreenW, ui::kScreenH}, {0x00, 0x00, 0x00, a});
        }
        ui.end();
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    // Pause / end-mission menu on the shared window. Returns Resume (false) or the session exit.
    std::optional<SpResult> run_menu(std::optional<std::uint32_t> page) {
        if (!has_level_menu) return std::nullopt;
        Frontend menu(ctx.assets, level_menu, &ctx.mp_data, &ctx.sp_data);
        PauseInfo info;
        if (mission_sys) {
            for (const MissionSystem::ObjectiveInfo& o : mission_sys->objectives()) {
                PauseObjective po;
                po.label = o.def.label;
                po.complete = o.state == ObjectiveState::Done || o.state == ObjectiveState::DoneShown;
                info.objectives.push_back(po);
            }
        }
        menu.set_pause_info(info);
        // The CONTROLS tab edits the player's controller style and Y-axis inversion: write them back however the
        // menu closes, into the game and the saved settings.
        menu.player_options().controller_style = world->settings(0).controller_style;
        menu.player_options().invert_y = world->settings(0).invert_look;
        menu.open(FrontendMode::Pause, page);
        struct KeepOptions {
            Impl& s;
            Frontend& menu;
            ~KeepOptions() {
                const PlayerOptions& o = menu.player_options();
                PlayerSettings& ps = s.world->settings(0);
                if (ps.controller_style == o.controller_style && ps.invert_look == o.invert_y) return;
                ps.controller_style = std::clamp(o.controller_style, 0, 7);
                ps.invert_look = o.invert_y;
                s.user_config.controller_style = s.config.controller_style = ps.controller_style;
                s.user_config.invert_y = s.config.invert_y = ps.invert_look;
                save_config(config_path(), s.user_config);
            }
        } keep{*this, menu};
        if (!page) {
            const Player& player = *world->player(0);
            draw_frame(camera_for_eye_yaw_pitch(player.shaken_eye(), player.yaw, player.view_pitch()), 1.0f);
            int w, h;
            window.begin_frame(w, h);
            const GameView view = game_view(config, w, h);
            ui.capture_backdrop(view.x, h - view.y - view.h, view.w, view.h);
        }
        PadHistory hist;
        bool wait_release = true;
        while (!menu.wants_close()) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) return SpResult{SpExit::QuitToMenu};
                if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE && !page) return std::nullopt;  // Esc resumes pause
            }
            const PadState s = menu_pad(gamepad);
            if (wait_release) {
                if (s.buttons == 0) wait_release = false;
                hist.push({});
            } else {
                hist.push(s);
            }
            menu.update(hist);
            // C_NIS accept: play the requested level cutscene and resume into it.
            if (mission_sys) {
                if (const std::uint32_t nis = menu.take_nis_request()) {
                    mission_sys->play_nis(nis);
                    return std::nullopt;
                }
            }
            int w, h;
            window.begin_frame(w, h);
            ui.begin(w, h);
            if (!page) ui.draw_backdrop();
            ui::select_prompts(0, InputContext::Menu);
            menu.draw(ui, text);
            ui.end();
            window.swap();
            SDL_Delay(33);
        }
        const FrontendResult& r = menu.result();
        if (r.action == FrontendResult::Action::Resume) return std::nullopt;
        if (r.action == FrontendResult::Action::RestartMission) return SpResult{SpExit::Restart};
        return SpResult{SpExit::QuitToMenu};
    }

    // sp_level id after this mission (pending destination, else the next row in sp_level order, else 0).
    std::uint32_t next_level_id() const {
        if (mission_sys && mission_sys->pending_level() != 0) return mission_sys->pending_level();
        const std::vector<SpLevelRow> rows = load_sp_level_order(ctx.action_elf);
        for (std::size_t i = 0; i < rows.size(); ++i) {
            if (rows[i].level != level_id) continue;
            if (i + 1 < rows.size()) return rows[i + 1].level;
            return 0;  // last mission: the bonus chain leads to wingame, then the frontend
        }
        return 0;
    }

    std::optional<SpResult> continuation() const {
        if (!mission_sys || mission_sys->state() == MissionSystem::State::Failed ||
            !world->player(0)->alive() || mission_sys->pending_level() == 0) return std::nullopt;
        const auto next = mission_sys->pending_level();
        const auto rows = load_mission_data(ctx.action_elf);
        const auto current = std::find_if(rows.begin(), rows.end(), [&](const MissionEntry& e) { return e.level == level_id; });
        const auto target = std::find_if(rows.begin(), rows.end(), [&](const MissionEntry& e) { return e.level == next; });
        if (current == rows.end() || target == rows.end() || current->base != target->base || next == level_id)
            return std::nullopt;
        return SpResult{SpExit::NextMission, false, false, next, mission_sys->carried_inventory(),
                        mission_sys->exit_channel()};
    }

    // Results content for P_NFRESULTS (stats + next-mission name).
    std::pair<MissionResults, std::uint32_t> build_results() {
        MissionResults results;
        if (mission_sys) {
            const MissionSystem::Stats& st = mission_sys->stats();
            const long secs = long(st.frames) / 30;
            char time[32];
            std::snprintf(time, sizeof(time), "%ld:%02ld", secs / 60, secs % 60);
            // Difficulty arrives as 1/2/3 (GameState+0x28) or 0/1/2 (menu/CLI index); normalize.
            const int d = launch.difficulty;
            const int difficulty = (d >= 1 && d <= 3) ? d : (d >= 0 && d <= 2) ? d + 1 : 2;
            PlrStats::Score score{};
            if (score_tables_ok_) score = plr_.compute(score_tables_, level_id, difficulty, mission_won);
            if (score.valid) results.score_text = separate_number(score.total);
            results.stats = {{"Time", time},
                             {"Shots", std::to_string(st.shots)},
                             {"Hits", std::to_string(st.hits)},
                             {"Kills", std::to_string(mission_sys->kills())}};
            if (score.valid && score.done_better) results.stats.emplace_back("Best", "New!");
        }
        const std::uint32_t next = next_level_id();
        if (next != 0) {
            for (const SpLevelRow& row : load_sp_level_order(ctx.action_elf)) {
                if (row.level != next) continue;
                const std::string_view name = ctx.assets.strings.label(row.name);
                if (!name.empty()) results.next_target_text = std::string(name);
                break;
            }
        }
        return {results, next};
    }

    // Headless results shot (the interactive run_results drove the same page with live input).
    void draw_results_shot(const std::string& path) {
        const auto [results, next] = build_results();
        (void)next;
        Frontend menu(ctx.assets, ctx.menu, &ctx.mp_data, &ctx.sp_data);
        menu.set_mission_results(results);
        menu.open(FrontendMode::MainMenu, kPageResults);
        PadHistory hist;
        for (int i = 0; i < 30; ++i) {
            hist.push({});
            menu.update(hist);
        }
        int w, h;
        window.begin_frame(w, h);
        ui.begin(w, h);
        ui::select_prompts(0, InputContext::Menu);
        menu.draw(ui, text);
        ui.end();
        if (!window.save_bmp(path)) throw std::runtime_error("cannot write shot");
        std::printf("results -> %s\n", path.c_str());
    }

    // Mission-complete results (P_NFRESULTS) on the frontend script. MissionDone continues to next_level_id().
    std::optional<SpResult> run_results() {
        Frontend menu(ctx.assets, ctx.menu, &ctx.mp_data, &ctx.sp_data);
        const auto [results, next] = build_results();
        menu.set_mission_results(results);
        menu.open(FrontendMode::MainMenu, kPageResults);
        PadHistory hist;
        bool wait_release = true;
        while (!menu.wants_close()) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) return SpResult{SpExit::QuitToMenu};
            }
            const PadState s = menu_pad(gamepad);
            if (wait_release) {
                if (s.buttons == 0) wait_release = false;
                hist.push({});
            } else {
                hist.push(s);
            }
            menu.update(hist);
            // Results chain movies (wingame): play fullscreen, then run the post-movie transition.
            if (std::uint32_t movie = menu.take_movie_request()) {
                MovieScreen screen(window, ui, text, audio.get(), gamepad, ctx.gamedir);
                screen.play(movie);
                menu.movie_finished();
            }
            int w, h;
            window.begin_frame(w, h);
            ui.begin(w, h);
            ui::select_prompts(0, InputContext::Menu);
            menu.draw(ui, text);
            ui.end();
            window.swap();
            SDL_Delay(33);
        }
        if (menu.result().action == FrontendResult::Action::MissionDone)
            return SpResult{SpExit::NextMission, false, true, next};
        return SpResult{SpExit::QuitToMenu, false, true, 0};
    }
};

SpSession::SpSession(AppContext& ctx, Window& window, ui::Renderer& ui, ui::TextRenderer& text, const SpLaunch& launch,
                     AppConfig& cfg)
    : impl_(std::make_unique<Impl>(ctx, window, ui, text, launch, cfg)) {
    ready_ = impl_->build();
}

SpSession::~SpSession() = default;

SpResult SpSession::run_interactive() {
    Impl& s = *impl_;
    int count = 0;
    if (SDL_JoystickID* ids = SDL_GetGamepads(&count)) {
        if (count > 0) {
            s.gamepad = SDL_OpenGamepad(ids[0]);
            s.pad = LocalPad(true, s.gamepad);
        }
        SDL_free(ids);
    }
    const bool has_audio = s.audio->open_device();
    std::printf("audio: output device %s\n", has_audio ? "open" : "none");

    const Player& initial_player = *s.world->player(0);
    Vec3 previous_eye = initial_player.shaken_eye();
    float previous_yaw = initial_player.yaw;
    float previous_pitch = initial_player.view_pitch();
    bool running = true, captured = false;
    double accumulator = 0;
    Uint64 last = SDL_GetTicksNS();
    const double kStep = s.timing.REC_FRAME_RATE;
    SpResult done{SpExit::QuitToMenu};
    bool finished = false;
    while (running && !finished) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            s.pad.handle_event(e);
            if (e.type == SDL_EVENT_QUIT) running = false;
            else if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && !captured) {
                captured = SDL_SetWindowRelativeMouseMode(s.window.sdl(), true);
                s.pad.set_captured(captured);
            } else if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE) {
                if (captured) {
                    captured = !SDL_SetWindowRelativeMouseMode(s.window.sdl(), false);
                    s.pad.set_captured(captured);
                }
                if (auto r = s.run_menu(std::nullopt)) {
                    done = *r;
                    finished = true;
                }
                const Player& resumed_player = *s.world->player(0);
                previous_eye = resumed_player.shaken_eye();
                previous_yaw = resumed_player.yaw;
                previous_pitch = resumed_player.view_pitch();
                last = SDL_GetTicksNS();
            }
        }
        const Uint64 now = SDL_GetTicksNS();
        accumulator = std::min(accumulator + double(now - last) * 1e-9, 0.25);
        last = now;
        while (accumulator >= kStep) {
            PadState pad = s.pad.sample();
            if (pad.buttons & kPadStart) {
                if (auto r = s.run_menu(std::nullopt)) {
                    done = *r;
                    finished = true;
                } else {
                    last = SDL_GetTicksNS();
                    accumulator = 0;
                    const Player& resumed_player = *s.world->player(0);
                    previous_eye = resumed_player.shaken_eye();
                    previous_yaw = resumed_player.yaw;
                    previous_pitch = resumed_player.view_pitch();
                }
                break;
            }
            const Player& previous_player = *s.world->player(0);
            previous_eye = previous_player.shaken_eye();
            previous_yaw = previous_player.yaw;
            previous_pitch = previous_player.view_pitch();
            s.tick_world(pad);
            accumulator -= kStep;
            if (auto next = s.continuation()) {
                done = std::move(*next);
                finished = true;
                break;
            }
            if (s.death_frames > 3 * s.timing.FRAME_RATE_INT && !s.end_shown) {
                s.end_shown = true;
                if (s.mission_won) {
                    // Mission complete -> results chain, then the next mission.
                    if (auto r = s.run_results()) {
                        done = *r;
                        finished = true;
                    } else {
                        done = SpResult{SpExit::QuitToMenu, false, true, 0};
                        finished = true;
                    }
                } else if (auto r = s.run_menu(kPageEndMission)) {
                    done = *r;
                    done.failed = true;
                    finished = true;
                } else {
                    done = SpResult{SpExit::QuitToMenu, true};
                    finished = true;
                }
                break;
            }
        }
        if (finished) break;
        const Player& p = *s.world->player(0);
        s.draw_frame(interpolate_camera(previous_eye, previous_yaw, previous_pitch, p,
                                        float(accumulator / kStep)),
                     float(accumulator / kStep));
        s.window.swap();
    }
    if (s.gamepad) SDL_CloseGamepad(s.gamepad);
    return done;
}

SpResult SpSession::run_headless(const SpHeadless& headless) {
    Impl& s = *impl_;
    // Optional nfgame-format replay (start line + `frame word rx ry lx ly` lines).
    struct Frame {
        PadState pad;
    };
    std::vector<Frame> script;
    Vec3 start_pos{};
    bool has_start = false;
    float start_yaw = 0, start_pitch = 0;
    if (!headless.inputs.empty()) {
        std::ifstream in(headless.inputs);
        if (!in) throw std::runtime_error("cannot open inputs file " + headless.inputs);
        std::string line;
        while (std::getline(in, line)) {
            if (const auto hash = line.find('#'); hash != std::string::npos) line.resize(hash);
            std::istringstream ls(line);
            std::string head;
            if (!(ls >> head)) continue;
            if (head == "start") {
                if (!(ls >> start_pos[0] >> start_pos[1] >> start_pos[2] >> start_yaw)) throw std::runtime_error("bad start line");
                ls >> start_pitch;
                has_start = true;
                continue;
            }
            Frame f;
            unsigned word, rx, ry, lx, ly;
            const long n = std::strtol(head.c_str(), nullptr, 10);
            if (!(ls >> std::hex >> word >> std::dec >> rx >> ry >> lx >> ly)) throw std::runtime_error("bad input line");
            f.pad.buttons = buttons_from_sony_pad_word(std::uint16_t(word));
            f.pad.rx = std::uint8_t(rx), f.pad.ry = std::uint8_t(ry), f.pad.lx = std::uint8_t(lx),
            f.pad.ly = std::uint8_t(ly);
            if (n >= long(script.size())) script.resize(std::size_t(n) + 1);
            script[std::size_t(n)] = f;
        }
    }
    if (has_start) s.world->player(0)->place_at_rest(start_pos, start_yaw, start_pitch, 1.0f);
    long ran = 0;
    for (long i = 0; i < headless.frames; ++i) {
        // Headless neutral: raw 0x80 sticks read as +1/128 strafe/turn (axis()), so a still
        // mission would drift. Live pads pass compensate_sticks (0x80 -> 0x7F -> exact 0);
        // script pads from --inputs already contain compensated bytes (see actions.hpp).
        const PadState pad =
            i < long(script.size()) ? script[std::size_t(i)].pad : compensate_sticks(PadState{});
        s.tick_world(pad);
        ran = i + 1;
        if (auto next = s.continuation()) {
            std::printf("%s -> %08x (entry channel %u)\n", s.launch.bin.c_str(), next->next_level, next->entry_channel);
            return *next;
        }
        if (s.mission_sys && s.mission_sys->state() == MissionSystem::State::Done) break;
    }
    const Player& p = *s.world->player(0);
    const char* mstate = s.mission_sys
                             ? (s.mission_sys->state() == MissionSystem::State::Done
                                    ? (s.mission_won ? "mission-done-won" : "mission-done")
                                    : "mission-playing")
                             : "no-mission";
    std::printf("%s: %ld frames, pos %.2f,%.2f,%.2f yaw %.3f alive %d npcs %zu %s\n", s.launch.bin.c_str(), ran,
                p.pos[0], p.pos[1], p.pos[2], p.yaw, int(p.alive()), s.spsys->spawned(), mstate);
    if (!headless.shot.empty()) {
        s.draw_frame(camera_for_eye_yaw_pitch(p.shaken_eye(), p.yaw, p.view_pitch()), 1.0f);
        const bool ok = s.window.save_bmp(headless.shot);
        std::printf("shot -> %s\n", ok ? headless.shot.c_str() : SDL_GetError());
        if (!ok) throw std::runtime_error("cannot write shot");
        if (s.mission_won) {
            std::string results = headless.shot;
            if (const auto dot = results.find_last_of('.'); dot != std::string::npos)
                results = results.substr(0, dot) + "_results" + results.substr(dot);
            else
                results += "_results.bmp";
            s.draw_results_shot(results);
        }
    }
    const bool failed = !p.alive() || s.spsys->mission_fail_reason != 0 ||
                        (s.mission_sys && s.mission_sys->state() == MissionSystem::State::Done && !s.mission_won);
    return SpResult{SpExit::QuitToMenu, failed, s.mission_won, 0};
}

}  // namespace nf::app
