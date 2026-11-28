// Single-player ACTION session: build, tick, draw, pause and end-mission.
#include "app/session_sp.hpp"

#include "app/menu_music.hpp"
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
#include "assets/credit_data.hpp"
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
#include "ui/menu_audio.hpp"

namespace nf::app {

namespace {

constexpr float kTau = 6.2831853071795864769f;

// Mission_Update sends these final maps to a pre-rendered outro before P_NFRESULTS.
std::uint32_t mission_outro(std::uint32_t level) {
    switch (level) {
    case 0x07000004: case 0x07000008: case 0x0700000b: case 0x0700000d:
    case 0x0700000f: case 0x07000013: case 0x0700001b:
        return level + 0x00300000;
    case 0x07000016:
        return 0x07300017;
    default:
        return 0;
    }
}

struct Checkpoint {
    std::string_view key, name, bin;
    std::optional<Vec3> position;
    float yaw;
    int give;
    std::vector<std::pair<int, int>> channels;
};

const Checkpoint kExchangeCheckpoints[] = {
    {"truck", "Truck approach", "07000005.bin", Vec3{54, 29.3f, 93.7f}, -1.57f, -1, {}},
    {"library", "Library after Drake's speech", "07000007.bin", std::nullopt, 0, -1,
        {{71, 1}, {72, 1}, {73, 1}, {21, 1}, {39, 1}, {169, 1}}},
    {"safe", "Safe-room approach", "07000008.bin", Vec3{153.6f, 43.033f, -120.72f}, 1.57f, 78, {{137, 1}}},
    {"gondola", "Gondola departure", "07000008.bin", Vec3{112, 27.8f, -125.8f}, 3.14f, 44, {{74, 1}}},
};

const Checkpoint* find_checkpoint(std::string_view key) {
    for (const auto& checkpoint : kExchangeCheckpoints)
        if (checkpoint.key == key) return &checkpoint;
    return nullptr;
}

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

std::optional<SpLaunch> checkpoint_launch(std::string_view key, int difficulty) {
    const auto* checkpoint = find_checkpoint(key);
    if (!checkpoint) return std::nullopt;
    SpLaunch launch{std::string(checkpoint->bin), difficulty, checkpoint->channels, checkpoint->give};
    launch.checkpoint = key;
    return launch;
}

struct SpSession::Impl {
    Impl(AppContext& c, Window& w, ui::Renderer& u, ui::TextRenderer& t, const SpLaunch& l, AppConfig& cfg,
         audio::AudioSystem* menus)
        : ctx(c), window(w), ui(u), text(t), launch(l), config(cfg), timing(float(cfg.logic_hz)), user_config(cfg),
          menu_audio(menus) {}

    AppContext& ctx;
    Window& window;
    ui::Renderer& ui;
    ui::TextRenderer& text;
    SpLaunch launch;
    AppConfig config;
    FrameTiming timing;
    AppConfig& user_config;   // controller style / Y-inversion edited in the pause menu persist here
    audio::AudioSystem* menu_audio;
    const CutscenePlayer* skip_prompt_scene = nullptr;
    float skip_prompt_time = 0;
    bool previous_action = false, swallow_action = false;

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
    std::uint32_t mission_base = 0;
    std::uint64_t play_frames = 0;
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
    audio::SfxHandle watch_laser_sound = 0;
    std::int32_t next_script_sound_tag = 1;
    float step_distance = 0.0f;
    SDL_Gamepad* gamepad = nullptr;
    LocalPad pad{true, nullptr};

    int death_frames = 0;  // ticks since the end began (Done, death or fail signal)
    bool end_shown = false;
    bool mission_won = false;  // the state machine reported Succeeded before Done
    PlrStats plr_;              // mission counters feeding the results score
    bool bond_pickup_popup = false;
    PlrScoreTables score_tables_;
    bool score_tables_ok_ = false;
    bool give_given = false, give_done = false;  // --give ID debug equip progress
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
            mission_base = entry.base;
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
            if (const auto* checkpoint = find_checkpoint(launch.checkpoint)) {
                if (checkpoint->position)
                    world->player(0)->place_at_rest(*checkpoint->position, checkpoint->yaw, 0, 1);
                if (checkpoint->key == "library") {
                    mission_sys->finish_checkpoint_scene(*world, 0x06000073, timing);
                    mission_sys->finish_checkpoint_scene(*world, 0x06000072, timing);
                }
                std::printf("mission checkpoint: %s at %.2f,%.2f,%.2f\n", launch.checkpoint.c_str(),
                            world->player(0)->pos[0], world->player(0)->pos[1], world->player(0)->pos[2]);
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
            // Snd2Lbl's high bit forces display. It is not part of the string-table key.
            hud->add_message(HudMessage{HudMsgType::Subtitle, subtitle.label & 0x7FFFFFFFu, {},
                                         int(std::ceil(subtitle.duration_sec * FrameTiming::kReferenceHz))});
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
        // Scoring tables + per-mission counter reset (results score source).
        try {
            score_tables_ = load_plr_score_tables(ctx.action_elf);
            score_tables_ok_ = true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "nightfire: score tables unavailable: %s\n", e.what());
        }
        plr_ = launch.stats;
        play_frames = launch.play_frames;
        plr_.carry_bond_count(launch.bond_count);
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
        constexpr std::uint32_t watch_laser_sfx = 18; // SFX_GADGET_WATCH_LASER_SHOOT
        const bool cutting = !weapons->events().beams.empty();
        if (cutting && !audio->is_playing(watch_laser_sound))
            watch_laser_sound = audio->play_sfx(watch_laser_sfx);
        else if (!cutting && watch_laser_sound != 0)
            audio->stop_sfx(std::exchange(watch_laser_sound, 0));
        for (const SoundEvent& e : weapons->events().sounds) {
            if (e.id == int(watch_laser_sfx)) continue; // owned by beam activity, not its animation cue
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
            if (!weapons->owns(0, launch.give) && !weapons->give_weapon(0, launch.give, 999)) return;
            give_given = true;
        }
        if (weapons->anim_state(0) != WeaponAnim::Idle) return;
        weapons->select_weapon(0, launch.give);
        give_done = true;
        std::printf("mission: debug give weapon %d equipped\n", launch.give);
    }

    std::uint16_t action_button() {
        const auto button = ui::action_button(kActUse, world->settings(0).controller_style);
        return button != 0 ? button : std::uint16_t(kPadCross);
    }

    bool tick_world(PadState pad) {
        if (mission_won) pad = compensate_sticks(PadState{});
        bool skipped = false;
        const auto* scene = mission_sys ? mission_sys->skippable_cutscene() : nullptr;
        const auto button = action_button();
        const bool action = (pad.buttons & button) != 0;
        if (!action) swallow_action = false;
        if (skip_prompt_scene != scene || skip_prompt_time <= 0) {
            skip_prompt_scene = nullptr;
            skip_prompt_time = 0;
        }
        if (scene && action && !previous_action) {
            swallow_action = true;
            if (skip_prompt_scene == scene) {
                mission_sys->skip_cutscene(*world, timing);
                skipped = true;
                skip_prompt_scene = nullptr;
                skip_prompt_time = 0;
            } else {
                skip_prompt_scene = scene;
                skip_prompt_time = 180;
            }
        }
        previous_action = action;
        if (scene || swallow_action) pad.buttons &= ~button;
        if (skip_prompt_scene) skip_prompt_time -= timing.mul();
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
            bond_pickup_popup |= collected != 0;
        }
        if (weather)
            weather->update(world->player(0)->eye(), timing.FRAME_RATE_MUL, timing.REC_FRAME_RATE,
                            [this](int ch) { return spsys->channels.on(ch); });
        if (!mission_won) {
            ++play_frames;
            feed_stats();
        }
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
        update_hud();
        return skipped;
    }

    void update_hud() {
        HudState hs;
        weapons->fill_hud(0, hs);
        const auto& player = *world->player(0);
        hs.context_icon = player.icon_context();
        hs.controller_style = world->settings(0).controller_style;
        hs.bond_moment = std::exchange(bond_pickup_popup, false);
        if (const auto* missile = player.alive() && player.enabled() ? weapons->guided_projectile(0) : nullptr) {
            hs.redeemer_flying = true;
            hs.redeemer_charge = missile->travelled / weapons->table().weapon(missile->weapon).range;
            hs.crosshair_enabled = false;
        }
        hud->enable(HudPane::Redeemer, hs.redeemer_flying);
        const bool wall_hug = !(mission_sys && mission_sys->script_camera()) && player.substate == SubState::Creep;
        if (wall_hug) hs.cam_mode = 1;
        if (hud->enabled(HudPane::Ammo) == wall_hug) hud->enable(HudPane::Ammo, !wall_hug);
        hud->update(hs);
    }

    // PlrStat counter feed (Log* mirrors): trigger pulls from the muzzle edge, hits and
    // scenery from the local player's impacts, kills from drone deaths, elapsed from frames.
    // Detections, disabled/surrender detail, health and bond ids have no observable source
    // yet and stay 0 (documented in plr_stats.hpp).
    void feed_stats() {
        for (int slot : weapons->events().player_shots)
            if (slot == 0) plr_.log_shot_fired();
        for (const ImpactEvent& e : weapons->events().impacts) {
            if (e.shooter != 0) continue;
            const auto& weapon = weapons->table().weapon(e.weapon);
            if ((weapon.flags2 & wf2::kLaserBeam) != 0 || (weapon.flags3 & wf3::kGrapple) != 0 ||
                weapon.base == 1) continue;
            if (e.on_body || (mission_sys && mission_sys->hit_copter_hull(e))) plr_.log_shot_hit_enemy();
        }
        const std::uint64_t deaths = spsys->stats.deaths;
        while (prev_deaths_ < deaths) {
            plr_.log_kill();
            ++prev_deaths_;
        }
        plr_.set_elapsed_100ths(std::uint32_t(play_frames * 100u / unsigned(timing.FRAME_RATE_INT)));
    }
    CharacterInstance* sample_actor(const CutscenePlayer::ActorPose& pose) {
        auto& actor = script_characters[{pose.skin, pose.script}];
        if (!actor) {
            const SkinDef* skin = bank->skin(pose.skin);
            if (!skin) return nullptr;
            auto model = std::make_unique<CharacterInstance>(*bank, *skin);
            if (!model->play(pose.script, false)) return nullptr;
            actor = std::move(model);
        }
        actor->extract_root_motion(pose.transform != identity());
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
                    options.position = transform_point(s.actor.transform, {root[12], root[13], root[14]});
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
        if (auto smoke = mission_sys->take_copter_smoke()) effects->emit_aircraft_smoke(*smoke);
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
        const auto* missile = !cutscene && player.alive() && player.enabled() ? weapons->guided_projectile(0) : nullptr;
        const bool wall_hug = !cutscene && player.substate == SubState::Creep;
        if (cutscene) {
            const Vec3& f = cutscene->forward;
            wc = camera_for_eye_yaw_pitch(cutscene->eye, std::atan2(f[0], f[2]),
                                          std::asin(std::clamp(f[1], -1.0f, 1.0f)));
            wc.fovy = cutscene->fov;
        } else if (missile) {
            const Vec3& f = missile->dir;
            const auto eye = missile->previous_pos + (missile->pos - missile->previous_pos) * interpolation;
            wc = camera_for_eye_yaw_pitch(eye, std::atan2(f[0], f[2]), std::asin(std::clamp(f[1], -1.0f, 1.0f)));
            wc.fovy = kViewFovY;
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
        if (mission_sys) {
            const int room = (cutscene || wall_hug || missile) ? RoomMap::kNone : player.water.room;
            auto hidden = world->rooms().hidden_geometry(world->collision(), wc.eye,
                                                         renderer->view_projection(wc, aspect), room);
            const auto& objects = mission_sys->hides();
            hidden.insert(hidden.end(), objects.begin(), objects.end());
            renderer->set_hidden_placements(std::move(hidden));
        }
        renderer->draw(wc, aspect, false);
        if (weather && weather->active())
            weather->draw(wc, renderer->view_projection(wc, aspect));
        if (drone_renderer && drones)
            drone_renderer->draw(wc, aspect, *drones, interpolation,
                                 [this](const Vec3& root) { return character_lighting(root); });
        auto object_draws = effects->take_blast_draws(interpolation);
        const auto rockets = effects->projectile_draws(weapons->projectiles(), interpolation, missile);
        object_draws.insert(object_draws.end(), rockets.begin(), rockets.end());
        if (mission_sys) {
            for (const auto& draw : mission_sys->draws()) {
                const auto& placement = level->placements()[draw.placement];
                object_draws.push_back({placement.chunk, draw.model == SIZE_MAX ? placement.model : draw.model,
                                        draw.transform, draw.tint});
            }
            for (const auto& model : mission_sys->script_models())
                object_draws.push_back({model.chunk, model.model, model.transform});
            for (const auto& pose : mission_sys->script_actors()) {
                auto* actor = sample_actor(pose);
                if (!actor) continue;
                const Mat4 root = actor->bone_world(0);
                const Vec3 position = transform_point(pose.transform, {root[12], root[13], root[14]});
                chars->draw(wc, aspect, actor->skin(), actor->palette(), pose.transform, 0, actor->facial(),
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
        if (!cutscene && !missile && !wall_hug && vm.visible && vm.skin && vm.anim) {
            const auto& def = weapons->table().weapon(vm.weapon);
            const Vec3 muzzle = weapon_view->draw(wc, aspect, vm, def, effects->lighting_at(wc.eye, 2.0f),
                                                  &world->collision());
            if (vm.muzzle_flash > 0.0f && muzzle != Vec3{}) effects->muzzle_flash(muzzle, def, 0);
        }
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        // HUD overlay (the ui canvas letterboxes over the 3D frame; no clear).
        ui.begin(width, height, false);
        ui::select_prompts(0, InputContext::OnFoot);
        if (cutscene) {
            const float edge = (ui.canvas_width() - ui::kScreenW) * -0.5f;
            constexpr float bar = 64.0f;
            ui.fill({edge, 0, ui.canvas_width(), bar}, {0, 0, 0, 255});
            ui.fill({edge, ui::kScreenH - bar, ui.canvas_width(), bar}, {0, 0, 0, 255});
            ui.set_blend(ui::Blend::Additive); // Script_SpriteStart sets GS blend flag 0x100
            for (const auto& sprite : mission_sys->script_sprites()) {
                const auto [width, height] = ui.texture_size(sprite.hash);
                if (width == 0 || height == 0) continue;
                const bool flip_x = (sprite.flags & 2) != 0;
                const bool flip_y = (sprite.flags & 4) != 0;
                constexpr float y_scale = ui::kScreenH / 480.0f;
                ui.draw(sprite.hash, {float(sprite.x), sprite.y * y_scale, float(sprite.w), sprite.h * y_scale},
                        {flip_x ? float(width) : 0.0f, flip_y ? float(height) : 0.0f,
                         float(width) * (flip_x ? -1 : 1), float(height) * (flip_y ? -1 : 1)},
                        {0x80, 0x80, 0x80, 0x80});
            }
            ui.set_blend(ui::Blend::Alpha);
            hud->draw_subtitle(ui, text);
        } else {
            hud->draw(ui, text);
        }
        // Cutscene fade overlay (MissionSystem::fade 0..1 black).
        if (mission_sys && mission_sys->fade() > 0.001f) {
            const auto a = std::uint8_t(std::clamp(mission_sys->fade(), 0.0f, 1.0f) * 128.0f);
            ui.fill({0, 0, ui::kScreenW, ui::kScreenH}, {0x00, 0x00, 0x00, a});
        }
        if (skip_prompt_time > 0 && skip_prompt_scene && mission_sys->skippable_cutscene() == skip_prompt_scene) {
            const char glyph = ui::prompt_key(action_button());
            const std::string action = glyph ? std::string("~") + glyph : "Action";
            text.draw((ui.canvas_width() + ui::kScreenW) * .5f - 20, ui::kScreenH - 24,
                      "Press " + action + " again to skip",
                      ui::TextStyle{.font = 2, .drop_shadow = true, .align = ui::Align::Right});
        }
        ui.end();
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    // Pause / end-mission menu on the shared window. Returns Resume (false) or the session exit.
    std::optional<SpResult> run_menu(std::optional<std::uint32_t> page) {
        if (!has_level_menu) return std::nullopt;
        if (watch_laser_sound != 0) audio->stop_sfx(std::exchange(watch_laser_sound, 0));
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
        bool checkpoints_open = false;
        int checkpoint_row = 0;
        const bool checkpoints_available = !page && level_id >= 0x07000005 && level_id <= 0x07000008;
        while (!menu.wants_close()) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) return SpResult{SpExit::QuitToMenu};
                if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat && !page) {
                    if (e.key.key == SDLK_ESCAPE) {
                        if (!checkpoints_open) return std::nullopt;
                        checkpoints_open = false;
                        wait_release = true;
                    } else if (e.key.key == SDLK_F6 && checkpoints_available) {
                        checkpoints_open = !checkpoints_open;
                        wait_release = true;
                    }
                }
            }
            const PadState s = menu_pad(gamepad);
            if (wait_release) {
                if (s.buttons == 0) wait_release = false;
                hist.push({});
            } else {
                hist.push(s);
            }
            if (checkpoints_available && hist.pressed(kPadSelect)) checkpoints_open = !checkpoints_open;
            else if (checkpoints_open) {
                const int count = int(std::size(kExchangeCheckpoints));
                if (hist.pressed(kPadUp)) checkpoint_row = (checkpoint_row + count - 1) % count;
                if (hist.pressed(kPadDown)) checkpoint_row = (checkpoint_row + 1) % count;
                if (hist.pressed(kPadCircle)) checkpoints_open = false;
                if (hist.pressed(kPadCross)) {
                    SpResult result{SpExit::Restart};
                    result.checkpoint = kExchangeCheckpoints[checkpoint_row].key;
                    return result;
                }
            } else menu.update(hist);
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
            if (checkpoints_open) {
                const float x = (ui::kScreenW - 364) * .5f;
                ui.fill({x, 88, 364, 257}, {14, 24, 31, 255});
                text.draw(x + 24, 126, "Debug Checkpoints", ui::TextStyle{.font = 3});
                for (int i = 0; i < int(std::size(kExchangeCheckpoints)); ++i) {
                    const float y = 169.0f + i * 32.0f;
                    if (i == checkpoint_row) ui.fill({x + 15, y - 19, 334, 28}, {42, 72, 86, 255});
                    text.draw(x + 24, y, kExchangeCheckpoints[i].name, ui::TextStyle{.font = 2});
                }
                text.draw(x + 24, 320, "~A Launch     ~X Back", ui::TextStyle{.font = 2});
            } else if (checkpoints_available) {
                const ui::TextStyle style{.font = 2, .align = ui::Align::Center};
                constexpr std::string_view label = "F6 / ~T Debug Checkpoints";
                const float width = text.measure(label, style).width + 20;
                const float centre = ui::kScreenW * .5f;
                ui.fill({centre - width * .5f, ui::kScreenH - 39, width, 26}, {14, 24, 31, 255});
                text.draw(centre, ui::kScreenH - 20, label, style);
            }
            ui.end();
            window.swap();
            SDL_Delay(33);
        }
        const FrontendResult& r = menu.result();
        if (r.action == FrontendResult::Action::Resume) return std::nullopt;
        if (r.action == FrontendResult::Action::RestartMission) return SpResult{SpExit::Restart};
        return SpResult{SpExit::QuitToMenu};
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
        SpResult result{SpExit::NextMission, false, false, next, mission_sys->carried_inventory(),
                        mission_sys->exit_channel(), plr_.bond_count()};
        result.stats = plr_;
        result.play_frames = play_frames;
        return result;
    }

    void setup_results(Frontend& menu, bool persist) {
        const std::string codename = user_config.profile.empty() ? "Bond" : user_config.profile;
        const std::uint32_t base = mission_base != 0 ? mission_base : level_id;
        Profile profile = load_profile(codename).value_or(fresh_profile(codename));
        profile.difficulty = std::uint32_t(launch.difficulty);
        menu.set_profile(std::move(profile));

        MissionResults results;
        PlrStats::Score score{};
        if (score_tables_ok_) score = plr_.compute(score_tables_, base, launch.difficulty, mission_won);
        int medal = 0;
        if (score.valid) {
            results.score_text = separate_number(score.total);
            const auto* row = score_tables_.find(base);
            while (medal < 4 && score.total >= row->totals[std::size_t(medal)]) ++medal;
            results.medal_sprite = kCampaignMedalSprites[medal];
            if (medal != 0) results.name_line = std::string(ctx.assets.strings.label(0x2cb - medal));
            results.next_target_text = separate_number(row->totals[std::size_t(std::min(medal, 3))]);
        }
        const auto seconds = play_frames / unsigned(timing.FRAME_RATE_INT);
        char time[32];
        std::snprintf(time, sizeof(time), "%llu:%02llu", static_cast<unsigned long long>(seconds / 60),
                      static_cast<unsigned long long>(seconds % 60));
        results.stats = {{"Time", time},
                         {"Shots", std::to_string(plr_.shots())},
                         {"Hits", std::to_string(plr_.hits())},
                         {"Kills", std::to_string(plr_.kills())},
                         {"007 Tokens", std::to_string(plr_.bond_count())}};
        if (score.valid && score.total > std::uint32_t(std::max(0, menu.profile().best_score(base))))
            results.stats.emplace_back("Best", "New!");
        results.leads_to_wingame = !ctx.sp_data.levels.empty() && base == ctx.sp_data.levels.back().value;
        if (results.leads_to_wingame)
            menu.set_credits(load_credits(ctx.action_elf, ctx.assets.strings));
        menu.complete_mission(base, int(score.total), medal);
        if (persist && !nf::save_profile(menu.profile()))
            std::fprintf(stderr, "nightfire: could not save mission progress for %s\n", codename.c_str());

        menu.set_dossier(DossierInfo{.mission_level = base});
        menu.set_mission_results(std::move(results));
    }

    // Headless results shot (the interactive run_results drove the same page with live input).
    void draw_results_shot(const std::string& path) {
        Frontend menu(ctx.assets, ctx.menu, &ctx.mp_data, &ctx.sp_data);
        setup_results(menu, false);
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

    SpResult run_results() {
        Frontend menu(ctx.assets, ctx.menu, &ctx.mp_data, &ctx.sp_data);
        setup_results(menu, true);
        audio->stop_music();
        audio->clear_map_sounds();
        audio->unload_all_banks();
        if (const auto movie = mission_outro(level_id)) {
            MovieScreen screen(window, ui, text, audio.get(), gamepad, ctx.gamedir);
            screen.play(movie);
        }
        play_menu_music(menu_audio, kMenuBondTheme, config.music_volume);
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
            if (menu.page_id() == 0x40000002) break;
            if (menu_audio) {
                pump_menu_sounds(menu.take_sounds(), [this](std::uint32_t id) { menu_audio->play_sfx(id); });
                menu_audio->update();
                menu_audio->update();
            }
            // Results chain movies (wingame): play fullscreen, then run the post-movie transition.
            if (std::uint32_t movie = menu.take_movie_request()) {
                stop_menu_music(menu_audio);
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
            return SpResult{SpExit::MissionSelect, false, true};
        return SpResult{SpExit::QuitToMenu, false, true, 0};
    }
};

SpSession::SpSession(AppContext& ctx, Window& window, ui::Renderer& ui, ui::TextRenderer& text, const SpLaunch& launch,
                     AppConfig& cfg, audio::AudioSystem* menu_audio)
    : impl_(std::make_unique<Impl>(ctx, window, ui, text, launch, cfg, menu_audio)) {
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
    s.pad.set_mouse_sensitivity(s.config.mouse_look_sensitivity);
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
            PadState pad = s.pad.sample(s.world->settings(0).invert_look);
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
            if (s.tick_world(pad)) {
                const Player& resumed = *s.world->player(0);
                previous_eye = resumed.shaken_eye();
                previous_yaw = resumed.yaw;
                previous_pitch = resumed.view_pitch();
            }
            accumulator -= kStep;
            if (auto next = s.continuation()) {
                done = std::move(*next);
                finished = true;
                break;
            }
            const int end_delay = s.mission_won ? s.timing.FRAME_RATE_INT : 3 * s.timing.FRAME_RATE_INT;
            if (s.death_frames > end_delay && !s.end_shown) {
                s.end_shown = true;
                if (s.mission_won) {
                    done = s.run_results();
                    finished = true;
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
