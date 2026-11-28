// Driving mission session: build, tick, draw, text HUD, pause, headless shots.
#include "app/session_drive.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <tuple>
#include <vector>

#include "audio/driving_mixer.hpp"
#include "assets/menu_validate.hpp"
#include "driving/drive_session.hpp"
#include "driving/driving_level.hpp"
#include "driving/input_script.hpp"
#include "driving/mission.hpp"
#include "driving/particles.hpp"
#include "driving/scene_renderer.hpp"
#include "driving/snow_tracks.hpp"
#include "driving/vehicle_params.hpp"
#include "game/drone_render.hpp"
#include "game/local_pad.hpp"
#include "render/gl.hpp"
#include "ui/frontend.hpp"
#include "ui/hud.hpp"
#include "ui/prompts.hpp"

namespace nf::app {
namespace driving_app = nf::driving;

namespace {

PadState drive_pad(SDL_Gamepad* gamepad, bool gunner) {
    PadState pad;
    const bool* k = SDL_GetKeyboardState(nullptr);
    auto key = [&](SDL_Scancode a, SDL_Scancode b = SDL_SCANCODE_UNKNOWN) {
        return k[a] || (b != SDL_SCANCODE_UNKNOWN && k[b]);
    };
    pad.buttons = input_bindings().keyboard_buttons(InputContext::Driving, k);
    const int steer = int(key(SDL_SCANCODE_D, SDL_SCANCODE_RIGHT)) - int(key(SDL_SCANCODE_A, SDL_SCANCODE_LEFT));
    if (steer != 0) pad.lx = steer < 0 ? 0 : 255;
    if (gamepad) {
        pad.buttons |= input_bindings().gamepad_buttons(InputContext::Driving, gamepad);
        auto axis = [&](SDL_GamepadAxis id, std::uint8_t& value) {
            const int raw = SDL_GetGamepadAxis(gamepad, id);
            if (std::abs(raw) > 8000) value = std::uint8_t(std::clamp(raw / 128 + 128, 0, 255));
        };
        axis(SDL_GAMEPAD_AXIS_LEFTX, pad.lx);
        if (gunner) {
            axis(SDL_GAMEPAD_AXIS_LEFTY, pad.ly);
            axis(SDL_GAMEPAD_AXIS_RIGHTX, pad.rx);
            axis(SDL_GAMEPAD_AXIS_RIGHTY, pad.ry);
        }
    }
    if (gunner) {
        const int pitch = int(key(SDL_SCANCODE_S, SDL_SCANCODE_DOWN)) - int(key(SDL_SCANCODE_W, SDL_SCANCODE_UP));
        if (pitch != 0) pad.ly = pitch < 0 ? 0 : 255;
        if (k[SDL_SCANCODE_SPACE] || (SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_LMASK)) pad.buttons |= kPadR1;
        if (k[SDL_SCANCODE_LSHIFT] || (SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_RMASK)) pad.buttons |= kPadL1;
    }
    return pad;
}

// Small untextured octahedron for projectiles/pickups (same stand-in as nfdrive).
driving_app::SceneMesh drive_marker(float r, std::uint32_t color) {
    driving_app::SceneMesh m;
    driving_app::MeshBatch b;
    const float v[6][3] = {{r, 0, 0}, {-r, 0, 0}, {0, r, 0}, {0, -r, 0}, {0, 0, r}, {0, 0, -r}};
    for (auto& p : v) b.vertices.push_back({{p[0], p[1], p[2]}, {0, 0}, color});
    const int t[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
    for (auto& tri : t) {
        b.indices.push_back(tri[0]);
        b.indices.push_back(tri[1]);
        b.indices.push_back(tri[2]);
    }
    m.batches.push_back(std::move(b));
    return m;
}

// Frontend/drive-menu pad: the shared menu bindings from the keyboard plus a gamepad.
PadState drive_menu_pad(SDL_Gamepad* gamepad) { return input_bindings().sample(InputContext::Menu, gamepad); }

// Build a seated pose on the ACTION humanoid rig. The DRIVING rider animation
// bank has a different format. Seat and grip positions are in sled space.
Palette seated_rider(const CharacterBank& bank, const SkinDef& skin, bool gunner) {
    const Skeleton& skeleton = *bank.skeleton(skin.skeleton);
    const Palette bind = bind_palette(skin, skeleton);
    Pose pose = rest_pose(skeleton, &skin);
    if (skin.parent.size() <= 0x38) return bind;
    for (std::size_t i = 0; i < skin.parent.size(); ++i) {
        const auto parent = skin.bone_parent(i);
        const Vec3 p = transform_point(bind.world[i], {});
        pose.translation[i] = parent == 0x7f ? p : p - transform_point(bind.world[parent], {});
    }
    const auto pitch = [](float angle) { return Quat{std::sin(angle * 0.5f), 0, 0, std::cos(angle * 0.5f)}; };
    const auto turn_between = [](const Vec3& from, const Vec3& to) {
        const Vec3 axis = cross(from, to);
        const float w = length(from) * length(to) + dot(from, to);
        const float norm = std::sqrt(dot(axis, axis) + w * w);
        return Quat{axis[0] / norm, axis[1] / norm, axis[2] / norm, w / norm};
    };
    pose.rotation[1] = pitch(gunner ? 0.14f : 0.28f);
    pose.rotation[5] = pitch(gunner ? -0.10f : -0.18f);
    for (const auto& [hip, knee, sign] : {std::tuple{0x31, 0x32, -1.0f}, std::tuple{0x35, 0x36, 1.0f}}) {
        pose.rotation[hip] = turn_between(pose.translation[knee], {0.13f * sign, -0.10f, 0.42f});
        pose.rotation[knee] = pitch(1.64f);
    }
    for (const auto& [shoulder, elbow, wrist, sign] :
         {std::tuple{0x13, 0x14, 0x15, 1.0f}, std::tuple{0x1f, 0x20, 0x23, -1.0f}}) {
        pose.rotation[shoulder] = turn_between(pose.translation[elbow], {0.04f * sign, -0.20f, 0.23f});
        const Mat4 shoulder_rotation = quat_trans_to_mat(pose.rotation[shoulder], {});
        const Vec3 reach{-0.10f * sign, gunner ? 0.05f : 0.11f, 0.24f};
        const Vec3 local_reach{
            dot(reach, {shoulder_rotation[0], shoulder_rotation[1], shoulder_rotation[2]}),
            dot(reach, {shoulder_rotation[4], shoulder_rotation[5], shoulder_rotation[6]}),
            dot(reach, {shoulder_rotation[8], shoulder_rotation[9], shoulder_rotation[10]}),
        };
        pose.rotation[elbow] = turn_between(pose.translation[wrist], local_reach);
    }
    return build_palette(skin, pose);
}

driving_app::SceneMesh pursuit_sled_mesh(driving_app::DrivingLevel& level) {
    const auto bytes = level.read_file("data\\car\\model\\small_snowmobile.crp");
    const CarpFile carp(bytes);
    const auto parts = driving_app::build_vehicle_parts(carp, carp.load_elf(), true);
    return driving_app::build_bound_vehicle_body(carp, parts, 9);
}
}  // namespace

struct DriveSessionApp::Impl {
    Impl(AppContext& c, Window& w, ui::Renderer& u, ui::TextRenderer& t, std::string l, std::string car,
         const AppConfig& cfg, bool muted, std::optional<driving_app::AlpineCheckpoint> start)
        : ctx(c), window(w), ui(u), text(t), level_name(std::move(l)), car_name(std::move(car)),
          config(cfg), mute(muted), checkpoint(start) {}

    AppContext& ctx;
    Window& window;
    ui::Renderer& ui;
    ui::TextRenderer& text;
    std::string level_name, car_name;
    AppConfig config;
    bool mute;
    std::optional<driving_app::AlpineCheckpoint> checkpoint;

    std::unique_ptr<driving_app::DrivingLevel> level;
    const driving_app::LevelDesc* desc = nullptr;
    std::unique_ptr<driving_app::Mission> mission;
    std::unique_ptr<driving_app::SceneRenderer> renderer;
    driving_app::SceneRenderer::Handle track_handle{};
    driving_app::SceneRenderer::Handle sky_handle{}, snow_underlay{};
    driving_app::SceneRenderer::Handle body_handle{}, gunner_body_handle{};
    driving_app::SceneRenderer::Handle wheel_handles[4]{};
    struct AiHandles {
        driving_app::SceneRenderer::Handle body{};
        driving_app::SceneRenderer::Handle wheels[4]{};
    };
    std::vector<AiHandles> ai_handles;
    driving_app::SceneRenderer::Handle shot_handle{};
    driving_app::SceneRenderer::Handle pickup_handle{};
    std::unique_ptr<driving_app::ParticleSystem> particles;
    std::unique_ptr<driving_app::ParticleRenderer> particle_renderer;
    std::unique_ptr<driving_app::SnowTracks> snow_tracks;
    std::unique_ptr<driving_app::SnowTrackRenderer> snow_track_renderer;
    SDL_Gamepad* gamepad = nullptr;
    std::array<driving_app::SceneRenderer::Handle, 6> garage_doors{};
    driving_app::SceneRenderer::Handle garage_intact{}, garage_wreck{}, garage_bomb{}, hill_token{}, hill_shack{};
    std::vector<driving_app::SceneRenderer::Handle> barrel_handles, garage_fragments;
    std::array<driving_app::SceneRenderer::Handle, 4> bomb_digits{};
    std::array<driving_app::SceneRenderer::Handle, 2> castle_gate{};
    driving_app::SceneRenderer::Handle drain_grate{}, bridge_intact{}, bridge_ends{}, ravine_armor{};
    std::vector<driving_app::SceneRenderer::Handle> bridge_debris;
    driving_app::SceneRenderer::Handle chalet_door{}, chalet_door_exterior{};
    std::array<driving_app::SceneRenderer::Handle, 3> chalet_windows{};
    SshFile helicopter_shapes;
    driving_app::SceneRenderer::Handle helicopter_body{}, helicopter_rotor{};
    driving_app::SceneRenderer::Handle gun_handle{}, gun_mount{}, muzzle_flash{};
    std::unique_ptr<Hud> gunner_hud;
    std::unique_ptr<CharacterBank> guard_bank;
    std::unique_ptr<CharacterRenderer> guard_renderer;
    std::vector<AnimSet> guard_anim_sets;
    std::vector<std::unique_ptr<CharacterInstance>> guards;
    Palette pursuit_rider;
    std::unique_ptr<CharacterBank> riders_bank;
    std::unique_ptr<CharacterRenderer> riders_renderer;
    struct SledRider {
        const SkinDef* skin = nullptr;
        Palette pose;
        Vec3 seat;
    };
    std::array<SledRider, 2> riders;
    driving_app::SceneRenderer::Handle pursuit_sled{};
    SshFile pursuit_shapes;
    std::unique_ptr<audio::DrivingAudio> drive_audio;
    std::optional<MenuFile> pause_menu;
    std::array<std::shared_ptr<const EaStream>, std::size(driving_app::AlpineEscape::kSpeech)> alpine_speech;
    audio::DrivingAudio::VoiceId sled_idle = 0, sled_cruise = 0, sled_snow = 0, heli_voice = 0;
    float guard_foot_offset = 0.0f;

    bool build() {
        desc = driving_app::find_level(level_name);
        if (!desc) {
            std::fprintf(stderr, "nightfire: unknown driving level %s\n", level_name.c_str());
            return false;
        }
        const bool alpine = desc->viv == "MIS3" && (car_name.empty() || car_name == "supersnow");
        if (checkpoint && !alpine) {
            std::fprintf(stderr, "nightfire: driving checkpoints are available in Alpine Escape with its mission snowmobile\n");
            return false;
        }
        using Alpine = driving_app::AlpineEscape;
        constexpr std::size_t shutters[] = {
            Alpine::kGarageDoorInstance, Alpine::kExitDoorInstance,
            Alpine::kOtherShutters[0], Alpine::kOtherShutters[1], Alpine::kOtherShutters[2], Alpine::kOtherShutters[3],
        };
        std::vector<driving_app::DetachedTrackMesh> props{
            {Alpine::kGrateInstance, {}},
            {Alpine::kGateLeaves[0], {}},
            {Alpine::kGateLeaves[1], {}},
            {Alpine::kIntactGarageInstance, {}},
            {Alpine::kWreckGarageInstance, {}},
            {Alpine::kBombInstance, {}},
            {Alpine::kHillTokenInstance, {}},
            {Alpine::kHillShackInstance, {}},
            {Alpine::kBridgeInstance, {}},
            {Alpine::kRavineArmorInstance, {}},
            {Alpine::kChaletDoorInstance, {}},
            {Alpine::kChaletDoorExteriorInstance, {}},
            {Alpine::kChaletWindowInstances[0], {}},
            {Alpine::kChaletWindowInstances[1], {}},
            {Alpine::kChaletWindowInstances[2], {}},
        };
        for (const auto shutter : shutters) props.push_back({shutter, {}});
        const std::span<driving_app::DetachedTrackMesh> detached = alpine
            ? std::span<driving_app::DetachedTrackMesh>(props)
            : std::span<driving_app::DetachedTrackMesh>{};
        level = std::make_unique<driving_app::DrivingLevel>(ctx.gamedir, *desc, detached);
        mission = std::make_unique<driving_app::Mission>(*level, car_name, alpine,
            checkpoint.value_or(driving_app::AlpineCheckpoint::Garage));
        renderer = std::make_unique<driving_app::SceneRenderer>(level->shapes());
        renderer->set_fog(level->fog_colour(), level->fog_start(), level->fog_end());
        renderer->set_ambient(level->ambient());
        track_handle = renderer->upload(level->track());
        const driving_app::DriveSession& s = mission->session();
        if (checkpoint) {
            const auto& options = driving_app::kAlpineCheckpoints;
            const auto point = std::ranges::find(options, *checkpoint, &driving_app::AlpineCheckpointOption::checkpoint);
            const Vec3 pos = s.player_position();
            std::printf("Alpine Escape checkpoint: %s at %.2f, %.2f, %.2f\n", point->key.data(), pos[0], pos[1], pos[2]);
            std::fflush(stdout);
        }
        if (desc->viv == "MIS3") {
            renderer->set_key_light(level->key_light_direction(), level->key_light_colour());
            sky_handle = renderer->upload(level->render_extras().sky);
            snow_underlay = renderer->upload(driving_app::build_snow_underlay(s.collision(), level->track()));
        }
        body_handle = renderer->upload(s.body_mesh(), {&s.car_shapes()});
        if (const auto* opening = mission->alpine()) {
            snow_tracks = std::make_unique<driving_app::SnowTracks>(level->track(), opening->guards().size() + 1);
            snow_track_renderer = std::make_unique<driving_app::SnowTrackRenderer>();
            gunner_body_handle = renderer->upload(s.gunner_body_mesh(), {&s.car_shapes()});
            auto upload = [&](std::size_t instance) {
                return renderer->upload(std::ranges::find(props, instance, &driving_app::DetachedTrackMesh::instance)->mesh);
            };
            for (std::size_t i = 0; i < std::size(shutters); ++i) garage_doors[i] = upload(shutters[i]);
            drain_grate = upload(Alpine::kGrateInstance);
            castle_gate[0] = upload(Alpine::kGateLeaves[0]);
            castle_gate[1] = upload(Alpine::kGateLeaves[1]);
            garage_intact = upload(Alpine::kIntactGarageInstance);
            garage_wreck = upload(Alpine::kWreckGarageInstance);
            for (const auto& fragment : level->render_extras().garage_fragments)
                garage_fragments.push_back(renderer->upload(fragment.mesh));
            garage_bomb = upload(Alpine::kBombInstance);
            for (std::size_t i = 0; i < bomb_digits.size(); ++i)
                bomb_digits[i] = renderer->upload(level->render_extras().bomb_digits[i]);
            hill_token = upload(Alpine::kHillTokenInstance);
            hill_shack = upload(Alpine::kHillShackInstance);
            bridge_intact = upload(Alpine::kBridgeInstance);
            ravine_armor = upload(Alpine::kRavineArmorInstance);
            chalet_door = upload(Alpine::kChaletDoorInstance);
            chalet_door_exterior = upload(Alpine::kChaletDoorExteriorInstance);
            for (std::size_t i = 0; i < chalet_windows.size(); ++i)
                chalet_windows[i] = upload(Alpine::kChaletWindowInstances[i]);
            bridge_ends = renderer->upload(level->render_extras().bridge_ends);
            for (const auto& fragment : level->render_extras().bridge_debris)
                bridge_debris.push_back(renderer->upload(fragment.mesh));
            const auto helicopter_bytes = level->read_file("data\\car\\model\\openheli.crp");
            const CarpFile helicopter_carp(helicopter_bytes);
            const auto helicopter_parts = driving_app::build_vehicle_parts(helicopter_carp, helicopter_carp.load_elf(), true);
            helicopter_shapes = parse_ssh(level->read_file("data\\car\\model\\openheli.ssh"));
            constexpr int omit_rotors[] = {3, 4};
            constexpr int omit_hull[] = {0, 1, 2};
            helicopter_body = renderer->upload(
                driving_app::build_bound_vehicle_body(helicopter_carp, helicopter_parts, 11, omit_rotors), {&helicopter_shapes});
            helicopter_rotor = renderer->upload(
                driving_app::build_bound_vehicle_body(helicopter_carp, helicopter_parts, 3, omit_hull), {&helicopter_shapes});
            for (const auto& barrel : level->render_extras().barrels)
                barrel_handles.push_back(renderer->upload(barrel.mesh));
            gun_handle = renderer->upload(s.mounted_gun_mesh(), {&s.car_shapes()});
            gun_mount = renderer->upload(s.gun_mount_mesh(), {&s.car_shapes()});
            muzzle_flash = renderer->upload(drive_marker(1.0f, 0xFF50BFFF));
            gunner_hud = std::make_unique<Hud>(ctx.assets, ctx.hud_data);
            gunner_hud->disable_all();
            gunner_hud->enable(HudPane::Health, true);
            if (const auto* snow_guard = ctx.mp_data.find_character(6)) {
                guard_bank = open_character_bank(ctx.files, "07000024.bin");
                if (const SkinDef* skin = guard_bank->skin(snow_guard->skin.skin_hash)) {
                    guard_renderer = std::make_unique<CharacterRenderer>(*guard_bank);
                    Vec3 lo{}, hi{};
                    guard_renderer->bounds(*skin, 0, lo, hi);
                    guard_foot_offset = std::max(0.0f, -lo[1]);
                    pursuit_rider = seated_rider(*guard_bank, *skin, false);
                    guard_anim_sets = read_anim_sets(ctx.action_elf);
                    const auto rifle = std::ranges::find(guard_anim_sets, "Rifle", &AnimSet::name);
                    for (std::size_t i = 0; i < opening->guards().size(); ++i) {
                        auto& character = guards.emplace_back(std::make_unique<CharacterInstance>(*guard_bank, *skin));
                        if (rifle != guard_anim_sets.end()) character->set_anim_set(&*rifle);
                    }
                }
            }
            pursuit_shapes = parse_ssh(level->read_file("data\\car\\model\\small_snowmobile.ssh"));
            pursuit_sled = renderer->upload(pursuit_sled_mesh(*level), {&pursuit_shapes});
            riders_bank = open_character_bank(ctx.files, "07000007.bin");
            riders_renderer = std::make_unique<CharacterRenderer>(*riders_bank);
            riders[0].skin = riders_bank->skin(0x0500003b);  // Zoe
            riders[0].seat = {0, 1.20f, 0.24f};
            riders[1].skin = riders_bank->skin(0x0500000d);  // Bond's tuxedo skin
            riders[1].seat = {0, 1.31f, -0.72f};
            for (std::size_t i = 0; i < riders.size(); ++i)
                if (riders[i].skin) riders[i].pose = seated_rider(*riders_bank, *riders[i].skin, i == 1);
            gunner_hud->update(HudState{});
        }
        for (int w = 0; w < 4; ++w) wheel_handles[w] = renderer->upload(s.wheel_meshes()[w], {&s.car_shapes()});
        for (const driving_app::AiCar& a : mission->ai()) {
            AiHandles h;
            if (a.model >= 0) {
                const driving_app::CarModel& m = mission->models()[std::size_t(a.model)];
                h.body = renderer->upload(m.body, {&m.shapes});
                for (int w = 0; w < 4; ++w) h.wheels[w] = renderer->upload(m.wheels[w], {&m.shapes});
            }
            ai_handles.push_back(h);
        }
        shot_handle = renderer->upload(drive_marker(0.5f, 0xFF33CCFF));
        pickup_handle = renderer->upload(drive_marker(1.2f, 0xFF33FF66));
        particles = std::make_unique<driving_app::ParticleSystem>(
            driving_app::ParticleSystem::Config::for_archive(desc->viv));
        particle_renderer = std::make_unique<driving_app::ParticleRenderer>();
        std::printf("%.*s: mission ready (car %s)\n", int(desc->name.size()), desc->name.data(),
                    mission->session().car().c_str());
        return true;
    }

    void start_audio() {
        if (!mission->alpine() || mute || config.sfx_volume == 0) return;
        try {
            if (const auto dir = find_driving_dir(ctx.gamedir)) {
                DrivingMission data(*dir, std::string(desc->viv));
                drive_audio = std::make_unique<audio::DrivingAudio>();
                drive_audio->load_level(data, desc->track);
                if (!drive_audio->open_device()) {
                    drive_audio.reset();
                    return;
                }
                const float volume = config.sfx_volume / 100.0f;
                sled_idle = drive_audio->play_sample("Engine", "SFX_Idle", {.volume = 0.53f * volume});
                sled_cruise = drive_audio->play_sample("Engine", "SFX_sno_cruz", {.volume = 0.005f});
                sled_snow = drive_audio->play_sample("Generic", "SFX_sno_snowRN", {.volume = 0.005f});
                if (auto archive = data.open_speech("EN")) {
                    for (std::size_t i = 0; i < alpine_speech.size(); ++i)
                        if (const auto* entry = archive->find(driving_app::AlpineEscape::kSpeech[i]))
                            alpine_speech[i] = std::make_shared<const EaStream>(DrivingMission::read_stream(*archive, *entry));
                }
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "nightfire: driving audio: %s\n", e.what());
        }
    }

    void tick(const PadState& pad) {
        const auto* alpine = mission->alpine();
        const bool running = mission->state() == driving_app::MissionState::Running;
        const bool door_broken = alpine && alpine->chalet_door_broken();
        const bool window_broken = alpine && alpine->chalet_window_broken();
        mission->tick(pad);
        if (particles) particles->tick(mission->session(), *mission);
        const auto sounds = mission->drain_sfx();
        if (!alpine) return;
        if (running) {
            const auto& session = mission->session();
            snow_tracks->tick(driving_app::kTickDt);
            snow_tracks->sample(0, session.player_position(), session.collision());
            for (std::size_t i = 0; i < alpine->guards().size(); ++i) {
                const auto& guard = alpine->guards()[i];
                if (!guard.sled) continue;
                const auto position = guard.active && guard.health > 0 ? std::optional{guard.position} : std::nullopt;
                snow_tracks->sample(i + 1, position, session.collision());
            }
        }
        if (particles && running) {
            for (const auto& blast : alpine->blasts()) particles->burst_explosion(blast.center, blast.scale);
            const auto direction = mission->session().player_forward();
            if (!door_broken && alpine->chalet_door_broken())
                particles->burst_chalet_debris(driving_app::AlpineEscape::kChaletDoor, direction, false);
            if (!window_broken && alpine->chalet_window_broken())
                particles->burst_chalet_debris(driving_app::AlpineEscape::kChaletWindow, direction, true);
        }
        HudState state;
        state.health = 100.0f * (1.0f - mission->hud().damage01);
        state.armor = alpine->armor_points();
        state.crosshair_enabled = false;
        if (alpine->took_damage() && mission->state() == driving_app::MissionState::Running)
            // The shared HUD fades one alpha step per tick. Clear each hit
            // before a guard's next burst so it does not tint the whole chase.
            state.damage = HudDamage{0xF, 48};
        gunner_hud->update(state);
        for (std::size_t i = 0; i < guards.size(); ++i) {
            const auto& g = alpine->guards()[i];
            if (!g.active || g.health <= 0 || g.sled) continue;
            guards[i]->update_locomotion(g.step_distance, 0.10f);
            guards[i]->advance(driving_app::kTickDt);
            guards[i]->take_events();
        }
        if (drive_audio) {
            const float gain = config.sfx_volume / 100.0f;
            const float speed = mission->session().player_speed();
            const float motor = std::clamp(speed / 12.0f, 0.0f, 1.0f);
            drive_audio->set_voice(sled_idle, {.volume = gain * 0.53f * (1.0f - motor), .pitch = 1.0f + 0.35f * motor});
            drive_audio->set_voice(sled_cruise, {.volume = gain * 0.57f * motor, .pitch = 0.82f + 0.026f * speed});
            drive_audio->set_voice(sled_snow, {.volume = gain * 0.23f * motor, .pitch = 0.82f + 0.019f * speed});
            const auto& camera = alpine->camera();
            const Vec3 forward = camera.target - camera.eye;
            const Vec3 right = cross(camera.up, forward);
            if (const auto& heli = alpine->helicopter()) {
                if (!drive_audio->voice_playing(heli_voice))
                    heli_voice = drive_audio->play_sample("Generic", "SFX_sno_heli", {.volume = 0.0f});
                const Vec3 offset = Vec3{(*heli)[12], (*heli)[13], (*heli)[14]} - camera.eye;
                const float azimuth = std::atan2(dot(offset, right), dot(offset, forward)) / 6.2831853f;
                const float distance = length(offset);
                drive_audio->set_voice(heli_voice, {.volume = gain * 0.67f / (1.0f + distance * 0.044f), .azimuth = azimuth});
            } else if (heli_voice) {
                drive_audio->stop_voice(heli_voice);
                heli_voice = 0;
            }
            for (const auto& sound : sounds) {
                if (sound.speech) {
                    const auto& names = driving_app::AlpineEscape::kSpeech;
                    const auto it = std::ranges::find(names, sound.name);
                    if (it != std::end(names) && alpine_speech[it - std::begin(names)])
                        drive_audio->queue_stream(audio::DrivingAudio::Channel::Speech, alpine_speech[it - std::begin(names)],
                                                  {.volume = sound.volume * gain});
                    continue;
                }
                const Vec3 offset = sound.pos - camera.eye;
                const float distance = length(offset);
                const float volume = sound.volume * (config.sfx_volume / 100.0f) / (1.0f + distance * 0.055f);
                const float azimuth = std::atan2(dot(offset, right), dot(offset, forward)) / 6.2831853f;
                const audio::DrivingPlayParams params{.volume = volume, .azimuth = azimuth, .loop = false};
                if (!drive_audio->play_sample("Weapons", sound.name, params) &&
                    !drive_audio->play_sample("Generic", sound.name, params) &&
                    !drive_audio->play_sample("Collisions", sound.name, params) &&
                    !drive_audio->play_sample("Smackables", sound.name, params))
                    drive_audio->play_sample("Actors", sound.name, params);
            }
        }
    }

    void draw_gunner_hud(const driving_app::AlpineEscape& alpine, ui::TextStyle style) {
        if (alpine.exterior()) return;
        gunner_hud->draw(ui, text);
        style.font = 2;
        style.color = 0x745E3CFF;
        style.align = ui::Align::Right;
        const float right = 320.0f + ui.canvas_width() * 0.5f;
        const int ammo = alpine.grenade_selected() ? alpine.grenade_ammo() : alpine.ammo();
        text.draw(right - 30.0f, 367, std::to_string(ammo) + (alpine.grenade_selected() ? "\nJL-7C" : "\nJL-7"), style);
        const ui::Color sight = alpine.locked() ? ui::Color{0x73, 0xB0, 0x34, 0x80}
                                              : ui::Color{0xB0, 0x24, 0x17, 0x80};
        ui.fill({315, 223, 3, 1.5f}, sight);
        ui.fill({322, 223, 3, 1.5f}, sight);
        ui.fill({319.3f, 219, 1.5f, 3}, sight);
        ui.fill({319.3f, 226, 1.5f, 3}, sight);
        style.align = ui::Align::Center;
        if (mission->hud().time_s < 5.5f) text.draw(320, 99, mission->hud().objective, style);
        if (alpine.show_ravine_objective()) text.draw(320, 104, std::string("NEW OBJECTIVE\n") + alpine.objective(), style);
        if (alpine.gate_waiting() && !alpine.gate_destroyed())
            text.draw(320, 99, alpine.grenade_selected() ? "Aim at the gate and fire" : "C / R2   Select JL-7C", style);
        if (mission->hud().message_timer > 0 || alpine.trail_end()) text.draw(320, 385, mission->hud().message, style);
        if (alpine.token_notice() > 0) {
            style.color = 0xFFE4A3FF;
            text.draw(320, 106, "007 Token Collected", style);
        }
    }

    void draw_bridge(const driving_app::AlpineEscape& alpine, const Mat4& vp) {
        if (alpine.bridge_destroyed()) {
            renderer->draw(bridge_ends, vp);
            // The bridge aq streams contain 15 Hz quaternions and positions.
            const float frame = alpine.bridge_blast_age() * 15.0f;
            const auto& pieces = level->render_extras().bridge_debris;
            for (std::size_t i = 0; i < pieces.size(); ++i) {
                const auto& piece = pieces[i];
                const float f = std::min(frame, float(piece.frames.size() - 1));
                const std::size_t first = std::size_t(f), next = std::min(first + 1, piece.frames.size() - 1);
                const float t = f - float(first);
                const auto& a = piece.frames[first];
                const auto& b = piece.frames[next];
                const Mat4 local = quat_trans_to_mat(slerp(a.rotation, b.rotation, t), a.position + (b.position - a.position) * t);
                renderer->draw(bridge_debris[i], vp, mul(piece.placement, local));
            }
        } else {
            renderer->draw(bridge_intact, vp);
        }
        if (const auto& heli = alpine.helicopter()) {
            constexpr float quarter_turn = 1.5707963f;
            const Mat4 model = mul(*heli, drone::drone_model_matrix({}, quarter_turn));
            renderer->draw(helicopter_body, vp, model);
            // Spin the original main rotor around its bind-table hub.
            constexpr Vec3 hub{0, 0, -0.24f};
            const Mat4 spin = drone::drone_model_matrix(hub, alpine.clock() * 34.0f);
            Mat4 origin = identity();
            origin[14] = -hub[2];
            renderer->draw(helicopter_rotor, vp, mul(model, mul(spin, origin)));
        }
        if (const auto rocket = alpine.bridge_rocket()) {
            Vec3 direction = driving_app::AlpineEscape::kBridgeCenter - *rocket;
            direction = direction * (1.0f / length(direction));
            Vec3 right = cross(Vec3{0, 1, 0}, direction);
            right = right * (1.0f / length(right));
            const Vec3 up = cross(direction, right);
            const Mat4 trail{right[0] * .18f, right[1] * .18f, right[2] * .18f, 0,
                             up[0] * .18f, up[1] * .18f, up[2] * .18f, 0,
                             direction[0] * 2.8f, direction[1] * 2.8f, direction[2] * 2.8f, 0,
                             (*rocket)[0], (*rocket)[1], (*rocket)[2], 1};
            renderer->draw(shot_handle, vp, trail, false);
        }
        if (const auto round = alpine.helicopter_round()) {
            Mat4 model = drone::drone_model_matrix(*round, 0);
            model[0] = model[5] = model[10] = 0.29f;
            renderer->draw(shot_handle, vp, model, false);
        }
        if (!alpine.ravine_armor_collected()) renderer->draw(ravine_armor, vp);
    }

    void draw_chalet(const driving_app::AlpineEscape& alpine, const Mat4& vp) {
        if (!alpine.chalet_door_broken()) {
            renderer->draw(chalet_door_exterior, vp);
            renderer->draw(chalet_door, vp);
        } else if (alpine.chalet_door_age() < 0.95f) {
            const float t = alpine.chalet_door_age();
            const Vec3 anchor = driving_app::AlpineEscape::kChaletDoor;
            const Vec3 travel = {t * 3.7f, t * (2.0f - t * 5.0f), -t * 5.1f};
            const Mat4 spin = drone::drone_model_matrix(anchor + travel, t * 4.5f);
            Mat4 local = identity();
            for (int axis = 0; axis < 3; ++axis) local[12 + axis] = -anchor[axis];
            renderer->draw(chalet_door, vp, mul(spin, local));
        }
        if (!alpine.chalet_window_broken())
            for (const auto window : chalet_windows) renderer->draw(window, vp);
    }

    void draw_frame() {
        int w, h;
        window.begin_frame(w, h);
        // The drive camera uses the selected native window aspect; UI elements draw
        // over the full window unless the saved original 4:3 pillarbox option is on.
        const GameView gv = game_view(config, w, h);
        const int gl_y = h - gv.y - gv.h;
        glDisable(GL_SCISSOR_TEST);
        glViewport(0, 0, w, h);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_SCISSOR_TEST);
        glViewport(gv.x, gl_y, gv.w, gv.h);
        glScissor(gv.x, gl_y, gv.w, gv.h);
        glClearColor(level->fog_colour()[0], level->fog_colour()[1], level->fog_colour()[2], 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const auto* alpine = mission->alpine();
        const driving_app::CameraPose& c = alpine ? alpine->camera() : mission->session().camera_pose();
        const Mat4 view = driving_app::drive_view_matrix(c.eye, c.target, c.up);
        const Mat4 projection = perspective(c.fovy, gv.aspect, c.znear, c.zfar);
        const Mat4 vp = mul(projection, view);
        if (desc->viv == "MIS3") {
            const auto& sled = mission->session();
            renderer->set_view_lights(c.eye, level->render_extras().lamps, sled.player_position(), sled.player_forward());
            renderer->draw_sky(sky_handle, vp, c.eye);
            renderer->draw(snow_underlay, vp, identity(), true, 0.0f, c.znear);
        }
        renderer->draw(track_handle, vp);
        if (snow_track_renderer) snow_track_renderer->draw(*snow_tracks, vp);
        const bool riding = alpine && !alpine->exterior();
        // Clip the seat and inner cowling close to Bond's eyes. When the
        // sled pitches into a dip those polygons otherwise cover the trail.
        renderer->draw(riding ? gunner_body_handle : body_handle, vp, mission->session().body_matrix(),
                       true, riding ? 0.92f : 0.0f);
        if (alpine) {
            if (alpine->garage_destroyed()) {
                renderer->draw(garage_wreck, vp);
                const float age = alpine->garage_blast_age();
                if (age < 2.35f) {
                    const auto& fragments = level->render_extras().garage_fragments;
                    for (std::size_t i = 0; i < fragments.size(); ++i) {
                        const Vec3 anchor = fragments[i].position;
                        const float side = (i % 2 == 0) ? 1.0f : -1.0f;
                        const float lift = 6.5f + float(i % 7) * 0.48f;
                        const Vec3 offset{side * age * 3.1f, std::max(-1.0f, age * (lift - 4.9f * age)),
                                          age * (float(i % 5) - 2.0f) * 1.9f};
                        const Mat4 spin = drone::drone_model_matrix(anchor + offset, age * side * 4.2f);
                        Mat4 local = identity();
                        for (int axis = 0; axis < 3; ++axis) local[12 + axis] = -anchor[axis];
                        renderer->draw(garage_fragments[i], vp, mul(spin, local));
                    }
                }
            } else {
                renderer->draw(garage_intact, vp);
                Mat4 door = identity();
                door[13] = alpine->door_lift();
                renderer->draw(garage_doors[0], vp, door);
                door[13] = alpine->exit_door_lift();
                renderer->draw(garage_doors[1], vp, door);
                for (std::size_t i = 2; i < garage_doors.size(); ++i) renderer->draw(garage_doors[i], vp);
            }
            if (alpine->timer_cut()) {
                renderer->draw(garage_bomb, vp, identity(), false);
                renderer->draw(bomb_digits[alpine->timer_digit()], vp, identity(), false);
            }
            for (std::size_t i = 0; i < barrel_handles.size(); ++i)
                if (!alpine->barrels()[i].destroyed) renderer->draw(barrel_handles[i], vp);
            if (!alpine->hill_shack_destroyed()) renderer->draw(hill_shack, vp);
            if (alpine->token_visible()) {
                const Vec3 anchor = driving_app::AlpineEscape::kHillToken;
                const float t = alpine->clock();
                const Mat4 spin = drone::drone_model_matrix(anchor + Vec3{0, 0.17f * std::sin(t * 4.0f), 0}, t * 2.6f);
                Mat4 local = identity();
                for (int i = 0; i < 3; ++i) local[12 + i] = -anchor[i];
                renderer->draw(hill_token, vp, mul(spin, local));
            }
            draw_bridge(*alpine, vp);
            draw_chalet(*alpine, vp);
            if (!alpine->grate_broken()) renderer->draw(drain_grate, vp);
            if (!alpine->gate_destroyed()) {
                constexpr Vec3 hinges[] = {{1637.731f, -169.308f, 284.806f}, {1637.759f, -169.224f, 272.411f}};
                for (std::size_t i = 0; i < castle_gate.size(); ++i) {
                    const float angle = alpine->gate_closure() * (i == 0 ? 1.5707963f : -1.5707963f);
                    const float c = std::cos(angle), s = std::sin(angle);
                    const Vec3 h = hinges[i];
                    const Mat4 leaf{c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0,
                                    h[0] - c * h[0] - s * h[2], 0, h[2] + s * h[0] - c * h[2], 1};
                    renderer->draw(castle_gate[i], vp, leaf);
                }
            }
            if (alpine->grenade().life > 0) {
                Mat4 shot = identity();
                const auto& p = alpine->grenade().pos;
                for (int a = 0; a < 3; ++a) {
                    shot[a * 5] = 0.25f;
                    shot[12 + a] = p[a];
                }
                renderer->draw(shot_handle, vp, shot, false);
            }
            const Vec3 right{view[0], view[4], view[8]};
            const Vec3 up{view[1], view[5], view[9]};
            const CharacterLighting snow_lighting{.tint={0.55f, 0.59f, 0.67f}};
            if (alpine->exterior()) {
                const Mat4 sled = mission->session().body_matrix();
                for (const auto& rider : riders) {
                    if (!rider.skin) continue;
                    Mat4 seat = identity();
                    std::copy(rider.seat.begin(), rider.seat.end(), seat.begin() + 12);
                    riders_renderer->draw_projected(vp, right, up, *rider.skin, rider.pose, mul(sled, seat),
                                                    0, {}, snow_lighting);
                }
            }
            for (const auto& guard : alpine->guards()) {
                if (!guard.sled || !guard.active) continue;
                const bool down = guard.health <= 0;
                Mat4 model = drone::drone_model_matrix(down ? guard.sled_wreck : guard.position, guard.yaw);
                if (down) {
                    const float c = std::cos(guard.sled_roll), s = std::sin(guard.sled_roll);
                    const Mat4 roll{c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0.18f, 0, 1};
                    model = mul(model, roll);
                }
                renderer->draw(pursuit_sled, vp, model);
            }
            for (std::size_t i = 0; i < guards.size(); ++i) {
                const auto& guard = alpine->guards()[i];
                if (!guard.active) continue;
                const auto& pose = *guards[i];
                const bool down = guard.health <= 0;
                const Vec3 root = down ? guard.position
                    : guard.position + Vec3{0, guard.sled ? 0.94f : guard_foot_offset, 0};
                Mat4 model = drone::drone_model_matrix(root, guard.yaw);
                if (down) {
                    const float c = std::cos(guard.fall_angle), s = std::sin(guard.fall_angle);
                    const Mat4 tilt{1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1};
                    model = mul(model, tilt);
                }
                guard_renderer->draw_projected(vp, right, up, pose.skin(),
                    guard.sled && !down ? pursuit_rider : pose.palette(), model, 0, {}, snow_lighting);
            }
        }
        for (int i = 0; i < 4; ++i) renderer->draw(wheel_handles[i], vp, mission->session().wheel_matrix(i));
        for (std::size_t i = 0; i < mission->ai().size() && i < ai_handles.size(); ++i) {
            const driving_app::AiCar& a = mission->ai()[i];
            if (a.model < 0) continue;
            if (a.driver && a.driver->role() == driving_app::AiRole::Heli) {
                renderer->draw(ai_handles[i].body, vp, mission->heli_matrix(*a.driver));
                continue;
            }
            renderer->draw(ai_handles[i].body, vp, mission->ai_body_matrix(i));
            if (mission->models()[std::size_t(a.model)].has_wheels)
                for (int k = 0; k < 4; ++k)
                    renderer->draw(ai_handles[i].wheels[k], vp, mission->ai_wheel_matrix(i, k));
        }
        for (const driving_app::Projectile& p : mission->projectiles()) {
            const Mat4 m = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, p.pos[0], p.pos[1], p.pos[2], 1};
            renderer->draw(shot_handle, vp, m, false);
        }
        for (const driving_app::Pickup& p : mission->pickups()) {
            if (p.respawn > 0) continue;
            const Mat4 m = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, p.pos[0], p.pos[1], p.pos[2], 1};
            renderer->draw(pickup_handle, vp, m);
        }
        if (particles && particle_renderer) {
            Vec3 fwd = c.target - c.eye;
            const float fwd_len = length(fwd);
            if (fwd_len > 1e-6f) {
                fwd = fwd * (1.0f / fwd_len);
                Vec3 right = cross(c.up, fwd);
                if (length(right) > 1e-6f) {
                    right = right * (1.0f / length(right));
                    particle_renderer->draw(*particles, vp, c.eye, right, cross(fwd, right));
                }
            }
        }
        renderer->draw_lamp_glows(vp, c.eye, c.target, c.up);
        if (alpine && !alpine->exterior()) {
            if (alpine->shot_seconds() > 0) {
                const Vec3 from = c.eye + Vec3{0, -0.33f, 0};
                const Vec3 ray = alpine->shot_end() - from;
                const float distance = length(ray);
                if (distance > 2.0f) {
                    const Vec3 dir = ray * (1.0f / distance);
                    const Vec3 p = alpine->shot_end() - dir * std::min(4.0f, distance * 0.35f);
                    const Vec3 side = cross(c.up, dir) * 0.025f;
                    const Vec3 up = cross(dir, side);
                    const Vec3 long_axis = dir * std::min(3.0f, distance * 0.25f);
                    const Mat4 tracer{side[0], side[1], side[2], 0, up[0], up[1], up[2], 0,
                                      long_axis[0], long_axis[1], long_axis[2], 0, p[0], p[1], p[2], 1};
                    renderer->draw(muzzle_flash, vp, tracer, false);
                }
            }
            glClear(GL_DEPTH_BUFFER_BIT);
            Mat4 mount = identity();
            mount[10] = -1;
            mount[13] = -0.46f;
            mount[14] = -1.28f;
            if (dot(c.target - c.eye, mission->session().player_forward()) > 0)
                renderer->draw(gun_mount, projection, mount, false);
            Mat4 gun = identity();
            gun[10] = -1;
            gun[13] = -1.26f;
            gun[14] = -0.82f;
            if (alpine->shot_seconds() > 0) gun[14] += 0.06f;
            renderer->draw(gun_handle, projection, gun, false);
            if (alpine->shot_seconds() > 0) {
                const Mat4 flash{0.10f, 0, 0, 0, 0, 0.085f, 0, 0, 0, 0, 0.6f, 0, 0, -0.08f, -1.78f, 1};
                renderer->draw(muzzle_flash, projection, flash, false);
            }
        }
        const driving_app::DrivingHud& hud = mission->hud();
        ui.begin(w, h, false);
        ui::select_prompts(0, InputContext::Driving);
        ui::TextStyle style;
        style.font = 1;
        style.drop_shadow = true;
        if (alpine && alpine->exterior()) {
            const float width = ui.canvas_width();
            ui.fill({320 - width * 0.5f, 0, width, 45}, {0, 0, 0, 0x80});
            ui.fill({320 - width * 0.5f, 403, width, 45}, {0, 0, 0, 0x80});
        } else if (alpine) draw_gunner_hud(*alpine, style);
        else {
            std::string top = hud.speed_text() + "   " + hud.time_text();
            if (!hud.objective.empty()) top += "   " + hud.objective;
            text.draw(16, 420, top, style);
            if (!hud.message.empty() && hud.message_timer > 0) text.draw(16, 396, hud.message, style);
        }
        if (hud.won || hud.lost) {
            ui::TextStyle big = style;
            big.scale_x = big.scale_y = 2.0f;
            big.align = ui::Align::Center;
            text.draw(320, 240, hud.banner, big);
        }
        ui.end();
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }

    // Driving pause: the frontend pause page (Resume restarts the loop, Quit leaves).
    std::optional<DriveResult> run_pause(SDL_Gamepad* gamepad, bool open_checkpoints) {
        const bool checkpoints_available = mission->alpine() != nullptr;
        const auto& options = driving_app::kAlpineCheckpoints;
        constexpr int count = int(std::size(options));
        bool checkpoints_open = checkpoints_available && open_checkpoints;
        int checkpoint_row = 0;
        if (checkpoint) {
            const auto current = std::ranges::find(options, *checkpoint, &driving_app::AlpineCheckpointOption::checkpoint);
            if (current != std::end(options)) checkpoint_row = int(current - std::begin(options));
        }
        // Driving VIVs don't carry an ACTION menu script. Use the shared
        // campaign pause page from the first Exchange level.
        if (!pause_menu) {
            const auto* pause_source = ctx.files.find("07000005.bin");
            if (!pause_source) return std::nullopt;
            const auto bytes = ctx.files.read(*pause_source);
            auto menu = load_menu_from_bin(bytes);
            if (!menu.page(0x4000004b)) return std::nullopt;
            add_level_sprites(ctx.assets.sprites, bytes);
            pause_menu = std::move(menu);
        }
        Frontend menu(ctx.assets, *pause_menu, &ctx.mp_data, &ctx.sp_data);
        menu.open(FrontendMode::Pause);
        draw_frame();
        int w, h;
        window.begin_frame(w, h);
        const GameView view = game_view(config, w, h);
        ui.capture_backdrop(view.x, h - view.y - view.h, view.w, view.h);
        PadHistory hist;
        bool wait_release = true;
        while (!menu.wants_close()) {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) return DriveResult{DriveExit::QuitToMenu};
                if (e.type != SDL_EVENT_KEY_DOWN || e.key.repeat) continue;
                if (e.key.key == SDLK_ESCAPE) {
                    if (!checkpoints_open) return std::nullopt;
                    checkpoints_open = false;
                    wait_release = true;
                } else if (e.key.key == SDLK_F6 && checkpoints_available) {
                    checkpoints_open = !checkpoints_open;
                    wait_release = true;
                }
            }
            PadState s = drive_menu_pad(gamepad);
            if (wait_release) {
                if (s.buttons == 0) wait_release = false;
                hist.push({});
            } else {
                hist.push(s);
            }
            if (checkpoints_available && hist.pressed(kPadSelect)) checkpoints_open = !checkpoints_open;
            else if (checkpoints_open) {
                if (hist.pressed(kPadUp)) checkpoint_row = (checkpoint_row + count - 1) % count;
                if (hist.pressed(kPadDown)) checkpoint_row = (checkpoint_row + 1) % count;
                if (hist.pressed(kPadCircle)) checkpoints_open = false;
                else if (hist.pressed(kPadCross))
                    return DriveResult{DriveExit::Restart, false, options[checkpoint_row].checkpoint};
            } else menu.update(hist);
            window.begin_frame(w, h);
            ui.begin(w, h);
            ui.draw_backdrop();
            ui::select_prompts(0, InputContext::Menu);
            menu.draw(ui, text);
            if (checkpoints_open) {
                constexpr float x = (ui::kScreenW - 364.0f) * 0.5f;
                ui.fill({x, 66, 364, 316}, {14, 24, 31, 255});
                text.draw(x + 24, 106, "Debug Checkpoints", ui::TextStyle{.font = 3});
                for (int i = 0; i < count; ++i) {
                    const float y = 146.0f + i * 30.0f;
                    if (i == checkpoint_row) ui.fill({x + 15, y - 20, 334, 27}, {42, 72, 86, 255});
                    text.draw(x + 24, y, options[i].label, ui::TextStyle{.font = 2});
                }
                text.draw(x + 24, 363, "~A Launch     ~X Back", ui::TextStyle{.font = 2});
            } else if (checkpoints_available) {
                const ui::TextStyle style{.font = 2, .align = ui::Align::Center};
                constexpr std::string_view label = "F6 / ~T Debug Checkpoints";
                const float width = text.measure(label, style).width + 20;
                const float centre = ui::kScreenW * 0.5f;
                ui.fill({centre - width * 0.5f, ui::kScreenH - 39, width, 26}, {14, 24, 31, 255});
                text.draw(centre, ui::kScreenH - 20, label, style);
            }
            ui.end();
            window.swap();
            SDL_Delay(33);
        }
        const FrontendResult& r = menu.result();
        if (r.action == FrontendResult::Action::Resume) return std::nullopt;
        if (r.action == FrontendResult::Action::RestartMission) return DriveResult{DriveExit::Restart};
        return DriveResult{DriveExit::QuitToMenu};
    }
};

DriveSessionApp::DriveSessionApp(AppContext& ctx, Window& window, ui::Renderer& ui, ui::TextRenderer& text,
                                 const std::string& level, const std::string& car, const AppConfig& cfg, bool mute,
                                 std::optional<driving::AlpineCheckpoint> checkpoint)
    : impl_(std::make_unique<Impl>(ctx, window, ui, text, level, car, cfg, mute, checkpoint)) {
    ready_ = impl_->build();
}

DriveSessionApp::~DriveSessionApp() = default;

DriveResult DriveSessionApp::run_interactive() {
    Impl& s = *impl_;
    const bool gunner = s.mission->alpine() != nullptr;
    const auto capture_mouse = [&](bool on) {
        if (gunner) SDL_SetWindowRelativeMouseMode(s.window.sdl(), on);
    };
    capture_mouse(true);
    s.start_audio();
    int count = 0;
    if (SDL_JoystickID* ids = SDL_GetGamepads(&count)) {
        if (count > 0) s.gamepad = SDL_OpenGamepad(ids[0]);
        SDL_free(ids);
    }
    bool running = true;
    DriveResult done{DriveExit::QuitToMenu};
    bool finished = false;
    int end_frames = 0;  // banner show time once the mission is decided
    double accumulator = 0;
    Uint64 last = SDL_GetTicksNS();
    constexpr double kStep = 1.0 / driving_app::kTickHz;
    float mouse_x = 0, mouse_y = 0;
    bool start_was_down = drive_pad(s.gamepad, gunner).held(kPadStart);
    auto pause = [&](bool checkpoints = false) {
        capture_mouse(false);
        if (s.drive_audio) s.drive_audio->set_paused(true);
        if (auto r = s.run_pause(s.gamepad, checkpoints)) {
            done = *r;
            finished = true;
        }
        if (!finished) {
            if (s.drive_audio) s.drive_audio->set_paused(false);
            capture_mouse(true);
        }
        mouse_x = mouse_y = 0;
        accumulator = 0;
        last = SDL_GetTicksNS();
    };
    while (running && !finished) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
            else if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE && !e.key.repeat) {
                pause();
            } else if (gunner && e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_F6 && !e.key.repeat) {
                pause(true);
            } else if (gunner && e.type == SDL_EVENT_MOUSE_MOTION && SDL_GetWindowRelativeMouseMode(s.window.sdl())) {
                mouse_x += e.motion.xrel;
                mouse_y += e.motion.yrel;
            }
        }
        if (finished || !running) break;
        const Uint64 now = SDL_GetTicksNS();
        accumulator = std::min(accumulator + double(now - last) * 1e-9, 0.25);
        last = now;
        while (accumulator >= kStep) {
            PadState pad = drive_pad(s.gamepad, gunner);
            const bool start = pad.held(kPadStart);
            if (start && !start_was_down) {
                start_was_down = true;
                pause();
                break;
            }
            start_was_down = start;
            if (gunner) {
                pad.mouse = mouse_look_from_motion(mouse_x, mouse_y, s.config.mouse_look_sensitivity / 100.0f);
                mouse_x = mouse_y = 0;
            }
            s.tick(pad);
            accumulator -= kStep;
            if (s.mission->state() != driving_app::MissionState::Running) {
                // Let the banner show for 3 seconds, then fall through to the results.
                if (++end_frames > 180) {
                    done = DriveResult{DriveExit::QuitToMenu, s.mission->state() == driving_app::MissionState::Won};
                    finished = true;
                    break;
                }
            }
        }
        if (finished) break;
        s.draw_frame();
        s.window.swap();
    }
    capture_mouse(false);
    s.drive_audio.reset();
    if (s.gamepad) SDL_CloseGamepad(s.gamepad);
    if (done.exit == DriveExit::Restart) {
        const auto checkpoint = done.checkpoint ? done.checkpoint : s.checkpoint;
        auto restarted = std::make_unique<Impl>(s.ctx, s.window, s.ui, s.text,
            s.level_name, s.car_name, s.config, s.mute, checkpoint);
        ready_ = restarted->build();
        if (ready_) impl_ = std::move(restarted);
        else done.exit = DriveExit::QuitToMenu;
    }
    return done;
}

DriveResult DriveSessionApp::run_headless(const DriveHeadless& headless) {
    Impl& s = *impl_;
    std::vector<PadState> script;
    if (!headless.inputs.empty()) {
        std::ifstream f(headless.inputs);
        if (!f) throw std::runtime_error("cannot open " + headless.inputs);
        std::stringstream ss;
        ss << f.rdbuf();
        script = driving_app::parse_input_script(ss.str());
    }
    for (long t = 0; t < headless.frames; ++t) {
        PadState pad;
        if (std::size_t(t) < script.size()) pad = script[std::size_t(t)];
        else pad.buttons |= kPadCross;  // straight gas
        s.tick(pad);
    }
    const driving_app::DrivingHud& hud = s.mission->hud();
    const Vec3 p = s.mission->session().player_position();
    std::printf("drive %s: %ld ticks, pos %.1f,%.1f,%.1f %s banner '%s'\n", s.level_name.c_str(), headless.frames, p[0],
                p[1], p[2], hud.speed_text().c_str(), hud.banner.c_str());
    if (const auto* alpine = s.mission->alpine())
        std::printf("JL-7 %d rounds, lock %s, opening checkpoint %s\n", alpine->ammo(),
                    alpine->locked() ? "yes" : "no", alpine->checkpoint() ? "yes" : "no");
    if (!headless.shot.empty()) {
        s.draw_frame();
        const bool ok = s.window.save_bmp(headless.shot);
        std::printf("shot -> %s\n", ok ? headless.shot.c_str() : SDL_GetError());
        if (!ok) throw std::runtime_error("cannot write shot");
    }
    return DriveResult{DriveExit::QuitToMenu, s.mission->state() == driving_app::MissionState::Won};
}

}  // namespace nf::app
