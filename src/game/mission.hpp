#pragma once

// Single-player mission flow: `Mission_Init` / `Mission_Update` / `Mission_MonitorObjectives`
// (ACTION.ELF 0x182270/0x182d70/0x182f30), the `P_ENDMISSION` menu-page side, per-level loadouts
// (`Player_InitWeapon`, docs/spec-weapons.md §10), inventory carry (`Player_RamSave`/`RamLoad`),
// and ownership of the dynamic objects (`game/objects.hpp`) plus the cutscene players
// (`game/script_player.hpp`). Behavioural notes:
//   * objectives watch switch channels exactly like the monitor: state 0 announces when the level
//     is reached, state 1 waits for its second channel, state 2 fails the mission when its
//     channel clears, state 3 completes when its channel sets; all channels set ends the mission.
//   * success/failure collapse the original's InternalState 2/3/4/6/7 pages into timers: the
//     mission reports `Succeeded`/`Failed` with the same messages, music events (6/7/8) and
//     destinations, and the frontend (nfgame/NIS/menu) presents them.
//   * there are no mid-level checkpoints on the disc: `RamSave` only runs at level transitions
//     (script event 18), carried as a weapon/ammo/armour snapshot [INFERENCE: BLData carries more].
//   * drones come from the Bots slice (`SpSystem`): placed NPCs spawn through it, `Drone_EnableAll`
//     maps onto `enable_drone`, and coder-spawns queue raw requests for a Bots-owned hook.

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "assets/cutscene.hpp"
#include "assets/mission_data.hpp"
#include "game/object_world.hpp"
#include "game/objects.hpp"
#include "game/script_player.hpp"
#include "game/sp_common.hpp"

namespace nf {

namespace sp {
class SpSystem;
}
class WeaponSystem;
struct ImpactEvent;
class CharacterBank;
class CharacterInstance;
class StringTable;

class MissionSystem : public System, public CutscenePlayer::Host {
public:
    // Presentation hooks (all optional, null = logged and counted only). Sounds use SFX ids with
    // world positions; messages are Txt labels with Text_AddMsg types; music is Music_Event.
    struct Hooks {
        std::function<void(std::uint32_t, const Vec3&, bool)> sound;
        std::function<void(std::uint32_t, int, int)> message;  // (label, frames, type)
        std::function<void(int, int)> music;                  // (event, value)
        std::function<void(float)> fade;                      // fullscreen black target/rate
        std::function<void(const std::array<float, 3>&, const std::uint32_t[4])> spawn_drone;
    };

    // Result of the mission state machine (`Mission_Update` InternalState).
    enum class State { Playing, Succeeded, Failed, Done };

    MissionSystem(Level& level, std::uint32_t level_id, const MissionEntry& entry, WeaponSystem& weapons,
                  sp::SpSystem* sp = nullptr);
    ~MissionSystem() override;

    void set_hooks(Hooks hooks) { hooks_ = std::move(hooks); }
    // Cutscene scripts of this level's .bin (type-7 entries by hash); empty = no NIS.
    void add_scripts(std::vector<std::pair<std::uint32_t, CutsceneBin>> scripts);

    // Object updates that must run BEFORE the players collide (movers for this tick).
    // `use` = per-slot Cross pressed; call once per tick before `World::tick`.
    void pre_tick(World& world, const std::vector<bool>& use, FrameTiming timing = FrameTiming{},
                  const WeaponEvents* previous_weapon_events = nullptr);

    void tick(World& world, FrameTiming timing) override;

    State state() const { return state_; }
    std::uint32_t fail_label() const { return fail_label_; }
    std::uint32_t pending_level() const { return pending_level_; }  // ResetMap_LevelToLoad target
    float fade() const { return fade_; }
    // Script-camera override for the renderer (eye + forward + fov), when a camera runs.
    std::optional<CutscenePlayer::Camera> script_camera() const;
    const CutscenePlayer* skippable_cutscene() const;
    void skip_cutscene(World& world, FrameTiming timing);
    void finish_checkpoint_scene(World& world, std::uint32_t hash, FrameTiming timing);
    std::vector<CutscenePlayer::ActorPose> script_actors() const;
    std::vector<CutscenePlayer::Sprite> script_sprites() const;
    const std::vector<Placement>& script_models() const { return script_models_; }
    void set_character_bank(CharacterBank* bank) { character_bank_ = bank; }
    // Queued presentation (also delivered through Hooks): sounds, texts, music, drone spawns.
    struct Sound : CutscenePlayer::Sound {
        std::size_t script_instance = 0;
    };
    struct Text {
        std::uint32_t label = 0;
        int frames = 180;
        int type = 2;
    };
    struct Music {
        int event = 0, value = 0;
    };
    struct Spawn {
        std::array<float, 3> pos{};
        std::uint32_t args[4]{};
    };
    struct Light {
        std::array<float, 3> pos{};
        std::uint8_t intensity = 0;
        bool type = false;
    };
    std::vector<Sound> take_sounds();
    std::vector<Text> take_texts();
    std::vector<Music> take_music();
    std::vector<Spawn> take_spawns();
    std::vector<Light> take_lights();
    std::optional<Vec3> take_copter_smoke();
    bool hit_copter_hull(const ImpactEvent& impact) const;
    // Channel transitions since the last call (frame, channel, value): poll-based watch for
    // tracing objective drivers headless (e.g. which tick sets ch70 on 07000005).
    struct ChannelEvent {
        std::uint64_t frame = 0;
        int channel = 0;
        bool value = false;
    };
    std::vector<ChannelEvent> take_channel_log();
    // Movers/draws/hides for the collision world and the renderer.
    const std::vector<Mover>& movers() const;
    const std::vector<SpObjects::DrawOverride>& draws() const;
    const std::vector<std::size_t>& hides() const;
    // Plays a level cutscene by hash now (pause-menu NIS requests, MoviePlayer fallbacks).
    void play_nis(std::uint32_t hash) { play_script(hash); }
    // Fails the mission now (`DroneFunc_SetMissionFailReason` via SpSystem::on_mission_fail).
    void fail_mission(std::uint32_t label);
    sp::SwitchChannels& channels() { return channels_; }
    SpObjects& objects() { return *objects_; }

    // Apply the level's start loadout (`Player_InitWeapon`); `carried` restores a RamSave first.
    struct Inventory {
        std::vector<std::pair<int, int>> weapons;  // (weapon id, rounds)
        std::vector<std::pair<int, int>> ammo;     // (ammo type, rounds)
        float armour = 0;
        int selected = 1;
        bool valid = false;
    };
    void apply_loadout(int slot);
    Inventory ram_save(int slot) const;
    void ram_load(World& world, int slot, const Inventory& inv);
    std::uint16_t exit_channel() const { return objects_->exit_channel(); }
    const Inventory& carried_inventory() const { return carried_inventory_; }

    // Mission stats (`PlrStat_*` essence): frames, shots, hits, kills (drones report via Bots).
    struct Stats {
        std::uint64_t frames = 0, shots = 0, hits = 0;
    };
    const Stats& stats() const { return stats_; }
    // Objectives for the pause OBJECTIVES tab and results (`Mission_MonitorObjectives` states).
    struct ObjectiveInfo {
        MissionObjective def;
        ObjectiveState state;
    };
    std::vector<ObjectiveInfo> objectives() const;
    // Drones killed this level (Bots' `SpSystem::stats.deaths`); feeds the results score line
    // together with stats() (frames/shots/hits) — the score formula itself is frontend-side.
    std::uint64_t kills() const;

    // CutscenePlayer::Host implementation (routes into the hooks/queues above).
    bool channel(std::uint16_t ch) const override;
    void set_channel(std::uint16_t ch, std::uint8_t value) override;
    void spawn_drone(const std::array<float, 3>& pos, const std::uint32_t args[4]) override;
    void enable_drones(bool enable) override;
    void break_near(const std::array<float, 3>& pos) override;
    void set_link_byte(const std::array<float, 3>& pos, std::uint8_t value) override;
    void load_level(std::uint32_t id) override;
    void set_scriptcam(std::uint32_t id) override;
    void camera_mode(std::uint32_t mode) override;
    void ram_save() override;
    void text(std::uint32_t label, std::uint16_t frames) override;
    void fade(float seconds) override;
    void music(std::uint32_t id, std::int32_t value) override;
    void message_callback(std::uint16_t arg, std::uint32_t data) override;

private:
    void monitor_objectives();
    void update_state(FrameTiming timing);
    CutscenePlayer* play_script(std::uint32_t hash, bool stage = false);
    void start_ready_scripts(World& world);
    void apply_script_locks(World& world);
    void finish_cutscene(World& world, CutscenePlayer& scene, FrameTiming timing);
    void update_moving_geometry(World& world);
    void update_script_models(World& world, const WeaponEvents& events);
    void update_escape_copter(World& world, const WeaponEvents& events, FrameTiming timing);

    Level& level_;
    std::uint32_t level_id_ = 0;
    MissionEntry entry_;
    WeaponSystem& weapons_;
    sp::SpSystem* sp_ = nullptr;
    Hooks hooks_;
    sp::SwitchChannels channels_;
    std::unique_ptr<SpObjects> objects_;
    struct Objective {
        MissionObjective def;
        ObjectiveState state = ObjectiveState::Announce;
    };
    std::vector<Objective> objectives_;
    State state_ = State::Playing;
    std::uint32_t fail_label_ = 0;
    std::uint32_t pending_level_ = 0;
    Inventory carried_inventory_;
    float state_timer_ = 0;  // 60 Hz frames left in Succeeded/Failed before Done
    float fade_ = 0, fade_target_ = 0, fade_rate_ = 0;
    std::uint32_t scriptcam_ = 0;
    bool player_disabled_ = false;
    bool drones_disabled_ = false;
    bool spotlight_contact_ = false;
    std::vector<int> suspended_drones_;
    std::optional<CutscenePlayer::PlayerHandoff> player_handoff_;
    bool fast_forwarding_ = false;
    std::vector<std::pair<std::uint32_t, CutsceneBin>> scripts_;
    std::vector<std::unique_ptr<CutscenePlayer>> players_;
    CharacterBank* character_bank_ = nullptr;
    std::unique_ptr<CharacterInstance> truck_;
    std::optional<CutscenePlayer::ActorPose> parked_truck_pose_;
    std::optional<std::size_t> truck_handle_;
    std::optional<std::size_t> gondola_floor_handle_;
    CollisionWorld* moving_geometry_ = nullptr;
    std::vector<Placement> script_models_;
    struct ScriptCollider {
        std::size_t handle;
        bool active = false;
        std::optional<float> health;
    };
    std::map<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>, ScriptCollider> script_colliders_;
    std::set<std::tuple<std::size_t, std::size_t, std::uint32_t>> collected_script_pickups_;
    struct EscapeCopter {
        std::size_t placement;
        std::uint16_t start, done, restart_key, path_duration;
        std::uint32_t flight_script, crash_script;
        PathTrack path;
        CutscenePlayer* flight = nullptr;
        struct Hull {
            std::size_t handle;
            Placement model;
        };
        std::optional<Hull> hull = std::nullopt;
        float clock = 0, health = 0, max_health = 0, smoke_wait = 0;
        float attack_wait = 240, burst_remaining = 0, shot_wait = 0;
        Mat4 pose = identity();
        bool destroyed = false;
    };
    std::optional<EscapeCopter> escape_copter_;
    std::optional<Vec3> copter_smoke_;
    std::vector<std::size_t> door_handles_;
    std::unordered_map<std::size_t, std::size_t> breakable_models_;
    std::vector<Sound> sounds_;
    std::vector<Text> texts_;
    std::vector<Music> music_;
    std::vector<Spawn> spawns_;
    std::vector<Light> lights_;
    std::vector<ChannelEvent> channel_log_;
    std::array<std::uint8_t, 256> channel_watch_{};
    bool watch_init_ = false;
    Stats stats_;
    std::uint64_t frame_ = 0;
};

}  // namespace nf
