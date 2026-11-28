#pragma once

// Cutscene playback: `Script_Run` (0x1a7db8) + `Script_Update` (0x1a7c60) + the `Script_*Start`
// payload handlers, reimplemented against `assets/cutscene.hpp`'s decoded commands. A player runs
// one `CutsceneBin` entry: stream times advance by 60 Hz frames, commands fire in order, cameras
// and entities interpolate the entry's KEYED_POSROT key array, and every side effect goes through
// `Host` (implemented by `game/mission.hpp`'s mission system) so this stays free of audio/UI/AI
// dependencies. Only the effects the level scripts use are modelled; the rest is noted per site.
//
// Cameras use the range in CameraStart. Entity streams use their directory's inclusive key
// range. Positions lerp or use Catmull-Rom, rotations slerp, and key times are 60 Hz frames.

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "assets/cutscene.hpp"
#include "core/math.hpp"

namespace nf {
class CharacterBank;
class CutscenePlayer {
public:
    // Side effects, mirroring the original call sites. Positions are world-space.
    class Host {
    public:
        virtual ~Host() = default;
        virtual bool channel(std::uint16_t ch) const = 0;
        virtual void set_channel(std::uint16_t ch, std::uint8_t value) = 0;
        virtual void spawn_drone(const std::array<float, 3>& pos, const std::uint32_t args[4]) = 0;
        virtual void enable_drones(bool enable) = 0;
        virtual void break_near(const std::array<float, 3>& pos) = 0;
        virtual void set_link_byte(const std::array<float, 3>& pos, std::uint8_t value) = 0;
        virtual void load_level(std::uint32_t id) = 0;
        virtual void set_scriptcam(std::uint32_t id) = 0;
        virtual void camera_mode(std::uint32_t mode) = 0;
        virtual void ram_save() = 0;
        virtual void text(std::uint32_t label, std::uint16_t frames) = 0;
        virtual void fade(float seconds) = 0;
        virtual void music(std::uint32_t id, std::int32_t value) = 0;
        virtual void message_callback(std::uint16_t arg, std::uint32_t data) = 0;
    };

    struct ActorPose {
        std::uint32_t skin = 0, script = 0;
        float frame = 1;
        Mat4 transform = identity();
    };
    enum class SoundAction { Start, Move, End };
    struct Sound {
        std::uint32_t id = 0;
        std::array<float, 3> pos{};
        bool positional = false;
        SoundAction action = SoundAction::Start;
        std::uint16_t stream = 0;
        ActorPose actor{};
        bool animation = false;
    };
    // `Script_LightStart` lights (`DynamicLights::create`): entity position, intensity byte [1],
    // type flag (byte [2] == 1); radius 255, colour constants, infinite life (Characters-2).
    struct Light {
        std::array<float, 3> pos{};
        std::uint8_t intensity = 0;
        bool type = false;
    };

    CutscenePlayer(const CutsceneBin* bin, std::uint32_t id, Host* host, const CharacterBank* bank = nullptr);

    std::uint32_t id() const { return id_; }
    void play(bool fast);  // `Script_Play`: fast = skip-timescale (`Script_FFwd`)
    void stop();           // `Script_Stop`
    bool playing() const { return playing_; }
    bool locks_player() const { return playing_ && locks_player_; }
    bool locks_drones() const { return playing_ && locks_drones_; }
    float time() const { return time_; }  // SCRIPTINFO+160, 60 Hz frames

    // Advances every stream by `frames` (60 Hz frames; one 30 Hz tick = 2.0).
    void tick(float frames);
    void finish(float frames);
    bool staged() const { return staged_; }
    void stage();
    void resume() { staged_ = false; }
    void set_origin(const Mat4& origin) { origin_ = origin; }

    // Script-camera override (`ScriptCam` + `Script_UpdateCamera`): eye and forward when a camera
    // stream is active, with the +32 lens value projected for the original 4:3 view.
    struct Camera {
        std::array<float, 3> eye{};
        std::array<float, 3> forward{0, 0, 1};
        float fov = 1.0471976f;  // vertical radians
    };
    std::optional<Camera> camera() const;

    // Current world transform of a script entity (created by EntityStart), for the renderer.
    struct EntityPose {
        std::uint32_t hash = 0;
        std::array<float, 3> pos{};
        std::array<float, 4> quat{0, 0, 0, 1};
        bool visible = true;
        std::uint8_t kind = 0;
        Mat4 transform{};
        std::size_t stream = 0;
        std::array<std::uint32_t, 7> params{};
    };
    std::vector<EntityPose> entities() const;
    struct Sprite {
        std::uint32_t hash = 0;
        std::uint16_t x, y, w, h, flags;
    };
    std::vector<Sprite> sprites() const;

    std::vector<ActorPose> actors() const;

    // Script_Free passes the saved matrix and mode to Player_Enable.
    struct PlayerHandoff {
        Vec3 position{};
        float yaw = 0;
        std::uint8_t mode = 0;
    };
    std::optional<PlayerHandoff> take_handoff();

    std::vector<Sound> take_sounds();
    std::vector<Light> take_lights();

private:
    struct Stream;
    void fire(const ScriptCommand& cmd, Stream& s);
    void interpolate(Stream& s);
    std::array<float, 3> key_pos(std::size_t i) const;
    std::array<float, 4> key_quat(std::size_t i) const;
    float key_time(std::size_t i) const;
    float key_aux(std::size_t i) const;  // +32 float (fov / aux channel)
    std::size_t key_count() const;
    void cue_sound(Stream& stream, SoundAction action);
    void animation_sounds(Stream& stream, float frame);
    Mat4 entity_transform(std::size_t stream, std::size_t depth = 0) const;
    ActorPose actor_pose(const Stream& stream) const;

    const CutsceneBin* bin_;
    std::uint32_t id_;
    Host* host_;
    const CharacterBank* bank_;
    struct Stream {
        std::vector<ScriptCommand> cmds;
        std::size_t cursor = 0;
        float time = 0, next = 0;  // +8 / +12
        bool done = false;
        // Entity state (+16/+32/+34/+36/+37 equivalents).
        bool has_entity = false;
        bool detached_entity = false;
        std::size_t entity_first = 0, entity_last = 0;
        std::uint32_t entity_hash = 0;
        std::uint8_t entity_kind = 0;
        std::array<std::uint32_t, 7> entity_params{};
        float entity_start = 0;  // stream time of EntityStart
        bool spline = false;     // +34 == 2
        int obj_ref = -1;        // +37: entity index other streams address (-1 none)
        std::array<float, 3> epos{};              // interpolated pose (`Script_GetInterp`)
        std::array<float, 4> equat{0, 0, 0, 1};  // (identity = rest pose before first tick)
        ActorPose actor;
        float last_actor_frame = 0;
        Sprite sprite{};
        std::unique_ptr<CutsceneBin> subscript_bin;
        std::unique_ptr<CutscenePlayer> subscript;
        std::uint32_t sound_id = 0;
        int sound_source = -1;
        // Camera state.
        bool has_camera = false;
        std::size_t key_begin = 0, key_end = 0;
        float fov = 60.0f;
        std::array<float, 3> eye{};
        std::array<float, 3> forward{0, 0, 1};
        bool camera_live = false;
    };
    std::vector<Stream> streams_;
    float time_ = 0;  // +160
    float end_time_ = 0;
    bool playing_ = false;
    bool staged_ = false;
    bool detach_entities_ = false;
    Mat4 origin_ = identity();
    unsigned depth_ = 0;
    bool locks_player_ = false;
    bool locks_drones_ = false;
    std::optional<PlayerHandoff> handoff_;
    std::vector<Sound> sounds_;
    std::vector<Light> lights_;
};

}  // namespace nf
