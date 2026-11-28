#pragma once

#include <optional>
#include <string_view>
#include <vector>

#include "assets/anim.hpp"
#include "driving/alpine_checkpoint.hpp"
#include "driving/car_weapons.hpp"
#include "driving/drive_session.hpp"
#include "driving/mission_data.hpp"

namespace nf::driving {

// MIS3 opening encounter. Zoe follows the course while Bond controls the mounted
// weapon independently. The driver pose, gun aim and camera cuts share one tick.
class AlpineEscape {
public:
    static constexpr std::size_t kGarageDoorInstance = 3351;
    static constexpr std::size_t kExitDoorInstance = 3348;
    static constexpr std::size_t kIntactGarageInstance = 3345;
    static constexpr std::size_t kWreckGarageInstance = 3344;
    static constexpr std::size_t kBombInstance = 3392;
    static constexpr std::size_t kHillTokenInstance = 3333;
    static constexpr std::size_t kHillShackInstance = 3390;
    static constexpr Vec3 kHillToken{375.027f, 1.781f, -297.116f};
    static constexpr std::size_t kOtherShutters[] = {3347, 3349, 3350, 3352};
    static constexpr std::size_t kGrateInstance = 3342;
    static constexpr std::size_t kChaletDoorInstance = 3336;
    static constexpr std::size_t kChaletDoorExteriorInstance = 145;
    static constexpr std::size_t kChaletWindowInstances[] = {3338, 3340, 3341};
    static constexpr Vec3 kChaletDoor{1831.958f, -235.882f, 57.128f};
    static constexpr Vec3 kChaletWindow{1875.254f, -239.963f, 10.536f};
    static constexpr std::size_t kBridgeInstance = 3334;
    static constexpr std::size_t kRavineArmorInstance = 3672;
    static constexpr Vec3 kBridgeCenter{1744.490f, -171.5f, 239.904f};
    static constexpr std::size_t kGateLeaves[] = {3686, 3687};
    static constexpr Vec3 kGateCenter{1637.73f, -170.45f, 278.61f};
    static constexpr Vec3 kGratePosition{916.941f, -92.586f, 134.562f};
    static constexpr std::string_view kSpeech[] = {
        "sno_r_bond_1.asf", "sno_r_zoe_10a.asf", "sno_r_zoe_10b.asf", "chicken.asf", "gateclose.asf", "007logo.asf", "bridgeblow.asf", "chalet.asf",
    };

    struct Guard {
        Vec3 position{};
        Vec3 destination{};
        float appears_at = 0;
        float yaw = 0;
        float turn_rate = 0;
        float health = 36;
        float fire_at = 0;
        bool active = false;
        float step_distance = 0;
        bool sled = false;
        float lane = 0;
        float lead = 0;
        bool forest = false;
        Vec3 fall_velocity{};
        float fall_angle = 0;
        float fall_spin = 0;
        bool landed = false;
        Vec3 sled_wreck{};
        float sled_drift = 0;
        float sled_roll = 0;
    };

    struct Barrel {
        Vec3 center;
        bool destroyed = false;
    };

    struct Blast {
        Vec3 center;
        float scale;
    };

    AlpineEscape(DriveSession& session, const CarpFile& carp, const MissionData& data, const TrackRenderExtras& extras,
                 AlpineCheckpoint checkpoint = AlpineCheckpoint::Garage);

    // Returns incoming damage this tick. Sound events use the common driving mixer.
    float tick(const PadState& pad, DriveSession& session, std::vector<SoundEvent>& sounds);
    const CameraPose& camera() const { return camera_; }
    const std::vector<Guard>& guards() const { return guards_; }
    const Vec3& shot_end() const { return shot_end_; }
    float shot_seconds() const { return shot_seconds_; }
    float door_lift() const { return door_lift_; }
    float exit_door_lift() const { return exit_door_lift_; }
    int ammo() const { return ammo_; }
    bool locked() const { return lock_.has_value(); }
    bool timer_cut() const { return departure_ >= kTimerCutStart && departure_ < kGarageDetonation; }
    bool garage_blast_cut() const { return departure_ >= kGarageDetonation && departure_ < kDepartureCutEnd; }
    bool garage_destroyed() const { return garage_detonated_at_ >= 0; }
    float garage_blast_age() const { return clock_ - garage_detonated_at_; }
    bool token_visible() const { return token_revealed_ && !token_collected_; }
    bool token_collected() const { return token_collected_; }
    bool hill_shack_destroyed() const { return token_revealed_; }
    float token_notice() const { return token_notice_; }
    float clock() const { return clock_; }
    int timer_digit() const;
    const std::vector<Barrel>& barrels() const { return barrels_; }
    const std::vector<Blast>& blasts() const { return blasts_; }
    bool exterior() const {
        return timer_cut() || garage_blast_cut() || stage_ == Stage::RoadCut ||
               stage_ == Stage::HeadOn || stage_ == Stage::GateCut || stage_ == Stage::BridgeCut ||
               stage_ == Stage::ChaletExit;
    }
    bool checkpoint() const { return passed_opening_; }
    bool in_chalet() const { return stage_ == Stage::ChaletInterior; }
    bool chalet_chase() const { return stage_ == Stage::ChaletChase; }
    bool chalet_door_broken() const { return chalet_door_at_ >= 0; }
    bool chalet_window_broken() const { return chalet_window_at_ >= 0; }
    float chalet_door_age() const { return clock_ - chalet_door_at_; }
    float chalet_window_age() const { return clock_ - chalet_window_at_; }
    bool trail_end() const { return stage_ == Stage::TrailEnd; }
    bool grate_broken() const { return grate_broken_; }
    bool gate_waiting() const { return stage_ == Stage::GateHold; }
    bool gate_destroyed() const { return gate_destroyed_; }
    bool bridge_destroyed() const { return bridge_destroyed_at_ >= 0; }
    float bridge_blast_age() const { return clock_ - bridge_destroyed_at_; }
    bool pursuing_helicopter() const { return stage_ == Stage::Ravine; }
    bool show_ravine_objective() const { return pursuing_helicopter() && stage_time_ < 5.5f; }
    const std::optional<Mat4>& helicopter() const { return helicopter_; }
    std::optional<Vec3> bridge_rocket() const;
    std::optional<Vec3> helicopter_round() const;
    bool ravine_armor_collected() const { return ravine_armor_collected_; }
    float gate_closure() const { return gate_closure_; }
    bool grenade_selected() const { return grenade_selected_; }
    int grenade_ammo() const { return grenade_ammo_; }
    const Projectile& grenade() const { return grenade_; }
    bool armor() const { return armor_; }
    float armor_points() const { return armor_points_; }
    bool took_damage() const { return took_damage_; }
    const char* objective() const;

private:
    static constexpr float kTimerCutStart = 0.2f;
    static constexpr float kTimerZero = 1.75f;
    static constexpr float kGarageDetonation = 1.97f;
    static constexpr float kDepartureCutEnd = 3.55f;

    enum class Stage {
        Garage, Escape, RoadCut, Chase, Pursuit, HeadOn, ForestRun, GateCut,
        GateApproach, GateHold, GateClear, BridgeCut, Ravine, ChaletInterior, ChaletExit, ChaletChase, TrailEnd
    };

    struct ScenePath {
        struct PositionKey { float frame; Vec3 value; };
        struct RotationKey { float frame; nf::Quat value; };
        std::vector<PositionKey> positions;
        std::vector<RotationKey> rotations;
        static ScenePath load(const CarpFile& carp, std::string_view name);
        Mat4 pose(float frame) const;
        float end_frame() const { return positions.back().frame; }
    };
    void start_at(DriveSession& session, AlpineCheckpoint checkpoint);
    void begin_chalet_chase(std::size_t route_point);
    void advance_bridge(DriveSession& session);
    void advance_chalet(DriveSession& session);
    void follow_scene_sled(DriveSession& session, const Mat4& pose, float height_offset);
    void update_helicopter();
    float tick_helicopter_attack(const DriveSession& session, std::vector<SoundEvent>& sounds);

    void advance_driver(DriveSession& session);
    void follow_ground(DriveSession& session, Vec3 position);
    void update_camera(const DriveSession& session);
    float tick_guards(const DriveSession& session, std::vector<SoundEvent>& sounds);
    void place_sled(Guard& guard, const DriveSession& session, float distance, float lane, bool oncoming, bool was_active);
    void tick_gun(const PadState& pad, const DriveSession& session, std::vector<SoundEvent>& sounds);
    void update_grenade(const DriveSession& session, std::vector<SoundEvent>& sounds);
    bool visible(const DriveSession& session, const Vec3& target) const;
    Barrel* barrel_hit(const Vec3& origin, const Vec3& direction, float& nearest);
    void damage_guard(Guard& guard, float damage, const Vec3& impulse, std::vector<SoundEvent>& sounds);
    void advance_fall(Guard& guard, const DriveSession& session);
    void explode(const Vec3& center, float scale, float radius, std::string_view sound,
                 const DriveSession& session, std::vector<SoundEvent>& sounds);
    void detonate(Barrel& barrel, const DriveSession& session, std::vector<SoundEvent>& sounds);

    std::vector<Barrel> barrels_;
    std::vector<Blast> blasts_;
    std::vector<Vec3> bomb_camera_;
    float departure_ = 0, garage_detonated_at_ = -1, token_notice_ = 0;
    bool token_revealed_ = false, token_collected_ = false;

    std::vector<Vec3> escape_path_, pickup_path_, chase_path_, head_on_camera_, gate_camera_;
    std::vector<float> chase_distances_;
    std::vector<Guard> guards_;
    Stage stage_ = Stage::Garage;
    CameraPose camera_{};
    float clock_ = 0, stage_time_ = 0;
    float door_lift_ = 0, exit_door_lift_ = 0;
    float speed_ = 0, heading_ = 1.079f;
    float turn_rate_ = 0;
    float course_y_, heave_speed_ = 0;
    float chase_distance_ = 0;
    std::size_t opening_end_ = 0;
    std::size_t oncoming_rider_ = 0;
    std::size_t forest_checkpoint_ = 0;
    std::size_t gate_stop_ = 0;
    std::size_t bridge_entry_ = 0;
    ScenePath bridge_camera_, bridge_sled_, bridge_heli_, ravine_heli_;
    ScenePath chalet_interior_, chalet_exit_sled_, chalet_exit_camera_, chalet_reverse_camera_;
    ScenePath chalet_pursuer_path_, chalet_heli_, post_chalet_heli_;
    std::size_t chalet_entry_ = 0, chalet_rejoin_ = 0, chalet_pursuer_ = 0;
    float chalet_door_at_ = -1, chalet_window_at_ = -1;
    std::optional<Mat4> helicopter_;
    float bridge_destroyed_at_ = -1;
    float helicopter_attack_at_ = 0;
    struct Strafe {
        Vec3 muzzle, impact;
        float starts_at;
    };
    std::optional<Strafe> strafe_;
    bool ravine_armor_collected_ = false;
    bool passed_opening_ = false;
    bool grate_broken_ = false;
    bool gate_destroyed_ = false, grenade_selected_ = false, switch_was_down_ = false;
    bool collected_grenades_ = false;
    float gate_closure_ = 0;
    int grenade_ammo_ = 0;
    Projectile grenade_{};
    Vec3 sled_up_{0, 1, 0};
    std::size_t waypoint_ = 0;
    float yaw_ = 0, pitch_ = 0;
    std::optional<std::size_t> lock_;
    int ammo_ = 3000;
    float next_shot_ = 0, shot_seconds_ = 0;
    bool armor_ = false;
    float armor_points_ = 0;
    bool took_damage_ = false;
    Vec3 shot_end_{};
};

}  // namespace nf::driving
