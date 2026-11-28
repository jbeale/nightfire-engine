// Single-player ACTION session of the `nightfire` app: the mission's level
// with the player, weapons, placed NPCs (SpSystem over DroneSystem), HUD,
// audio (SFX + music) and the pause / end-mission pages. A scripted exit can
// return the next section and the inventory to carry into it.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "app/app.hpp"
#include "game/mission.hpp"
#include "game/plr_stats.hpp"
#include "render/window.hpp"
#include "ui/frontend.hpp"
#include "ui/hud.hpp"
#include "ui/renderer.hpp"
#include "ui/text.hpp"

namespace nf::audio { class AudioSystem; }

namespace nf::app {

struct SpLaunch {
    std::string bin;      // FILES.BIN level name, e.g. "07000001.bin"
    int difficulty = 2;   // GameState+0x28: 1 easy, 2 normal, 3 hard
    // Debug channel presets (nfgame --channel): applied to the mission switch channels at start.
    std::vector<std::pair<int, int>> channels;
    int give = -1;  // debug equip (nightfire --give ID): give + select at start (< 0 = loadout only)
    MissionSystem::Inventory inventory{};
    std::uint16_t entry_channel = 0;
    std::uint32_t bond_count = 0;
    std::string checkpoint{};
    PlrStats stats{};
    std::uint64_t play_frames = 0;
};

std::optional<SpLaunch> checkpoint_launch(std::string_view key, int difficulty);

enum class SpExit {
    QuitToMenu,   // pause quit, P_ENDMISSION quit, or window close
    Restart,      // pause restart or P_ENDMISSION retry
    NextMission,  // another map within this mission
    MissionSelect,
};

struct SpResult {
    SpExit exit = SpExit::QuitToMenu;
    bool failed = false;  // death or the mission-fail hook fired
    bool won = false;     // MissionSystem reported Succeeded
    std::uint32_t next_level = 0;  // sp_level id after a win (0x070000xx ACTION, 0x0900000x driving, 0 = frontend)
    MissionSystem::Inventory inventory{};
    std::uint16_t entry_channel = 0;
    std::uint32_t bond_count = 0;
    std::string checkpoint{};
    PlrStats stats{};
    std::uint64_t play_frames = 0;
};

struct SpHeadless {
    long frames = 300;          // scripted ticks with an idle pad
    std::string shot;           // screenshot after the frames (empty = none)
    std::string inputs;         // nfgame --inputs replay file (empty = idle)
};

class SpSession {
public:
    SpSession(AppContext& ctx, Window& window, ui::Renderer& ui, ui::TextRenderer& text, const SpLaunch& launch,
              AppConfig& cfg, audio::AudioSystem* menu_audio = nullptr);
    ~SpSession();

    SpSession(const SpSession&) = delete;
    SpSession& operator=(const SpSession&) = delete;

    bool ready() const { return ready_; }

    // Blocking interactive loop (60/30 Hz ticks, HUD, pause menu). Returns why it ended.
    SpResult run_interactive();
    // Scripted run for verification: `headless.frames` ticks, then `shot`.
    SpResult run_headless(const SpHeadless& headless);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool ready_ = false;
};

}  // namespace nf::app
