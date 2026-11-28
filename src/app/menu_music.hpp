#pragma once

#include "audio/audio.hpp"
#include "ui/menu_audio.hpp"

namespace nf::app {

inline void stop_menu_music(audio::AudioSystem* audio) {
    if (!audio) return;
    audio->remove_sfx(0, kMenuNightfireTheme);
    audio->remove_sfx(0, kMenuBondTheme);
}

inline void play_menu_music(audio::AudioSystem* audio, std::uint32_t theme, int volume) {
    if (!audio) return;
    audio->remove_sfx(0, theme == kMenuBondTheme ? kMenuNightfireTheme : kMenuBondTheme);
    if (!audio->is_sfx_playing(theme)) audio->play_sfx(theme, {.volume = volume});
}

}  // namespace nf::app
