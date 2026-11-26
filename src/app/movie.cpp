// Fullscreen PSS playback: video pacing, PCM audio, skip + headless shots.
#include "app/movie.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cstdio>
#include <limits>

#include "audio/audio.hpp"
#include "game/input.hpp"
#include "media/movie_player.hpp"

#include "ui/layout.hpp"
namespace nf::app {

struct MovieScreen::Impl {
    Impl(Window& w, ui::Renderer& u, ui::TextRenderer& t, audio::AudioSystem* m, SDL_Gamepad* g, std::string dir)
        : window(w), ui(u), text(t), music(m), gamepad(g), gamedir(std::move(dir)) {}

    Window& window;
    ui::Renderer& ui;
    ui::TextRenderer& text;
    audio::AudioSystem* music;
    SDL_Gamepad* gamepad;
    std::string gamedir;

    static std::string path_for(const std::string& gamedir, std::uint32_t id) {
        char name[16];
        std::snprintf(name, sizeof(name), "%08X.PSS", id);
        return gamedir + "/MOVIES/30_FPS/" + name;
    }

    PadState menu_pad() {
        PadState s;
        const bool* k = SDL_GetKeyboardState(nullptr);
        if (k[SDL_SCANCODE_Z] || k[SDL_SCANCODE_RETURN] || k[SDL_SCANCODE_SPACE]) s.buttons |= kPadCross;
        if (k[SDL_SCANCODE_X] || k[SDL_SCANCODE_ESCAPE]) s.buttons |= kPadCircle;
        if (k[SDL_SCANCODE_BACKSPACE]) s.buttons |= kPadSelect;
        if (gamepad) {
            if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_SOUTH)) s.buttons |= kPadCross;
            if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_EAST)) s.buttons |= kPadCircle;
            if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_START)) s.buttons |= kPadStart;
            if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_BACK)) s.buttons |= kPadSelect;
        }
        return s;
    }
};

MovieScreen::MovieScreen(Window& window, ui::Renderer& ui, ui::TextRenderer& text, audio::AudioSystem* music,
                         SDL_Gamepad* gamepad, const std::string& gamedir)
    : impl_(std::make_unique<Impl>(window, ui, text, music, gamepad, gamedir)) {}

MovieScreen::~MovieScreen() = default;

bool MovieScreen::play(std::uint32_t id, const std::string& shot, long max_frames) {
    Impl& s = *impl_;
    media::MoviePlayer player;
    const std::string path = Impl::path_for(s.gamedir, id);
    if (!player.open(path)) {
        std::printf("movie %08X: %s (fallback transition)\n", id, player.error().c_str());
        return false;
    }
    std::printf("movie %08X: %dx%d %.1fs%s\n", id, player.video().width, player.video().height,
                player.video().duration, player.audio().present ? " +audio" : "");
    if (s.music) s.music->pause_music(true);
    struct MusicResume {
        audio::AudioSystem* m;
        ~MusicResume() {
            if (m) m->pause_music(false);
        }
    } resume{s.music};

    media::MovieFrame frame, pending;
    bool pending_valid = false, video_eof = false, done = false;
    long shown = 0;
    struct MovieAudio {
        SDL_AudioStream* stream = nullptr;
        bool initialized = false;
        ~MovieAudio() {
            if (stream) SDL_DestroyAudioStream(stream);
            if (initialized) SDL_QuitSubSystem(SDL_INIT_AUDIO);
        }
    } audio;
    const auto pcm = player.audio_pcm();
    const std::size_t audio_bytes = pcm.size_bytes();
    const SDL_AudioSpec spec{SDL_AUDIO_S16, player.audio().channels, player.audio().sample_rate};
    const double bytes_per_second = spec.freq * spec.channels * sizeof(std::int16_t);
    // Queue the decoded track before starting the device so video rendering never has to feed it.
    if (!pcm.empty() && shot.empty() && audio_bytes <= std::numeric_limits<int>::max()) {
        audio.initialized = SDL_InitSubSystem(SDL_INIT_AUDIO);
        if (audio.initialized)
            audio.stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, nullptr, nullptr);
        if (!audio.stream || !SDL_PutAudioStreamData(audio.stream, pcm.data(), int(audio_bytes)) ||
            !SDL_FlushAudioStream(audio.stream) || !SDL_ResumeAudioStreamDevice(audio.stream)) {
            std::printf("movie: audio output unavailable (%s)\n", SDL_GetError());
            if (audio.stream) SDL_DestroyAudioStream(audio.stream);
            audio.stream = nullptr;
        }
    }
    const Uint64 t0 = SDL_GetTicksNS();
    double wall_offset = 0.0;
    int previous_queued = int(audio.stream ? audio_bytes : 0);
    double last_audio_progress = 0.0;
    PadHistory hist;
    while (!done) {
        const double wall = double(SDL_GetTicksNS() - t0) * 1e-9;
        int queued = audio.stream ? SDL_GetAudioStreamQueued(audio.stream) : 0;
        if (queued != previous_queued) last_audio_progress = wall;
        previous_queued = queued;
        if (queued > 0 && wall - last_audio_progress > 2.0) {
            SDL_DestroyAudioStream(audio.stream);
            audio.stream = nullptr;
            queued = 0;
        }
        if (!shot.empty()) {
            // Headless: take frames in decode order (no clock), bounded by max_frames/EOF.
            if (player.next_video(frame)) {
                ++shown;
                if (max_frames >= 0 && shown >= max_frames) done = true;
            } else {
                video_eof = true;
                done = true;
            }
        } else {
            // Pace video by consumed audio. Continue from that clock on wall time after it drains.
            double clock = wall + wall_offset;
            if (queued > 0) {
                clock = (audio_bytes - std::size_t(queued)) / bytes_per_second;
                wall_offset = clock - wall;
            }
            for (;;) {
                if (!pending_valid) {
                    if (!player.next_video(pending)) {
                        video_eof = true;
                        break;
                    }
                    pending_valid = true;
                }
                if (pending.pts <= clock + 1e-3) {
                    frame = std::move(pending);
                    pending_valid = false;
                    ++shown;
                } else {
                    break;
                }
            }
            if (max_frames >= 0 && shown >= max_frames) done = true;
        }
        if (!shot.empty()) {
            if (video_eof || (max_frames >= 0 && shown >= max_frames)) done = true;
        } else {
            SDL_Event e;
            while (SDL_PollEvent(&e)) {
                if (e.type == SDL_EVENT_QUIT) return true;
            }
            hist.push(s.menu_pad());
            // Skip on fresh Cross/Circle/Start/Select presses, like the menu movie pages.
            if (hist.pressed(kPadCross) || hist.pressed(kPadCircle) || hist.pressed(kPadStart) ||
                hist.pressed(kPadSelect))
                done = true;
            if (video_eof && !pending_valid && queued <= 0) done = true;
        }
        int w, h;
        s.window.begin_frame(w, h);
        s.ui.begin(w, h);
        if (!frame.rgba.empty()) {
            const ui::Layout layout = ui::Layout::from_canvas_width(s.ui.canvas_width());
            float movie_w = layout.width(), movie_h = layout.height();
            const float movie_aspect = float(frame.width) / float(std::max(frame.height, 1)) * ui::kPssPixelAspect;
            if (layout.width() * ui::kScreenXScale / layout.height() > movie_aspect)
                movie_w = movie_h * movie_aspect / ui::kScreenXScale;
            else
                movie_h = movie_w * ui::kScreenXScale / movie_aspect;
            s.ui.draw_frame({ui::Layout::kDesignWidth * 0.5f - movie_w * 0.5f,
                             ui::Layout::kDesignHeight * 0.5f - movie_h * 0.5f, movie_w, movie_h},
                            frame.width, frame.height, frame.rgba.data());
        }
        s.ui.end();
        if (shot.empty() || !done) s.window.swap();
        if (shot.empty()) SDL_Delay(5);
    }
    if (!shot.empty()) {
        const bool ok = s.window.save_bmp(shot);
        std::printf("movie shot -> %s\n", ok ? shot.c_str() : SDL_GetError());
        if (!ok) throw std::runtime_error("cannot write shot");
    }
    return true;
}

}  // namespace nf::app
