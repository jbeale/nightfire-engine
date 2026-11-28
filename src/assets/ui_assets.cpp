#include "assets/ui_assets.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include "assets/bin_archive.hpp"

namespace nf {
namespace {

void load_local_font_bakes(UiAssets& ui, const std::filesystem::path& gamedir) {
    const char* override_dir = std::getenv("NF_FONT_ATLAS_DIR");
    if (override_dir && !*override_dir) return;
    const std::filesystem::path dir = override_dir ? std::filesystem::path(override_dir) : gamedir / "font-atlases";
    for (auto [number, filename] : {std::pair{2, "conduit-2.alpha"}, std::pair{3, "conduit-3.alpha"}}) {
        Font& font = ui.fonts.fonts[std::size_t(number - 1)];
        const Texture* stock = ui.sprites.find(font.texture_hash);
        const auto path = dir / filename;
        if (!stock || !std::filesystem::is_regular_file(path)) continue;
        Texture atlas{stock->flags, stock->width * kFontBakeScale, stock->height * kFontBakeScale, 1, 1, {}};
        auto coverage = read_file(path);
        if (coverage.size() != std::size_t(atlas.width) * atlas.height) {
            std::fprintf(stderr, "Ignoring font bake with incorrect dimensions: %s\n", path.string().c_str());
            continue;
        }
        atlas.rgba.reserve(coverage.size());
        for (std::uint32_t alpha : coverage) atlas.rgba.push_back(0x00FFFFFFu | (alpha << 24));
        // Level chunks can register the original texture again. Keep the bake's hash distinct.
        font.texture_hash = 0x7E000000u + std::uint32_t(number);
        font.raster_scale = kFontBakeScale;
        ui.sprites.add(font.texture_hash, std::move(atlas));
    }
}

// psiDecompressWoman composites a sliding window of the source texture through
// the mask in the level's Woman entry, then uploads it as the pause sprite.
void add_pause_animation(SpriteLibrary& sprites, Bytes mask) {
    constexpr std::uint32_t kSource = 0x030000DA, kSprite = 0x030000DB;
    constexpr std::size_t kWidth = 128, kFrameSize = kWidth * kWidth;
    constexpr std::uint32_t kFrames = 348;
    const Texture* source = sprites.find(kSource);
    const Texture* sprite = sprites.find(kSprite);
    if (!source || !sprite || source->width != kWidth || source->height != kWidth * 2) return;

    Texture animation{sprite->flags, kWidth, kWidth, kFrames, 2, {}};
    animation.rgba.resize(kFrameSize * kFrames, 0xFF000000u);
    std::size_t cursor = 0;
    const auto read_count = [&] {
        const Bytes count = slice(mask, cursor, 2);
        cursor += 2;
        return std::size_t(count[0]) * 256 + count[1];
    };
    for (std::size_t frame = 0; frame < kFrames; ++frame) {
        const std::size_t offset = (frame % 64) * 256 + kWidth - frame % 128;
        auto dst = animation.rgba.begin() + frame * kFrameSize;
        auto pixels = source->rgba.begin() + offset;
        std::size_t pos = 0;
        while (pos < kFrameSize) {
            pos += read_count();  // long black run
            if (pos == kFrameSize) break;
            if (pos > kFrameSize) throw FormatError("pause animation run exceeds frame");
            const std::size_t count = read_count();
            const Bytes runs = slice(mask, cursor, count);
            cursor += count;
            for (std::size_t i = 0; i < runs.size(); ++i) {
                const std::size_t length = runs[i];
                if (length > kFrameSize - pos) throw FormatError("pause animation run exceeds frame");
                if (i % 2 == 0) std::copy_n(pixels + pos, length, dst + pos);
                pos += length;
            }
        }
    }
    sprites.add(kSprite, std::move(animation));
}

}  // namespace

void add_level_sprites(SpriteLibrary& sprites, Bytes bin) {
    Bytes pause_mask;
    for (const auto& entry : parse_bin_archive(bin)) {
        if (is_map_chunk_file(entry.type)) sprites.add(parse_map_chunk(entry.data));
        if (entry.type == EntryType::Woman) pause_mask = entry.data;
    }
    if (!pause_mask.empty()) add_pause_animation(sprites, pause_mask);
}

UiAssets load_ui_assets(const std::filesystem::path& gamedir, GameFiles& files, std::string_view language_file) {
    UiAssets ui;
    ui.fonts = load_fonts(Elf32(read_file(gamedir / "ACTION.ELF")));

    const GameFile* text = files.find(language_file);
    if (!text) throw FormatError("FILES.BIN has no " + std::string(language_file));
    auto text_data = files.read(*text);
    // The `...TxtU.dat` tables are the UTF-16 variants.
    bool wide = language_file.size() >= 5 && (language_file[language_file.size() - 5] | 0x20) == 'u';
    ui.strings = StringTable::parse(Bytes(text_data), wide);

    const GameFile* front = files.find(kFrontEndBin);
    if (!front) throw FormatError("FILES.BIN has no front end bin " + std::string(kFrontEndBin));
    auto bin = files.read(*front);
    add_level_sprites(ui.sprites, Bytes(bin));
    load_local_font_bakes(ui, gamedir);
    return ui;
}

}  // namespace nf
