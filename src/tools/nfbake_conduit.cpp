#include <CoreText/CoreText.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "assets/ui_assets.hpp"

namespace {

using namespace nf;
constexpr int kScale = int(kFontBakeScale);

UniChar unicode(std::uint16_t code) {
    // The stock labels and glyph table use the Windows-1252 byte values.
    constexpr UniChar extended[] = {
        0x20ac, 0, 0x201a, 0x0192, 0x201e, 0x2026, 0x2020, 0x2021, 0x02c6, 0x2030, 0x0160, 0x2039,
        0x0152, 0, 0x017d, 0, 0, 0x2018, 0x2019, 0x201c, 0x201d, 0x2022, 0x2013, 0x2014, 0x02dc,
        0x2122, 0x0161, 0x203a, 0x0153, 0, 0x017e, 0x0178,
    };
    return code >= 0x80 && code <= 0x9f ? extended[code - 0x80] : code;
}

struct Bake {
    std::vector<std::uint8_t> alpha;
    double error = 0;
    unsigned replaced = 0;
};

Bake bake(const Font& font, const Texture& stock, CTFontRef face, CGColorSpaceRef gray) {
    Bake result;
    const std::size_t stride = stock.width * kScale;
    result.alpha.resize(stride * stock.height * kScale);
    for (const Glyph& g : font.glyphs) {
        const auto stock_alpha = [&](int x, int y) { return stock.rgba[(g.v + y) * stock.width + g.u + x] >> 24; };
        int left = g.w, right = 0, top = g.h, bottom = 0;
        for (int y = 0; y < g.h; ++y) {
            for (int x = 0; x < g.w; ++x) {
                if (stock_alpha(x, y) < 64) continue;
                left = std::min(left, x);
                right = std::max(right, x + 1);
                top = std::min(top, y);
                bottom = std::max(bottom, y + 1);
            }
        }
        const UniChar letter = unicode(g.code);
        CGGlyph index = 0;
        CGPathRef outline = letter && CTFontGetGlyphsForCharacters(face, &letter, &index, 1)
                                ? CTFontCreatePathForGlyph(face, index, nullptr)
                                : nullptr;
        const CGRect bounds = outline ? CGPathGetPathBoundingBox(outline) : CGRectNull;
        const int width = g.w * kScale, height = g.h * kScale;
        std::vector<std::uint8_t> mask(std::size_t(width) * height);
        if (left < right && top < bottom && !CGRectIsEmpty(bounds)) {
            CGContextRef ctx = CGBitmapContextCreate(mask.data(), width, height, 8, width, gray, kCGImageAlphaNone);
            if (!ctx) {
                CGPathRelease(outline);
                throw std::runtime_error("could not allocate glyph bitmap");
            }
            // Fit the outline to the stock ink bounds inside the original cell. The layout also
            // retains the original bearings, advances and kerning independently of this fit.
            CGContextTranslateCTM(ctx, left * kScale, height - bottom * kScale);
            CGContextScaleCTM(ctx, (right - left) * kScale / bounds.size.width,
                             (bottom - top) * kScale / bounds.size.height);
            CGContextTranslateCTM(ctx, -bounds.origin.x, -bounds.origin.y);
            CGContextSetGrayFillColor(ctx, 1, 1);
            CGContextAddPath(ctx, outline);
            CGContextFillPath(ctx);
            CGContextRelease(ctx);
            ++result.replaced;
        } else {
            for (int y = 0; y < height; ++y)
                for (int x = 0; x < width; ++x)
                    mask[y * width + x] = std::uint8_t(stock_alpha(x / kScale, y / kScale));
        }
        if (outline) CGPathRelease(outline);

        // Area-average to the original resolution before scoring the weight against the stock mask.
        for (int y = 0; y < g.h; ++y) {
            for (int x = 0; x < g.w; ++x) {
                float sum = 0.0f;
                for (int dy = 0; dy < kScale; ++dy)
                    for (int dx = 0; dx < kScale; ++dx)
                        sum += mask[(y * kScale + dy) * width + x * kScale + dx];
                const float difference = sum / (kScale * kScale) - stock_alpha(x, y);
                result.error += difference * difference;
            }
        }
        for (int y = 0; y < height; ++y)
            std::copy_n(mask.data() + y * width, width,
                        result.alpha.data() + (g.v * kScale + y) * stride + g.u * kScale);
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) try {
    if (argc < 4) {
        std::fprintf(stderr, "Usage: nfbake-conduit GAMEDIR OUTPUT_DIR CONDUIT_OTF [CONDUIT_OTF ...]\n");
        return 1;
    }
    const std::filesystem::path gamedir = argv[1], output = argv[2];
    // Select stock assets even when baking again into the installed override directory.
    setenv("NF_FONT_ATLAS_DIR", "", 1);
    GameFiles files(gamedir);
    const auto ui = load_ui_assets(gamedir, files);
    std::filesystem::create_directories(output);
    CGColorSpaceRef gray = CGColorSpaceCreateDeviceGray();
    for (int number : {2, 3}) {
        const auto& font = ui.fonts.font(number);
        const auto* stock = ui.sprites.find(font.texture_hash);
        if (!stock) throw std::runtime_error("stock font atlas missing");
        Bake best;
        best.error = std::numeric_limits<double>::infinity();
        const char* winner = nullptr;
        for (int i = 3; i < argc; ++i) {
            CGDataProviderRef provider = CGDataProviderCreateWithFilename(argv[i]);
            if (!provider) throw std::runtime_error(std::string("cannot read font ") + argv[i]);
            CGFontRef outline = CGFontCreateWithDataProvider(provider);
            CGDataProviderRelease(provider);
            if (!outline) throw std::runtime_error(std::string("invalid font ") + argv[i]);
            CTFontRef face = CTFontCreateWithGraphicsFont(outline, 100, nullptr, nullptr);
            CGFontRelease(outline);
            Bake candidate = bake(font, *stock, face, gray);
            CFRelease(face);
            if (candidate.replaced > best.replaced ||
                (candidate.replaced == best.replaced && candidate.error < best.error)) {
                best = std::move(candidate);
                winner = argv[i];
            }
        }
        const auto path = output / ("conduit-" + std::to_string(number) + ".alpha");
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(best.alpha.data()), std::streamsize(best.alpha.size()));
        if (!file) throw std::runtime_error("cannot write " + path.string());
        std::printf("Font %d: %s (%u/%zu glyphs) -> %s\n", number, winner, best.replaced,
                    font.glyphs.size(), path.string().c_str());
    }
    CGColorSpaceRelease(gray);
    return 0;
} catch (const std::exception& ex) {
    std::fprintf(stderr, "nfbake-conduit: %s\n", ex.what());
    return 1;
}
