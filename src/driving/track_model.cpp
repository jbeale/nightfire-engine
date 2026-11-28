#include "driving/track_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <unordered_map>

#include "assets/reader.hpp"
#include "assets/ssh_texture.hpp"
#include "driving/track_collision.hpp"

namespace nf::driving {
namespace {

constexpr float kTrackUvScale = 1.0f / 256.0f;     // V2-16 texture coordinates: 0x8000-biased fixed point
constexpr float kTrackPositionScale = 1.0f / 256.0f;
constexpr float kVehicleUvScale = 1.0f / 1024.0f;

std::uint32_t u32(const ElfImage& e, std::size_t off) { return load<std::uint32_t>(e.data, off); }
std::uint16_t u16(const ElfImage& e, std::size_t off) { return load<std::uint16_t>(e.data, off); }
float f32(Bytes b, std::size_t off) { return load<float>(b, off); }

struct Article {
    std::string name;
    std::map<int, std::string> part_symbol;   // `as` index -> model symbol
    Bytes animated_parts;                   // optional model-space instance table
    Bytes lamp_effects;
    std::map<int, Bytes> piece_keys;
    Vec3 bounds_min{}, bounds_max{};
    bool has_bounds = false, multipart = false;
};

struct ChainData {
    std::vector<MeshVertex> vertices;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> strips;  // [begin, end) vertex ranges
    std::string shape;
    bool alpha_test = false, translucent = false, triangle_list = false;
};

// The chain-object VIF stream (DMA `ret` tag at +8): decoded generically, unpack streams by kind.
struct ChainState {
    std::string shape;
    bool textured = true, alpha_test = false, translucent = false, triangle_list = false;
};

void update_primitive_state(ChainState& state, std::string_view symbol) {
    if (symbol.find("GeoPrimState:::STATE::") == std::string_view::npos) return;
    state.textured = symbol.find("texture=off") == std::string_view::npos;
    state.alpha_test = symbol.find("alphatest=on") != std::string_view::npos ||
                       symbol.find("texalpha=1bitA") != std::string_view::npos;
    state.translucent = !state.alpha_test && symbol.find("texalpha=deepA") != std::string_view::npos;
    state.triangle_list = symbol.find("primtype=trilist") != std::string_view::npos;
}

bool chain_fits_article(const ChainData& chain, const Article& article, bool mirror_depth = false) {
    return std::ranges::all_of(chain.vertices, [&](const MeshVertex& vertex) {
        for (int axis = 0; axis < 3; ++axis) {
            const float coordinate = mirror_depth && axis == 2 ? -vertex.pos[axis] : vertex.pos[axis];
            if (coordinate < article.bounds_min[axis] - 0.10f ||
                coordinate > article.bounds_max[axis] + 0.10f) return false;
        }
        return true;
    });
}

bool is_alpine_terrain(std::string_view name) {
    constexpr std::string_view labels[]{
        "Mount", "Moutain", "mount", "Cliff", "cliff", "Snow", "snow", "terrain_ground", "gulley",
    };
    return std::ranges::any_of(labels, [name](std::string_view label) {
        return name.find(label) != std::string_view::npos;
    });
}

void append_triangles(MeshBatch& batch, std::uint32_t first, const ChainData& chain) {
    for (const auto& [begin, end] : chain.strips) {
        const unsigned step = chain.triangle_list ? 3 : 1;
        for (std::uint32_t i = begin; i + 2 < end; i += step) {
            const bool odd = !chain.triangle_list && ((i - begin) & 1);
            batch.indices.push_back(first + i + (odd ? 1 : 0));
            batch.indices.push_back(first + i + (odd ? 0 : 1));
            batch.indices.push_back(first + i + 2);
        }
    }
}

// Skydome/celestial texture shapes (`.ssh` names) are drawn without fog.
bool is_sky_shape(const std::string& shape) {
    if (shape == "moon") return true;
    return shape.size() >= 3 && (shape[0] == 's' || shape[0] == 'S') && (shape[1] == 'k' || shape[1] == 'K') &&
           (shape[2] == 'y' || shape[2] == 'Y');
}

struct Decoder {
    const ElfImage& elf;
    const std::unordered_map<std::uint32_t, ChainState>& state_of;   // chain object -> bound material state
    float uv_scale;

    bool decode(std::uint32_t chain, ChainData& out) const {
        const std::size_t tag = chain + 8;
        const std::uint32_t qwc = u32(elf, tag) & 0xFFFF;
        std::size_t pos = tag + 8;
        const std::size_t end = tag + 16 + std::size_t(qwc) * 16;
        std::uint32_t n = 0;
        std::vector<std::uint8_t> bounds;
        std::vector<std::array<std::uint16_t, 3>> pos16;
        std::vector<std::array<float, 3>> posf;
        std::vector<std::array<std::uint16_t, 2>> uv16;
        std::vector<std::array<float, 2>> uvf;
        std::vector<std::array<std::uint8_t, 3>> col8;
        bool have_header = false;
        while (pos < end) {
            const std::uint32_t w = u32(elf, pos);
            const unsigned cmd = w >> 24, num = (w >> 16) & 0xFF;
            pos += 4;
            if (cmd >= 0x60) {
                const unsigned vn = (cmd >> 2) & 3, vl = cmd & 3;
                const std::size_t count = num ? num : 256;
                static const unsigned bits[4] = {32, 16, 8, 5};
                const std::size_t bytes = ((count * (vn + 1) * bits[vl] + 7) / 8 + 3) & ~std::size_t(3);
                const std::size_t at = pos;
                pos += bytes;
                if (vn == 3 && vl == 0 && !have_header) {          // V4-32 x1: vertex count
                    n = u32(elf, at);
                    have_header = true;
                } else if (vn == 3 && vl == 2 && count == 1 && bounds.size() < 12) {   // V4-8 x1: strip ends
                    for (int i = 0; i < 4; ++i) bounds.push_back(elf.data.at(at + i));
                } else if (vn == 3 && vl == 0 && count == n && uv_scale == kVehicleUvScale) {
                    // The full-precision car microprogram puts positions at
                    // VU address 0x22 and normalized ST at 0x7a. The V4 arrays
                    // between and after them carry normals and lighting data.
                    const unsigned address = w & 0x3ff;
                    if (address == 0x22)
                        for (std::size_t i = 0; i < count; ++i)
                            posf.push_back({f32(elf.data, at + 16 * i), f32(elf.data, at + 16 * i + 4),
                                            f32(elf.data, at + 16 * i + 8)});
                    else if (address == 0x7a)
                        for (std::size_t i = 0; i < count; ++i)
                            uvf.push_back({f32(elf.data, at + 16 * i), f32(elf.data, at + 16 * i + 4)});
                } else if (vn == 2 && vl == 1) {                   // V3-16 USN: biased 8.8 positions
                    for (std::size_t i = 0; i < count; ++i)
                        pos16.push_back({u16(elf, at + i * 6), u16(elf, at + i * 6 + 2), u16(elf, at + i * 6 + 4)});
                } else if (vn == 2 && vl == 0) {                   // V3-32: float positions (vehicles)
                    for (std::size_t i = 0; i < count; ++i)
                        posf.push_back({load<float>(elf.data, at + i * 12), load<float>(elf.data, at + i * 12 + 4),
                                        load<float>(elf.data, at + i * 12 + 8)});
                } else if (vn == 1 && vl == 1) {                   // V2-16: texture coordinates
                    for (std::size_t i = 0; i < count; ++i)
                        uv16.push_back({u16(elf, at + i * 4), u16(elf, at + i * 4 + 2)});
                } else if (vn == 2 && vl == 2) {                   // V3-8: vertex colours
                    for (std::size_t i = 0; i < count; ++i)
                        col8.push_back({elf.data.at(at + i * 3), elf.data.at(at + i * 3 + 1), elf.data.at(at + i * 3 + 2)});
                }
                continue;
            }
            switch (cmd) {
                case 0x20: pos += 4; break;         // STMASK
                case 0x30: case 0x31: pos += 16; break;  // STROW / STCOL
                case 0x4A: pos += (num ? num : 256) * 8; break;  // MPG
                default: break;
            }
        }
        const bool quantised = pos16.size() == n;
        const bool float_uv = uvf.size() == n;
        if (!have_header || n == 0 || (!quantised && posf.size() != n) || (!float_uv && uv16.size() != n)) return false;

        out.vertices.resize(n);
        for (std::uint32_t i = 0; i < n; ++i) {
            MeshVertex& v = out.vertices[i];
            for (int a = 0; a < 3; ++a)
                v.pos[a] = quantised ? (pos16[i][a] - 32768.0f) * kTrackPositionScale : posf[i][a];
            for (int a = 0; a < 2; ++a)
                v.uv[a] = float_uv ? uvf[i][a] : (uv16[i][a] - 32768.0f) * uv_scale;
            if (quantised && col8.size() == n) {
                auto c = [](unsigned b) { return std::min(255u, b * 2); };
                v.rgba = c(col8[i][0]) | c(col8[i][1]) << 8 | c(col8[i][2]) << 16 | 0xFF000000u;
            } else {
                v.rgba = 0xFFFFFFFFu;
            }
        }
        std::uint32_t begin = 0;
        for (std::uint8_t b : bounds) {
            if (b == 0) break;
            const std::uint32_t e = b / 3;
            if (e > begin && e < n) out.strips.emplace_back(begin, e), begin = e;
        }
        out.strips.emplace_back(begin, n);

        if (const auto it = state_of.find(chain); it != state_of.end()) {
            if (it->second.textured) out.shape = it->second.shape;
            out.alpha_test = it->second.alpha_test;
            out.translucent = it->second.translucent;
            out.triangle_list = it->second.triangle_list;
        }
        return true;
    }
};

}  // namespace

SceneMesh build_track_scene(const CarpFile& carp, const ElfImage& elf, std::span<DetachedTrackMesh> detached, TrackRenderExtras* extras) {
    const auto& entries = carp.entries();

    // Texture objects and per-relocation state symbols.
    std::unordered_map<std::uint32_t, std::string> shape_at;
    for (const TarBinding& b : tar_bindings(elf)) shape_at[b.data_offset] = b.shape_name;
    std::unordered_map<std::uint32_t, std::uint32_t> reloc_symbol;  // .data offset -> symbol (non-internal)
    for (const ElfReloc& r : elf.relocs)
        if (r.symbol != 1) reloc_symbol[r.offset] = r.symbol;
    std::unordered_map<std::string, std::uint32_t> model_symbol;
    for (const ElfSymbol& s : elf.symbols)
        if (s.defined && s.name.rfind("__Model:::", 0) == 0) model_symbol[s.name] = s.value;

    // Material state per chain. A chain's preamble (VIF packet between the chain's preamble pointer and its
    // GeoPrim object) only carries what changes: the bound TAR texture object and GeoPrimState symbol. The
    // draw lists replay chains in order, so a chain without its own binding keeps the previous one.
    std::unordered_map<std::uint32_t, ChainState> state_of;
    std::unordered_map<std::string, std::vector<std::uint32_t>> model_chains;
    {
        std::vector<std::uint32_t> lists;
        for (std::size_t o = 0; o + 12 <= elf.data.size(); o += 4)
            if (u32(elf, o) == 0xA0000000 && u32(elf, o + 4) == 0 && !elf.is_pointer(o)) {
                const std::uint32_t n = u32(elf, o + 8);
                if (n % 2 || n == 0 || n > 400 || o + 12 + std::size_t(n) * 4 > elf.data.size()) continue;
                bool ok = true;
                for (std::uint32_t k = 0; k < n / 2 && ok; ++k) ok = u32(elf, o + 12 + 8 * k) == 0xA000FFFF;
                if (ok) lists.push_back(static_cast<std::uint32_t>(o));
            }
        ChainState cur;
        for (std::uint32_t list : lists)
            for (std::uint32_t k = 0; k < u32(elf, list + 8) / 2; ++k) {
                const std::uint32_t chain = u32(elf, list + 12 + 8 * k + 4);
                if (std::size_t(chain) + 24 > elf.data.size() || !elf.is_pointer(chain)) continue;
                const std::uint32_t obj = u32(elf, chain);
                if (std::size_t(obj) + 24 > elf.data.size()) continue;
                const std::uint32_t pre = u32(elf, obj);
                for (std::size_t a = pre; a < obj && a + 4 <= elf.data.size(); a += 4) {
                    if (elf.is_pointer(a)) {
                        if (const auto it = shape_at.find(u32(elf, a)); it != shape_at.end()) cur.shape = it->second;
                    } else if (const auto r = reloc_symbol.find(static_cast<std::uint32_t>(a)); r != reloc_symbol.end()) {
                        update_primitive_state(cur, elf.symbols[r->second].name);
                    }
                }
                if (!state_of.try_emplace(chain, cur).second) continue;
                // GeoPrim names identify their render model. Adjacent DMA lists can belong to
                // unrelated articles, even when their model instances share one material table.
                const auto name_offset = u32(elf, obj + 0x14);
                if (name_offset >= elf.data.size()) continue;
                const auto name = load_cstr(elf.data, name_offset);
                constexpr std::string_view prefix = "__GPRenderMethod_";
                if (!name.starts_with(prefix) || !name.ends_with("_name")) continue;
                const auto suffix = name.rfind('_', name.size() - 6);
                if (suffix == std::string_view::npos || suffix <= prefix.size()) continue;
                model_chains["__Model:::" + std::string(name.substr(prefix.size(), suffix - prefix.size()))]
                    .push_back(chain);
            }
    }

    // Articles.
    std::vector<Article> articles;
    std::unordered_map<std::string, std::size_t> article_by_name;
    std::size_t world_head = SIZE_MAX;
    for (std::size_t h = 0; h < entries.size(); ++h) {
        if (!entries[h].is_head) continue;
        if (entries[h].tag == "Map ") world_head = h;
        if (entries[h].tag != "Arti") continue;
        Article a;
        std::map<int, const CarpEntry*> as, sr;
        for (std::size_t m = entries[h].first_member; m < entries[h].first_member + entries[h].count; ++m) {
            const CarpEntry& e = entries[m];
            if (e.tag == "Name") {
                const auto s = load_cstr(carp.payload(e), 0);
                a.name = std::string(s);
            } else if (e.tag == "Base" && e.size >= 44) {
                const auto base = carp.payload(e);
                for (int axis = 0; axis < 3; ++axis) {
                    a.bounds_min[axis] = f32(base, 16 + axis * 4);
                    a.bounds_max[axis] = f32(base, 32 + axis * 4);
                }
                a.has_bounds = true;
            } else if (e.tag == "as") as[e.index] = &e;
            else if (e.tag == "sr") sr[e.index] = &e;
            else if (e.tag == "in") {
                a.multipart |= e.count > 1;
                if (e.index == 1 && (!detached.empty() || extras)) a.animated_parts = carp.payload(e);
            } else if (extras && e.tag == "ef" && e.index == 1) a.lamp_effects = carp.payload(e);
        }
        if (extras && !detached.empty() && a.name == "BigBridgeExplosion")
            for (std::size_t m = entries[h].first_member; m < entries[h].first_member + entries[h].count; ++m)
                if (entries[m].tag == "aq") a.piece_keys[entries[m].index] = carp.payload(entries[m]);
        for (auto& [k, e] : as) {
            const Bytes p = carp.payload(*e);
            const int ref = load<std::uint16_t>(p, p.size() - 4);   // trailing {u16 idx, "sr"}
            const auto it = sr.find(ref);
            if (it == sr.end()) continue;
            std::string s(load_cstr(carp.payload(*it->second), 0));
            if (s.rfind("EAGL::", 0) == 0) a.part_symbol[k] = "__Model:::" + s.substr(6);
        }
        article_by_name[a.name] = articles.size();
        articles.push_back(std::move(a));
    }
    if (world_head == SIZE_MAX) throw FormatError("track has no world group");

    // World instances and the `CARP::<article>::{as  NNNN}` names their references resolve to.
    const CarpEntry* inst_table = nullptr;
    std::map<int, std::string> world_ref;
    // References resolve through the `sr` strings of the `CDat` group (whole-article names) and of `Map `.
    for (std::size_t head = 0; head < entries.size(); ++head) {
        if (!entries[head].is_head || (entries[head].tag != "CDat" && entries[head].tag != "Map ")) continue;
        for (std::size_t m = entries[head].first_member; m < entries[head].first_member + entries[head].count; ++m) {
            const CarpEntry& e = entries[m];
            if (e.tag == "in" && head == world_head && (!inst_table || e.count > inst_table->count)) inst_table = &e;   // the big per-instance array
            else if (e.tag == "sr") world_ref[e.index] = std::string(load_cstr(carp.payload(e), 0));
        }
    }
    if (!inst_table) throw FormatError("track world group has no instance table");
    const Bytes inst = carp.payload(*inst_table);

    const Decoder dec{elf, state_of, kTrackUvScale};
    if (extras) {
        // A terrain model's first geometry packet can carry the preceding
        // model's GeoPrim name. Use the order in the ELF and the two CARP
        // bounds to recover it before instancing the meshes.
        struct TerrainPacket {
            std::uint32_t offset;
            const std::string* model;
            const Article* article;
        };
        std::vector<TerrainPacket> ordered;
        for (const auto& [model, packets] : model_chains) {
            constexpr std::string_view prefix = "__Model:::";
            const auto end = model.find("::{as");
            const auto name = model.substr(prefix.size(), end - prefix.size());
            const auto article = article_by_name.find(name);
            const Article* terrain = nullptr;
            if (article != article_by_name.end() && is_alpine_terrain(name) && model.ends_with("::{as  0000}::Model[0].00")) {
                const auto& a = articles[article->second];
                if (a.has_bounds && !a.multipart) terrain = &a;
            }
            for (const auto packet : packets) ordered.push_back({packet, &model, terrain});
        }
        std::ranges::sort(ordered, {}, &TerrainPacket::offset);
        for (std::size_t i = 1; i < ordered.size(); ++i) {
            const auto& packet = ordered[i - 1];
            const auto& next = ordered[i];
            if (!packet.article || !next.article || packet.model == next.model) continue;
            ChainData geometry;
            if (!dec.decode(packet.offset, geometry) || !chain_fits_article(geometry, *next.article)) continue;
            if (chain_fits_article(geometry, *packet.article)) {
                // A rock packet can fit both boxes. Quantized vertices are
                // identical along strip boundaries inside the same model.
                const auto seam = [&](const TerrainPacket& owner) {
                    std::set<Vec3> shared;
                    for (const auto offset : model_chains.at(*owner.model)) {
                        ChainData other;
                        if (offset == packet.offset || !dec.decode(offset, other) ||
                            !chain_fits_article(other, *owner.article)) continue;
                        for (const auto& v : geometry.vertices) {
                            if (std::ranges::any_of(other.vertices, [&](const auto& w) {
                                    return std::ranges::equal(v.pos, w.pos);
                                })) shared.insert({v.pos[0], v.pos[1], v.pos[2]});
                        }
                    }
                    return shared.size();
                };
                const bool matches_next = seam(next) >= 3 && seam(packet) == 0;
                bool full_extent = true;
                for (int axis = 0; axis < 3; ++axis) {
                    const auto extent = std::ranges::minmax_element(geometry.vertices, {}, [axis](const auto& v) {
                        return v.pos[axis];
                    });
                    full_extent &= std::abs(extent.min->pos[axis] - next.article->bounds_min[axis]) < 0.10f &&
                                   std::abs(extent.max->pos[axis] - next.article->bounds_max[axis]) < 0.10f;
                }
                const bool empty_destination = full_extent && model_chains.at(*packet.model).size() > 1 &&
                    std::ranges::none_of(model_chains.at(*next.model), [&](std::uint32_t offset) {
                        ChainData other;
                        return dec.decode(offset, other) && chain_fits_article(other, *next.article);
                    });
                if (!matches_next && !empty_destination) continue;
            }

            std::erase(model_chains.at(*packet.model), packet.offset);
            model_chains.at(*next.model).insert(model_chains.at(*next.model).begin(), packet.offset);
            // A displaced packet can also inherit the previous texture.
            // The destination model header names its texture bindings.
            const auto sym = model_symbol.find(*next.model);
            if (sym == model_symbol.end() || !elf.is_pointer(sym->second)) continue;
            const auto data = u32(elf, sym->second);
            if (data + 24ull > elf.data.size()) continue;
            std::string_view first_texture;
            bool bound = false;
            std::string_view bindings(reinterpret_cast<const char*>(elf.data.data() + data), 24);
            while (!bindings.empty() && bindings.front() != '\0') {
                const auto end = bindings.find('\0');
                if (end == std::string_view::npos) break;
                const auto shape = bindings.substr(0, end);
                if (shape.size() == 4 && std::ranges::any_of(shape_at, [shape](const auto& binding) {
                        return binding.second == shape;
                    })) {
                    if (first_texture.empty()) first_texture = shape;
                    bound |= geometry.shape == shape;
                }
                bindings.remove_prefix(end + 1);
            }
            if (!bound && !first_texture.empty()) {
                ChainState& state = state_of.at(packet.offset);
                state.shape = first_texture;
                state.textured = true;
            }
        }

        // The first packet of these chalet models is named for the end of
        // the preceding list. Reassign it so it uses the floor's placement,
        // bounds and material instead of the preceding article's.
        constexpr std::pair<const char*, const char*> floor_packets[] = {
            {"CHAL_Walls", "CHAL_floor2_redo"},
            {"CHAL_floor4", "CHAL_floor3_redo"},
            {"CHAL_floor5", "CHAL_floor4"},
            {"CHAL_Patio", "CHAL_floor5"},
            {"CHAL_WallTall_fix", "CHAL_FloorTOP"},
        };
        for (const auto& [from, to] : floor_packets) {
            const auto source = article_by_name.find(from), target = article_by_name.find(to);
            if (source == article_by_name.end() || target == article_by_name.end()) continue;
            const Article& floor = articles[target->second];
            const Article& previous = articles[source->second];
            const auto source_model = previous.part_symbol.find(0), floor_model = floor.part_symbol.find(0);
            if (source_model == previous.part_symbol.end() || floor_model == floor.part_symbol.end()) continue;
            const auto list = model_chains.find(source_model->second);
            if (list == model_chains.end() || list->second.empty()) continue;
            const auto last = std::ranges::max_element(list->second);
            const auto packet = *last;
            ChainData geometry;
            if (!floor.has_bounds || !dec.decode(packet, geometry) || !chain_fits_article(geometry, floor)) continue;
            list->second.erase(last);
            ChainState& state = state_of.at(packet);
            state.shape = "chl2";
            state.textured = true;
            state.alpha_test = state.translucent = false;
            auto& floor_list = model_chains[floor_model->second];
            floor_list.insert(floor_list.begin(), packet);
        }
    }
    SceneMesh scene;
    scene.min = {1e30f, 1e30f, 1e30f};
    scene.max = {-1e30f, -1e30f, -1e30f};
    for (auto& item : detached) {
        item.mesh = {};
        item.mesh.min = scene.min;
        item.mesh.max = scene.max;
    }
    if (extras) *extras = {};

    // Part meshes (article, part) -> chains, decoded once.
    struct PartMesh { std::vector<ChainData> chains; bool ok = false; };
    std::map<std::pair<std::size_t, int>, PartMesh> parts;
    std::map<std::pair<std::string, int>, std::size_t> batch_index;  // (shape, flags) -> batch
    std::vector<std::map<std::pair<std::string, int>, std::size_t>> detached_indices(detached.size());
    std::map<std::pair<std::string, int>, std::size_t> sky_index;

    auto part_mesh = [&](std::size_t art, int part) -> const PartMesh& {
        const auto key = std::make_pair(art, part);
        if (auto it = parts.find(key); it != parts.end()) return it->second;
        PartMesh& pm = parts[key];
        const Article& a = articles[art];
        if (extras && a.has_bounds && (a.name == "Chal_WinGlass" || a.name == "Chal_WinGlassSm")) {
            // The window draw packets are shared by longer joined walls.
            // Instantiate each pane from its own extent and glass material.
            const auto& lo = a.bounds_min;
            const auto& hi = a.bounds_max;
            ChainData pane;
            pane.shape = "glas";
            pane.translucent = true;
            constexpr std::uint32_t tint = 0xDCFFFFFF;
            pane.vertices = {
                {{lo[0], lo[1], 0}, {0, 1}, tint}, {{hi[0], lo[1], 0}, {1, 1}, tint},
                {{lo[0], hi[1], 0}, {0, 0}, tint}, {{hi[0], hi[1], 0}, {1, 0}, tint},
            };
            pane.strips.emplace_back(0, 4);
            pm.chains.push_back(std::move(pane));
            pm.ok = true;
            return pm;
        }
        const auto sym = a.part_symbol.find(part);
        if (sym == a.part_symbol.end()) return pm;
        const auto ms = model_symbol.find(sym->second);
        if (ms == model_symbol.end()) return pm;
        auto chains = model_chains.find(sym->second);
        if (chains == model_chains.end()) {
            // Per-placement variants share the base mesh when their model slots point
            // to the same object. The shared geometry is named with the .00 suffix.
            const auto suffix = sym->second.rfind('.');
            if (suffix == std::string::npos) return pm;
            const auto base_name = sym->second.substr(0, suffix) + ".00";
            const auto base = model_symbol.find(base_name);
            if (base == model_symbol.end() || !elf.is_pointer(base->second) || !elf.is_pointer(ms->second) ||
                u32(elf, base->second) != u32(elf, ms->second)) return pm;
            chains = model_chains.find(base_name);
            if (chains == model_chains.end()) return pm;
        }
        for (std::uint32_t chain : chains->second) {
            ChainData cd;
            if (dec.decode(chain, cd)) pm.chains.push_back(std::move(cd));
        }
        if (extras && a.name == "CHAL_Door3Ext") {
            // The packed door panel travels with the preceding roof list.
            const auto shared = model_chains.find("__Model:::CHAL_RoofCeiling::{as  0000}::Model[0].00");
            if (shared != model_chains.end())
                for (const auto chain : shared->second) {
                    ChainData panel;
                    if (!dec.decode(chain, panel) || !chain_fits_article(panel, a)) continue;
                    panel.shape = "tbch";
                    pm.chains.push_back(std::move(panel));
                }
        }
        if (extras && !detached.empty() && a.name == "bomb" && part == 8) {
            // One charge/case packet was exported under the preceding
            // treeline model's name. Its positions fit the bomb's bounds.
            const auto shared = model_chains.find("__Model:::treelinexquadsmall::{as  0000}::Model[0].00");
            if (shared != model_chains.end())
                for (const auto chain : shared->second) {
                    ChainData cd;
                    if (!dec.decode(chain, cd) || !chain_fits_article(cd, a)) continue;
                    cd.shape = "tbgn";
                    cd.alpha_test = cd.translucent = false;
                    pm.chains.push_back(std::move(cd));
                }
        }
        pm.ok = !pm.chains.empty();
        return pm;
    };

    const std::size_t count = inst_table->count;
    for (std::size_t i = 0; i < count; ++i) {
        ++scene.instances;
        const std::size_t base = i * 64;
        const auto ref = world_ref.find(load<std::uint16_t>(inst, base + 28));
        std::size_t art = SIZE_MAX;
        int part = -1;   // -1: whole article
        if (ref != world_ref.end()) {
            // "CARP::<article>::{as  NNNN}" names a part; plain "CARP::<article>" the article's first part.
            const std::string& s = ref->second;
            if (s.rfind("CARP::", 0) == 0) {
                const auto open = s.rfind("::{as  ");
                const std::string name = open == std::string::npos ? s.substr(6) : s.substr(6, open - 6);
                const auto it = article_by_name.find(name);
                if (it != article_by_name.end()) {
                    art = it->second;
                    part = open == std::string::npos ? -1 : int(std::strtol(s.c_str() + open + 7, nullptr, 16));
                }
            }
        }
        if (art == SIZE_MAX) { ++scene.unresolved; continue; }

        // A plain "CARP::<article>" reference draws every child part (`as` 1..n; part 0 is the aggregate model
        // of the article when it has children), a numbered one just that part.
        Mat4 placement;
        for (std::size_t k = 0; k < placement.size(); ++k) placement[k] = f32(inst, base + 4 * k);
        placement[3] = placement[7] = placement[11] = 0;
        placement[15] = 1;
        std::vector<std::pair<int, Mat4>> wanted;
        const Article& article = articles[art];
        const bool sky = extras && (article.name == "sky01_G" || article.name == "sky02_G");
        if (sky) placement[12] = placement[13] = placement[14] = 0;
        const bool lamp_fixture = extras && article.name == "POBG_parkinglotlight";
        if (extras && (article.name == "CHAL_ChandLight" || article.name == "CHAL_LightPool" ||
                       article.name == "CHAL_LightSide" || article.name == "CHAL_LightLampNS")) {
            const float height = article.name == "CHAL_LightLampNS" ? 1.4f : 0.0f;
            extras->lamps.push_back({placement[12], placement[13] + height, placement[14]});
        }
        if (extras && (lamp_fixture || article.name == "POBG_postreetlamp"))
            // Each lamp effect has a float4 socket followed by the effect type.
            // 0x14 identifies the lamp entries among the other 64-byte effects.
            for (std::size_t off = 0; off + 64 <= article.lamp_effects.size(); off += 64) {
                const Bytes effect = article.lamp_effects.subspan(off, 64);
                if (load<std::uint32_t>(effect, 16) != 0x14) continue;
                const Vec3 socket{f32(effect, 0), f32(effect, 4), f32(effect, 8)};
                extras->lamps.push_back(transform_point(placement, socket) + Vec3{0, -0.18f, 0});
            }
        const bool garage_door = article.name == "NOBG_garage_rollup01";
        const bool propane_tank = extras && article.name == "PTank";
        const bool lookout = extras && article.name == "LookOutTowerExplosion";
        const bool bomb = extras && !detached.empty() && article.name == "bomb";
        const bool garage_wreck = extras && !detached.empty() && article.name == "NOBG_Garage_Explode";
        const bool bridge = extras && !detached.empty() && article.name == "BigBridgeExplosion";
        constexpr int bomb_digits[] = {13, 12, 10};
        if (bomb) {
            // The placed reference is just one digit. Part 8 contains the
            // case and the initial clock quad, with its own atlas coordinates.
            wanted.emplace_back(8, identity());
            for (int digit_part : bomb_digits) wanted.emplace_back(digit_part, identity());
        } else if (garage_wreck) {
            // Part 32 is the closed shell. The open, damaged shell sits at
            // the same anchor in the explosion model's second bind table.
            wanted.emplace_back(56, identity());
            const Bytes bind = article.animated_parts;
            for (std::size_t off = 64; off + 64 <= bind.size(); off += 64) {
                const int fragment = load<std::uint16_t>(bind, off + 28);
                if (fragment >= 56) break;
                Mat4 local;
                for (std::size_t k = 0; k < local.size(); ++k) local[k] = f32(bind, off + 4 * k);
                local[3] = local[7] = local[11] = 0;
                local[15] = 1;
                wanted.emplace_back(fragment, local);
            }
        } else if (bridge) {
            // The placed reference is an effect. Its bind table contains the
            // intact bridge, broken ends, and separately animated timbers.
            const Bytes bind = article.animated_parts;
            for (std::size_t off = 0; off + 64 <= bind.size(); off += 64) {
                if (load<std::uint16_t>(bind, off + 30) != 0x6173) continue;
                const int part_id = load<std::uint16_t>(bind, off + 28);
                if (part_id < 34) continue;
                Mat4 local;
                for (std::size_t k = 0; k < local.size(); ++k) local[k] = f32(bind, off + 4 * k);
                local[3] = local[7] = local[11] = 0;
                local[15] = 1;
                wanted.emplace_back(part_id, local);
            }
        } else if ((garage_door || lamp_fixture || propane_tank || lookout) && !article.animated_parts.empty()) {
            // These objects need all the visible pieces in
            // their placed variant's bind table, not just the referenced leaf.
            const Bytes leaves = article.animated_parts;
            for (std::size_t off = 0; off + 64 <= leaves.size(); off += 64) {
                if (load<std::uint16_t>(leaves, off + 30) != 0x6173) continue;
                if ((propane_tank || lookout) && (load<std::uint8_t>(leaves, off + 12) & 1)) continue;
                Mat4 local;
                for (std::size_t k = 0; k < local.size(); ++k) local[k] = f32(leaves, off + 4 * k);
                local[3] = local[7] = local[11] = 0;
                local[15] = 1;
                wanted.emplace_back(load<std::uint16_t>(leaves, off + 28), local);
            }
        } else if (!detached.empty() && (article.name == "CHAL_FrontDoor_Smash" ||
                                         article.name == "Chal_WinGlass_Smash")) {
            // The first three bind slots contain the panel and its pieces.
            // The last slot points into a shared list outside the object.
            const Bytes bind = article.animated_parts;
            for (std::size_t off = 0; off + 64 <= bind.size() && off < 192; off += 64) {
                Mat4 local;
                for (std::size_t k = 0; k < local.size(); ++k) local[k] = f32(bind, off + 4 * k);
                local[3] = local[7] = local[11] = 0;
                local[15] = 1;
                wanted.emplace_back(load<std::uint16_t>(bind, off + 28), local);
            }
        } else if (!detached.empty() && article.name == "CHAL_Window_Smash") {
            // The world reference is the event slot, while part 0 (LOD 0)
            // carries the full window frame and trim before the crash.
            Mat4 frame = identity();
            frame[13] = 0.1688f;
            wanted.emplace_back(0, frame);
        } else if (extras && article.name == "explodingbarrel" && part == 1) {
            // The placement references the explosion effect. Its intact
            // barrel shell is part 0 in the article's bind table.
            wanted.emplace_back(0, identity());
        } else if (part >= 0) wanted.emplace_back(part, identity());
        else {
            for (const auto& [k, sym] : articles[art].part_symbol)
                if (k != 0 || articles[art].part_symbol.size() == 1) wanted.emplace_back(k, identity());
        }
        const auto item = std::ranges::find(detached, i, &DetachedTrackMesh::instance);
        const bool separate = item != detached.end();
        SceneMesh* barrel = nullptr;
        if (!separate && extras && !detached.empty() && article.name == "explodingbarrel")
            barrel = &extras->barrels.emplace_back(PlacedTrackMesh{
                {placement[12], placement[13], placement[14]}, {}}).mesh;
        SceneMesh& destination = separate ? item->mesh : barrel ? *barrel : sky ? extras->sky : scene;
        std::map<std::pair<std::string, int>, std::size_t> barrel_index;
        auto& destination_index = separate ? detached_indices[item - detached.begin()] :
                                  barrel ? barrel_index : sky ? sky_index : batch_index;
        bool any = false;
        // Shared PS2 draw lists can contain a neighbouring object's vertices
        // under the same GeoPrim name. Reject those packets using the
        // exporter bounds of single-piece Alpine articles. The shutter Base
        // describes a panel in its own frame, before the bind transform.
        const bool castle_gate = article.name == "Castle_Door_Left" || article.name == "Castle_Door_Right";
        const bool check_bounds = extras && article.has_bounds && (!article.multipart || garage_door || castle_gate);
        // The front wall panels are exported on the opposite side of the
        // thin local Z slab described by their bounds.
        const bool chalet_facade = article.name == "CHAL_WallDoorExt";
        // The exporter put the masonry packets in DrainPipe's render list.
        // The wall placement still owns their bounds and world transform.
        const auto drain_pipe = extras && article.name == "NewDrainPipe_Wall" ? article_by_name.find("DrainPipe")
                                                                             : article_by_name.end();
        for (const auto& [wanted_part, local] : wanted) {
            const auto digit = bomb ? std::ranges::find(bomb_digits, wanted_part) : std::end(bomb_digits);
            const int clock_digit = digit == std::end(bomb_digits) ? -1 : int(digit - std::begin(bomb_digits));
            const bool fragment = garage_wreck && wanted_part < 56;
            const bool drain_wall = drain_pipe != article_by_name.end();
            const PartMesh& pm = drain_wall ? part_mesh(drain_pipe->second, 1) : part_mesh(art, wanted_part);
            if (!pm.ok) continue;
            const bool broken_ends = bridge && wanted_part == 36;
            const bool bridge_piece = bridge && wanted_part != 34 && !broken_ends;
            const Mat4 m = bridge_piece ? identity() : mul(placement, local);
            TrackAnimatedPiece* debris = nullptr;
            if (bridge_piece) {
                const auto keys = article.piece_keys.find(wanted_part - 33);
                if (keys == article.piece_keys.end()) continue;
                debris = &extras->bridge_debris.emplace_back(TrackAnimatedPiece{{}, placement, {}});
                for (std::size_t i = 0; i + 32 <= keys->second.size(); i += 32) {
                    const Bytes frame = keys->second.subspan(i, 32);
                    debris->frames.push_back({{f32(frame, 0), f32(frame, 4), f32(frame, 8), f32(frame, 12)},
                                              {f32(frame, 16), f32(frame, 20), f32(frame, 24)}});
                }
            }
            SceneMesh& part_destination = debris ? debris->mesh : broken_ends ? extras->bridge_ends : fragment
                ? extras->garage_fragments.emplace_back(PlacedTrackMesh{{m[12], m[13], m[14]}, {}}).mesh : destination;
            std::map<std::pair<std::string, int>, std::size_t> part_index;
            for (const ChainData& cd : pm.chains) {
                if (drain_wall && chain_fits_article(cd, articles[drain_pipe->second])) continue;
                if (bridge_piece && !std::ranges::all_of(cd.vertices, [&](const MeshVertex& v) {
                        // Reject neighbouring draw packets outside the intact span.
                        return std::abs(v.pos[0] + local[12]) < 26.0f;
                    })) continue;
                // The two fence articles share the same silhouette at
                // slightly different lengths in their render/collision data.
                const bool wood_fence = cd.shape == "wdnf" &&
                                        (article.name == "fence_00" || article.name == "fence_00_l");
                if ((sky && !is_sky_shape(cd.shape)) ||
                    (article.name == "CHAL_Window_Smash" && !chain_fits_article(cd, article)) ||
                    (check_bounds && !wood_fence && !chain_fits_article(cd, article) &&
                     !(chalet_facade && chain_fits_article(cd, article, true)))) continue;
                any = true;
                const bool first_digit = bomb && wanted_part == 8 && cd.vertices.size() == 4;
                const int clock_frame = first_digit ? 3 : clock_digit;
                const std::string shape = first_digit ? "bmbn" : cd.shape;
                SceneMesh& draw = clock_frame >= 0 ? extras->bomb_digits[clock_frame] : part_destination;
                auto& draw_index = clock_frame >= 0 || fragment || broken_ends || debris ? part_index : destination_index;
                const auto key = std::make_pair(shape, int(cd.alpha_test) | int(cd.translucent) << 1);
                auto [it, fresh] = draw_index.try_emplace(key, draw.batches.size());
                if (fresh) {
                    MeshBatch& batch = draw.batches.emplace_back();
                    batch.shape = shape;
                    batch.alpha_test = cd.alpha_test;
                    batch.translucent = cd.translucent;
                    batch.fogged = !is_sky_shape(cd.shape);
                }
                MeshBatch& b = draw.batches[it->second];
                const std::uint32_t first = static_cast<std::uint32_t>(b.vertices.size());
                for (const MeshVertex& v : cd.vertices) {
                    MeshVertex w = v;
                    for (int a = 0; a < 3; ++a)
                        w.pos[a] = v.pos[0] * m[a] + v.pos[1] * m[4 + a] + v.pos[2] * m[8 + a] + m[12 + a];
                    for (int a = 0; a < 3; ++a)
                        draw.min[a] = std::min(draw.min[a], w.pos[a]), draw.max[a] = std::max(draw.max[a], w.pos[a]);
                    b.vertices.push_back(w);
                }
                append_triangles(b, first, cd);
            }
            if (debris && (debris->mesh.batches.empty() || debris->frames.empty())) extras->bridge_debris.pop_back();
        }
        if (!any) ++scene.unresolved;
    }
    return scene;
}

SceneMesh build_snow_underlay(const TrackCollision& collision, const SceneMesh& authored) {
    // Borrow the bake from nearby opaque snow. A flat fallback colour
    // makes every exposed collision triangle stand out at the render seams.
    constexpr float cell = 8.0f;
    struct Tint {
        Vec3 sum{};
        unsigned count = 0;
    };
    std::map<std::array<int, 3>, Tint> snow_tints, rock_tints;
    for (const auto& batch : authored.batches) {
        const bool rock = batch.shape == "rk1j";
        if (batch.translucent || batch.alpha_test ||
            (!rock && batch.shape != "snw1" && batch.shape != "snwb")) continue;
        auto& tints = rock ? rock_tints : snow_tints;
        for (const auto& v : batch.vertices) {
            auto& tint = tints[{int(std::floor(v.pos[0] / cell)), int(std::floor(v.pos[1] / cell)),
                                int(std::floor(v.pos[2] / cell))}];
            for (int i = 0; i < 3; ++i) tint.sum[i] += float((v.rgba >> (8 * i)) & 0xff);
            ++tint.count;
        }
    }
    const auto tint_at = [&](const Vec3& p, bool rock) {
        const auto& tints = rock ? rock_tints : snow_tints;
        const Vec3 fallback = rock ? Vec3{141, 141, 141} : Vec3{187, 187, 187};
        const float x = p[0] / cell - 0.5f, z = p[2] / cell - 0.5f;
        const int ix = int(std::floor(x)), iy = int(std::floor(p[1] / cell)), iz = int(std::floor(z));
        Vec3 colour{};
        for (int dz = 0; dz < 2; ++dz) {
            for (int dx = 0; dx < 2; ++dx) {
                const auto it = tints.find({ix + dx, iy, iz + dz});
                const float wx = dx ? x - ix : 1.0f - (x - ix);
                const float wz = dz ? z - iz : 1.0f - (z - iz);
                const Vec3 sample = it == tints.end() ? fallback : it->second.sum * (1.0f / it->second.count);
                colour += sample * (wx * wz);
            }
        }
        std::uint32_t rgba = 0xff000000u;
        for (int i = 0; i < 3; ++i) rgba |= std::uint32_t(std::clamp(colour[i], 85.0f, 255.0f)) << (8 * i);
        return rgba;
    };
    SceneMesh mesh;
    mesh.batches.resize(3);
    auto& snow = mesh.batches[0];
    auto& road = mesh.batches[1];
    auto& rock = mesh.batches[2];
    snow.shape = "snwb"; // opaque snow, including the texels snw1 uses as cutouts
    road.shape = "snr1";
    rock.shape = "rk1j";
    for (const Tri& t : collision.triangles()) {
        const bool on_road = t.surface == Surface::Paved || t.surface == Surface::Ice;
        if (!t.solid() || (!on_road && t.surface != Surface::Snow) || std::abs(t.normal[1]) < 0.18f ||
            (t.normal[1] < 0 && !t.two_sided)) continue;
        // The chalet's floor and ceiling colliders use the road material.
        // Let the authored timber and tile draw there instead of icy fill.
        const Vec3 center = (t.v[0] + t.v[1] + t.v[2]) * (1.0f / 3.0f);
        const float lengthwise = center[0] - center[2], sideways = center[0] + center[2];
        if (on_road && center[1] > -243.0f && center[1] < -228.0f &&
            lengthwise > 1767.0f && lengthwise < 1880.0f && sideways > 1878.0f && sideways < 1909.0f) continue;
        // Some cliff colliders use the road surface ID. Only use the road
        // texture on shallow slopes where a sled could actually drive.
        const bool steep = std::abs(t.normal[1]) < 0.54f;
        MeshBatch& surface = steep ? rock : on_road ? road : snow;
        for (const Vec3& p : t.v) {
            const float u = steep && std::abs(t.normal[0]) > std::abs(t.normal[2]) ? p[2] : p[0];
            const float v = steep ? -p[1] : p[2];
            surface.indices.push_back(static_cast<std::uint32_t>(surface.vertices.size()));
            surface.vertices.push_back({{p[0], p[1] - 0.14f, p[2]}, {u / 6.0f, v / 6.0f},
                                        on_road && !steep ? 0xFFBBBBBB : tint_at(p, steep)});
        }
    }
    return mesh;
}

// ---- vehicles ---------------------------------------------------------------------------------------

SceneMesh build_bound_vehicle_body(const CarpFile& carp, std::span<const VehiclePart> parts, int last_part,
                                   std::span<const int> omit) {
    SceneMesh body;
    const CarpEntry* table = carp.find("in", 0);
    if (!table) return body;
    const Bytes instances = carp.payload(*table);
    for (std::size_t i = 0; i < table->count && (i + 1) * 64 <= instances.size(); ++i) {
        const Bytes record = instances.subspan(i * 64, 64);
        const auto part_id = load<std::uint16_t>(record, 28);
        if (load<std::uint16_t>(record, 30) != 0x6173 || part_id > last_part ||
            std::ranges::find(omit, part_id) != omit.end()) continue;
        const auto part = std::ranges::find(parts, part_id, &VehiclePart::id);
        if (part == parts.end()) continue;
        Mat4 transform{};
        for (std::size_t j = 0; j < transform.size(); ++j) transform[j] = load<float>(record, j * 4);
        const bool mirrored = dot(cross({transform[0], transform[1], transform[2]},
                                        {transform[4], transform[5], transform[6]}),
                                  {transform[8], transform[9], transform[10]}) < 0.0f;
        for (const MeshBatch& source : part->mesh.batches) {
            if (source.shape == "dmng") continue;
            MeshBatch& batch = body.batches.emplace_back(source);
            for (MeshVertex& v : batch.vertices) {
                const auto point = transform_point(transform, {v.pos[0], v.pos[1], v.pos[2]});
                std::copy(point.begin(), point.end(), v.pos);
            }
            if (mirrored)
                for (std::size_t j = 0; j + 2 < batch.indices.size(); j += 3)
                    std::swap(batch.indices[j + 1], batch.indices[j + 2]);
        }
    }
    return body;
}

std::vector<VehiclePart> build_vehicle_parts(const CarpFile& carp, const ElfImage& elf, bool primary_lod_only) {
    (void)carp;
    std::unordered_map<std::uint32_t, std::string> shape_at;
    for (const TarBinding& b : tar_bindings(elf)) shape_at[b.data_offset] = b.shape_name;
    std::unordered_map<std::uint32_t, std::uint32_t> reloc_symbol;
    for (const ElfReloc& r : elf.relocs)
        if (r.symbol != 1) reloc_symbol[r.offset] = r.symbol;

    std::unordered_map<std::uint32_t, ChainState> state_of;
    std::map<int, std::vector<std::uint32_t>> chains_of;   // `{as  NNNN}` id -> chains
    ChainState cur;
    for (std::size_t o = 0; o + 12 <= elf.data.size(); o += 4) {
        if (u32(elf, o) != 0xA0000000 || u32(elf, o + 4) != 0 || elf.is_pointer(o)) continue;
        const std::uint32_t n = u32(elf, o + 8);
        if (n % 2 || n == 0 || n > 400 || o + 12 + std::size_t(n) * 4 > elf.data.size()) continue;
        bool ok = true;
        for (std::uint32_t k = 0; k < n / 2 && ok; ++k) ok = u32(elf, o + 12 + 8 * k) == 0xA000FFFF;
        if (!ok) continue;
        for (std::uint32_t k = 0; k < n / 2; ++k) {
            const std::uint32_t chain = u32(elf, o + 12 + 8 * k + 4);
            const std::uint32_t obj = u32(elf, chain), pre = u32(elf, obj);
            for (std::size_t a = pre; a < obj && a + 4 <= elf.data.size(); a += 4) {
                if (elf.is_pointer(a)) {
                    if (const auto it = shape_at.find(u32(elf, a)); it != shape_at.end()) cur.shape = it->second;
                } else if (const auto r = reloc_symbol.find(static_cast<std::uint32_t>(a)); r != reloc_symbol.end()) {
                    update_primitive_state(cur, elf.symbols[r->second].name);
                }
            }
            state_of[chain] = cur;
            // The chain's GeoPrim object names it `<car>::{as  NNNN}::Model[0].00_<n>_name` (+0x14).
            const std::uint32_t name = u32(elf, obj + 0x14);
            const auto text = load_cstr(elf.data, name);
            // Near-camera vehicles draw one LOD per part. Combining both
            // revisions overlays a second shell and can distort the bounds.
            if (primary_lod_only && text.find("::Model[0].") == std::string_view::npos) continue;
            const auto open = text.find("{as  ");
            if (open != std::string_view::npos)
                chains_of[int(std::strtol(std::string(text.substr(open + 5, 4)).c_str(), nullptr, 16))].push_back(chain);
        }
    }

    const Decoder dec{elf, state_of, kVehicleUvScale};
    std::vector<VehiclePart> parts;
    for (const auto& [id, list] : chains_of) {
        VehiclePart part;
        part.id = id;
        part.mesh.min = {1e30f, 1e30f, 1e30f};
        part.mesh.max = {-1e30f, -1e30f, -1e30f};
        std::map<std::pair<std::string, int>, std::size_t> index;
        for (std::uint32_t chain : list) {
            ChainData cd;
            if (!dec.decode(chain, cd)) continue;
            const auto key = std::make_pair(cd.shape, int(cd.alpha_test) | int(cd.translucent) << 1);
            auto [it, fresh] = index.try_emplace(key, part.mesh.batches.size());
            if (fresh) {
                part.mesh.batches.emplace_back();
                part.mesh.batches.back().shape = cd.shape;
                part.mesh.batches.back().alpha_test = cd.alpha_test;
                part.mesh.batches.back().translucent = cd.translucent;
            }
            MeshBatch& b = part.mesh.batches[it->second];
            const std::uint32_t first = static_cast<std::uint32_t>(b.vertices.size());
            for (const MeshVertex& v : cd.vertices) {
                for (int a = 0; a < 3; ++a)
                    part.mesh.min[a] = std::min(part.mesh.min[a], v.pos[a]), part.mesh.max[a] = std::max(part.mesh.max[a], v.pos[a]);
                b.vertices.push_back(v);
            }
            append_triangles(b, first, cd);
        }
        if (!part.mesh.batches.empty()) parts.push_back(std::move(part));
    }
    return parts;
}

}  // namespace nf::driving
