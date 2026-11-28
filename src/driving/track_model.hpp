#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "assets/anim.hpp"
#include "assets/carp_file.hpp"
#include "core/math.hpp"

namespace nf::driving {

class TrackCollision;
struct MeshVertex {
    float pos[3];
    float uv[2];
    std::uint32_t rgba;  // vertex colour, 0xFF = 1.0 per channel
};

// One draw batch: triangles sharing a texture shape and render state.
struct MeshBatch {
    std::string shape;       // 4-character `.ssh` shape name of the bound TAR texture; empty = untextured
    bool alpha_test = false; // GeoPrimState alphatest=on or texalpha=1bitA
    bool translucent = false;// GeoPrimState texalpha=deepA
    bool fogged = true;      // false for sky dome/celestial shapes (drawn unfogged)
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices;  // triangle list
};

// A decoded world (or vehicle) mesh set.
struct SceneMesh {
    std::vector<MeshBatch> batches;
    Vec3 min{0, 0, 0}, max{0, 0, 0};
    std::size_t instances = 0;      // placed instances
    std::size_t unresolved = 0;     // instances whose geometry could not be found
};

// Bakes every instance of a track .crp (CARP file + its embedded ELF object) into world space.
//
// Layout (DRIVING.ELF sub_22FAF8 world setup, sub_1C7658/sub_1C7D78 scene objects; docs/driving.md):
//   * the world group's `in` #0 record array holds one 0x40-byte row-major matrix per instance
//     (rows = X/Y/Z axes, row 3 = translation); word +0x1C of each is a `{u16 idx, "sr"}` reference to a
//     `CARP::<article>::{as  NNNN}` string of the same group, naming article + part;
//   * article groups ("Arti" heads) carry Name / Base (bounding box) / as / sr members; an `as` member
//     identifies its `EAGL::<article>::{as  0000}::Model[0].NN` render model. Its chains are
//     identified by the matching model name embedded in each GeoPrim object;
//   * a chain is VIF data: header quad (vertex count), three quads of cumulative strip ends (byte / 3),
//     biased 8.8 V3-16 positions in metres, V2-16 texture coordinates, optional
//     V3-8 vertex colours.
// A placed mesh that mission code can transform separately from the baked track.
struct DetachedTrackMesh {
    std::size_t instance;
    SceneMesh mesh;
};
struct PlacedTrackMesh {
    Vec3 position;
    SceneMesh mesh;
};
struct TrackAnimatedPiece {
    struct Frame { nf::Quat rotation; Vec3 position; };
    SceneMesh mesh;
    Mat4 placement;
    std::vector<Frame> frames;
};
struct TrackRenderExtras {
    SceneMesh bridge_ends;
    std::vector<TrackAnimatedPiece> bridge_debris;
    std::vector<PlacedTrackMesh> barrels;
    std::vector<PlacedTrackMesh> garage_fragments;
    std::array<SceneMesh, 4> bomb_digits;
    SceneMesh sky;
    std::vector<Vec3> lamps;
};
SceneMesh build_track_scene(const CarpFile& carp, const ElfImage& elf, std::span<DetachedTrackMesh> detached = {},
                            TrackRenderExtras* extras = nullptr);

// Supplemental render surface for snow sections whose authored graphics leave gaps.
// Kept beneath the driveable surface so the original terrain and decals draw over it.
SceneMesh build_snow_underlay(const TrackCollision& collision, const SceneMesh& authored);

// One `{as  NNNN}` part of a vehicle model: geometry in car space (metres, +Z rear, -Z front).
struct VehiclePart {
    int id = 0;
    SceneMesh mesh;
};

// Vehicle .crp: chains carry float V3-32 positions and are grouped by the asset id in their GeoPrim name.
std::vector<VehiclePart> build_vehicle_parts(const CarpFile& carp, const ElfImage& elf, bool primary_lod_only = false);
// Place the intact render parts of a vehicle using the model's in #0 bind table.
SceneMesh build_bound_vehicle_body(const CarpFile& carp, std::span<const VehiclePart> parts, int last_part,
                                   std::span<const int> omit = {});

}  // namespace nf::driving
