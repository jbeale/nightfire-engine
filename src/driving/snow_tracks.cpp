#include "driving/snow_tracks.hpp"

#include <algorithm>
#include <cmath>

#include "driving/track_model.hpp"
#include "driving/world_query.hpp"

namespace nf::driving {
namespace {

constexpr Vec3 kUp{0, 1, 0};
constexpr float kCellSize = 10.0f;
constexpr float kHalfWidth = 0.96f;
constexpr std::size_t kMaxSegments = 3072;

int cell(float x) { return int(std::floor(x / kCellSize)); }

}  // namespace

SnowTracks::SnowTracks(const SceneMesh& scene, std::size_t sleds) : trails_(sleds) {
    // Road collision has simpler heights than the textured snow. Index the
    // nearby render faces too, so marks stay on top of the visible surface.
    for (const auto& batch : scene.batches) {
        if (batch.translucent || !batch.fogged ||
            !(batch.shape.starts_with("sn") || batch.shape.starts_with("rk") || batch.shape == "jmn1")) continue;
        for (std::size_t i = 0; i + 2 < batch.indices.size(); i += 3) {
            Triangle triangle;
            for (int j = 0; j < 3; ++j) {
                const auto& p = batch.vertices[batch.indices[i + j]].pos;
                triangle[j] = {p[0], p[1], p[2]};
            }
            const auto& [a, b, c] = triangle;
            const Vec3 normal = cross(b - a, c - a);
            if (std::abs(normal[1]) < std::max(0.0001f, length(normal) * 0.48f)) continue;
            for (int x = cell(std::min({a[0], b[0], c[0]})); x <= cell(std::max({a[0], b[0], c[0]})); ++x)
                for (int z = cell(std::min({a[2], b[2], c[2]})); z <= cell(std::max({a[2], b[2], c[2]})); ++z)
                    cells_[{x, z}].push_back(surface_.size());
            surface_.push_back(triangle);
        }
    }
}

std::optional<Vec3> SnowTracks::project(Vec3 p, const CollisionWorld& collision) const {
    GroundHit hit;
    if (!collision.ground_below(p + kUp * 0.52f, hit) || hit.surface != Surface::Snow ||
        hit.normal[1] < 0.52f || p[1] - hit.point[1] > 0.88f) return std::nullopt;

    float height = hit.point[1] - 0.14f;  // matches the render fallback
    const auto bin = cells_.find({cell(p[0]), cell(p[2])});
    if (bin != cells_.end()) {
        for (const std::size_t i : bin->second) {
            const auto& [a, b, c] = surface_[i];
            const Vec3 ab = b - a, ac = c - a, ap = p - a;
            const float det = ab[0] * ac[2] - ab[2] * ac[0];
            const float u = (ap[0] * ac[2] - ap[2] * ac[0]) / det;
            const float v = (ab[0] * ap[2] - ab[2] * ap[0]) / det;
            if (u < 0 || v < 0 || u + v > 1) continue;
            const float y = a[1] + ab[1] * u + ac[1] * v;
            if (y <= hit.point[1] + 0.55f) height = std::max(height, y);
        }
    }
    return Vec3{p[0], height + 0.026f, p[2]};
}

void SnowTracks::tick(float dt) {
    time_ += dt;
    while (!segments_.empty() && age(segments_.front()) >= kLifetime) segments_.pop_front();
}

void SnowTracks::sample(std::size_t sled, std::optional<Vec3> position, const CollisionWorld& collision) {
    Trail& trail = trails_[sled];
    if (!position || !trail.point) {
        trail = {position, {}};
        return;
    }
    Vec3 delta = *position - *trail.point;
    delta[1] = 0;
    const float distance = length(delta);
    if (distance < 0.44f) return;
    if (distance > 2.2f || std::abs((*position)[1] - (*trail.point)[1]) > 1.3f) {
        trail = {position, {}};
        return;
    }

    const Vec3 side = cross(kUp, delta) * (kHalfWidth / distance);
    const auto section = [&](const Vec3& point, float along) -> std::optional<Edge> {
        const auto center = project(point, collision);
        if (!center) return std::nullopt;
        const auto left = project(point - side, collision), right = project(point + side, collision);
        if (!left || !right || std::abs((*left)[1] - (*right)[1]) > 1.2f) return std::nullopt;
        return Edge{*left, *center, *right, along};
    };

    auto start = trail.edge ? trail.edge : section(*trail.point, 0.0f);
    auto end = section(*position, start ? start->distance + distance : 0.0f);
    if (start && end && length(start->left - end->left) < 2.5f &&
        length(start->right - end->right) < 2.5f) {
        if (segments_.size() == kMaxSegments) segments_.pop_front();
        segments_.push_back({*start, *end, time_});
    }
    trail = {position, end};
}

}  // namespace nf::driving
