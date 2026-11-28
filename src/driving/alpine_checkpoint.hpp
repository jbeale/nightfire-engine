#pragma once

#include <string_view>

namespace nf::driving {

enum class AlpineCheckpoint { Garage, Forest, Gate, Bridge, Chalet, Pursuit, LowerForest };

struct AlpineCheckpointOption {
    AlpineCheckpoint checkpoint;
    std::string_view key;
    std::string_view label;
};

inline constexpr AlpineCheckpointOption kAlpineCheckpoints[] = {
    {AlpineCheckpoint::Garage, "garage", "Garage"},
    {AlpineCheckpoint::Forest, "forest", "Forest chase"},
    {AlpineCheckpoint::Gate, "gate", "Castle gate"},
    {AlpineCheckpoint::Bridge, "bridge", "Bridge approach"},
    {AlpineCheckpoint::Chalet, "chalet", "Chalet approach"},
    {AlpineCheckpoint::Pursuit, "pursuit", "Chase after the chalet"},
    {AlpineCheckpoint::LowerForest, "lower-forest", "Lower forest bend"},
};

}  // namespace nf::driving
