#pragma once

#include <kpi4/core/types.hpp>
#include <kpi4/protocol/mission_transfer.hpp>

#include <cstdint>
#include <variant>

namespace kpi4::protocol {

enum class MissionNormalizationError {
    UnsupportedFrame,
    MissingPosition,
    LatitudeOutOfRange,
    LongitudeOutOfRange,
    NonFiniteAltitude,
};

using MissionNormalizationResult =
    std::variant<core::MissionItem, MissionNormalizationError>;

// Normalizes global MISSION_ITEM_INT coordinates while preserving the
// altitude reference. Non-global frames require a separate origin/transform.
MissionNormalizationResult normalize_mission_item(const MissionItemInt& item);

}  // namespace kpi4::protocol
