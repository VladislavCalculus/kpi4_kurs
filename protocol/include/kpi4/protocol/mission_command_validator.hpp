#pragma once

#include <kpi4/protocol/mission_transfer.hpp>

#include <cstdint>
#include <variant>

namespace kpi4::protocol {

enum class SupportedMissionCommand {
    waypoint,
};

enum class MissionCommandRejectReason {
    unsupported_in_demo_subset,
};

struct MissionCommandRejection {
    std::uint16_t command;
    MissionCommandRejectReason reason;
};

using MissionCommandValidationResult =
    std::variant<SupportedMissionCommand, MissionCommandRejection>;

// The vehicle-independent demonstration accepts waypoint items only.
MissionCommandValidationResult validate_mission_item_command(
    const MissionItemInt& item);

}  // namespace kpi4::protocol
