#pragma once

#include <kpi4/core/types.hpp>
#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace kpi4::protocol {

struct EstimatedPosition {
    InputChannel channel;
    std::uint32_t sender_sysid;
    std::uint8_t sender_compid;
    // Sender boot time is not the module's monotonic measurement timestamp.
    std::uint32_t time_boot_ms;
    core::Wgs84Position position;
    core::Altitude altitude_msl;
    core::Altitude altitude_relative_to_home;
    core::NorthEastVelocity horizontal_velocity;
    core::MetersPerSecond velocity_down;
    // MAVLink hdg is vehicle yaw, not GPS course over ground.
    std::optional<core::Degrees> heading;
};

enum class EstimatedPositionError {
    InvalidLength,
    LatitudeOutOfRange,
    LongitudeOutOfRange,
    HeadingOutOfRange,
};

using EstimatedPositionResult =
    std::variant<EstimatedPosition, EstimatedPositionError>;

// Accepts a CRC-validated FrameReader event. Unrelated or rejected frames
// yield no result; malformed GLOBAL_POSITION_INT data yields an error.
std::optional<EstimatedPositionResult> decode_global_position_int(
    const FrameReadResult& event);

}  // namespace kpi4::protocol
