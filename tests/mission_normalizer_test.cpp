#include <kpi4/protocol/mission_normalizer.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

kpi4::protocol::MissionItemInt item(std::uint8_t frame, std::int32_t x,
                                    std::int32_t y, float z) {
    return {{0, 0, MAV_MISSION_TYPE_MISSION}, 17, frame,
            MAV_CMD_NAV_WAYPOINT, 0, 1, 0.0f, 0.0f, 0.0f, 0.0f,
            x, y, z};
}

const kpi4::core::MissionItem& normalized(
    const kpi4::protocol::MissionNormalizationResult& result) {
    const auto* value = std::get_if<kpi4::core::MissionItem>(&result);
    require(value != nullptr, "valid item was rejected");
    return *value;
}

void rejected(const kpi4::protocol::MissionItemInt& source,
              kpi4::protocol::MissionNormalizationError expected) {
    const auto result = kpi4::protocol::normalize_mission_item(source);
    const auto* error = std::get_if<kpi4::protocol::MissionNormalizationError>(&result);
    require(error && *error == expected, "item returned the wrong error");
}

}  // namespace

int main() {
    using kpi4::core::AltitudeReference;
    using kpi4::protocol::MissionNormalizationError;

    const auto msl = normalized(kpi4::protocol::normalize_mission_item(
        item(MAV_FRAME_GLOBAL_INT, 900000000, -1800000000, 125.5f)));
    require(msl.sequence == 17 && msl.target.latitude.value == 90.0 &&
                msl.target.longitude.value == -180.0,
            "global coordinate scale or upper boundaries are wrong");
    require(msl.altitude_reference == AltitudeReference::mean_sea_level &&
                msl.altitude && msl.altitude->value == 125.5,
            "MSL altitude or reference changed");

    const auto relative = normalized(kpi4::protocol::normalize_mission_item(
        item(MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, -900000000, 1800000000, -20.25f)));
    require(relative.target.latitude.value == -90.0 &&
                relative.target.longitude.value == 180.0,
            "global coordinate lower boundaries are wrong");
    require(relative.altitude_reference == AltitudeReference::relative_to_home &&
                relative.altitude && relative.altitude->value == -20.25,
            "relative altitude or reference changed");

    const auto fractional = normalized(kpi4::protocol::normalize_mission_item(
        item(MAV_FRAME_GLOBAL_INT, -431234567, 1799999999, 0.0f)));
    require(std::abs(fractional.target.latitude.value - (-43.1234567)) < 1e-12 &&
                std::abs(fractional.target.longitude.value - 179.9999999) < 1e-12,
            "negative/fractional coordinate conversion lost resolution");

    const auto terrain = normalized(kpi4::protocol::normalize_mission_item(
        item(MAV_FRAME_GLOBAL_TERRAIN_ALT_INT, 0, 0,
             std::numeric_limits<float>::quiet_NaN())));
    require(terrain.altitude_reference == AltitudeReference::above_terrain &&
                !terrain.altitude,
            "unspecified terrain altitude lost its reference or became zero");

    const std::array<std::uint8_t, 6> unsupported_frames{
        MAV_FRAME_GLOBAL, MAV_FRAME_LOCAL_NED, MAV_FRAME_MISSION,
        MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_FRAME_LOCAL_ENU, 255};
    for (const auto frame : unsupported_frames) {
        rejected(item(frame, 0, 0, 1.0f),
                 MissionNormalizationError::UnsupportedFrame);
    }
    rejected(item(MAV_FRAME_GLOBAL_INT, 900000001, 0, 1.0f),
             MissionNormalizationError::LatitudeOutOfRange);
    rejected(item(MAV_FRAME_GLOBAL_INT, -900000001, 0, 1.0f),
             MissionNormalizationError::LatitudeOutOfRange);
    rejected(item(MAV_FRAME_GLOBAL_INT, 0, 1800000001, 1.0f),
             MissionNormalizationError::LongitudeOutOfRange);
    rejected(item(MAV_FRAME_GLOBAL_INT, 0, -1800000001, 1.0f),
             MissionNormalizationError::LongitudeOutOfRange);
    rejected(item(MAV_FRAME_GLOBAL_INT, std::numeric_limits<std::int32_t>::max(),
                  0, 1.0f),
             MissionNormalizationError::MissingPosition);
    rejected(item(MAV_FRAME_GLOBAL_INT, 0,
                  std::numeric_limits<std::int32_t>::max(), 1.0f),
             MissionNormalizationError::MissingPosition);
    rejected(item(MAV_FRAME_GLOBAL_INT, 0, 0,
                  std::numeric_limits<float>::infinity()),
             MissionNormalizationError::NonFiniteAltitude);
}
