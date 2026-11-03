#include <kpi4/protocol/mission_normalizer.hpp>

#include <cmath>
#include <limits>
#include <optional>

namespace kpi4::protocol {
namespace {

std::optional<core::AltitudeReference> altitude_reference(std::uint8_t frame) {
    switch (frame) {
        case MAV_FRAME_GLOBAL_INT:
            return core::AltitudeReference::mean_sea_level;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            return core::AltitudeReference::relative_to_home;
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            return core::AltitudeReference::above_terrain;
        default:
            return std::nullopt;
    }
}

}  // namespace

MissionNormalizationResult normalize_mission_item(const MissionItemInt& item) {
    const auto reference = altitude_reference(item.frame);
    if (!reference) {
        return MissionNormalizationError::UnsupportedFrame;
    }
    if (item.x == std::numeric_limits<std::int32_t>::max() ||
        item.y == std::numeric_limits<std::int32_t>::max()) {
        return MissionNormalizationError::MissingPosition;
    }

    constexpr double degrees_per_scaled_unit = 1.0 / 10'000'000.0;
    const double latitude = static_cast<double>(item.x) * degrees_per_scaled_unit;
    const double longitude = static_cast<double>(item.y) * degrees_per_scaled_unit;
    if (latitude < -90.0 || latitude > 90.0) {
        return MissionNormalizationError::LatitudeOutOfRange;
    }
    if (longitude < -180.0 || longitude > 180.0) {
        return MissionNormalizationError::LongitudeOutOfRange;
    }

    std::optional<core::Meters> altitude;
    if (std::isnan(item.z)) {
        altitude = std::nullopt;
    } else if (!std::isfinite(item.z)) {
        return MissionNormalizationError::NonFiniteAltitude;
    } else {
        altitude = core::Meters{item.z};
    }

    return core::MissionItem{
        item.seq,
        core::Wgs84Position{core::Degrees{latitude}, core::Degrees{longitude}},
        *reference,
        altitude,
        std::nullopt,
    };
}

}  // namespace kpi4::protocol
