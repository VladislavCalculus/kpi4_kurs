#pragma once

#include <kpi4/core/types.hpp>
#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace kpi4::protocol {

// Raw GNSS report. Position and altitude are reported values, not yet
// accepted as a usable StateSample; fix and freshness checks follow later.
struct RawGps {
    InputChannel channel;
    std::uint32_t sender_sysid;
    std::uint8_t sender_compid;
    // Sender timestamp may be UNIX epoch or time since boot; keep it raw.
    std::uint64_t time_usec;
    std::uint8_t fix_type;
    core::Wgs84Position position;
    core::Altitude altitude_msl;
    std::optional<double> horizontal_dop;
    std::optional<double> vertical_dop;
    std::optional<core::MetersPerSecond> ground_speed;
    // Course over ground is movement direction, not vehicle yaw.
    std::optional<core::Degrees> course_over_ground;
    std::optional<std::uint8_t> satellites_visible;
    // MAVLink 2 extension fields are absent when the payload ends before them.
    std::optional<core::Meters> altitude_ellipsoid;
    std::optional<core::Meters> horizontal_uncertainty;
    std::optional<core::Meters> vertical_uncertainty;
    std::optional<core::MetersPerSecond> speed_uncertainty;
    std::optional<core::Degrees> track_uncertainty;
    std::optional<core::Degrees> gps_yaw;
};

enum class RawGpsError {
    InvalidLength,
    LatitudeOutOfRange,
    LongitudeOutOfRange,
    CourseOutOfRange,
    YawOutOfRange,
};

using RawGpsResult = std::variant<RawGps, RawGpsError>;

// Accepts a CRC-validated FrameReader event. Unrelated/rejected frames yield
// no result; malformed GPS_RAW_INT fields yield a structured error.
std::optional<RawGpsResult> decode_gps_raw_int(const FrameReadResult& event);

}  // namespace kpi4::protocol
