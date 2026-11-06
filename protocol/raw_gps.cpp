#include <kpi4/protocol/raw_gps.hpp>

#include <cstring>
#include <limits>

namespace kpi4::protocol {

std::optional<RawGpsResult> decode_gps_raw_int(const FrameReadResult& event) {
    if (!event.frame || event.frame->msgid != MAVLINK_MSG_ID_GPS_RAW_INT) {
        return std::nullopt;
    }
    const auto length = event.frame->len;
    if (length == 0 || length > MAVLINK_MSG_ID_GPS_RAW_INT_LEN ||
        (event.frame->magic == MAVLINK_STX_MAVLINK1 &&
         length != MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN)) {
        return RawGpsResult{RawGpsError::InvalidLength};
    }

    // MAVLink 2 trims trailing zero bytes, even inside the base payload.
    auto padded = *event.frame;
    std::memset(_MAV_PAYLOAD_NON_CONST(&padded) + length, 0,
                MAVLINK_MSG_ID_GPS_RAW_INT_LEN - length);
    padded.len = MAVLINK_MSG_ID_GPS_RAW_INT_LEN;
    mavlink_gps_raw_int_t raw{};
    mavlink_msg_gps_raw_int_decode(&padded, &raw);

    if (raw.lat < -900'000'000 || raw.lat > 900'000'000) {
        return RawGpsResult{RawGpsError::LatitudeOutOfRange};
    }
    if (raw.lon < -1'800'000'000 || raw.lon > 1'800'000'000) {
        return RawGpsResult{RawGpsError::LongitudeOutOfRange};
    }
    if (raw.cog != std::numeric_limits<std::uint16_t>::max() &&
        raw.cog > 35'999) {
        return RawGpsResult{RawGpsError::CourseOutOfRange};
    }
    if (length > 50 && raw.yaw != 0 &&
        raw.yaw != std::numeric_limits<std::uint16_t>::max() &&
        raw.yaw > 36'000) {
        return RawGpsResult{RawGpsError::YawOutOfRange};
    }

    constexpr double degrees_per_e7 = 1.0 / 10'000'000.0;
    constexpr double meters_per_mm = 1.0 / 1000.0;
    constexpr double meters_per_second_per_cms = 1.0 / 100.0;
    constexpr double degrees_per_cdeg = 1.0 / 100.0;
    constexpr double degrees_per_e5 = 1.0 / 100'000.0;
    constexpr auto unknown_u16 = std::numeric_limits<std::uint16_t>::max();

    RawGps result{
        event.channel,
        event.frame->sysid,
        event.frame->compid,
        raw.time_usec,
        raw.fix_type,
        core::Wgs84Position{
            core::Degrees{static_cast<double>(raw.lat) * degrees_per_e7},
            core::Degrees{static_cast<double>(raw.lon) * degrees_per_e7}},
        core::Altitude{core::Meters{static_cast<double>(raw.alt) * meters_per_mm},
                       core::AltitudeReference::mean_sea_level},
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
        std::nullopt,
    };

    if (raw.eph != unknown_u16) {
        result.horizontal_dop = static_cast<double>(raw.eph) / 100.0;
    }
    if (raw.epv != unknown_u16) {
        result.vertical_dop = static_cast<double>(raw.epv) / 100.0;
    }
    if (raw.vel != unknown_u16) {
        result.ground_speed = core::MetersPerSecond{
            static_cast<double>(raw.vel) * meters_per_second_per_cms};
    }
    if (raw.cog != unknown_u16) {
        result.course_over_ground =
            core::Degrees{static_cast<double>(raw.cog) * degrees_per_cdeg};
    }
    if (raw.satellites_visible != std::numeric_limits<std::uint8_t>::max()) {
        result.satellites_visible = raw.satellites_visible;
    }

    // A zero-valued extension cannot be distinguished from an omitted trailing
    // zero field; expose it only if the wire payload reaches that field.
    if (length > 30) {
        result.altitude_ellipsoid =
            core::Meters{static_cast<double>(raw.alt_ellipsoid) * meters_per_mm};
    }
    if (length > 34) {
        result.horizontal_uncertainty =
            core::Meters{static_cast<double>(raw.h_acc) * meters_per_mm};
    }
    if (length > 38) {
        result.vertical_uncertainty =
            core::Meters{static_cast<double>(raw.v_acc) * meters_per_mm};
    }
    if (length > 42) {
        result.speed_uncertainty =
            core::MetersPerSecond{static_cast<double>(raw.vel_acc) * meters_per_mm};
    }
    if (length > 46) {
        result.track_uncertainty =
            core::Degrees{static_cast<double>(raw.hdg_acc) * degrees_per_e5};
    }
    if (length > 50 && raw.yaw != 0 && raw.yaw != unknown_u16) {
        result.gps_yaw =
            core::Degrees{static_cast<double>(raw.yaw) * degrees_per_cdeg};
    }
    return RawGpsResult{result};
}

}  // namespace kpi4::protocol
