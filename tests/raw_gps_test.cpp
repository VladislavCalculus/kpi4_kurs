#include <kpi4/protocol/raw_gps.hpp>

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

void near(double actual, double expected, const char* message) {
    require(std::abs(actual - expected) < 1e-10, message);
}

mavlink_message_t pack(const mavlink_gps_raw_int_t& raw, bool mavlink1 = false) {
    mavlink_message_t message{};
    mavlink_status_t status{};
    if (mavlink1) {
        status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
    mavlink_msg_gps_raw_int_encode_status(9, 3, &status, &message, &raw);
    return message;
}

kpi4::protocol::FrameReadResult read_wire(
    const mavlink_message_t& message, kpi4::protocol::InputChannel channel) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const auto count = mavlink_msg_to_send_buffer(bytes.data(), &message);
    kpi4::protocol::FrameReader reader;
    kpi4::protocol::FrameReadResult result{channel, std::nullopt, std::nullopt};
    for (std::size_t i = 0; i < count; ++i) {
        const auto event = reader.consume(channel, bytes[i]);
        require(!event.rejection, "packed GPS frame was rejected");
        if (event.frame) {
            require(!result.frame, "multiple GPS frames emitted");
            result = event;
        }
    }
    require(result.frame.has_value(), "GPS frame was not emitted");
    return result;
}

kpi4::protocol::RawGps decoded(
    const kpi4::protocol::FrameReadResult& frame) {
    const auto result = kpi4::protocol::decode_gps_raw_int(frame);
    require(result.has_value(), "GPS frame was ignored");
    const auto* value = std::get_if<kpi4::protocol::RawGps>(&*result);
    require(value != nullptr, "GPS frame was rejected");
    return *value;
}

void expect_error(const mavlink_gps_raw_int_t& raw,
                  kpi4::protocol::RawGpsError expected) {
    const auto frame = read_wire(pack(raw), kpi4::protocol::InputChannel::ControllerX);
    const auto result = kpi4::protocol::decode_gps_raw_int(frame);
    require(result.has_value(), "invalid GPS frame was ignored");
    const auto* error = std::get_if<kpi4::protocol::RawGpsError>(&*result);
    require(error && *error == expected, "wrong GPS rejection reason");
}

}  // namespace

int main() {
    using namespace kpi4::protocol;
    constexpr auto unknown16 = std::numeric_limits<std::uint16_t>::max();

    mavlink_gps_raw_int_t raw{};
    raw.time_usec = 1'700'000'000'123'456ULL;
    raw.fix_type = GPS_FIX_TYPE_RTK_FIXED;
    raw.lat = -432'345'678;
    raw.lon = 1'791'234'567;
    raw.alt = -12'345;
    raw.eph = 123;
    raw.epv = 456;
    raw.vel = 789;
    raw.cog = 27'001;
    raw.satellites_visible = 14;
    raw.alt_ellipsoid = -11'234;
    raw.h_acc = 250;
    raw.v_acc = 1'200;
    raw.vel_acc = 340;
    raw.hdg_acc = 123'456;
    raw.yaw = 36'000;
    const auto full_frame = read_wire(pack(raw), InputChannel::ControllerX);
    require(full_frame.frame->len == MAVLINK_MSG_ID_GPS_RAW_INT_LEN,
            "full GPS extension payload was trimmed");
    const auto gps = decoded(full_frame);
    require(gps.channel == InputChannel::ControllerX &&
                gps.sender_sysid == 9 && gps.sender_compid == 3 &&
                gps.time_usec == raw.time_usec && gps.fix_type == raw.fix_type,
            "GPS source, raw time or fix type changed");
    near(gps.position.latitude.value, -43.2345678, "latitude scale/sign");
    near(gps.position.longitude.value, 179.1234567, "longitude scale/sign");
    near(gps.altitude_msl.value.value, -12.345, "MSL altitude scale/sign");
    require(gps.altitude_msl.reference ==
                kpi4::core::AltitudeReference::mean_sea_level,
            "GPS altitude reference changed");
    near(*gps.horizontal_dop, 1.23, "HDOP scale");
    near(*gps.vertical_dop, 4.56, "VDOP scale");
    near(gps.ground_speed->value, 7.89, "ground speed scale");
    near(gps.course_over_ground->value, 270.01, "course scale");
    require(gps.satellites_visible == 14, "satellite count changed");
    near(gps.altitude_ellipsoid->value, -11.234, "ellipsoid altitude scale");
    near(gps.horizontal_uncertainty->value, 0.25, "horizontal accuracy scale");
    near(gps.vertical_uncertainty->value, 1.2, "vertical accuracy scale");
    near(gps.speed_uncertainty->value, 0.34, "speed accuracy scale");
    near(gps.track_uncertainty->value, 1.23456, "track accuracy scale");
    near(gps.gps_yaw->value, 360.0, "north yaw special value changed");

    mavlink_gps_raw_int_t missing{};
    missing.fix_type = GPS_FIX_TYPE_NO_FIX;
    missing.eph = unknown16;
    missing.epv = unknown16;
    missing.vel = unknown16;
    missing.cog = unknown16;
    missing.satellites_visible = std::numeric_limits<std::uint8_t>::max();
    const auto no_fix_frame = read_wire(pack(missing), InputChannel::GroundStation);
    const auto no_fix = decoded(no_fix_frame);
    require(no_fix.fix_type == GPS_FIX_TYPE_NO_FIX &&
                no_fix.position.latitude.value == 0.0 &&
                !no_fix.horizontal_dop && !no_fix.vertical_dop &&
                !no_fix.ground_speed && !no_fix.course_over_ground &&
                !no_fix.satellites_visible && !no_fix.altitude_ellipsoid &&
                !no_fix.horizontal_uncertainty && !no_fix.vertical_uncertainty &&
                !no_fix.speed_uncertainty && !no_fix.track_uncertainty &&
                !no_fix.gps_yaw,
            "missing GPS values became zero measurements");
    require(no_fix_frame.frame->len == MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN,
            "base-only GPS payload length changed");

    const auto v1_frame = read_wire(pack(missing, true), InputChannel::ControllerX);
    require(v1_frame.frame->magic == MAVLINK_STX_MAVLINK1 &&
                v1_frame.frame->len == MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN &&
                !decoded(v1_frame).horizontal_uncertainty,
            "MAVLink 1 GPS base fields or extension absence changed");

    auto zero_speed = missing;
    zero_speed.fix_type = GPS_FIX_TYPE_3D_FIX;
    zero_speed.vel = 0;
    zero_speed.cog = 0;
    zero_speed.eph = 0;
    zero_speed.epv = 0;
    zero_speed.satellites_visible = 0;
    const auto still_known = decoded(read_wire(pack(zero_speed),
                                               InputChannel::ControllerX));
    require(still_known.ground_speed && still_known.ground_speed->value == 0.0 &&
                still_known.course_over_ground &&
                still_known.course_over_ground->value == 0.0 &&
                still_known.horizontal_dop && *still_known.horizontal_dop == 0.0 &&
                still_known.satellites_visible && *still_known.satellites_visible == 0,
            "valid zero GPS values were mistaken for missing");

    auto partial = missing;
    partial.alt_ellipsoid = 0;
    partial.h_acc = 0;
    partial.v_acc = 1'000;
    const auto partial_frame = read_wire(pack(partial), InputChannel::ControllerX);
    require(partial_frame.frame->len > 38 && partial_frame.frame->len <= 42,
            "partial extension boundary changed");
    const auto partial_gps = decoded(partial_frame);
    require(partial_gps.altitude_ellipsoid &&
                partial_gps.altitude_ellipsoid->value == 0.0 &&
                partial_gps.horizontal_uncertainty &&
                partial_gps.horizontal_uncertainty->value == 0.0 &&
                partial_gps.vertical_uncertainty &&
                partial_gps.vertical_uncertainty->value == 1.0 &&
                !partial_gps.speed_uncertainty && !partial_gps.track_uncertainty,
            "partial MAVLink 2 extensions were misread");

    auto invalid = raw;
    invalid.lat = 900'000'001;
    expect_error(invalid, RawGpsError::LatitudeOutOfRange);
    invalid = raw;
    invalid.lon = -1'800'000'001;
    expect_error(invalid, RawGpsError::LongitudeOutOfRange);
    invalid = raw;
    invalid.cog = 36'000;
    expect_error(invalid, RawGpsError::CourseOutOfRange);
    invalid = raw;
    invalid.yaw = 36'001;
    expect_error(invalid, RawGpsError::YawOutOfRange);

    auto unknown_yaw = raw;
    unknown_yaw.yaw = unknown16;
    unknown_yaw.fix_type = 254;
    const auto unknown = decoded(read_wire(pack(unknown_yaw),
                                           InputChannel::ControllerX));
    require(!unknown.gps_yaw && unknown.fix_type == 254,
            "unknown yaw or fix enum was invented");

    auto malformed = full_frame;
    malformed.frame->magic = MAVLINK_STX_MAVLINK1;
    malformed.frame->len = MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN - 1;
    const auto too_short = decode_gps_raw_int(malformed);
    require(too_short && std::get_if<RawGpsError>(&*too_short) &&
                std::get<RawGpsError>(*too_short) == RawGpsError::InvalidLength,
            "short MAVLink 1 GPS payload was decoded");
    malformed.frame->magic = MAVLINK_STX;
    malformed.frame->len = MAVLINK_MSG_ID_GPS_RAW_INT_LEN + 1;
    const auto too_long = decode_gps_raw_int(malformed);
    require(too_long && std::get_if<RawGpsError>(&*too_long) &&
                std::get<RawGpsError>(*too_long) == RawGpsError::InvalidLength,
            "oversized GPS payload was decoded");
    malformed.frame->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    require(!decode_gps_raw_int(malformed), "unrelated frame decoded as GPS");
    malformed.frame.reset();
    require(!decode_gps_raw_int(malformed), "absent frame decoded as GPS");
}
