#include <kpi4/protocol/estimated_position.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_near(double actual, double expected, const char* message) {
    require(std::abs(actual - expected) < 1e-12, message);
}

mavlink_message_t packed(std::uint32_t time_boot_ms, std::int32_t latitude_e7,
                         std::int32_t longitude_e7, std::int32_t altitude_mm,
                         std::int32_t relative_altitude_mm, std::int16_t north_cms,
                         std::int16_t east_cms, std::int16_t down_cms,
                         std::uint16_t heading_cdeg) {
    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(
        7, 1, &message, time_boot_ms, latitude_e7, longitude_e7,
        altitude_mm, relative_altitude_mm, north_cms, east_cms, down_cms,
        heading_cdeg);
    return message;
}

kpi4::protocol::FrameReadResult read_wire(
    const mavlink_message_t& message, kpi4::protocol::InputChannel channel) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
    const auto length = mavlink_msg_to_send_buffer(bytes.data(), &message);
    kpi4::protocol::FrameReader reader;
    kpi4::protocol::FrameReadResult result{channel, std::nullopt, std::nullopt};
    for (std::size_t i = 0; i < length; ++i) {
        const auto event = reader.consume(channel, bytes[i]);
        require(!event.rejection, "packed frame was rejected");
        if (event.frame) {
            require(!result.frame, "multiple frames emitted");
            result = event;
        }
    }
    require(result.frame.has_value(), "frame was not emitted");
    return result;
}

kpi4::protocol::EstimatedPosition decoded(
    const kpi4::protocol::FrameReadResult& frame) {
    const auto result = kpi4::protocol::decode_global_position_int(frame);
    require(result.has_value(), "position frame was ignored");
    const auto* value = std::get_if<kpi4::protocol::EstimatedPosition>(&*result);
    require(value != nullptr, "position frame was rejected");
    return *value;
}

void expect_error(const mavlink_message_t& message,
                  kpi4::protocol::EstimatedPositionError expected) {
    const auto event = read_wire(message, kpi4::protocol::InputChannel::ControllerX);
    const auto result = kpi4::protocol::decode_global_position_int(event);
    require(result.has_value(), "invalid position frame was ignored");
    const auto* error = std::get_if<kpi4::protocol::EstimatedPositionError>(&*result);
    if (!error || *error != expected) {
        throw std::runtime_error(
            "wrong position rejection reason; expected " +
            std::to_string(static_cast<int>(expected)) + ", actual " +
            (error ? std::to_string(static_cast<int>(*error)) : "decoded value") +
            ", payload length " + std::to_string(event.frame->len));
    }
}

}  // namespace

int main() {
    using namespace kpi4::protocol;
    using kpi4::core::AltitudeReference;

    const auto event = read_wire(
        packed(4'000'000'000u, -432'345'678, 1'791'234'567,
               -12'345, 2'500, -321, 456, -70, 27'001),
        InputChannel::ControllerX);
    const auto& position = decoded(event);
    require(position.channel == InputChannel::ControllerX &&
                position.sender_sysid == 7 && position.sender_compid == 1 &&
                position.time_boot_ms == 4'000'000'000u,
            "sender metadata or boot time changed");
    require_near(position.position.latitude.value, -43.2345678,
                 "latitude degE7 scale or sign is wrong");
    require_near(position.position.longitude.value, 179.1234567,
                 "longitude degE7 scale or sign is wrong");
    require(position.altitude_msl.reference ==
                AltitudeReference::mean_sea_level &&
                position.altitude_relative_to_home.reference ==
                    AltitudeReference::relative_to_home,
            "altitude references were mixed");
    require_near(position.altitude_msl.value.value, -12.345,
                 "MSL altitude millimeters were not converted");
    require_near(position.altitude_relative_to_home.value.value, 2.5,
                 "home-relative altitude millimeters were not converted");
    require_near(position.horizontal_velocity.north.value, -3.21,
                 "north velocity sign or cm/s scale is wrong");
    require_near(position.horizontal_velocity.east.value, 4.56,
                 "east velocity sign or cm/s scale is wrong");
    require_near(position.velocity_down.value, -0.7,
                 "down-positive vertical velocity was reversed");
    require(position.heading.has_value(), "known vehicle heading was lost");
    require_near(position.heading->value, 270.01,
                 "heading centidegrees were not converted");

    const auto unknown_heading = read_wire(
        packed(0, 0, 0, 0, 0, 0, 0, 0,
               std::numeric_limits<std::uint16_t>::max()),
        InputChannel::GroundStation);
    const auto& at_origin = decoded(unknown_heading);
    require(at_origin.channel == InputChannel::GroundStation &&
                at_origin.time_boot_ms == 0 && !at_origin.heading &&
                at_origin.position.latitude.value == 0.0 &&
                at_origin.position.longitude.value == 0.0,
            "zero coordinate/time or unknown heading was misinterpreted");

    const auto trimmed = read_wire(
        packed(0, 0, 0, 0, 0, 0, 0, 0, 0), InputChannel::ControllerX);
    require(trimmed.frame->len == 1,
            "MAVLink 2 all-zero payload was not trimmed to one byte");
    const auto zero_heading = decoded(trimmed);
    require(zero_heading.heading && zero_heading.heading->value == 0.0 &&
                zero_heading.velocity_down.value == 0.0,
            "trimmed zero fields were not restored before decoding");

    const auto& bounds = decoded(read_wire(
        packed(1, 900'000'000, -1'800'000'000, 0, 0,
               std::numeric_limits<std::int16_t>::min(),
               std::numeric_limits<std::int16_t>::max(), 0, 35'999),
        InputChannel::ControllerX));
    require(bounds.position.latitude.value == 90.0 &&
                bounds.position.longitude.value == -180.0,
            "valid coordinate boundaries were rejected");
    require_near(bounds.horizontal_velocity.north.value, -327.68,
                 "minimum signed velocity was changed");
    require_near(bounds.horizontal_velocity.east.value, 327.67,
                 "maximum signed velocity was changed");
    require(bounds.heading.has_value(), "maximum valid heading was lost");
    require_near(bounds.heading->value, 359.99,
                 "maximum valid heading was scaled incorrectly");

    expect_error(packed(1, 900'000'001, 0, 0, 0, 0, 0, 0, 0),
                 EstimatedPositionError::LatitudeOutOfRange);
    expect_error(packed(1, -900'000'001, 0, 0, 0, 0, 0, 0, 0),
                 EstimatedPositionError::LatitudeOutOfRange);
    expect_error(packed(1, 0, 1'800'000'001, 0, 0, 0, 0, 0, 0),
                 EstimatedPositionError::LongitudeOutOfRange);
    expect_error(packed(1, 0, -1'800'000'001, 0, 0, 0, 0, 0, 0),
                 EstimatedPositionError::LongitudeOutOfRange);
    expect_error(packed(1, 0, 0, 0, 0, 0, 0, 0, 36'000),
                 EstimatedPositionError::HeadingOutOfRange);
    expect_error(packed(1, 0, 0, 0, 0, 0, 0, 0, 65'534),
                 EstimatedPositionError::HeadingOutOfRange);

    auto malformed = event;
    malformed.frame->magic = MAVLINK_STX_MAVLINK1;
    malformed.frame->len = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN - 1;
    const auto short_result = decode_global_position_int(malformed);
    require(short_result && std::get_if<EstimatedPositionError>(&*short_result) &&
                std::get<EstimatedPositionError>(*short_result) ==
                    EstimatedPositionError::InvalidLength,
            "short position payload was decoded");
    malformed.frame->len = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN + 1;
    const auto long_result = decode_global_position_int(malformed);
    require(long_result && std::get_if<EstimatedPositionError>(&*long_result) &&
                std::get<EstimatedPositionError>(*long_result) ==
                    EstimatedPositionError::InvalidLength,
            "oversized position payload was decoded");
    malformed.frame->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    require(!decode_global_position_int(malformed),
            "unrelated message was decoded as position");
    malformed.frame.reset();
    require(!decode_global_position_int(malformed),
            "absent frame was decoded as position");
}
