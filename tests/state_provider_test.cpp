#include <kpi4/state/mavlink_provider.hpp>
#include <kpi4/state/simulated_provider.hpp>

#include <array>
#include <cmath>
#include <cstdint>
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

kpi4::protocol::FrameReadResult read_wire(const mavlink_message_t& message) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> wire{};
    const auto count = mavlink_msg_to_send_buffer(wire.data(), &message);
    kpi4::protocol::FrameReader reader;
    kpi4::protocol::FrameReadResult result{
        kpi4::protocol::InputChannel::ControllerX, std::nullopt, std::nullopt};
    for (std::size_t i = 0; i < count; ++i) {
        const auto event = reader.consume(result.channel, wire[i]);
        require(!event.rejection, "packed state frame was rejected");
        if (event.frame) {
            result = event;
        }
    }
    require(result.frame.has_value(), "state frame was not emitted");
    return result;
}

template <typename T, typename Result>
T decoded(const std::optional<Result>& result) {
    require(result.has_value(), "state frame was ignored");
    const auto* value = std::get_if<T>(&*result);
    require(value != nullptr, "state frame was rejected");
    return *value;
}

void same_measurement(const kpi4::core::StateSample& actual,
                      const kpi4::core::StateSample& expected) {
    require(actual.measured_at.value == expected.measured_at.value &&
                actual.received_at.value == expected.received_at.value,
            "monotonic measurement or reception time differs");
    near(actual.position.latitude.value, expected.position.latitude.value,
         "latitude differs");
    near(actual.position.longitude.value, expected.position.longitude.value,
         "longitude differs");
    require(actual.altitude.has_value() == expected.altitude.has_value() &&
                actual.velocity.has_value() == expected.velocity.has_value() &&
                actual.heading_true_north.has_value() ==
                    expected.heading_true_north.has_value() &&
                actual.horizontal_accuracy.has_value() ==
                    expected.horizontal_accuracy.has_value() &&
                actual.gps_fix == expected.gps_fix,
            "optional state fields differ");
    if (actual.altitude) {
        require(actual.altitude->reference == expected.altitude->reference,
                "altitude reference differs");
        near(actual.altitude->value.value, expected.altitude->value.value,
             "altitude differs");
    }
    if (actual.velocity) {
        near(actual.velocity->north.value, expected.velocity->north.value,
             "north velocity differs");
        near(actual.velocity->east.value, expected.velocity->east.value,
             "east velocity differs");
    }
    if (actual.heading_true_north) {
        near(actual.heading_true_north->value,
             expected.heading_true_north->value, "heading differs");
    }
    if (actual.horizontal_accuracy) {
        near(actual.horizontal_accuracy->value,
             expected.horizontal_accuracy->value, "accuracy differs");
    }
}

}  // namespace

int main() {
    using namespace kpi4;
    const state::SampleTimes times{{1'000}, {1'250}};
    state::MavlinkStateProvider mavlink(core::StateSource::controller_telemetry);
    state::SimulatedStateProvider simulated;
    state::StateProvider& live_interface = mavlink;
    state::StateProvider& simulation_interface = simulated;
    require(!live_interface.take_latest() && !simulation_interface.take_latest(),
            "empty provider produced a sample");

    mavlink_message_t estimated_message{};
    mavlink_msg_global_position_int_pack(
        9, 3, &estimated_message, 4'000'000'000u, -432'345'678,
        1'791'234'567, -12'345, 2'500, -321, 456, -70, 27'001);
    const auto estimated = decoded<protocol::EstimatedPosition>(
        protocol::decode_global_position_int(read_wire(estimated_message)));
    mavlink.accept(estimated, times);
    simulated.publish(core::StateSample{
        times.measured_at, times.received_at, core::StateSource::ground_station,
        {{-43.2345678}, {179.1234567}},
        core::Altitude{{-12.345}, core::AltitudeReference::mean_sea_level},
        core::NorthEastVelocity{{-3.21}, {4.56}}, core::Degrees{270.01},
        std::nullopt, std::nullopt});
    const auto from_mavlink = live_interface.take_latest();
    const auto from_simulation = simulation_interface.take_latest();
    require(from_mavlink && from_simulation &&
                from_mavlink->source == core::StateSource::controller_telemetry &&
                from_simulation->source == core::StateSource::simulation,
            "state provider lost or mislabelled source");
    same_measurement(*from_mavlink, *from_simulation);
    require(!live_interface.take_latest() && !simulation_interface.take_latest(),
            "sample was delivered twice");

    mavlink_gps_raw_int_t raw{};
    raw.time_usec = 1'700'000'000'123'456ULL;
    raw.fix_type = GPS_FIX_TYPE_3D_FIX;
    raw.lat = 100'000'000;
    raw.lon = -200'000'000;
    raw.alt = 1'000;
    raw.eph = 10'000;  // HDOP 100 is not horizontal accuracy in meters.
    raw.epv = 200;
    raw.vel = 500;
    raw.cog = 9'000;
    raw.h_acc = 250;
    raw.yaw = 27'000;  // Yaw differs from course over ground.
    mavlink_message_t raw_message{};
    mavlink_msg_gps_raw_int_encode(9, 3, &raw_message, &raw);
    const auto gps = decoded<protocol::RawGps>(
        protocol::decode_gps_raw_int(read_wire(raw_message)));
    const state::SampleTimes later{{2'000}, {2'100}};
    mavlink.accept(gps, later);
    simulated.publish(core::StateSample{
        later.measured_at, later.received_at, core::StateSource::ground_station,
        {{10.0}, {-20.0}},
        core::Altitude{{1.0}, core::AltitudeReference::mean_sea_level},
        core::NorthEastVelocity{{0.0}, {5.0}}, core::Degrees{270.0},
        core::Meters{0.25}, core::GpsFix::three_dimensional});
    const auto from_raw = live_interface.take_latest();
    const auto simulated_raw = simulation_interface.take_latest();
    require(from_raw && simulated_raw, "raw GPS or simulated sample missing");
    same_measurement(*from_raw, *simulated_raw);
    require(from_raw->measured_at.value != raw.time_usec &&
                from_raw->horizontal_accuracy->value != 100.0,
            "sender time or HDOP was used as module time or meter accuracy");

    auto unknown = gps;
    unknown.fix_type = 254;
    unknown.ground_speed.reset();
    unknown.gps_yaw.reset();
    mavlink.accept(unknown, later);
    const auto incomplete = live_interface.take_latest();
    require(incomplete && !incomplete->gps_fix && !incomplete->velocity &&
                !incomplete->heading_true_north,
            "unknown fix or missing speed/yaw became a valid value");

    auto north_yaw = gps;
    north_yaw.gps_yaw = core::Degrees{360.0};
    mavlink.accept(north_yaw, later);
    const auto north = live_interface.take_latest();
    require(north && north->heading_true_north &&
                north->heading_true_north->value == 0.0,
            "GPS yaw north marker was not normalized to zero degrees");

    mavlink.accept(estimated, times);
    mavlink.accept(gps, later);
    const auto latest = live_interface.take_latest();
    require(latest && latest->measured_at.value == later.measured_at.value &&
                !live_interface.take_latest(),
            "latest-sample delivery did not replace the older observation");
}
