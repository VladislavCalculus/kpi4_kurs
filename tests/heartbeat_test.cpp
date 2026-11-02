#include <kpi4/protocol/heartbeat.hpp>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

kpi4::protocol::FrameReadResult read_fixture(
    const char* name, kpi4::protocol::InputChannel channel) {
    const std::string path = std::string{KPI4_FIXTURE_DIR} + '/' + name;
    std::ifstream file(path, std::ios::binary);
    require(static_cast<bool>(file), "fixture could not be opened");
    const std::vector<std::uint8_t> bytes{
        std::istreambuf_iterator<char>{file}, std::istreambuf_iterator<char>{}};
    kpi4::protocol::FrameReader reader;
    kpi4::protocol::FrameReadResult result{channel, std::nullopt, std::nullopt};
    for (const auto byte : bytes) {
        const auto event = reader.consume(channel, byte);
        require(!event.rejection, "fixture was rejected");
        if (event.frame) {
            require(!result.frame, "fixture has multiple frames");
            result = event;
        }
    }
    require(result.frame.has_value(), "fixture has no complete frame");
    return result;
}

}  // namespace

int main() {
    using namespace kpi4::protocol;

    const auto gcs = decode_heartbeat(
        read_fixture("heartbeat_gcs_v2.bin", InputChannel::ControllerX));
    require(gcs.has_value(), "GCS heartbeat was not decoded");
    require(gcs->channel == InputChannel::ControllerX, "input channel changed");
    require(gcs->sysid == 42 && gcs->compid == 190, "sender IDs changed");
    require(gcs->type == MAV_TYPE_GCS && gcs->autopilot == MAV_AUTOPILOT_INVALID,
            "GCS identity changed");
    require(gcs->system_status == MAV_STATE_STANDBY, "GCS status changed");
    require(gcs->base_mode == 4 && gcs->custom_mode == 0x12345678 &&
                gcs->mavlink_version == 3,
            "heartbeat mode or version changed");
    require(heartbeat_type_name(gcs->type) == "GCS" &&
                heartbeat_autopilot_name(gcs->autopilot) == "INVALID" &&
                heartbeat_state_name(gcs->system_status) == "STANDBY",
            "GCS enum names are wrong");

    const auto vehicle = decode_heartbeat(
        read_fixture("heartbeat_vehicle_v2.bin", InputChannel::GroundStation));
    require(vehicle.has_value(), "vehicle heartbeat was not decoded");
    require(vehicle->channel == InputChannel::GroundStation, "vehicle channel changed");
    require(vehicle->sysid == 7 && vehicle->compid == 1 &&
                vehicle->type == MAV_TYPE_QUADROTOR &&
                vehicle->autopilot == MAV_AUTOPILOT_PX4 &&
                vehicle->system_status == MAV_STATE_ACTIVE,
            "vehicle identity or status changed");
    require(heartbeat_type_name(vehicle->type) == "QUADROTOR" &&
                heartbeat_autopilot_name(vehicle->autopilot) == "PX4" &&
                heartbeat_state_name(vehicle->system_status) == "ACTIVE",
            "vehicle enum names are wrong");

    const auto unknown = decode_heartbeat(
        read_fixture("heartbeat_unknown_v2.bin", InputChannel::Unspecified));
    require(unknown.has_value() && unknown->type == 255 &&
                unknown->autopilot == 255 && unknown->system_status == 255,
            "unknown enum values were lost");
    require(heartbeat_type_name(unknown->type) == "UNKNOWN" &&
                heartbeat_autopilot_name(unknown->autopilot) == "UNKNOWN" &&
                heartbeat_state_name(unknown->system_status) == "UNKNOWN",
            "unknown enum values were silently interpreted");
    require(heartbeat_type_name(MAV_TYPE_VTOL_RESERVED5) == "RESERVED" &&
                heartbeat_autopilot_name(MAV_AUTOPILOT_RESERVED) == "RESERVED",
            "reserved values were not identified");

    const auto legacy = decode_heartbeat(
        read_fixture("heartbeat_v1.bin", InputChannel::Unspecified));
    require(legacy && legacy->type == MAV_TYPE_GENERIC &&
                legacy->system_status == MAV_STATE_UNINIT,
            "MAVLink 1 heartbeat was not decoded");

    auto non_heartbeat = read_fixture("heartbeat_v2.bin", InputChannel::Unspecified);
    non_heartbeat.frame->msgid = MAVLINK_MSG_ID_PING;
    require(!decode_heartbeat(non_heartbeat), "non-heartbeat was decoded");
    non_heartbeat.frame->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    non_heartbeat.frame->len = MAVLINK_MSG_ID_HEARTBEAT_LEN - 1;
    require(!decode_heartbeat(non_heartbeat), "short heartbeat was decoded");
    non_heartbeat.frame.reset();
    require(!decode_heartbeat(non_heartbeat), "absent frame was decoded");
}
