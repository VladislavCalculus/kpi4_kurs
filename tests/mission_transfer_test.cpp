#include <kpi4/protocol/mission_transfer.hpp>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
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

template <typename T>
const T& payload_as(const kpi4::protocol::MissionTransferEvent& event) {
    const auto* result = std::get_if<T>(&event.payload);
    require(result != nullptr, "wrong mission event type");
    return *result;
}

}  // namespace

int main() {
    using namespace kpi4::protocol;
    mavlink_message_t message{};

    mavlink_msg_mission_count_pack(42, 190, &message, 70000, 1, 513, 2,
                                   0xFEDCBA98);
    auto count_event = decode_mission_transfer(
        read_wire(message, InputChannel::GroundStation));
    require(count_event.has_value(), "count was not decoded");
    require(count_event->channel == InputChannel::GroundStation &&
                count_event->sender_sysid == 42 && count_event->sender_compid == 190,
            "count sender or channel changed");
    const auto& count = payload_as<MissionCount>(*count_event);
    require(count.address.target_system == 70000 &&
                count.address.target_component == 1 &&
                count.address.mission_type == 2 && count.count == 513 &&
                count.opaque_id == 0xFEDCBA98,
            "count payload changed or wide target was truncated");

    mavlink_msg_mission_request_int_pack(7, 1, &message, 42, 190, 65535, 0);
    auto request_event = decode_mission_transfer(
        read_wire(message, InputChannel::ControllerX));
    require(request_event.has_value(), "request was not decoded");
    require(request_event->sender_sysid == 7 && request_event->sender_compid == 1 &&
                request_event->channel == InputChannel::ControllerX,
            "request sender or channel changed");
    const auto& request = payload_as<MissionRequestInt>(*request_event);
    require(request.address.target_system == 42 &&
                request.address.target_component == 190 &&
                request.address.mission_type == 0 && request.seq == 65535,
            "request payload or default mission type changed");

    mavlink_msg_mission_item_int_pack(
        42, 190, &message, 7, 1, 257, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_WAYPOINT, 1, 0, 1.125f, -2.5f, 3.75f, -4.125f,
        -2147483647, 2147483647, 123.875f, 1);
    auto item_event = decode_mission_transfer(
        read_wire(message, InputChannel::GroundStation));
    require(item_event.has_value(), "item was not decoded");
    const auto& item = payload_as<MissionItemInt>(*item_event);
    require(item.address.target_system == 7 &&
                item.address.target_component == 1 &&
                item.address.mission_type == 1 && item.seq == 257 &&
                item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
                item.command == MAV_CMD_NAV_WAYPOINT &&
                item.current == 1 && item.autocontinue == 0,
            "item routing or flags changed");
    require(item.param1 == 1.125f && item.param2 == -2.5f &&
                item.param3 == 3.75f && item.param4 == -4.125f &&
                item.x == -2147483647 && item.y == 2147483647 &&
                item.z == 123.875f,
            "item coordinates or float parameters lost precision");

    mavlink_msg_mission_ack_pack(7, 1, &message, 42, 190,
                                 MAV_MISSION_INVALID_SEQUENCE, 1, 0x12345678);
    auto ack_event = decode_mission_transfer(
        read_wire(message, InputChannel::ControllerX));
    require(ack_event.has_value(), "ack was not decoded");
    const auto& ack = payload_as<MissionAck>(*ack_event);
    require(ack.address.target_system == 42 &&
                ack.address.target_component == 190 &&
                ack.address.mission_type == 1 &&
                ack.result == MAV_MISSION_INVALID_SEQUENCE &&
                ack.opaque_id == 0x12345678,
            "ack result or routing changed");

    auto invalid = read_wire(message, InputChannel::Unspecified);
    invalid.frame->len = MAVLINK_MSG_ID_MISSION_ACK_MIN_LEN - 1;
    require(!decode_mission_transfer(invalid), "short ack was decoded");
    invalid.frame->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    require(!decode_mission_transfer(invalid), "unrelated message was decoded");
    invalid.frame.reset();
    require(!decode_mission_transfer(invalid), "absent frame was decoded");
}
