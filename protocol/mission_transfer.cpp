#include <kpi4/protocol/mission_transfer.hpp>

namespace kpi4::protocol {
namespace {

MissionAddress address_of(const mavlink_message_t& frame,
                          std::uint8_t target_component,
                          std::uint8_t mission_type) {
    return {mavlink_msg_get_target_sysid(
                &frame, mavlink_get_msg_entry(frame.msgid)),
            target_component, mission_type};
}

}  // namespace

std::optional<MissionTransferEvent> decode_mission_transfer(
    const FrameReadResult& event) {
    if (!event.frame) {
        return std::nullopt;
    }
    const auto& frame = *event.frame;
    const auto wrap = [&](MissionTransferPayload payload) {
        return MissionTransferEvent{event.channel, frame.sysid, frame.compid,
                                    payload};
    };

    switch (frame.msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT: {
            if (frame.len < MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN) {
                return std::nullopt;
            }
            mavlink_mission_count_t raw{};
            mavlink_msg_mission_count_decode(&frame, &raw);
            return wrap(MissionCount{address_of(frame, raw.target_component,
                                                raw.mission_type),
                                     raw.count, raw.opaque_id});
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
            if (frame.len < MAVLINK_MSG_ID_MISSION_REQUEST_INT_MIN_LEN) {
                return std::nullopt;
            }
            mavlink_mission_request_int_t raw{};
            mavlink_msg_mission_request_int_decode(&frame, &raw);
            return wrap(MissionRequestInt{address_of(frame, raw.target_component,
                                                     raw.mission_type),
                                          raw.seq});
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            if (frame.len < MAVLINK_MSG_ID_MISSION_ITEM_INT_MIN_LEN) {
                return std::nullopt;
            }
            mavlink_mission_item_int_t raw{};
            mavlink_msg_mission_item_int_decode(&frame, &raw);
            return wrap(MissionItemInt{
                address_of(frame, raw.target_component, raw.mission_type),
                raw.seq, raw.frame, raw.command, raw.current, raw.autocontinue,
                raw.param1, raw.param2, raw.param3, raw.param4,
                raw.x, raw.y, raw.z});
        }
        case MAVLINK_MSG_ID_MISSION_ACK: {
            if (frame.len < MAVLINK_MSG_ID_MISSION_ACK_MIN_LEN) {
                return std::nullopt;
            }
            mavlink_mission_ack_t raw{};
            mavlink_msg_mission_ack_decode(&frame, &raw);
            return wrap(MissionAck{address_of(frame, raw.target_component,
                                              raw.mission_type),
                                   raw.type, raw.opaque_id});
        }
        default:
            return std::nullopt;
    }
}

}  // namespace kpi4::protocol
