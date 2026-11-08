#include <kpi4/protocol/mission_upload.hpp>

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

kpi4::protocol::MissionTransferEvent from_wire(
    const mavlink_message_t& message,
    kpi4::protocol::InputChannel channel =
        kpi4::protocol::InputChannel::GroundStation) {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> wire{};
    const auto size = mavlink_msg_to_send_buffer(wire.data(), &message);
    kpi4::protocol::FrameReader reader;
    std::optional<kpi4::protocol::MissionTransferEvent> decoded;
    for (std::size_t i = 0; i < size; ++i) {
        const auto event = reader.consume(channel, wire[i]);
        require(!event.rejection, "mission wire frame was rejected");
        if (event.frame) {
            decoded = kpi4::protocol::decode_mission_transfer(event);
        }
    }
    require(decoded.has_value(), "mission wire frame was not decoded");
    return *decoded;
}

kpi4::protocol::MissionTransferEvent count(std::uint16_t size,
                                           std::uint8_t mission_type =
                                               MAV_MISSION_TYPE_MISSION) {
    mavlink_message_t message{};
    mavlink_msg_mission_count_pack(42, 190, &message, 7, 1, size,
                                   mission_type, 0);
    return from_wire(message);
}

kpi4::protocol::MissionTransferEvent item(
    std::uint16_t sequence, std::uint16_t command = MAV_CMD_NAV_WAYPOINT,
    std::int32_t latitude_e7 = 500'000'000,
    std::uint8_t mission_type = MAV_MISSION_TYPE_MISSION,
    float hold_seconds = 0.0f) {
    mavlink_message_t message{};
    mavlink_msg_mission_item_int_pack(
        42, 190, &message, 7, 1, sequence, MAV_FRAME_GLOBAL_INT,
        command, 0, 1, hold_seconds, 0.0f, 0.0f, 0.0f,
        latitude_e7, 300'000'000, 120.0f, mission_type);
    return from_wire(message);
}

template <typename T>
const T& transition_as(const kpi4::protocol::MissionUploadTransition& value) {
    const auto* result = std::get_if<T>(&value);
    require(result != nullptr, "wrong mission upload transition");
    return *result;
}

}  // namespace

int main() {
    using namespace kpi4::protocol;
    MissionUploadAssembler upload(7, 1, 2);
    require(!upload.expected_sequence() &&
                !upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED),
            "empty upload exposed a sequence or ACK");

    const auto start = upload.ingest(count(2));
    const auto& request0 = transition_as<MissionUploadRequest>(start);
    require(request0.seq == 0 && request0.target.channel ==
                InputChannel::GroundStation && request0.target.system_id == 42 &&
                request0.target.component_id == 190 &&
                request0.mission_type == MAV_MISSION_TYPE_MISSION &&
                upload.expected_sequence() == 0,
            "COUNT did not request item zero from the uploader");
    require(!upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED),
            "partial upload received an ACK");

    const auto early = upload.ingest(item(1));
    require(transition_as<MissionUploadRejection>(early).error ==
                MissionUploadError::UnexpectedSequence &&
                upload.expected_sequence() == 0,
            "out-of-sequence item advanced the upload");

    auto foreign = item(0);
    foreign.sender_sysid = 43;
    require(transition_as<MissionUploadRejection>(upload.ingest(foreign)).error ==
                MissionUploadError::WrongSource &&
                upload.expected_sequence() == 0,
            "another sender changed the upload");
    auto other_type = item(0);
    std::get<MissionItemInt>(other_type.payload).address.mission_type =
        MAV_MISSION_TYPE_FENCE;
    require(transition_as<MissionUploadRejection>(upload.ingest(other_type)).error ==
                MissionUploadError::AddressMismatch &&
                upload.expected_sequence() == 0,
            "another mission type changed the upload");
    require(transition_as<MissionUploadRejection>(upload.ingest(count(1))).error ==
                MissionUploadError::UploadInProgress,
            "second COUNT silently replaced the pending plan");

    const auto first = upload.ingest(item(0));
    const auto& request1 = transition_as<MissionUploadRequest>(first);
    require(request1.seq == 1 && upload.expected_sequence() == 1 &&
                !upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED),
            "first item exposed a partial plan or ACK");
    const auto final = upload.ingest(item(1, MAV_CMD_NAV_WAYPOINT,
                                          -400'000'000,
                                          MAV_MISSION_TYPE_MISSION, 3.5f));
    const auto& complete = transition_as<MissionUploadCompleted>(final);
    require(complete.candidate.source.system_id == 42 &&
                complete.candidate.source.component_id == 190 &&
                complete.candidate.mission_type == MAV_MISSION_TYPE_MISSION &&
                complete.candidate.items.size() == 2 &&
                complete.candidate.raw_items.size() == 2 &&
                complete.candidate.items[0].sequence == 0 &&
                complete.candidate.items[1].sequence == 1 &&
                complete.candidate.raw_items[1].command ==
                    MAV_CMD_NAV_WAYPOINT &&
                complete.candidate.raw_items[1].x == -400'000'000 &&
                complete.candidate.raw_items[1].param1 == 3.5f &&
                complete.candidate.items[0].target.latitude.value == 50.0 &&
                complete.candidate.items[1].target.latitude.value == -40.0 &&
                complete.candidate.items[1].altitude_reference ==
                    kpi4::core::AltitudeReference::mean_sea_level,
            "completed candidate is incomplete or misnormalized");
    require(!upload.expected_sequence() &&
                transition_as<MissionUploadRejection>(upload.ingest(count(1))).error ==
                    MissionUploadError::UploadInProgress,
            "new upload started before plan decision and ACK");
    const auto ack = upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED);
    require(ack && ack->target.system_id == 42 &&
                ack->target.component_id == 190 &&
                ack->mission_type == MAV_MISSION_TYPE_MISSION &&
                ack->result == MAV_MISSION_ACCEPTED &&
                !upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED),
            "final ACK was missing, misaddressed or repeated");

    const auto one_start = upload.ingest(count(1));
    require(transition_as<MissionUploadRequest>(one_start).seq == 0,
            "fresh one-item upload did not start");
    const auto one_complete = upload.ingest(item(0));
    require(transition_as<MissionUploadCompleted>(one_complete)
                .candidate.items.size() == 1,
            "one-item upload did not complete");
    const auto rejected_ack = upload.finish_after_plan_decision(MAV_MISSION_ERROR);
    require(rejected_ack && rejected_ack->result == MAV_MISSION_ERROR,
            "plan-owner rejection did not reach the ACK result");

    auto broadcast_count = count(1);
    std::get<MissionCount>(broadcast_count.payload).address.target_component = 0;
    require(transition_as<MissionUploadRequest>(
                upload.ingest(broadcast_count)).seq == 0,
            "broadcast component COUNT was not accepted");
    require(transition_as<MissionUploadCompleted>(upload.ingest(item(0)))
                .candidate.items.size() == 1,
            "directed ITEM did not match broadcast component COUNT");
    require(upload.finish_after_plan_decision(MAV_MISSION_ACCEPTED).has_value(),
            "broadcast-addressed upload was not acknowledged");

    MissionUploadAssembler limited(7, 1, 2);
    const auto wrong_target = count(1);
    auto addressed_elsewhere = wrong_target;
    std::get<MissionCount>(addressed_elsewhere.payload).address.target_system = 8;
    require(transition_as<MissionUploadRejection>(
                limited.ingest(addressed_elsewhere)).error ==
                MissionUploadError::WrongTarget &&
                !limited.expected_sequence(),
            "COUNT addressed elsewhere started an upload");
    const auto unsupported = limited.ingest(count(1, MAV_MISSION_TYPE_FENCE));
    const auto& unsupported_result =
        transition_as<MissionUploadRejection>(unsupported);
    require(unsupported_result.error == MissionUploadError::UnsupportedMissionType &&
                unsupported_result.ack &&
                unsupported_result.ack->result == MAV_MISSION_UNSUPPORTED,
            "unsupported plan type was accepted");
    const auto empty = limited.ingest(count(0));
    require(transition_as<MissionUploadRejection>(empty).error ==
                MissionUploadError::EmptyMissionUnsupported &&
                !limited.expected_sequence(),
            "empty mission became a candidate plan");
    const auto oversized = limited.ingest(count(3));
    require(transition_as<MissionUploadRejection>(oversized).error ==
                MissionUploadError::TooManyItems &&
                transition_as<MissionUploadRejection>(oversized).ack->result ==
                    MAV_MISSION_NO_SPACE,
            "storage limit was ignored");

    limited.ingest(count(2));
    const auto bad_command = limited.ingest(item(0, MAV_CMD_NAV_LAND));
    require(transition_as<MissionUploadRejection>(bad_command).error ==
                MissionUploadError::UnsupportedCommand &&
                transition_as<MissionUploadRejection>(bad_command).ack->result ==
                    MAV_MISSION_UNSUPPORTED &&
                !limited.expected_sequence(),
            "unsupported command leaked into the plan");
    limited.ingest(count(1));
    const auto bad_position = limited.ingest(item(0, MAV_CMD_NAV_WAYPOINT,
                                                  900'000'001));
    require(transition_as<MissionUploadRejection>(bad_position).error ==
                MissionUploadError::InvalidItem &&
                transition_as<MissionUploadRejection>(bad_position).ack->result ==
                    MAV_MISSION_INVALID &&
                !limited.expected_sequence(),
            "invalid coordinate leaked into the plan");

    MissionUploadAssembler invalid_config(7, 1, 0);
    require(transition_as<MissionUploadRejection>(
                invalid_config.ingest(count(1))).error ==
                MissionUploadError::InvalidConfiguration,
            "zero storage configuration was accepted");

    MissionTransferEvent incoming_ack{
        InputChannel::GroundStation, 42, 190,
        MissionAck{{7, 1, MAV_MISSION_TYPE_MISSION}, MAV_MISSION_ACCEPTED, 0}};
    require(std::holds_alternative<std::monostate>(upload.ingest(incoming_ack)),
            "incoming ACK was mistaken for upload completion");
}
