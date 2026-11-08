#include <kpi4/protocol/mission_upload.hpp>

#include <kpi4/protocol/mission_command_validator.hpp>
#include <kpi4/protocol/mission_normalizer.hpp>

#include <utility>

namespace kpi4::protocol {
namespace {

MissionUploadPeer peer_of(const MissionTransferEvent& event) {
    return {event.channel, event.sender_sysid, event.sender_compid};
}

bool same_peer(const MissionUploadPeer& left, const MissionUploadPeer& right) {
    return left.channel == right.channel && left.system_id == right.system_id &&
           left.component_id == right.component_id;
}

MissionUploadAck ack_to(const MissionUploadPeer& target,
                        std::uint8_t mission_type, std::uint8_t result) {
    return {target, mission_type, result};
}

}  // namespace

MissionUploadAssembler::MissionUploadAssembler(std::uint32_t system_id,
                                               std::uint8_t component_id,
                                               std::uint16_t max_items)
    : system_id_(system_id), component_id_(component_id),
      max_items_(max_items) {}

std::optional<std::uint16_t> MissionUploadAssembler::expected_sequence() const {
    if (!pending_) {
        return std::nullopt;
    }
    return pending_->next_seq;
}

std::optional<MissionUploadAck>
MissionUploadAssembler::finish_after_plan_decision(std::uint8_t result) {
    if (!awaiting_plan_decision_) {
        return std::nullopt;
    }
    auto ack = *awaiting_plan_decision_;
    ack.result = result;
    awaiting_plan_decision_.reset();
    return ack;
}

MissionUploadTransition MissionUploadAssembler::ingest(
    const MissionTransferEvent& event) {
    const auto source = peer_of(event);
    if (const auto* count = std::get_if<MissionCount>(&event.payload)) {
        if (max_items_ == 0 || system_id_ == 0) {
            return MissionUploadRejection{MissionUploadError::InvalidConfiguration,
                                          std::nullopt};
        }
        if (count->address.target_system != system_id_ ||
            (count->address.target_component != component_id_ &&
             count->address.target_component != 0)) {
            return MissionUploadRejection{MissionUploadError::WrongTarget,
                                          std::nullopt};
        }
        if (pending_ || awaiting_plan_decision_) {
            return MissionUploadRejection{MissionUploadError::UploadInProgress,
                                          std::nullopt};
        }
        if (count->address.mission_type != MAV_MISSION_TYPE_MISSION) {
            return MissionUploadRejection{
                MissionUploadError::UnsupportedMissionType,
                ack_to(source, count->address.mission_type,
                       MAV_MISSION_UNSUPPORTED)};
        }
        if (count->count == 0) {
            // The protocol defines count=0 as clear; this assembler only
            // constructs nonempty candidate plans.
            return MissionUploadRejection{
                MissionUploadError::EmptyMissionUnsupported,
                ack_to(source, count->address.mission_type, MAV_MISSION_ERROR)};
        }
        if (count->count > max_items_) {
            return MissionUploadRejection{
                MissionUploadError::TooManyItems,
                ack_to(source, count->address.mission_type, MAV_MISSION_NO_SPACE)};
        }

        pending_ = Pending{source, count->address, count->count, 0, {}, {}};
        pending_->items.reserve(count->count);
        pending_->raw_items.reserve(count->count);
        return MissionUploadRequest{source, count->address.mission_type, 0};
    }

    const auto* item = std::get_if<MissionItemInt>(&event.payload);
    if (!item) {
        return std::monostate{};
    }
    if (!pending_) {
        return MissionUploadRejection{MissionUploadError::NoUploadInProgress,
                                      std::nullopt};
    }
    if (!same_peer(source, pending_->source)) {
        return MissionUploadRejection{MissionUploadError::WrongSource,
                                      std::nullopt};
    }
    if (item->address.target_system != system_id_ ||
        (item->address.target_component != 0 &&
         item->address.target_component != component_id_) ||
        item->address.mission_type != pending_->address.mission_type) {
        return MissionUploadRejection{MissionUploadError::AddressMismatch,
                                      std::nullopt};
    }
    if (item->seq != pending_->next_seq) {
        // Timeout, duplicate and re-request behavior belongs to T02.10.
        return MissionUploadRejection{MissionUploadError::UnexpectedSequence,
                                      std::nullopt};
    }
    if (std::holds_alternative<MissionCommandRejection>(
            validate_mission_item_command(*item))) {
        const auto ack = ack_to(source, item->address.mission_type,
                                MAV_MISSION_UNSUPPORTED);
        pending_.reset();
        return MissionUploadRejection{MissionUploadError::UnsupportedCommand, ack};
    }
    auto normalized = normalize_mission_item(*item);
    if (const auto* error = std::get_if<MissionNormalizationError>(&normalized)) {
        const auto result = *error == MissionNormalizationError::UnsupportedFrame
                                ? MAV_MISSION_UNSUPPORTED_FRAME
                                : MAV_MISSION_INVALID;
        const auto ack = ack_to(source, item->address.mission_type, result);
        pending_.reset();
        return MissionUploadRejection{MissionUploadError::InvalidItem, ack};
    }

    pending_->items.push_back(std::get<core::MissionItem>(std::move(normalized)));
    pending_->raw_items.push_back(*item);
    ++pending_->next_seq;
    if (pending_->next_seq < pending_->count) {
        return MissionUploadRequest{source, pending_->address.mission_type,
                                    pending_->next_seq};
    }

    MissionUploadCandidate candidate{pending_->source,
                                     pending_->address.mission_type,
                                     std::move(pending_->items),
                                     std::move(pending_->raw_items)};
    pending_.reset();
    awaiting_plan_decision_ =
        ack_to(source, item->address.mission_type, MAV_MISSION_ERROR);
    return MissionUploadCompleted{std::move(candidate)};
}

}  // namespace kpi4::protocol
