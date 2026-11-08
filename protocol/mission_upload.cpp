#include <kpi4/protocol/mission_upload.hpp>

#include <kpi4/protocol/mission_command_validator.hpp>
#include <kpi4/protocol/mission_normalizer.hpp>

#include <cstring>
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

bool same_item(const MissionItemInt& left, const MissionItemInt& right) {
    const auto same_float = [](const float& a, const float& b) {
        return std::memcmp(&a, &b, sizeof(float)) == 0;
    };
    return left.address.target_system == right.address.target_system &&
           left.address.target_component == right.address.target_component &&
           left.address.mission_type == right.address.mission_type &&
           left.seq == right.seq && left.frame == right.frame &&
           left.command == right.command && left.current == right.current &&
           left.autocontinue == right.autocontinue &&
           same_float(left.param1, right.param1) &&
           same_float(left.param2, right.param2) &&
           same_float(left.param3, right.param3) &&
           same_float(left.param4, right.param4) &&
           left.x == right.x && left.y == right.y &&
           same_float(left.z, right.z);
}

MissionUploadAck ack_to(const MissionUploadPeer& target,
                        std::uint8_t mission_type, std::uint8_t result) {
    return {target, mission_type, result};
}

}  // namespace

MissionUploadAssembler::MissionUploadAssembler(std::uint32_t system_id,
                                               std::uint8_t component_id,
                                               std::uint16_t max_items,
                                               MissionUploadPolicy policy)
    : system_id_(system_id), component_id_(component_id),
      max_items_(max_items), policy_(policy) {}

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
    if (completed_final_item_) {
        last_ack_ = LastAck{ack.target, *completed_final_item_, ack};
        completed_final_item_.reset();
    }
    awaiting_plan_decision_.reset();
    return ack;
}

MissionUploadTransition MissionUploadAssembler::retry_or_cancel(
    core::MonotonicMicroseconds now) {
    if (!pending_) return std::monostate{};
    if (pending_->retries >= policy_.max_retries) {
        const auto ack = ack_to(pending_->source,
                                pending_->address.mission_type,
                                MAV_MISSION_OPERATION_CANCELLED);
        pending_.reset();
        return MissionUploadRejection{MissionUploadError::RetriesExhausted, ack};
    }
    ++pending_->retries;
    if (now.value >= pending_->last_request_at.value)
        pending_->last_request_at = now;
    return MissionUploadRequest{pending_->source,
                                pending_->address.mission_type,
                                pending_->next_seq};
}

MissionUploadTransition MissionUploadAssembler::poll(
    core::MonotonicMicroseconds now) {
    if (!pending_ || now.value < pending_->last_request_at.value ||
        now.value - pending_->last_request_at.value < policy_.item_timeout_us)
        return std::monostate{};
    return retry_or_cancel(now);
}

MissionUploadTransition MissionUploadAssembler::ingest(
    const MissionTransferEvent& event, core::MonotonicMicroseconds now) {
    const auto source = peer_of(event);
    if (const auto* count = std::get_if<MissionCount>(&event.payload)) {
        if (max_items_ == 0 || system_id_ == 0 || policy_.item_timeout_us == 0) {
            return MissionUploadRejection{MissionUploadError::InvalidConfiguration,
                                          std::nullopt};
        }
        if (count->address.target_system != system_id_ ||
            (count->address.target_component != component_id_ &&
             count->address.target_component != 0)) {
            return MissionUploadRejection{MissionUploadError::WrongTarget,
                                          std::nullopt};
        }
        if (pending_ && same_peer(source, pending_->source) &&
            count->count == pending_->count &&
            count->opaque_id == pending_->opaque_id &&
            count->address.target_system == pending_->address.target_system &&
            count->address.target_component == pending_->address.target_component &&
            count->address.mission_type == pending_->address.mission_type) {
            return retry_or_cancel(now);
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

        last_ack_.reset();
        pending_ = Pending{source, count->address, count->count,
                           count->opaque_id, 0, {}, {}, now, 0};
        pending_->items.reserve(count->count);
        pending_->raw_items.reserve(count->count);
        return MissionUploadRequest{source, count->address.mission_type, 0};
    }

    const auto* item = std::get_if<MissionItemInt>(&event.payload);
    if (!item) {
        return std::monostate{};
    }
    if (!pending_) {
        if (last_ack_ && same_peer(source, last_ack_->source) &&
            same_item(*item, last_ack_->final_item))
            return last_ack_->ack;
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
        // An out-of-order or duplicate item is dropped; request the missing
        // sequence again without changing the candidate.
        return retry_or_cancel(now);
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
    pending_->last_request_at = now;
    pending_->retries = 0;
    if (pending_->next_seq < pending_->count) {
        return MissionUploadRequest{source, pending_->address.mission_type,
                                    pending_->next_seq};
    }

    completed_final_item_ = *item;
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
