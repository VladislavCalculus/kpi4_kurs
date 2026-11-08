#pragma once

#include <kpi4/core/types.hpp>
#include <kpi4/protocol/mission_transfer.hpp>

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace kpi4::protocol {

struct MissionUploadPeer {
    InputChannel channel;
    std::uint32_t system_id;
    std::uint8_t component_id;
};

// Logical outbound messages. A transport adapter will serialize these later.
struct MissionUploadRequest {
    MissionUploadPeer target;
    std::uint8_t mission_type;
    std::uint16_t seq;
};

struct MissionUploadAck {
    MissionUploadPeer target;
    std::uint8_t mission_type;
    std::uint8_t result;
};

struct MissionUploadCandidate {
    MissionUploadPeer source;
    std::uint8_t mission_type;
    std::vector<core::MissionItem> items;
    // Preserve command parameters and flags for the plan owner's final check.
    std::vector<MissionItemInt> raw_items;
};

struct MissionUploadCompleted {
    MissionUploadCandidate candidate;
};

enum class MissionUploadError {
    InvalidConfiguration,
    WrongTarget,
    UnsupportedMissionType,
    EmptyMissionUnsupported,
    TooManyItems,
    UploadInProgress,
    NoUploadInProgress,
    WrongSource,
    AddressMismatch,
    UnexpectedSequence,
    UnsupportedCommand,
    InvalidItem,
};

struct MissionUploadRejection {
    MissionUploadError error;
    std::optional<MissionUploadAck> ack;
};

using MissionUploadTransition =
    std::variant<std::monostate, MissionUploadRequest,
                 MissionUploadCompleted, MissionUploadRejection>;

class MissionUploadAssembler {
public:
    // Endpoint identity and storage bound are supplied by integration.
    MissionUploadAssembler(std::uint32_t system_id, std::uint8_t component_id,
                           std::uint16_t max_items);

    // Handles GCS COUNT and ITEM_INT for one upload. REQUEST_INT and ACK are
    // outbound roles here; incoming instances are ignored.
    MissionUploadTransition ingest(const MissionTransferEvent& event);
    // The owner accepts/rejects the complete candidate before this method
    // produces the final ACK. Returns no ACK before completion.
    std::optional<MissionUploadAck> finish_after_plan_decision(
        std::uint8_t result);
    std::optional<std::uint16_t> expected_sequence() const;

private:
    struct Pending {
        MissionUploadPeer source;
        MissionAddress address;
        std::uint16_t count;
        std::uint16_t next_seq;
        std::vector<core::MissionItem> items;
        std::vector<MissionItemInt> raw_items;
    };

    std::uint32_t system_id_;
    std::uint8_t component_id_;
    std::uint16_t max_items_;
    std::optional<Pending> pending_;
    std::optional<MissionUploadAck> awaiting_plan_decision_;
};

}  // namespace kpi4::protocol
