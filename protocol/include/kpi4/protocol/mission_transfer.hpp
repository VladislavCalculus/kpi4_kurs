#pragma once

#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <optional>
#include <variant>

namespace kpi4::protocol {

struct MissionAddress {
    std::uint32_t target_system;
    std::uint8_t target_component;
    std::uint8_t mission_type;
};

struct MissionCount {
    MissionAddress address;
    std::uint16_t count;
    std::uint32_t opaque_id;
};

struct MissionRequestInt {
    MissionAddress address;
    std::uint16_t seq;
};

struct MissionItemInt {
    MissionAddress address;
    std::uint16_t seq;
    std::uint8_t frame;
    std::uint16_t command;
    std::uint8_t current;
    std::uint8_t autocontinue;
    float param1;
    float param2;
    float param3;
    float param4;
    std::int32_t x;
    std::int32_t y;
    float z;
};

struct MissionAck {
    MissionAddress address;
    std::uint8_t result;
    std::uint32_t opaque_id;
};

using MissionTransferPayload =
    std::variant<MissionCount, MissionRequestInt, MissionItemInt, MissionAck>;

struct MissionTransferEvent {
    InputChannel channel;
    std::uint32_t sender_sysid;
    std::uint8_t sender_compid;
    MissionTransferPayload payload;
};

// Accepts complete, CRC-validated frames from FrameReader. Unknown message IDs
// and payloads shorter than their MAVLink minimum length yield no event.
std::optional<MissionTransferEvent> decode_mission_transfer(
    const FrameReadResult& event);

}  // namespace kpi4::protocol
