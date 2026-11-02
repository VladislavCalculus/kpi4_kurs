#pragma once

#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <optional>
#include <string_view>

namespace kpi4::protocol {

// These are values reported by the sender, not a verified source identity.
struct Heartbeat {
    InputChannel channel;
    std::uint32_t sysid;
    std::uint8_t compid;
    std::uint8_t type;
    std::uint8_t autopilot;
    std::uint8_t base_mode;
    std::uint32_t custom_mode;
    std::uint8_t system_status;
    std::uint8_t mavlink_version;
};

// The caller supplies only a CRC-validated frame from FrameReader.
std::optional<Heartbeat> decode_heartbeat(const FrameReadResult& event);

std::string_view heartbeat_type_name(std::uint8_t value);
std::string_view heartbeat_autopilot_name(std::uint8_t value);
std::string_view heartbeat_state_name(std::uint8_t value);

}  // namespace kpi4::protocol
