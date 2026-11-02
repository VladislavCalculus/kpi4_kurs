#pragma once

#include <common/mavlink.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace kpi4::protocol {

enum class InputChannel : std::uint8_t {
    Unspecified,
    GroundStation,
    ControllerX,
};

struct FrameCounts {
    std::size_t accepted = 0;
    std::size_t rejected = 0;
    std::size_t incomplete = 0;
};

enum class FrameRejectReason {
    BadCrc,
    BadSignature,
    InvalidHeader,
};

struct FrameReadResult {
    InputChannel channel;
    std::optional<mavlink_message_t> frame;
    std::optional<FrameRejectReason> rejection;
};

class FrameReader {
public:
    FrameReadResult consume(InputChannel channel, std::uint8_t byte);
    FrameCounts counts(InputChannel channel) const;

private:
    struct ChannelState {
        mavlink_message_t receive_buffer{};
        mavlink_message_t candidate{};
        mavlink_status_t status{};
        mavlink_status_t result_status{};
        std::size_t accepted = 0;
        std::size_t rejected = 0;
    };

    std::array<ChannelState,
               static_cast<std::size_t>(InputChannel::ControllerX) + 1> channels_{};
};

}  // namespace kpi4::protocol
