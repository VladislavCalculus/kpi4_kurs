#pragma once

#include <common/mavlink.h>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace kpi4::protocol {

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
    std::optional<mavlink_message_t> frame;
    std::optional<FrameRejectReason> rejection;
};

class FrameReader {
public:
    FrameReadResult consume(std::uint8_t byte);
    FrameCounts counts() const;

private:
    mavlink_message_t receive_buffer_{};
    mavlink_message_t candidate_{};
    mavlink_status_t status_{};
    mavlink_status_t result_status_{};
    std::size_t accepted_ = 0;
    std::size_t rejected_ = 0;
};

}  // namespace kpi4::protocol
