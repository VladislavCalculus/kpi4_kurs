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

class FrameReader {
public:
    std::optional<mavlink_message_t> consume(std::uint8_t byte);
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
