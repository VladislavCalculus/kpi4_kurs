#include <kpi4/protocol/frame_reader.hpp>

namespace kpi4::protocol {

std::optional<mavlink_message_t> FrameReader::consume(std::uint8_t byte) {
    const auto framing = mavlink_frame_char_buffer(
        &receive_buffer_, &status_, byte, &candidate_, &result_status_);

    if (framing == MAVLINK_FRAMING_OK) {
        ++accepted_;
        return candidate_;
    }
    if (framing == MAVLINK_FRAMING_BAD_CRC ||
        framing == MAVLINK_FRAMING_BAD_SIGNATURE ||
        result_status_.packet_rx_drop_count != 0) {
        ++rejected_;
    }
    return std::nullopt;
}

FrameCounts FrameReader::counts() const {
    const bool pending = status_.parse_state != MAVLINK_PARSE_STATE_IDLE &&
                         status_.parse_state != MAVLINK_PARSE_STATE_UNINIT;
    return {accepted_, rejected_, pending ? 1u : 0u};
}

}  // namespace kpi4::protocol
