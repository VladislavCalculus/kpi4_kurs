#include <kpi4/protocol/frame_reader.hpp>

namespace kpi4::protocol {

FrameReadResult FrameReader::consume(InputChannel channel, std::uint8_t byte) {
    auto& state = channels_[static_cast<std::size_t>(channel)];
    const auto framing = mavlink_frame_char_buffer(
        &state.receive_buffer, &state.status, byte, &state.candidate,
        &state.result_status);

    if (framing == MAVLINK_FRAMING_OK) {
        ++state.accepted;
        return {channel, state.candidate, std::nullopt};
    }

    std::optional<FrameRejectReason> reason;
    if (framing == MAVLINK_FRAMING_BAD_CRC) {
        reason = FrameRejectReason::BadCrc;
    } else if (framing == MAVLINK_FRAMING_BAD_SIGNATURE) {
        reason = FrameRejectReason::BadSignature;
    } else if (state.result_status.packet_rx_drop_count != 0) {
        reason = FrameRejectReason::InvalidHeader;
    }
    if (reason) {
        ++state.rejected;
        if (state.status.parse_state == MAVLINK_PARSE_STATE_IDLE &&
            (byte == MAVLINK_STX || byte == MAVLINK_STX_MAVLINK1)) {
            mavlink_frame_char_buffer(
                &state.receive_buffer, &state.status, byte, &state.candidate,
                &state.result_status);
        }
    }
    return {channel, std::nullopt, reason};
}

FrameCounts FrameReader::counts(InputChannel channel) const {
    const auto& state = channels_[static_cast<std::size_t>(channel)];
    const bool pending = state.status.parse_state != MAVLINK_PARSE_STATE_IDLE &&
                         state.status.parse_state != MAVLINK_PARSE_STATE_UNINIT;
    return {state.accepted, state.rejected, pending ? 1u : 0u};
}

}  // namespace kpi4::protocol
