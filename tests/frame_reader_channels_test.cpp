#include <kpi4/protocol/frame_reader.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Channel = kpi4::protocol::InputChannel;
using Result = kpi4::protocol::FrameReadResult;
using Bytes = std::vector<std::uint8_t>;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Bytes load_fixture(const char* name) {
    std::ifstream stream(std::string{KPI4_FIXTURE_DIR} + '/' + name,
                         std::ios::binary);
    require(static_cast<bool>(stream), "fixture could not be opened");
    return {std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

void feed(kpi4::protocol::FrameReader& reader, Channel channel,
          const Bytes& bytes, std::size_t begin, std::size_t end,
          std::vector<Result>& events) {
    for (std::size_t i = begin; i < end; ++i) {
        const auto result = reader.consume(channel, bytes[i]);
        require(result.channel == channel, "result lost source channel");
        if (result.frame || result.rejection) {
            events.push_back(result);
        }
    }
}

}  // namespace

int main() {
    const auto gs_frame = load_fixture("heartbeat_v1.bin");
    const auto controller_frame = load_fixture("heartbeat_v2.bin");
    const auto bad_crc = load_fixture("heartbeat_v2_bad_crc.bin");
    kpi4::protocol::FrameReader reader;
    std::vector<Result> events;

    const auto longest = std::max(gs_frame.size(), controller_frame.size());
    for (std::size_t i = 0; i < longest; ++i) {
        if (i < gs_frame.size()) {
            feed(reader, Channel::GroundStation, gs_frame, i, i + 1, events);
        }
        if (i < controller_frame.size()) {
            feed(reader, Channel::ControllerX, controller_frame, i, i + 1, events);
        }
    }
    require(events.size() == 2, "interleaved frames were not both decoded");
    require(events[0].channel == Channel::GroundStation && events[0].frame &&
                events[0].frame->magic == MAVLINK_STX_MAVLINK1 &&
                !events[0].rejection,
            "Ground Station frame has wrong origin or version");
    require(events[1].channel == Channel::ControllerX && events[1].frame &&
                events[1].frame->magic == MAVLINK_STX && !events[1].rejection,
            "controller frame has wrong origin or version");

    events.clear();
    feed(reader, Channel::GroundStation, bad_crc, 0, 10, events);
    feed(reader, Channel::ControllerX, controller_frame, 0, 7, events);
    require(reader.counts(Channel::GroundStation).incomplete == 1 &&
                reader.counts(Channel::ControllerX).incomplete == 1,
            "partial frames did not remain independent");
    feed(reader, Channel::GroundStation, bad_crc, 10, bad_crc.size(), events);
    feed(reader, Channel::ControllerX, controller_frame, 7,
         controller_frame.size(), events);
    require(events.size() == 2, "interleaved error and frame events differ");
    require(events[0].channel == Channel::GroundStation &&
                !events[0].frame && events[0].rejection &&
                *events[0].rejection == kpi4::protocol::FrameRejectReason::BadCrc,
            "Ground Station rejection has wrong origin or reason");
    require(events[1].channel == Channel::ControllerX && events[1].frame &&
                !events[1].rejection,
            "controller frame was affected by Ground Station CRC error");

    const auto gs_counts = reader.counts(Channel::GroundStation);
    const auto controller_counts = reader.counts(Channel::ControllerX);
    const auto unspecified_counts = reader.counts(Channel::Unspecified);
    require(gs_counts.accepted == 1 && gs_counts.rejected == 1 &&
                gs_counts.incomplete == 0,
            "Ground Station counts differ");
    require(controller_counts.accepted == 2 && controller_counts.rejected == 0 &&
                controller_counts.incomplete == 0,
            "controller counts differ");
    require(unspecified_counts.accepted == 0 &&
                unspecified_counts.rejected == 0 &&
                unspecified_counts.incomplete == 0,
            "unrelated channel was modified");
}
