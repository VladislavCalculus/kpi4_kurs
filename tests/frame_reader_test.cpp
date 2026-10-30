#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<std::uint8_t> load_fixture(const char* name) {
    const std::string path = std::string{KPI4_FIXTURE_DIR} + '/' + name;
    std::ifstream stream(path, std::ios::binary);
    require(static_cast<bool>(stream), "fixture could not be opened");
    return {std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

void check_valid(const char* name, std::uint8_t magic) {
    kpi4::protocol::FrameReader reader;
    std::size_t emitted = 0;
    for (const auto byte : load_fixture(name)) {
        const auto event = reader.consume(byte);
        require(!event.rejection, "valid frame was rejected");
        if (event.frame) {
            ++emitted;
            require(event.frame->magic == magic, "wrong MAVLink version");
            require(event.frame->msgid == MAVLINK_MSG_ID_HEARTBEAT, "wrong message ID");
            require(event.frame->sysid == 1, "wrong system ID");
            require(event.frame->compid == 1, "wrong component ID");
        }
    }
    const auto counts = reader.counts();
    require(emitted == 1, "valid frame was not emitted exactly once");
    require(counts.accepted == 1, "valid frame count is wrong");
    require(counts.rejected == 0, "valid frame was rejected");
    require(counts.incomplete == 0, "valid frame remains pending");
}

void check_invalid(const char* name, std::size_t rejected,
                   std::size_t incomplete) {
    kpi4::protocol::FrameReader reader;
    std::size_t bad_crc_events = 0;
    for (const auto byte : load_fixture(name)) {
        const auto event = reader.consume(byte);
        require(!event.frame, "invalid frame was emitted");
        if (event.rejection) {
            require(*event.rejection == kpi4::protocol::FrameRejectReason::BadCrc,
                    "unexpected rejection reason");
            ++bad_crc_events;
        }
    }
    const auto counts = reader.counts();
    require(counts.accepted == 0, "invalid frame was accepted");
    require(counts.rejected == rejected, "rejected count is wrong");
    require(counts.incomplete == incomplete, "incomplete count is wrong");
    require(bad_crc_events == rejected, "bad CRC event count is wrong");
}

}  // namespace

int main() {
    check_valid("heartbeat_v1.bin", MAVLINK_STX_MAVLINK1);
    check_valid("heartbeat_v2.bin", MAVLINK_STX);
    check_invalid("heartbeat_v2_bad_crc.bin", 1, 0);
    check_invalid("heartbeat_v2_truncated.bin", 0, 1);

    kpi4::protocol::FrameReader empty;
    const auto counts = empty.counts();
    require(counts.accepted == 0 && counts.rejected == 0 && counts.incomplete == 0,
            "empty input should have zero counts");
}
