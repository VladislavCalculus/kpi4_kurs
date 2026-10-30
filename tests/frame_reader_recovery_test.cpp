#include <kpi4/protocol/frame_reader.hpp>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<std::uint8_t>;
using Reason = kpi4::protocol::FrameRejectReason;

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

void check_recovery_stream() {
    kpi4::protocol::FrameReader reader;
    std::vector<Reason> rejections;
    std::size_t accepted_events = 0;
    for (const auto byte : load_fixture("recovery_stream.bin")) {
        const auto event = reader.consume(byte);
        require(!(event.frame && event.rejection), "one byte produced conflicting events");
        if (event.frame) {
            ++accepted_events;
            require(event.frame->msgid == MAVLINK_MSG_ID_HEARTBEAT,
                    "unexpected recovered message");
        }
        if (event.rejection) {
            rejections.push_back(*event.rejection);
        }
    }
    const auto counts = reader.counts();
    require(accepted_events == 2 && counts.accepted == 2,
            "valid frames after damage were lost");
    require(rejections == std::vector<Reason>{Reason::BadCrc, Reason::BadCrc},
            "rejection reasons or order differ");
    require(counts.rejected == 2 && counts.incomplete == 0,
            "recovery counts differ");
}

void check_noise_before_valid_frame() {
    const auto valid = load_fixture("heartbeat_v2.bin");
    Bytes input{0x00, 0x7F, 0x12};
    input.insert(input.end(), valid.begin(), valid.end());

    kpi4::protocol::FrameReader reader;
    std::size_t accepted_events = 0;
    for (const auto byte : input) {
        const auto event = reader.consume(byte);
        require(!event.rejection, "noise caused a rejected frame");
        accepted_events += event.frame.has_value() ? 1u : 0u;
    }
    const auto counts = reader.counts();
    require(accepted_events == 1 && counts.accepted == 1 &&
                counts.rejected == 0 && counts.incomplete == 0,
            "valid frame after noise was lost");
}

void check_invalid_header_recovery() {
    const auto valid = load_fixture("heartbeat_v2.bin");
    Bytes input{MAVLINK_STX, 0x00};
    input.insert(input.end(), valid.begin(), valid.end());

    kpi4::protocol::FrameReader reader;
    std::vector<Reason> rejections;
    std::size_t accepted_events = 0;
    for (const auto byte : input) {
        const auto event = reader.consume(byte);
        if (event.frame) {
            ++accepted_events;
        }
        if (event.rejection) {
            rejections.push_back(*event.rejection);
        }
    }
    const auto counts = reader.counts();
    require(rejections == std::vector<Reason>{Reason::InvalidHeader},
            "invalid header reason missing");
    require(accepted_events == 1 && counts.accepted == 1 &&
                counts.rejected == 1 && counts.incomplete == 0,
            "frame after invalid header was lost");
}

}  // namespace

int main() {
    check_noise_before_valid_frame();
    check_recovery_stream();
    check_invalid_header_recovery();
}
