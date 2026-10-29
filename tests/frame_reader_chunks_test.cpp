#include "byte_source.hpp"
#include <kpi4/protocol/frame_reader.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<std::uint8_t>;

struct Outcome {
    std::vector<Bytes> frames;
    kpi4::protocol::FrameCounts counts;
};

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Bytes load_fixture(const char* name) {
    std::ifstream stream(std::string{KPI4_FIXTURE_DIR} + '/' + name,
                         std::ios::binary);
    require(static_cast<bool>(stream), std::string{"fixture missing: "} + name);
    return {std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

void feed(kpi4::protocol::FrameReader& reader, const std::uint8_t* data,
          std::size_t size, Outcome& outcome) {
    for (std::size_t i = 0; i < size; ++i) {
        const auto frame = reader.consume(data[i]);
        if (frame) {
            std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> serialized{};
            const auto length = mavlink_msg_to_send_buffer(serialized.data(), &*frame);
            require(length <= serialized.size(), "serialized frame exceeds buffer");
            outcome.frames.emplace_back(serialized.begin(), serialized.begin() + length);
        }
    }
    outcome.counts = reader.counts();
}

Outcome run_chunks(const std::vector<Bytes>& chunks) {
    kpi4::protocol::FrameReader reader;
    Outcome outcome;
    for (const auto& chunk : chunks) {
        feed(reader, chunk.data(), chunk.size(), outcome);
    }
    return outcome;
}

Outcome run_stream(const Bytes& bytes, std::size_t capacity) {
    const std::string input(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    std::istringstream stream(input, std::ios::in | std::ios::binary);
    kpi4::host::StreamByteSource source(stream);
    kpi4::protocol::FrameReader reader;
    Bytes buffer(capacity);
    Outcome outcome;
    std::size_t total = 0;
    while (const auto size = source.read(buffer.data(), buffer.size())) {
        total += size;
        feed(reader, buffer.data(), size, outcome);
    }
    require(!source.failed() && total == bytes.size(), "stream input was not fully read");
    return outcome;
}

void require_same(const Outcome& actual, const Outcome& expected,
                  const std::string& case_name) {
    require(actual.frames == expected.frames, case_name + ": emitted frames differ");
    require(actual.counts.accepted == expected.counts.accepted,
            case_name + ": accepted count differs");
    require(actual.counts.rejected == expected.counts.rejected,
            case_name + ": rejected count differs");
    require(actual.counts.incomplete == expected.counts.incomplete,
            case_name + ": incomplete count differs");
}

}  // namespace

int main() {
    const auto v1 = load_fixture("heartbeat_v1.bin");
    const auto v2 = load_fixture("heartbeat_v2.bin");
    const auto baseline = run_chunks({v2});
    require(baseline.frames == std::vector<Bytes>{v2}, "whole v2 frame differs");
    require(baseline.counts.accepted == 1 && baseline.counts.rejected == 0 &&
                baseline.counts.incomplete == 0,
            "whole v2 frame counts differ");

    for (std::size_t split = 1; split < v2.size(); ++split) {
        kpi4::protocol::FrameReader reader;
        Outcome outcome;
        feed(reader, v2.data(), split, outcome);
        require(outcome.frames.empty() && outcome.counts.incomplete == 1,
                "premature frame at split " + std::to_string(split));
        feed(reader, v2.data() + split, v2.size() - split, outcome);
        require_same(outcome, baseline, "split " + std::to_string(split));
    }

    const auto split_fixture = run_chunks({load_fixture("heartbeat_v2_chunks/01.bin"),
                                           load_fixture("heartbeat_v2_chunks/02.bin"),
                                           load_fixture("heartbeat_v2_chunks/03.bin")});
    require_same(split_fixture, baseline, "three saved chunks");

    Bytes combined = v1;
    combined.insert(combined.end(), v2.begin(), v2.end());
    combined.insert(combined.end(), v1.begin(), v1.end());
    const auto multiple = run_chunks({combined});
    require(multiple.frames == std::vector<Bytes>{v1, v2, v1},
            "multiple frames in one chunk differ");
    require(multiple.counts.accepted == 3 && multiple.counts.rejected == 0 &&
                multiple.counts.incomplete == 0,
            "multiple frame counts differ");

    for (std::size_t capacity = 1; capacity <= v2.size(); ++capacity) {
        require_same(run_stream(v2, capacity), baseline,
                     "stream capacity " + std::to_string(capacity));
    }
    require_same(run_stream(combined, combined.size()), multiple,
                 "multiple frames from one stream read");
}
