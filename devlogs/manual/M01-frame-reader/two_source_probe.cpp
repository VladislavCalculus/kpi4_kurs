#include "frame_log.hpp"
#include <kpi4/protocol/frame_reader.hpp>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>

namespace {

using Channel = kpi4::protocol::InputChannel;

const char* channel_name(Channel channel) {
    return channel == Channel::GroundStation ? "gs" : "controller";
}

const char* rejection_name(kpi4::protocol::FrameRejectReason reason) {
    switch (reason) {
        case kpi4::protocol::FrameRejectReason::BadCrc:
            return "bad_crc";
        case kpi4::protocol::FrameRejectReason::BadSignature:
            return "bad_signature";
        case kpi4::protocol::FrameRejectReason::InvalidHeader:
            return "invalid_header";
    }
    return "unknown";
}

bool read_one(std::ifstream& input, Channel channel,
              kpi4::protocol::FrameReader& reader, std::size_t& bytes) {
    const auto next = input.get();
    if (next == std::char_traits<char>::eof()) {
        return false;
    }
    ++bytes;
    const auto event = reader.consume(channel, static_cast<std::uint8_t>(next));
    if (event.frame) {
        std::cout << "source=" << channel_name(event.channel) << ' ';
        kpi4::host::write_frame_metadata(
            std::cout, *event.frame, std::chrono::system_clock::now());
    }
    if (event.rejection) {
        std::cout << "source=" << channel_name(event.channel)
                  << " rejected=" << rejection_name(*event.rejection) << '\n';
    }
    return true;
}

void print_counts(Channel channel, std::size_t bytes,
                  const kpi4::protocol::FrameReader& reader) {
    const auto counts = reader.counts(channel);
    std::cout << "source=" << channel_name(channel)
              << " bytes=" << bytes
              << " accepted=" << counts.accepted
              << " rejected=" << counts.rejected
              << " incomplete=" << counts.incomplete << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: m01_two_source_probe GS_FILE CONTROLLER_FILE\n";
        return 2;
    }

    std::ifstream gs(argv[1], std::ios::binary);
    std::ifstream controller(argv[2], std::ios::binary);
    if (!gs || !controller) {
        std::cerr << "Cannot open both input files\n";
        return 1;
    }

    kpi4::protocol::FrameReader reader;
    std::size_t gs_bytes = 0;
    std::size_t controller_bytes = 0;
    bool gs_done = false;
    bool controller_done = false;
    while (!gs_done || !controller_done) {
        if (!gs_done) {
            gs_done = !read_one(gs, Channel::GroundStation, reader, gs_bytes);
        }
        if (!controller_done) {
            controller_done = !read_one(
                controller, Channel::ControllerX, reader, controller_bytes);
        }
    }
    if (gs.bad() || controller.bad()) {
        std::cerr << "Input read failed\n";
        return 1;
    }
    print_counts(Channel::GroundStation, gs_bytes, reader);
    print_counts(Channel::ControllerX, controller_bytes, reader);
}
