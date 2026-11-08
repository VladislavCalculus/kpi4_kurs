#include "frame_log.hpp"
#include <kpi4/protocol/frame_reader.hpp>
#include <kpi4/protocol/heartbeat.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::vector<std::uint8_t> from_hex(const std::string& hex) {
    if (hex.size() % 2) throw std::runtime_error("odd hex length");
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < hex.size(); i += 2)
        bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    return bytes;
}

std::string to_hex(const mavlink_message_t& frame) {
    std::vector<std::uint8_t> bytes(MAVLINK_MAX_PACKET_LEN);
    const auto count = mavlink_msg_to_send_buffer(bytes.data(), &frame);
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < count; ++i) out << std::setw(2) << static_cast<unsigned>(bytes[i]);
    return out.str();
}
}

int main() {
    kpi4::protocol::FrameReader reader;
    std::optional<mavlink_message_t> last;
    std::string line;
    try {
        while (std::getline(std::cin, line)) {
            std::istringstream input(line);
            std::string command;
            input >> command;
            if (command == "feed") {
                unsigned channel;
                std::string hex;
                input >> channel >> hex;
                const auto source = static_cast<kpi4::protocol::InputChannel>(channel);
                for (const auto byte : from_hex(hex)) {
                    const auto result = reader.consume(source, byte);
                    if (result.rejection)
                        std::cout << "REJECT\t" << channel << '\t' << static_cast<int>(*result.rejection) << '\n';
                    if (result.frame) {
                        last = *result.frame;
                        const auto& f = *result.frame;
                        std::cout << "FRAME\t" << channel << '\t' << static_cast<unsigned>(f.magic)
                                  << '\t' << f.sysid << '\t' << static_cast<unsigned>(f.compid)
                                  << '\t' << f.msgid << '\t' << static_cast<unsigned>(f.seq)
                                  << '\t' << static_cast<unsigned>(f.len) << '\t' << to_hex(f) << '\n';
                        if (const auto heartbeat = kpi4::protocol::decode_heartbeat(result))
                            std::cout << "HEARTBEAT\t" << static_cast<unsigned>(heartbeat->type)
                                      << '\t' << static_cast<unsigned>(heartbeat->autopilot)
                                      << '\t' << static_cast<unsigned>(heartbeat->base_mode)
                                      << '\t' << heartbeat->custom_mode
                                      << '\t' << static_cast<unsigned>(heartbeat->system_status)
                                      << '\t' << static_cast<unsigned>(heartbeat->mavlink_version)
                                      << '\t' << kpi4::protocol::heartbeat_type_name(heartbeat->type)
                                      << '\t' << kpi4::protocol::heartbeat_autopilot_name(heartbeat->autopilot)
                                      << '\t' << kpi4::protocol::heartbeat_state_name(heartbeat->system_status) << '\n';
                    }
                }
                const auto counts = reader.counts(source);
                std::cout << "COUNT\t" << channel << '\t' << counts.accepted << '\t'
                          << counts.rejected << '\t' << counts.incomplete << '\n';
            } else if (command == "format") {
                std::int64_t milliseconds;
                input >> milliseconds;
                if (!last) {
                    std::cout << "FORMATNONE\n";
                } else {
                    std::ostringstream out;
                    kpi4::host::write_frame_metadata(out, *last,
                        std::chrono::system_clock::time_point{std::chrono::milliseconds{milliseconds}});
                    auto value = out.str();
                    if (!value.empty() && value.back() == '\n') value.pop_back();
                    std::cout << "FORMAT\t" << value << '\n';
                }
            } else if (!command.empty()) {
                throw std::runtime_error("unknown command: " + command);
            }
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
