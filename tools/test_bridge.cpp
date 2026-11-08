#include "frame_log.hpp"
#include <kpi4/protocol/frame_reader.hpp>
#include <kpi4/protocol/heartbeat.hpp>
#include <kpi4/protocol/mission_command_validator.hpp>
#include <kpi4/protocol/mission_normalizer.hpp>
#include <kpi4/protocol/mission_transfer.hpp>
#include <kpi4/protocol/mission_upload.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace kpi4;
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

void emit_upload(const protocol::MissionUploadTransition& transition) {
    if (const auto* value = std::get_if<protocol::MissionUploadRequest>(&transition)) {
        std::cout << "UPREQ\t" << value->target.system_id << '\t'
                  << static_cast<unsigned>(value->target.component_id) << '\t'
                  << static_cast<unsigned>(value->mission_type) << '\t' << value->seq << '\n';
    } else if (const auto* value = std::get_if<protocol::MissionUploadCompleted>(&transition)) {
        std::cout << "UPCOMPLETE\t" << value->candidate.source.system_id << '\t'
                  << static_cast<unsigned>(value->candidate.source.component_id) << '\t'
                  << static_cast<unsigned>(value->candidate.mission_type) << '\t'
                  << value->candidate.items.size() << '\n';
        for (std::size_t i = 0; i < value->candidate.items.size(); ++i) {
            const auto& item = value->candidate.items[i];
            const auto& raw = value->candidate.raw_items[i];
            std::cout << "UPITEM\t" << item.sequence << '\t' << item.target.latitude.value
                      << '\t' << item.target.longitude.value << '\t'
                      << static_cast<int>(item.altitude_reference) << '\t';
            if (item.altitude) std::cout << item.altitude->value;
            else std::cout << '-';
            std::cout << '\t' << raw.command << '\t' << raw.param1 << '\n';
        }
    } else if (const auto* value = std::get_if<protocol::MissionUploadRejection>(&transition)) {
        std::cout << "UPREJECT\t" << static_cast<int>(value->error) << '\t';
        if (value->ack) std::cout << static_cast<unsigned>(value->ack->result);
        else std::cout << '-';
        std::cout << '\n';
    }
}

void emit_mission(const protocol::MissionTransferEvent& event) {
    const auto common = [&](const protocol::MissionAddress& address) {
        std::cout << '\t' << event.sender_sysid << '\t'
                  << static_cast<unsigned>(event.sender_compid) << '\t'
                  << address.target_system << '\t'
                  << static_cast<unsigned>(address.target_component) << '\t'
                  << static_cast<unsigned>(address.mission_type);
    };
    if (const auto* value = std::get_if<protocol::MissionCount>(&event.payload)) {
        std::cout << "MCOUNT"; common(value->address);
        std::cout << '\t' << value->count << '\t' << value->opaque_id << '\n';
    } else if (const auto* value = std::get_if<protocol::MissionRequestInt>(&event.payload)) {
        std::cout << "MREQUEST"; common(value->address);
        std::cout << '\t' << value->seq << '\n';
    } else if (const auto* value = std::get_if<protocol::MissionItemInt>(&event.payload)) {
        std::cout << "MITEM"; common(value->address);
        std::cout << '\t' << value->seq << '\t' << static_cast<unsigned>(value->frame)
                  << '\t' << value->command << '\t' << static_cast<unsigned>(value->current)
                  << '\t' << static_cast<unsigned>(value->autocontinue)
                  << '\t' << value->param1 << '\t' << value->param2
                  << '\t' << value->param3 << '\t' << value->param4
                  << '\t' << value->x << '\t' << value->y << '\t' << value->z << '\n';
    } else if (const auto* value = std::get_if<protocol::MissionAck>(&event.payload)) {
        std::cout << "MACK"; common(value->address);
        std::cout << '\t' << static_cast<unsigned>(value->result) << '\t'
                  << value->opaque_id << '\n';
    }
}
}

int main() {
    kpi4::protocol::FrameReader reader;
    std::optional<mavlink_message_t> last;
    std::unique_ptr<protocol::MissionUploadAssembler> upload;
    std::cout << std::setprecision(17);
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
                        if (const auto mission = protocol::decode_mission_transfer(result)) {
                            emit_mission(*mission);
                            if (upload) emit_upload(upload->ingest(*mission));
                        }
                    }
                }
                const auto counts = reader.counts(source);
                std::cout << "COUNT\t" << channel << '\t' << counts.accepted << '\t'
                          << counts.rejected << '\t' << counts.incomplete << '\n';
            } else if (command == "upload") {
                std::uint32_t system;
                unsigned component, maximum;
                input >> system >> component >> maximum;
                upload = std::make_unique<protocol::MissionUploadAssembler>(
                    system, static_cast<std::uint8_t>(component),
                    static_cast<std::uint16_t>(maximum));
            } else if (command == "wide_count") {
                std::uint32_t target;
                input >> target;
                mavlink_message_t message{};
                mavlink_msg_mission_count_pack(42, 190, &message, target, 1, 513, 2,
                                               0xFEDCBA98);
                std::cout << "WIRE\t" << to_hex(message) << '\n';
            } else if (command == "ack") {
                unsigned result;
                input >> result;
                const auto value = upload ? upload->finish_after_plan_decision(
                    static_cast<std::uint8_t>(result)) : std::nullopt;
                if (value) std::cout << "UPACK\t" << value->target.system_id << '\t'
                                     << static_cast<unsigned>(value->target.component_id) << '\t'
                                     << static_cast<unsigned>(value->mission_type) << '\t'
                                     << static_cast<unsigned>(value->result) << '\n';
                else std::cout << "UPACKNONE\n";
            } else if (command == "normalize") {
                unsigned frame, id, sequence;
                std::int32_t x, y;
                std::string z;
                input >> frame >> id >> sequence >> x >> y >> z;
                protocol::MissionItemInt item{};
                item.frame = static_cast<std::uint8_t>(frame);
                item.command = static_cast<std::uint16_t>(id);
                item.seq = static_cast<std::uint16_t>(sequence);
                item.x = x; item.y = y; item.z = std::stof(z);
                const auto value = protocol::normalize_mission_item(item);
                if (const auto* accepted = std::get_if<core::MissionItem>(&value)) {
                    std::cout << "NORMAL\taccepted\t" << accepted->sequence << '\t'
                              << accepted->target.latitude.value << '\t'
                              << accepted->target.longitude.value << '\t'
                              << static_cast<int>(accepted->altitude_reference) << '\t';
                    if (accepted->altitude) std::cout << accepted->altitude->value;
                    else std::cout << '-';
                    std::cout << '\n';
                } else {
                    std::cout << "NORMAL\trejected\t"
                              << static_cast<int>(std::get<protocol::MissionNormalizationError>(value)) << '\n';
                }
            } else if (command == "command") {
                unsigned id;
                input >> id;
                protocol::MissionItemInt item{};
                item.command = static_cast<std::uint16_t>(id);
                const auto value = protocol::validate_mission_item_command(item);
                std::cout << "COMMAND\t" << id << '\t'
                          << (std::holds_alternative<protocol::SupportedMissionCommand>(value)
                                  ? "accepted" : "rejected") << '\n';
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
