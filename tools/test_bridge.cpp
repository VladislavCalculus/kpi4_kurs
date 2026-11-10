#include "frame_log.hpp"
#include <kpi4/protocol/frame_reader.hpp>
#include <kpi4/protocol/heartbeat.hpp>
#include <kpi4/protocol/estimated_position.hpp>
#include <kpi4/protocol/raw_gps.hpp>
#include <kpi4/protocol/mission_command_validator.hpp>
#include <kpi4/protocol/mission_normalizer.hpp>
#include <kpi4/protocol/mission_transfer.hpp>
#include <kpi4/protocol/mission_upload.hpp>
#include <kpi4/mission/plan_store.hpp>
#include <kpi4/state/mavlink_provider.hpp>
#include <kpi4/state/simulated_provider.hpp>
#include <kpi4/state/validator.hpp>
#include <kpi4/config/host_config.hpp>
#include <chrono>
#include <cstdint>
#include <fstream>
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
    } else if (const auto* value = std::get_if<protocol::MissionUploadAck>(&transition)) {
        std::cout << "UPACK\t" << value->target.system_id << '\t'
                  << static_cast<unsigned>(value->target.component_id) << '\t'
                  << static_cast<unsigned>(value->mission_type) << '\t'
                  << static_cast<unsigned>(value->result) << '\n';
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

template <typename T> void optional_field(const std::optional<T>& value) {
    if (value) std::cout << '\t' << *value;
    else std::cout << "\t-";
}

void emit_state(const core::StateSample& sample) {
    std::cout << "STATE\t" << static_cast<int>(sample.source) << '\t'
              << sample.measured_at.value << '\t' << sample.received_at.value
              << '\t' << sample.position.latitude.value << '\t'
              << sample.position.longitude.value;
    optional_field(sample.altitude ? std::optional<double>{sample.altitude->value.value} : std::nullopt);
    optional_field(sample.altitude ? std::optional<int>{static_cast<int>(sample.altitude->reference)} : std::nullopt);
    optional_field(sample.velocity ? std::optional<double>{sample.velocity->north.value} : std::nullopt);
    optional_field(sample.velocity ? std::optional<double>{sample.velocity->east.value} : std::nullopt);
    optional_field(sample.heading_true_north ? std::optional<double>{sample.heading_true_north->value} : std::nullopt);
    optional_field(sample.horizontal_accuracy ? std::optional<double>{sample.horizontal_accuracy->value} : std::nullopt);
    optional_field(sample.gps_fix ? std::optional<int>{static_cast<int>(*sample.gps_fix)} : std::nullopt);
    std::cout << '\n';
}

void emit_estimated(const protocol::EstimatedPosition& value) {
    std::cout << "EST\t" << value.time_boot_ms << '\t'
              << value.position.latitude.value << '\t' << value.position.longitude.value
              << '\t' << value.altitude_msl.value.value << '\t'
              << value.altitude_relative_to_home.value.value << '\t'
              << value.horizontal_velocity.north.value << '\t'
              << value.horizontal_velocity.east.value << '\t'
              << value.velocity_down.value;
    optional_field(value.heading ? std::optional<double>{value.heading->value} : std::nullopt);
    std::cout << '\n';
}

void emit_gps(const protocol::RawGps& value) {
    std::cout << "GPS\t" << value.time_usec << '\t'
              << static_cast<unsigned>(value.fix_type) << '\t'
              << value.position.latitude.value << '\t'
              << value.position.longitude.value << '\t' << value.altitude_msl.value.value;
    optional_field(value.horizontal_dop);
    optional_field(value.vertical_dop);
    optional_field(value.ground_speed ? std::optional<double>{value.ground_speed->value} : std::nullopt);
    optional_field(value.course_over_ground ? std::optional<double>{value.course_over_ground->value} : std::nullopt);
    optional_field(value.satellites_visible ? std::optional<int>{*value.satellites_visible} : std::nullopt);
    optional_field(value.altitude_ellipsoid ? std::optional<double>{value.altitude_ellipsoid->value} : std::nullopt);
    optional_field(value.horizontal_uncertainty ? std::optional<double>{value.horizontal_uncertainty->value} : std::nullopt);
    optional_field(value.vertical_uncertainty ? std::optional<double>{value.vertical_uncertainty->value} : std::nullopt);
    optional_field(value.speed_uncertainty ? std::optional<double>{value.speed_uncertainty->value} : std::nullopt);
    optional_field(value.track_uncertainty ? std::optional<double>{value.track_uncertainty->value} : std::nullopt);
    optional_field(value.gps_yaw ? std::optional<double>{value.gps_yaw->value} : std::nullopt);
    std::cout << '\n';
}
}

int main() {
    kpi4::protocol::FrameReader reader;
    std::optional<mavlink_message_t> last;
    std::unique_ptr<protocol::MissionUploadAssembler> upload;
    mission::MissionPlanStore plan_store;
    config::HostConfigStore config_store;
    std::shared_ptr<const mission::MissionPlan> saved_plan;
    std::optional<protocol::MissionUploadCandidate> staged_plan;
    std::unique_ptr<state::MavlinkStateProvider> state_provider;
    std::unique_ptr<state::StateValidator> validator;
    state::SimulatedStateProvider simulated;
    state::SampleTimes times{{0}, {0}};
    core::MonotonicMicroseconds now{0};
    const auto emit_validity = [&](const core::StateSample& sample) {
        if (!validator) return;
        const auto result = validator->validate(sample, now);
        if (const auto* reason = std::get_if<state::StateRejectReason>(&result))
            std::cout << "VALID\trejected\t" << static_cast<int>(*reason) << '\n';
        else std::cout << "VALID\taccepted\n";
    };
    const auto emit_latest = [&]() {
        if (!state_provider) return;
        const auto sample = state_provider->take_latest();
        if (sample) { emit_state(*sample); emit_validity(*sample); }
    };
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
                            if (upload) {
                                auto transition = upload->ingest(*mission, now);
                                if (const auto* completed =
                                        std::get_if<protocol::MissionUploadCompleted>(&transition))
                                    staged_plan = completed->candidate;
                                emit_upload(transition);
                            }
                        }
                        if (const auto estimated = protocol::decode_global_position_int(result)) {
                            if (const auto* value = std::get_if<protocol::EstimatedPosition>(&*estimated)) {
                                emit_estimated(*value);
                                if (state_provider) { state_provider->accept(*value, times); emit_latest(); }
                            } else std::cout << "ESTERR\t"
                                             << static_cast<int>(std::get<protocol::EstimatedPositionError>(*estimated)) << '\n';
                        }
                        if (const auto gps = protocol::decode_gps_raw_int(result)) {
                            if (const auto* value = std::get_if<protocol::RawGps>(&*gps)) {
                                emit_gps(*value);
                                if (state_provider) { state_provider->accept(*value, times); emit_latest(); }
                            } else std::cout << "GPSERR\t"
                                             << static_cast<int>(std::get<protocol::RawGpsError>(*gps)) << '\n';
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
                std::uint64_t timeout = 250'000;
                unsigned retries = 5;
                if (input >> timeout) {
                    if (!(input >> retries))
                        throw std::runtime_error("upload policy requires timeout and retries");
                }
                upload = std::make_unique<protocol::MissionUploadAssembler>(
                    system, static_cast<std::uint8_t>(component),
                    static_cast<std::uint16_t>(maximum),
                    protocol::MissionUploadPolicy{timeout, static_cast<std::uint8_t>(retries)});
            } else if (command == "tick") {
                input >> now.value;
                if (upload) {
                    auto transition = upload->poll(now);
                    if (std::holds_alternative<protocol::MissionUploadRejection>(transition))
                        staged_plan.reset();
                    emit_upload(transition);
                }
            } else if (command == "active") {
                const auto plan = plan_store.active();
                if (plan)
                    std::cout << "ACTIVE\t" << plan->owner.system_id << '\t'
                              << plan->items.size() << '\t'
                              << plan->items.front().target.latitude.value << '\n';
                else std::cout << "ACTIVE\t-\n";
            } else if (command == "plan" || command == "saved") {
                const auto plan = command == "plan" ? plan_store.active() : saved_plan;
                if (plan)
                    std::cout << "PLAN\t" << plan->version << '\t'
                              << static_cast<int>(plan->owner.channel) << '\t'
                              << plan->owner.system_id << '\t'
                              << static_cast<unsigned>(plan->owner.component_id) << '\t'
                              << plan->items.size() << '\t'
                              << plan->items.front().target.latitude.value << '\n';
                else std::cout << "PLAN\t-\n";
            } else if (command == "save") {
                saved_plan = plan_store.active();
            } else if (command == "plan_raw") {
                const auto plan = plan_store.active();
                if (plan)
                    std::cout << "PLANRAW\t" << plan->version << '\t'
                              << plan->raw_items.size() << '\t'
                              << plan->raw_items.front().seq << '\t'
                              << plan->raw_items.front().command << '\t'
                              << plan->raw_items.front().x << '\t'
                              << plan->raw_items.front().param1 << '\n';
                else std::cout << "PLANRAW\t-\n";
            } else if (command == "corrupt_staged") {
                if (!staged_plan) throw std::runtime_error("no staged plan");
                staged_plan->raw_items.clear();
            } else if (command == "start") {
                std::cout << "START\t" << static_cast<int>(plan_store.start()) << '\n';
            } else if (command == "discard") {
                std::cout << "DISCARD\t" << plan_store.discard_preflight() << '\n';
            } else if (command == "wide_count") {
                std::uint32_t target;
                input >> target;
                mavlink_message_t message{};
                mavlink_msg_mission_count_pack(42, 190, &message, target, 1, 513, 2,
                                               0xFEDCBA98);
                std::cout << "WIRE\t" << to_hex(message) << '\n';
            } else if (command == "state") {
                int source;
                input >> source;
                state_provider = std::make_unique<state::MavlinkStateProvider>(
                    static_cast<core::StateSource>(source));
            } else if (command == "config_load") {
                std::string path;
                input >> path;
                std::ifstream file(path);
                const auto result = config_store.load(file, path);
                if (const auto* snapshot =
                        std::get_if<std::shared_ptr<const config::ConfigSnapshot>>(&result))
                    std::cout << "CONFIG\taccepted\t" << (*snapshot)->version << '\n';
                else {
                    const auto& failure = std::get<config::ConfigFailure>(result);
                    std::cout << "CONFIG\trejected\t" << static_cast<int>(failure.error)
                              << '\t' << failure.field << '\n';
                }
            } else if (command == "config_current") {
                const auto snapshot = config_store.current();
                if (!snapshot) std::cout << "CONFIGCURRENT\t-\n";
                else {
                    std::cout << "CONFIGCURRENT\t" << snapshot->version << '\t'
                              << static_cast<int>(snapshot->source) << '\t'
                              << static_cast<int>(snapshot->reading_type) << '\t'
                              << snapshot->state_limits.max_measurement_age_us << '\t'
                              << snapshot->state_limits.max_delivery_delay_us;
                    optional_field(snapshot->state_limits.max_horizontal_accuracy ?
                        std::optional<double>{snapshot->state_limits.max_horizontal_accuracy->value} :
                        std::nullopt);
                    optional_field(snapshot->state_limits.minimum_gps_fix ?
                        std::optional<int>{static_cast<int>(*snapshot->state_limits.minimum_gps_fix)} :
                        std::nullopt);
                    std::cout << '\t' << snapshot->origin << '\n';
                }
            } else if (command == "config_apply") {
                const auto snapshot = config_store.current();
                if (!snapshot) std::cout << "CONFIGAPPLY\trejected\n";
                else {
                    validator = std::make_unique<state::StateValidator>(snapshot->state_limits);
                    std::cout << "CONFIGAPPLY\taccepted\n";
                }
            } else if (command == "clock") {
                input >> times.measured_at.value >> times.received_at.value >> now.value;
            } else if (command == "validator") {
                std::uint64_t age, delay;
                std::string accuracy, fix;
                input >> age >> delay >> accuracy >> fix;
                state::StateValidationConfig config{
                    age, delay,
                    accuracy == "-" ? std::nullopt :
                        std::optional<core::Meters>{core::Meters{std::stod(accuracy)}},
                    fix == "-" ? std::nullopt :
                        std::optional<core::GpsFix>{static_cast<core::GpsFix>(std::stoi(fix))}};
                validator = std::make_unique<state::StateValidator>(config);
            } else if (command == "sim" || command == "direct") {
                int source;
                std::uint64_t measured, received;
                std::string lat, lon, altitude, reference, north, east, heading, accuracy, fix;
                input >> source >> measured >> received >> lat >> lon >> altitude
                      >> reference >> north >> east >> heading >> accuracy >> fix;
                const auto number = [](const std::string& text) -> std::optional<double> {
                    return text == "-" ? std::nullopt : std::optional<double>{std::stod(text)};
                };
                core::StateSample sample{{measured}, {received},
                    static_cast<core::StateSource>(source),
                    {{std::stod(lat)}, {std::stod(lon)}},
                    std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
                if (const auto value = number(altitude))
                    sample.altitude = core::Altitude{{*value},
                        static_cast<core::AltitudeReference>(std::stoi(reference))};
                if (const auto value = number(north)) {
                    const auto east_value = number(east);
                    if (!east_value) throw std::runtime_error("missing east velocity");
                    sample.velocity = core::NorthEastVelocity{{*value}, {*east_value}};
                }
                if (const auto value = number(heading)) sample.heading_true_north = core::Degrees{*value};
                if (const auto value = number(accuracy)) sample.horizontal_accuracy = core::Meters{*value};
                if (fix != "-") sample.gps_fix = static_cast<core::GpsFix>(std::stoi(fix));
                if (command == "direct") { emit_state(sample); emit_validity(sample); }
                else {
                    simulated.publish(sample);
                    const auto value = simulated.take_latest();
                    if (value) { emit_state(*value); emit_validity(*value); }
                }
            } else if (command == "ack") {
                unsigned result;
                input >> result;
                if (result == MAV_MISSION_ACCEPTED && staged_plan) {
                    const auto stored = plan_store.accept(std::move(*staged_plan));
                    if (const auto* error = std::get_if<mission::PlanStoreError>(&stored)) {
                        std::cout << "STORE\trejected\t" << static_cast<int>(*error) << '\n';
                        result = *error == mission::PlanStoreError::InvalidCandidate
                                     ? MAV_MISSION_INVALID : MAV_MISSION_DENIED;
                    } else {
                        std::cout << "STORE\taccepted\t"
                                  << std::get<std::shared_ptr<const mission::MissionPlan>>(stored)->version
                                  << '\n';
                    }
                }
                const auto value = upload ? upload->finish_after_plan_decision(
                    static_cast<std::uint8_t>(result)) : std::nullopt;
                staged_plan.reset();
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
            } else if (command == "format_raw") {
                std::int64_t milliseconds;
                unsigned magic, system, component, message, sequence, length, signed_flag;
                input >> milliseconds >> magic >> system >> component >> message
                      >> sequence >> length >> signed_flag;
                mavlink_message_t frame{};
                frame.magic = static_cast<std::uint8_t>(magic);
                frame.sysid = system;
                frame.compid = static_cast<std::uint8_t>(component);
                frame.msgid = message;
                frame.seq = static_cast<std::uint8_t>(sequence);
                frame.len = static_cast<std::uint8_t>(length);
                frame.incompat_flags = signed_flag ? MAVLINK_IFLAG_SIGNED : 0;
                std::ostringstream out;
                host::write_frame_metadata(out, frame,
                    std::chrono::system_clock::time_point{std::chrono::milliseconds{milliseconds}});
                auto value = out.str();
                if (!value.empty() && value.back() == '\n') value.pop_back();
                std::cout << "FORMAT\t" << value << '\n';
            } else if (command == "probe") {
                std::string kind;
                unsigned magic, length, message;
                input >> kind >> magic >> length >> message;
                if (!last) {
                    std::cout << "PROBE\tignored\n";
                } else {
                    auto modified = *last;
                    modified.magic = static_cast<std::uint8_t>(magic);
                    modified.len = static_cast<std::uint8_t>(length);
                    modified.msgid = message;
                    protocol::FrameReadResult event{protocol::InputChannel::Unspecified,
                                                     modified, std::nullopt};
                    if (kind == "heartbeat") {
                        std::cout << "PROBE\t"
                                  << (protocol::decode_heartbeat(event) ? "accepted" : "ignored") << '\n';
                    } else if (kind == "mission") {
                        std::cout << "PROBE\t"
                                  << (protocol::decode_mission_transfer(event) ? "accepted" : "ignored") << '\n';
                    } else if (kind == "estimated") {
                        const auto value = protocol::decode_global_position_int(event);
                        if (!value) std::cout << "PROBE\tignored\n";
                        else if (const auto* error = std::get_if<protocol::EstimatedPositionError>(&*value))
                            std::cout << "PROBE\trejected\t" << static_cast<int>(*error) << '\n';
                        else std::cout << "PROBE\taccepted\n";
                    } else if (kind == "gps") {
                        const auto value = protocol::decode_gps_raw_int(event);
                        if (!value) std::cout << "PROBE\tignored\n";
                        else if (const auto* error = std::get_if<protocol::RawGpsError>(&*value))
                            std::cout << "PROBE\trejected\t" << static_cast<int>(*error) << '\n';
                        else std::cout << "PROBE\taccepted\n";
                    } else throw std::runtime_error("unknown probe kind");
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
