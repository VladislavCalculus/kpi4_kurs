#include <kpi4/protocol/mission_command_validator.hpp>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

kpi4::protocol::MissionItemInt item(std::uint16_t command) {
    return {{1, 1, MAV_MISSION_TYPE_MISSION}, 0,
            MAV_FRAME_GLOBAL_INT, command, 0, 1,
            0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f};
}

}  // namespace

int main() {
    using namespace kpi4::protocol;

    const auto waypoint = validate_mission_item_command(
        item(MAV_CMD_NAV_WAYPOINT));
    const auto* supported = std::get_if<SupportedMissionCommand>(&waypoint);
    require(supported && *supported == SupportedMissionCommand::waypoint,
            "NAV_WAYPOINT was not accepted");

    const std::array<std::uint16_t, 8> unsupported_commands{
        MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND, MAV_CMD_NAV_RETURN_TO_LAUNCH,
        MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_LOITER_TURNS,
        MAV_CMD_NAV_LOITER_TIME, MAV_CMD_MISSION_START, 65535};
    for (const auto command : unsupported_commands) {
        const auto result = validate_mission_item_command(item(command));
        const auto* rejection = std::get_if<MissionCommandRejection>(&result);
        require(rejection != nullptr, "unsupported command was accepted");
        require(rejection->command == command,
                "unsupported command ID was not preserved");
        require(rejection->reason ==
                    MissionCommandRejectReason::unsupported_in_demo_subset,
                "unsupported command reason is not explicit");
    }
}
