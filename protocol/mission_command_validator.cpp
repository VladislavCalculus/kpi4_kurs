#include <kpi4/protocol/mission_command_validator.hpp>

namespace kpi4::protocol {

MissionCommandValidationResult validate_mission_item_command(
    const MissionItemInt& item) {
    if (item.command == MAV_CMD_NAV_WAYPOINT) {
        return SupportedMissionCommand::waypoint;
    }
    return MissionCommandRejection{
        item.command, MissionCommandRejectReason::unsupported_in_demo_subset};
}

}  // namespace kpi4::protocol
