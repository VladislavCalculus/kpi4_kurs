#include <kpi4/protocol/heartbeat.hpp>

#include <array>

namespace kpi4::protocol {
namespace {

template <std::size_t N>
std::string_view name_at(const std::array<std::string_view, N>& names,
                         std::uint8_t value) {
    return value < names.size() ? names[value] : "UNKNOWN";
}

constexpr std::array<std::string_view, MAV_TYPE_ENUM_END> type_names = {
    "GENERIC", "FIXED_WING", "QUADROTOR", "COAXIAL", "HELICOPTER",
    "ANTENNA_TRACKER", "GCS", "AIRSHIP", "FREE_BALLOON", "ROCKET",
    "GROUND_ROVER", "SURFACE_BOAT", "SUBMARINE", "HEXAROTOR", "OCTOROTOR",
    "TRICOPTER", "FLAPPING_WING", "KITE", "ONBOARD_CONTROLLER",
    "VTOL_TAILSITTER_DUOROTOR", "VTOL_TAILSITTER_QUADROTOR",
    "VTOL_TILTROTOR", "VTOL_FIXEDROTOR", "VTOL_TAILSITTER",
    "VTOL_TILTWING", "RESERVED", "GIMBAL", "ADSB", "PARAFOIL",
    "DODECAROTOR", "CAMERA", "CHARGING_STATION", "FLARM", "SERVO",
    "ODID", "DECAROTOR", "BATTERY", "PARACHUTE", "LOG", "OSD", "IMU",
    "GPS", "WINCH", "GENERIC_MULTIROTOR", "ILLUMINATOR",
    "SPACECRAFT_ORBITER", "GROUND_QUADRUPED", "VTOL_GYRODYNE", "GRIPPER",
    "RADIO",
};

constexpr std::array<std::string_view, MAV_AUTOPILOT_ENUM_END> autopilot_names = {
    "GENERIC", "RESERVED", "SLUGS", "ARDUPILOTMEGA", "OPENPILOT",
    "GENERIC_WAYPOINTS_ONLY", "GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY",
    "GENERIC_MISSION_FULL", "INVALID", "PPZ", "UDB", "FP", "PX4",
    "SMACCMPILOT", "AUTOQUAD", "ARMAZILA", "AEROB", "ASLUAV",
    "SMARTAP", "AIRRAILS", "REFLEX", "FLIX", "BETAFLIGHT",
};

constexpr std::array<std::string_view, MAV_STATE_ENUM_END> state_names = {
    "UNINIT", "BOOT", "CALIBRATING", "STANDBY", "ACTIVE", "CRITICAL",
    "EMERGENCY", "POWEROFF", "FLIGHT_TERMINATION",
};

static_assert(type_names.size() == MAV_TYPE_ENUM_END);
static_assert(autopilot_names.size() == MAV_AUTOPILOT_ENUM_END);
static_assert(state_names.size() == MAV_STATE_ENUM_END);

}  // namespace

std::optional<Heartbeat> decode_heartbeat(const FrameReadResult& event) {
    if (!event.frame || event.frame->msgid != MAVLINK_MSG_ID_HEARTBEAT ||
        event.frame->len != MAVLINK_MSG_ID_HEARTBEAT_LEN) {
        return std::nullopt;
    }

    mavlink_heartbeat_t payload{};
    mavlink_msg_heartbeat_decode(&*event.frame, &payload);
    return Heartbeat{event.channel,
                     event.frame->sysid,
                     event.frame->compid,
                     payload.type,
                     payload.autopilot,
                     payload.base_mode,
                     payload.custom_mode,
                     payload.system_status,
                     payload.mavlink_version};
}

std::string_view heartbeat_type_name(std::uint8_t value) {
    return name_at(type_names, value);
}

std::string_view heartbeat_autopilot_name(std::uint8_t value) {
    return name_at(autopilot_names, value);
}

std::string_view heartbeat_state_name(std::uint8_t value) {
    return name_at(state_names, value);
}

}  // namespace kpi4::protocol
