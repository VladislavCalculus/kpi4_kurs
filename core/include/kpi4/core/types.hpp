#pragma once

#include <cstdint>
#include <optional>
#include <variant>

namespace kpi4::core {

struct Degrees {
    double value;
};

struct Meters {
    double value;
};

struct MetersPerSecond {
    double value;
};

// Both state timestamps use the module's monotonic clock.
struct MonotonicMicroseconds {
    std::uint64_t value;
};

struct Wgs84Position {
    Degrees latitude;
    Degrees longitude;
};

enum class AltitudeReference {
    mean_sea_level,
    relative_to_home,
    above_terrain,
};

struct Altitude {
    Meters value;
    AltitudeReference reference;
};

struct NorthEastVelocity {
    MetersPerSecond north;
    MetersPerSecond east;
};

struct MissionItem {
    std::uint32_t sequence;
    Wgs84Position target;
    std::optional<Altitude> altitude;
    std::optional<MetersPerSecond> desired_speed;
};

enum class StateSource {
    controller_telemetry,
    direct_gnss,
    ground_station,
    simulation,
};

enum class GpsFix {
    no_fix,
    two_dimensional,
    three_dimensional,
};

struct StateSample {
    MonotonicMicroseconds measured_at;
    MonotonicMicroseconds received_at;
    StateSource source;
    Wgs84Position position;
    std::optional<Altitude> altitude;
    std::optional<NorthEastVelocity> velocity;
    std::optional<Degrees> heading_true_north;
    std::optional<Meters> horizontal_accuracy;
    std::optional<GpsFix> gps_fix;
};

enum class ModuleStatus {
    ready,
    moving_without_correction,
    corrected_target,
    invalid_mission,
    invalid_configuration,
    invalid_state,
    stale_state,
    command_unavailable,
};

// The output adapter chooses how to encode a supported domain target.
using GuidanceTarget = std::variant<Wgs84Position, NorthEastVelocity>;

struct GuidanceResult {
    std::optional<GuidanceTarget> target;
    std::optional<Meters> cross_track_error;
    std::optional<std::uint32_t> active_segment;
    bool correction_required;
    ModuleStatus status;
};

}  // namespace kpi4::core
