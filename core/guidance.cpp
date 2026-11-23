#include <kpi4/core/guidance.hpp>

#include <algorithm>
#include <cmath>

namespace kpi4::core {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;
constexpr double local_limit = 2000.0;

bool valid_position(Wgs84Position p) {
    return std::isfinite(p.latitude.value) && std::isfinite(p.longitude.value) &&
           std::abs(p.latitude.value) <= 90.0 && std::abs(p.longitude.value) <= 180.0;
}

bool valid_local(LocalPosition p) {
    return std::isfinite(p.east.value) && std::isfinite(p.north.value) &&
           std::abs(p.east.value) <= local_limit && std::abs(p.north.value) <= local_limit;
}

bool valid_origin(Wgs84Position p) {
    return valid_position(p) && std::abs(p.latitude.value) <= 80.0;
}

LocalPosition scales(Wgs84Position origin) {
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    const double latitude = origin.latitude.value * radians;
    const double denominator = 1.0 - e2 * std::pow(std::sin(latitude), 2);
    return {{a / std::sqrt(denominator) * std::cos(latitude) * radians},
            {a * (1.0 - e2) / std::pow(denominator, 1.5) * radians}};
}

bool positive(double value) { return std::isfinite(value) && value > 0.0; }

SegmentStep rejected(ModuleStatus status) {
    return {{std::nullopt, std::nullopt, std::nullopt, false, status}, std::nullopt};
}
}  // namespace

std::optional<LocalPosition> to_local(Wgs84Position origin, Wgs84Position p) {
    if (!valid_origin(origin) || !valid_position(p)) return std::nullopt;
    const auto scale = scales(origin);
    const LocalPosition local{
        {std::remainder(p.longitude.value - origin.longitude.value, 360.0) * scale.east.value},
        {(p.latitude.value - origin.latitude.value) * scale.north.value}};
    return valid_local(local) ? std::optional<LocalPosition>{local} : std::nullopt;
}

std::optional<Wgs84Position> to_wgs84(Wgs84Position origin, LocalPosition p) {
    if (!valid_origin(origin) || !valid_local(p)) return std::nullopt;
    const auto scale = scales(origin);
    const Wgs84Position result{
        {origin.latitude.value + p.north.value / scale.north.value},
        {std::remainder(origin.longitude.value + p.east.value / scale.east.value, 360.0)}};
    return valid_position(result) ? std::optional<Wgs84Position>{result} : std::nullopt;
}

SegmentStep follow_segment(const MissionItem& from, const MissionItem& to,
                           const StateSample& sample, const GuidanceConfig& c) {
    if (!positive(c.minimum_segment_length.value) || !positive(c.reach_radius.value) ||
        !std::isfinite(c.correction_deadband.value) || c.correction_deadband.value < 0.0 ||
        !positive(c.minimum_lookahead.value) || !positive(c.lookahead_seconds) ||
        !positive(c.cruise_speed.value) || !positive(c.max_speed.value))
        return rejected(ModuleStatus::invalid_configuration);
    const auto end = to_local(from.target, to.target);
    if (!end) return rejected(ModuleStatus::invalid_mission);
    const double de = end->east.value, dn = end->north.value;
    const double length = std::hypot(de, dn);
    if (length < c.minimum_segment_length.value ||
        (to.desired_speed && !positive(to.desired_speed->value)))
        return rejected(ModuleStatus::invalid_mission);
    const auto p = to_local(from.target, sample.position);
    if (!p || !sample.velocity || !std::isfinite(sample.velocity->north.value) ||
        !std::isfinite(sample.velocity->east.value)) return rejected(ModuleStatus::invalid_state);
    const double ground_speed = std::hypot(sample.velocity->north.value, sample.velocity->east.value);
    if (!std::isfinite(ground_speed)) return rejected(ModuleStatus::invalid_state);
    const double unit_east = de / length, unit_north = dn / length;
    const double progress = (unit_east * p->east.value + unit_north * p->north.value) / length;
    const double cross = unit_east * p->north.value - unit_north * p->east.value;
    const double distance = std::hypot(de - p->east.value, dn - p->north.value);
    const double closest = std::clamp(progress, 0.0, 1.0);
    SegmentStep result{{std::nullopt, Meters{cross}, to.sequence, false,
                        ModuleStatus::target_reached},
                       SegmentGeometry{{length}, progress, {cross}, {distance},
                                       {{closest * de}, {closest * dn}}}};
    if (distance <= c.reach_radius.value) return result;
    const double requested = to.desired_speed ? to.desired_speed->value : c.cruise_speed.value;
    const double speed = std::min(requested, c.max_speed.value);
    result.speed_limited = requested > c.max_speed.value;
    const bool correcting = std::abs(cross) > c.correction_deadband.value || progress > 1.0;
    double east = unit_east, north = unit_north;
    if (correcting) {
        const double lookahead = std::min(length,
            std::max(c.minimum_lookahead.value, c.lookahead_seconds * ground_speed));
        const double aim = std::min(length, std::max(0.0, progress * length) + lookahead);
        east = unit_east * aim - p->east.value;
        north = unit_north * aim - p->north.value;
        const double norm = std::hypot(east, north);
        // The aim can coincide with the sample only at the endpoint, already
        // handled by reach_radius above.
        east /= norm;
        north /= norm;
    }
    result.guidance.target = NorthEastVelocity{{speed * north}, {speed * east}};
    result.guidance.correction_required = correcting;
    result.guidance.status = correcting ? ModuleStatus::corrected_target
                                        : ModuleStatus::moving_without_correction;
    return result;
}
}  // namespace kpi4::core
