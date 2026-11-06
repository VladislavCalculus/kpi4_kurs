#include <kpi4/state/mavlink_provider.hpp>

#include <cmath>
#include <utility>

namespace kpi4::state {

MavlinkStateProvider::MavlinkStateProvider(core::StateSource source)
    : source_(source) {}

void MavlinkStateProvider::accept(const protocol::EstimatedPosition& position,
                                  SampleTimes times) {
    latest_ = core::StateSample{
        times.measured_at,
        times.received_at,
        source_,
        position.position,
        position.altitude_msl,
        position.horizontal_velocity,
        position.heading,
        std::nullopt,
        std::nullopt,
    };
}

void MavlinkStateProvider::accept(const protocol::RawGps& gps,
                                  SampleTimes times) {
    std::optional<core::NorthEastVelocity> velocity;
    if (gps.ground_speed && gps.course_over_ground) {
        constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
        const auto radians = gps.course_over_ground->value * radians_per_degree;
        velocity = core::NorthEastVelocity{
            core::MetersPerSecond{gps.ground_speed->value * std::cos(radians)},
            core::MetersPerSecond{gps.ground_speed->value * std::sin(radians)},
        };
    }

    std::optional<core::GpsFix> fix;
    switch (gps.fix_type) {
        case GPS_FIX_TYPE_NO_GPS:
        case GPS_FIX_TYPE_NO_FIX:
            fix = core::GpsFix::no_fix;
            break;
        case GPS_FIX_TYPE_2D_FIX:
            fix = core::GpsFix::two_dimensional;
            break;
        case GPS_FIX_TYPE_3D_FIX:
        case GPS_FIX_TYPE_DGPS:
        case GPS_FIX_TYPE_RTK_FLOAT:
        case GPS_FIX_TYPE_RTK_FIXED:
        case GPS_FIX_TYPE_PPP:
            fix = core::GpsFix::three_dimensional;
            break;
        default:
            break;
    }

    auto heading = gps.gps_yaw;
    if (heading && heading->value == 360.0) {
        heading = core::Degrees{0.0};
    }

    latest_ = core::StateSample{
        times.measured_at,
        times.received_at,
        source_,
        gps.position,
        gps.altitude_msl,
        velocity,
        heading,
        gps.horizontal_uncertainty,
        fix,
    };
}

std::optional<core::StateSample> MavlinkStateProvider::take_latest() {
    return std::exchange(latest_, std::nullopt);
}

}  // namespace kpi4::state
