#pragma once

#include <kpi4/core/types.hpp>

namespace kpi4::core {

struct LocalPosition {
    Meters east;
    Meters north;
};

// First-order WGS84 tangent frame: |latitude| <= 80 degrees, each local
// coordinate <= 2 km. Intended for short horizontal demonstration segments.
std::optional<LocalPosition> to_local(Wgs84Position origin, Wgs84Position position);
std::optional<Wgs84Position> to_wgs84(Wgs84Position origin, LocalPosition position);

struct GuidanceConfig {
    Meters minimum_segment_length{1.0};
    Meters reach_radius{3.0};
    Meters correction_deadband{0.1};
    Meters minimum_lookahead{5.0};
    double lookahead_seconds{1.0};
    MetersPerSecond cruise_speed{8.0};
    MetersPerSecond max_speed{12.0};
};

struct SegmentGeometry {
    Meters length;
    double progress;  // Unclamped: 0 at the start, 1 at the end.
    Meters cross_track_error;  // Positive to the left of the directed segment.
    Meters distance_to_end;
    LocalPosition closest;
};

struct SegmentStep {
    GuidanceResult guidance;
    std::optional<SegmentGeometry> geometry;
    bool speed_limited{false};
};

// Pure horizontal guidance. The caller validates sample freshness/source first.
// A reached segment produces no velocity target; mission lifecycle belongs to
// the caller. Altitude control and physical controller encoding are separate.
SegmentStep follow_segment(const MissionItem& from, const MissionItem& to,
                           const StateSample& sample, const GuidanceConfig& config);

}  // namespace kpi4::core
