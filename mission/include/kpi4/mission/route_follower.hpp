#pragma once

#include <kpi4/core/guidance.hpp>

#include <cstddef>
#include <vector>

namespace kpi4::mission {

// A snapshot of a route. Item 0 is the segment origin, item 1 the first goal.
// Time/source validation belongs to the caller and must precede update().
class RouteFollower {
public:
    RouteFollower(std::vector<core::MissionItem> items, core::GuidanceConfig config);
    core::SegmentStep update(const core::StateSample& validated_sample);
    std::size_t completed_segments() const { return completed_; }
    std::optional<std::uint32_t> active_segment() const;

private:
    std::vector<core::MissionItem> items_;
    core::GuidanceConfig config_;
    std::size_t completed_{0};
    core::ModuleStatus initial_status_{core::ModuleStatus::ready};
};

}  // namespace kpi4::mission
