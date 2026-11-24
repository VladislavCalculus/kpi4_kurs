#include <kpi4/mission/route_follower.hpp>

#include <utility>

namespace kpi4::mission {
namespace {
core::SegmentStep status_only(core::ModuleStatus status,
                             std::optional<std::uint32_t> segment = std::nullopt) {
    return {{std::nullopt, std::nullopt, segment, false, status}, std::nullopt};
}
}  // namespace

RouteFollower::RouteFollower(std::vector<core::MissionItem> items, core::GuidanceConfig config)
    : items_(std::move(items)), config_(config) {
    if (items_.size() < 2) {
        initial_status_ = core::ModuleStatus::invalid_mission;
        return;
    }
    // Reject the complete route before issuing any target. Reuse the same
    // configuration/geometry checks as the segment controller.
    for (std::size_t i = 1; i < items_.size(); ++i) {
        const core::StateSample probe{{0}, {0}, core::StateSource::simulation,
            items_[i - 1].target, std::nullopt, core::NorthEastVelocity{{0}, {0}},
            std::nullopt, std::nullopt, std::nullopt};
        const auto checked = core::follow_segment(items_[i - 1], items_[i], probe, config_);
        if (!checked.geometry) {
            initial_status_ = checked.guidance.status;
            return;
        }
    }
}

std::optional<std::uint32_t> RouteFollower::active_segment() const {
    if (initial_status_ != core::ModuleStatus::ready || completed_ + 1 >= items_.size())
        return std::nullopt;
    return items_[completed_ + 1].sequence;
}

core::SegmentStep RouteFollower::update(const core::StateSample& sample) {
    if (initial_status_ != core::ModuleStatus::ready) return status_only(initial_status_);
    if (completed_ + 1 == items_.size()) return status_only(core::ModuleStatus::mission_completed);
    while (completed_ + 1 < items_.size()) {
        auto step = core::follow_segment(items_[completed_], items_[completed_ + 1], sample, config_);
        if (step.guidance.status != core::ModuleStatus::target_reached) return step;
        ++completed_;
        if (completed_ + 1 == items_.size()) {
            step.guidance.status = core::ModuleStatus::mission_completed;
            step.guidance.active_segment.reset();
            return step;
        }
        // The same accepted sample starts the next segment, without an empty
        // command cycle at an intermediate waypoint.
    }
    return status_only(core::ModuleStatus::mission_completed);
}
}  // namespace kpi4::mission
