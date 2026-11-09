#include <kpi4/mission/plan_store.hpp>

#include <limits>
#include <utility>

namespace kpi4::mission {
namespace {

bool same_source(const protocol::MissionUploadPeer& left,
                 const protocol::MissionUploadPeer& right) {
    return left.channel == right.channel &&
           left.system_id == right.system_id &&
           left.component_id == right.component_id;
}

bool complete(const protocol::MissionUploadCandidate& candidate) {
    if (candidate.mission_type != MAV_MISSION_TYPE_MISSION ||
        candidate.items.empty() ||
        candidate.items.size() != candidate.raw_items.size()) {
        return false;
    }
    for (std::size_t i = 0; i < candidate.items.size(); ++i) {
        if (candidate.items[i].sequence != i ||
            candidate.raw_items[i].seq != i) {
            return false;
        }
    }
    return true;
}

}  // namespace

PlanStoreResult MissionPlanStore::accept(
    protocol::MissionUploadCandidate candidate) {
    if (!complete(candidate)) return PlanStoreError::InvalidCandidate;
    if (started_) {
        return active_ && same_source(active_->owner, candidate.source)
                   ? PlanStoreError::MissionStarted
                   : PlanStoreError::OwnerLocked;
    }
    if (active_ && !same_source(active_->owner, candidate.source)) {
        conflict_ = true;
        return PlanStoreError::ConflictingSource;
    }
    if (conflict_) return PlanStoreError::ConflictUnresolved;
    if (last_version_ == std::numeric_limits<std::uint64_t>::max())
        return PlanStoreError::VersionExhausted;

    auto plan = std::make_shared<const MissionPlan>(MissionPlan{
        last_version_ + 1, candidate.source, candidate.mission_type,
        std::move(candidate.items), std::move(candidate.raw_items)});
    active_ = plan;
    ++last_version_;
    return plan;
}

std::shared_ptr<const MissionPlan> MissionPlanStore::active() const {
    return active_;
}

MissionStartResult MissionPlanStore::start() {
    if (started_) return MissionStartResult::AlreadyStarted;
    if (conflict_) return MissionStartResult::Conflict;
    if (!active_) return MissionStartResult::NoPlan;
    started_ = true;
    return MissionStartResult::Started;
}

bool MissionPlanStore::has_conflict() const { return conflict_; }

bool MissionPlanStore::started() const { return started_; }

bool MissionPlanStore::discard_preflight() {
    if (started_) return false;
    active_.reset();
    conflict_ = false;
    return true;
}

}  // namespace kpi4::mission
