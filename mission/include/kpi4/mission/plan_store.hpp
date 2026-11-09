#pragma once

#include <kpi4/protocol/mission_upload.hpp>

#include <cstdint>
#include <memory>
#include <variant>

namespace kpi4::mission {

struct MissionPlan {
    std::uint64_t version;
    protocol::MissionUploadPeer owner;
    std::uint8_t mission_type;
    std::vector<core::MissionItem> items;
    std::vector<protocol::MissionItemInt> raw_items;
};

enum class PlanStoreError {
    InvalidCandidate,
    ConflictingSource,
    ConflictUnresolved,
    MissionStarted,
    OwnerLocked,
    VersionExhausted,
};

enum class MissionStartResult {
    Started,
    NoPlan,
    Conflict,
    AlreadyStarted,
};

using PlanStoreResult =
    std::variant<std::shared_ptr<const MissionPlan>, PlanStoreError>;

class MissionPlanStore {
public:
    // Accept only a complete candidate. Returned snapshots remain immutable
    // after later preflight replacements.
    PlanStoreResult accept(protocol::MissionUploadCandidate candidate);
    std::shared_ptr<const MissionPlan> active() const;
    MissionStartResult start();
    bool has_conflict() const;
    bool started() const;
    // Requires an explicit caller decision to discard the preflight plan.
    bool discard_preflight();

private:
    std::shared_ptr<const MissionPlan> active_;
    std::uint64_t last_version_ = 0;
    bool conflict_ = false;
    bool started_ = false;
};

}  // namespace kpi4::mission
