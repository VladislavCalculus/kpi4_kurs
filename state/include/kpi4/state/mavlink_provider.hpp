#pragma once

#include <kpi4/protocol/estimated_position.hpp>
#include <kpi4/protocol/raw_gps.hpp>
#include <kpi4/state/provider.hpp>

namespace kpi4::state {

class MavlinkStateProvider final : public StateProvider {
public:
    // The caller identifies the physical/logical source. A MAVLink input
    // channel alone does not establish sensor provenance.
    explicit MavlinkStateProvider(core::StateSource source);

    // Times must already be mapped to the module's monotonic clock. Sender
    // boot/epoch timestamps are not implicitly substituted for them.
    void accept(const protocol::EstimatedPosition& position, SampleTimes times);
    void accept(const protocol::RawGps& gps, SampleTimes times);
    std::optional<core::StateSample> take_latest() override;

private:
    core::StateSource source_;
    std::optional<core::StateSample> latest_;
};

}  // namespace kpi4::state
