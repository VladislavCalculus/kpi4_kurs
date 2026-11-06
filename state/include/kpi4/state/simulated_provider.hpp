#pragma once

#include <kpi4/state/provider.hpp>

namespace kpi4::state {

class SimulatedStateProvider final : public StateProvider {
public:
    // The simulation owns the motion model and timestamps. This adapter
    // gives its observations the same delivery contract as live telemetry.
    void publish(core::StateSample sample);
    std::optional<core::StateSample> take_latest() override;

private:
    std::optional<core::StateSample> latest_;
};

}  // namespace kpi4::state
