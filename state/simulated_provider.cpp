#include <kpi4/state/simulated_provider.hpp>

#include <utility>

namespace kpi4::state {

void SimulatedStateProvider::publish(core::StateSample sample) {
    sample.source = core::StateSource::simulation;
    latest_ = sample;
}

std::optional<core::StateSample> SimulatedStateProvider::take_latest() {
    return std::exchange(latest_, std::nullopt);
}

}  // namespace kpi4::state
