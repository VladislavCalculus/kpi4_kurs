#pragma once

#include <kpi4/core/types.hpp>

#include <optional>

namespace kpi4::state {

struct SampleTimes {
    core::MonotonicMicroseconds measured_at;
    core::MonotonicMicroseconds received_at;
};

class StateProvider {
public:
    virtual ~StateProvider() = default;

    // Returns the latest unread candidate, then clears it. The caller must
    // validate the sample before using it for guidance. Providers retain only
    // one sample so a slow consumer receives the newest observation.
    virtual std::optional<core::StateSample> take_latest() = 0;
};

}  // namespace kpi4::state
