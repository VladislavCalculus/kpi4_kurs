#pragma once

#include <kpi4/state/provider.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <variant>

namespace kpi4::state {

// All limits are supplied by the integration or scenario; there are no
// implicit timing, GPS-fix, or accuracy thresholds.
struct StateValidationConfig {
    std::uint64_t max_measurement_age_us;
    std::uint64_t max_delivery_delay_us;
    std::optional<core::Meters> max_horizontal_accuracy;
    std::optional<core::GpsFix> minimum_gps_fix;
};

enum class StateRejectReason {
    InvalidConfiguration,
    UnknownSource,
    InvalidPosition,
    InvalidAltitude,
    InvalidVelocity,
    InvalidHeading,
    InvalidAccuracy,
    UnknownFix,
    NoFix,
    InsufficientFix,
    AccuracyUnavailable,
    AccuracyExceedsLimit,
    MeasurementAfterReception,
    ReceptionInFuture,
    MeasurementStale,
    DeliveryDelayExceeded,
    OutOfOrder,
};

using StateValidationResult =
    std::variant<core::StateSample, StateRejectReason>;

class StateValidator {
public:
    explicit StateValidator(StateValidationConfig config);

    // Accepts a candidate with times already expressed on the module's
    // monotonic clock. Only accepted samples advance per-source order.
    StateValidationResult validate(const core::StateSample& sample,
                                   core::MonotonicMicroseconds now);

private:
    struct LastAccepted {
        std::uint64_t measured_at;
        std::uint64_t received_at;
    };

    StateValidationConfig config_;
    bool configuration_valid_;
    std::array<std::optional<LastAccepted>, 4> last_accepted_{};
};

}  // namespace kpi4::state
