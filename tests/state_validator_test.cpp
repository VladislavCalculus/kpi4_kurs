#include <kpi4/state/validator.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_rejection(const kpi4::state::StateValidationConfig& config,
                      const kpi4::core::StateSample& sample,
                      std::uint64_t now,
                      kpi4::state::StateRejectReason reason) {
    kpi4::state::StateValidator validator(config);
    const auto result = validator.validate(sample, {now});
    const auto* actual = std::get_if<kpi4::state::StateRejectReason>(&result);
    require(actual && *actual == reason, "wrong state rejection reason");
}

void expect_acceptance(const kpi4::state::StateValidationConfig& config,
                       const kpi4::core::StateSample& sample,
                       std::uint64_t now) {
    kpi4::state::StateValidator validator(config);
    const auto result = validator.validate(sample, {now});
    require(std::get_if<kpi4::core::StateSample>(&result) != nullptr,
            "valid state was rejected");
}

}  // namespace

int main() {
    using namespace kpi4;
    using state::StateRejectReason;

    // Demo scenario limits, not universal flight tolerances.
    const state::StateValidationConfig config{
        200'000,  // maximum measurement age: 200 ms
        50'000,   // maximum delivery delay: 50 ms
        core::Meters{5.0},
        core::GpsFix::two_dimensional,
    };
    const core::StateSample base{
        {1'000'000}, {1'020'000}, core::StateSource::direct_gnss,
        {{50.0}, {30.0}},
        core::Altitude{{120.0}, core::AltitudeReference::mean_sea_level},
        core::NorthEastVelocity{{2.0}, {-3.0}},
        core::Degrees{270.0}, core::Meters{4.0},
        core::GpsFix::three_dimensional,
    };
    constexpr std::uint64_t now = 1'100'000;
    state::StateValidator validator(config);
    const auto accepted = validator.validate(base, {now});
    require(std::get_if<core::StateSample>(&accepted),
            "valid state was rejected");

    auto boundary = base;
    boundary.measured_at.value = 900'000;
    boundary.received_at.value = 950'000;
    boundary.horizontal_accuracy = core::Meters{5.0};
    boundary.gps_fix = core::GpsFix::two_dimensional;
    expect_acceptance(config, boundary, now);

    auto sample = base;
    sample.position.latitude.value = std::numeric_limits<double>::quiet_NaN();
    expect_rejection(config, sample, now, StateRejectReason::InvalidPosition);
    sample = base;
    sample.position.longitude.value = 180.000001;
    expect_rejection(config, sample, now, StateRejectReason::InvalidPosition);
    sample = base;
    sample.altitude->value.value = std::numeric_limits<double>::infinity();
    expect_rejection(config, sample, now, StateRejectReason::InvalidAltitude);
    sample = base;
    sample.altitude->reference = static_cast<core::AltitudeReference>(99);
    expect_rejection(config, sample, now, StateRejectReason::InvalidAltitude);
    sample = base;
    sample.velocity->east.value = -std::numeric_limits<double>::infinity();
    expect_rejection(config, sample, now, StateRejectReason::InvalidVelocity);
    sample = base;
    sample.heading_true_north->value = 360.0;
    expect_rejection(config, sample, now, StateRejectReason::InvalidHeading);
    sample = base;
    sample.horizontal_accuracy->value = -1.0;
    expect_rejection(config, sample, now, StateRejectReason::InvalidAccuracy);
    sample = base;
    sample.source = static_cast<core::StateSource>(99);
    expect_rejection(config, sample, now, StateRejectReason::UnknownSource);
    sample = base;
    sample.gps_fix = static_cast<core::GpsFix>(99);
    expect_rejection(config, sample, now, StateRejectReason::UnknownFix);

    sample = base;
    sample.gps_fix = core::GpsFix::no_fix;
    expect_rejection(config, sample, now, StateRejectReason::NoFix);
    sample = base;
    sample.gps_fix.reset();
    expect_rejection(config, sample, now, StateRejectReason::InsufficientFix);
    sample = base;
    sample.gps_fix = core::GpsFix::two_dimensional;
    auto three_dimensional_config = config;
    three_dimensional_config.minimum_gps_fix = core::GpsFix::three_dimensional;
    expect_rejection(three_dimensional_config, sample, now,
                     StateRejectReason::InsufficientFix);
    sample = base;
    sample.horizontal_accuracy.reset();
    expect_rejection(config, sample, now,
                     StateRejectReason::AccuracyUnavailable);
    sample = base;
    sample.horizontal_accuracy = core::Meters{5.01};
    expect_rejection(config, sample, now,
                     StateRejectReason::AccuracyExceedsLimit);

    sample = base;
    sample.measured_at.value = sample.received_at.value + 1;
    expect_rejection(config, sample, now,
                     StateRejectReason::MeasurementAfterReception);
    sample = base;
    sample.received_at.value = now + 1;
    expect_rejection(config, sample, now,
                     StateRejectReason::ReceptionInFuture);
    sample = base;
    sample.measured_at.value = now - config.max_measurement_age_us - 1;
    expect_rejection(config, sample, now,
                     StateRejectReason::MeasurementStale);
    sample = base;
    sample.received_at.value = sample.measured_at.value +
                               config.max_delivery_delay_us + 1;
    expect_rejection(config, sample, now,
                     StateRejectReason::DeliveryDelayExceeded);

    // Estimated telemetry can be accepted without fix/accuracy only when
    // that absence is an explicit choice in the supplied configuration.
    const state::StateValidationConfig permissive{
        200'000, 50'000, std::nullopt, std::nullopt};
    sample = base;
    sample.source = core::StateSource::controller_telemetry;
    sample.gps_fix.reset();
    sample.horizontal_accuracy.reset();
    expect_acceptance(permissive, sample, now);
    sample.gps_fix = core::GpsFix::no_fix;
    expect_rejection(permissive, sample, now, StateRejectReason::NoFix);

    auto bad_config = config;
    bad_config.max_horizontal_accuracy =
        core::Meters{std::numeric_limits<double>::quiet_NaN()};
    expect_rejection(bad_config, base, now,
                     StateRejectReason::InvalidConfiguration);
    bad_config = config;
    bad_config.minimum_gps_fix = core::GpsFix::no_fix;
    expect_rejection(bad_config, base, now,
                     StateRejectReason::InvalidConfiguration);

    // Accepted order is tracked per source. Rejections must not advance it.
    const auto duplicate = validator.validate(base, {now});
    require(std::get_if<StateRejectReason>(&duplicate) &&
                std::get<StateRejectReason>(duplicate) ==
                    StateRejectReason::OutOfOrder,
            "duplicate measurement timestamp was accepted");
    auto old = base;
    old.measured_at.value -= 1;
    old.received_at.value += 1;
    const auto reversed = validator.validate(old, {now});
    require(std::get_if<StateRejectReason>(&reversed) &&
                std::get<StateRejectReason>(reversed) ==
                    StateRejectReason::OutOfOrder,
            "older measurement was accepted");
    auto newer = base;
    newer.measured_at.value += 1;
    newer.received_at.value += 1;
    const auto accepted_newer = validator.validate(newer, {now});
    require(std::get_if<core::StateSample>(&accepted_newer),
            "rejected sample advanced the sequence");
    auto other_source = base;
    other_source.source = core::StateSource::ground_station;
    const auto accepted_other = validator.validate(other_source, {now});
    require(std::get_if<core::StateSample>(&accepted_other),
            "one source incorrectly advanced another source's sequence");
    auto reversed_receipt = newer;
    reversed_receipt.measured_at.value += 1;
    reversed_receipt.received_at.value -= 1;
    const auto out_of_order_receipt = validator.validate(reversed_receipt, {now});
    require(std::get_if<StateRejectReason>(&out_of_order_receipt) &&
                std::get<StateRejectReason>(out_of_order_receipt) ==
                    StateRejectReason::OutOfOrder,
            "older reception time was accepted");
}
