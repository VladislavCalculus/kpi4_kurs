#include <kpi4/state/validator.hpp>

#include <cmath>
#include <cstddef>

namespace kpi4::state {
namespace {

std::optional<std::size_t> source_index(core::StateSource source) {
    switch (source) {
        case core::StateSource::controller_telemetry:
            return 0;
        case core::StateSource::direct_gnss:
            return 1;
        case core::StateSource::ground_station:
            return 2;
        case core::StateSource::simulation:
            return 3;
    }
    return std::nullopt;
}

bool known_fix(core::GpsFix fix) {
    switch (fix) {
        case core::GpsFix::no_fix:
        case core::GpsFix::two_dimensional:
        case core::GpsFix::three_dimensional:
            return true;
    }
    return false;
}

}  // namespace

StateValidator::StateValidator(StateValidationConfig config)
    : config_(config),
      configuration_valid_(
          (!config.max_horizontal_accuracy ||
           (std::isfinite(config.max_horizontal_accuracy->value) &&
            config.max_horizontal_accuracy->value >= 0.0)) &&
          (!config.minimum_gps_fix ||
           *config.minimum_gps_fix == core::GpsFix::two_dimensional ||
           *config.minimum_gps_fix == core::GpsFix::three_dimensional)) {}

StateValidationResult StateValidator::validate(
    const core::StateSample& sample, core::MonotonicMicroseconds now) {
    if (!configuration_valid_) {
        return StateRejectReason::InvalidConfiguration;
    }
    const auto index = source_index(sample.source);
    if (!index) {
        return StateRejectReason::UnknownSource;
    }
    const auto latitude = sample.position.latitude.value;
    const auto longitude = sample.position.longitude.value;
    if (!std::isfinite(latitude) || latitude < -90.0 || latitude > 90.0 ||
        !std::isfinite(longitude) || longitude < -180.0 || longitude > 180.0) {
        return StateRejectReason::InvalidPosition;
    }
    if (sample.altitude && !std::isfinite(sample.altitude->value.value)) {
        return StateRejectReason::InvalidAltitude;
    }
    if (sample.altitude) {
        switch (sample.altitude->reference) {
            case core::AltitudeReference::mean_sea_level:
            case core::AltitudeReference::relative_to_home:
            case core::AltitudeReference::above_terrain:
                break;
            default:
                return StateRejectReason::InvalidAltitude;
        }
    }
    if (sample.velocity &&
        (!std::isfinite(sample.velocity->north.value) ||
         !std::isfinite(sample.velocity->east.value))) {
        return StateRejectReason::InvalidVelocity;
    }
    if (sample.heading_true_north &&
        (!std::isfinite(sample.heading_true_north->value) ||
         sample.heading_true_north->value < 0.0 ||
         sample.heading_true_north->value >= 360.0)) {
        return StateRejectReason::InvalidHeading;
    }
    if (sample.horizontal_accuracy &&
        (!std::isfinite(sample.horizontal_accuracy->value) ||
         sample.horizontal_accuracy->value < 0.0)) {
        return StateRejectReason::InvalidAccuracy;
    }
    if (sample.gps_fix && !known_fix(*sample.gps_fix)) {
        return StateRejectReason::UnknownFix;
    }
    if (sample.gps_fix && *sample.gps_fix == core::GpsFix::no_fix) {
        return StateRejectReason::NoFix;
    }
    if (config_.minimum_gps_fix &&
        (!sample.gps_fix ||
         (*config_.minimum_gps_fix == core::GpsFix::three_dimensional &&
          *sample.gps_fix != core::GpsFix::three_dimensional))) {
        return StateRejectReason::InsufficientFix;
    }
    if (config_.max_horizontal_accuracy && !sample.horizontal_accuracy) {
        return StateRejectReason::AccuracyUnavailable;
    }
    if (config_.max_horizontal_accuracy &&
        sample.horizontal_accuracy->value >
            config_.max_horizontal_accuracy->value) {
        return StateRejectReason::AccuracyExceedsLimit;
    }

    const auto measured = sample.measured_at.value;
    const auto received = sample.received_at.value;
    if (measured > received) {
        return StateRejectReason::MeasurementAfterReception;
    }
    if (received > now.value) {
        return StateRejectReason::ReceptionInFuture;
    }
    if (now.value - measured > config_.max_measurement_age_us) {
        return StateRejectReason::MeasurementStale;
    }
    if (received - measured > config_.max_delivery_delay_us) {
        return StateRejectReason::DeliveryDelayExceeded;
    }
    const auto& previous = last_accepted_[*index];
    if (previous &&
        (measured <= previous->measured_at ||
         received < previous->received_at)) {
        return StateRejectReason::OutOfOrder;
    }

    last_accepted_[*index] = LastAccepted{measured, received};
    return sample;
}

}  // namespace kpi4::state
