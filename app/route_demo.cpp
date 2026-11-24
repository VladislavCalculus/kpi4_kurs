#include "route_demo.hpp"

#include <kpi4/mission/route_follower.hpp>
#include <kpi4/state/simulated_provider.hpp>
#include <kpi4/state/validator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace kpi4::demo {
namespace {
std::ofstream file(const std::filesystem::path& path) {
    std::ofstream out(path);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << std::setprecision(12);
    return out;
}

std::string reject_reason(state::StateRejectReason reason) {
    switch (reason) {
        case state::StateRejectReason::MeasurementStale: return "measurement_stale";
        case state::StateRejectReason::NoFix: return "no_fix";
        default: return "state_rejected_" + std::to_string(static_cast<int>(reason));
    }
}
}  // namespace

int run_route(const std::filesystem::path& output, const RouteOptions& options) {
    constexpr double dt = 0.05;
    constexpr double response = 0.5;
    constexpr unsigned max_steps = 1200;
    const core::Wgs84Position origin{{50.0}, {30.0}};
    const std::array<core::LocalPosition, 4> points{{{{0}, {0}}, {{60}, {0}},
                                                  {{60}, {50}}, {{110}, {50}}}};
    std::vector<core::MissionItem> items;
    auto waypoints = file(output / "waypoints.csv");
    waypoints << "sequence,east_m,north_m\n";
    for (std::size_t i = 0; i < points.size(); ++i) {
        items.push_back({static_cast<std::uint32_t>(i), *core::to_wgs84(origin, points[i]),
                         core::AltitudeReference::relative_to_home, std::nullopt,
                         core::MetersPerSecond{options.speed}});
        waypoints << i << ',' << points[i].east.value << ',' << points[i].north.value << '\n';
    }
    waypoints.close();
    mission::RouteFollower follower(items, {});
    state::SimulatedStateProvider provider;
    state::StateValidator validator({100000, 100000, core::Meters{1.0},
                                     core::GpsFix::three_dimensional});
    core::LocalPosition position{{0}, {options.offset}};
    core::NorthEastVelocity velocity{{0}, {0}};
    std::mt19937 random(options.seed);
    const auto noise = [&]() {
        // Explicit integer-to-double mapping avoids library-specific real distributions.
        return options.noise * (2.0 * static_cast<double>(random()) / std::mt19937::max() - 1.0);
    };
    std::optional<core::StateSample> old_sample;
    double max_cross = 0, max_waypoint_distance = 0, seconds = 0;
    unsigned accepted = 0, rejected = 0, missing = 0;
    bool complete = false, recovered = options.fault == Fault::none;
    auto csv = file(output / "route.csv");
    csv << "time_s,east_m,north_m,velocity_east_mps,velocity_north_mps,"
           "measured_east_m,measured_north_m,measurement_age_s,active_segment,"
           "completed_segments,true_cross_track_m,distance_to_active_goal_m,"
           "command_east_mps,command_north_mps,model_target_east_mps,model_target_north_mps,"
           "command_available,correction_applied,status,state_reason\n";
    for (unsigned i = 0; i <= max_steps; ++i) {
        seconds = i * dt;
        const core::MonotonicMicroseconds now{static_cast<std::uint64_t>(i) * 50000};
        const core::LocalPosition measured{{position.east.value + noise()},
                                           {position.north.value + noise()}};
        const auto gps = core::to_wgs84(origin, measured);
        if (!gps) throw std::runtime_error("route model left local frame");
        core::StateSample sample{now, now, core::StateSource::simulation, *gps,
            std::nullopt, velocity, std::nullopt, core::Meters{std::sqrt(2.0) * options.noise},
            core::GpsFix::three_dimensional};
        if (i == 70) old_sample = sample;  // Observation from t=3.5 s.
        const bool fault = i >= 80 && i < 120;  // [4, 6) seconds, exactly 40 cycles.
        if (fault && options.fault == Fault::stale) {
            sample = *old_sample;
            sample.received_at = now;
        }
        if (fault && options.fault == Fault::no_fix) sample.gps_fix = core::GpsFix::no_fix;
        if (!(fault && options.fault == Fault::missing)) provider.publish(sample);
        const auto observation = provider.take_latest();
        std::string reason = "missing", status = "state_missing";
        std::optional<core::SegmentStep> step;
        const auto completed_before = follower.completed_segments();
        if (!observation) ++missing;
        else {
            const auto validated = validator.validate(*observation, now);
            if (const auto* valid = std::get_if<core::StateSample>(&validated)) {
                ++accepted;
                reason = "accepted";
                step = follower.update(*valid);
                switch (step->guidance.status) {
                    case core::ModuleStatus::mission_completed: status = "mission_completed"; break;
                    case core::ModuleStatus::corrected_target: status = "corrected"; break;
                    case core::ModuleStatus::moving_without_correction: status = "on_course"; break;
                    default: throw std::runtime_error("route guidance rejected scenario");
                }
            } else {
                ++rejected;
                const auto error = std::get<state::StateRejectReason>(validated);
                reason = reject_reason(error);
                status = error == state::StateRejectReason::MeasurementStale ? "stale_state" : "invalid_state";
            }
        }
        const bool available = step && step->guidance.target.has_value();
        // No target is emitted on a missing/rejected sample. Only this toy
        // executor substitutes zero velocity; it still experiences drift.
        core::NorthEastVelocity drive{{0}, {0}};
        if (available) drive = std::get<core::NorthEastVelocity>(*step->guidance.target);
        if (i >= 120 && available) recovered = true;
        for (std::size_t reached = completed_before + 1;
             reached <= follower.completed_segments(); ++reached) {
            max_waypoint_distance = std::max(max_waypoint_distance,
                std::hypot(position.east.value - points[reached].east.value,
                           position.north.value - points[reached].north.value));
        }
        const auto goal = std::min(follower.completed_segments() + 1, points.size() - 1);
        const double de = points[goal].east.value - points[goal - 1].east.value;
        const double dn = points[goal].north.value - points[goal - 1].north.value;
        // Ground-truth diagnostics only, never fed into guidance.
        const double cross = (de * (position.north.value - points[goal - 1].north.value) -
                              dn * (position.east.value - points[goal - 1].east.value)) / std::hypot(de, dn);
        max_cross = std::max(max_cross, std::abs(cross));
        const double distance = std::hypot(position.east.value - points[goal].east.value,
                                          position.north.value - points[goal].north.value);
        complete = step && step->guidance.status == core::ModuleStatus::mission_completed;
        csv << seconds << ',' << position.east.value << ',' << position.north.value
            << ',' << velocity.east.value << ',' << velocity.north.value << ',';
        if (observation) {
            const auto observed = core::to_local(origin, observation->position);
            csv << observed->east.value << ',' << observed->north.value << ','
                << (now.value - observation->measured_at.value) / 1e6;
        } else csv << ",,";
        csv << ',';
        if (const auto segment = follower.active_segment()) csv << *segment;
        csv << ',' << follower.completed_segments() << ',' << cross << ',' << distance << ',';
        if (available) csv << drive.east.value << ',' << drive.north.value;
        else csv << ',';
        csv << ',' << drive.east.value << ',' << drive.north.value << ',' << available << ','
            << (available && step->guidance.correction_required) << ',' << status << ',' << reason << '\n';
        if (complete || i == max_steps) break;
        const double alpha = 1.0 - std::exp(-dt / response);
        velocity.east.value += alpha * (drive.east.value - velocity.east.value);
        velocity.north.value += alpha * (drive.north.value + options.wind - velocity.north.value);
        position.east.value += velocity.east.value * dt;
        position.north.value += velocity.north.value * dt;
    }
    csv.close();
    const double final_distance = std::hypot(position.east.value - points.back().east.value,
                                             position.north.value - points.back().north.value);
    const bool fault_counts_ok = options.fault == Fault::none ? rejected == 0 && missing == 0 :
        options.fault == Fault::missing ? missing == 40 && rejected == 0 : rejected == 40 && missing == 0;
    const bool passed = complete && max_waypoint_distance <= 4.0 &&
        max_cross <= std::max(8.0, std::abs(options.offset) + 2.0) && recovered && fault_counts_ok;
    auto summary = file(output / "summary.csv");
    const std::string header = "completed,completed_segments,time_s,max_cross_track_m,"
        "max_waypoint_distance_m,final_distance_m,accepted_samples,rejected_samples,missing_samples,recovered\n";
    const auto row = [&](std::ostream& out) {
        out << complete << ',' << follower.completed_segments() << ',' << seconds << ',' << max_cross
            << ',' << max_waypoint_distance << ',' << final_distance << ',' << accepted << ','
            << rejected << ',' << missing << ',' << recovered << '\n';
    };
    summary << header;
    row(summary);
    summary.close();
    std::cout << std::setprecision(6) << header;
    row(std::cout);
    std::cout << "scenario=route speed=" << options.speed << " offset=" << options.offset
              << " wind=" << options.wind << " noise=" << options.noise << " seed=" << options.seed
              << " fault=" << static_cast<int>(options.fault) << " result=" << (passed ? "PASS" : "FAIL") << '\n';
    return passed ? 0 : 1;
}
}  // namespace kpi4::demo
