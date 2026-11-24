#include <kpi4/core/guidance.hpp>
#include <kpi4/state/simulated_provider.hpp>
#include <kpi4/state/validator.hpp>
#include "route_demo.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace kpi4;
constexpr double dt = 0.05;
constexpr double response_seconds = 0.5;
constexpr double segment_length = 100.0;
constexpr double corridor = 5.0;
constexpr unsigned max_steps = 600;

struct Metrics {
    bool reached{false};
    bool crossed_end{false};
    double seconds{0};
    double max_cross{0};
    double final_distance{0};
};

std::ofstream output_file(const std::filesystem::path& path) {
    std::ofstream out(path);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << std::setprecision(12);
    return out;
}

Metrics run(const std::filesystem::path& file, bool corrected, double drift) {
    const core::Wgs84Position origin{{50.0}, {30.0}};
    const core::GuidanceConfig config{};
    const core::MissionItem start{0, origin, core::AltitudeReference::relative_to_home,
                                  std::nullopt, std::nullopt};
    const core::MissionItem finish{1,
        *core::to_wgs84(origin, {{segment_length}, {0.0}}),
        core::AltitudeReference::relative_to_home, std::nullopt, core::MetersPerSecond{8.0}};
    state::SimulatedStateProvider provider;
    state::StateValidator validator({100000, 100000, core::Meters{1.0},
                                     core::GpsFix::three_dimensional});
    core::LocalPosition position{{0.0}, {0.0}};
    core::NorthEastVelocity velocity{{0.0}, {0.0}};
    Metrics metrics;
    auto csv = output_file(file);
    csv << "time_s,east_m,north_m,velocity_east_mps,velocity_north_mps,"
           "cross_track_m,distance_to_end_m,command_east_mps,command_north_mps,"
           "correction_applied,status\n";
    for (unsigned i = 0; i <= max_steps; ++i) {
        const auto position_wgs = core::to_wgs84(origin, position);
        if (!position_wgs) throw std::runtime_error("model left local frame");
        const core::MonotonicMicroseconds now{static_cast<std::uint64_t>(i) * 50000};
        provider.publish({now, now, core::StateSource::simulation, *position_wgs,
                          std::nullopt, velocity, std::nullopt, core::Meters{0.0},
                          core::GpsFix::three_dimensional});
        const auto reading = provider.take_latest();
        if (!reading) throw std::runtime_error("missing simulated sample");
        const auto validated = validator.validate(*reading, now);
        const auto* sample = std::get_if<core::StateSample>(&validated);
        if (!sample) throw std::runtime_error("simulated sample rejected");
        const auto step = core::follow_segment(start, finish, *sample, config);
        if (!step.geometry) throw std::runtime_error("guidance rejected scenario");
        metrics.seconds = i * dt;
        metrics.max_cross = std::max(metrics.max_cross, std::abs(step.geometry->cross_track_error.value));
        metrics.final_distance = step.geometry->distance_to_end.value;
        metrics.reached = step.guidance.status == core::ModuleStatus::target_reached;
        metrics.crossed_end = step.geometry->progress >= 1.0;
        const bool done = metrics.reached || metrics.crossed_end || i == max_steps;
        core::NorthEastVelocity command{{0.0}, {0.0}};
        bool applied = false;
        if (!done) {
            if (corrected) {
                if (!step.guidance.target) throw std::runtime_error("missing guidance target");
                command = std::get<core::NorthEastVelocity>(*step.guidance.target);
                applied = step.guidance.correction_required;
            } else {
                command.east.value = config.cruise_speed.value;
            }
        }
        const char* status = done ? (metrics.reached ? "reached" :
                                     metrics.crossed_end ? "crossed_end" : "timeout") :
                             !corrected ? "open_loop" : applied ? "corrected" : "on_course";
        csv << metrics.seconds << ',' << position.east.value << ',' << position.north.value
            << ',' << velocity.east.value << ',' << velocity.north.value
            << ',' << step.geometry->cross_track_error.value << ',' << metrics.final_distance
            << ',' << command.east.value << ',' << command.north.value
            << ',' << applied << ',' << status << '\n';
        if (done) break;
        // Stabilized executor with first-order response and a constant lateral
        // velocity bias. This is a disturbance model, not aircraft aerodynamics.
        const double alpha = 1.0 - std::exp(-dt / response_seconds);
        velocity.east.value += alpha * (command.east.value - velocity.east.value);
        velocity.north.value += alpha * (command.north.value + drift - velocity.north.value);
        position.east.value += velocity.east.value * dt;
        position.north.value += velocity.north.value * dt;
    }
    csv.close();
    return metrics;
}

void metric_row(std::ostream& out, const char* name, const Metrics& m) {
    out << name << ',' << m.reached << ',' << m.crossed_end << ',' << m.seconds
        << ',' << m.max_cross << ',' << m.final_distance << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path output{"demo-output"};
        demo::RouteOptions options;
        bool route = false, route_options = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "Usage: kpi4_demo [--output DIRECTORY] [--wind NORTH_MPS]\n"
                             "       [--scenario single|route] [--speed MPS] [--offset NORTH_M]\n"
                             "       [--noise METRES] [--seed UINT32] [--fault none|stale|missing|no_fix]\n"
                             "Speed, offset, noise, seed and fault apply to route only.\n";
                return 0;
            }
            if (i + 1 == argc) throw std::runtime_error("missing option value");
            const std::string value = argv[++i];
            if (arg == "--output") output = value;
            else if (arg == "--scenario") {
                if (value != "single" && value != "route") throw std::runtime_error("unknown scenario");
                route = value == "route";
            } else if (arg == "--fault") {
                route_options = true;
                if (value == "none") options.fault = demo::Fault::none;
                else if (value == "stale") options.fault = demo::Fault::stale;
                else if (value == "missing") options.fault = demo::Fault::missing;
                else if (value == "no_fix") options.fault = demo::Fault::no_fix;
                else throw std::runtime_error("unknown fault");
            } else if (arg == "--seed") {
                route_options = true;
                if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                    throw std::runtime_error("seed must be UINT32");
                const auto seed = std::stoull(value);
                if (seed > UINT32_MAX) throw std::runtime_error("seed exceeds UINT32");
                options.seed = static_cast<std::uint32_t>(seed);
            } else if (arg == "--wind" || arg == "--speed" || arg == "--offset" || arg == "--noise") {
                std::size_t consumed = 0;
                const double number = std::stod(value, &consumed);
                if (consumed != value.size() || !std::isfinite(number)) throw std::runtime_error("invalid number");
                if (arg == "--wind") options.wind = number;
                else {
                    route_options = true;
                    if (arg == "--speed") options.speed = number;
                    else if (arg == "--offset") options.offset = number;
                    else options.noise = number;
                }
            } else throw std::runtime_error("unknown option: " + arg);
        }
        if (std::abs(options.wind) > 5 || options.speed < 4 || options.speed > 12 ||
            std::abs(options.offset) > 20 || options.noise < 0 || options.noise > 0.5)
            throw std::runtime_error("limits: wind +/-5, speed [4,12], offset +/-20, noise [0,0.5]");
        if (!route && route_options) throw std::runtime_error("route options require --scenario route");
        std::filesystem::create_directories(output);
        if (route) return demo::run_route(output, options);
        const auto baseline = run(output / "baseline.csv", false, options.wind);
        const auto corrected = run(output / "corrected.csv", true, options.wind);
        auto summary = output_file(output / "summary.csv");
        const char* header = "mode,reached,crossed_end,time_s,max_cross_track_m,final_distance_m\n";
        summary << header;
        metric_row(summary, "baseline", baseline);
        metric_row(summary, "corrected", corrected);
        summary.close();
        std::cout << std::setprecision(6) << header;
        metric_row(std::cout, "baseline", baseline);
        metric_row(std::cout, "corrected", corrected);
        const bool improved = std::abs(options.wind) < 0.1 ||
            (corrected.max_cross < baseline.max_cross && corrected.final_distance < baseline.final_distance);
        const bool passed = corrected.reached && corrected.max_cross <= corridor && improved;
        std::cout << "scenario: length=100m speed=8m/s max_command=12m/s wind=" << options.wind
                  << "m/s dt=0.05s response=0.5s reach=3m corridor=5m seed=none\n"
                  << "result=" << (passed ? "PASS" : "FAIL") << '\n';
        return passed ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "demo: " << e.what() << '\n';
        return 2;
    }
}
