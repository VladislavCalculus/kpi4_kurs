#pragma once

#include <cstdint>
#include <filesystem>

namespace kpi4::demo {
enum class Fault { none, stale, missing, no_fix };
struct RouteOptions {
    double wind{2.0};
    double speed{8.0};
    double offset{0.0};
    double noise{0.0};
    std::uint32_t seed{42};
    Fault fault{Fault::none};
};
int run_route(const std::filesystem::path& output, const RouteOptions& options);
}  // namespace kpi4::demo
