#include <kpi4/config/host_config.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace kpi4::config {
namespace {

using Fields = std::map<std::string, std::string>;

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

bool digits_only(const std::string& text) {
    return !text.empty() && std::all_of(text.begin(), text.end(),
        [](unsigned char c) { return std::isdigit(c) != 0; });
}

std::optional<std::uint64_t> positive_microseconds(const std::string& text) {
    if (!digits_only(text)) return std::nullopt;
    try {
        const auto value = std::stoull(text);
        if (value > 0) return value;
    } catch (const std::exception&) {}
    return std::nullopt;
}

std::optional<core::Meters> accuracy(const std::string& text) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size() && std::isfinite(value) && value >= 0.0)
            return core::Meters{value};
    } catch (const std::exception&) {}
    return std::nullopt;
}

bool known_field(const std::string& key) {
    static const std::array<std::string, 6> names{
        "source", "reading_type", "max_measurement_age_us",
        "max_delivery_delay_us", "max_horizontal_accuracy_m",
        "minimum_gps_fix"};
    return std::find(names.begin(), names.end(), key) != names.end();
}

std::optional<core::StateSource> parse_source(const std::string& text) {
    if (text == "controller_telemetry") return core::StateSource::controller_telemetry;
    if (text == "direct_gnss") return core::StateSource::direct_gnss;
    if (text == "ground_station") return core::StateSource::ground_station;
    if (text == "simulation") return core::StateSource::simulation;
    return std::nullopt;
}

std::optional<ReadingType> parse_reading_type(const std::string& text) {
    if (text == "position_estimate") return ReadingType::position_estimate;
    if (text == "gnss_fix") return ReadingType::gnss_fix;
    if (text == "simulated_sample") return ReadingType::simulated_sample;
    return std::nullopt;
}

}  // namespace

ConfigLoadResult HostConfigStore::load(std::istream& input, std::string origin) {
    if (!input) return ConfigFailure{ConfigError::InputUnavailable, std::move(origin)};

    Fields fields;
    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line.front() == '#') continue;
        const auto equals = line.find('=');
        if (equals == std::string::npos)
            return ConfigFailure{ConfigError::InvalidLine, line};
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (key.empty() || value.empty() || value.find('=') != std::string::npos)
            return ConfigFailure{ConfigError::InvalidLine, key};
        if (!known_field(key)) return ConfigFailure{ConfigError::UnknownField, key};
        if (!fields.emplace(key, value).second)
            return ConfigFailure{ConfigError::DuplicateField, key};
    }
    if (input.bad()) return ConfigFailure{ConfigError::InputUnavailable, std::move(origin)};
    static const std::array<std::string, 6> required{
        "source", "reading_type", "max_measurement_age_us",
        "max_delivery_delay_us", "max_horizontal_accuracy_m",
        "minimum_gps_fix"};
    for (const auto& key : required) {
        if (!fields.count(key)) return ConfigFailure{ConfigError::MissingField, key};
    }

    const auto source = parse_source(fields.at("source"));
    if (!source) return ConfigFailure{ConfigError::InvalidValue, "source"};
    const auto reading = parse_reading_type(fields.at("reading_type"));
    if (!reading) return ConfigFailure{ConfigError::InvalidValue, "reading_type"};
    const auto age = positive_microseconds(fields.at("max_measurement_age_us"));
    if (!age) return ConfigFailure{ConfigError::InvalidValue, "max_measurement_age_us"};
    const auto delay = positive_microseconds(fields.at("max_delivery_delay_us"));
    if (!delay) return ConfigFailure{ConfigError::InvalidValue, "max_delivery_delay_us"};
    if (*delay > *age)
        return ConfigFailure{ConfigError::IncompatibleFields, "max_delivery_delay_us"};

    std::optional<core::Meters> max_accuracy;
    const auto& accuracy_text = fields.at("max_horizontal_accuracy_m");
    if (accuracy_text != "none") {
        max_accuracy = accuracy(accuracy_text);
        if (!max_accuracy)
            return ConfigFailure{ConfigError::InvalidValue, "max_horizontal_accuracy_m"};
    }
    std::optional<core::GpsFix> minimum_fix;
    const auto& fix_text = fields.at("minimum_gps_fix");
    if (fix_text == "2d") minimum_fix = core::GpsFix::two_dimensional;
    else if (fix_text == "3d") minimum_fix = core::GpsFix::three_dimensional;
    else if (fix_text != "none")
        return ConfigFailure{ConfigError::InvalidValue, "minimum_gps_fix"};

    if ((*source == core::StateSource::simulation) !=
            (*reading == ReadingType::simulated_sample) ||
        (*source == core::StateSource::direct_gnss &&
            *reading != ReadingType::gnss_fix) ||
        (*reading == ReadingType::position_estimate &&
            (max_accuracy || minimum_fix)) ||
        (*reading == ReadingType::gnss_fix && !minimum_fix)) {
        return ConfigFailure{ConfigError::IncompatibleFields, "reading_type"};
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_ && current_->version == std::numeric_limits<std::uint64_t>::max())
        return ConfigFailure{ConfigError::VersionExhausted, "version"};

    auto snapshot = std::make_shared<const ConfigSnapshot>(ConfigSnapshot{
        current_ ? current_->version + 1 : 1, std::move(origin),
        *source, *reading, {*age, *delay, max_accuracy, minimum_fix}});
    current_ = snapshot;
    return snapshot;
}

std::shared_ptr<const ConfigSnapshot> HostConfigStore::current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
}

}  // namespace kpi4::config
