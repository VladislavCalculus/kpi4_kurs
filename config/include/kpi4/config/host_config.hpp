#pragma once

#include <kpi4/state/validator.hpp>

#include <cstdint>
#include <istream>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

namespace kpi4::config {

// Describes the meaning of a state reading, not its transport or wire format.
enum class ReadingType {
    position_estimate,
    gnss_fix,
    simulated_sample,
};

struct ConfigSnapshot {
    std::uint64_t version;
    std::string origin;
    core::StateSource source;
    ReadingType reading_type;
    state::StateValidationConfig state_limits;
};

enum class ConfigError {
    InputUnavailable,
    InvalidLine,
    UnknownField,
    DuplicateField,
    MissingField,
    InvalidValue,
    IncompatibleFields,
    VersionExhausted,
};

struct ConfigFailure {
    ConfigError error;
    std::string field;
};

using ConfigLoadResult =
    std::variant<std::shared_ptr<const ConfigSnapshot>, ConfigFailure>;

class HostConfigStore {
public:
    // Parse and validate the whole host file before publishing a new snapshot.
    ConfigLoadResult load(std::istream& input, std::string origin);
    std::shared_ptr<const ConfigSnapshot> current() const;

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const ConfigSnapshot> current_;
};

}  // namespace kpi4::config
