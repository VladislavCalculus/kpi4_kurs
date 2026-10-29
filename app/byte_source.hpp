#pragma once

#include <cstddef>
#include <cstdint>
#include <istream>

namespace kpi4::host {

class StreamByteSource {
public:
    explicit StreamByteSource(std::istream& stream);

    std::size_t read(std::uint8_t* destination, std::size_t capacity);
    bool failed() const;

private:
    std::istream& stream_;
};

}  // namespace kpi4::host
