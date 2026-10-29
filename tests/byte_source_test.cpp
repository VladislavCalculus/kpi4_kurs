#include "byte_source.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

int main() {
    const std::string bytes{'\0', static_cast<char>(0xFD), '\0',
                            static_cast<char>(0xFF), '\n'};
    std::istringstream input(bytes, std::ios::in | std::ios::binary);
    kpi4::host::StreamByteSource source(input);
    std::array<std::uint8_t, 2> buffer{};
    std::vector<std::uint8_t> result;
    std::vector<std::size_t> chunk_sizes;

    while (true) {
        const auto count = source.read(buffer.data(), buffer.size());
        if (count == 0) {
            break;
        }
        chunk_sizes.push_back(count);
        result.insert(result.end(), buffer.begin(), buffer.begin() + count);
    }

    const std::vector<std::uint8_t> expected(bytes.begin(), bytes.end());
    return source.failed() || result != expected ||
                   chunk_sizes != std::vector<std::size_t>{2, 2, 1};
}
