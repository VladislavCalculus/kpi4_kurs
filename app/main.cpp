#include "byte_source.hpp"

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

constexpr std::size_t default_chunk_size = 4096;
constexpr std::size_t maximum_chunk_size = 65536;

void print_usage(std::ostream& output) {
    output << "Usage: kpi4_cli --input FILE|- [--chunk-size N]\n"
              "Read binary bytes without decoding MAVLink. N is a 1..65536 byte maximum.\n";
}

bool parse_chunk_size(std::string_view text, std::size_t& size) {
    const auto* end = text.data() + text.size();
    const auto result = std::from_chars(text.data(), end, size);
    return result.ec == std::errc{} && result.ptr == end && size > 0 &&
           size <= maximum_chunk_size;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string_view{argv[1]} == "--help") {
        print_usage(std::cout);
        return 0;
    }

    if ((argc != 3 && argc != 5) || std::string_view{argv[1]} != "--input") {
        print_usage(std::cerr);
        return 2;
    }

    std::size_t chunk_size = default_chunk_size;
    if (argc == 5 &&
        (std::string_view{argv[3]} != "--chunk-size" ||
         !parse_chunk_size(argv[4], chunk_size))) {
        print_usage(std::cerr);
        return 2;
    }

    std::ifstream file;
    std::istream* input = &std::cin;
    if (std::string_view{argv[2]} != "-") {
        file.open(argv[2], std::ios::binary);
        if (!file) {
            std::cerr << "Cannot open input file: " << argv[2] << '\n';
            return 1;
        }
        input = &file;
    }

    kpi4::host::StreamByteSource source(*input);
    std::vector<std::uint8_t> buffer(chunk_size);
    std::size_t total_bytes = 0;
    std::size_t chunks = 0;
    while (true) {
        const auto count = source.read(buffer.data(), buffer.size());
        if (count == 0) {
            break;
        }
        if (count > std::numeric_limits<std::size_t>::max() - total_bytes) {
            std::cerr << "Input byte count overflow\n";
            return 1;
        }
        total_bytes += count;
        ++chunks;
    }

    if (source.failed()) {
        std::cerr << "Input read failed\n";
        return 1;
    }

    std::cout << "bytes=" << total_bytes << " chunks=" << chunks << '\n';
    return 0;
}
