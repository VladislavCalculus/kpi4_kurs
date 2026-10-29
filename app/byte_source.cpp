#include "byte_source.hpp"

namespace kpi4::host {

StreamByteSource::StreamByteSource(std::istream& stream) : stream_(stream) {}

std::size_t StreamByteSource::read(std::uint8_t* destination, std::size_t capacity) {
    if (capacity == 0) {
        return 0;
    }

    const auto first = stream_.get();
    if (first == std::char_traits<char>::eof()) {
        return 0;
    }
    destination[0] = static_cast<std::uint8_t>(first);
    const auto available = stream_.readsome(
        reinterpret_cast<char*>(destination + 1), static_cast<std::streamsize>(capacity - 1));
    return 1 + static_cast<std::size_t>(available);
}

bool StreamByteSource::failed() const {
    return stream_.bad();
}

}  // namespace kpi4::host
