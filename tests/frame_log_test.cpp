#include "frame_log.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require_equal(const std::string& actual, const std::string& expected) {
    if (actual != expected) {
        throw std::runtime_error("frame metadata differs:\n" + actual +
                                 "expected:\n" + expected);
    }
}

}  // namespace

int main() {
    const auto received_at = std::chrono::system_clock::time_point{} +
                             std::chrono::milliseconds{123};
    mavlink_message_t frame{};
    frame.magic = MAVLINK_STX_MAVLINK1;
    frame.sysid = 1;
    frame.compid = 2;
    frame.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    frame.seq = 7;
    frame.len = 9;

    std::ostringstream output;
    kpi4::host::write_frame_metadata(output, frame, received_at);
    require_equal(output.str(),
                  "frame time=1970-01-01T00:00:00.123Z mavlink=1 sysid=1 "
                  "compid=2 msgid=0 seq=7 len=9 signed=no\n");

    frame.magic = MAVLINK_STX;
    frame.incompat_flags = MAVLINK_IFLAG_SIGNED;
    frame.sysid = 42;
    frame.compid = 255;
    frame.msgid = 300;
    frame.seq = 254;
    frame.len = 13;
    output.str("");
    output.clear();
    kpi4::host::write_frame_metadata(output, frame, received_at);
    require_equal(output.str(),
                  "frame time=1970-01-01T00:00:00.123Z mavlink=2 sysid=42 "
                  "compid=255 msgid=300 seq=254 len=13 signed=yes\n");
}
