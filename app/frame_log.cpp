#include "frame_log.hpp"

#include <ctime>
#include <iomanip>
#include <ostream>

namespace kpi4::host {

void write_frame_metadata(std::ostream& output, const mavlink_message_t& frame,
                          std::chrono::system_clock::time_point received_at) {
    const auto whole_seconds = std::chrono::time_point_cast<std::chrono::seconds>(
        received_at);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        received_at - whole_seconds).count();
    const std::time_t epoch_seconds = std::chrono::system_clock::to_time_t(
        whole_seconds);
    const std::tm utc = *std::gmtime(&epoch_seconds);
    const auto previous_fill = output.fill();

    output << "frame time=" << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << milliseconds << 'Z';
    output.fill(previous_fill);
    output << " mavlink=" << (frame.magic == MAVLINK_STX_MAVLINK1 ? 1 : 2)
           << " sysid=" << frame.sysid
           << " compid=" << static_cast<unsigned>(frame.compid)
           << " msgid=" << frame.msgid
           << " seq=" << static_cast<unsigned>(frame.seq)
           << " len=" << static_cast<unsigned>(frame.len)
           << " signed=" << ((frame.magic == MAVLINK_STX &&
                                (frame.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0)
                                   ? "yes" : "no")
           << '\n';
}

}  // namespace kpi4::host
