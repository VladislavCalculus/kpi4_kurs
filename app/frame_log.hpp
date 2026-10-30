#pragma once

#include <common/mavlink.h>

#include <chrono>
#include <iosfwd>

namespace kpi4::host {

void write_frame_metadata(std::ostream& output, const mavlink_message_t& frame,
                          std::chrono::system_clock::time_point received_at);

}  // namespace kpi4::host
