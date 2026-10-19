// ugv02_protocol.hpp — everything that depends on the UGV02 JSON format lives HERE (and only here).
//
// Boilerplate on purpose (protocol plumbing, written by Claude). The ROS logic stays in the bridge.
// Why one file: the feedback format is an ASSUMPTION (docs/ugv02-protocol.md). On day 1 with the real
// robot we record the real lines; if they differ, only parse_feedback() changes.
//
//   std::string cmd = scr_ugv02::make_enable_feedback();        // {"T":131,"cmd":1}
//   scr_ugv02::Feedback fb;
//   if (scr_ugv02::parse_feedback(line, fb)) { fb.left, fb.right, fb.voltage ... }
#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace scr_ugv02
{

// One T:1001 line from the chassis. Units are [가정] until measured on the real robot.
struct Feedback
{
  double left{0.0};      // left wheel speed, m/s        [가정]
  double right{0.0};     // right wheel speed, m/s       [가정]
  double roll{0.0};      // IMU roll  (unit unknown)     [가정]
  double pitch{0.0};     // IMU pitch (unit unknown)     [가정]
  double voltage{0.0};   // battery voltage, V           [근거: Waveshare app]
};

// Ask the chassis to stream T:1001 continuously (default is OFF). Send once after opening the port.
inline std::string make_enable_feedback()
{
  return R"({"T":131,"cmd":1})";
}

// Parse one line. Returns false for anything that is not a well-formed T:1001 line
// (other message types, half lines, garbage) -> the caller just skips it.
inline bool parse_feedback(const std::string & line, Feedback & out)
{
  const auto j = nlohmann::json::parse(line, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object()) {
    return false;
  }
  Feedback f;   // fill a temporary first, so `out` is untouched when parsing fails halfway
  try {
    if (j.value("T", 0) != 1001) {
      return false;
    }
    f.left = j.value("L", 0.0);
    f.right = j.value("R", 0.0);
    f.roll = j.value("r", 0.0);
    f.pitch = j.value("p", 0.0);
    f.voltage = j.value("v", 0.0);
  } catch (const nlohmann::json::exception &) {   // a field had the wrong type, e.g. "L":"abc"
    return false;
  }
  out = f;
  return true;
}

}  // namespace scr_ugv02
