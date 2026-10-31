// ugv02_protocol.hpp — everything that depends on the UGV02 JSON format lives HERE (and only here).
//
// Boilerplate on purpose (protocol plumbing, written by Claude). The ROS logic stays in the bridge.
// v3 (2026-10-31): format and units MEASURED on the real robot → docs/ugv02-protocol.md "Real feedback",
//                  docs/calibration/odom-2026-10-31.md
//   {"T":1001,"L":0,"R":0,"ax":12,"ay":-248,"az":8568,"gx":14,"gy":1,"gz":-5,"mx":16,"my":-75,"mz":549,"odl":0,"odr":0,"v":1122}
//
//   std::string cmd = scr_ugv02::make_enable_feedback();        // {"T":131,"cmd":1}
//   scr_ugv02::Feedback fb;
//   if (scr_ugv02::parse_feedback(line, fb)) { fb.odl_m, fb.gz, fb.voltage ... }
#pragma once

#include <cmath>
#include <string>

#include <nlohmann/json.hpp>

namespace scr_ugv02
{

// ---- unit conversions (measured 10-31) ----
constexpr double kOdoMetersPerUnit = 0.01;    // odl/odr: 1 unit = 1 cm   (60 cm by tape → 61, 64)
constexpr double kVoltsPerUnit = 0.01;        // v: centivolts            (1150 → 11.50 V)
constexpr double kGyroUnitsPerDegS = 15.65;   // gx/gy/gz: 15.65 units = 1 °/s. Datasheet ±2000 °/s says 16.4, but on THIS board
                                              // bridge v3 vs lidar truth (10-31): +184.3/+175.4, +88.6/+83.0, −91.1/−88.7
                                              // → always ~4.8% short in both directions = scale, not bias. 16.4 / 1.048 = 15.65
                                              // → residual +0.3 %, +1.8 %, −2.0 % (= lidar truth precision ±1-2°)

// Raw gyro units (bias already removed!) → rad/s.  + = counter-clockwise for gz.
inline double gyro_to_rad_s(double raw_minus_bias)
{
  return raw_minus_bias / kGyroUnitsPerDegS * M_PI / 180.0;
}

// One T:1001 line from the chassis, already in SI units where the unit is known.
struct Feedback
{
  double left{0.0};     // left wheel speed, m/s                    [확인: ∫L dt ≈ tape, +4%]
  double right{0.0};    // right wheel speed, m/s                   [확인]
  double odl_m{0.0};    // left wheel odometer, m (cumulative, 1 cm steps)   [확인]
  double odr_m{0.0};    // right wheel odometer, m                  [확인]
  int ax{0}, ay{0}, az{0};   // accelerometer, raw (az ≈ 8500 at rest ≈ 1 g)  [scale 가정]
  int gx{0}, gy{0}, gz{0};   // gyro, raw — use gyro_to_rad_s(gz - bias)      [확인 ±5%]
  double voltage{0.0};  // battery, V                               [확인]
};

// Ask the chassis to stream T:1001 continuously. (Real board: already ON by default; harmless to send.)
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
    f.odl_m = j.value("odl", 0.0) * kOdoMetersPerUnit;
    f.odr_m = j.value("odr", 0.0) * kOdoMetersPerUnit;
    f.ax = j.value("ax", 0); f.ay = j.value("ay", 0); f.az = j.value("az", 0);
    f.gx = j.value("gx", 0); f.gy = j.value("gy", 0); f.gz = j.value("gz", 0);
    f.voltage = j.value("v", 0.0) * kVoltsPerUnit;
  } catch (const nlohmann::json::exception &) {   // a field had the wrong type, e.g. "L":"abc"
    return false;
  }
  out = f;
  return true;
}

}  // namespace scr_ugv02
