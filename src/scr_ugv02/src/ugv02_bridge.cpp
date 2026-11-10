// ugv02_bridge — translator between ROS 2 and the UGV02 base (JSON lines over a serial cable).
// Written by Jasper (logic). Boilerplate by Claude: serial_port.hpp, ugv02_protocol.hpp.
//
//   down (commands):  /cmd_vel (TwistStamped) ──▶ {"T":1,"L":..,"R":..}\n ──▶ serial ──▶ UGV02 ESP32
//   up   (reports):   /odom + TF odom→base_footprint + /battery_state ◀── {"T":1001,"L":..,"R":..,"v":..}\n ◀── ESP32
//
// v0  /cmd_vel → T:13 JSON.
// v2  safety: dead-man (no /cmd_vel for cmd_timeout 0.5 s → stop once) + stop on shutdown.
//     Limit: if the bridge itself is killed hard (kill -9 / crash) only the firmware 3 s timeout remains
//     → defense in depth (teleop 0.5 s → bridge 0.5 s → firmware 3 s).
//     11-03 real test: the "firmware 3 s" did NOT exist for T:13 — one T:13 kept the wheels turning 6 s+
//     (10-31 living room: bridge died → robot kept going). Firmware source: only T:1/T:11 refresh its heartbeat.
// v4  drive with T:1 (L/R wheel speeds) + set firmware heartbeat T:136 = 500 ms at start
//     → ESP32 stops by itself 0.5 s after the bridge goes silent (crash, hang, Pi off). Tested 11-03: stopped < 0.8 s.
// v5  drive_mode "pwm": WE control the motors (T:11 PWM) instead of the firmware speed PID — for slow, smooth crawling.
//     Why: firmware PID measures wheel speed over very short periods → at low speed it reads mostly 0 → over-reacts → stop-go
//     (ax jitter ~450 at 0.03-0.10 m/s). Constant PWM rolls smoothly (PWM 30: 0.08 m/s, jitter 92). docs/calibration/lowspeed-2026-11-03.md
//     How: forward PWM from the measured straight table + turn PWM from the spin table, corrected by a slow GYRO loop (v5.1:
//     arcs drove straight without it); gentle start (raise PWM until the wheel moves, then drop); speeds below the floor are
//     snapped up (0.05 m/s straight, 0.13 rad/s spin). T:11 also refreshes the heartbeat (firmware source).
//     drive_mode "pid" = v4 behaviour (T:1, firmware PID).
// v5.2 TAP: asks slower than the floor (Nav2's final creep) → short pushes whose duty = asked / floor (11-04: hunting fix).
// v5.3 TAP decides from the AVERAGE ask of the last 0.5 s, not the latest one (11-05 explore: stuck 18 cm from home next to
//      a wall, MPPI asked x ±0.005 z ±0.02-0.06 flipping sign every tick → every flip became a real tap → back-and-forth
//      and head-shaking for 80 s). Flipping noise averages to ~0 → rest. A real slow ask keeps its sign → still taps.
// v8.1 FORCE LIMIT (11-06): PWM ≤ 40 straight / ≤ 55 turning (params push_cap_*) — blocked wheels no longer push harder.
// v8.2 ESCAPE FORCE (11-10): /whisker/force (Float32, PWM) raises the breakaway limit ONLY while whisker drives and keeps
//      sending it (0.3 s), never above kForceMax (70). Breakaway only: once the wheel turns, the push drops to the holding
//      PWM as before → a freed robot does not shoot off. Whisker quiet → normal limits again (Jasper 11-10: 탈출 모듈).
// v8.3 OLED (11-10): /robot/display (String, up to 4 lines joined by \n, from scripts/status_display.py) → only the lines
//      that changed go out as {"T":3,"lineNum":i,"Text":...}. The bridge is the only serial writer, so the display lives here.
// v8  Whisker hooks (11-06), the bridge stays thin — all touch logic lives in scripts/whisker.py (Python, no rebuild):
//     ① /motor_out [pwm_l, pwm_r, cmd_x, cmd_z, whisker] every feedback line → whisker's "efference copy" (our own jolts)
//     ② /whisker/cmd_vel has PRIORITY over /cmd_vel while it keeps coming (0.3 s): stop, back off, wait — Nav2 is ignored.
//        A zero command from whisker = stop NOW (no tap tail). When whisker goes quiet, Nav2 drives again.
// v6  /imu (11-06): raw IMU every feedback line → touch/bump/terrain sensing (scripts/imu_lab.py measures it).
// v1  feedback T:1001 → wheel odometry (dead reckoning) → /odom, TF, battery.
//     Known limit: dt is measured from ARRIVAL time on the Pi, so two lines read in the same tick get dt≈0
//     (distance under-counted). Encoder tick counts would avoid this — check if the real UGV02 offers them.
//
// Protocol: docs/ugv02-protocol.md      Tests & results: README.md
//
// How a ROS node class is laid out (3 zones):
//   1. constructor  = setup, runs ONCE  (parameters, open port, publishers/subscriptions/timers)
//   2. functions    = behavior          (callbacks run by the executor on events / timer ticks)
//   3. member vars  = the node's memory (shared between callbacks; safe because callbacks run one at a time)

#include <algorithm>  // std::min
#include <cmath>      // std::cos, std::sin, std::lround
#include <cstdio>     // std::snprintf
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>    // std::pair
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.hpp"       // yaw → quaternion
#include "tf2_ros/transform_broadcaster.hpp"   // sends the odom → base_footprint TF

#include "scr_ugv02/serial_port.hpp"           // open / write_line / read_lines (non-blocking, line framing)
#include "scr_ugv02/ugv02_protocol.hpp"        // make_enable_feedback(), parse_feedback() — only file that knows the JSON format

using namespace std::chrono_literals;

class Ugv02Bridge : public rclcpp::Node
{
public:
  // ===== zone 1: constructor (setup, runs once) =====
  Ugv02Bridge() : Node("ugv02_bridge") {
    port_ = declare_parameter<std::string>("port", "/tmp/ugv02_bridge");   // real robot: /dev/ttyAMA0
    baud_ = declare_parameter<int>("baud", 115200);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);           // s, dead-man limit
    track_width_ = declare_parameter<double>("track_width", 0.170);         // m, splits /cmd_vel into L/R wheel speeds (UGV02 measured 0.170)
    heartbeat_ms_ = declare_parameter<int>("heartbeat_ms", 500);             // firmware auto-stop if no T:1 for this long
    drive_mode_ = declare_parameter<std::string>("drive_mode", "pid");       // "pid" (T:1, firmware PID) | "pwm" (T:11, our control, v5)
    pwm_max_ = declare_parameter<double>("pwm_max", 100.0);                  // safety cap for PWM mode (80 ≈ 0.34 m/s)
    // v8.1 (11-06) FORCE LIMIT — "can't crush what it can't detect" (Jasper: a child's hand, a pet's paw). Blocked wheels used
    // to get PWM raised up to 100 by the break-away push (it pushed Jasper's hand). Crawling needs PWM 30 (0.08 m/s), a
    // break-away 35-40, an on-the-spot turn 40-50 → cap there. Too low = can't climb a rug edge → Whisker feels the stall.
    push_cap_straight_ = declare_parameter<double>("push_cap_straight", 40.0);
    push_cap_turn_ = declare_parameter<double>("push_cap_turn", 55.0);
    if (drive_mode_ != "pid" && drive_mode_ != "pwm") {
      throw std::runtime_error("drive_mode must be 'pid' or 'pwm'");
    }

    // A bridge without a port must not pretend to run → stop the program with a clear reason.
    if (!serial_.open(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "failed to open serial port %s at %d baud: %s",
        port_.c_str(), baud_, serial_.error().c_str());
      throw std::runtime_error("failed to open serial port");
    }

    // The chassis does NOT report by default. Ask it once to stream T:1001 feedback continuously.
    serial_.write_line(scr_ugv02::make_enable_feedback());

    // v4 safety: firmware heartbeat. If no T:1 speed command for heartbeat_ms, the ESP32 stops the wheels BY ITSELF
    // → covers what the bridge can't: bridge crash, bridge hang, Pi power loss. (Setting is lost on ESP32 reboot → send every start.)
    // Only T:1 refreshes the firmware's timer; T:13 does NOT on our firmware (11-03 test: T:13 kept running 6 s) → we drive with T:1.
    char hb[48];
    std::snprintf(hb, sizeof(hb), "{\"T\":136,\"cmd\":%d}", heartbeat_ms_);
    serial_.write_line(hb);
    RCLCPP_INFO(get_logger(), "firmware heartbeat %d ms (auto-stop if bridge goes silent)", heartbeat_ms_);

    // Outputs (up direction).
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 20);   // v6 (11-06): raw IMU for touch/bump sensing
    motor_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor_out", 20);   // v8: what WE told the motors
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Input (down direction): when a /cmd_vel message arrives, call on_cmd_vel with it.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {on_cmd_vel(msg);});
    // v8: Whisker (scripts/whisker.py) takes over while it talks.
    whisker_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "whisker/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_whisker_time_ = now();
        apply_cmd(msg->twist.linear.x, msg->twist.angular.z);
      });
    // v8.3: OLED lines
    display_sub_ = create_subscription<std_msgs::msg::String>(
      "robot/display", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_display(msg->data);});
    // v8.2: escape force (PWM) — only counts while whisker drives
    force_sub_ = create_subscription<std_msgs::msg::Float32>(
      "whisker/force", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        force_ = msg->data;
        last_force_time_ = now();
      });

    // Two heartbeats with different jobs:
    //   20 ms → on_read  : read feedback (arrives every 50 ms; read faster than it comes so nothing piles up)
    //   50 ms → on_timer : safety check. 50 ms is how OFTEN we check, NOT the timeout (0.5 s).
    read_timer_ = create_wall_timer(20ms, [this]() {on_read();});
    timer_ = create_wall_timer(50ms, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "ready: /cmd_vel -> %s (%d baud), drive_mode=%s", port_.c_str(), baud_, drive_mode_.c_str());
  }

  // Runs when the node is destroyed (Ctrl+C → spin returns). Must never throw: send_stop only logs on failure.
  ~Ugv02Bridge() {
    send_stop("shutdown");
  }

private:
  // ===== zone 2: behavior =====

  // ---- down: commands ----

  // Every /cmd_vel message → one T:1 JSON line (left/right wheel speed). Also records "a command just arrived" for the dead-man.
  // Why T:1 and not T:13 (forward + turn): only T:1 refreshes the firmware heartbeat (see constructor).
  // The split is the same one the firmware did for T:13: L = x - z*track/2, R = x + z*track/2.
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (whisker_active()) {return;}   // v8: Whisker is handling a touch → Nav2 waits
    apply_cmd(msg->twist.linear.x, msg->twist.angular.z);
  }

  // v8.3: write only the OLED lines that changed. Text kept to printable ASCII without quotes/backslash (JSON-safe), 20 chars.
  void on_display(const std::string & text) {
    size_t start = 0;
    for (int i = 0; i < 4; ++i) {
      const size_t end = text.find('\n', start);
      std::string line = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
      std::string safe;
      for (char c : line) {
        if (c >= 32 && c < 127 && c != '"' && c != '\\') {safe += c;}
        if (safe.size() >= 20) {break;}
      }
      if (safe != oled_[i]) {
        char json[96];
        std::snprintf(json, sizeof(json), "{\"T\":3,\"lineNum\":%d,\"Text\":\"%s\"}", i, safe.c_str());
        if (serial_.write_line(json)) {oled_[i] = safe;}
      }
      if (end == std::string::npos) {start = text.size();} else {start = end + 1;}
    }
  }

  bool whisker_active() {
    return last_whisker_time_.nanoseconds() != 0 && (now() - last_whisker_time_).seconds() < kWhiskerHold;
  }

  void apply_cmd(const double x, const double z) {   // x: forward speed m/s, z: turn speed rad/s (+ = left)
    stopped_ = false;
    last_cmd_vel_time_ = now();

    const double left = x - z * track_width_ / 2.0;    // m/s
    const double right = x + z * track_width_ / 2.0;   // m/s

    if (drive_mode_ == "pwm") {   // v5: only remember the goal; on_timer turns it into PWM every 50 ms
      cmd_x_ = x;
      cmd_z_ = z;
      return;
    }

    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":1,\"L\":%.3f,\"R\":%.3f}", left, right);

    // A single failed write may be temporary → log and keep running (unlike a failed open).
    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }

    // Avoid log floods: at most 1 line per second. (Downside: it can hide a stop sent in between.)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "sent: %s", json);
  }

  // Every 50 ms. If no /cmd_vel for cmd_timeout_ (0.5 s), send a stop command once.
  void on_timer() {
    const double silent = (now() - last_cmd_vel_time_).seconds();
    if (!stopped_ && silent > cmd_timeout_) {
      wheel_l_ = Wheel{};
      wheel_r_ = Wheel{};
      cmd_x_ = cmd_z_ = 0.0; turn_trim_ = 0.0; turn_trim_dir_ = 0;
      tap_t_ = 0.0; tap_sum_x_ = tap_sum_z_ = 0.0; tap_n_ = 0; tap_x_ = tap_z_ = tap_duty_ = 0.0;   // resume = fresh tap
      send_stop("cmd_vel timeout");
    }
    if (drive_mode_ == "pwm" && !stopped_ && whisker_active() && cmd_x_ == 0.0 && cmd_z_ == 0.0) {
      // v8: Whisker says "stop" → PWM 0 right now (the tap mode would otherwise finish its current push)
      wheel_l_ = Wheel{}; wheel_r_ = Wheel{}; turn_trim_ = 0.0; turn_trim_dir_ = 0;
      tap_t_ = 0.0; tap_sum_x_ = tap_sum_z_ = 0.0; tap_n_ = 0; tap_x_ = tap_z_ = tap_duty_ = 0.0;
      serial_.write_line("{\"T\":11,\"L\":0,\"R\":0}");
      return;
    }
    if (drive_mode_ == "pwm" && !stopped_) {
      pwm_step();
    }
  }

  // ---- v5: our own motor control (drive_mode "pwm") ----
  // Measured 11-03 on the living-room floor (docs/calibration/lowspeed-2026-11-03.md). {wheel speed m/s, PWM}
  //   straight: PWM held after a short push → speed.  Below 0.05 m/s the wheel can't keep rolling → snapped up.
  //   spin    : on-the-spot turn, per-wheel speed = w * track/2 (0.13 / 0.35 / 0.64 rad/s). Skid-steer drags the wheels
  //             sideways → much more PWM for the same wheel speed.
  struct Wheel {
    double meas{0.0};     // m/s, firmware feedback L/R (coarse, but non-zero = "it is turning")
    int dir{0};           // -1, 0, +1 of the current push
    bool rolling{false};  // false = still breaking away from standstill
    double push{0.0};     // PWM while breaking away (rises each tick)
    double out{0.0};      // PWM sent (signed)
    rclcpp::Time last_move{0, 0, RCL_ROS_TIME};
  };

  static double lookup(const std::vector<std::pair<double, double>> & t, double v) {
    if (v <= t.front().first) {return t.front().second;}            // floor: slower than we can roll → floor PWM
    for (size_t i = 1; i < t.size(); ++i) {
      if (v <= t[i].first) {
        const double a = (v - t[i - 1].first) / (t[i].first - t[i - 1].first);
        return t[i - 1].second + a * (t[i].second - t[i - 1].second);
      }
    }
    const auto & p = t[t.size() - 2]; const auto & q = t.back();     // faster than measured → extend last segment
    return q.second + (v - q.first) * (q.second - p.second) / (q.first - p.first);
  }

  // v5.1 (11-03): split the command into FORWARD (both wheels) and TURN (difference) instead of per-wheel tables.
  //   crawl_test: arcs (0.06 m/s, 0.3 rad/s) drove STRAIGHT — a small PWM difference can't beat the sideways scrub of a
  //   skid-steer. The gyro tells us the real turn rate → a slow loop adds turn PWM until the robot really turns.
  //   forward PWM = table(|x|)                         (open loop; Nav2 + LiDAR correct position)
  //   turn PWM    = table(|z|) + trim,  trim += kTurnGain * (z_cmd − z_gyro) * dt   (closed loop on the gyro)
  //   left = forward − turn,  right = forward + turn
  static double forward_ff(double v) {   // |x| m/s → PWM (both wheels), 11-03 straight test (held PWM after a push)
    static const std::vector<std::pair<double, double>> k =
      {{0.050, 21}, {0.070, 27}, {0.080, 30}, {0.138, 40}, {0.180, 50}, {0.244, 60}, {0.340, 80}};
    return lookup(k, v);
  }
  static double turn_ff(double z, bool on_the_spot) {   // |z| rad/s → PWM difference per wheel
    static const std::vector<std::pair<double, double>> spin = {{0.13, 40}, {0.35, 50}, {0.64, 60}};   // 11-03 spin test
    // On the spot the wheels must break sideways friction → spin table (floor 40).
    // While rolling forward they already move → start lower; the gyro loop adds what is missing.
    return on_the_spot ? lookup(spin, z) : 40.0 * z;   // 40 PWM per rad/s ≈ slope of the spin table
  }

  void control_wheel(Wheel & w, double desired, const rclcpp::Time & t, double cap) {   // desired = signed PWM; cap = force limit
    if (std::abs(desired) < kPwmDeadzone) {     // nothing worth sending (also tiny Nav2 corrections)
      w.out = 0.0; w.dir = 0; w.rolling = false;
      return;
    }
    const int dir = desired > 0 ? 1 : -1;
    const double mag = std::abs(desired);
    if (mag < kNoPushBelow) {                    // inner wheel of an arc: too weak to roll alone → just help, never kick
      w.out = dir * mag; w.dir = dir; w.rolling = false;
      return;
    }
    if (dir != w.dir) {                          // new start or reversal → break away again
      w.dir = dir; w.rolling = false; w.push = mag - kPushStep;
    }
    const bool moving = dir * w.meas > 1e-3;     // turning in the direction we push (not the old direction)
    if (moving) {w.last_move = t;}

    if (!w.rolling) {
      if (moving && w.push >= mag) {             // it turns → drop to the holding PWM (the "kick" ends here)
        w.rolling = true;
      } else {
        w.push = std::min(std::max(w.push, mag - kPushStep) + kPushStep, cap);   // gentle: raise until it moves (≤ force cap)
      }
    } else if ((t - w.last_move).seconds() > kStallTime) {   // stuck while rolling (rug edge) → push again
      w.rolling = false; w.push = mag - kPushStep;
    }
    w.out = dir * std::min({w.rolling ? mag : std::max(w.push, mag), cap, pwm_max_});
  }

  void pwm_step() {
    const rclcpp::Time t = now();
    const double dt = last_pwm_time_.nanoseconds() ? (t - last_pwm_time_).seconds() : 0.05;
    last_pwm_time_ = t;

    const double x = cmd_x_, z = cmd_z_;
    const bool go = std::abs(x) >= kXDeadzone;
    const bool turn = std::abs(z) >= kZDeadzone;

    // v5.2 TAP mode (11-04): Nav2's final creep near a goal asks for less than we can roll continuously
    // (x < 0.05 m/s, z < 0.13 rad/s). Snapping those up to the floor overshot back and forth (hunting at 8 and 10 cm).
    // Instead: short pushes. Every kPulsePeriod, push for a fraction of it (= asked speed / floor speed), then rest.
    // Each tap moves ~1 cm (or ~1-2°) → the AVERAGE speed is what Nav2 asked, and Nav2 sees the result before the next tap.
    const double duty = std::max(go ? std::abs(x) / kXFloor : 0.0, turn ? std::abs(z) / kZFloor : 0.0);
    if (duty < 1.0) {   // includes "nothing asked": zero asks pull the average down too
      tap_step(x, z, dt);
      return;
    }
    tap_t_ = 0.0; tap_sum_x_ = tap_sum_z_ = 0.0; tap_n_ = 0; tap_x_ = tap_z_ = tap_duty_ = 0.0;
    const double fwd = go ? (x > 0 ? 1 : -1) * forward_ff(std::abs(x)) : 0.0;

    double dif = 0.0;
    if (turn) {
      if (turn_trim_dir_ == 0 || (z > 0) != (turn_trim_dir_ > 0)) {   // new turn → fresh trim
        turn_trim_ = 0.0; turn_trim_dir_ = z > 0 ? 1 : -1;
      }
      turn_trim_ += kTurnGain * (z - w_) * dt * turn_trim_dir_;          // positive trim = turn harder in z's direction
      turn_trim_ = std::clamp(turn_trim_, -kTurnTrimMax, kTurnTrimMax);
      dif = turn_trim_dir_ * std::max(0.0, turn_ff(std::abs(z), !go) + turn_trim_);
    } else {
      turn_trim_ = 0.0; turn_trim_dir_ = 0;
    }
    double cap = turn ? push_cap_turn_ : push_cap_straight_;   // v8.1 force limit
    if (whisker_active() && last_force_time_.nanoseconds() != 0 && (t - last_force_time_).seconds() < kWhiskerHold) {
      cap = std::max(cap, std::min(static_cast<double>(force_), kForceMax));   // v8.2 escape: higher breakaway limit
    }
    control_wheel(wheel_l_, fwd - dif, t, cap);
    control_wheel(wheel_r_, fwd + dif, t, cap);

    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":11,\"L\":%d,\"R\":%d}",
      static_cast<int>(std::lround(wheel_l_.out)), static_cast<int>(std::lround(wheel_r_.out)));
    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "pwm: %s  cmd x %.3f z %.2f | gyro z %.2f trim %.1f | rolling %d%d",
      json, x, z, w_, turn_trim_, wheel_l_.rolling, wheel_r_.rolling);
  }

  void tap_step(double x, double z, double dt) {
    // Collect asks during this cycle; at the cycle start, the NEXT tap is decided from the previous cycle's average.
    // Cost: a slow ask starts moving 0.5 s later — fine ("quiet and accurate, need not be fast").
    tap_sum_x_ += x; tap_sum_z_ += z; ++tap_n_;
    tap_t_ += dt;
    if (tap_t_ >= kPulsePeriod) {
      tap_t_ = 0.0;
      tap_x_ = tap_sum_x_ / tap_n_; tap_z_ = tap_sum_z_ / tap_n_;
      tap_sum_x_ = tap_sum_z_ = 0.0; tap_n_ = 0;
      tap_duty_ = std::min(1.0, std::max(std::abs(tap_x_) / kXFloor, std::abs(tap_z_) / kZFloor));
      if (tap_duty_ < kTapMinDuty) {tap_duty_ = 0.0;}   // undecided controller (or a real stop) → rest, stay quiet
    }
    const double on = std::max(kPulseMinOn, tap_duty_ * kPulsePeriod);
    double l = 0.0, r = 0.0;
    if (tap_duty_ > 0.0 && tap_t_ < on) {   // push phase: only the bigger of the two asks, so a tap is one clean motion
      if (std::abs(tap_x_) / kXFloor >= std::abs(tap_z_) / kZFloor) {
        l = r = (tap_x_ > 0 ? 1 : -1) * kPulseFwdPwm;
      } else {
        const double d = (tap_z_ > 0 ? 1 : -1) * kPulseTurnPwm;
        l = -d; r = d;
      }
    }
    wheel_l_ = Wheel{}; wheel_r_ = Wheel{};   // after tapping, a normal start breaks away again from scratch
    turn_trim_ = 0.0; turn_trim_dir_ = 0;
    wheel_l_.out = l; wheel_r_.out = r;

    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":11,\"L\":%d,\"R\":%d}",
      static_cast<int>(std::lround(l)), static_cast<int>(std::lround(r)));
    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }
    if (tap_duty_ > 0.0 || std::abs(x) >= kXDeadzone || std::abs(z) >= kZDeadzone) {   // silent while fully stopped
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "pwm: %s  cmd x %.3f z %.2f | TAP avg x %.3f z %.2f duty %.2f", json, x, z, tap_x_, tap_z_, tap_duty_);
    }
  }

  // PWM mode stops with PWM 0 (T:11), NOT T:1: T:1 hands the wheels back to the firmware PID while they still roll,
  // and its fresh integral brakes so hard the wheels kick BACKWARDS (11-03 hang test: −0.19…−0.38 m/s for 0.2 s).
  // After PWM 0 the wheels stop in ~0.1 s at crawl speed; if the heartbeat later switches to the PID, it sees
  // "target 0, speed 0" → output 0 (firmware rule) → no kick.
  void send_stop(const char * reason) {
    char json[64];
    if (drive_mode_ == "pwm") {
      std::snprintf(json, sizeof(json), "{\"T\":11,\"L\":0,\"R\":0}");
    } else {
      std::snprintf(json, sizeof(json), "{\"T\":1,\"L\":0.000,\"R\":0.000}");
    }
    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }
    stopped_ = true;
    RCLCPP_INFO(get_logger(), "sent stop (%s), %.3f s after last /cmd_vel",
      reason, (now() - last_cmd_vel_time_).seconds());
  }

  // ---- up: reports ----

  // Every 20 ms. Read every COMPLETE line the ESP32 sent since last time:
  //   parse (JSON → numbers) → integrate (speeds → position) → publish (position → ROS messages).
  void on_read() {
    std::vector<std::string> lines;
    serial_.read_lines(lines);   // never waits

    for (const auto & line : lines) {   // = Python "for line in lines" (const: read-only, &: no copy)
      scr_ugv02::Feedback fb;
      // Not a T:1001 line (other message, garbage) → skip it, log at most once per second.
      if (!scr_ugv02::parse_feedback(line, fb)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "failed to parse feedback: %s", line.c_str());
        continue;
      }
      wheel_l_.meas = fb.left;    // v5 PWM control: "is this wheel turning?"
      wheel_r_.meas = fb.right;
      integrate_odom(fb);
      publish_odom();
      publish_battery(fb);
      publish_imu(fb);
      publish_motor();
    }
  }

  // v3 (10-31): ODOMETRY = distance from the wheel odometers (odl/odr), heading from the GYRO.
  // Why: measured on the real robot, the wheels inflate in-place rotation 2.3-2.7x (skid-steer slip),
  // the gyro is within ±5% → docs/calibration/odom-2026-10-31.md
  // Plumbing (state, first report, bias bookkeeping) = Claude. The three ★ blanks = Jasper.
  void integrate_odom(const scr_ugv02::Feedback & fb) {
    const rclcpp::Time t = now();

    // First report: nothing to compare with yet → remember time and odometer readings only.
    if (last_feedback_time_.nanoseconds() == 0) {
      last_feedback_time_ = t;
      last_odl_ = fb.odl_m;
      last_odr_ = fb.odr_m;
      return;
    }
    const double dt = (t - last_feedback_time_).seconds();   // time between this report and the previous one
    last_feedback_time_ = t;

    // ① Start-up gyro bias: the robot stands still for the first kBiasWindow seconds after the bridge starts.
    //    Average gz over that time = what the gyro says when NOT rotating. Until then: no odometry.
    if (!bias_ready_) {
      still_sum_ += fb.gz;
      still_count_ += 1;
      still_time_ += dt;
      last_odl_ = fb.odl_m;
      last_odr_ = fb.odr_m;
      if (still_time_ >= kBiasWindow) {
        gyro_bias_ = still_sum_ / still_count_;
        bias_ready_ = true;
        RCLCPP_INFO(get_logger(), "gyro bias %.2f (from %d samples, %.1f s) - odometry starts", gyro_bias_, still_count_, still_time_);
        still_sum_ = 0.0; still_count_ = 0; still_time_ = 0.0;
      }
      return;
    }

    // ② ★ Jasper: turn rate in rad/s (+ = counter-clockwise), from the gyro.
    w_ = scr_ugv02::gyro_to_rad_s(fb.gz - gyro_bias_);
    yaw_ += w_ * dt;

    // ③ ★ Jasper: how far the robot CENTER moved since the previous report, in metres.
    const double d = (fb.odl_m - last_odl_ + fb.odr_m - last_odr_) / 2.0;   // m
    last_odl_ = fb.odl_m;
    last_odr_ = fb.odr_m;

    // Distance is along the robot's nose → split onto the floor axes with the current heading.
    x_ += d * std::cos(yaw_);
    y_ += d * std::sin(yaw_);
    v_ = (fb.left + fb.right) / 2;   // /odom velocity field: wheel SPEEDS (smooth; odometer steps are 1 cm)

    // ⑤ ★ Jasper: "really standing still" → only then may the bias be re-measured.
    //   wheels report exactly 0 at rest; gz at rest jitters about ±2-4 units around the bias (10-31 bias runs)
    //   → 6 units = 0.37 °/s: rest noise passes, a slow hand turn (1 °/s = 16 units) does not.  [Claude, 10-31: tune while driving]
    const bool still = std::abs(fb.left) < 1e-3 && std::abs(fb.right) < 1e-3 &&
                       std::abs(fb.gz - gyro_bias_) < kStillGyroUnits;
    if (still) {
      still_sum_ += fb.gz;
      still_count_ += 1;
      still_time_ += dt;
      if (still_time_ >= kBiasWindow) {   // still for the whole window → replace the bias with this average
        const double new_bias = still_sum_ / still_count_;
        if (std::abs(new_bias - gyro_bias_) > 0.5) {   // log only real changes (edge, not every 3 s)
          RCLCPP_INFO(get_logger(), "gyro bias %.2f -> %.2f", gyro_bias_, new_bias);
        }
        gyro_bias_ = new_bias;
        still_sum_ = 0.0; still_count_ = 0; still_time_ = 0.0;
      }
    } else {   // moved (or rotated by hand) → start the still window over
      still_sum_ = 0.0; still_count_ = 0; still_time_ = 0.0;
    }
  }

  // Publish the same pose two ways, with ONE timestamp (same fact → same time):
  //   /odom message : for nodes that want pose + velocity (simple_mover, Nav2)
  //   TF            : the odom → base_footprint link of the frame chain (RViz, SLAM)
  // Fields that fail SILENTLY if wrong: frame_id / child_frame_id, stamp, pose vs twist, orientation (quaternion).
  // v6 (11-06): the IMU as it comes from the board (every feedback line, ~20 Hz) → /imu, for the "sensitive robot"
  // (bump / climbing a carpet edge / someone touching it). Axes = the BOARD's axes (which one is "forward" is measured
  // with scripts/imu_lab.py, not assumed). Accel: raw / 8500 × 9.81 (az ≈ 8500 at rest = 1 g, scale assumed).
  // Gyro: all three axes with the same scale as gz; only gz has its bias removed (the others: measure at rest).
  // No orientation estimate (orientation_covariance[0] = -1 = "not provided", ROS convention).
  void publish_imu(const scr_ugv02::Feedback & fb) {
    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = "base_link";
    m.orientation_covariance[0] = -1.0;
    constexpr double kAccel = 9.81 / 8500.0;
    m.linear_acceleration.x = fb.ax * kAccel;
    m.linear_acceleration.y = fb.ay * kAccel;
    m.linear_acceleration.z = fb.az * kAccel;
    m.angular_velocity.x = scr_ugv02::gyro_to_rad_s(fb.gx);
    m.angular_velocity.y = scr_ugv02::gyro_to_rad_s(fb.gy);
    m.angular_velocity.z = scr_ugv02::gyro_to_rad_s(fb.gz - gyro_bias_);
    imu_pub_->publish(m);
  }

  // v8: efference copy for whisker.py — the motor outputs WE chose (PWM mode) and the command we follow.
  void publish_motor() {
    std_msgs::msg::Float32MultiArray m;
    const bool pwm = drive_mode_ == "pwm";
    m.data = {static_cast<float>(pwm ? wheel_l_.out : 0.0), static_cast<float>(pwm ? wheel_r_.out : 0.0),
              static_cast<float>(cmd_x_), static_cast<float>(cmd_z_), whisker_active() ? 1.0f : 0.0f};
    motor_pub_->publish(m);
  }

  void publish_odom() {
    const rclcpp::Time stamp = now();

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);   // ROS stores orientation as a quaternion, not as an angle

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;   // "position is measured in the odom frame"
    odom_msg.child_frame_id = base_frame_;    // "...and it is the position of base_footprint"
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.twist.twist.linear.x = v_;       // velocity, in the robot (child) frame
    odom_msg.twist.twist.angular.z = w_;
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  // Battery voltage for task_manager (later: "low battery → go charge"). Published at feedback rate for now.
  void publish_battery(const scr_ugv02::Feedback & fb) {
    sensor_msgs::msg::BatteryState battery_msg;
    battery_msg.header.stamp = now();
    battery_msg.voltage = fb.voltage;   // float32 → 12.29 shows as 12.2899999…
    battery_pub_->publish(battery_msg);
  }

  // ===== zone 3: member variables (the node's memory) =====
  // parameters
  std::string port_;
  int baud_{115200};
  double cmd_timeout_{0.5};
  double track_width_{0.170};
  int heartbeat_ms_{500};
  std::string drive_mode_{"pid"};
  double pwm_max_{100.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};   // root frame of the TB3 URDF

  // ROS handles (kept as members, otherwise they are destroyed when the constructor ends)
  scr_ugv02::SerialPort serial_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr whisker_sub_;
  rclcpp::Time last_whisker_time_{0, 0, RCL_ROS_TIME};
  double push_cap_straight_{40.0}, push_cap_turn_{55.0};   // v8.1 force limit (PWM)
  static constexpr double kWhiskerHold = 0.3;   // s, whisker keeps priority this long after its last command
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_sub_;   // v8.2
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_sub_;  // v8.3
  std::string oled_[4];
  float force_{0.0f};
  rclcpp::Time last_force_time_{0, 0, RCL_ROS_TIME};
  static constexpr double kForceMax = 70.0;     // PWM, escape breakaway never above this (11-07 threshold test climbed at 70)
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  // down: dead-man state
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  bool stopped_{true};

  // v5 PWM control state
  // Deadzones = only true noise. 11-04: 0.01 / 0.03 swallowed Nav2's final creep (x −0.007, z −0.01 near the goal)
  // → PWM 0 → "Failed to make progress" forever. Rule now: if Nav2 asks to move, we move — small asks are snapped
  // UP to the floor (0.05 m/s / 0.13 rad/s); Nav2 re-plans every 50 ms and corrects any overshoot.
  static constexpr double kXDeadzone = 0.003;       // m/s, below → no forward part
  static constexpr double kZDeadzone = 0.01;        // rad/s, below → no turn part
  static constexpr double kPwmDeadzone = 5.0;       // PWM, below → wheel off
  static constexpr double kNoPushBelow = 18.0;      // PWM, below the rolling floor (21) → no breakaway push (arc inner wheel)
  static constexpr double kPushStep = 3.0;          // PWM per 50 ms tick while breaking away (= 60 PWM/s)
  static constexpr double kStallTime = 0.5;         // s without wheel motion while "rolling" → push again
  static constexpr double kTurnGain = 60.0;         // PWM per rad of turn-rate error (0.1 rad/s short → +6 PWM/s)
  static constexpr double kTurnTrimMax = 40.0;      // PWM, how much the gyro loop may add/remove
  // v5.2 tap mode (slower than we can roll → short pushes). Pushes = the 11-03 "kick" levels that reliably break away.
  static constexpr double kXFloor = 0.05;           // m/s, slowest continuous roll (PWM 21 after a push)
  static constexpr double kZFloor = 0.13;           // rad/s, slowest continuous on-the-spot turn (PWM 40)
  static constexpr double kPulsePeriod = 0.5;       // s, one tap cycle
  static constexpr double kPulseMinOn = 0.10;       // s, shortest push that still moves the wheels
  static constexpr double kPulseFwdPwm = 35.0;      // PWM of a forward/backward tap
  static constexpr double kPulseTurnPwm = 55.0;     // PWM of a turning tap (sideways scrub needs more)
  static constexpr double kTapMinDuty = 0.2;        // average ask below 20% of the floor → no tap (noise / undecided)
  double tap_t_{0.0};                               // s, position inside the current tap cycle
  double tap_sum_x_{0.0}, tap_sum_z_{0.0};          // asks collected during this cycle
  int tap_n_{0};
  double tap_x_{0.0}, tap_z_{0.0}, tap_duty_{0.0};  // the tap decided at this cycle's start (from the last cycle)
  Wheel wheel_l_, wheel_r_;
  double cmd_x_{0.0}, cmd_z_{0.0};                  // latest /cmd_vel (m/s, rad/s)
  double turn_trim_{0.0};                           // PWM, learned by the gyro loop during this turn
  int turn_trim_dir_{0};
  rclcpp::Time last_pwm_time_{0, 0, RCL_ROS_TIME};

  // up: odometry state
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0}, y_{0.0}, yaw_{0.0};   // pose in the odom frame (m, m, rad)

  // gyro bias + odometer state (v3)
  static constexpr double kBiasWindow = 3.0;   // s, how long "standing still" must last to (re)measure the bias
  static constexpr double kStillGyroUnits = 6.0; // raw gz units, max |gz - bias| that still counts as "not rotating"
  bool bias_ready_{false};
  double gyro_bias_{0.0};                       // raw gz units at rest (real robot ≈ −15)
  double still_sum_{0.0};                       // sum of gz during the current still window
  int still_count_{0};
  double still_time_{0.0};                      // s
  double last_odl_{0.0}, last_odr_{0.0};        // previous odometer readings, m
  double v_{0.0}, w_{0.0};              // latest speeds (m/s, rad/s), also published in /odom twist

  // bias
  int start_bias_{0};   // the first feedback line may have a non-zero bias (e.g., 0.01 m/s) → ignore it
  
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ugv02Bridge>());
  rclcpp::shutdown();
  return 0;
}
