// ugv02_bridge — translator between ROS 2 and the UGV02 base (JSON lines over a serial cable).
// Written by Jasper (logic). Boilerplate by Claude: serial_port.hpp, ugv02_protocol.hpp.
//
//   down (commands):  /cmd_vel (TwistStamped) ──▶ {"T":13,"X":..,"Z":..}\n ──▶ serial ──▶ UGV02 ESP32
//   up   (reports):   /odom + TF odom→base_footprint + /battery_state ◀── {"T":1001,"L":..,"R":..,"v":..}\n ◀── ESP32
//
// v0  /cmd_vel → T:13 JSON.
// v2  safety: dead-man (no /cmd_vel for cmd_timeout 0.5 s → stop once) + stop on shutdown.
//     Limit: if the bridge itself is killed hard (kill -9 / crash) only the firmware 3 s timeout remains
//     → defense in depth (teleop 0.5 s → bridge 0.5 s → firmware 3 s).
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

#include <cmath>      // std::cos, std::sin
#include <cstdio>     // std::snprintf
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
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
    track_width_ = declare_parameter<double>("track_width", 0.160);         // m, left↔right wheel distance (TB3 sim; UGV02: calibrate)

    // A bridge without a port must not pretend to run → stop the program with a clear reason.
    if (!serial_.open(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "failed to open serial port %s at %d baud: %s",
        port_.c_str(), baud_, serial_.error().c_str());
      throw std::runtime_error("failed to open serial port");
    }

    // The chassis does NOT report by default. Ask it once to stream T:1001 feedback continuously.
    serial_.write_line(scr_ugv02::make_enable_feedback());

    // Outputs (up direction).
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Input (down direction): when a /cmd_vel message arrives, call on_cmd_vel with it.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {on_cmd_vel(msg);});

    // Two heartbeats with different jobs:
    //   20 ms → on_read  : read feedback (arrives every 50 ms; read faster than it comes so nothing piles up)
    //   50 ms → on_timer : safety check. 50 ms is how OFTEN we check, NOT the timeout (0.5 s).
    read_timer_ = create_wall_timer(20ms, [this]() {on_read();});
    timer_ = create_wall_timer(50ms, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "ready: /cmd_vel -> %s (%d baud)", port_.c_str(), baud_);
  }

  // Runs when the node is destroyed (Ctrl+C → spin returns). Must never throw: send_stop only logs on failure.
  ~Ugv02Bridge() {
    send_stop("shutdown");
  }

private:
  // ===== zone 2: behavior =====

  // ---- down: commands ----

  // Every /cmd_vel message → one T:13 JSON line. Also records "a command just arrived" for the dead-man.
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    stopped_ = false;
    last_cmd_vel_time_ = now();

    const double x = msg->twist.linear.x;    // forward speed, m/s
    const double z = msg->twist.angular.z;   // turn speed, rad/s (+ = left)

    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", x, z);

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
      send_stop("cmd_vel timeout");
    }
  }

  // One place for the stop command (used by on_timer and the destructor).
  // stopped_ is set only after a successful write, so a failed write is retried on the next tick.
  void send_stop(const char * reason) {
    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":13,\"X\":0.000,\"Z\":0.000}");
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
      integrate_odom(fb);
      publish_odom();
      publish_battery(fb);
    }
  }

  // THE CORE OF v1: wheel odometry (dead reckoning).
  // The ESP32 only reports wheel SPEEDS, never a position → we add up "speed × time" to estimate where we are.
  // Errors accumulate over time (drift); SLAM later corrects it with map → odom.
  void integrate_odom(const scr_ugv02::Feedback & fb) {
    const rclcpp::Time t = now();

    // First report: nothing to compare with yet → remember the time only.
    if (last_feedback_time_.nanoseconds() == 0) {
      last_feedback_time_ = t;
      return;
    }
    const double dt = (t - last_feedback_time_).seconds();   // time between this report and the previous one
    last_feedback_time_ = t;

    v_ = (fb.left + fb.right) / 2;              // forward speed of the robot center, m/s
    w_ = (fb.right - fb.left) / track_width_;   // turn rate, rad/s (+ = left / counter-clockwise)

    // v is along the robot's nose (base frame); x/y are on the floor (odom frame)
    // → split v onto the floor axes with the current heading: cos for x, sin for y.
    x_ += v_ * std::cos(yaw_) * dt;
    y_ += v_ * std::sin(yaw_) * dt;
    yaw_ += w_ * dt;
  }

  // Publish the same pose two ways, with ONE timestamp (same fact → same time):
  //   /odom message : for nodes that want pose + velocity (simple_mover, Nav2)
  //   TF            : the odom → base_footprint link of the frame chain (RViz, SLAM)
  // Fields that fail SILENTLY if wrong: frame_id / child_frame_id, stamp, pose vs twist, orientation (quaternion).
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
  double track_width_{0.160};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};   // root frame of the TB3 URDF

  // ROS handles (kept as members, otherwise they are destroyed when the constructor ends)
  scr_ugv02::SerialPort serial_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  // down: dead-man state
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  bool stopped_{true};

  // up: odometry state
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0}, y_{0.0}, yaw_{0.0};   // pose in the odom frame (m, m, rad)
  double v_{0.0}, w_{0.0};              // latest speeds (m/s, rad/s), also published in /odom twist
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ugv02Bridge>());
  rclcpp::shutdown();
  return 0;
}
