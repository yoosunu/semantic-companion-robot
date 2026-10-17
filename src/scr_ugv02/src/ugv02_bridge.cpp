// ugv02_bridge — translator between ROS 2 and the UGV02 base (JSON lines over serial).
//
//   /cmd_vel (geometry_msgs/TwistStamped) ──> {"T":13,"X":<linear.x>,"Z":<angular.z>} ──> serial
//
// Protocol: docs/ugv02-protocol.md      Plan & pass criteria: this package's README.md
//
// v0 — written by Jasper. Review fixes by Claude are marked "// [fix]".
// v2 (safety stop) — written by Jasper. Dead-man rule: no /cmd_vel for cmd_timeout (0.5 s) -> send stop.
//   Also sends stop on shutdown (destructor). Limit: if the bridge itself is killed (kill -9 / crash),
//   only the UGV02 firmware 3 s timeout remains -> defense in depth.
// Serial I/O: include/scr_ugv02/serial_port.hpp (boilerplate).
//
// How a ROS node class is laid out (3 zones):
//   1. constructor  = setup, runs ONCE  (read parameters, open port, register subscriptions)
//   2. functions    = behavior          (on_cmd_vel runs every time a /cmd_vel message arrives)
//   3. member vars  = the node's memory (things that must live as long as the node does)

#include <cstdio>     // [fix] std::snprintf
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "scr_ugv02/serial_port.hpp"

using namespace std::chrono_literals;

class Ugv02Bridge : public rclcpp::Node
{
public:
  // ===== zone 1: constructor (setup, runs once) =====
  Ugv02Bridge() : Node("ugv02_bridge") {
    
    port_ = declare_parameter<std::string>("port", "/tmp/ugv02_bridge");
    baud_ = declare_parameter<int>("baud", 115200);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);

    // A bridge without a port must not pretend to run 
    // -> stop the program with a clear reason.
    if (!serial_.open(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "failed to open serial port %s at %d baud: %s",
        port_.c_str(), baud_, serial_.error().c_str());
      throw std::runtime_error("failed to open serial port");
    }

    // "When a /cmd_vel message arrives, call on_cmd_vel with it."
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {on_cmd_vel(msg);});

    // 50 ms = how OFTEN we check (period). NOT the timeout. Timeout = how long silence is allowed (0.5 s).
    timer_ = create_wall_timer(50ms, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(), "ready: /cmd_vel -> %s (%d baud)", port_.c_str(), baud_);
  }

  // Runs when the node is destroyed (Ctrl+C -> spin returns). Must never throw: send_stop only logs on failure.
  ~Ugv02Bridge() {
    send_stop("shutdown");
  }
  

private:
  // ===== zone 2: behavior =====
  // Called every time a /cmd_vel message arrives.
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    stopped_ = false;
    last_cmd_vel_time_ = now();

    const double x = msg->twist.linear.x;    // forward speed, m/s
    const double z = msg->twist.angular.z;   // turn speed, rad/s (+ = left)

    // write a JSON line to the serial port.
    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", x, z);

    // exception
    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }

    // avoid spamming the log: throttle to 1 message per second (1000 ms)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "sent: %s", json);
  }

  void on_timer() {
    // Runs every 50 ms. If no /cmd_vel for cmd_timeout_ (0.5 s), send a stop command once.
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

  // ===== zone 3: member variables (the node's memory) =====
  std::string port_;
  int baud_{115200};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  scr_ugv02::SerialPort serial_;

  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;

  bool stopped_{true};
  double cmd_timeout_{0.5}; 
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ugv02Bridge>());
  rclcpp::shutdown();
  return 0;
}
