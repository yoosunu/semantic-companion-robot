// ugv02_bridge — translator between ROS 2 and the UGV02 base (JSON lines over serial).
//
//   /cmd_vel (geometry_msgs/TwistStamped) ──> {"T":13,"X":<linear.x>,"Z":<angular.z>} ──> serial
//
// Protocol: docs/ugv02-protocol.md      Plan & pass criteria: this package's README.md
//
// v0 — written by Jasper. Review fixes by Claude are marked "// [fix]".
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

class Ugv02Bridge : public rclcpp::Node
{
public:
  // ===== zone 1: constructor (setup, runs once) =====
  Ugv02Bridge()
  : Node("ugv02_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/tmp/ugv02_bridge");
    baud_ = declare_parameter<int>("baud", 115200);

    // A bridge without a port must not pretend to run -> stop the program with a clear reason.
    if (!serial_.open(port_, baud_)) {
      RCLCPP_ERROR(get_logger(), "failed to open serial port %s at %d baud: %s",
        port_.c_str(), baud_, serial_.error().c_str());
      throw std::runtime_error("failed to open serial port");
    }

    // "When a /cmd_vel message arrives, call on_cmd_vel with it."
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {on_cmd_vel(msg);});

    RCLCPP_INFO(get_logger(), "ready: /cmd_vel -> %s (%d baud)", port_.c_str(), baud_);
  }

private:
  // ===== zone 2: behavior =====
  // [fix] parameter type: this callback receives TwistStamped (what we subscribed to), not Odometry
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // [fix] x, z, json only live during this call -> local variables, not members (t_, z_, json)
    const double x = msg->twist.linear.x;    // forward speed, m/s
    const double z = msg->twist.angular.z;   // turn speed, rad/s (+ = left)

    char json[64];
    std::snprintf(json, sizeof(json), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", x, z);

    if (!serial_.write_line(json)) {
      RCLCPP_ERROR(get_logger(), "failed to write to serial port: %s", serial_.error().c_str());
      return;
    }
    // [fix] moved inside the function: a log line is an action, so it must be inside a function
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "sent: %s", json);
  }

  // ===== zone 3: member variables (the node's memory) =====
  // [fix] types: port is just text, baud is just a number, the subscription matches TwistStamped
  std::string port_;
  int baud_{115200};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  scr_ugv02::SerialPort serial_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ugv02Bridge>());
  rclcpp::shutdown();
  return 0;
}
