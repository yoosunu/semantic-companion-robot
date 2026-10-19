// simple_mover: move the robot by a commanded distance or angle, using only /cmd_vel and /odom.
//
// Commands (std_msgs/String on /simple_mover/command):
//   "forward 1.0"   move forward 1.0 m        "backward 0.5"  move backward 0.5 m
//   "left 90"       turn left 90 deg          "right 45"      turn right 45 deg
//   "stop"          stop immediately
//
// Interfaces (same in Gazebo and on the real UGV02 once its bridge exists):
//   publishes  /cmd_vel  geometry_msgs/TwistStamped   (our Gazebo bridge expects TwistStamped)
//   subscribes /odom     nav_msgs/Odometry
//
// BASELINE ON PURPOSE (upgrade targets are listed in this package's README):
//   - constant speed until the target is reached -> the robot overshoots a little
//   - no obstacle check
//   - one command at a time (a new command replaces the current one)

#include <chrono> // unit of time, e.g. 50ms
#include <cmath>  
#include <memory> // std::make_shared => smart pointer
#include <sstream> // for dividing string into words
#include <string> 

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp" // main library for ROS2 C++ nodes
#include "std_msgs/msg/string.hpp"

#include "angles/angles.h"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class SimpleMover : public rclcpp::Node
{
public:
  SimpleMover() : Node("simple_mover") {
    // Parameters can be changed at launch: --ros-args -p linear_speed:=0.1
    linear_speed_ = declare_parameter<double>("linear_speed", 0.15);    // m/s   (Burger max 0.22)
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);   // rad/s
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);     // s, stop if odom goes silent

    // Publish /cmd_vel (gonna be subscribed by the ugv02_bridge)
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10); // 10 means size of Queue

    // Subscribe /odom (gonna be published by the ugv02_bridge)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(msg);});

    // "~/command" = a topic under this node's name -> /simple_mover/command
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "~/command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {on_command(msg->data);});

    // Control loop at 20 Hz => control ros2's heart beat.
    timer_ = create_wall_timer(50ms, [this]() {on_timer();});

    RCLCPP_INFO(get_logger(),
      "ready. send e.g.: ros2 topic pub --once /simple_mover/command std_msgs/msg/String \"{data: 'forward 1.0'}\"");
  }

private:
  // define Mode of operation for the robot by enum
  enum class Mode { kIdle, kStraight, kTurn };

  // ---------- callbacks ----------
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // get x_, y_, yaw_ from the odometry message
    x_ = msg->pose.pose.position.x; 
    y_ = msg->pose.pose.position.y;
    const double yaw = tf2::getYaw(msg->pose.pose.orientation);

    // While turning, add up small yaw changes so turns larger than 180 deg are measured correctly.
    if (mode_ == Mode::kTurn && have_odom_) {
      turned_ += angles::normalize_angle(yaw - yaw_);
    }
    yaw_ = yaw;
    have_odom_ = true;
    last_odom_time_ = now();
  }

  void on_command(const std::string & text)
  {
    std::istringstream in(text);
    std::string verb;
    double value = 0.0;
    in >> verb >> value;

    if (verb == "stop") {
      stop("stop command");
      return;
    }

    // exceptions: no odom yet, or non-positive value
    if (!have_odom_) {
      RCLCPP_WARN(get_logger(), "no /odom yet, ignoring '%s'", text.c_str());
      return;
    }
    if (value <= 0.0) {
      RCLCPP_WARN(get_logger(), "need a positive value, e.g. 'forward 1.0' or 'left 90' (got '%s')", text.c_str());
      return;
    }

    // operations
    if (verb == "forward" || verb == "backward") {
      mode_ = Mode::kStraight;
      direction_ = (verb == "forward") ? 1.0 : -1.0;
      target_ = value;                      // meters
      start_x_ = x_;
      start_y_ = y_;
      RCLCPP_INFO(get_logger(), "%s %.2f m", verb.c_str(), target_);
    } else if (verb == "left" || verb == "right") {
      mode_ = Mode::kTurn;
      direction_ = (verb == "left") ? 1.0 : -1.0;   // ROS convention: +z rotation = counter-clockwise = left
      target_ = angles::from_degrees(value);        // degrees -> radians
      turned_ = 0.0;
      RCLCPP_INFO(get_logger(), "%s %.1f deg", verb.c_str(), value);
    } else {
      RCLCPP_WARN(get_logger(), "unknown command '%s' (use forward/backward/left/right/stop)", text.c_str());
    }
  }

  // timer (like heartbeat) runs every 50 ms
  void on_timer()
  {
    if (mode_ == Mode::kIdle) {
      return;   // stay quiet when idle, so other nodes (teleop, Nav2) can use /cmd_vel
    }

    // Safety: timeout
    if ((now() - last_odom_time_).seconds() > odom_timeout_) {
      stop("odom timeout");
      return;
    }

    // straight or turn: check if the target is reached, otherwise publish a command
    if (mode_ == Mode::kStraight) {
      const double moved = std::hypot(x_ - start_x_, y_ - start_y_);
      if (moved >= target_) {
        RCLCPP_INFO(get_logger(), "done: target %.3f m, moved %.3f m at stop command", target_, moved);
        stop("target reached");
      } else {
        // publish
        publish(direction_ * linear_speed_, 0.0);
      }
    } else if (mode_ == Mode::kTurn) {
      if (std::fabs(turned_) >= target_) {
        RCLCPP_INFO(get_logger(), "done: target %.1f deg, turned %.1f deg at stop command",
          angles::to_degrees(target_), angles::to_degrees(std::fabs(turned_))); 
        stop("target reached");
      } else {
        // publishㄷ
        publish(0.0, direction_ * angular_speed_);
      }
    }
  }

  // ---------- helpers ----------
  void publish(double linear, double angular)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    cmd_pub_->publish(msg);
  }

  void stop(const char * reason)
  {
    publish(0.0, 0.0);
    if (mode_ != Mode::kIdle) {
      RCLCPP_INFO(get_logger(), "stopped (%s)", reason);
    }
    mode_ = Mode::kIdle;
  }

  // ---------- state ----------
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double linear_speed_{0.15};
  double angular_speed_{0.5};
  double odom_timeout_{0.5};

  bool have_odom_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0}, y_{0.0}, yaw_{0.0};

  Mode mode_{Mode::kIdle};
  double direction_{1.0};
  double target_{0.0};
  double start_x_{0.0}, start_y_{0.0};
  double turned_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMover>());
  rclcpp::shutdown();
  return 0;
}
