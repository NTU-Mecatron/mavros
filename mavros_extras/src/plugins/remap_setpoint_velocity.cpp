#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <mutex>
#include <chrono>

namespace mavros {
namespace extra_plugins {

using namespace std::placeholders;
using mavlink::common::MAV_FRAME;

class RemapSetpointVelocityPlugin :
  public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<RemapSetpointVelocityPlugin> {
public:
  explicit RemapSetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "remap_setpoint_velocity"),
    reset_timeout_(1.0)
  {
    enable_node_watch_parameters();

    node_declare_and_watch_parameter(
      "remap_setpoint_vel_reset_timeout", 1.0,
      [&](const rclcpp::Parameter & p) {
        reset_timeout_ = p.as_double();
      });

    auto sensor_qos = rclcpp::SensorDataQoS();

    twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      "setpoint_velocity/cmd_vel_unstamped", 10);

    vel_x_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/x", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_.linear.x = msg->data;
        last_vel_x_time_ = node->now();
      });

    vel_y_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/y", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_.linear.y = msg->data;
        last_vel_y_time_ = node->now();
      });

    vel_z_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/z", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_.linear.z = msg->data;
        last_vel_z_time_ = node->now();
      });

    vel_r_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/r", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        twist_.angular.z = msg->data;
        last_vel_r_time_ = node->now();
      });

    using namespace std::chrono_literals;
    publish_timer_ = rclcpp::create_timer(
      node, node->get_clock(),
      rclcpp::Duration(100ms),
      std::bind(&RemapSetpointVelocityPlugin::publisher_timer_cb, this));

    auto now = node->now();
    last_vel_x_time_ = now;
    last_vel_y_time_ = now;
    last_vel_z_time_ = now;
    last_vel_r_time_ = now;

    twist_ = geometry_msgs::msg::Twist();
  }

  ~RemapSetpointVelocityPlugin() = default;

  Subscriptions get_subscriptions() override {
      return { /* Rx disabled */ };
  }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_y_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_z_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_r_sub_;

    geometry_msgs::msg::Twist twist_;
    std::mutex mutex_;
    rclcpp::Time last_vel_x_time_;
    rclcpp::Time last_vel_y_time_;
    rclcpp::Time last_vel_z_time_;
    rclcpp::Time last_vel_r_time_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    double reset_timeout_;

    void publisher_timer_cb() {
      geometry_msgs::msg::Twist twist_copy;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        rclcpp::Time current_time = node->now();


        if ((current_time - last_vel_x_time_).seconds() > reset_timeout_)
          twist_.linear.x = 0.0;

        if ((current_time - last_vel_y_time_).seconds() > reset_timeout_)
          twist_.linear.y = 0.0;

        if ((current_time - last_vel_z_time_).seconds() > reset_timeout_)
          twist_.linear.z = 0.0;

        if ((current_time - last_vel_r_time_).seconds() > reset_timeout_)
          twist_.angular.z = 0.0;

        twist_copy = twist_;
      }

      twist_pub_->publish(twist_copy);
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RemapSetpointVelocityPlugin)
