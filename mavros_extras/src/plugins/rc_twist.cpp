/**
 * @brief RC Twsit plugin
 * @file rc_twist.cpp
 * @author Tien Luc Vu <luc001@e.ntu.edu.sg>
 *
 * This plugin exposed a twist topic, as well as four float32 topics to control x,y,z,r
 * It will only send control signals at most 10hz inside a timer callback, not inside any of the subscriber cb.
 * After a timeout duration, it will reset and stop sending signals.
 * @addtogroup plugin
 * @{
 */

#include <vector>
#include <cstdint>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT
using Twist = geometry_msgs::msg::Twist;
using Float32 = std_msgs::msg::Float32;

/**
 * @brief RC Twist plugin
 * @plugin rc_twist
 */
class RCTwistPlugin : public plugin::Plugin
{
  public:
  explicit RCTwistPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "rc_twist")
  {
    initialize_rc_override_msg();
    last_rc_override_time = this->get_clock()->now();

    enable_node_watch_parameters();
    node_declare_and_watch_parameter(
      "rc_override_timeout", 1.0, [&](const rclcpp::Parameter & p) {
        rc_override_timeout = p.as_double();
      });

    twist_sub = node->create_subscription<Twist>(
      "~/send", 10, std::bind(&RCTwistPlugin::twist_cb, this, _1));

    twist_x_sub = node->create_subscription<Float32>(
      "~/send/x", 10, [this](const Float32::SharedPtr msg) {
        lock_guard lock(mutex); last_rc_override_time = this->get_clock()->now();
        rc_override_msg.chan5_raw = convert_normalized_to_pwm(msg->data);  // Forward (X-axis)
      });

    twist_y_sub = node->create_subscription<Float32>(
      "~/send/y", 10, [this](const Float32::SharedPtr msg) {
        lock_guard lock(mutex); last_rc_override_time = this->get_clock()->now();
        rc_override_msg.chan6_raw = convert_normalized_to_pwm(msg->data);  // Lateral (Y-axis)
      });

    twist_z_sub = node->create_subscription<Float32>(
      "~/send/z", 10, [this](const Float32::SharedPtr msg) {
        lock_guard lock(mutex); last_rc_override_time = this->get_clock()->now();
        rc_override_msg.chan3_raw = convert_normalized_to_pwm(msg->data);  // Throttle (Z-axis)
      });

    twist_yaw_sub = node->create_subscription<Float32>(
      "~/send/r", 10, [this](const Float32::SharedPtr msg) {
        lock_guard lock(mutex); last_rc_override_time = this->get_clock()->now();
        rc_override_msg.chan4_raw = convert_normalized_to_pwm(msg->data);  // Yaw
      });

    // Create timer for regular RC override publishing at 10Hz
    rc_override_timer = rclcpp::create_timer(
      node, node->get_clock(), rclcpp::Duration::from_seconds(0.1),
      std::bind(&RCTwistPlugin::timer_cb, this));
        
    enable_connection_cb();

    if (!uas->is_ardupilotmega() && !uas->is_px4()) {
      RCLCPP_WARN(get_logger(), "RC override not supported by this FCU!");
    }
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }

private:
  using lock_guard = std::lock_guard<std::mutex>;
  std::mutex mutex;

  // Subscribers for twist and individual axes of twist
  rclcpp::Subscription<Twist>::SharedPtr twist_sub;
  rclcpp::Subscription<Float32>::SharedPtr twist_x_sub;
  rclcpp::Subscription<Float32>::SharedPtr twist_y_sub;
  rclcpp::Subscription<Float32>::SharedPtr twist_z_sub;
  rclcpp::Subscription<Float32>::SharedPtr twist_yaw_sub;

  // Timer for regular RC override publishing
  rclcpp::TimerBase::SharedPtr rc_override_timer;
  rclcpp::Time last_rc_override_time;
  double rc_override_timeout;

  mavlink::common::msg::RC_CHANNELS_OVERRIDE rc_override_msg;

  void connection_cb([[maybe_unused]] bool connected) override
  {
    lock_guard lock(mutex);
  }

  /* -*- callbacks -*- */

  void twist_cb(const Twist::SharedPtr req)
  {
    lock_guard lock(mutex); last_rc_override_time = this->get_clock()->now();

    rc_override_msg.chan1_raw = convert_normalized_to_pwm(req->angular.y);  // Pitch
    rc_override_msg.chan2_raw = convert_normalized_to_pwm(req->angular.x);  // Roll
    rc_override_msg.chan3_raw = convert_normalized_to_pwm(req->linear.z);  // Throttle
    rc_override_msg.chan4_raw = convert_normalized_to_pwm(req->angular.z);  // Yaw
    rc_override_msg.chan5_raw = convert_normalized_to_pwm(req->linear.x);  // Forward
    rc_override_msg.chan6_raw = convert_normalized_to_pwm(req->linear.y);  // Lateral
  }

  void timer_cb()
  {
    lock_guard lock(mutex);
    if ((this->get_clock()->now() - last_rc_override_time).seconds() > rc_override_timeout) {
      // If no command received for more than 1 second, reset all channels and return
      initialize_rc_override_msg();
      RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 10000, "Time elapsed: %f seconds", (this->get_clock()->now() - last_rc_override_time).seconds());
      return;
    }
    RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 10000, "Publishing RC override message");
    uas->msg_set_target(rc_override_msg);
    uas->send_message(rc_override_msg);
  }

  uint16_t convert_normalized_to_pwm(double normalized)
  {
    normalized = std::max(-1.0, std::min(1.0, normalized));
    if (abs(normalized) < 0.01) {
      return UINT16_MAX;
    }
    return static_cast<uint16_t>(normalized * 400 + 1500);
  }

  void initialize_rc_override_msg()
  {
    rc_override_msg = {};
    rc_override_msg.chan1_raw = UINT16_MAX;
    rc_override_msg.chan2_raw = UINT16_MAX;
    rc_override_msg.chan3_raw = UINT16_MAX;
    rc_override_msg.chan4_raw = UINT16_MAX;
    rc_override_msg.chan5_raw = UINT16_MAX;
    rc_override_msg.chan6_raw = UINT16_MAX;
    rc_override_msg.chan7_raw = UINT16_MAX;
    rc_override_msg.chan8_raw = UINT16_MAX;
    rc_override_msg.chan9_raw = UINT16_MAX;
    rc_override_msg.chan10_raw = UINT16_MAX;
    rc_override_msg.chan11_raw = UINT16_MAX;
    rc_override_msg.chan12_raw = UINT16_MAX;
    rc_override_msg.chan13_raw = UINT16_MAX;
    rc_override_msg.chan14_raw = UINT16_MAX;
    rc_override_msg.chan15_raw = UINT16_MAX;
    rc_override_msg.chan16_raw = UINT16_MAX;
    rc_override_msg.chan17_raw = UINT16_MAX;
    rc_override_msg.chan18_raw = UINT16_MAX;
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RCTwistPlugin)
