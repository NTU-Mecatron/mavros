#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <atomic>
#include <mutex>

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;
using mavlink::common::MAV_FRAME;

/**
 * @brief Custom Setpoint velocity plugin
 * @plugin custom_setpoint_velocity
 *
 * Send setpoint velocities to FCU controller via individual Float32 topics.
 */
class RemapSetpointVelocityPlugin : public plugin::Plugin,
  private plugin::SetPositionTargetLocalNEDMixin<RemapSetpointVelocityPlugin>
{
public:
  explicit RemapSetpointVelocityPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "remap_setpoint_velocity"),
    cur_vel_x(0.0f),
    cur_vel_y(0.0f), 
    cur_vel_z(0.0f),
    cur_vel_r(0.0f),
    active_(false)
  {
    enable_node_watch_parameters();

    auto sensor_qos = rclcpp::SensorDataQoS();

    twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      "setpoint_velocity/cmd_vel_unstamped", 10);
      
    vel_x_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/x", sensor_qos, 
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        cur_vel_x.store(msg->data, std::memory_order_release);
        {
          std::lock_guard<std::mutex> lock(time_mutex_);
          last_vel_x_time_ = node->now();
        }
        active_.store(true, std::memory_order_release);
      });

    vel_y_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/y", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        cur_vel_y.store(msg->data, std::memory_order_release);
        {
          std::lock_guard<std::mutex> lock(time_mutex_);
          last_vel_y_time_ = node->now();
        }
        active_.store(true, std::memory_order_release);
      });

    vel_z_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/z", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        cur_vel_z.store(msg->data, std::memory_order_release);
        {
          std::lock_guard<std::mutex> lock(time_mutex_);
          last_vel_z_time_ = node->now();
        }
        active_.store(true, std::memory_order_release);
      });

    vel_r_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      "setpoint_velocity/cmd_vel_unstamped/r", sensor_qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        cur_vel_r.store(msg->data, std::memory_order_release);
        {
          std::lock_guard<std::mutex> lock(time_mutex_);
          last_vel_r_time_ = node->now();
        }
        active_.store(true, std::memory_order_release);
      });

    // Timer for timeout checking
    timeout_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { check_timeout(); });

    // Timer for publishing at 40 Hz
    publish_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(25), // 25ms = 40 Hz
      [this]() { 
        if (active_.load(std::memory_order_acquire)) {
          send_setpoint_velocity();
        }
      });

    auto now = node->now();
    last_vel_x_time_ = now;
    last_vel_y_time_ = now;
    last_vel_z_time_ = now;
    last_vel_r_time_ = now;
  }

  ~RemapSetpointVelocityPlugin() = default;

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_x_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_y_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_z_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_r_sub_;
  
  std::atomic<float> cur_vel_x;
  std::atomic<float> cur_vel_y;
  std::atomic<float> cur_vel_z;
  std::atomic<float> cur_vel_r;

  std::atomic<bool> active_;
  
  // Timeout management (protected by mutex for thread safety)
  std::mutex time_mutex_;
  rclcpp::Time last_vel_x_time_;
  rclcpp::Time last_vel_y_time_;
  rclcpp::Time last_vel_z_time_;
  rclcpp::Time last_vel_r_time_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  static constexpr double COMPONENT_TIMEOUT_ = 0.5;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send combined velocity setpoint to MAVROS
   * 
   * @warning Send only VX VY VZ and RZ
   */
  void send_setpoint_velocity()
  {
    geometry_msgs::msg::Twist twist;

    twist.linear.x = cur_vel_x.load(std::memory_order_acquire);
    twist.linear.y = cur_vel_y.load(std::memory_order_acquire);
    twist.linear.z = cur_vel_z.load(std::memory_order_acquire);
    twist.angular.z = cur_vel_r.load(std::memory_order_acquire);

    twist_pub_->publish(twist);
  }

  /**
   * @brief Check timeout for each component (axis) topics. 
   * For each axis, if there is no new msg in 0.5s, it will set buffer to 0 
   * 
   * @warning Send only VX VY VZ and RZ
   */
  void check_timeout()
  {
    if (!active_.load(std::memory_order_acquire))
      return;

    rclcpp::Time current_time = node->now();
    bool any_active = false;
    {
      std::lock_guard<std::mutex> lock(time_mutex_);
      
      if ((current_time - last_vel_x_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_x.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_y_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_y.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_z_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_z.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
      
      if ((current_time - last_vel_r_time_).seconds() > COMPONENT_TIMEOUT_) {
        cur_vel_r.store(0.0f, std::memory_order_release);
      } else {
        any_active = true;
      }
    }

    if (!any_active) {
      RCLCPP_WARN(get_logger(), "All velocity components timed out - stopping");
      active_.store(false, std::memory_order_release);
    }
  }
};

} // namespace extra_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::RemapSetpointVelocityPlugin)