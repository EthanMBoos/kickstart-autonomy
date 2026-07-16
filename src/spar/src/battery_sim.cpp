// Simulated battery for the SPAR Husky. Drains at a fixed rate while
// the robot is out working, recharges when the robot is parked within
// dock_radius of the dock. Publishes sensor_msgs/BatteryState the same way a
// real BMS would, so the autonomy layer can't tell the difference — swap this
// node out on real hardware and nothing above it changes.
//
// For demos: publish a value to battery/set (std_msgs/Float32, percent) to
// jump the battery, e.g. force it low and watch the tree bail out to the dock.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spar {

class BatterySim : public rclcpp::Node {
public:
  BatterySim() : rclcpp::Node("battery_sim") {
    declare_parameter("initial_percent", 100.0);
    declare_parameter("drain_rate_pct_per_s", 0.5);
    declare_parameter("charge_rate_pct_per_s", 5.0);
    declare_parameter("dock_x", 0.0);
    declare_parameter("dock_y", 0.0);
    declare_parameter("dock_radius", 1.5);
    declare_parameter("map_frame", "map");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_rate_hz", 2.0);

    percent_ = get_parameter("initial_percent").as_double();
    drain_rate_ = get_parameter("drain_rate_pct_per_s").as_double();
    charge_rate_ = get_parameter("charge_rate_pct_per_s").as_double();
    dock_x_ = get_parameter("dock_x").as_double();
    dock_y_ = get_parameter("dock_y").as_double();
    dock_radius_ = get_parameter("dock_radius").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    state_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery/state", 10);
    set_sub_ = create_subscription<std_msgs::msg::Float32>(
        "battery/set", 10, [this](const std_msgs::msg::Float32& msg) {
          percent_ = std::clamp(static_cast<double>(msg.data), 0.0, 100.0);
          RCLCPP_INFO(get_logger(), "battery set to %.1f%%", percent_);
        });

    const double rate = get_parameter("publish_rate_hz").as_double();
    timer_ = rclcpp::create_timer(this, get_clock(),
                                  rclcpp::Duration::from_seconds(1.0 / rate),
                                  [this] { update(); });
  }

private:
  bool at_dock() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return false;  // not localized yet — can't be charging
    }
    const double dx = tf.transform.translation.x - dock_x_;
    const double dy = tf.transform.translation.y - dock_y_;
    return std::hypot(dx, dy) <= dock_radius_;
  }

  void update() {
    const rclcpp::Time stamp = now();
    const bool charging = at_dock();
    if (last_update_) {
      const double dt = (stamp - *last_update_).seconds();
      if (dt > 0.0) {
        percent_ += (charging ? charge_rate_ : -drain_rate_) * dt;
        percent_ = std::clamp(percent_, 0.0, 100.0);
      }
    }
    last_update_ = stamp;

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = stamp;
    msg.percentage = static_cast<float>(percent_ / 100.0);
    msg.voltage = static_cast<float>(21.0 + 8.4 * percent_ / 100.0);  // fake 24V pack
    msg.present = true;
    msg.power_supply_status =
        charging ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                 : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    state_pub_->publish(msg);
  }

  double percent_ = 100.0;
  double drain_rate_ = 0.5;
  double charge_rate_ = 5.0;
  double dock_x_ = 0.0, dock_y_ = 0.0, dock_radius_ = 1.5;
  std::string map_frame_, base_frame_;
  std::optional<rclcpp::Time> last_update_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace spar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spar::BatterySim>());
  rclcpp::shutdown();
  return 0;
}
