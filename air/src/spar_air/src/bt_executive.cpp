// The behavior layer for the SPAR Skydio: registers this package's node
// types with a BehaviorTree.CPP factory and ticks the tree from
// behavior_trees/air_tree.xml. Sits above PX4 (attitude/position control,
// EKF2, failsafes) and below the mission layer, mirroring the ground
// executive's conventions: boots idle, mission arrives on mission/command
// ("start"/"stop"), status is the same JSON shape on bt/status, ticks
// follow /clock. PX4's own failsafes stay armed but thresholded below this
// tree's (see the yaml), so the BT always decides first.

#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spar_air/msg/detection.hpp>
#include <std_msgs/msg/string.hpp>

#include "bt/anomaly_seen.hpp"
#include "bt/battery_low.hpp"
#include "bt/blackboard_keys.hpp"
#include "bt/idle.hpp"
#include "bt/mission_active.hpp"
#include "bt/stamped.hpp"
#include "frames.hpp"
#include "leaves/leaves.hpp"
#include "offboard_link.hpp"

namespace spar_air {

class BtExecutive : public rclcpp::Node {
public:
  BtExecutive() : rclcpp::Node("bt_executive") {
    declare_parameter("tick_rate_hz", 10.0);
    declare_parameter("waypoints_x", std::vector<double>{});
    declare_parameter("waypoints_y", std::vector<double>{});
    declare_parameter("pad_x", 0.0);
    declare_parameter("pad_y", 0.0);
    declare_parameter("cruise_alt_m", 4.0);
    declare_parameter("aloft_alt_m", 1.5);
    declare_parameter("accept_radius_m", 0.6);
    declare_parameter("orbit_radius_m", 2.0);
    declare_parameter("orbit_alt_m", 3.0);
    declare_parameter("orbit_rate_rad_s", 0.4);
    declare_parameter("battery_low_percent", 30.0);
    declare_parameter("battery_resume_percent", 90.0);
    declare_parameter("battery_stale_sec", 2.0);
    declare_parameter("anomaly_stale_sec", 6.0);
    declare_parameter("inspect_cooldown_sec", 45.0);
    declare_parameter("anomaly_label", "anomaly");
    declare_parameter("autostart_mission", false);
    declare_parameter("bt_xml_path",
        ament_index_cpp::get_package_share_directory("spar_air") +
        "/behavior_trees/air_tree.xml");
  }

  // Building the tree needs `*this` fully constructed, so it lives here.
  void init() {
    const double pad_x = get_parameter("pad_x").as_double();
    const double pad_y = get_parameter("pad_y").as_double();
    link_ = std::make_unique<OffboardLink>(*this, pad_x, pad_y);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MissionActive>("MissionActive");
    factory.registerNodeType<Idle>("Idle");

    BatteryLow::Params battery_params;
    battery_params.low_percent = get_parameter("battery_low_percent").as_double();
    battery_params.resume_percent =
        get_parameter("battery_resume_percent").as_double();
    battery_params.max_age_sec = get_parameter("battery_stale_sec").as_double();
    factory.registerBuilder<BatteryLow>(
        "BatteryLow",
        [battery_params](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<BatteryLow>(name, config, battery_params);
        });

    AnomalySeen::Params anomaly_params;
    anomaly_params.max_age_sec = get_parameter("anomaly_stale_sec").as_double();
    anomaly_params.cooldown_sec = get_parameter("inspect_cooldown_sec").as_double();
    factory.registerBuilder<AnomalySeen>(
        "AnomalySeen",
        [anomaly_params](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<AnomalySeen>(name, config, anomaly_params);
        });

    TakeOff::Params takeoff_params;
    takeoff_params.cruise_alt_m = get_parameter("cruise_alt_m").as_double();
    takeoff_params.aloft_alt_m = get_parameter("aloft_alt_m").as_double();
    factory.registerBuilder<TakeOff>(
        "TakeOff",
        [this, takeoff_params](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<TakeOff>(name, config, *link_, takeoff_params);
        });

    auto xs = get_parameter("waypoints_x").as_double_array();
    auto ys = get_parameter("waypoints_y").as_double_array();
    if (xs.size() != ys.size()) {
      throw std::runtime_error("waypoints_x/y must have equal lengths");
    }
    std::vector<GotoWaypoint::Waypoint> waypoints;
    for (size_t i = 0; i < xs.size(); ++i) waypoints.push_back({xs[i], ys[i]});
    GotoWaypoint::Params goto_params;
    goto_params.cruise_alt_m = takeoff_params.cruise_alt_m;
    goto_params.accept_radius_m = get_parameter("accept_radius_m").as_double();
    factory.registerBuilder<GotoWaypoint>(
        "GotoWaypoint",
        [this, waypoints, goto_params](const std::string& name,
                                       const BT::NodeConfig& config) {
          return std::make_unique<GotoWaypoint>(name, config, *link_, waypoints,
                                                goto_params);
        });

    InspectAnomaly::Params inspect_params;
    inspect_params.orbit_radius_m = get_parameter("orbit_radius_m").as_double();
    inspect_params.orbit_alt_m = get_parameter("orbit_alt_m").as_double();
    inspect_params.orbit_rate_rad_s = get_parameter("orbit_rate_rad_s").as_double();
    factory.registerBuilder<InspectAnomaly>(
        "InspectAnomaly",
        [this, inspect_params](const std::string& name,
                               const BT::NodeConfig& config) {
          return std::make_unique<InspectAnomaly>(name, config, *link_,
                                                  inspect_params);
        });

    ReturnToPad::Params pad_params;
    pad_params.pad_x = pad_x;
    pad_params.pad_y = pad_y;
    pad_params.cruise_alt_m = takeoff_params.cruise_alt_m;
    pad_params.accept_radius_m = goto_params.accept_radius_m;
    factory.registerBuilder<ReturnToPad>(
        "ReturnToPad",
        [this, pad_params](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<ReturnToPad>(name, config, *link_, pad_params);
        });

    Land::Params land_params;  // defaults; not yaml knobs until someone needs them
    factory.registerBuilder<Land>(
        "Land",
        [this, land_params](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<Land>(name, config, *link_, land_params);
        });

    tree_ = factory.createTreeFromFile(get_parameter("bt_xml_path").as_string());
    auto blackboard = tree_.rootBlackboard();
    blackboard->set<bool>(keys::kMissionActive,
                          get_parameter("autostart_mission").as_bool());

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
        [this, blackboard, pad_x, pad_y](
            const px4_msgs::msg::VehicleLocalPosition& msg) {
          // EKF local NED -> map ENU; the EKF origin is the pad (spawn).
          auto enu = nedToEnu(msg.x, msg.y, msg.z);
          blackboard->set<Stamped<Vec3>>(
              keys::kPosition,
              {{enu.x + pad_x, enu.y + pad_y, enu.z}, now().seconds()});
        });

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
        [blackboard](const px4_msgs::msg::VehicleStatus& msg) {
          blackboard->set<bool>(keys::kArmed, msg.arming_state == 2);
        });

    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
        "fmu/out/battery_status", rclcpp::SensorDataQoS(),
        [this, blackboard](const px4_msgs::msg::BatteryStatus& msg) {
          blackboard->set<Stamped<double>>(
              keys::kBatteryPercent, {msg.remaining * 100.0, now().seconds()});
        });

    const auto anomaly_label = get_parameter("anomaly_label").as_string();
    detections_sub_ = create_subscription<spar_air::msg::Detection>(
        "perception/detections", 10,
        [this, blackboard, anomaly_label](const spar_air::msg::Detection& msg) {
          if (msg.label != anomaly_label) return;
          blackboard->set<Stamped<geometry_msgs::msg::Point>>(
              keys::kAnomalyPoint, {msg.point, now().seconds()});
        });

    mission_sub_ = create_subscription<std_msgs::msg::String>(
        "mission/command", 10,
        [this, blackboard](const std_msgs::msg::String& msg) {
          if (msg.data == "start" || msg.data == "stop") {
            blackboard->set<bool>(keys::kMissionActive, msg.data == "start");
            RCLCPP_INFO(get_logger(), "mission command: %s", msg.data.c_str());
          } else {
            RCLCPP_WARN(get_logger(),
                        "unknown mission command '%s' (want start|stop)",
                        msg.data.c_str());
          }
        });

    status_pub_ = create_publisher<std_msgs::msg::String>("bt/status", 10);

    const double tick_rate = get_parameter("tick_rate_hz").as_double();
    timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0 / tick_rate),
        [this] { tick_once(); });

    RCLCPP_INFO(get_logger(), "air bt_executive up: ticking at %.1f Hz", tick_rate);
  }

private:
  void tick_once() {
    auto blackboard = tree_.rootBlackboard();
    blackboard->set<double>(keys::kNowSec, now().seconds());
    const BT::NodeStatus root_status = tree_.tickExactlyOnce();

    double now_sec = 0.0;
    (void)blackboard->get<double>(keys::kNowSec, now_sec);
    Stamped<double> battery;
    const bool have_battery =
        blackboard->get<Stamped<double>>(keys::kBatteryPercent, battery);
    Stamped<geometry_msgs::msg::Point> anomaly;
    const bool have_anomaly = blackboard->get<Stamped<geometry_msgs::msg::Point>>(
        keys::kAnomalyPoint, anomaly);
    bool mission = false;
    (void)blackboard->get<bool>(keys::kMissionActive, mission);

    // Same JSON shape as the ground executive: the smoke test and any
    // dashboard read both robots the same way.
    std_msgs::msg::String status;
    std::ostringstream out;
    out << "{\"root_status\":\"" << BT::toStr(root_status) << "\""
        << ",\"mission_active\":" << (mission ? "true" : "false")
        << ",\"anomaly_age_sec\":"
        << (have_anomaly ? std::to_string(now_sec - anomaly.stamp) : "null")
        << ",\"active_leaf\":\"" << active_leaf() << "\""
        << ",\"battery_percent\":"
        << (have_battery ? std::to_string(battery.value) : "null")
        << ",\"battery_age_sec\":"
        << (have_battery ? std::to_string(now_sec - battery.stamp) : "null")
        << "}";
    status.data = out.str();
    status_pub_->publish(status);
  }

  std::string active_leaf() const {
    std::string active = "none";
    tree_.applyVisitor([&](const BT::TreeNode* node) {
      if (node->type() == BT::NodeType::ACTION &&
          node->status() != BT::NodeStatus::IDLE) {
        active = node->registrationName();
      }
    });
    return active;
  }

  BT::Tree tree_;
  std::unique_ptr<OffboardLink> link_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<spar_air::msg::Detection>::SharedPtr detections_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace spar_air

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<spar_air::BtExecutive>();
    node->init();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
