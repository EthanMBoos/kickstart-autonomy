// The behavior layer for the SPAR Husky: registers this package's node
// types with a BehaviorTree.CPP factory and ticks the tree loaded from
// behavior_trees/main_tree.xml (that file has the tree's shape and the
// reasoning for it). Sits above Nav2 (planner/controller) and below the
// mission layer. The robot boots idle; the mission arrives as a command on
// mission/command ("start"/"stop" — see scripts/mission.sh), and stopping
// mid-run preempts whatever is active and cancels its Nav2 goal. Ticks at
// ~10 Hz, following /clock so the tree pauses with the sim.

#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <spar/msg/detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "bt/anomaly_seen.hpp"
#include "bt/battery_low.hpp"
#include "bt/blackboard_keys.hpp"
#include "bt/idle.hpp"
#include "bt/mission_active.hpp"
#include "bt/stamped.hpp"
#include "leaves/navigate_leaf.hpp"

namespace spar {

class BtExecutive : public rclcpp::Node {
public:
  BtExecutive() : rclcpp::Node("bt_executive") {
    declare_parameter("tick_rate_hz", 10.0);
    declare_parameter("goal_frame", "map");
    declare_parameter("cmd_vel_topic", "cmd_vel");
    declare_parameter("waypoints_x", std::vector<double>{});
    declare_parameter("waypoints_y", std::vector<double>{});
    declare_parameter("waypoints_yaw", std::vector<double>{});
    declare_parameter("dock_x", 0.0);
    declare_parameter("dock_y", 0.0);
    declare_parameter("dock_yaw", 0.0);
    declare_parameter("battery_low_percent", 30.0);
    declare_parameter("battery_resume_percent", 90.0);
    declare_parameter("battery_stale_sec", 2.0);
    declare_parameter("nav_retry_cooldown_sec", 2.0);
    declare_parameter("waypoint_max_retries", 3);
    declare_parameter("base_frame", "base_link");
    declare_parameter("anomaly_stale_sec", 2.0);
    declare_parameter("inspect_cooldown_sec", 30.0);
    // A detector can report more than one kind of thing on
    // perception/detections (a VLM or segmentation model, say); this is the
    // one label AnomalySeen filters for.
    declare_parameter("anomaly_label", "anomaly");
    declare_parameter("inspect_standoff_m", 1.2);
    // The robot boots idle and waits to be told. Set true to skip the wait
    // (the smoke test and impatient demos).
    declare_parameter("autostart_mission", false);
    // Overridable so a student can point at an alternate tree without a
    // rebuild; defaults to the one this package ships.
    declare_parameter("bt_xml_path",
        ament_index_cpp::get_package_share_directory("spar") +
        "/behavior_trees/main_tree.xml");
  }

  // Building the tree needs `*this` fully constructed, so it lives here
  // rather than in the constructor.
  void init() {
    BT::BehaviorTreeFactory factory;

    // Conditions with no tunables at all: zero-arg registration.
    factory.registerNodeType<MissionActive>("MissionActive");
    factory.registerNodeType<Idle>("Idle");

    BatteryLow::Params battery_params;
    battery_params.low_percent = get_parameter("battery_low_percent").as_double();
    battery_params.resume_percent = get_parameter("battery_resume_percent").as_double();
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

    const auto goal_frame = get_parameter("goal_frame").as_string();
    const auto base_frame = get_parameter("base_frame").as_string();
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto cooldown = get_parameter("nav_retry_cooldown_sec").as_double();

    auto xs = get_parameter("waypoints_x").as_double_array();
    auto ys = get_parameter("waypoints_y").as_double_array();
    auto yaws = get_parameter("waypoints_yaw").as_double_array();
    if (xs.size() != ys.size() || xs.size() != yaws.size()) {
      throw std::runtime_error("waypoints_x/y/yaw must have equal lengths");
    }
    std::vector<RoundsLeaf::Waypoint> waypoints;
    for (size_t i = 0; i < xs.size(); ++i) {
      waypoints.push_back({xs[i], ys[i], yaws[i]});
    }
    const int max_retries =
        static_cast<int>(get_parameter("waypoint_max_retries").as_int());

    RoundsLeaf::Waypoint dock{get_parameter("dock_x").as_double(),
                              get_parameter("dock_y").as_double(),
                              get_parameter("dock_yaw").as_double()};

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    InspectLeaf::Params inspect_params;
    inspect_params.standoff_m = get_parameter("inspect_standoff_m").as_double();

    factory.registerBuilder<RoundsLeaf>(
        "Rounds",
        [this, waypoints, goal_frame, cooldown, max_retries](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<RoundsLeaf>(name, config, *this, waypoints,
                                              goal_frame, cooldown, max_retries);
        });
    factory.registerBuilder<DockLeaf>(
        "ReturnToDock",
        [this, dock, goal_frame, cooldown](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<DockLeaf>(name, config, *this, dock,
                                            goal_frame, cooldown);
        });
    factory.registerBuilder<InspectLeaf>(
        "Inspect",
        [this, goal_frame, base_frame, cooldown, inspect_params](
            const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<InspectLeaf>(name, config, *this, *tf_buffer_,
                                               goal_frame, base_frame, cooldown,
                                               inspect_params);
        });
    factory.registerBuilder<HoldLeaf>(
        "HoldPosition",
        [this, cmd_vel_topic](const std::string& name, const BT::NodeConfig& config) {
          return std::make_unique<HoldLeaf>(name, config, *this, cmd_vel_topic);
        });

    tree_ = factory.createTreeFromFile(get_parameter("bt_xml_path").as_string());
    auto blackboard = tree_.rootBlackboard();

    // The mission layer's single knob. One writer per key: this and
    // mission_sub_'s callback below.
    blackboard->set<bool>(keys::kMissionActive,
                          get_parameter("autostart_mission").as_bool());

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        "battery/state", 10,
        [this, blackboard](const sensor_msgs::msg::BatteryState& msg) {
          // BatteryState.percentage is 0..1; the blackboard stores 0..100.
          blackboard->set<Stamped<double>>(
              keys::kBatteryPercent, {msg.percentage * 100.0, now().seconds()});
        });

    // The detector may report several kinds of things on this topic; only
    // hits labeled anomaly_label feed AnomalySeen, filtered here rather than
    // trusting everything that arrives.
    const auto anomaly_label = get_parameter("anomaly_label").as_string();
    detections_sub_ = create_subscription<spar::msg::Detection>(
        "perception/detections", 10,
        [this, blackboard, anomaly_label](const spar::msg::Detection& msg) {
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

    // Sim-time timer: ticks follow /clock, so the tree pauses with the sim.
    const double tick_rate = get_parameter("tick_rate_hz").as_double();
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / tick_rate),
        [this] { tick_once(); });

    RCLCPP_INFO(get_logger(), "bt_executive up: ticking at %.1f Hz", tick_rate);
  }

private:
  void tick_once() {
    auto blackboard = tree_.rootBlackboard();
    // The clock every staleness check in this tree compares against — see
    // bt/stamped.hpp for why this isn't BT.CPP's own blackboard timestamp.
    blackboard->set<double>(keys::kNowSec, now().seconds());
    // tickExactlyOnce(), not tickOnce(): tickOnce() re-ticks internally if
    // any node called emitWakeUpSignal(), which none of ours do, but the
    // timer callback below is already the single source of tick timing —
    // this should never loop on its own.
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

  // The tree itself already knows which leaf is running; no leaf needs to
  // announce its own name onto the blackboard (that was hand-maintained
  // bookkeeping that could silently drift from the XML). Exactly one action
  // node is non-IDLE after a full tick — composites halt every branch they
  // don't select, and StatefulActionNode resets to IDLE on halt — so this
  // finds it and reports it by its registration ID, the same name used in
  // main_tree.xml and factory.registerBuilder/registerNodeType.
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<spar::msg::Detection>::SharedPtr detections_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace spar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<spar::BtExecutive>();
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
