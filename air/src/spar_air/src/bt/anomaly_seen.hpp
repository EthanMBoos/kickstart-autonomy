#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/point.hpp>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar_air {

// SUCCESS when there is a fresh detection and the last inspection was long
// enough ago (cooldown, so one drum doesn't trap the drone in orbit).
class AnomalySeen : public BT::ConditionNode {
public:
  struct Params {
    double max_age_sec = 2.0;
    double cooldown_sec = 30.0;
  };

  AnomalySeen(const std::string& name, const BT::NodeConfig& config, Params params)
      : BT::ConditionNode(name, config), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    double now = 0.0;
    (void)config().blackboard->get<double>(keys::kNowSec, now);
    double last = -1.0;
    (void)config().blackboard->get<double>(keys::kLastInspected, last);
    if (last >= 0.0 && now - last < params_.cooldown_sec) {
      return BT::NodeStatus::FAILURE;
    }
    Stamped<geometry_msgs::msg::Point> anomaly;
    const bool have = config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
        keys::kAnomalyPoint, anomaly);
    return (have && fresh(anomaly, now, params_.max_age_sec))
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }

private:
  Params params_;
};

}  // namespace spar_air
