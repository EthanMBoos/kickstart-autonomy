#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar {

// Condition node: SUCCESS when perception has a fresh anomaly fix (older
// than max_age_sec fails) and no inspection ended within the last
// cooldown_sec (Inspect stamps keys::kLastInspected, so the robot goes back
// to its rounds instead of orbiting the same object forever).
//
// TODO(multi-detection): keys::kAnomalyPoint holds exactly one point, and a
// detector that reports several things in the same frame will just
// overwrite it — whichever hit arrives last on perception/detections wins,
// arbitrarily, and cooldown_sec is a single global timestamp, so inspecting
// one object silences every other one too. If you hook up a model that
// finds multiple things at once (two drums, a person and a cone, whatever),
// think about what "handle both" should mean here: kAnomalyPoint would need
// to become a small collection instead of one slot, and cooldown would need
// to key off which point was actually visited, not just when.
class AnomalySeen : public BT::ConditionNode {
public:
  struct Params {
    double max_age_sec = 2.0;
    double cooldown_sec = 30.0;
  };

  AnomalySeen(const std::string& name, const BT::NodeConfig& config, Params params)
      : BT::ConditionNode(name, config), params_(params) {}

  BT::NodeStatus tick() override {
    double now = 0.0;
    (void)config().blackboard->get<double>(keys::kNowSec, now);
    Stamped<geometry_msgs::msg::Point> anomaly;
    const bool have = config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
        keys::kAnomalyPoint, anomaly);
    if (!have || !fresh(anomaly, now, params_.max_age_sec)) {
      return BT::NodeStatus::FAILURE;
    }
    double last_inspected = -1.0;  // -1: never inspected
    (void)config().blackboard->get<double>(keys::kLastInspected, last_inspected);
    if (last_inspected >= 0.0 && now - last_inspected < params_.cooldown_sec) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  Params params_;
};

}  // namespace spar
