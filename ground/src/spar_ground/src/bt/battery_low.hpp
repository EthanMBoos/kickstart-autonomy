#pragma once

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar_ground {

// Condition node: SUCCESS when the robot should be heading for the dock.
// A missing or stale battery reading counts as low, and once low the
// condition stays low until the battery recharges past resume_percent
// (hysteresis, so the tree doesn't flap at the threshold).
class BatteryLow : public BT::ConditionNode {
public:
  struct Params {
    double low_percent = 30.0;
    double resume_percent = 90.0;
    double max_age_sec = 2.0;
  };

  BatteryLow(const std::string& name, const BT::NodeConfig& config, Params params)
      : BT::ConditionNode(name, config), params_(params) {}

  BT::NodeStatus tick() override {
    double now = 0.0;
    (void)config().blackboard->get<double>(keys::kNowSec, now);
    Stamped<double> reading;
    const bool have =
        config().blackboard->get<Stamped<double>>(keys::kBatteryPercent, reading);
    if (!have || !fresh(reading, now, params_.max_age_sec)) {
      latched_low_ = true;
      return BT::NodeStatus::SUCCESS;
    }
    if (latched_low_) {
      if (reading.value >= params_.resume_percent) latched_low_ = false;
    } else if (reading.value <= params_.low_percent) {
      latched_low_ = true;
    }
    return latched_low_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  Params params_;
  bool latched_low_ = false;
};

}  // namespace spar_ground
