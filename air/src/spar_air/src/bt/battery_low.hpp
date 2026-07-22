#pragma once

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar_air {

// SUCCESS when the drone should head home: low, stale, or missing battery,
// with hysteresis so the tree doesn't flap at the threshold. Battery comes
// from PX4's simulated battery, not a battery_sim copy: PX4 already owns a
// battery model and the BT just reads it.
class BatteryLow : public BT::ConditionNode {
public:
  struct Params {
    double low_percent = 30.0;
    double resume_percent = 90.0;
    double max_age_sec = 2.0;
  };

  BatteryLow(const std::string& name, const BT::NodeConfig& config, Params params)
      : BT::ConditionNode(name, config), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

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

}  // namespace spar_air
