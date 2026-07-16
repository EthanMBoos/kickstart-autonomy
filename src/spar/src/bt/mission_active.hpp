#pragma once

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"

namespace spar {

// Condition node: SUCCESS while the mission layer has told the robot to
// run. This is the seam between mission and behavior: the tree does
// nothing until something above it flips the flag. A missing flag means no
// mission: a robot that has never been commanded stays put.
class MissionActive : public BT::ConditionNode {
public:
  MissionActive(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}

  // No ports; the factory requires this method to exist even so.
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    bool active = false;
    (void)config().blackboard->get<bool>(keys::kMissionActive, active);
    return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace spar
