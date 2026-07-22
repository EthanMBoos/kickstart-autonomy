#pragma once

#include <behaviortree_cpp/condition_node.h>

#include "bt/blackboard_keys.hpp"

namespace spar_air {

// SUCCESS while the mission layer says fly (mission/command "start").
class MissionActive : public BT::ConditionNode {
public:
  MissionActive(const std::string& name, const BT::NodeConfig& config)
      : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    bool active = false;
    (void)config().blackboard->get<bool>(keys::kMissionActive, active);
    return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace spar_air
