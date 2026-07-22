#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>

namespace spar_ground {

// The do-nothing leaf at the very bottom of the tree: no goals, no
// commands, just RUNNING. With every Nav2 goal already cancelled by
// onHalted() on the way down, publishing nothing is what leaves the robot
// parked.
class Idle : public BT::StatefulActionNode {
public:
  Idle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  // No ports; the factory requires this method to exist even so.
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override { return onRunning(); }

  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }

  void onHalted() override {}
};

}  // namespace spar_ground
