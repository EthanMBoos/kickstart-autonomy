#pragma once

#include <behaviortree_cpp/action_node.h>

namespace spar_air {

// The do-nothing leaf at the very bottom of the tree. The offboard link
// keeps streaming the last setpoint, so a mid-air "stop" freezes the drone
// where it is; on the ground it just stays parked.
class Idle : public BT::StatefulActionNode {
public:
  Idle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
  void onHalted() override {}
};

}  // namespace spar_air
