#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>

#include <behaviortree_cpp/action_node.h>

namespace spar {

// Base for every leaf that delegates motion to Nav2 (the planner/controller
// for this stack). The one rule that keeps the tree reactive: onStart()/
// onRunning() never wait on Nav2. A goal is sent asynchronously, RUNNING is
// returned, and the result is polled on later ticks. onHalted() cancels the
// in-flight goal so a preempted behavior stops feeding the controller a
// stale trajectory. StatefulActionNode's base halt() only calls onHalted()
// while the node is genuinely RUNNING, so there is no need to guard against
// a spurious halt while idle here.
class NavigateLeaf : public BT::StatefulActionNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // hold_success_sec: after a goal succeeds, keep reporting SUCCESS for this
  // long instead of immediately re-sending the same goal. Zero disables the
  // hold (Rounds wants the next waypoint right away; ReturnToDock does not
  // need to re-navigate to a dock it is already parked on every tick).
  NavigateLeaf(const std::string& name, const BT::NodeConfig& config,
               rclcpp::Node& node, double retry_cooldown_sec,
               double hold_success_sec = 0.0);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  // Subclasses supply the pose to navigate to. nullopt fails the tick.
  virtual std::optional<geometry_msgs::msg::PoseStamped> next_goal() = 0;
  // Called once when a goal finishes (not when preempted by onHalted()).
  virtual void on_result(bool succeeded) { (void)succeeded; }

  rclcpp::Node& node_;

private:
  void send_goal(const geometry_msgs::msg::PoseStamped& pose);
  void reset_goal_state();
  bool in_cooldown() const;
  bool in_success_hold() const;

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::optional<rclcpp_action::ResultCode> result_;
  // Bumped whenever this leaf abandons a goal, so late callbacks from that
  // goal can be recognized and ignored.
  uint64_t goal_seq_ = 0;
  std::optional<rclcpp::Time> last_failure_;
  std::optional<rclcpp::Time> last_success_;
  rclcpp::Duration retry_cooldown_;
  rclcpp::Duration hold_success_;
};

// The rounds: cycles through a fixed list of checkpoints forever. A waypoint
// that keeps failing is skipped after max_retries so one blocked corner
// doesn't wedge the whole route.
class RoundsLeaf : public NavigateLeaf {
public:
  struct Waypoint {
    double x = 0.0, y = 0.0, yaw = 0.0;
  };

  RoundsLeaf(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node& node, std::vector<Waypoint> waypoints,
             std::string goal_frame, double retry_cooldown_sec, int max_retries);

protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;
  void on_result(bool succeeded) override;

private:
  std::vector<Waypoint> waypoints_;
  std::string goal_frame_;
  size_t index_ = 0;
  int consecutive_failures_ = 0;
  int max_retries_;
};

// Navigates to the fixed dock pose.
class DockLeaf : public NavigateLeaf {
public:
  DockLeaf(const std::string& name, const BT::NodeConfig& config,
           rclcpp::Node& node, RoundsLeaf::Waypoint dock,
           std::string goal_frame, double retry_cooldown_sec);

protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;

private:
  RoundsLeaf::Waypoint dock_;
  std::string goal_frame_;
};

// Drives to a stand-off point near the last anomaly fix on the blackboard
// (perception wrote it; this leaf only reads). Success or failure, it stamps
// keys::kLastInspected so the AnomalySeen? condition backs off and the robot
// returns to its rounds instead of orbiting one object forever. A successful
// inspection publishes the anomaly's position to inspection/findings: the
// robot's work product is a report, and the run logs are the record.
class InspectLeaf : public NavigateLeaf {
public:
  struct Params {
    double standoff_m = 1.2;
  };

  InspectLeaf(const std::string& name, const BT::NodeConfig& config,
              rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
              std::string goal_frame, std::string base_frame,
              double retry_cooldown_sec, Params params);

protected:
  std::optional<geometry_msgs::msg::PoseStamped> next_goal() override;
  void on_result(bool succeeded) override;

private:
  tf2_ros::Buffer& tf_buffer_;
  std::string goal_frame_;
  std::string base_frame_;
  Params params_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr findings_pub_;
};

// The safe default at the bottom of the tree: stream zero velocity so the
// robot stands still instead of coasting on whatever command came last.
// A StatefulActionNode, not a SyncActionNode, because it must report RUNNING
// forever (SyncActionNode throws if tick() ever returns RUNNING).
class HoldLeaf : public BT::StatefulActionNode {
public:
  HoldLeaf(const std::string& name, const BT::NodeConfig& config,
           rclcpp::Node& node, const std::string& cmd_vel_topic);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

}  // namespace spar
