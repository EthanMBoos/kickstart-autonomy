#include "leaves/navigate_leaf.hpp"

#include <cmath>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"

namespace spar_ground {

namespace {
geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw,
                                          const std::string& frame,
                                          const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}
}  // namespace

NavigateLeaf::NavigateLeaf(const std::string& name, const BT::NodeConfig& config,
                           rclcpp::Node& node, double retry_cooldown_sec,
                           double hold_success_sec)
    : BT::StatefulActionNode(name, config),
      node_(node),
      retry_cooldown_(rclcpp::Duration::from_seconds(retry_cooldown_sec)),
      hold_success_(rclcpp::Duration::from_seconds(hold_success_sec)) {
  client_ = rclcpp_action::create_client<NavigateToPose>(&node, "navigate_to_pose");
}

BT::NodeStatus NavigateLeaf::onStart() {
  if (in_success_hold()) return BT::NodeStatus::SUCCESS;
  if (in_cooldown()) return BT::NodeStatus::FAILURE;
  if (!client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "%s: navigate_to_pose action server not available yet",
                         registrationName().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto goal = next_goal();
  if (!goal) return BT::NodeStatus::FAILURE;
  send_goal(*goal);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateLeaf::onRunning() {
  if (!result_) return BT::NodeStatus::RUNNING;
  const bool succeeded = *result_ == rclcpp_action::ResultCode::SUCCEEDED;
  reset_goal_state();
  if (succeeded) {
    last_success_ = node_.now();
  } else {
    last_failure_ = node_.now();
  }
  on_result(succeeded);
  return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NavigateLeaf::onHalted() {
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
  // Being preempted is not a failure — no cooldown.
  reset_goal_state();
}

void NavigateLeaf::send_goal(const geometry_msgs::msg::PoseStamped& pose) {
  NavigateToPose::Goal goal;
  goal.pose = pose;

  const uint64_t seq = ++goal_seq_;
  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback = [this, seq](GoalHandle::SharedPtr handle) {
    if (seq != goal_seq_) {
      // The leaf already abandoned this goal; if Nav2 accepted it anyway,
      // cancel so nothing keeps driving a behavior the tree gave up on.
      if (handle) client_->async_cancel_goal(handle);
      return;
    }
    if (handle) {
      goal_handle_ = handle;
    } else {
      result_ = rclcpp_action::ResultCode::ABORTED;  // rejected
    }
  };
  options.result_callback = [this, seq](const GoalHandle::WrappedResult& result) {
    if (seq != goal_seq_) return;
    result_ = result.code;
  };

  client_->async_send_goal(goal, options);
  goal_handle_.reset();
  result_.reset();
}

void NavigateLeaf::reset_goal_state() {
  goal_handle_.reset();
  result_.reset();
  ++goal_seq_;
}

bool NavigateLeaf::in_cooldown() const {
  return last_failure_ && (node_.now() - *last_failure_) < retry_cooldown_;
}

bool NavigateLeaf::in_success_hold() const {
  return hold_success_ > rclcpp::Duration(0, 0) && last_success_ &&
         (node_.now() - *last_success_) < hold_success_;
}

RoundsLeaf::RoundsLeaf(const std::string& name, const BT::NodeConfig& config,
                       rclcpp::Node& node, std::vector<Waypoint> waypoints,
                       std::string goal_frame, double retry_cooldown_sec,
                       int max_retries)
    : NavigateLeaf(name, config, node, retry_cooldown_sec),
      waypoints_(std::move(waypoints)),
      goal_frame_(std::move(goal_frame)),
      max_retries_(max_retries) {}

std::optional<geometry_msgs::msg::PoseStamped> RoundsLeaf::next_goal() {
  if (waypoints_.empty()) {
    RCLCPP_ERROR_ONCE(node_.get_logger(), "Rounds: no waypoints configured");
    return std::nullopt;
  }
  const Waypoint& wp = waypoints_[index_];
  return make_pose(wp.x, wp.y, wp.yaw, goal_frame_, node_.now());
}

void RoundsLeaf::on_result(bool succeeded) {
  if (succeeded) {
    consecutive_failures_ = 0;
    index_ = (index_ + 1) % waypoints_.size();
    return;
  }
  if (++consecutive_failures_ >= max_retries_) {
    RCLCPP_WARN(node_.get_logger(),
                "Rounds: waypoint %zu failed %d times, skipping it", index_,
                consecutive_failures_);
    consecutive_failures_ = 0;
    index_ = (index_ + 1) % waypoints_.size();
  }
}

DockLeaf::DockLeaf(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node& node, RoundsLeaf::Waypoint dock,
                   std::string goal_frame, double retry_cooldown_sec)
    : NavigateLeaf(name, config, node, retry_cooldown_sec,
                   /*hold_success_sec=*/5.0),
      dock_(dock),
      goal_frame_(std::move(goal_frame)) {}

std::optional<geometry_msgs::msg::PoseStamped> DockLeaf::next_goal() {
  return make_pose(dock_.x, dock_.y, dock_.yaw, goal_frame_, node_.now());
}

InspectLeaf::InspectLeaf(const std::string& name, const BT::NodeConfig& config,
                         rclcpp::Node& node, tf2_ros::Buffer& tf_buffer,
                         std::string goal_frame, std::string base_frame,
                         double retry_cooldown_sec, Params params)
    : NavigateLeaf(name, config, node, retry_cooldown_sec),
      tf_buffer_(tf_buffer),
      goal_frame_(std::move(goal_frame)),
      base_frame_(std::move(base_frame)),
      params_(std::move(params)) {
  findings_pub_ = node.create_publisher<geometry_msgs::msg::PointStamped>(
      "inspection/findings", 10);
}

std::optional<geometry_msgs::msg::PoseStamped> InspectLeaf::next_goal() {
  Stamped<geometry_msgs::msg::Point> anomaly;
  if (!config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
          keys::kAnomalyPoint, anomaly)) {
    return std::nullopt;
  }

  // Stand off from the anomaly along the robot->anomaly line, facing it.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(goal_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Inspect: no %s->%s transform yet (%s)",
                         goal_frame_.c_str(), base_frame_.c_str(), e.what());
    return std::nullopt;
  }
  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double dx = anomaly.value.x - rx;
  const double dy = anomaly.value.y - ry;
  const double dist = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);
  if (dist <= params_.standoff_m) {
    // Already close enough — just turn in place to face it.
    return make_pose(rx, ry, yaw, goal_frame_, node_.now());
  }
  const double scale = (dist - params_.standoff_m) / dist;
  return make_pose(rx + dx * scale, ry + dy * scale, yaw, goal_frame_,
                   node_.now());
}

void InspectLeaf::on_result(bool succeeded) {
  // Either way this inspection is over: stamp the cooldown so the condition
  // above backs off and the rounds resume. A failed approach retried forever
  // would pin the robot to one corner of the map.
  config().blackboard->set<double>(keys::kLastInspected, node_.now().seconds());

  // A successful inspection produces the work product: a finding at the
  // anomaly's map position. The run logs keep the record.
  Stamped<geometry_msgs::msg::Point> anomaly;
  const bool have = config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
      keys::kAnomalyPoint, anomaly);
  if (!succeeded || !have) return;
  geometry_msgs::msg::PointStamped finding;
  finding.header.frame_id = goal_frame_;
  finding.header.stamp = node_.now();
  finding.point = anomaly.value;
  findings_pub_->publish(finding);
  RCLCPP_INFO(node_.get_logger(), "finding logged: anomaly at (%.1f, %.1f)",
              anomaly.value.x, anomaly.value.y);
}

HoldLeaf::HoldLeaf(const std::string& name, const BT::NodeConfig& config,
                   rclcpp::Node& node, const std::string& cmd_vel_topic)
    : BT::StatefulActionNode(name, config) {
  cmd_pub_ = node.create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
}

BT::NodeStatus HoldLeaf::onStart() { return onRunning(); }

BT::NodeStatus HoldLeaf::onRunning() {
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
  return BT::NodeStatus::RUNNING;
}

}  // namespace spar_ground
