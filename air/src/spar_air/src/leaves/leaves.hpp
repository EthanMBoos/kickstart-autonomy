#pragma once

// The air track's five leaves, all speaking offboard position setpoints
// through the one OffboardLink. Position feedback comes off the blackboard
// (map ENU, written by the executive from PX4's estimate); the leaves never
// touch a PX4 message directly.

#include <cmath>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/point.hpp>

#include "bt/blackboard_keys.hpp"
#include "bt/stamped.hpp"
#include "offboard_link.hpp"

namespace spar_air {

inline bool getPosition(const BT::NodeConfig& config, Vec3& out) {
  Stamped<Vec3> pos;
  if (!config.blackboard->get<Stamped<Vec3>>(keys::kPosition, pos)) return false;
  out = pos.value;
  return true;
}

inline double nowSec(const BT::NodeConfig& config) {
  double now = 0.0;
  (void)config.blackboard->get<double>(keys::kNowSec, now);
  return now;
}

// Arm, switch to offboard, climb to cruise over the current spot. Doubles
// as the "airborne?" gate in front of GotoWaypoint: once aloft it returns
// SUCCESS immediately every tick.
class TakeOff : public BT::StatefulActionNode {
public:
  struct Params {
    double cruise_alt_m = 4.0;
    double aloft_alt_m = 1.5;
  };

  TakeOff(const std::string& name, const BT::NodeConfig& config,
          OffboardLink& link, Params params)
      : BT::StatefulActionNode(name, config), link_(link), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    Vec3 pos;
    if (!getPosition(config(), pos)) return BT::NodeStatus::RUNNING;
    if (pos.z >= params_.aloft_alt_m) return BT::NodeStatus::SUCCESS;
    link_.setTarget(pos.x, pos.y, params_.cruise_alt_m, 0.0);
    next_command_ = 0.0;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    Vec3 pos;
    if (!getPosition(config(), pos)) return BT::NodeStatus::RUNNING;
    if (pos.z >= params_.aloft_alt_m) return BT::NodeStatus::SUCCESS;
    // Re-send arm + offboard until they stick: PX4 refuses the mode until
    // the setpoint stream has been up for a moment, so one shot is a race.
    // The target rides along in case onStart ran before the first position
    // fix (mission started before PX4) and never got to send one.
    bool armed = false;
    (void)config().blackboard->get<bool>(keys::kArmed, armed);
    const double now = nowSec(config());
    if (!armed && now >= next_command_) {
      next_command_ = now + 1.0;
      link_.setTarget(pos.x, pos.y, params_.cruise_alt_m, 0.0);
      link_.offboardMode();
      link_.arm();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  OffboardLink& link_;
  Params params_;
  double next_command_ = 0.0;
};

// Cycle the patrol waypoints at cruise altitude forever (the battery
// branch is what ends a mission).
class GotoWaypoint : public BT::StatefulActionNode {
public:
  struct Waypoint {
    double x = 0, y = 0;
  };
  struct Params {
    double cruise_alt_m = 4.0;
    double accept_radius_m = 0.6;
  };

  GotoWaypoint(const std::string& name, const BT::NodeConfig& config,
               OffboardLink& link, std::vector<Waypoint> waypoints, Params params)
      : BT::StatefulActionNode(name, config),
        link_(link),
        waypoints_(std::move(waypoints)),
        params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    aim();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    Vec3 pos;
    if (getPosition(config(), pos)) {
      const auto& wp = waypoints_[index_];
      if (std::hypot(pos.x - wp.x, pos.y - wp.y) < params_.accept_radius_m) {
        index_ = (index_ + 1) % waypoints_.size();
        aim();
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  void aim() {
    const auto& wp = waypoints_[index_];
    Vec3 pos;
    // Face the direction of travel; the camera looks down-forward.
    double yaw = 0.0;
    if (getPosition(config(), pos)) yaw = std::atan2(wp.y - pos.y, wp.x - pos.x);
    link_.setTarget(wp.x, wp.y, params_.cruise_alt_m, yaw);
  }

  OffboardLink& link_;
  std::vector<Waypoint> waypoints_;
  Params params_;
  size_t index_ = 0;
};

// One camera-inward orbit around the detection, then stamp the cooldown.
class InspectAnomaly : public BT::StatefulActionNode {
public:
  struct Params {
    double orbit_radius_m = 2.0;
    double orbit_alt_m = 3.0;
    double orbit_rate_rad_s = 0.4;
  };

  InspectAnomaly(const std::string& name, const BT::NodeConfig& config,
                 OffboardLink& link, Params params)
      : BT::StatefulActionNode(name, config), link_(link), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    Stamped<geometry_msgs::msg::Point> anomaly;
    if (!config().blackboard->get<Stamped<geometry_msgs::msg::Point>>(
            keys::kAnomalyPoint, anomaly)) {
      return BT::NodeStatus::FAILURE;
    }
    center_ = {anomaly.value.x, anomaly.value.y, 0.0};
    Vec3 pos;
    angle_start_ = getPosition(config(), pos)
                       ? std::atan2(pos.y - center_.y, pos.x - center_.x)
                       : 0.0;
    swept_ = 0.0;
    last_tick_ = nowSec(config());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const double now = nowSec(config());
    swept_ += params_.orbit_rate_rad_s * (now - last_tick_);
    last_tick_ = now;
    if (swept_ >= 2.0 * M_PI) {
      config().blackboard->set<double>(keys::kLastInspected, now);
      return BT::NodeStatus::SUCCESS;
    }
    const double a = angle_start_ + swept_;
    const double x = center_.x + params_.orbit_radius_m * std::cos(a);
    const double y = center_.y + params_.orbit_radius_m * std::sin(a);
    // Yaw faces the center: the camera keeps the anomaly in frame.
    link_.setTarget(x, y, params_.orbit_alt_m, std::atan2(center_.y - y, center_.x - x));
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  OffboardLink& link_;
  Params params_;
  Vec3 center_;
  double angle_start_ = 0.0, swept_ = 0.0, last_tick_ = 0.0;
};

// Fly home over the pad at cruise altitude.
class ReturnToPad : public BT::StatefulActionNode {
public:
  struct Params {
    double pad_x = 0, pad_y = 0;
    double cruise_alt_m = 4.0;
    double accept_radius_m = 0.6;
  };

  ReturnToPad(const std::string& name, const BT::NodeConfig& config,
              OffboardLink& link, Params params)
      : BT::StatefulActionNode(name, config), link_(link), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    Vec3 pos;
    double yaw = 0.0;
    if (getPosition(config(), pos)) {
      yaw = std::atan2(params_.pad_y - pos.y, params_.pad_x - pos.x);
    }
    link_.setTarget(params_.pad_x, params_.pad_y, params_.cruise_alt_m, yaw);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    Vec3 pos;
    if (!getPosition(config(), pos)) return BT::NodeStatus::RUNNING;
    return std::hypot(pos.x - params_.pad_x, pos.y - params_.pad_y) <
                   params_.accept_radius_m
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  OffboardLink& link_;
  Params params_;
};

// Hand the landing to PX4 (NAV_LAND descends and auto-disarms), with one
// guard: PX4's land detector is occasionally blind to a MuJoCo touchdown
// (contact micro-jitter), so once the drone has demonstrably been on the
// ground for a while and is still armed, force-disarm it. Stays RUNNING
// after disarm on purpose: PX4's simulated battery refills on disarm and
// the battery branch releases on its own once it reads full again.
class Land : public BT::StatefulActionNode {
public:
  struct Params {
    double grounded_alt_m = 0.3;
    double force_disarm_after_sec = 15.0;
  };

  Land(const std::string& name, const BT::NodeConfig& config, OffboardLink& link,
       Params params)
      : BT::StatefulActionNode(name, config), link_(link), params_(params) {}
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
    link_.landCmd();
    next_force_ = nowSec(config()) + params_.force_disarm_after_sec;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    bool armed = false;
    (void)config().blackboard->get<bool>(keys::kArmed, armed);
    Vec3 pos;
    const double now = nowSec(config());
    if (armed && getPosition(config(), pos) && pos.z < params_.grounded_alt_m &&
        now >= next_force_) {
      next_force_ = now + 2.0;
      link_.disarmForce();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  OffboardLink& link_;
  Params params_;
  double next_force_ = 0.0;
};

}  // namespace spar_air
