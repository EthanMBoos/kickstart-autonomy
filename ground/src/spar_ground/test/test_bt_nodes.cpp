#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>

#include "bt/anomaly_seen.hpp"
#include "bt/battery_low.hpp"
#include "bt/stamped.hpp"

namespace spar_ground {
namespace {

TEST(Stamped, NeverWrittenIsNeverFresh) {
  Stamped<double> s;
  EXPECT_FALSE(fresh(s, /*now=*/100.0, /*max_age_sec=*/10.0));
}

TEST(Stamped, FreshWithinWindow) {
  Stamped<double> s{50.0, /*stamp=*/100.0};
  EXPECT_TRUE(fresh(s, 102.0, 5.0));
  EXPECT_TRUE(fresh(s, 105.0, 5.0));  // exactly at the edge
}

TEST(Stamped, StaleOutsideWindow) {
  Stamped<double> s{50.0, /*stamp=*/100.0};
  EXPECT_FALSE(fresh(s, 106.0, 5.0));
}

BT::NodeConfig FreshConfig() {
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();
  return config;
}

TEST(BatteryLow, MissingReadingIsLow) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 100.0);
  BatteryLow node("BatteryLow?", config, BatteryLow::Params{});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

TEST(BatteryLow, StaleReadingIsLow) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 100.0);
  config.blackboard->set<Stamped<double>>("battery_percent", {80.0, 50.0});  // 50s old
  BatteryLow node("BatteryLow?", config, BatteryLow::Params{});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

TEST(BatteryLow, HysteresisLatchesAndResumes) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 0.0);
  BatteryLow node("BatteryLow?", config, BatteryLow::Params{});

  config.blackboard->set<Stamped<double>>("battery_percent", {50.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);  // not low yet

  config.blackboard->set<Stamped<double>>("battery_percent", {25.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);  // crosses low_percent

  config.blackboard->set<Stamped<double>>("battery_percent", {50.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);  // latched below resume_percent

  config.blackboard->set<Stamped<double>>("battery_percent", {95.0, 0.0});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);  // recovered past resume_percent
}

TEST(AnomalySeen, StaleAnomalyFails) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 100.0);
  config.blackboard->set<Stamped<geometry_msgs::msg::Point>>(
      "anomaly_point", {geometry_msgs::msg::Point{}, 50.0});  // 50s old
  AnomalySeen node("AnomalySeen?", config, AnomalySeen::Params{});
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
}

TEST(AnomalySeen, CooldownBlocksReinspection) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 100.0);
  config.blackboard->set<Stamped<geometry_msgs::msg::Point>>(
      "anomaly_point", {geometry_msgs::msg::Point{}, 100.0});
  config.blackboard->set<double>("last_inspected", 90.0);  // 10s ago
  AnomalySeen::Params params;
  params.cooldown_sec = 30.0;
  AnomalySeen node("AnomalySeen?", config, params);
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
}

TEST(AnomalySeen, FreshAndPastCooldownSucceeds) {
  auto config = FreshConfig();
  config.blackboard->set<double>("now_sec", 100.0);
  config.blackboard->set<Stamped<geometry_msgs::msg::Point>>(
      "anomaly_point", {geometry_msgs::msg::Point{}, 100.0});
  config.blackboard->set<double>("last_inspected", 50.0);  // 50s ago
  AnomalySeen::Params params;
  params.cooldown_sec = 30.0;
  AnomalySeen node("AnomalySeen?", config, params);
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

}  // namespace
}  // namespace spar_ground
