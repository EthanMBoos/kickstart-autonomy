#pragma once

namespace spar_air::keys {

// Blackboard key names, in one place so a typo in one file can't silently
// desync from another.
inline constexpr const char* kMissionActive = "mission_active";
inline constexpr const char* kBatteryPercent = "battery_percent";
inline constexpr const char* kAnomalyPoint = "anomaly_point";
inline constexpr const char* kLastInspected = "last_inspected";
inline constexpr const char* kNowSec = "now_sec";
inline constexpr const char* kPosition = "position";  // Stamped<Vec3>, map ENU
inline constexpr const char* kArmed = "armed";

}  // namespace spar_air::keys
