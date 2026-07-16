#pragma once

namespace spar::keys {

// Blackboard key names, in one place so a typo in one file can't silently
// desync from another. Every writer/reader in this package uses these.
inline constexpr const char* kMissionActive = "mission_active";
inline constexpr const char* kBatteryPercent = "battery_percent";
inline constexpr const char* kAnomalyPoint = "anomaly_point";
inline constexpr const char* kLastInspected = "last_inspected";
inline constexpr const char* kNowSec = "now_sec";

}  // namespace spar::keys
