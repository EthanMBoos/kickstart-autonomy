#pragma once

namespace spar_air {

// A blackboard value paired with the clock time it was written, stamped
// with ROS/sim time so freshness checks follow /clock (same reasoning as
// the ground package's copy; the two BT packages share conventions, not
// code).
template <typename T>
struct Stamped {
  T value{};
  double stamp = -1.0;  // negative: never written
};

template <typename T>
bool fresh(const Stamped<T>& s, double now, double max_age_sec) {
  return s.stamp >= 0.0 && now - s.stamp <= max_age_sec;
}

}  // namespace spar_air
