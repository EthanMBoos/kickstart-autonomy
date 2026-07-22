#pragma once

namespace spar_ground {

// A blackboard value paired with the clock time it was written. BT.CPP's own
// Blackboard has a native per-key timestamp too, but it stamps with
// steady_clock, which is blind to /clock and would silently measure
// wall-clock deltas under use_sim_time (this repo runs with it on
// everywhere). Stamp with the same clock you compare against instead: every
// writer here uses the ROS/sim time, and so does the freshness check below.
template <typename T>
struct Stamped {
  T value{};
  double stamp = -1.0;  // negative: never written
};

// True when a Stamped<T> exists and is no older than max_age_sec as of now.
template <typename T>
bool fresh(const Stamped<T>& s, double now, double max_age_sec) {
  return s.stamp >= 0.0 && now - s.stamp <= max_age_sec;
}

}  // namespace spar_ground
