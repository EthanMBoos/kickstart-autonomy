#!/usr/bin/env bash
# End-to-end smoke test for the air stack. Run inside the air container
# while PX4 SITL (+ its zenoh module) and air.launch.py are up:
#
#   docker exec -it spar-air /ws/scripts/smoke_test_air.sh
#
# Exercises the arc the air track promises: boots idle -> mission start ->
# takeoff -> patrol -> the drum gets inspected (orbit) -> forced low
# battery -> return to pad -> land -> disarm -> the battery refills on
# disarm (PX4's simulated battery does that) and the patrol resumes ->
# stop. Mirrors scripts/smoke_test.sh phase for phase.
source /ws/scripts/env.sh
set -u

PASS=0
FAIL=1
PX4_BIN=/opt/px4/build/px4_sitl_zenoh/bin

log() { echo "[smoke] $*"; }

wait_for() {
  local what="$1" timeout_s="$2" check="$3"
  local deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    if eval "$check" >/dev/null 2>&1; then
      log "OK: $what"
      return 0
    fi
    sleep 2
  done
  log "TIMEOUT after ${timeout_s}s: $what"
  return 1
}

active_leaf() {
  timeout 10 ros2 topic echo --once "$NS/bt/status" std_msgs/msg/String 2>/dev/null \
    | grep -o '"active_leaf":"[^"]*"' | cut -d'"' -f4
}

# PX4's out topics are best-effort; --qos-profile sensor_data or silence.
arming_state() {
  timeout 10 ros2 topic echo --once --qos-profile sensor_data \
    "$NS/fmu/out/vehicle_status" 2>/dev/null \
    | grep -m1 "^arming_state:" | awk '{print $2}'
}

# Generous: covers PX4 boot, the lockstep engaging, and EKF2 convergence.
log "1/7 waiting for the EKF position estimate (sim + PX4 + EKF2 up)"
wait_for "vehicle_local_position on $NS/fmu/out" 240 \
  "timeout 10 ros2 topic echo --once --qos-profile sensor_data $NS/fmu/out/vehicle_local_position" \
  || exit $FAIL

log "2/7 waiting for the behavior tree (air.launch.py up)"
wait_for "bt/status publishing" 120 \
  "timeout 10 ros2 topic echo --once $NS/bt/status" || exit $FAIL

log "3/7 drone must boot idle and disarmed (no mission commanded yet)"
wait_for "active_leaf == Idle" 60 "[ \"\$(active_leaf)\" = Idle ]" || exit $FAIL
wait_for "disarmed" 30 "[ \"\$(arming_state)\" = 1 ]" || exit $FAIL

log "4/7 starting the mission -> takeoff (the drum can enter the camera"
log "    during the climb, so InspectAnomaly may fire before any waypoint)"
ros2 topic pub --once "$NS/mission/command" std_msgs/msg/String '{data: start}' >/dev/null
wait_for "active_leaf is TakeOff, GotoWaypoint, or InspectAnomaly" 90 \
  "[ \"\$(active_leaf)\" = TakeOff ] || [ \"\$(active_leaf)\" = GotoWaypoint ] || [ \"\$(active_leaf)\" = InspectAnomaly ]" \
  || exit $FAIL

log "4b/7 the red drum gets inspected (one orbit)"
wait_for "active_leaf == InspectAnomaly" 180 \
  "[ \"\$(active_leaf)\" = InspectAnomaly ]" || exit $FAIL

log "4c/7 the orbit ends (cooldown) and the patrol takes over"
wait_for "active_leaf == GotoWaypoint" 120 \
  "[ \"\$(active_leaf)\" = GotoWaypoint ]" || exit $FAIL

# The forcing knob: PX4's simulated battery drains toward SIM_BAT_MIN_PCT
# and the default floor of 50 never crosses the BT's 30 percent threshold.
# Dropping the floor lets it keep draining; there is no /battery/set here.
log "5/7 dropping the battery floor -> expect ReturnToPad"
"$PX4_BIN/px4-param" set SIM_BAT_MIN_PCT 10 >/dev/null 2>&1
wait_for "active_leaf == ReturnToPad" 180 \
  "[ \"\$(active_leaf)\" = ReturnToPad ] || [ \"\$(active_leaf)\" = Land ]" || exit $FAIL

log "6/7 landing at the pad"
wait_for "active_leaf == Land" 120 "[ \"\$(active_leaf)\" = Land ]" || exit $FAIL

log "7/7 waiting for touchdown + disarm, then the refill relaunch, then stop"
wait_for "disarmed after landing" 120 "[ \"\$(arming_state)\" = 1 ]" || exit $FAIL
# Restore the floor so the relaunched patrol doesn't immediately drain out.
"$PX4_BIN/px4-param" set SIM_BAT_MIN_PCT 50 >/dev/null 2>&1
wait_for "patrol resumes on the refilled battery" 180 \
  "[ \"\$(active_leaf)\" = TakeOff ] || [ \"\$(active_leaf)\" = GotoWaypoint ]" || exit $FAIL
ros2 topic pub --once "$NS/mission/command" std_msgs/msg/String '{data: stop}' >/dev/null
wait_for "stop preempts back to Idle" 30 "[ \"\$(active_leaf)\" = Idle ]" || exit $FAIL

log "PASS: idle -> start -> takeoff -> inspect -> patrol -> low battery -> pad -> land -> disarm -> relaunch -> stop"
exit $PASS
