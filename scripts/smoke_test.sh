#!/usr/bin/env bash
# End-to-end smoke test for the SPAR stack. Run inside the container
# while autonomy.launch.py is up:
#
#   docker exec -it spar /ws/scripts/smoke_test.sh
#
# Exercises the full loop the repo promises: boots idle -> mission start ->
# rounds -> forced low battery -> behavior tree preempts to ReturnToDock ->
# recharge at dock -> rounds resume.
source /ws/scripts/env.sh
set -u

PASS=0
FAIL=1

log() { echo "[smoke] $*"; }

# Waits until `ros2 topic echo --once` on a topic yields a line matching a
# pattern, or times out.
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

battery_pct() {
  timeout 10 ros2 topic echo --once "$NS/battery/state" --field percentage 2>/dev/null \
    | head -1 | awk '{printf "%d", $1 * 100}'
}

log "1/7 waiting for odometry (sim up)"
wait_for "odometry on $NS/platform/odom" 120 \
  "timeout 10 ros2 topic echo --once $NS/platform/odom" || exit $FAIL

log "2/7 waiting for Nav2 action server"
wait_for "navigate_to_pose action" 180 \
  "ros2 action list 2>/dev/null | grep -q $NS/navigate_to_pose" || exit $FAIL

log "3/7 robot must boot idle (no mission commanded yet)"
wait_for "active_leaf == Idle" 60 "[ \"\$(active_leaf)\" = Idle ]" || exit $FAIL

log "4/7 starting the mission -> tree goes to work (the drum is visible"
log "    from the dock, so Inspect may fire before the first checkpoint)"
ros2 topic pub --once "$NS/mission/command" std_msgs/msg/String '{data: start}' >/dev/null
wait_for "active_leaf is Rounds or Inspect" 60 \
  "[ \"\$(active_leaf)\" = Rounds ] || [ \"\$(active_leaf)\" = Inspect ]" || exit $FAIL

log "4b/7 the red drum gets inspected"
wait_for "active_leaf == Inspect" 180 "[ \"\$(active_leaf)\" = Inspect ]" || exit $FAIL

log "4c/7 inspection ends (cooldown) and the rounds take over"
wait_for "active_leaf == Rounds" 120 "[ \"\$(active_leaf)\" = Rounds ]" || exit $FAIL

log "5/7 forcing battery to 10% -> expect ReturnToDock"
ros2 topic pub --once "$NS/battery/set" std_msgs/msg/Float32 '{data: 10.0}' >/dev/null
wait_for "active_leaf == ReturnToDock" 30 "[ \"\$(active_leaf)\" = ReturnToDock ]" || exit $FAIL

log "6/7 waiting for recharge at the dock (battery >= 90%)"
wait_for "battery recharged" 300 "[ \"\$(battery_pct)\" -ge 90 ] 2>/dev/null" || exit $FAIL

log "7/7 waiting for the rounds to resume, then stopping the mission"
wait_for "active_leaf == Rounds again" 60 "[ \"\$(active_leaf)\" = Rounds ]" || exit $FAIL
ros2 topic pub --once "$NS/mission/command" std_msgs/msg/String '{data: stop}' >/dev/null
wait_for "stop preempts back to Idle" 30 "[ \"\$(active_leaf)\" = Idle ]" || exit $FAIL

log "PASS: idle -> start -> rounds -> low battery -> dock -> recharge -> rounds -> stop"
exit $PASS
