---
name: verify
description: Run and verify changes to this repo end to end without user help - container C++ build and tests, Unity C# compile, the map pipeline, headless sim, the smoke test, and a screenshot-based rviz visual check. Use after any code, scene, or script change, and for physics experiments in a scratch workspace.
---

# Verifying SPAR autonomously

Every check here runs unattended. Never ask the user to click anything in
Unity or Docker; every step has a headless path. Run the cheapest check that
covers the change, and always finish a session of edits with the checks in
"Which checks for which change" below.

## Ground rules

- Use absolute paths in shell commands. The repo root is the directory
  containing this file's `.claude/`. A `cd` in one command does not reliably
  persist, and can even reset mid-session.
- Put every temporary file (test worlds, rasterizer output, compile logs) in
  the session scratchpad directory, never in the repo tree and never in
  `/tmp` directly.
- Read the Makefile before inventing a command. Most workflows are one
  target; the raw commands below exist only where no target does.
- Never commit. Leave everything in the working tree for the user.
- IDE/clangd diagnostics on C++ files are noise on this machine (no ROS
  include paths on the host). The container build is the only authority.
- Never run `make ros2_container` from here. It ends by exec-ing into an
  interactive `docker exec -it` shell for a human to drive the robot stack
  from by hand (that's the point of it); with no TTY attached, it hangs.
  Bring the container up with the raw `docker compose` command instead (see
  check 1), and bring up the robot stack with `docker exec -d ... ros2
  launch spar_bringup autonomy.launch.py` (see check 5). This is the
  one place a fully automated, hands-off bring-up belongs at all; the
  normal dev loop is deliberately manual.

## The checks, cheapest first

### 1. C++ build (container)

Required after any change under `src/`. The container must be up:

```bash
docker ps --filter name=spar --format '{{.Names}} {{.Status}}'
```

If the daemon itself is down: `open -a Docker`, then poll `docker info`
until it answers (up to ~60 s). If the container is down, start it directly
(not `make ros2_container`, see the ground rule above):

```bash
mkdir -p build install logs
WORLD=blank docker compose -f docker/compose.yaml up --build -d
```

Nothing auto-launches, the container just idles (see docker/entrypoint.sh).
Build the same way a user would, with colcon:

```bash
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --packages-select spar --symlink-install'
```

Success is `Finished <<< spar` and exit 0. Note the flag
order: `--log-base` must come BEFORE the `build`/`test` subcommand.

### 2. C++ unit tests (container)

Required after any change to the BT node types (`src/bt/`) or their tests
(`test/test_bt_nodes.cpp`). The tree engine itself is BehaviorTree.CPP,
tested upstream; these tests cover only the logic this repo still owns
(staleness, hysteresis, cooldowns). Runs in under a second:

```bash
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log test --packages-select spar && colcon --log-base /ws/build/log test-result --verbose'
```

Success is `0 errors, 0 failures` in the summary line.

### 3. Unity C# compile

Required after any change under `unity/SparSim/Assets/`. The editor
version comes from the Makefile (`UNITY_APP`); if it 127s, read
`unity/SparSim/ProjectSettings/ProjectVersion.txt` and use
`/Applications/Unity/Hub/Editor/<version>/Unity.app/Contents/MacOS/Unity`.

No Unity instance may be running on the project (files under `Assets/` give
EPERM while one is, and a second instance aborts on the lockfile):

```bash
make unity-stop
```

Then compile (2-4 min; always `-logFile -` redirected to a file, because
`-logFile <path>` occasionally segfaults at startup):

```bash
LOG=<scratchpad>/unity_compile.log
"/Applications/Unity/Hub/Editor/6000.5.4f1/Unity.app/Contents/MacOS/Unity" \
  -batchmode -quit -projectPath <repo>/unity/SparSim -logFile - > "$LOG" 2>&1
echo "exit=$?"; grep -c "error CS" "$LOG"
```

Success is exit 0 and zero `error CS` lines. If Unity aborts complaining
another instance is running and none is, delete the stale
`unity/SparSim/Temp/UnityLockfile` and retry.

### 4. Map pipeline

Required after any change to collision geometry in the scene, to the Husky
prefab, or to `scripts/lint_world.py` / `scripts/rasterize_map.py`.

`make map` does dump -> lint -> rasterize and OVERWRITES the committed map.
For verification without touching the repo, dump once (or reuse an existing
`logs/blank.xml`) and run the scripts by hand into the scratchpad:

```bash
.venv/bin/python scripts/lint_world.py logs/blank.xml src/spar_bringup/config/autonomy_blank.yaml
.venv/bin/python scripts/rasterize_map.py logs/blank.xml <scratchpad>/maps
cmp src/spar_bringup/maps/blank.pgm <scratchpad>/maps/blank.pgm && echo byte-identical
```

For script-only changes, byte-identical output on the blank world is the
regression bar. For geometry changes, the map is EXPECTED to differ; the
bar is lint passing plus a fresh `make map` committed alongside the scene.

### 5. Headless sim + smoke test (full end to end)

Required after bridge/sensor changes, tree structure changes, or before any
sign-off. This is the one place a fully automated, hands-off bring-up of the
whole stack belongs. The normal dev loop is deliberately manual
(`make ros2_container` drops a human into a shell to run
`autonomy.launch.py` by hand; bridge, localization, Nav2, and behavior are
all one launch, one thing to bring up), so this check can't lean on it and
has to bring everything up itself.

Order is load-bearing: Unity FIRST, then the ROS side. Unity owns /clock,
and starting it jumps sim time backward, which would deactivate Nav2
lifecycle servers ("Action server is inactive") if they were already up
when it happened; starting fresh after Unity is already running sidesteps
that instead of racing it.

**Always recompile before smoke, even with zero C++ changes, and always
relaunch `autonomy.launch.py` fresh: never reuse one left running from a
previous check, and never restart it with a bare `pkill` (it isn't run
under `setsid`; a plain `pkill` orphans its children instead of killing
them). Always go through `make shut_down` for a clean teardown, full
container included.** A stale binary or a stale process both produce a
false PASS against code that isn't the code you're verifying.

```bash
make unity-stop
make unity-headless
sleep 25 && grep -m1 "bridge ids ok" logs/unity-headless.log   # retry until it appears
grep -m1 "camera sensor mounted" logs/unity-headless.log

make shut_down    # clean slate, whatever was running before
mkdir -p build install logs
WORLD=blank docker compose -f docker/compose.yaml up --build -d # nothing builds or launches yet (never `make ros2_container`, see ground rules)
docker exec spar bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon --log-base /ws/build/log build --symlink-install'
docker exec -d spar bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch spar_bringup autonomy.launch.py world:=blank'
# bash -lc matters here, not bash -c: ROS_LOG_DIR is exported via
# /etc/profile.d (see docker/entrypoint.sh), which only login shells source

make smoke         # ~4 min; run in background and poll if doing other work
make unity-stop    # leave the machine as found; the container is fine to leave running
```

Success is the final line `[smoke] PASS: idle -> start -> rounds -> low
battery -> dock -> recharge -> rounds -> stop`. Any earlier `[smoke] FAIL`
names the step; the per-run logs are in the newest `logs/runNNN/`.

### 6. rviz visual check (screenshot)

Required after changes to `src/spar_bringup/rviz/spar.rviz` or
`scripts/rviz.sh`; worth running after any perception/TF/costmap change too,
since checks 1-5 confirm the topics and logs are right but not that the
picture is. Needs check 5's stack up first (`ros2 launch` and Unity both
alive) — rviz has nothing to draw otherwise.

`imagemagick` (for `import`) is deliberately not in the Dockerfile — it's an
assistant-only verification tool, not something students need (they have
noVNC and a real browser). Install it ad hoc; it's small and doesn't survive
a container recreate:

```bash
docker exec spar bash -lc 'apt-get update -qq && apt-get install -y -qq --no-install-recommends imagemagick >/dev/null 2>&1'
docker exec -d spar /ws/scripts/rviz.sh
sleep 8   # Xvnc + rviz2 + the first map/costmap swatch
docker exec spar bash -lc 'DISPLAY=:1 import -window root /ws/logs/rviz_check.png'
docker cp spar:/ws/logs/rviz_check.png <scratchpad>/rviz_check.png
```

Read the PNG back with the Read tool (it renders images directly). Success:
the Displays panel's `Global Status` reads `Ok`, not an error like `Frame
[map] does not exist`; and whatever the change should affect is visibly
right — a TF triad that moves between two screenshots a few seconds apart
for a localization/bridge change, a costmap blob in the right place for a
costmap tuning change, a new display actually rendering (not just listed)
for an rviz config change.

## Physics experiments in the scratchpad

The repo venv has mujoco (`.venv/bin/python`). To test a hypothesis about
the map pipeline or MuJoCo semantics, write a minimal MJCF world in the
scratchpad and run the real scripts against it. The scripts import from
each other by directory, so add `scripts/` to the path:

```bash
.venv/bin/python - <<'EOF'
import sys; sys.path.insert(0, "<repo>/scripts")
import mujoco, lint_world, rasterize_map
m = mujoco.MjModel.from_xml_path("<scratchpad>/test_world.xml")
# interrogate m / call the scripts' functions directly
EOF
```

Two conventions the test worlds must respect: the scripts find the lidar
plane via a site whose name starts with `lidar` (no site = hard exit), and
world membership is `body_weldid == 0` (see the docstring in
rasterize_map.py). Prove a claim with a minimal world before changing the
scripts, and keep the world as the regression check after.

## Which checks for which change

| Change | Required checks |
| --- | --- |
| BT node types / leaves (C++) | 1, 2, and 5 if behavior changed |
| behavior_trees/main_tree.xml (tree shape) | 1 (it's installed by CMake), then 5 |
| anomaly_detector / battery_sim | 1, then 5 |
| Bridge / camera / any C# | 3, then 5 |
| Scene or prefab geometry | 3, 4 (fresh `make map`), 5 |
| lint_world / rasterize_map | 4 (byte-identical on blank) + a scratchpad probe world |
| autonomy.launch.py / nav2.yaml / localization.yaml / autonomy_\<world\>.yaml / compose.yaml / entrypoint.sh / scripts/*.sh | 5 (its bring-up always tears down and starts the whole stack fresh, so this covers any of these; configs are symlinked, no rebuild needed for yaml-only changes) |
| scripts/rviz.sh / spar.rviz | 6 (needs 5's stack up first) |
| Docs / comments only | none, but grep that referenced files/names still exist |

A full sign-off pass is 1 through 5 in order. Report results plainly: what
ran, the exact pass/fail evidence (the PASS line, the byte-identical cmp,
the error count), and anything skipped with the reason.
