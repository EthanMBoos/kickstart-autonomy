# SPAR - Sim Portable Autonomy Runtime

Robot autonomy is advancing fast on two fronts, RL and VLAs. The
capabilities are real; the reliability needed for long-horizon,
unsupervised work in the wild is not. SPAR is a runtime for hybrid
autonomy: a deterministic behavior tree owns the mission, and learned
components slot in under it (a trained policy as a behavior node, a VLM
as the perception source) without the rest of the stack changing.

The sim is **Unity with the MuJoCo plugin**: MuJoCo is the physics, Unity
is the render and authoring front end, and nothing is vendor-locked the
way Isaac Sim is. One authored world runs three ways on the same physics
and sensors: windowed for authoring, headless for tests, and as a
pure-MuJoCo Gymnasium env for fast RL training, with the full stack as
the eval harness. The robot is configured as it would be in the real
world. A sim-to-real gap remains, but this is the lowest-friction way to
stress a behavior across a wide variety of scenarios.

The bet is that foundation models and VLAs keep improving every year, so
the hard part becomes systems integration: verifying a policy against the
reliability bar real users actually hold. This repo is that testground.
Build scenarios, run regressions against the edge cases, and be able to
say, with evidence, that a policy is good to go before it leaves sim.

Today there are two concrete tracks in the same world. The ground track
is a ROS2 + Nav2 Husky that makes inspection rounds of a site, inspects
anomalies (a red drum) when perception spots them, and returns to its
dock when the battery runs low. The air track is a PX4-flown Skydio X2
that patrols overhead, orbits the same drum when its camera finds it, and
returns to its pad on low battery ("The air track" below). The substrate
spans domains by swapping dynamics, sensors, and planner, not by forking.

Starter repo for the
[GT Cloud Robotics](https://www.gtcloudrobotics.com/course-home/) Autonomy
and LLM tracks. Design rationale lives in
[docs/sim-architecture.md](docs/sim-architecture.md).

The sim runs in Unity on your machine (`unity/SparSim`, scene
`BlankWorld.unity`). The autonomy stack runs in one Docker container and
talks to Unity over a TCP bridge. Unity publishes clock/odom/TF/lidar/
perception and consumes `cmd_vel`.

## Quickstart

Requirements: [Docker](https://docs.docker.com/get-docker/) with 8 GB+
memory (Docker Desktop, Settings, Resources), and
[Unity](https://unity.com/download) (6000.5+, free Personal license) with the
project at `unity/SparSim` added in Unity Hub. macOS also needs
[MuJoCo.app](https://github.com/google-deepmind/mujoco/releases) 3.10 in
`/Applications`.

```bash
git clone https://github.com/EthanMBoos/spar.git
cd spar
make ros2_container                # builds the image, starts the container, drops you into a shell

colcon build --symlink-install
source install/setup.bash          # only needed this once; later shells (make shell) source it for you

ros2 launch spar_bringup autonomy.launch.py world:=blank
```

Ctrl-C and rerun any time. After editing code, rebuild first (see "Working
on the autonomy code" below), then rerun the launch.

From a second terminal on your host, run the sim:

```bash
make unity-gui       # open the Unity editor and press Play for you
make unity-headless  # no editor window (make unity-stop ends it)
```

The robot boots idle at its dock. Open another shell (`make shell`) to talk
to the mission layer:

```bash
scripts/mission.sh start
ros2 topic echo /husky/bt/status   # active leaf, mission state, battery
scripts/mission.sh stop
ros2 topic pub --once /husky/battery/set std_msgs/msg/Float32 '{data: 10.0}'  # force low battery
```

From a host terminal:

```bash
make smoke     # end-to-end check (~4 min, ends in PASS)
```

`make` lists every command. Each target is one or two lines in the
[Makefile](Makefile).

## The air track

Everything above is the ground track and works unchanged. The air track
is the same world flown by a different stack: PX4 SITL (the real
autopilot: EKF2, position control, failsafes) in its own container, with
a behavior tree above it speaking offboard setpoints. The `_air` make
targets point at it; the first build compiles PX4 from source and takes
a while.

```bash
make unity-gui
make ros2_container_air
colcon build --symlink-install     # in that shell: builds spar_air

# start PX4, in that shell
cd /opt/px4/build/px4_sitl_zenoh
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=none_iris \
  PX4_SIM_HOSTNAME=host.docker.internal ./bin/px4 -d > /tmp/px4.log 2>&1 &
./bin/px4-zenoh start              # joins PX4 to the ROS graph

ros2 launch spar_air air.launch.py world:=blank
```

Give EKF2 ~10 s to converge after PX4 starts, then the same mission
controls as the ground robot, drone-flavored:

```bash
make shell_air
scripts/mission.sh start
ros2 topic echo /skydio/bt/status
/opt/px4/build/px4_sitl_zenoh/bin/px4-param set SIM_BAT_MIN_PCT 10  # force low battery
```

`make smoke_air` runs the whole arc unattended (takeoff, inspect,
patrol, battery return, land, disarm, relaunch). A mixed demo is both
containers up and both missions started; nothing else to configure.

One trap the two tracks share: if you restart Unity, restart the
containers after it (`make shut_down`, bring both back up). Unity owns
the clock, and both Nav2 and PX4 react badly to time rewinding under
them.

## Working on the autonomy code

`src/spar/`:

```
src/bt/                          BT.CPP node types: conditions, staleness helpers
src/leaves/                      leaves that own Nav2 action calls
src/bt_executive.cpp             registers node types, ticks the tree at 10 Hz
src/anomaly_detector.cpp         camera pixels -> map-frame anomaly point
src/battery_sim.cpp              fake BMS: drains, recharges at the dock
behavior_trees/main_tree.xml     the tree's shape (BehaviorTree.CPP XML)
test/                            gtest for the node types this repo owns
```

Edit on your host; the workspace is bind-mounted, build artifacts land in
`build/` and `install/` (symlinked, no separate install step). `colcon
build --symlink-install` (Quickstart) is for the first build; after that,
rebuild faster straight through the generated Makefile, in your `make
ros2_container` shell:

```bash
cd build/spar && make
```

Plain `make` reruns cmake itself if needed. Colcon's build dir already
caches its own source path in `CMakeCache.txt`. Ctrl-C the launch and
rerun it to pick up the change.

Behavior parameters (waypoints, dock pose, battery thresholds, detector
tuning): `src/spar_bringup/config/autonomy_<world>.yaml`. Nav2 (speed
limits, costmaps): `config/nav2.yaml`. Both are symlinked into the install
tree, so edits take effect on the next launch, no rebuild needed.

Logs: `logs/run001, run002, ...` at the repo root, one directory per launch,
one file per node plus `run-info` naming the world.

## The environment

Worlds are authored in Unity; the scene is the world. `BlankWorld` is a
small inspection site: storage racks, checkpoint pads, the dock pad, and a
red drum. Drag objects into the scene and MuJoCo picks them up live: give a
prop a primitive `MjGeom` child (box/sphere/capsule) for lidar and physics,
keep the pretty mesh a plain Unity visual. The robot is a prefab
(`Assets/Prefabs/Husky.prefab`); a new world is a new scene with that prefab
dragged in.

Two things travel with a world:

- **Its map** (`src/spar_bringup/maps/<world>.*`). Regenerate after
  changing collision geometry:

  ```bash
  make map    # dump the scene as MJCF -> lint gate -> rasterize the map
  ```

- **Its behavior config**: `config/autonomy_<world>.yaml`.

## Reference

| Thing | Where |
| --- | --- |
| Build the code (first time, or a full rebuild) | `colcon build --symlink-install` inside the shell (then `source install/setup.bash` if that shell predates the build) |
| Bring up the stack | `ros2 launch spar_bringup autonomy.launch.py world:=blank`, from a `make ros2_container`/`make shell` |
| Start/stop the mission | `scripts/mission.sh start` / `stop`, from a `make shell` |
| Behavior status feed | `ros2 topic echo /husky/bt/status`, from a `make shell`; JSON: active leaf, mission, battery |
| Live node logs | `make tail` (`/rosout`, every node merged), from a host terminal (fails if nothing is launched) |
| rviz2 | `make rviz`, from a host terminal — opens a browser tab (noVNC); Ctrl-C stops it |
| Rebuild after code edits | `cd build/spar && make` inside the shell (fast, after the first build above) |
| End-to-end test | `make smoke` (needs Unity running and `autonomy.launch.py` up) |
| Stop and remove the container | `make shut_down` |
| Clean rebuild | `make clean` (shuts down, removes `build/` + `install/`), then `make ros2_container` |
| Logs of past runs | `logs/runNNN/`, one per launch |
| The Unity sim | `unity/SparSim`, scene `BlankWorld.unity`, sensors in `Assets/Scripts/SparRos/` |
| Perception | camera renders in Unity (windowed or headless); the containerized detector turns pixels into labeled points on `perception/detections` |
| Unity/ROS bridge | ROS-TCP endpoint on port 10000 (`src/ros_tcp_endpoint`, vendored) |
| Behavior config | `src/spar_bringup/config/autonomy_<world>.yaml` |
| Nav2 config | `src/spar_bringup/config/nav2.yaml` (speed: see `vx_max` comments) |
| The air stack | `air/src/spar_air` (BT + TF + detector), the `make *_air` targets |
| Air behavior config | `air/src/spar_air/config/autonomy_<world>.yaml` |
| PX4 pins and the topic mapping | `docker/Dockerfile.air`, `docker/air/{pub,sub}.csv` |
| Why it's built this way | [docs/sim-architecture.md](docs/sim-architecture.md) |
| Using the sim for RL | [docs/rl.md](docs/rl.md) |
