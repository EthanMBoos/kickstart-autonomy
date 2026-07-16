# The simulation architecture

One decision drives everything here: MuJoCo is the physics, and Unity sits on
top of it. The MuJoCo plugin runs the physics inside the Unity process, so
rendering, drag-and-drop world authoring, and the robot all share one world.
A C# sensor layer (`SparRosBridge.cs`) computes the ROS topics directly
from MuJoCo's state and ships them over TCP to a Docker container that runs
everything ROS: Nav2, AMCL, the behavior tree. There is no simulator in the
container.

```
        ┌───────── container (ROS): Nav2 / AMCL / behavior tree ─────────┐
        │        pure ROS + the :10000 endpoint Unity connects to        │
        └───────────────────────────▲────────────────────────────────────┘
                                    │ ROS-TCP
        ┌───────────────────────────┴────────────────────────────────────┐
        │   ONE substrate: Unity + MuJoCo plugin (physics in-process)    │
        │   one C# sensor layer · the scene IS the world                 │
        │   MJCF flows OUT only: dump → lint → rasterize (make map)      │
        └────────────────────────────────────────────────────────────────┘
          run it three ways, same env, same sensors, same physics:
            • WINDOWED    make unity-gui: the editor opens and plays itself
            • WINDOWLESS  make unity-headless: tests and smoke
            • RL          a training loop drives the env instead of Nav2
```

Why MuJoCo underneath instead of Unity's own physics:

- MuJoCo's contacts are trustworthy for a mobile base; PhysX defaults aren't.
- The stack is never married to Unity, because the physics is never PhysX.
- An RL policy trains and deploys on the same physics ([rl.md](rl.md)).

## Worlds

The Unity scene is the world. You edit it directly, and nothing is ever
generated into it; MJCF only flows *out*. `make map` dumps the exact model
the plugin compiled, runs it through a lint gate (geometry the robot can hit
but its lidar can't see, blocked docks and waypoints), and rasterizes the map
AMCL localizes against, so the map can't drift from the world.

Keep collision geometry to primitives (box/sphere/capsule) and make the
pretty meshes plain Unity visuals. That split is what lets rendering get as
fancy as it wants without ever touching physics, lidar, or the map.

The robot looks like a Husky (Clearpath's meshes, wheels spinning) but is a
differential drive under the hood: two invisible drive spheres plus
frictionless casters, because a four-wheel skid-steer with honest friction
barely pivots. The tuning and the reasoning live in
`SparWheelVisuals.cs`, next to the code they explain.

## Perception

The camera is real in every run: Unity renders color + depth, and the
container's detector turns pixels into a labeled point on
`perception/detections` (message `Detection`: header, label, map-frame
point). The label is generic on purpose: a small pretrained VLM or
segmentation model can replace the HSV node and report whatever classes it
sees, one Detection per hit, without the topic name changing; the behavior
tree filters at the subscription for the labels it cares about
(`anomaly_label`), not just "anomaly". That, not vision RL, is how vision
models enter this stack ([rl.md](rl.md)).

## Topics

Everything lives under `/husky`, published by the sensor layer in every run
mode:

| Topic / TF | Type | Notes |
| --- | --- | --- |
| `/clock` | rosgraph_msgs/Clock | the sim owns time |
| `platform/odom`, TF `odom→base_link` | nav_msgs/Odometry | drift-free, twist in the child frame |
| TF `base_link→{lidar2d_0_laser, camera_0_link}` | static TF | republished at 1 Hz |
| `sensors/lidar2d_0/scan` | LaserScan | 720 rays at 15 Hz |
| `sensors/camera_0/color/image` (+ `camera_info`, `…/depth/image`) | Image | rendered frames, 10 Hz |
| `perception/detections` | spar/Detection (map) | the pixel detector's labeled hits; `bt_executive`'s `anomaly_label` param picks the one label it acts on |
| `cmd_vel` (subscribed) | TwistStamped | Nav2 → skid-steer mixer |

Transport inside the container is Zenoh over TCP; Unity connects to the
ROS-TCP endpoint on :10000.

## Settled: tried it, or weighed it, and closed it

- Pure Unity/PhysX, and Unity as a render-only viewer over container physics:
  both rejected. The first loses honest physics; the second needs fragile
  pose-feedback with contacts split across two authorities.
- A second Python sensor layer for a Unity-free mode: built once, then
  deleted. Every sensor existed twice and had to be parity-tested forever.
- Time acceleration (RTF): deleted. Planners and vision don't speed up with
  sim time, so the stack runs at RTF ≈ 1.
- Generating scenes from MJCF: no. Scenes are built in the editor, period.
- The Unity packages are vendored in `third_party/`; both the plugin and the
  ROS-TCP connector needed small Unity-6.5 patches.
- Out of scope: manipulation, learning from pixels, high-fidelity hydro/aero.

## Roadmap

- The RL harness: `make map` → pure-MuJoCo Gym env → deploy the policy as a
  behavior node ([rl.md](rl.md) has the full design).
- Themed worlds and moving agents (asset-pack worlds, ML-Agents/NavMesh
  movers, per-world scene selection in the map pipeline).
- Marine and air. A domain is three swaps, not a fork: dynamics (a force
  module in the sim loop), sensors (the one C# layer), and planning (above
  the topic boundary). The 2D lidar + map + AMCL are the ground domain's
  module, not universal truth.

The bring-up traps (name resolution after the plugin recompiles the scene,
`tf_static` over ROS-TCP, the ignored MJCF timestep, and friends) are
documented as comments next to the code that handles each one, mostly the
bridge, the bootstrap, and the Makefile.
