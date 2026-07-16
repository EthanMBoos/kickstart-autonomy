# Reinforcement learning on this sim

The architecture buys exactly one great RL trick, and it's the recommended way
to do RL here:

> **Dump the world from Unity as MJCF (`make map` already produces it), build a
> pure-MuJoCo Gymnasium env on that file — no ROS, no Unity — train fast, then
> deploy the policy as a behavior node and evaluate it in the full stack.**

Because MuJoCo is the physics in Unity *and* in the training loop, and because
the dump is the exact compiled model the Unity sim runs (same geometry, same
`timestep`, same solver options), the policy never crosses a physics boundary
between training and deployment. That identity is the payoff of
"MuJoCo is the backend" (docs/sim-architecture.md).

> **Status:** the dump exists today (`make map` leaves `logs/<world>.xml`); the
> Gym wrapper and deploy path are on the roadmap. This doc is the design and
> the student starting point, not a finished `make rl`.

## The shape: train fast, eval in the stack

```
Unity scene (authored world)
   └── make map  →  logs/<world>.xml          (the exact compiled model)
         └── Gymnasium env: MuJoCo + ~100 lines of Python
               obs: lidar (mj_multiRay), poses, oracle, battery
               act: (v, ω) → skid-steer mix → ctrl        no ROS, no Unity
               10⁵–10⁷ steps/s territory; trivial resets; mjx-able
                     └── train (SB3 / PPO, or mjx+Brax for massive parallel)
                           └── deploy policy as a behavior node
                                 └── eval in the full Unity + ROS stack
```

Two loops, only one of which must be fast:

- **Training** — the loop above. ROS is *never* in it: a ROS-TCP round-trip per
  step is orders of magnitude too slow and its timing is nondeterministic.
- **Eval** — the full stack (Unity render, Nav2, AMCL, the behavior tree),
  run at fidelity to check the learned behavior holds up. `make smoke` is the
  template.

**Where the behavior tree fits — it is not in the training loop, by design.**
Training *replaces* the machinery around the thing being learned:

- Learning a low-level policy (a local planner)? Neither the BT nor Nav2
  belongs in the loop — rewards come from ground-truth poses.
- Learning the high-level executive? The BT's arbitration *is what the policy
  replaces*; a cheap analytic go-to controller (pure-pursuit to a waypoint)
  stands in for Nav2 during training.
- At **eval**, the stack returns: the policy runs as a behavior node inside the
  BT, and the honest test is whether it beats the hand-written tree on the same
  world.

The BT core is deliberately ROS-free C++, but its leaves own Nav2 action
calls — don't embed it in the training loop; you don't want it there anyway.

**What the Python env reimplements** (the accepted, bounded cost): the tiny
observation/action layer — skid-steer mixing (`WHEEL_RADIUS`/`TRACK_WIDTH`
constants), lidar via `mj_multiRay`, the visibility oracle (FOV + range +
`mj_ray` line-of-sight), reward terms. The conventions are already specified —
the mixer and lidar live in `SparRosBridge.cs`; reference implementations
of the oracle (C#) and the full Python sensor set (`sim_node.py`) are in git
history. Port, don't invent.

## What to train (non-vision)

Observations are low-dimensional and semantic: downsampled lidar, relative
poses, "target in view + bearing" from the oracle, battery, distances. That's
a rich state for real RL, and everything below runs at fast-lane speeds.

**Tier 1 — low-level control** (continuous `(v, ω)`):

- **Learned local planner** — lidar + local goal → velocity; the canonical
  first task, batches well.
- **Dynamic obstacle dodging** — add movers; the policy learns to yield from
  lidar alone. A case where learned can beat the classical planner.
- **Recovery / unstuck** — learn the back-up-rotate-reapproach behavior that
  recovery servers hand-code.
- **Domain flavors later** — marine station-keeping under wave disturbance,
  multirotor altitude hold: pure state-based control on the same substrate.

**Tier 2 — high-level behavior** (discrete choice among sub-behaviors — the
tier that matches this repo's focus):

- **Learned executive** — learn the BT's arbitration: choose among
  patrol / investigate / return-to-dock from `{battery, target-seen?,
  distances, time}`; reward targets investigated, penalize a dead battery.
  Then race it against the hand-written tree.
- **Active search** — find the target: obs = lidar + oracle bearing-if-seen;
  reward detection / time-to-detection. This rehearses the repo's whole
  search → recognize → trigger loop with zero pixels.
- **Coverage / patrol optimization** — area per unit battery; penalize
  revisits.

The **visibility oracle is the unlock**: any behavior that would trigger off
the camera trains against ground-truth "is it in view", then gets re-validated
against the real camera + detector at eval. And because the sim knows every
pose, rewards are dense, cheap, and exact — half the battle in RL.

## Vision: pretrained models, not vision RL

Learning *from pixels* is out of scope here, deliberately. It sits in the worst
corner of both costs at once: the renderer throttles sampling (~10³ steps/s at
best with Unity in the loop, vs 10⁵–10⁷ without), *and* pixel policies need
enormous sample counts — GPU-weeks, not a laptop afternoon. Meanwhile the
state-based RL above trains in **hours on a laptop** precisely because the
observations are low-dimensional.

Vision enters this stack a different way: **pretrained models at inference
time.** Small VLMs/VLAs (Moondream-class) run on a laptop and consume Unity's
rendered camera frames directly. The swap point is already built:
`anomaly_detector` is deliberately just "pixels -> labeled map-frame point on
`perception/detections`" - replace the HSV node with a model node that
publishes the same Detection message with its own label, and the behavior
tree doesn't change. That was the design intent from day one (see the
detector's own header comment).

So the division of labor is:

- **RL learns control and decision-making** in low-dimensional state, fast,
  in the pure-MuJoCo loop.
- **Pretrained models supply recognition** from Unity's real rendered pixels,
  with zero training.
- They meet at eval: learned behaviors, triggered by model-recognized things,
  in the full stack.

(Heavy inference dependencies stay out of the base container image — a model
node runs in your own environment and just subscribes to the camera topics.)

> **TODO:** don't let a model override or second-guess the BT's/policy's
> decisions in real time (a "VLM supervisor" that can accept, modify, or
> veto actions). [Anthropic's robotics eval](https://www.anthropic.com/research/claude-plays-robotics)
> tried exactly that — a model supervising a pretrained VLA policy — and it
> scored *worse* than just running the policy alone; only their best model
> broke even, and only on tasks the policy had never seen. Keep models
> perception-only here (labeling pixels, like `anomaly_detector` does
> today), not in the decision loop.

## Visibility: instrumenting the training loop with Rerun

Nothing in the loop above has a visibility story yet: no viewer, no replay,
no run comparison. [Rerun](https://rerun.io) fills that gap specifically,
not the deployed stack's (that's rviz, `make rviz`): a Python SDK
(`pip install rerun-sdk`, no ROS, no Unity, no container changes) built for
exactly this kind of multi-rate robotics/RL data. Keep it out of the base
image, same reasoning as the vision models above: it's a training-time tool
for your own environment, not part of what students run to get the stack up.

**Two timelines, two logging rates.** Spatial detail every step (10⁵-10⁷
steps/s) would drown both disk and the viewer, so split by what's cheap:

- `step` (a `sequence` timeline): scalars, every step. Reward, episode
  length, PPO loss/entropy, straight out of the SB3 callback.
- `sim_time` (a `duration` timeline): spatial detail, only for one
  designated rollout every N episodes (an eval rollout, not every training
  env). Robot pose, lidar hits, goal position, oracle bearing.

```python
import rerun as rr

rr.init("spar_rl", spawn=True)   # or rr.connect_grpc() to a viewer
                                        # already running (headless boxes),
                                        # or rr.save("run.rrd") for batch/offline

# every step, cheap:
rr.set_time("step", sequence=global_step)
rr.log("train/reward", rr.Scalars(reward))
rr.log("train/episode_length", rr.Scalars(episode_len))

# every N episodes, one designated env only, full detail:
rr.set_time("sim_time", duration=t)
rr.log("world/robot", rr.Transform3D(translation=pos, rotation=quat))
rr.log("world/lidar", rr.Points3D(lidar_hit_points))
rr.log("world/goal", rr.Points3D([goal_xy]))
```

With a `VecEnv`/parallel training (SB3's default, or thousands-wide under
mjx/Brax), log scalars aggregated across all envs but spatial detail from
one fixed env index only; logging 32+ robots' full lidar every step is both
slow and unreadable. For the scalar side at high step rates, `rr.send_columns()`
batches a whole episode's worth of values into one call instead of one
`rr.log()` per step, and is significantly faster.

Entity paths under `world/` deliberately echo the `/husky/...` ROS topic
names from the deployed stack, not because Rerun needs it, but so a
side-by-side of a training rollout and an eval rollout (below) reads as the
same robot, not two different ones.

**Where this earns its keep beyond training curves**: at eval time (deploy
the policy as a behavior node, run it in the full Unity+ROS stack), log the
same `world/robot` and `world/lidar` entities from the deployed run into a
second Rerun recording. Because MuJoCo is the physics in both the training
env and the Unity plugin, the two trajectories should overlay almost
exactly for the same policy on the same world; if they don't, the "same
physics on both sides" claim ([sim-architecture.md](sim-architecture.md))
doesn't hold in practice, and this is what would show it, not just assert it.

**Deliberately not doing yet**: instrumenting `bt_executive.cpp` or the live
ROS stack with the C++ SDK. The visibility gap that actually exists today is
the training loop; the deployed stack already has rviz. Add the C++ side
only if a real second need shows up (a BT execution trace correlated
against battery/position over a run is the likely candidate) — an
abstraction earns its place on the third caller, not the first.

## Student starting point

1. **Get the world:** `make map` (leaves `logs/blank.xml` — the compiled
   model, robot included).
2. **Wrap it:** a Gymnasium env holding `mujoco.MjModel/MjData`:

   ```
   obs    = concat(lidar_64, goal_dx, goal_dy, target_seen, bearing, battery)
   action = (v, omega)          # wl,wr = (v ∓ ω·TRACK/2) / WHEEL_R → data.ctrl
   reward = progress − collision − time                # from ground truth
   done   = reached | collided | timeout;  reset = mj_resetData + new goal
   ```

3. **Train:** Stable-Baselines3 PPO/SAC to start; graduate to `mjx` + Brax /
   PureJaxRL when you want thousands of parallel envs on a GPU. Instrument
   with Rerun as you go (see above), not after the fact.
4. **Deploy + eval:** load the policy as a behavior node, run the full stack
   (`make unity-gui` + `ros2 launch spar_bringup autonomy.launch.py` +
   `scripts/mission.sh start`), and compare against the hand-written BT
   (overlay the Rerun trajectory from training against this run to confirm
   the physics matches).

Caveat for the `mjx` jump: it supports a subset of MuJoCo (fine for
primitive-collider worlds and this robot), so the fast-parallel path may want a
lightly trimmed model rather than the byte-identical dump.
