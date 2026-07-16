# CLAUDE.md

## What this is

A teaching template for the GT Cloud Robotics Autonomy and LLM tracks:
ROS2 + Nav2 + a small C++ behavior tree, simulated in Unity with the MuJoCo
plugin. Students fork it and put their own spin on a robotics behavior. The
repo's job is to be read, understood, and modified by a student in a
weekend, not to be a product.

The shape: MuJoCo physics runs inside Unity (the plugin); one C# sensor
layer (`unity/SparSim/Assets/Scripts/SparRos/`) publishes ROS
topics over TCP to a Docker container running Nav2, AMCL, and the behavior
tree (`src/spar/`). The Unity scene is the world; MJCF only
flows out (`make map`). Design decisions and their reasons are in
`docs/sim-architecture.md`.

## The code I want

- Simple, clean, robust, in that order of visibility: a student should read
  any file top to bottom in one sitting and understand it.
- No enterprise patterns. No factories, no interfaces with one
  implementation, no dependency injection frameworks, no manager classes,
  no config indirection. Reach for a function first, a class second, an
  abstraction only when the third caller exists.
- Prefer deleting code to guarding it. One-time utilities get removed after
  use; git history keeps them.
- Robust means handling the failures that actually happen (stale data,
  unavailable action servers, late callbacks) plainly and locally, not
  wrapping everything in defensive checks.
- Match the conventions already here: one BT node per header, params
  declared and read in constructors, the existing naming.

## Comments and docs

- Comments state non-obvious constraints and hard-won gotchas, next to the
  code they protect, with source links when a claim needs receipts (see
  `SparWheelVisuals.cs`). Never narrate what the code does, and never
  explain the same lesson in two files.
- Docs are terse human prose. No em dashes. Implementation rationale
  belongs in code comments, not in markdown; the md docs cover only what
  and why at the architecture level.
- Don't add a doc when a comment will do; don't add a comment when the
  code can say it.

## Verifying changes

The `verify` skill (`.claude/skills/verify/SKILL.md`) is the complete
playbook: which checks each kind of change requires, the exact commands,
and the traps (restart order, Unity lockfiles, log redirection). Use it;
don't improvise the commands. Everything in it runs unattended.

Commit only when asked.
