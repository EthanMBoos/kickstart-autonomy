// Spins the four visual wheels to match the two physics drive spheres.
//
// The Husky LOOKS like a 4-wheel skid-steer, but no simulator drives it as
// one. Clearpath's own stack treats it as a differential drive everywhere:
// their controller pairs the four wheel joints into left/right and computes
// odometry from a two-wheel model whose track width is inflated by
// wheel_separation_multiplier: 1.875 to absorb the skidding the model
// doesn't capture:
//   https://github.com/husky/husky/blob/noetic-devel/husky_control/config/control.yaml
// Their Gazebo sim keeps nominally honest wheel friction (mu1=mu2=1.0):
//   https://github.com/husky/husky/blob/noetic-devel/husky_description/urdf/wheel.urdf.xacro
// and turning still works only because ODE's iterative solver under-enforces
// the friction cone, quietly leaking the lateral slip a real tire gets by
// scrubbing. The folklore of tuning mu2/fdir1/slip for skid-steer exists
// because of exactly that:
//   http://answers.gazebosim.org/question/5476/parameters-for-a-skid-steeringsimulated-tracked-robot-use-of-the-fdir1-tag/
//
// MuJoCo's solver actually enforces the cone (and has no fdir1 equivalent),
// so the leak isn't available: an honest 4-wheel model barely pivots. We make
// the same simplification Clearpath does, just openly and in the physics
// layer: two drive spheres on the wheelbase midline plus frictionless
// casters, which IS the two-wheel robot their controller pretends the Husky
// is — with the true track width and no 1.875 fudge factor. This script is
// the costume: it copies each drive sphere's hinge rotation onto the wheel
// meshes on its side, so all four wheels spin at the commanded speed like
// Gazebo's do.
//
// The reduction's tuning, as set on the Husky prefab:
//  - Drive spheres: r 0.165 (the real wheel radius, so v = omega*r holds),
//    sliding friction 0.9, hinge damping 0.2 as rolling resistance.
//  - Casters: near-frictionless (0.005) with Priority = 1. The priority is
//    load-bearing: MuJoCo resolves a contact pair with the HIGHER-priority
//    geom's friction, so without it the floor's friction (1.0) would win
//    the pairing and the "frictionless" casters would drag:
//    https://mujoco.readthedocs.io/en/stable/computation/index.html#contact-parameters
//  - Actuators: proportional velocity servos (gain 35, bias -35: force =
//    35*(ctrl - qvel)), ctrl ±15 rad/s ≈ 2.5 m/s at the wheel, force
//    clamped ±80 N·m to keep stall torque physical.
using UnityEngine;

public class SparWheelVisuals : MonoBehaviour {

  [Tooltip("The left/right drive-sphere bodies (their transforms carry the hinge rotation).")]
  public Transform leftDrive;
  public Transform rightDrive;

  [Tooltip("Visual wheel meshes on each side (front and rear).")]
  public Transform[] leftWheels;
  public Transform[] rightWheels;

  Quaternion _leftDrive0Inv, _rightDrive0Inv;
  Quaternion[] _leftWheels0, _rightWheels0;

  void Start() {
    _leftDrive0Inv = Quaternion.Inverse(leftDrive.localRotation);
    _rightDrive0Inv = Quaternion.Inverse(rightDrive.localRotation);
    _leftWheels0 = CaptureRest(leftWheels);
    _rightWheels0 = CaptureRest(rightWheels);
  }

  static Quaternion[] CaptureRest(Transform[] wheels) {
    var rest = new Quaternion[wheels.Length];
    for (var i = 0; i < wheels.Length; i++) rest[i] = wheels[i].localRotation;
    return rest;
  }

  void Update() {
    // The MuJoCo plugin syncs each MjBody transform from the physics state,
    // so the drive sphere's rotation delta IS the hinge angle, with the axis
    // and sign already correct — no unit or handedness reasoning needed.
    Apply(leftDrive.localRotation * _leftDrive0Inv, leftWheels, _leftWheels0);
    Apply(rightDrive.localRotation * _rightDrive0Inv, rightWheels, _rightWheels0);
  }

  static void Apply(Quaternion spin, Transform[] wheels, Quaternion[] rest) {
    for (var i = 0; i < wheels.Length; i++) {
      wheels[i].localRotation = spin * rest[i];
    }
  }
}
