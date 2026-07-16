// The sensor layer: computes the robot's ROS topics from the MuJoCo plugin's
// MjData and publishes them over ROS-TCP to the autonomy container.
//
// Design note: all ROS data is read from MjData directly (MuJoCo world
// coordinates — right-handed, Z-up — which are also ROS coordinates), so no
// Unity coordinate conversion ever touches the ROS data path. Unity transforms
// are only used to *render* the same state.

using System;
using Mujoco;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SparRosBridge : MonoBehaviour {

  [Header("Connection")]
  public string rosIPAddress = "127.0.0.1";
  public int rosPort = 10000;
  [Tooltip("Topic namespace, matching the container launch namespace.")]
  public string ns = "husky";

  [Header("Physics parity")]
  [Tooltip("MuJoCo timestep. The plugin steps at Unity's fixedDeltaTime and " +
           "IGNORES the MJCF <option timestep>, so this must equal the " +
           "container sim's timestep (0.002) for identical physics.")]
  public float mujocoTimestep = 0.002f;

  [Header("Rates (Hz)")]
  public float odomRate = 50f;   // clock + odom + TF publish together
  public float scanRate = 15f;

  [Header("Lidar")]
  public int scanRays = 720;
  public float scanMaxRange = 25f;
  public float scanMinRange = 0.1f;   // range_min in the LaserScan message

  // Perception is the camera, always: Unity renders it, the containerized
  // pixel detector turns pixels into a labeled point on perception/detections.
  // (Training loops that want ground-truth "is it in view" build it in the
  // pure-MuJoCo RL env, docs/rl.md, not here.)
  [Header("Camera")]
  public int camWidth = 320;
  public int camHeight = 240;
  public float camFovyDeg = 58f;   // MJCF camera_0 fovy
  public float camRate = 10f;

  // Husky drive geometry + command watchdog
  const float WheelRadius = 0.165f;
  const float TrackWidth = 0.5708f;
  const float CmdTimeoutSec = 0.5f;

  ROSConnection _ros;
  MjScene _scene;

  // MuJoCo ids, resolved after the scene compiles. The plugin RECOMPILES the
  // scene from the component tree at Play, and the runtime model's names are
  // not the MJCF names — so ids must come from the components' MujocoId, never
  // from mj_name2id. Nothing publishes or writes ctrl until _ready.
  bool _ready;
  int _baseBody = -1;
  int _lidarSite = -1;
  int _actLeft = -1;
  int _actRight = -1;
  int _cameraSite = -1;

  // cmd_vel state (written by the ROS thread, read in the physics step).
  // _simTime is the physics step's clock cached for the ROS thread — the ROS
  // thread must never dereference MjData itself (the physics thread owns it).
  readonly object _cmdLock = new object();
  double _cmdV, _cmdW;
  double _cmdStamp = -1.0;
  double _simTime;

  // publish pacing, in sim time
  double _nextOdom, _nextScan, _nextStaticTf, _nextCam;

  SparCameraSensor _camera;
  Transform _baseBodyTransform;

  // lidar scratch buffers
  double[] _rayOrigin = new double[3];
  double[] _rayDirs;
  int[] _rayGeomIds;
  double[] _rayDists;
  byte[] _rayGroups = { 1, 0, 0, 1, 0, 0 };  // groups 0 (world) + 3 (robot collision)
  float[] _ranges;

  string Topic(string t) => $"/{ns}/{t}";

  void Awake() {
    // Physics parity with the container sim: the plugin steps MuJoCo once per
    // FixedUpdate at fixedDeltaTime, ignoring the MJCF timestep.
    Time.fixedDeltaTime = mujocoTimestep;

    _rayDirs = new double[scanRays * 3];
    _rayGeomIds = new int[scanRays];
    _rayDists = new double[scanRays];
    _ranges = new float[scanRays];

    _ros = ROSConnection.GetOrCreateInstance();
    _ros.RosIPAddress = rosIPAddress;
    _ros.RosPort = rosPort;

    _ros.RegisterPublisher<ClockMsg>("/clock");
    _ros.RegisterPublisher<OdometryMsg>(Topic("platform/odom"));
    _ros.RegisterPublisher<LaserScanMsg>(Topic("sensors/lidar2d_0/scan"));
    // The container remaps /tf -> tf inside the namespace, so TF lives at
    // /husky/tf — publish there.
    _ros.RegisterPublisher<TFMessageMsg>(Topic("tf"));
    _ros.RegisterPublisher<TFMessageMsg>(Topic("tf_static"));
    _ros.Subscribe<TwistStampedMsg>(Topic("cmd_vel"), OnCmdVel);
  }

  void Start() {
    _scene = MjScene.Instance;  // creates + compiles the MuJoCo scene
    _scene.postUpdateEvent += (_, args) => PostPhysicsStep(args);
    _scene.ctrlCallback += (_, args) => ApplyCmd(args);
  }

  static T FindComponent<T>(string name) where T : MjComponent {
    foreach (var c in UnityEngine.Object.FindObjectsByType<T>(FindObjectsSortMode.None)) {
      if (c.name == name) return c;
    }
    return null;
  }

  static int FindId<T>(string name) where T : MjComponent {
    var c = FindComponent<T>(name);
    return c != null ? c.MujocoId : -1;
  }

  // Called from the physics-step callbacks until it succeeds: the MjScene
  // compiles during its own lifecycle, so ids are only trustworthy once the
  // step events start firing. (Static TF needs no kick-off here — the 1 Hz
  // republish below fires on the first ready step.)
  bool TryResolveIds() {
    var baseBody = FindComponent<MjBody>("base_link");
    _baseBody = baseBody != null ? baseBody.MujocoId : -1;
    _lidarSite = FindId<MjSite>("lidar2d_0_laser");
    _cameraSite = FindId<MjSite>("camera_0_link");
    _actLeft = FindId<MjActuator>("act_left");
    _actRight = FindId<MjActuator>("act_right");
    if (_baseBody < 0 || _lidarSite < 0 || _cameraSite < 0 ||
        _actLeft < 0 || _actRight < 0) {
      Debug.LogError(
          $"[spar] bridge could not resolve MujocoIds: base_link={_baseBody} " +
          $"lidar={_lidarSite} camera={_cameraSite} " +
          $"act_left={_actLeft} act_right={_actRight}");
      return false;
    }
    _baseBodyTransform = baseBody.transform;
    Debug.Log($"[spar] bridge ids ok: body={_baseBody} site={_lidarSite} " +
              $"actL={_actLeft} actR={_actRight}");
    return true;
  }

  // One-time setup that needs the compiled model: the MJCF dump and the
  // camera mount.
  unsafe void OnReady() {
    // World pipeline: MJCF only ever flows OUT of Unity. When asked (env var),
    // dump the exact model the physics is running — the same compile path,
    // so there is no separate exporter and no coordinate conversion to trust.
    // rasterize_map.py and the linter consume this dump (make map).
    var dumpPath = System.Environment.GetEnvironmentVariable("SPAR_DUMP_MJCF");
    if (!string.IsNullOrEmpty(dumpPath)) {
      var err = new System.Text.StringBuilder(1024);
      var ok = MujocoLib.mj_saveLastXML(dumpPath, _scene.Model, err, err.Capacity);
      Debug.Log(ok != 0
          ? $"[spar] MJCF dumped: {dumpPath}"
          : $"[spar] MJCF dump FAILED: {err}");
    }

    // Mount the camera at the MJCF site (MuJoCo local -> Unity local: (x,y,z)->(x,z,y)).
    var m = _scene.Model;
    var sitePos = new Vector3(
        (float)m->site_pos[3 * _cameraSite + 0],
        (float)m->site_pos[3 * _cameraSite + 2],
        (float)m->site_pos[3 * _cameraSite + 1]);
    _camera = new SparCameraSensor(
        _ros, Topic("sensors/camera_0/color/image"),
        Topic("sensors/camera_0/depth/image"),
        Topic("sensors/camera_0/color/camera_info"),
        _baseBodyTransform, sitePos, camWidth, camHeight, camFovyDeg);
    Debug.Log("[spar] camera sensor mounted");
  }

  // ------------------------------------------------------------------ input

  void OnCmdVel(TwistStampedMsg msg) {
    if (!_ready) return;
    lock (_cmdLock) {
      _cmdV = msg.twist.linear.x;
      _cmdW = msg.twist.angular.z;
      _cmdStamp = _simTime;
    }
  }

  unsafe void ApplyCmd(MjStepArgs args) {
    if (!_ready) return;
    double v, w, stamp;
    lock (_cmdLock) { v = _cmdV; w = _cmdW; stamp = _cmdStamp; }
    // stale command is no command
    if (stamp < 0.0 || args.data->time - stamp > CmdTimeoutSec) { v = 0.0; w = 0.0; }
    var wl = (v - w * TrackWidth / 2.0) / WheelRadius;
    var wr = (v + w * TrackWidth / 2.0) / WheelRadius;
    args.data->ctrl[_actLeft] = wl;
    args.data->ctrl[_actRight] = wr;
  }

  // ------------------------------------------------------------- publishing

  unsafe void PostPhysicsStep(MjStepArgs args) {
    if (!_ready) {
      if (!TryResolveIds()) return;
      OnReady();
      _ready = true;
    }
    var t = args.data->time;
    lock (_cmdLock) { _simTime = t; }
    if (t >= _nextOdom) {
      _nextOdom = t + 1.0 / odomRate;
      PublishClock(t);
      PublishOdomTf(args, t);
    }
    if (t >= _nextScan) {
      _nextScan = t + 1.0 / scanRate;
      PublishScan(args, t);
    }
    // tf_static is latched (transient-local) in native ROS, but the TCP
    // endpoint publishes with volatile QoS — a late-joining or restarted
    // container would never see it. Republish slowly instead.
    if (t >= _nextStaticTf) {
      _nextStaticTf = t + 1.0;
      PublishStaticTf();
    }
    if (_camera != null && t >= _nextCam) {
      _nextCam = t + 1.0 / camRate;
      _camera.Publish(Stamp(t));
    }
  }

  static TimeMsg Stamp(double t) =>
      new TimeMsg((int)t, (uint)((t - Math.Floor(t)) * 1e9));

  // Base-body yaw from its world quaternion (MuJoCo stores w x y z).
  unsafe double BaseYaw(MujocoLib.mjData_* d) {
    var qw = d->xquat[4 * _baseBody + 0];
    var qx = d->xquat[4 * _baseBody + 1];
    var qy = d->xquat[4 * _baseBody + 2];
    var qz = d->xquat[4 * _baseBody + 3];
    return Math.Atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }

  void PublishClock(double t) {
    _ros.Publish("/clock", new ClockMsg(Stamp(t)));
  }

  // Odometry conventions:
  //  - odom frame == world frame (drift-free odometry, robot spawns at origin)
  //  - orientation is yaw-only
  //  - twist is in the CHILD (base_link) frame via mj_objectVelocity with
  //    mjOBJ_XBODY and flg_local=1 — the hard-won lesson; anything else feeds
  //    Nav2 sign-flipped feedback off the +x heading.
  unsafe void PublishOdomTf(MjStepArgs args, double t) {
    var d = args.data;
    var px = d->xpos[3 * _baseBody + 0];
    var py = d->xpos[3 * _baseBody + 1];
    var pz = d->xpos[3 * _baseBody + 2];
    var yaw = BaseYaw(d);

    var vel = stackalloc double[6];  // [ang(3), lin(3)] in local frame
    MujocoLib.mj_objectVelocity(
        args.model, d, (int)MujocoLib.mjtObj.mjOBJ_XBODY, _baseBody, vel, 1);

    var stamp = Stamp(t);
    var ozw = Math.Cos(yaw / 2.0);
    var ozz = Math.Sin(yaw / 2.0);

    var odom = new OdometryMsg();
    odom.header = new HeaderMsg(stamp, "odom");
    odom.child_frame_id = "base_link";
    odom.pose.pose.position = new PointMsg(px, py, 0.0);
    odom.pose.pose.orientation = new QuaternionMsg(0.0, 0.0, ozz, ozw);
    odom.twist.twist.linear = new Vector3Msg(vel[3], 0.0, 0.0);
    odom.twist.twist.angular = new Vector3Msg(0.0, 0.0, vel[2]);
    _ros.Publish(Topic("platform/odom"), odom);

    var tf = new TransformStampedMsg(
        new HeaderMsg(stamp, "odom"), "base_link",
        new TransformMsg(
            new Vector3Msg(px, py, pz),
            new QuaternionMsg(0.0, 0.0, ozz, ozw)));
    _ros.Publish(Topic("tf"), new TFMessageMsg(new[] { tf }));
  }

  // Static TF: the sensor sites, expressed in the base_link frame.
  unsafe void PublishStaticTf() {
    var m = _scene.Model;
    var tfs = new TransformStampedMsg[2];
    var names = new[] { "lidar2d_0_laser", "camera_0_link" };
    for (var i = 0; i < names.Length; i++) {
      var site = FindId<MjSite>(names[i]);
      tfs[i] = new TransformStampedMsg(
          new HeaderMsg(Stamp(0.0), "base_link"), names[i],
          new TransformMsg(
              new Vector3Msg(
                  m->site_pos[3 * site + 0],
                  m->site_pos[3 * site + 1],
                  m->site_pos[3 * site + 2]),
              new QuaternionMsg(0.0, 0.0, 0.0, 1.0)));
    }
    _ros.Publish(Topic("tf_static"), new TFMessageMsg(tfs));
  }

  // Lidar: one revolution in the base plane, ray directions in the world
  // frame offset by the base yaw, mj_multiRay against geom groups 0+3
  // excluding the robot's own body, inf where nothing hit.
  unsafe void PublishScan(MjStepArgs args, double t) {
    var d = args.data;
    _rayOrigin[0] = d->site_xpos[3 * _lidarSite + 0];
    _rayOrigin[1] = d->site_xpos[3 * _lidarSite + 1];
    _rayOrigin[2] = d->site_xpos[3 * _lidarSite + 2];
    var yaw = BaseYaw(d);

    var angleInc = 2.0 * Math.PI / scanRays;  // [-pi, pi), endpoint excluded
    for (var i = 0; i < scanRays; i++) {
      var a = -Math.PI + i * angleInc + yaw;
      _rayDirs[3 * i + 0] = Math.Cos(a);
      _rayDirs[3 * i + 1] = Math.Sin(a);
      _rayDirs[3 * i + 2] = 0.0;
    }

    fixed (double* pnt = _rayOrigin, vec = _rayDirs, dist = _rayDists)
    fixed (byte* groups = _rayGroups)
    fixed (int* geomid = _rayGeomIds) {
      MujocoLib.mj_multiRay(
          args.model, d, pnt, vec, groups, 1, _baseBody,
          geomid, dist, null, scanRays, scanMaxRange);
    }

    for (var i = 0; i < scanRays; i++) {
      _ranges[i] = _rayGeomIds[i] >= 0 ? (float)_rayDists[i] : float.PositiveInfinity;
    }

    var scan = new LaserScanMsg {
      header = new HeaderMsg(Stamp(t), "lidar2d_0_laser"),
      angle_min = (float)(-Math.PI),
      angle_max = (float)(-Math.PI + (scanRays - 1) * angleInc),
      angle_increment = (float)angleInc,
      time_increment = 0f,
      scan_time = 1f / scanRate,
      range_min = scanMinRange,
      range_max = scanMaxRange,
      ranges = _ranges,
      intensities = Array.Empty<float>(),
    };
    _ros.Publish(Topic("sensors/lidar2d_0/scan"), scan);
  }
}
