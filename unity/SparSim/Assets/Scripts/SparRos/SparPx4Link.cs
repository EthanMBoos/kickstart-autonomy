// The PX4 HIL link: a peer of SparRosBridge for the air track. Per physics
// step it computes HIL_SENSOR (IMU, mag, baro) and HIL_GPS from MuJoCo
// state, sends them to PX4 SITL on :4560, blocks for the HIL_ACTUATOR_CONTROLS
// reply (lockstep), and writes the four normalized motor outputs onto the
// X2's thrust actuators. One socket, one loop, no MAVLink library: only
// four message types cross this link, so the framing is hand rolled below.
//
// Frames, decided once:
//  - MuJoCo world is right-handed Z-up; this file treats +X east, +Y north.
//    PX4 wants NED world and FRD body, so: north=+Y, east=+X, down=-Z, and
//    body FLU -> FRD is (x, -y, -z). The matching NED->ENU / FRD->FLU halves
//    live in the container's TF node, nowhere else.
//  - HIL timestamps are MuJoCo sim time, so PX4's clock equals /clock by
//    construction; a Unity restart rewinds both, which is why the restart
//    order in the Makefile header includes the air container.

using System;
using System.Net.Sockets;
using Mujoco;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SparPx4Link : MonoBehaviour {

  [Header("Connection")]
  // PX4 v1.17's simulator_mavlink is the TCP CLIENT (px4-rc.mavlinksim
  // starts it with -c): the simulator listens and PX4 dials out, so the
  // air container starts PX4 with PX4_SIM_HOSTNAME=host.docker.internal
  // and no port is published for this. An inbound mapping would do nothing.
  public int listenPort = 4560;
  [Tooltip("Lockstep receive timeout. On timeout the rotors zero (the drone " +
           "falls visibly instead of flying on stale commands), the link " +
           "drops, and PX4 needs a restart. Sized for editor GC hitches.")]
  public int timeoutMs = 100;

  [Header("Model names in the scene")]
  public string bodyName = "x2";
  // The rotors are a force module, not MjActuators: the plugin's MjActuator
  // only does joint/tendon transmission, and the X2's motors act at sites.
  // Each step the link applies thrust +z and the drag torque km*thrust
  // about z (both body frame) at each site via mj_applyFT.
  // PX4 outputs follow none_iris rotor geometry (CA_ROTOR0..3: front-right,
  // rear-left, front-left, rear-right). Sites listed in that PX4 order,
  // matched to the Menagerie X2 by position. The km signs are the
  // controller's, not the vendor's: CA_ROTOR_KM is defined in FRD (z DOWN),
  // so +KM there is a NEGATIVE z-up torque here. Pairing the Menagerie
  // gear signs naively inverts every yaw reaction and the drone spins up
  // and crashes seconds after a clean takeoff.
  // Not serialized fields: Unity bakes public-field values into the scene
  // at authoring time, and a stale copy of these once flew the drone into
  // the ground. They are wire facts, not tuning knobs.
  static readonly string[] SiteNames = { "thrust4", "thrust2", "thrust3", "thrust1" };
  static readonly float[] KmPerForce = { -0.0201f, -0.0201f, 0.0201f, 0.0201f };
  const float MaxThrustN = 13f;  // Menagerie X2 motor ctrlrange upper bound

  [Header("Geodetic origin (EKF2 local origin lands here)")]
  public double originLatDeg = 33.7756;   // GT campus
  public double originLonDeg = -84.3963;
  public double originAltM = 300.0;

  [Header("Camera")]
  public string ns = "skydio";
  // DevNote: both robots' cameras share the one ROS-TCP endpoint; if a
  // mixed demo ever drops frames, this rate is the first knob to turn.
  public float camRate = 10f;
  // 45 deg down-forward so patrol legs sweep the ground ahead; the
  // container's tf_from_px4 publishes the matching TF mount and the two
  // must agree (camera_pitch_down_deg in the air yaml).
  public float camPitchDownDeg = 45f;

  // Wire facts from the pinned PX4 build's generated headers
  // (build/px4_sitl_zenoh/mavlink/): message id, v1 payload length
  // (extension fields dropped -- the receiver zero-fills them), CRC_EXTRA.
  const byte MsgHeartbeat = 0, CrcHeartbeat = 50; const int LenHeartbeat = 9;
  const byte MsgHilSensor = 107, CrcHilSensor = 108; const int LenHilSensor = 64;
  const byte MsgHilGps = 113, CrcHilGps = 124; const int LenHilGps = 36;
  const int MsgHilActuators = 93;

  MjScene _scene;
  TcpListener _listener;
  TcpClient _tcp;
  NetworkStream _stream;
  bool _ready, _firstReply;
  int _baseBody = -1;
  readonly int[] _siteIds = { -1, -1, -1, -1 };
  readonly double[] _thrustN = new double[4];
  Transform _bodyTransform;
  SparCameraSensor _camera;
  double _nextCam;

  // PX4 SITL's simulation contract is a 250 Hz sensor stream (it replies
  // one HIL_ACTUATOR_CONTROLS per HIL_SENSOR at that rate; faster sends
  // get every-other replies and the lockstep read times out). MuJoCo keeps
  // stepping at 500 Hz; only the exchange is paced.
  const double SensorInterval = 0.004;

  byte _seq;
  double _nextSensor, _nextGps, _nextHeartbeat;
  int _freeDofAdr = -1;  // the freejoint's first dof: qacc linear accel, world frame

  readonly byte[] _tx = new byte[128];
  readonly byte[] _rx = new byte[512];

  // A noiseless sensor is a broken sensor to PX4: bit-identical readings
  // trip its stuck-sensor detection (the mag goes stale in seconds and EKF
  // never yaw-aligns), so every HIL simulator dithers its outputs.
  readonly System.Random _noise = new System.Random(1);
  double Dither(double sigma) {
    var u1 = 1.0 - _noise.NextDouble();
    var u2 = _noise.NextDouble();
    return sigma * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
  }

  void Start() {
    _scene = MjScene.Instance;
    _scene.ctrlCallback += (_, args) => Lockstep(args);
    _listener = new TcpListener(System.Net.IPAddress.Any, listenPort);
    _listener.Start();
  }

  void OnDestroy() {
    _listener?.Stop();
  }

  void Update() {
    if (_stream != null || _listener == null || !_listener.Pending()) return;
    _tcp = _listener.AcceptTcpClient();
    _tcp.NoDelay = true;  // Nagle would batch the per-step sends: no lockstep
    _stream = _tcp.GetStream();
    _stream.ReadTimeout = 1000;  // a partial frame must never hang the physics thread
    _firstReply = true;
    Debug.Log("[spar] px4 link connected");
  }

  void Drop(string why) {
    Debug.LogWarning($"[spar] px4 link dropped ({why}, after {_reads} lockstep " +
                     "reads); rotors zeroed, restart the air container to fly again");
    _reads = 0;
    _stream?.Close();
    _tcp?.Close();
    _stream = null;
    _tcp = null;
  }

  bool TryResolveIds() {
    var body = SparRosBridge.FindComponent<MjBody>(bodyName);
    _baseBody = body != null ? body.MujocoId : -1;
    for (var i = 0; i < 4; i++) {
      _siteIds[i] = SparRosBridge.FindId<MjSite>(SiteNames[i]);
    }
    if (_baseBody < 0 || Array.IndexOf(_siteIds, -1) >= 0) {
      Debug.LogError($"[spar] px4 link could not resolve MujocoIds: " +
                     $"body={_baseBody} sites=[{string.Join(",", _siteIds)}]");
      return false;
    }
    _bodyTransform = body.transform;
    Debug.Log($"[spar] px4 link ids ok: body={_baseBody} " +
              $"sites=[{string.Join(",", _siteIds)}]");
    return true;
  }

  // The drone reuses SparCameraSensor unchanged: it applies its own +x-facing
  // yaw inside, so the down-angle comes from a pre-rotated mount parent
  // (Unity z here is the MuJoCo body y the TF mount pitches about).
  void MountCamera() {
    var mount = new GameObject("drone_cam_mount");
    mount.transform.SetParent(_bodyTransform, worldPositionStays: false);
    mount.transform.localPosition = new Vector3(0.1f, 0.05f, 0f);
    mount.transform.localRotation = Quaternion.Euler(0f, 0f, -camPitchDownDeg);
    _camera = new SparCameraSensor(
        ROSConnection.GetOrCreateInstance(),
        $"/{ns}/sensors/camera_0/color/image",
        $"/{ns}/sensors/camera_0/depth/image",
        $"/{ns}/sensors/camera_0/color/camera_info",
        mount.transform, Vector3.zero, 320, 240, 58f);
    Debug.Log("[spar] drone camera mounted");
  }

  // ------------------------------------------------------------- lockstep

  unsafe void Lockstep(MjStepArgs args) {
    if (!_ready) {
      if (!TryResolveIds()) return;
      MountCamera();
      _ready = true;
    }
    // The camera publishes whenever the sim runs, PX4 up or not, like every
    // other sensor in the scene.
    var t = args.data->time;
    if (_camera != null && t >= _nextCam) {
      _nextCam = t + 1.0 / camRate;
      _camera.Publish(new TimeMsg((int)t, (uint)((t - Math.Floor(t)) * 1e9)));
    }
    // The drone owns qfrc_applied: cleared every step, then the rotor
    // forces for this step (zero while disconnected -- the drone falls).
    var nv = (int)args.model->nv;
    for (var i = 0; i < nv; i++) args.data->qfrc_applied[i] = 0.0;
    if (_stream == null) return;

    try {
      if (t >= _nextSensor) {
        _nextSensor = t + SensorInterval;
        if (t >= _nextHeartbeat) {
          _nextHeartbeat = t + 1.0;
          SendHeartbeat();
        }
        if (t >= _nextGps) {
          _nextGps = t + 0.1;
          SendHilGps(args, t);
        }
        SendHilSensor(args, t);
        // PX4's clock only advances on HIL_SENSOR timestamps, and it boots
        // (sensors, EKF, commander) across many steps before the first
        // controls message. Blocking for a reply from step one would
        // deadlock both sides, so the link free-runs until PX4's first
        // HIL_ACTUATOR_CONTROLS, then locks step for good.
        if (_firstReply) {
          PollActuators();
        } else if (!ReadActuators()) {
          Drop("lockstep timeout");
          return;
        }
      }
      ApplyRotorForces(args);
    } catch (Exception e) when (e is System.IO.IOException || e is SocketException
                                || e is ObjectDisposedException) {
      Drop(e.GetType().Name);
    }
  }

  unsafe void ApplyRotorForces(MjStepArgs args) {
    var d = args.data;
    var qw = d->xquat[4 * _baseBody + 0];
    var qx = d->xquat[4 * _baseBody + 1];
    var qy = d->xquat[4 * _baseBody + 2];
    var qz = d->xquat[4 * _baseBody + 3];
    var force = stackalloc double[3];
    var torque = stackalloc double[3];
    var point = stackalloc double[3];
    for (var i = 0; i < 4; i++) {
      var f = _thrustN[i];
      // Thrust +z and drag torque km*f about z, both body frame -> world.
      Rotate(qw, qx, qy, qz, 0.0, 0.0, f,
             out force[0], out force[1], out force[2]);
      Rotate(qw, qx, qy, qz, 0.0, 0.0, KmPerForce[i] * f,
             out torque[0], out torque[1], out torque[2]);
      point[0] = d->site_xpos[3 * _siteIds[i] + 0];
      point[1] = d->site_xpos[3 * _siteIds[i] + 1];
      point[2] = d->site_xpos[3 * _siteIds[i] + 2];
      MujocoLib.mj_applyFT(args.model, d, force, torque, point, _baseBody,
                           d->qfrc_applied);
    }
  }

  // Block until PX4's HIL_ACTUATOR_CONTROLS for this step arrives. PX4
  // sends exactly one per sensor step once lockstep engages
  // (SimulatorMavlink.cpp: lockstep progress on HIL_SENSOR, then
  // send_controls()), plus occasional heartbeats etc., which are skipped.
  // PX4 replies v2-framed (0xFD) with trailing-zero payload truncation.
  void PollActuators() {
    while (_stream.DataAvailable) {
      if (ParseFrame()) {
        _firstReply = false;
        Debug.Log("[spar] px4 lockstep engaged");
        return;
      }
    }
  }

  int _reads;  // successful lockstep reads since engage, for drop diagnostics

  bool ReadActuators() {
    _stream.ReadTimeout = timeoutMs;
    var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
    while (DateTime.UtcNow < deadline) {
      if (ParseFrame()) { _reads++; return true; }
    }
    return false;
  }

  // Reads one frame; true when it was HIL_ACTUATOR_CONTROLS (thrusts
  // stored). NetworkStream.Read throws IOException on ReadTimeout, which
  // the lockstep catch treats as a drop.
  bool ParseFrame() {
    if (!ReadExact(_rx, 1)) return false;
    int msgId, payloadLen;
    if (_rx[0] == 0xFD) {            // v2: len incompat compat seq sys comp id[3]
      if (!ReadExact(_rx, 9)) return false;
      payloadLen = _rx[0];
      msgId = _rx[6] | (_rx[7] << 8) | (_rx[8] << 16);
    } else if (_rx[0] == 0xFE) {     // v1: len seq sys comp id
      if (!ReadExact(_rx, 5)) return false;
      payloadLen = _rx[0];
      msgId = _rx[4];
    } else {
      return false;  // resync on the next magic byte
    }
    if (!ReadExact(_rx, payloadLen + 2)) return false;  // payload + crc
    if (msgId != MsgHilActuators) return false;
    // controls[16] floats at offset 16 (after time_usec u64 + flags u64),
    // zero-extended when truncated; motors are 0..1 normalized.
    for (var i = 0; i < 4; i++) {
      var off = 16 + 4 * i;
      var c = off + 4 <= payloadLen ? BitConverter.ToSingle(_rx, off) : 0f;
      _thrustN[i] = Math.Clamp(c, 0f, 1f) * MaxThrustN;
    }
    return true;
  }

  bool ReadExact(byte[] buf, int n) {
    var got = 0;
    while (got < n) {
      var r = _stream.Read(buf, got, n - got);
      if (r <= 0) return false;
      got += r;
    }
    return true;
  }

  // ------------------------------------------------------------- sensors

  unsafe void SendHilSensor(MjStepArgs args, double t) {
    var d = args.data;
    var m = args.model;

    // The accelerometer must read specific force, f = R^T (a - g), which
    // is -g rotated into the body at rest, not zero. The freejoint's
    // translational qacc from the last completed step IS the body's linear
    // acceleration in world coordinates: exact, no finite differencing
    // (an aliased hand-rolled derivative here diverges EKF2 in flight).
    if (_freeDofAdr < 0) {
      _freeDofAdr = m->jnt_dofadr[m->body_jntadr[_baseBody]];
    }
    var ax = d->qacc[_freeDofAdr + 0];
    var ay = d->qacc[_freeDofAdr + 1];
    var az = d->qacc[_freeDofAdr + 2];

    // Body-frame (FLU) angular velocity straight from MuJoCo.
    var velLocal = stackalloc double[6];
    MujocoLib.mj_objectVelocity(
        m, d, (int)MujocoLib.mjtObj.mjOBJ_XBODY, _baseBody, velLocal, 1);

    // World -> body via the conjugate of xquat (w x y z, body -> world).
    var qw = d->xquat[4 * _baseBody + 0];
    var qx = d->xquat[4 * _baseBody + 1];
    var qy = d->xquat[4 * _baseBody + 2];
    var qz = d->xquat[4 * _baseBody + 3];

    double fbx, fby, fbz;
    RotateByConjugate(qw, qx, qy, qz, ax, ay, az + 9.80665,
                      out fbx, out fby, out fbz);

    // Mid-latitude magnetic field, NED (gauss) -> world ENU -> body.
    const double magN = 0.21523, magD = 0.42741;
    double mbx, mby, mbz;
    RotateByConjugate(qw, qx, qy, qz, 0.0, magN, -magD, out mbx, out mby, out mbz);

    // ISA barometer from geometric altitude.
    var alt = d->xpos[3 * _baseBody + 2] + originAltM;
    var pressureHPa = 1013.25 * Math.Pow(1.0 - 2.25577e-5 * alt, 5.2559);

    var p = PayloadStart(MsgHilSensor, LenHilSensor);
    p = PutU64(p, (ulong)(t * 1e6));
    // FLU -> FRD: keep x, negate y and z.
    p = PutF(p, (float)(fbx + Dither(0.01)));
    p = PutF(p, (float)(-fby + Dither(0.01)));
    p = PutF(p, (float)(-fbz + Dither(0.01)));
    p = PutF(p, (float)(velLocal[0] + Dither(0.001)));
    p = PutF(p, (float)(-velLocal[1] + Dither(0.001)));
    p = PutF(p, (float)(-velLocal[2] + Dither(0.001)));
    p = PutF(p, (float)(mbx + Dither(0.002)));
    p = PutF(p, (float)(-mby + Dither(0.002)));
    p = PutF(p, (float)(-mbz + Dither(0.002)));
    p = PutF(p, (float)(pressureHPa + Dither(0.02)));
    p = PutF(p, 0f);
    p = PutF(p, (float)alt);
    p = PutF(p, 20f);
    PutU32(p, 0x1FFF);  // all fields valid
    Send(LenHilSensor, CrcHilSensor);
  }

  unsafe void SendHilGps(MjStepArgs args, double t) {
    var d = args.data;
    var north = d->xpos[3 * _baseBody + 1];
    var east = d->xpos[3 * _baseBody + 0];
    var up = d->xpos[3 * _baseBody + 2];
    var vel = stackalloc double[6];
    MujocoLib.mj_objectVelocity(
        args.model, d, (int)MujocoLib.mjtObj.mjOBJ_XBODY, _baseBody, vel, 0);
    var vn = vel[4]; var ve = vel[3]; var vd = -vel[5];

    const double earthR = 6378137.0;
    var latRad = originLatDeg * Math.PI / 180.0;
    var lat = originLatDeg + (north / earthR) * (180.0 / Math.PI);
    var lon = originLonDeg + (east / (earthR * Math.Cos(latRad))) * (180.0 / Math.PI);

    var p = PayloadStart(MsgHilGps, LenHilGps);
    p = PutU64(p, (ulong)(t * 1e6));
    p = PutI32(p, (int)(lat * 1e7));
    p = PutI32(p, (int)(lon * 1e7));
    p = PutI32(p, (int)((originAltM + up) * 1000.0));
    p = PutU16(p, 30); p = PutU16(p, 60);  // eph/epv, dilution * 100
    p = PutU16(p, (ushort)(Math.Sqrt(vn * vn + ve * ve) * 100.0));
    p = PutI16(p, (short)(vn * 100));
    p = PutI16(p, (short)(ve * 100));
    p = PutI16(p, (short)(vd * 100));
    p = PutU16(p, ushort.MaxValue);  // course over ground: unknown
    _tx[p] = 3; _tx[p + 1] = 12;     // 3D fix, 12 sats
    Send(LenHilGps, CrcHilGps);
  }

  void SendHeartbeat() {
    var p = PayloadStart(MsgHeartbeat, LenHeartbeat);
    p = PutU32(p, 0);
    _tx[p] = 6;      // type MAV_TYPE_GCS keeps PX4 from treating us as a vehicle
    _tx[p + 1] = 8;  // autopilot MAV_AUTOPILOT_INVALID
    _tx[p + 2] = 0; _tx[p + 3] = 0; _tx[p + 4] = 3;
    Send(LenHeartbeat, CrcHeartbeat);
  }

  // q * v * q^-1 (body vector -> world) and its conjugate form (world ->
  // body), for unit q stored MuJoCo-style (w x y z).
  static void Rotate(double qw, double qx, double qy, double qz,
                     double vx, double vy, double vz,
                     out double rx, out double ry, out double rz) {
    var tx = 2.0 * (qy * vz - qz * vy);
    var ty = 2.0 * (qz * vx - qx * vz);
    var tz = 2.0 * (qx * vy - qy * vx);
    rx = vx + qw * tx + (qy * tz - qz * ty);
    ry = vy + qw * ty + (qz * tx - qx * tz);
    rz = vz + qw * tz + (qx * ty - qy * tx);
  }

  static void RotateByConjugate(double qw, double qx, double qy, double qz,
                                double vx, double vy, double vz,
                                out double rx, out double ry, out double rz) {
    Rotate(qw, -qx, -qy, -qz, vx, vy, vz, out rx, out ry, out rz);
  }

  // ---------------------------------------------------- MAVLink v1 framing
  // STX len seq sys comp msgid payload crc16, X25 CRC over len..payload
  // plus the per-message CRC_EXTRA byte. Offsets index into _tx.

  int PayloadStart(byte msgId, int len) {
    _tx[0] = 0xFE;
    _tx[1] = (byte)len;
    _tx[2] = _seq++;
    _tx[3] = 1;    // sysid: the simulated vehicle
    _tx[4] = 200;  // compid: not the autopilot
    _tx[5] = msgId;
    Array.Clear(_tx, 6, len);
    return 6;
  }

  void Send(int len, byte crcExtra) {
    ushort crc = 0xFFFF;
    for (var i = 1; i < 6 + len; i++) crc = CrcAccumulate(_tx[i], crc);
    crc = CrcAccumulate(crcExtra, crc);
    _tx[6 + len] = (byte)(crc & 0xFF);
    _tx[7 + len] = (byte)(crc >> 8);
    _stream.Write(_tx, 0, 8 + len);
  }

  static ushort CrcAccumulate(byte b, ushort crc) {
    var tmp = (byte)(b ^ (crc & 0xFF));
    tmp = (byte)(tmp ^ (tmp << 4));
    return (ushort)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
  }

  int PutU64(int p, ulong v) {
    for (var i = 0; i < 8; i++) _tx[p + i] = (byte)(v >> (8 * i));
    return p + 8;
  }
  int PutU32(int p, uint v) {
    for (var i = 0; i < 4; i++) _tx[p + i] = (byte)(v >> (8 * i));
    return p + 4;
  }
  int PutI32(int p, int v) => PutU32(p, (uint)v);
  int PutU16(int p, ushort v) { _tx[p] = (byte)v; _tx[p + 1] = (byte)(v >> 8); return p + 2; }
  int PutI16(int p, short v) => PutU16(p, (ushort)v);
  int PutF(int p, float v) => PutU32(p, (uint)BitConverter.SingleToInt32Bits(v));
}
