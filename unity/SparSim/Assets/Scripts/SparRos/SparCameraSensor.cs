// The robot camera: renders color + metric depth from a Unity Camera mounted
// at the MJCF camera site and publishes the three topics the pixel detector
// consumes: rgb8 color, 32FC1 eye-depth (meters), and camera_info with
// K = [fy 0 cx / 0 fy cy / 0 0 1] — the shapes anomaly_detector expects.
//
// Not a MonoBehaviour: the bridge owns the cadence and calls Publish from
// the physics-step callback (main thread, so rendering is legal).

using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SparCameraSensor {
  readonly ROSConnection _ros;
  readonly string _colorTopic, _depthTopic, _infoTopic;
  readonly int _w, _h;
  readonly float _fovyDeg;

  readonly Camera _cam;
  readonly RenderTexture _colorRT, _depthRT;
  readonly Texture2D _colorTex, _depthTex;
  readonly Shader _depthShader;
  readonly byte[] _rgb;
  readonly byte[] _depthBytes;

  public SparCameraSensor(
      ROSConnection ros, string colorTopic, string depthTopic, string infoTopic,
      Transform baseBody, Vector3 localPos, int width, int height, float fovyDeg) {
    _ros = ros;
    _colorTopic = colorTopic;
    _depthTopic = depthTopic;
    _infoTopic = infoTopic;
    _w = width;
    _h = height;
    _fovyDeg = fovyDeg;

    _depthShader = Shader.Find("Spar/Depth");

    // Mount at the camera site, facing the robot's +x (Unity local +Z -> +X).
    var go = new GameObject("camera_0_sensor");
    go.transform.SetParent(baseBody, worldPositionStays: false);
    go.transform.localPosition = localPos;
    go.transform.localRotation = Quaternion.Euler(0f, 90f, 0f);

    _cam = go.AddComponent<Camera>();
    _cam.fieldOfView = fovyDeg;   // Unity's fieldOfView is vertical == fovy
    _cam.nearClipPlane = 0.05f;   // MJCF znear
    _cam.farClipPlane = 100f;
    _cam.enabled = false;         // rendered manually, on the sensor cadence

    _colorRT = new RenderTexture(_w, _h, 24, RenderTextureFormat.ARGB32);
    _depthRT = new RenderTexture(_w, _h, 24, RenderTextureFormat.RFloat);
    _colorTex = new Texture2D(_w, _h, TextureFormat.RGBA32, false);
    _depthTex = new Texture2D(_w, _h, TextureFormat.RFloat, false);
    _rgb = new byte[_w * _h * 3];
    _depthBytes = new byte[_w * _h * 4];

    _ros.RegisterPublisher<ImageMsg>(_colorTopic);
    _ros.RegisterPublisher<ImageMsg>(_depthTopic);
    _ros.RegisterPublisher<CameraInfoMsg>(_infoTopic);
  }

  public void Publish(TimeMsg stamp) {
    var header = new HeaderMsg(stamp, "camera_0_color_optical_frame");

    // Color pass (normal shaders, skybox background).
    _cam.clearFlags = CameraClearFlags.Skybox;
    _cam.targetTexture = _colorRT;
    _cam.Render();
    ReadInto(_colorTex, _colorRT);

    // Depth pass (replacement shader -> linear eye depth; background = 0).
    _cam.clearFlags = CameraClearFlags.SolidColor;
    _cam.backgroundColor = Color.clear;
    _cam.targetTexture = _depthRT;
    _cam.RenderWithShader(_depthShader, "");
    ReadInto(_depthTex, _depthRT);
    _cam.targetTexture = null;

    // Pack color: GPU readback is bottom-up; ROS wants the top row first.
    var pixels = _colorTex.GetPixels32();
    for (var row = 0; row < _h; row++) {
      var src = (_h - 1 - row) * _w;
      var dst = row * _w * 3;
      for (var col = 0; col < _w; col++) {
        var p = pixels[src + col];
        _rgb[dst + 3 * col + 0] = p.r;
        _rgb[dst + 3 * col + 1] = p.g;
        _rgb[dst + 3 * col + 2] = p.b;
      }
    }
    _ros.Publish(_colorTopic, new ImageMsg {
      header = header,
      height = (uint)_h, width = (uint)_w,
      encoding = "rgb8", is_bigendian = 0,
      step = (uint)(_w * 3), data = _rgb,
    });

    // Pack depth: same vertical flip, rows of little-endian float32 meters.
    var depths = _depthTex.GetRawTextureData<float>().ToArray();
    for (var row = 0; row < _h; row++) {
      var src = (_h - 1 - row) * _w;
      System.Buffer.BlockCopy(depths, src * 4, _depthBytes, row * _w * 4, _w * 4);
    }
    _ros.Publish(_depthTopic, new ImageMsg {
      header = header,
      height = (uint)_h, width = (uint)_w,
      encoding = "32FC1", is_bigendian = 0,
      step = (uint)(_w * 4), data = _depthBytes,
    });

    var fy = (_h / 2.0) / System.Math.Tan(_fovyDeg * System.Math.PI / 360.0);
    _ros.Publish(_infoTopic, new CameraInfoMsg {
      header = header,
      height = (uint)_h, width = (uint)_w,
      k = new[] { fy, 0.0, _w / 2.0, 0.0, fy, _h / 2.0, 0.0, 0.0, 1.0 },
    });
  }

  static void ReadInto(Texture2D tex, RenderTexture rt) {
    var prev = RenderTexture.active;
    RenderTexture.active = rt;
    tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
    RenderTexture.active = prev;
  }
}
