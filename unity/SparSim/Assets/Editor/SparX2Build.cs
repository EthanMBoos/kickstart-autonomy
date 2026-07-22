// One-time builder for the Skydio X2 prefab, run headless:
//   Unity -batchmode -quit -executeMethod SparX2Build.Build
//
// Numbers come from third_party/skydio_x2/x2.xml (MuJoCo Menagerie, meshes
// by Skydio, Apache 2.0). The plugin importer was not used because the X2's
// motors act at sites (<motor site=... gear="0 0 1 0 0 km">), which
// MjActuator cannot express; instead the prefab carries only geometry,
// mass, a freejoint, and the four thrust sites, and SparPx4Link applies
// rotor forces at those sites each step. All mass lives in the four rotor
// geoms (0.25 each) and the central ellipsoid (0.325), as upstream.
// Delete this file once the prefab is settled; git history keeps it.

using Mujoco;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class SparX2Build {

  const string ObjPath = "Assets/Models/SkydioX2/X2_lowpoly.obj";
  const string TexPath = "Assets/Models/SkydioX2/X2_lowpoly_texture_SpinningProps_1024.png";
  const string MatPath = "Assets/Materials/x2_mat.mat";
  const string PrefabPath = "Assets/Prefabs/SkydioX2.prefab";

  static GameObject AddGeom(GameObject parent, string name, Vector3 mjPos) {
    var go = new GameObject(name);
    go.transform.SetParent(parent.transform, false);
    go.transform.localPosition = MjEngineTool.UnityVector3(mjPos);
    return go;
  }

  static void Collision(MjGeom g) {
    g.Settings.Filtering.Group = 3;  // robot collision, same group as the Husky
  }

  public static void Build() {
    // The Menagerie OBJ is authored in centimeters (MJCF mesh scale 0.01).
    var importer = (ModelImporter)AssetImporter.GetAtPath(ObjPath);
    importer.globalScale = 0.01f;
    importer.SaveAndReimport();

    var root = new GameObject("x2");
    root.AddComponent<MjBody>();
    var free = new GameObject("x2_free");
    free.transform.SetParent(root.transform, false);
    free.AddComponent<MjFreeJoint>();

    // Chassis collision boxes (mass 0: all mass is rotors + core, upstream).
    var boxes = new[] {
      (new Vector3(.04f, 0, .02f), new Vector3(.06f, .027f, .02f), 0f),
      (new Vector3(.04f, 0, .06f), new Vector3(.06f, .027f, .02f), 0f),
      (new Vector3(-.07f, 0, .065f), new Vector3(.05f, .027f, .02f), 0f),
      (new Vector3(-.137f, .008f, .065f), new Vector3(.023f, .017f, .01f), 90f),
    };
    for (var i = 0; i < boxes.Length; i++) {
      var go = AddGeom(root, $"box{i}", boxes[i].Item1);
      // MJCF quat "1 0 0 1" on the last box: +90 deg about MuJoCo z.
      if (boxes[i].Item3 != 0) go.transform.localRotation = Quaternion.Euler(0, -boxes[i].Item3, 0);
      var g = go.AddComponent<MjGeom>();
      g.ShapeType = MjShapeComponent.ShapeTypes.Box;
      g.Box.Extents = MjEngineTool.UnityExtents(boxes[i].Item2);
      g.Density = 0;  // Mass=0 means "use density" to the plugin; upstream wants massless
      Collision(g);
    }

    // Rotor guards: the collision footprint and the mass.
    var rotors = new[] {
      new Vector3(-.14f, -.18f, .05f), new Vector3(-.14f, .18f, .05f),
      new Vector3(.14f, .18f, .08f), new Vector3(.14f, -.18f, .08f),
    };
    for (var i = 0; i < 4; i++) {
      var go = AddGeom(root, $"rotor{i + 1}", rotors[i]);
      var g = go.AddComponent<MjGeom>();
      g.ShapeType = MjShapeComponent.ShapeTypes.Ellipsoid;
      g.Ellipsoid.Radiuses = MjEngineTool.UnityExtents(new Vector3(.13f, .13f, .01f));
      g.Mass = 0.25f;
      Collision(g);
      var site = AddGeom(root, $"thrust{i + 1}", rotors[i]);
      site.AddComponent<MjSite>();
    }

    // Central body mass, no contacts (upstream's invisible ellipsoid).
    var core = AddGeom(root, "core", new Vector3(0, 0, .02f));
    var coreGeom = core.AddComponent<MjGeom>();
    coreGeom.ShapeType = MjShapeComponent.ShapeTypes.Ellipsoid;
    coreGeom.Ellipsoid.Radiuses = MjEngineTool.UnityExtents(new Vector3(.16f, .04f, .02f));
    coreGeom.Mass = 0.325f;
    coreGeom.Settings.Filtering.Contype = 0;
    coreGeom.Settings.Filtering.Conaffinity = 0;
    coreGeom.Settings.Filtering.Group = 2;

    // Visual shell: plain Unity renderer, no Mj component (the costume).
    var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(TexPath);
    var mat = new Material(Shader.Find("Standard")) { mainTexture = tex };
    AssetDatabase.CreateAsset(mat, MatPath);
    var model = AssetDatabase.LoadAssetAtPath<GameObject>(ObjPath);
    var visual = Object.Instantiate(model, root.transform);
    visual.name = "x2_visual";
    // The MJCF visual quat "0 0 1 1" must NOT be re-applied here: it maps
    // mesh (x,y,z)->(-x,z,y), the plugin's MuJoCo->Unity swap gives
    // (-x,y,z), and Unity's OBJ import already mirrors x — identity.
    visual.transform.localRotation = Quaternion.identity;
    foreach (var r in visual.GetComponentsInChildren<MeshRenderer>()) {
      r.sharedMaterial = mat;
    }

    PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
    Object.DestroyImmediate(root);

    // Place it in the world, inert like the docked Husky, plus the link.
    // Idempotent: reruns replace the instance instead of stacking drones.
    var scene = EditorSceneManager.OpenScene("Assets/Scenes/BlankWorld.unity");
    foreach (var name in new[] { "x2", "SkydioX2", "SparPx4Link", "landing_pad" }) {
      var old = GameObject.Find(name);
      if (old != null) Object.DestroyImmediate(old);
    }
    var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
    var drone = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
    drone.name = "x2";  // the link resolves the MjBody by GameObject name
    // MuJoCo (-2.0, -1.0, 0.1): off the Husky's patrol legs and transit
    // diagonals, clear of the racks; pad_x/pad_y in the air yaml match.
    drone.transform.position = new Vector3(-2.0f, 0.1f, -1.0f);
    var link = new GameObject("SparPx4Link");
    link.AddComponent<SparPx4Link>();

    // The landing pad, visual only like dock_pad: no MjGeom, so it can't
    // become a stealth obstacle and the map never changes.
    var pad = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
    pad.name = "landing_pad";
    Object.DestroyImmediate(pad.GetComponent<Collider>());
    pad.transform.position = new Vector3(-2.0f, 0.005f, -1.0f);
    pad.transform.localScale = new Vector3(1.0f, 0.005f, 1.0f);
    var padMat = new Material(Shader.Find("Standard")) {
      color = new Color(0.25f, 0.3f, 0.4f)
    };
    AssetDatabase.CreateAsset(padMat, "Assets/Materials/pad_mat.mat");
    pad.GetComponent<MeshRenderer>().sharedMaterial = padMat;
    EditorSceneManager.SaveScene(scene);
    Debug.Log("[spar] x2 prefab built and placed");
    if (Application.isBatchMode) EditorApplication.Exit(0);
  }
}
