// Editor entry points for running the sim without a window.
//
// The scene (Assets/Scenes/BlankWorld.unity) is the canonical world: authored
// and edited in Unity, never generated from MJCF. The robot in it is the
// Husky prefab (Assets/Prefabs/Husky.prefab).

using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class SparBootstrap {

  // Open the scene and enter play mode — no clicking. Works both headless
  // (`make unity-headless`: -batchmode, editor stays alive publishing until
  // killed) and windowed (`make unity-gui`: the editor opens and starts
  // playing).
  public static void Play() {
    EditorSceneManager.OpenScene("Assets/Scenes/BlankWorld.unity");
    Debug.Log($"[spar] entering play mode " +
              $"({(Application.isBatchMode ? "headless" : "windowed")})");
    EditorApplication.isPlaying = true;
  }

  // World pipeline: enter play just long enough for the bridge to dump the
  // compiled model as MJCF (SPAR_DUMP_MJCF names the output), then exit.
  // Driven by `make map`, which feeds the dump to rasterize_map.py.
  //
  // The exit watcher must survive the domain reload that entering play mode
  // triggers, so it's re-armed from [InitializeOnLoadMethod] via SessionState
  // (which persists across reloads) rather than registered inline.
  public static void DumpWorld() {
    var path = System.Environment.GetEnvironmentVariable("SPAR_DUMP_MJCF");
    if (string.IsNullOrEmpty(path)) {
      Debug.LogError("[spar] DumpWorld: SPAR_DUMP_MJCF not set");
      EditorApplication.Exit(1);
      return;
    }
    if (System.IO.File.Exists(path)) System.IO.File.Delete(path);
    EditorSceneManager.OpenScene("Assets/Scenes/BlankWorld.unity");
    SessionState.SetBool("spar_dump_watch", true);
    SessionState.SetString("spar_dump_path", path);
    ArmDumpWatch();
    EditorApplication.isPlaying = true;
  }

  [InitializeOnLoadMethod]
  static void ResumeDumpWatchAfterReload() {
    if (SessionState.GetBool("spar_dump_watch", false)) ArmDumpWatch();
  }

  static void ArmDumpWatch() {
    var path = SessionState.GetString("spar_dump_path", "");
    var deadline = EditorApplication.timeSinceStartup + 120.0;
    EditorApplication.update += () => {
      if (System.IO.File.Exists(path)) {
        Debug.Log("[spar] dump complete, exiting");
        EditorApplication.Exit(0);
      } else if (EditorApplication.timeSinceStartup > deadline) {
        Debug.LogError("[spar] DumpWorld timed out");
        EditorApplication.Exit(1);
      }
    };
  }
}
