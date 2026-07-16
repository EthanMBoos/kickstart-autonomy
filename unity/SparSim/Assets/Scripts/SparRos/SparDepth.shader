// Replacement shader for the depth pass of the robot camera: writes linear
// eye depth (meters along the view axis) into an RFloat target — the metric
// depth anomaly_detector's unprojection expects. Background clears to 0
// (= invalid; the detector rejects d <= 0.1).
Shader "Spar/Depth" {
  SubShader {
    Tags { "RenderType"="Opaque" }
    Pass {
      CGPROGRAM
      #pragma vertex vert
      #pragma fragment frag
      #include "UnityCG.cginc"

      struct v2f {
        float4 pos : SV_POSITION;
        float depth : TEXCOORD0;
      };

      v2f vert(appdata_base v) {
        v2f o;
        o.pos = UnityObjectToClipPos(v.vertex);
        COMPUTE_EYEDEPTH(o.depth);
        return o;
      }

      float4 frag(v2f i) : SV_Target {
        return float4(i.depth, 0, 0, 1);
      }
      ENDCG
    }
  }
}
