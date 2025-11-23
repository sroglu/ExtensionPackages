using System.Collections.Generic;
using Sirenix.OdinInspector;
using UnityEngine;

namespace mehmetsrl.Extensions.VerletSystem
{
    [RequireComponent(typeof(VerletRope))]
    public class RopeColorizer : MonoBehaviour
    {
        // Debug amaçlı inspector'da görmek isterseniz
        public List<RopeColorSection> debugSegments = new();

        // Texture cache
        private Texture2D gradientTexture;

        // Obi referansı yerine kendi VerletRope'umuz
        public VerletRope verletRope;

        private void Awake()
        {
            if (verletRope == null) verletRope = GetComponent<VerletRope>();
        }

        [Button]
        public void UpdateRopeColors(List<RopeColorSection> segments)
        {
            debugSegments = segments;
            if (verletRope == null) return;

            LineRenderer lineRenderer = verletRope.GetComponent<LineRenderer>();
            if (lineRenderer == null) return;

            // --- 1. TEXTURE OLUŞTURMA ---
            var width = 1;
            var height = 256;

            if (gradientTexture == null || gradientTexture.height != height)
            {
                gradientTexture = new Texture2D(width, height);
                gradientTexture.wrapMode = TextureWrapMode.Clamp; // Tekrar etmesin
                gradientTexture.filterMode =
                    FilterMode.Point; // Keskin geçişler (Zebra için Point iyi, yumuşak için Bilinear)
            }

            var pixels = new Color[width * height];

            // Arka planı temizle (default beyaz)
            for (var i = 0; i < pixels.Length; i++) pixels[i] = Color.white;

            // Segmentlere göre piksel boyama
            // Not: RopeColorSection start-end aralığını kullanıyoruz.
            foreach (var segment in segments)
            {
                var startY = Mathf.FloorToInt(Mathf.Clamp01(segment.start) * height);
                var endY = Mathf.FloorToInt(Mathf.Clamp01(segment.end) * height);

                // startY'den endY'ye kadar boya
                for (var y = startY; y < endY && y < height; y++) pixels[y] = segment.color;
            }

            gradientTexture.SetPixels(pixels);
            gradientTexture.Apply();

            // --- 2. LINE RENDERER MATERYALİNE ATAMA ---

            // LineRenderer'ın materyaline eriş (Instance oluşturur)
            var mat = lineRenderer.material;

            // Texture'ı shader'ın ilgili slotuna ata
            // Standart Shaderlar için:
            mat.mainTexture = gradientTexture;

            // URP/HDRP veya Custom Shaderlar için (Genellikle _BaseMap veya _MainTex):
            if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", gradientTexture);
            if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", gradientTexture);

            // LineRenderer'ın kendi vertex rengini BEYAZ yapmalıyız ki Texture görünsün
            // Yoksa vertex color texture'ın üzerine çarpan (multiply) olarak gelir.
            lineRenderer.startColor = Color.white;
            lineRenderer.endColor = Color.white;

            // Texture Mode'un 'Stretch' olduğundan emin olmalıyız
            lineRenderer.textureMode = LineTextureMode.Stretch;
        }
    }
}