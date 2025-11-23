using UnityEngine;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;

namespace mehmetsrl.Extensions.VerletSystem
{
    [RequireComponent(typeof(LineRenderer))]
    public class LineRopeVisualizer : MonoBehaviour, IRopeVisualizer
    {
        private LineRenderer _lineRenderer;
        private Texture2D _gradientTexture;

        void Awake()
        {
            _lineRenderer = GetComponent<LineRenderer>();
            _lineRenderer.useWorldSpace = true;
            _lineRenderer.textureMode = LineTextureMode.Stretch;
        }

        public void UpdateVisualization(List<VerletNode> nodes, float thickness)
        {
            if (_lineRenderer == null) return;

            _lineRenderer.positionCount = nodes.Count;
            for (int i = 0; i < nodes.Count; i++)
            {
                _lineRenderer.SetPosition(i, nodes[i].position);
            }

            _lineRenderer.startWidth = thickness;
            _lineRenderer.endWidth = thickness;
        }

        public void SetColorSegments(List<RopeColorSection> segments)
        {
            if (_lineRenderer == null || segments == null || segments.Count == 0) return;

            // --- Texture Oluşturma Mantığı (Önceki koddan alındı) ---
            int width = 1;
            int height = 256;

            if (_gradientTexture == null || _gradientTexture.height != height)
            {
                _gradientTexture = new Texture2D(width, height);
                _gradientTexture.wrapMode = TextureWrapMode.Clamp;
                _gradientTexture.filterMode = FilterMode.Point; // Zebra deseni için Point
            }

            Color[] pixels = new Color[width * height];
            for (int i = 0; i < pixels.Length; i++) pixels[i] = Color.white;

            foreach (var segment in segments)
            {
                int startY = Mathf.FloorToInt(Mathf.Clamp01(segment.start) * height);
                int endY = Mathf.FloorToInt(Mathf.Clamp01(segment.end) * height);

                for (int y = startY; y < endY && y < height; y++)
                {
                    pixels[y] = segment.color;
                }
            }

            _gradientTexture.SetPixels(pixels);
            _gradientTexture.Apply();

            // Materyale Ata
            Material mat = _lineRenderer.material;
            if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", _gradientTexture); // URP
            else if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", _gradientTexture); // Standard

            // Vertex Color'ı beyaz yap ki texture görünsün
            _lineRenderer.startColor = Color.white;
            _lineRenderer.endColor = Color.white;
        }

        public void SetVisible(bool isVisible)
        {
            if (_lineRenderer) _lineRenderer.enabled = isVisible;
        }

        public void SetMaterial(Material material)
        {
            if (_lineRenderer != null) _lineRenderer.material = material;
        }
    }
}