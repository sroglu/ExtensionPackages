using UnityEngine;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;

namespace mehmetsrl.Extensions.VerletSystem
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class TubeRopeVisualizer : MonoBehaviour, IRopeVisualizer
    {
        [Header("Mesh Settings")]
        public int radialSegments = 8; // Silindirin yuvarlaklık detayı
        public bool smoothNormals = true;
        
        private MeshFilter _meshFilter;
        private MeshRenderer _meshRenderer;
        private Mesh _mesh;
        
        // Texture Cache
        private Texture2D _gradientTexture;

        // Mesh Data Buffers
        private Vector3[] vertices;
        private int[] triangles;
        private Vector2[] uvs;
        private Vector3[] normals;

        void Awake()
        {
            _meshFilter = GetComponent<MeshFilter>();
            _meshRenderer = GetComponent<MeshRenderer>();
            
            _mesh = new Mesh();
            _mesh.MarkDynamic(); // Sık güncelleneceğini Unity'e bildirir
            _mesh.name = "ProceduralRope";
            _meshFilter.mesh = _mesh;
        }

        public void UpdateVisualization(List<VerletNode> nodes, float thickness)
        {
            if (nodes == null || nodes.Count < 2) return;

            int edgeLoops = nodes.Count;
            int vertCount = edgeLoops * (radialSegments + 1);
            int triCount = (edgeLoops - 1) * radialSegments * 6;

            // Bufferları boyutlandır (Gerekirse)
            if (vertices == null || vertices.Length != vertCount)
            {
                vertices = new Vector3[vertCount];
                uvs = new Vector2[vertCount];
                triangles = new int[triCount];
                normals = new Vector3[vertCount];
            }

            // Vertexleri Hesapla
            for (int i = 0; i < nodes.Count; i++)
            {
                // Düğümün yönünü hesapla (LookAt mantığı)
                Vector3 currentPos = nodes[i].position;
                Vector3 forward;

                if (i < nodes.Count - 1) forward = (nodes[i + 1].position - currentPos).normalized;
                else forward = (currentPos - nodes[i - 1].position).normalized;

                // Quaternion rotasyonu (Z ekseni forward olacak şekilde)
                Quaternion rotation = Quaternion.LookRotation(forward);

                float t = (float)i / (nodes.Count - 1); // 0 ile 1 arası normalize uzunluk

                // Halka (Ring) oluştur
                for (int s = 0; s <= radialSegments; s++)
                {
                    float angle = (float)s / radialSegments * Mathf.PI * 2.0f;
                    
                    // Yerel çember noktası
                    Vector3 localPos = new Vector3(Mathf.Cos(angle) * thickness, Mathf.Sin(angle) * thickness, 0);
                    
                    // Rotasyon ve Pozisyon uygula
                    Vector3 worldPos = currentPos + (rotation * localPos);
                    
                    // Local Space'e çevir (Mesh Filter transformuna göre)
                    vertices[i * (radialSegments + 1) + s] = transform.InverseTransformPoint(worldPos);
                    
                    // UV: X yatay tekrar, Y dikey gradient (renk için önemli olan Y)
                    uvs[i * (radialSegments + 1) + s] = new Vector2((float)s / radialSegments, t);
                }
            }

            // Üçgenleri Hesapla (Sadece ilk oluşumda veya topoloji değişirse yapılabilir, ama dinamik yapıda her zaman güvenlidir)
            int triIndex = 0;
            for (int i = 0; i < nodes.Count - 1; i++)
            {
                for (int s = 0; s < radialSegments; s++)
                {
                    int current = i * (radialSegments + 1) + s;
                    int next = (i + 1) * (radialSegments + 1) + s;

                    // Quad'ın iki üçgeni
                    triangles[triIndex++] = current;
                    triangles[triIndex++] = next;
                    triangles[triIndex++] = current + 1;

                    triangles[triIndex++] = next;
                    triangles[triIndex++] = next + 1;
                    triangles[triIndex++] = current + 1;
                }
            }

            // Mesh'i güncelle
            _mesh.Clear();
            _mesh.vertices = vertices;
            _mesh.uv = uvs;
            _mesh.triangles = triangles;
            
            if(smoothNormals) _mesh.RecalculateNormals();
        }

        public void SetColorSegments(List<RopeColorSection> segments)
        {
            // --- Texture Oluşturma (Aynı Mantık) ---
            int height = 256;
            if (_gradientTexture == null)
            {
                _gradientTexture = new Texture2D(1, height);
                _gradientTexture.wrapMode = TextureWrapMode.Clamp;
                _gradientTexture.filterMode = FilterMode.Bilinear; // Mesh için yumuşak geçiş iyidir
            }

            Color[] pixels = new Color[height];
            for (int i = 0; i < height; i++) pixels[i] = Color.white;

            foreach (var seg in segments)
            {
                int start = Mathf.FloorToInt(Mathf.Clamp01(seg.start) * height);
                int end = Mathf.FloorToInt(Mathf.Clamp01(seg.end) * height);
                for (int y = start; y < end; y++) pixels[y] = seg.color;
            }
            
            _gradientTexture.SetPixels(pixels);
            _gradientTexture.Apply();

            // Shader'a Texture Ata
            Material mat = _meshRenderer.material;
            // URP: _BaseMap, Built-in: _MainTex
            if(mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", _gradientTexture);
            else mat.mainTexture = _gradientTexture;
        }
        
        public void SetVisible(bool isVisible) => _meshRenderer.enabled = isVisible;
        
        public void SetMaterial(Material material)
        {
            if (_meshRenderer != null) _meshRenderer.material = material;
        }
    }
}