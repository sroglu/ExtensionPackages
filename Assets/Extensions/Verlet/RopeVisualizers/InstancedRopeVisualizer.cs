using UnityEngine;
using System.Collections.Generic;
using mehmetsrl.Extensions.VerletSystem.Solver;

namespace mehmetsrl.Extensions.VerletSystem
{
    public class InstancedRopeVisualizer : MonoBehaviour, IRopeVisualizer
    {
        [Header("Instancing Settings")] [Tooltip("Zincir halkası veya ip parçası modeli")]
        public Mesh linkMesh;

        [Tooltip("Custom/SoftToonyURP shader'lı materyal")]
        public Material defaultMaterial;

        [Tooltip("Her parçanın boyut çarpanı")]
        public Vector3 scaleCorrection = Vector3.one;

        [Tooltip("Texture'ın ip boyunca yayılması. 1 = İp boyunca 1 kere tekrar et.")]
        public float textureTiling = 1.0f;

        // --- Internal State ---
        private Material _currentMaterial;
        private bool _isVisible = true;

        // GPU Instancing Buffers
        private Matrix4x4[] _matrices;
        private Vector4[] _diffuseColors; // Shader'daki _DiffuseColor için
        private Vector4[] _textureSTs; // Shader'daki _MainTex_ST için (Scale-Offset)

        private MaterialPropertyBlock _propertyBlock;

        // Gradient Cache
        private List<RopeColorSection> _cachedSegments;
        private Texture2D _generatedTexture;

        void Awake()
        {
            _propertyBlock = new MaterialPropertyBlock();

            // Başlangıç materyalini ayarla
            SetMaterial(defaultMaterial);
        }

        /// <summary>
        /// Fizik motorundan gelen veriyi görselleştirir.
        /// Rope.cs veya PPFVerletRope tarafından her kare çağrılır.
        /// </summary>
        public void UpdateVisualization(List<VerletNode> nodes, float thickness)
        {
            // 1. Görünürlük ve Veri Kontrolü
            if (!_isVisible || linkMesh == null || _currentMaterial == null || nodes == null || nodes.Count < 2)
                return;

            int count = nodes.Count;

            // GPU Instancing, tek draw call'da en fazla 1023 instance çizebilir.
            // Daha fazlası için döngüyü bölmek gerekir (Burada basitlik için limitliyoruz).
            if (count > 1023) count = 1023;

            // 2. Buffer Boyut Kontrolü (GC'yi azaltmak için sadece gerekirse yeniden oluştur)
            if (_matrices == null || _matrices.Length != count)
            {
                _matrices = new Matrix4x4[count];
                _diffuseColors = new Vector4[count];
                _textureSTs = new Vector4[count];
            }

            // 3. Instance Verilerini Hesapla
            for (int i = 0; i < count; i++)
            {
                // A. Pozisyon ve Rotasyon
                Vector3 pos = nodes[i].position;
                Quaternion rot = Quaternion.identity;

                // Bir sonraki düğüme bak
                if (i < count - 1)
                {
                    Vector3 dir = (nodes[i + 1].position - pos);

                    // Vektörün uzunluğunun karesi 0'dan (veya çok küçük bir sayıdan) büyük mü diye bakıyoruz.
                    // Vector3.zero check'inden daha performanslıdır.
                    if (dir.sqrMagnitude > 0.00001f)
                    {
                        rot = Quaternion.LookRotation(dir);
                    }
                }
                else if (i > 0)
                {
                    // Son parça bir öncekine göre hizalanır
                    rot = Quaternion.LookRotation(pos - nodes[i - 1].position);
                }

                // Zincir efekti: Her halkayı 90 derece çevir (İsteğe bağlı, halat ise kapatılabilir)
                if (i % 2 == 0) rot *= Quaternion.Euler(0, 0, 90);

                // B. Scale
                Vector3 finalScale = scaleCorrection * thickness;
                _matrices[i] = Matrix4x4.TRS(pos, rot, finalScale);

                // C. Renk ve Texture Hesaplaması
                float t = (float)i / (float)(count - 1); // 0 ile 1 arası normalize değer

                // Renk (Gradient)
                // Listeden rengi o anki 't' değerine göre buluyoruz
                Color col = EvaluateColorFromList(t);
                _diffuseColors[i] = col;

                // Texture Mapping (Zebra veya desenin ip boyunca akması için)
                // _MainTex_ST: (Tiling X, Tiling Y, Offset X, Offset Y)
                // Y eksenini ip boyunca yayılan eksen olarak varsayıyoruz.
                // Tiling.y = textureTiling / count (Texture'ı parçalara böl)
                // Offset.y = t * textureTiling (Parçayı kaydır)
                float tileY = textureTiling / (float)count;
                float offsetY = t * textureTiling;

                // X ekseni normal kalsın (1, 0)
                _textureSTs[i] = new Vector4(1, tileY, 0, offsetY);
            }

            // 4. Property Block'a Verileri Yükle
            // Shader'ındaki değişken isimleri: _DiffuseColor ve _MainTex_ST
            _propertyBlock.SetVectorArray("_DiffuseColor", _diffuseColors);
            _propertyBlock.SetVectorArray("_MainTex_ST", _textureSTs);

            // 5. Çizim Çağrısı
            // Shadow casting ayarı: ShadowCastingMode.On
            Graphics.DrawMeshInstanced(linkMesh, 0, _currentMaterial, _matrices, count, _propertyBlock,
                UnityEngine.Rendering.ShadowCastingMode.On, true);
        }
        
        /// <summary>
        /// Renk bölümlerini (Gradient) ayarlar.
        /// </summary>
        public void SetColorSegments(List<RopeColorSection> segments)
        {
            _cachedSegments = segments;

            // Texture oluşturma (Opsiyonel - Eğer shader texture kullanıyorsa)
            // Senin shader'ın _MainTex ve _DiffuseColor kullanıyor.
            // Biz _DiffuseColor'ı yukarıda instance array ile sürüyoruz.
            // Ancak texture tabanlı bir materyal kullanacaksan, gradient texture'ı da oluşturup atayabiliriz.

            if (segments == null || segments.Count == 0) return;

            int height = 256;
            if (_generatedTexture == null)
            {
                _generatedTexture = new Texture2D(1, height);
                _generatedTexture.wrapMode = TextureWrapMode.Clamp;
                _generatedTexture.filterMode = FilterMode.Bilinear;
            }

            Color[] pixels = new Color[height];
            for (int i = 0; i < height; i++)
            {
                float t = (float)i / (height - 1);
                pixels[i] = EvaluateColorFromList(t); // Helper metodumuzu kullanıyoruz
            }

            _generatedTexture.SetPixels(pixels);
            _generatedTexture.Apply();

            // Texture'ı materyale ata (Böylece _MainTex_ST ile oynadığımızda bu texture kayar)
            ApplyTextureToMaterial(_currentMaterial, _generatedTexture);
        }

        /// <summary>
        /// Materyali değiştirir ve GPU Instancing'i zorlar.
        /// </summary>
        public void SetMaterial(Material material)
        {
            if (material == null) return;

            _currentMaterial = material;

            // Shader instancing desteklese bile materyalde kapalı olabilir, zorla aç.
            if (!_currentMaterial.enableInstancing)
            {
                _currentMaterial.enableInstancing = true;
            }

            // Eğer önceden oluşturulmuş bir texture varsa, yeni materyale de uygula
            if (_generatedTexture != null)
            {
                ApplyTextureToMaterial(_currentMaterial, _generatedTexture);
            }
        }

        /// <summary>
        /// Görünürlüğü açıp kapatır.
        /// </summary>
        public void SetVisible(bool isVisible)
        {
            _isVisible = isVisible;
        }

        // --- Helper Methods ---

        private void ApplyTextureToMaterial(Material mat, Texture2D tex)
        {
            if (mat == null || tex == null) return;

            // Shader özelliklerini kontrol et
            if (mat.HasProperty("_MainTex"))
                mat.SetTexture("_MainTex", tex);
            else if (mat.HasProperty("_BaseMap")) // URP Default
                mat.SetTexture("_BaseMap", tex);
        }

// RopeColorSection listesinden 't' (0..1) anındaki rengi bulur
        private Color EvaluateColorFromList(float t)
        {
            // EĞER LİSTE BOŞSA:
            if (_cachedSegments == null || _cachedSegments.Count == 0)
            {
                // HATA DÜZELTMESİ:
                // Color.white döndürmek yerine, atalı olan materyalin rengini döndür.
                // Böylece inspector'da seçtiğin Kırmızı renk görünür.
                if (_currentMaterial != null && _currentMaterial.HasProperty("_DiffuseColor"))
                {
                    return _currentMaterial.GetColor("_DiffuseColor");
                }

                return Color.white; // Materyal yoksa beyaz dön
            }

            // ... (Geri kalan mantık aynı) ...
            foreach (var seg in _cachedSegments)
            {
                if (t >= seg.start && t <= seg.end)
                {
                    return seg.color;
                }
            }

            return _cachedSegments[^1].color;
        }
    }
}