using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    [System.Serializable]
    public struct RigidBodyConfig
    {
        [Tooltip("0 = Rigid (Concrete), 1 = Soft (Rubber).")]
        [Range(0f, 1f)] public float DeformationCompliance; 
        public static RigidBodyConfig Default => new RigidBodyConfig { DeformationCompliance = 0.0f };
    }

    public enum BodyTopology
    {
        BoxCornersOnly, 
        MeshVertices    
    }

    [RequireComponent(typeof(MeshFilter))]
    public class XPBDRigidBody : MonoBehaviour
    {
        // --- PUBLIC FIELDS ---
        public float TotalMass = 10.0f;
        public float ParticleRadius = 0.05f; 
        public bool IsKinematic = false;
        public RigidBodyConfig Config = RigidBodyConfig.Default;
        
        public BodyTopology GenerationMode = BodyTopology.BoxCornersOnly;
        public int MaxVertexLimit = 2500;

        public bool LockX = false; public bool LockY = false; public bool LockZ = false;
        public bool LockRotationX = false; public bool LockRotationY = false; public bool LockRotationZ = false;

        // --- RUNTIME STATE ---
        private PhysicsSceneHook _hook;
        private int _bodyIndex = -1;
        private List<int> _proxyIndices = new List<int>(); 
        
        // OPTIMIZATION: Removed _initialLocalPositions field to save memory.
        // We will use 'simData.ParticleLocalPosition' from the Engine instead.

        private Quaternion _initialRotation;
        private bool _isInitialized = false;
        private int _bodyInstanceID;

        void Start()
        {
            _hook = FindFirstObjectByType<PhysicsSceneHook>();
            if (_hook == null) return;
            _bodyInstanceID = GetInstanceID();
            _bodyInstanceID = GetInstanceID(); 
            InitializeBody();
        }

        // --- INITIALIZATION ---

        void InitializeBody()
        {
            if (_isInitialized) return;
            Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null) return;

            // Performance Safety Check
            if (GenerationMode == BodyTopology.MeshVertices && mesh.vertexCount > MaxVertexLimit)
            {
                Debug.LogWarning($"[XPBDRigidBody] Vertex count {mesh.vertexCount} > {MaxVertexLimit}. Switching to BoxCornersOnly.");
                GenerationMode = BodyTopology.BoxCornersOnly;
            }

            _initialRotation = transform.rotation;
            List<Vector3> finalPoints = new List<Vector3>();
            Bounds b = mesh.bounds;
            int[] indexRemap = null;

            // 1. Generate Points
            if (GenerationMode == BodyTopology.BoxCornersOnly)
            {
                finalPoints.Add(b.min); finalPoints.Add(b.max);
                finalPoints.Add(new Vector3(b.min.x, b.min.y, b.max.z));
                finalPoints.Add(new Vector3(b.min.x, b.max.y, b.min.z));
                finalPoints.Add(new Vector3(b.max.x, b.min.y, b.max.z));
                finalPoints.Add(new Vector3(b.max.x, b.max.y, b.min.z));
                finalPoints.Add(new Vector3(b.min.x, b.max.y, b.max.z));
                finalPoints.Add(new Vector3(b.max.x, b.min.y, b.min.z));
            }
            else // MeshVertices
            {
                // OPTIMIZATION: O(N) Welding
                finalPoints = CleanVerticesWithRemap(mesh.vertices, 0.001f, out indexRemap);
            }

            bool useCenterStrut = (GenerationMode == BodyTopology.MeshVertices);
            if (useCenterStrut) finalPoints.Add(b.center); 

            int count = finalPoints.Count;
            
            // 2. Calculate COM & Local Offsets
            Vector3 localCOM = Vector3.zero;
            foreach (var p in finalPoints) localCOM += p;
            localCOM /= count;

            // Local variable for initialization only
            Vector3[] localOffsets = new Vector3[count];
            for(int i=0; i<count; i++) localOffsets[i] = finalPoints[i] - localCOM;

            // 3. Setup Body
            Vector3 dims = Vector3.Scale(transform.localScale, mesh.bounds.size);
            float simMass = IsKinematic ? 0.0f : TotalMass;
            float3 invInertia = CalculateInverseInertia(simMass, dims);
            
            float3 linLockMask = new float3(LockX?0:1, LockY?0:1, LockZ?0:1);
            float3 angLockMask = new float3(LockRotationX?0:1, LockRotationY?0:1, LockRotationZ?0:1);

            _bodyIndex = _hook.World.AddBody(
                (float3)transform.position, (quaternion)transform.rotation, 
                simMass * linLockMask.x, invInertia * angLockMask, float3.zero
            );

            // 4. Add Proxies
            for (int i = 0; i < count; i++)
            {
                float r = (useCenterStrut && i == count - 1) ? 0.0f : ParticleRadius;
                _proxyIndices.Add(_hook.World.AddCollisionProxy(
                    (float3)localOffsets[i], r, _bodyIndex
                ));
            }

            // 5. Topology
            if (GenerationMode == BodyTopology.MeshVertices)
            {
                int[] triangles = mesh.triangles;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    int idxA = indexRemap[triangles[i]];
                    int idxB = indexRemap[triangles[i+1]];
                    int idxC = indexRemap[triangles[i+2]];
                    
                    int pA = _proxyIndices[idxA];
                    int pB = _proxyIndices[idxB];
                    int pC = _proxyIndices[idxC];
                    
                    _hook.World.AddTriangle(pA, pB, pC, _bodyIndex);
                }
                
                // Note: Removed Edge Constraints & Struts as RBD handles rigidity internally.
            }
            
            _isInitialized = true;
        }

        // --- HELPERS (OPTIMIZED) ---

        // OPTIMIZATION: O(N) Hash-Based Vertex Welding
        List<Vector3> CleanVerticesWithRemap(Vector3[] source, float threshold, out int[] remapTable)
        {
            List<Vector3> clean = new List<Vector3>();
            remapTable = new int[source.Length];
            
            // Use Dictionary for O(1) lookup instead of O(N) linear search
            Dictionary<Vector3Int, int> uniqueVerts = new Dictionary<Vector3Int, int>();
            float inverseStep = 1.0f / threshold;

            for (int i = 0; i < source.Length; i++)
            {
                Vector3 p = source[i];
                
                // Quantize position to grid bucket
                Vector3Int key = new Vector3Int(
                    Mathf.RoundToInt(p.x * inverseStep),
                    Mathf.RoundToInt(p.y * inverseStep),
                    Mathf.RoundToInt(p.z * inverseStep)
                );

                if (uniqueVerts.TryGetValue(key, out int index))
                {
                    // Duplicate found
                    remapTable[i] = index;
                }
                else
                {
                    // New vertex
                    clean.Add(p);
                    int newIndex = clean.Count - 1;
                    uniqueVerts.Add(key, newIndex);
                    remapTable[i] = newIndex;
                }
            }
            return clean;
        }

        private float3 CalculateInverseInertia(float mass, Vector3 dimensions)
        {
            if (mass < PhysicsConstants.Epsilon) return float3.zero;
            float invMass = 1.0f / mass;
            float x = dimensions.x; float y = dimensions.y; float z = dimensions.z;
            float invIx = 12.0f * invMass / (y * y + z * z);
            float invIy = 12.0f * invMass / (x * x + z * z);
            float invIz = 12.0f * invMass / (x * x + y * y);
            return new float3(invIx, invIy, invIz);
        }

        // --- RUNTIME SYNC ---

        void FixedUpdate()
        {
            if (!_isInitialized || _bodyIndex == -1 || _hook == null || _hook.World == null) return;
            var simData = _hook.World.Context;

            float3 linLockMask = new float3(LockX?0:1, LockY?0:1, LockZ?0:1);
            float3 angLockMask = new float3(LockRotationX?0:1, LockRotationY?0:1, LockRotationZ?0:1);

            float invMass = (TotalMass > 0.00001f) ? 1.0f / TotalMass : 0.0f;
            Vector3 dims = Vector3.Scale(transform.localScale, GetComponent<MeshFilter>().sharedMesh.bounds.size);
            float3 invInertia = CalculateInverseInertia(TotalMass, dims);
            
            simData.BodyInverseMass[_bodyIndex] = IsKinematic ? 0.0f : invMass * linLockMask.x;
            simData.BodyInverseInertia[_bodyIndex] = invInertia * angLockMask;

            if (IsKinematic)
            {
                simData.BodyPosition[_bodyIndex] = (float3)transform.position;
                simData.BodyRotation[_bodyIndex] = (quaternion)transform.rotation;
                simData.BodyVelocity[_bodyIndex] = float3.zero;
                simData.BodyAngularVelocity[_bodyIndex] = float3.zero;
                
                // Sync predicted to prevent interpolation glitch
                simData.BodyPredictedPos[_bodyIndex] = (float3)transform.position;
                simData.BodyPredictedRot[_bodyIndex] = (quaternion)transform.rotation;
            }
            else
            {
                // Physics -> Unity
                float3 pos = simData.BodyPosition[_bodyIndex];
                quaternion rot = simData.BodyRotation[_bodyIndex];
                
                if(!math.any(math.isnan(pos))) 
                {
                    transform.position = (Vector3)pos;
                    transform.rotation = (Quaternion)rot;
                }
            }
        }
        
        // OPTIMIZATION: Removed UpdateTransformAndApplyRotationLocks
        // Rigidity is now handled natively by the Core Engine (RBD), 
        // so we don't need manual shape matching or position correction here.
        // Visual sync is done in FixedUpdate by reading BodyPosition directly.

        void OnDrawGizmosSelected()
        {
            if (_proxyIndices.Count > 0 && _hook != null && _hook.World != null)
            {
                Gizmos.color = Color.green;
                var simData = _hook.World.Context;
                foreach (var idx in _proxyIndices)
                {
                    if(idx < simData.ParticleCount)
                        Gizmos.DrawSphere((Vector3)simData.CurrentPosition[idx], ParticleRadius);
                }
            }
        }
    }
}