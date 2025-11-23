using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    [System.Serializable]
    public struct RigidBodyConfig
    {
        [Tooltip("0 = Rigid (Concrete), 1 = Soft (Rubber).")] [Range(0f, 1f)]
        public float DeformationCompliance;

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
        [Header("Physical Properties")] public float TotalMass = 10.0f;
        public float ParticleRadius = 0.05f;
        public bool IsKinematic = false;
        public RigidBodyConfig Config = RigidBodyConfig.Default;

        [Header("Topology")] public BodyTopology GenerationMode = BodyTopology.BoxCornersOnly;

        [Header("Constraints")] public bool LockX = false;
        public bool LockY = false;
        public bool LockZ = false;
        public bool LockRotationX = false;
        public bool LockRotationY = false;
        public bool LockRotationZ = false;

        private PhysicsSceneHook _hook;
        private int _bodyIndex = -1;
        private List<int> _proxyIndices = new List<int>();
        private bool _isInitialized = false;
        private int _bodyInstanceID;

        void Start()
        {
            _hook = FindFirstObjectByType<PhysicsSceneHook>();
            if (_hook == null) return;
            _bodyInstanceID = GetInstanceID();
            InitializeBody();
        }

        // --- HELPERS ---
        private float3 CalculateInverseInertia(float mass, Vector3 dimensions)
        {
            if (mass < PhysicsConstants.Epsilon) return float3.zero;
            float invMass = 1.0f / mass;
            float x = dimensions.x;
            float y = dimensions.y;
            float z = dimensions.z;
            float invIx = 12.0f * invMass / (y * y + z * z);
            float invIy = 12.0f * invMass / (x * x + z * z);
            float invIz = 12.0f * invMass / (x * x + y * y);
            return new float3(invIx, invIy, invIz);
        }

        List<Vector3> CleanVerticesWithRemap(Vector3[] source, float threshold, out int[] remapTable)
        {
            List<Vector3> clean = new List<Vector3>();
            remapTable = new int[source.Length];
            float threshSq = threshold * threshold;
            for (int i = 0; i < source.Length; i++)
            {
                Vector3 p = source[i];
                int foundIndex = -1;
                for (int j = 0; j < clean.Count; j++)
                {
                    if (Vector3.SqrMagnitude(p - clean[j]) < threshSq)
                    {
                        foundIndex = j;
                        break;
                    }
                }

                if (foundIndex != -1) remapTable[i] = foundIndex;
                else
                {
                    clean.Add(p);
                    remapTable[i] = clean.Count - 1;
                }
            }

            return clean;
        }

        // --- INITIALIZATION ---
        void InitializeBody()
        {
            if (_isInitialized) return;
            Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null) return;

            List<Vector3> finalPoints = new List<Vector3>();
            Bounds b = mesh.bounds;
            int[] indexRemap = null;

            if (GenerationMode == BodyTopology.BoxCornersOnly)
            {
                finalPoints.Add(b.min);
                finalPoints.Add(b.max);
                finalPoints.Add(new Vector3(b.min.x, b.min.y, b.max.z));
                finalPoints.Add(new Vector3(b.min.x, b.max.y, b.min.z));
                finalPoints.Add(new Vector3(b.max.x, b.min.y, b.max.z));
                finalPoints.Add(new Vector3(b.max.x, b.max.y, b.min.z));
                finalPoints.Add(new Vector3(b.min.x, b.max.y, b.max.z));
                finalPoints.Add(new Vector3(b.max.x, b.min.y, b.min.z));
            }
            else
            {
                finalPoints = CleanVerticesWithRemap(mesh.vertices, 0.001f, out indexRemap);
            }

            bool useCenterStrut = (GenerationMode == BodyTopology.MeshVertices);
            if (useCenterStrut) finalPoints.Add(b.center);

            int count = finalPoints.Count;
            Vector3 localCOM = Vector3.zero;
            foreach (var p in finalPoints) localCOM += p;
            localCOM /= count;

            Vector3[] adjustedPoints = new Vector3[count];
            for (int i = 0; i < count; i++) adjustedPoints[i] = finalPoints[i] - localCOM;

            Vector3 dims = Vector3.Scale(transform.localScale, mesh.bounds.size);
            float simMass = IsKinematic ? 0.0f : TotalMass;
            float3 invInertia = CalculateInverseInertia(TotalMass, dims);

            float3 linLock = new float3(LockX ? 0 : 1, LockY ? 0 : 1, LockZ ? 0 : 1);
            float3 angLock = new float3(LockRotationX ? 0 : 1, LockRotationY ? 0 : 1, LockRotationZ ? 0 : 1);

            // 1. Add Body
            _bodyIndex = _hook.World.AddBody(
                (float3)transform.position, (quaternion)transform.rotation,
                simMass * linLock.x, invInertia * angLock, float3.zero
            );

            // 2. Add Proxies
            for (int i = 0; i < count; i++)
            {
                float r = (useCenterStrut && i == count - 1) ? 0.0f : ParticleRadius;
                _proxyIndices.Add(_hook.World.AddCollisionProxy(
                    (float3)adjustedPoints[i], r, _bodyIndex
                ));
            }

            // 3. Add Triangles (Collision Surface)
            if (GenerationMode == BodyTopology.MeshVertices)
            {
                int[] triangles = mesh.triangles;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    int idxA = indexRemap[triangles[i]];
                    int idxB = indexRemap[triangles[i + 1]];
                    int idxC = indexRemap[triangles[i + 2]];

                    int pA = _proxyIndices[idxA];
                    int pB = _proxyIndices[idxB];
                    int pC = _proxyIndices[idxC];

                    // FIX: Send _bodyIndex to prevent self-collision
                    _hook.World.AddTriangle(pA, pB, pC, _bodyIndex);
                }
            }

            _isInitialized = true;
        }

        // --- RUNTIME ---
        void FixedUpdate()
        {
            if (!_isInitialized || _bodyIndex == -1 || _hook == null || _hook.World == null) return;
            var simData = _hook.World.Context;

            float3 linLock = new float3(LockX ? 0 : 1, LockY ? 0 : 1, LockZ ? 0 : 1);
            float3 angLock = new float3(LockRotationX ? 0 : 1, LockRotationY ? 0 : 1, LockRotationZ ? 0 : 1);

            float invMass = (TotalMass > 0.00001f) ? 1.0f / TotalMass : 0.0f;
            Vector3 dims = Vector3.Scale(transform.localScale, GetComponent<MeshFilter>().sharedMesh.bounds.size);
            float3 invInertia = CalculateInverseInertia(TotalMass, dims);

            simData.BodyInverseMass[_bodyIndex] = IsKinematic ? 0.0f : invMass * linLock.x;
            simData.BodyInverseInertia[_bodyIndex] = invInertia * angLock;

            if (IsKinematic)
            {
                simData.BodyPosition[_bodyIndex] = (float3)transform.position;
                simData.BodyRotation[_bodyIndex] = (quaternion)transform.rotation;
                simData.BodyVelocity[_bodyIndex] = float3.zero;
                simData.BodyAngularVelocity[_bodyIndex] = float3.zero;

                // Sync predicted to avoid interpolation glitches
                simData.BodyPredictedPos[_bodyIndex] = (float3)transform.position;
                simData.BodyPredictedRot[_bodyIndex] = (quaternion)transform.rotation;
            }
            else
            {
                float3 pos = simData.BodyPosition[_bodyIndex];
                quaternion rot = simData.BodyRotation[_bodyIndex];

                if (!math.any(math.isnan(pos)))
                {
                    transform.position = (Vector3)pos;
                    transform.rotation = (Quaternion)rot;
                }
            }
        }

        void OnDrawGizmosSelected()
        {
            if (_proxyIndices.Count > 0 && _hook != null && _hook.World != null)
            {
                Gizmos.color = Color.green;
                var simData = _hook.World.Context;
                foreach (var idx in _proxyIndices)
                {
                    if (idx < simData.ParticleCount)
                        Gizmos.DrawSphere((Vector3)simData.CurrentPosition[idx], ParticleRadius);
                }
            }
        }
    }
}