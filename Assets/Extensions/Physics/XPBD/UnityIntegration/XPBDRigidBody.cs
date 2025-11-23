using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    [System.Serializable]
    public struct RigidBodyConfig
    {
        [Tooltip("Inverse Stiffness. 0 = Rigid (Concrete), 1 = Soft (Rubber).")]
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
        [Header("Physical Properties")]
        public float TotalMass = 10.0f;
        public float ParticleRadius = 0.05f; 
        public bool IsKinematic = false;
        public RigidBodyConfig Config = RigidBodyConfig.Default;
        
        [Header("Topology")]
        public BodyTopology GenerationMode = BodyTopology.BoxCornersOnly;

        [Header("Constraints")]
        public bool LockX = false; public bool LockY = false; public bool LockZ = false;
        public bool LockRotationX = false; public bool LockRotationY = false; public bool LockRotationZ = false;

        // Runtime State
        private PhysicsSceneHook _hook;
        private int[] _particleIndices; 
        private Vector3[] _initialLocalPositions;
        private Quaternion _initialRotation;
        private bool _isInitialized = false;
        private int _bodyInstanceID;

        void Start()
        {
            _hook = FindFirstObjectByType<PhysicsSceneHook>();
            if (_hook == null) return;
            _bodyInstanceID = GetInstanceID(); 
            InitializeBody();
        }

        // --- CORE INITIALIZATION ---

        void InitializeBody()
        {
            if (_isInitialized) return;
            Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
            if (mesh == null) return;

            _initialRotation = transform.rotation;
            List<Vector3> finalPoints = new List<Vector3>();
            Bounds b = mesh.bounds;

            int[] indexRemap = null;

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
                finalPoints = CleanVerticesWithRemap(mesh.vertices, 0.001f, out indexRemap);
            }

            // Center Strut setup
            bool useCenterStrut = (GenerationMode == BodyTopology.MeshVertices);
            int centerPointIndex = -1;
            if (useCenterStrut) finalPoints.Add(b.center); 

            // 2. CREATE PARTICLES
            int count = finalPoints.Count;
            _particleIndices = new int[count];
            
            Vector3 localCOM = Vector3.zero;
            foreach (var p in finalPoints) localCOM += p;
            localCOM /= count;

            _initialLocalPositions = new Vector3[count];
            for(int i=0; i<count; i++) _initialLocalPositions[i] = finalPoints[i] - localCOM;

            float massPerParticle = IsKinematic ? 0.0f : (TotalMass / count);
            float stiffness = (Config.DeformationCompliance < PhysicsConstants.Epsilon) ? PhysicsConstants.MaxStiffness : (1.0f / Config.DeformationCompliance);
            float3 lockMask = new float3(LockX?0:1, LockY?0:1, LockZ?0:1);

            for (int i = 0; i < count; i++)
            {
                Vector3 worldPos = transform.TransformPoint(_initialLocalPositions[i]);
                float r = (useCenterStrut && i == count - 1) ? 0.0f : ParticleRadius;

                _particleIndices[i] = _hook.World.AddBody(
                    worldPos, massPerParticle, r, lockMask, _bodyInstanceID, stiffness
                );
            }

            if (useCenterStrut) centerPointIndex = _particleIndices[count - 1];

            // 3. CREATE TOPOLOGY
            if (GenerationMode == BodyTopology.MeshVertices)
            {
                int[] triangles = mesh.triangles;
                HashSet<long> addedEdges = new HashSet<long>(); 

                for (int i = 0; i < triangles.Length; i += 3)
                {
                    int idxA_New = indexRemap[triangles[i]];
                    int idxB_New = indexRemap[triangles[i+1]];
                    int idxC_New = indexRemap[triangles[i+2]];

                    // A. Edges (Structure)
                    AddEdgeConstraint(idxA_New, idxB_New, addedEdges, _initialLocalPositions);
                    AddEdgeConstraint(idxB_New, idxC_New, addedEdges, _initialLocalPositions);
                    AddEdgeConstraint(idxC_New, idxA_New, addedEdges, _initialLocalPositions);
                    
                    // B. Triangles (Collision Surface)
                    if (idxA_New < count && idxB_New < count && idxC_New < count)
                    {
                        int pA = _particleIndices[idxA_New];
                        int pB = _particleIndices[idxB_New];
                        int pC = _particleIndices[idxC_New];
                        _hook.World.AddTriangle(pA, pB, pC);
                    }
                }

                // C. Center Struts
                if (centerPointIndex != -1)
                {
                    Vector3 centerLocalPos = _initialLocalPositions[count - 1];
                    float strutCompliance = Mathf.Max(Config.DeformationCompliance, 0.0001f);

                    for (int i = 0; i < count - 1; i++)
                    {
                        float restDist = Vector3.Distance(_initialLocalPositions[i], centerLocalPos);
                        _hook.World.AddConstraint(_particleIndices[i], centerPointIndex, restDist, strutCompliance);
                    }
                }
            }
            else // Box Corners
            {
                for (int i = 0; i < count; i++)
                {
                    for (int j = i + 1; j < count; j++)
                    {
                        float restDist = Vector3.Distance(_initialLocalPositions[i], _initialLocalPositions[j]);
                        _hook.World.AddConstraint(_particleIndices[i], _particleIndices[j], restDist, Config.DeformationCompliance);
                    }
                }
            }

            _isInitialized = true;
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

                if (foundIndex != -1)
                {
                    remapTable[i] = foundIndex;
                }
                else
                {
                    clean.Add(p);
                    remapTable[i] = clean.Count - 1;
                }
            }
            return clean;
        }

        void AddEdgeConstraint(int localIdxA, int localIdxB, HashSet<long> edges, Vector3[] points)
        {
            if (localIdxA == localIdxB) return;

            int a = Mathf.Min(localIdxA, localIdxB);
            int b = Mathf.Max(localIdxA, localIdxB);
            long key = ((long)a << 32) | (uint)b;

            if (!edges.Contains(key))
            {
                edges.Add(key);
                float dist = Vector3.Distance(points[a], points[b]);
                _hook.World.AddConstraint(_particleIndices[a], _particleIndices[b], dist, Config.DeformationCompliance);
            }
        }

        // --- FIXED UPDATE & SHAPE MATCHING ---

        void FixedUpdate()
        {
            if (!_isInitialized || _particleIndices == null || _hook == null || _hook.World == null) return;
            var simData = _hook.World.Context;

            float3 lockMask = new float3(LockX?0:1, LockY?0:1, LockZ?0:1);

            for (int i = 0; i < _particleIndices.Length; i++)
            {
                int idx = _particleIndices[i];
                if (idx >= simData.ParticleCount) continue;

                simData.PositionLockMask[idx] = lockMask;

                if (IsKinematic)
                {
                    Vector3 targetPos = transform.TransformPoint(_initialLocalPositions[i]);
                    simData.CurrentPosition[idx] = targetPos;
                    simData.PredictedPosition[idx] = targetPos;
                    simData.Velocity[idx] = float3.zero;
                    simData.InverseMass[idx] = 0.0f;
                }
                else
                {
                    if (simData.InverseMass[idx] == 0.0f && TotalMass > 0)
                         simData.InverseMass[idx] = 1.0f / (TotalMass / _particleIndices.Length);
                }
            }

            if (!IsKinematic) UpdateTransformAndApplyRotationLocks();
        }

        void UpdateTransformAndApplyRotationLocks()
        {
            var simData = _hook.World.Context;
            int count = _particleIndices.Length;

            // 1. Center of Mass
            Vector3 currentCOM = Vector3.zero;
            int validParticles = 0;

            for (int i = 0; i < count; i++)
            {
                int idx = _particleIndices[i];
                if (idx < simData.ParticleCount)
                {
                    float3 p = simData.CurrentPosition[idx];
                    if (math.any(math.isnan(p)) || math.any(math.isinf(p))) return; 
                    currentCOM += (Vector3)p;
                    validParticles++;
                }
            }
            
            if (validParticles == 0) return;
            currentCOM /= validParticles;

            // 2. Rotation (Shape Matching)
            Vector3 col0 = Vector3.zero, col1 = Vector3.zero, col2 = Vector3.zero;
            for (int i = 0; i < count; i++)
            {
                int idx = _particleIndices[i];
                if (idx >= simData.ParticleCount) continue;

                Vector3 p = (Vector3)simData.CurrentPosition[idx] - currentCOM;
                Vector3 q = Vector3.Scale(_initialLocalPositions[i], transform.localScale); 

                col0 += p * q.x; col1 += p * q.y; col2 += p * q.z;
            }

            Matrix4x4 A = Matrix4x4.identity;
            A.SetColumn(0, col0); A.SetColumn(1, col1); A.SetColumn(2, col2); A[3, 3] = 1;

            Vector3 forward = A.GetColumn(2); 
            Vector3 up = A.GetColumn(1);      
            Quaternion derivedRotation = transform.rotation;

            if (forward.sqrMagnitude > PhysicsConstants.Epsilon && up.sqrMagnitude > PhysicsConstants.Epsilon)
                derivedRotation = Quaternion.LookRotation(forward, up);

            // 3. Locks
            Vector3 currentEuler = derivedRotation.eulerAngles;
            Vector3 initEuler = _initialRotation.eulerAngles;

            if (LockRotationX) currentEuler.x = initEuler.x;
            if (LockRotationY) currentEuler.y = initEuler.y;
            if (LockRotationZ) currentEuler.z = initEuler.z;

            Quaternion finalRotation = Quaternion.Euler(currentEuler);

            transform.position = currentCOM;
            transform.rotation = finalRotation;

            // 4. Inverse Dynamics (Shape Correction & Velocity Sync)
            if (!IsKinematic)
            {
                // Stiffness Alpha (Stiffness clamping for Jitter reduction)
                float stiffnessAlpha = (Config.DeformationCompliance < 1e-5f) ? 1.0f : (0.1f / Config.DeformationCompliance);
                stiffnessAlpha = Mathf.Min(stiffnessAlpha, 0.998f); 
                stiffnessAlpha = Mathf.Clamp01(stiffnessAlpha);
                float dt = Time.fixedDeltaTime;

                for (int i = 0; i < count; i++)
                {
                    int idx = _particleIndices[i];
                    if (idx >= simData.ParticleCount) continue;

                    Vector3 targetPos = currentCOM + (finalRotation * Vector3.Scale(_initialLocalPositions[i], transform.localScale));
                    Vector3 currentPos = (Vector3)simData.CurrentPosition[idx];
                    
                    // Position Correction
                    Vector3 newPos = Vector3.Lerp(currentPos, targetPos, stiffnessAlpha);
                    
                    // Velocity Correction (CRITICAL FIX)
                    Vector3 prevPos = (Vector3)simData.PreviousPosition[idx];
                    Vector3 newVel = (newPos - prevPos) / dt;

                    // Clamping and Damping
                    if (newVel.sqrMagnitude > 10000.0f) newVel = newVel.normalized * 100.0f;
                    
                    // Damping (Adjusted for stable fall - 0.97f was too high)
                    newVel *= 0.995f; 

                    // Write back
                    simData.CurrentPosition[idx] = newPos;
                    simData.PredictedPosition[idx] = newPos;
                    simData.Velocity[idx] = newVel;
                }
            }
        }

        void OnDrawGizmosSelected()
        {
            if (_particleIndices != null && _hook != null && _hook.World != null)
            {
                Gizmos.color = Color.green;
                var simData = _hook.World.Context;
                foreach (var idx in _particleIndices)
                {
                    if(idx < simData.ParticleCount)
                        Gizmos.DrawSphere((Vector3)simData.CurrentPosition[idx], ParticleRadius);
                }
            }
        }
    }
}