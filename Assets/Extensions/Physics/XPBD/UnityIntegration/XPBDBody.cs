using UnityEngine;
using Unity.Mathematics;
using mehmetsrl.Physics.XPBD.Core;

namespace mehmetsrl.Physics.XPBD.UnityIntegration
{
    public class XPBDBody : MonoBehaviour
    {
        [Header("Physical Properties")] public float Mass = 1.0f;
        public float Radius = 0.5f;
        public bool IsKinematic = false;

        [Header("Material")] [Tooltip("Inverse Stiffness. 0 = Hard, 1 = Soft.")] [Range(0f, 1f)]
        public float MaterialCompliance = 0.0f;

        [Header("Constraints (Lock Axis)")] public bool LockX = false;
        public bool LockY = false;
        public bool LockZ = false;

        private int _bodyIndex = -1;
        private PhysicsSceneHook _hook;
        private int _bodyInstanceID; // For self-collision filtering

        void Start()
        {
            _hook = FindFirstObjectByType<PhysicsSceneHook>();
            if (_hook == null)
            {
                Debug.LogError("[XPBDBody] Error: PhysicsSceneHook missing.");
                return;
            }

            _bodyInstanceID = GetInstanceID();
            RegisterBody();
        }

        void RegisterBody()
        {
            float simMass = IsKinematic ? 0.0f : Mass;

            float3 lockMask = new float3(
                LockX ? 0.0f : 1.0f,
                LockY ? 0.0f : 1.0f,
                LockZ ? 0.0f : 1.0f
            );

            // Calculate Stiffness (k = 1 / alpha)
            float initialStiffness = (MaterialCompliance < 1e-9f)
                ? PhysicsConstants.MaxStiffness
                : (1.0f / MaterialCompliance);

            if (_hook.World == null) return;

            _bodyIndex = _hook.World.AddBody(
                (float3)transform.position,
                simMass,
                Radius,
                lockMask,
                _bodyInstanceID,
                initialStiffness
            );
        }

        void FixedUpdate()
        {
            if (_bodyIndex == -1 || _hook == null || _hook.World == null) return;

            var simData = _hook.World.Context;

            // Safety check
            if (_bodyIndex >= simData.ParticleCount)
            {
                _bodyIndex = -1;
                return;
            }

            // 1. Update Locks & Mass Runtime
            float3 currentLockMask = new float3(LockX ? 0f : 1f, LockY ? 0f : 1f, LockZ ? 0f : 1f);
            simData.PositionLockMask[_bodyIndex] = currentLockMask;

            if (IsKinematic)
            {
                simData.InverseMass[_bodyIndex] = 0.0f;
            }
            else
            {
                float invMass = (Mass > 0.00001f) ? 1.0f / Mass : 0.0f;
                simData.InverseMass[_bodyIndex] = invMass;
            }

            // 2. Sync Transform
            if (IsKinematic)
            {
                // Unity -> Physics
                float3 currentPos = (float3)transform.position;
                simData.CurrentPosition[_bodyIndex] = currentPos;
                simData.PredictedPosition[_bodyIndex] = currentPos;
                simData.Velocity[_bodyIndex] = float3.zero;
            }
            else
            {
                // Physics -> Unity
                float3 newPos = simData.CurrentPosition[_bodyIndex];

                if (math.any(math.isnan(newPos)))
                {
                    Debug.LogError($"[XPBDBody] {name} NaN detected!");
                    return;
                }

                transform.position = (Vector3)newPos;
            }
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, Radius);
        }
    }
}